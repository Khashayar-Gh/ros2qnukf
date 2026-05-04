#include "ros2qnukf/ingestion_node.hpp"
#include "ros2qnukf/quaternion_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <functional>
#include <random>
#include <sstream>
#include <stdexcept>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace ros2qnukf
{

namespace
{

std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>> read_bias_means_from_gt_csv(
  const std::string & csv_path)
{
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    return std::nullopt;
  }

  Eigen::Vector3d gyro_bias_sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias_sum = Eigen::Vector3d::Zero();
  std::size_t sample_count = 0U;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line.front() == '#') {
      continue;
    }
    std::istringstream stream(line);
    std::string field;
    std::vector<double> values;
    values.reserve(20);
    while (std::getline(stream, field, ',')) {
      if (field.empty()) {
        continue;
      }
      values.push_back(std::atof(field.c_str()));
    }
    // EuRoC/OpenVINS CSV: [t, p(3), q(4), v(3), b_w(3), b_a(3), ...]
    if (values.size() < 17) {
      continue;
    }
    gyro_bias_sum += Eigen::Vector3d{values[11], values[12], values[13]};
    accel_bias_sum += Eigen::Vector3d{values[14], values[15], values[16]};
    ++sample_count;
  }
  if (sample_count == 0U) {
    return std::nullopt;
  }
  const auto inv_count = 1.0 / static_cast<double>(sample_count);
  return std::make_pair(gyro_bias_sum * inv_count, accel_bias_sum * inv_count);
}

void trim_string(std::string & s)
{
  const auto not_space = [](unsigned char c) { return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
}

}  // namespace

std::vector<IngestionNode::GtPoseMeasurement> IngestionNode::load_gt_csv_trajectory(
  const std::string & csv_path)
{
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open GT CSV file: " + csv_path);
  }

  std::vector<GtPoseMeasurement> out;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line.front() == '#') {
      continue;
    }
    const auto comma = line.find(',');
    if (comma == std::string::npos) {
      continue;
    }
    std::string ts_str = line.substr(0, comma);
    trim_string(ts_str);
    int64_t t_ns = 0;
    try {
      t_ns = std::stoll(ts_str);
    } catch (const std::exception &) {
      continue;
    }

    std::istringstream stream(line.substr(comma + 1));
    std::string field;
    std::vector<double> values;
    values.reserve(16);
    while (std::getline(stream, field, ',')) {
      if (field.empty()) {
        continue;
      }
      trim_string(field);
      values.push_back(std::atof(field.c_str()));
    }
    // [p(3), q(wxyz)(4), ...] — need at least 7 fields after timestamp
    if (values.size() < 7) {
      continue;
    }

    const int64_t sec_i64 = t_ns / 1000000000LL;
    const uint32_t nsec_u32 = static_cast<uint32_t>(t_ns % 1000000000LL);

    GtPoseMeasurement row{};
    row.stamp = rclcpp::Time{static_cast<int32_t>(sec_i64), nsec_u32, RCL_ROS_TIME};
    row.position = Eigen::Vector3d{values[0], values[1], values[2]};
    row.orientation = Eigen::Quaterniond{values[3], values[4], values[5], values[6]};
    if (row.orientation.norm() < 1e-12) {
      throw std::runtime_error(
        "GT CSV row has quaternion with near-zero norm (file " + csv_path + ").");
    }
    row.orientation.normalize();
    out.push_back(row);
  }

  if (out.size() < 2U) {
    throw std::runtime_error(
      "GT CSV must contain at least two valid pose rows: " + csv_path);
  }
  std::sort(
    out.begin(),
    out.end(),
    [](const GtPoseMeasurement & a, const GtPoseMeasurement & b) { return a.stamp < b.stamp; });
  return out;
}

IngestionNode::GtPoseMeasurement IngestionNode::lookup_gt_pose_from_csv_strict(
  const rclcpp::Time & stamp) const
{
  if (gt_csv_trajectory_.empty()) {
    throw std::runtime_error("GT CSV trajectory is empty.");
  }
  const auto & traj = gt_csv_trajectory_;
  auto later_it = std::lower_bound(
    traj.begin(),
    traj.end(),
    stamp,
    [](const GtPoseMeasurement & sample, const rclcpp::Time & target) {
      return sample.stamp < target;
    });

  if (later_it != traj.end() && later_it->stamp == stamp) {
    return *later_it;
  }
  if (later_it == traj.begin()) {
    throw std::runtime_error(
      "GT CSV lookup failed: stamp is before first CSV sample (strict bracketing requires a sample "
      "before and after).");
  }
  if (later_it == traj.end()) {
    throw std::runtime_error(
      "GT CSV lookup failed: stamp is after last CSV sample (strict bracketing requires a sample "
      "before and after).");
  }

  const auto earlier_it = std::prev(later_it);
  const double t0 = earlier_it->stamp.seconds();
  const double t1 = later_it->stamp.seconds();
  const double dt = t1 - t0;
  if (std::fabs(dt) < 1e-9) {
    return *later_it;
  }
  const double alpha = (stamp.seconds() - t0) / dt;
  GtPoseMeasurement interpolated{};
  interpolated.stamp = stamp;
  interpolated.position = earlier_it->position + alpha * (later_it->position - earlier_it->position);
  interpolated.orientation = earlier_it->orientation.slerp(alpha, later_it->orientation);
  interpolated.orientation = quaternion_cleanup(interpolated.orientation);
  return interpolated;
}

IngestionNode::IngestionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node{"ros2qnukf_ingestion_node", options}
{
  random_engine_.seed(std::random_device{}());

  imu_topic_ = this->declare_parameter<std::string>("imu_topic", imu_topic_);
  left_image_topic_ = this->declare_parameter<std::string>("left_image_topic", left_image_topic_);
  right_image_topic_ = this->declare_parameter<std::string>("right_image_topic", right_image_topic_);
  estimate_pose_topic_ = this->declare_parameter<std::string>("estimate_pose_topic", estimate_pose_topic_);
  estimate_pose_cov_topic_ =
    this->declare_parameter<std::string>("estimate_pose_cov_topic", estimate_pose_cov_topic_);
  estimate_path_topic_ = this->declare_parameter<std::string>("estimate_path_topic", estimate_path_topic_);
  debug_ = this->declare_parameter<bool>("debug", debug_);
  use_stereo_ = this->declare_parameter<bool>("use_stereo", use_stereo_);
  const auto declared_qos_depth = this->declare_parameter<int64_t>(
    "sensor_qos_depth",
    sensor_qos_depth_);
  sensor_qos_depth_ = static_cast<int>(std::max<int64_t>(1, declared_qos_depth));
  cam_to_imu_dt_sec_ = this->declare_parameter<double>("cam_to_imu_dt_sec", cam_to_imu_dt_sec_);
  imu_history_sec_ = std::max(
    0.2,
    this->declare_parameter<double>("imu_history_sec", imu_history_sec_));
  pseudo_noise_stddev_ = std::max(
    0.0,
    this->declare_parameter<double>("pseudo_noise_stddev", pseudo_noise_stddev_));
  pseudo_feature_count_ = static_cast<int>(std::max<int64_t>(
      4,
      this->declare_parameter<int64_t>("pseudo_feature_count", pseudo_feature_count_)));
  stereo_sync_queue_size_ = static_cast<int>(std::max<int64_t>(
      2,
      this->declare_parameter<int64_t>("stereo_sync_queue_size", stereo_sync_queue_size_)));
  stereo_queue_max_ = static_cast<int>(std::max<int64_t>(
      8,
      this->declare_parameter<int64_t>("stereo_queue_max", stereo_queue_max_)));
  path_publish_period_sec_ = std::max(
    0.0,
    this->declare_parameter<double>("path_publish_period_sec", path_publish_period_sec_));
  camera_qos_reliable_ = this->declare_parameter<bool>("camera_qos_reliable", camera_qos_reliable_);
  publish_gt_feature_markers_ =
    this->declare_parameter<bool>("publish_gt_feature_markers", publish_gt_feature_markers_);
  gt_feature_markers_topic_ =
    this->declare_parameter<std::string>("gt_feature_markers_topic", gt_feature_markers_topic_);
  publish_pseudo_measurement_markers_ = this->declare_parameter<bool>(
    "publish_pseudo_measurement_markers", publish_pseudo_measurement_markers_);
  pseudo_measurement_markers_topic_ = this->declare_parameter<std::string>(
    "pseudo_measurement_markers_topic", pseudo_measurement_markers_topic_);
  init_bias_from_gt_csv_ = this->declare_parameter<bool>("init_bias_from_gt_csv", init_bias_from_gt_csv_);
  path_gt_csv_ = this->declare_parameter<std::string>("path_gt_csv", path_gt_csv_);
  gyro_noise_stddev_ = this->declare_parameter<double>("gyro_noise_stddev", gyro_noise_stddev_);
  accel_noise_stddev_ = this->declare_parameter<double>("accel_noise_stddev", accel_noise_stddev_);
  gyro_bias_rw_stddev_ = this->declare_parameter<double>("gyro_bias_rw_stddev", gyro_bias_rw_stddev_);
  accel_bias_rw_stddev_ = this->declare_parameter<double>("accel_bias_rw_stddev", accel_bias_rw_stddev_);
  initial_covariance_diagonal_ = this->declare_parameter<std::vector<double>>(
    "initial_covariance_diagonal", initial_covariance_diagonal_);
  if (initial_covariance_diagonal_.size() != 15U) {
    throw std::invalid_argument("Parameter initial_covariance_diagonal must have exactly 15 values.");
  }
  gt_feature_marker_diameter_ = std::max(
    1e-6,
    this->declare_parameter<double>("gt_feature_marker_diameter", gt_feature_marker_diameter_));
  pseudo_measurement_marker_diameter_ = std::max(
    1e-6,
    this->declare_parameter<double>(
      "pseudo_measurement_marker_diameter",
      pseudo_measurement_marker_diameter_));
  gt_feature_markers_publish_hz_ = std::clamp(
    this->declare_parameter<double>("gt_feature_markers_publish_hz", gt_feature_markers_publish_hz_),
    0.1,
    50.0);

  if (path_gt_csv_.empty()) {
    throw std::invalid_argument(
      "Parameter path_gt_csv must be set to a EuRoC-format GT CSV (required for pose lookup).");
  }
  gt_csv_trajectory_ = load_gt_csv_trajectory(path_gt_csv_);
  RCLCPP_INFO(
    this->get_logger(),
    "Loaded GT CSV trajectory: path=%s samples=%zu first=%.9f last=%.9f",
    path_gt_csv_.c_str(),
    gt_csv_trajectory_.size(),
    gt_csv_trajectory_.front().stamp.seconds(),
    gt_csv_trajectory_.back().stamp.seconds());

  const auto sensor_qos = build_sensor_qos(sensor_qos_depth_);
  const rclcpp::QoS camera_qos =
    camera_qos_reliable_
      ? rclcpp::QoS(rclcpp::KeepLast(sensor_qos_depth_)).reliable().durability_volatile()
      : sensor_qos;
  const auto rmw_sensor_qos = camera_qos.get_rmw_qos_profile();
  const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(sensor_qos_depth_))
    .reliable()
    .durability_volatile();
  Eigen::Matrix<double, 15, 1> initial_covariance_diag_eigen;
  for (std::size_t idx = 0U; idx < initial_covariance_diagonal_.size(); ++idx) {
    initial_covariance_diag_eigen(static_cast<Eigen::Index>(idx)) = initial_covariance_diagonal_[idx];
  }
  filter_.set_process_noise_params(
    gyro_noise_stddev_,
    accel_noise_stddev_,
    gyro_bias_rw_stddev_,
    accel_bias_rw_stddev_);
  filter_.set_initial_covariance_diagonal(initial_covariance_diag_eigen);

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_,
    sensor_qos,
    std::bind(&IngestionNode::imu_callback, this, std::placeholders::_1));

  estimate_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>(estimate_pose_topic_, output_qos);
  estimate_pose_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    estimate_pose_cov_topic_, output_qos);
  estimate_path_pub_ =
    this->create_publisher<nav_msgs::msg::Path>(estimate_path_topic_, output_qos);
  estimate_path_msg_.header.frame_id = "world";
  if (publish_gt_feature_markers_) {
    gt_feature_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      gt_feature_markers_topic_, output_qos);
  }
  if (publish_pseudo_measurement_markers_) {
    pseudo_measurement_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      pseudo_measurement_markers_topic_, output_qos);
  }

  left_image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>();
  right_image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>();
  left_image_sub_->subscribe(this, left_image_topic_, rmw_sensor_qos);
  right_image_sub_->subscribe(this, right_image_topic_, rmw_sensor_qos);
  stereo_sync_ = std::make_shared<StereoSynchronizer>(
    StereoSyncPolicy(stereo_sync_queue_size_),
    *left_image_sub_,
    *right_image_sub_);
  stereo_sync_->registerCallback(
    std::bind(&IngestionNode::stereo_callback, this, std::placeholders::_1, std::placeholders::_2));

  initialize_pseudo_world_points();

  if (init_bias_from_gt_csv_ && !path_gt_csv_.empty()) {
    const auto means = read_bias_means_from_gt_csv(path_gt_csv_);
    if (means.has_value()) {
      initial_gyro_bias_ = means->first;
      initial_accel_bias_ = means->second;
      RCLCPP_INFO(
        this->get_logger(),
        "Initialized bias means from GT CSV. bw=[%.6f %.6f %.6f], ba=[%.6f %.6f %.6f]",
        initial_gyro_bias_.x(), initial_gyro_bias_.y(), initial_gyro_bias_.z(),
        initial_accel_bias_.x(), initial_accel_bias_.y(), initial_accel_bias_.z());
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to compute bias means from GT CSV '%s'; using zero init biases.",
        path_gt_csv_.c_str());
    }
  }

  if (publish_gt_feature_markers_ && gt_feature_markers_pub_) {
    const auto period_ns = static_cast<int64_t>(1e9 / gt_feature_markers_publish_hz_);
    gt_feature_markers_timer_ = this->create_timer(
      std::chrono::nanoseconds{period_ns},
      std::bind(&IngestionNode::gt_feature_markers_timer_callback, this));
  }

  filter_.set_debug(debug_);
}

void IngestionNode::imu_callback(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  static bool logged_first_imu = false;
  static bool logged_init_success = false;
  std::scoped_lock lock{data_mutex_};
  QnukfFilter::ImuMeasurement measurement{};
  measurement.stamp = rclcpp::Time{msg->header.stamp};
  measurement.omega = Eigen::Vector3d{
    msg->angular_velocity.x,
    msg->angular_velocity.y,
    msg->angular_velocity.z};
  measurement.accel = Eigen::Vector3d{
    msg->linear_acceleration.x,
    msg->linear_acceleration.y,
    msg->linear_acceleration.z};
  imu_history_.push_back(measurement);
  if (!logged_first_imu) {
    RCLCPP_INFO(
      this->get_logger(),
      "First IMU stamp observed at %.9f",
      measurement.stamp.seconds());
    logged_first_imu = true;
  }

  if (!filter_.initialized()) {
    try {
      const auto gt_pose = lookup_gt_pose_from_csv_strict(measurement.stamp);
      filter_.initialize_from_pose(
        gt_pose.position,
        gt_pose.orientation,
        initial_gyro_bias_,
        initial_accel_bias_);
      if (!logged_init_success) {
        RCLCPP_INFO(
          this->get_logger(),
          "Filter initialized from CSV at imu stamp %.9f",
          measurement.stamp.seconds());
        logged_init_success = true;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Initialization lookup failed at imu stamp %.9f: %s",
        measurement.stamp.seconds(),
        e.what());
      throw;
    }
  } else if (imu_history_.size() >= 2U) { //this would effectivly always be 2 and not more than 2
    const auto & prev = imu_history_[imu_history_.size() - 2U];
    const auto & curr = imu_history_[imu_history_.size() - 1U];
    filter_.predict_step(prev, curr);
    trim_imu_history(curr.stamp);
    drain_pending_stereo_frames_locked();
  } 
}

void IngestionNode::stereo_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & left_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & right_msg)
{
  if (!use_stereo_) {
    return;
  }
  const auto left_stamp = rclcpp::Time{left_msg->header.stamp};
  const auto right_stamp = rclcpp::Time{right_msg->header.stamp};
  const auto frame_stamp = (left_stamp >= right_stamp) ? left_stamp : right_stamp;

  std::scoped_lock lock{data_mutex_};

  pending_stereo_frame_stamps_.push_back(frame_stamp);
  while (static_cast<int>(pending_stereo_frame_stamps_.size()) > stereo_queue_max_) {
    pending_stereo_frame_stamps_.pop_front();
  }
  drain_pending_stereo_frames_locked(); //start processing the images
}

void IngestionNode::drain_pending_stereo_frames_locked()
{
  while (!pending_stereo_frame_stamps_.empty()) {
    const auto frame_stamp = pending_stereo_frame_stamps_.front();
    if (!try_process_stereo_frame(frame_stamp)) {
      break; //if the frame is not processed, break the loop
    } else {
      pending_stereo_frame_stamps_.pop_front();
    }
    
  }
}

bool IngestionNode::try_process_stereo_frame(const rclcpp::Time & frame_stamp)
{
  static bool logged_waiting_for_init = false;
  static rclcpp::Time last_imu_lag_log{0, 0, RCL_ROS_TIME};
  static std::size_t published_pose_count = 0U;

  if (!filter_.initialized()) {
    if (!logged_waiting_for_init) {
      RCLCPP_INFO(
        this->get_logger(),
        "Stereo frame waiting for initialization. frame_stamp=%.9f",
        frame_stamp.seconds());
      logged_waiting_for_init = true;
    }
    return false;
  }
  logged_waiting_for_init = false;

  if (imu_history_.empty()) {
    throw std::runtime_error(
      "IMU history is empty. Cannot process stereo frame.");
  }

  const auto target_imu_stamp = frame_stamp + rclcpp::Duration::from_seconds(cam_to_imu_dt_sec_);
  if (imu_history_.back().stamp < target_imu_stamp) {
    if (
      last_imu_lag_log.nanoseconds() == 0 ||
      std::fabs((frame_stamp - last_imu_lag_log).seconds()) > 0.2)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Stereo blocked: waiting IMU coverage. frame=%.9f target_imu=%.9f last_imu=%.9f queue=%zu",
        frame_stamp.seconds(),
        target_imu_stamp.seconds(),
        imu_history_.back().stamp.seconds(),
        pending_stereo_frame_stamps_.size());
      last_imu_lag_log = frame_stamp;
    }
    return false;
  }

  GtPoseMeasurement gt_pose{};
  try {
    gt_pose = lookup_gt_pose_from_csv_strict(frame_stamp);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Stereo CSV lookup failed at frame stamp %.9f: %s",
      frame_stamp.seconds(),
      e.what());
    throw;
  }

  const auto pseudo_measurement = build_pseudo_vision_measurement(gt_pose);
  publish_pseudo_measurement_markers_gt_frame(frame_stamp, pseudo_measurement, gt_pose);

  try_filter_update(pseudo_measurement);
  publish_filter_estimate(frame_stamp);
  ++published_pose_count;
  if (published_pose_count == 1U || (published_pose_count % 30U == 0U)) {
    RCLCPP_INFO(
      this->get_logger(),
      "Published pose #%zu at %.9f (pending queue=%zu)",
      published_pose_count,
      frame_stamp.seconds(),
      pending_stereo_frame_stamps_.size());
  }
  return true;
}

rclcpp::QoS IngestionNode::build_sensor_qos(const int depth) const
{
  return rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort().durability_volatile();
}

QnukfFilter::PseudoVisionMeasurement IngestionNode::build_pseudo_vision_measurement(
  const GtPoseMeasurement & gt_pose)
{
  if (pseudo_world_points_.empty()) {
    throw std::runtime_error("Pseudo world points are not initialized.");
  }

  QnukfFilter::PseudoVisionMeasurement measurement{};
  measurement.world_points = pseudo_world_points_;
  measurement.body_points.reserve(pseudo_world_points_.size());

  const auto rotation_body_to_world = gt_pose.orientation.toRotationMatrix();
  const auto rotation_world_to_body = rotation_body_to_world.transpose();
  for (const auto & world_point : pseudo_world_points_) {
    Eigen::Vector3d body_point = rotation_world_to_body * (world_point - gt_pose.position);
    body_point.x() += noise_distribution_(random_engine_) * pseudo_noise_stddev_;
    body_point.y() += noise_distribution_(random_engine_) * pseudo_noise_stddev_;
    body_point.z() += noise_distribution_(random_engine_) * pseudo_noise_stddev_;
    measurement.body_points.push_back(body_point);
  }
  return measurement;
}

void IngestionNode::trim_imu_history(const rclcpp::Time & keep_after)
{
  // Keep newest IMU sample; all older samples already consumed by predict_step.
  while (!imu_history_.empty() && imu_history_.front().stamp < keep_after) {
    imu_history_.pop_front();
  }
}

void IngestionNode::initialize_pseudo_world_points()
{
  constexpr double kCylinderRadiusMeters = 6.0;
  constexpr double kCylinderHeightMeters = 6.0;
  constexpr double kGoldenAngleRad = 2.39996322972865332;
  constexpr double kInvPhi = 0.61803398874989485;

  pseudo_world_points_.clear();
  pseudo_world_points_.reserve(static_cast<std::size_t>(pseudo_feature_count_));
  for (auto index = 0; index < pseudo_feature_count_; ++index) {
    const auto i = static_cast<double>(index);
    const auto count = static_cast<double>(pseudo_feature_count_);
    const auto u = std::fmod((i + 0.5) * kInvPhi, 1.0);
    const auto radius = kCylinderRadiusMeters * std::sqrt(u);
    const auto theta = kGoldenAngleRad * i;
    const auto x = radius * std::cos(theta);
    const auto y = radius * std::sin(theta);
    const auto z = kCylinderHeightMeters * ((i + 0.5) / count);
    pseudo_world_points_.push_back(Eigen::Vector3d{x, y, z});
  }
}

void IngestionNode::try_filter_update(
  const QnukfFilter::PseudoVisionMeasurement & pseudo_measurement)
{
  filter_.update_pseudo_vision(pseudo_measurement, pseudo_noise_stddev_);
}

void IngestionNode::gt_feature_markers_timer_callback()
{
  publish_gt_feature_markers(this->now());
}

void IngestionNode::publish_gt_feature_markers(const rclcpp::Time & stamp)
{
  if (!gt_feature_markers_pub_ || pseudo_world_points_.empty()) {
    return;    
  }
  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = stamp;
  marker.ns = "gt_pseudo_features";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  const double d = gt_feature_marker_diameter_;
  marker.scale.x = d;
  marker.scale.y = d;
  marker.scale.z = d;
  marker.lifetime.sec = 0;
  marker.lifetime.nanosec = 0;
  marker.points.reserve(pseudo_world_points_.size());
  marker.colors.reserve(pseudo_world_points_.size());
  const auto denom =
    pseudo_world_points_.size() > 1U ? static_cast<double>(pseudo_world_points_.size() - 1U) : 1.0;
  for (std::size_t i = 0; i < pseudo_world_points_.size(); ++i) {
    geometry_msgs::msg::Point p;
    p.x = pseudo_world_points_[i].x();
    p.y = pseudo_world_points_[i].y();
    p.z = pseudo_world_points_[i].z();
    marker.points.push_back(p);
    std_msgs::msg::ColorRGBA c;
    const auto t = static_cast<float>(static_cast<double>(i) / denom);
    c.r = 0.2F + 0.8F * t;
    c.g = 0.9F;
    c.b = 0.3F + 0.7F * (1.0F - t);
    c.a = 1.0F;
    marker.colors.push_back(c);
  }
  array.markers.push_back(marker);
  gt_feature_markers_pub_->publish(array);
}

void IngestionNode::publish_pseudo_measurement_markers_gt_frame(
  const rclcpp::Time & stamp,
  const QnukfFilter::PseudoVisionMeasurement & pseudo_measurement,
  const GtPoseMeasurement & gt_pose)
{
  if (!publish_pseudo_measurement_markers_ || !pseudo_measurement_markers_pub_) {
    return;
  }
  if (pseudo_measurement.body_points.empty()) {
    return;
  }

  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = stamp;
  marker.ns = "pseudo_measurements_gt_frame";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = pseudo_measurement_marker_diameter_;
  marker.scale.y = pseudo_measurement_marker_diameter_;
  marker.scale.z = pseudo_measurement_marker_diameter_;
  marker.lifetime.sec = 0;
  marker.lifetime.nanosec = 0;
  marker.points.reserve(pseudo_measurement.body_points.size());
  marker.colors.reserve(pseudo_measurement.body_points.size());

  const auto rotation_body_to_world = gt_pose.orientation.toRotationMatrix();
  const auto denom =
    pseudo_measurement.body_points.size() > 1U
      ? static_cast<double>(pseudo_measurement.body_points.size() - 1U)
      : 1.0;
  for (std::size_t i = 0; i < pseudo_measurement.body_points.size(); ++i) {
    const Eigen::Vector3d point_world =
      rotation_body_to_world * pseudo_measurement.body_points[i] + gt_pose.position;
    geometry_msgs::msg::Point p;
    p.x = point_world.x();
    p.y = point_world.y();
    p.z = point_world.z();
    marker.points.push_back(p);

    std_msgs::msg::ColorRGBA c;
    const auto t = static_cast<float>(static_cast<double>(i) / denom);
    c.r = 1.0F;
    c.g = 0.35F + 0.5F * (1.0F - t);
    c.b = 0.1F + 0.8F * t;
    c.a = 1.0F;
    marker.colors.push_back(c);
  }

  array.markers.push_back(marker);
  pseudo_measurement_markers_pub_->publish(array);
}

void IngestionNode::publish_filter_estimate(const rclcpp::Time & stamp)
{
  if (!estimate_pose_pub_) {
    return;
  }
  const auto filter_state = filter_.state();
  geometry_msgs::msg::PoseStamped message{};
  message.header.stamp = stamp;
  message.header.frame_id = "world";
  message.pose.position.x = filter_state.position.x();
  message.pose.position.y = filter_state.position.y();
  message.pose.position.z = filter_state.position.z();
  message.pose.orientation.w = filter_state.orientation.w();
  message.pose.orientation.x = filter_state.orientation.x();
  message.pose.orientation.y = filter_state.orientation.y();
  message.pose.orientation.z = filter_state.orientation.z();
  estimate_pose_pub_->publish(message);
  if (estimate_pose_cov_pub_) {
    geometry_msgs::msg::PoseWithCovarianceStamped message_cov{};
    message_cov.header = message.header;
    message_cov.pose.pose = message.pose;
    estimate_pose_cov_pub_->publish(message_cov);
  }
  estimate_path_msg_.header.stamp = stamp;
  estimate_path_msg_.poses.push_back(message);
  if (estimate_path_msg_.poses.size() > 2000U) {
    estimate_path_msg_.poses.erase(estimate_path_msg_.poses.begin());
  }
  const bool publish_path = path_publish_period_sec_ <= 0.0 ||
    !last_path_publish_stamp_.has_value() ||
    std::fabs((stamp - *last_path_publish_stamp_).seconds()) >= path_publish_period_sec_;
  if (publish_path) {
    estimate_path_pub_->publish(estimate_path_msg_);
    last_path_publish_stamp_ = stamp;
  }
}

}  // namespace ros2qnukf
