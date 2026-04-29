#include "ros2qnukf/ingestion_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <random>
#include <stdexcept>

namespace ros2qnukf
{

IngestionNode::IngestionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node{"ros2qnukf_ingestion_node", options}
{
  random_engine_.seed(std::random_device{}());

  imu_topic_ = this->declare_parameter<std::string>("imu_topic", imu_topic_);
  left_image_topic_ = this->declare_parameter<std::string>("left_image_topic", left_image_topic_);
  right_image_topic_ = this->declare_parameter<std::string>("right_image_topic", right_image_topic_);
  gt_pose_topic_ = this->declare_parameter<std::string>("gt_pose_topic", gt_pose_topic_);
  gt_transform_topic_ = this->declare_parameter<std::string>("gt_transform_topic", gt_transform_topic_);
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
  gt_lookup_max_dt_sec_ = std::max(
    0.0,
    this->declare_parameter<double>("gt_lookup_max_dt_sec", gt_lookup_max_dt_sec_));
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
  pseudo_pose_when_no_gt_ = this->declare_parameter<bool>("pseudo_pose_when_no_gt", pseudo_pose_when_no_gt_);
  camera_qos_reliable_ = this->declare_parameter<bool>("camera_qos_reliable", camera_qos_reliable_);

  const auto sensor_qos = build_sensor_qos(sensor_qos_depth_);
  const rclcpp::QoS camera_qos =
    camera_qos_reliable_
      ? rclcpp::QoS(rclcpp::KeepLast(sensor_qos_depth_)).reliable().durability_volatile()
      : sensor_qos;
  const auto rmw_sensor_qos = camera_qos.get_rmw_qos_profile();
  const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(sensor_qos_depth_))
    .reliable()
    .durability_volatile();

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_,
    sensor_qos,
    std::bind(&IngestionNode::imu_callback, this, std::placeholders::_1));

  if (!gt_pose_topic_.empty()) {
    gt_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      gt_pose_topic_,
      sensor_qos,
      std::bind(&IngestionNode::gt_pose_callback, this, std::placeholders::_1));
  }
  if (!gt_transform_topic_.empty()) {
    gt_transform_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      gt_transform_topic_,
      sensor_qos,
      std::bind(&IngestionNode::gt_transform_callback, this, std::placeholders::_1));
  }
  estimate_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>(estimate_pose_topic_, output_qos);
  estimate_pose_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    estimate_pose_cov_topic_, output_qos);
  estimate_path_pub_ =
    this->create_publisher<nav_msgs::msg::Path>(estimate_path_topic_, output_qos);
  estimate_path_msg_.header.frame_id = "world";

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

  filter_.set_debug(debug_);
}

void IngestionNode::imu_callback(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
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

  if (!filter_.initialized()) {
    const auto gt_pose = lookup_gt_pose_interpolated(measurement.stamp);
    if (gt_pose.has_value()) {
      filter_.initialize_from_pose(gt_pose->position, gt_pose->orientation);
    } else {
      // IMU-first startup: if GT has not arrived yet, start from a neutral pose.
      filter_.initialize_from_pose(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
    }
  } else if (imu_history_.size() >= 2U) {
    const auto & prev = imu_history_[imu_history_.size() - 2U];
    const auto & curr = imu_history_[imu_history_.size() - 1U];
    filter_.predict_step(prev, curr);
  }

  trim_imu_history(measurement.stamp - rclcpp::Duration::from_seconds(imu_history_sec_));
  drain_pending_stereo_frames_locked();
}

void IngestionNode::gt_pose_callback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  std::scoped_lock lock{data_mutex_};
  GtPoseMeasurement gt{};
  gt.stamp = rclcpp::Time{msg->header.stamp};
  gt.position = Eigen::Vector3d{
    msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z};
  gt.orientation = Eigen::Quaterniond{
    msg->pose.orientation.w,
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z};
  gt.orientation.normalize();
  gt_history_.push_back(gt);
  trim_gt_history(gt.stamp - rclcpp::Duration::from_seconds(imu_history_sec_));
  drain_pending_stereo_frames_locked();
}

void IngestionNode::gt_transform_callback(geometry_msgs::msg::TransformStamped::ConstSharedPtr msg)
{
  std::scoped_lock lock{data_mutex_};
  GtPoseMeasurement gt{};
  gt.stamp = rclcpp::Time{msg->header.stamp};
  gt.position = Eigen::Vector3d{
    msg->transform.translation.x,
    msg->transform.translation.y,
    msg->transform.translation.z};
  gt.orientation = Eigen::Quaterniond{
    msg->transform.rotation.w,
    msg->transform.rotation.x,
    msg->transform.rotation.y,
    msg->transform.rotation.z};
  if (gt.orientation.norm() < 1e-9) {
    throw std::runtime_error("Received GT transform quaternion with near-zero norm.");
  }
  gt.orientation.normalize();
  gt_history_.push_back(gt);
  trim_gt_history(gt.stamp - rclcpp::Duration::from_seconds(imu_history_sec_));
  drain_pending_stereo_frames_locked();
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
      break;
    }
    pending_stereo_frame_stamps_.pop_front();
  }
}

bool IngestionNode::try_process_stereo_frame(const rclcpp::Time & frame_stamp)
{
  if (!filter_.initialized()) {
    return false;
  }

  if (imu_history_.empty()) {
    return false;
  }

  const auto target_imu_stamp = frame_stamp + rclcpp::Duration::from_seconds(cam_to_imu_dt_sec_);
  if (imu_history_.back().stamp < target_imu_stamp) {
    return false;
  }

  auto gt_pose = lookup_gt_pose_interpolated(frame_stamp);
  if (!gt_pose.has_value()) {
    if (!pseudo_pose_when_no_gt_) {
      return false;
    }
    const auto filter_state = filter_.state();
    GtPoseMeasurement pseudo_gt{};
    pseudo_gt.stamp = frame_stamp;
    pseudo_gt.position = filter_state.position;
    pseudo_gt.orientation = filter_state.orientation;
    gt_pose = pseudo_gt;
  }

  const auto pseudo_measurement = build_pseudo_vision_measurement(*gt_pose);

  try_filter_update(pseudo_measurement);
  publish_filter_estimate(frame_stamp);
  return true;
}

rclcpp::QoS IngestionNode::build_sensor_qos(const int depth) const
{
  return rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort().durability_volatile();
}

std::optional<IngestionNode::GtPoseMeasurement> IngestionNode::lookup_gt_pose_interpolated(
  const rclcpp::Time & stamp) const
{
  if (gt_history_.empty()) {
    return std::nullopt;
  }
  if (gt_history_.size() == 1U) {
    GtPoseMeasurement out = gt_history_.front();
    out.stamp = stamp;
    return out;
  }

  auto later_it = std::lower_bound(
    gt_history_.begin(),
    gt_history_.end(),
    stamp,
    [](const GtPoseMeasurement & sample, const rclcpp::Time & target) {
      return sample.stamp < target;
    });

  if (later_it != gt_history_.end() && later_it->stamp == stamp) {
    return *later_it;
  }

  auto earlier_it = gt_history_.end();
  if (later_it == gt_history_.begin()) {
    // Target is before history window: extrapolate from first two samples.
    earlier_it = gt_history_.begin();
    later_it = std::next(gt_history_.begin());
  } else if (later_it == gt_history_.end()) {
    // Target is after history window: extrapolate from last two samples.
    later_it = std::prev(gt_history_.end());
    earlier_it = std::prev(later_it);
  } else {
    earlier_it = std::prev(later_it);
  }

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
  interpolated.orientation.normalize();
  return interpolated;
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
  // Remove samples strictly before keep_after; keep stamps t >= keep_after.
  while (!imu_history_.empty() && imu_history_.front().stamp < keep_after) {
    imu_history_.pop_front();
  }
}

void IngestionNode::trim_gt_history(const rclcpp::Time & keep_after)
{
  while (!gt_history_.empty() && gt_history_.front().stamp < keep_after) {
    gt_history_.pop_front();
  }
}

void IngestionNode::initialize_pseudo_world_points()
{
  pseudo_world_points_.clear();
  pseudo_world_points_.reserve(static_cast<std::size_t>(pseudo_feature_count_));
  for (auto index = 0; index < pseudo_feature_count_; ++index) {
    const auto ring = static_cast<double>(index / 4) + 1.0;
    const auto column = static_cast<double>(index % 4) - 1.5;
    const auto x = 0.5 * ring;
    const auto y = 0.4 * column;
    const auto z = 1.5 + 0.15 * std::sin(static_cast<double>(index));
    pseudo_world_points_.push_back(Eigen::Vector3d{x, y, z});
  }
}

void IngestionNode::try_filter_update(
  const QnukfFilter::PseudoVisionMeasurement & pseudo_measurement)
{
  filter_.update_pseudo_vision(pseudo_measurement, pseudo_noise_stddev_);
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
