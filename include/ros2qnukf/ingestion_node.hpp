#ifndef ROS2QNUKF__INGESTION_NODE_HPP_
#define ROS2QNUKF__INGESTION_NODE_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ros2qnukf/qnukf_filter.hpp"

namespace ros2qnukf
{

class IngestionNode : public rclcpp::Node
{
public:
  explicit IngestionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  struct GtPoseMeasurement
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  };

  using StereoSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image>;
  using StereoSynchronizer = message_filters::Synchronizer<StereoSyncPolicy>;

  void imu_callback(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void stereo_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & left_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & right_msg);

  bool try_process_stereo_frame(const rclcpp::Time & frame_stamp);
  void drain_pending_stereo_frames_locked();

  static std::vector<GtPoseMeasurement> load_gt_csv_trajectory(const std::string & csv_path);
  GtPoseMeasurement lookup_gt_pose_from_csv_strict(const rclcpp::Time & stamp) const;
  QnukfFilter::PseudoVisionMeasurement build_pseudo_vision_measurement(
    const GtPoseMeasurement & gt_pose);

  void trim_imu_history(const rclcpp::Time & keep_after);
  void initialize_pseudo_world_points();
  void try_filter_update(const QnukfFilter::PseudoVisionMeasurement & pseudo_measurement);
  void publish_filter_estimate(const rclcpp::Time & stamp);
  void publish_gt_feature_markers(const rclcpp::Time & stamp);
  void publish_pseudo_measurement_markers_gt_frame(
    const rclcpp::Time & stamp,
    const QnukfFilter::PseudoVisionMeasurement & pseudo_measurement,
    const GtPoseMeasurement & gt_pose);
  void gt_feature_markers_timer_callback();

  rclcpp::QoS build_sensor_qos(int depth) const;

  std::string imu_topic_{"/imu0"};
  std::string left_image_topic_{"/cam0/image_raw"};
  std::string right_image_topic_{"/cam1/image_raw"};
  std::string estimate_pose_topic_{"/ros2qnukf/pose_estimate"};
  std::string estimate_pose_cov_topic_{"/ros2qnukf/pose_estimate_cov"};
  std::string estimate_path_topic_{"/ros2qnukf/path_estimate"};
  bool debug_{false};
  bool use_stereo_{true};
  int sensor_qos_depth_{10};
  double cam_to_imu_dt_sec_{0.0};
  double imu_history_sec_{2.0};
  double pseudo_noise_stddev_{0.01};
  int pseudo_feature_count_{50};
  int stereo_sync_queue_size_{15};
  int stereo_queue_max_{512};
  double path_publish_period_sec_{0.0};
  bool camera_qos_reliable_{false};
  bool publish_gt_feature_markers_{true};
  double gt_feature_marker_diameter_{0.12};
  double gt_feature_markers_publish_hz_{5.0};
  std::string gt_feature_markers_topic_{"/ros2qnukf/gt_feature_points"};
  bool publish_pseudo_measurement_markers_{true};
  double pseudo_measurement_marker_diameter_{0.08};
  std::string pseudo_measurement_markers_topic_{"/ros2qnukf/pseudo_measurements_gt"};
  bool init_bias_from_gt_csv_{true};
  std::string path_gt_csv_{};
  double gyro_noise_stddev_{2.399e-3};
  double accel_noise_stddev_{2.828e-2};
  double gyro_bias_rw_stddev_{1.371e-6};
  double accel_bias_rw_stddev_{2.121e-4};
  std::vector<double> initial_covariance_diagonal_{
    1e-1, 1e-1, 1e-1, 1e-1, 1e-1,
    1e-1, 1e-1, 1e-1, 1e-1, 1e-1,
    1e-1, 1e-1, 1e-1, 1e-1, 1e-1};
  Eigen::Vector3d initial_gyro_bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d initial_accel_bias_{Eigen::Vector3d::Zero()};

  std::mutex data_mutex_{};
  QnukfFilter filter_{};
  std::deque<QnukfFilter::ImuMeasurement> imu_history_{};
  std::vector<GtPoseMeasurement> gt_csv_trajectory_{};
  std::vector<Eigen::Vector3d> pseudo_world_points_{};
  std::optional<rclcpp::Time> last_path_publish_stamp_{};
  std::deque<rclcpp::Time> pending_stereo_frame_stamps_{};

  std::mt19937 random_engine_{};
  std::normal_distribution<double> noise_distribution_{0.0, 1.0};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_{};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimate_pose_pub_{};
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr estimate_pose_cov_pub_{};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr estimate_path_pub_{};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gt_feature_markers_pub_{};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pseudo_measurement_markers_pub_{};
  rclcpp::TimerBase::SharedPtr gt_feature_markers_timer_{};
  nav_msgs::msg::Path estimate_path_msg_{};
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_image_sub_{};
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_image_sub_{};
  std::shared_ptr<StereoSynchronizer> stereo_sync_{};
};

}  // namespace ros2qnukf

#endif  // ROS2QNUKF__INGESTION_NODE_HPP_
