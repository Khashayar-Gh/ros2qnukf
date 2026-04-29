#ifndef ROS2QNUKF__GT_CSV_PUBLISHER_NODE_HPP_
#define ROS2QNUKF__GT_CSV_PUBLISHER_NODE_HPP_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ros2qnukf
{

class GtCsvPublisherNode : public rclcpp::Node
{
public:
  explicit GtCsvPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  struct GtSample
  {
    double t_sec{0.0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  };

  void load_csv(const std::string & path);
  void timer_callback();
  GtSample interpolate(double t_sec) const;

  std::string path_gt_csv_{};
  std::string gt_pose_topic_{"/ov_msckf/posegt"};
  std::string gt_path_topic_{"/ov_msckf/pathgt"};
  std::string frame_id_{"world"};
  double publish_rate_hz_{60.0};
  bool publish_path_{true};
  int max_path_length_{16384};

  std::vector<GtSample> samples_{};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gt_pose_pub_{};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gt_path_pub_{};
  rclcpp::TimerBase::SharedPtr timer_{};
  nav_msgs::msg::Path gt_path_msg_{};
};

}  // namespace ros2qnukf

#endif  // ROS2QNUKF__GT_CSV_PUBLISHER_NODE_HPP_
