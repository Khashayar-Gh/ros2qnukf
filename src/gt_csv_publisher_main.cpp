#include "ros2qnukf/gt_csv_publisher_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2qnukf::GtCsvPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
