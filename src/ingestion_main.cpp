#include <memory>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2qnukf/ingestion_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2qnukf::IngestionNode>();
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions{}, 4U};
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
