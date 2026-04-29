cmake_minimum_required(VERSION 3.8)
project(ros2qnukf)

# Enables clangd / C++ extension IntelliSense via compile_commands.json (colcon: build/<pkg>/).
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ros2qnukf_ingestion_node
  src/ingestion_main.cpp
  src/ingestion_node.cpp
  src/qnukf_filter.cpp
)

add_executable(ros2qnukf_gt_csv_publisher
  src/gt_csv_publisher_main.cpp
  src/gt_csv_publisher_node.cpp
)

target_include_directories(ros2qnukf_ingestion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(ros2qnukf_gt_csv_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ros2qnukf_ingestion_node
  geometry_msgs
  message_filters
  nav_msgs
  rclcpp
  sensor_msgs
)

ament_target_dependencies(ros2qnukf_gt_csv_publisher
  geometry_msgs
  nav_msgs
  rclcpp
)

target_link_libraries(ros2qnukf_ingestion_node Eigen3::Eigen)
target_link_libraries(ros2qnukf_gt_csv_publisher Eigen3::Eigen)

install(
  TARGETS
    ros2qnukf_ingestion_node
    ros2qnukf_gt_csv_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
