################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(flex_bt_turtlebot3_demo_bringup)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


################################################################################
# Install
################################################################################
install(
  DIRECTORY param launch rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

add_executable(simple_battery_status_publisher
  src/simple_battery_status_publisher.cpp
)
ament_target_dependencies(simple_battery_status_publisher rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs)

install(
  TARGETS
  simple_battery_status_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
