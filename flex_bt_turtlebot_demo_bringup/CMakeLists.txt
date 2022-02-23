################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(flex_bt_turtlebot_demo_bringup)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

################################################################################
# Install
################################################################################
install(
  DIRECTORY param launch rviz urdf behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

add_executable(simple_cmd_publisher
	src/simple_cmd_publisher.cpp
)
ament_target_dependencies(simple_cmd_publisher rclcpp geometry_msgs)

install(
	TARGETS
		simple_cmd_publisher
	RUNTIME DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
