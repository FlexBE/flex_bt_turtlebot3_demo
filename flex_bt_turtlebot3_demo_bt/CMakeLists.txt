cmake_minimum_required(VERSION 3.5)
project(flex_bt_turtlebot3_demo_bt)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  nav2_behavior_tree
  behaviortree_cpp_v3
  geometry_msgs
)

include_directories(
  include
)

add_library(flex_bt_turtlebot3_is_battery_low_bt_node SHARED conditions/is_battery_low.cpp)
list(APPEND conditions flex_bt_turtlebot3_is_battery_low_bt_node)

foreach(bt_condition ${conditions})
  ament_target_dependencies(${bt_condition} ${dependencies})
  target_compile_definitions(${bt_condition} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${library_name}
                ${actions}
                ${conditions}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

ament_package()
