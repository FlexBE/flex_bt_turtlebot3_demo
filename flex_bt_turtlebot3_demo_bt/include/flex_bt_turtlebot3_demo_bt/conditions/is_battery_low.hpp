#ifndef FLEX_BT_TURTLEBOT3_DEMO_BT__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
#define FLEX_BT_TURTLEBOT3_DEMO_BT__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace flex_bt_turtlebot3
{

class IsBatteryLowCondition : public BT::ConditionNode
{
public:
  IsBatteryLowCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsBatteryLowCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("min_battery", "Minimum battery percentage/voltage"),
      BT::InputPort<std::string>(
        "battery_topic", std::string("/battery_status"), "Battery topic"),
      BT::InputPort<bool>(
        "is_voltage", false, "If true voltage will be used to check for low battery"),
      BT::OutputPort<double>(
        "battery_voltage", "The current battery voltage of the robot"),
      BT::OutputPort<double>(
        "battery_percentage", "The current battery percentage of the robot"),
    };
  }

private:
  void batteryCallback(sensor_msgs::msg::BatteryState::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  std::string battery_topic_;
  double min_battery_;
  double battery_life_;
  bool is_voltage_;
  bool is_battery_low_;
};

}

#endif
