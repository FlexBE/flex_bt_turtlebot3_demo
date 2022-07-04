#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

// This is intended as a most trivial example of a battery load based
// purely on time

class SimpleBatteryStatusPublisher : public rclcpp::Node
{
  public:
    SimpleBatteryStatusPublisher()
    : Node("simple_battery_status_publisher"), count_(0)
    {
      fraction_ = 1.00;
      charging_ = false;

      timer_ = this->create_wall_timer(500ms,
                    std::bind(&SimpleBatteryStatusPublisher::timer_callback, this));

      // @TODO - make this parameterized
      recharge_rate_ = 1/(8.0/0.5); // 1 / steps required for X seconds at callback rate
      discharge_rate_ = 1/(180.0/0.5); // 1 / steps required for X seconds at callback rate

      publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_status", 10);

      sub_charging_ = this->create_subscription<std_msgs::msg::Bool>("robot_charger", rclcpp::SensorDataQoS(),
        std::bind(&SimpleBatteryStatusPublisher::charging_callback, this, std::placeholders::_1));
    }

  private:
    void timer_callback() {
      auto message = sensor_msgs::msg::BatteryState();

      if (charging_) {
        fraction_ += recharge_rate_;
      }
      else {
        fraction_ -= discharge_rate_;
      }

      if (fraction_ > 1.0) {
        fraction_ = 1.0;
      } else if (fraction_ < 0.0) {
        fraction_ = 0.0;
      }

      message.percentage = fraction_;
      publisher_->publish(message);

      RCLCPP_INFO(this->get_logger(), "Publishing battery status: %s", std::to_string(fraction_).c_str());
    }

    void charging_callback(const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg->data) {
        charging_ = true;
      }
      else {
        charging_ = false;
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_charging_;

    size_t count_;
    float fraction_;
    bool charging_;
    float recharge_rate_; // steps required for X seconds at callback rate
    float discharge_rate_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBatteryStatusPublisher>());
  rclcpp::shutdown();
  return 0;
}
