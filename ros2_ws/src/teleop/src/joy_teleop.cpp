#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Game pad -> /cmd_vel, gated by a deadman button (hold to move; release = zero Twist).
// Two schemes, picked by the `scheme` parameter (config/joystick.yaml):
//
//   arcade  left stick X steers, right trigger forward, left trigger reverse
//             linear.x  = (forward - reverse) * max_vel
//             angular.z = steer * steer_scale
//   tank    left stick Y = left wheels, right stick Y = right wheels
//             linear.x  = (left + right) / 2 * max_vel
//             angular.z = (right - left) / track_width * max_vel
//
// Xbox and PlayStation pads share these indices through `joy_node` on Linux:
//   axes[0] left stick X, axes[1] left stick Y, axes[2] LT / L2, axes[4] right stick Y,
//   axes[5] RT / R2, buttons[4] LB / L1, buttons[5] RB / R1.
// Every index is a parameter, so a different pad is a yaml edit, not a rebuild. An index
// the pad doesn't have logs a warning and publishes zeros instead of crashing.
//
// Class/executable name is historical ("tank drive").
class JoyTankDrive : public rclcpp::Node
{
public:
  JoyTankDrive()
  : Node("joy_tank_drive")
  {
    const auto scheme = this->declare_parameter("scheme", std::string("arcade"));
    if (scheme != "arcade" && scheme != "tank") {
      throw std::invalid_argument("scheme must be 'arcade' or 'tank', got '" + scheme + "'");
    }
    tank_ = scheme == "tank";
    deadmanButton_ = this->declare_parameter("deadman_button", 4);
    maxVel_ = this->declare_parameter("max_vel", 1.0);

    // arcade
    steerAxis_ = this->declare_parameter("steer_axis", 0);
    steerScale_ = this->declare_parameter("steer_scale", 1.5);
    invertSteer_ = this->declare_parameter("invert_steer", false);
    throttleAxis_ = this->declare_parameter("throttle_axis", 5);
    reverseAxis_ = this->declare_parameter("reverse_axis", 2);
    triggerRest_ = this->declare_parameter("trigger_rest", 1.0);
    triggerPress_ = this->declare_parameter("trigger_press", -1.0);

    // tank
    leftAxis_ = this->declare_parameter("left_axis", 1);
    rightAxis_ = this->declare_parameter("right_axis", 4);
    trackWidth_ = this->declare_parameter("track_width", 0.67);
    if (tank_ && !(trackWidth_ > 0.0)) {
      throw std::invalid_argument("track_width must be > 0");
    }

    RCLCPP_INFO(
      this->get_logger(), "%s drive, deadman button %d, max_vel %.2f m/s",
      scheme.c_str(), deadmanButton_, maxVel_);

    // depth 1: a fresh Joy packet immediately supersedes the previous one
    joySub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 1, std::bind(&JoyTankDrive::onJoy, this, std::placeholders::_1));
    twistPub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  }

private:
  // Map a raw trigger axis value to a 0..1 "how far pressed" amount.
  // Some drivers report 0.0 for an untouched trigger until it is first moved;
  // treat that as released until we have seen a real (non-zero) reading.
  double triggerAmount(double raw, bool & seen) const
  {
    if (!seen) {
      if (raw == 0.0) {
        return 0.0;
      }
      seen = true;
    }
    const double span = triggerRest_ - triggerPress_;
    if (span == 0.0) {
      return 0.0;
    }
    return std::clamp((triggerRest_ - raw) / span, 0.0, 1.0);
  }

  static bool inRange(size_t size, int i)
  {
    return i >= 0 && static_cast<size_t>(i) < size;
  }

  bool indicesOk(const sensor_msgs::msg::Joy & msg) const
  {
    if (!inRange(msg.buttons.size(), deadmanButton_)) {
      return false;
    }
    if (tank_) {
      return inRange(msg.axes.size(), leftAxis_) && inRange(msg.axes.size(), rightAxis_);
    }
    return inRange(msg.axes.size(), steerAxis_) && inRange(msg.axes.size(), throttleAxis_) &&
           inRange(msg.axes.size(), reverseAxis_);
  }

  void onJoy(const sensor_msgs::msg::Joy & msg)
  {
    auto twist = geometry_msgs::msg::Twist();

    if (!indicesOk(msg)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Joy message has %zu axes / %zu buttons; a configured index is out of "
        "range. Check config/joystick.yaml against `ros2 topic echo /joy`.",
        msg.axes.size(), msg.buttons.size());
      twistPub_->publish(twist);        // all zeros
      return;
    }

    if (msg.buttons[deadmanButton_]) {
      if (tank_) {
        const double left = msg.axes[leftAxis_];
        const double right = msg.axes[rightAxis_];
        twist.linear.x = (left + right) / 2.0 * maxVel_;
        twist.angular.z = (right - left) / trackWidth_ * maxVel_;
      } else {
        const double forward = triggerAmount(msg.axes[throttleAxis_], throttleSeen_);
        const double reverse = triggerAmount(msg.axes[reverseAxis_], reverseSeen_);
        const double steer = (invertSteer_ ? -1.0 : 1.0) * msg.axes[steerAxis_];
        twist.linear.x = (forward - reverse) * maxVel_;
        twist.angular.z = steer * steerScale_;
      }
      if (!std::isfinite(twist.linear.x) || !std::isfinite(twist.angular.z)) {
        twist = geometry_msgs::msg::Twist();
      }
    }

    twistPub_->publish(twist);
  }

  bool tank_ = false;
  int deadmanButton_;
  double maxVel_;

  int steerAxis_;
  int throttleAxis_;
  int reverseAxis_;
  double steerScale_;
  double triggerRest_;
  double triggerPress_;
  bool invertSteer_;
  bool throttleSeen_ = false;
  bool reverseSeen_ = false;

  int leftAxis_;
  int rightAxis_;
  double trackWidth_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistPub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyTankDrive>());
  rclcpp::shutdown();
  return 0;
}
