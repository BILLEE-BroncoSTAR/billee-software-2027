#include <algorithm>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Arcade drive from a game pad: left stick = steering, right trigger = forward,
// left trigger = reverse, one shoulder button gates all motion (deadman).
//
// Xbox and PlayStation pads expose the same layout through `joy_node` on Linux:
//   steer   = axes[0]     left stick X   (left = +1, right = -1)
//   reverse = axes[2]     LT / L2        (released ~ +1, pressed ~ -1)
//   forward = axes[5]     RT / R2        (released ~ +1, pressed ~ -1)
//   deadman = buttons[5]  RB / R1
// Every index and sign is a parameter (see config/joystick.yaml) so a pad or
// driver that differs is a yaml edit, not a rebuild.
//
// Class/executable name is historical ("tank drive"); the scheme is arcade now.
class JoyTankDrive : public rclcpp::Node
{
public:
    JoyTankDrive() : Node("joy_tank_drive")
    {
        deadmanButton_ = this->declare_parameter("deadman_button", 5);
        steerAxis_ = this->declare_parameter("steer_axis", 0);
        steerScale_ = this->declare_parameter("steer_scale", 1.5);
        invertSteer_ = this->declare_parameter("invert_steer", false);
        throttleAxis_ = this->declare_parameter("throttle_axis", 5);
        reverseAxis_ = this->declare_parameter("reverse_axis", 2);
        speedScale_ = this->declare_parameter("speed_scale", 2.0);
        triggerRest_ = this->declare_parameter("trigger_rest", 1.0);
        triggerPress_ = this->declare_parameter("trigger_press", -1.0);

        // depth 1: a fresh Joy packet immediately supersedes the previous one
        joySub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 1, std::bind(&JoyTankDrive::onJoy, this, std::placeholders::_1));
        twistPub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    }

private:
    // Map a raw trigger axis value to a 0..1 "how far pressed" amount.
    // Some drivers report 0.0 for an untouched trigger until it is first moved;
    // treat that as released until we have seen a real (non-zero) reading.
    double triggerAmount(double raw, bool& seen) const
    {
        if (!seen)
        {
            if (raw == 0.0)
            {
                return 0.0;
            }
            seen = true;
        }
        const double span = triggerRest_ - triggerPress_;
        if (span == 0.0)
        {
            return 0.0;
        }
        return std::clamp((triggerRest_ - raw) / span, 0.0, 1.0);
    }

    bool axisInRange(const sensor_msgs::msg::Joy& msg, int i) const
    {
        return i >= 0 && static_cast<size_t>(i) < msg.axes.size();
    }

    void onJoy(const sensor_msgs::msg::Joy& msg)
    {
        auto twist = geometry_msgs::msg::Twist();

        const bool indicesOk =
            axisInRange(msg, steerAxis_) && axisInRange(msg, throttleAxis_) &&
            axisInRange(msg, reverseAxis_) && deadmanButton_ >= 0 &&
            static_cast<size_t>(deadmanButton_) < msg.buttons.size();

        if (!indicesOk)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Joy message has %zu axes / %zu buttons; a configured index is out of "
                "range. Check config/joystick.yaml against `ros2 topic echo /joy`.",
                msg.axes.size(), msg.buttons.size());
            twistPub_->publish(twist);  // all zeros
            return;
        }

        if (msg.buttons[deadmanButton_])
        {
            const double forward = triggerAmount(msg.axes[throttleAxis_], throttleSeen_);
            const double reverse = triggerAmount(msg.axes[reverseAxis_], reverseSeen_);
            const double steer = (invertSteer_ ? -1.0 : 1.0) * msg.axes[steerAxis_];

            twist.linear.x = (forward - reverse) * speedScale_;
            twist.angular.z = steer * steerScale_;
        }

        twistPub_->publish(twist);
    }

    int deadmanButton_;
    int steerAxis_;
    int throttleAxis_;
    int reverseAxis_;
    double steerScale_;
    double speedScale_;
    double triggerRest_;
    double triggerPress_;
    bool invertSteer_;

    bool throttleSeen_ = false;
    bool reverseSeen_ = false;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistPub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyTankDrive>());
    rclcpp::shutdown();
    return 0;
}
