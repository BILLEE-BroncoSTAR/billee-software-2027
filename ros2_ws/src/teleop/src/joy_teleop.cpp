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
// A pad reaching us through a browser (Foxglove's Joystick panel) follows the
// W3C "standard" mapping instead, which has no trigger axes at all - only the
// two sticks - and puts LT/RT in buttons[6]/[7]:
//   steer   = axes[0]     left stick X   (right = +1, so invert_steer: true)
//   reverse = buttons[6]  LT / L2
//   forward = buttons[7]  RT / R2
//   deadman = buttons[5]  RB / R1        (same index as Linux)
// Set throttle_button / reverse_button for that source (see
// chassis_bringup/config/tele_params_browser.yaml).
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
        // -1 = take the trigger from its axis above; >= 0 = from that button instead.
        throttleButton_ = this->declare_parameter("throttle_button", -1);
        reverseButton_ = this->declare_parameter("reverse_button", -1);
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

    // Is the source configured for one trigger present in this message?
    bool triggerInRange(const sensor_msgs::msg::Joy& msg, int axis, int button) const
    {
        return button >= 0 ? static_cast<size_t>(button) < msg.buttons.size()
                           : axisInRange(msg, axis);
    }

    // One trigger's 0..1 press amount, from its axis or - when a button index is
    // configured - from that button. sensor_msgs/Joy stores buttons as int32, so a
    // button-sourced trigger is on/off, not proportional.
    double triggerFor(const sensor_msgs::msg::Joy& msg, int axis, int button, bool& seen) const
    {
        if (button >= 0)
        {
            return msg.buttons[button] != 0 ? 1.0 : 0.0;
        }
        return triggerAmount(msg.axes[axis], seen);
    }

    void onJoy(const sensor_msgs::msg::Joy& msg)
    {
        auto twist = geometry_msgs::msg::Twist();

        const bool indicesOk =
            axisInRange(msg, steerAxis_) &&
            triggerInRange(msg, throttleAxis_, throttleButton_) &&
            triggerInRange(msg, reverseAxis_, reverseButton_) && deadmanButton_ >= 0 &&
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
            const double forward =
                triggerFor(msg, throttleAxis_, throttleButton_, throttleSeen_);
            const double reverse =
                triggerFor(msg, reverseAxis_, reverseButton_, reverseSeen_);
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
    int throttleButton_;
    int reverseButton_;
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
