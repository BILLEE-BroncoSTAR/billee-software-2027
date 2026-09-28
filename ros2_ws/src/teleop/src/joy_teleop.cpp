#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "teleop/drive_mix.hpp"

// Game-pad teleop with two selectable control schemes. Pick one with the
// `scheme` parameter; every index, sign and scale is a parameter too, so a pad
// or driver that differs is a yaml edit, not a rebuild.
//
//   scheme: "arcade"  left stick steers, right trigger forward, left reverse
//   scheme: "tank"    left stick drives the left track, right stick the right
//
// One shoulder button gates all motion in both schemes (deadman); releasing it
// publishes an all-zero Twist.
//
// Xbox and PlayStation pads expose the same layout through `joy_node` on Linux:
//   axes[0] left stick X (left = +1)   axes[1] left stick Y (up = +1)
//   axes[3] right stick X              axes[4] right stick Y
//   axes[2] LT / L2, axes[5] RT / R2   (released ~ +1, pressed ~ -1)
//   buttons[5] RB / R1
//
// A pad reaching us through a browser (Foxglove's Joystick panel) follows the
// W3C "standard" mapping instead, which has no trigger axes at all - only the
// two sticks - and puts LT/RT in buttons[6]/[7]:
//   steer   = axes[0]     left stick X   (right = +1, so invert_steer: true)
//   reverse = buttons[6]  LT / L2
//   forward = buttons[7]  RT / R2
//   deadman = buttons[5]  RB / R1        (same index as Linux)
// Set throttle_button / reverse_button for that source (see
// teleop/config/joystick_browser.yaml).
class JoyDrive : public rclcpp::Node
{
public:
    JoyDrive() : Node("joy_drive")
    {
        scheme_ = this->declare_parameter("scheme", std::string("arcade"));
        if (scheme_ != "arcade" && scheme_ != "tank")
        {
            RCLCPP_WARN(
                this->get_logger(), "unknown scheme '%s'; using arcade.", scheme_.c_str());
            scheme_ = "arcade";
        }

        deadmanButton_ = this->declare_parameter("deadman_button", 5);
        speedScale_ = this->declare_parameter("speed_scale", 2.0);

        // arcade
        steerAxis_ = this->declare_parameter("steer_axis", 0);
        steerScale_ = this->declare_parameter("steer_scale", 1.5);
        invertSteer_ = this->declare_parameter("invert_steer", false);
        throttleAxis_ = this->declare_parameter("throttle_axis", 5);
        reverseAxis_ = this->declare_parameter("reverse_axis", 2);
        // -1 = take the trigger from its axis above; >= 0 = from that button instead.
        throttleButton_ = this->declare_parameter("throttle_button", -1);
        reverseButton_ = this->declare_parameter("reverse_button", -1);
        triggerRest_ = this->declare_parameter("trigger_rest", 1.0);
        triggerPress_ = this->declare_parameter("trigger_press", -1.0);

        // tank
        leftAxis_ = this->declare_parameter("left_axis", 1);
        rightAxis_ = this->declare_parameter("right_axis", 4);
        // Measure this on the rover; it sets how much stick difference is how
        // many rad/s, so a wrong value makes every turn the wrong rate.
        trackWidth_ = this->declare_parameter("track_width", 0.67);

        RCLCPP_INFO(this->get_logger(), "control scheme: %s", scheme_.c_str());

        // depth 1: a fresh Joy packet immediately supersedes the previous one
        joySub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 1, std::bind(&JoyDrive::onJoy, this, std::placeholders::_1));
        twistPub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    }

private:
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
    //
    // Some drivers report 0.0 for an untouched trigger until it is first moved;
    // `seen` holds that off until we have had a real (non-zero) reading.
    double triggerFor(const sensor_msgs::msg::Joy& msg, int axis, int button, bool& seen) const
    {
        if (button >= 0)
        {
            return msg.buttons[button] != 0 ? 1.0 : 0.0;
        }
        const double raw = msg.axes[axis];
        if (!seen)
        {
            if (raw == 0.0)
            {
                return 0.0;
            }
            seen = true;
        }
        return teleop::triggerAmount(raw, triggerRest_, triggerPress_);
    }

    bool indicesOk(const sensor_msgs::msg::Joy& msg) const
    {
        const bool deadmanOk = deadmanButton_ >= 0 &&
            static_cast<size_t>(deadmanButton_) < msg.buttons.size();
        if (!deadmanOk)
        {
            return false;
        }
        if (scheme_ == "tank")
        {
            return axisInRange(msg, leftAxis_) && axisInRange(msg, rightAxis_);
        }
        return axisInRange(msg, steerAxis_) &&
            triggerInRange(msg, throttleAxis_, throttleButton_) &&
            triggerInRange(msg, reverseAxis_, reverseButton_);
    }

    void onJoy(const sensor_msgs::msg::Joy& msg)
    {
        auto twist = geometry_msgs::msg::Twist();

        if (!indicesOk(msg))
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Joy message has %zu axes / %zu buttons; an index configured for the "
                "'%s' scheme is out of range. Check the joystick parameters against "
                "`ros2 topic echo /joy`.",
                msg.axes.size(), msg.buttons.size(), scheme_.c_str());
            twistPub_->publish(twist);  // all zeros
            return;
        }

        if (msg.buttons[deadmanButton_])
        {
            teleop::DriveCmd cmd{0.0, 0.0};
            if (scheme_ == "tank")
            {
                cmd = teleop::tankMix(
                    msg.axes[leftAxis_], msg.axes[rightAxis_], trackWidth_, speedScale_);
            }
            else
            {
                const double forward =
                    triggerFor(msg, throttleAxis_, throttleButton_, throttleSeen_);
                const double reverse =
                    triggerFor(msg, reverseAxis_, reverseButton_, reverseSeen_);
                const double steer = (invertSteer_ ? -1.0 : 1.0) * msg.axes[steerAxis_];
                cmd = teleop::arcadeMix(forward, reverse, steer, speedScale_, steerScale_);
            }
            twist.linear.x = cmd.linear;
            twist.angular.z = cmd.angular;
        }

        twistPub_->publish(twist);
    }

    std::string scheme_;
    int deadmanButton_;
    int steerAxis_;
    int throttleAxis_;
    int reverseAxis_;
    int throttleButton_;
    int reverseButton_;
    int leftAxis_;
    int rightAxis_;
    double steerScale_;
    double speedScale_;
    double trackWidth_;
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
    rclcpp::spin(std::make_shared<JoyDrive>());
    rclcpp::shutdown();
    return 0;
}
