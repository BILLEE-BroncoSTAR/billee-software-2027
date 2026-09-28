#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

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
        // Declared with an explicit constraint string so a UI that reads parameter
        // descriptors (Foxglove's Parameters panel, rqt) shows the valid values.
        rcl_interfaces::msg::ParameterDescriptor schemeDesc;
        schemeDesc.description =
            "Control scheme: 'arcade' (left stick steers, triggers drive) or 'tank' "
            "(one stick per track). Settable at runtime - the change applies to the "
            "next /joy message, no relaunch.";
        schemeDesc.additional_constraints = "one of: arcade, tank";
        scheme_ = this->declare_parameter("scheme", std::string("arcade"), schemeDesc);
        if (!teleop::isValidScheme(scheme_))
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

        // Announced here, not at the top of the constructor: the experimental
        // warning quotes track_width, which is only declared above.
        warnIfExperimental(scheme_);

        // depth 1: a fresh Joy packet immediately supersedes the previous one
        joySub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 1, std::bind(&JoyDrive::onJoy, this, std::placeholders::_1));
        twistPub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

        // Live scheme switching. Without this the member below is only ever read in
        // this constructor, so `ros2 param set` (and Foxglove's Parameters panel,
        // which goes through the same service) would appear to succeed and change
        // nothing. Switching mid-drive is safe: the deadman still gates all motion
        // and the next /joy message is simply mixed the new way.
        paramCb_ = this->add_on_set_parameters_callback(
            std::bind(&JoyDrive::onSetParameters, this, std::placeholders::_1));

        // Watchdog: publish zero when /joy goes SILENT.
        //
        // onJoy() is otherwise the only publish site, so if /joy simply stops - the
        // Foxglove tab is closed, the laptop sleeps, the pad is unplugged - this node
        // publishes nothing at all, and the last non-zero Twist stays the last thing
        // on the wire. That is survivable only because diff_drive_controller has its
        // own cmd_vel_timeout; this makes the ground side stop asking rather than
        // relying on the rover to stop listening.
        //
        // Matters most for joy_source:=browser, which has no joy_node and therefore
        // no autorepeat at all: /joy stops the instant the browser does.
        //
        // Steady clock, not this->now(): ground_station.launch.py can run with
        // use_sim_time, and a frozen /clock would silently disable this watchdog.
        joyTimeout_ = this->declare_parameter("joy_timeout", 0.5);
        lastJoy_ = steadyClock_.now();
        watchdog_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&JoyDrive::onWatchdog, this));
    }

private:
    // ponytail: tank is selectable but has never been driven on the rover; the warning
    // is the cheap guard. Drop it once someone has verified the mapping on hardware.
    void warnIfExperimental(const std::string& scheme) const
    {
        if (scheme == "tank")
        {
            RCLCPP_WARN(
                this->get_logger(),
                "scheme 'tank' is UNVALIDATED on hardware: its axis defaults were "
                "inferred, not measured, and track_width (%.3f m) is a placeholder that "
                "sets the turn rate. Check both against `ros2 topic echo /joy` before "
                "driving.",
                trackWidth_);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "control scheme: %s", scheme.c_str());
        }
    }

    // Rejects an invalid scheme rather than silently falling back, so a typo in a UI
    // surfaces as a failed parameter set instead of the rover driving the wrong way.
    rcl_interfaces::msg::SetParametersResult onSetParameters(
        const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& p : params)
        {
            if (p.get_name() != "scheme")
            {
                continue;
            }
            const std::string requested = p.as_string();
            if (!teleop::isValidScheme(requested))
            {
                result.successful = false;
                result.reason = "scheme must be 'arcade' or 'tank', got '" + requested + "'";
                return result;
            }
            if (requested != scheme_)
            {
                scheme_ = requested;
                warnIfExperimental(scheme_);
            }
        }
        return result;
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

    // Zero the command whenever /joy has gone quiet for longer than joy_timeout.
    // Publishes on every tick while silent rather than once: it is idempotent, and a
    // live stream of zeros keeps diff_drive_controller fed instead of leaning on its
    // cmd_vel_timeout as a second mechanism.
    //
    // ponytail: this cannot see a HUNG pad. joy_node republishes the last state at
    // autorepeat_rate (20 Hz), so a frozen driver holding the deadman is
    // indistinguishable from a driver genuinely holding it. Lowering autorepeat_rate
    // does not help - at 0 a held stick publishes nothing and this would zero
    // spuriously. The check is physical: unplug the pad mid-drive and confirm /joy
    // actually stops.
    void onWatchdog()
    {
        if ((steadyClock_.now() - lastJoy_).seconds() <= joyTimeout_)
        {
            return;
        }
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "no /joy for more than %.2fs - publishing zero. The pad, the link or the "
            "Foxglove panel has gone away.",
            joyTimeout_);
        twistPub_->publish(geometry_msgs::msg::Twist());  // all zeros
    }

    void onJoy(const sensor_msgs::msg::Joy& msg)
    {
        lastJoy_ = steadyClock_.now();
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

    double joyTimeout_;
    // Steady, so a stopped or rewound /clock cannot disable the watchdog.
    rclcpp::Clock steadyClock_{RCL_STEADY_TIME};
    rclcpp::Time lastJoy_;

    OnSetParametersCallbackHandle::SharedPtr paramCb_;
    rclcpp::TimerBase::SharedPtr watchdog_;
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
