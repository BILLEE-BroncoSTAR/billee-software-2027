// Pure drive-mixing math, kept out of the node so it can be tested without ROS.
#ifndef TELEOP__DRIVE_MIX_HPP_
#define TELEOP__DRIVE_MIX_HPP_

#include <algorithm>
#include <string>

namespace teleop
{

// The control schemes joy_drive accepts. Kept here rather than in the node so the
// validation that guards runtime `scheme` changes is testable without a ROS graph:
// rejecting a bad value is what stops a typo in a UI silently changing how the rover
// drives.
inline bool isValidScheme(const std::string& scheme)
{
    return scheme == "arcade" || scheme == "tank";
}

struct DriveCmd
{
    double linear;   // m/s along x
    double angular;  // rad/s about z
};

// Map a raw trigger axis reading to a 0..1 "how far pressed" amount.
// `rest` is the axis value with the trigger released, `press` fully pulled.
inline double triggerAmount(double raw, double rest, double press)
{
    const double span = rest - press;
    if (span == 0.0)
    {
        return 0.0;
    }
    return std::clamp((rest - raw) / span, 0.0, 1.0);
}

// Arcade: one stick steers, the triggers drive. Throttle and steering are
// independent, so speed_scale and steer_scale set the two limits separately.
inline DriveCmd arcadeMix(
    double forward, double reverse, double steer, double speedScale, double steerScale)
{
    return {(forward - reverse) * speedScale, steer * steerScale};
}

// Tank: each stick drives one track. Differential-drive inverse kinematics —
// the mean of the two track speeds is forward motion, their difference over the
// track width is the yaw rate. Both sticks forward = straight; opposite = spin
// in place. Needs the real track width to get rad/s right (see config).
inline DriveCmd tankMix(double left, double right, double trackWidth, double speedScale)
{
    if (trackWidth <= 0.0)
    {
        return {0.0, 0.0};
    }
    return {
        (left + right) / 2.0 * speedScale,
        (right - left) / trackWidth * speedScale};
}

}  // namespace teleop

#endif  // TELEOP__DRIVE_MIX_HPP_
