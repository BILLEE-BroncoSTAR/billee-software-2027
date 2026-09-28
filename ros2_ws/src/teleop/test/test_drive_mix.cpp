// Checks the drive mixing in include/teleop/drive_mix.hpp. Run with:
//   colcon test --packages-select teleop && colcon test-result --verbose
#include <gtest/gtest.h>

#include "teleop/drive_mix.hpp"

using teleop::arcadeMix;
using teleop::tankMix;
using teleop::triggerAmount;

TEST(TriggerAmount, MapsRestToZeroAndPressToOne)
{
    EXPECT_DOUBLE_EQ(triggerAmount(1.0, 1.0, -1.0), 0.0);
    EXPECT_DOUBLE_EQ(triggerAmount(-1.0, 1.0, -1.0), 1.0);
    EXPECT_DOUBLE_EQ(triggerAmount(0.0, 1.0, -1.0), 0.5);
    // Out-of-range readings clamp rather than running away.
    EXPECT_DOUBLE_EQ(triggerAmount(-2.0, 1.0, -1.0), 1.0);
    EXPECT_DOUBLE_EQ(triggerAmount(2.0, 1.0, -1.0), 0.0);
    // Degenerate config must not divide by zero.
    EXPECT_DOUBLE_EQ(triggerAmount(0.5, 1.0, 1.0), 0.0);
}

TEST(ArcadeMix, TriggersOppose)
{
    EXPECT_DOUBLE_EQ(arcadeMix(1.0, 0.0, 0.0, 2.0, 1.5).linear, 2.0);
    EXPECT_DOUBLE_EQ(arcadeMix(0.0, 1.0, 0.0, 2.0, 1.5).linear, -2.0);
    // Both held = no motion, not double speed.
    EXPECT_DOUBLE_EQ(arcadeMix(1.0, 1.0, 0.0, 2.0, 1.5).linear, 0.0);
    EXPECT_DOUBLE_EQ(arcadeMix(0.0, 0.0, 1.0, 2.0, 1.5).angular, 1.5);
}

TEST(TankMix, StraightSpinAndPivot)
{
    // Both tracks forward: straight ahead, no yaw.
    const auto straight = tankMix(1.0, 1.0, 0.67, 2.0);
    EXPECT_DOUBLE_EQ(straight.linear, 2.0);
    EXPECT_DOUBLE_EQ(straight.angular, 0.0);

    // Opposite tracks: spin in place.
    const auto spin = tankMix(-1.0, 1.0, 0.67, 2.0);
    EXPECT_DOUBLE_EQ(spin.linear, 0.0);
    EXPECT_DOUBLE_EQ(spin.angular, 2.0 * 2.0 / 0.67);

    // One track only: half speed forward, turning away from the driven side.
    const auto pivot = tankMix(0.0, 1.0, 0.67, 2.0);
    EXPECT_DOUBLE_EQ(pivot.linear, 1.0);
    EXPECT_GT(pivot.angular, 0.0);

    // A track width of zero would divide by zero; it must stop instead.
    EXPECT_DOUBLE_EQ(tankMix(1.0, -1.0, 0.0, 2.0).angular, 0.0);
}

TEST(TankMix, IsSymmetric)
{
    // Mirroring the sticks mirrors the yaw and leaves forward speed alone.
    const auto a = tankMix(0.25, 0.75, 0.67, 2.0);
    const auto b = tankMix(0.75, 0.25, 0.67, 2.0);
    EXPECT_DOUBLE_EQ(a.linear, b.linear);
    EXPECT_DOUBLE_EQ(a.angular, -b.angular);
}
