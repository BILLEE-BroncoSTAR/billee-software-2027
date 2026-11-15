#include <gtest/gtest.h>

#include <cmath>

#include "tf2/LinearMath/Quaternion.h"
#include "zed_openpose/vio_pose.hpp"

using zed_openpose::Covariance6;
using zed_openpose::base_covariance;
using zed_openpose::camera_to_base;
using zed_openpose::kUnfusedVariance;

namespace
{

constexpr double kEps = 1e-9;

tf2::Transform pose(double yaw, double x, double y, double z)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::Transform(q, tf2::Vector3(x, y, z));
}

double yaw_of(const tf2::Transform & t)
{
  double roll, pitch, yaw;
  t.getBasis().getRPY(roll, pitch, yaw);
  return yaw;
}

// The URDF mount: camera_link 0.3624 m ahead of and 0.3424 m above base_link, no rotation.
const tf2::Transform kMount = pose(0.0, 0.3624, 0.0, 0.3424);

}  // namespace

TEST(CameraToBase, StationaryCameraIsIdentity)
{
  const tf2::Transform base = camera_to_base(kMount, tf2::Transform::getIdentity());
  EXPECT_NEAR(base.getOrigin().length(), 0.0, kEps);
  EXPECT_NEAR(yaw_of(base), 0.0, kEps);
}

TEST(CameraToBase, DrivingForwardMovesTheBaseForward)
{
  const tf2::Transform base = camera_to_base(kMount, pose(0.0, 1.0, 0.0, 0.0));
  EXPECT_NEAR(base.getOrigin().x(), 1.0, kEps);
  EXPECT_NEAR(base.getOrigin().y(), 0.0, kEps);
  EXPECT_NEAR(base.getOrigin().z(), 0.0, kEps);
}

TEST(CameraToBase, SpinInPlaceIsPureYaw)
{
  // The rover turns 90 deg left about base_link. The camera, 0.3624 m ahead, swings from
  // (0.3624, 0) to (0, 0.3624) in odom, i.e. by (-0.3624, +0.3624) from where it started, and
  // turns 90 deg with the rover.
  const tf2::Transform camera = pose(M_PI / 2, -0.3624, 0.3624, 0.0);
  const tf2::Transform base = camera_to_base(kMount, camera);
  EXPECT_NEAR(base.getOrigin().length(), 0.0, kEps);
  EXPECT_NEAR(yaw_of(base), M_PI / 2, kEps);
}

TEST(CameraToBase, YawedMountRotatesTheMotion)
{
  // A camera looking left (+90 deg): driving along its own x is the base moving along +y.
  const tf2::Transform base = camera_to_base(pose(M_PI / 2, 0.2, 0.0, 0.3), pose(0.0, 1.0, 0.0, 0.0));
  EXPECT_NEAR(base.getOrigin().x(), 0.0, kEps);
  EXPECT_NEAR(base.getOrigin().y(), 1.0, kEps);
}

TEST(BaseCovariance, FloorsAndUnfusedAxes)
{
  Covariance6 sdk{};  // a ZED reporting (near) zero uncertainty
  const Covariance6 out = base_covariance(tf2::Matrix3x3::getIdentity(), sdk, 9e-4, 4e-4);
  EXPECT_DOUBLE_EQ(out[0], 9e-4);
  EXPECT_DOUBLE_EQ(out[7], 9e-4);
  EXPECT_DOUBLE_EQ(out[35], 4e-4);
  EXPECT_DOUBLE_EQ(out[14], kUnfusedVariance);  // z
  EXPECT_DOUBLE_EQ(out[21], kUnfusedVariance);  // roll
  EXPECT_DOUBLE_EQ(out[28], kUnfusedVariance);  // pitch
}

TEST(BaseCovariance, KeepsLargerSdkValuesAndRotatesThem)
{
  Covariance6 sdk{};
  sdk[0] = 0.04;   // x in the camera's world
  sdk[7] = 0.01;   // y
  sdk[35] = 0.02;  // yaw
  sdk[2 * 6 + 0] = sdk[0 * 6 + 2] = 0.5;  // x-z cross term: must not survive
  tf2::Matrix3x3 left;
  left.setRPY(0.0, 0.0, M_PI / 2);  // camera looking left: its x is the base's y
  const Covariance6 out = base_covariance(left, sdk, 9e-4, 4e-4);
  EXPECT_NEAR(out[0], 0.01, kEps);
  EXPECT_NEAR(out[7], 0.04, kEps);
  EXPECT_NEAR(out[35], 0.02, kEps);
  for (int k = 0; k < 6; ++k) {
    if (k == 2) {continue;}
    EXPECT_DOUBLE_EQ(out[2 * 6 + k], 0.0);
    EXPECT_DOUBLE_EQ(out[k * 6 + 2], 0.0);
  }
}

TEST(BaseCovariance, NonFiniteFallsBackToFloors)
{
  Covariance6 sdk{};
  sdk[0] = std::nan("");
  sdk[7] = 5.0;
  const Covariance6 out = base_covariance(tf2::Matrix3x3::getIdentity(), sdk, 9e-4, 4e-4);
  EXPECT_DOUBLE_EQ(out[0], 9e-4);
  EXPECT_DOUBLE_EQ(out[7], 9e-4);
  EXPECT_DOUBLE_EQ(out[35], 4e-4);
}
