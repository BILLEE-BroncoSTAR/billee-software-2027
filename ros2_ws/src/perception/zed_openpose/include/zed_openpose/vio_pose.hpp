// ZED positional tracking (VIO) -> the rover's base pose, for the EKF.
//
// The ZED SDK tracks its left camera C in a world frame W that starts where the camera started.
// The EKF wants the base B (base_footprint) in odom O, where O starts where the base started, as
// the synthetic VIO (zed2i) publishes it. O = W moved by the camera mount T_B_C, so
//
//   T_O_B(t) = T_B_C * T_W_C(t) * T_B_C^-1
//
// Without this the camera's lever arm (0.36 m ahead of base_link) makes an in-place spin look
// like the base drove around a circle. Free of ZED types so it can be unit-tested; the node
// passes the sl::Pose in as a tf2::Transform and a 6x6 covariance.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"

namespace zed_openpose
{

using Covariance6 = std::array<double, 36>;  // row-major x, y, z, roll, pitch, yaw

// Variance given to the axes a planar EKF must not fuse (z, roll, pitch), like the synthetic VIO.
constexpr double kUnfusedVariance = 1e6;

// T_B_C: the camera's pose in the base frame (TF base_frame <- zed_frame).
// T_W_C: the camera's pose in the ZED's world frame (sl::Camera::getPosition, WORLD).
inline tf2::Transform camera_to_base(const tf2::Transform & T_B_C, const tf2::Transform & T_W_C)
{
  return T_B_C * T_W_C * T_B_C.inverse();
}

// The ZED's pose covariance (translation, then rotation in so3, in W) rotated into O, with the
// x, y and yaw variances floored and z, roll and pitch set to kUnfusedVariance (cross terms 0).
// The lever arm's rotation-to-translation coupling is left out: the floors dominate it.
// A covariance with any non-finite entry is replaced by the floors alone.
inline Covariance6 base_covariance(
  const tf2::Matrix3x3 & R_B_C, const Covariance6 & sdk, double min_position_var,
  double min_yaw_var)
{
  Covariance6 out{};
  const bool finite = std::all_of(sdk.begin(), sdk.end(), [](double v) {return std::isfinite(v);});
  if (finite) {
    // out = J * sdk * J^T with J = blockdiag(R, R)
    const auto J = [&R_B_C](int r, int c) {
        return (r / 3 == c / 3) ? R_B_C[r % 3][c % 3] : 0.0;
      };
    for (int r = 0; r < 6; ++r) {
      for (int c = 0; c < 6; ++c) {
        double sum = 0.0;
        for (int i = 0; i < 6; ++i) {
          for (int j = 0; j < 6; ++j) {
            sum += J(r, i) * sdk[i * 6 + j] * J(c, j);
          }
        }
        out[r * 6 + c] = sum;
      }
    }
  }
  for (const int unfused : {2, 3, 4}) {
    for (int k = 0; k < 6; ++k) {
      out[unfused * 6 + k] = 0.0;
      out[k * 6 + unfused] = 0.0;
    }
    out[unfused * 6 + unfused] = kUnfusedVariance;
  }
  out[0] = std::max(out[0], min_position_var);    // x
  out[7] = std::max(out[7], min_position_var);    // y
  out[35] = std::max(out[35], min_yaw_var);       // yaw
  return out;
}

}  // namespace zed_openpose
