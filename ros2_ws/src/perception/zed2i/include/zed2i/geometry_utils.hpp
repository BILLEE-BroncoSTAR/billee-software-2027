// Copyright 2026 URC-2027 Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ZED2I__GEOMETRY_UTILS_HPP_
#define ZED2I__GEOMETRY_UTILS_HPP_

#include <cmath>

#include "geometry_msgs/msg/quaternion.hpp"

namespace zed2i::geometry
{
constexpr double kPi = 3.14159265358979323846;

inline double wrap_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

inline double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

inline geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}
}  // namespace zed2i::geometry

#endif  // ZED2I__GEOMETRY_UTILS_HPP_
