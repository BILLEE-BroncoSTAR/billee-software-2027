// Copyright (c) 2023 Eric Cox
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

#include "roboclaw_hardware_interface/motor_joint.hpp"

namespace roboclaw_hardware_interface
{

MotorJoint::MotorJoint(
  const std::string joint_name, const int32_t qppr, std::optional<int> limit_switch_pin,
  bool duty_output, double max_velocity)
: ticks_per_radian_(static_cast<double>(qppr) * 0.5 * M_1_PI), max_velocity_(max_velocity),
  duty_output_(duty_output), name(joint_name)
{
  if (limit_switch_pin) {
    limit_switch_ = std::make_unique<LimitSwitch>(*limit_switch_pin);
  }
}

int16_t MotorJoint::getDutyCommand() const
{
  double duty = duty_command_;
  if (!(duty == duty)) {  // NaN: no controller has claimed it yet
    duty = 0.0;
  }
  duty = duty < -1.0 ? -1.0 : (duty > 1.0 ? 1.0 : duty);
  return static_cast<int16_t>(duty * 32767.0);
}

int32_t MotorJoint::getTickRateCommand() const
{
  double velocity = velocity_command_;
  if (!(velocity == velocity)) {  // NaN: no controller has claimed it yet
    velocity = 0.0;
  }
  if (max_velocity_ > 0.0) {
    velocity = velocity < -max_velocity_ ? -max_velocity_ :
      (velocity > max_velocity_ ? max_velocity_ : velocity);
  }
  return static_cast<int32_t>(velocity * ticks_per_radian_);
}

// Set the position given the current wheel encoder count
void MotorJoint::setPositionState(const int32_t & encoder_count)
{
  if (initialized_encoder_count_) {
    // Update the joint angle
    position_state_ +=
      static_cast<double>(encoder_count - prior_encoder_count_) / ticks_per_radian_;
  } else {
    initialized_encoder_count_ = true;
  }
  // Store the prior encoder count for next time
  prior_encoder_count_ = encoder_count;
}
}  // namespace roboclaw_hardware_interface
