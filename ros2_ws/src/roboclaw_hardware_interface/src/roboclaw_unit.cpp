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

#include "roboclaw_hardware_interface/roboclaw_unit.hpp"

#include <map>
#include <tuple>
#include <roboclaw_serial/command.hpp>
#include <roboclaw_serial/interface.hpp>

#include "roboclaw_hardware_interface/motor_joint.hpp"

namespace roboclaw_hardware_interface
{

RoboClawUnit::RoboClawUnit(
  roboclaw_serial::Interface::SharedPtr interface, uint8_t address, MotorJoint::SharedPtr m1,
  MotorJoint::SharedPtr m2)
: address_(address)
{
  // Copy the pointer to the roboclaw interface
  interface_ = interface;

  // Set motor joint configurations
  joints[0] = m1;
  joints[1] = m2;
}

// Read the encoder counts from the roboclaw and update position state
void RoboClawUnit::read()
{
  // Read and update position
  interface_->read(encoder_state_, address_);

  // Get constant references to fields in the encoder counts message
  const auto & [m1_ticks, m2_ticks] = encoder_state_.fields;

  // Convert tick counts to position states for each motor (a duty output has no encoder)
  if (joints[0] && !joints[0]->isDutyOutput()) {
    joints[0]->setPositionState(m1_ticks);
  }
  if (joints[1] && !joints[1]->isDutyOutput()) {
    joints[1]->setPositionState(m2_ticks);
  }
}

// Write the tick rate request to the roboclaw and update
void RoboClawUnit::write()
{
  // BILLEE: two motors keep the single combined speed command (one message per cycle). A duty
  // output, or a unit with only one channel configured, gets one command per channel.
  const bool both_motors = joints[0] && joints[1] && !joints[0]->isDutyOutput() &&
    !joints[1]->isDutyOutput();
  if (!both_motors) {
    if (joints[0]) {
      if (joints[0]->isDutyOutput()) {
        std::get<0>(m1_duty_command_.fields) = joints[0]->getDutyCommand();
        interface_->write(m1_duty_command_, address_);
        joints[0]->setDutyState();
      } else {
        std::get<0>(m1_speed_command_.fields) = joints[0]->getTickRateCommand();
        interface_->write(m1_speed_command_, address_);
      }
    }
    if (joints[1]) {
      if (joints[1]->isDutyOutput()) {
        std::get<0>(m2_duty_command_.fields) = joints[1]->getDutyCommand();
        interface_->write(m2_duty_command_, address_);
        joints[1]->setDutyState();
      } else {
        std::get<0>(m2_speed_command_.fields) = joints[1]->getTickRateCommand();
        interface_->write(m2_speed_command_, address_);
      }
    }
    return;
  }

  // Get references to fields in the tick rate command message
  auto & [m1_speed, m2_speed] = tick_rate_command_.fields;

  // Set values to each field if the corresponding joint exists
  if (joints[0]) {
    m1_speed = joints[0]->getTickRateCommand();
  }
  if (joints[1]) {
    m2_speed = joints[1]->getTickRateCommand();
  }

  // Write the rate request to the roboclaw driver
  interface_->write(tick_rate_command_, address_);
}
}  // namespace roboclaw_hardware_interface
