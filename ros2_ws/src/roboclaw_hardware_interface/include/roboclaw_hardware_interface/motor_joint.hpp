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

#ifndef ROBOCLAW_HARDWARE_INTERFACE__MOTOR_JOINT_HPP_
#define ROBOCLAW_HARDWARE_INTERFACE__MOTOR_JOINT_HPP_

#include <math.h>

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "roboclaw_hardware_interface/limit_switch.hpp"

namespace roboclaw_hardware_interface
{
class MotorJoint
{
private:
  // The radians of rotation per tick
  const double ticks_per_radian_;

  // Desired velocity of the wheel in radians per second
  double velocity_command_ = 0;

  // BILLEE: |velocity command| is clamped to this (rad/s; <= 0 = no limit). Keeps a 12 V motor
  // on a higher-voltage battery at or under its rated speed whatever a controller sends.
  const double max_velocity_;

  // BILLEE: duty-output channel (e.g. a solenoid on M2): no encoder, commanded by duty in
  // [-1, 1] (sent as signed duty -32767..32767). Its state mirrors the command.
  const bool duty_output_;
  double duty_command_ = 0;
  double duty_state_ = 0;

  // Position of the wheel in radians
  double position_state_ = 0;

  // Store the prior encoder count for updating position state
  int32_t prior_encoder_count_;
  bool initialized_encoder_count_ = false;

  // Optional Jetson GPIO limit switch configured for this hardware joint.
  std::unique_ptr<LimitSwitch> limit_switch_;

public:
  typedef std::shared_ptr<MotorJoint> SharedPtr;

  // Name of the parent joint
  const std::string name;

  // Constructor. duty_output: a channel driven by duty with no encoder (qppr unused).
  MotorJoint(
    const std::string joint_name, const int32_t qppr,
    std::optional<int> limit_switch_pin = std::nullopt, bool duty_output = false,
    double max_velocity = 0.0);

  // BILLEE: true for a duty-output channel (command "duty"), false for a motor ("velocity").
  bool isDutyOutput() const {return duty_output_;}

  // BILLEE: the duty command as the RoboClaw's signed 16-bit duty, clamped to [-1, 1].
  int16_t getDutyCommand() const;

  // BILLEE: zero the command (whichever kind this joint has).
  void zeroCommand() {velocity_command_ = 0; duty_command_ = 0;}

  // BILLEE: called after a duty command was sent, so the state reports what's applied.
  void setDutyState() {duty_state_ = duty_command_;}

  // Return the tick rate required to execute the current velocity command
  int32_t getTickRateCommand() const;

  // Set the position given the current wheel encoder count
  void setPositionState(const int32_t & encoder_count);

  // Accessor methods to enable ros2_control to access joint interface pointers
  inline double * getPositionStatePtr() {return &position_state_;}
  inline double * getVelocityCommandPtr() {return &velocity_command_;}
  inline double * getDutyCommandPtr() {return &duty_command_;}
  inline double * getDutyStatePtr() {return &duty_state_;}
  // The interface ros2_control sees: "velocity"/"position" for a motor, "duty"/"duty" for a
  // duty output.
  inline const char * commandInterfaceName() const {return duty_output_ ? "duty" : "velocity";}
  inline const char * stateInterfaceName() const {return duty_output_ ? "duty" : "position";}
  inline double * getCommandPtr() {return duty_output_ ? &duty_command_ : &velocity_command_;}
  inline double * getStatePtr() {return duty_output_ ? &duty_state_ : &position_state_;}
  const LimitSwitch * getLimitSwitch() const {return limit_switch_.get();}
};
}  // namespace roboclaw_hardware_interface

#endif  // ROBOCLAW_HARDWARE_INTERFACE__MOTOR_JOINT_HPP_
