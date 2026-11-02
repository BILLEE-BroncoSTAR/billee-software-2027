// pxx.hpp
//
// ros2_control SystemInterface for the BILLEE arm: P17 / P20 motors on a shared
// CAN bus, one per arm joint, spoken to over a raw Linux SocketCAN socket (no
// extra ROS CAN dependency).
//
// Interfaces per joint:
//   command: position (rad, at the joint)
//   state:   position (rad), velocity (rad/s)
//
// The motor<->joint gear-ratio conversion and transport details live in the
// selected MotorBackend; protocol encoding lives in include/pxx/can.hpp.
//
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "pxx/motor_backend.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace pxx
{

class PxxSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PxxSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ---- configuration (from the URDF <ros2_control> block) ----
  std::vector<std::string> joint_names_;
  std::vector<uint8_t> node_ids_;
  std::vector<double> gear_ratios_;

  // ---- ros2_control interface storage (indexed by joint) ----
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<JointState> backend_states_;
  std::unique_ptr<MotorBackend> backend_;
};

}  // namespace pxx
