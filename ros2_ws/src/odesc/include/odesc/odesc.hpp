// odesc.hpp
//
// ros2_control SystemInterface for the BILLEE drivetrain: six ODrive/ODESC
// motor controllers on a shared CAN bus, one per wheel joint, spoken to over a
// raw Linux SocketCAN socket (no extra ROS CAN dependency).
//
// This is the "real hardware" counterpart to the Gazebo ign_ros2_control plugin.
// diff_drive_controller does not know or care which one is loaded — it only ever
// sees wheel-joint position/velocity interfaces. The motor<->wheel gear-ratio
// conversion and transport details live in the selected DriveBackend.
//
// CAN command subset used (see include/odesc/can.hpp for protocol and transport):
//   0x07 Set_Axis_Requested_State  — CLOSED_LOOP_CONTROL on activate, IDLE on stop
//   0x09 Get_Encoder_Estimates     — cyclic RX: two LE float32 = pos/vel in motor turns
//   0x0D Set_Input_Vel             — TX per cycle: commanded motor turns/s
//
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "odesc/drive_backend.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace odesc
{

class OdescSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OdescSystemHardware)

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
  double gear_ratio_{48.0};   // ODESC V4.2 + NEO REV v1.1, per the team 2026-09-06
  std::vector<std::string> joint_names_;
  std::vector<uint8_t> node_ids_;

  // ---- ros2_control interface storage (indexed by joint) ----
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<WheelState> backend_states_;
  std::unique_ptr<DriveBackend> backend_;
};

}  // namespace odesc
