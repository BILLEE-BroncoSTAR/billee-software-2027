// odesc.cpp — implementation of OdescSystemHardware.
//
// See odesc.hpp for the high-level description. This package is Linux-only
// because it communicates through the kernel's SocketCAN interface.

#include "odesc/odesc.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "odesc/can.hpp"
#include "odesc/mock_drive_backend.hpp"
#include "odesc/socketcan_drive_backend.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr const char * kLogger = "OdescSystemHardware";
}  // namespace

namespace odesc
{

std::unique_ptr<DriveBackend> CreateDriveBackend(const DriveBackendConfig & config)
{
  if (config.endpoint == "mock" || config.endpoint == "none") {
    return std::make_unique<MockDriveBackend>(
      config.joint_count, config.gear_ratio, config.endpoint);
  }
  return std::make_unique<SocketCanDriveBackend>(
    config.endpoint, config.node_ids, config.gear_ratio);
}

hardware_interface::CallbackReturn OdescSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---- hardware-level parameters ----
  std::string connection{"can0"};
  if (info_.hardware_parameters.count("can_interface")) {
    connection = info_.hardware_parameters.at("can_interface");
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger(kLogger),
      "no 'can_interface' hardware param set; defaulting to '%s' — confirm this "
      "against the actual Jetson/USB-CAN enumeration before relying on it.",
      connection.c_str());
  }

  if (info_.hardware_parameters.count("gear_ratio")) {
    gear_ratio_ = std::stod(info_.hardware_parameters.at("gear_ratio"));
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger(kLogger),
      "no 'gear_ratio' hardware param set; defaulting to %.3f (ODESC V4.2 + NEO "
      "REV v1.1, per odesc/config/node_map.yaml). Confirm with the §5.4 bench test.",
      gear_ratio_);
  }

  if (gear_ratio_ == 0.0 || !std::isfinite(gear_ratio_)) {
    RCLCPP_FATAL(
      rclcpp::get_logger(kLogger), "gear_ratio must be a non-zero finite number, got %f",
      gear_ratio_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---- per-joint parameters + interface sanity ----
  const size_t n = info_.joints.size();
  joint_names_.reserve(n);
  node_ids_.reserve(n);

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLogger),
        "joint '%s' must have exactly one '%s' command interface.",
        joint.name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    bool has_pos = false, has_vel = false;
    for (const auto & si : joint.state_interfaces) {
      has_pos |= (si.name == hardware_interface::HW_IF_POSITION);
      has_vel |= (si.name == hardware_interface::HW_IF_VELOCITY);
    }
    if (!has_pos || !has_vel) {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLogger),
        "joint '%s' must expose both 'position' and 'velocity' state interfaces.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    auto nid_it = joint.parameters.find("node_id");
    if (nid_it == joint.parameters.end()) {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLogger),
        "joint '%s' is missing the required <param name=\"node_id\"> "
        "(canonical map: odesc/config/node_map.yaml).",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const int nid = std::stoi(nid_it->second);
    if (nid < 0 || nid > static_cast<int>(odrive_can::kMaxNodeId)) {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLogger), "joint '%s' node_id %d out of range [0, %d].",
        joint.name.c_str(), nid, static_cast<int>(odrive_can::kMaxNodeId));
      return hardware_interface::CallbackReturn::ERROR;
    }

    joint_names_.push_back(joint.name);
    node_ids_.push_back(static_cast<uint8_t>(nid));
  }

  hw_positions_.assign(n, 0.0);
  hw_velocities_.assign(n, 0.0);
  hw_commands_.assign(n, 0.0);
  backend_states_.assign(n, WheelState{});

  backend_ = CreateDriveBackend({connection, n, node_ids_, gear_ratio_});
  const auto & backend_info = backend_->info();

  RCLCPP_INFO(
    rclcpp::get_logger(kLogger),
    "initialised %zu joints with %s backend (endpoint='%s', gear_ratio=%.3f).",
    n, backend_info.name.c_str(), backend_info.endpoint.c_str(), gear_ratio_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OdescSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    ifaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    ifaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return ifaces;
}

std::vector<hardware_interface::CommandInterface> OdescSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    ifaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }
  return ifaces;
}

hardware_interface::CallbackReturn OdescSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string backend_error;
  if (!backend_->activate(&backend_error)) {
    RCLCPP_FATAL(rclcpp::get_logger(kLogger), "backend activation failed: %s", backend_error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_positions_.begin(), hw_positions_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);

  const auto & backend_info = backend_->info();
  RCLCPP_INFO(
    rclcpp::get_logger(kLogger),
    "activated %s backend (%zu joints, endpoint='%s', gear_ratio=%.3f).",
    backend_info.name.c_str(), joint_names_.size(), backend_info.endpoint.c_str(), gear_ratio_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdescSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string backend_error;
  if (!backend_->deactivate(&backend_error)) {
    RCLCPP_WARN(rclcpp::get_logger(kLogger), "backend deactivation issue: %s", backend_error.c_str());
  }
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
  const auto & backend_info = backend_->info();
  RCLCPP_INFO(
    rclcpp::get_logger(kLogger), "deactivated %s backend (endpoint='%s').",
    backend_info.name.c_str(), backend_info.endpoint.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdescSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_deactivate(previous_state);
}

hardware_interface::CallbackReturn OdescSystemHardware::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_deactivate(previous_state);
}

hardware_interface::return_type OdescSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  std::string backend_error;
  if (!backend_->read(backend_states_, period.seconds(), &backend_error)) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogger), "backend read failed: %s", backend_error.c_str());
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    hw_positions_[i] = backend_states_[i].position_rad;
    hw_velocities_[i] = backend_states_[i].velocity_rad_s;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OdescSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::string backend_error;
  if (!backend_->write(hw_commands_, &backend_error)) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogger), "backend write failed: %s", backend_error.c_str());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace odesc

PLUGINLIB_EXPORT_CLASS(odesc::OdescSystemHardware, hardware_interface::SystemInterface)
