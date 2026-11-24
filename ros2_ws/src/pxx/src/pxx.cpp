// pxx.cpp — implementation of PxxSystemHardware.
//
// See pxx.hpp for the high-level description. This package is Linux-only
// because it communicates through the kernel's SocketCAN interface.

#include "pxx/pxx.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "pxx/can.hpp"
#include "pxx/mock_motor_backend.hpp"
#include "pxx/socketcan_motor_backend.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr const char * kLogger = "PxxSystemHardware";
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

namespace pxx
{

std::unique_ptr<MotorBackend> CreateMotorBackend(const MotorBackendConfig & config)
{
  if (config.endpoint == "mock" || config.endpoint == "none") {
    return std::make_unique<MockMotorBackend>(config.node_ids.size(), config.endpoint);
  }
  return std::make_unique<SocketCanMotorBackend>(
    config.endpoint, config.node_ids, config.gear_ratios);
}

hardware_interface::CallbackReturn PxxSystemHardware::on_init(
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
      "no 'can_interface' hardware param set; defaulting to '%s'.", connection.c_str());
  }

  // ---- per-joint parameters + interface sanity ----
  const size_t n = info_.joints.size();
  joint_names_.reserve(n);
  node_ids_.reserve(n);
  gear_ratios_.reserve(n);

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLogger),
        "joint '%s' must have exactly one '%s' command interface.",
        joint.name.c_str(), hardware_interface::HW_IF_POSITION);
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
        "(canonical map: pxx/config/node_map.yaml).",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const int nid = std::stoi(nid_it->second);
    if (nid < static_cast<int>(pxx_can::kMinNodeId) || nid > static_cast<int>(pxx_can::kMaxNodeId)) {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLogger), "joint '%s' node_id %d out of range [%d, %d].",
        joint.name.c_str(), nid, static_cast<int>(pxx_can::kMinNodeId),
        static_cast<int>(pxx_can::kMaxNodeId));
      return hardware_interface::CallbackReturn::ERROR;
    }

    double gear_ratio = 1.0;
    auto gr_it = joint.parameters.find("gear_ratio");
    if (gr_it != joint.parameters.end()) {
      gear_ratio = std::stod(gr_it->second);
    }
    if (gear_ratio == 0.0 || !std::isfinite(gear_ratio)) {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLogger), "joint '%s' gear_ratio must be a non-zero finite number, got %f",
        joint.name.c_str(), gear_ratio);
      return hardware_interface::CallbackReturn::ERROR;
    }

    joint_names_.push_back(joint.name);
    node_ids_.push_back(static_cast<uint8_t>(nid));
    gear_ratios_.push_back(gear_ratio);
  }

  hw_positions_.assign(n, 0.0);
  hw_velocities_.assign(n, 0.0);
  hw_commands_.assign(n, kNaN);
  backend_states_.assign(n, JointState{});

  backend_ = CreateMotorBackend({connection, node_ids_, gear_ratios_});
  const auto & backend_info = backend_->info();

  RCLCPP_INFO(
    rclcpp::get_logger(kLogger),
    "initialised %zu joints with %s backend (endpoint='%s').",
    n, backend_info.name.c_str(), backend_info.endpoint.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> PxxSystemHardware::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface> PxxSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    ifaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return ifaces;
}

hardware_interface::CallbackReturn PxxSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string backend_error;
  if (!backend_->activate(&backend_error)) {
    RCLCPP_FATAL(rclcpp::get_logger(kLogger), "backend activation failed: %s", backend_error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Unlike a velocity interface, a zero position command is NOT safe — it would
  // drive every joint to 0 rad. Commands stay NaN (nothing sent) until a
  // controller writes one; joint_trajectory_controller seeds them from state.
  std::fill(hw_commands_.begin(), hw_commands_.end(), kNaN);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);

  const auto & backend_info = backend_->info();
  RCLCPP_INFO(
    rclcpp::get_logger(kLogger),
    "activated %s backend (%zu joints, endpoint='%s').",
    backend_info.name.c_str(), joint_names_.size(), backend_info.endpoint.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PxxSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string backend_error;
  if (!backend_->deactivate(&backend_error)) {
    RCLCPP_WARN(rclcpp::get_logger(kLogger), "backend deactivation issue: %s", backend_error.c_str());
  }
  std::fill(hw_commands_.begin(), hw_commands_.end(), kNaN);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
  const auto & backend_info = backend_->info();
  RCLCPP_INFO(
    rclcpp::get_logger(kLogger), "deactivated %s backend (endpoint='%s').",
    backend_info.name.c_str(), backend_info.endpoint.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PxxSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_deactivate(previous_state);
}

hardware_interface::CallbackReturn PxxSystemHardware::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_deactivate(previous_state);
}

hardware_interface::return_type PxxSystemHardware::read(
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

hardware_interface::return_type PxxSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::string backend_error;
  if (!backend_->write(hw_commands_, &backend_error)) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogger), "backend write failed: %s", backend_error.c_str());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace pxx

PLUGINLIB_EXPORT_CLASS(pxx::PxxSystemHardware, hardware_interface::SystemInterface)
