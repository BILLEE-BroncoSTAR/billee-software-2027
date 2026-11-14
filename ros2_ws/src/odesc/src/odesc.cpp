#include "odesc/can_helpers.hpp"
#include "odesc/can_simple_messages.hpp"
#include "odesc/config_validator.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odesc/odrive_enums.h"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "odesc/socket_can.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>

// The ODESC boards run pre-0.5.2 ODrive firmware: the heartbeat (0x01) is the only
// cyclic CAN message. Encoder estimates (0x09) are sent only in reply to an RTR request,
// so write() polls every axis each cycle and read() picks the replies up next cycle.

namespace
{
constexpr const char * kLogger = "OdescHardwareInterface";

using SteadyClock = std::chrono::steady_clock;
constexpr auto kEncoderTimeout = std::chrono::milliseconds(250);
constexpr auto kHeartbeatTimeout = std::chrono::milliseconds(500);
constexpr auto kSendFailTimeout = std::chrono::milliseconds(250);
// After activation, give the axes time to process Clear_Errors and reach closed loop.
constexpr auto kActivationGrace = std::chrono::seconds(1);
constexpr auto kLogThrottle = std::chrono::seconds(1);

rclcpp::Logger logger() {return rclcpp::get_logger(kLogger);}

long long ms_since(SteadyClock::time_point now, SteadyClock::time_point then)
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(now - then).count();
}

// Per-axis log throttle (the RCLCPP_*_THROTTLE macros throttle per call site, so one
// failing wheel would hide another).
bool throttle_ok(SteadyClock::time_point now, std::optional<SteadyClock::time_point> & last)
{
  if (last && now - *last < kLogThrottle) {
    return false;
  }
  last = now;
  return true;
}
}  // namespace

namespace odesc_ros2_control{

  enum class FaultPolicy { kWarn, kFault };

  struct Axis{
    Axis(SocketCanIntf* can_interface, std::string joint_name, uint32_t node_id, bool invert,
      double gearing)
    : can_interface_(can_interface), joint_name_(std::move(joint_name)), node_id_(node_id),
      invert_(invert), gearing_(gearing) {}

    void on_can_msg(const can_frame& frame);

    SocketCanIntf* can_interface_;
    std::string joint_name_;
    uint32_t node_id_;
    bool invert_ = false;
    double gearing_ = 1.0;

    // ros2_control interfaces (wheel rad/s, rad).
    double vel_setpoint_ = 0.0;
    double pos_estimate_ = 0.0;
    double vel_estimate_ = 0.0;

    // Health, from the CAN traffic.
    SteadyClock::time_point last_encoder_rx_{};
    SteadyClock::time_point last_heartbeat_rx_{};
    std::optional<SteadyClock::time_point> vel_send_fail_since_;
    uint32_t axis_error_ = 0;
    uint8_t axis_state_ = 0;
    std::optional<SteadyClock::time_point> last_health_log_;
    std::optional<SteadyClock::time_point> last_send_log_;

    template <typename T>
    bool send(const T& msg) const{
      struct can_frame frame{};
      frame.can_id = (node_id_ << 5) | msg.cmd_id;
      frame.can_dlc = msg.msg_length;
      msg.encode_buf(frame.data);

      return can_interface_->send_can_frame(frame);
    }

    // Remote request: the firmware answers with the data frame for cmd_id.
    bool send_rtr(uint8_t cmd_id, uint8_t dlc) const{
      struct can_frame frame{};
      frame.can_id = ((node_id_ << 5) | cmd_id) | CAN_RTR_FLAG;
      frame.can_dlc = dlc;

      return can_interface_->send_can_frame(frame);
    }
  };

  class OdescHardwareInterface final : public hardware_interface::SystemInterface{
    public:
      using return_type = hardware_interface::return_type;
      using State = rclcpp_lifecycle::State;


      CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;
      CallbackReturn on_configure(
        const State& previous_state) override;
      CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
      CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;
      CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;
      CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State & previous_state) override;
      CallbackReturn on_error(
        const rclcpp_lifecycle::State & previous_state) override;

      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
      return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

    private:
        void on_can_msg(const can_frame& frame);
        bool set_axis_command_mode(const Axis& axis);
        bool stop_all_axes();
        bool check_health(SteadyClock::time_point now);

        bool active_ = false;
        EpollEventLoop event_loop_;
        std::vector<Axis> axes_;
        std::string can_interface_name_;
        double gearing_ = 48.0;
        FaultPolicy fault_policy_ = FaultPolicy::kWarn;
        SocketCanIntf can_interface_;
        SteadyClock::time_point activated_at_{};
  };
}

  using namespace odesc_ros2_control;

  using hardware_interface::CallbackReturn;
  using hardware_interface::return_type;

  CallbackReturn OdescHardwareInterface::on_init(const hardware_interface::HardwareInfo& info){
    if(hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS){
      return CallbackReturn::ERROR;
    }

    const auto can_interface = info_.hardware_parameters.find("can_interface");
    const auto gear_ratio = info_.hardware_parameters.find("gear_ratio");
    if (can_interface == info_.hardware_parameters.end() || can_interface->second.empty()) {
      RCLCPP_ERROR(logger(), "missing required 'can_interface' hardware parameter");
      return CallbackReturn::ERROR;
    }
    if (gear_ratio == info_.hardware_parameters.end()) {
      RCLCPP_ERROR(logger(), "missing required 'gear_ratio' hardware parameter");
      return CallbackReturn::ERROR;
    }

    double parsed_gearing = 0.0;
    if (!odesc::validation::parse_positive_finite_double(gear_ratio->second, parsed_gearing)) {
      RCLCPP_ERROR(logger(),
        "gear_ratio must be a positive finite number, got '%s'", gear_ratio->second.c_str());
      return CallbackReturn::ERROR;
    }
    gearing_ = parsed_gearing;

    // fault_policy: what read() does when a wheel is unhealthy. Optional, default warn.
    FaultPolicy fault_policy = FaultPolicy::kWarn;
    const auto fault_policy_parameter = info_.hardware_parameters.find("fault_policy");
    if (fault_policy_parameter != info_.hardware_parameters.end()) {
      if (fault_policy_parameter->second == "fault") {
        fault_policy = FaultPolicy::kFault;
      } else if (fault_policy_parameter->second != "warn") {
        RCLCPP_ERROR(logger(), "fault_policy must be 'warn' or 'fault', got '%s'",
          fault_policy_parameter->second.c_str());
        return CallbackReturn::ERROR;
      }
    }

    // CANSimple reserves six bits of a standard CAN ID for the node ID.
    // Validate the entire map before replacing the driver's active axis map.
    std::vector<Axis> configured_axes;
    configured_axes.reserve(info_.joints.size());
    std::unordered_map<uint32_t, std::string> node_owners;
    for (const auto & joint : info_.joints) {
      const auto node_id_parameter = joint.parameters.find("node_id");
      if (node_id_parameter == joint.parameters.end()) {
        RCLCPP_ERROR(logger(),
          "joint '%s' is missing required node_id parameter", joint.name.c_str());
        return CallbackReturn::ERROR;
      }

      uint32_t node_id = 0;
      if (!odesc::validation::parse_can_simple_node_id(node_id_parameter->second, node_id)) {
        RCLCPP_ERROR(logger(),
          "joint '%s' has invalid node_id '%s'; expected a decimal value in [0, 63]",
          joint.name.c_str(), node_id_parameter->second.c_str());
        return CallbackReturn::ERROR;
      }

      const auto existing_node = node_owners.find(node_id);
      if (existing_node != node_owners.end()) {
        RCLCPP_ERROR(logger(),
          "joint '%s' duplicates CAN node_id %u already assigned to joint '%s'",
          joint.name.c_str(), node_id, existing_node->second.c_str());
        return CallbackReturn::ERROR;
      }
      node_owners.emplace(node_id, joint.name);

      bool invert = false;
      const auto invert_parameter = joint.parameters.find("invert");
      if (invert_parameter != joint.parameters.end()) {
        if (!odesc::validation::parse_boolean(invert_parameter->second, invert)) {
          RCLCPP_ERROR(logger(),
            "joint '%s' has invalid invert value '%s' (use true or false)",
            joint.name.c_str(), invert_parameter->second.c_str());
          return CallbackReturn::ERROR;
        }
      }

      configured_axes.emplace_back(&can_interface_, joint.name, node_id, invert, gearing_);
    }

    can_interface_name_ = can_interface->second;
    fault_policy_ = fault_policy;
    axes_ = std::move(configured_axes);
    RCLCPP_INFO(logger(), "%zu ODESC axes on %s, gear ratio %.2f, fault_policy %s",
      axes_.size(), can_interface_name_.c_str(), gearing_,
      fault_policy_ == FaultPolicy::kFault ? "fault" : "warn");

    return CallbackReturn::SUCCESS;

  }

  CallbackReturn OdescHardwareInterface::on_configure(const State&){
    active_ = false;
    if (!can_interface_.init(
        can_interface_name_, &event_loop_, [this](const can_frame & frame) {on_can_msg(frame);})) {
      RCLCPP_ERROR(logger(), "failed to initialize SocketCAN on %s",
        can_interface_name_.c_str());
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> OdescHardwareInterface::export_state_interfaces(){
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for(size_t i = 0; i < info_.joints.size(); i++){

      state_interfaces.emplace_back(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &axes_[i].pos_estimate_
      );

      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &axes_[i].vel_estimate_
      ));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> OdescHardwareInterface::export_command_interfaces(){
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for(size_t i = 0; i < info_.joints.size(); i++){
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &axes_[i].vel_setpoint_
      ));
    }

    return command_interfaces;

  }

  return_type OdescHardwareInterface::read(const rclcpp::Time&, const rclcpp::Duration&){
    while(can_interface_.read_nonblocking()){
      // repeat until interface has no more messages
    }

    if (!active_) {
      return return_type::OK;
    }
    const bool healthy = check_health(SteadyClock::now());
    return healthy || fault_policy_ == FaultPolicy::kWarn ? return_type::OK : return_type::ERROR;
  }

  return_type OdescHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&){
    if (!active_) {
      RCLCPP_ERROR(logger(), "refusing velocity command while inactive");
      return return_type::ERROR;
    }

    for (const auto& axis : axes_) {
      if (!std::isfinite(axis.vel_setpoint_)) {
        RCLCPP_ERROR(logger(), "refusing non-finite velocity command for %s",
          axis.joint_name_.c_str());
        return return_type::ERROR;
      }
    }

    const auto now = SteadyClock::now();

    // Velocity every cycle (a lost frame is replaced next cycle), all axes before any
    // poll so a full TX queue drops a poll rather than a command.
    for(auto& axis : axes_){
      Set_Input_Vel_msg_t msg;
      const double wheel_rad_s = axis.invert_ ? -axis.vel_setpoint_ : axis.vel_setpoint_;
      // ros2_control commands wheel rad/s; CANSimple expects motor turns/s.
      msg.Input_Vel = static_cast<float>(wheel_rad_s * axis.gearing_ / (2 * M_PI));
      if (axis.send(msg)) {
        axis.vel_send_fail_since_.reset();
        continue;
      }
      const int err = errno;
      if (!axis.vel_send_fail_since_) {
        axis.vel_send_fail_since_ = now;
      }
      if (throttle_ok(now, axis.last_send_log_)) {
        RCLCPP_WARN(logger(), "velocity command to %s (node %u) not sent: %s",
          axis.joint_name_.c_str(), axis.node_id_, std::strerror(err));
      }
    }

    // Encoder poll. A missed poll shows up as a stale encoder in check_health().
    for(auto& axis : axes_){
      if (axis.send_rtr(Get_Encoder_Estimates_msg_t::cmd_id, Get_Encoder_Estimates_msg_t::msg_length)) {
        continue;
      }
      const int err = errno;
      if (throttle_ok(now, axis.last_send_log_)) {
        RCLCPP_WARN(logger(), "encoder request to %s (node %u) not sent: %s",
          axis.joint_name_.c_str(), axis.node_id_, std::strerror(err));
      }
    }

    return return_type::OK;
  }


void Axis::on_can_msg(const can_frame& frame) {
    const uint8_t cmd = frame.can_id & 0x1f;

    auto try_decode = [&](auto& msg) {
        using Msg = std::decay_t<decltype(msg)>;
        if (frame.can_dlc < Msg::msg_length) {
            return false;
        }
        msg.decode_buf(frame.data);
        return true;
    };

    switch (cmd) {
        case Get_Encoder_Estimates_msg_t::cmd_id: {
            if (Get_Encoder_Estimates_msg_t msg; try_decode(msg)) {
                pos_estimate_ = msg.Pos_Estimate * (2 * M_PI) / this->gearing_;
                vel_estimate_ = msg.Vel_Estimate * (2 * M_PI) / this->gearing_;

                if(this->invert_){
                  vel_estimate_ *= -1;
                  pos_estimate_ *= -1;
                }
                last_encoder_rx_ = SteadyClock::now();
            }
        } break;
        case Heartbeat_msg_t::cmd_id: {
            // Bytes 0-3 axis error, byte 4 axis state (same layout on this firmware).
            if (Heartbeat_msg_t msg; try_decode(msg)) {
                axis_error_ = msg.Axis_Error;
                axis_state_ = msg.Axis_State;
                last_heartbeat_rx_ = SteadyClock::now();
            }
        } break;
        default:
            // silently ignore unimplemented command IDs
            break;
    }
}

void OdescHardwareInterface::on_can_msg(const can_frame& frame) {
    // Requests (ours or another tool's) carry no data; extended and error frames aren't
    // CANSimple. Decoding any of them as a reply would corrupt the state.
    if (frame.can_id & (CAN_RTR_FLAG | CAN_EFF_FLAG | CAN_ERR_FLAG)) {
        return;
    }
    const auto can_id = frame.can_id & CAN_SFF_MASK;
    const auto node_id = can_id >> 5;
    for (auto& axis : axes_) {
        if (axis.node_id_ == node_id) {
            axis.on_can_msg(frame);
            return;
        }
    }
}

bool OdescHardwareInterface::check_health(SteadyClock::time_point now) {
    const bool past_grace = now - activated_at_ >= kActivationGrace;
    bool healthy = true;
    for (auto& axis : axes_) {
        std::string reasons;
        auto add = [&reasons](const std::string & reason) {
            reasons += reasons.empty() ? reason : "; " + reason;
        };
        if (now - axis.last_encoder_rx_ > kEncoderTimeout) {
            add("no encoder reply for " + std::to_string(ms_since(now, axis.last_encoder_rx_)) + " ms");
        }
        if (now - axis.last_heartbeat_rx_ > kHeartbeatTimeout) {
            add("no heartbeat for " + std::to_string(ms_since(now, axis.last_heartbeat_rx_)) + " ms");
        }
        if (past_grace && axis.axis_error_ != 0) {
            // Hex only: odrive_enums.h names are for 0.6 firmware, not this one.
            char buf[32];
            std::snprintf(buf, sizeof(buf), "axis_error 0x%08X", axis.axis_error_);
            add(buf);
        }
        if (past_grace && axis.axis_state_ != AXIS_STATE_CLOSED_LOOP_CONTROL) {
            add("axis state " + std::to_string(axis.axis_state_) + ", expected 8 (closed loop)");
        }
        if (axis.vel_send_fail_since_ && now - *axis.vel_send_fail_since_ > kSendFailTimeout) {
            add("velocity commands failing for " +
                std::to_string(ms_since(now, *axis.vel_send_fail_since_)) + " ms");
        }
        if (reasons.empty()) {
            continue;
        }
        healthy = false;
        if (throttle_ok(now, axis.last_health_log_)) {
            RCLCPP_ERROR(logger(), "wheel %s (node %u) unhealthy: %s%s",
              axis.joint_name_.c_str(), axis.node_id_, reasons.c_str(),
              fault_policy_ == FaultPolicy::kFault ? " -> stopping the drivetrain (fault_policy fault)" : "");
        }
    }
    return healthy;
}

bool OdescHardwareInterface::set_axis_command_mode(const Axis& axis) {
    if (!active_) {
        Set_Axis_State_msg_t idle_msg;
        idle_msg.Axis_Requested_State = AXIS_STATE_IDLE;
        return axis.send(idle_msg);
    }

    Clear_Errors_msg_t clear_errors_msg;
    clear_errors_msg.Identify = 0;
    Set_Controller_Mode_msg_t controller_mode_msg;
    controller_mode_msg.Control_Mode = CONTROL_MODE_VELOCITY_CONTROL;
    controller_mode_msg.Input_Mode = INPUT_MODE_VEL_RAMP;
    Set_Input_Vel_msg_t zero_velocity_msg;
    Set_Axis_State_msg_t closed_loop_msg;
    closed_loop_msg.Axis_Requested_State = AXIS_STATE_CLOSED_LOOP_CONTROL;

    return axis.send(clear_errors_msg) && axis.send(controller_mode_msg) &&
           axis.send(zero_velocity_msg) && axis.send(closed_loop_msg);
}

CallbackReturn OdescHardwareInterface::on_activate(const State&) {
    RCLCPP_INFO(logger(), "activating ODescs...");
    active_ = true;
    activated_at_ = SteadyClock::now();
    for (auto& axis : axes_) {
      axis.vel_setpoint_ = 0.0;
      // Start every wheel healthy; real traffic must keep it that way.
      axis.last_encoder_rx_ = activated_at_;
      axis.last_heartbeat_rx_ = activated_at_;
      axis.vel_send_fail_since_.reset();
      axis.axis_error_ = 0;
      axis.axis_state_ = AXIS_STATE_CLOSED_LOOP_CONTROL;
      if (!set_axis_command_mode(axis)) {
        RCLCPP_ERROR(logger(), "failed to activate CAN node %u", axis.node_id_);
        active_ = false;
        for (const auto& idle_axis : axes_) {
          (void)set_axis_command_mode(idle_axis);
        }
        return CallbackReturn::ERROR;
      }
    }
    return CallbackReturn::SUCCESS;
}

// Zero velocity, then IDLE, on every axis. Best effort: tries every axis even if one fails.
bool OdescHardwareInterface::stop_all_axes() {
    bool success = true;
    for (auto& axis : axes_) {
      Set_Input_Vel_msg_t zero_velocity_msg;
      success = axis.send(zero_velocity_msg) && success;
      axis.vel_setpoint_ = 0.0;
    }
    active_ = false;
    for (auto& axis : axes_) {
      success = set_axis_command_mode(axis) && success;
    }
    return success;
}

CallbackReturn OdescHardwareInterface::on_cleanup(const State& previous_state){
  if (active_) {
    (void)on_deactivate(previous_state);
  }
  can_interface_.deinit();
  return CallbackReturn::SUCCESS;
}

CallbackReturn OdescHardwareInterface::on_deactivate(const State&) {
    RCLCPP_INFO(logger(), "deactivating ODescs...");
    if (!stop_all_axes()) {
      RCLCPP_ERROR(logger(), "failed to stop one or more ODESC axes");
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

// Called by ros2_control after read()/write() returned ERROR (e.g. fault_policy fault).
// Leave the wheels stopped, not at their last command, and release the socket: the
// component goes to UNCONFIGURED and a later configure opens a new one.
CallbackReturn OdescHardwareInterface::on_error(const State&) {
    active_ = false;
    if (!can_interface_.is_open()) {
      return CallbackReturn::SUCCESS;  // e.g. configure failed: nothing was ever sent
    }
    RCLCPP_ERROR(logger(), "drivetrain error: stopping all ODESC axes");
    if (!stop_all_axes()) {
      RCLCPP_ERROR(logger(), "failed to stop one or more ODESC axes");
    }
    can_interface_.deinit();
    return CallbackReturn::SUCCESS;
}

CallbackReturn OdescHardwareInterface::on_shutdown(const State& state) {
  return on_cleanup(state);
}

PLUGINLIB_EXPORT_CLASS(odesc_ros2_control::OdescHardwareInterface, hardware_interface::SystemInterface);
