#include "odesc/can_helpers.hpp"
#include "odesc/can_simple_messages.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odesc/odrive_enums.h"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "odesc/socket_can.hpp"

#include <cmath>
#include <exception>
#include <string>

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;
constexpr const char * kLogger = "OdescHardwareInterface";
}  // namespace

namespace odesc_ros2_control{
  class Axis;

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

      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
      return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

    private:
        void on_can_msg(const can_frame& frame);
        bool set_axis_command_mode(const Axis& axis);

        bool active_ = false;
        EpollEventLoop event_loop_;
        std::vector<Axis> axes_;
        std::string can_interface_name_;
        double gearing_ = 48.0;
        SocketCanIntf can_interface_;
        rclcpp::Time timestamp_;
  };

  struct Axis{
    Axis(SocketCanIntf* can_interface_, uint32_t node_id_, bool invert_, float gearing_ ): can_interface_(can_interface_), node_id_(node_id_), invert_(invert_), gearing_(gearing_){}

    void on_can_msg(const::rclcpp::Time& timestamp, const can_frame& frame);
    void on_can_msg();

    SocketCanIntf* can_interface_;
    uint32_t node_id_;
    bool invert_ = false;
    float gearing_ = 1.0;

    double vel_setpoint_ = 0.0f;
    double last_vel_setpoint_ = NAN;
    double pos_estimate_ = 0.0;
    double vel_estimate_ = 0.0;

    template <typename T>
    bool send(const T& msg) const{
      struct can_frame frame{};
      frame.can_id = (node_id_ << 5) | msg.cmd_id;
      frame.can_dlc = msg.msg_length;
      msg.encode_buf(frame.data);

      return can_interface_->send_can_frame(frame);
    }

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
      RCLCPP_ERROR(rclcpp::get_logger(kLogger), "missing required 'can_interface' hardware parameter");
      return CallbackReturn::ERROR;
    }
    if (gear_ratio == info_.hardware_parameters.end()) {
      RCLCPP_ERROR(rclcpp::get_logger(kLogger), "missing required 'gear_ratio' hardware parameter");
      return CallbackReturn::ERROR;
    }

    try {
      gearing_ = std::stod(gear_ratio->second);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(rclcpp::get_logger(kLogger), "invalid gear_ratio '%s': %s",
        gear_ratio->second.c_str(), error.what());
      return CallbackReturn::ERROR;
    }
    if (!std::isfinite(gearing_) || gearing_ <= 0.0) {
      RCLCPP_ERROR(rclcpp::get_logger(kLogger),
        "gear_ratio must be a positive finite number, got %f", gearing_);
      return CallbackReturn::ERROR;
    }

    can_interface_name_ = can_interface->second;
    axes_.clear();
    axes_.reserve(info_.joints.size());
    try {
      for (const auto & joint : info_.joints) {
        bool invert = false;
        const auto invert_parameter = joint.parameters.find("invert");
        if (invert_parameter != joint.parameters.end()) {
          if (invert_parameter->second == "true" || invert_parameter->second == "1") {
            invert = true;
          } else if (invert_parameter->second != "false" && invert_parameter->second != "0") {
            RCLCPP_ERROR(rclcpp::get_logger(kLogger),
              "joint '%s' has invalid invert value '%s' (use true or false)",
              joint.name.c_str(), invert_parameter->second.c_str());
            return CallbackReturn::ERROR;
          }
        }
        axes_.emplace_back(
          &can_interface_, static_cast<uint32_t>(std::stoul(joint.parameters.at("node_id"))),
          invert, static_cast<float>(gearing_));
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(rclcpp::get_logger(kLogger), "invalid joint node_id: %s", error.what());
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;

  }

  CallbackReturn OdescHardwareInterface::on_configure(const State&){
    active_ = false;
    if (!can_interface_.init(
        can_interface_name_, &event_loop_, std::bind(&OdescHardwareInterface::on_can_msg, this, _1))) {
      RCLCPP_ERROR(rclcpp::get_logger(kLogger), "failed to initialize SocketCAN on %s",
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

  return_type OdescHardwareInterface::read(const rclcpp::Time& timestamp, const rclcpp::Duration&){
    timestamp_ = timestamp;

    while(can_interface_.read_nonblocking()){
      // repeat until interface has no mroe messages
    }

    return return_type::OK;
  }

  return_type OdescHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&){
    if (!active_) {
      RCLCPP_ERROR(rclcpp::get_logger(kLogger), "refusing velocity command while inactive");
      return return_type::ERROR;
    }

    for(auto& axis : axes_){
      if (!std::isfinite(axis.vel_setpoint_)) {
        RCLCPP_ERROR(rclcpp::get_logger(kLogger), "refusing non-finite velocity command");
        return return_type::ERROR;
      }
      if(axis.last_vel_setpoint_ != axis.vel_setpoint_){
        Set_Input_Vel_msg_t msg;
        msg.Input_Vel = axis.invert_ ? -(axis.vel_setpoint_) : axis.vel_setpoint_;
        // ros2_control commands wheel rad/s; CANSimple expects motor turns/s.
        msg.Input_Vel *= axis.gearing_ / kTwoPi;
        if (!axis.send(msg)) {
          RCLCPP_ERROR(rclcpp::get_logger(kLogger),
            "failed to send velocity command to CAN node %u", axis.node_id_);
          return return_type::ERROR;
        }
        // Only acknowledge the setpoint locally after the frame was queued.
        axis.last_vel_setpoint_ = axis.vel_setpoint_;
      }
    }

    return return_type::OK;
  }


void Axis::on_can_msg(const rclcpp::Time&, const can_frame& frame) {
    uint8_t cmd = frame.can_id & 0x1f;

    auto try_decode = [&](auto& msg) {
        if (frame.can_dlc < Get_Encoder_Estimates_msg_t::msg_length) {
            RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterface"), "message %d too short", cmd);
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
            }
        } break;
            // silently ignore unimplemented command IDs
    }
}

void OdescHardwareInterface::on_can_msg(const can_frame& frame) {
    const auto can_id = frame.can_id & CAN_SFF_MASK;
    const auto node_id = can_id >> 5;
    for (auto& axis : axes_) {
        if (axis.node_id_ == node_id) {
            axis.on_can_msg(timestamp_, frame);
            return;
        }
    }
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
    RCLCPP_INFO(rclcpp::get_logger(kLogger), "activating ODescs...");
    active_ = true;
    for (auto& axis : axes_) {
      axis.vel_setpoint_ = 0.0;
      axis.last_vel_setpoint_ = NAN;
      if (!set_axis_command_mode(axis)) {
        RCLCPP_ERROR(rclcpp::get_logger(kLogger), "failed to activate CAN node %u", axis.node_id_);
        active_ = false;
        for (const auto& idle_axis : axes_) {
          (void)set_axis_command_mode(idle_axis);
        }
        return CallbackReturn::ERROR;
      }
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn OdescHardwareInterface::on_cleanup(const State& previous_state){
  if (active_) {
    (void)on_deactivate(previous_state);
  }
  can_interface_.deinit();
  return CallbackReturn::SUCCESS;
}

CallbackReturn OdescHardwareInterface::on_deactivate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger(kLogger), "deactivating ODescs...");
    bool success = true;
    for (auto& axis : axes_) {
      Set_Input_Vel_msg_t zero_velocity_msg;
      success = axis.send(zero_velocity_msg) && success;
      axis.vel_setpoint_ = 0.0;
      axis.last_vel_setpoint_ = NAN;
    }
    active_ = false;
    for (auto& axis : axes_) {
      success = set_axis_command_mode(axis) && success;
    }
    if (!success) {
      RCLCPP_ERROR(rclcpp::get_logger(kLogger), "failed to stop one or more ODESC axes");
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn OdescHardwareInterface::on_shutdown(const State& state) {
  return on_cleanup(state);
}

PLUGINLIB_EXPORT_CLASS(odesc_ros2_control::OdescHardwareInterface, hardware_interface::SystemInterface);
