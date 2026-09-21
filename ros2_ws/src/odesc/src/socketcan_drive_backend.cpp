#include "odesc/socketcan_drive_backend.hpp"

#include <cmath>
#include <utility>

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;

void SetError(std::string * error, const std::string & message)
{
  if (error != nullptr) {
    *error = message;
  }
}
}  // namespace

namespace odesc
{

SocketCanDriveBackend::SocketCanDriveBackend(
  std::string can_interface, std::vector<std::uint8_t> node_ids, double gear_ratio)
: DriveBackend({"SocketCAN", can_interface}), can_interface_(std::move(can_interface)),
  node_ids_(std::move(node_ids)), gear_ratio_(gear_ratio)
{
}

SocketCanDriveBackend::~SocketCanDriveBackend()
{
  deactivate();
}

bool SocketCanDriveBackend::activate(std::string * error)
{
  if (!can_transport_.open(can_interface_, error)) {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(estimate_mutex_);
    estimates_.fill(EncoderEstimate{});
  }

  rx_running_ = true;
  rx_thread_ = std::thread(&SocketCanDriveBackend::rx_loop, this);
  request_axis_state_all(odrive_can::AxisState::kClosedLoopControl, error);
  return true;
}

bool SocketCanDriveBackend::deactivate(std::string * error)
{
  bool success = true;
  if (can_transport_.is_open()) {
    success = request_axis_state_all(odrive_can::AxisState::kIdle, error);
  }

  rx_running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  can_transport_.close();
  return success;
}

bool SocketCanDriveBackend::read(
  std::vector<WheelState> & states, double /*period_seconds*/, std::string * error)
{
  if (!can_transport_.is_open()) {
    SetError(error, "SocketCAN backend is not active");
    return false;
  }

  states.resize(node_ids_.size());
  std::lock_guard<std::mutex> lock(estimate_mutex_);
  for (std::size_t i = 0; i < node_ids_.size(); ++i) {
    const auto & estimate = estimates_[node_ids_[i]];
    states[i].position_rad = estimate.position_turns * kTwoPi / gear_ratio_;
    states[i].velocity_rad_s = estimate.velocity_turns_s * kTwoPi / gear_ratio_;
  }
  return true;
}

bool SocketCanDriveBackend::write(
  const std::vector<double> & velocity_commands_rad_s, std::string * error)
{
  if (!can_transport_.is_open()) {
    SetError(error, "SocketCAN backend is not active");
    return false;
  }
  if (velocity_commands_rad_s.size() != node_ids_.size()) {
    SetError(error, "SocketCAN command count does not match its node count");
    return false;
  }

  bool success = true;
  for (std::size_t i = 0; i < node_ids_.size(); ++i) {
    const double motor_turns_s = velocity_commands_rad_s[i] * gear_ratio_ / kTwoPi;
    success &= send_frame(
      odrive_can::MakeInputVelocityFrame(node_ids_[i], static_cast<float>(motor_turns_s)), error);
  }
  return success;
}

bool SocketCanDriveBackend::send_frame(const odrive_can::Frame & frame, std::string * error)
{
  return can_transport_.send(frame, error);
}

bool SocketCanDriveBackend::request_axis_state_all(
  odrive_can::AxisState axis_state, std::string * error)
{
  bool success = true;
  for (const auto node_id : node_ids_) {
    success &= send_frame(odrive_can::MakeAxisStateFrame(node_id, axis_state), error);
  }
  return success;
}

void SocketCanDriveBackend::rx_loop()
{
  while (rx_running_.load()) {
    const auto frame = can_transport_.receive(std::chrono::milliseconds{100});
    if (!frame) {
      continue;
    }

    const auto id = odrive_can::ParseArbitrationId(frame->arbitration_id);
    const auto estimate = odrive_can::ParseEncoderEstimate(*frame);
    if (id.command_id == odrive_can::MSG_GET_ENCODER_ESTIMATES && estimate) {
      std::lock_guard<std::mutex> lock(estimate_mutex_);
      estimates_[id.node_id].position_turns = static_cast<double>(estimate->position_turns);
      estimates_[id.node_id].velocity_turns_s = static_cast<double>(estimate->velocity_turns_s);
    }
  }
}

}  // namespace odesc
