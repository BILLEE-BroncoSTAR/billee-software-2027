// ODrive CANSimple drivetrain backend implemented over Linux SocketCAN.
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "odesc/can.hpp"
#include "odesc/drive_backend.hpp"

namespace odesc
{

class SocketCanDriveBackend final : public DriveBackend
{
public:
  SocketCanDriveBackend(
    std::string can_interface, std::vector<std::uint8_t> node_ids, double gear_ratio);
  ~SocketCanDriveBackend() override;

  bool activate(std::string * error = nullptr) override;
  bool deactivate(std::string * error = nullptr) override;
  bool read(std::vector<WheelState> & states, double period_seconds, std::string * error = nullptr) override;
  bool write(
    const std::vector<double> & velocity_commands_rad_s, std::string * error = nullptr) override;

private:
  struct EncoderEstimate
  {
    double position_turns{0.0};
    double velocity_turns_s{0.0};
  };

  bool send_frame(const odrive_can::Frame & frame, std::string * error);
  bool request_axis_state_all(odrive_can::AxisState axis_state, std::string * error);
  void rx_loop();

  std::string can_interface_;
  std::vector<std::uint8_t> node_ids_;
  double gear_ratio_;
  odrive_can::SocketCanTransport can_transport_;
  std::thread rx_thread_;
  std::atomic<bool> rx_running_{false};
  std::mutex estimate_mutex_;
  std::array<EncoderEstimate, odrive_can::kMaxNodeId + 1> estimates_{};
};

}  // namespace odesc
