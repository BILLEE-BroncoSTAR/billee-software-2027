// P17 / P20 arm backend implemented over Linux SocketCAN (CANopen CiA402).
#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "pxx/can.hpp"
#include "pxx/motor_backend.hpp"

namespace pxx
{

class SocketCanMotorBackend final : public MotorBackend
{
public:
  SocketCanMotorBackend(
    std::string can_interface, std::vector<std::uint8_t> node_ids,
    std::vector<double> gear_ratios);
  ~SocketCanMotorBackend() override;

  bool activate(std::string * error = nullptr) override;
  bool deactivate(std::string * error = nullptr) override;
  bool read(std::vector<JointState> & states, double period_seconds, std::string * error = nullptr) override;
  bool write(
    const std::vector<double> & position_commands_rad, std::string * error = nullptr) override;

private:
  struct MotorFeedback
  {
    std::int32_t position_counts{0};
    std::int32_t velocity_raw{0};
    std::uint16_t status_word{0};
    std::uint16_t error_code{0};  // last EMCY error code
  };

  bool configure_node(std::uint8_t node_id, std::string * error);
  bool sdo_write(
    std::uint8_t node_id, std::uint16_t index, std::uint8_t subindex, std::uint32_t value,
    std::uint8_t size, std::string * error);
  bool sdo_read(
    std::uint8_t node_id, std::uint16_t index, std::uint8_t subindex, std::uint32_t & value,
    std::string * error);
  bool sdo_transaction(const pxx_can::Frame & request, std::uint32_t * value, std::string * error);
  bool send_frame(const pxx_can::Frame & frame, std::string * error);
  void rx_loop();

  std::string can_interface_;
  std::vector<std::uint8_t> node_ids_;
  std::vector<double> gear_ratios_;
  pxx_can::SocketCanTransport can_transport_;
  std::thread rx_thread_;
  std::atomic<bool> rx_running_{false};
  std::mutex feedback_mutex_;
  std::array<MotorFeedback, pxx_can::kMaxNodeId + 1> feedback_{};

  // One SDO transaction is in flight at a time (configuration only).
  std::mutex sdo_mutex_;
  std::condition_variable sdo_cv_;
  std::optional<pxx_can::SdoResponse> sdo_response_;
};

}  // namespace pxx
