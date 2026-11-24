#include "pxx/socketcan_motor_backend.hpp"

#include <cmath>
#include <cstdio>
#include <utility>

namespace
{
void SetError(std::string * error, const std::string & message)
{
  if (error != nullptr) {
    *error = message;
  }
}

std::string Hex(std::uint32_t value, int width)
{
  char buffer[16];
  std::snprintf(buffer, sizeof(buffer), "0x%0*X", width, value);
  return buffer;
}
}  // namespace

namespace pxx
{

SocketCanMotorBackend::SocketCanMotorBackend(
  std::string can_interface, std::vector<std::uint8_t> node_ids,
  std::vector<double> gear_ratios)
: MotorBackend({"SocketCAN", can_interface}), can_interface_(std::move(can_interface)),
  node_ids_(std::move(node_ids)), gear_ratios_(std::move(gear_ratios))
{
}

SocketCanMotorBackend::~SocketCanMotorBackend()
{
  deactivate();
}

bool SocketCanMotorBackend::activate(std::string * error)
{
  if (!can_transport_.open(can_interface_, error)) {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_.fill(MotorFeedback{});
  }

  rx_running_ = true;
  rx_thread_ = std::thread(&SocketCanMotorBackend::rx_loop, this);

  for (const auto node_id : node_ids_) {
    if (!configure_node(node_id, error)) {
      deactivate();
      return false;
    }
  }
  return true;
}

// Brings one drive from boot to "operation enabled" in cyclic synchronous
// position mode, holding its current position.
bool SocketCanMotorBackend::configure_node(std::uint8_t node_id, std::string * error)
{
  using namespace pxx_can;
  const std::uint32_t rpdo1 = cob::kRpdo1 + node_id;
  const std::uint32_t tpdo1 = cob::kTpdo1 + node_id;
  const std::uint32_t tpdo2 = cob::kTpdo2 + node_id;

  // PDO mapping can only change in pre-operational.
  if (!send_frame(MakeNmtFrame(NmtCommand::kEnterPreOperational, node_id), error)) {
    return false;
  }

  const bool configured =
    sdo_write(node_id, od::kControlWord, 0, control_word::kFaultReset, 2, error) &&
    sdo_write(node_id, od::kModeOfOperation, 0,
      static_cast<std::uint8_t>(Mode::kCyclicSyncPosition), 1, error) &&
    // RPDO1: 607Ah target position, applied on SYNC.
    sdo_write(node_id, od::kRpdo1Communication, 1, kPdoDisabled | rpdo1, 4, error) &&
    sdo_write(node_id, od::kRpdo1Mapping, 0, 0, 1, error) &&
    sdo_write(node_id, od::kRpdo1Mapping, 1, PdoMapEntry(od::kTargetPosition, 0, 32), 4, error) &&
    sdo_write(node_id, od::kRpdo1Mapping, 0, 1, 1, error) &&
    sdo_write(node_id, od::kRpdo1Communication, 2, kPdoTransmissionSyncEvery, 1, error) &&
    sdo_write(node_id, od::kRpdo1Communication, 1, rpdo1, 4, error) &&
    // TPDO1: 6064h position + 606Ch velocity, sent on every SYNC.
    sdo_write(node_id, od::kTpdo1Communication, 1, kPdoDisabled | tpdo1, 4, error) &&
    sdo_write(node_id, od::kTpdo1Mapping, 0, 0, 1, error) &&
    sdo_write(node_id, od::kTpdo1Mapping, 1, PdoMapEntry(od::kPositionActual, 0, 32), 4, error) &&
    sdo_write(node_id, od::kTpdo1Mapping, 2, PdoMapEntry(od::kVelocityActual, 0, 32), 4, error) &&
    sdo_write(node_id, od::kTpdo1Mapping, 0, 2, 1, error) &&
    sdo_write(node_id, od::kTpdo1Communication, 2, kPdoTransmissionSyncEvery, 1, error) &&
    sdo_write(node_id, od::kTpdo1Communication, 1, tpdo1, 4, error) &&
    // TPDO2: 6041h status word, sent on every SYNC.
    sdo_write(node_id, od::kTpdo2Communication, 1, kPdoDisabled | tpdo2, 4, error) &&
    sdo_write(node_id, od::kTpdo2Mapping, 0, 0, 1, error) &&
    sdo_write(node_id, od::kTpdo2Mapping, 1, PdoMapEntry(od::kStatusWord, 0, 16), 4, error) &&
    sdo_write(node_id, od::kTpdo2Mapping, 0, 1, 1, error) &&
    sdo_write(node_id, od::kTpdo2Communication, 2, kPdoTransmissionSyncEvery, 1, error) &&
    sdo_write(node_id, od::kTpdo2Communication, 1, tpdo2, 4, error);
  if (!configured) {
    return false;
  }

  // In CSP the drive moves to 607Ah as soon as it is enabled, so seed the
  // target with the current position before enabling.
  std::uint32_t position{};
  if (!sdo_read(node_id, od::kPositionActual, 0, position, error) ||
    !sdo_write(node_id, od::kTargetPosition, 0, position, 4, error))
  {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_[node_id].position_counts = static_cast<std::int32_t>(position);
  }

  if (!send_frame(MakeNmtFrame(NmtCommand::kStart, node_id), error) ||
    !sdo_write(node_id, od::kControlWord, 0, control_word::kShutdown, 2, error) ||
    !sdo_write(node_id, od::kControlWord, 0, control_word::kSwitchOn, 2, error) ||
    !sdo_write(node_id, od::kControlWord, 0, control_word::kEnableOperation, 2, error))
  {
    return false;
  }

  std::uint32_t status{};
  if (!sdo_read(node_id, od::kStatusWord, 0, status, error)) {
    return false;
  }
  if ((status & status_word::kFault) || !(status & status_word::kOperationEnabled)) {
    std::uint32_t error_code{};
    sdo_read(node_id, od::kErrorCode, 0, error_code, nullptr);
    SetError(
      error, "node " + std::to_string(node_id) + " failed to enable (status word " +
      Hex(status, 4) + ", error " + Hex(error_code, 4) + ": " +
      ErrorCodeText(static_cast<std::uint16_t>(error_code)) + ")");
    return false;
  }
  return true;
}

bool SocketCanMotorBackend::deactivate(std::string * error)
{
  bool success = true;
  if (can_transport_.is_open()) {
    // TODO: hardware — confirm the brake (2014h:01) engages on disable so the
    // arm does not drop when the drives release torque.
    for (const auto node_id : node_ids_) {
      success &= send_frame(
        pxx_can::MakeSdoWriteFrame(
          node_id, pxx_can::od::kControlWord, 0, pxx_can::control_word::kShutdown, 2), error);
      success &= send_frame(
        pxx_can::MakeNmtFrame(pxx_can::NmtCommand::kEnterPreOperational, node_id), error);
    }
  }

  rx_running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  can_transport_.close();
  return success;
}

bool SocketCanMotorBackend::read(
  std::vector<JointState> & states, double /*period_seconds*/, std::string * error)
{
  if (!can_transport_.is_open()) {
    SetError(error, "SocketCAN backend is not active");
    return false;
  }

  // Feedback arrives as TPDOs in response to the SYNC sent by write().
  states.resize(node_ids_.size());
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  for (std::size_t i = 0; i < node_ids_.size(); ++i) {
    const auto & feedback = feedback_[node_ids_[i]];
    if (feedback.status_word & pxx_can::status_word::kFault) {
      SetError(
        error, "node " + std::to_string(node_ids_[i]) + " in fault (status word " +
        Hex(feedback.status_word, 4) + ", EMCY " + Hex(feedback.error_code, 4) + ": " +
        pxx_can::ErrorCodeText(feedback.error_code) + ")");
      return false;
    }
    states[i].position_rad = pxx_can::CountsToRad(feedback.position_counts) / gear_ratios_[i];
    states[i].velocity_rad_s = pxx_can::VelocityToRadS(feedback.velocity_raw) / gear_ratios_[i];
  }
  return true;
}

bool SocketCanMotorBackend::write(
  const std::vector<double> & position_commands_rad, std::string * error)
{
  if (!can_transport_.is_open()) {
    SetError(error, "SocketCAN backend is not active");
    return false;
  }
  if (position_commands_rad.size() != node_ids_.size()) {
    SetError(error, "SocketCAN command count does not match its node count");
    return false;
  }

  bool success = true;
  for (std::size_t i = 0; i < node_ids_.size(); ++i) {
    if (std::isnan(position_commands_rad[i])) {
      continue;  // no controller has claimed this joint yet; drive holds its target
    }
    const double motor_rad = position_commands_rad[i] * gear_ratios_[i];
    success &= send_frame(
      pxx_can::MakeTargetPositionFrame(node_ids_[i], pxx_can::RadToCounts(motor_rad)), error);
  }
  // SYNC latches the RPDO targets and triggers the feedback TPDOs.
  success &= send_frame(pxx_can::MakeSyncFrame(), error);
  return success;
}

bool SocketCanMotorBackend::sdo_write(
  std::uint8_t node_id, std::uint16_t index, std::uint8_t subindex, std::uint32_t value,
  std::uint8_t size, std::string * error)
{
  return sdo_transaction(
    pxx_can::MakeSdoWriteFrame(node_id, index, subindex, value, size), nullptr, error);
}

bool SocketCanMotorBackend::sdo_read(
  std::uint8_t node_id, std::uint16_t index, std::uint8_t subindex, std::uint32_t & value,
  std::string * error)
{
  return sdo_transaction(pxx_can::MakeSdoReadFrame(node_id, index, subindex), &value, error);
}

bool SocketCanMotorBackend::sdo_transaction(
  const pxx_can::Frame & request, std::uint32_t * value, std::string * error)
{
  const auto node_id = static_cast<std::uint8_t>(request.arbitration_id & pxx_can::cob::kNodeMask);
  const auto index = static_cast<std::uint16_t>(request.data[1] | (request.data[2] << 8));
  const auto subindex = request.data[3];
  const std::string object = "node " + std::to_string(node_id) + " SDO " + Hex(index, 4) + ":" +
    Hex(subindex, 2);

  std::unique_lock<std::mutex> lock(sdo_mutex_);
  sdo_response_.reset();
  if (!send_frame(request, error)) {
    return false;
  }
  const bool answered = sdo_cv_.wait_for(
    lock, pxx_can::sdo::kTimeout, [&] {
      return sdo_response_ && sdo_response_->node_id == node_id &&
             sdo_response_->index == index && sdo_response_->subindex == subindex;
    });
  if (!answered) {
    SetError(error, object + " timed out");
    return false;
  }
  if (sdo_response_->aborted) {
    SetError(
      error, object + " aborted: " + Hex(sdo_response_->value, 8) + " (" +
      pxx_can::SdoAbortText(sdo_response_->value) + ")");
    return false;
  }
  if (value != nullptr) {
    *value = sdo_response_->value;
  }
  return true;
}

bool SocketCanMotorBackend::send_frame(const pxx_can::Frame & frame, std::string * error)
{
  return can_transport_.send(frame, error);
}

void SocketCanMotorBackend::rx_loop()
{
  while (rx_running_.load()) {
    const auto frame = can_transport_.receive(std::chrono::milliseconds{100});
    if (!frame) {
      continue;
    }

    if (const auto motion = pxx_can::ParseMotionFeedback(*frame)) {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      feedback_[motion->node_id].position_counts = motion->position_counts;
      feedback_[motion->node_id].velocity_raw = motion->velocity_raw;
    } else if (const auto status = pxx_can::ParseStatusFeedback(*frame)) {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      feedback_[status->node_id].status_word = status->status_word;
    } else if (const auto emergency = pxx_can::ParseEmergency(*frame)) {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      feedback_[emergency->node_id].error_code = emergency->error_code;
    } else if (const auto response = pxx_can::ParseSdoResponse(*frame)) {
      {
        std::lock_guard<std::mutex> lock(sdo_mutex_);
        sdo_response_ = response;
      }
      sdo_cv_.notify_all();
    }
  }
}

}  // namespace pxx
