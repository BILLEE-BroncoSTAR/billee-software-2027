// P17 / P20 CAN protocol helpers and a small Linux SocketCAN transport.
//
// This header deliberately contains no ROS dependencies.  It is shared by the
// ros2_control plugin and can also be used by small diagnostic tools.
//
// The motors speak CANopen (CiA301 communication, CiA402 drive profile) over
// 11-bit CAN2.0A identifiers; see canopen_protocol_descriptions.md.  All
// multi-byte SDO / PDO payloads are little-endian.
//
// The SocketCanTransport below is lifted from odesc/can.hpp and is protocol
// agnostic.
#pragma once

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace pxx_can
{

// ---------------------------------------------------------------------------
// Protocol: CANopen CiA301 / CiA402
// ---------------------------------------------------------------------------

inline constexpr std::uint8_t kMinNodeId = 1;
inline constexpr std::uint8_t kMaxNodeId = 127;
inline constexpr std::uint8_t kMaxDataLength = 8;

// COB-ID = function code (high 4 bits) + node ID (low 7 bits).
namespace cob
{
inline constexpr std::uint32_t kNmt = 0x000;
inline constexpr std::uint32_t kSync = 0x080;
inline constexpr std::uint32_t kEmcy = 0x080;  // + node ID
inline constexpr std::uint32_t kTpdo1 = 0x180;
inline constexpr std::uint32_t kTpdo2 = 0x280;
inline constexpr std::uint32_t kRpdo1 = 0x200;
inline constexpr std::uint32_t kTsdo = 0x580;  // drive -> host SDO response
inline constexpr std::uint32_t kRsdo = 0x600;  // host -> drive SDO request
inline constexpr std::uint32_t kHeartbeat = 0x700;
inline constexpr std::uint32_t kFunctionMask = 0x780;
inline constexpr std::uint32_t kNodeMask = 0x07F;
}  // namespace cob

enum class NmtCommand : std::uint8_t {
  kStart = 0x01,  // -> operational (PDOs active)
  kStop = 0x02,
  kEnterPreOperational = 0x80,  // SDO only; PDO mapping may be changed
  kResetNode = 0x81,
  kResetCommunication = 0x82,
};

// Object dictionary entries used by the driver.
namespace od
{
inline constexpr std::uint16_t kRpdo1Communication = 0x1400;
inline constexpr std::uint16_t kRpdo1Mapping = 0x1600;
inline constexpr std::uint16_t kTpdo1Communication = 0x1800;
inline constexpr std::uint16_t kTpdo2Communication = 0x1801;
inline constexpr std::uint16_t kTpdo1Mapping = 0x1A00;
inline constexpr std::uint16_t kTpdo2Mapping = 0x1A01;
inline constexpr std::uint16_t kErrorCode = 0x603F;
inline constexpr std::uint16_t kControlWord = 0x6040;
inline constexpr std::uint16_t kStatusWord = 0x6041;
inline constexpr std::uint16_t kModeOfOperation = 0x6060;
inline constexpr std::uint16_t kPositionActual = 0x6064;
inline constexpr std::uint16_t kVelocityActual = 0x606C;
inline constexpr std::uint16_t kTargetPosition = 0x607A;
}  // namespace od

// PDO mapping entry: (index << 16) | (subindex << 8) | bit length.
constexpr std::uint32_t PdoMapEntry(std::uint16_t index, std::uint8_t subindex, std::uint8_t bits)
{
  return (static_cast<std::uint32_t>(index) << 16) | (static_cast<std::uint32_t>(subindex) << 8) |
         bits;
}
// Bit 31 of a PDO COB-ID entry (14xx:01 / 18xx:01) disables the PDO.
inline constexpr std::uint32_t kPdoDisabled = 0x80000000;
// PDO transmission type 1: TPDO sent / RPDO applied on every SYNC.
inline constexpr std::uint8_t kPdoTransmissionSyncEvery = 1;

// CiA402 control word (6040h) commands.
namespace control_word
{
inline constexpr std::uint16_t kDisableVoltage = 0x0000;
inline constexpr std::uint16_t kShutdown = 0x0006;         // -> ready to switch on
inline constexpr std::uint16_t kSwitchOn = 0x0007;         // -> switched on
inline constexpr std::uint16_t kEnableOperation = 0x000F;  // -> operation enabled
inline constexpr std::uint16_t kFaultReset = 0x0080;       // rising edge clears fault
}  // namespace control_word

// CiA402 status word (6041h) bits.
namespace status_word
{
inline constexpr std::uint16_t kReadyToSwitchOn = 1u << 0;
inline constexpr std::uint16_t kSwitchedOn = 1u << 1;
inline constexpr std::uint16_t kOperationEnabled = 1u << 2;
inline constexpr std::uint16_t kFault = 1u << 3;
inline constexpr std::uint16_t kVoltageEnabled = 1u << 4;
inline constexpr std::uint16_t kQuickStop = 1u << 5;
inline constexpr std::uint16_t kSwitchOnDisabled = 1u << 6;
inline constexpr std::uint16_t kWarning = 1u << 7;
inline constexpr std::uint16_t kTargetReached = 1u << 10;
}  // namespace status_word

// CiA402 mode of operation (6060h) values.
enum class Mode : std::int8_t {
  kProfilePosition = 1,
  kProfileVelocity = 3,
  kProfileTorque = 4,
  kInterpolatedPosition = 7,
  kCyclicSyncPosition = 8,
  kCyclicSyncVelocity = 9,
  kCyclicSyncTorque = 10,
};

// SDO command specifiers (byte 0).
namespace sdo
{
inline constexpr std::uint8_t kUpload = 0x40;  // read request
inline constexpr std::uint8_t kDownloadResponse = 0x60;  // write acknowledged
inline constexpr std::uint8_t kAbort = 0x80;
inline constexpr std::chrono::milliseconds kTimeout{100};
}  // namespace sdo

// Drive feedback units -> SI.
// encoder counts per motor revolution assumes the P20-90-PRO-121-B, which 
// includes a 1:121 reducer, resulting in an encoder resolution of 7929856 per rotation
inline constexpr double kPositionCountsPerRev = 7929856.0;
inline constexpr double kVelocityCountsPerRevPerSec = 7929856.0;  // assumes counts/s
inline constexpr double kTwoPi = 6.283185307179586;

inline double CountsToRad(std::int32_t counts) {return counts * kTwoPi / kPositionCountsPerRev;}
inline std::int32_t RadToCounts(double rad)
{
  return static_cast<std::int32_t>(std::lround(rad * kPositionCountsPerRev / kTwoPi));
}
inline double VelocityToRadS(std::int32_t raw) {return raw * kTwoPi / kVelocityCountsPerRevPerSec;}

struct Frame
{
  std::uint32_t arbitration_id{0};
  bool extended{false};  // CANopen uses 11-bit IDs; kept for the generic transport
  std::uint8_t data_length{0};
  std::array<std::uint8_t, kMaxDataLength> data{};
};

namespace detail
{
inline void PutLe(Frame & frame, std::size_t offset, std::uint32_t value, std::size_t bytes)
{
  for (std::size_t i = 0; i < bytes; ++i) {
    frame.data[offset + i] = static_cast<std::uint8_t>(value >> (i * 8));
  }
}

inline std::uint32_t GetLe(const Frame & frame, std::size_t offset, std::size_t bytes)
{
  std::uint32_t value{};
  for (std::size_t i = 0; i < bytes; ++i) {
    value |= static_cast<std::uint32_t>(frame.data[offset + i]) << (i * 8);
  }
  return value;
}
}  // namespace detail

// node_id 0 addresses every node.
inline Frame MakeNmtFrame(NmtCommand command, std::uint8_t node_id)
{
  Frame frame{};
  frame.arbitration_id = cob::kNmt;
  frame.data_length = 2;
  frame.data[0] = static_cast<std::uint8_t>(command);
  frame.data[1] = node_id;
  return frame;
}

inline Frame MakeSyncFrame()
{
  Frame frame{};
  frame.arbitration_id = cob::kSync;
  return frame;
}

// Expedited SDO download (write) of 1, 2 or 4 bytes.
inline Frame MakeSdoWriteFrame(
  std::uint8_t node_id, std::uint16_t index, std::uint8_t subindex, std::uint32_t value,
  std::uint8_t size)
{
  Frame frame{};
  frame.arbitration_id = cob::kRsdo + node_id;
  frame.data_length = 8;
  // 0x2F / 0x2B / 0x27 / 0x23 for 1 / 2 / 3 / 4 data bytes.
  frame.data[0] = static_cast<std::uint8_t>(0x23 | ((4 - size) << 2));
  detail::PutLe(frame, 1, index, 2);
  frame.data[3] = subindex;
  detail::PutLe(frame, 4, value, size);
  return frame;
}

inline Frame MakeSdoReadFrame(std::uint8_t node_id, std::uint16_t index, std::uint8_t subindex)
{
  Frame frame{};
  frame.arbitration_id = cob::kRsdo + node_id;
  frame.data_length = 8;
  frame.data[0] = sdo::kUpload;
  detail::PutLe(frame, 1, index, 2);
  frame.data[3] = subindex;
  return frame;
}

// RPDO1 is mapped to 607Ah target position (int32) during activation.
inline Frame MakeTargetPositionFrame(std::uint8_t node_id, std::int32_t target_counts)
{
  Frame frame{};
  frame.arbitration_id = cob::kRpdo1 + node_id;
  frame.data_length = 4;
  detail::PutLe(frame, 0, static_cast<std::uint32_t>(target_counts), 4);
  return frame;
}

struct SdoResponse
{
  std::uint8_t node_id{0};
  std::uint16_t index{0};
  std::uint8_t subindex{0};
  bool aborted{false};
  std::uint32_t value{0};  // read data, or the abort code when aborted
};

inline std::optional<SdoResponse> ParseSdoResponse(const Frame & frame)
{
  if (frame.extended || frame.data_length < 8 ||
    (frame.arbitration_id & cob::kFunctionMask) != cob::kTsdo)
  {
    return std::nullopt;
  }
  SdoResponse response{};
  response.node_id = static_cast<std::uint8_t>(frame.arbitration_id & cob::kNodeMask);
  response.index = static_cast<std::uint16_t>(detail::GetLe(frame, 1, 2));
  response.subindex = frame.data[3];
  response.aborted = frame.data[0] == sdo::kAbort;
  response.value = detail::GetLe(frame, 4, 4);
  return response;
}

// Human-readable text for the abort codes listed in the P series manual.
inline const char * SdoAbortText(std::uint32_t code)
{
  switch (code) {
    case 0x05040000: return "SDO timeout";
    case 0x06010001: return "read not allowed";
    case 0x06010002: return "write not allowed";
    case 0x06020000: return "object does not exist";
    case 0x06090030: return "value out of range";
    default: return "unknown abort code";
  }
}

// Human-readable text for the 603Fh / EMCY error codes in the P series manual.
inline const char * ErrorCodeText(std::uint16_t code)
{
  switch (code) {
    case 0x0000: return "no error";
    case 0x3230: return "overload protection";
    case 0x4210: return "temperature too high";
    case 0x7121: return "motor stalled";
    case 0x7310: return "motor overspeed";
    case 0x8130: return "heartbeat offline";
    case 0x8500: return "speed error too large";
    case 0x8611: return "position error too large";
    default: return "unknown error";
  }
}

// TPDO1 is mapped to 6064h position (int32) + 606Ch velocity (int32).
struct MotionFeedback
{
  std::uint8_t node_id{0};
  std::int32_t position_counts{0};
  std::int32_t velocity_raw{0};
};

inline std::optional<MotionFeedback> ParseMotionFeedback(const Frame & frame)
{
  if (frame.extended || frame.data_length < 8 ||
    (frame.arbitration_id & cob::kFunctionMask) != cob::kTpdo1)
  {
    return std::nullopt;
  }
  MotionFeedback feedback{};
  feedback.node_id = static_cast<std::uint8_t>(frame.arbitration_id & cob::kNodeMask);
  feedback.position_counts = static_cast<std::int32_t>(detail::GetLe(frame, 0, 4));
  feedback.velocity_raw = static_cast<std::int32_t>(detail::GetLe(frame, 4, 4));
  return feedback;
}

// TPDO2 is mapped to 6041h status word (uint16).
struct StatusFeedback
{
  std::uint8_t node_id{0};
  std::uint16_t status_word{0};
};

inline std::optional<StatusFeedback> ParseStatusFeedback(const Frame & frame)
{
  if (frame.extended || frame.data_length < 2 ||
    (frame.arbitration_id & cob::kFunctionMask) != cob::kTpdo2)
  {
    return std::nullopt;
  }
  StatusFeedback feedback{};
  feedback.node_id = static_cast<std::uint8_t>(frame.arbitration_id & cob::kNodeMask);
  feedback.status_word = static_cast<std::uint16_t>(detail::GetLe(frame, 0, 2));
  return feedback;
}

// EMCY shares function code 0x080 with SYNC; SYNC is node 0 with no payload.
struct Emergency
{
  std::uint8_t node_id{0};
  std::uint16_t error_code{0};  // 0x0000 = error reset / no error
};

inline std::optional<Emergency> ParseEmergency(const Frame & frame)
{
  const auto node_id = static_cast<std::uint8_t>(frame.arbitration_id & cob::kNodeMask);
  if (frame.extended || frame.data_length < 2 || node_id == 0 ||
    (frame.arbitration_id & cob::kFunctionMask) != cob::kEmcy)
  {
    return std::nullopt;
  }
  return Emergency{node_id, static_cast<std::uint16_t>(detail::GetLe(frame, 0, 2))};
}

// ---------------------------------------------------------------------------
// Transport (from odesc/can.hpp, plus extended-ID support)
// ---------------------------------------------------------------------------

class SocketCanTransport
{
public:
  SocketCanTransport() = default;
  ~SocketCanTransport() {close();}

  SocketCanTransport(const SocketCanTransport &) = delete;
  SocketCanTransport & operator=(const SocketCanTransport &) = delete;

  bool open(const std::string & interface_name, std::string * error = nullptr)
  {
    close();
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
      SetError(error, "socket(PF_CAN) failed: " + std::string(std::strerror(errno)));
      return false;
    }

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
      SetError(error, "CAN interface '" + interface_name + "' not found: " +
        std::string(std::strerror(errno)));
      close();
      return false;
    }

    struct sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd_, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
      SetError(error, "bind(" + interface_name + ") failed: " +
        std::string(std::strerror(errno)));
      close();
      return false;
    }
    return true;
  }

  void close()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool is_open() const {return fd_ >= 0;}

  bool send(const Frame & frame, std::string * error = nullptr) const
  {
    if (!is_open() || frame.data_length > kMaxDataLength) {
      SetError(error, "SocketCAN transport is closed or frame length is invalid");
      return false;
    }
    struct can_frame socket_frame{};
    socket_frame.can_id = frame.extended ?
      ((frame.arbitration_id & CAN_EFF_MASK) | CAN_EFF_FLAG) :
      (frame.arbitration_id & CAN_SFF_MASK);
    socket_frame.can_dlc = frame.data_length;
    std::memcpy(socket_frame.data, frame.data.data(), frame.data_length);
    if (::write(fd_, &socket_frame, sizeof(socket_frame)) !=
      static_cast<ssize_t>(sizeof(socket_frame)))
    {
      SetError(error, "CAN write failed: " + std::string(std::strerror(errno)));
      return false;
    }
    return true;
  }

  std::optional<Frame> receive(
    std::chrono::milliseconds timeout, std::string * error = nullptr) const
  {
    if (!is_open()) {
      SetError(error, "SocketCAN transport is closed");
      return std::nullopt;
    }
    struct pollfd poll_fd{};
    poll_fd.fd = fd_;
    poll_fd.events = POLLIN;
    const int poll_result = ::poll(&poll_fd, 1, static_cast<int>(timeout.count()));
    if (poll_result == 0 || (poll_result < 0 && errno == EINTR)) {
      return std::nullopt;
    }
    if (poll_result < 0 || !(poll_fd.revents & POLLIN)) {
      SetError(error, "CAN poll failed: " + std::string(std::strerror(errno)));
      return std::nullopt;
    }

    struct can_frame socket_frame{};
    if (::read(fd_, &socket_frame, sizeof(socket_frame)) != static_cast<ssize_t>(sizeof(socket_frame))) {
      SetError(error, "CAN read failed: " + std::string(std::strerror(errno)));
      return std::nullopt;
    }

    Frame frame{};
    frame.extended = (socket_frame.can_id & CAN_EFF_FLAG) != 0;
    frame.arbitration_id = frame.extended ?
      (socket_frame.can_id & CAN_EFF_MASK) : (socket_frame.can_id & CAN_SFF_MASK);
    frame.data_length = socket_frame.can_dlc;
    std::memcpy(frame.data.data(), socket_frame.data, frame.data_length);
    return frame;
  }

private:
  static void SetError(std::string * error, const std::string & message)
  {
    if (error != nullptr) {
      *error = message;
    }
  }

  int fd_{-1};
};

}  // namespace pxx_can
