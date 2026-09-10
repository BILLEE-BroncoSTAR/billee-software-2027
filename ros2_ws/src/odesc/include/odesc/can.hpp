// ODrive CANSimple protocol helpers and a small Linux SocketCAN transport.
//
// This header deliberately contains no ROS dependencies.  It is shared by the
// ros2_control plugin and can also be used by small diagnostic tools.
#pragma once

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace odrive_can
{

using CommandId = std::uint8_t;

inline constexpr std::uint8_t kCommandIdBits = 5;
inline constexpr std::uint8_t kMaxNodeId = 0x3F;
inline constexpr std::uint8_t kBroadcastNodeId = 0x3F;
inline constexpr std::uint8_t kMaxDataLength = 8;
inline constexpr std::uint16_t CO_HEARTBEAT_MESSAGE_ID = 0x700;

enum : CommandId {
  MSG_CO_NMT_CTRL = 0x00,
  MSG_ODRIVE_HEARTBEAT = 0x01,
  MSG_ODRIVE_ESTOP = 0x02,
  MSG_GET_MOTOR_ERROR = 0x03,
  MSG_GET_ENCODER_ERROR = 0x04,
  MSG_GET_SENSORLESS_ERROR = 0x05,
  MSG_SET_AXIS_NODE_ID = 0x06,
  MSG_SET_AXIS_REQUESTED_STATE = 0x07,
  MSG_SET_AXIS_STARTUP_CONFIG = 0x08,
  MSG_GET_ENCODER_ESTIMATES = 0x09,
  MSG_GET_ENCODER_COUNT = 0x0A,
  MSG_SET_CONTROLLER_MODES = 0x0B,
  MSG_SET_INPUT_POS = 0x0C,
  MSG_SET_INPUT_VEL = 0x0D,
  MSG_SET_INPUT_TORQUE = 0x0E,
  MSG_SET_LIMITS = 0x0F,
  MSG_START_ANTICOGGING = 0x10,
  MSG_SET_TRAJ_VEL_LIMIT = 0x11,
  MSG_SET_TRAJ_ACCEL_LIMITS = 0x12,
  MSG_SET_TRAJ_INERTIA = 0x13,
  MSG_GET_IQ = 0x14,
  MSG_GET_SENSORLESS_ESTIMATES = 0x15,
  MSG_REBOOT_ODRIVE = 0x16,
  MSG_GET_VBUS_VOLTAGE = 0x17,
  MSG_CLEAR_ERRORS = 0x18,
  MSG_SET_LINEAR_COUNT = 0x19,
  MSG_SET_POSITION_GAIN = 0x1A,
  MSG_SET_VEL_GAINS = 0x1B,
  MSG_GET_ADC_VOLTAGE = 0x1C,
  MSG_GET_CONTROLLER_ERROR = 0x1D,
};

enum class AxisState : std::uint32_t {
  kIdle = 1,
  kClosedLoopControl = 8,
};

struct ArbitrationId
{
  std::uint8_t node_id;
  CommandId command_id;
};

struct Frame
{
  std::uint16_t arbitration_id{0};
  std::uint8_t data_length{0};
  std::array<std::uint8_t, kMaxDataLength> data{};
};

struct EncoderEstimate
{
  float position_turns{0.0F};
  float velocity_turns_s{0.0F};
};

constexpr std::uint16_t MakeArbitrationId(std::uint8_t node_id, CommandId command_id)
{
  return static_cast<std::uint16_t>((node_id << kCommandIdBits) | (command_id & 0x1F));
}

constexpr ArbitrationId ParseArbitrationId(std::uint16_t arbitration_id)
{
  return {
    static_cast<std::uint8_t>((arbitration_id >> kCommandIdBits) & kMaxNodeId),
    static_cast<CommandId>(arbitration_id & 0x1F),
  };
}

inline Frame MakeAxisStateFrame(std::uint8_t node_id, AxisState axis_state)
{
  Frame frame{};
  frame.arbitration_id = MakeArbitrationId(node_id, MSG_SET_AXIS_REQUESTED_STATE);
  frame.data_length = 4;
  const auto value = static_cast<std::uint32_t>(axis_state);
  for (std::size_t i = 0; i < frame.data_length; ++i) {
    frame.data[i] = static_cast<std::uint8_t>(value >> (i * 8));
  }
  return frame;
}

inline Frame MakeInputVelocityFrame(std::uint8_t node_id, float velocity_turns_s)
{
  static_assert(sizeof(float) == 4, "ODrive CANSimple requires 32-bit floats");
  static_assert(std::numeric_limits<float>::is_iec559, "ODrive CANSimple requires IEEE-754 floats");

  Frame frame{};
  frame.arbitration_id = MakeArbitrationId(node_id, MSG_SET_INPUT_VEL);
  frame.data_length = 8;  // Input_Vel followed by zero Input_Torque_FF.
  std::uint32_t bits{};
  std::memcpy(&bits, &velocity_turns_s, sizeof(bits));
  for (std::size_t i = 0; i < sizeof(bits); ++i) {
    frame.data[i] = static_cast<std::uint8_t>(bits >> (i * 8));
  }
  return frame;
}

inline std::optional<EncoderEstimate> ParseEncoderEstimate(const Frame & frame)
{
  if (frame.data_length < 8) {
    return std::nullopt;
  }

  std::uint32_t position_bits{};
  std::uint32_t velocity_bits{};
  for (std::size_t i = 0; i < 4; ++i) {
    position_bits |= static_cast<std::uint32_t>(frame.data[i]) << (i * 8);
    velocity_bits |= static_cast<std::uint32_t>(frame.data[i + 4]) << (i * 8);
  }

  EncoderEstimate estimate{};
  std::memcpy(&estimate.position_turns, &position_bits, sizeof(position_bits));
  std::memcpy(&estimate.velocity_turns_s, &velocity_bits, sizeof(velocity_bits));
  return estimate;
}

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
    socket_frame.can_id = frame.arbitration_id & CAN_SFF_MASK;
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
    frame.arbitration_id = static_cast<std::uint16_t>(socket_frame.can_id & CAN_SFF_MASK);
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

}  // namespace odrive_can
