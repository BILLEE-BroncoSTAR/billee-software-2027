// Common joint-level interface for arm motor implementations.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace pxx
{

struct MotorBackendInfo
{
  std::string name;
  std::string endpoint;
};

// ros2_control-facing units: radians and radians per second at the arm joint.
struct JointState
{
  double position_rad{0.0};
  double velocity_rad_s{0.0};
};

class MotorBackend
{
public:
  explicit MotorBackend(MotorBackendInfo info) : info_(std::move(info)) {}
  virtual ~MotorBackend() = default;

  const MotorBackendInfo & info() const {return info_;}

  virtual bool activate(std::string * error = nullptr) = 0;
  virtual bool deactivate(std::string * error = nullptr) = 0;

  virtual bool read(
    std::vector<JointState> & states, double period_seconds, std::string * error = nullptr) = 0;
  // A NaN command means "no command yet" — the backend sends nothing for that joint.
  virtual bool write(
    const std::vector<double> & position_commands_rad, std::string * error = nullptr) = 0;

private:
  MotorBackendInfo info_;
};

struct MotorBackendConfig
{
  std::string endpoint;
  std::vector<std::uint8_t> node_ids;
  std::vector<double> gear_ratios;  // per joint, motor turns per joint turn
};

std::unique_ptr<MotorBackend> CreateMotorBackend(const MotorBackendConfig & config);

}  // namespace pxx
