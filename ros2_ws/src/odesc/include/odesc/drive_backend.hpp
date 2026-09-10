// Common wheel-level interface for drivetrain implementations.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace odesc
{

struct DriveBackendInfo
{
  std::string name;
  std::string endpoint;
};

// ros2_control-facing units: radians and radians per second at the wheel joint.
struct WheelState
{
  double position_rad{0.0};
  double velocity_rad_s{0.0};
};

class DriveBackend
{
public:
  explicit DriveBackend(DriveBackendInfo info) : info_(std::move(info)) {}
  virtual ~DriveBackend() = default;

  const DriveBackendInfo & info() const {return info_;}

  virtual bool activate(std::string * error = nullptr) = 0;
  virtual bool deactivate(std::string * error = nullptr) = 0;

  virtual bool read(
    std::vector<WheelState> & states, double period_seconds, std::string * error = nullptr) = 0;
  virtual bool write(
    const std::vector<double> & velocity_commands_rad_s, std::string * error = nullptr) = 0;

private:
  DriveBackendInfo info_;
};

struct DriveBackendConfig
{
  std::string endpoint;
  std::size_t joint_count{0};
  std::vector<std::uint8_t> node_ids;
  double gear_ratio{1.0};
};

std::unique_ptr<DriveBackend> CreateDriveBackend(const DriveBackendConfig & config);

}  // namespace odesc
