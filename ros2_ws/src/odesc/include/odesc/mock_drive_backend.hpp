// Hardware-free wheel-feedback backend for controller and odometry testing.
#pragma once

#include <cstddef>
#include <vector>

#include "odesc/drive_backend.hpp"

namespace odesc
{

class MockDriveBackend final : public DriveBackend
{
public:
  MockDriveBackend(std::size_t joint_count, double gear_ratio, std::string endpoint);

  bool activate(std::string * error = nullptr) override;
  bool deactivate(std::string * error = nullptr) override;
  bool read(std::vector<WheelState> & states, double period_seconds, std::string * error = nullptr) override;
  bool write(
    const std::vector<double> & velocity_commands_rad_s, std::string * error = nullptr) override;

private:
  double gear_ratio_;
  std::vector<double> commands_rad_s_;
  std::vector<WheelState> states_;
};

}  // namespace odesc
