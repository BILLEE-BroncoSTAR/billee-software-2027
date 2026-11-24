// Hardware-free joint-feedback backend for controller testing.
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "pxx/motor_backend.hpp"

namespace pxx
{

class MockMotorBackend final : public MotorBackend
{
public:
  MockMotorBackend(std::size_t joint_count, std::string endpoint);

  bool activate(std::string * error = nullptr) override;
  bool deactivate(std::string * error = nullptr) override;
  bool read(std::vector<JointState> & states, double period_seconds, std::string * error = nullptr) override;
  bool write(
    const std::vector<double> & position_commands_rad, std::string * error = nullptr) override;

private:
  std::vector<double> commands_rad_;
  std::vector<JointState> states_;
};

}  // namespace pxx
