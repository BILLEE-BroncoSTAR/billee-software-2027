#include "odesc/mock_drive_backend.hpp"

#include <algorithm>
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

MockDriveBackend::MockDriveBackend(std::size_t joint_count, double gear_ratio, std::string endpoint)
: DriveBackend({"mock", std::move(endpoint)}), gear_ratio_(gear_ratio),
  commands_rad_s_(joint_count, 0.0), states_(joint_count)
{
}

bool MockDriveBackend::activate(std::string * /*error*/)
{
  std::fill(commands_rad_s_.begin(), commands_rad_s_.end(), 0.0);
  std::fill(states_.begin(), states_.end(), WheelState{});
  return true;
}

bool MockDriveBackend::deactivate(std::string * /*error*/)
{
  std::fill(commands_rad_s_.begin(), commands_rad_s_.end(), 0.0);
  for (auto & state : states_) {
    state.velocity_rad_s = 0.0;
  }
  return true;
}

bool MockDriveBackend::read(
  std::vector<WheelState> & states, double period_seconds, std::string * error)
{
  if (!std::isfinite(period_seconds)) {
    SetError(error, "mock backend received a non-finite control period");
    return false;
  }

  for (std::size_t i = 0; i < states_.size(); ++i) {
    // Preserve the real backend's wheel -> motor -> wheel conversion round trip.
    const double motor_turns_s = commands_rad_s_[i] * gear_ratio_ / kTwoPi;
    const double wheel_rad_s = motor_turns_s * kTwoPi / gear_ratio_;
    states_[i].velocity_rad_s = wheel_rad_s;
    if (period_seconds > 0.0) {
      states_[i].position_rad += wheel_rad_s * period_seconds;
    }
  }
  states = states_;
  return true;
}

bool MockDriveBackend::write(
  const std::vector<double> & velocity_commands_rad_s, std::string * error)
{
  if (velocity_commands_rad_s.size() != commands_rad_s_.size()) {
    SetError(error, "mock backend command count does not match its joint count");
    return false;
  }
  commands_rad_s_ = velocity_commands_rad_s;
  return true;
}

}  // namespace odesc
