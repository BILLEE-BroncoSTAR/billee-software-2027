#include "pxx/mock_motor_backend.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace
{
void SetError(std::string * error, const std::string & message)
{
  if (error != nullptr) {
    *error = message;
  }
}
}  // namespace

namespace pxx
{

MockMotorBackend::MockMotorBackend(std::size_t joint_count, std::string endpoint)
: MotorBackend({"mock", std::move(endpoint)}),
  commands_rad_(joint_count, std::numeric_limits<double>::quiet_NaN()), states_(joint_count)
{
}

bool MockMotorBackend::activate(std::string * /*error*/)
{
  std::fill(commands_rad_.begin(), commands_rad_.end(), std::numeric_limits<double>::quiet_NaN());
  std::fill(states_.begin(), states_.end(), JointState{});
  return true;
}

bool MockMotorBackend::deactivate(std::string * /*error*/)
{
  std::fill(commands_rad_.begin(), commands_rad_.end(), std::numeric_limits<double>::quiet_NaN());
  for (auto & state : states_) {
    state.velocity_rad_s = 0.0;
  }
  return true;
}

bool MockMotorBackend::read(
  std::vector<JointState> & states, double period_seconds, std::string * error)
{
  if (!std::isfinite(period_seconds)) {
    SetError(error, "mock backend received a non-finite control period");
    return false;
  }

  // Ideal position servo: the joint reaches the commanded position each cycle.
  for (std::size_t i = 0; i < states_.size(); ++i) {
    if (std::isnan(commands_rad_[i])) {
      states_[i].velocity_rad_s = 0.0;
      continue;
    }
    const double delta = commands_rad_[i] - states_[i].position_rad;
    states_[i].velocity_rad_s = period_seconds > 0.0 ? delta / period_seconds : 0.0;
    states_[i].position_rad = commands_rad_[i];
  }
  states = states_;
  return true;
}

bool MockMotorBackend::write(
  const std::vector<double> & position_commands_rad, std::string * error)
{
  if (position_commands_rad.size() != commands_rad_.size()) {
    SetError(error, "mock backend command count does not match its joint count");
    return false;
  }
  commands_rad_ = position_commands_rad;
  return true;
}

}  // namespace pxx
