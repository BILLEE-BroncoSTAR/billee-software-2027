#include "odesc/config_validator.hpp"

#include <charconv>
#include <cmath>
#include <exception>

namespace odesc::validation
{

bool parse_can_simple_node_id(const std::string & value, uint32_t & node_id)
{
  if (value.empty()) {
    return false;
  }

  const auto result = std::from_chars(
    value.data(), value.data() + value.size(), node_id, 10);
  return result.ec == std::errc{} && result.ptr == value.data() + value.size() &&
         node_id <= kMaxCanSimpleNodeId;
}

bool parse_positive_finite_double(const std::string & value, double & parsed_value)
{
  try {
    std::size_t parsed_characters = 0;
    const double candidate = std::stod(value, &parsed_characters);
    if (parsed_characters != value.size() || !std::isfinite(candidate) || candidate <= 0.0) {
      return false;
    }
    parsed_value = candidate;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool parse_boolean(const std::string & value, bool & parsed_value)
{
  if (value == "true" || value == "1") {
    parsed_value = true;
    return true;
  }
  if (value == "false" || value == "0") {
    parsed_value = false;
    return true;
  }
  return false;
}

}  // namespace odesc::validation
