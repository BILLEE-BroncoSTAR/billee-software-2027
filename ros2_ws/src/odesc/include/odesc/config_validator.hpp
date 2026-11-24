#ifndef ODESC_CONFIG_VALIDATOR_HPP
#define ODESC_CONFIG_VALIDATOR_HPP

#include <cstdint>
#include <string>

namespace odesc::validation
{

constexpr uint32_t kMaxCanSimpleNodeId = 63;

bool parse_can_simple_node_id(const std::string & value, uint32_t & node_id);
bool parse_positive_finite_double(const std::string & value, double & parsed_value);
bool parse_boolean(const std::string & value, bool & parsed_value);

}  // namespace odesc::validation

#endif  // ODESC_CONFIG_VALIDATOR_HPP
