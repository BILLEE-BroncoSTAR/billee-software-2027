// Sim-only relay: fills in covariance on GPS fixes that arrive without one.
//
// Gazebo's NavSat message has no covariance field, so ros_gz_bridge publishes
// all zeros with COVARIANCE_TYPE_UNKNOWN. robot_localization treats zero
// covariance as a near-perfect measurement, which made every 1 Hz GPS fix kick
// the global EKF heading by 5-11 deg after turns (Test 5). This node republishes
// /gps with a diagonal covariance so the EKF weights GPS against wheel odometry.
//
// Fixes that already carry covariance (e.g. the real ZED-F9P via comms/GpsNode)
// pass through unchanged, so running this by mistake on hardware is harmless.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class GpsCovarianceRelay : public rclcpp::Node
{
public:
  GpsCovarianceRelay()
  : Node("gps_covariance_relay")
  {
    // 0.5 m matches RTK float / no-corrections on the ZED-F9P, the harder case
    // for heading. Use 0.02 to mimic RTK fixed.
    const double horizontal_stddev = declare_parameter("horizontal_stddev", 0.5);
    const double vertical_stddev = declare_parameter("vertical_stddev", 1.0);
    horizontal_var_ = horizontal_stddev * horizontal_stddev;
    vertical_var_ = vertical_stddev * vertical_stddev;

    pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("gps/with_cov", 10);
    // SensorDataQoS (best effort) matches both reliable and best-effort publishers.
    sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "gps", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::NavSatFix::UniquePtr msg) {
        if (msg->position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
          msg->position_covariance.fill(0.0);
          msg->position_covariance[0] = horizontal_var_;  // east
          msg->position_covariance[4] = horizontal_var_;  // north
          msg->position_covariance[8] = vertical_var_;    // up
          msg->position_covariance_type =
            sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        }
        pub_->publish(std::move(msg));
      });

    RCLCPP_INFO(
      get_logger(), "Filling unknown GPS covariance: horizontal %.3f m, vertical %.3f m (1 sigma)",
      horizontal_stddev, vertical_stddev);
  }

private:
  double horizontal_var_;
  double vertical_var_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsCovarianceRelay>());
  rclcpp::shutdown();
  return 0;
}
