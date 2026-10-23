#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zed2i/geometry_utils.hpp"
#include "zed2i/synthetic_vio_constants.hpp"

namespace zed2i
{
struct NoiseModel
{
  double position_noise_std{0.03};              // m, independent sample noise
  double yaw_noise_std{0.02};                   // rad, independent sample noise
  double position_drift_rw_std{0.003};          // m / sqrt(s)
  double yaw_drift_rw_std{0.003};               // rad / sqrt(s)
  double translation_scale_std{0.005};          // dimensionless, per run
  double yaw_scale_std{0.005};                  // dimensionless, per run
  double dropout_probability{0.0};              // probability per source message
  double outlier_probability{0.0};              // probability per source message
  double outlier_position_std{0.50};            // m
  double outlier_yaw_std{0.35};                 // rad
  int latency_ms{0};                            // delivery latency, message stamp is retained
  double reported_covariance_scale{1.0};        // 1 = covariance agrees with nominal model
};

struct PendingMessage
{
  rclcpp::Time release_time;
  nav_msgs::msg::Odometry message;
};

namespace
{
uint64_t fnv1a(const std::string & text)
{
  uint64_t value = 14695981039346656037ULL;
  for (const unsigned char character : text) {
    value ^= character;
    value *= 1099511628211ULL;
  }
  return value;
}

uint64_t stream_seed(uint64_t seed, const std::string & name)
{
  // SplitMix64 finalizer: independent, stable streams without implementation-defined hashes.
  uint64_t value = seed ^ (fnv1a(name) + 0x9e3779b97f4a7c15ULL);
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

}  // namespace

class SyntheticVioNode : public rclcpp::Node
{
public:
  SyntheticVioNode()
  : Node("synthetic_vio")
  {
    truth_topic_ = declare_parameter<std::string>("truth_topic", "/gz/odom");
    vio_topic_ = declare_parameter<std::string>("vio_topic", "/sim/zed/vio/odom");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "odom");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
    profile_ = declare_parameter<std::string>("profile", "calibration");
    const auto requested_seed = declare_parameter<int64_t>("seed", -1);

    nominal_model_.position_noise_std = declare_parameter<double>("position_noise_std", 0.03);
    nominal_model_.yaw_noise_std = declare_parameter<double>("yaw_noise_std", 0.02);
    nominal_model_.position_drift_rw_std =
      declare_parameter<double>("position_drift_rw_std", 0.003);
    nominal_model_.yaw_drift_rw_std = declare_parameter<double>("yaw_drift_rw_std", 0.003);
    nominal_model_.translation_scale_std =
      declare_parameter<double>("translation_scale_std", 0.005);
    nominal_model_.yaw_scale_std = declare_parameter<double>("yaw_scale_std", 0.005);
    nominal_model_.dropout_probability = declare_parameter<double>("dropout_probability", 0.0);
    nominal_model_.outlier_probability = declare_parameter<double>("outlier_probability", 0.0);
    nominal_model_.outlier_position_std =
      declare_parameter<double>("outlier_position_std", 0.50);
    nominal_model_.outlier_yaw_std = declare_parameter<double>("outlier_yaw_std", 0.35);
    nominal_model_.latency_ms = declare_parameter<int>("latency_ms", 0);
    nominal_model_.reported_covariance_scale =
      declare_parameter<double>("reported_covariance_scale", 1.0);

    validation_min_multiplier_ = declare_parameter<double>("validation_min_multiplier", 0.5);
    validation_max_multiplier_ = declare_parameter<double>("validation_max_multiplier", 2.0);
    stress_min_multiplier_ = declare_parameter<double>("stress_min_multiplier", 2.0);
    stress_max_multiplier_ = declare_parameter<double>("stress_max_multiplier", 5.0);
    stress_dropout_probability_ = declare_parameter<double>("stress_dropout_probability", 0.05);
    stress_outlier_probability_ = declare_parameter<double>("stress_outlier_probability", 0.01);
    stress_latency_ms_ = declare_parameter<int>("stress_latency_ms", 150);
    stress_reported_covariance_scale_ =
      declare_parameter<double>("stress_reported_covariance_scale", 0.7);

    validate_parameters(requested_seed);
    resolved_seed_ = resolve_seed(requested_seed);
    initialize_random_streams();
    model_ = select_model();
    sample_run_calibration();

    publisher_ = create_publisher<nav_msgs::msg::Odometry>(vio_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      truth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SyntheticVioNode::truth_callback, this, std::placeholders::_1));

    // The timer is deliberately independent of truth callbacks so delayed measurements
    // are published even if the truth publisher pauses after the final sample.
    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(1), [this]() {publish_due_messages();});

    RCLCPP_INFO(
      get_logger(),
      "Synthetic VIO profile='%s', seed=%llu, source='%s', output='%s'. "
      "This node publishes no TF; let the EKF be the sole odom->base_link publisher.",
      profile_.c_str(), static_cast<unsigned long long>(resolved_seed_), truth_topic_.c_str(),
      vio_topic_.c_str());
  }

private:
  void validate_parameters(int64_t requested_seed)
  {
    if (
      profile_ != "calibration" && profile_ != "nominal" && profile_ != "validation" &&
      profile_ != "stress")
    {
      throw std::invalid_argument(
              "profile must be one of: calibration, validation, stress (nominal is an alias)");
    }
    if (requested_seed < -1) {
      throw std::invalid_argument("seed must be -1 (random) or a non-negative integer");
    }
    if (validation_min_multiplier_ <= 0.0 ||
      validation_max_multiplier_ < validation_min_multiplier_ ||
      stress_min_multiplier_ <= 0.0 || stress_max_multiplier_ < stress_min_multiplier_)
    {
      throw std::invalid_argument("profile multiplier ranges must be positive and ordered");
    }
    const double probabilities[] = {
      nominal_model_.dropout_probability, nominal_model_.outlier_probability,
      stress_dropout_probability_, stress_outlier_probability_};
    for (const auto probability : probabilities) {
      if (probability < 0.0 || probability > 1.0) {
        throw std::invalid_argument("all probabilities must be in [0, 1]");
      }
    }
    if (nominal_model_.latency_ms < 0 || stress_latency_ms_ < 0) {
      throw std::invalid_argument("latency values must be non-negative");
    }
  }

  uint64_t resolve_seed(int64_t requested_seed)
  {
    if (requested_seed >= 0) {
      return static_cast<uint64_t>(requested_seed);
    }
    std::random_device random_device;
    return (static_cast<uint64_t>(random_device()) << 32U) ^ random_device();
  }

  void initialize_random_streams()
  {
    white_rng_.seed(stream_seed(resolved_seed_, "white_noise"));
    drift_rng_.seed(stream_seed(resolved_seed_, "drift"));
    fault_rng_.seed(stream_seed(resolved_seed_, "faults"));
    profile_rng_.seed(stream_seed(resolved_seed_, "profile"));
  }

  NoiseModel select_model()
  {
    NoiseModel selected = nominal_model_;
    if (profile_ == "calibration" || profile_ == "nominal") {
      return selected;
    }

    const auto range = profile_ == "validation" ?
      std::pair<double, double>{validation_min_multiplier_, validation_max_multiplier_} :
    std::pair<double, double>{stress_min_multiplier_, stress_max_multiplier_};
    std::uniform_real_distribution<double> multiplier_distribution(range.first, range.second);
    const auto scale = [&]() {return multiplier_distribution(profile_rng_);};
    // Sample independently so validation spans combinations rather than merely a "noisier" run.
    selected.position_noise_std *= scale();
    selected.yaw_noise_std *= scale();
    selected.position_drift_rw_std *= scale();
    selected.yaw_drift_rw_std *= scale();
    selected.translation_scale_std *= scale();
    selected.yaw_scale_std *= scale();

    if (profile_ == "stress") {
      selected.dropout_probability = stress_dropout_probability_;
      selected.outlier_probability = stress_outlier_probability_;
      selected.latency_ms = stress_latency_ms_;
      selected.reported_covariance_scale = stress_reported_covariance_scale_;
    }
    return selected;
  }

  void sample_run_calibration()
  {
    std::normal_distribution<double> standard_normal(0.0, 1.0);
    translation_scale_ = 1.0 + model_.translation_scale_std * standard_normal(drift_rng_);
    yaw_scale_ = 1.0 + model_.yaw_scale_std * standard_normal(drift_rng_);
  }

  double white_noise(double standard_deviation)
  {
    return std::normal_distribution<double>(0.0, standard_deviation)(white_rng_);
  }

  double drift_noise(double standard_deviation)
  {
    return std::normal_distribution<double>(0.0, standard_deviation)(drift_rng_);
  }

  void truth_callback(const nav_msgs::msg::Odometry::SharedPtr truth)
  {
    const auto stamp = rclcpp::Time(truth->header.stamp);
    if (stamp.nanoseconds() == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring ground-truth odometry with a zero timestamp; use Gazebo simulation time.");
      return;
    }

    if (!initialized_) {
      initial_x_ = truth->pose.pose.position.x;
      initial_y_ = truth->pose.pose.position.y;
      initial_yaw_ = geometry::yaw_from_quaternion(truth->pose.pose.orientation);
      previous_stamp_ = stamp;
      initialized_ = true;
    }

    const double dt = std::max(0.0, (stamp - previous_stamp_).seconds());
    if (dt > 0.0) {
      const double root_dt = std::sqrt(dt);
      position_drift_x_ += drift_noise(model_.position_drift_rw_std * root_dt);
      position_drift_y_ += drift_noise(model_.position_drift_rw_std * root_dt);
      yaw_drift_ += drift_noise(model_.yaw_drift_rw_std * root_dt);
      elapsed_seconds_ += dt;
      previous_stamp_ = stamp;
    }

    std::uniform_real_distribution<double> unit_uniform(0.0, 1.0);
    if (unit_uniform(fault_rng_) < model_.dropout_probability) {
      return;
    }

    auto vio = make_measurement(*truth);
    const bool outlier = unit_uniform(fault_rng_) < model_.outlier_probability;
    if (outlier) {
      vio.pose.pose.position.x += white_noise(model_.outlier_position_std);
      vio.pose.pose.position.y += white_noise(model_.outlier_position_std);
      const auto yaw = geometry::wrap_angle(
        geometry::yaw_from_quaternion(vio.pose.pose.orientation) +
        white_noise(model_.outlier_yaw_std));
      vio.pose.pose.orientation = geometry::quaternion_from_yaw(yaw);
    }

    PendingMessage pending;
    pending.release_time = stamp + rclcpp::Duration::from_nanoseconds(
      static_cast<int64_t>(model_.latency_ms) * 1000000LL);
    pending.message = std::move(vio);
    pending_messages_.push_back(std::move(pending));
    publish_due_messages(stamp);
  }

  nav_msgs::msg::Odometry make_measurement(const nav_msgs::msg::Odometry & truth)
  {
    nav_msgs::msg::Odometry vio;
    vio.header.stamp = truth.header.stamp;
    vio.header.frame_id = output_frame_id_;
    vio.child_frame_id = child_frame_id_;

    const double truth_yaw = geometry::yaw_from_quaternion(truth.pose.pose.orientation);
    const double relative_x = truth.pose.pose.position.x - initial_x_;
    const double relative_y = truth.pose.pose.position.y - initial_y_;
    const double relative_yaw = geometry::wrap_angle(truth_yaw - initial_yaw_);
    const double measurement_yaw = geometry::wrap_angle(
      initial_yaw_ + yaw_scale_ * relative_yaw + yaw_drift_ + white_noise(model_.yaw_noise_std));

    vio.pose.pose.position.x = initial_x_ + translation_scale_ * relative_x + position_drift_x_ +
      white_noise(model_.position_noise_std);
    vio.pose.pose.position.y = initial_y_ + translation_scale_ * relative_y + position_drift_y_ +
      white_noise(model_.position_noise_std);
    vio.pose.pose.position.z = truth.pose.pose.position.z;
    vio.pose.pose.orientation = geometry::quaternion_from_yaw(measurement_yaw);

    set_covariance(vio, relative_x, relative_y, relative_yaw);
    for (std::size_t index = 0; index < 6; ++index) {
      vio.twist.covariance[index * 6 + index] = kUnusedCovariance;
    }
    return vio;
  }

  void set_covariance(
    nav_msgs::msg::Odometry & vio, double relative_x, double relative_y, double relative_yaw)
  {
    // Nominally this reflects the model's expected uncertainty. The stress profile can
    // intentionally under/over-report it to test consistency and rejection behaviour.
    const double position_scale_error = model_.translation_scale_std *
      std::hypot(relative_x, relative_y);
    const double yaw_scale_error = model_.yaw_scale_std * std::abs(relative_yaw);
    const double position_std = model_.reported_covariance_scale * std::sqrt(
      model_.position_noise_std * model_.position_noise_std +
      position_scale_error * position_scale_error +
      model_.position_drift_rw_std * model_.position_drift_rw_std * elapsed_seconds_);
    const double yaw_std = model_.reported_covariance_scale * std::sqrt(
      model_.yaw_noise_std * model_.yaw_noise_std +
      yaw_scale_error * yaw_scale_error +
      model_.yaw_drift_rw_std * model_.yaw_drift_rw_std * elapsed_seconds_);

    for (std::size_t index = 0; index < 6; ++index) {
      vio.pose.covariance[index * 6 + index] = kUnusedCovariance;
    }
    vio.pose.covariance[0] = position_std * position_std;
    vio.pose.covariance[7] = position_std * position_std;
    vio.pose.covariance[35] = yaw_std * yaw_std;
  }

  void publish_due_messages()
  {
    publish_due_messages(get_clock()->now());
  }

  void publish_due_messages(const rclcpp::Time & time)
  {
    while (!pending_messages_.empty() && pending_messages_.front().release_time <= time) {
      publisher_->publish(pending_messages_.front().message);
      pending_messages_.pop_front();
    }
  }

  std::string truth_topic_;
  std::string vio_topic_;
  std::string output_frame_id_;
  std::string child_frame_id_;
  std::string profile_;
  double validation_min_multiplier_{0.5};
  double validation_max_multiplier_{2.0};
  double stress_min_multiplier_{2.0};
  double stress_max_multiplier_{5.0};
  double stress_dropout_probability_{0.05};
  double stress_outlier_probability_{0.01};
  int stress_latency_ms_{150};
  double stress_reported_covariance_scale_{0.7};
  NoiseModel nominal_model_;
  NoiseModel model_;

  uint64_t resolved_seed_{0};
  std::mt19937_64 white_rng_;
  std::mt19937_64 drift_rng_;
  std::mt19937_64 fault_rng_;
  std::mt19937_64 profile_rng_;
  double translation_scale_{1.0};
  double yaw_scale_{1.0};
  bool initialized_{false};
  double initial_x_{0.0};
  double initial_y_{0.0};
  double initial_yaw_{0.0};
  rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};
  double elapsed_seconds_{0.0};
  double position_drift_x_{0.0};
  double position_drift_y_{0.0};
  double yaw_drift_{0.0};
  std::deque<PendingMessage> pending_messages_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};
}  // namespace zed2i

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<zed2i::SyntheticVioNode>());
  rclcpp::shutdown();
  return 0;
}
