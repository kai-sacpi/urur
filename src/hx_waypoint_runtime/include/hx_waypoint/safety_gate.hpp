#pragma once

#include <string>
#include <vector>

#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace hx_waypoint
{

enum class SafetyGateFailureReason
{
  NONE,
  INVALID_TRAJECTORY,
  INVALID_JOINT_STATE,
  STALE_JOINT_STATE,
  NOT_SETTLED,
  POSITION_MISMATCH,
};

struct SafetyGateConfig
{
  double max_joint_error_rad{0.02};
  double settle_velocity_rad_s{0.01};
  double max_joint_state_age_s{0.5};
};

struct SafetyGateResult
{
  bool accepted{false};
  SafetyGateFailureReason failure_reason{SafetyGateFailureReason::NONE};
  std::string message;
  double max_joint_error_rad{0.0};
  double max_abs_velocity_rad_s{0.0};
};

class SafetyGate
{
public:
  explicit SafetyGate(SafetyGateConfig config = {});

  [[nodiscard]] SafetyGateResult checkStartState(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const sensor_msgs::msg::JointState & actual,
    const rclcpp::Time & now) const;

  [[nodiscard]] const SafetyGateConfig & config() const noexcept;

private:
  SafetyGateConfig config_;
};

}  // namespace hx_waypoint
