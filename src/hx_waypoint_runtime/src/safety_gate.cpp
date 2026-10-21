#include "hx_waypoint/safety_gate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace hx_waypoint
{

SafetyGate::SafetyGate(SafetyGateConfig config)
: config_(config)
{
  if (!std::isfinite(config_.max_joint_error_rad) || config_.max_joint_error_rad <= 0.0) {
    throw std::invalid_argument("max_joint_error_rad must be finite and positive");
  }
  if (!std::isfinite(config_.settle_velocity_rad_s) || config_.settle_velocity_rad_s < 0.0) {
    throw std::invalid_argument("settle_velocity_rad_s must be finite and non-negative");
  }
  if (!std::isfinite(config_.max_joint_state_age_s) || config_.max_joint_state_age_s <= 0.0) {
    throw std::invalid_argument("max_joint_state_age_s must be finite and positive");
  }
}

SafetyGateResult SafetyGate::checkStartState(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const sensor_msgs::msg::JointState & actual,
  const rclcpp::Time & now) const
{
  SafetyGateResult result;
  if (trajectory.joint_names.empty() || trajectory.points.empty()) {
    result.failure_reason = SafetyGateFailureReason::INVALID_TRAJECTORY;
    result.message = "recorded trajectory has no joints or points";
    return result;
  }
  const auto & first = trajectory.points.front();
  if (first.positions.size() != trajectory.joint_names.size()) {
    result.failure_reason = SafetyGateFailureReason::INVALID_TRAJECTORY;
    result.message = "recorded first point does not contain one position per joint";
    return result;
  }
  if (!std::all_of(first.positions.begin(), first.positions.end(), [](double value) {
      return std::isfinite(value);
    }))
  {
    result.failure_reason = SafetyGateFailureReason::INVALID_TRAJECTORY;
    result.message = "recorded first point contains a non-finite position";
    return result;
  }
  if (actual.name.size() != actual.position.size()) {
    result.failure_reason = SafetyGateFailureReason::INVALID_JOINT_STATE;
    result.message = "latest JointState name/position lengths differ";
    return result;
  }
  if (actual.velocity.size() != actual.name.size()) {
    result.failure_reason = SafetyGateFailureReason::INVALID_JOINT_STATE;
    result.message = "latest JointState has no complete velocity vector; settled state cannot be proven";
    return result;
  }

  if (actual.header.stamp.sec != 0 || actual.header.stamp.nanosec != 0U) {
    const rclcpp::Time stamp(actual.header.stamp, now.get_clock_type());
    const double age_s = (now - stamp).seconds();
    if (!std::isfinite(age_s) || age_s > config_.max_joint_state_age_s) {
      result.failure_reason = SafetyGateFailureReason::STALE_JOINT_STATE;
      result.message = "latest JointState is stale";
      return result;
    }
    if (age_s < -0.1) {
      result.failure_reason = SafetyGateFailureReason::STALE_JOINT_STATE;
      result.message = "latest JointState timestamp is unexpectedly in the future";
      return result;
    }
  }

  std::unordered_map<std::string, std::size_t> actual_index;
  actual_index.reserve(actual.name.size());
  for (std::size_t index = 0; index < actual.name.size(); ++index) {
    if (actual.name[index].empty() || !actual_index.emplace(actual.name[index], index).second) {
      result.failure_reason = SafetyGateFailureReason::INVALID_JOINT_STATE;
      result.message = "latest JointState contains an empty or duplicate joint name";
      return result;
    }
  }

  result.max_joint_error_rad = 0.0;
  result.max_abs_velocity_rad_s = 0.0;
  std::string worst_position_joint;
  std::string worst_velocity_joint;
  for (std::size_t trajectory_index = 0; trajectory_index < trajectory.joint_names.size();
    ++trajectory_index)
  {
    const auto & joint_name = trajectory.joint_names[trajectory_index];
    const auto found = actual_index.find(joint_name);
    if (found == actual_index.end()) {
      result.failure_reason = SafetyGateFailureReason::INVALID_JOINT_STATE;
      result.message = "latest JointState is missing required joint '" + joint_name + "'";
      return result;
    }
    const double position = actual.position[found->second];
    const double velocity = actual.velocity[found->second];
    if (!std::isfinite(position) || !std::isfinite(velocity)) {
      result.failure_reason = SafetyGateFailureReason::INVALID_JOINT_STATE;
      result.message = "latest JointState contains a non-finite value for joint '" + joint_name + "'";
      return result;
    }
    const double error = std::abs(position - first.positions[trajectory_index]);
    const double speed = std::abs(velocity);
    if (error > result.max_joint_error_rad) {
      result.max_joint_error_rad = error;
      worst_position_joint = joint_name;
    }
    if (speed > result.max_abs_velocity_rad_s) {
      result.max_abs_velocity_rad_s = speed;
      worst_velocity_joint = joint_name;
    }
  }

  if (result.max_abs_velocity_rad_s >= config_.settle_velocity_rad_s) {
    result.failure_reason = SafetyGateFailureReason::NOT_SETTLED;
    std::ostringstream message;
    message << "robot is not settled at " << worst_velocity_joint << ": max speed " <<
      result.max_abs_velocity_rad_s << " rad/s exceeds " << config_.settle_velocity_rad_s <<
      " rad/s";
    result.message = message.str();
    return result;
  }
  if (result.max_joint_error_rad >= config_.max_joint_error_rad) {
    result.failure_reason = SafetyGateFailureReason::POSITION_MISMATCH;
    std::ostringstream message;
    message << "start state mismatch at " << worst_position_joint << ": max error " <<
      result.max_joint_error_rad << " rad must be < " << config_.max_joint_error_rad << " rad";
    result.message = message.str();
    return result;
  }

  result.accepted = true;
  result.failure_reason = SafetyGateFailureReason::NONE;
  result.message = "start state and settle velocity checks passed";
  return result;
}

const SafetyGateConfig & SafetyGate::config() const noexcept
{
  return config_;
}

}  // namespace hx_waypoint
