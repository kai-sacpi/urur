#include "hx_waypoint/prepare_workflow.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace hx_waypoint
{
namespace
{

[[nodiscard]] double durationSeconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1.0e-9;
}

[[nodiscard]] bool allFinite(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    });
}

}  // namespace

SettledStateWait::SettledStateWait(
  Clock::time_point started_at, std::chrono::nanoseconds timeout,
  bool allow_target_convergence, std::chrono::nanoseconds dwell,
  std::size_t minimum_samples)
: deadline_(started_at + timeout), dwell_(dwell), minimum_samples_(minimum_samples),
  allow_target_convergence_(allow_target_convergence)
{
  if (timeout <= std::chrono::nanoseconds::zero() ||
    dwell <= std::chrono::nanoseconds::zero() || minimum_samples < 2U)
  {
    throw std::invalid_argument("settled-state wait bounds are invalid");
  }
  last_safety_.message = "no fresh JointState received during settle wait";
}

SettledStateWaitStatus SettledStateWait::poll(Clock::time_point now, bool canceled)
{
  if (status_ == SettledStateWaitStatus::WAITING) {
    if (canceled) {
      status_ = SettledStateWaitStatus::CANCELED;
    } else if (now >= deadline_) {
      status_ = SettledStateWaitStatus::TIMED_OUT;
    }
  }
  return status_;
}

SettledStateWaitStatus SettledStateWait::observe(
  const SafetyGateResult & sample, Clock::time_point now, bool canceled)
{
  if (poll(now, canceled) != SettledStateWaitStatus::WAITING) {
    return status_;
  }
  last_safety_ = sample;
  if (!sample.accepted) {
    settled_samples_ = 0U;
    if (sample.failure_reason != SafetyGateFailureReason::NOT_SETTLED &&
      !(allow_target_convergence_ &&
      sample.failure_reason == SafetyGateFailureReason::POSITION_MISMATCH))
    {
      status_ = SettledStateWaitStatus::REJECTED;
    }
    return status_;
  }
  // A long gap cannot carry an earlier stationary sample into a continuous
  // dwell proof, even if the next arriving sample is itself fresh and settled.
  if (settled_samples_ > 0U && now - last_settled_at_ > dwell_) {
    settled_samples_ = 0U;
  }
  last_settled_at_ = now;
  if (settled_samples_++ == 0U) {
    settled_since_ = now;
  }
  if (settled_samples_ >= minimum_samples_ && now - settled_since_ >= dwell_) {
    status_ = SettledStateWaitStatus::SETTLED;
  }
  return status_;
}

const SafetyGateResult & SettledStateWait::lastSafety() const noexcept
{
  return last_safety_;
}

bool extractOrderedJointPositions(
  const sensor_msgs::msg::JointState & state,
  const std::vector<std::string> & expected_joint_names,
  std::vector<double> & positions,
  std::string & error)
{
  positions.clear();
  error.clear();
  if (expected_joint_names.empty() ||
    std::unordered_set<std::string>(expected_joint_names.begin(), expected_joint_names.end()).size() !=
    expected_joint_names.size())
  {
    error = "expected prepare joint names are empty or duplicated";
    return false;
  }
  if (state.name.size() != state.position.size()) {
    error = "JointState name/position lengths differ";
    return false;
  }
  std::unordered_map<std::string, std::size_t> index;
  for (std::size_t item = 0U; item < state.name.size(); ++item) {
    if (state.name[item].empty() || !index.emplace(state.name[item], item).second) {
      error = "JointState contains an empty or duplicate joint name";
      return false;
    }
  }
  positions.reserve(expected_joint_names.size());
  for (const auto & name : expected_joint_names) {
    const auto found = index.find(name);
    if (found == index.end()) {
      error = "JointState is missing required joint '" + name + "'";
      positions.clear();
      return false;
    }
    const double value = state.position[found->second];
    if (!std::isfinite(value)) {
      error = "JointState contains a non-finite position for joint '" + name + "'";
      positions.clear();
      return false;
    }
    positions.push_back(value);
  }
  return true;
}

PrepareTrajectoryValidationResult validatePrepareTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<std::string> & expected_joint_names,
  const std::vector<double> & expected_start_positions,
  const std::vector<double> & expected_target_positions,
  const PrepareTrajectoryValidationConfig & config)
{
  if (!std::isfinite(config.endpoint_tolerance_rad) || config.endpoint_tolerance_rad <= 0.0 ||
    !std::isfinite(config.endpoint_derivative_tolerance) ||
    config.endpoint_derivative_tolerance < 0.0 || !std::isfinite(config.max_duration_s) ||
    config.max_duration_s <= 0.0)
  {
    throw std::invalid_argument("prepare trajectory validation config is invalid");
  }

  PrepareTrajectoryValidationResult result;
  if (trajectory.joint_names != expected_joint_names) {
    result.message = "prepare trajectory joint_names do not exactly match recording order";
    return result;
  }
  const std::size_t width = expected_joint_names.size();
  if (width == 0U || expected_start_positions.size() != width ||
    expected_target_positions.size() != width || !allFinite(expected_start_positions) ||
    !allFinite(expected_target_positions))
  {
    result.message = "prepare expected endpoints are incomplete or non-finite";
    return result;
  }
  if (trajectory.points.size() < 2U) {
    result.message = "prepare trajectory must contain at least two complete points";
    return result;
  }

  double previous_time_s = -1.0;
  for (std::size_t index = 0U; index < trajectory.points.size(); ++index) {
    const auto & point = trajectory.points[index];
    if (point.positions.size() != width || point.velocities.size() != width ||
      point.accelerations.size() != width || !allFinite(point.positions) ||
      !allFinite(point.velocities) || !allFinite(point.accelerations))
    {
      result.message = "prepare trajectory point " + std::to_string(index) +
        " does not contain finite q/qd/qdd vectors";
      return result;
    }
    const double time_s = durationSeconds(point.time_from_start);
    if (!std::isfinite(time_s) || time_s < 0.0 ||
      (index == 0U && std::abs(time_s) > 1.0e-9) ||
      (index > 0U && time_s <= previous_time_s))
    {
      result.message = "prepare trajectory time_from_start is invalid";
      return result;
    }
    previous_time_s = time_s;
  }
  result.duration_s = previous_time_s;
  if (result.duration_s <= 0.0 || result.duration_s > config.max_duration_s) {
    result.message = "prepare trajectory duration is outside the configured bound";
    return result;
  }

  const auto & first = trajectory.points.front();
  const auto & last = trajectory.points.back();
  for (std::size_t joint = 0U; joint < width; ++joint) {
    result.max_start_error_rad = std::max(
      result.max_start_error_rad,
      std::abs(first.positions[joint] - expected_start_positions[joint]));
    result.max_target_error_rad = std::max(
      result.max_target_error_rad,
      std::abs(last.positions[joint] - expected_target_positions[joint]));
  }
  if (result.max_start_error_rad > config.endpoint_tolerance_rad) {
    result.message = "prepare trajectory start does not match the requested current state";
    return result;
  }
  if (result.max_target_error_rad > config.endpoint_tolerance_rad) {
    result.message = "prepare trajectory end does not match the recording start";
    return result;
  }
  for (const auto * endpoint : {&first, &last}) {
    for (const auto * values : {&endpoint->velocities, &endpoint->accelerations}) {
      if (std::any_of(values->begin(), values->end(), [&config](double value) {
          return std::abs(value) > config.endpoint_derivative_tolerance;
        }))
      {
        result.message = "prepare trajectory endpoints must have zero velocity and acceleration";
        return result;
      }
    }
  }

  result.accepted = true;
  result.message = "prepare trajectory shape, timing, and endpoints are valid";
  return result;
}

}  // namespace hx_waypoint
