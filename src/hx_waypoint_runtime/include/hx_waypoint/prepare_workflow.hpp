#pragma once

#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "hx_waypoint/safety_gate.hpp"

namespace hx_waypoint
{

enum class SettledStateWaitStatus
{
  WAITING,
  SETTLED,
  REJECTED,
  TIMED_OUT,
  CANCELED,
};

// A wait is not a relaxed safety gate: every contributing, newly received actual
// sample must pass the original gate. A velocity spike restarts the dwell;
// malformed/stale data fail immediately. Only post-motion target convergence may
// explicitly permit POSITION_MISMATCH while waiting.
class SettledStateWait
{
public:
  using Clock = std::chrono::steady_clock;

  explicit SettledStateWait(
    Clock::time_point started_at, std::chrono::nanoseconds timeout,
    bool allow_target_convergence = false,
    std::chrono::nanoseconds dwell = std::chrono::milliseconds(100),
    std::size_t minimum_samples = 3U);

  [[nodiscard]] SettledStateWaitStatus poll(Clock::time_point now, bool canceled = false);
  [[nodiscard]] SettledStateWaitStatus observe(
    const SafetyGateResult & sample, Clock::time_point now, bool canceled = false);
  [[nodiscard]] const SafetyGateResult & lastSafety() const noexcept;

private:
  Clock::time_point deadline_;
  Clock::time_point settled_since_{};
  Clock::time_point last_settled_at_{};
  std::chrono::nanoseconds dwell_;
  std::size_t minimum_samples_;
  std::size_t settled_samples_{0U};
  bool allow_target_convergence_;
  SettledStateWaitStatus status_{SettledStateWaitStatus::WAITING};
  SafetyGateResult last_safety_;
};

struct PrepareTrajectoryValidationConfig
{
  double endpoint_tolerance_rad{1.0e-3};
  double endpoint_derivative_tolerance{1.0e-6};
  double max_duration_s{120.0};
};

struct PrepareTrajectoryValidationResult
{
  bool accepted{false};
  std::string message;
  double duration_s{0.0};
  double max_start_error_rad{0.0};
  double max_target_error_rad{0.0};
};

[[nodiscard]] bool extractOrderedJointPositions(
  const sensor_msgs::msg::JointState & state,
  const std::vector<std::string> & expected_joint_names,
  std::vector<double> & positions,
  std::string & error);

[[nodiscard]] PrepareTrajectoryValidationResult validatePrepareTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<std::string> & expected_joint_names,
  const std::vector<double> & expected_start_positions,
  const std::vector<double> & expected_target_positions,
  const PrepareTrajectoryValidationConfig & config = {});

}  // namespace hx_waypoint
