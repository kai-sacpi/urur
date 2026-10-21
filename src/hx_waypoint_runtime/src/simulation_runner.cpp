#include "hx_waypoint/simulation_runner.hpp"

#include <exception>
#include <stdexcept>
#include <utility>

namespace hx_waypoint
{
namespace
{

[[nodiscard]] double durationSeconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1.0e-9;
}

}  // namespace

SimulationRunner::SimulationRunner(
  rclcpp::Node * node,
  std::string action_name,
  std::vector<std::string> joint_names,
  std::string joint_states_topic)
: SimulationRunner(
    node, std::move(action_name), std::move(joint_names),
    [&joint_states_topic]() {
      TrajectoryRecorderOptions options;
      options.joint_states_topic = std::move(joint_states_topic);
      return options;
    }())
{
}

SimulationRunner::SimulationRunner(
  rclcpp::Node * node,
  std::string action_name,
  std::vector<std::string> joint_names,
  TrajectoryRecorderOptions recorder_options)
: node_(node),
  executor_(node, std::move(action_name)),
  recorder_(node, std::move(joint_names), std::move(recorder_options))
{
  if (node_ == nullptr) {
    throw std::invalid_argument("SimulationRunner requires a node");
  }
}

void SimulationRunner::configureRecordingKinematics(const RobotValidationModel & robot)
{
  recorder_.configureKinematics(robot);
}

bool SimulationRunner::run(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::filesystem::path & actual_output_file,
  std::chrono::nanoseconds server_timeout,
  Callbacks callbacks)
{
  if (!executor_.waitForServer(server_timeout)) {
    if (callbacks.failed_to_start) {
      callbacks.failed_to_start(
        "simulation FollowJointTrajectory server is unavailable at " + executor_.actionName());
    }
    return false;
  }
  actual_output_file_ = actual_output_file;
  write_all_outputs_ = false;
  auto shared_callbacks = std::make_shared<Callbacks>(std::move(callbacks));
  const double trajectory_duration_s = durationSeconds(
    trajectory.points.back().time_from_start);
  const double nominal_sample_dt_s = durationSeconds(
    trajectory.points[1].time_from_start) - durationSeconds(
    trajectory.points.front().time_from_start);
  PlaybackExecutor::Callbacks executor_callbacks;
  executor_callbacks.accepted =
    [this, shared_callbacks, trajectory_duration_s, nominal_sample_dt_s]() {
      // Recording begins only after the simulation controller has accepted the complete goal.
      recorder_.start(node_->now(), trajectory_duration_s, nominal_sample_dt_s);
      if (shared_callbacks->accepted) {
        shared_callbacks->accepted();
      }
    };
  executor_callbacks.feedback = [shared_callbacks](
    const control_msgs::action::FollowJointTrajectory::Feedback & feedback) {
      if (shared_callbacks->feedback) {
        shared_callbacks->feedback(feedback);
      }
    };
  executor_callbacks.completed = [this, shared_callbacks](
    const PlaybackExecutor::GoalHandle::WrappedResult & result) {
      std::filesystem::path written_path;
      try {
        if (write_all_outputs_) {
          const auto paths = recorder_.stopAndWriteAll(output_files_);
          written_path = paths.actual_joint;
        } else {
          written_path = recorder_.stopAndWrite(actual_output_file_);
        }
      } catch (const std::exception & error) {
        if (shared_callbacks->failed_to_start) {
          shared_callbacks->failed_to_start(
            "simulation completed but actual-state recording failed: " + std::string(error.what()));
        }
        return;
      }
      if (shared_callbacks->completed) {
        shared_callbacks->completed(result, written_path);
      }
    };
  executor_callbacks.failed_to_start = [this, shared_callbacks](const std::string & error) {
      recorder_.stop();
      if (shared_callbacks->failed_to_start) {
        shared_callbacks->failed_to_start(error);
      }
    };
  return executor_.sendTrajectory(trajectory, std::move(executor_callbacks));
}

bool SimulationRunner::run(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const SimulationRecordingPaths & output_files,
  std::chrono::nanoseconds server_timeout,
  Callbacks callbacks)
{
  if (!executor_.waitForServer(server_timeout)) {
    if (callbacks.failed_to_start) {
      callbacks.failed_to_start(
        "simulation FollowJointTrajectory server is unavailable at " + executor_.actionName());
    }
    return false;
  }
  output_files_ = output_files;
  actual_output_file_ = output_files.actual_joint;
  write_all_outputs_ = true;
  auto shared_callbacks = std::make_shared<Callbacks>(std::move(callbacks));
  const double trajectory_duration_s = durationSeconds(
    trajectory.points.back().time_from_start);
  const double nominal_sample_dt_s = durationSeconds(
    trajectory.points[1].time_from_start) - durationSeconds(
    trajectory.points.front().time_from_start);
  PlaybackExecutor::Callbacks executor_callbacks;
  executor_callbacks.accepted =
    [this, shared_callbacks, trajectory_duration_s, nominal_sample_dt_s]() {
      recorder_.start(node_->now(), trajectory_duration_s, nominal_sample_dt_s);
      if (shared_callbacks->accepted) {
        shared_callbacks->accepted();
      }
    };
  executor_callbacks.feedback = [shared_callbacks](
    const control_msgs::action::FollowJointTrajectory::Feedback & feedback) {
      if (shared_callbacks->feedback) {
        shared_callbacks->feedback(feedback);
      }
    };
  executor_callbacks.completed = [this, shared_callbacks](
    const PlaybackExecutor::GoalHandle::WrappedResult & result) {
      try {
        const auto paths = recorder_.stopAndWriteAll(output_files_);
        if (shared_callbacks->completed) {
          shared_callbacks->completed(result, paths.actual_joint);
        }
      } catch (const std::exception & error) {
        if (shared_callbacks->failed_to_start) {
          shared_callbacks->failed_to_start(
            "simulation completed but required recording outputs failed: " +
            std::string(error.what()));
        }
      }
    };
  executor_callbacks.failed_to_start = [this, shared_callbacks](const std::string & error) {
      recorder_.stop();
      if (shared_callbacks->failed_to_start) {
        shared_callbacks->failed_to_start(error);
      }
    };
  return executor_.sendTrajectory(trajectory, std::move(executor_callbacks));
}

void SimulationRunner::cancel()
{
  executor_.cancelActive();
}

const std::string & SimulationRunner::actionName() const noexcept
{
  return executor_.actionName();
}

}  // namespace hx_waypoint
