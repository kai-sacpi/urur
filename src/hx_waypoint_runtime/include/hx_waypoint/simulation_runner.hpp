#pragma once

#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "hx_waypoint/playback_executor.hpp"
#include "hx_waypoint/trajectory_recorder.hpp"

namespace hx_waypoint
{

class SimulationRunner
{
public:
  struct Callbacks
  {
    std::function<void()> accepted;
    std::function<void(const control_msgs::action::FollowJointTrajectory::Feedback &)> feedback;
    std::function<void(
      const PlaybackExecutor::GoalHandle::WrappedResult &,
      const std::filesystem::path &)> completed;
    std::function<void(const std::string &)> failed_to_start;
  };

  SimulationRunner(
    rclcpp::Node * node,
    std::string action_name,
    std::vector<std::string> joint_names,
    std::string joint_states_topic);
  SimulationRunner(
    rclcpp::Node * node,
    std::string action_name,
    std::vector<std::string> joint_names,
    TrajectoryRecorderOptions recorder_options);

  void configureRecordingKinematics(const RobotValidationModel & robot);
  [[nodiscard]] bool run(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::filesystem::path & actual_output_file,
    std::chrono::nanoseconds server_timeout,
    Callbacks callbacks);
  [[nodiscard]] bool run(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const SimulationRecordingPaths & output_files,
    std::chrono::nanoseconds server_timeout,
    Callbacks callbacks);
  void cancel();

  [[nodiscard]] const std::string & actionName() const noexcept;

private:
  rclcpp::Node * node_;
  PlaybackExecutor executor_;
  TrajectoryRecorder recorder_;
  std::filesystem::path actual_output_file_;
  SimulationRecordingPaths output_files_;
  bool write_all_outputs_{false};
};

}  // namespace hx_waypoint
