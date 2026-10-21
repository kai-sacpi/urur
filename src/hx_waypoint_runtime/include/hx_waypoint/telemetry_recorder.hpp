#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <hx_waypoint_interfaces/msg/execution_telemetry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "hx_waypoint/tip_kinematics.hpp"

namespace hx_waypoint
{

struct TelemetryRecorderOptions
{
  std::string joint_states_topic{"/joint_states"};
  std::string telemetry_topic{"~/execution_telemetry"};
  std::string base_frame{"base_link"};
  std::string tip_frame{"rod_tip"};
  std::string speed_scaling_topic{"/speed_scaling_state_broadcaster/speed_scaling"};
};

struct DesiredJointState
{
  std::vector<double> positions;
  std::vector<double> velocities;
};

[[nodiscard]] DesiredJointState sampleDesiredJointState(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  double trajectory_time_s);

[[nodiscard]] bool isUnitSpeedScaling(double speed_scaling, double tolerance) noexcept;
[[nodiscard]] double urSpeedScalingPercentToRatio(double speed_scaling_percent) noexcept;

class TelemetryRecorder
{
public:
  using SpeedScalingViolationCallback = std::function<void(const std::string &)>;

  TelemetryRecorder(
    rclcpp::Node * node,
    std::vector<std::string> joint_names,
    TelemetryRecorderOptions options = {});
  ~TelemetryRecorder();

  [[nodiscard]] bool prepare(
    const trajectory_msgs::msg::JointTrajectory & desired_trajectory,
    const std::filesystem::path & environment_snapshot,
    std::string & error);
  [[nodiscard]] bool start(
    const std::filesystem::path & output_file,
    const rclcpp::Time & trajectory_start,
    const std::string & state = "EXECUTING");
  [[nodiscard]] std::filesystem::path stop(const std::string & final_state);
  void setSpeedScaling(double speed_scaling);
  [[nodiscard]] bool armStrictSpeedScalingGuard(
    std::chrono::nanoseconds fresh_sample_timeout,
    double tolerance,
    std::string & error);
  // Read-only pre-submit proof. Never re-arm or clear a latched violation.
  [[nodiscard]] bool checkArmedStrictSpeedScalingGuard(std::string & error) const;
  [[nodiscard]] bool activateStrictSpeedScalingGuard(
    SpeedScalingViolationCallback on_violation,
    std::string & error);
  [[nodiscard]] bool finalizeStrictSpeedScalingGuard(std::string & violation);
  void clearStrictSpeedScalingGuard();
  void updateTrajectoryProgress(double trajectory_time_s, const rclcpp::Time & received_at);
  [[nodiscard]] bool active() const;

private:
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr message);
  void onSpeedScaling(const std_msgs::msg::Float64::SharedPtr message);
  void checkStrictSpeedScalingFreshness();

  rclcpp::Node * node_;
  std::vector<std::string> joint_names_;
  TelemetryRecorderOptions options_;
  rclcpp::Publisher<hx_waypoint_interfaces::msg::ExecutionTelemetry>::SharedPtr publisher_;
  rclcpp::CallbackGroup::SharedPtr joint_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::CallbackGroup::SharedPtr speed_scaling_callback_group_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_scaling_subscription_;
  rclcpp::TimerBase::SharedPtr speed_scaling_watchdog_;

  mutable std::mutex mutex_;
  std::condition_variable speed_scaling_condition_;
  std::atomic_bool active_{false};
  bool prepared_{false};
  std::string state_;
  rclcpp::Time trajectory_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_feedback_received_at_{0, 0, RCL_ROS_TIME};
  double trajectory_time_s_{0.0};
  std::int64_t trajectory_time_ns_{0};
  double trajectory_duration_s_{0.0};
  trajectory_msgs::msg::JointTrajectory desired_trajectory_;
  std::unique_ptr<TipKinematics> tip_kinematics_;
  geometry_msgs::msg::Pose latest_tip_pose_;
  geometry_msgs::msg::Twist latest_tip_twist_;
  double desired_tip_speed_mps_{0.0};
  double position_error_m_{0.0};
  double speed_scaling_{1.0};
  double latest_speed_scaling_sample_{1.0};
  std::chrono::steady_clock::time_point latest_speed_scaling_received_at_{};
  std::uint64_t speed_scaling_sequence_{0U};
  bool strict_speed_scaling_guard_armed_{false};
  bool strict_speed_scaling_guard_active_{false};
  double strict_speed_scaling_tolerance_{0.0};
  std::chrono::nanoseconds strict_speed_scaling_fresh_timeout_{0};
  std::string strict_speed_scaling_violation_;
  SpeedScalingViolationCallback speed_scaling_violation_callback_;
  std::filesystem::path output_file_;
  std::ofstream output_;
};

}  // namespace hx_waypoint
