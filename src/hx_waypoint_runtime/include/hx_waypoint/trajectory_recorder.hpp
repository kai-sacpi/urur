#pragma once

#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "hx_waypoint/robot_validation_model.hpp"
#include "hx_waypoint/tip_kinematics.hpp"

namespace hx_waypoint
{

struct ActualJointSample
{
  double ros_time_s{0.0};
  double trajectory_time_s{0.0};
  std::vector<double> positions;
  std::vector<double> velocities;
};

struct TrajectoryRecorderOptions
{
  std::string joint_states_topic{"/joint_states"};
  std::string controller_state_topic{"/joint_trajectory_controller/controller_state"};
  std::string base_frame{"base_link"};
  std::vector<std::string> frame_names{
    "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link",
    "wrist_3_link", "tool0", "rod_tip"};
  std::string tip_frame{"rod_tip"};
};

struct SimulationRecordingPaths
{
  std::filesystem::path actual_joint;
  std::filesystem::path controller_state;
  std::filesystem::path joint_frames;
  std::filesystem::path actual_tip;
};

class TrajectoryRecorder
{
public:
  TrajectoryRecorder(
    rclcpp::Node * node,
    std::vector<std::string> joint_names,
    std::string joint_states_topic);
  TrajectoryRecorder(
    rclcpp::Node * node,
    std::vector<std::string> joint_names,
    TrajectoryRecorderOptions options);

  void configureKinematics(const RobotValidationModel & robot);
  void start(
    const rclcpp::Time & trajectory_start,
    double trajectory_duration_s,
    double nominal_sample_dt_s);
  [[nodiscard]] std::filesystem::path stopAndWrite(const std::filesystem::path & output_file);
  [[nodiscard]] SimulationRecordingPaths stopAndWriteAll(
    const SimulationRecordingPaths & output_files);
  void stop();
  [[nodiscard]] bool active() const;
  [[nodiscard]] std::size_t sampleCount() const;

private:
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr message);
  void onControllerState(
    const control_msgs::msg::JointTrajectoryControllerState::SharedPtr message);

  struct ControllerSample
  {
    double ros_time_s{0.0};
    double trajectory_time_s{0.0};
    trajectory_msgs::msg::JointTrajectoryPoint reference;
    trajectory_msgs::msg::JointTrajectoryPoint feedback;
    trajectory_msgs::msg::JointTrajectoryPoint error;
    double speed_scaling{1.0};
  };

  struct FrameSample
  {
    double ros_time_s{0.0};
    double trajectory_time_s{0.0};
    geometry_msgs::msg::TransformStamped transform;
  };

  struct JointPrerollSample
  {
    ActualJointSample actual;
    std::vector<FrameSample> frames;
  };

  void trimPrerollLocked(double newest_ros_time_s);
  bool lockControllerTimeOriginFromPrerollLocked();
  void replayPrerollLocked();
  void appendActualSampleLocked(
    ActualJointSample actual,
    std::vector<FrameSample> frames);
  void appendControllerSampleLocked(ControllerSample sample);

  rclcpp::Node * node_;
  std::vector<std::string> joint_names_;
  TrajectoryRecorderOptions options_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
    controller_subscription_;
  std::unique_ptr<TipKinematics> kinematics_;
  mutable std::mutex mutex_;
  bool active_{false};
  bool controller_time_origin_locked_{false};
  rclcpp::Time recording_arm_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time trajectory_start_{0, 0, RCL_ROS_TIME};
  double trajectory_duration_s_{0.0};
  double nominal_sample_dt_s_{0.0};
  double last_controller_trajectory_time_s_{-1.0};
  std::optional<std::string> recording_error_;
  std::vector<ActualJointSample> samples_;
  std::vector<ControllerSample> controller_samples_;
  std::vector<FrameSample> frame_samples_;
  std::optional<ActualJointSample> cached_actual_sample_;
  std::optional<ControllerSample> cached_controller_sample_;
  std::vector<FrameSample> cached_frame_samples_;
  std::optional<ActualJointSample> start_actual_sample_;
  std::vector<FrameSample> start_frame_samples_;
  std::deque<JointPrerollSample> joint_preroll_;
  std::deque<ControllerSample> controller_preroll_;
};

}  // namespace hx_waypoint
