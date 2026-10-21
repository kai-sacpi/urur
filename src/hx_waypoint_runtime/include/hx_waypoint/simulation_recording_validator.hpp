#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "hx_waypoint/robot_validation_model.hpp"
#include "hx_waypoint/trajectory_recorder.hpp"

namespace hx_waypoint
{

struct SamplingValidationMetrics
{
  std::size_t sample_count{0U};
  double first_trajectory_time_s{0.0};
  double last_trajectory_time_s{0.0};
  double coverage_duration_s{0.0};
  double max_time_interval_s{0.0};
  double max_time_jitter_s{0.0};
  std::size_t missing_sample_estimate{0U};
};

struct SimulationValidationConfig
{
  double sample_dt_s{0.0};
  std::size_t max_missing_samples{0U};
  double max_time_jitter_s{0.0};
  double max_quaternion_norm_error{0.0};
  bool require_action_success{true};
  double max_completion_overrun_s{0.0};
  double max_controller_reference_position_error_rad{0.0};
  double max_controller_reference_velocity_error_rad_s{0.0};
  double max_joint_position_tracking_error_rad{0.0};
  double max_joint_velocity_tracking_error_rad_s{0.0};
  double max_actual_controller_position_error_rad{0.0};
  double max_actual_controller_velocity_error_rad_s{0.0};
  double max_frame_fk_position_error_m{0.0};
  double max_frame_fk_orientation_error_deg{0.0};
};

struct SimulationKinematicsValidationContext
{
  RobotValidationModel robot;
  std::filesystem::path setup_config_path;
  std::filesystem::path source_trajectory_path;
  double max_tip_source_position_error_m{0.0};
  double max_tip_source_orientation_error_deg{0.0};
};

struct CanonicalTipPoseMetrics
{
  double max_tip_position_error_m{0.0};
  double max_tip_orientation_error_deg{0.0};
};

struct PoseValidationMetrics
{
  SamplingValidationMetrics sampling;
  double max_quaternion_norm_error{0.0};
};

struct FrameValidationMetrics
{
  std::string frame_id;
  PoseValidationMetrics pose;
};

struct SimulationRecordingMetrics
{
  SamplingValidationMetrics actual_joint;
  SamplingValidationMetrics controller_state;
  std::size_t joint_frame_sample_count{0U};
  std::vector<FrameValidationMetrics> joint_frame_streams;
  double max_joint_frame_quaternion_norm_error{0.0};
  PoseValidationMetrics actual_tip;
  double max_speed_scaling_error{0.0};
  double max_controller_reference_position_error_rad{0.0};
  double max_controller_reference_velocity_error_rad_s{0.0};
  double max_joint_position_tracking_error_rad{0.0};
  double max_joint_velocity_tracking_error_rad_s{0.0};
  double max_actual_controller_position_error_rad{0.0};
  double max_actual_controller_velocity_error_rad_s{0.0};
  double max_frame_fk_position_error_m{0.0};
  double max_frame_fk_orientation_error_deg{0.0};
  double max_tip_fk_position_error_m{0.0};
  double max_tip_fk_orientation_error_deg{0.0};
  double max_tip_source_position_error_m{0.0};
  double max_tip_source_orientation_error_deg{0.0};
};

class SimulationRecordingValidator
{
public:
  [[nodiscard]] static SimulationValidationConfig loadConfig(
    const std::filesystem::path & recording_config_path);

  [[nodiscard]] static SimulationKinematicsValidationContext loadKinematicsContext(
    const std::filesystem::path & environment_directory,
    const std::filesystem::path & source_trajectory_path,
    const std::vector<std::string> & expected_joint_names);

  [[nodiscard]] static CanonicalTipPoseMetrics validateCanonicalSourcePoses(
    const trajectory_msgs::msg::JointTrajectory & commanded_trajectory,
    const SimulationKinematicsValidationContext & kinematics_context);

  [[nodiscard]] static SimulationRecordingMetrics validate(
    const SimulationRecordingPaths & paths,
    const std::vector<std::string> & joint_names,
    double expected_duration_s,
    bool action_succeeded,
    const SimulationValidationConfig & config);

  [[nodiscard]] static SimulationRecordingMetrics validate(
    const SimulationRecordingPaths & paths,
    const trajectory_msgs::msg::JointTrajectory & commanded_trajectory,
    bool action_succeeded,
    const SimulationValidationConfig & config);

  [[nodiscard]] static SimulationRecordingMetrics validate(
    const SimulationRecordingPaths & paths,
    const trajectory_msgs::msg::JointTrajectory & commanded_trajectory,
    bool action_succeeded,
    const SimulationValidationConfig & config,
    const SimulationKinematicsValidationContext & kinematics_context);
};

}  // namespace hx_waypoint
