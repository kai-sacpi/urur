#pragma once

#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace hx_waypoint
{

struct RecordingManifest
{
  int schema_version{0};
  std::string recording_id;
  std::string status;
  std::string environment_id;
  std::string trajectory_id;
  std::string robot_type;
  std::vector<std::string> joint_names;
  double duration_s{0.0};
  std::size_t point_count{0};
  std::string interpolation;
  std::vector<std::string> trajectory_fields;
  std::string curobo_revision;
  std::unordered_map<std::string, std::string> hashes;
};

struct CompiledRunEvidence
{
  std::filesystem::path run_directory;
  std::filesystem::path source_trajectory_path;
  std::filesystem::path joint_trajectory_path;
  std::filesystem::path compile_report_path;
  std::string compile_report_sha256;
  std::string compiler_contract_revision;
  std::string curobo_revision;
  std::string source_trajectory_sha256;
  std::string joint_trajectory_sha256;
  std::string joint_winding_policy;
  std::vector<double> joint_winding_reference_positions_rad;
  std::vector<double> selected_seed_offsets_rad;
  std::vector<double> post_optimization_offsets_rad;
  std::unordered_map<std::string, std::string> environment_hashes;
  double duration_s{0.0};
  std::size_t point_count{0U};
  std::size_t source_waypoint_count{0U};
  std::size_t matched_source_waypoint_count{0U};
  double source_duration_s{0.0};
  double max_source_time_error_s{0.0};
  double uniform_time_scale{1.0};
  double max_joint_velocity_ratio{0.0};
  double max_joint_acceleration_ratio{0.0};
  double max_joint_jerk_ratio{0.0};
  double max_tip_position_error_m{0.0};
  double max_tip_orientation_error_deg{0.0};
  double minimum_collision_distance_m{0.0};
  double max_scene_collision_cost{0.0};
  double max_self_collision_cost{0.0};
  double max_joint_bound_cost{0.0};
  std::size_t checked_sample_count{0U};
};

struct RecordingArtifact
{
  RecordingManifest manifest;
  std::filesystem::path directory;
  trajectory_msgs::msg::JointTrajectory trajectory;
  std::vector<std::string> phases;
};

struct RecordingLoadOptions
{
  std::filesystem::path environment_directory;
  std::string expected_environment_id;
  std::string expected_robot_type;
  std::vector<std::string> expected_joint_names;
};

class RecordingStore
{
public:
  explicit RecordingStore(std::filesystem::path environments_root);

  [[nodiscard]] RecordingArtifact loadValidated(
    const std::string & environment_id,
    const std::string & recording_id,
    const std::string & expected_robot_type,
    const std::vector<std::string> & expected_joint_names) const;

  [[nodiscard]] static RecordingArtifact loadValidatedDirectory(
    const std::filesystem::path & recording_directory,
    const RecordingLoadOptions & options);

  [[nodiscard]] static trajectory_msgs::msg::JointTrajectory loadJointTrajectoryCsv(
    const std::filesystem::path & trajectory_path,
    const std::vector<std::string> & joint_names,
    std::vector<std::string> * phases = nullptr);

  [[nodiscard]] static CompiledRunEvidence validateCompiledRun(
    const std::filesystem::path & runs_root,
    const std::filesystem::path & environment_directory,
    const std::filesystem::path & source_trajectory_path,
    const std::filesystem::path & joint_trajectory_path,
    const std::string & expected_environment_id,
    const std::string & expected_trajectory_id);

  [[nodiscard]] static std::string sha256File(const std::filesystem::path & path);

  [[nodiscard]] const std::filesystem::path & environmentsRoot() const noexcept;

private:
  std::filesystem::path environments_root_;
};

}  // namespace hx_waypoint
