#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace hx_waypoint
{

struct JointSafetyLimit
{
  double lower_position_rad{0.0};
  double upper_position_rad{0.0};
  double max_velocity_rad_s{0.0};
  double max_acceleration_rad_s2{0.0};
  double max_jerk_rad_s3{0.0};
};

struct RobotValidationModel
{
  std::filesystem::path environment_directory;
  std::filesystem::path urdf_path;
  std::string robot_type;
  std::string base_link;
  std::string tool_link;
  std::string tip_frame;
  std::vector<std::string> joint_names;
  std::vector<JointSafetyLimit> joint_limits;
  std::array<double, 3> tool_to_tip_translation_m{};
  std::array<double, 4> tool_to_tip_quaternion_xyzw{{0.0, 0.0, 0.0, 1.0}};
};

[[nodiscard]] RobotValidationModel loadRobotValidationModel(
  const std::filesystem::path & environment_directory,
  const std::vector<std::string> & expected_joint_names = {});

}  // namespace hx_waypoint
