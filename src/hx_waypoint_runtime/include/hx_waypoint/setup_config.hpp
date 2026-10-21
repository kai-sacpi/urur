#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace hx_waypoint
{

struct SetupRigidTransform
{
  std::array<double, 3> translation_m{};
  std::array<double, 4> quaternion_xyzw{{0.0, 0.0, 0.0, 1.0}};
};

struct WorkFrameSetup
{
  std::string parent_frame;
  std::string child_frame;
  // The teach-pendant pose is the raw parent -> Installation Feature pose.
  // ``feature_to_work_frame_translation_m`` is expressed in that Feature's
  // local coordinates and defines the optional work-frame origin offset.
  // Keep both values so the raw dashboard provenance is never overwritten.
  std::array<double, 6> teach_pendant_pose_xyz_rotvec{};
  std::array<double, 3> feature_to_work_frame_translation_m{{0.0, 0.0, 0.0}};
  std::vector<std::string> aliases;

  // Return the unmodified parent -> Installation Feature transform represented
  // by the teach-pendant pose.  This is useful for provenance/raw-dashboard
  // checks; callers that resolve the configured work frame should use
  // parentToChild().
  [[nodiscard]] SetupRigidTransform parentToFeature() const;

  // Return the authoritative parent -> work-frame transform.  The local
  // translation offset is rotated by the raw Feature orientation before it is
  // added to the raw Feature origin.
  [[nodiscard]] SetupRigidTransform parentToChild() const;
};

struct ToolSetup
{
  std::string tool_id;
  std::string parent_link;
  std::string tip_frame;
  std::array<double, 4> tip_orientation_xyzw{{0.0, 0.0, 0.0, 1.0}};
  std::array<double, 3> axis_in_parent_xyz{{0.0, 0.0, 1.0}};
  double length_m{0.0};
  double radius_m{0.0};
  double sphere_spacing_m{0.0};

  [[nodiscard]] SetupRigidTransform parentToTip() const;
};

struct SetupConfig
{
  std::string setup_id;
  WorkFrameSetup work_frame;
  ToolSetup tool;
};

[[nodiscard]] SetupConfig loadSetupConfig(const std::filesystem::path & path);

}  // namespace hx_waypoint
