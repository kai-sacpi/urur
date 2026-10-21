#include "hx_waypoint/setup_config.hpp"

#include <algorithm>
#include <cmath>
#include <regex>
#include <stdexcept>
#include <unordered_set>

#include <yaml-cpp/yaml.h>

namespace hx_waypoint
{
namespace
{

constexpr double kVectorTolerance = 1.0e-9;

void requireFields(
  const YAML::Node & node,
  const std::unordered_set<std::string> & allowed,
  const std::string & label)
{
  if (!node || !node.IsMap()) {
    throw std::runtime_error(label + " must be a mapping");
  }
  for (const auto & entry : node) {
    if (!entry.first.IsScalar()) {
      throw std::runtime_error(label + " contains a non-scalar field name");
    }
    const std::string key = entry.first.as<std::string>();
    if (allowed.count(key) == 0U) {
      throw std::runtime_error(label + " contains unsupported field '" + key + "'");
    }
  }
}

[[nodiscard]] std::string requiredIdentifier(
  const YAML::Node & node,
  const std::string & key,
  const std::regex & pattern,
  const std::string & label)
{
  const YAML::Node value = node[key];
  if (!value || !value.IsScalar()) {
    throw std::runtime_error(label + "." + key + " must be an identifier");
  }
  const std::string result = value.as<std::string>();
  if (!std::regex_match(result, pattern)) {
    throw std::runtime_error(label + "." + key + " is not a valid identifier");
  }
  return result;
}

template<std::size_t Size>
[[nodiscard]] std::array<double, Size> finiteArray(
  const YAML::Node & node,
  const std::string & key,
  const std::string & label)
{
  const YAML::Node values = node[key];
  if (!values || !values.IsSequence() || values.size() != Size) {
    throw std::runtime_error(
            label + "." + key + " must contain exactly " + std::to_string(Size) +
            " finite numbers");
  }
  std::array<double, Size> result{};
  for (std::size_t index = 0U; index < Size; ++index) {
    result[index] = values[index].as<double>();
    if (!std::isfinite(result[index])) {
      throw std::runtime_error(label + "." + key + " contains a non-finite number");
    }
  }
  return result;
}

[[nodiscard]] double positiveScalar(
  const YAML::Node & node,
  const std::string & key,
  const std::string & label)
{
  const YAML::Node value = node[key];
  if (!value || !value.IsScalar()) {
    throw std::runtime_error(label + "." + key + " must be a finite positive number");
  }
  const double result = value.as<double>();
  if (!std::isfinite(result) || result <= 0.0) {
    throw std::runtime_error(label + "." + key + " must be a finite positive number");
  }
  return result;
}

template<std::size_t Size>
[[nodiscard]] double norm(const std::array<double, Size> & values)
{
  double squared = 0.0;
  for (const double value : values) {
    squared += value * value;
  }
  return std::sqrt(squared);
}

[[nodiscard]] std::array<double, 4> normalizedQuaternion(
  const std::array<double, 4> & values,
  const std::string & label)
{
  const double magnitude = norm(values);
  if (!std::isfinite(magnitude) || std::abs(magnitude - 1.0) > kVectorTolerance) {
    throw std::runtime_error(label + " must be normalized");
  }
  std::array<double, 4> result{};
  for (std::size_t index = 0U; index < result.size(); ++index) {
    result[index] = values[index] / magnitude;
  }
  return result;
}

[[nodiscard]] std::array<double, 3> unitVector(
  const std::array<double, 3> & values,
  const std::string & label)
{
  const double magnitude = norm(values);
  if (!std::isfinite(magnitude) || std::abs(magnitude - 1.0) > kVectorTolerance) {
    throw std::runtime_error(label + " must be a unit vector");
  }
  return {{values[0] / magnitude, values[1] / magnitude, values[2] / magnitude}};
}

[[nodiscard]] std::array<double, 4> rotvecQuaternion(
  const std::array<double, 3> & rotation_vector)
{
  const double angle = norm(rotation_vector);
  if (angle <= 1.0e-15) {
    return {{0.0, 0.0, 0.0, 1.0}};
  }
  const double scale = std::sin(0.5 * angle) / angle;
  const std::array<double, 4> quaternion{{
      rotation_vector[0] * scale,
      rotation_vector[1] * scale,
      rotation_vector[2] * scale,
      std::cos(0.5 * angle)}};
  const double magnitude = norm(quaternion);
  return {{
      quaternion[0] / magnitude,
      quaternion[1] / magnitude,
      quaternion[2] / magnitude,
      quaternion[3] / magnitude}};
}

[[nodiscard]] std::array<double, 3> rotateVector(
  const std::array<double, 4> & quaternion,
  const std::array<double, 3> & vector)
{
  // Apply a unit quaternion rotation without introducing a dependency on a
  // second geometry library.  This is the same q*v*q^-1 convention used by
  // the Python setup loader: the offset is in the raw Feature's local frame.
  const double qx = quaternion[0];
  const double qy = quaternion[1];
  const double qz = quaternion[2];
  const double qw = quaternion[3];
  const double vx = vector[0];
  const double vy = vector[1];
  const double vz = vector[2];
  const double tx = 2.0 * (qy * vz - qz * vy);
  const double ty = 2.0 * (qz * vx - qx * vz);
  const double tz = 2.0 * (qx * vy - qy * vx);
  return {{
      vx + qw * tx + qy * tz - qz * ty,
      vy + qw * ty + qz * tx - qx * tz,
      vz + qw * tz + qx * ty - qy * tx}};
}

}  // namespace

SetupRigidTransform WorkFrameSetup::parentToFeature() const
{
  return SetupRigidTransform{
    {{
        teach_pendant_pose_xyz_rotvec[0], teach_pendant_pose_xyz_rotvec[1],
        teach_pendant_pose_xyz_rotvec[2]}},
    rotvecQuaternion({{
        teach_pendant_pose_xyz_rotvec[3], teach_pendant_pose_xyz_rotvec[4],
        teach_pendant_pose_xyz_rotvec[5]}})};
}

SetupRigidTransform WorkFrameSetup::parentToChild() const
{
  const auto parent_to_feature = parentToFeature();
  const auto rotated_offset = rotateVector(
    parent_to_feature.quaternion_xyzw, feature_to_work_frame_translation_m);
  return SetupRigidTransform{
    {{
        parent_to_feature.translation_m[0] + rotated_offset[0],
        parent_to_feature.translation_m[1] + rotated_offset[1],
        parent_to_feature.translation_m[2] + rotated_offset[2]}},
    parent_to_feature.quaternion_xyzw};
}

SetupRigidTransform ToolSetup::parentToTip() const
{
  return SetupRigidTransform{
    {{
        axis_in_parent_xyz[0] * length_m,
        axis_in_parent_xyz[1] * length_m,
        axis_in_parent_xyz[2] * length_m}},
    tip_orientation_xyzw};
}

SetupConfig loadSetupConfig(const std::filesystem::path & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path.string());
    requireFields(root, {"schema_version", "setup_id", "work_frame", "tool"}, "setup.yaml");
    if (root["schema_version"].as<int>() != 1) {
      throw std::runtime_error("setup.yaml schema_version must be 1");
    }

    const std::regex safe_id("^[A-Za-z0-9][A-Za-z0-9_.-]*$");
    const std::regex safe_frame("^[A-Za-z][A-Za-z0-9_]*$");
    SetupConfig result;
    result.setup_id = requiredIdentifier(root, "setup_id", safe_id, "setup.yaml");

    const YAML::Node work = root["work_frame"];
    requireFields(
      work,
      {"parent_frame", "child_frame", "teach_pendant_pose_xyz_rotvec", "aliases",
        "feature_to_work_frame_translation_m"},
      "setup.yaml.work_frame");
    result.work_frame.parent_frame = requiredIdentifier(
      work, "parent_frame", safe_frame, "setup.yaml.work_frame");
    result.work_frame.child_frame = requiredIdentifier(
      work, "child_frame", safe_frame, "setup.yaml.work_frame");
    if (result.work_frame.parent_frame != "base" ||
      result.work_frame.child_frame != "plane1")
    {
      throw std::runtime_error(
              "setup.yaml work_frame must describe the authoritative base -> plane1 transform");
    }
    result.work_frame.teach_pendant_pose_xyz_rotvec = finiteArray<6U>(
      work, "teach_pendant_pose_xyz_rotvec", "setup.yaml.work_frame");
    // This field is intentionally optional for backwards compatibility with
    // every existing environment.  An omitted offset means that the
    // configured work frame is exactly the measured Installation Feature.
    const YAML::Node feature_offset = work["feature_to_work_frame_translation_m"];
    if (feature_offset.IsDefined()) {
      result.work_frame.feature_to_work_frame_translation_m = finiteArray<3U>(
        work, "feature_to_work_frame_translation_m", "setup.yaml.work_frame");
    }
    const YAML::Node aliases = work["aliases"];
    if (!aliases || !aliases.IsSequence()) {
      throw std::runtime_error("setup.yaml.work_frame.aliases must be a sequence");
    }
    std::unordered_set<std::string> alias_set;
    for (const auto & alias : aliases) {
      if (!alias.IsScalar()) {
        throw std::runtime_error("setup.yaml.work_frame.aliases contains a non-scalar value");
      }
      const std::string value = alias.as<std::string>();
      if (!std::regex_match(value, safe_frame) ||
        value == result.work_frame.parent_frame || value == result.work_frame.child_frame ||
        !alias_set.insert(value).second)
      {
        throw std::runtime_error(
                "setup.yaml.work_frame.aliases must contain unique valid frame ids");
      }
      result.work_frame.aliases.push_back(value);
    }

    const YAML::Node tool = root["tool"];
    requireFields(
      tool,
      {"tool_id", "parent_link", "tip_frame", "tip_orientation_xyzw", "rod"},
      "setup.yaml.tool");
    result.tool.tool_id = requiredIdentifier(tool, "tool_id", safe_id, "setup.yaml.tool");
    result.tool.parent_link = requiredIdentifier(
      tool, "parent_link", safe_frame, "setup.yaml.tool");
    result.tool.tip_frame = requiredIdentifier(
      tool, "tip_frame", safe_frame, "setup.yaml.tool");
    if (result.tool.parent_link != "tool0" || result.tool.tip_frame != "rod_tip") {
      throw std::runtime_error(
              "setup.yaml tool must use the controlled tool0 -> rod_tip frame contract");
    }
    result.tool.tip_orientation_xyzw = normalizedQuaternion(
      finiteArray<4U>(tool, "tip_orientation_xyzw", "setup.yaml.tool"),
      "setup.yaml.tool.tip_orientation_xyzw");

    const YAML::Node rod = tool["rod"];
    requireFields(
      rod,
      {"axis_in_parent_xyz", "length_m", "radius_m", "sphere_spacing_m"},
      "setup.yaml.tool.rod");
    result.tool.axis_in_parent_xyz = unitVector(
      finiteArray<3U>(rod, "axis_in_parent_xyz", "setup.yaml.tool.rod"),
      "setup.yaml.tool.rod.axis_in_parent_xyz");
    result.tool.length_m = positiveScalar(rod, "length_m", "setup.yaml.tool.rod");
    result.tool.radius_m = positiveScalar(rod, "radius_m", "setup.yaml.tool.rod");
    result.tool.sphere_spacing_m = positiveScalar(
      rod, "sphere_spacing_m", "setup.yaml.tool.rod");
    if (result.tool.length_m <= 2.0 * result.tool.radius_m) {
      throw std::runtime_error("setup.yaml rod length must exceed its diameter");
    }
    if (result.tool.sphere_spacing_m > 2.0 * result.tool.radius_m) {
      throw std::runtime_error("setup.yaml rod collision spheres must overlap or touch");
    }
    return result;
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("failed to parse setup.yaml: " + std::string(error.what()));
  }
}

}  // namespace hx_waypoint
