#include "hx_waypoint/robot_validation_model.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <unordered_set>

#include <urdf/model.h>
#include <yaml-cpp/yaml.h>

#include "hx_waypoint/recording_store.hpp"
#include "hx_waypoint/setup_config.hpp"

namespace hx_waypoint
{
namespace
{

[[nodiscard]] std::string trim(std::string value)
{
  const auto not_space = [](unsigned char character) {return !std::isspace(character);};
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

[[nodiscard]] std::string normalizedHash(std::string value)
{
  value = trim(std::move(value));
  constexpr const char * prefix = "sha256:";
  if (value.rfind(prefix, 0U) == 0U) {
    value.erase(0U, std::strlen(prefix));
  }
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });
  if (value.size() != 64U || !std::all_of(
      value.begin(), value.end(), [](unsigned char character) {
        return std::isxdigit(character) != 0;
      }))
  {
    throw std::runtime_error("invalid SHA-256 value in robot asset provenance");
  }
  return value;
}

[[nodiscard]] std::filesystem::path safeRelativePath(
  const YAML::Node & node,
  const std::string & label)
{
  if (!node || !node.IsScalar()) {
    throw std::runtime_error(label + " must be a scalar relative path");
  }
  const std::string raw = trim(node.as<std::string>());
  const std::filesystem::path relative = std::filesystem::path(raw).lexically_normal();
  const bool has_parent = std::any_of(
    relative.begin(), relative.end(), [](const auto & component) {return component == "..";});
  if (raw.empty() || relative.empty() || relative == "." || relative.is_absolute() ||
    has_parent || raw.find('\\') != std::string::npos || relative.generic_string() != raw)
  {
    throw std::runtime_error(label + " is not a safe normalized relative path");
  }
  return relative;
}

[[nodiscard]] std::string requiredString(
  const YAML::Node & parent,
  const std::string & key,
  const std::string & label)
{
  const YAML::Node value = parent[key];
  if (!value || !value.IsScalar()) {
    throw std::runtime_error(label + "." + key + " must be a scalar");
  }
  const std::string result = trim(value.as<std::string>());
  if (result.empty()) {
    throw std::runtime_error(label + "." + key + " must not be empty");
  }
  return result;
}

[[nodiscard]] std::vector<std::string> requiredStringSequence(
  const YAML::Node & parent,
  const std::string & key,
  const std::string & label)
{
  const YAML::Node values = parent[key];
  if (!values || !values.IsSequence() || values.size() == 0U) {
    throw std::runtime_error(label + "." + key + " must be a non-empty sequence");
  }
  std::vector<std::string> result;
  result.reserve(values.size());
  for (const auto & value : values) {
    if (!value.IsScalar()) {
      throw std::runtime_error(label + "." + key + " contains a non-scalar value");
    }
    result.push_back(trim(value.as<std::string>()));
  }
  if (std::any_of(result.begin(), result.end(), [](const std::string & value) {
      return value.empty();
    }) || std::unordered_set<std::string>(result.begin(), result.end()).size() != result.size())
  {
    throw std::runtime_error(label + "." + key + " must contain unique non-empty values");
  }
  return result;
}

[[nodiscard]] std::vector<double> requiredPositiveSequence(
  const YAML::Node & parent,
  const std::string & key,
  std::size_t expected_size,
  const std::string & label)
{
  const YAML::Node values = parent[key];
  if (!values || !values.IsSequence() || values.size() != expected_size) {
    throw std::runtime_error(
            label + "." + key + " must contain exactly " +
            std::to_string(expected_size) + " values");
  }
  std::vector<double> result;
  result.reserve(expected_size);
  for (const auto & value : values) {
    const double parsed = value.as<double>();
    if (!std::isfinite(parsed) || parsed <= 0.0) {
      throw std::runtime_error(label + "." + key + " must contain positive finite values");
    }
    result.push_back(parsed);
  }
  return result;
}

[[nodiscard]] YAML::Node loadYaml(
  const std::filesystem::path & path,
  const std::string & label)
{
  if (!std::filesystem::is_regular_file(path) || std::filesystem::is_symlink(path)) {
    throw std::runtime_error(label + " is missing, not regular, or is a symlink");
  }
  try {
    const YAML::Node root = YAML::LoadFile(path.string());
    if (!root.IsMap() || root["schema_version"].as<int>() != 1) {
      throw std::runtime_error(label + " schema_version must be 1");
    }
    return root;
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("failed to parse " + label + ": " + std::string(error.what()));
  }
}

}  // namespace

RobotValidationModel loadRobotValidationModel(
  const std::filesystem::path & environment_directory,
  const std::vector<std::string> & expected_joint_names)
{
  if (!std::filesystem::is_directory(environment_directory)) {
    throw std::runtime_error(
            "robot validation environment directory does not exist: " +
            environment_directory.string());
  }
  const auto environment = std::filesystem::canonical(environment_directory);
  const YAML::Node robot_model = loadYaml(
    environment / "robot/robot_model.yaml", "robot/robot_model.yaml");
  const YAML::Node provenance = loadYaml(
    environment / "robot/asset_provenance.yaml", "robot/asset_provenance.yaml");
  const YAML::Node curobo = loadYaml(
    environment / "robot/curobo_robot.yaml", "robot/curobo_robot.yaml");
  const SetupConfig setup = loadSetupConfig(environment / "setup.yaml");

  RobotValidationModel model;
  model.environment_directory = environment;
  try {
    const YAML::Node robot = robot_model["robot"];
    if (!robot.IsMap()) {
      throw std::runtime_error("robot_model.yaml robot must be a map");
    }
    model.robot_type = requiredString(robot, "type", "robot_model.robot");
    model.base_link = requiredString(robot, "base_link", "robot_model.robot");
    model.tool_link = requiredString(robot, "tool_link", "robot_model.robot");
    model.joint_names = requiredStringSequence(robot, "joint_names", "robot_model.robot");
    if (!expected_joint_names.empty() && model.joint_names != expected_joint_names) {
      throw std::runtime_error(
              "robot_model joint_names do not match the canonical trajectory order");
    }
    const YAML::Node limits = robot["limits"];
    if (!limits.IsMap()) {
      throw std::runtime_error("robot_model.robot.limits must be a map");
    }
    const auto velocity = requiredPositiveSequence(
      limits, "velocity_rad_s", model.joint_names.size(), "robot_model.robot.limits");
    const auto acceleration = requiredPositiveSequence(
      limits, "acceleration_rad_s2", model.joint_names.size(), "robot_model.robot.limits");
    const auto jerk = requiredPositiveSequence(
      limits, "jerk_rad_s3", model.joint_names.size(), "robot_model.robot.limits");

    if (requiredString(provenance, "robot_type", "asset_provenance") != model.robot_type) {
      throw std::runtime_error("asset_provenance robot_type differs from robot_model");
    }
    const YAML::Node generated_urdf = provenance["generated_urdf"];
    if (!generated_urdf.IsMap()) {
      throw std::runtime_error("asset_provenance.generated_urdf must be a map");
    }
    const auto provenance_urdf_relative = safeRelativePath(
      generated_urdf["path"], "asset_provenance.generated_urdf.path");
    const std::string provenance_urdf_hash = normalizedHash(
      requiredString(generated_urdf, "sha256", "asset_provenance.generated_urdf"));

    const YAML::Node kinematics = curobo["robot_cfg"]["kinematics"];
    const YAML::Node cspace = kinematics["cspace"];
    if (!kinematics.IsMap() || !cspace.IsMap()) {
      throw std::runtime_error("curobo_robot kinematics/cspace must be maps");
    }
    const auto asset_root = safeRelativePath(
      kinematics["asset_root_path"], "curobo_robot.asset_root_path");
    const auto urdf_name = safeRelativePath(
      kinematics["urdf_path"], "curobo_robot.urdf_path");
    const auto curobo_urdf_relative = (asset_root / urdf_name).lexically_normal();
    if (curobo_urdf_relative != provenance_urdf_relative) {
      throw std::runtime_error(
              "asset_provenance and curobo_robot do not select the same URDF");
    }
    if (requiredString(kinematics, "base_link", "curobo_robot.kinematics") !=
      model.base_link)
    {
      throw std::runtime_error("curobo_robot base_link differs from robot_model");
    }
    if (requiredStringSequence(cspace, "joint_names", "curobo_robot.cspace") !=
      model.joint_names)
    {
      throw std::runtime_error("curobo_robot cspace joint_names differ from robot_model");
    }
    const auto tool_frames = requiredStringSequence(
      kinematics, "tool_frames", "curobo_robot.kinematics");
    if (std::find(tool_frames.begin(), tool_frames.end(), model.tool_link) == tool_frames.end()) {
      throw std::runtime_error("curobo_robot tool_frames does not contain robot_model tool_link");
    }

    model.urdf_path = environment / provenance_urdf_relative;
    if (!std::filesystem::is_regular_file(model.urdf_path) ||
      std::filesystem::is_symlink(model.urdf_path) ||
      RecordingStore::sha256File(model.urdf_path) != provenance_urdf_hash)
    {
      throw std::runtime_error("selected URDF does not match hash-bound asset provenance");
    }

    if (setup.tool.parent_link != model.tool_link) {
      throw std::runtime_error("setup tool parent_link differs from robot_model tool_link");
    }
    model.tip_frame = setup.tool.tip_frame;
    const auto tool_to_tip = setup.tool.parentToTip();
    model.tool_to_tip_translation_m = tool_to_tip.translation_m;
    model.tool_to_tip_quaternion_xyzw = tool_to_tip.quaternion_xyzw;
    const double quaternion_norm = std::sqrt(
      model.tool_to_tip_quaternion_xyzw[0] * model.tool_to_tip_quaternion_xyzw[0] +
      model.tool_to_tip_quaternion_xyzw[1] * model.tool_to_tip_quaternion_xyzw[1] +
      model.tool_to_tip_quaternion_xyzw[2] * model.tool_to_tip_quaternion_xyzw[2] +
      model.tool_to_tip_quaternion_xyzw[3] * model.tool_to_tip_quaternion_xyzw[3]);
    if (!std::isfinite(quaternion_norm) || std::abs(quaternion_norm - 1.0) > 1.0e-9) {
      throw std::runtime_error("setup tool quaternion must be normalized");
    }

    urdf::Model urdf_model;
    if (!urdf_model.initFile(model.urdf_path.string())) {
      throw std::runtime_error("failed to parse hash-bound URDF");
    }
    model.joint_limits.reserve(model.joint_names.size());
    for (std::size_t index = 0U; index < model.joint_names.size(); ++index) {
      const auto joint = urdf_model.getJoint(model.joint_names[index]);
      if (!joint || joint->type != urdf::Joint::REVOLUTE || !joint->limits ||
        !std::isfinite(joint->limits->lower) || !std::isfinite(joint->limits->upper) ||
        joint->limits->lower >= joint->limits->upper)
      {
        throw std::runtime_error(
                "configured joint lacks finite revolute position bounds in the hash-bound URDF: " +
                model.joint_names[index]);
      }
      model.joint_limits.push_back(JointSafetyLimit{
          joint->limits->lower, joint->limits->upper,
          velocity[index], acceleration[index], jerk[index]});
    }
    if (RecordingStore::sha256File(model.urdf_path) != provenance_urdf_hash) {
      throw std::runtime_error("hash-bound URDF changed while it was being parsed");
    }
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "robot validation model contains missing or invalid fields: " +
            std::string(error.what()));
  }
  return model;
}

}  // namespace hx_waypoint
