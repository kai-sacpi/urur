#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <condition_variable>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <future>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/list_hardware_components.hpp>
#include <hx_waypoint_interfaces/action/build_recording.hpp>
#include <hx_waypoint_interfaces/action/play_recording.hpp>
#include <hx_waypoint_interfaces/srv/plan_joint_path.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <yaml-cpp/yaml.h>

#include "hx_waypoint/playback_executor.hpp"
#include "hx_waypoint/prepare_workflow.hpp"
#include "hx_waypoint/recording_store.hpp"
#include "hx_waypoint/runtime_policy.hpp"
#include "hx_waypoint/safety_gate.hpp"
#include "hx_waypoint/setup_config.hpp"
#include "hx_waypoint/simulation_recording_validator.hpp"
#include "hx_waypoint/simulation_runner.hpp"
#include "hx_waypoint/telemetry_recorder.hpp"

namespace hx_waypoint
{
namespace
{

using PlayRecording = hx_waypoint_interfaces::action::PlayRecording;
using PlayGoalHandle = rclcpp_action::ServerGoalHandle<PlayRecording>;
using BuildRecording = hx_waypoint_interfaces::action::BuildRecording;
using BuildGoalHandle = rclcpp_action::ServerGoalHandle<BuildRecording>;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using ListControllers = controller_manager_msgs::srv::ListControllers;
using ListHardwareComponents = controller_manager_msgs::srv::ListHardwareComponents;
using PlanJointPath = hx_waypoint_interfaces::srv::PlanJointPath;

constexpr double kStrictSpeedScalingTolerance = 1.0e-9;
constexpr std::chrono::milliseconds kInitialSimulationIdentityAttemptTimeout{250};
constexpr std::chrono::milliseconds kInitialSimulationIdentityRetryInterval{100};
constexpr const char * kRequiredSpeedScalingTopic =
  "/speed_scaling_state_broadcaster/speed_scaling";

void requireUnremappedSimulationSafetyEndpoints(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base)
{
  requireUnremappedActionName(
    node_base, kRequiredSimulationActionName, "simulation action");
  requireUnremappedName(
    node_base, kRequiredJointStatesTopic, false, "authoritative JointState input");
  requireUnremappedName(node_base, kRequiredTfTopic, false, "simulation dynamic TF input");
  requireUnremappedName(
    node_base, kRequiredTfStaticTopic, false, "simulation static TF input");
  requireUnremappedName(
    node_base, kRequiredSimulationControllerStateTopic, false,
    "simulation controller-state topic");
  requireUnremappedName(
    node_base, kRequiredPrepareServiceName, true, "cuRobo prepare service");
  requireUnremappedName(
    node_base, kRequiredSimulationControllerManagerService, true,
    "simulation controller-manager identity service");
  requireUnremappedName(
    node_base, kRequiredSimulationHardwareComponentsService, true,
    "simulation hardware identity service");
}

void requireUnremappedRealSafetyEndpoints(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
  const std::string & action_name)
{
  requireUnremappedActionName(node_base, action_name, "real action");
  requireUnremappedName(
    node_base, kRequiredJointStatesTopic, false, "authoritative JointState input");
  requireUnremappedName(
    node_base, kRequiredPrepareServiceName, true, "cuRobo prepare service");
  requireUnremappedName(
    node_base, kRequiredSpeedScalingTopic, false, "speed_scaling topic");
}

[[nodiscard]] double durationSeconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1.0e-9;
}

[[nodiscard]] std::string timestampString()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
#ifdef _WIN32
  localtime_s(&local, &time);
#else
  localtime_r(&time, &local);
#endif
  std::ostringstream output;
  output << std::put_time(&local, "%Y%m%d_%H%M%S");
  return output.str();
}

[[nodiscard]] std::string resultCodeName(rclcpp_action::ResultCode code)
{
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return "SUCCEEDED";
    case rclcpp_action::ResultCode::ABORTED:
      return "ABORTED";
    case rclcpp_action::ResultCode::CANCELED:
      return "CANCELED";
    default:
      return "UNKNOWN";
  }
}

[[nodiscard]] trajectory_msgs::msg::JointTrajectory makeStartGateTrajectory(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & positions)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = joint_names;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions;
  point.velocities.assign(positions.size(), 0.0);
  point.accelerations.assign(positions.size(), 0.0);
  trajectory.points.push_back(std::move(point));
  return trajectory;
}

constexpr std::array<const char *, 12> kEnvironmentFiles{{
    "environment.yaml", "setup.yaml", "zones.yaml", "planner.yaml",
    "recording.yaml", "controllers.yaml", "robot/robot_model.yaml",
    "robot/calibrated_kinematics.yaml", "robot/asset_provenance.yaml",
    "robot/driver_kinematics.yaml", "robot/curobo_robot.yaml",
    "robot/collision_spheres.yaml"}};
constexpr std::array<const char *, 2> kEnvironmentTreeDirectories{{
    "meshes", "robot/assets"}};

[[nodiscard]] bool safeIdentifier(const std::string & value)
{
  return !value.empty() && std::all_of(value.begin(), value.end(), [](unsigned char character) {
      return std::isalnum(character) != 0 || character == '_' || character == '-';
    });
}

[[nodiscard]] bool isLowercaseSha256(const std::string & value)
{
  return value.size() == 64U && std::all_of(
    value.begin(), value.end(), [](unsigned char character) {
      return std::isdigit(character) != 0 || (character >= 'a' && character <= 'f');
    });
}

[[nodiscard]] bool safeRelativePath(const std::filesystem::path & path)
{
  return !path.empty() && !path.is_absolute() && path != "." &&
         std::none_of(path.begin(), path.end(), [](const auto & component) {
           return component == "..";
         });
}

struct EnvironmentIdentity
{
  std::string environment_id;
  std::string robot_type;
  std::vector<std::string> joint_names;
  std::vector<double> home_joint_positions;
};

struct StartStatePolicy
{
  double max_joint_error_rad{0.0};
  double settle_velocity_rad_s{0.0};
  std::string mismatch_policy;
  std::string prepare_sequence{"direct_to_recording_start"};
};

[[nodiscard]] StartStatePolicy loadStartStatePolicy(
  const std::filesystem::path & environment)
{
  try {
    const auto root = YAML::LoadFile((environment / "recording.yaml").string());
    if (root["schema_version"].as<int>() != 1) {
      throw std::runtime_error("recording.yaml schema_version must be 1");
    }
    const auto start = root["start_state"];
    StartStatePolicy policy;
    policy.max_joint_error_rad = start["max_joint_error_rad"].as<double>();
    policy.settle_velocity_rad_s = start["settle_velocity_rad_s"].as<double>();
    policy.mismatch_policy = start["mismatch_policy"].as<std::string>();
    if (start["prepare_sequence"]) {
      policy.prepare_sequence = start["prepare_sequence"].as<std::string>();
    }
    if (!std::isfinite(policy.max_joint_error_rad) || policy.max_joint_error_rad <= 0.0 ||
      !std::isfinite(policy.settle_velocity_rad_s) || policy.settle_velocity_rad_s < 0.0 ||
      (policy.mismatch_policy != "reject" && policy.mismatch_policy != "curobo_prepare") ||
      (policy.prepare_sequence != "direct_to_recording_start" &&
      policy.prepare_sequence != "home_then_recording_start") ||
      (policy.prepare_sequence == "home_then_recording_start" &&
      policy.mismatch_policy != "curobo_prepare"))
    {
      throw std::runtime_error("recording.yaml start_state policy is invalid");
    }
    return policy;
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "failed to load recording start-state policy: " + std::string(error.what()));
  }
}

[[nodiscard]] EnvironmentIdentity loadEnvironmentIdentity(const std::filesystem::path & environment)
{
  try {
    const auto environment_yaml = YAML::LoadFile((environment / "environment.yaml").string());
    const auto robot_yaml = YAML::LoadFile((environment / "robot/robot_model.yaml").string());
    if (environment_yaml["schema_version"].as<int>() != 1 ||
      robot_yaml["schema_version"].as<int>() != 1)
    {
      throw std::runtime_error("environment and robot_model schema_version must be 1");
    }
    EnvironmentIdentity identity;
    identity.environment_id = environment_yaml["environment_id"].as<std::string>();
    identity.robot_type = robot_yaml["robot"]["type"].as<std::string>();
    identity.joint_names =
      robot_yaml["robot"]["joint_names"].as<std::vector<std::string>>();
    identity.home_joint_positions =
      robot_yaml["robot"]["home_joint_positions"].as<std::vector<double>>();
    if (identity.environment_id.empty() || identity.robot_type.empty() ||
      identity.joint_names.empty() ||
      identity.home_joint_positions.size() != identity.joint_names.size() ||
      !std::all_of(
        identity.home_joint_positions.begin(), identity.home_joint_positions.end(),
        [](double value) {return std::isfinite(value);}))
    {
      throw std::runtime_error("environment identity fields must not be empty");
    }
    return identity;
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("failed to load environment identity: " + std::string(error.what()));
  }
}

void requireRealCalibration(
  const std::filesystem::path & environment,
  const std::filesystem::path & driver_kinematics_file)
{
  try {
    const auto calibration = YAML::LoadFile(
      (environment / "robot/calibrated_kinematics.yaml").string());
    const auto provenance = YAML::LoadFile(
      (environment / "robot/asset_provenance.yaml").string());
    const auto robot_model = YAML::LoadFile(
      (environment / "robot/robot_model.yaml").string());
    const auto curobo_robot = YAML::LoadFile(
      (environment / "robot/curobo_robot.yaml").string());
    if (calibration["schema_version"].as<int>() != 1 ||
      provenance["schema_version"].as<int>() != 1 ||
      robot_model["schema_version"].as<int>() != 1 ||
      curobo_robot["schema_version"].as<int>() != 1)
    {
      throw std::runtime_error("calibration and generated-asset provenance schemas must be 1");
    }
    if (!calibration["deployment_allowed"] ||
      !calibration["deployment_allowed"].as<bool>())
    {
      throw std::runtime_error(
              "calibrated_kinematics.yaml has deployment_allowed=false; real playback is forbidden");
    }
    if (!std::filesystem::is_regular_file(driver_kinematics_file)) {
      throw std::runtime_error(
              "the UR driver kinematics_params_file is missing: " + driver_kinematics_file.string());
    }
    const std::string robot_type = robot_model["robot"]["type"].as<std::string>();
    if (robot_type.empty() || calibration["robot_type"].as<std::string>() != robot_type ||
      provenance["robot_type"].as<std::string>() != robot_type)
    {
      throw std::runtime_error(
              "calibration, asset provenance, and robot model types do not match");
    }
    const auto source_relative = std::filesystem::path(
      provenance["source_kinematics"]["path"].as<std::string>()).lexically_normal();
    if (source_relative != std::filesystem::path("robot/driver_kinematics.yaml") ||
      !safeRelativePath(source_relative))
    {
      throw std::runtime_error(
              "asset provenance must use robot/driver_kinematics.yaml as its controlled source");
    }
    const auto kinematics = curobo_robot["robot_cfg"]["kinematics"];
    const auto asset_root = std::filesystem::path(
      kinematics["asset_root_path"].as<std::string>()).lexically_normal();
    const auto urdf_name = std::filesystem::path(
      kinematics["urdf_path"].as<std::string>()).lexically_normal();
    if (!safeRelativePath(asset_root) || !safeRelativePath(urdf_name)) {
      throw std::runtime_error("cuRobo generated-asset paths must remain environment-relative");
    }
    const auto expected_urdf_relative = (asset_root / urdf_name).lexically_normal();
    const auto provenance_urdf_relative = std::filesystem::path(
      provenance["generated_urdf"]["path"].as<std::string>()).lexically_normal();
    if (!safeRelativePath(provenance_urdf_relative) ||
      provenance_urdf_relative != expected_urdf_relative)
    {
      throw std::runtime_error(
              "asset provenance generated_urdf.path does not match the cuRobo robot model");
    }
    const auto controlled_source = environment / source_relative;
    const auto controlled_urdf = environment / provenance_urdf_relative;
    if (!std::filesystem::is_regular_file(controlled_source) ||
      std::filesystem::is_symlink(controlled_source) ||
      !std::filesystem::is_regular_file(controlled_urdf) ||
      std::filesystem::is_symlink(controlled_urdf))
    {
      throw std::runtime_error(
              "controlled driver kinematics and generated URDF must be regular non-symlink files");
    }
    const std::string controlled_source_hash = RecordingStore::sha256File(controlled_source);
    const std::string controlled_urdf_hash = RecordingStore::sha256File(controlled_urdf);
    const std::string provenance_source_hash =
      provenance["source_kinematics"]["sha256"].as<std::string>();
    const std::string provenance_urdf_hash =
      provenance["generated_urdf"]["sha256"].as<std::string>();
    const std::string expected_hash = calibration["driver_kinematics_sha256"].as<std::string>();
    if (!isLowercaseSha256(expected_hash) ||
      !isLowercaseSha256(provenance_source_hash) ||
      !isLowercaseSha256(provenance_urdf_hash))
    {
      throw std::runtime_error(
              "calibration/provenance SHA-256 values must be lowercase digests");
    }
    if (controlled_source_hash != expected_hash ||
      controlled_source_hash != provenance_source_hash ||
      controlled_urdf_hash != provenance_urdf_hash)
    {
      throw std::runtime_error(
              "controlled calibration source and generated URDF do not match asset provenance");
    }
    if (RecordingStore::sha256File(driver_kinematics_file) != controlled_source_hash) {
      throw std::runtime_error(
              "the UR driver kinematics_params_file does not match the controlled calibration source");
    }
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "cannot prove real calibration deployment is allowed: " + std::string(error.what()));
  }
}

void requireRealControllerContract(
  const std::filesystem::path & environment,
  const std::string & selected_action_name)
{
  try {
    const auto controllers = YAML::LoadFile((environment / "controllers.yaml").string());
    if (controllers["schema_version"].as<int>() != 1) {
      throw std::runtime_error("controllers.yaml schema_version must be 1");
    }
    const auto real = controllers["real"];
    const std::string controller_type = real["controller_type"].as<std::string>();
    const std::string action_name = real["action_name"].as<std::string>();
    const std::string interpolation = real["interpolation"].as<std::string>();
    const bool allow_fallback = real["allow_fallback"].as<bool>();
    const bool approved_passthrough =
      controller_type == "passthrough_trajectory_controller" &&
      action_name == kRequiredRealPassthroughActionName;
    const bool approved_unscaled_jtc =
      controller_type == "joint_trajectory_controller" &&
      action_name == kRequiredRealUnscaledJointTrajectoryActionName;
    if ((!approved_passthrough && !approved_unscaled_jtc) ||
      interpolation != "spline" || allow_fallback)
    {
      throw std::runtime_error(
              "controllers.yaml real contract is not an approved exact unscaled spline controller");
    }
    if (action_name != selected_action_name) {
      throw std::runtime_error(
              "runtime real action does not match the current environment controller contract");
    }
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "failed to load real controller contract: " + std::string(error.what()));
  }
}

void writeYaml(const std::filesystem::path & path, const YAML::Node & node)
{
  std::filesystem::create_directories(path.parent_path());
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("failed to open YAML output: " + path.string());
  }
  output << node;
  if (!output) {
    throw std::runtime_error("failed to write YAML output: " + path.string());
  }
}

void writeSimulationExecutionIdentity(
  YAML::Node & validation,
  const SimulationExecutionIdentity & identity)
{
  validation["simulation_execution_contract_revision"] = identity.contract_revision;
  validation["simulation_controller_name"] = identity.controller_name;
  validation["simulation_controller_type"] = identity.controller_type;
  validation["simulation_controller_state"] = identity.controller_state;
  YAML::Node claimed_interfaces(YAML::NodeType::Sequence);
  for (const auto & interface_name : identity.claimed_interfaces) {
    claimed_interfaces.push_back(interface_name);
  }
  validation["simulation_controller_claimed_interfaces"] = claimed_interfaces;
  YAML::Node required_command_interfaces(YAML::NodeType::Sequence);
  for (const auto & interface_name : identity.required_command_interfaces) {
    required_command_interfaces.push_back(interface_name);
  }
  validation["simulation_controller_required_command_interfaces"] =
    required_command_interfaces;
  YAML::Node required_state_interfaces(YAML::NodeType::Sequence);
  for (const auto & interface_name : identity.required_state_interfaces) {
    required_state_interfaces.push_back(interface_name);
  }
  validation["simulation_controller_required_state_interfaces"] =
    required_state_interfaces;
  validation["simulation_hardware_name"] = identity.hardware_name;
  validation["simulation_hardware_type"] = identity.hardware_type;
  validation["simulation_hardware_plugin"] = identity.hardware_plugin;
  validation["simulation_hardware_state_id"] =
    static_cast<unsigned int>(identity.hardware_state_id);
  validation["simulation_hardware_state_label"] = identity.hardware_state_label;
}

void copyExact(const std::filesystem::path & source, const std::filesystem::path & destination)
{
  if (!std::filesystem::is_regular_file(source)) {
    throw std::runtime_error("required input file does not exist: " + source.string());
  }
  std::filesystem::create_directories(destination.parent_path());
  std::filesystem::copy_file(source, destination, std::filesystem::copy_options::none);
}

void requireCompiledEvidenceFilesUnchanged(
  const std::filesystem::path & directory,
  const CompiledRunEvidence & evidence,
  const std::string & context)
{
  const std::array<std::pair<std::filesystem::path, std::string>, 3> expected{{
      {directory / "source_tip_trajectory.csv", evidence.source_trajectory_sha256},
      {directory / "joint_trajectory.csv", evidence.joint_trajectory_sha256},
      {directory / "compile_report.yaml", evidence.compile_report_sha256}}};
  for (const auto & [path, expected_hash] : expected) {
    if (RecordingStore::sha256File(path) != expected_hash) {
      throw std::runtime_error(
              context + ": " + path.filename().string() +
              " no longer matches the validated compiled-run evidence");
    }
  }
}

void snapshotEnvironment(
  const std::filesystem::path & environment, const std::filesystem::path & destination)
{
  for (const char * relative : kEnvironmentFiles) {
    copyExact(environment / relative, destination / relative);
  }
  for (const char * relative : kEnvironmentTreeDirectories) {
    const auto source_directory = environment / relative;
    if (std::filesystem::is_directory(source_directory)) {
      for (const auto & entry : std::filesystem::recursive_directory_iterator(source_directory)) {
        if (entry.is_regular_file()) {
          copyExact(
            entry.path(),
            destination / relative / entry.path().lexically_relative(source_directory));
        }
      }
    }
  }
}

[[nodiscard]] std::unordered_map<std::string, std::string> environmentHashes(
  const std::filesystem::path & environment)
{
  std::unordered_map<std::string, std::string> hashes;
  for (const char * relative : kEnvironmentFiles) {
    hashes.emplace(relative, RecordingStore::sha256File(environment / relative));
  }
  for (const char * relative : kEnvironmentTreeDirectories) {
    const auto source_directory = environment / relative;
    if (std::filesystem::is_directory(source_directory)) {
      for (const auto & entry : std::filesystem::recursive_directory_iterator(source_directory)) {
        if (entry.is_regular_file()) {
          hashes.emplace(
            (std::filesystem::path(relative) /
            entry.path().lexically_relative(source_directory)).generic_string(),
            RecordingStore::sha256File(entry.path()));
        }
      }
    }
  }
  return hashes;
}

void requirePrepareEnvironmentEvidence(
  const PlanJointPath::Response & response,
  const std::unordered_map<std::string, std::string> & expected)
{
  if (response.environment_paths.size() != response.environment_sha256.size() ||
    response.environment_paths.size() != expected.size())
  {
    throw std::runtime_error(
            "prepare planner environment evidence does not match the required file set");
  }
  std::unordered_map<std::string, std::string> reported;
  for (std::size_t index = 0U; index < response.environment_paths.size(); ++index) {
    const auto relative = std::filesystem::path(response.environment_paths[index]).lexically_normal();
    const std::string digest = response.environment_sha256[index];
    if (relative.empty() || relative.is_absolute() || relative.string().rfind("..", 0U) == 0U ||
      digest.size() != 64U || !std::all_of(
        digest.begin(), digest.end(), [](unsigned char character) {
          return std::isxdigit(character) != 0;
        }))
    {
      throw std::runtime_error("prepare planner returned unsafe environment evidence");
    }
    if (!reported.emplace(relative.generic_string(), digest).second) {
      throw std::runtime_error("prepare planner returned duplicate environment evidence");
    }
  }
  for (const auto & [path, digest] : expected) {
    const auto found = reported.find(path);
    if (found == reported.end() || found->second != digest) {
      throw std::runtime_error(
              "prepare planner environment differs from the authoritative environment at '" +
              path + "'");
    }
  }
}

void writeChecksums(const std::filesystem::path & recording_directory)
{
  const auto checksum_path = recording_directory / "sha256sums.txt";
  std::vector<std::filesystem::path> files;
  for (const auto & entry : std::filesystem::recursive_directory_iterator(recording_directory)) {
    if (entry.is_regular_file() && entry.path() != checksum_path) {
      files.push_back(entry.path());
    }
  }
  std::sort(files.begin(), files.end());
  std::ofstream output(checksum_path);
  if (!output) {
    throw std::runtime_error("failed to open sha256sums.txt");
  }
  for (const auto & file : files) {
    output << RecordingStore::sha256File(file) << "  " <<
      file.lexically_relative(recording_directory).generic_string() << '\n';
  }
}

void requireCompleteRecording(const std::filesystem::path & directory)
{
  static const std::array<const char *, 9> required{{
      "manifest.yaml", "source_tip_trajectory.csv", "joint_trajectory.csv", "joint_frames.csv",
      "sim_controller_state.csv", "sim_actual_joint.csv", "sim_actual_tip.csv",
      "compile_report.yaml", "validation_report.yaml"}};
  for (const char * name : required) {
    const auto path = directory / name;
    if (!std::filesystem::is_regular_file(path) || std::filesystem::file_size(path) == 0U) {
      throw std::runtime_error("recording is incomplete or empty: " + std::string(name));
    }
  }
  if (!std::filesystem::is_directory(directory / "environment_snapshot")) {
    throw std::runtime_error("recording environment_snapshot is missing");
  }
}

}  // namespace

class RuntimeNode : public rclcpp::Node
{
public:
  explicit RuntimeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("hx_waypoint_runtime", options),
    execution_mode_(declare_parameter<std::string>("execution_mode", "real")),
    environments_root_(declare_parameter<std::string>(
        "environments_root", (std::filesystem::current_path() / "environments").string())),
    telemetry_root_(declare_parameter<std::string>(
        "telemetry_root", (std::filesystem::current_path() / "runs").string())),
    robot_type_(declare_parameter<std::string>("robot_type", "")),
    driver_kinematics_params_file_(declare_parameter<std::string>(
        "driver_kinematics_params_file", "")),
    joint_names_(declare_parameter<std::vector<std::string>>(
        "joint_names",
        {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
          "wrist_2_joint", "wrist_3_joint"})),
    joint_states_topic_(declare_parameter<std::string>(
        "joint_states_topic", kRequiredJointStatesTopic)),
    action_server_timeout_s_(declare_parameter<double>("action_server_timeout_s", 5.0)),
    speed_scaling_fresh_timeout_s_(declare_parameter<double>(
        "speed_scaling_fresh_timeout_s", 2.0)),
    store_(std::filesystem::path(environments_root_)),
    safety_gate_(SafetyGateConfig{
        declare_parameter<double>("start_state.max_joint_error_rad", 0.02),
        declare_parameter<double>("start_state.settle_velocity_rad_s", 0.01),
        declare_parameter<double>("start_state.max_joint_state_age_s", 0.5)})
  {
    setup_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    setup_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *setup_tf_buffer_, this, false);
    simulation_action_name_ = declare_parameter<std::string>(
      "simulation_action_name", kRequiredSimulationActionName);
    const std::string real_action_name = declare_parameter<std::string>(
      "real_action_name", "/passthrough_trajectory_controller/follow_joint_trajectory");
    const std::string explicit_action_name = declare_parameter<std::string>("action_name", "");
    selected_action_name_ = selectControllerAction(
      execution_mode_, simulation_action_name_, real_action_name, explicit_action_name);
    requireJointStatesTopic(joint_states_topic_);
    if (execution_mode_ == "simulation") {
      requireUnremappedSimulationSafetyEndpoints(get_node_base_interface());
    } else {
      requireUnremappedRealSafetyEndpoints(
        get_node_base_interface(), selected_action_name_);
    }
    if (!std::isfinite(action_server_timeout_s_) || action_server_timeout_s_ <= 0.0 ||
      !std::isfinite(speed_scaling_fresh_timeout_s_) || speed_scaling_fresh_timeout_s_ <= 0.0)
    {
      throw std::invalid_argument(
              "action_server_timeout_s and speed_scaling_fresh_timeout_s must be finite and positive");
    }

    prepare_plan_service_name_ = declare_parameter<std::string>(
      "prepare.plan_service_name", kRequiredPrepareServiceName);
    requirePrepareServiceName(prepare_plan_service_name_);
    prepare_planning_timeout_s_ = declare_parameter<double>("prepare.planning_timeout_s", 120.0);
    prepare_settle_timeout_s_ = declare_parameter<double>("prepare.settle_timeout_s", 5.0);
    prepare_validation_config_.endpoint_tolerance_rad = declare_parameter<double>(
      "prepare.endpoint_tolerance_rad", 1.0e-3);
    prepare_validation_config_.endpoint_derivative_tolerance = declare_parameter<double>(
      "prepare.endpoint_derivative_tolerance", 1.0e-6);
    prepare_validation_config_.max_duration_s = declare_parameter<double>(
      "prepare.max_duration_s", 120.0);
    if (prepare_plan_service_name_.empty() || !std::isfinite(prepare_planning_timeout_s_) ||
      prepare_planning_timeout_s_ <= 0.0 || !std::isfinite(prepare_settle_timeout_s_) ||
      prepare_settle_timeout_s_ <= 0.0 ||
      !std::isfinite(prepare_validation_config_.endpoint_tolerance_rad) ||
      prepare_validation_config_.endpoint_tolerance_rad <= 0.0 ||
      !std::isfinite(prepare_validation_config_.endpoint_derivative_tolerance) ||
      prepare_validation_config_.endpoint_derivative_tolerance < 0.0 ||
      !std::isfinite(prepare_validation_config_.max_duration_s) ||
      prepare_validation_config_.max_duration_s <= 0.0)
    {
      throw std::invalid_argument("prepare service name and timeouts must be valid");
    }

    executor_ = std::make_unique<PlaybackExecutor>(this, selected_action_name_);
    prepare_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    prepare_client_ = create_client<PlanJointPath>(
      prepare_plan_service_name_, rclcpp::ServicesQoS(), prepare_callback_group_);
    if (execution_mode_ == "simulation") {
      simulation_contract_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);
      simulation_controller_manager_client_ = create_client<ListControllers>(
        kRequiredSimulationControllerManagerService, rclcpp::ServicesQoS(),
        simulation_contract_callback_group_);
      simulation_hardware_components_client_ = create_client<ListHardwareComponents>(
        kRequiredSimulationHardwareComponentsService, rclcpp::ServicesQoS(),
        simulation_contract_callback_group_);
      if (std::string(simulation_controller_manager_client_->get_service_name()) !=
        kRequiredSimulationControllerManagerService ||
        std::string(simulation_hardware_components_client_->get_service_name()) !=
        kRequiredSimulationHardwareComponentsService)
      {
        throw std::invalid_argument(
                "resolved simulation identity service does not match the safety contract");
      }
    }
    if (std::string(prepare_client_->get_service_name()) != kRequiredPrepareServiceName)
    {
      throw std::invalid_argument("resolved cuRobo prepare service does not match the safety contract");
    }
    TelemetryRecorderOptions telemetry_options;
    telemetry_options.joint_states_topic = joint_states_topic_;
    telemetry_options.telemetry_topic = declare_parameter<std::string>(
      "telemetry_topic", "~/execution_telemetry");
    telemetry_options.base_frame = declare_parameter<std::string>(
      "telemetry_base_frame", "base_link");
    telemetry_options.tip_frame = declare_parameter<std::string>(
      "telemetry_tip_frame", "rod_tip");
    telemetry_options.speed_scaling_topic = declare_parameter<std::string>(
      "speed_scaling_topic", kRequiredSpeedScalingTopic);
    if (execution_mode_ == "real" &&
      telemetry_options.speed_scaling_topic != kRequiredSpeedScalingTopic)
    {
      throw std::invalid_argument(
              "real execution requires the exact speed_scaling topic " +
              std::string(kRequiredSpeedScalingTopic));
    }
    telemetry_recorder_ = std::make_unique<TelemetryRecorder>(
      this, joint_names_, std::move(telemetry_options));

    TrajectoryRecorderOptions simulation_recording_options;
    simulation_recording_options.joint_states_topic = joint_states_topic_;
    simulation_recording_options.controller_state_topic = declare_parameter<std::string>(
      "simulation_controller_state_topic", kRequiredSimulationControllerStateTopic);
    requireSimulationControllerStateTopic(simulation_recording_options.controller_state_topic);
    simulation_recording_options.base_frame = declare_parameter<std::string>(
      "simulation_base_frame", kRequiredSimulationBaseFrame);
    simulation_recording_options.frame_names = declare_parameter<std::vector<std::string>>(
      "simulation_frame_names", kRequiredSimulationFrameNames);
    simulation_recording_options.tip_frame = declare_parameter<std::string>(
      "simulation_tip_frame", kRequiredSimulationTipFrame);
    requireSimulationFrameContract(
      simulation_recording_options.base_frame, simulation_recording_options.frame_names,
      simulation_recording_options.tip_frame);
    simulation_runner_ = std::make_unique<SimulationRunner>(
      this, simulation_action_name_, joint_names_, std::move(simulation_recording_options));

    joint_state_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions joint_state_subscription_options;
    joint_state_subscription_options.callback_group = joint_state_callback_group_;
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr message) {
        {
          std::lock_guard<std::mutex> lock(joint_state_mutex_);
          latest_joint_state_ = std::move(message);
          ++joint_state_sequence_;
        }
        joint_state_condition_.notify_all();
      },
      joint_state_subscription_options);

    const std::string play_action_name = declare_parameter<std::string>(
      "play_recording_action_name", "/hx_waypoint/play_recording");
    // Both servers share worker_thread_: preserve their mutual exclusion, without allowing
    // worker joins or action finalization to monopolize the default parameter-service group.
    action_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    play_server_ = rclcpp_action::create_server<PlayRecording>(
      this,
      play_action_name,
      std::bind(&RuntimeNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RuntimeNode::handleCancel, this, std::placeholders::_1),
      std::bind(&RuntimeNode::handleAccepted, this, std::placeholders::_1),
      rcl_action_server_get_default_options(), action_callback_group_);
    const std::string build_action_name = declare_parameter<std::string>(
      "build_recording_action_name", "/hx_waypoint/build_recording");
    build_server_ = rclcpp_action::create_server<BuildRecording>(
      this,
      build_action_name,
      std::bind(&RuntimeNode::handleBuildGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RuntimeNode::handleBuildCancel, this, std::placeholders::_1),
      std::bind(&RuntimeNode::handleBuildAccepted, this, std::placeholders::_1),
      rcl_action_server_get_default_options(), action_callback_group_);

    RCLCPP_INFO(
      get_logger(),
      "runtime mode=%s action=%s environments_root=%s",
      execution_mode_.c_str(), selected_action_name_.c_str(), environments_root_.c_str());
    if (execution_mode_ == "real") {
      RCLCPP_INFO(
        get_logger(),
        "real playback uses the configured action exactly; no fallback controller will be tried");
    }
  }

  ~RuntimeNode() override
  {
    if (executor_) {
      executor_->shutdown();
    }
    if (simulation_runner_) {
      simulation_runner_->cancel();
    }
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PlayRecording::Goal> goal)
  {
    if (!safeIdentifier(goal->environment_id) || !safeIdentifier(goal->recording_id)) {
      RCLCPP_WARN(get_logger(), "rejecting PlayRecording goal with empty identifiers");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->auto_prepare && goal->externally_prepared_start) {
      RCLCPP_WARN(
        get_logger(),
        "rejecting PlayRecording goal with mutually exclusive prepare modes");
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (goal_active_) {
      RCLCPP_WARN(get_logger(), "rejecting PlayRecording goal because another playback is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    goal_active_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<PlayGoalHandle>)
  {
    executor_->cancelActive();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<PlayGoalHandle> goal_handle)
  {
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
    worker_thread_ = std::thread([this, goal_handle]() {executeGoal(goal_handle);});
  }

  rclcpp_action::GoalResponse handleBuildGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const BuildRecording::Goal> goal)
  {
    if (execution_mode_ != "simulation") {
      RCLCPP_WARN(get_logger(), "rejecting BuildRecording outside simulation mode");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!safeIdentifier(goal->environment_id) || !safeIdentifier(goal->trajectory_id) ||
      goal->source_trajectory_path.empty() || goal->compiled_trajectory_path.empty())
    {
      RCLCPP_WARN(get_logger(), "rejecting BuildRecording goal with invalid identifiers or paths");
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (goal_active_) {
      RCLCPP_WARN(get_logger(), "rejecting BuildRecording goal because another operation is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    goal_active_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleBuildCancel(const std::shared_ptr<BuildGoalHandle>)
  {
    executor_->cancelActive();
    simulation_runner_->cancel();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleBuildAccepted(const std::shared_ptr<BuildGoalHandle> goal_handle)
  {
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
    worker_thread_ = std::thread([this, goal_handle]() {executeBuildGoal(goal_handle);});
  }

  void executeBuildGoal(const std::shared_ptr<BuildGoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto finish_once = std::make_shared<std::atomic_bool>(false);
    auto staging_directory = std::make_shared<std::filesystem::path>();
    auto final_directory = std::make_shared<std::filesystem::path>();
    auto finish =
      std::make_shared<std::function<void(bool, bool, const std::string &, const std::string &)>>();
    *finish = [this, goal_handle, finish_once, staging_directory, final_directory](
      bool success, bool canceled, const std::string & state, const std::string & message) {
        if (finish_once->exchange(true)) {
          return;
        }
        const auto release_goal = [this]() {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          goal_active_ = false;
        };
        try {
          if (!success && !staging_directory->empty() &&
            std::filesystem::is_directory(*staging_directory))
          {
            try {
              YAML::Node failed;
              failed["schema_version"] = 1;
              failed["recording_id"] = staging_directory->filename().string();
              if (!failed["recording_id"].as<std::string>().empty() &&
                failed["recording_id"].as<std::string>().front() == '.')
              {
                failed["recording_id"] =
                  failed["recording_id"].as<std::string>().substr(1U);
              }
              const std::string suffix = ".staging";
              std::string id = failed["recording_id"].as<std::string>();
              if (id.size() > suffix.size() &&
                id.compare(id.size() - suffix.size(), suffix.size(), suffix) == 0)
              {
                id.erase(id.size() - suffix.size());
                failed["recording_id"] = id;
              }
              failed["status"] = canceled ? "canceled" : "failed";
              failed["state"] = state;
              failed["message"] = message;
              writeYaml(*staging_directory / "manifest.yaml", failed);
            } catch (const std::exception & error) {
              RCLCPP_ERROR(
                get_logger(), "failed to mark staging recording failed: %s", error.what());
            }
          }
          const bool upstream_canceling = goal_handle->is_canceling();
          const bool terminal_success = success && !upstream_canceling;
          auto result = std::make_shared<BuildRecording::Result>();
          result->success = terminal_success;
          if (terminal_success) {
            result->recording_id = final_directory->filename().string();
            result->recording_path = final_directory->string();
          } else {
            result->recording_path = staging_directory->string();
          }
          result->message = message;

          release_goal();
          if (upstream_canceling) {
            goal_handle->canceled(result);
          } else if (!goal_handle->is_active()) {
            RCLCPP_WARN(
              get_logger(), "BuildRecording goal was already terminal while finishing state=%s",
              state.c_str());
          } else if (terminal_success) {
            goal_handle->succeed(result);
          } else {
            goal_handle->abort(result);
          }
        } catch (const std::exception & error) {
          release_goal();
          RCLCPP_ERROR(
            get_logger(), "failed to finalize BuildRecording goal safely: %s", error.what());
        } catch (...) {
          release_goal();
          RCLCPP_ERROR(get_logger(), "failed to finalize BuildRecording goal safely");
        }
      };

    publishBuildFeedback(goal_handle, "VALIDATING_INPUT", 0.02F, "loading environment and compiled trajectory");
    std::filesystem::path environment_directory;
    EnvironmentIdentity identity;
    CompiledRunEvidence compiled_evidence;
    SimulationValidationConfig simulation_validation_config;
    StartStatePolicy start_policy;
    trajectory_msgs::msg::JointTrajectory trajectory;
    std::vector<std::string> phases;
    try {
      const auto controlled_environment = std::filesystem::canonical(
        store_.environmentsRoot() / goal->environment_id);
      if (!goal->environment_path.empty() &&
        std::filesystem::canonical(goal->environment_path) != controlled_environment)
      {
        throw std::runtime_error(
                "environment_path must resolve to the selected environment under environments_root");
      }
      environment_directory = controlled_environment;
      identity = loadEnvironmentIdentity(environment_directory);
      if (identity.environment_id != goal->environment_id) {
        throw std::runtime_error("environment_path environment_id does not match action goal");
      }
      if (identity.joint_names != joint_names_) {
        throw std::runtime_error("environment joint_names do not match runtime/controller order");
      }
      if (!robot_type_.empty() && identity.robot_type != robot_type_) {
        throw std::runtime_error("environment robot type does not match runtime robot_type");
      }
      requireLiveSetupTransforms(environment_directory);
      try {
        const auto initial_identity = waitForInitialSimulationExecutionIdentity(
          [this, &identity](std::chrono::nanoseconds timeout) {
            return querySimulationExecutionContract(
              identity.robot_type, "BuildRecording initial simulation identity query failed", timeout);
          },
          [goal_handle]() {return goal_handle->is_canceling();},
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(action_server_timeout_s_)),
          kInitialSimulationIdentityAttemptTimeout, kInitialSimulationIdentityRetryInterval,
          "BuildRecording initial simulation identity check");
        if (!initial_identity) {
          (*finish)(
            false, true, "CANCELED", "build canceled while waiting for simulation identity");
          return;
        }
      } catch (const std::exception & error) {
        (*finish)(false, false, "SIMULATION_IDENTITY_REJECTED", error.what());
        return;
      }
      simulation_validation_config = SimulationRecordingValidator::loadConfig(
        environment_directory / "recording.yaml");
      start_policy = loadStartStatePolicy(environment_directory);
      const auto & runtime_gate = safety_gate_.config();
      if (std::abs(start_policy.max_joint_error_rad - runtime_gate.max_joint_error_rad) > 1.0e-12 ||
        std::abs(start_policy.settle_velocity_rad_s - runtime_gate.settle_velocity_rad_s) > 1.0e-12)
      {
        throw std::runtime_error(
                "runtime start-state thresholds do not match recording environment policy");
      }

      compiled_evidence = RecordingStore::validateCompiledRun(
        std::filesystem::path(telemetry_root_), environment_directory,
        goal->source_trajectory_path,
        goal->compiled_trajectory_path, goal->environment_id, goal->trajectory_id);
      const auto authoritative_source = std::filesystem::canonical(
        environment_directory / "trajectories" / (goal->trajectory_id + ".csv"));
      if (RecordingStore::sha256File(authoritative_source) !=
        RecordingStore::sha256File(compiled_evidence.source_trajectory_path))
      {
        throw std::runtime_error(
                "compiled run source_tip_trajectory.csv does not match the selected environment trajectory");
      }
      const std::string recording_id = goal->trajectory_id + "__" + timestampString();
      const auto recordings_root = environment_directory / "recordings";
      std::filesystem::create_directories(recordings_root);
      *staging_directory = recordings_root / ("." + recording_id + ".staging");
      *final_directory = recordings_root / recording_id;
      if (std::filesystem::exists(*staging_directory) || std::filesystem::exists(*final_directory)) {
        throw std::runtime_error("recording id already exists: " + recording_id);
      }
      std::filesystem::create_directory(*staging_directory);

      YAML::Node building;
      building["schema_version"] = 1;
      building["recording_id"] = recording_id;
      building["status"] = "building";
      building["environment_id"] = identity.environment_id;
      building["trajectory_id"] = goal->trajectory_id;
      writeYaml(*staging_directory / "manifest.yaml", building);

      copyExact(
        compiled_evidence.source_trajectory_path,
        *staging_directory / "source_tip_trajectory.csv");
      // The exact file parsed into the ROS goal is retained byte-for-byte as the canonical payload.
      copyExact(
        compiled_evidence.joint_trajectory_path,
        *staging_directory / "joint_trajectory.csv");
      copyExact(
        compiled_evidence.compile_report_path,
        *staging_directory / "compile_report.yaml");
      requireCompiledEvidenceFilesUnchanged(
        *staging_directory, compiled_evidence, "compiled input copy verification failed");
      snapshotEnvironment(environment_directory, *staging_directory / "environment_snapshot");
      trajectory = RecordingStore::loadJointTrajectoryCsv(
        *staging_directory / "joint_trajectory.csv", identity.joint_names, &phases);
      const double compiled_duration_s = durationSeconds(
        trajectory.points.back().time_from_start);
      if (trajectory.points.size() != compiled_evidence.point_count ||
        std::abs(compiled_duration_s - compiled_evidence.duration_s) > 1.0e-6)
      {
        throw std::runtime_error(
                "joint_trajectory.csv point_count/duration does not match compile_report.yaml");
      }
      simulation_runner_->configureRecordingKinematics(
        loadRobotValidationModel(
          *staging_directory / "environment_snapshot", identity.joint_names));
    } catch (const std::exception & error) {
      (*finish)(false, false, "INPUT_REJECTED", error.what());
      return;
    }

    if (goal_handle->is_canceling()) {
      (*finish)(false, true, "CANCELED", "build canceled before simulation");
      return;
    }

    if (start_policy.mismatch_policy == "curobo_prepare") {
      std::vector<std::pair<std::string, std::vector<double>>> prepare_stages;
      if (start_policy.prepare_sequence == "home_then_recording_start") {
        prepare_stages.emplace_back("configured startup posture", identity.home_joint_positions);
      }
      prepare_stages.emplace_back(
        "authoritative recording start", trajectory.points.front().positions);
      const float stage_span = 0.07F / static_cast<float>(prepare_stages.size());
      for (std::size_t stage_index = 0U; stage_index < prepare_stages.size(); ++stage_index) {
        const auto & [stage_label, target_positions] = prepare_stages[stage_index];
        const float stage_base = 0.03F + stage_span * static_cast<float>(stage_index);
        const auto stage_feedback =
          [this, goal_handle, stage_base, stage_span](
          const std::string & phase, double, float progress, const std::string & detail) {
            const float phase_progress = phase == "PLANNING" ? 0.05F :
              (phase == "SETTLING" ? 0.95F : 0.15F + 0.75F * progress);
            publishBuildFeedback(
              goal_handle,
              phase == "PLANNING" ? "PREPARE_PLANNING" :
              (phase == "SETTLING" ? "PREPARE_SETTLING" : "PREPARING"),
              stage_base + stage_span * phase_progress,
              detail + "; prepare is outside the authoritative recording window");
          };
        const auto outcome = runPrepareStage(
          goal->environment_id, trajectory.joint_names, target_positions,
          compiled_evidence.curobo_revision, compiled_evidence.environment_hashes,
          stage_label, [goal_handle]() {return goal_handle->is_canceling();}, stage_feedback);
        if (!outcome.success) {
          (*finish)(
            false, outcome.canceled, outcome.canceled ? "CANCELED" : "PREPARE_REJECTED",
            outcome.message);
          return;
        }
      }
    } else {
      const auto baseline = jointStateSnapshot();
      const auto settled_state = waitForFreshJointStateAfter(
        baseline.sequence,
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(action_server_timeout_s_)),
        [goal_handle]() {return goal_handle->is_canceling();});
      if (!settled_state || !safety_gate_.checkStartState(
          trajectory, *settled_state, now()).accepted)
      {
        (*finish)(
          false, goal_handle->is_canceling(),
          goal_handle->is_canceling() ? "CANCELED" : "START_STATE_REJECTED",
          "recording environment forbids prepare and mock state is not at source t=0");
        return;
      }
    }

    const SimulationRecordingPaths output_paths{
      *staging_directory / "sim_actual_joint.csv",
      *staging_directory / "sim_controller_state.csv",
      *staging_directory / "joint_frames.csv",
      *staging_directory / "sim_actual_tip.csv"};
    const double trajectory_duration_s = durationSeconds(trajectory.points.back().time_from_start);
    auto trajectory_copy = std::make_shared<trajectory_msgs::msg::JointTrajectory>(trajectory);
    auto identity_copy = std::make_shared<EnvironmentIdentity>(std::move(identity));
    auto environment_copy = std::make_shared<std::filesystem::path>(environment_directory);
    auto compiled_evidence_copy = std::make_shared<CompiledRunEvidence>(compiled_evidence);
    auto validation_config_copy = std::make_shared<SimulationValidationConfig>(
      simulation_validation_config);

    SimulationRunner::Callbacks callbacks;
    callbacks.accepted = [this, goal_handle]() {
        if (goal_handle->is_canceling()) {
          simulation_runner_->cancel();
          return;
        }
        publishBuildFeedback(
          goal_handle, "SIMULATING", 0.10F,
          "mock controller accepted one complete q/qd/qdd/time trajectory goal");
      };
    callbacks.feedback = [this, goal_handle, trajectory_duration_s](
      const FollowJointTrajectory::Feedback & feedback) {
        const double elapsed = durationSeconds(feedback.actual.time_from_start);
        const float progress = trajectory_duration_s > 0.0 ?
          static_cast<float>(0.10 + 0.75 * std::clamp(elapsed / trajectory_duration_s, 0.0, 1.0)) :
          0.10F;
        publishBuildFeedback(goal_handle, "SIMULATING", progress, "recording controller, joints, and TF");
      };
    callbacks.failed_to_start = [finish](const std::string & error) {
        (*finish)(false, false, "SIMULATION_FAILED", error);
      };
    callbacks.completed =
      [this, goal_handle, finish, staging_directory, final_directory, trajectory_copy,
        identity_copy, environment_copy, compiled_evidence_copy, validation_config_copy](
        const PlaybackExecutor::GoalHandle::WrappedResult & result,
        const std::filesystem::path &) {
        if (result.code == rclcpp_action::ResultCode::CANCELED || goal_handle->is_canceling()) {
          (*finish)(false, true, "CANCELED", "simulation trajectory was canceled");
          return;
        }
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
          result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL)
        {
          std::string message = "simulation controller did not succeed: " + resultCodeName(result.code);
          if (result.result && !result.result->error_string.empty()) {
            message += ": " + result.result->error_string;
          }
          (*finish)(false, false, "SIMULATION_FAILED", message);
          return;
        }

        try {
          publishBuildFeedback(
            goal_handle, "VALIDATING_RECORDING", 0.90F,
            "controller succeeded; validating required files and sealing hashes");
          const auto simulation_identity = requireSimulationExecutionContract(
            identity_copy->robot_type,
            "BuildRecording post-execution simulation identity check failed");
          requireCompiledEvidenceFilesUnchanged(
            *staging_directory, *compiled_evidence_copy,
            "compiled evidence changed during simulation");
          const SimulationRecordingPaths validation_paths{
            *staging_directory / "sim_actual_joint.csv",
            *staging_directory / "sim_controller_state.csv",
            *staging_directory / "joint_frames.csv",
            *staging_directory / "sim_actual_tip.csv"};
          const auto kinematics_context =
            SimulationRecordingValidator::loadKinematicsContext(
            *staging_directory / "environment_snapshot",
            *staging_directory / "source_tip_trajectory.csv",
            trajectory_copy->joint_names);
          const double command_duration_s = durationSeconds(
            trajectory_copy->points.back().time_from_start);
          const auto recording_metrics = SimulationRecordingValidator::validate(
            validation_paths, *trajectory_copy, true,
            *validation_config_copy, kinematics_context);

          YAML::Node validation;
          validation["schema_version"] = 1;
          validation["simulation_completed"] = true;
          validation["controller_result"] = "succeeded";
          writeSimulationExecutionIdentity(validation, simulation_identity);
          validation["actual_joint_recorded"] = true;
          validation["controller_state_recorded"] = true;
          validation["joint_frames_recorded"] = true;
          validation["actual_tip_recorded"] = true;
          validation["point_count"] = trajectory_copy->points.size();
          validation["duration_s"] = command_duration_s;
          validation["source_waypoint_count"] =
            compiled_evidence_copy->source_waypoint_count;
          validation["matched_source_waypoint_count"] =
            compiled_evidence_copy->matched_source_waypoint_count;
          validation["source_duration_s"] = compiled_evidence_copy->source_duration_s;
          validation["max_source_time_error_s"] =
            compiled_evidence_copy->max_source_time_error_s;
          validation["sample_dt_s"] = validation_config_copy->sample_dt_s;
          validation["max_missing_samples_allowed"] =
            validation_config_copy->max_missing_samples;
          validation["max_time_jitter_s_allowed"] =
            validation_config_copy->max_time_jitter_s;
          validation["require_action_success"] =
            validation_config_copy->require_action_success;
          validation["actual_joint_sample_count"] =
            recording_metrics.actual_joint.sample_count;
          validation["actual_joint_first_trajectory_time_s"] =
            recording_metrics.actual_joint.first_trajectory_time_s;
          validation["actual_joint_last_trajectory_time_s"] =
            recording_metrics.actual_joint.last_trajectory_time_s;
          validation["actual_joint_coverage_duration_s"] =
            recording_metrics.actual_joint.coverage_duration_s;
          validation["actual_joint_max_time_interval_s"] =
            recording_metrics.actual_joint.max_time_interval_s;
          validation["actual_joint_max_time_jitter_s"] =
            recording_metrics.actual_joint.max_time_jitter_s;
          validation["actual_joint_missing_sample_estimate"] =
            recording_metrics.actual_joint.missing_sample_estimate;
          validation["controller_sample_count"] =
            recording_metrics.controller_state.sample_count;
          validation["controller_first_trajectory_time_s"] =
            recording_metrics.controller_state.first_trajectory_time_s;
          validation["controller_last_trajectory_time_s"] =
            recording_metrics.controller_state.last_trajectory_time_s;
          validation["controller_coverage_duration_s"] =
            recording_metrics.controller_state.coverage_duration_s;
          validation["controller_max_time_interval_s"] =
            recording_metrics.controller_state.max_time_interval_s;
          validation["controller_max_time_jitter_s"] =
            recording_metrics.controller_state.max_time_jitter_s;
          validation["controller_missing_sample_estimate"] =
            recording_metrics.controller_state.missing_sample_estimate;
          validation["joint_frame_sample_count"] =
            recording_metrics.joint_frame_sample_count;
          validation["joint_frame_stream_count"] =
            recording_metrics.joint_frame_streams.size();
          validation["max_joint_frame_quaternion_norm_error"] =
            recording_metrics.max_joint_frame_quaternion_norm_error;
          YAML::Node joint_frame_streams(YAML::NodeType::Sequence);
          for (const auto & frame_metrics : recording_metrics.joint_frame_streams) {
            YAML::Node frame;
            frame["frame_id"] = frame_metrics.frame_id;
            frame["sample_count"] = frame_metrics.pose.sampling.sample_count;
            frame["first_trajectory_time_s"] =
              frame_metrics.pose.sampling.first_trajectory_time_s;
            frame["last_trajectory_time_s"] =
              frame_metrics.pose.sampling.last_trajectory_time_s;
            frame["coverage_duration_s"] =
              frame_metrics.pose.sampling.coverage_duration_s;
            frame["max_time_interval_s"] =
              frame_metrics.pose.sampling.max_time_interval_s;
            frame["max_time_jitter_s"] =
              frame_metrics.pose.sampling.max_time_jitter_s;
            frame["missing_sample_estimate"] =
              frame_metrics.pose.sampling.missing_sample_estimate;
            frame["max_quaternion_norm_error"] =
              frame_metrics.pose.max_quaternion_norm_error;
            joint_frame_streams.push_back(frame);
          }
          validation["joint_frame_streams"] = joint_frame_streams;
          validation["actual_tip_sample_count"] =
            recording_metrics.actual_tip.sampling.sample_count;
          validation["actual_tip_first_trajectory_time_s"] =
            recording_metrics.actual_tip.sampling.first_trajectory_time_s;
          validation["actual_tip_last_trajectory_time_s"] =
            recording_metrics.actual_tip.sampling.last_trajectory_time_s;
          validation["actual_tip_coverage_duration_s"] =
            recording_metrics.actual_tip.sampling.coverage_duration_s;
          validation["actual_tip_max_time_interval_s"] =
            recording_metrics.actual_tip.sampling.max_time_interval_s;
          validation["actual_tip_max_time_jitter_s"] =
            recording_metrics.actual_tip.sampling.max_time_jitter_s;
          validation["actual_tip_missing_sample_estimate"] =
            recording_metrics.actual_tip.sampling.missing_sample_estimate;
          validation["actual_tip_max_quaternion_norm_error"] =
            recording_metrics.actual_tip.max_quaternion_norm_error;
          validation["max_quaternion_norm_error_allowed"] =
            validation_config_copy->max_quaternion_norm_error;
          validation["max_completion_overrun_s_allowed"] =
            validation_config_copy->max_completion_overrun_s;
          validation["max_speed_scaling_error"] =
            recording_metrics.max_speed_scaling_error;
          validation["speed_scaling_evidence"] =
            kRequiredSimulationSpeedScalingEvidence;
          validation["max_controller_reference_position_error_rad"] =
            recording_metrics.max_controller_reference_position_error_rad;
          validation["max_controller_reference_position_error_rad_allowed"] =
            validation_config_copy->max_controller_reference_position_error_rad;
          validation["max_controller_reference_velocity_error_rad_s"] =
            recording_metrics.max_controller_reference_velocity_error_rad_s;
          validation["max_controller_reference_velocity_error_rad_s_allowed"] =
            validation_config_copy->max_controller_reference_velocity_error_rad_s;
          validation["max_joint_position_tracking_error_rad"] =
            recording_metrics.max_joint_position_tracking_error_rad;
          validation["max_joint_position_tracking_error_rad_allowed"] =
            validation_config_copy->max_joint_position_tracking_error_rad;
          validation["max_joint_velocity_tracking_error_rad_s"] =
            recording_metrics.max_joint_velocity_tracking_error_rad_s;
          validation["max_joint_velocity_tracking_error_rad_s_allowed"] =
            validation_config_copy->max_joint_velocity_tracking_error_rad_s;
          validation["max_actual_controller_position_error_rad"] =
            recording_metrics.max_actual_controller_position_error_rad;
          validation["max_actual_controller_position_error_rad_allowed"] =
            validation_config_copy->max_actual_controller_position_error_rad;
          validation["max_actual_controller_velocity_error_rad_s"] =
            recording_metrics.max_actual_controller_velocity_error_rad_s;
          validation["max_actual_controller_velocity_error_rad_s_allowed"] =
            validation_config_copy->max_actual_controller_velocity_error_rad_s;
          validation["max_frame_fk_position_error_m"] =
            recording_metrics.max_frame_fk_position_error_m;
          validation["max_frame_fk_position_error_m_allowed"] =
            validation_config_copy->max_frame_fk_position_error_m;
          validation["max_frame_fk_orientation_error_deg"] =
            recording_metrics.max_frame_fk_orientation_error_deg;
          validation["max_frame_fk_orientation_error_deg_allowed"] =
            validation_config_copy->max_frame_fk_orientation_error_deg;
          validation["max_tip_fk_position_error_m"] =
            recording_metrics.max_tip_fk_position_error_m;
          validation["max_tip_fk_position_error_m_allowed"] =
            validation_config_copy->max_frame_fk_position_error_m;
          validation["max_tip_fk_orientation_error_deg"] =
            recording_metrics.max_tip_fk_orientation_error_deg;
          validation["max_tip_fk_orientation_error_deg_allowed"] =
            validation_config_copy->max_frame_fk_orientation_error_deg;
          validation["max_tip_source_position_error_m"] =
            recording_metrics.max_tip_source_position_error_m;
          validation["max_tip_source_position_error_m_allowed"] =
            kinematics_context.max_tip_source_position_error_m;
          validation["max_tip_source_orientation_error_deg"] =
            recording_metrics.max_tip_source_orientation_error_deg;
          validation["max_tip_source_orientation_error_deg_allowed"] =
            kinematics_context.max_tip_source_orientation_error_deg;
          validation["uniform_time_scale"] = compiled_evidence_copy->uniform_time_scale;
          validation["max_joint_velocity_ratio"] =
            compiled_evidence_copy->max_joint_velocity_ratio;
          validation["max_joint_acceleration_ratio"] =
            compiled_evidence_copy->max_joint_acceleration_ratio;
          validation["max_joint_jerk_ratio"] = compiled_evidence_copy->max_joint_jerk_ratio;
          validation["max_tip_position_error_m"] =
            compiled_evidence_copy->max_tip_position_error_m;
          validation["max_tip_orientation_error_deg"] =
            compiled_evidence_copy->max_tip_orientation_error_deg;
          validation["minimum_collision_distance_m"] =
            compiled_evidence_copy->minimum_collision_distance_m;
          validation["max_scene_collision_cost"] =
            compiled_evidence_copy->max_scene_collision_cost;
          validation["max_self_collision_cost"] =
            compiled_evidence_copy->max_self_collision_cost;
          validation["max_joint_bound_cost"] =
            compiled_evidence_copy->max_joint_bound_cost;
          validation["checked_sample_count"] =
            compiled_evidence_copy->checked_sample_count;
          writeYaml(*staging_directory / "validation_report.yaml", validation);

          YAML::Node manifest;
          manifest["schema_version"] = 1;
          manifest["recording_id"] = final_directory->filename().string();
          manifest["status"] = "validated";
          manifest["environment_id"] = identity_copy->environment_id;
          manifest["trajectory_id"] = final_directory->filename().string().substr(
            0U, final_directory->filename().string().rfind("__"));
          manifest["robot"]["type"] = identity_copy->robot_type;
          for (const auto & name : identity_copy->joint_names) {
            manifest["robot"]["joint_names"].push_back(name);
          }
          manifest["trajectory"]["duration_s"] =
            durationSeconds(trajectory_copy->points.back().time_from_start);
          manifest["trajectory"]["point_count"] = trajectory_copy->points.size();
          manifest["trajectory"]["source_waypoint_count"] =
            compiled_evidence_copy->source_waypoint_count;
          manifest["trajectory"]["matched_source_waypoint_count"] =
            compiled_evidence_copy->matched_source_waypoint_count;
          manifest["trajectory"]["source_duration_s"] =
            compiled_evidence_copy->source_duration_s;
          manifest["trajectory"]["max_source_time_error_s"] =
            compiled_evidence_copy->max_source_time_error_s;
          if (trajectory_copy->points.size() > 1U) {
            manifest["trajectory"]["nominal_dt_s"] =
              durationSeconds(trajectory_copy->points[1].time_from_start) -
              durationSeconds(trajectory_copy->points[0].time_from_start);
          }
          manifest["trajectory"]["interpolation"] = "spline";
          manifest["trajectory"]["uniform_time_scale"] =
            compiled_evidence_copy->uniform_time_scale;
          manifest["trajectory"]["max_joint_velocity_ratio"] =
            compiled_evidence_copy->max_joint_velocity_ratio;
          manifest["trajectory"]["max_joint_acceleration_ratio"] =
            compiled_evidence_copy->max_joint_acceleration_ratio;
          manifest["trajectory"]["max_joint_jerk_ratio"] =
            compiled_evidence_copy->max_joint_jerk_ratio;
          for (const auto & field : {"position", "velocity", "acceleration"}) {
            manifest["trajectory"]["fields"].push_back(field);
          }
          manifest["validation"] = validation;

          // Seal exactly the environment evidence used by compilation.  Both the
          // live environment and the byte-for-byte recording snapshot must still
          // equal that evidence; otherwise a long simulation could publish a
          // snapshot from A with a manifest recomputed from later environment B.
          requireCompiledEvidenceFilesUnchanged(
            *staging_directory, *compiled_evidence_copy,
            "compiled evidence changed before recording seal");
          const auto current_environment_hashes = environmentHashes(*environment_copy);
          const auto snapshot_environment_hashes = environmentHashes(
            *staging_directory / "environment_snapshot");
          if (current_environment_hashes != compiled_evidence_copy->environment_hashes) {
            throw std::runtime_error(
                    "authoritative environment changed after compilation; refusing to seal recording");
          }
          if (snapshot_environment_hashes != compiled_evidence_copy->environment_hashes) {
            throw std::runtime_error(
                    "recording environment_snapshot differs from compiled environment evidence");
          }
          for (const auto & [key, value] : compiled_evidence_copy->environment_hashes) {
            manifest["hashes"][key] = value;
          }
          manifest["hashes"]["source_trajectory"] =
            compiled_evidence_copy->source_trajectory_sha256;
          manifest["hashes"]["joint_trajectory"] =
            compiled_evidence_copy->joint_trajectory_sha256;
          manifest["hashes"]["curobo_revision"] = compiled_evidence_copy->curobo_revision;
          writeYaml(*staging_directory / "manifest.yaml", manifest);

          requireCompleteRecording(*staging_directory);
          writeChecksums(*staging_directory);
          if (!std::filesystem::is_regular_file(*staging_directory / "sha256sums.txt") ||
            std::filesystem::file_size(*staging_directory / "sha256sums.txt") == 0U)
          {
            throw std::runtime_error("sha256sums.txt was not sealed");
          }
          std::filesystem::rename(*staging_directory, *final_directory);
          publishBuildFeedback(goal_handle, "VALIDATED", 1.0F, "recording atomically published");
          (*finish)(true, false, "VALIDATED", "simulation succeeded and recording was sealed");
        } catch (const std::exception & error) {
          (*finish)(false, false, "VALIDATION_FAILED", error.what());
        }
      };

    try {
      (void)requireSimulationExecutionContract(
        identity_copy->robot_type,
        "BuildRecording pre-execution simulation identity check failed");
    } catch (const std::exception & error) {
      (*finish)(false, false, "SIMULATION_IDENTITY_REJECTED", error.what());
      return;
    }
    const auto pre_recording_baseline = jointStateSnapshot();
    const auto pre_recording_state = waitForFreshJointStateAfter(
      pre_recording_baseline.sequence,
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(action_server_timeout_s_)),
      [goal_handle]() {return goal_handle->is_canceling();});
    if (goal_handle->is_canceling()) {
      (*finish)(false, true, "CANCELED", "build canceled before formal recording execution");
      return;
    }
    if (!pre_recording_state) {
      (*finish)(
        false, false, "START_STATE_REJECTED",
        "no new JointState was received immediately before formal recording execution");
      return;
    }
    const auto pre_recording_safety = safety_gate_.checkStartState(
      trajectory, *pre_recording_state, now());
    if (!pre_recording_safety.accepted) {
      (*finish)(
        false, false, "START_STATE_REJECTED",
        "final start-state gate rejected formal recording execution: " +
        pre_recording_safety.message);
      return;
    }
    if (!simulation_runner_->run(
        trajectory, output_paths,
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(action_server_timeout_s_)),
        std::move(callbacks)))
    {
      RCLCPP_ERROR(
        get_logger(), "failed to submit simulation trajectory to %s",
        simulation_action_name_.c_str());
    }
  }

  void executeGoal(const std::shared_ptr<PlayGoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto finish_once = std::make_shared<std::atomic_bool>(false);
    auto telemetry_path = std::make_shared<std::filesystem::path>();
    auto finish = std::make_shared<std::function<void(bool, bool, const std::string &, const std::string &)>>();
    *finish = [this, goal_handle, finish_once, telemetry_path](
      bool success, bool /*canceled*/, const std::string & state, const std::string & message) {
        if (finish_once->exchange(true)) {
          return;
        }
        const auto release_goal = [this]() {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          goal_active_ = false;
        };
        try {
          telemetry_recorder_->clearStrictSpeedScalingGuard();
          if (telemetry_recorder_->active()) {
            *telemetry_path = telemetry_recorder_->stop(state);
          }
          auto result = std::make_shared<PlayRecording::Result>();
          result->success = success;
          result->telemetry_path = telemetry_path->string();
          result->message = message;

          // Release the process-wide operation gate before any rclcpp_action terminal
          // transition, because those transitions can throw on an unexpected state.
          release_goal();
          if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
          } else if (!goal_handle->is_active()) {
            RCLCPP_WARN(
              get_logger(), "PlayRecording goal was already terminal while finishing state=%s",
              state.c_str());
          } else if (success) {
            goal_handle->succeed(result);
          } else {
            // A downstream CANCELED result without an upstream cancel request is an abort,
            // not a legal upstream CANCELED transition.
            goal_handle->abort(result);
          }
        } catch (const std::exception & error) {
          release_goal();
          RCLCPP_ERROR(
            get_logger(), "failed to finalize PlayRecording goal safely: %s", error.what());
        } catch (...) {
          release_goal();
          RCLCPP_ERROR(get_logger(), "failed to finalize PlayRecording goal safely");
        }
      };

    publishFeedback(goal_handle, "VALIDATING", 0.0, 0.0, "loading validated recording");
    RecordingArtifact artifact;
    std::filesystem::path environment_directory;
    EnvironmentIdentity environment_identity;
    StartStatePolicy start_policy;
    try {
      environment_directory = std::filesystem::canonical(
        store_.environmentsRoot() / goal->environment_id);
      if (!goal->environment_path.empty() &&
        std::filesystem::canonical(goal->environment_path) != environment_directory)
      {
        throw std::runtime_error(
                "environment_path must resolve to the selected environment under environments_root");
      }
      artifact = store_.loadValidated(
        goal->environment_id, goal->recording_id, robot_type_, joint_names_);
      environment_identity = loadEnvironmentIdentity(environment_directory);
      if (environment_identity.environment_id != goal->environment_id ||
        environment_identity.robot_type != artifact.manifest.robot_type ||
        environment_identity.joint_names != artifact.trajectory.joint_names)
      {
        throw std::runtime_error(
                "current environment identity does not match the validated recording");
      }
      requireLiveSetupTransforms(environment_directory);
      start_policy = loadStartStatePolicy(environment_directory);
      const auto & runtime_gate = safety_gate_.config();
      if (std::abs(start_policy.max_joint_error_rad - runtime_gate.max_joint_error_rad) > 1.0e-12 ||
        std::abs(start_policy.settle_velocity_rad_s - runtime_gate.settle_velocity_rad_s) > 1.0e-12)
      {
        throw std::runtime_error(
                "runtime start-state thresholds do not match the validated environment policy");
      }
      if (execution_mode_ == "real") {
        requireRealCalibration(
          environment_directory, std::filesystem::path(driver_kinematics_params_file_));
        requireRealControllerContract(environment_directory, selected_action_name_);
      }
    } catch (const std::exception & error) {
      (*finish)(false, false, "REJECTED", "recording validation failed: " + std::string(error.what()));
      return;
    }

    if (execution_mode_ == "simulation") {
      try {
        const auto initial_identity = waitForInitialSimulationExecutionIdentity(
          [this, &artifact](std::chrono::nanoseconds timeout) {
            return querySimulationExecutionContract(
              artifact.manifest.robot_type,
              "PlayRecording initial simulation identity query failed", timeout);
          },
          [goal_handle]() {return goal_handle->is_canceling();},
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(action_server_timeout_s_)),
          kInitialSimulationIdentityAttemptTimeout, kInitialSimulationIdentityRetryInterval,
          "PlayRecording initial simulation identity check");
        if (!initial_identity) {
          (*finish)(
            false, true, "CANCELED", "playback canceled while waiting for simulation identity");
          return;
        }
      } catch (const std::exception & error) {
        (*finish)(false, false, "SIMULATION_IDENTITY_REJECTED", error.what());
        return;
      }
    }

    if (!executor_->waitForServer(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(action_server_timeout_s_))))
    {
      (*finish)(
        false, false, "REJECTED",
        "configured controller action is unavailable: " + selected_action_name_ +
        "; no fallback was attempted");
      return;
    }

    const auto run_directory = std::filesystem::path(telemetry_root_) /
      (goal->recording_id + "__" + timestampString()) / execution_mode_;
    *telemetry_path = run_directory / "execution_telemetry.csv";
    auto artifact_ptr = std::make_shared<RecordingArtifact>(std::move(artifact));
    if (goal->externally_prepared_start) {
      const auto settled = waitForSettledTrajectoryStart(
        artifact_ptr->trajectory, [goal_handle]() {return goal_handle->is_canceling();});
      if (!settled.state || goal_handle->is_canceling()) {
        (*finish)(
          false, settled.canceled || goal_handle->is_canceling(),
          settled.canceled || goal_handle->is_canceling() ? "CANCELED" : "REJECTED",
          "externally prepared start gate rejected playback: " + settled.message);
        return;
      }
      RCLCPP_INFO(
        get_logger(),
        "externally prepared start matches recording %s; sending sealed trajectory directly",
        goal->recording_id.c_str());
      sendRecordingTrajectory(goal_handle, artifact_ptr, telemetry_path, finish);
      return;
    }
    std::vector<std::pair<std::string, std::vector<double>>> prepare_stages;
    if (start_policy.prepare_sequence == "home_then_recording_start") {
      prepare_stages.emplace_back(
        "configured startup posture", environment_identity.home_joint_positions);
      prepare_stages.emplace_back(
        "authoritative recording start", artifact_ptr->trajectory.points.front().positions);
    } else {
      const auto latest = latestJointState();
      if (!latest) {
        (*finish)(false, false, "REJECTED", "no JointState received on " + joint_states_topic_);
        return;
      }
      const SafetyGateResult safety = safety_gate_.checkStartState(
        artifact_ptr->trajectory, *latest, now());
      if (safety.accepted) {
        sendRecordingTrajectory(goal_handle, artifact_ptr, telemetry_path, finish);
        return;
      }
      if (!goal->auto_prepare) {
        (*finish)(
          false, false, "REJECTED", "start safety gate rejected playback: " + safety.message);
        return;
      }
      if (start_policy.mismatch_policy != "curobo_prepare" ||
        safety.failure_reason != SafetyGateFailureReason::POSITION_MISMATCH)
      {
        (*finish)(
          false, false, "REJECTED",
          "automatic prepare is forbidden for this start-state failure: " + safety.message);
        return;
      }
      prepare_stages.emplace_back(
        "authoritative recording start", artifact_ptr->trajectory.points.front().positions);
    }

    const auto current_environment_hashes = environmentHashes(environment_directory);
    for (std::size_t stage_index = 0U; stage_index < prepare_stages.size(); ++stage_index) {
      const auto & [stage_label, target_positions] = prepare_stages[stage_index];
      const float stage_offset = static_cast<float>(stage_index) /
        static_cast<float>(prepare_stages.size());
      const float stage_scale = 1.0F / static_cast<float>(prepare_stages.size());
      const auto stage_feedback =
        [this, goal_handle, stage_offset, stage_scale](
        const std::string & phase, double elapsed_s, float progress,
        const std::string & detail) {
          const float overall_progress = stage_offset + stage_scale * progress;
          publishFeedback(
            goal_handle,
            phase == "PLANNING" ? "PREPARE_PLANNING" :
            (phase == "SETTLING" ? "PREPARE_SETTLING" : "PREPARING"),
            elapsed_s, overall_progress,
            detail + "; canonical recording telemetry has not started");
        };
      const auto outcome = runPrepareStage(
        goal->environment_id, artifact_ptr->trajectory.joint_names, target_positions,
        artifact_ptr->manifest.curobo_revision, current_environment_hashes, stage_label,
        [goal_handle]() {return goal_handle->is_canceling();}, stage_feedback);
      if (!outcome.success) {
        (*finish)(
          false, outcome.canceled, outcome.canceled ? "CANCELED" : "REJECTED",
          outcome.message);
        return;
      }
    }
    sendRecordingTrajectory(goal_handle, artifact_ptr, telemetry_path, finish);
  }

  using PlayFinishFunction =
    std::function<void(bool, bool, const std::string &, const std::string &)>;

  struct JointStateSnapshot
  {
    sensor_msgs::msg::JointState::SharedPtr state;
    std::uint64_t sequence{0U};
  };

  JointStateSnapshot jointStateSnapshot()
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    return JointStateSnapshot{latest_joint_state_, joint_state_sequence_};
  }

  sensor_msgs::msg::JointState::SharedPtr latestJointState()
  {
    return jointStateSnapshot().state;
  }

  JointStateSnapshot waitForFreshJointStateSnapshotAfter(
    std::uint64_t baseline_sequence,
    std::chrono::nanoseconds timeout,
    const std::function<bool()> & stop_waiting)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::unique_lock<std::mutex> lock(joint_state_mutex_);
    while (joint_state_sequence_ <= baseline_sequence) {
      if (stop_waiting && stop_waiting()) {
        return {};
      }
      const auto now_steady = std::chrono::steady_clock::now();
      if (now_steady >= deadline) {
        return {};
      }
      joint_state_condition_.wait_for(
        lock, std::min(std::chrono::milliseconds(50),
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now_steady)));
    }
    return JointStateSnapshot{latest_joint_state_, joint_state_sequence_};
  }

  sensor_msgs::msg::JointState::SharedPtr waitForFreshJointStateAfter(
    std::uint64_t baseline_sequence,
    std::chrono::nanoseconds timeout,
    const std::function<bool()> & stop_waiting)
  {
    return waitForFreshJointStateSnapshotAfter(baseline_sequence, timeout, stop_waiting).state;
  }

  struct SettledJointStateResult
  {
    sensor_msgs::msg::JointState::SharedPtr state;
    bool canceled{false};
    std::string message;
  };

  SettledJointStateResult waitForSettledJointState(
    const std::function<SafetyGateResult(const sensor_msgs::msg::JointState &)> & check,
    const std::function<bool()> & cancel_requested,
    bool allow_target_convergence = false)
  {
    const auto started_at = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(prepare_settle_timeout_s_));
    const auto deadline = started_at + timeout;
    SettledStateWait wait(started_at, timeout, allow_target_convergence);
    auto sequence = jointStateSnapshot().sequence;
    while (true) {
      auto status = wait.poll(std::chrono::steady_clock::now(), cancel_requested());
      if (status == SettledStateWaitStatus::WAITING) {
        const auto remaining = std::chrono::duration_cast<std::chrono::nanoseconds>(
          deadline - std::chrono::steady_clock::now());
        const auto sample = waitForFreshJointStateSnapshotAfter(
          sequence, std::min(remaining, std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::milliseconds(200))), cancel_requested);
        if (!sample.state) {
          continue;
        }
        sequence = sample.sequence;
        status = wait.observe(check(*sample.state), std::chrono::steady_clock::now(),
          cancel_requested());
        if (status == SettledStateWaitStatus::SETTLED) {
          return {sample.state, false, "fresh actual JointStates remained settled for 100 ms"};
        }
      }
      if (status == SettledStateWaitStatus::REJECTED) {
        return {{}, false, wait.lastSafety().message};
      }
      if (status == SettledStateWaitStatus::CANCELED) {
        return {{}, true, "canceled while waiting for fresh settled JointStates"};
      }
      if (status == SettledStateWaitStatus::TIMED_OUT) {
        return {{}, false, "settle wait timed out: " + wait.lastSafety().message};
      }
    }
  }

  SettledJointStateResult waitForSettledTrajectoryStart(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::function<bool()> & cancel_requested)
  {
    return waitForSettledJointState(
      [this, &trajectory](const auto & state) {
        auto safety = safety_gate_.checkStartState(trajectory, state, now());
        // The original gate reports speed before position. At a fixed start,
        // being outside its position bound is a hard failure, not a reason to
        // wait for the robot to move itself back into place.
        if (safety.failure_reason == SafetyGateFailureReason::NOT_SETTLED &&
          safety.max_joint_error_rad >= safety_gate_.config().max_joint_error_rad)
        {
          safety.failure_reason = SafetyGateFailureReason::POSITION_MISMATCH;
          safety.message = "robot is outside the fixed trajectory start position bound";
        }
        return safety;
      }, cancel_requested);
  }

  struct PrepareStageOutcome
  {
    bool success{false};
    bool canceled{false};
    std::string message;
  };

  using PrepareStageFeedback =
    std::function<void(const std::string &, double, float, const std::string &)>;

  PrepareStageOutcome runPrepareStage(
    const std::string & environment_id,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & target_positions,
    const std::string & expected_curobo_revision,
    const std::unordered_map<std::string, std::string> & expected_environment_hashes,
    const std::string & stage_label,
    const std::function<bool()> & cancel_requested,
    const PrepareStageFeedback & feedback)
  {
    PrepareStageOutcome outcome;
    const auto fail = [&outcome, &stage_label](const std::string & message) {
        outcome.message = stage_label + ": " + message;
        return outcome;
      };
    if (joint_names.empty() || target_positions.size() != joint_names.size() ||
      !std::all_of(target_positions.begin(), target_positions.end(), [](double value) {
        return std::isfinite(value);
      }))
    {
      return fail("target joint positions are incomplete or non-finite");
    }
    if (cancel_requested()) {
      outcome.canceled = true;
      return fail("canceled before planning");
    }

    std::vector<double> current_positions;
    std::string extraction_error;
    const auto initial = waitForSettledJointState(
      [this, &joint_names, &current_positions, &extraction_error](const auto & state) {
        if (!extractOrderedJointPositions(
            state, joint_names, current_positions, extraction_error))
        {
          SafetyGateResult invalid;
          invalid.failure_reason = SafetyGateFailureReason::INVALID_JOINT_STATE;
          invalid.message = "cannot extract current joint positions: " + extraction_error;
          return invalid;
        }
        return safety_gate_.checkStartState(
          makeStartGateTrajectory(joint_names, current_positions), state, now());
      }, cancel_requested);
    if (!initial.state) {
      outcome.canceled = initial.canceled;
      return fail("current robot state is not fresh and settled: " + initial.message);
    }

    if (!prepare_client_->wait_for_service(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(action_server_timeout_s_))))
    {
      return fail("cuRobo prepare planner service is unavailable");
    }
    feedback(
      "PLANNING", 0.0, 0.0F,
      "planning collision-aware motion for " + stage_label);
    auto request = std::make_shared<PlanJointPath::Request>();
    request->environment_id = environment_id;
    request->joint_names = joint_names;
    request->current_positions = current_positions;
    request->target_positions = target_positions;
    auto future = prepare_client_->async_send_request(request);
    const auto planning_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(prepare_planning_timeout_s_));
    while (future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
      if (cancel_requested()) {
        outcome.canceled = true;
        return fail("canceled during cuRobo planning");
      }
      if (std::chrono::steady_clock::now() >= planning_deadline) {
        return fail("cuRobo planning timed out");
      }
    }
    const auto response = future.get();
    if (cancel_requested()) {
      outcome.canceled = true;
      return fail("canceled after cuRobo planning");
    }
    if (!response || !response->success) {
      return fail(
        "cuRobo planning failed: " +
        (response ? response->message : std::string("empty service response")));
    }
    if (response->curobo_revision != expected_curobo_revision) {
      return fail("cuRobo revision differs from the validated trajectory evidence");
    }
    try {
      requirePrepareEnvironmentEvidence(*response, expected_environment_hashes);
    } catch (const std::exception & error) {
      return fail(error.what());
    }
    const auto validation = validatePrepareTrajectory(
      response->trajectory, joint_names, current_positions, target_positions,
      prepare_validation_config_);
    if (!validation.accepted) {
      return fail("prepare trajectory validation failed: " + validation.message);
    }

    // Planning can be expensive. Re-prove a fresh, settled state at the exact
    // planned start before sending the collision-checked controller goal.
    const auto pre_send = waitForSettledJointState(
      [this, &response](const auto & state) {
        auto safety = safety_gate_.checkStartState(response->trajectory, state, now());
        // Preserve the stricter prepare endpoint check even when the speed gate
        // reports NOT_SETTLED first: never wait away a changed planned start.
        if ((safety.accepted || safety.failure_reason == SafetyGateFailureReason::NOT_SETTLED ||
          safety.failure_reason == SafetyGateFailureReason::POSITION_MISMATCH) &&
          safety.max_joint_error_rad > prepare_validation_config_.endpoint_tolerance_rad)
        {
          safety.accepted = false;
          safety.failure_reason = SafetyGateFailureReason::POSITION_MISMATCH;
          safety.message = "robot moved after planning; planned start error is " +
            std::to_string(safety.max_joint_error_rad) + " rad";
        }
        return safety;
      }, cancel_requested);
    if (!pre_send.state) {
      outcome.canceled = pre_send.canceled;
      return fail("robot state changed or became unsafe during planning: " + pre_send.message);
    }
    if (!executor_->waitForServer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(action_server_timeout_s_))))
    {
      return fail("configured controller is unavailable for prepare execution");
    }

    struct ExecutionState
    {
      std::mutex mutex;
      bool done{false};
      bool success{false};
      bool canceled{false};
      std::string message;
    };
    auto execution = std::make_shared<ExecutionState>();
    PlaybackExecutor::Callbacks callbacks;
    callbacks.accepted = [feedback, stage_label]() {
        feedback(
          "EXECUTING", 0.0, 0.0F,
          "controller accepted collision-aware motion for " + stage_label);
      };
    callbacks.feedback = [feedback, stage_label, validation](
      const FollowJointTrajectory::Feedback & controller_feedback) {
        const double elapsed_s = durationSeconds(controller_feedback.actual.time_from_start);
        const float progress = validation.duration_s > 0.0 ? static_cast<float>(
          std::clamp(elapsed_s / validation.duration_s, 0.0, 1.0)) : 0.0F;
        feedback(
          "EXECUTING", elapsed_s, progress,
          "executing collision-aware motion for " + stage_label);
      };
    callbacks.failed_to_start = [execution](const std::string & error) {
        std::lock_guard<std::mutex> lock(execution->mutex);
        execution->done = true;
        execution->message = error;
      };
    callbacks.completed = [execution](
      const PlaybackExecutor::GoalHandle::WrappedResult & result) {
        std::lock_guard<std::mutex> lock(execution->mutex);
        execution->done = true;
        execution->success =
          result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
          result.result->error_code == FollowJointTrajectory::Result::SUCCESSFUL;
        execution->canceled = result.code == rclcpp_action::ResultCode::CANCELED;
        execution->message = "controller result=" + resultCodeName(result.code);
        if (result.result && !result.result->error_string.empty()) {
          execution->message += ": " + result.result->error_string;
        }
      };
    if (!executor_->sendTrajectory(response->trajectory, std::move(callbacks))) {
      std::lock_guard<std::mutex> lock(execution->mutex);
      return fail(
        execution->message.empty() ?
        "failed to submit prepare trajectory" : execution->message);
    }

    const auto execution_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(
        validation.duration_s + action_server_timeout_s_ + 5.0));
    while (true) {
      {
        std::lock_guard<std::mutex> lock(execution->mutex);
        if (execution->done) {
          break;
        }
      }
      if (cancel_requested()) {
        executor_->cancelActive();
        outcome.canceled = true;
        return fail("canceled during prepare execution");
      }
      if (std::chrono::steady_clock::now() >= execution_deadline) {
        executor_->cancelActive();
        return fail("prepare execution timed out");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    {
      std::lock_guard<std::mutex> lock(execution->mutex);
      if (!execution->success) {
        outcome.canceled = execution->canceled || cancel_requested();
        return fail(execution->message);
      }
    }

    feedback(
      "SETTLING", validation.duration_s, 1.0F,
      "waiting for a fresh settled state after " + stage_label);
    const auto target_gate = makeStartGateTrajectory(joint_names, target_positions);
    const auto settled = waitForSettledJointState(
      [this, &target_gate](const auto & state) {
        return safety_gate_.checkStartState(target_gate, state, now());
      }, cancel_requested, true);
    if (!settled.state) {
      outcome.canceled = settled.canceled;
      return fail("target was not reached and settled: " + settled.message);
    }
    outcome.success = true;
    outcome.message = stage_label + ": collision-checked motion completed and settled";
    return outcome;
  }

  void sendRecordingTrajectory(
    const std::shared_ptr<PlayGoalHandle> & goal_handle,
    const std::shared_ptr<RecordingArtifact> & artifact,
    const std::shared_ptr<std::filesystem::path> & telemetry_path,
    const std::shared_ptr<PlayFinishFunction> & finish)
  {
    try {
      requireLiveSetupTransforms(artifact->directory / "environment_snapshot");
    } catch (const std::exception & error) {
      (*finish)(
        false, false, "REJECTED",
        "final live setup TF gate rejected authoritative playback: " +
        std::string(error.what()));
      return;
    }
    const double duration_s = durationSeconds(
      artifact->trajectory.points.back().time_from_start);
    std::string telemetry_error;
    if (!telemetry_recorder_->prepare(
        artifact->trajectory, artifact->directory / "environment_snapshot", telemetry_error))
    {
      (*finish)(
        false, false, "REJECTED",
        "recording telemetry kinematics could not be prepared: " + telemetry_error);
      return;
    }
    if (execution_mode_ == "simulation") {
      try {
        (void)requireSimulationExecutionContract(
          artifact->manifest.robot_type,
          "PlayRecording pre-execution simulation identity check failed");
      } catch (const std::exception & error) {
        (*finish)(false, false, "SIMULATION_IDENTITY_REJECTED", error.what());
        return;
      }
    }
    const auto pre_recording = waitForSettledTrajectoryStart(
      artifact->trajectory, [goal_handle]() {return goal_handle->is_canceling();});
    if (pre_recording.canceled || goal_handle->is_canceling()) {
      (*finish)(false, true, "CANCELED", "playback canceled before authoritative execution");
      return;
    }
    if (!pre_recording.state) {
      (*finish)(
        false, false, "REJECTED",
        "final start-state gate rejected authoritative playback: " +
        pre_recording.message);
      return;
    }
    if (execution_mode_ == "real") {
      publishFeedback(
        goal_handle, "WAITING_SPEED_SCALING", 0.0, 0.0,
        "waiting for a new speed_scaling=1.0 sample before sending the authoritative trajectory");
      std::string speed_scaling_error;
      if (!telemetry_recorder_->armStrictSpeedScalingGuard(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(speed_scaling_fresh_timeout_s_)),
          kStrictSpeedScalingTolerance, speed_scaling_error))
      {
        (*finish)(
          false, false, "REJECTED",
          "strict speed_scaling gate rejected authoritative playback: " + speed_scaling_error);
        return;
      }

      // The speed sample wait can itself consume time. Re-prove the fixed
      // start with new actual samples; the strict speed guard remains armed.
      const auto immediate = waitForSettledTrajectoryStart(
        artifact->trajectory, [goal_handle]() {return goal_handle->is_canceling();});
      if (!immediate.state || goal_handle->is_canceling()) {
        (*finish)(
          false, immediate.canceled || goal_handle->is_canceling(),
          immediate.canceled || goal_handle->is_canceling() ? "CANCELED" : "REJECTED",
          "start state changed while waiting for speed_scaling evidence: " +
          immediate.message);
        return;
      }
      // Settling can consume up to its bounded timeout. Do not submit even a
      // single goal if speed evidence failed while that wait was in progress.
      // This check must not re-arm and erase an earlier non-unit sample.
      if (!telemetry_recorder_->checkArmedStrictSpeedScalingGuard(speed_scaling_error)) {
        (*finish)(
          false, false, "REJECTED",
          "final strict speed_scaling gate rejected authoritative playback: " +
          speed_scaling_error);
        return;
      }
    }
    publishFeedback(
      goal_handle, "READY", 0.0, 0.0,
      "validated recording and settled start state; sending one complete trajectory goal");

    PlaybackExecutor::Callbacks callbacks;
    callbacks.accepted = [this, goal_handle, telemetry_path, finish]() {
        if (!goal_handle->is_active() || goal_handle->is_canceling()) {
          executor_->cancelActive();
          if (goal_handle->is_canceling()) {
            (*finish)(false, true, "CANCELED", "playback canceled before controller execution");
          }
          return;
        }
        if (!telemetry_recorder_->start(*telemetry_path, now(), "EXECUTING"))
        {
          executor_->cancelActive();
          (*finish)(false, false, "ABORTED", "failed to start actual telemetry recorder");
          return;
        }
        if (execution_mode_ == "real") {
          std::string speed_scaling_error;
          if (!telemetry_recorder_->activateStrictSpeedScalingGuard(
              [this, finish](const std::string & violation) {
                RCLCPP_ERROR(get_logger(), "%s", violation.c_str());
                executor_->cancelActive();
                (*finish)(
                  false, false, "SPEED_SCALING_VIOLATION",
                  "authoritative playback aborted: " + violation);
              },
              speed_scaling_error))
          {
            executor_->cancelActive();
            (*finish)(
              false, false, "SPEED_SCALING_VIOLATION",
              "authoritative playback aborted before execution feedback: " +
              speed_scaling_error);
            return;
          }
        }
        publishFeedback(
          goal_handle, "EXECUTING", 0.0, 0.0,
          "controller accepted the complete recorded trajectory");
      };
    callbacks.feedback = [this, goal_handle, duration_s](
      const FollowJointTrajectory::Feedback & feedback) {
        const double trajectory_time_s = durationSeconds(feedback.actual.time_from_start);
        telemetry_recorder_->updateTrajectoryProgress(trajectory_time_s, now());
        const float progress = duration_s > 0.0 ? static_cast<float>(
          std::clamp(trajectory_time_s / duration_s, 0.0, 1.0)) : 0.0F;
        publishFeedback(
          goal_handle, "EXECUTING", trajectory_time_s, progress,
          "receiving controller execution feedback");
      };
    callbacks.completed = [this, goal_handle, artifact, finish](
      const PlaybackExecutor::GoalHandle::WrappedResult & result) {
        if (execution_mode_ == "real") {
          std::string speed_scaling_violation;
          if (!telemetry_recorder_->finalizeStrictSpeedScalingGuard(speed_scaling_violation)) {
            (*finish)(
              false, false, "SPEED_SCALING_VIOLATION",
              "authoritative playback aborted: " + speed_scaling_violation);
            return;
          }
        }
        const bool controller_success =
          result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
          result.result->error_code == FollowJointTrajectory::Result::SUCCESSFUL;
        const bool canceled = goal_handle->is_canceling();
        if (execution_mode_ == "simulation" && controller_success && !canceled) {
          try {
            (void)requireSimulationExecutionContract(
              artifact->manifest.robot_type,
              "PlayRecording post-execution simulation identity check failed");
          } catch (const std::exception & error) {
            (*finish)(false, false, "SIMULATION_IDENTITY_REJECTED", error.what());
            return;
          }
        }
        std::string message = "controller result=" + resultCodeName(result.code);
        if (result.result && !result.result->error_string.empty()) {
          message += ": " + result.result->error_string;
        }
        (*finish)(
          controller_success && !canceled, canceled,
          controller_success && !canceled ? "SUCCEEDED" : (canceled ? "CANCELED" : "ABORTED"),
          message);
      };
    callbacks.failed_to_start = [goal_handle, finish](const std::string & error) {
        const bool canceled = goal_handle->is_canceling();
        (*finish)(
          false, canceled, canceled ? "CANCELED" : "REJECTED", error);
      };

    if (!executor_->sendTrajectory(artifact->trajectory, std::move(callbacks))) {
      RCLCPP_ERROR(
        get_logger(), "failed to submit trajectory to configured action %s",
        selected_action_name_.c_str());
    }
  }

  void requireLiveSetupTransforms(const std::filesystem::path & environment_directory)
  {
    const SetupConfig setup = loadSetupConfig(environment_directory / "setup.yaml");
    const auto require_transform = [this](
      const std::string & parent,
      const std::string & child,
      const SetupRigidTransform & expected) {
        const auto deadline = std::chrono::steady_clock::now() +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(action_server_timeout_s_));
        std::string last_error;
        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
          try {
            const auto actual = setup_tf_buffer_->lookupTransform(
              parent, child, rclcpp::Time(0, 0, get_clock()->get_clock_type()));
            const std::array<double, 3> translation{{
                actual.transform.translation.x,
                actual.transform.translation.y,
                actual.transform.translation.z}};
            const std::array<double, 4> quaternion{{
                actual.transform.rotation.x,
                actual.transform.rotation.y,
                actual.transform.rotation.z,
                actual.transform.rotation.w}};
            const double translation_error = std::sqrt(
              std::pow(translation[0] - expected.translation_m[0], 2) +
              std::pow(translation[1] - expected.translation_m[1], 2) +
              std::pow(translation[2] - expected.translation_m[2], 2));
            const double actual_norm = std::sqrt(
              quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
              quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
            const double expected_norm = std::sqrt(
              expected.quaternion_xyzw[0] * expected.quaternion_xyzw[0] +
              expected.quaternion_xyzw[1] * expected.quaternion_xyzw[1] +
              expected.quaternion_xyzw[2] * expected.quaternion_xyzw[2] +
              expected.quaternion_xyzw[3] * expected.quaternion_xyzw[3]);
            if (!std::isfinite(translation_error) || !std::isfinite(actual_norm) ||
              actual_norm <= 1.0e-15 || !std::isfinite(expected_norm) ||
              expected_norm <= 1.0e-15)
            {
              throw std::runtime_error("live setup TF contains non-finite geometry");
            }
            double dot = 0.0;
            for (std::size_t index = 0U; index < quaternion.size(); ++index) {
              dot += quaternion[index] * expected.quaternion_xyzw[index] /
                (actual_norm * expected_norm);
            }
            dot = std::clamp(std::abs(dot), 0.0, 1.0);
            const double orientation_error =
              1.0 - dot <= 1.0e-15 ? 0.0 : 2.0 * std::acos(dot);
            if (translation_error > 1.0e-9 || orientation_error > 1.0e-9) {
              std::ostringstream message;
              message << "live TF " << parent << " -> " << child
                      << " differs from setup.yaml: translation_error_m="
                      << translation_error << " orientation_error_rad=" << orientation_error;
              throw std::runtime_error(message.str());
            }
            return;
          } catch (const tf2::TransformException & error) {
            last_error = error.what();
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        throw std::runtime_error(
                "live TF " + parent + " -> " + child +
                " is unavailable for setup validation: " + last_error);
      };

    require_transform(
      setup.work_frame.parent_frame,
      setup.work_frame.child_frame,
      setup.work_frame.parentToChild());
    const SetupRigidTransform identity{};
    for (const auto & alias : setup.work_frame.aliases) {
      require_transform(setup.work_frame.child_frame, alias, identity);
    }
    require_transform(
      setup.tool.parent_link,
      setup.tool.tip_frame,
      setup.tool.parentToTip());
  }

  void publishFeedback(
    const std::shared_ptr<PlayGoalHandle> & goal_handle,
    const std::string & state,
    double trajectory_time_s,
    float progress,
    const std::string & detail)
  {
    if (!goal_handle->is_active()) {
      return;
    }
    auto feedback = std::make_shared<PlayRecording::Feedback>();
    feedback->state = state;
    feedback->trajectory_time_s = trajectory_time_s;
    feedback->progress = progress;
    feedback->detail = detail;
    goal_handle->publish_feedback(feedback);
  }

  void publishBuildFeedback(
    const std::shared_ptr<BuildGoalHandle> & goal_handle,
    const std::string & state,
    float progress,
    const std::string & detail)
  {
    if (!goal_handle->is_active()) {
      return;
    }
    auto feedback = std::make_shared<BuildRecording::Feedback>();
    feedback->state = state;
    feedback->progress = progress;
    feedback->detail = detail;
    goal_handle->publish_feedback(feedback);
  }

  SimulationExecutionIdentity requireSimulationExecutionContract(
    const std::string & expected_hardware_name,
    const std::string & context)
  {
    return querySimulationExecutionContract(
      expected_hardware_name, context,
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(action_server_timeout_s_)));
  }

  SimulationExecutionIdentity querySimulationExecutionContract(
    const std::string & expected_hardware_name,
    const std::string & context,
    std::chrono::nanoseconds timeout)
  {
    if (execution_mode_ != "simulation" || !simulation_controller_manager_client_ ||
      !simulation_hardware_components_client_)
    {
      throw std::runtime_error(
              context + ": simulation identity clients are not configured");
    }
    if (timeout <= std::chrono::nanoseconds::zero()) {
      throw std::runtime_error(context + ": simulation identity query timeout is not positive");
    }
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    const auto remaining = [&deadline]() {
        const auto now = std::chrono::steady_clock::now();
        return now >= deadline ? std::chrono::nanoseconds::zero() :
               std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now);
      };
    if (!simulation_controller_manager_client_->wait_for_service(remaining())) {
      throw std::runtime_error(
              context + ": simulation controller-manager identity service is unavailable");
    }
    if (!simulation_hardware_components_client_->wait_for_service(remaining())) {
      throw std::runtime_error(
              context + ": simulation hardware identity service is unavailable");
    }
    auto controller_future = simulation_controller_manager_client_->async_send_request(
      std::make_shared<ListControllers::Request>());
    auto hardware_future = simulation_hardware_components_client_->async_send_request(
      std::make_shared<ListHardwareComponents::Request>());
    if (controller_future.wait_for(remaining()) != std::future_status::ready) {
      (void)simulation_controller_manager_client_->remove_pending_request(controller_future);
      (void)simulation_hardware_components_client_->remove_pending_request(hardware_future);
      throw std::runtime_error(
              context + ": simulation controller-manager identity query timed out");
    }
    if (hardware_future.wait_for(remaining()) != std::future_status::ready) {
      (void)simulation_hardware_components_client_->remove_pending_request(hardware_future);
      throw std::runtime_error(context + ": simulation hardware identity query timed out");
    }
    const auto controller_response = controller_future.get();
    const auto hardware_response = hardware_future.get();
    if (!controller_response || !hardware_response) {
      throw std::runtime_error(context + ": simulation identity query returned no response");
    }
    try {
      return requireSimulationExecutionIdentity(
        controller_response->controller, hardware_response->component, joint_names_,
        expected_hardware_name);
    } catch (const std::exception & error) {
      throw std::runtime_error(context + ": " + error.what());
    }
  }

  std::string execution_mode_;
  std::string environments_root_;
  std::string telemetry_root_;
  std::string robot_type_;
  std::string driver_kinematics_params_file_;
  std::vector<std::string> joint_names_;
  std::string joint_states_topic_;
  double action_server_timeout_s_;
  double speed_scaling_fresh_timeout_s_;
  std::string simulation_action_name_;
  std::string selected_action_name_;
  std::string prepare_plan_service_name_;
  double prepare_planning_timeout_s_{120.0};
  double prepare_settle_timeout_s_{5.0};
  PrepareTrajectoryValidationConfig prepare_validation_config_;

  RecordingStore store_;
  SafetyGate safety_gate_;
  std::unique_ptr<PlaybackExecutor> executor_;
  rclcpp::CallbackGroup::SharedPtr prepare_callback_group_;
  rclcpp::Client<PlanJointPath>::SharedPtr prepare_client_;
  rclcpp::Client<ListControllers>::SharedPtr simulation_controller_manager_client_;
  rclcpp::Client<ListHardwareComponents>::SharedPtr simulation_hardware_components_client_;
  rclcpp::CallbackGroup::SharedPtr simulation_contract_callback_group_;
  std::unique_ptr<TelemetryRecorder> telemetry_recorder_;
  std::unique_ptr<SimulationRunner> simulation_runner_;
  std::unique_ptr<tf2_ros::Buffer> setup_tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> setup_tf_listener_;
  rclcpp::CallbackGroup::SharedPtr joint_state_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;
  rclcpp_action::Server<PlayRecording>::SharedPtr play_server_;
  rclcpp_action::Server<BuildRecording>::SharedPtr build_server_;

  std::mutex joint_state_mutex_;
  std::condition_variable joint_state_condition_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  std::uint64_t joint_state_sequence_{0U};
  std::mutex goal_mutex_;
  bool goal_active_{false};
  std::thread worker_thread_;
};

}  // namespace hx_waypoint

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<hx_waypoint::RuntimeNode>();
    // Bound idle RMW waits so a missed DDS wakeup cannot leave services asleep
    // indefinitely after playback. Ready callbacks still run immediately; this
    // timeout does not pace trajectories, controller feedback, or telemetry.
    rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 4U, false, std::chrono::milliseconds(50));
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("hx_waypoint_runtime"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
