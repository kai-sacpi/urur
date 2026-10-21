#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <controller_manager_msgs/msg/controller_state.hpp>
#include <controller_manager_msgs/msg/hardware_component_state.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>

namespace hx_waypoint
{

inline constexpr char kRequiredSimulationActionName[] =
  "/joint_trajectory_controller/follow_joint_trajectory";
inline constexpr char kRequiredRealPassthroughActionName[] =
  "/passthrough_trajectory_controller/follow_joint_trajectory";
inline constexpr char kRequiredRealUnscaledJointTrajectoryActionName[] =
  "/joint_trajectory_controller/follow_joint_trajectory";
inline constexpr char kRequiredJointStatesTopic[] = "/joint_states";
inline constexpr char kRequiredTfTopic[] = "/tf";
inline constexpr char kRequiredTfStaticTopic[] = "/tf_static";
inline constexpr char kRequiredSimulationControllerStateTopic[] =
  "/joint_trajectory_controller/controller_state";
inline constexpr char kRequiredSimulationBaseFrame[] = "base_link";
inline constexpr char kRequiredSimulationTipFrame[] = "rod_tip";
inline const std::vector<std::string> kRequiredSimulationFrameNames{
  "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link",
  "wrist_2_link", "wrist_3_link", "tool0", "rod_tip"};
inline constexpr char kRequiredPrepareServiceName[] =
  "/hx_waypoint/curobo/plan_joint_path";
inline constexpr char kRequiredSimulationControllerManagerService[] =
  "/controller_manager/list_controllers";
inline constexpr char kRequiredSimulationHardwareComponentsService[] =
  "/controller_manager/list_hardware_components";
inline constexpr char kRequiredSimulationControllerName[] =
  "joint_trajectory_controller";
inline constexpr char kRequiredSimulationControllerType[] =
  "joint_trajectory_controller/JointTrajectoryController";
inline constexpr char kRequiredSimulationHardwareType[] = "system";
inline constexpr char kRequiredSimulationHardwarePlugin[] =
  "mock_components/GenericSystem";
inline constexpr char kRequiredSimulationSpeedScalingEvidence[] =
  "locked_standard_unscaled_joint_trajectory_controller";
inline constexpr std::uint32_t kSimulationExecutionContractRevision = 1U;

struct SimulationExecutionIdentity
{
  std::uint32_t contract_revision{kSimulationExecutionContractRevision};
  std::string controller_name;
  std::string controller_type;
  std::string controller_state;
  std::vector<std::string> claimed_interfaces;
  std::vector<std::string> required_command_interfaces;
  std::vector<std::string> required_state_interfaces;
  std::string hardware_name;
  std::string hardware_type;
  std::string hardware_plugin;
  std::uint8_t hardware_state_id{0U};
  std::string hardware_state_label;
};

[[nodiscard]] std::string selectControllerAction(
  const std::string & execution_mode,
  const std::string & simulation_action_name,
  const std::string & real_action_name,
  const std::string & explicit_action_name);

void requireSimulationControllerStateTopic(const std::string & controller_state_topic);

void requireSimulationFrameContract(
  const std::string & base_frame,
  const std::vector<std::string> & frame_names,
  const std::string & tip_frame);

void requireJointStatesTopic(const std::string & joint_states_topic);

void requirePrepareServiceName(const std::string & prepare_service_name);

void requireStandardUnscaledSimulationController(
  const std::string & name,
  const std::string & type,
  const std::string & state);

void requireStandardUnscaledSimulationController(
  const controller_manager_msgs::msg::ControllerState & controller,
  const std::vector<std::string> & expected_joint_names);

void requireStandardMockSimulationHardware(
  const std::vector<controller_manager_msgs::msg::HardwareComponentState> & components,
  const std::string & expected_hardware_name);

[[nodiscard]] SimulationExecutionIdentity requireSimulationExecutionIdentity(
  const std::vector<controller_manager_msgs::msg::ControllerState> & controllers,
  const std::vector<controller_manager_msgs::msg::HardwareComponentState> & components,
  const std::vector<std::string> & expected_joint_names,
  const std::string & expected_hardware_name);

[[nodiscard]] std::optional<SimulationExecutionIdentity>
waitForInitialSimulationExecutionIdentity(
  const std::function<SimulationExecutionIdentity(std::chrono::nanoseconds)> & query,
  const std::function<bool()> & stop_waiting,
  std::chrono::nanoseconds timeout,
  std::chrono::nanoseconds attempt_timeout,
  std::chrono::nanoseconds retry_interval,
  const std::string & context);

void requireUnremappedName(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
  const std::string & name,
  bool is_service,
  const std::string & label);

void requireUnremappedActionName(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
  const std::string & action_name,
  const std::string & label);

}  // namespace hx_waypoint
