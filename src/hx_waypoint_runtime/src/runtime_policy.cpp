#include "hx_waypoint/runtime_policy.hpp"

#include <algorithm>
#include <cctype>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace hx_waypoint
{
namespace
{

void requireExactName(
  const std::string & configured,
  const std::string & required,
  const std::string & label)
{
  if (configured != required) {
    throw std::invalid_argument(label + " requires the exact endpoint " + required);
  }
}

void requireUniqueValues(
  const std::vector<std::string> & values,
  const std::string & label)
{
  std::set<std::string> unique;
  for (const auto & value : values) {
    if (value.empty()) {
      throw std::invalid_argument(label + " must not contain an empty entry");
    }
    if (!unique.insert(value).second) {
      throw std::invalid_argument(label + " contains duplicate entry '" + value + "'");
    }
  }
}

void requireExactInterfaceSet(
  const std::vector<std::string> & actual,
  const std::vector<std::string> & expected,
  const std::string & label)
{
  requireUniqueValues(actual, label);
  const std::set<std::string> actual_set(actual.begin(), actual.end());
  const std::set<std::string> expected_set(expected.begin(), expected.end());
  if (actual_set != expected_set) {
    throw std::invalid_argument(
            label + " must exactly match the required unique interface set");
  }
}

std::vector<std::string> expectedInterfaces(
  const std::vector<std::string> & joint_names,
  const std::vector<std::string> & interface_names)
{
  if (joint_names.empty()) {
    throw std::invalid_argument("simulation controller joint set must not be empty");
  }
  requireUniqueValues(joint_names, "simulation controller joint set");
  std::vector<std::string> expected;
  expected.reserve(joint_names.size() * interface_names.size());
  for (const auto & joint_name : joint_names) {
    for (const auto & interface_name : interface_names) {
      expected.push_back(joint_name + "/" + interface_name);
    }
  }
  return expected;
}

std::vector<std::string> sorted(std::vector<std::string> values)
{
  std::sort(values.begin(), values.end());
  return values;
}

void requirePathSafeUrHardwareName(const std::string & name)
{
  if (name.size() < 3U || name.rfind("ur", 0U) != 0U || !std::all_of(
      name.begin(), name.end(), [](unsigned char character) {
        return std::islower(character) != 0 || std::isdigit(character) != 0 ||
               character == '_' || character == '-';
      }))
  {
    throw std::invalid_argument(
            "simulation hardware identity must be a path-safe lowercase UR robot type");
  }
}

}  // namespace

std::string selectControllerAction(
  const std::string & execution_mode,
  const std::string & simulation_action_name,
  const std::string & real_action_name,
  const std::string & explicit_action_name)
{
  if (execution_mode != "simulation" && execution_mode != "real") {
    throw std::invalid_argument("execution_mode must be 'simulation' or 'real'");
  }
  requireExactName(
    simulation_action_name, kRequiredSimulationActionName, "simulation execution");
  if (real_action_name.empty()) {
    throw std::invalid_argument("real controller action name must not be empty");
  }
  if (execution_mode == "real") {
    if (!explicit_action_name.empty() && explicit_action_name != real_action_name) {
      throw std::invalid_argument(
              "real execution forbids action_name overrides that differ from real_action_name");
    }
    if (real_action_name != kRequiredRealPassthroughActionName &&
      real_action_name != kRequiredRealUnscaledJointTrajectoryActionName)
    {
      throw std::invalid_argument(
              "real execution requires an approved exact unscaled action endpoint");
    }
    return real_action_name;
  }

  if (!explicit_action_name.empty() && explicit_action_name != kRequiredSimulationActionName) {
    throw std::invalid_argument(
            "simulation execution forbids action_name overrides that differ from the required "
            "unscaled controller action " + std::string(kRequiredSimulationActionName));
  }
  return kRequiredSimulationActionName;
}

void requireSimulationControllerStateTopic(const std::string & controller_state_topic)
{
  requireExactName(
    controller_state_topic, kRequiredSimulationControllerStateTopic,
    "simulation recording");
}

void requireSimulationFrameContract(
  const std::string & base_frame,
  const std::vector<std::string> & frame_names,
  const std::string & tip_frame)
{
  requireExactName(base_frame, kRequiredSimulationBaseFrame, "simulation recording base frame");
  requireExactName(tip_frame, kRequiredSimulationTipFrame, "simulation recording tip frame");
  if (frame_names != kRequiredSimulationFrameNames) {
    throw std::invalid_argument(
            "simulation recording frame_names must exactly match the required ordered frame set");
  }
}

void requireJointStatesTopic(const std::string & joint_states_topic)
{
  requireExactName(joint_states_topic, kRequiredJointStatesTopic, "authoritative JointState input");
}

void requirePrepareServiceName(const std::string & prepare_service_name)
{
  requireExactName(
    prepare_service_name, kRequiredPrepareServiceName, "cuRobo prepare planning");
}

void requireStandardUnscaledSimulationController(
  const std::string & name,
  const std::string & type,
  const std::string & state)
{
  requireExactName(name, kRequiredSimulationControllerName, "simulation controller identity");
  requireExactName(type, kRequiredSimulationControllerType, "simulation controller plugin");
  if (state != "active") {
    throw std::invalid_argument(
            "simulation controller must be active, got state '" + state + "'");
  }
}

void requireStandardUnscaledSimulationController(
  const controller_manager_msgs::msg::ControllerState & controller,
  const std::vector<std::string> & expected_joint_names)
{
  requireStandardUnscaledSimulationController(
    controller.name, controller.type, controller.state);
  if (controller.is_async) {
    throw std::invalid_argument("simulation controller must execute synchronously");
  }
  if (controller.is_chainable || controller.is_chained ||
    !controller.exported_state_interfaces.empty() ||
    !controller.reference_interfaces.empty() || !controller.chain_connections.empty())
  {
    throw std::invalid_argument(
            "simulation controller must not use chained-controller semantics");
  }
  const auto expected_command_interfaces = expectedInterfaces(
    expected_joint_names, {"position"});
  const auto expected_state_interfaces = expectedInterfaces(
    expected_joint_names, {"position", "velocity"});
  requireExactInterfaceSet(
    controller.claimed_interfaces, expected_command_interfaces,
    "simulation controller claimed_interfaces");
  requireExactInterfaceSet(
    controller.required_command_interfaces, expected_command_interfaces,
    "simulation controller required_command_interfaces");
  requireExactInterfaceSet(
    controller.required_state_interfaces, expected_state_interfaces,
    "simulation controller required_state_interfaces");
}

void requireStandardMockSimulationHardware(
  const std::vector<controller_manager_msgs::msg::HardwareComponentState> & components,
  const std::string & expected_hardware_name)
{
  requirePathSafeUrHardwareName(expected_hardware_name);
  if (components.size() != 1U) {
    throw std::invalid_argument(
            "simulation requires exactly one controller-manager hardware component");
  }
  const auto & component = components.front();
  requireExactName(
    component.name, expected_hardware_name, "simulation hardware identity");
  requireExactName(
    component.type, kRequiredSimulationHardwareType, "simulation hardware component type");
  requireExactName(
    component.plugin_name, kRequiredSimulationHardwarePlugin, "simulation hardware plugin");
  if (component.is_async) {
    throw std::invalid_argument("simulation hardware must execute synchronously");
  }
  constexpr std::uint8_t kLifecycleStateActive = 3U;
  if (component.state.id != kLifecycleStateActive || component.state.label != "active") {
    throw std::invalid_argument(
            "simulation hardware must be active, got state id=" +
            std::to_string(component.state.id) + " label='" + component.state.label + "'");
  }
}

SimulationExecutionIdentity requireSimulationExecutionIdentity(
  const std::vector<controller_manager_msgs::msg::ControllerState> & controllers,
  const std::vector<controller_manager_msgs::msg::HardwareComponentState> & components,
  const std::vector<std::string> & expected_joint_names,
  const std::string & expected_hardware_name)
{
  const controller_manager_msgs::msg::ControllerState * required_controller = nullptr;
  for (const auto & controller : controllers) {
    if (controller.name != kRequiredSimulationControllerName) {
      continue;
    }
    if (required_controller != nullptr) {
      throw std::invalid_argument(
              "simulation controller-manager returned a duplicate required controller");
    }
    required_controller = &controller;
  }
  if (required_controller == nullptr) {
    throw std::invalid_argument(
            "simulation controller-manager did not expose the required standard unscaled JTC");
  }

  requireStandardUnscaledSimulationController(*required_controller, expected_joint_names);
  requireStandardMockSimulationHardware(components, expected_hardware_name);
  const auto & hardware = components.front();

  SimulationExecutionIdentity identity;
  identity.controller_name = required_controller->name;
  identity.controller_type = required_controller->type;
  identity.controller_state = required_controller->state;
  identity.claimed_interfaces = sorted(required_controller->claimed_interfaces);
  identity.required_command_interfaces = sorted(
    required_controller->required_command_interfaces);
  identity.required_state_interfaces = sorted(required_controller->required_state_interfaces);
  identity.hardware_name = hardware.name;
  identity.hardware_type = hardware.type;
  identity.hardware_plugin = hardware.plugin_name;
  identity.hardware_state_id = hardware.state.id;
  identity.hardware_state_label = hardware.state.label;
  return identity;
}

std::optional<SimulationExecutionIdentity> waitForInitialSimulationExecutionIdentity(
  const std::function<SimulationExecutionIdentity(std::chrono::nanoseconds)> & query,
  const std::function<bool()> & stop_waiting,
  std::chrono::nanoseconds timeout,
  std::chrono::nanoseconds attempt_timeout,
  std::chrono::nanoseconds retry_interval,
  const std::string & context)
{
  if (!query || timeout <= std::chrono::nanoseconds::zero() ||
    attempt_timeout <= std::chrono::nanoseconds::zero() ||
    retry_interval <= std::chrono::nanoseconds::zero())
  {
    throw std::invalid_argument(
            "initial simulation identity wait requires a query and positive timeouts");
  }
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::string last_error = "no simulation identity response was received";
  while (true) {
    if (stop_waiting && stop_waiting()) {
      return std::nullopt;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      break;
    }
    const auto remaining = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now);
    try {
      return query(std::min(attempt_timeout, remaining));
    } catch (const std::exception & error) {
      last_error = error.what();
    }
    if (stop_waiting && stop_waiting()) {
      return std::nullopt;
    }
    const auto after_query = std::chrono::steady_clock::now();
    if (after_query >= deadline) {
      break;
    }
    std::this_thread::sleep_for(std::min(
        retry_interval,
        std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - after_query)));
  }
  throw std::runtime_error(
          context + " timed out waiting for a valid simulation execution identity; last error: " +
          last_error);
}

void requireUnremappedName(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
  const std::string & name,
  bool is_service,
  const std::string & label)
{
  if (!node_base) {
    throw std::invalid_argument("cannot verify ROS remapping without a node base interface");
  }
  const std::string resolved = node_base->resolve_topic_or_service_name(name, is_service);
  if (resolved != name) {
    throw std::invalid_argument(
            label + " must not be remapped: expected '" + name + "', resolved '" + resolved + "'");
  }
}

void requireUnremappedActionName(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
  const std::string & action_name,
  const std::string & label)
{
  requireUnremappedName(node_base, action_name, true, label + " endpoint");
  requireUnremappedName(
    node_base, action_name + "/_action/send_goal", true, label + " send_goal endpoint");
  requireUnremappedName(
    node_base, action_name + "/_action/get_result", true, label + " get_result endpoint");
  requireUnremappedName(
    node_base, action_name + "/_action/cancel_goal", true, label + " cancel_goal endpoint");
  requireUnremappedName(
    node_base, action_name + "/_action/feedback", false, label + " feedback endpoint");
  requireUnremappedName(
    node_base, action_name + "/_action/status", false, label + " status endpoint");
}

}  // namespace hx_waypoint
