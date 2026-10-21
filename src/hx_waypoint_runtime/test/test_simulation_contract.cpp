#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <controller_manager_msgs/msg/controller_state.hpp>
#include <controller_manager_msgs/msg/hardware_component_state.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "hx_waypoint/runtime_policy.hpp"

namespace
{

using ControllerState = controller_manager_msgs::msg::ControllerState;
using HardwareComponentState = controller_manager_msgs::msg::HardwareComponentState;

const std::vector<std::string> kJointNames{
  "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

class ScopedRclcppContext
{
public:
  ScopedRclcppContext()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
      owns_context_ = true;
    }
  }

  ~ScopedRclcppContext()
  {
    if (owns_context_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

private:
  bool owns_context_{false};
};

ControllerState validController()
{
  ControllerState controller;
  controller.name = hx_waypoint::kRequiredSimulationControllerName;
  controller.type = hx_waypoint::kRequiredSimulationControllerType;
  controller.state = "active";
  for (const auto & joint_name : kJointNames) {
    controller.claimed_interfaces.push_back(joint_name + "/position");
    controller.required_command_interfaces.push_back(joint_name + "/position");
    controller.required_state_interfaces.push_back(joint_name + "/position");
    controller.required_state_interfaces.push_back(joint_name + "/velocity");
  }
  return controller;
}

HardwareComponentState validHardware(const std::string & name = "ur5")
{
  HardwareComponentState hardware;
  hardware.name = name;
  hardware.type = hx_waypoint::kRequiredSimulationHardwareType;
  hardware.plugin_name = hx_waypoint::kRequiredSimulationHardwarePlugin;
  hardware.state.id = 3U;
  hardware.state.label = "active";
  return hardware;
}

void requireIdentity(
  const ControllerState & controller,
  const std::vector<HardwareComponentState> & hardware,
  const std::string & expected_hardware_name = "ur5")
{
  const std::vector<ControllerState> controllers{controller};
  (void)hx_waypoint::requireSimulationExecutionIdentity(
    controllers, hardware, kJointNames, expected_hardware_name);
}

TEST(SimulationContract, InterfaceSetsAreOrderIndependentAndHardwareNameTracksUrType)
{
  auto controller = validController();
  std::reverse(controller.claimed_interfaces.begin(), controller.claimed_interfaces.end());
  std::rotate(
    controller.required_command_interfaces.begin(),
    controller.required_command_interfaces.begin() + 2,
    controller.required_command_interfaces.end());
  std::reverse(
    controller.required_state_interfaces.begin(), controller.required_state_interfaces.end());
  const std::vector<ControllerState> controllers{controller};
  const std::vector<HardwareComponentState> hardware{validHardware()};
  const auto identity = hx_waypoint::requireSimulationExecutionIdentity(
    controllers, hardware, kJointNames, "ur5");
  EXPECT_TRUE(std::is_sorted(
      identity.claimed_interfaces.begin(), identity.claimed_interfaces.end()));
  EXPECT_TRUE(std::is_sorted(
      identity.required_command_interfaces.begin(),
      identity.required_command_interfaces.end()));
  EXPECT_TRUE(std::is_sorted(
      identity.required_state_interfaces.begin(), identity.required_state_interfaces.end()));

  EXPECT_NO_THROW(requireIdentity(controller, {validHardware("ur5e")}, "ur5e"));
}

TEST(SimulationContract, InterfaceSetsRejectMissingExtraDuplicateAndScaledInterfaces)
{
  auto missing = validController();
  missing.claimed_interfaces.pop_back();
  EXPECT_THROW(requireIdentity(missing, {validHardware()}), std::invalid_argument);

  auto extra = validController();
  extra.required_command_interfaces.push_back("shoulder_pan_joint/velocity");
  EXPECT_THROW(requireIdentity(extra, {validHardware()}), std::invalid_argument);

  auto duplicate = validController();
  duplicate.required_state_interfaces[1] = duplicate.required_state_interfaces[0];
  EXPECT_THROW(requireIdentity(duplicate, {validHardware()}), std::invalid_argument);

  auto scaled = validController();
  scaled.required_state_interfaces.push_back("speed_scaling/speed_scaling_factor");
  EXPECT_THROW(requireIdentity(scaled, {validHardware()}), std::invalid_argument);
}

TEST(SimulationContract, ControllerRejectsAsyncAndChainedExecutionSemantics)
{
  auto async_controller = validController();
  async_controller.is_async = true;
  EXPECT_THROW(requireIdentity(async_controller, {validHardware()}), std::invalid_argument);

  auto chainable = validController();
  chainable.is_chainable = true;
  EXPECT_THROW(requireIdentity(chainable, {validHardware()}), std::invalid_argument);

  auto chained = validController();
  chained.is_chained = true;
  EXPECT_THROW(requireIdentity(chained, {validHardware()}), std::invalid_argument);

  auto exported = validController();
  exported.exported_state_interfaces.push_back("unexpected/reference");
  EXPECT_THROW(requireIdentity(exported, {validHardware()}), std::invalid_argument);

  auto referenced = validController();
  referenced.reference_interfaces.push_back("unexpected/reference");
  EXPECT_THROW(requireIdentity(referenced, {validHardware()}), std::invalid_argument);

  auto connected = validController();
  connected.chain_connections.resize(1U);
  EXPECT_THROW(requireIdentity(connected, {validHardware()}), std::invalid_argument);
}

TEST(SimulationContract, HardwareRejectsWrongPluginInactiveAsyncNameAndExtraComponents)
{
  auto wrong_plugin = validHardware();
  wrong_plugin.plugin_name = "custom/ScaledMockSystem";
  EXPECT_THROW(requireIdentity(validController(), {wrong_plugin}), std::invalid_argument);

  auto inactive = validHardware();
  inactive.state.id = 2U;
  inactive.state.label = "inactive";
  EXPECT_THROW(requireIdentity(validController(), {inactive}), std::invalid_argument);

  auto async_hardware = validHardware();
  async_hardware.is_async = true;
  EXPECT_THROW(requireIdentity(validController(), {async_hardware}), std::invalid_argument);

  EXPECT_THROW(
    requireIdentity(validController(), {validHardware("ur5e")}, "ur5"),
    std::invalid_argument);

  const std::vector<HardwareComponentState> extra_hardware{
    validHardware(), validHardware("ur5_aux")};
  EXPECT_THROW(requireIdentity(validController(), extra_hardware), std::invalid_argument);
}

TEST(SimulationContract, HardwareIdentityRequiresPathSafeUrRobotType)
{
  EXPECT_THROW(
    requireIdentity(validController(), {validHardware("robot")}, "robot"),
    std::invalid_argument);
  EXPECT_THROW(
    requireIdentity(validController(), {validHardware("ur5/../../evil")}, "ur5/../../evil"),
    std::invalid_argument);
}

TEST(SimulationContract, InitialIdentityWaitRetriesTransientRejectionUntilValid)
{
  std::size_t attempts = 0U;
  const auto identity = hx_waypoint::waitForInitialSimulationExecutionIdentity(
    [&attempts](std::chrono::nanoseconds timeout) {
      EXPECT_GT(timeout, std::chrono::nanoseconds::zero());
      ++attempts;
      if (attempts < 3U) {
        throw std::invalid_argument("controller is not active yet");
      }
      hx_waypoint::SimulationExecutionIdentity valid;
      valid.hardware_name = "ur5";
      return valid;
    },
    []() {return false;}, std::chrono::milliseconds(100), std::chrono::milliseconds(10),
    std::chrono::milliseconds(1), "test initial identity");
  ASSERT_TRUE(identity.has_value());
  EXPECT_EQ(identity->hardware_name, "ur5");
  EXPECT_EQ(attempts, 3U);
}

TEST(SimulationContract, InitialIdentityWaitCancelsAndPersistentInvalidityTimesOut)
{
  std::size_t canceled_attempts = 0U;
  const auto canceled = hx_waypoint::waitForInitialSimulationExecutionIdentity(
    [&canceled_attempts](std::chrono::nanoseconds) -> hx_waypoint::SimulationExecutionIdentity {
      ++canceled_attempts;
      throw std::invalid_argument("temporarily invalid");
    },
    [&canceled_attempts]() {return canceled_attempts > 0U;},
    std::chrono::milliseconds(100), std::chrono::milliseconds(10),
    std::chrono::milliseconds(1), "test canceled identity");
  EXPECT_FALSE(canceled.has_value());
  EXPECT_EQ(canceled_attempts, 1U);

  std::size_t invalid_attempts = 0U;
  EXPECT_THROW(
    (void)hx_waypoint::waitForInitialSimulationExecutionIdentity(
      [&invalid_attempts](std::chrono::nanoseconds) ->
      hx_waypoint::SimulationExecutionIdentity {
        ++invalid_attempts;
        throw std::invalid_argument("still inactive");
      },
      []() {return false;}, std::chrono::milliseconds(20), std::chrono::milliseconds(5),
      std::chrono::milliseconds(1), "test persistent invalid identity"),
    std::runtime_error);
  EXPECT_GE(invalid_attempts, 1U);
}

TEST(SimulationContract, SimulationFrameParametersMustMatchTheCanonicalOrderedSet)
{
  EXPECT_NO_THROW(hx_waypoint::requireSimulationFrameContract(
      hx_waypoint::kRequiredSimulationBaseFrame,
      hx_waypoint::kRequiredSimulationFrameNames,
      hx_waypoint::kRequiredSimulationTipFrame));

  EXPECT_THROW(
    hx_waypoint::requireSimulationFrameContract(
      "world", hx_waypoint::kRequiredSimulationFrameNames,
      hx_waypoint::kRequiredSimulationTipFrame),
    std::invalid_argument);
  EXPECT_THROW(
    hx_waypoint::requireSimulationFrameContract(
      hx_waypoint::kRequiredSimulationBaseFrame,
      hx_waypoint::kRequiredSimulationFrameNames, "tool0"),
    std::invalid_argument);

  auto missing = hx_waypoint::kRequiredSimulationFrameNames;
  missing.pop_back();
  EXPECT_THROW(
    hx_waypoint::requireSimulationFrameContract(
      hx_waypoint::kRequiredSimulationBaseFrame, missing,
      hx_waypoint::kRequiredSimulationTipFrame),
    std::invalid_argument);

  auto extra = hx_waypoint::kRequiredSimulationFrameNames;
  extra.push_back("base_link");
  EXPECT_THROW(
    hx_waypoint::requireSimulationFrameContract(
      hx_waypoint::kRequiredSimulationBaseFrame, extra,
      hx_waypoint::kRequiredSimulationTipFrame),
    std::invalid_argument);

  auto duplicate = hx_waypoint::kRequiredSimulationFrameNames;
  duplicate.back() = duplicate.front();
  EXPECT_THROW(
    hx_waypoint::requireSimulationFrameContract(
      hx_waypoint::kRequiredSimulationBaseFrame, duplicate,
      hx_waypoint::kRequiredSimulationTipFrame),
    std::invalid_argument);

  auto reordered = hx_waypoint::kRequiredSimulationFrameNames;
  std::swap(reordered[0], reordered[1]);
  EXPECT_THROW(
    hx_waypoint::requireSimulationFrameContract(
      hx_waypoint::kRequiredSimulationBaseFrame, reordered,
      hx_waypoint::kRequiredSimulationTipFrame),
    std::invalid_argument);
}

TEST(SimulationContract, PrepareServiceParameterOverrideIsRejected)
{
  EXPECT_NO_THROW(hx_waypoint::requirePrepareServiceName(
      hx_waypoint::kRequiredPrepareServiceName));
  EXPECT_THROW(
    hx_waypoint::requirePrepareServiceName("/unsafe_prepare_service"),
    std::invalid_argument);
  EXPECT_THROW(
    hx_waypoint::requirePrepareServiceName("hx_waypoint/curobo/plan_joint_path"),
    std::invalid_argument);
}

TEST(SimulationContract, JointStatesParameterOverrideIsRejected)
{
  EXPECT_NO_THROW(hx_waypoint::requireJointStatesTopic(
      hx_waypoint::kRequiredJointStatesTopic));
  EXPECT_THROW(
    hx_waypoint::requireJointStatesTopic("/robot_1/joint_states"),
    std::invalid_argument);
  EXPECT_THROW(
    hx_waypoint::requireJointStatesTopic("joint_states"),
    std::invalid_argument);
}

TEST(SimulationContract, PrepareServiceMustNotBeRemapped)
{
  ScopedRclcppContext context;
  auto clean_node = std::make_shared<rclcpp::Node>("simulation_prepare_contract_clean");
  EXPECT_NO_THROW(hx_waypoint::requireUnremappedName(
      clean_node->get_node_base_interface(), hx_waypoint::kRequiredPrepareServiceName, true,
      "cuRobo prepare service"));

  rclcpp::NodeOptions remap_options;
  remap_options.arguments({
      "--ros-args", "-r",
      std::string(hx_waypoint::kRequiredPrepareServiceName) + ":=/unsafe_prepare_service"});
  auto remapped_node = std::make_shared<rclcpp::Node>(
    "simulation_prepare_contract_remapped", remap_options);
  EXPECT_THROW(
    hx_waypoint::requireUnremappedName(
      remapped_node->get_node_base_interface(), hx_waypoint::kRequiredPrepareServiceName, true,
      "cuRobo prepare service"),
    std::invalid_argument);
}

TEST(SimulationContract, JointStateAndTfSafetyInputsMustNotBeRemapped)
{
  ScopedRclcppContext context;
  auto clean_node = std::make_shared<rclcpp::Node>("simulation_input_contract_clean");
  EXPECT_NO_THROW(hx_waypoint::requireUnremappedName(
      clean_node->get_node_base_interface(), hx_waypoint::kRequiredJointStatesTopic, false,
      "authoritative JointState input"));
  EXPECT_NO_THROW(hx_waypoint::requireUnremappedName(
      clean_node->get_node_base_interface(), hx_waypoint::kRequiredTfTopic, false,
      "simulation dynamic TF input"));
  EXPECT_NO_THROW(hx_waypoint::requireUnremappedName(
      clean_node->get_node_base_interface(), hx_waypoint::kRequiredTfStaticTopic, false,
      "simulation static TF input"));

  const auto expect_remap_rejected = [](
      const std::string & node_name, const std::string & endpoint, const std::string & replacement) {
      rclcpp::NodeOptions options;
      options.arguments({"--ros-args", "-r", endpoint + ":=" + replacement});
      auto node = std::make_shared<rclcpp::Node>(node_name, options);
      EXPECT_THROW(
        hx_waypoint::requireUnremappedName(
          node->get_node_base_interface(), endpoint, false, "simulation safety input"),
        std::invalid_argument);
    };
  expect_remap_rejected(
    "simulation_joint_states_contract_remapped", hx_waypoint::kRequiredJointStatesTopic,
    "/unsafe_joint_states");
  expect_remap_rejected(
    "simulation_tf_contract_remapped", hx_waypoint::kRequiredTfTopic, "/unsafe_tf");
  expect_remap_rejected(
    "simulation_tf_static_contract_remapped", hx_waypoint::kRequiredTfStaticTopic,
    "/unsafe_tf_static");
}

TEST(SimulationContract, HardwareIdentityServiceMustNotBeRemapped)
{
  ScopedRclcppContext context;
  auto clean_node = std::make_shared<rclcpp::Node>("simulation_hardware_contract_clean");
  EXPECT_NO_THROW(hx_waypoint::requireUnremappedName(
      clean_node->get_node_base_interface(),
      hx_waypoint::kRequiredSimulationHardwareComponentsService, true,
      "simulation hardware identity service"));

  rclcpp::NodeOptions remap_options;
  remap_options.arguments({
      "--ros-args", "-r",
      std::string(hx_waypoint::kRequiredSimulationHardwareComponentsService) +
      ":=/unsafe_list_hardware_components"});
  auto remapped_node = std::make_shared<rclcpp::Node>(
    "simulation_hardware_contract_remapped", remap_options);
  EXPECT_THROW(
    hx_waypoint::requireUnremappedName(
      remapped_node->get_node_base_interface(),
      hx_waypoint::kRequiredSimulationHardwareComponentsService, true,
      "simulation hardware identity service"),
    std::invalid_argument);
}

}  // namespace
