#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "hx_waypoint/recording_store.hpp"
#include "hx_waypoint/canonical_trajectory_validator.hpp"
#include "hx_waypoint/playback_executor.hpp"
#include "hx_waypoint/prepare_workflow.hpp"
#include "hx_waypoint/runtime_policy.hpp"
#include "hx_waypoint/robot_validation_model.hpp"
#include "hx_waypoint/safety_gate.hpp"
#include "hx_waypoint/simulation_recording_validator.hpp"
#include "hx_waypoint/telemetry_recorder.hpp"
#include "hx_waypoint/tip_kinematics.hpp"

namespace
{

constexpr const char * kCuroboCommit = "a35a708ecfbb26eb9ab2d7ef22c65919c4fae4a9";
constexpr const char * kCompilerContractRevision = "fixed_time_waypoint_v2";

const std::vector<std::string> & environmentFiles()
{
  static const std::vector<std::string> files{
    "environment.yaml", "setup.yaml", "zones.yaml", "planner.yaml",
    "recording.yaml", "controllers.yaml", "robot/robot_model.yaml",
    "robot/calibrated_kinematics.yaml", "robot/asset_provenance.yaml",
    "robot/driver_kinematics.yaml", "robot/curobo_robot.yaml",
    "robot/collision_spheres.yaml"};
  return files;
}

class TemporaryTree
{
public:
  explicit TemporaryTree(const std::string & prefix)
  : path_(std::filesystem::temp_directory_path() /
      (prefix + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())))
  {
    std::filesystem::create_directories(path_);
  }

  ~TemporaryTree()
  {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  TemporaryTree(const TemporaryTree &) = delete;
  TemporaryTree & operator=(const TemporaryTree &) = delete;

  [[nodiscard]] const std::filesystem::path & path() const noexcept {return path_;}

private:
  std::filesystem::path path_;
};

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

void writeText(const std::filesystem::path & path, const std::string & contents)
{
  std::filesystem::create_directories(path.parent_path());
  std::ofstream output(path, std::ios::binary);
  if (!output) {
    throw std::runtime_error("failed to create test file: " + path.string());
  }
  output << contents;
  if (!output) {
    throw std::runtime_error("failed to write test file: " + path.string());
  }
}

std::string readText(const std::filesystem::path & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("failed to read test file: " + path.string());
  }
  return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

std::string sampleJointTrajectoryCsv()
{
  return
    "time_s,joint_a_pos,joint_b_pos,joint_a_vel,joint_b_vel,joint_a_acc,joint_b_acc,phase\n"
    "0,0,1,0,0,0,0,draw\n"
    "1,0.5,1.5,0,0,0,0,draw\n";
}

struct TestPose
{
  std::array<double, 3> position_m{};
  std::array<double, 4> quaternion_xyzw{{0.0, 0.0, 0.0, 1.0}};
};

TestPose planarFramePose(const std::string & frame_id, double joint_a, double joint_b)
{
  TestPose pose;
  double first_length = 0.0;
  double second_length = 0.0;
  double yaw = joint_a;
  if (frame_id == "shoulder_link") {
    first_length = 0.0;
  } else if (frame_id == "upper_arm_link") {
    first_length = 1.0;
  } else {
    first_length = 2.0;
    yaw = joint_a + joint_b;
    if (frame_id == "wrist_1_link") {
      second_length = 0.1;
    } else if (frame_id == "wrist_2_link") {
      second_length = 0.2;
    } else if (frame_id == "wrist_3_link") {
      second_length = 0.3;
    } else if (frame_id == "tool0") {
      second_length = 0.4;
    } else if (frame_id == "rod_tip") {
      second_length = 0.6;
    } else if (frame_id != "forearm_link") {
      throw std::runtime_error("unknown test frame: " + frame_id);
    }
  }
  pose.position_m = {{
      first_length * std::cos(joint_a) + second_length * std::cos(joint_a + joint_b),
      first_length * std::sin(joint_a) + second_length * std::sin(joint_a + joint_b),
      0.0}};
  pose.quaternion_xyzw = {{0.0, 0.0, std::sin(0.5 * yaw), std::cos(0.5 * yaw)}};
  return pose;
}

std::string sampleSourceTrajectoryCsv()
{
  const auto start = planarFramePose("rod_tip", 0.0, 1.0);
  const auto finish = planarFramePose("rod_tip", 0.5, 1.5);
  std::ostringstream output;
  output << std::setprecision(17) <<
    "time_s,frame_id,tip_x_m,tip_y_m,tip_z_m,tip_qx,tip_qy,tip_qz,tip_qw,phase\n" <<
    "0,base_link," << start.position_m[0] << ',' << start.position_m[1] << ',' <<
    start.position_m[2] << ',' << start.quaternion_xyzw[0] << ',' <<
    start.quaternion_xyzw[1] << ',' << start.quaternion_xyzw[2] << ',' <<
    start.quaternion_xyzw[3] << ",draw\n" <<
    "1,base_link," << finish.position_m[0] << ',' << finish.position_m[1] << ',' <<
    finish.position_m[2] << ',' << finish.quaternion_xyzw[0] << ',' <<
    finish.quaternion_xyzw[1] << ',' << finish.quaternion_xyzw[2] << ',' <<
    finish.quaternion_xyzw[3] << ",draw\n";
  return output.str();
}

hx_waypoint::SimulationRecordingPaths writeSimulationSamples(
  const std::filesystem::path & root,
  const std::vector<double> & times,
  double position_error,
  double velocity_error,
  double speed_scaling = 1.0,
  double reference_position_shift = 0.0,
  double reference_velocity_shift = 0.0)
{
  hx_waypoint::SimulationRecordingPaths paths{
    root / "sim_actual_joint.csv",
    root / "sim_controller_state.csv",
    root / "joint_frames.csv",
    root / "sim_actual_tip.csv"};
  std::ostringstream actual;
  actual << "ros_time_s,trajectory_time_s,joint_a_pos,joint_b_pos,joint_a_vel,joint_b_vel\n";
  for (const double time : times) {
    actual << 100.0 + time << ',' << time << ",1,2,0.5,0.6\n";
  }
  writeText(paths.actual_joint, actual.str());

  std::ostringstream controller;
  controller <<
    "ros_time_s,trajectory_time_s,speed_scaling,"
    "reference_joint_a_pos,reference_joint_b_pos,"
    "reference_joint_a_vel,reference_joint_b_vel,"
    "feedback_joint_a_pos,feedback_joint_b_pos,"
    "feedback_joint_a_vel,feedback_joint_b_vel,"
    "error_joint_a_pos,error_joint_b_pos,error_joint_a_vel,error_joint_b_vel\n";
  for (const double time : times) {
    const double reference_a_position = 1.0 + reference_position_shift;
    const double reference_b_position = 2.0 + reference_position_shift;
    const double reference_a_velocity = 0.5 + reference_velocity_shift;
    const double reference_b_velocity = 0.6 + reference_velocity_shift;
    controller << 100.0 + time << ',' << time << ',' << speed_scaling << ',' <<
      reference_a_position << ',' << reference_b_position << ',' <<
      reference_a_velocity << ',' << reference_b_velocity << ',' <<
      reference_a_position - position_error << ',' <<
      reference_b_position - position_error << ',' <<
      reference_a_velocity - velocity_error << ',' <<
      reference_b_velocity - velocity_error << ',' <<
      position_error << ',' << position_error << ',' << velocity_error << ',' <<
      velocity_error << '\n';
  }
  writeText(paths.controller_state, controller.str());

  std::ostringstream frames;
  frames << "ros_time_s,trajectory_time_s,frame_id,x_m,y_m,z_m,qx,qy,qz,qw\n";
  std::ostringstream tip;
  tip << "ros_time_s,trajectory_time_s,x_m,y_m,z_m,qx,qy,qz,qw\n";
  for (const double time : times) {
    frames << 100.0 + time << ',' << time << ",link_a," << time << ",0,0,0,0,0,1\n";
    frames << 100.0 + time << ',' << time << ",rod_tip," << time << ",0,1,0,0,0,1\n";
    tip << 100.0 + time << ',' << time << ',' << time << ",0,1,0,0,0,1\n";
  }
  writeText(paths.joint_frames, frames.str());
  writeText(paths.actual_tip, tip.str());
  return paths;
}

hx_waypoint::SimulationValidationConfig simulationValidationConfig()
{
  hx_waypoint::SimulationValidationConfig config;
  config.sample_dt_s = 0.01;
  config.max_missing_samples = 0U;
  config.max_time_jitter_s = 0.002;
  config.max_quaternion_norm_error = 0.001;
  config.require_action_success = true;
  config.max_completion_overrun_s = 0.05;
  config.max_controller_reference_position_error_rad = 0.02;
  config.max_controller_reference_velocity_error_rad_s = 0.10;
  config.max_joint_position_tracking_error_rad = 0.02;
  config.max_joint_velocity_tracking_error_rad_s = 0.10;
  config.max_actual_controller_position_error_rad = 0.02;
  config.max_actual_controller_velocity_error_rad_s = 0.10;
  config.max_frame_fk_position_error_m = 0.001;
  config.max_frame_fk_orientation_error_deg = 0.1;
  return config;
}

std::string simulationRecordingConfigYaml()
{
  const auto config = simulationValidationConfig();
  std::ostringstream output;
  output << std::setprecision(17) <<
    "schema_version: 1\n"
    "recording:\n"
    "  sample_dt_s: " << config.sample_dt_s << '\n' <<
    "  record_controller_actual: true\n"
    "  record_joint_states: true\n"
    "  record_joint_frames: true\n"
    "  record_tip_state: true\n"
    "  time_source: trajectory_time\n"
    "  canonical_source: controller_action_goal\n"
    "  output_fields:\n"
    "    positions: true\n"
    "    velocities: true\n"
    "    accelerations: true\n"
    "  first_point:\n"
    "    time_s: 0.0\n"
    "    require_zero_velocity: true\n"
    "    require_zero_acceleration: true\n"
    "  last_point:\n"
    "    require_zero_velocity: true\n"
    "    require_zero_acceleration: true\n"
    "  validation:\n"
    "    max_missing_samples: " << config.max_missing_samples << '\n' <<
    "    max_time_jitter_s: " << config.max_time_jitter_s << '\n' <<
    "    max_quaternion_norm_error: " << config.max_quaternion_norm_error << '\n' <<
    "    require_action_success: true\n"
    "    max_completion_overrun_s: " << config.max_completion_overrun_s << '\n' <<
    "    max_controller_reference_position_error_rad: " <<
    config.max_controller_reference_position_error_rad << '\n' <<
    "    max_controller_reference_velocity_error_rad_s: " <<
    config.max_controller_reference_velocity_error_rad_s << '\n' <<
    "    max_joint_position_tracking_error_rad: " <<
    config.max_joint_position_tracking_error_rad << '\n' <<
    "    max_joint_velocity_tracking_error_rad_s: " <<
    config.max_joint_velocity_tracking_error_rad_s << '\n' <<
    "    max_actual_controller_position_error_rad: " <<
    config.max_actual_controller_position_error_rad << '\n' <<
    "    max_actual_controller_velocity_error_rad_s: " <<
    config.max_actual_controller_velocity_error_rad_s << '\n' <<
    "    max_frame_fk_position_error_m: " <<
    config.max_frame_fk_position_error_m << '\n' <<
    "    max_frame_fk_orientation_error_deg: " <<
    config.max_frame_fk_orientation_error_deg << '\n';
  return output.str();
}

trajectory_msgs::msg::JointTrajectory validationCommand(const std::vector<double> & times)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint_a", "joint_b"};
  for (const double time_s : times) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {1.0, 2.0};
    point.velocities = {0.5, 0.6};
    point.accelerations = {0.0, 0.0};
    const auto nanoseconds = static_cast<std::int64_t>(std::llround(time_s * 1.0e9));
    point.time_from_start.sec = static_cast<std::int32_t>(nanoseconds / 1'000'000'000LL);
    point.time_from_start.nanosec = static_cast<std::uint32_t>(
      nanoseconds % 1'000'000'000LL);
    trajectory.points.push_back(std::move(point));
  }
  return trajectory;
}

trajectory_msgs::msg::JointTrajectory sampleTrajectory()
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint_a", "joint_b"};
  trajectory_msgs::msg::JointTrajectoryPoint first;
  first.positions = {1.0, -1.0};
  first.velocities = {0.0, 0.0};
  first.accelerations = {0.0, 0.0};
  trajectory.points.push_back(first);
  trajectory_msgs::msg::JointTrajectoryPoint second = first;
  second.positions = {1.1, -0.9};
  second.time_from_start.sec = 1;
  trajectory.points.push_back(second);
  return trajectory;
}

sensor_msgs::msg::JointState sampleJointState()
{
  sensor_msgs::msg::JointState state;
  state.name = {"joint_b", "joint_a"};
  state.position = {-1.005, 1.005};
  state.velocity = {0.001, -0.001};
  return state;
}

TEST(TelemetrySampler, UsesOneQuinticForDesiredPositionAndVelocity)
{
  const auto desired = hx_waypoint::sampleDesiredJointState(sampleTrajectory(), 0.5);
  ASSERT_EQ(desired.positions.size(), 2U);
  ASSERT_EQ(desired.velocities.size(), 2U);
  EXPECT_NEAR(desired.positions[0], 1.05, 1.0e-12);
  EXPECT_NEAR(desired.positions[1], -0.95, 1.0e-12);
  EXPECT_NEAR(desired.velocities[0], 0.1875, 1.0e-12);
  EXPECT_NEAR(desired.velocities[1], 0.1875, 1.0e-12);
}

TEST(TelemetrySpeedScaling, RequiresFreshUnityAndLatchesEveryDeviation)
{
  ScopedRclcppContext context;
  auto node = std::make_shared<rclcpp::Node>("strict_speed_scaling_test");
  hx_waypoint::TelemetryRecorderOptions options;
  options.joint_states_topic = "/strict_speed_scaling_test/joint_states";
  options.telemetry_topic = "/strict_speed_scaling_test/telemetry";
  options.speed_scaling_topic.clear();
  hx_waypoint::TelemetryRecorder recorder(node.get(), {"joint_a"}, options);

  std::string error;
  EXPECT_FALSE(recorder.armStrictSpeedScalingGuard(
      std::chrono::milliseconds(2), 1.0e-9, error));
  EXPECT_NE(error.find("no fresh speed_scaling sample"), std::string::npos);

  std::thread non_unit_sample([&recorder]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      recorder.setSpeedScaling(0.99);
    });
  const bool accepted_non_unit = recorder.armStrictSpeedScalingGuard(
    std::chrono::milliseconds(200), 1.0e-9, error);
  non_unit_sample.join();
  EXPECT_FALSE(accepted_non_unit);
  EXPECT_NE(error.find("received"), std::string::npos);

  std::thread unit_sample([&recorder]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      recorder.setSpeedScaling(1.0);
    });
  const bool accepted_unit = recorder.armStrictSpeedScalingGuard(
    std::chrono::milliseconds(200), 1.0e-9, error);
  unit_sample.join();
  ASSERT_TRUE(accepted_unit) << error;
  recorder.setSpeedScaling(0.999);
  EXPECT_FALSE(recorder.activateStrictSpeedScalingGuard(
      [](const std::string &) {}, error));
  EXPECT_NE(error.find("must be canceled"), std::string::npos);

  std::thread second_unit_sample([&recorder]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      recorder.setSpeedScaling(1.0);
    });
  const bool accepted_second_unit = recorder.armStrictSpeedScalingGuard(
    std::chrono::milliseconds(200), 1.0e-9, error);
  second_unit_sample.join();
  ASSERT_TRUE(accepted_second_unit) << error;
  std::atomic_int violation_count{0};
  std::string violation_message;
  ASSERT_TRUE(recorder.activateStrictSpeedScalingGuard(
      [&violation_count, &violation_message](const std::string & violation) {
        violation_message = violation;
        ++violation_count;
      },
      error)) << error;
  recorder.setSpeedScaling(1.0 + 0.5e-9);
  EXPECT_EQ(violation_count.load(), 0);
  recorder.setSpeedScaling(1.0 + 2.0e-9);
  EXPECT_EQ(violation_count.load(), 1);
  EXPECT_NE(violation_message.find("controller-side scaling and retiming are forbidden"),
    std::string::npos);
  recorder.setSpeedScaling(0.5);
  EXPECT_EQ(violation_count.load(), 1);
  std::string latched_violation;
  EXPECT_FALSE(recorder.finalizeStrictSpeedScalingGuard(latched_violation));
  EXPECT_NE(latched_violation.find("must remain 1.0"), std::string::npos);

  std::thread third_unit_sample([&recorder]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      recorder.setSpeedScaling(1.0);
    });
  const bool accepted_third_unit = recorder.armStrictSpeedScalingGuard(
    std::chrono::milliseconds(200), 1.0e-9, error);
  third_unit_sample.join();
  ASSERT_TRUE(accepted_third_unit) << error;
  std::atomic_bool violation_callback_entered{false};
  std::atomic_bool release_violation_callback{false};
  ASSERT_TRUE(recorder.activateStrictSpeedScalingGuard(
      [&violation_callback_entered, &release_violation_callback](const std::string &) {
        violation_callback_entered.store(true);
        while (!release_violation_callback.load()) {
          std::this_thread::yield();
        }
      },
      error)) << error;
  std::thread racing_violation([&recorder]() {recorder.setSpeedScaling(0.75);});
  const auto callback_deadline = std::chrono::steady_clock::now() +
    std::chrono::seconds(1);
  while (!violation_callback_entered.load() &&
    std::chrono::steady_clock::now() < callback_deadline)
  {
    std::this_thread::yield();
  }
  if (violation_callback_entered.load()) {
    latched_violation.clear();
    EXPECT_FALSE(recorder.finalizeStrictSpeedScalingGuard(latched_violation));
    EXPECT_NE(latched_violation.find("received 0.75"), std::string::npos);
  }
  release_violation_callback.store(true);
  racing_violation.join();
  ASSERT_TRUE(violation_callback_entered.load());
  recorder.clearStrictSpeedScalingGuard();
}

TEST(TelemetrySpeedScaling, PreSubmitCheckRejectsLatchedDeviationEvenAfterUnitRecovery)
{
  ScopedRclcppContext context;
  auto node = std::make_shared<rclcpp::Node>("speed_scaling_presubmit_latch_test");
  hx_waypoint::TelemetryRecorderOptions options;
  options.joint_states_topic = "/speed_scaling_presubmit_latch_test/joint_states";
  options.telemetry_topic = "/speed_scaling_presubmit_latch_test/telemetry";
  options.speed_scaling_topic.clear();
  hx_waypoint::TelemetryRecorder recorder(node.get(), {"joint_a"}, options);
  std::string error;
  EXPECT_FALSE(recorder.checkArmedStrictSpeedScalingGuard(error));
  std::thread initial_sample([&recorder]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      recorder.setSpeedScaling(1.0);
    });
  const bool armed = recorder.armStrictSpeedScalingGuard(
    std::chrono::milliseconds(200), 1.0e-9, error);
  initial_sample.join();
  ASSERT_TRUE(armed) << error;
  EXPECT_TRUE(recorder.checkArmedStrictSpeedScalingGuard(error)) << error;
  recorder.setSpeedScaling(0.5);
  recorder.setSpeedScaling(1.0);
  EXPECT_FALSE(recorder.checkArmedStrictSpeedScalingGuard(error));
  EXPECT_NE(error.find("received 0.5"), std::string::npos);
  const auto original_violation = error;
  EXPECT_FALSE(recorder.checkArmedStrictSpeedScalingGuard(error));
  EXPECT_EQ(error, original_violation);
  // The read-only pre-submit check did not clear the accepted-callback proof.
  EXPECT_FALSE(recorder.activateStrictSpeedScalingGuard([](const std::string &) {}, error));
  EXPECT_EQ(error, original_violation);
}

TEST(TelemetrySpeedScaling, PreSubmitCheckRejectsSilentSpeedStreamBeforeAnyGoal)
{
  ScopedRclcppContext context;
  auto node = std::make_shared<rclcpp::Node>("speed_scaling_presubmit_silence_test");
  hx_waypoint::TelemetryRecorderOptions options;
  options.joint_states_topic = "/speed_scaling_presubmit_silence_test/joint_states";
  options.telemetry_topic = "/speed_scaling_presubmit_silence_test/telemetry";
  options.speed_scaling_topic.clear();
  hx_waypoint::TelemetryRecorder recorder(node.get(), {"joint_a"}, options);
  std::string error;
  std::thread initial_sample([&recorder]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      recorder.setSpeedScaling(1.0);
    });
  const bool armed = recorder.armStrictSpeedScalingGuard(
    std::chrono::milliseconds(40), 1.0e-9, error);
  initial_sample.join();
  ASSERT_TRUE(armed) << error;
  EXPECT_TRUE(recorder.checkArmedStrictSpeedScalingGuard(error)) << error;
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_FALSE(recorder.checkArmedStrictSpeedScalingGuard(error));
  EXPECT_NE(error.find("stopped publishing fresh samples"), std::string::npos);
  // No re-arm, activation, callback, or trajectory submission was necessary.
  EXPECT_FALSE(recorder.checkArmedStrictSpeedScalingGuard(error));
}

TEST(TelemetrySpeedScaling, ConvertsUrBroadcasterPercentWithoutClamping)
{
  EXPECT_DOUBLE_EQ(hx_waypoint::urSpeedScalingPercentToRatio(100.0), 1.0);
  EXPECT_DOUBLE_EQ(hx_waypoint::urSpeedScalingPercentToRatio(99.0), 0.99);
  EXPECT_GT(hx_waypoint::urSpeedScalingPercentToRatio(100.01), 1.0);
  EXPECT_LT(hx_waypoint::urSpeedScalingPercentToRatio(-1.0), 0.0);
  EXPECT_TRUE(std::isnan(hx_waypoint::urSpeedScalingPercentToRatio(
      std::numeric_limits<double>::quiet_NaN())));
}

TEST(TelemetrySpeedScaling, CancelsWhenFreshSamplesStopDuringExecution)
{
  ScopedRclcppContext context;
  auto node = std::make_shared<rclcpp::Node>("speed_scaling_freshness_watchdog_test");
  hx_waypoint::TelemetryRecorderOptions options;
  options.joint_states_topic = "/speed_scaling_freshness_watchdog_test/joint_states";
  options.telemetry_topic = "/speed_scaling_freshness_watchdog_test/telemetry";
  options.speed_scaling_topic.clear();
  hx_waypoint::TelemetryRecorder recorder(node.get(), {"joint_a"}, options);

  std::string error;
  std::thread initial_sample([&recorder]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      recorder.setSpeedScaling(1.0);
    });
  ASSERT_TRUE(recorder.armStrictSpeedScalingGuard(
      std::chrono::milliseconds(40), 1.0e-9, error)) << error;
  initial_sample.join();

  std::atomic_int violation_count{0};
  std::string violation_message;
  ASSERT_TRUE(recorder.activateStrictSpeedScalingGuard(
      [&violation_count, &violation_message](const std::string & violation) {
        violation_message = violation;
        ++violation_count;
      },
      error)) << error;

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  for (int index = 0; index < 5; ++index) {
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    recorder.setSpeedScaling(1.0);
    EXPECT_EQ(violation_count.load(), 0);
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (violation_count.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_EQ(violation_count.load(), 1);
  EXPECT_NE(violation_message.find("stopped publishing fresh samples"), std::string::npos);
  std::string latched_violation;
  EXPECT_FALSE(recorder.finalizeStrictSpeedScalingGuard(latched_violation));
  EXPECT_NE(latched_violation.find("missing speed-scaling evidence"), std::string::npos);

  executor.cancel();
  spin_thread.join();
  executor.remove_node(node);
}

TEST(PlaybackExecutorLifecycle, PendingCancelCancelsAControllerGoalAcceptedLate)
{
  ScopedRclcppContext context;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using ServerGoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  auto server_node = std::make_shared<rclcpp::Node>("late_accept_controller_test_server");
  auto client_node = std::make_shared<rclcpp::Node>("late_accept_controller_test_client");
  const std::string action_name = "/late_accept_controller/follow_joint_trajectory";
  std::atomic_bool goal_request_seen{false};
  std::atomic_int cancel_requests{0};
  std::atomic_int accepted_callbacks{0};
  std::atomic_int completed_code{-1};
  std::atomic_int failed_callbacks{0};
  std::atomic_bool client_terminal{false};
  std::mutex server_workers_mutex;
  std::vector<std::thread> server_workers;

  auto server = rclcpp_action::create_server<FollowJointTrajectory>(
    server_node,
    action_name,
    [&goal_request_seen](
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const FollowJointTrajectory::Goal>) {
      goal_request_seen.store(true);
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [&cancel_requests](const std::shared_ptr<ServerGoalHandle>) {
      ++cancel_requests;
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&server_workers_mutex, &server_workers](const std::shared_ptr<ServerGoalHandle> goal_handle) {
      std::thread worker([goal_handle]() {
          const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
          while (!goal_handle->is_canceling() && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
          }
          auto result = std::make_shared<FollowJointTrajectory::Result>();
          if (goal_handle->is_canceling()) {
            result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
            goal_handle->canceled(result);
          } else {
            result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
            goal_handle->abort(result);
          }
        });
      std::lock_guard<std::mutex> lock(server_workers_mutex);
      server_workers.push_back(std::move(worker));
    });

  rclcpp::executors::MultiThreadedExecutor ros_executor(rclcpp::ExecutorOptions(), 4U);
  ros_executor.add_node(server_node);
  ros_executor.add_node(client_node);
  std::thread spin_thread([&ros_executor]() {ros_executor.spin();});

  hx_waypoint::PlaybackExecutor playback(client_node.get(), action_name);
  const bool server_available = playback.waitForServer(std::chrono::seconds(2));
  hx_waypoint::PlaybackExecutor::Callbacks callbacks;
  callbacks.accepted = [&accepted_callbacks]() {++accepted_callbacks;};
  callbacks.completed = [&completed_code, &client_terminal](
    const hx_waypoint::PlaybackExecutor::GoalHandle::WrappedResult & result) {
      completed_code.store(static_cast<int>(result.code));
      client_terminal.store(true);
    };
  callbacks.failed_to_start = [&failed_callbacks, &client_terminal](const std::string &) {
      ++failed_callbacks;
      client_terminal.store(true);
    };
  const bool submitted = server_available && playback.sendTrajectory(
    sampleTrajectory(), std::move(callbacks));

  const auto request_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!goal_request_seen.load() && std::chrono::steady_clock::now() < request_deadline) {
    std::this_thread::yield();
  }
  if (goal_request_seen.load()) {
    playback.cancelActive();
  }
  const auto terminal_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!client_terminal.load() && std::chrono::steady_clock::now() < terminal_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  playback.shutdown();
  ros_executor.cancel();
  spin_thread.join();
  std::vector<std::thread> workers;
  {
    std::lock_guard<std::mutex> lock(server_workers_mutex);
    workers = std::move(server_workers);
  }
  for (auto & worker : workers) {
    if (worker.joinable()) {
      worker.join();
    }
  }

  EXPECT_TRUE(server_available);
  EXPECT_TRUE(submitted);
  EXPECT_TRUE(goal_request_seen.load());
  EXPECT_GT(cancel_requests.load(), 0);
  EXPECT_EQ(accepted_callbacks.load(), 1);
  EXPECT_EQ(failed_callbacks.load(), 0);
  EXPECT_EQ(
    completed_code.load(), static_cast<int>(rclcpp_action::ResultCode::CANCELED));
  (void)server;
}

TEST(TipKinematics, ComputesJacobianVelocityAtTheOffsetRodTip)
{
  TemporaryTree temporary("hx_waypoint_tip_kinematics_");
  const auto urdf = temporary.path() / "one_joint.urdf";
  writeText(
    urdf,
    "<?xml version=\"1.0\"?>\n"
    "<robot name=\"one_joint\">\n"
    "  <link name=\"base\"/>\n"
    "  <link name=\"tool\"/>\n"
    "  <joint name=\"joint_a\" type=\"revolute\">\n"
    "    <parent link=\"base\"/><child link=\"tool\"/>\n"
    "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n"
    "    <axis xyz=\"0 0 1\"/>\n"
    "    <limit lower=\"-6.3\" upper=\"6.3\" effort=\"10\" velocity=\"10\"/>\n"
    "  </joint>\n"
    "</robot>\n");
  hx_waypoint::TipKinematics kinematics(
    urdf, "base", "tool", "rod_tip", {{1.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0, 1.0}},
    {"joint_a"});

  const auto at_zero = kinematics.evaluate({0.0}, {2.0});
  EXPECT_NEAR(at_zero.position_m[0], 1.0, 1.0e-12);
  EXPECT_NEAR(at_zero.position_m[1], 0.0, 1.0e-12);
  EXPECT_NEAR(at_zero.linear_velocity_m_s[0], 0.0, 1.0e-12);
  EXPECT_NEAR(at_zero.linear_velocity_m_s[1], 2.0, 1.0e-12);
  EXPECT_NEAR(at_zero.angular_velocity_rad_s[2], 2.0, 1.0e-12);

  const auto at_quarter_turn = kinematics.evaluate({0.5 * std::acos(-1.0)}, {2.0});
  EXPECT_NEAR(at_quarter_turn.position_m[0], 0.0, 1.0e-12);
  EXPECT_NEAR(at_quarter_turn.position_m[1], 1.0, 1.0e-12);
  EXPECT_NEAR(at_quarter_turn.linear_velocity_m_s[0], -2.0, 1.0e-12);
  EXPECT_NEAR(at_quarter_turn.linear_velocity_m_s[1], 0.0, 1.0e-12);
}

struct RecordingFixture
{
  std::filesystem::path environment;
  std::filesystem::path recording;
};

std::string sampleRobotUrdf()
{
  return
    "<?xml version=\"1.0\"?>\n"
    "<robot name=\"ur_test\">\n"
    "  <link name=\"base_link\"/>\n"
    "  <link name=\"base\"/>\n"
    "  <joint name=\"base_link_to_base\" type=\"fixed\">\n"
    "    <parent link=\"base_link\"/><child link=\"base\"/>\n"
    "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n"
    "  </joint>\n"
    "  <link name=\"shoulder_link\"/>\n"
    "  <joint name=\"joint_a\" type=\"revolute\">\n"
    "    <parent link=\"base_link\"/><child link=\"shoulder_link\"/>\n"
    "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/><axis xyz=\"0 0 1\"/>\n"
    "    <limit lower=\"-2\" upper=\"2\" effort=\"10\" velocity=\"10\"/>\n"
    "  </joint>\n"
    "  <link name=\"upper_arm_link\"/>\n"
    "  <joint name=\"shoulder_to_upper\" type=\"fixed\">\n"
    "    <parent link=\"shoulder_link\"/><child link=\"upper_arm_link\"/>\n"
    "    <origin xyz=\"1 0 0\" rpy=\"0 0 0\"/>\n"
    "  </joint>\n"
    "  <link name=\"forearm_link\"/>\n"
    "  <joint name=\"joint_b\" type=\"revolute\">\n"
    "    <parent link=\"upper_arm_link\"/><child link=\"forearm_link\"/>\n"
    "    <origin xyz=\"1 0 0\" rpy=\"0 0 0\"/><axis xyz=\"0 0 1\"/>\n"
    "    <limit lower=\"-2\" upper=\"2\" effort=\"10\" velocity=\"10\"/>\n"
    "  </joint>\n"
    "  <link name=\"wrist_1_link\"/>\n"
    "  <joint name=\"forearm_to_wrist_1\" type=\"fixed\">\n"
    "    <parent link=\"forearm_link\"/><child link=\"wrist_1_link\"/>\n"
    "    <origin xyz=\"0.1 0 0\" rpy=\"0 0 0\"/>\n"
    "  </joint>\n"
    "  <link name=\"wrist_2_link\"/>\n"
    "  <joint name=\"wrist_1_to_wrist_2\" type=\"fixed\">\n"
    "    <parent link=\"wrist_1_link\"/><child link=\"wrist_2_link\"/>\n"
    "    <origin xyz=\"0.1 0 0\" rpy=\"0 0 0\"/>\n"
    "  </joint>\n"
    "  <link name=\"wrist_3_link\"/>\n"
    "  <joint name=\"wrist_2_to_wrist_3\" type=\"fixed\">\n"
    "    <parent link=\"wrist_2_link\"/><child link=\"wrist_3_link\"/>\n"
    "    <origin xyz=\"0.1 0 0\" rpy=\"0 0 0\"/>\n"
    "  </joint>\n"
    "  <link name=\"tool0\"/>\n"
    "  <joint name=\"wrist_3_to_tool0\" type=\"fixed\">\n"
    "    <parent link=\"wrist_3_link\"/><child link=\"tool0\"/>\n"
    "    <origin xyz=\"0.1 0 0\" rpy=\"0 0 0\"/>\n"
    "  </joint>\n"
    "</robot>\n";
}

void createTestEnvironment(const std::filesystem::path & environment)
{
  const auto urdf_path = environment / "robot/assets/ur_test.urdf";
  writeText(urdf_path, sampleRobotUrdf());
  for (const auto & relative : environmentFiles()) {
    const auto path = environment / relative;
    if (relative == "environment.yaml") {
      writeText(path, "schema_version: 1\nenvironment_id: lab\n");
    } else if (relative == "robot/robot_model.yaml") {
      writeText(
        path,
        "schema_version: 1\nrobot:\n  type: ur_test\n"
        "  base_link: base_link\n  tool_link: tool0\n"
        "  joint_names: [joint_a, joint_b]\n"
        "  home_joint_positions: [0.0, 0.0]\n"
        "  limits:\n"
        "    velocity_rad_s: [10.0, 10.0]\n"
        "    acceleration_rad_s2: [100.0, 100.0]\n"
        "    jerk_rad_s3: [1000.0, 1000.0]\n");
    } else if (relative == "robot/asset_provenance.yaml") {
      writeText(
        path,
        "schema_version: 1\nrobot_type: ur_test\nrobot_name: ur_test\n"
        "generated_urdf:\n  path: robot/assets/ur_test.urdf\n  sha256: " +
        hx_waypoint::RecordingStore::sha256File(urdf_path) + "\n");
    } else if (relative == "robot/curobo_robot.yaml") {
      writeText(
        path,
        "schema_version: 1\nrobot_cfg:\n  kinematics:\n"
        "    asset_root_path: robot/assets\n    urdf_path: ur_test.urdf\n"
        "    base_link: base_link\n    tool_frames: [tool0]\n"
        "    cspace:\n      joint_names: [joint_a, joint_b]\n");
    } else if (relative == "setup.yaml") {
      writeText(
        path,
        "schema_version: 1\nsetup_id: test_setup\n"
        "work_frame:\n  parent_frame: base\n  child_frame: plane1\n"
        "  teach_pendant_pose_xyz_rotvec: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n"
        "  aliases: []\n"
        "tool:\n  tool_id: test_rod\n  parent_link: tool0\n  tip_frame: rod_tip\n"
        "  tip_orientation_xyzw: [0.0, 0.0, 0.0, 1.0]\n"
        "  rod:\n    axis_in_parent_xyz: [1.0, 0.0, 0.0]\n"
        "    length_m: 0.2\n    radius_m: 0.01\n    sphere_spacing_m: 0.02\n");
    } else if (relative == "planner.yaml") {
      writeText(
        path,
        "schema_version: 1\nvalidation:\n  tip_position_tolerance_m: 0.0001\n"
        "  tip_orientation_tolerance_deg: 0.5\n"
        "  minimum_collision_distance_m: 0.005\n"
        "limits:\n  on_violation: reject\n"
        "  allow_uniform_time_scale: false\n  max_uniform_time_scale: 1.0\n");
    } else if (relative == "recording.yaml") {
      writeText(path, simulationRecordingConfigYaml());
    } else {
      writeText(path, "schema_version: 1\n");
    }
  }
}

void writeChecksums(const std::filesystem::path & recording)
{
  std::vector<std::filesystem::path> files;
  for (const auto & entry : std::filesystem::recursive_directory_iterator(recording)) {
    if (entry.is_regular_file() && entry.path().filename() != "sha256sums.txt") {
      files.push_back(entry.path());
    }
  }
  std::sort(files.begin(), files.end());
  std::ostringstream checksums;
  for (const auto & file : files) {
    checksums << hx_waypoint::RecordingStore::sha256File(file) << "  " <<
      file.lexically_relative(recording).generic_string() << '\n';
  }
  writeText(recording / "sha256sums.txt", checksums.str());
}

std::vector<double> playbackValidationTimes()
{
  std::vector<double> times;
  times.reserve(101U);
  for (std::size_t index = 0U; index <= 100U; ++index) {
    times.push_back(static_cast<double>(index) * 0.01);
  }
  return times;
}

hx_waypoint::SimulationRecordingPaths writePlaybackValidationSamplesAtTimes(
  const std::filesystem::path & recording,
  const std::vector<double> & times)
{
  hx_waypoint::SimulationRecordingPaths paths{
    recording / "sim_actual_joint.csv",
    recording / "sim_controller_state.csv",
    recording / "joint_frames.csv",
    recording / "sim_actual_tip.csv"};

  std::ostringstream actual;
  actual << std::setprecision(17) <<
    "ros_time_s,trajectory_time_s,joint_a_pos,joint_b_pos,joint_a_vel,joint_b_vel\n";
  for (const double time : times) {
    const double joint_a = 0.5 * time;
    const double joint_b = 1.0 + 0.5 * time;
    actual << 100.0 + time << ',' << time << ',' << joint_a << ',' << joint_b <<
      ",0,0\n";
  }
  writeText(paths.actual_joint, actual.str());

  std::ostringstream controller;
  controller << std::setprecision(17) <<
    "ros_time_s,trajectory_time_s,speed_scaling,"
    "reference_joint_a_pos,reference_joint_b_pos,"
    "reference_joint_a_vel,reference_joint_b_vel,"
    "feedback_joint_a_pos,feedback_joint_b_pos,"
    "feedback_joint_a_vel,feedback_joint_b_vel,"
    "error_joint_a_pos,error_joint_b_pos,error_joint_a_vel,error_joint_b_vel\n";
  for (const double time : times) {
    const double joint_a = 0.5 * time;
    const double joint_b = 1.0 + 0.5 * time;
    controller << 100.0 + time << ',' << time << ",1," <<
      joint_a << ',' << joint_b << ",0,0," <<
      joint_a << ',' << joint_b << ",0,0,0,0,0,0\n";
  }
  writeText(paths.controller_state, controller.str());

  std::ostringstream frames;
  frames << std::setprecision(17) <<
    "ros_time_s,trajectory_time_s,frame_id,x_m,y_m,z_m,qx,qy,qz,qw\n";
  for (const double time : times) {
    const double joint_a = 0.5 * time;
    const double joint_b = 1.0 + 0.5 * time;
    for (std::size_t frame_index = 0U;
      frame_index < hx_waypoint::kRequiredSimulationFrameNames.size(); ++frame_index)
    {
      const auto pose = planarFramePose(
        hx_waypoint::kRequiredSimulationFrameNames[frame_index], joint_a, joint_b);
      frames << 100.0 + time << ',' << time << ',' <<
        hx_waypoint::kRequiredSimulationFrameNames[frame_index] << ',' <<
        pose.position_m[0] << ',' << pose.position_m[1] << ',' << pose.position_m[2] << ',' <<
        pose.quaternion_xyzw[0] << ',' << pose.quaternion_xyzw[1] << ',' <<
        pose.quaternion_xyzw[2] << ',' << pose.quaternion_xyzw[3] << '\n';
    }
  }
  writeText(paths.joint_frames, frames.str());

  std::ostringstream tip;
  tip << std::setprecision(17) <<
    "ros_time_s,trajectory_time_s,x_m,y_m,z_m,qx,qy,qz,qw\n";
  for (const double time : times) {
    const auto pose = planarFramePose("rod_tip", 0.5 * time, 1.0 + 0.5 * time);
    tip << 100.0 + time << ',' << time << ',' <<
      pose.position_m[0] << ',' << pose.position_m[1] << ',' << pose.position_m[2] << ',' <<
      pose.quaternion_xyzw[0] << ',' << pose.quaternion_xyzw[1] << ',' <<
      pose.quaternion_xyzw[2] << ',' << pose.quaternion_xyzw[3] << '\n';
  }
  writeText(paths.actual_tip, tip.str());
  return paths;
}

hx_waypoint::SimulationRecordingPaths writePlaybackValidationSamples(
  const std::filesystem::path & recording)
{
  return writePlaybackValidationSamplesAtTimes(recording, playbackValidationTimes());
}

void appendSamplingValidationYaml(
  std::ostringstream & output,
  const std::string & indentation,
  const std::string & prefix,
  const hx_waypoint::SamplingValidationMetrics & metrics)
{
  const auto key = [&prefix](const std::string & suffix) {
      return prefix.empty() ? suffix : prefix + "_" + suffix;
    };
  output << indentation << key("sample_count") << ": " << metrics.sample_count << '\n' <<
    indentation << key("first_trajectory_time_s") << ": " <<
    metrics.first_trajectory_time_s << '\n' <<
    indentation << key("last_trajectory_time_s") << ": " <<
    metrics.last_trajectory_time_s << '\n' <<
    indentation << key("coverage_duration_s") << ": " <<
    metrics.coverage_duration_s << '\n' <<
    indentation << key("max_time_interval_s") << ": " <<
    metrics.max_time_interval_s << '\n' <<
    indentation << key("max_time_jitter_s") << ": " <<
    metrics.max_time_jitter_s << '\n' <<
    indentation << key("missing_sample_estimate") << ": " <<
    metrics.missing_sample_estimate << '\n';
}

std::string playbackValidationYaml(
  const hx_waypoint::SimulationRecordingMetrics & metrics,
  const hx_waypoint::SimulationValidationConfig & config,
  const hx_waypoint::CanonicalTrajectoryMetrics & canonical_dynamics,
  const hx_waypoint::CanonicalTipPoseMetrics & canonical_tip,
  const hx_waypoint::SimulationKinematicsValidationContext & kinematics_context)
{
  std::ostringstream validation;
  validation << std::setprecision(17) << std::boolalpha <<
    "schema_version: 1\n"
    "simulation_completed: true\n"
    "controller_result: succeeded\n"
    "simulation_execution_contract_revision: 1\n"
    "simulation_controller_name: joint_trajectory_controller\n"
    "simulation_controller_type: joint_trajectory_controller/JointTrajectoryController\n"
    "simulation_controller_state: active\n"
    "simulation_controller_claimed_interfaces: [joint_a/position, joint_b/position]\n"
    "simulation_controller_required_command_interfaces: [joint_a/position, joint_b/position]\n"
    "simulation_controller_required_state_interfaces: "
    "[joint_a/position, joint_a/velocity, joint_b/position, joint_b/velocity]\n"
    "simulation_hardware_name: ur_test\n"
    "simulation_hardware_type: system\n"
    "simulation_hardware_plugin: mock_components/GenericSystem\n"
    "simulation_hardware_state_id: 3\n"
    "simulation_hardware_state_label: active\n"
    "actual_joint_recorded: true\n"
    "controller_state_recorded: true\n"
    "joint_frames_recorded: true\n"
    "actual_tip_recorded: true\n"
    "point_count: 2\n"
    "duration_s: 1.0\n"
    "source_waypoint_count: 2\n"
    "matched_source_waypoint_count: 2\n"
    "source_duration_s: 1.0\n"
    "max_source_time_error_s: 0.0\n"
    "sample_dt_s: " << config.sample_dt_s << '\n' <<
    "max_missing_samples_allowed: " << config.max_missing_samples << '\n' <<
    "max_time_jitter_s_allowed: " << config.max_time_jitter_s << '\n' <<
    "require_action_success: " << config.require_action_success << '\n';
  appendSamplingValidationYaml(validation, "", "actual_joint", metrics.actual_joint);
  appendSamplingValidationYaml(validation, "", "controller", metrics.controller_state);
  validation <<
    "joint_frame_sample_count: " << metrics.joint_frame_sample_count << '\n' <<
    "joint_frame_stream_count: " << metrics.joint_frame_streams.size() << '\n' <<
    "max_joint_frame_quaternion_norm_error: " <<
    metrics.max_joint_frame_quaternion_norm_error << '\n' <<
    "joint_frame_streams:\n";
  for (const auto & frame : metrics.joint_frame_streams) {
    validation << "  - frame_id: " << frame.frame_id << '\n';
    appendSamplingValidationYaml(validation, "    ", "", frame.pose.sampling);
    validation << "    max_quaternion_norm_error: " <<
      frame.pose.max_quaternion_norm_error << '\n';
  }
  appendSamplingValidationYaml(validation, "", "actual_tip", metrics.actual_tip.sampling);
  validation <<
    "actual_tip_max_quaternion_norm_error: " <<
    metrics.actual_tip.max_quaternion_norm_error << '\n' <<
    "max_quaternion_norm_error_allowed: " << config.max_quaternion_norm_error << '\n' <<
    "max_completion_overrun_s_allowed: " << config.max_completion_overrun_s << '\n' <<
    "max_speed_scaling_error: " << metrics.max_speed_scaling_error << '\n' <<
    "speed_scaling_evidence: locked_standard_unscaled_joint_trajectory_controller\n"
    "max_controller_reference_position_error_rad: " <<
    metrics.max_controller_reference_position_error_rad << '\n' <<
    "max_controller_reference_position_error_rad_allowed: " <<
    config.max_controller_reference_position_error_rad << '\n' <<
    "max_controller_reference_velocity_error_rad_s: " <<
    metrics.max_controller_reference_velocity_error_rad_s << '\n' <<
    "max_controller_reference_velocity_error_rad_s_allowed: " <<
    config.max_controller_reference_velocity_error_rad_s << '\n' <<
    "max_joint_position_tracking_error_rad: " <<
    metrics.max_joint_position_tracking_error_rad << '\n' <<
    "max_joint_position_tracking_error_rad_allowed: " <<
    config.max_joint_position_tracking_error_rad << '\n' <<
    "max_joint_velocity_tracking_error_rad_s: " <<
    metrics.max_joint_velocity_tracking_error_rad_s << '\n' <<
    "max_joint_velocity_tracking_error_rad_s_allowed: " <<
    config.max_joint_velocity_tracking_error_rad_s << '\n' <<
    "max_actual_controller_position_error_rad: " <<
    metrics.max_actual_controller_position_error_rad << '\n' <<
    "max_actual_controller_position_error_rad_allowed: " <<
    config.max_actual_controller_position_error_rad << '\n' <<
    "max_actual_controller_velocity_error_rad_s: " <<
    metrics.max_actual_controller_velocity_error_rad_s << '\n' <<
    "max_actual_controller_velocity_error_rad_s_allowed: " <<
    config.max_actual_controller_velocity_error_rad_s << '\n' <<
    "max_frame_fk_position_error_m: " << metrics.max_frame_fk_position_error_m << '\n' <<
    "max_frame_fk_position_error_m_allowed: " << config.max_frame_fk_position_error_m << '\n' <<
    "max_frame_fk_orientation_error_deg: " <<
    metrics.max_frame_fk_orientation_error_deg << '\n' <<
    "max_frame_fk_orientation_error_deg_allowed: " <<
    config.max_frame_fk_orientation_error_deg << '\n' <<
    "max_tip_fk_position_error_m: " << metrics.max_tip_fk_position_error_m << '\n' <<
    "max_tip_fk_position_error_m_allowed: " << config.max_frame_fk_position_error_m << '\n' <<
    "max_tip_fk_orientation_error_deg: " << metrics.max_tip_fk_orientation_error_deg << '\n' <<
    "max_tip_fk_orientation_error_deg_allowed: " <<
    config.max_frame_fk_orientation_error_deg << '\n' <<
    "max_tip_source_position_error_m: " << metrics.max_tip_source_position_error_m << '\n' <<
    "max_tip_source_position_error_m_allowed: " <<
    kinematics_context.max_tip_source_position_error_m << '\n' <<
    "max_tip_source_orientation_error_deg: " <<
    metrics.max_tip_source_orientation_error_deg << '\n' <<
    "max_tip_source_orientation_error_deg_allowed: " <<
    kinematics_context.max_tip_source_orientation_error_deg << '\n' <<
    "uniform_time_scale: 1.0\n"
    "max_joint_velocity_ratio: " << canonical_dynamics.max_joint_velocity_ratio << '\n' <<
    "max_joint_acceleration_ratio: " << canonical_dynamics.max_joint_acceleration_ratio << '\n' <<
    "max_joint_jerk_ratio: " << canonical_dynamics.max_joint_jerk_ratio << '\n' <<
    "max_tip_position_error_m: " << canonical_tip.max_tip_position_error_m << '\n' <<
    "max_tip_orientation_error_deg: " << canonical_tip.max_tip_orientation_error_deg << '\n' <<
    "minimum_collision_distance_m: 0.01\n"
    "max_scene_collision_cost: 0.0\n"
    "max_self_collision_cost: 0.0\n"
    "max_joint_bound_cost: 0.0\n"
    "checked_sample_count: 2\n";
  return validation.str();
}

std::string indentYaml(const std::string & contents, const std::string & indentation)
{
  std::istringstream input(contents);
  std::ostringstream output;
  std::string line;
  while (std::getline(input, line)) {
    output << indentation << line << '\n';
  }
  return output.str();
}

std::size_t replaceAll(
  std::string & contents,
  const std::string & from,
  const std::string & to)
{
  if (from.empty()) {
    throw std::invalid_argument("replaceAll requires a non-empty search string");
  }
  std::size_t count = 0U;
  std::size_t position = 0U;
  while ((position = contents.find(from, position)) != std::string::npos) {
    contents.replace(position, from.size(), to);
    position += to.size();
    ++count;
  }
  return count;
}

void eraseLineContaining(std::string & contents, const std::string & marker)
{
  const auto marker_position = contents.find(marker);
  if (marker_position == std::string::npos) {
    throw std::runtime_error("YAML line marker was not found: " + marker);
  }
  const auto line_start_position = contents.rfind('\n', marker_position);
  const std::size_t line_start = line_start_position == std::string::npos ?
    0U : line_start_position + 1U;
  const auto line_end_position = contents.find('\n', marker_position);
  const std::size_t erase_size = line_end_position == std::string::npos ?
    contents.size() - line_start : line_end_position - line_start + 1U;
  contents.erase(line_start, erase_size);
}

void mutateCsvField(
  const std::filesystem::path & path,
  std::size_t data_row,
  std::size_t column,
  const std::string & replacement)
{
  std::istringstream input(readText(path));
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(input, line)) {
    lines.push_back(line);
  }
  if (data_row + 1U >= lines.size()) {
    throw std::runtime_error("CSV mutation data row is out of range");
  }
  std::vector<std::string> fields;
  std::istringstream row_input(lines[data_row + 1U]);
  std::string field;
  while (std::getline(row_input, field, ',')) {
    fields.push_back(field);
  }
  if (!lines[data_row + 1U].empty() && lines[data_row + 1U].back() == ',') {
    fields.emplace_back();
  }
  if (column >= fields.size()) {
    throw std::runtime_error("CSV mutation column is out of range");
  }
  fields[column] = replacement;
  std::ostringstream mutated_row;
  for (std::size_t index = 0U; index < fields.size(); ++index) {
    if (index > 0U) {
      mutated_row << ',';
    }
    mutated_row << fields[index];
  }
  lines[data_row + 1U] = mutated_row.str();

  std::ostringstream output;
  for (const auto & output_line : lines) {
    output << output_line << '\n';
  }
  writeText(path, output.str());
}

void removeCsvDataRow(const std::filesystem::path & path, std::size_t data_row)
{
  std::istringstream input(readText(path));
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(input, line)) {
    lines.push_back(line);
  }
  if (data_row + 1U >= lines.size()) {
    throw std::runtime_error("CSV removal data row is out of range");
  }
  lines.erase(lines.begin() + static_cast<std::ptrdiff_t>(data_row + 1U));
  std::ostringstream output;
  for (const auto & output_line : lines) {
    output << output_line << '\n';
  }
  writeText(path, output.str());
}

RecordingFixture createValidatedRecording(const std::filesystem::path & root)
{
  RecordingFixture fixture;
  fixture.environment = root / "lab";
  fixture.recording = fixture.environment / "recordings" / "line__20260717_120000";
  std::filesystem::create_directories(fixture.recording / "environment_snapshot");

  createTestEnvironment(fixture.environment);
  for (const auto & relative : environmentFiles()) {
    const auto path = fixture.environment / relative;
    const auto snapshot_path = fixture.recording / "environment_snapshot" / relative;
    std::filesystem::create_directories(snapshot_path.parent_path());
    std::filesystem::copy_file(path, snapshot_path);
  }
  const auto asset_relative = std::filesystem::path("robot/assets/ur_test.urdf");
  const auto asset_snapshot = fixture.recording / "environment_snapshot" / asset_relative;
  std::filesystem::create_directories(asset_snapshot.parent_path());
  std::filesystem::copy_file(fixture.environment / asset_relative, asset_snapshot);

  writeText(fixture.recording / "source_tip_trajectory.csv", sampleSourceTrajectoryCsv());
  writeText(fixture.recording / "joint_trajectory.csv", sampleJointTrajectoryCsv());
  const auto commanded_trajectory = hx_waypoint::RecordingStore::loadJointTrajectoryCsv(
    fixture.recording / "joint_trajectory.csv", {"joint_a", "joint_b"});
  const auto robot_validation_model = hx_waypoint::loadRobotValidationModel(
    fixture.recording / "environment_snapshot", {"joint_a", "joint_b"});
  const auto canonical_dynamics = hx_waypoint::validateCanonicalTrajectoryDynamics(
    commanded_trajectory, robot_validation_model);
  const auto kinematics_context =
    hx_waypoint::SimulationRecordingValidator::loadKinematicsContext(
    fixture.recording / "environment_snapshot",
    fixture.recording / "source_tip_trajectory.csv", {"joint_a", "joint_b"});
  const auto canonical_tip =
    hx_waypoint::SimulationRecordingValidator::validateCanonicalSourcePoses(
    commanded_trajectory, kinematics_context);
  std::ostringstream compile_report;
  compile_report << std::setprecision(17) <<
    "schema_version: 2\ncompiler_contract_revision: " << kCompilerContractRevision << '\n' <<
    "environment_id: lab\ntrajectory_id: line\n"
    "curobo_revision: " << kCuroboCommit << '\n' <<
    "source_trajectory_sha256: " << hx_waypoint::RecordingStore::sha256File(
    fixture.recording / "source_tip_trajectory.csv") << '\n' <<
    "joint_trajectory_sha256: " << hx_waypoint::RecordingStore::sha256File(
    fixture.recording / "joint_trajectory.csv") << '\n' <<
    "joint_winding:\n"
    "  policy: constant_whole_path_nearest_home_without_margin_loss\n"
    "  reference_positions_rad: [0.0, 0.0]\n"
    "  selected_seed_offsets_rad: [0.0, 0.0]\n"
    "  post_optimization_offsets_rad: [0.0, 0.0]\n"
    "environment_hashes:\n";
  for (const auto & relative : environmentFiles()) {
    compile_report << "  " << relative << ": " << hx_waypoint::RecordingStore::sha256File(
      fixture.environment / relative) << '\n';
  }
  compile_report << "  " << asset_relative.generic_string() << ": " <<
    hx_waypoint::RecordingStore::sha256File(fixture.environment / asset_relative) << '\n' <<
    "duration_s: 1.0\npoint_count: 2\nsource_waypoint_count: 2\n"
    "matched_source_waypoint_count: 2\nsource_duration_s: 1.0\n"
    "max_source_time_error_s: 0.0\nuniform_time_scale: 1.0\n"
    "max_joint_velocity_ratio: " << canonical_dynamics.max_joint_velocity_ratio << '\n' <<
    "max_joint_acceleration_ratio: " << canonical_dynamics.max_joint_acceleration_ratio << '\n' <<
    "max_joint_jerk_ratio: " << canonical_dynamics.max_joint_jerk_ratio << '\n' <<
    "max_tip_position_error_m: " << canonical_tip.max_tip_position_error_m << '\n' <<
    "max_tip_orientation_error_deg: " << canonical_tip.max_tip_orientation_error_deg << '\n' <<
    "minimum_collision_distance_m: 0.01\n"
    "max_scene_collision_cost: 0.0\nmax_self_collision_cost: 0.0\n"
    "max_joint_bound_cost: 0.0\nchecked_sample_count: 2\n";
  writeText(fixture.recording / "compile_report.yaml", compile_report.str());
  const auto simulation_paths = writePlaybackValidationSamples(fixture.recording);
  const auto validation_config = hx_waypoint::SimulationRecordingValidator::loadConfig(
    fixture.recording / "environment_snapshot/recording.yaml");
  const auto simulation_metrics = hx_waypoint::SimulationRecordingValidator::validate(
    simulation_paths, commanded_trajectory, true, validation_config, kinematics_context);
  const std::string validation_yaml = playbackValidationYaml(
    simulation_metrics, validation_config, canonical_dynamics, canonical_tip,
    kinematics_context);
  writeText(fixture.recording / "validation_report.yaml", validation_yaml);

  std::ostringstream manifest;
  manifest << std::setprecision(17) <<
    "schema_version: 1\n"
    "recording_id: line__20260717_120000\n"
    "status: validated\n"
    "environment_id: lab\n"
    "trajectory_id: line\n"
    "robot:\n  type: ur_test\n  joint_names: [joint_a, joint_b]\n"
    "trajectory:\n  duration_s: 1.0\n  point_count: 2\n"
    "  source_waypoint_count: 2\n  matched_source_waypoint_count: 2\n"
    "  source_duration_s: 1.0\n  max_source_time_error_s: 0.0\n"
    "  nominal_dt_s: 1.0\n"
    "  uniform_time_scale: 1.0\n  max_joint_velocity_ratio: " <<
    canonical_dynamics.max_joint_velocity_ratio << '\n' <<
    "  max_joint_acceleration_ratio: " << canonical_dynamics.max_joint_acceleration_ratio <<
    '\n' << "  max_joint_jerk_ratio: " << canonical_dynamics.max_joint_jerk_ratio << '\n' <<
    "  interpolation: spline\n  fields: [position, velocity, acceleration]\n"
    "validation:\n" << indentYaml(validation_yaml, "  ") <<
    "hashes:\n";
  for (const auto & relative : environmentFiles()) {
    manifest << "  " << relative << ": " <<
      hx_waypoint::RecordingStore::sha256File(fixture.environment / relative) << '\n';
  }
  manifest << "  " << asset_relative.generic_string() << ": " <<
    hx_waypoint::RecordingStore::sha256File(fixture.environment / asset_relative) << '\n';
  manifest << "  source_trajectory: " << hx_waypoint::RecordingStore::sha256File(
    fixture.recording / "source_tip_trajectory.csv") << '\n';
  manifest << "  joint_trajectory: " << hx_waypoint::RecordingStore::sha256File(
    fixture.recording / "joint_trajectory.csv") << '\n';
  manifest << "  curobo_revision: " << kCuroboCommit << '\n';
  writeText(fixture.recording / "manifest.yaml", manifest.str());
  writeChecksums(fixture.recording);
  return fixture;
}

hx_waypoint::RecordingLoadOptions loadOptions(const RecordingFixture & fixture)
{
  hx_waypoint::RecordingLoadOptions options;
  options.environment_directory = fixture.environment;
  options.expected_environment_id = "lab";
  options.expected_robot_type = "ur_test";
  options.expected_joint_names = {"joint_a", "joint_b"};
  return options;
}

struct CompiledRunFixture
{
  std::filesystem::path environment;
  std::filesystem::path runs_root;
  std::filesystem::path run;
  std::filesystem::path source;
  std::filesystem::path trajectory;
  std::filesystem::path report;
  hx_waypoint::CanonicalTrajectoryMetrics dynamics;
  hx_waypoint::CanonicalTipPoseMetrics tip;
};

CompiledRunFixture createCompiledRun(const std::filesystem::path & workspace)
{
  CompiledRunFixture fixture;
  fixture.environment = workspace / "environments" / "lab";
  fixture.runs_root = workspace / "runs";
  fixture.run = fixture.runs_root / "line__compiled__20260717_120000_000000";
  fixture.source = fixture.run / "source_tip_trajectory.csv";
  fixture.trajectory = fixture.run / "joint_trajectory.csv";
  fixture.report = fixture.run / "compile_report.yaml";
  writeText(
    workspace / "curobo.lock",
    std::string("schema_version: 1\ncommit: ") + kCuroboCommit + "\n");
  createTestEnvironment(fixture.environment);
  writeText(fixture.environment / "trajectories/line.csv", sampleSourceTrajectoryCsv());
  writeText(fixture.environment / "meshes/wall.stl", "solid wall\nendsolid wall\n");
  writeText(fixture.source, sampleSourceTrajectoryCsv());
  writeText(fixture.trajectory, sampleJointTrajectoryCsv());
  const auto commanded_trajectory = hx_waypoint::RecordingStore::loadJointTrajectoryCsv(
    fixture.trajectory, {"joint_a", "joint_b"});
  const auto robot_validation_model = hx_waypoint::loadRobotValidationModel(
    fixture.environment, {"joint_a", "joint_b"});
  fixture.dynamics = hx_waypoint::validateCanonicalTrajectoryDynamics(
    commanded_trajectory, robot_validation_model);
  const auto kinematics_context =
    hx_waypoint::SimulationRecordingValidator::loadKinematicsContext(
    fixture.environment, fixture.source, {"joint_a", "joint_b"});
  fixture.tip = hx_waypoint::SimulationRecordingValidator::validateCanonicalSourcePoses(
    commanded_trajectory, kinematics_context);
  std::ostringstream report;
  report << std::setprecision(17) <<
    "schema_version: 2\ncompiler_contract_revision: " << kCompilerContractRevision << '\n' <<
    "environment_id: lab\ntrajectory_id: line\n"
    "curobo_revision: " << kCuroboCommit << '\n' <<
    "source_trajectory_sha256: " << hx_waypoint::RecordingStore::sha256File(fixture.source) << '\n' <<
    "joint_trajectory_sha256: " <<
    hx_waypoint::RecordingStore::sha256File(fixture.trajectory) << '\n' <<
    "joint_winding:\n"
    "  policy: constant_whole_path_nearest_home_without_margin_loss\n"
    "  reference_positions_rad: [0.0, 0.0]\n"
    "  selected_seed_offsets_rad: [0.0, 0.0]\n"
    "  post_optimization_offsets_rad: [0.0, 0.0]\n"
    "environment_hashes:\n";
  for (const auto & relative : environmentFiles()) {
    report << "  " << relative << ": " << hx_waypoint::RecordingStore::sha256File(
      fixture.environment / relative) << '\n';
  }
  report << "  meshes/wall.stl: " << hx_waypoint::RecordingStore::sha256File(
    fixture.environment / "meshes/wall.stl") << '\n' <<
    "  robot/assets/ur_test.urdf: " << hx_waypoint::RecordingStore::sha256File(
    fixture.environment / "robot/assets/ur_test.urdf") << '\n' <<
    "duration_s: 1.0\npoint_count: 2\nsource_waypoint_count: 2\n"
    "matched_source_waypoint_count: 2\nsource_duration_s: 1.0\n"
    "max_source_time_error_s: 0.0\nuniform_time_scale: 1.0\n"
    "max_joint_velocity_ratio: " << fixture.dynamics.max_joint_velocity_ratio << '\n' <<
    "max_joint_acceleration_ratio: " << fixture.dynamics.max_joint_acceleration_ratio << '\n' <<
    "max_joint_jerk_ratio: " << fixture.dynamics.max_joint_jerk_ratio << '\n' <<
    "max_tip_position_error_m: " << fixture.tip.max_tip_position_error_m << '\n' <<
    "max_tip_orientation_error_deg: " << fixture.tip.max_tip_orientation_error_deg << '\n' <<
    "minimum_collision_distance_m: 0.01\n"
    "max_scene_collision_cost: 0.0\nmax_self_collision_cost: 0.0\n"
    "max_joint_bound_cost: 0.0\nchecked_sample_count: 2\n";
  writeText(fixture.report, report.str());
  return fixture;
}

TEST(SafetyGate, AcceptsReorderedSettledJointStateNearStart)
{
  hx_waypoint::SafetyGate gate({0.02, 0.01, 0.5});
  const auto result = gate.checkStartState(
    sampleTrajectory(), sampleJointState(), rclcpp::Time(10, 0, RCL_ROS_TIME));
  EXPECT_TRUE(result.accepted) << result.message;
  EXPECT_NEAR(result.max_joint_error_rad, 0.005, 1.0e-12);
}

TEST(SafetyGate, RejectsMovingRobot)
{
  hx_waypoint::SafetyGate gate({0.02, 0.01, 0.5});
  auto state = sampleJointState();
  state.velocity[0] = 0.02;
  const auto result = gate.checkStartState(
    sampleTrajectory(), state, rclcpp::Time(10, 0, RCL_ROS_TIME));
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.failure_reason, hx_waypoint::SafetyGateFailureReason::NOT_SETTLED);
  EXPECT_NE(result.message.find("not settled"), std::string::npos);
}

TEST(SafetyGate, ReportsPositionMismatchOnlyForOtherwiseSettledState)
{
  hx_waypoint::SafetyGate gate({0.02, 0.01, 0.5});
  auto state = sampleJointState();
  state.position = {-1.4, 1.4};
  auto result = gate.checkStartState(
    sampleTrajectory(), state, rclcpp::Time(10, 0, RCL_ROS_TIME));
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.failure_reason, hx_waypoint::SafetyGateFailureReason::POSITION_MISMATCH);

  state.velocity[0] = 0.02;
  result = gate.checkStartState(
    sampleTrajectory(), state, rclcpp::Time(10, 0, RCL_ROS_TIME));
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.failure_reason, hx_waypoint::SafetyGateFailureReason::NOT_SETTLED);
}

TEST(PrepareWorkflow, ExtractsOrderedCurrentPositions)
{
  std::vector<double> positions;
  std::string error;
  EXPECT_TRUE(hx_waypoint::extractOrderedJointPositions(
      sampleJointState(), {"joint_a", "joint_b"}, positions, error)) << error;
  ASSERT_EQ(positions.size(), 2U);
  EXPECT_DOUBLE_EQ(positions[0], 1.005);
  EXPECT_DOUBLE_EQ(positions[1], -1.005);
}

TEST(PrepareWorkflow, SettledWaitRequiresNewSamplesAndDwellAfterTransientMotion)
{
  using Status = hx_waypoint::SettledStateWaitStatus;
  const auto start = hx_waypoint::SettledStateWait::Clock::time_point{};
  hx_waypoint::SettledStateWait wait(start, std::chrono::seconds(5));
  hx_waypoint::SafetyGate gate;
  auto state = sampleJointState();
  state.velocity[0] = 0.0209484;
  EXPECT_EQ(wait.observe(gate.checkStartState(
      sampleTrajectory(), state, rclcpp::Time(10, 0, RCL_ROS_TIME)), start), Status::WAITING);
  state.velocity[0] = 0.0;
  const auto settled = gate.checkStartState(
    sampleTrajectory(), state, rclcpp::Time(10, 0, RCL_ROS_TIME));
  ASSERT_TRUE(settled.accepted);
  EXPECT_EQ(wait.observe(settled, start + std::chrono::milliseconds(10)), Status::WAITING);
  // Time passage alone does not reuse a cached sample to prove settling.
  EXPECT_EQ(wait.poll(start + std::chrono::milliseconds(120)), Status::WAITING);
  EXPECT_EQ(wait.observe(settled, start + std::chrono::milliseconds(130)), Status::WAITING);
  EXPECT_EQ(wait.observe(settled, start + std::chrono::milliseconds(140)), Status::WAITING);
  EXPECT_EQ(wait.observe(settled, start + std::chrono::milliseconds(230)), Status::SETTLED);
}

TEST(PrepareWorkflow, SettledWaitRestartsProofAfterGapInFreshActualSamples)
{
  using Status = hx_waypoint::SettledStateWaitStatus;
  const auto start = hx_waypoint::SettledStateWait::Clock::time_point{};
  hx_waypoint::SettledStateWait wait(start, std::chrono::seconds(5));
  hx_waypoint::SafetyGate gate;
  const auto settled = gate.checkStartState(
    sampleTrajectory(), sampleJointState(), rclcpp::Time(10, 0, RCL_ROS_TIME));
  ASSERT_TRUE(settled.accepted);
  EXPECT_EQ(wait.observe(settled, start), Status::WAITING);
  EXPECT_EQ(wait.observe(settled, start + std::chrono::milliseconds(50)), Status::WAITING);
  EXPECT_EQ(wait.observe(settled, start + std::chrono::milliseconds(2000)), Status::WAITING);
  EXPECT_EQ(wait.observe(settled, start + std::chrono::milliseconds(2050)), Status::WAITING);
  EXPECT_EQ(wait.observe(settled, start + std::chrono::milliseconds(2099)), Status::WAITING);
  EXPECT_EQ(wait.observe(settled, start + std::chrono::milliseconds(2100)), Status::SETTLED);
}

TEST(PrepareWorkflow, SettledWaitRestartsDwellOnAnyActualVelocitySpike)
{
  using Status = hx_waypoint::SettledStateWaitStatus;
  const auto start = hx_waypoint::SettledStateWait::Clock::time_point{};
  hx_waypoint::SettledStateWait wait(start, std::chrono::seconds(5));
  hx_waypoint::SafetyGate gate;
  auto state = sampleJointState();
  const auto settled = gate.checkStartState(
    sampleTrajectory(), state, rclcpp::Time(10, 0, RCL_ROS_TIME));
  ASSERT_TRUE(settled.accepted);
  EXPECT_EQ(wait.observe(settled, start), Status::WAITING);
  EXPECT_EQ(wait.observe(settled, start + std::chrono::milliseconds(80)), Status::WAITING);
  state.velocity[0] = 0.01;  // The original strict boundary is not relaxed.
  const auto moving = gate.checkStartState(
    sampleTrajectory(), state, rclcpp::Time(10, 0, RCL_ROS_TIME));
  ASSERT_EQ(moving.failure_reason, hx_waypoint::SafetyGateFailureReason::NOT_SETTLED);
  EXPECT_EQ(wait.observe(moving, start + std::chrono::milliseconds(90)), Status::WAITING);
  EXPECT_EQ(wait.observe(settled, start + std::chrono::milliseconds(100)), Status::WAITING);
  EXPECT_EQ(wait.observe(settled, start + std::chrono::milliseconds(150)), Status::WAITING);
  EXPECT_EQ(wait.observe(settled, start + std::chrono::milliseconds(199)), Status::WAITING);
  EXPECT_EQ(wait.observe(settled, start + std::chrono::milliseconds(200)), Status::SETTLED);
}

TEST(PrepareWorkflow, SettledWaitTimesOutForSustainedMotionOrMissingSamples)
{
  using Status = hx_waypoint::SettledStateWaitStatus;
  const auto start = hx_waypoint::SettledStateWait::Clock::time_point{};
  hx_waypoint::SettledStateWait wait(start, std::chrono::seconds(5));
  hx_waypoint::SafetyGate gate;
  auto state = sampleJointState();
  state.velocity[0] = 0.02;
  const auto moving = gate.checkStartState(
    sampleTrajectory(), state, rclcpp::Time(10, 0, RCL_ROS_TIME));
  EXPECT_EQ(wait.observe(moving, start), Status::WAITING);
  EXPECT_EQ(wait.observe(moving, start + std::chrono::seconds(4)), Status::WAITING);
  EXPECT_EQ(wait.poll(start + std::chrono::seconds(5)), Status::TIMED_OUT);
  state.velocity[0] = 0.0;
  EXPECT_EQ(wait.observe(gate.checkStartState(
      sampleTrajectory(), state, rclcpp::Time(10, 0, RCL_ROS_TIME)),
    start + std::chrono::seconds(6)), Status::TIMED_OUT);
  hx_waypoint::SettledStateWait no_samples(start, std::chrono::seconds(5));
  EXPECT_EQ(no_samples.poll(start + std::chrono::seconds(5)), Status::TIMED_OUT);
}

TEST(PrepareWorkflow, SettledWaitImmediatelyRejectsStaleMalformedAndNonfiniteSamples)
{
  using Status = hx_waypoint::SettledStateWaitStatus;
  const auto start = hx_waypoint::SettledStateWait::Clock::time_point{};
  hx_waypoint::SafetyGate gate;
  std::vector<sensor_msgs::msg::JointState> invalid_states(3U, sampleJointState());
  invalid_states[0].header.stamp.sec = 8;
  invalid_states[1].velocity.clear();
  invalid_states[2].velocity[0] = std::numeric_limits<double>::quiet_NaN();
  for (const auto & state : invalid_states) {
    hx_waypoint::SettledStateWait wait(start, std::chrono::seconds(5), true);
    const auto invalid = gate.checkStartState(
      sampleTrajectory(), state, rclcpp::Time(10, 0, RCL_ROS_TIME));
    ASSERT_FALSE(invalid.accepted);
    EXPECT_EQ(wait.observe(invalid, start), Status::REJECTED) << invalid.message;
    EXPECT_EQ(wait.lastSafety().failure_reason, invalid.failure_reason);
  }
}

TEST(PrepareWorkflow, SettledWaitRejectsMovedStartUnlessExplicitlyWaitingAfterMotion)
{
  using Status = hx_waypoint::SettledStateWaitStatus;
  const auto start = hx_waypoint::SettledStateWait::Clock::time_point{};
  hx_waypoint::SafetyGate gate;
  auto state = sampleJointState();
  state.position[0] += 0.1;
  const auto moved = gate.checkStartState(
    sampleTrajectory(), state, rclcpp::Time(10, 0, RCL_ROS_TIME));
  ASSERT_EQ(moved.failure_reason, hx_waypoint::SafetyGateFailureReason::POSITION_MISMATCH);
  hx_waypoint::SettledStateWait fixed_start(start, std::chrono::seconds(5));
  EXPECT_EQ(fixed_start.observe(moved, start), Status::REJECTED);
  hx_waypoint::SettledStateWait after_motion(start, std::chrono::seconds(5), true);
  EXPECT_EQ(after_motion.observe(moved, start), Status::WAITING);
}

TEST(PrepareWorkflow, SettledWaitHonorsCancellationBeforeAcceptingFinalSample)
{
  using Status = hx_waypoint::SettledStateWaitStatus;
  const auto start = hx_waypoint::SettledStateWait::Clock::time_point{};
  hx_waypoint::SettledStateWait wait(start, std::chrono::seconds(5));
  hx_waypoint::SafetyGate gate;
  const auto settled = gate.checkStartState(
    sampleTrajectory(), sampleJointState(), rclcpp::Time(10, 0, RCL_ROS_TIME));
  EXPECT_EQ(wait.observe(settled, start), Status::WAITING);
  EXPECT_EQ(wait.observe(settled, start + std::chrono::milliseconds(50)), Status::WAITING);
  EXPECT_EQ(wait.observe(settled, start + std::chrono::milliseconds(100), true), Status::CANCELED);
  EXPECT_EQ(wait.observe(settled, start + std::chrono::milliseconds(200)), Status::CANCELED);
}

TEST(PrepareWorkflow, AcceptsCompleteTrajectoryWithStrictEndpoints)
{
  const auto trajectory = sampleTrajectory();
  const auto result = hx_waypoint::validatePrepareTrajectory(
    trajectory, trajectory.joint_names, trajectory.points.front().positions,
    trajectory.points.back().positions);
  EXPECT_TRUE(result.accepted) << result.message;
  EXPECT_DOUBLE_EQ(result.duration_s, 1.0);
  EXPECT_DOUBLE_EQ(result.max_start_error_rad, 0.0);
  EXPECT_DOUBLE_EQ(result.max_target_error_rad, 0.0);
}

TEST(PrepareWorkflow, RejectsWrongEndpointOrIncompleteDerivatives)
{
  auto wrong_endpoint = sampleTrajectory();
  auto result = hx_waypoint::validatePrepareTrajectory(
    wrong_endpoint, wrong_endpoint.joint_names, {0.0, 0.0},
    wrong_endpoint.points.back().positions);
  EXPECT_FALSE(result.accepted);

  auto incomplete = sampleTrajectory();
  incomplete.points[1].accelerations.clear();
  result = hx_waypoint::validatePrepareTrajectory(
    incomplete, incomplete.joint_names, incomplete.points.front().positions,
    incomplete.points.back().positions);
  EXPECT_FALSE(result.accepted);

  auto moving_endpoint = sampleTrajectory();
  moving_endpoint.points.back().velocities[0] = 0.01;
  result = hx_waypoint::validatePrepareTrajectory(
    moving_endpoint, moving_endpoint.joint_names, moving_endpoint.points.front().positions,
    moving_endpoint.points.back().positions);
  EXPECT_FALSE(result.accepted);
}

TEST(RuntimePolicy, RequiresApprovedUnscaledControllerForRealExecution)
{
  EXPECT_EQ(
    hx_waypoint::selectControllerAction(
      "real", "/joint_trajectory_controller/follow_joint_trajectory",
      "/passthrough_trajectory_controller/follow_joint_trajectory", ""),
    "/passthrough_trajectory_controller/follow_joint_trajectory");
  EXPECT_EQ(
    hx_waypoint::selectControllerAction(
      "real", "/joint_trajectory_controller/follow_joint_trajectory",
      "/joint_trajectory_controller/follow_joint_trajectory", ""),
    "/joint_trajectory_controller/follow_joint_trajectory");
  EXPECT_THROW(
    hx_waypoint::selectControllerAction(
      "real", "/joint_trajectory_controller/follow_joint_trajectory",
      "/scaled_joint_trajectory_controller/follow_joint_trajectory", ""),
    std::invalid_argument);
  EXPECT_THROW(
    hx_waypoint::selectControllerAction(
      "real", "/joint_trajectory_controller/follow_joint_trajectory",
      "/robot_1/joint_trajectory_controller/follow_joint_trajectory", ""),
    std::invalid_argument);
  EXPECT_THROW(
    hx_waypoint::selectControllerAction(
      "real", "/joint_trajectory_controller/follow_joint_trajectory",
      "/joint_trajectory_controller/follow_joint_trajectory/debug", ""),
    std::invalid_argument);
}

TEST(RuntimePolicy, RejectsUnsafeControllerOverrides)
{
  EXPECT_THROW(
    hx_waypoint::selectControllerAction(
      "real", "/joint_trajectory_controller/follow_joint_trajectory",
      "/joint_trajectory_controller/follow_joint_trajectory",
      "/passthrough_trajectory_controller/follow_joint_trajectory"),
    std::invalid_argument);
  EXPECT_THROW(
    hx_waypoint::selectControllerAction(
      "simulation", "/joint_trajectory_controller/follow_joint_trajectory",
      "/joint_trajectory_controller/follow_joint_trajectory",
      "/passthrough_trajectory_controller/follow_joint_trajectory"),
    std::invalid_argument);
  EXPECT_THROW(
    hx_waypoint::selectControllerAction(
      "simulation", "/scaled_joint_trajectory_controller/follow_joint_trajectory",
      "/joint_trajectory_controller/follow_joint_trajectory", ""),
    std::invalid_argument);
  EXPECT_THROW(
    hx_waypoint::selectControllerAction(
      "simulation", "/robot_1/joint_trajectory_controller/follow_joint_trajectory",
      "/joint_trajectory_controller/follow_joint_trajectory", ""),
    std::invalid_argument);
  EXPECT_THROW(
    hx_waypoint::selectControllerAction(
      "real", "/scaled_joint_trajectory_controller/follow_joint_trajectory",
      "/joint_trajectory_controller/follow_joint_trajectory", ""),
    std::invalid_argument);
  EXPECT_EQ(
    hx_waypoint::selectControllerAction(
      "simulation", hx_waypoint::kRequiredSimulationActionName,
      "/joint_trajectory_controller/follow_joint_trajectory",
      hx_waypoint::kRequiredSimulationActionName),
    hx_waypoint::kRequiredSimulationActionName);

  EXPECT_NO_THROW(hx_waypoint::requireSimulationControllerStateTopic(
      hx_waypoint::kRequiredSimulationControllerStateTopic));
  EXPECT_THROW(
    hx_waypoint::requireSimulationControllerStateTopic(
      "/scaled_joint_trajectory_controller/controller_state"),
    std::invalid_argument);
  EXPECT_THROW(
    hx_waypoint::requireSimulationControllerStateTopic(
      "/robot_1/joint_trajectory_controller/controller_state"),
    std::invalid_argument);

  EXPECT_NO_THROW(hx_waypoint::requireStandardUnscaledSimulationController(
      hx_waypoint::kRequiredSimulationControllerName,
      hx_waypoint::kRequiredSimulationControllerType,
      "active"));
  EXPECT_THROW(
    hx_waypoint::requireStandardUnscaledSimulationController(
      hx_waypoint::kRequiredSimulationControllerName,
      "ur_controllers/ScaledJointTrajectoryController",
      "active"),
    std::invalid_argument);
  EXPECT_THROW(
    hx_waypoint::requireStandardUnscaledSimulationController(
      hx_waypoint::kRequiredSimulationControllerName,
      hx_waypoint::kRequiredSimulationControllerType,
      "inactive"),
    std::invalid_argument);
}

TEST(RuntimePolicy, RejectsRosRemapsOfLockedSimulationEndpoints)
{
  ScopedRclcppContext context;

  auto clean_node = std::make_shared<rclcpp::Node>("simulation_contract_clean_names");
  EXPECT_NO_THROW(hx_waypoint::requireUnremappedActionName(
      clean_node->get_node_base_interface(), hx_waypoint::kRequiredSimulationActionName,
      "simulation action"));
  EXPECT_NO_THROW(hx_waypoint::requireUnremappedName(
      clean_node->get_node_base_interface(),
      hx_waypoint::kRequiredSimulationControllerStateTopic, false,
      "simulation controller-state topic"));
  EXPECT_NO_THROW(hx_waypoint::requireUnremappedName(
      clean_node->get_node_base_interface(),
      hx_waypoint::kRequiredSimulationControllerManagerService, true,
      "simulation controller-manager identity service"));

  rclcpp::NodeOptions action_remap_options;
  action_remap_options.arguments({
      "--ros-args", "-r",
      std::string(hx_waypoint::kRequiredSimulationActionName) + ":=/unsafe_action"});
  auto action_remapped_node = std::make_shared<rclcpp::Node>(
    "simulation_contract_action_remap", action_remap_options);
  EXPECT_THROW(
    hx_waypoint::requireUnremappedActionName(
      action_remapped_node->get_node_base_interface(),
      hx_waypoint::kRequiredSimulationActionName, "simulation action"),
    std::invalid_argument);

  rclcpp::NodeOptions transport_remap_options;
  transport_remap_options.arguments({
      "--ros-args", "-r",
      std::string(hx_waypoint::kRequiredSimulationActionName) +
      "/_action/send_goal:=/unsafe_send_goal"});
  auto transport_remapped_node = std::make_shared<rclcpp::Node>(
    "simulation_contract_transport_remap", transport_remap_options);
  EXPECT_THROW(
    hx_waypoint::requireUnremappedActionName(
      transport_remapped_node->get_node_base_interface(),
      hx_waypoint::kRequiredSimulationActionName, "simulation action"),
    std::invalid_argument);

  rclcpp::NodeOptions state_remap_options;
  state_remap_options.arguments({
      "--ros-args", "-r",
      std::string(hx_waypoint::kRequiredSimulationControllerStateTopic) +
      ":=/unsafe_controller_state"});
  auto state_remapped_node = std::make_shared<rclcpp::Node>(
    "simulation_contract_state_remap", state_remap_options);
  EXPECT_THROW(
    hx_waypoint::requireUnremappedName(
      state_remapped_node->get_node_base_interface(),
      hx_waypoint::kRequiredSimulationControllerStateTopic, false,
      "simulation controller-state topic"),
    std::invalid_argument);

  rclcpp::NodeOptions manager_remap_options;
  manager_remap_options.arguments({
      "--ros-args", "-r",
      std::string(hx_waypoint::kRequiredSimulationControllerManagerService) +
      ":=/unsafe_list_controllers"});
  auto manager_remapped_node = std::make_shared<rclcpp::Node>(
    "simulation_contract_manager_remap", manager_remap_options);
  EXPECT_THROW(
    hx_waypoint::requireUnremappedName(
      manager_remapped_node->get_node_base_interface(),
      hx_waypoint::kRequiredSimulationControllerManagerService, true,
      "simulation controller-manager identity service"),
    std::invalid_argument);
}

TEST(SimulationRecordingValidator, LoadsStrictValidationConfiguration)
{
  TemporaryTree temporary("hx_waypoint_recording_config_");
  const auto path = temporary.path() / "recording.yaml";
  writeText(
    path,
    "schema_version: 1\nrecording:\n  sample_dt_s: 0.01\n"
    "  record_controller_actual: true\n"
    "  record_joint_states: true\n"
    "  record_joint_frames: true\n"
    "  record_tip_state: true\n"
    "  time_source: trajectory_time\n"
    "  canonical_source: controller_action_goal\n"
    "  output_fields:\n    positions: true\n    velocities: true\n"
    "    accelerations: true\n"
    "  first_point:\n    time_s: 0.0\n    require_zero_velocity: true\n"
    "    require_zero_acceleration: true\n"
    "  last_point:\n    require_zero_velocity: true\n"
    "    require_zero_acceleration: true\n"
    "  validation:\n"
    "    max_missing_samples: 0\n    max_time_jitter_s: 0.002\n"
    "    max_quaternion_norm_error: 0.001\n"
    "    require_action_success: true\n"
    "    max_completion_overrun_s: 0.05\n"
    "    max_controller_reference_position_error_rad: 0.02\n"
    "    max_controller_reference_velocity_error_rad_s: 0.10\n"
    "    max_joint_position_tracking_error_rad: 0.02\n"
    "    max_joint_velocity_tracking_error_rad_s: 0.10\n"
    "    max_actual_controller_position_error_rad: 0.02\n"
    "    max_actual_controller_velocity_error_rad_s: 0.10\n"
    "    max_frame_fk_position_error_m: 0.001\n"
    "    max_frame_fk_orientation_error_deg: 0.1\n");
  const auto config = hx_waypoint::SimulationRecordingValidator::loadConfig(path);
  EXPECT_DOUBLE_EQ(config.sample_dt_s, 0.01);
  EXPECT_EQ(config.max_missing_samples, 0U);
  EXPECT_DOUBLE_EQ(config.max_quaternion_norm_error, 0.001);
  EXPECT_TRUE(config.require_action_success);
  EXPECT_DOUBLE_EQ(config.max_completion_overrun_s, 0.05);
  EXPECT_DOUBLE_EQ(config.max_controller_reference_position_error_rad, 0.02);
  EXPECT_DOUBLE_EQ(config.max_controller_reference_velocity_error_rad_s, 0.10);
  EXPECT_DOUBLE_EQ(config.max_joint_position_tracking_error_rad, 0.02);
  EXPECT_DOUBLE_EQ(config.max_joint_velocity_tracking_error_rad_s, 0.10);
  EXPECT_DOUBLE_EQ(config.max_actual_controller_position_error_rad, 0.02);
  EXPECT_DOUBLE_EQ(config.max_actual_controller_velocity_error_rad_s, 0.10);
  EXPECT_DOUBLE_EQ(config.max_frame_fk_position_error_m, 0.001);
  EXPECT_DOUBLE_EQ(config.max_frame_fk_orientation_error_deg, 0.1);
}

TEST(SimulationRecordingValidator, RejectsDisabledRequiredRecordingOrActionEvidence)
{
  TemporaryTree temporary("hx_waypoint_recording_config_required_streams_");
  const auto path = temporary.path() / "recording.yaml";
  for (const char * field : {
      "record_controller_actual", "record_joint_states", "record_joint_frames",
      "record_tip_state", "require_action_success"})
  {
    SCOPED_TRACE(field);
    std::string contents = simulationRecordingConfigYaml();
    const std::string key(field);
    ASSERT_EQ(replaceAll(contents, key + ": true", key + ": false"), 1U);
    writeText(path, contents);
    EXPECT_THROW(
      (void)hx_waypoint::SimulationRecordingValidator::loadConfig(path),
      std::runtime_error);
  }
}

TEST(SimulationRecordingValidator, ComputesSamplingAndTrackingMetrics)
{
  TemporaryTree temporary("hx_waypoint_sim_metrics_");
  const auto paths = writeSimulationSamples(
    temporary.path(), {0.0, 0.01, 0.02, 0.03, 0.04}, 0.005, 0.02);
  const auto command = validationCommand({0.0, 0.01, 0.02, 0.03, 0.04});
  const auto metrics = hx_waypoint::SimulationRecordingValidator::validate(
    paths, command, true, simulationValidationConfig());
  EXPECT_EQ(metrics.actual_joint.sample_count, 5U);
  EXPECT_EQ(metrics.controller_state.sample_count, 5U);
  EXPECT_EQ(metrics.joint_frame_sample_count, 10U);
  ASSERT_EQ(metrics.joint_frame_streams.size(), 2U);
  EXPECT_EQ(metrics.joint_frame_streams[0].frame_id, "link_a");
  EXPECT_EQ(metrics.joint_frame_streams[0].pose.sampling.sample_count, 5U);
  EXPECT_EQ(metrics.joint_frame_streams[1].frame_id, "rod_tip");
  EXPECT_EQ(metrics.actual_tip.sampling.sample_count, 5U);
  EXPECT_NEAR(metrics.actual_joint.coverage_duration_s, 0.04, 1.0e-12);
  EXPECT_NEAR(metrics.controller_state.max_time_interval_s, 0.01, 1.0e-12);
  EXPECT_EQ(metrics.actual_joint.missing_sample_estimate, 0U);
  EXPECT_EQ(metrics.controller_state.missing_sample_estimate, 0U);
  EXPECT_DOUBLE_EQ(metrics.max_joint_frame_quaternion_norm_error, 0.0);
  EXPECT_DOUBLE_EQ(metrics.actual_tip.max_quaternion_norm_error, 0.0);
  EXPECT_DOUBLE_EQ(metrics.max_speed_scaling_error, 0.0);
  EXPECT_DOUBLE_EQ(metrics.max_controller_reference_position_error_rad, 0.0);
  EXPECT_DOUBLE_EQ(metrics.max_controller_reference_velocity_error_rad_s, 0.0);
  EXPECT_NEAR(metrics.max_joint_position_tracking_error_rad, 0.005, 1.0e-12);
  EXPECT_NEAR(metrics.max_joint_velocity_tracking_error_rad_s, 0.02, 1.0e-12);
}

TEST(SimulationRecordingValidator, AcceptsOversampledControllerAndJointStateStreams)
{
  TemporaryTree temporary("hx_waypoint_sim_oversampled_");
  const auto paths = writeSimulationSamples(
    temporary.path(), {0.0, 0.005, 0.01, 0.015, 0.02, 0.025, 0.03, 0.035, 0.04},
    0.005, 0.02);
  const auto metrics = hx_waypoint::SimulationRecordingValidator::validate(
    paths, {"joint_a", "joint_b"}, 0.04, true, simulationValidationConfig());
  EXPECT_EQ(metrics.actual_joint.sample_count, 9U);
  EXPECT_EQ(metrics.actual_joint.missing_sample_estimate, 0U);
  EXPECT_DOUBLE_EQ(metrics.actual_joint.max_time_jitter_s, 0.0);
}

TEST(SimulationRecordingValidator, InterpolatesTipTelemetryToAuthoritativeSourceTimes)
{
  TemporaryTree temporary("hx_waypoint_sim_source_time_interpolation_");
  const auto environment = temporary.path() / "environment";
  const auto source_path = temporary.path() / "source_tip_trajectory.csv";
  const auto trajectory_path = temporary.path() / "joint_trajectory.csv";
  const auto telemetry = temporary.path() / "telemetry";
  createTestEnvironment(environment);

  std::ostringstream source;
  source << std::setprecision(17) <<
    "time_s,frame_id,tip_x_m,tip_y_m,tip_z_m,tip_qx,tip_qy,tip_qz,tip_qw,phase\n";
  for (const double time : {0.0, 0.5, 1.0}) {
    const auto pose = planarFramePose("rod_tip", 0.5 * time, 1.0 + 0.5 * time);
    source << time << ",base_link," << pose.position_m[0] << ',' << pose.position_m[1] << ',' <<
      pose.position_m[2] << ',' << pose.quaternion_xyzw[0] << ',' <<
      pose.quaternion_xyzw[1] << ',' << pose.quaternion_xyzw[2] << ',' <<
      pose.quaternion_xyzw[3] << ",draw\n";
  }
  writeText(source_path, source.str());
  writeText(
    trajectory_path,
    "time_s,joint_a_pos,joint_b_pos,joint_a_vel,joint_b_vel,joint_a_acc,joint_b_acc,phase\n"
    "0,0,1,0,0,0,0,draw\n"
    "0.5,0.25,1.25,0,0,0,0,draw\n"
    "1,0.5,1.5,0,0,0,0,draw\n");
  const auto command = hx_waypoint::RecordingStore::loadJointTrajectoryCsv(
    trajectory_path, {"joint_a", "joint_b"});
  const auto kinematics_context =
    hx_waypoint::SimulationRecordingValidator::loadKinematicsContext(
    environment, source_path, {"joint_a", "joint_b"});

  std::vector<double> telemetry_times;
  telemetry_times.reserve(126U);
  for (std::size_t index = 0U; index < 125U; ++index) {
    telemetry_times.push_back(static_cast<double>(index) * 0.008);
  }
  telemetry_times.push_back(1.0);
  ASSERT_EQ(telemetry_times.back(), 1.0);
  ASSERT_TRUE(std::find(telemetry_times.begin(), telemetry_times.end(), 0.5) ==
    telemetry_times.end());
  const auto paths = writePlaybackValidationSamplesAtTimes(telemetry, telemetry_times);

  const auto metrics = hx_waypoint::SimulationRecordingValidator::validate(
    paths, command, true, simulationValidationConfig(), kinematics_context);

  EXPECT_GT(metrics.max_tip_source_position_error_m, 0.0);
  EXPECT_LT(
    metrics.max_tip_source_position_error_m,
    kinematics_context.max_tip_source_position_error_m);
  EXPECT_LT(
    metrics.max_tip_source_orientation_error_deg,
    kinematics_context.max_tip_source_orientation_error_deg);
}

TEST(SimulationRecordingValidator, RejectsExecutionThatRunsLongerThanUnscaledCommand)
{
  TemporaryTree temporary("hx_waypoint_sim_scaled_duration_");
  const auto paths = writeSimulationSamples(
    temporary.path(),
    {0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.10},
    0.0, 0.0);
  EXPECT_THROW(
    (void)hx_waypoint::SimulationRecordingValidator::validate(
      paths, {"joint_a", "joint_b"}, 0.04, true, simulationValidationConfig()),
    std::runtime_error);
}

TEST(SimulationRecordingValidator, RejectsControllerReferenceThatDiffersFromCanonicalCommand)
{
  TemporaryTree temporary("hx_waypoint_sim_reference_mismatch_");
  const std::vector<double> times{0.0, 0.01, 0.02, 0.03, 0.04};
  const auto paths = writeSimulationSamples(
    temporary.path(), times, 0.0, 0.0, 1.0, 0.03, 0.0);
  const auto command = validationCommand(times);
  EXPECT_THROW(
    (void)hx_waypoint::SimulationRecordingValidator::validate(
      paths, command, true, simulationValidationConfig()),
    std::runtime_error);
}

TEST(SimulationRecordingValidator, RejectsMissingSamplesAndTrackingViolations)
{
  TemporaryTree temporary("hx_waypoint_sim_reject_");
  const auto missing_paths = writeSimulationSamples(
    temporary.path() / "missing", {0.0, 0.01, 0.03, 0.04}, 0.005, 0.02);
  EXPECT_THROW(
    (void)hx_waypoint::SimulationRecordingValidator::validate(
      missing_paths, {"joint_a", "joint_b"}, 0.04, true, simulationValidationConfig()),
    std::runtime_error);

  const auto tracking_paths = writeSimulationSamples(
    temporary.path() / "tracking", {0.0, 0.01, 0.02, 0.03, 0.04}, 0.03, 0.02);
  EXPECT_THROW(
    (void)hx_waypoint::SimulationRecordingValidator::validate(
      tracking_paths, {"joint_a", "joint_b"}, 0.04, true, simulationValidationConfig()),
    std::runtime_error);
}

TEST(SimulationRecordingValidator, RejectsMissingPoseRecordingAndNonFiniteFrameValues)
{
  TemporaryTree temporary("hx_waypoint_sim_pose_missing_");
  const auto missing_paths = writeSimulationSamples(
    temporary.path() / "missing", {0.0, 0.01, 0.02, 0.03, 0.04}, 0.005, 0.02);
  std::filesystem::remove(missing_paths.actual_tip);
  EXPECT_THROW(
    (void)hx_waypoint::SimulationRecordingValidator::validate(
      missing_paths, {"joint_a", "joint_b"}, 0.04, true, simulationValidationConfig()),
    std::runtime_error);

  const auto nonfinite_paths = writeSimulationSamples(
    temporary.path() / "nonfinite", {0.0, 0.01, 0.02, 0.03, 0.04}, 0.005, 0.02);
  std::ostringstream frames;
  frames << "ros_time_s,trajectory_time_s,frame_id,x_m,y_m,z_m,qx,qy,qz,qw\n";
  for (const double time : {0.0, 0.01, 0.02, 0.03, 0.04}) {
    frames << 100.0 + time << ',' << time << ",rod_tip,"
      << (time == 0.02 ? "nan" : "0") << ",0,1,0,0,0,1\n";
  }
  writeText(nonfinite_paths.joint_frames, frames.str());
  EXPECT_THROW(
    (void)hx_waypoint::SimulationRecordingValidator::validate(
      nonfinite_paths, {"joint_a", "joint_b"}, 0.04, true, simulationValidationConfig()),
    std::runtime_error);
}

TEST(SimulationRecordingValidator, RejectsNonUnitTipQuaternionAndNonMonotonicTime)
{
  TemporaryTree temporary("hx_waypoint_sim_tip_numeric_");
  const auto quaternion_paths = writeSimulationSamples(
    temporary.path() / "quaternion", {0.0, 0.01, 0.02, 0.03, 0.04}, 0.005, 0.02);
  std::ostringstream invalid_quaternion;
  invalid_quaternion << "ros_time_s,trajectory_time_s,x_m,y_m,z_m,qx,qy,qz,qw\n";
  for (const double time : {0.0, 0.01, 0.02, 0.03, 0.04}) {
    invalid_quaternion << 100.0 + time << ',' << time << ",0,0,1,0,0,0,2\n";
  }
  writeText(quaternion_paths.actual_tip, invalid_quaternion.str());
  EXPECT_THROW(
    (void)hx_waypoint::SimulationRecordingValidator::validate(
      quaternion_paths, {"joint_a", "joint_b"}, 0.04, true, simulationValidationConfig()),
    std::runtime_error);

  const auto time_paths = writeSimulationSamples(
    temporary.path() / "time", {0.0, 0.01, 0.02, 0.03, 0.04}, 0.005, 0.02);
  writeText(
    time_paths.actual_tip,
    "ros_time_s,trajectory_time_s,x_m,y_m,z_m,qx,qy,qz,qw\n"
    "100.00,0.00,0,0,1,0,0,0,1\n"
    "100.01,0.01,0,0,1,0,0,0,1\n"
    "100.02,0.02,0,0,1,0,0,0,1\n"
    "100.03,0.01,0,0,1,0,0,0,1\n"
    "100.04,0.04,0,0,1,0,0,0,1\n");
  EXPECT_THROW(
    (void)hx_waypoint::SimulationRecordingValidator::validate(
      time_paths, {"joint_a", "joint_b"}, 0.04, true, simulationValidationConfig()),
    std::runtime_error);
}

TEST(SimulationRecordingValidator, RejectsPerFrameMissingSamplesAndIncompleteTipCoverage)
{
  TemporaryTree temporary("hx_waypoint_sim_pose_coverage_");
  const auto frame_paths = writeSimulationSamples(
    temporary.path() / "frames", {0.0, 0.01, 0.02, 0.03, 0.04}, 0.005, 0.02);
  std::ostringstream frames;
  frames << "ros_time_s,trajectory_time_s,frame_id,x_m,y_m,z_m,qx,qy,qz,qw\n";
  for (const double time : {0.0, 0.01, 0.02, 0.03, 0.04}) {
    frames << 100.0 + time << ',' << time << ",link_a,0,0,0,0,0,0,1\n";
  }
  for (const double time : {0.0, 0.01, 0.03, 0.04}) {
    frames << 100.0 + time << ',' << time << ",rod_tip,0,0,1,0,0,0,1\n";
  }
  writeText(frame_paths.joint_frames, frames.str());
  EXPECT_THROW(
    (void)hx_waypoint::SimulationRecordingValidator::validate(
      frame_paths, {"joint_a", "joint_b"}, 0.04, true, simulationValidationConfig()),
    std::runtime_error);

  const auto tip_paths = writeSimulationSamples(
    temporary.path() / "tip", {0.0, 0.01, 0.02, 0.03, 0.04}, 0.005, 0.02);
  writeText(
    tip_paths.actual_tip,
    "ros_time_s,trajectory_time_s,x_m,y_m,z_m,qx,qy,qz,qw\n"
    "100.00,0.00,0,0,1,0,0,0,1\n"
    "100.01,0.01,0,0,1,0,0,0,1\n");
  EXPECT_THROW(
    (void)hx_waypoint::SimulationRecordingValidator::validate(
      tip_paths, {"joint_a", "joint_b"}, 0.04, true, simulationValidationConfig()),
    std::runtime_error);
}

TEST(SimulationRecordingValidator, RequiresUnitSimulationSpeedScaling)
{
  TemporaryTree temporary("hx_waypoint_sim_speed_scaling_");
  const auto paths = writeSimulationSamples(
    temporary.path(), {0.0, 0.01, 0.02, 0.03, 0.04}, 0.005, 0.02, 0.99);
  EXPECT_THROW(
    (void)hx_waypoint::SimulationRecordingValidator::validate(
      paths, {"joint_a", "joint_b"}, 0.04, true, simulationValidationConfig()),
    std::runtime_error);
}

TEST(SimulationRecordingValidator, EnforcesActionSuccessConfiguration)
{
  hx_waypoint::SimulationRecordingPaths paths;
  EXPECT_THROW(
    (void)hx_waypoint::SimulationRecordingValidator::validate(
      paths, {"joint_a", "joint_b"}, 1.0, false, simulationValidationConfig()),
    std::runtime_error);
}

TEST(RecordingStore, ComputesKnownSha256)
{
  TemporaryTree temporary("hx_waypoint_sha256_");
  const auto path = temporary.path() / "input.txt";
  writeText(path, "abc");
  EXPECT_EQ(
    hx_waypoint::RecordingStore::sha256File(path),
    "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

TEST(RecordingStore, LoadsCompleteValidatedArtifactWithAuthoritativeTiming)
{
  TemporaryTree temporary("hx_waypoint_recording_");
  const auto fixture = createValidatedRecording(temporary.path());
  const auto options = loadOptions(fixture);
  const auto artifact = hx_waypoint::RecordingStore::loadValidatedDirectory(
    fixture.recording, options);
  EXPECT_EQ(artifact.trajectory.points.size(), 2U);
  EXPECT_EQ(artifact.trajectory.joint_names, options.expected_joint_names);
  EXPECT_EQ(artifact.manifest.curobo_revision, kCuroboCommit);
}

TEST(RecordingStore, RejectsPlaybackWhenAuthoritativeSetupChanges)
{
  TemporaryTree temporary("hx_waypoint_setup_mismatch_");
  const auto fixture = createValidatedRecording(temporary.path());
  writeText(
    fixture.environment / "setup.yaml",
    readText(fixture.environment / "setup.yaml") + "\n# changed setup bytes\n");

  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(
      fixture.recording, loadOptions(fixture)),
    std::runtime_error);
}

TEST(RecordingStore, RejectsActualJointStreamThatDoesNotMatchControllerFeedback)
{
  TemporaryTree temporary("hx_waypoint_actual_joint_execution_tamper_");
  const auto fixture = createValidatedRecording(temporary.path());
  mutateCsvField(fixture.recording / "sim_actual_joint.csv", 50U, 2U, "123.0");
  writeChecksums(fixture.recording);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(
      fixture.recording, loadOptions(fixture)),
    std::runtime_error);
}

TEST(RecordingStore, RejectsAllZeroJointFrameTransforms)
{
  TemporaryTree temporary("hx_waypoint_all_zero_frame_tamper_");
  const auto fixture = createValidatedRecording(temporary.path());
  std::ostringstream frames;
  frames << "ros_time_s,trajectory_time_s,frame_id,x_m,y_m,z_m,qx,qy,qz,qw\n";
  for (const double time : playbackValidationTimes()) {
    for (const auto & frame_id : hx_waypoint::kRequiredSimulationFrameNames) {
      frames << std::setprecision(17) << 100.0 + time << ',' << time << ',' << frame_id <<
        ",0,0,0,0,0,0,1\n";
    }
  }
  writeText(fixture.recording / "joint_frames.csv", frames.str());
  writeChecksums(fixture.recording);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(
      fixture.recording, loadOptions(fixture)),
    std::runtime_error);
}

TEST(RecordingStore, RejectsActualTipDeviationFromAuthoritativeSourcePose)
{
  TemporaryTree temporary("hx_waypoint_actual_tip_source_tamper_");
  const auto fixture = createValidatedRecording(temporary.path());
  const auto expected = planarFramePose("rod_tip", 0.0, 1.0);
  std::ostringstream shifted_x;
  shifted_x << std::setprecision(17) << expected.position_m[0] + 0.0005;
  mutateCsvField(
    fixture.recording / "sim_actual_tip.csv", 0U, 2U, shifted_x.str());
  writeChecksums(fixture.recording);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(
      fixture.recording, loadOptions(fixture)),
    std::runtime_error);
}

TEST(RecordingStore, RejectsMissingRecomputedTelemetryEvidenceInBothValidationDocuments)
{
  TemporaryTree temporary("hx_waypoint_missing_recomputed_telemetry_evidence_");
  const auto fixture = createValidatedRecording(temporary.path());
  const auto validation_path = fixture.recording / "validation_report.yaml";
  const auto manifest_path = fixture.recording / "manifest.yaml";
  std::string validation = readText(validation_path);
  std::string manifest = readText(manifest_path);
  eraseLineContaining(validation, "actual_joint_sample_count:");
  eraseLineContaining(manifest, "actual_joint_sample_count:");
  writeText(validation_path, validation);
  writeText(manifest_path, manifest);
  writeChecksums(fixture.recording);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(
      fixture.recording, loadOptions(fixture)),
    std::runtime_error);
}

TEST(RecordingStore, RejectsValidationReportThatDiffersFromManifestValidation)
{
  TemporaryTree temporary("hx_waypoint_validation_document_mismatch_");
  const auto fixture = createValidatedRecording(temporary.path());
  const auto validation_path = fixture.recording / "validation_report.yaml";
  std::string validation = readText(validation_path);
  validation += "unmatched_runtime_note: tampered\n";
  writeText(validation_path, validation);
  writeChecksums(fixture.recording);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(
      fixture.recording, loadOptions(fixture)),
    std::runtime_error);
}

TEST(RecordingStore, RejectsAllowedTelemetryThresholdThatDiffersFromSnapshot)
{
  TemporaryTree temporary("hx_waypoint_allowed_threshold_tamper_");
  const auto fixture = createValidatedRecording(temporary.path());
  const auto validation_path = fixture.recording / "validation_report.yaml";
  const auto manifest_path = fixture.recording / "manifest.yaml";
  std::string validation = readText(validation_path);
  std::string manifest = readText(manifest_path);
  const std::string original = "max_joint_position_tracking_error_rad_allowed: 0.02";
  ASSERT_EQ(replaceAll(validation, original, "max_joint_position_tracking_error_rad_allowed: 0.5"),
    1U);
  ASSERT_EQ(replaceAll(manifest, original, "max_joint_position_tracking_error_rad_allowed: 0.5"),
    1U);
  writeText(validation_path, validation);
  writeText(manifest_path, manifest);
  writeChecksums(fixture.recording);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(
      fixture.recording, loadOptions(fixture)),
    std::runtime_error);
}

TEST(RecordingStore, RejectsTelemetryRowCountThatDiffersFromSealedEvidence)
{
  TemporaryTree temporary("hx_waypoint_telemetry_row_count_tamper_");
  const auto fixture = createValidatedRecording(temporary.path());
  removeCsvDataRow(fixture.recording / "sim_actual_joint.csv", 100U);
  writeChecksums(fixture.recording);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(
      fixture.recording, loadOptions(fixture)),
    std::runtime_error);
}

TEST(RecordingStore, RejectsNonMonotonicSealedTelemetryTime)
{
  TemporaryTree temporary("hx_waypoint_telemetry_time_tamper_");
  const auto fixture = createValidatedRecording(temporary.path());
  mutateCsvField(fixture.recording / "sim_controller_state.csv", 50U, 1U, "0.49");
  writeChecksums(fixture.recording);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(
      fixture.recording, loadOptions(fixture)),
    std::runtime_error);
}

TEST(RecordingStore, RejectsSealedTelemetrySpeedScalingDeviation)
{
  TemporaryTree temporary("hx_waypoint_telemetry_speed_scaling_tamper_");
  const auto fixture = createValidatedRecording(temporary.path());
  mutateCsvField(fixture.recording / "sim_controller_state.csv", 50U, 2U, "0.99");
  writeChecksums(fixture.recording);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(
      fixture.recording, loadOptions(fixture)),
    std::runtime_error);
}

TEST(RecordingStore, RejectsSealedControllerReferenceThatDiffersFromCanonicalTrajectory)
{
  TemporaryTree temporary("hx_waypoint_telemetry_reference_tamper_");
  const auto fixture = createValidatedRecording(temporary.path());
  const auto controller_path = fixture.recording / "sim_controller_state.csv";
  mutateCsvField(controller_path, 0U, 3U, "0.1");
  mutateCsvField(controller_path, 0U, 7U, "0.1");
  writeChecksums(fixture.recording);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(
      fixture.recording, loadOptions(fixture)),
    std::runtime_error);
}

TEST(RecordingStore, RejectsSealedJointTrackingViolation)
{
  TemporaryTree temporary("hx_waypoint_telemetry_tracking_tamper_");
  const auto fixture = createValidatedRecording(temporary.path());
  const auto controller_path = fixture.recording / "sim_controller_state.csv";
  mutateCsvField(controller_path, 50U, 7U, "0.35");
  mutateCsvField(controller_path, 50U, 11U, "0.1");
  writeChecksums(fixture.recording);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(
      fixture.recording, loadOptions(fixture)),
    std::runtime_error);
}

TEST(RecordingStore, RejectsMissingOrExtraRequiredJointFrameStream)
{
  TemporaryTree temporary("hx_waypoint_telemetry_frame_set_tamper_");
  const auto fixture = createValidatedRecording(temporary.path());
  const auto frames_path = fixture.recording / "joint_frames.csv";
  std::string frames = readText(frames_path);
  ASSERT_EQ(replaceAll(frames, ",shoulder_link,", ",unexpected_link,"), 101U);
  writeText(frames_path, frames);
  writeChecksums(fixture.recording);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(
      fixture.recording, loadOptions(fixture)),
    std::runtime_error);
}

TEST(RecordingStore, RejectsSealedJointFrameQuaternionViolation)
{
  TemporaryTree temporary("hx_waypoint_telemetry_quaternion_tamper_");
  const auto fixture = createValidatedRecording(temporary.path());
  mutateCsvField(fixture.recording / "joint_frames.csv", 0U, 9U, "0.5");
  writeChecksums(fixture.recording);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(
      fixture.recording, loadOptions(fixture)),
    std::runtime_error);
}

TEST(RecordingStore, RejectsMissingOrIncorrectManifestNominalDt)
{
  TemporaryTree temporary("hx_waypoint_manifest_nominal_dt_");
  const auto missing_fixture = createValidatedRecording(temporary.path() / "missing");
  const auto missing_manifest_path = missing_fixture.recording / "manifest.yaml";
  std::string missing_manifest = readText(missing_manifest_path);
  eraseLineContaining(missing_manifest, "nominal_dt_s:");
  writeText(missing_manifest_path, missing_manifest);
  writeChecksums(missing_fixture.recording);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(
      missing_fixture.recording, loadOptions(missing_fixture)),
    std::runtime_error);

  const auto wrong_fixture = createValidatedRecording(temporary.path() / "wrong");
  const auto wrong_manifest_path = wrong_fixture.recording / "manifest.yaml";
  std::string wrong_manifest = readText(wrong_manifest_path);
  ASSERT_EQ(replaceAll(wrong_manifest, "nominal_dt_s: 1.0", "nominal_dt_s: 0.5"), 1U);
  writeText(wrong_manifest_path, wrong_manifest);
  writeChecksums(wrong_fixture.recording);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(
      wrong_fixture.recording, loadOptions(wrong_fixture)),
    std::runtime_error);
}

TEST(RecordingStore, RejectsLegacyCompileReportAtPlaybackLoad)
{
  TemporaryTree temporary("hx_waypoint_legacy_recording_report_");
  const auto fixture = createValidatedRecording(temporary.path());
  std::string report = readText(fixture.recording / "compile_report.yaml");
  const std::string schema = "schema_version: 2";
  const auto position = report.find(schema);
  ASSERT_NE(position, std::string::npos);
  report.replace(position, schema.size(), "schema_version: 1");
  writeText(fixture.recording / "compile_report.yaml", report);
  writeChecksums(fixture.recording);
  try {
    (void)hx_waypoint::RecordingStore::loadValidatedDirectory(
      fixture.recording, loadOptions(fixture));
    FAIL() << "legacy compile_report.yaml was accepted for playback";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("schema_version must be 2"), std::string::npos);
  }
}

TEST(RecordingStore, RejectsSealedCompileReportWithWrongJointHash)
{
  TemporaryTree temporary("hx_waypoint_recording_report_joint_hash_");
  const auto fixture = createValidatedRecording(temporary.path());
  std::string report = readText(fixture.recording / "compile_report.yaml");
  const std::string joint_hash = hx_waypoint::RecordingStore::sha256File(
    fixture.recording / "joint_trajectory.csv");
  const auto position = report.find(joint_hash);
  ASSERT_NE(position, std::string::npos);
  report.replace(position, joint_hash.size(), std::string(64U, '0'));
  writeText(fixture.recording / "compile_report.yaml", report);
  writeChecksums(fixture.recording);
  try {
    (void)hx_waypoint::RecordingStore::loadValidatedDirectory(
      fixture.recording, loadOptions(fixture));
    FAIL() << "compile report with a different joint payload hash was accepted for playback";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("joint_trajectory_sha256"), std::string::npos);
  }
}

TEST(RecordingStore, RejectsSealedCompileReportWithDifferentPlannerHash)
{
  TemporaryTree temporary("hx_waypoint_recording_report_planner_hash_");
  const auto fixture = createValidatedRecording(temporary.path());
  std::string report = readText(fixture.recording / "compile_report.yaml");
  const std::string planner_hash = hx_waypoint::RecordingStore::sha256File(
    fixture.environment / "planner.yaml");
  const auto position = report.find(planner_hash);
  ASSERT_NE(position, std::string::npos);
  report.replace(position, planner_hash.size(), std::string(64U, '0'));
  writeText(fixture.recording / "compile_report.yaml", report);
  writeChecksums(fixture.recording);
  try {
    (void)hx_waypoint::RecordingStore::loadValidatedDirectory(
      fixture.recording, loadOptions(fixture));
    FAIL() << "compile report from a different planner environment was accepted";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("environment_hashes"), std::string::npos);
  }
}

TEST(RecordingStore, RejectsSealedWindingReferenceThatDiffersFromSnapshotHome)
{
  TemporaryTree temporary("hx_waypoint_recording_winding_home_");
  const auto fixture = createValidatedRecording(temporary.path());
  std::string report = readText(fixture.recording / "compile_report.yaml");
  const std::string original = "reference_positions_rad: [0.0, 0.0]";
  const auto position = report.find(original);
  ASSERT_NE(position, std::string::npos);
  report.replace(position, original.size(), "reference_positions_rad: [0.1, 0.0]");
  writeText(fixture.recording / "compile_report.yaml", report);
  writeChecksums(fixture.recording);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(
      fixture.recording, loadOptions(fixture)),
    std::runtime_error);
}

TEST(RecordingStore, RejectsSelfConsistentSnapshotRobotModelThatDiffersFromManifest)
{
  TemporaryTree temporary("hx_waypoint_recording_snapshot_robot_hash_");
  const auto fixture = createValidatedRecording(temporary.path());
  const auto snapshot_robot_model =
    fixture.recording / "environment_snapshot/robot/robot_model.yaml";
  std::string robot_model = readText(snapshot_robot_model);
  const std::string original_home = "home_joint_positions: [0.0, 0.0]";
  const auto home_position = robot_model.find(original_home);
  ASSERT_NE(home_position, std::string::npos);
  robot_model.replace(home_position, original_home.size(),
    "home_joint_positions: [0.1, 0.0]");
  writeText(snapshot_robot_model, robot_model);

  const auto compile_report_path = fixture.recording / "compile_report.yaml";
  std::string report = readText(compile_report_path);
  const std::string original_reference = "reference_positions_rad: [0.0, 0.0]";
  const auto reference_position = report.find(original_reference);
  ASSERT_NE(reference_position, std::string::npos);
  report.replace(reference_position, original_reference.size(),
    "reference_positions_rad: [0.1, 0.0]");
  writeText(compile_report_path, report);
  writeChecksums(fixture.recording);

  try {
    (void)hx_waypoint::RecordingStore::loadValidatedDirectory(
      fixture.recording, loadOptions(fixture));
    FAIL() << "self-consistent snapshot robot model outside the manifest binding was accepted";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("bytes do not match the manifest"), std::string::npos);
  }
}

TEST(RecordingStore, RejectsManifestValidationMetricsThatDifferFromCompileReport)
{
  TemporaryTree temporary("hx_waypoint_recording_manifest_metrics_");
  const auto fixture = createValidatedRecording(temporary.path());
  const auto manifest_path = fixture.recording / "manifest.yaml";
  std::string manifest = readText(manifest_path);
  const std::string metric = "max_joint_jerk_ratio: ";
  const auto first = manifest.find(metric);
  ASSERT_NE(first, std::string::npos);
  const auto second = manifest.find(metric, first + metric.size());
  ASSERT_NE(second, std::string::npos);
  const auto line_end = manifest.find('\n', second);
  ASSERT_NE(line_end, std::string::npos);
  manifest.replace(second, line_end - second, "max_joint_jerk_ratio: 0.0");
  writeText(manifest_path, manifest);
  writeChecksums(fixture.recording);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(
      fixture.recording, loadOptions(fixture)),
    std::runtime_error);
}

TEST(RecordingStore, RejectsValidationReportWithoutSimulationExecutionEvidence)
{
  TemporaryTree temporary("hx_waypoint_recording_simulation_contract_");
  const auto fixture = createValidatedRecording(temporary.path());
  const auto validation_path = fixture.recording / "validation_report.yaml";
  std::string validation = readText(validation_path);
  const std::string evidence =
    "speed_scaling_evidence: locked_standard_unscaled_joint_trajectory_controller\n";
  const auto position = validation.find(evidence);
  ASSERT_NE(position, std::string::npos);
  validation.erase(position, evidence.size());
  writeText(validation_path, validation);
  writeChecksums(fixture.recording);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(
      fixture.recording, loadOptions(fixture)),
    std::runtime_error);
}

TEST(RecordingStore, RejectsInvalidSourceSchemaAtPlaybackLoad)
{
  TemporaryTree temporary("hx_waypoint_invalid_source_load_");
  const auto fixture = createValidatedRecording(temporary.path());
  const auto source = fixture.recording / "source_tip_trajectory.csv";
  const std::string old_hash = hx_waypoint::RecordingStore::sha256File(source);
  writeText(source, "not,a,pose,csv\n");
  const std::string new_hash = hx_waypoint::RecordingStore::sha256File(source);
  std::string manifest = readText(fixture.recording / "manifest.yaml");
  const auto hash_position = manifest.find(old_hash);
  ASSERT_NE(hash_position, std::string::npos);
  manifest.replace(hash_position, old_hash.size(), new_hash);
  writeText(fixture.recording / "manifest.yaml", manifest);
  writeChecksums(fixture.recording);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(fixture.recording, loadOptions(fixture)),
    std::runtime_error);
}

TEST(RecordingStore, RejectsSourceTimeOffsetAtPlaybackLoad)
{
  TemporaryTree temporary("hx_waypoint_offset_source_load_");
  const auto fixture = createValidatedRecording(temporary.path());
  const auto source = fixture.recording / "source_tip_trajectory.csv";
  const std::string old_hash = hx_waypoint::RecordingStore::sha256File(source);
  writeText(
    source,
    "time_s,frame_id,tip_x_m,tip_y_m,tip_z_m,tip_qx,tip_qy,tip_qz,tip_qw,phase\n"
    "0,base,0,0,0,0,0,0,1,draw\n"
    "2,base,0.1,0,0,0,0,0,1,draw\n");
  const std::string new_hash = hx_waypoint::RecordingStore::sha256File(source);
  std::string manifest = readText(fixture.recording / "manifest.yaml");
  const auto hash_position = manifest.find(old_hash);
  ASSERT_NE(hash_position, std::string::npos);
  manifest.replace(hash_position, old_hash.size(), new_hash);
  writeText(fixture.recording / "manifest.yaml", manifest);
  writeChecksums(fixture.recording);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(fixture.recording, loadOptions(fixture)),
    std::runtime_error);
}

TEST(RecordingStore, RejectsMissingRequiredRecordingFile)
{
  TemporaryTree temporary("hx_waypoint_missing_recording_");
  const auto fixture = createValidatedRecording(temporary.path());
  std::filesystem::remove(fixture.recording / "sim_actual_tip.csv");
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(fixture.recording, loadOptions(fixture)),
    std::runtime_error);
}

TEST(RecordingStore, RejectsChecksumMismatchOutsideJointTrajectory)
{
  TemporaryTree temporary("hx_waypoint_bad_checksum_");
  const auto fixture = createValidatedRecording(temporary.path());
  {
    std::ofstream output(fixture.recording / "validation_report.yaml", std::ios::app);
    ASSERT_TRUE(output);
    output << "tampered: true\n";
  }
  try {
    (void)hx_waypoint::RecordingStore::loadValidatedDirectory(
      fixture.recording, loadOptions(fixture));
    FAIL() << "tampered validation_report.yaml was accepted";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("validation_report.yaml"), std::string::npos);
  }
}

TEST(RecordingStore, RejectsUnchecksummedArtifactFile)
{
  TemporaryTree temporary("hx_waypoint_unchecksummed_");
  const auto fixture = createValidatedRecording(temporary.path());
  writeText(fixture.recording / "unsealed.txt", "not checksummed\n");
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadValidatedDirectory(fixture.recording, loadOptions(fixture)),
    std::runtime_error);
}

TEST(RecordingStore, AcceptsControlledCompiledRunAndReturnsLockedRevision)
{
  TemporaryTree temporary("hx_waypoint_compiled_run_");
  const auto fixture = createCompiledRun(temporary.path());
  const auto evidence = hx_waypoint::RecordingStore::validateCompiledRun(
    fixture.runs_root, fixture.environment, fixture.source, fixture.trajectory, "lab", "line");
  EXPECT_EQ(evidence.run_directory, std::filesystem::canonical(fixture.run));
  EXPECT_EQ(evidence.compiler_contract_revision, kCompilerContractRevision);
  EXPECT_EQ(evidence.curobo_revision, kCuroboCommit);
  EXPECT_EQ(
    evidence.source_trajectory_sha256,
    hx_waypoint::RecordingStore::sha256File(fixture.source));
  EXPECT_EQ(
    evidence.joint_trajectory_sha256,
    hx_waypoint::RecordingStore::sha256File(fixture.trajectory));
  EXPECT_EQ(
    evidence.compile_report_sha256,
    hx_waypoint::RecordingStore::sha256File(fixture.report));
  EXPECT_EQ(
    evidence.joint_winding_policy,
    "constant_whole_path_nearest_home_without_margin_loss");
  EXPECT_EQ(evidence.joint_winding_reference_positions_rad, (std::vector<double>{0.0, 0.0}));
  EXPECT_EQ(evidence.selected_seed_offsets_rad, (std::vector<double>{0.0, 0.0}));
  EXPECT_EQ(evidence.post_optimization_offsets_rad, (std::vector<double>{0.0, 0.0}));
  EXPECT_EQ(evidence.point_count, 2U);
  EXPECT_DOUBLE_EQ(evidence.duration_s, 1.0);
  EXPECT_EQ(evidence.source_waypoint_count, 2U);
  EXPECT_EQ(evidence.matched_source_waypoint_count, 2U);
  EXPECT_DOUBLE_EQ(evidence.source_duration_s, 1.0);
  EXPECT_DOUBLE_EQ(evidence.max_source_time_error_s, 0.0);
  EXPECT_DOUBLE_EQ(evidence.uniform_time_scale, 1.0);
  EXPECT_NEAR(
    evidence.max_joint_velocity_ratio,
    fixture.dynamics.max_joint_velocity_ratio, 1.0e-12);
  EXPECT_NEAR(
    evidence.max_joint_acceleration_ratio,
    fixture.dynamics.max_joint_acceleration_ratio, 1.0e-12);
  EXPECT_NEAR(
    evidence.max_joint_jerk_ratio,
    fixture.dynamics.max_joint_jerk_ratio, 1.0e-12);
  EXPECT_NEAR(
    evidence.max_tip_position_error_m,
    fixture.tip.max_tip_position_error_m, 1.0e-12);
  EXPECT_DOUBLE_EQ(evidence.minimum_collision_distance_m, 0.01);
  EXPECT_EQ(evidence.checked_sample_count, 2U);
  EXPECT_EQ(evidence.environment_hashes.size(), environmentFiles().size() + 2U);
}

TEST(RecordingStore, RejectsCompiledJointPayloadThatChangedWithoutRetiming)
{
  TemporaryTree temporary("hx_waypoint_joint_payload_hash_");
  const auto fixture = createCompiledRun(temporary.path());
  writeText(
    fixture.trajectory,
    "time_s,joint_a_pos,joint_b_pos,joint_a_vel,joint_b_vel,joint_a_acc,joint_b_acc,phase\n"
    "0,0,1,0,0,0,0,draw\n"
    "1,0.6,1.6,0,0,0,0,draw\n");
  try {
    (void)hx_waypoint::RecordingStore::validateCompiledRun(
      fixture.runs_root, fixture.environment, fixture.source, fixture.trajectory, "lab", "line");
    FAIL() << "joint payload replacement with unchanged timing was accepted";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("joint_trajectory_sha256"), std::string::npos);
  }
}

TEST(RecordingStore, RejectsForgedLowCanonicalDynamicRatio)
{
  TemporaryTree temporary("hx_waypoint_forged_low_dynamic_ratio_");
  const auto fixture = createCompiledRun(temporary.path());
  std::string report = readText(fixture.report);
  const std::string key = "max_joint_velocity_ratio: ";
  const auto position = report.find(key);
  ASSERT_NE(position, std::string::npos);
  const auto line_end = report.find('\n', position);
  ASSERT_NE(line_end, std::string::npos);
  report.replace(position, line_end - position, key + "0.0");
  writeText(fixture.report, report);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::validateCompiledRun(
      fixture.runs_root, fixture.environment, fixture.source, fixture.trajectory,
      "lab", "line"),
    std::runtime_error);
}

TEST(RecordingStore, RejectsInteriorJointBoundViolationWithUpdatedCompileHash)
{
  TemporaryTree temporary("hx_waypoint_interior_bound_violation_");
  const auto fixture = createCompiledRun(temporary.path());
  const std::string old_hash = hx_waypoint::RecordingStore::sha256File(fixture.trajectory);
  writeText(
    fixture.trajectory,
    "time_s,joint_a_pos,joint_b_pos,joint_a_vel,joint_b_vel,joint_a_acc,joint_b_acc,phase\n"
    "0,0,1,0,0,0,0,draw\n"
    "0.5,10,1.25,0,0,0,0,draw\n"
    "1,0.5,1.5,0,0,0,0,draw\n");
  const std::string new_hash = hx_waypoint::RecordingStore::sha256File(fixture.trajectory);
  std::string report = readText(fixture.report);
  ASSERT_EQ(replaceAll(report, old_hash, new_hash), 1U);
  ASSERT_EQ(replaceAll(report, "\npoint_count: 2\n", "\npoint_count: 3\n"), 1U);
  ASSERT_EQ(
    replaceAll(
      report, "\nchecked_sample_count: 2\n", "\nchecked_sample_count: 3\n"),
    1U);
  writeText(fixture.report, report);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::validateCompiledRun(
      fixture.runs_root, fixture.environment, fixture.source, fixture.trajectory,
      "lab", "line"),
    std::runtime_error);
}

TEST(RecordingStore, RejectsInvalidJointWindingEvidence)
{
  TemporaryTree temporary("hx_waypoint_joint_winding_evidence_");
  const auto fixture = createCompiledRun(temporary.path());
  std::string report = readText(fixture.report);
  const std::string original = "selected_seed_offsets_rad: [0.0, 0.0]";
  const auto position = report.find(original);
  ASSERT_NE(position, std::string::npos);
  report.replace(position, original.size(), "selected_seed_offsets_rad: [0.1, 0.0]");
  writeText(fixture.report, report);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::validateCompiledRun(
      fixture.runs_root, fixture.environment, fixture.source, fixture.trajectory, "lab", "line"),
    std::runtime_error);
}

TEST(RecordingStore, RejectsCanonicalPayloadWithoutEndpointRestOrStaticHold)
{
  TemporaryTree temporary("hx_waypoint_canonical_derivative_contract_");
  const auto trajectory = temporary.path() / "joint_trajectory.csv";
  const std::vector<std::string> joints{"joint_a", "joint_b"};

  writeText(
    trajectory,
    "time_s,joint_a_pos,joint_b_pos,joint_a_vel,joint_b_vel,joint_a_acc,joint_b_acc,phase\n"
    "0,0,1,0.1,0,0,0,draw\n"
    "1,0.5,1.5,0,0,0,0,draw\n");
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadJointTrajectoryCsv(trajectory, joints),
    std::runtime_error);

  writeText(
    trajectory,
    "time_s,joint_a_pos,joint_b_pos,joint_a_vel,joint_b_vel,joint_a_acc,joint_b_acc,phase\n"
    "0,0,1,0,0,0,0,draw\n"
    "1,0.1,1,0,0,0,0,hold\n");
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadJointTrajectoryCsv(trajectory, joints),
    std::runtime_error);

  writeText(
    trajectory,
    "time_s,joint_a_pos,joint_b_pos,joint_a_vel,joint_b_vel,joint_a_acc,joint_b_acc,phase\n"
    "0,0,1,0,0,0,0,hold\n"
    "1,0.5,1.5,0,0,0,0,draw\n");
  EXPECT_THROW(
    hx_waypoint::RecordingStore::loadJointTrajectoryCsv(trajectory, joints),
    std::runtime_error);
}

TEST(RecordingStore, RejectsCompiledTrajectoryOutsideControlledRunsRoot)
{
  TemporaryTree temporary("hx_waypoint_external_compiled_");
  const auto fixture = createCompiledRun(temporary.path());
  const auto external = temporary.path() / "external" / "joint_trajectory.csv";
  writeText(external, sampleJointTrajectoryCsv());
  EXPECT_THROW(
    hx_waypoint::RecordingStore::validateCompiledRun(
      fixture.runs_root, fixture.environment, fixture.source, external, "lab", "line"),
    std::runtime_error);
}

TEST(RecordingStore, RejectsCompileReportRevisionThatDiffersFromLock)
{
  TemporaryTree temporary("hx_waypoint_revision_mismatch_");
  const auto fixture = createCompiledRun(temporary.path());
  writeText(
    fixture.report,
    "schema_version: 1\nenvironment_id: lab\ntrajectory_id: line\n"
    "curobo_revision: 0000000000000000000000000000000000000000\n"
    "duration_s: 1.0\npoint_count: 2\n");
  EXPECT_THROW(
    hx_waypoint::RecordingStore::validateCompiledRun(
      fixture.runs_root, fixture.environment, fixture.source, fixture.trajectory, "lab", "line"),
    std::runtime_error);
}

TEST(RecordingStore, RejectsCompiledRunAfterPlannerConfigurationChanges)
{
  TemporaryTree temporary("hx_waypoint_planner_hash_mismatch_");
  const auto fixture = createCompiledRun(temporary.path());
  writeText(
    fixture.environment / "planner.yaml",
    "schema_version: 1\nlimits:\n  allow_uniform_time_scale: false\n");
  EXPECT_THROW(
    hx_waypoint::RecordingStore::validateCompiledRun(
      fixture.runs_root, fixture.environment, fixture.source, fixture.trajectory, "lab", "line"),
    std::runtime_error);
}

TEST(RecordingStore, RejectsUniformScaleWhenCurrentPlannerPolicyForbidsIt)
{
  TemporaryTree temporary("hx_waypoint_uniform_scale_policy_");
  const auto fixture = createCompiledRun(temporary.path());
  std::string report = readText(fixture.report);
  const std::string original = "uniform_time_scale: 1.0";
  const auto position = report.find(original);
  ASSERT_NE(position, std::string::npos);
  report.replace(position, original.size(), "uniform_time_scale: 2.0");
  writeText(fixture.report, report);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::validateCompiledRun(
      fixture.runs_root, fixture.environment, fixture.source, fixture.trajectory, "lab", "line"),
    std::runtime_error);
}

TEST(RecordingStore, RejectsAuthoritativeSourceTimeOffsetEvenWithUnitScale)
{
  TemporaryTree temporary("hx_waypoint_source_time_offset_");
  const auto fixture = createCompiledRun(temporary.path());
  writeText(
    fixture.trajectory,
    "time_s,joint_a_pos,joint_b_pos,joint_a_vel,joint_b_vel,joint_a_acc,joint_b_acc,phase\n"
    "0,0,1,0,0,0,0,approach\n"
    "1,0,1,0,0,0,0,draw\n"
    "2,0.5,1.5,0,0,0,0,draw\n");
  EXPECT_THROW(
    hx_waypoint::RecordingStore::validateCompiledRun(
      fixture.runs_root, fixture.environment, fixture.source, fixture.trajectory, "lab", "line"),
    std::runtime_error);
}

TEST(RecordingStore, RejectsPhaseChangeAtAuthoritativeWaypoint)
{
  TemporaryTree temporary("hx_waypoint_source_phase_change_");
  const auto fixture = createCompiledRun(temporary.path());
  std::string trajectory = readText(fixture.trajectory);
  const std::string original = ",draw\n";
  const auto position = trajectory.find(original);
  ASSERT_NE(position, std::string::npos);
  trajectory.replace(position, original.size(), ",engage\n");
  writeText(fixture.trajectory, trajectory);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::validateCompiledRun(
      fixture.runs_root, fixture.environment, fixture.source, fixture.trajectory, "lab", "line"),
    std::runtime_error);
}

TEST(RecordingStore, RejectsMissingOrExtraCompileEnvironmentHashes)
{
  TemporaryTree missing_temporary("hx_waypoint_missing_environment_hash_");
  const auto missing_fixture = createCompiledRun(missing_temporary.path());
  std::string missing_report = readText(missing_fixture.report);
  const std::string missing_prefix = "  setup.yaml: ";
  const auto missing_begin = missing_report.find(missing_prefix);
  ASSERT_NE(missing_begin, std::string::npos);
  const auto missing_end = missing_report.find('\n', missing_begin);
  missing_report.erase(missing_begin, missing_end - missing_begin + 1U);
  writeText(missing_fixture.report, missing_report);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::validateCompiledRun(
      missing_fixture.runs_root, missing_fixture.environment, missing_fixture.source,
      missing_fixture.trajectory, "lab", "line"),
    std::runtime_error);

  TemporaryTree extra_temporary("hx_waypoint_extra_environment_hash_");
  const auto extra_fixture = createCompiledRun(extra_temporary.path());
  std::string extra_report = readText(extra_fixture.report);
  const std::string map_header = "environment_hashes:\n";
  const auto insert_at = extra_report.find(map_header);
  ASSERT_NE(insert_at, std::string::npos);
  extra_report.insert(
    insert_at + map_header.size(),
    "  extra.yaml: 0000000000000000000000000000000000000000000000000000000000000000\n");
  writeText(extra_fixture.report, extra_report);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::validateCompiledRun(
      extra_fixture.runs_root, extra_fixture.environment, extra_fixture.source,
      extra_fixture.trajectory, "lab", "line"),
    std::runtime_error);
}

TEST(RecordingStore, RejectsIncompleteOrInfeasibleCompileValidationMetrics)
{
  TemporaryTree count_temporary("hx_waypoint_checked_count_");
  const auto count_fixture = createCompiledRun(count_temporary.path());
  std::string count_report = readText(count_fixture.report);
  const auto count_position = count_report.find("checked_sample_count: 2");
  ASSERT_NE(count_position, std::string::npos);
  count_report.replace(
    count_position, std::string("checked_sample_count: 2").size(),
    "checked_sample_count: 1");
  writeText(count_fixture.report, count_report);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::validateCompiledRun(
      count_fixture.runs_root, count_fixture.environment, count_fixture.source,
      count_fixture.trajectory, "lab", "line"),
    std::runtime_error);

  TemporaryTree cost_temporary("hx_waypoint_collision_cost_");
  const auto cost_fixture = createCompiledRun(cost_temporary.path());
  std::string cost_report = readText(cost_fixture.report);
  const auto cost_position = cost_report.find("max_scene_collision_cost: 0.0");
  ASSERT_NE(cost_position, std::string::npos);
  cost_report.replace(
    cost_position, std::string("max_scene_collision_cost: 0.0").size(),
    "max_scene_collision_cost: 0.001");
  writeText(cost_fixture.report, cost_report);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::validateCompiledRun(
      cost_fixture.runs_root, cost_fixture.environment, cost_fixture.source,
      cost_fixture.trajectory, "lab", "line"),
    std::runtime_error);

  TemporaryTree tip_temporary("hx_waypoint_tip_error_");
  const auto tip_fixture = createCompiledRun(tip_temporary.path());
  std::string tip_report = readText(tip_fixture.report);
  const std::string tip_key = "max_tip_position_error_m: ";
  const auto tip_position = tip_report.find(tip_key);
  ASSERT_NE(tip_position, std::string::npos);
  const auto tip_line_end = tip_report.find('\n', tip_position);
  ASSERT_NE(tip_line_end, std::string::npos);
  tip_report.replace(
    tip_position, tip_line_end - tip_position,
    "max_tip_position_error_m: 0.01");
  writeText(tip_fixture.report, tip_report);
  EXPECT_THROW(
    hx_waypoint::RecordingStore::validateCompiledRun(
      tip_fixture.runs_root, tip_fixture.environment, tip_fixture.source,
      tip_fixture.trajectory, "lab", "line"),
    std::runtime_error);
}

}  // namespace
