#include "hx_waypoint/telemetry_recorder.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "hx_waypoint/setup_config.hpp"

namespace hx_waypoint
{
namespace
{

[[nodiscard]] double durationSeconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1.0e-9;
}

[[nodiscard]] bool pathWithin(
  const std::filesystem::path & candidate,
  const std::filesystem::path & root)
{
  if (candidate == root) {
    return true;
  }
  const auto relative = candidate.lexically_relative(root);
  return !relative.empty() && !relative.is_absolute() && *relative.begin() != "..";
}

[[nodiscard]] std::unique_ptr<TipKinematics> loadTipKinematics(
  const std::filesystem::path & snapshot,
  const std::vector<std::string> & joint_names,
  const TelemetryRecorderOptions & options)
{
  if (!std::filesystem::is_directory(snapshot)) {
    throw std::runtime_error("recording environment snapshot is unavailable: " + snapshot.string());
  }
  const auto root = std::filesystem::canonical(snapshot);
  const auto robot_model = YAML::LoadFile((root / "robot/robot_model.yaml").string());
  const auto curobo_robot = YAML::LoadFile((root / "robot/curobo_robot.yaml").string());
  const auto setup = loadSetupConfig(root / "setup.yaml");
  if (robot_model["schema_version"].as<int>() != 1 ||
    curobo_robot["schema_version"].as<int>() != 1)
  {
    throw std::runtime_error("telemetry kinematics configuration schema_version must be 1");
  }

  const std::string base_link = robot_model["robot"]["base_link"].as<std::string>();
  const std::string tool_link = setup.tool.parent_link;
  const std::string tip_frame = setup.tool.tip_frame;
  if (base_link != options.base_frame || tip_frame != options.tip_frame) {
    throw std::runtime_error(
            "recording snapshot base/tip frame does not match the runtime telemetry configuration");
  }
  const auto tool_to_tip = setup.tool.parentToTip();
  const auto translation = tool_to_tip.translation_m;
  const auto quaternion = tool_to_tip.quaternion_xyzw;

  const YAML::Node kinematics = curobo_robot["robot_cfg"]["kinematics"];
  std::filesystem::path asset_root = kinematics["asset_root_path"].as<std::string>();
  if (!asset_root.is_absolute()) {
    asset_root = root / asset_root;
  }
  asset_root = std::filesystem::weakly_canonical(asset_root);
  if (!pathWithin(asset_root, root)) {
    throw std::runtime_error("telemetry robot asset_root_path escapes the recording snapshot");
  }
  std::filesystem::path urdf_path = kinematics["urdf_path"].as<std::string>();
  if (!urdf_path.is_absolute()) {
    urdf_path = asset_root / urdf_path;
  }
  urdf_path = std::filesystem::weakly_canonical(urdf_path);
  if (!pathWithin(urdf_path, asset_root) || !std::filesystem::is_regular_file(urdf_path)) {
    throw std::runtime_error("telemetry URDF is missing or escapes asset_root_path");
  }

  return std::make_unique<TipKinematics>(
    urdf_path, base_link, tool_link, tip_frame, translation, quaternion, joint_names);
}

[[nodiscard]] double vectorNorm(const std::array<double, 3> & values)
{
  return std::sqrt(
    values[0] * values[0] + values[1] * values[1] + values[2] * values[2]);
}

[[nodiscard]] std::string speedScalingViolationMessage(
  double speed_scaling,
  double tolerance)
{
  std::ostringstream message;
  message << std::setprecision(17) <<
    "controller speed_scaling must remain 1.0 during authoritative recording playback "
    "(tolerance " << tolerance << "); received " << speed_scaling <<
    ". The complete trajectory goal must be canceled; controller-side scaling and retiming "
    "are forbidden.";
  return message.str();
}

[[nodiscard]] std::string speedScalingFreshnessViolationMessage(
  std::chrono::nanoseconds timeout,
  std::chrono::steady_clock::duration elapsed)
{
  std::ostringstream message;
  message << std::setprecision(17) <<
    "controller speed_scaling stopped publishing fresh samples during authoritative "
    "recording playback; freshness timeout " <<
    std::chrono::duration<double>(timeout).count() << " s, elapsed " <<
    std::chrono::duration<double>(elapsed).count() <<
    " s. The complete trajectory goal must be canceled; missing speed-scaling evidence, "
    "controller-side scaling, and retiming are forbidden.";
  return message.str();
}

}  // namespace

bool isUnitSpeedScaling(double speed_scaling, double tolerance) noexcept
{
  return std::isfinite(speed_scaling) && std::isfinite(tolerance) && tolerance >= 0.0 &&
         std::abs(speed_scaling - 1.0) <= tolerance;
}

double urSpeedScalingPercentToRatio(double speed_scaling_percent) noexcept
{
  // ur_controllers/SpeedScalingStateBroadcaster publishes percent, so 100.0
  // is the unscaled state.  Keep the rest of the runtime and the telemetry
  // schema in the documented unit ratio [0, 1].  Deliberately do not clamp:
  // non-finite, negative, and over-100-percent samples must remain invalid and
  // be rejected by the strict guard.
  return speed_scaling_percent * 0.01;
}

DesiredJointState sampleDesiredJointState(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  double trajectory_time_s)
{
  if (trajectory.joint_names.empty() || trajectory.points.empty() ||
    !std::isfinite(trajectory_time_s))
  {
    throw std::invalid_argument("desired trajectory and sample time must be valid");
  }
  const auto width = trajectory.joint_names.size();
  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    const auto & point = trajectory.points[index];
    if (point.positions.size() != width || point.velocities.size() != width ||
      point.accelerations.size() != width)
    {
      throw std::invalid_argument("desired trajectory must contain complete q/qd/qdd vectors");
    }
    if (index > 0U && durationSeconds(point.time_from_start) <=
      durationSeconds(trajectory.points[index - 1U].time_from_start))
    {
      throw std::invalid_argument("desired trajectory time must be strictly increasing");
    }
  }

  const auto endpoint = [](const trajectory_msgs::msg::JointTrajectoryPoint & point) {
      return DesiredJointState{point.positions, point.velocities};
    };
  const auto & points = trajectory.points;
  if (trajectory_time_s <= durationSeconds(points.front().time_from_start)) {
    return endpoint(points.front());
  }
  if (trajectory_time_s >= durationSeconds(points.back().time_from_start)) {
    return endpoint(points.back());
  }

  const auto upper = std::upper_bound(
    points.begin(), points.end(), trajectory_time_s,
    [](double time, const trajectory_msgs::msg::JointTrajectoryPoint & point) {
      return time < durationSeconds(point.time_from_start);
    });
  const auto & after = *upper;
  const auto & before = *(upper - 1);
  const double before_time = durationSeconds(before.time_from_start);
  const double duration = durationSeconds(after.time_from_start) - before_time;
  const double elapsed = trajectory_time_s - before_time;
  const double elapsed2 = elapsed * elapsed;
  const double elapsed3 = elapsed2 * elapsed;
  const double elapsed4 = elapsed3 * elapsed;
  const double elapsed5 = elapsed4 * elapsed;
  const double duration2 = duration * duration;
  const double duration3 = duration2 * duration;
  const double duration4 = duration3 * duration;
  const double duration5 = duration4 * duration;

  DesiredJointState desired;
  desired.positions.reserve(width);
  desired.velocities.reserve(width);
  for (std::size_t index = 0; index < width; ++index) {
    const double c0 = before.positions[index];
    const double c1 = before.velocities[index];
    const double c2 = before.accelerations[index] * 0.5;
    const double delta_q = after.positions[index] -
      (c0 + c1 * duration + c2 * duration2);
    const double delta_v = after.velocities[index] - (c1 + 2.0 * c2 * duration);
    const double delta_a = after.accelerations[index] - 2.0 * c2;
    const double c3 = 10.0 * delta_q / duration3 - 4.0 * delta_v / duration2 +
      0.5 * delta_a / duration;
    const double c4 = -15.0 * delta_q / duration4 + 7.0 * delta_v / duration3 -
      delta_a / duration2;
    const double c5 = 6.0 * delta_q / duration5 - 3.0 * delta_v / duration4 +
      0.5 * delta_a / duration3;
    desired.positions.push_back(
      c0 + c1 * elapsed + c2 * elapsed2 + c3 * elapsed3 + c4 * elapsed4 + c5 * elapsed5);
    desired.velocities.push_back(
      c1 + 2.0 * c2 * elapsed + 3.0 * c3 * elapsed2 + 4.0 * c4 * elapsed3 +
      5.0 * c5 * elapsed4);
  }
  return desired;
}

TelemetryRecorder::TelemetryRecorder(
  rclcpp::Node * node,
  std::vector<std::string> joint_names,
  TelemetryRecorderOptions options)
: node_(node), joint_names_(std::move(joint_names)), options_(std::move(options))
{
  if (node_ == nullptr || joint_names_.empty() || options_.joint_states_topic.empty() ||
    options_.telemetry_topic.empty())
  {
    throw std::invalid_argument("TelemetryRecorder requires node, joint names, and telemetry topics");
  }
  if (options_.base_frame.empty() || options_.tip_frame.empty()) {
    throw std::invalid_argument("TelemetryRecorder base_frame and tip_frame must not be empty");
  }
  latest_tip_pose_.orientation.w = 1.0;
  position_error_m_ = std::numeric_limits<double>::quiet_NaN();
  publisher_ = node_->create_publisher<hx_waypoint_interfaces::msg::ExecutionTelemetry>(
    options_.telemetry_topic, rclcpp::QoS(10));
  joint_callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions joint_subscription_options;
  joint_subscription_options.callback_group = joint_callback_group_;
  joint_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    options_.joint_states_topic, rclcpp::SensorDataQoS(),
    std::bind(&TelemetryRecorder::onJointState, this, std::placeholders::_1),
    joint_subscription_options);
  if (!options_.speed_scaling_topic.empty()) {
    speed_scaling_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = speed_scaling_callback_group_;
    speed_scaling_subscription_ = node_->create_subscription<std_msgs::msg::Float64>(
      options_.speed_scaling_topic, rclcpp::QoS(10),
      std::bind(&TelemetryRecorder::onSpeedScaling, this, std::placeholders::_1),
      subscription_options);
  }
}

TelemetryRecorder::~TelemetryRecorder()
{
  clearStrictSpeedScalingGuard();
}

bool TelemetryRecorder::prepare(
  const trajectory_msgs::msg::JointTrajectory & desired_trajectory,
  const std::filesystem::path & environment_snapshot,
  std::string & error)
{
  error.clear();
  if (desired_trajectory.joint_names != joint_names_ || desired_trajectory.points.empty()) {
    error = "telemetry trajectory joint order is invalid";
    return false;
  }
  try {
    auto kinematics = loadTipKinematics(environment_snapshot, joint_names_, options_);
    const auto initial = sampleDesiredJointState(desired_trajectory, 0.0);
    (void)kinematics->evaluate(initial.positions, initial.velocities);
    const double duration = durationSeconds(desired_trajectory.points.back().time_from_start);
    if (!std::isfinite(duration) || duration <= 0.0) {
      throw std::runtime_error("telemetry trajectory duration must be positive");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (active_.load(std::memory_order_acquire)) {
      error = "telemetry recorder is already active";
      return false;
    }
    desired_trajectory_ = desired_trajectory;
    trajectory_duration_s_ = duration;
    tip_kinematics_ = std::move(kinematics);
    prepared_ = true;
    return true;
  } catch (const std::exception & exception) {
    error = exception.what();
    return false;
  }
}

bool TelemetryRecorder::start(
  const std::filesystem::path & output_file,
  const rclcpp::Time & trajectory_start,
  const std::string & state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_.load(std::memory_order_acquire) || !prepared_ || !tip_kinematics_) {
    return false;
  }
  if (!output_file.parent_path().empty()) {
    std::filesystem::create_directories(output_file.parent_path());
  }
  output_.open(output_file);
  if (!output_) {
    return false;
  }
  output_file_ = output_file;
  trajectory_start_ = trajectory_start;
  last_feedback_received_at_ = trajectory_start;
  trajectory_time_s_ = 0.0;
  trajectory_time_ns_ = 0;
  state_ = state;
  desired_tip_speed_mps_ = 0.0;
  position_error_m_ = std::numeric_limits<double>::quiet_NaN();
  latest_tip_pose_ = geometry_msgs::msg::Pose();
  latest_tip_pose_.orientation.w = 1.0;
  latest_tip_twist_ = geometry_msgs::msg::Twist();

  output_ <<
    "ros_time_s,sync_time_ns,ros_time_ns,trajectory_time_s,trajectory_time_ns,state";
  for (const auto & name : joint_names_) {output_ << ",desired_" << name << "_pos";}
  for (const auto & name : joint_names_) {output_ << ',' << name << "_pos";}
  for (const auto & name : joint_names_) {output_ << ',' << name << "_vel";}
  output_ <<
    ",tip_x,tip_y,tip_z,tip_vx,tip_vy,tip_vz,actual_tip_speed_mps"
    ",tip_wx,tip_wy,tip_wz,actual_tip_angular_speed_radps"
    ",desired_tip_speed_mps,position_error_m,speed_scaling\n";
  output_ << std::setprecision(std::numeric_limits<double>::max_digits10);
  active_.store(true, std::memory_order_release);
  return true;
}

std::filesystem::path TelemetryRecorder::stop(const std::string & final_state)
{
  // Stop new high-rate JointState callbacks before waiting for the shared
  // state lock.  Otherwise a 125 Hz callback can repeatedly reacquire the
  // mutex while doing FK and CSV formatting, starving action finalization.
  active_.store(false, std::memory_order_release);
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = final_state;
  prepared_ = false;
  tip_kinematics_.reset();
  if (output_.is_open()) {
    output_.flush();
    output_.close();
  }
  return output_file_;
}

void TelemetryRecorder::setSpeedScaling(double speed_scaling)
{
  SpeedScalingViolationCallback violation_callback;
  std::string violation;
  rclcpp::TimerBase::SharedPtr watchdog_to_cancel;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_speed_scaling_sample_ = speed_scaling;
    latest_speed_scaling_received_at_ = std::chrono::steady_clock::now();
    ++speed_scaling_sequence_;
    if (std::isfinite(speed_scaling) && speed_scaling >= 0.0) {
      speed_scaling_ = speed_scaling;
    }
    if (strict_speed_scaling_guard_armed_ &&
      !isUnitSpeedScaling(speed_scaling, strict_speed_scaling_tolerance_))
    {
      if (strict_speed_scaling_violation_.empty()) {
        strict_speed_scaling_violation_ = speedScalingViolationMessage(
          speed_scaling, strict_speed_scaling_tolerance_);
      }
      if (strict_speed_scaling_guard_active_ && speed_scaling_violation_callback_) {
        violation_callback = speed_scaling_violation_callback_;
        violation = strict_speed_scaling_violation_;
        speed_scaling_violation_callback_ = {};
        strict_speed_scaling_guard_active_ = false;
        watchdog_to_cancel = std::move(speed_scaling_watchdog_);
      }
    }
  }
  speed_scaling_condition_.notify_all();
  if (watchdog_to_cancel) {
    watchdog_to_cancel->cancel();
  }
  if (violation_callback) {
    violation_callback(violation);
  }
}

bool TelemetryRecorder::armStrictSpeedScalingGuard(
  std::chrono::nanoseconds fresh_sample_timeout,
  double tolerance,
  std::string & error)
{
  error.clear();
  if (fresh_sample_timeout <= std::chrono::nanoseconds::zero() ||
    !std::isfinite(tolerance) || tolerance < 0.0)
  {
    error =
      "strict speed_scaling timeout must be positive and tolerance must be finite and non-negative";
    return false;
  }

  std::unique_lock<std::mutex> lock(mutex_);
  if (speed_scaling_watchdog_) {
    speed_scaling_watchdog_->cancel();
    speed_scaling_watchdog_.reset();
  }
  strict_speed_scaling_guard_armed_ = true;
  strict_speed_scaling_guard_active_ = false;
  strict_speed_scaling_tolerance_ = tolerance;
  strict_speed_scaling_fresh_timeout_ = fresh_sample_timeout;
  strict_speed_scaling_violation_.clear();
  speed_scaling_violation_callback_ = {};
  const std::uint64_t baseline_sequence = speed_scaling_sequence_;
  if (!speed_scaling_condition_.wait_for(
      lock, fresh_sample_timeout,
      [this, baseline_sequence]() {return speed_scaling_sequence_ > baseline_sequence;}))
  {
    strict_speed_scaling_guard_armed_ = false;
    error = "no fresh speed_scaling sample was received on " +
      options_.speed_scaling_topic + " before authoritative recording playback";
    return false;
  }
  if (!strict_speed_scaling_violation_.empty()) {
    error = strict_speed_scaling_violation_;
    strict_speed_scaling_guard_armed_ = false;
    strict_speed_scaling_violation_.clear();
    return false;
  }
  if (!isUnitSpeedScaling(latest_speed_scaling_sample_, tolerance)) {
    error = speedScalingViolationMessage(latest_speed_scaling_sample_, tolerance);
    strict_speed_scaling_guard_armed_ = false;
    return false;
  }
  return true;
}

bool TelemetryRecorder::checkArmedStrictSpeedScalingGuard(std::string & error) const
{
  error.clear();
  std::lock_guard<std::mutex> lock(mutex_);
  if (!strict_speed_scaling_guard_armed_) {
    error = "strict speed_scaling guard was not armed with a fresh unit sample";
    return false;
  }
  if (!strict_speed_scaling_violation_.empty()) {
    error = strict_speed_scaling_violation_;
    return false;
  }
  if (!isUnitSpeedScaling(latest_speed_scaling_sample_, strict_speed_scaling_tolerance_)) {
    error = speedScalingViolationMessage(
      latest_speed_scaling_sample_, strict_speed_scaling_tolerance_);
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  if (latest_speed_scaling_received_at_ == std::chrono::steady_clock::time_point{} ||
    now - latest_speed_scaling_received_at_ >= strict_speed_scaling_fresh_timeout_)
  {
    const auto elapsed = latest_speed_scaling_received_at_ ==
      std::chrono::steady_clock::time_point{} ?
      strict_speed_scaling_fresh_timeout_ : now - latest_speed_scaling_received_at_;
    error = speedScalingFreshnessViolationMessage(
      strict_speed_scaling_fresh_timeout_, elapsed);
    return false;
  }
  return true;
}

bool TelemetryRecorder::activateStrictSpeedScalingGuard(
  SpeedScalingViolationCallback on_violation,
  std::string & error)
{
  error.clear();
  if (!on_violation) {
    error = "strict speed_scaling violation callback must be provided";
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!strict_speed_scaling_guard_armed_) {
    error = "strict speed_scaling guard was not armed with a fresh unit sample";
    return false;
  }
  if (!strict_speed_scaling_violation_.empty()) {
    error = strict_speed_scaling_violation_;
    strict_speed_scaling_guard_armed_ = false;
    strict_speed_scaling_guard_active_ = false;
    strict_speed_scaling_violation_.clear();
    speed_scaling_violation_callback_ = {};
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  if (latest_speed_scaling_received_at_ == std::chrono::steady_clock::time_point{} ||
    now - latest_speed_scaling_received_at_ >= strict_speed_scaling_fresh_timeout_)
  {
    const auto elapsed = latest_speed_scaling_received_at_ ==
      std::chrono::steady_clock::time_point{} ?
      strict_speed_scaling_fresh_timeout_ : now - latest_speed_scaling_received_at_;
    error = speedScalingFreshnessViolationMessage(
      strict_speed_scaling_fresh_timeout_, elapsed);
    strict_speed_scaling_guard_armed_ = false;
    return false;
  }
  speed_scaling_violation_callback_ = std::move(on_violation);
  strict_speed_scaling_guard_active_ = true;
  auto watchdog_period = std::min(
    strict_speed_scaling_fresh_timeout_ / 4,
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::milliseconds(100)));
  watchdog_period = std::max(
    watchdog_period,
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::milliseconds(1)));
  try {
    speed_scaling_watchdog_ = node_->create_wall_timer(
      watchdog_period,
      std::bind(&TelemetryRecorder::checkStrictSpeedScalingFreshness, this),
      speed_scaling_callback_group_);
  } catch (const std::exception & exception) {
    speed_scaling_violation_callback_ = {};
    strict_speed_scaling_guard_active_ = false;
    strict_speed_scaling_guard_armed_ = false;
    error = std::string("failed to start strict speed_scaling freshness watchdog: ") +
      exception.what();
    return false;
  }
  return true;
}

void TelemetryRecorder::checkStrictSpeedScalingFreshness()
{
  SpeedScalingViolationCallback violation_callback;
  std::string violation;
  rclcpp::TimerBase::SharedPtr watchdog_to_cancel;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!strict_speed_scaling_guard_armed_ || !strict_speed_scaling_guard_active_ ||
      strict_speed_scaling_fresh_timeout_ <= std::chrono::nanoseconds::zero())
    {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = now - latest_speed_scaling_received_at_;
    if (elapsed < strict_speed_scaling_fresh_timeout_) {
      return;
    }
    if (strict_speed_scaling_violation_.empty()) {
      strict_speed_scaling_violation_ = speedScalingFreshnessViolationMessage(
        strict_speed_scaling_fresh_timeout_, elapsed);
    }
    violation_callback = speed_scaling_violation_callback_;
    violation = strict_speed_scaling_violation_;
    speed_scaling_violation_callback_ = {};
    strict_speed_scaling_guard_active_ = false;
    watchdog_to_cancel = std::move(speed_scaling_watchdog_);
  }
  if (watchdog_to_cancel) {
    watchdog_to_cancel->cancel();
  }
  if (violation_callback) {
    violation_callback(violation);
  }
}

bool TelemetryRecorder::finalizeStrictSpeedScalingGuard(std::string & violation)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (strict_speed_scaling_guard_armed_ && strict_speed_scaling_guard_active_ &&
    strict_speed_scaling_fresh_timeout_ > std::chrono::nanoseconds::zero())
  {
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = now - latest_speed_scaling_received_at_;
    if (elapsed >= strict_speed_scaling_fresh_timeout_ &&
      strict_speed_scaling_violation_.empty())
    {
      strict_speed_scaling_violation_ = speedScalingFreshnessViolationMessage(
        strict_speed_scaling_fresh_timeout_, elapsed);
    }
  }
  violation = strict_speed_scaling_violation_;
  if (speed_scaling_watchdog_) {
    speed_scaling_watchdog_->cancel();
    speed_scaling_watchdog_.reset();
  }
  strict_speed_scaling_guard_armed_ = false;
  strict_speed_scaling_guard_active_ = false;
  strict_speed_scaling_violation_.clear();
  speed_scaling_violation_callback_ = {};
  return violation.empty();
}

void TelemetryRecorder::clearStrictSpeedScalingGuard()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (speed_scaling_watchdog_) {
    speed_scaling_watchdog_->cancel();
    speed_scaling_watchdog_.reset();
  }
  strict_speed_scaling_guard_armed_ = false;
  strict_speed_scaling_guard_active_ = false;
  strict_speed_scaling_violation_.clear();
  speed_scaling_violation_callback_ = {};
}

void TelemetryRecorder::updateTrajectoryProgress(
  double trajectory_time_s,
  const rclcpp::Time & received_at)
{
  if (!std::isfinite(trajectory_time_s) || trajectory_time_s < 0.0) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_.load(std::memory_order_acquire)) {
    return;
  }
  trajectory_time_s_ = std::clamp(trajectory_time_s, 0.0, trajectory_duration_s_);
  trajectory_time_ns_ = static_cast<std::int64_t>(std::llround(trajectory_time_s_ * 1.0e9));
  last_feedback_received_at_ = received_at;
}

bool TelemetryRecorder::active() const
{
  return active_.load(std::memory_order_acquire);
}

void TelemetryRecorder::onJointState(const sensor_msgs::msg::JointState::SharedPtr message)
{
  if (!active_.load(std::memory_order_acquire)) {
    return;
  }
  const auto sync_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  const rclcpp::Time now = node_->now();
  if (message->name.size() != message->position.size() ||
    message->name.size() != message->velocity.size())
  {
    return;
  }
  std::unordered_map<std::string, std::size_t> source_index;
  source_index.reserve(message->name.size());
  for (std::size_t index = 0; index < message->name.size(); ++index) {
    if (!source_index.emplace(message->name[index], index).second) {
      return;
    }
  }
  std::vector<double> actual_positions;
  std::vector<double> actual_velocities;
  actual_positions.reserve(joint_names_.size());
  actual_velocities.reserve(joint_names_.size());
  for (const auto & name : joint_names_) {
    const auto found = source_index.find(name);
    if (found == source_index.end() || !std::isfinite(message->position[found->second]) ||
      !std::isfinite(message->velocity[found->second]))
    {
      return;
    }
    actual_positions.push_back(message->position[found->second]);
    actual_velocities.push_back(message->velocity[found->second]);
  }

  hx_waypoint_interfaces::msg::ExecutionTelemetry telemetry;
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_.load(std::memory_order_acquire) || !tip_kinematics_) {
      return;
    }
    const auto desired = sampleDesiredJointState(desired_trajectory_, trajectory_time_s_);
    const auto actual_tip = tip_kinematics_->evaluate(actual_positions, actual_velocities);
    const auto desired_tip = tip_kinematics_->evaluate(desired.positions, desired.velocities);

    latest_tip_pose_.position.x = actual_tip.position_m[0];
    latest_tip_pose_.position.y = actual_tip.position_m[1];
    latest_tip_pose_.position.z = actual_tip.position_m[2];
    latest_tip_pose_.orientation.x = actual_tip.quaternion_xyzw[0];
    latest_tip_pose_.orientation.y = actual_tip.quaternion_xyzw[1];
    latest_tip_pose_.orientation.z = actual_tip.quaternion_xyzw[2];
    latest_tip_pose_.orientation.w = actual_tip.quaternion_xyzw[3];
    latest_tip_twist_.linear.x = actual_tip.linear_velocity_m_s[0];
    latest_tip_twist_.linear.y = actual_tip.linear_velocity_m_s[1];
    latest_tip_twist_.linear.z = actual_tip.linear_velocity_m_s[2];
    latest_tip_twist_.angular.x = actual_tip.angular_velocity_rad_s[0];
    latest_tip_twist_.angular.y = actual_tip.angular_velocity_rad_s[1];
    latest_tip_twist_.angular.z = actual_tip.angular_velocity_rad_s[2];
    desired_tip_speed_mps_ = vectorNorm(desired_tip.linear_velocity_m_s);
    const std::array<double, 3> position_delta{{
        actual_tip.position_m[0] - desired_tip.position_m[0],
        actual_tip.position_m[1] - desired_tip.position_m[1],
        actual_tip.position_m[2] - desired_tip.position_m[2]}};
    position_error_m_ = vectorNorm(position_delta);

    telemetry.stamp = static_cast<builtin_interfaces::msg::Time>(now);
    telemetry.trajectory_time_s = trajectory_time_s_;
    telemetry.state = state_;
    telemetry.joint_names = joint_names_;
    telemetry.desired_position = desired.positions;
    telemetry.actual_position = actual_positions;
    telemetry.actual_velocity = actual_velocities;
    telemetry.actual_tip_pose = latest_tip_pose_;
    telemetry.actual_tip_twist = latest_tip_twist_;
    telemetry.desired_tip_speed_mps = desired_tip_speed_mps_;
    telemetry.position_error_m = position_error_m_;
    telemetry.speed_scaling = speed_scaling_;

    const double linear_speed = vectorNorm(actual_tip.linear_velocity_m_s);
    const double angular_speed = vectorNorm(actual_tip.angular_velocity_rad_s);
    output_ << now.seconds() << ',' << sync_time_ns << ',' << now.nanoseconds() << ',' <<
      telemetry.trajectory_time_s << ',' << trajectory_time_ns_ << ',' << state_;
    for (const double value : telemetry.desired_position) {output_ << ',' << value;}
    for (const double value : telemetry.actual_position) {output_ << ',' << value;}
    for (const double value : telemetry.actual_velocity) {output_ << ',' << value;}
    output_ << ',' << actual_tip.position_m[0] << ',' << actual_tip.position_m[1] << ',' <<
      actual_tip.position_m[2] << ',' << actual_tip.linear_velocity_m_s[0] << ',' <<
      actual_tip.linear_velocity_m_s[1] << ',' << actual_tip.linear_velocity_m_s[2] << ',' <<
      linear_speed << ',' << actual_tip.angular_velocity_rad_s[0] << ',' <<
      actual_tip.angular_velocity_rad_s[1] << ',' << actual_tip.angular_velocity_rad_s[2] << ',' <<
      angular_speed << ',' << desired_tip_speed_mps_ << ',' << position_error_m_ << ',' <<
      speed_scaling_ << '\n';
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(node_->get_logger(), "failed to compute rod-tip telemetry: %s", exception.what());
    return;
  }
  publisher_->publish(std::move(telemetry));
}

void TelemetryRecorder::onSpeedScaling(const std_msgs::msg::Float64::SharedPtr message)
{
  setSpeedScaling(urSpeedScalingPercentToRatio(message->data));
}

}  // namespace hx_waypoint
