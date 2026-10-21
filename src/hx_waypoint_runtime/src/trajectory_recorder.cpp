#include "hx_waypoint/trajectory_recorder.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace hx_waypoint
{
namespace
{

[[nodiscard]] TrajectoryRecorderOptions optionsWithJointTopic(std::string topic)
{
  TrajectoryRecorderOptions options;
  options.joint_states_topic = std::move(topic);
  return options;
}

void ensureParent(const std::filesystem::path & output_file)
{
  if (!output_file.parent_path().empty()) {
    std::filesystem::create_directories(output_file.parent_path());
  }
}

void writeActualJointCsv(
  const std::filesystem::path & output_file,
  const std::vector<std::string> & joint_names,
  const std::vector<ActualJointSample> & samples)
{
  ensureParent(output_file);
  std::ofstream output(output_file);
  if (!output) {
    throw std::runtime_error("failed to open trajectory recording: " + output_file.string());
  }
  output << "ros_time_s,trajectory_time_s";
  for (const auto & name : joint_names) {output << ',' << name << "_pos";}
  for (const auto & name : joint_names) {output << ',' << name << "_vel";}
  output << '\n' << std::setprecision(std::numeric_limits<double>::max_digits10);
  for (const auto & sample : samples) {
    output << sample.ros_time_s << ',' << sample.trajectory_time_s;
    for (const double value : sample.positions) {output << ',' << value;}
    for (const double value : sample.velocities) {output << ',' << value;}
    output << '\n';
  }
  if (!output) {
    throw std::runtime_error("failed while writing trajectory recording: " + output_file.string());
  }
}

[[nodiscard]] trajectory_msgs::msg::JointTrajectoryPoint reorderPoint(
  const trajectory_msgs::msg::JointTrajectoryPoint & source,
  const std::vector<std::string> & source_names,
  const std::vector<std::string> & target_names)
{
  trajectory_msgs::msg::JointTrajectoryPoint result;
  result.time_from_start = source.time_from_start;
  std::unordered_map<std::string, std::size_t> index;
  for (std::size_t i = 0; i < source_names.size(); ++i) {
    index.emplace(source_names[i], i);
  }
  const auto reorder = [&index, &target_names](
    const std::vector<double> & values, std::vector<double> & destination) {
      if (values.size() != index.size()) {
        return;
      }
      destination.reserve(target_names.size());
      for (const auto & name : target_names) {
        const auto found = index.find(name);
        if (found == index.end()) {
          destination.clear();
          return;
        }
        destination.push_back(values[found->second]);
      }
    };
  reorder(source.positions, result.positions);
  reorder(source.velocities, result.velocities);
  reorder(source.accelerations, result.accelerations);
  return result;
}

[[nodiscard]] bool hasTimestamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec != 0 || stamp.nanosec != 0U;
}

[[nodiscard]] rclcpp::Time messageTimeOrNow(
  const builtin_interfaces::msg::Time & stamp,
  rclcpp::Node * node)
{
  if (hasTimestamp(stamp)) {
    return rclcpp::Time(stamp, node->get_clock()->get_clock_type());
  }
  return node->now();
}

[[nodiscard]] std::optional<std::int64_t> durationNanoseconds(
  const builtin_interfaces::msg::Duration & duration)
{
  constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
  if (duration.sec < 0 ||
    duration.nanosec >= static_cast<std::uint32_t>(kNanosecondsPerSecond))
  {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(duration.sec) * kNanosecondsPerSecond +
         static_cast<std::int64_t>(duration.nanosec);
}

}  // namespace

TrajectoryRecorder::TrajectoryRecorder(
  rclcpp::Node * node,
  std::vector<std::string> joint_names,
  std::string joint_states_topic)
: TrajectoryRecorder(
    node, std::move(joint_names), optionsWithJointTopic(std::move(joint_states_topic)))
{
}

TrajectoryRecorder::TrajectoryRecorder(
  rclcpp::Node * node,
  std::vector<std::string> joint_names,
  TrajectoryRecorderOptions options)
: node_(node), joint_names_(std::move(joint_names)), options_(std::move(options))
{
  if (node_ == nullptr || joint_names_.empty() || options_.joint_states_topic.empty()) {
    throw std::invalid_argument("TrajectoryRecorder requires node, joint names, and JointState topic");
  }
  if (options_.base_frame.empty() || options_.frame_names.empty() || options_.tip_frame.empty()) {
    throw std::invalid_argument("TrajectoryRecorder requires base, frame, and tip frame names");
  }
  // Both recording inputs keep their original mutual exclusion, independently of admin services.
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group_;
  subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    options_.joint_states_topic, rclcpp::SensorDataQoS(),
    std::bind(&TrajectoryRecorder::onJointState, this, std::placeholders::_1),
    subscription_options);
  if (!options_.controller_state_topic.empty()) {
    controller_subscription_ =
      node_->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      options_.controller_state_topic, rclcpp::QoS(100),
      std::bind(&TrajectoryRecorder::onControllerState, this, std::placeholders::_1),
      subscription_options);
  }
}

void TrajectoryRecorder::configureKinematics(const RobotValidationModel & robot)
{
  if (robot.base_link != options_.base_frame || robot.tip_frame != options_.tip_frame ||
    robot.joint_names != joint_names_)
  {
    throw std::invalid_argument(
            "simulation recording kinematics do not match the configured frame/joint contract");
  }
  auto kinematics = std::make_unique<TipKinematics>(
    robot.urdf_path,
    robot.base_link,
    robot.tool_link,
    robot.tip_frame,
    robot.tool_to_tip_translation_m,
    robot.tool_to_tip_quaternion_xyzw,
    robot.joint_names,
    options_.frame_names);
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_) {
    throw std::logic_error("cannot replace simulation recording kinematics while active");
  }
  kinematics_ = std::move(kinematics);
}

void TrajectoryRecorder::start(
  const rclcpp::Time & trajectory_start,
  double trajectory_duration_s,
  double nominal_sample_dt_s)
{
  if (!std::isfinite(trajectory_duration_s) || trajectory_duration_s <= 0.0 ||
    !std::isfinite(nominal_sample_dt_s) || nominal_sample_dt_s <= 0.0)
  {
    throw std::invalid_argument(
            "trajectory recording duration and nominal sample interval must be finite and positive");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!kinematics_) {
    throw std::logic_error("simulation recording kinematics were not configured");
  }
  samples_.clear();
  controller_samples_.clear();
  frame_samples_.clear();
  recording_arm_time_ = trajectory_start;
  trajectory_start_ = trajectory_start;
  trajectory_duration_s_ = trajectory_duration_s;
  nominal_sample_dt_s_ = nominal_sample_dt_s;
  controller_time_origin_locked_ = options_.controller_state_topic.empty();
  last_controller_trajectory_time_s_ = -1.0;
  recording_error_.reset();
  start_actual_sample_ = cached_actual_sample_;
  start_frame_samples_ = cached_frame_samples_;
  active_ = true;
  if (controller_time_origin_locked_) {
    replayPrerollLocked();
  }
}

std::filesystem::path TrajectoryRecorder::stopAndWrite(
  const std::filesystem::path & output_file)
{
  std::vector<ActualJointSample> samples;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    samples = samples_;
    if (recording_error_) {
      throw std::runtime_error(*recording_error_);
    }
  }
  if (samples.empty()) {
    throw std::runtime_error("simulation produced no actual JointState samples");
  }
  writeActualJointCsv(output_file, joint_names_, samples);
  return output_file;
}

SimulationRecordingPaths TrajectoryRecorder::stopAndWriteAll(
  const SimulationRecordingPaths & output_files)
{
  std::vector<ActualJointSample> actual_samples;
  std::vector<ControllerSample> controller_samples;
  std::vector<FrameSample> frame_samples;
  std::optional<std::string> recording_error;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    actual_samples = samples_;
    controller_samples = controller_samples_;
    frame_samples = frame_samples_;
    recording_error = recording_error_;
  }
  if (recording_error) {
    throw std::runtime_error(*recording_error);
  }
  if (actual_samples.empty()) {
    throw std::runtime_error("simulation produced no actual JointState samples");
  }
  if (controller_samples.empty()) {
    throw std::runtime_error(
            "simulation produced no JointTrajectoryControllerState samples; cannot validate desired/actual");
  }
  const auto all_finite = [](const std::vector<double> & values) {
      return std::all_of(values.begin(), values.end(), [](double value) {
          return std::isfinite(value);
        });
    };
  for (const auto & sample : actual_samples) {
    if (sample.positions.size() != joint_names_.size() ||
      sample.velocities.size() != joint_names_.size() ||
      !all_finite(sample.positions) || !all_finite(sample.velocities))
    {
      throw std::runtime_error(
              "simulation actual JointState recording has missing or non-finite q/qd values");
    }
  }
  for (const auto & sample : controller_samples) {
    for (const auto * point : {&sample.reference, &sample.feedback, &sample.error}) {
      if (point->positions.size() != joint_names_.size() ||
        point->velocities.size() != joint_names_.size() ||
        !all_finite(point->positions) || !all_finite(point->velocities))
      {
        throw std::runtime_error(
                "simulation controller-state recording has missing or non-finite q/qd values");
      }
    }
    if (!std::isfinite(sample.speed_scaling)) {
      throw std::runtime_error("simulation controller-state speed_scaling is non-finite");
    }
  }
  std::set<std::string> recorded_frames;
  for (const auto & sample : frame_samples) {
    recorded_frames.insert(sample.transform.child_frame_id);
  }
  for (const auto & frame : options_.frame_names) {
    if (recorded_frames.count(frame) == 0U) {
      throw std::runtime_error(
              "TF recording is missing required base-to-frame transform for '" + frame + "'");
    }
  }
  if (recorded_frames.count(options_.tip_frame) == 0U) {
    throw std::runtime_error("TF recording is missing required tip frame '" + options_.tip_frame + "'");
  }

  writeActualJointCsv(output_files.actual_joint, joint_names_, actual_samples);

  ensureParent(output_files.controller_state);
  std::ofstream controller_output(output_files.controller_state);
  if (!controller_output) {
    throw std::runtime_error("failed to open controller-state recording");
  }
  controller_output << "ros_time_s,trajectory_time_s,speed_scaling";
  for (const auto & prefix : {"reference", "feedback", "error"}) {
    for (const auto & name : joint_names_) {controller_output << ',' << prefix << '_' << name << "_pos";}
    for (const auto & name : joint_names_) {controller_output << ',' << prefix << '_' << name << "_vel";}
  }
  controller_output << '\n' <<
    std::setprecision(std::numeric_limits<double>::max_digits10);
  for (const auto & sample : controller_samples) {
    controller_output << sample.ros_time_s << ',' << sample.trajectory_time_s << ',' <<
      sample.speed_scaling;
    for (const auto * point : {&sample.reference, &sample.feedback, &sample.error}) {
      for (const double value : point->positions) {controller_output << ',' << value;}
      for (const double value : point->velocities) {controller_output << ',' << value;}
    }
    controller_output << '\n';
  }
  if (!controller_output) {
    throw std::runtime_error("failed while writing controller-state recording");
  }

  ensureParent(output_files.joint_frames);
  std::ofstream frames_output(output_files.joint_frames);
  if (!frames_output) {
    throw std::runtime_error("failed to open joint-frame recording");
  }
  frames_output << "ros_time_s,trajectory_time_s,frame_id,x_m,y_m,z_m,qx,qy,qz,qw\n" <<
    std::setprecision(std::numeric_limits<double>::max_digits10);
  for (const auto & sample : frame_samples) {
    const auto & translation = sample.transform.transform.translation;
    const auto & rotation = sample.transform.transform.rotation;
    frames_output << sample.ros_time_s << ',' << sample.trajectory_time_s << ',' <<
      sample.transform.child_frame_id << ',' << translation.x << ',' << translation.y << ',' <<
      translation.z << ',' << rotation.x << ',' << rotation.y << ',' << rotation.z << ',' <<
      rotation.w << '\n';
  }
  if (!frames_output) {
    throw std::runtime_error("failed while writing joint-frame recording");
  }

  ensureParent(output_files.actual_tip);
  std::ofstream tip_output(output_files.actual_tip);
  if (!tip_output) {
    throw std::runtime_error("failed to open simulated tip recording");
  }
  tip_output << "ros_time_s,trajectory_time_s,x_m,y_m,z_m,qx,qy,qz,qw\n" <<
    std::setprecision(std::numeric_limits<double>::max_digits10);
  for (const auto & sample : frame_samples) {
    if (sample.transform.child_frame_id != options_.tip_frame) {
      continue;
    }
    const auto & translation = sample.transform.transform.translation;
    const auto & rotation = sample.transform.transform.rotation;
    tip_output << sample.ros_time_s << ',' << sample.trajectory_time_s << ',' <<
      translation.x << ',' << translation.y << ',' << translation.z << ',' << rotation.x << ',' <<
      rotation.y << ',' << rotation.z << ',' << rotation.w << '\n';
  }
  if (!tip_output) {
    throw std::runtime_error("failed while writing simulated tip recording");
  }
  return output_files;
}

void TrajectoryRecorder::stop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
}

bool TrajectoryRecorder::active() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return active_;
}

std::size_t TrajectoryRecorder::sampleCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_.size();
}

void TrajectoryRecorder::trimPrerollLocked(double newest_ros_time_s)
{
  constexpr double kPrerollDurationS = 1.0;
  const double cutoff_s = newest_ros_time_s - kPrerollDurationS;
  while (!joint_preroll_.empty() && joint_preroll_.front().actual.ros_time_s < cutoff_s) {
    joint_preroll_.pop_front();
  }
  while (!controller_preroll_.empty() && controller_preroll_.front().ros_time_s < cutoff_s) {
    controller_preroll_.pop_front();
  }
}

bool TrajectoryRecorder::lockControllerTimeOriginFromPrerollLocked()
{
  if (!active_ || controller_time_origin_locked_ || controller_preroll_.empty()) {
    return false;
  }

  const double arm_time_s = recording_arm_time_.seconds();
  const double candidate_time_limit_s = std::min(
    trajectory_duration_s_, std::max(0.2, 20.0 * nominal_sample_dt_s_));
  std::optional<std::int64_t> best_origin_ns;
  double best_arm_distance_s = std::numeric_limits<double>::infinity();
  for (const auto & sample : controller_preroll_) {
    const auto controller_time_ns = durationNanoseconds(sample.reference.time_from_start);
    if (!controller_time_ns) {
      continue;
    }
    const double controller_time_s = static_cast<double>(*controller_time_ns) * 1.0e-9;
    if (controller_time_s > candidate_time_limit_s) {
      continue;
    }
    const auto sample_time_ns = static_cast<std::int64_t>(
      std::llround(sample.ros_time_s * 1.0e9));
    const std::int64_t origin_ns = sample_time_ns - *controller_time_ns;
    const double origin_s = static_cast<double>(origin_ns) * 1.0e-9;
    const double arm_distance_s = std::abs(origin_s - arm_time_s);
    if (!best_origin_ns || arm_distance_s + 1.0e-9 < best_arm_distance_s ||
      (std::abs(arm_distance_s - best_arm_distance_s) <= 1.0e-9 &&
      origin_ns > *best_origin_ns))
    {
      best_origin_ns = origin_ns;
      best_arm_distance_s = arm_distance_s;
    }
  }
  if (!best_origin_ns) {
    return false;
  }

  trajectory_start_ = rclcpp::Time(
    *best_origin_ns, recording_arm_time_.get_clock_type());
  controller_time_origin_locked_ = true;
  replayPrerollLocked();
  return true;
}

void TrajectoryRecorder::appendActualSampleLocked(
  ActualJointSample actual,
  std::vector<FrameSample> frames)
{
  double trajectory_time_s = actual.ros_time_s - trajectory_start_.seconds();
  if (!std::isfinite(trajectory_time_s) || trajectory_time_s < -1.0e-6 ||
    trajectory_time_s > trajectory_duration_s_)
  {
    return;
  }
  trajectory_time_s = std::max(0.0, trajectory_time_s);
  if (!samples_.empty() && trajectory_time_s <= samples_.back().trajectory_time_s + 1.0e-12) {
    return;
  }
  if (samples_.empty() && trajectory_time_s > 0.0 && start_actual_sample_) {
    auto boundary = *start_actual_sample_;
    boundary.ros_time_s = trajectory_start_.seconds();
    boundary.trajectory_time_s = 0.0;
    samples_.push_back(std::move(boundary));
    for (auto frame : start_frame_samples_) {
      frame.ros_time_s = trajectory_start_.seconds();
      frame.trajectory_time_s = 0.0;
      frame_samples_.push_back(std::move(frame));
    }
  }
  actual.trajectory_time_s = trajectory_time_s;
  for (auto & frame : frames) {
    frame.trajectory_time_s = trajectory_time_s;
  }
  samples_.push_back(std::move(actual));
  frame_samples_.insert(
    frame_samples_.end(), std::make_move_iterator(frames.begin()),
    std::make_move_iterator(frames.end()));
}

void TrajectoryRecorder::appendControllerSampleLocked(ControllerSample sample)
{
  const auto controller_time_ns = durationNanoseconds(sample.reference.time_from_start);
  if (!controller_time_ns) {
    recording_error_ =
      "simulation controller published an invalid reference.time_from_start";
    return;
  }
  const double controller_time_s = static_cast<double>(*controller_time_ns) * 1.0e-9;
  const double mapped_time_s = sample.ros_time_s - trajectory_start_.seconds();
  // A previous prepare/hold goal can also have a near-zero time_from_start in the raw
  // pre-roll.  Its inferred origin must not be accepted as part of the authoritative goal.
  // Header-stamp/time_from_start alignment is stable to much less than one controller cycle;
  // two cycles leaves room for transport jitter while still separating consecutive goals.
  const double clock_alignment_tolerance_s = std::max(0.005, 2.0 * nominal_sample_dt_s_);
  if (!std::isfinite(mapped_time_s) ||
    std::abs(controller_time_s - mapped_time_s) > clock_alignment_tolerance_s ||
    controller_time_s > trajectory_duration_s_)
  {
    return;
  }
  if (last_controller_trajectory_time_s_ >= 0.0 &&
    controller_time_s <= last_controller_trajectory_time_s_ + 1.0e-12)
  {
    return;
  }
  if (controller_samples_.empty() && controller_time_s > 0.0) {
    auto boundary = sample;
    boundary.ros_time_s = trajectory_start_.seconds();
    boundary.trajectory_time_s = 0.0;
    boundary.reference.time_from_start = builtin_interfaces::msg::Duration();
    boundary.feedback.time_from_start = builtin_interfaces::msg::Duration();
    boundary.error.time_from_start = builtin_interfaces::msg::Duration();
    controller_samples_.push_back(std::move(boundary));
  }
  sample.trajectory_time_s = controller_time_s;
  controller_samples_.push_back(std::move(sample));
  last_controller_trajectory_time_s_ = controller_time_s;
}

void TrajectoryRecorder::replayPrerollLocked()
{
  if (!active_ || !controller_time_origin_locked_) {
    return;
  }
  std::vector<JointPrerollSample> joint_samples(joint_preroll_.begin(), joint_preroll_.end());
  std::vector<ControllerSample> controller_samples(
    controller_preroll_.begin(), controller_preroll_.end());
  std::sort(
    joint_samples.begin(), joint_samples.end(),
    [](const JointPrerollSample & left, const JointPrerollSample & right) {
      return left.actual.ros_time_s < right.actual.ros_time_s;
    });
  std::sort(
    controller_samples.begin(), controller_samples.end(),
    [](const ControllerSample & left, const ControllerSample & right) {
      return left.ros_time_s < right.ros_time_s;
    });
  const double origin_s = trajectory_start_.seconds();
  for (const auto & preroll : joint_samples) {
    if (preroll.actual.ros_time_s <= origin_s + 1.0e-9) {
      start_actual_sample_ = preroll.actual;
      start_frame_samples_ = preroll.frames;
    }
  }
  for (const auto & sample : controller_samples) {
    appendControllerSampleLocked(sample);
  }
  for (const auto & preroll : joint_samples) {
    appendActualSampleLocked(preroll.actual, preroll.frames);
  }
}

void TrajectoryRecorder::onJointState(const sensor_msgs::msg::JointState::SharedPtr message)
{
  const rclcpp::Time sample_time = messageTimeOrNow(message->header.stamp, node_);
  if (message->name.size() != message->position.size()) {
    return;
  }
  std::unordered_map<std::string, std::size_t> source_index;
  source_index.reserve(message->name.size());
  for (std::size_t index = 0; index < message->name.size(); ++index) {
    source_index.emplace(message->name[index], index);
  }

  ActualJointSample actual;
  actual.ros_time_s = sample_time.seconds();
  actual.trajectory_time_s = 0.0;
  actual.positions.reserve(joint_names_.size());
  actual.velocities.reserve(joint_names_.size());
  for (const auto & name : joint_names_) {
    const auto found = source_index.find(name);
    if (found == source_index.end() || !std::isfinite(message->position[found->second])) {
      return;
    }
    actual.positions.push_back(message->position[found->second]);
    if (message->velocity.size() == message->name.size() &&
      std::isfinite(message->velocity[found->second]))
    {
      actual.velocities.push_back(message->velocity[found->second]);
    } else {
      actual.velocities.push_back(std::numeric_limits<double>::quiet_NaN());
    }
  }

  std::vector<FrameKinematicsResult> evaluated_frames;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!kinematics_) {
      return;
    }
    try {
      evaluated_frames = kinematics_->evaluateFrames(actual.positions);
    } catch (const std::exception &) {
      return;
    }
  }
  if (evaluated_frames.size() != options_.frame_names.size()) {
    return;
  }

  std::vector<FrameSample> frames;
  frames.reserve(options_.frame_names.size());
  for (std::size_t index = 0U; index < evaluated_frames.size(); ++index) {
    const auto & evaluated = evaluated_frames[index];
    if (evaluated.frame_id != options_.frame_names[index]) {
      return;
    }
    FrameSample sample;
    sample.ros_time_s = sample_time.seconds();
    sample.trajectory_time_s = 0.0;
    sample.transform.header.stamp = message->header.stamp;
    sample.transform.header.frame_id = options_.base_frame;
    sample.transform.child_frame_id = evaluated.frame_id;
    sample.transform.transform.translation.x = evaluated.position_m[0];
    sample.transform.transform.translation.y = evaluated.position_m[1];
    sample.transform.transform.translation.z = evaluated.position_m[2];
    sample.transform.transform.rotation.x = evaluated.quaternion_xyzw[0];
    sample.transform.transform.rotation.y = evaluated.quaternion_xyzw[1];
    sample.transform.transform.rotation.z = evaluated.quaternion_xyzw[2];
    sample.transform.transform.rotation.w = evaluated.quaternion_xyzw[3];
    frames.push_back(std::move(sample));
  }

  std::lock_guard<std::mutex> lock(mutex_);
  cached_actual_sample_ = actual;
  cached_frame_samples_ = frames;
  joint_preroll_.push_back(JointPrerollSample{actual, frames});
  trimPrerollLocked(actual.ros_time_s);
  if (!active_ || !controller_time_origin_locked_) {
    return;
  }
  appendActualSampleLocked(std::move(actual), std::move(frames));
}

void TrajectoryRecorder::onControllerState(
  const control_msgs::msg::JointTrajectoryControllerState::SharedPtr message)
{
  const rclcpp::Time sample_time = messageTimeOrNow(message->header.stamp, node_);
  if (message->joint_names.empty()) {
    return;
  }
  ControllerSample sample;
  sample.ros_time_s = sample_time.seconds();
  sample.trajectory_time_s = 0.0;
  sample.reference = reorderPoint(message->reference, message->joint_names, joint_names_);
  sample.feedback = reorderPoint(message->feedback, message->joint_names, joint_names_);
  sample.error = reorderPoint(message->error, message->joint_names, joint_names_);
  // The simulation contract uses the exact unscaled
  // joint_trajectory_controller action.  Its controller-state message leaves
  // speed_scaling_factor at the ROS default 0 because no scaling interface is
  // configured; that field is not an effective execution scale.  Record the
  // controller's actual fixed semantics (1.0).  Real playback is checked
  // independently against the required UR speed_scaling topic before and
  // throughout execution.
  sample.speed_scaling = 1.0;
  if (sample.reference.positions.size() != joint_names_.size() ||
    sample.feedback.positions.size() != joint_names_.size() ||
    sample.error.positions.size() != joint_names_.size())
  {
    return;
  }
  if (sample.reference.velocities.size() != joint_names_.size()) {
    sample.reference.velocities.assign(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  }
  if (sample.feedback.velocities.size() != joint_names_.size()) {
    sample.feedback.velocities.assign(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  }
  if (sample.error.velocities.size() != joint_names_.size()) {
    sample.error.velocities.assign(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  }

  std::lock_guard<std::mutex> lock(mutex_);
  cached_controller_sample_ = sample;
  controller_preroll_.push_back(sample);
  trimPrerollLocked(sample.ros_time_s);
  if (!active_) {
    return;
  }

  const auto controller_time_ns = durationNanoseconds(message->reference.time_from_start);
  if (!controller_time_ns) {
    recording_error_ =
      "simulation controller published an invalid reference.time_from_start";
    return;
  }
  if (!controller_time_origin_locked_) {
    // The goal-response callback and controller-state subscription are independent queues.
    // States from the preceding prepare/hold goal may therefore arrive after acceptance.  Keep
    // a short raw window, then choose the time origin closest to the acceptance timestamp and
    // replay only real samples whose header/time_from_start pair agrees with that origin.
    const double origin_selection_window_s = std::min(
      trajectory_duration_s_, std::max(0.1, 10.0 * nominal_sample_dt_s_));
    if (sample_time.seconds() < recording_arm_time_.seconds() + origin_selection_window_s) {
      return;
    }
    (void)lockControllerTimeOriginFromPrerollLocked();
    return;
  }
  appendControllerSampleLocked(std::move(sample));
}

}  // namespace hx_waypoint
