#include "hx_waypoint/simulation_recording_validator.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "hx_waypoint/runtime_policy.hpp"
#include "hx_waypoint/setup_config.hpp"
#include "hx_waypoint/tip_kinematics.hpp"

namespace hx_waypoint
{
namespace
{

constexpr double kSpeedScalingTolerance = 1.0e-9;

[[nodiscard]] std::string trim(std::string value)
{
  const auto not_space = [](unsigned char character) {return !std::isspace(character);};
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

[[nodiscard]] std::vector<std::string> splitCsvLine(const std::string & line)
{
  if (line.find('"') != std::string::npos) {
    throw std::runtime_error("simulation recording CSV must not contain quoted fields");
  }
  std::vector<std::string> fields;
  std::stringstream input(line);
  std::string field;
  while (std::getline(input, field, ',')) {
    fields.push_back(trim(std::move(field)));
  }
  if (!line.empty() && line.back() == ',') {
    fields.emplace_back();
  }
  return fields;
}

[[nodiscard]] double parseFinite(
  const std::string & value,
  const std::filesystem::path & path,
  std::size_t line_number,
  const std::string & column)
{
  std::size_t consumed = 0U;
  double parsed = 0.0;
  try {
    parsed = std::stod(value, &consumed);
  } catch (const std::exception &) {
    throw std::runtime_error(
            "invalid numeric value in " + path.filename().string() + " line " +
            std::to_string(line_number) + ", column " + column);
  }
  if (consumed != value.size() || !std::isfinite(parsed)) {
    throw std::runtime_error(
            "non-finite or malformed value in " + path.filename().string() + " line " +
            std::to_string(line_number) + ", column " + column);
  }
  return parsed;
}

[[nodiscard]] std::vector<std::vector<std::string>> readStrictCsv(
  const std::filesystem::path & path,
  const std::vector<std::string> & expected_header)
{
  if (!std::filesystem::is_regular_file(path)) {
    throw std::runtime_error("simulation recording CSV is missing: " + path.string());
  }
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open simulation recording CSV: " + path.string());
  }
  std::string line;
  if (!std::getline(input, line)) {
    throw std::runtime_error("simulation recording CSV is empty: " + path.string());
  }
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  if (splitCsvLine(line) != expected_header) {
    throw std::runtime_error(
            "simulation recording CSV header is not the expected strict schema: " + path.string());
  }

  std::vector<std::vector<std::string>> rows;
  std::size_t line_number = 1U;
  while (std::getline(input, line)) {
    ++line_number;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (trim(line).empty()) {
      throw std::runtime_error(
              "simulation recording CSV contains an empty row at line " +
              std::to_string(line_number));
    }
    auto fields = splitCsvLine(line);
    if (fields.size() != expected_header.size()) {
      throw std::runtime_error(
              "simulation recording CSV has the wrong column count at line " +
              std::to_string(line_number));
    }
    rows.push_back(std::move(fields));
  }
  if (rows.size() < 2U) {
    throw std::runtime_error(
            "simulation recording CSV requires at least two data samples: " + path.string());
  }
  return rows;
}

struct PoseStreamData
{
  std::vector<double> ros_times;
  std::vector<double> trajectory_times;
  std::vector<std::array<double, 3>> positions_m;
  std::vector<std::array<double, 4>> quaternions_xyzw;
  double max_quaternion_norm_error{0.0};
};

void appendPoseSample(
  const std::vector<std::string> & row,
  const std::filesystem::path & path,
  std::size_t line_number,
  std::size_t translation_offset,
  std::size_t quaternion_offset,
  PoseStreamData & stream)
{
  stream.ros_times.push_back(parseFinite(row[0], path, line_number, "ros_time_s"));
  stream.trajectory_times.push_back(
    parseFinite(row[1], path, line_number, "trajectory_time_s"));
  std::array<double, 3> position{};
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    position[axis] = parseFinite(
      row[translation_offset + axis], path, line_number, "translation");
  }
  stream.positions_m.push_back(position);

  double squared_norm = 0.0;
  std::array<double, 4> quaternion{};
  for (std::size_t component = 0U; component < 4U; ++component) {
    const double value = parseFinite(
      row[quaternion_offset + component], path, line_number, "quaternion");
    quaternion[component] = value;
    squared_norm += value * value;
  }
  const double norm = std::sqrt(squared_norm);
  if (!std::isfinite(norm) || norm <= std::numeric_limits<double>::epsilon()) {
    throw std::runtime_error(
            "quaternion norm is zero or non-finite in " + path.filename().string() + " line " +
            std::to_string(line_number));
  }
  stream.max_quaternion_norm_error = std::max(
    stream.max_quaternion_norm_error, std::abs(norm - 1.0));
  for (double & value : quaternion) {
    value /= norm;
  }
  stream.quaternions_xyzw.push_back(quaternion);
}

[[nodiscard]] SamplingValidationMetrics analyzeSampling(
  const std::vector<double> & times,
  double expected_duration_s,
  const SimulationValidationConfig & config,
  const std::string & stream_name)
{
  if (times.size() < 2U) {
    throw std::runtime_error(stream_name + " requires at least two data samples");
  }
  SamplingValidationMetrics metrics;
  metrics.sample_count = times.size();
  metrics.first_trajectory_time_s = times.front();
  metrics.last_trajectory_time_s = times.back();
  metrics.coverage_duration_s = times.back() - times.front();

  if (times.front() < 0.0) {
    throw std::runtime_error(stream_name + " contains a negative trajectory timestamp");
  }
  for (std::size_t index = 1U; index < times.size(); ++index) {
    const double interval = times[index] - times[index - 1U];
    if (!std::isfinite(interval) || interval <= 0.0) {
      throw std::runtime_error(stream_name + " trajectory timestamps must be strictly increasing");
    }
    metrics.max_time_interval_s = std::max(metrics.max_time_interval_s, interval);
    // Controller and JointState topics may publish faster than the configured recording cadence.
    // Extra samples improve coverage and are not "negative jitter" or missing samples. Only gaps
    // beyond sample_dt_s are mapped to the nearest number of nominal periods.
    const auto nominal_steps = interval <= config.sample_dt_s ? 1LL :
      std::max<long long>(1LL, std::llround(interval / config.sample_dt_s));
    const double jitter = interval <= config.sample_dt_s ? 0.0 : std::abs(
      interval - static_cast<double>(nominal_steps) * config.sample_dt_s);
    metrics.max_time_jitter_s = std::max(metrics.max_time_jitter_s, jitter);
    metrics.missing_sample_estimate += static_cast<std::size_t>(nominal_steps - 1LL);
  }

  const double endpoint_tolerance = config.sample_dt_s + config.max_time_jitter_s;
  if (metrics.first_trajectory_time_s > endpoint_tolerance) {
    throw std::runtime_error(stream_name + " starts too late to cover the commanded trajectory");
  }
  if (metrics.last_trajectory_time_s + endpoint_tolerance < expected_duration_s) {
    throw std::runtime_error(stream_name + " ends before the commanded trajectory is covered");
  }
  if (metrics.last_trajectory_time_s >
    expected_duration_s + config.max_completion_overrun_s + 1.0e-12)
  {
    throw std::runtime_error(
            stream_name + " continues beyond the unscaled commanded duration");
  }
  const double minimum_coverage = std::max(0.0, expected_duration_s - 2.0 * endpoint_tolerance);
  if (metrics.coverage_duration_s < minimum_coverage) {
    throw std::runtime_error(stream_name + " coverage duration is too short");
  }
  const std::size_t nominal_sample_count = static_cast<std::size_t>(
    std::floor(expected_duration_s / config.sample_dt_s)) + 1U;
  const std::size_t count_allowance = config.max_missing_samples + 2U;
  if (metrics.sample_count + count_allowance < nominal_sample_count) {
    throw std::runtime_error(stream_name + " sample count is too small for the commanded duration");
  }
  if (metrics.max_time_jitter_s > config.max_time_jitter_s + 1.0e-12) {
    throw std::runtime_error(stream_name + " exceeds recording.validation.max_time_jitter_s");
  }
  if (metrics.missing_sample_estimate > config.max_missing_samples) {
    throw std::runtime_error(stream_name + " exceeds recording.validation.max_missing_samples");
  }
  const double maximum_allowed_interval =
    (static_cast<double>(config.max_missing_samples) + 1.0) * config.sample_dt_s +
    config.max_time_jitter_s;
  if (metrics.max_time_interval_s > maximum_allowed_interval + 1.0e-12) {
    throw std::runtime_error(stream_name + " contains an excessive sample interval");
  }
  return metrics;
}

[[nodiscard]] PoseValidationMetrics analyzePoseStream(
  const PoseStreamData & stream,
  double expected_duration_s,
  const SimulationValidationConfig & config,
  const std::string & stream_name)
{
  if (stream.ros_times.size() != stream.trajectory_times.size() || stream.ros_times.size() < 2U) {
    throw std::runtime_error(stream_name + " requires at least two complete pose samples");
  }
  if (stream.ros_times.front() < 0.0) {
    throw std::runtime_error(stream_name + " contains a negative ROS timestamp");
  }
  for (std::size_t index = 1U; index < stream.ros_times.size(); ++index) {
    const double interval = stream.ros_times[index] - stream.ros_times[index - 1U];
    if (!std::isfinite(interval) || interval <= 0.0) {
      throw std::runtime_error(stream_name + " ROS timestamps must be strictly increasing");
    }
  }

  PoseValidationMetrics metrics;
  metrics.sampling = analyzeSampling(
    stream.trajectory_times, expected_duration_s, config, stream_name);
  metrics.max_quaternion_norm_error = stream.max_quaternion_norm_error;
  if (metrics.max_quaternion_norm_error > config.max_quaternion_norm_error + 1.0e-12) {
    throw std::runtime_error(
            stream_name + " exceeds recording.validation.max_quaternion_norm_error");
  }
  return metrics;
}

[[nodiscard]] std::vector<std::string> actualHeader(const std::vector<std::string> & joint_names)
{
  std::vector<std::string> header{"ros_time_s", "trajectory_time_s"};
  for (const auto & name : joint_names) {header.push_back(name + "_pos");}
  for (const auto & name : joint_names) {header.push_back(name + "_vel");}
  return header;
}

[[nodiscard]] std::vector<std::string> controllerHeader(
  const std::vector<std::string> & joint_names)
{
  std::vector<std::string> header{"ros_time_s", "trajectory_time_s", "speed_scaling"};
  for (const auto & prefix : {"reference", "feedback", "error"}) {
    for (const auto & name : joint_names) {header.push_back(prefix + std::string("_") + name + "_pos");}
    for (const auto & name : joint_names) {header.push_back(prefix + std::string("_") + name + "_vel");}
  }
  return header;
}

[[nodiscard]] std::vector<std::string> jointFramesHeader()
{
  return {
    "ros_time_s", "trajectory_time_s", "frame_id", "x_m", "y_m", "z_m",
    "qx", "qy", "qz", "qw"};
}

[[nodiscard]] std::vector<std::string> actualTipHeader()
{
  return {
    "ros_time_s", "trajectory_time_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"};
}

struct ReferenceValidationMetrics
{
  double max_position_error_rad{0.0};
  double max_velocity_error_rad_s{0.0};
};

[[nodiscard]] double pointTimeSeconds(
  const trajectory_msgs::msg::JointTrajectoryPoint & point)
{
  return static_cast<double>(point.time_from_start.sec) +
         static_cast<double>(point.time_from_start.nanosec) * 1.0e-9;
}

[[nodiscard]] ReferenceValidationMetrics compareCommandedControllerReference(
  const SimulationRecordingPaths & paths,
  const trajectory_msgs::msg::JointTrajectory & commanded_trajectory,
  const SimulationValidationConfig & config)
{
  const auto controller_rows = readStrictCsv(
    paths.controller_state, controllerHeader(commanded_trajectory.joint_names));
  const std::size_t joint_count = commanded_trajectory.joint_names.size();
  const std::size_t reference_position_offset = 3U;
  const std::size_t reference_velocity_offset = reference_position_offset + joint_count;
  std::vector<double> recorded_times;
  std::vector<std::vector<double>> recorded_positions;
  std::vector<std::vector<double>> recorded_velocities;
  recorded_times.reserve(controller_rows.size());
  recorded_positions.reserve(controller_rows.size());
  recorded_velocities.reserve(controller_rows.size());
  for (std::size_t row_index = 0U; row_index < controller_rows.size(); ++row_index) {
    const auto & row = controller_rows[row_index];
    const std::size_t line_number = row_index + 2U;
    recorded_times.push_back(
      parseFinite(row[1], paths.controller_state, line_number, "trajectory_time_s"));
    std::vector<double> positions;
    std::vector<double> velocities;
    positions.reserve(joint_count);
    velocities.reserve(joint_count);
    for (std::size_t joint = 0U; joint < joint_count; ++joint) {
      positions.push_back(parseFinite(
          row[reference_position_offset + joint], paths.controller_state, line_number,
          "reference position"));
      velocities.push_back(parseFinite(
          row[reference_velocity_offset + joint], paths.controller_state, line_number,
          "reference velocity"));
    }
    recorded_positions.push_back(std::move(positions));
    recorded_velocities.push_back(std::move(velocities));
  }

  ReferenceValidationMetrics metrics;
  std::size_t nearest_index = 0U;
  double previous_time_s = -1.0;
  const double match_tolerance_s = config.sample_dt_s + config.max_time_jitter_s;
  for (std::size_t point_index = 0U;
    point_index < commanded_trajectory.points.size(); ++point_index)
  {
    const auto & point = commanded_trajectory.points[point_index];
    const double command_time_s = pointTimeSeconds(point);
    if (!std::isfinite(command_time_s) || command_time_s < 0.0 ||
      (point_index == 0U && command_time_s != 0.0) ||
      (point_index > 0U && command_time_s <= previous_time_s) ||
      point.positions.size() != joint_count || point.velocities.size() != joint_count)
    {
      throw std::invalid_argument(
              "commanded simulation trajectory has invalid timing or q/qd dimensions");
    }
    previous_time_s = command_time_s;
    while (nearest_index + 1U < recorded_times.size() &&
      std::abs(recorded_times[nearest_index + 1U] - command_time_s) <=
      std::abs(recorded_times[nearest_index] - command_time_s))
    {
      ++nearest_index;
    }
    if (std::abs(recorded_times[nearest_index] - command_time_s) >
      match_tolerance_s + 1.0e-12)
    {
      throw std::runtime_error(
              "simulation controller reference does not cover every canonical command time");
    }
    for (std::size_t joint = 0U; joint < joint_count; ++joint) {
      if (!std::isfinite(point.positions[joint]) || !std::isfinite(point.velocities[joint])) {
        throw std::invalid_argument(
                "commanded simulation trajectory contains non-finite q/qd values");
      }
      metrics.max_position_error_rad = std::max(
        metrics.max_position_error_rad,
        std::abs(recorded_positions[nearest_index][joint] - point.positions[joint]));
      metrics.max_velocity_error_rad_s = std::max(
        metrics.max_velocity_error_rad_s,
        std::abs(recorded_velocities[nearest_index][joint] - point.velocities[joint]));
    }
  }
  return metrics;
}

struct RigidPose
{
  std::array<double, 3> position_m{};
  std::array<double, 4> quaternion_xyzw{{0.0, 0.0, 0.0, 1.0}};
};

[[nodiscard]] std::array<double, 4> normalizedQuaternion(
  std::array<double, 4> quaternion,
  const std::string & label)
{
  double squared_norm = 0.0;
  for (const double value : quaternion) {
    squared_norm += value * value;
  }
  const double norm = std::sqrt(squared_norm);
  if (!std::isfinite(norm) || norm <= std::numeric_limits<double>::epsilon() ||
    std::abs(norm - 1.0) > 1.0e-6)
  {
    throw std::runtime_error(label + " quaternion must be finite and normalized");
  }
  for (double & value : quaternion) {
    value /= norm;
  }
  return quaternion;
}

[[nodiscard]] std::array<double, 4> normalizeComputedQuaternion(
  std::array<double, 4> quaternion,
  const std::string & label)
{
  double squared_norm = 0.0;
  for (const double value : quaternion) {
    if (!std::isfinite(value)) {
      throw std::runtime_error(label + " quaternion must be finite");
    }
    squared_norm += value * value;
  }
  const double norm = std::sqrt(squared_norm);
  if (!std::isfinite(norm) || norm <= std::numeric_limits<double>::epsilon()) {
    throw std::runtime_error(label + " quaternion norm must be positive and finite");
  }
  for (double & value : quaternion) {
    value /= norm;
  }
  return quaternion;
}

[[nodiscard]] std::array<double, 4> quaternionMultiply(
  const std::array<double, 4> & left,
  const std::array<double, 4> & right)
{
  return {{
      left[3] * right[0] + left[0] * right[3] + left[1] * right[2] - left[2] * right[1],
      left[3] * right[1] - left[0] * right[2] + left[1] * right[3] + left[2] * right[0],
      left[3] * right[2] + left[0] * right[1] - left[1] * right[0] + left[2] * right[3],
      left[3] * right[3] - left[0] * right[0] - left[1] * right[1] - left[2] * right[2]}};
}

[[nodiscard]] std::array<double, 3> rotateVector(
  const std::array<double, 4> & quaternion,
  const std::array<double, 3> & vector)
{
  const std::array<double, 3> q{{quaternion[0], quaternion[1], quaternion[2]}};
  const std::array<double, 3> cross_q_v{{
      q[1] * vector[2] - q[2] * vector[1],
      q[2] * vector[0] - q[0] * vector[2],
      q[0] * vector[1] - q[1] * vector[0]}};
  const std::array<double, 3> cross_q_cross{{
      q[1] * cross_q_v[2] - q[2] * cross_q_v[1],
      q[2] * cross_q_v[0] - q[0] * cross_q_v[2],
      q[0] * cross_q_v[1] - q[1] * cross_q_v[0]}};
  return {{
      vector[0] + 2.0 * (quaternion[3] * cross_q_v[0] + cross_q_cross[0]),
      vector[1] + 2.0 * (quaternion[3] * cross_q_v[1] + cross_q_cross[1]),
      vector[2] + 2.0 * (quaternion[3] * cross_q_v[2] + cross_q_cross[2])}};
}

[[nodiscard]] RigidPose composePose(const RigidPose & parent, const RigidPose & child)
{
  const auto rotated = rotateVector(parent.quaternion_xyzw, child.position_m);
  RigidPose result;
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    result.position_m[axis] = parent.position_m[axis] + rotated[axis];
  }
  result.quaternion_xyzw = normalizedQuaternion(
    quaternionMultiply(parent.quaternion_xyzw, child.quaternion_xyzw),
    "composed pose");
  return result;
}

[[nodiscard]] double positionError(const RigidPose & actual, const RigidPose & expected)
{
  double squared = 0.0;
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    const double difference = actual.position_m[axis] - expected.position_m[axis];
    squared += difference * difference;
  }
  return std::sqrt(squared);
}

[[nodiscard]] double orientationErrorDeg(const RigidPose & actual, const RigidPose & expected)
{
  const auto actual_quaternion = normalizedQuaternion(
    actual.quaternion_xyzw, "actual orientation error pose");
  const auto expected_quaternion = normalizedQuaternion(
    expected.quaternion_xyzw, "expected orientation error pose");
  double dot = 0.0;
  for (std::size_t component = 0U; component < 4U; ++component) {
    dot += actual_quaternion[component] * expected_quaternion[component];
  }
  dot = std::clamp(std::abs(dot), 0.0, 1.0);
  if (1.0 - dot <= 1.0e-15) {
    return 0.0;
  }
  return 2.0 * std::acos(dot) * 180.0 / std::acos(-1.0);
}

[[nodiscard]] std::array<double, 4> slerpQuaternion(
  const std::array<double, 4> & start,
  const std::array<double, 4> & finish,
  double alpha)
{
  std::array<double, 4> end = finish;
  double dot = 0.0;
  for (std::size_t component = 0U; component < 4U; ++component) {
    dot += start[component] * end[component];
  }
  if (dot < 0.0) {
    dot = -dot;
    for (double & value : end) {
      value = -value;
    }
  }
  dot = std::clamp(dot, -1.0, 1.0);
  std::array<double, 4> result{};
  if (dot > 0.9995) {
    for (std::size_t component = 0U; component < 4U; ++component) {
      result[component] = start[component] + alpha * (end[component] - start[component]);
    }
    return normalizeComputedQuaternion(result, "interpolated source pose");
  }
  const double angle = std::acos(dot);
  const double denominator = std::sin(angle);
  const double start_weight = std::sin((1.0 - alpha) * angle) / denominator;
  const double end_weight = std::sin(alpha * angle) / denominator;
  for (std::size_t component = 0U; component < 4U; ++component) {
    result[component] = start_weight * start[component] + end_weight * end[component];
  }
  return normalizeComputedQuaternion(result, "interpolated source pose");
}

[[nodiscard]] RigidPose interpolatePoseStreamAtTime(
  const PoseStreamData & stream,
  double target_time_s,
  double coverage_tolerance_s)
{
  if (stream.trajectory_times.size() != stream.positions_m.size() ||
    stream.trajectory_times.size() != stream.quaternions_xyzw.size() ||
    stream.trajectory_times.size() < 2U)
  {
    throw std::runtime_error("pose stream is internally inconsistent");
  }
  if (!std::isfinite(target_time_s) || !std::isfinite(coverage_tolerance_s) ||
    coverage_tolerance_s < 0.0)
  {
    throw std::invalid_argument("pose interpolation requires finite time and tolerance");
  }

  const auto & times = stream.trajectory_times;
  if (target_time_s < times.front() - coverage_tolerance_s - 1.0e-12 ||
    target_time_s > times.back() + coverage_tolerance_s + 1.0e-12)
  {
    throw std::runtime_error("pose stream does not cover authoritative source time");
  }
  if (target_time_s <= times.front()) {
    return RigidPose{stream.positions_m.front(), stream.quaternions_xyzw.front()};
  }
  if (target_time_s >= times.back()) {
    return RigidPose{stream.positions_m.back(), stream.quaternions_xyzw.back()};
  }

  const auto finish_iterator = std::lower_bound(times.begin(), times.end(), target_time_s);
  if (finish_iterator == times.end()) {
    throw std::runtime_error("pose stream interpolation bracket is missing");
  }
  const std::size_t finish_index = static_cast<std::size_t>(
    std::distance(times.begin(), finish_iterator));
  if (std::abs(times[finish_index] - target_time_s) <= 1.0e-12) {
    return RigidPose{
      stream.positions_m[finish_index], stream.quaternions_xyzw[finish_index]};
  }
  if (finish_index == 0U) {
    throw std::runtime_error("pose stream interpolation start bracket is missing");
  }
  const std::size_t start_index = finish_index - 1U;
  const double interval_s = times[finish_index] - times[start_index];
  if (!std::isfinite(interval_s) || interval_s <= 0.0 ||
    target_time_s - times[start_index] > coverage_tolerance_s + 1.0e-12 ||
    times[finish_index] - target_time_s > coverage_tolerance_s + 1.0e-12)
  {
    throw std::runtime_error("pose stream interpolation bracket exceeds coverage tolerance");
  }

  const double alpha = std::clamp(
    (target_time_s - times[start_index]) / interval_s, 0.0, 1.0);
  RigidPose result;
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    result.position_m[axis] = stream.positions_m[start_index][axis] + alpha *
      (stream.positions_m[finish_index][axis] - stream.positions_m[start_index][axis]);
  }
  result.quaternion_xyzw = slerpQuaternion(
    stream.quaternions_xyzw[start_index],
    stream.quaternions_xyzw[finish_index], alpha);
  return result;
}

[[nodiscard]] std::unordered_map<std::string, RigidPose> loadBaseFrameTransforms(
  const std::filesystem::path & setup_path,
  const RobotValidationModel & robot)
{
  const SetupConfig setup = loadSetupConfig(setup_path);
  TipKinematics kinematics(
    robot.urdf_path,
    robot.base_link,
    robot.tool_link,
    robot.tip_frame,
    robot.tool_to_tip_translation_m,
    robot.tool_to_tip_quaternion_xyzw,
    robot.joint_names,
    {setup.work_frame.parent_frame});
  const std::vector<double> zeros(robot.joint_names.size(), 0.0);
  std::vector<double> midpoints;
  midpoints.reserve(robot.joint_limits.size());
  for (const auto & limit : robot.joint_limits) {
    midpoints.push_back(0.5 * (limit.lower_position_rad + limit.upper_position_rad));
  }
  const auto zero_frames = kinematics.evaluateFrames(zeros);
  const auto midpoint_frames = kinematics.evaluateFrames(midpoints);
  if (zero_frames.size() != 1U || midpoint_frames.size() != 1U ||
    zero_frames.front().frame_id != setup.work_frame.parent_frame ||
    midpoint_frames.front().frame_id != setup.work_frame.parent_frame)
  {
    throw std::runtime_error("setup work-frame parent could not be resolved from the URDF");
  }
  const RigidPose base_to_parent{
    zero_frames.front().position_m,
    normalizedQuaternion(zero_frames.front().quaternion_xyzw, "setup parent frame")};
  const RigidPose alternate_parent{
    midpoint_frames.front().position_m,
    normalizedQuaternion(midpoint_frames.front().quaternion_xyzw, "setup parent frame")};
  if (positionError(base_to_parent, alternate_parent) > 1.0e-12 ||
    orientationErrorDeg(base_to_parent, alternate_parent) > 1.0e-10)
  {
    throw std::runtime_error(
            "setup work-frame parent is not fixed relative to robot base_link");
  }
  const auto parent_to_child = setup.work_frame.parentToChild();
  const RigidPose base_to_work = composePose(
    base_to_parent,
    RigidPose{parent_to_child.translation_m, parent_to_child.quaternion_xyzw});

  std::unordered_map<std::string, RigidPose> resolved;
  resolved.emplace(robot.base_link, RigidPose{});
  resolved.emplace(setup.work_frame.child_frame, base_to_work);
  for (const auto & alias : setup.work_frame.aliases) {
    resolved.emplace(alias, base_to_work);
  }
  return resolved;
}

struct SourcePoseSample
{
  double trajectory_time_s{0.0};
  std::string frame_id;
  RigidPose frame_to_tip;
};

[[nodiscard]] std::vector<SourcePoseSample> loadSourcePoseSamples(
  const std::filesystem::path & source_path)
{
  const std::vector<std::string> header{
    "time_s", "frame_id", "tip_x_m", "tip_y_m", "tip_z_m",
    "tip_qx", "tip_qy", "tip_qz", "tip_qw", "phase"};
  const auto rows = readStrictCsv(source_path, header);
  std::vector<SourcePoseSample> samples;
  samples.reserve(rows.size());
  double previous_time_s = -1.0;
  for (std::size_t row_index = 0U; row_index < rows.size(); ++row_index) {
    const auto & row = rows[row_index];
    const std::size_t line_number = row_index + 2U;
    SourcePoseSample sample;
    sample.trajectory_time_s = parseFinite(row[0], source_path, line_number, "time_s");
    sample.frame_id = trim(row[1]);
    sample.frame_to_tip.position_m = {{
        parseFinite(row[2], source_path, line_number, "tip_x_m"),
        parseFinite(row[3], source_path, line_number, "tip_y_m"),
        parseFinite(row[4], source_path, line_number, "tip_z_m")}};
    sample.frame_to_tip.quaternion_xyzw = normalizedQuaternion({{
        parseFinite(row[5], source_path, line_number, "tip_qx"),
        parseFinite(row[6], source_path, line_number, "tip_qy"),
        parseFinite(row[7], source_path, line_number, "tip_qz"),
        parseFinite(row[8], source_path, line_number, "tip_qw")}}, "source tip");
    if (sample.frame_id.empty() || sample.trajectory_time_s < 0.0 ||
      sample.trajectory_time_s <= previous_time_s ||
      (row[9] != "draw" && row[9] != "hold"))
    {
      throw std::runtime_error("source trajectory pose row is invalid");
    }
    samples.push_back(std::move(sample));
    previous_time_s = samples.back().trajectory_time_s;
  }
  if (samples.front().trajectory_time_s != 0.0) {
    throw std::runtime_error("source trajectory pose evidence must start at time zero");
  }
  return samples;
}

[[nodiscard]] std::size_t nearestTimeIndex(
  const std::vector<double> & times,
  double target,
  double tolerance)
{
  const auto upper = std::lower_bound(times.begin(), times.end(), target);
  std::size_t index = upper == times.end() ? times.size() - 1U :
    static_cast<std::size_t>(std::distance(times.begin(), upper));
  if (index > 0U && std::abs(times[index - 1U] - target) <= std::abs(times[index] - target)) {
    --index;
  }
  if (std::abs(times[index] - target) > tolerance) {
    throw std::runtime_error("telemetry does not cover a required trajectory time");
  }
  return index;
}

}  // namespace

SimulationValidationConfig SimulationRecordingValidator::loadConfig(
  const std::filesystem::path & recording_config_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(recording_config_path.string());
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "failed to parse recording.yaml for simulation validation: " +
            std::string(error.what()));
  }
  SimulationValidationConfig config;
  try {
    if (!root.IsMap() || root["schema_version"].as<int>() != 1) {
      throw std::runtime_error("recording.yaml schema_version must be 1");
    }
    const YAML::Node recording = root["recording"];
    const YAML::Node validation = recording["validation"];
    if (!recording.IsMap() || !validation.IsMap()) {
      throw std::runtime_error("recording.yaml recording/validation fields must be maps");
    }
    if (recording["time_source"].as<std::string>() != "trajectory_time" ||
      recording["canonical_source"].as<std::string>() != "controller_action_goal" ||
      !recording["record_controller_actual"].as<bool>() ||
      !recording["record_joint_states"].as<bool>() ||
      !recording["record_joint_frames"].as<bool>() ||
      !recording["record_tip_state"].as<bool>() ||
      recording["first_point"]["time_s"].as<double>() != 0.0 ||
      !recording["output_fields"]["positions"].as<bool>() ||
      !recording["output_fields"]["velocities"].as<bool>() ||
      !recording["output_fields"]["accelerations"].as<bool>() ||
      !recording["first_point"]["require_zero_velocity"].as<bool>() ||
      !recording["first_point"]["require_zero_acceleration"].as<bool>() ||
      !recording["last_point"]["require_zero_velocity"].as<bool>() ||
      !recording["last_point"]["require_zero_acceleration"].as<bool>() ||
      !validation["require_action_success"].as<bool>())
    {
      throw std::runtime_error(
              "recording.yaml canonical trajectory and required telemetry contract must remain strict");
    }
    config.sample_dt_s = recording["sample_dt_s"].as<double>();
    config.max_missing_samples = validation["max_missing_samples"].as<std::size_t>();
    config.max_time_jitter_s = validation["max_time_jitter_s"].as<double>();
    config.max_quaternion_norm_error =
      validation["max_quaternion_norm_error"].as<double>();
    config.require_action_success = validation["require_action_success"].as<bool>();
    config.max_completion_overrun_s =
      validation["max_completion_overrun_s"].as<double>();
    config.max_controller_reference_position_error_rad =
      validation["max_controller_reference_position_error_rad"].as<double>();
    config.max_controller_reference_velocity_error_rad_s =
      validation["max_controller_reference_velocity_error_rad_s"].as<double>();
    config.max_joint_position_tracking_error_rad =
      validation["max_joint_position_tracking_error_rad"].as<double>();
    config.max_joint_velocity_tracking_error_rad_s =
      validation["max_joint_velocity_tracking_error_rad_s"].as<double>();
    config.max_actual_controller_position_error_rad =
      validation["max_actual_controller_position_error_rad"].as<double>();
    config.max_actual_controller_velocity_error_rad_s =
      validation["max_actual_controller_velocity_error_rad_s"].as<double>();
    config.max_frame_fk_position_error_m =
      validation["max_frame_fk_position_error_m"].as<double>();
    config.max_frame_fk_orientation_error_deg =
      validation["max_frame_fk_orientation_error_deg"].as<double>();
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "recording.yaml simulation validation fields are missing or invalid: " +
            std::string(error.what()));
  }
  if (!std::isfinite(config.sample_dt_s) || config.sample_dt_s <= 0.0 ||
    !std::isfinite(config.max_time_jitter_s) || config.max_time_jitter_s < 0.0 ||
    config.max_time_jitter_s >= config.sample_dt_s ||
    !std::isfinite(config.max_quaternion_norm_error) ||
    config.max_quaternion_norm_error <= 0.0 || config.max_quaternion_norm_error >= 1.0 ||
    !std::isfinite(config.max_completion_overrun_s) || config.max_completion_overrun_s < 0.0 ||
    !std::isfinite(config.max_controller_reference_position_error_rad) ||
    config.max_controller_reference_position_error_rad <= 0.0 ||
    !std::isfinite(config.max_controller_reference_velocity_error_rad_s) ||
    config.max_controller_reference_velocity_error_rad_s <= 0.0 ||
    !std::isfinite(config.max_joint_position_tracking_error_rad) ||
    config.max_joint_position_tracking_error_rad <= 0.0 ||
    !std::isfinite(config.max_joint_velocity_tracking_error_rad_s) ||
    config.max_joint_velocity_tracking_error_rad_s <= 0.0 ||
    !std::isfinite(config.max_actual_controller_position_error_rad) ||
    config.max_actual_controller_position_error_rad <= 0.0 ||
    !std::isfinite(config.max_actual_controller_velocity_error_rad_s) ||
    config.max_actual_controller_velocity_error_rad_s <= 0.0 ||
    !std::isfinite(config.max_frame_fk_position_error_m) ||
    config.max_frame_fk_position_error_m <= 0.0 ||
    !std::isfinite(config.max_frame_fk_orientation_error_deg) ||
    config.max_frame_fk_orientation_error_deg <= 0.0 ||
    config.max_frame_fk_orientation_error_deg > 180.0)
  {
    throw std::runtime_error("recording.yaml simulation validation limits are invalid");
  }
  return config;
}

SimulationKinematicsValidationContext SimulationRecordingValidator::loadKinematicsContext(
  const std::filesystem::path & environment_directory,
  const std::filesystem::path & source_trajectory_path,
  const std::vector<std::string> & expected_joint_names)
{
  SimulationKinematicsValidationContext context;
  context.robot = loadRobotValidationModel(environment_directory, expected_joint_names);
  context.setup_config_path = context.robot.environment_directory / "setup.yaml";
  if (!std::filesystem::is_regular_file(context.setup_config_path) ||
    std::filesystem::is_symlink(context.setup_config_path) ||
    !std::filesystem::is_regular_file(source_trajectory_path) ||
    std::filesystem::is_symlink(source_trajectory_path))
  {
    throw std::runtime_error(
            "simulation kinematics setup/source inputs must be regular non-symlink files");
  }
  context.source_trajectory_path = std::filesystem::canonical(source_trajectory_path);
  try {
    const YAML::Node planner = YAML::LoadFile(
      (context.robot.environment_directory / "planner.yaml").string());
    if (!planner.IsMap() || planner["schema_version"].as<int>() != 1 ||
      !planner["validation"].IsMap())
    {
      throw std::runtime_error("planner.yaml validation contract is invalid");
    }
    context.max_tip_source_position_error_m =
      planner["validation"]["tip_position_tolerance_m"].as<double>();
    context.max_tip_source_orientation_error_deg =
      planner["validation"]["tip_orientation_tolerance_deg"].as<double>();
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "failed to load planner tip validation limits: " + std::string(error.what()));
  }
  if (!std::isfinite(context.max_tip_source_position_error_m) ||
    context.max_tip_source_position_error_m < 0.0 ||
    !std::isfinite(context.max_tip_source_orientation_error_deg) ||
    context.max_tip_source_orientation_error_deg < 0.0 ||
    context.max_tip_source_orientation_error_deg > 180.0)
  {
    throw std::runtime_error("planner tip validation limits are invalid");
  }
  (void)loadBaseFrameTransforms(context.setup_config_path, context.robot);
  (void)loadSourcePoseSamples(context.source_trajectory_path);
  return context;
}

CanonicalTipPoseMetrics SimulationRecordingValidator::validateCanonicalSourcePoses(
  const trajectory_msgs::msg::JointTrajectory & commanded_trajectory,
  const SimulationKinematicsValidationContext & kinematics_context)
{
  if (commanded_trajectory.joint_names != kinematics_context.robot.joint_names ||
    commanded_trajectory.points.size() < 2U)
  {
    throw std::runtime_error("canonical source FK joint dimensions are invalid");
  }
  const auto base_frames = loadBaseFrameTransforms(
    kinematics_context.setup_config_path, kinematics_context.robot);
  const auto source_samples = loadSourcePoseSamples(
    kinematics_context.source_trajectory_path);
  TipKinematics kinematics(
    kinematics_context.robot.urdf_path,
    kinematics_context.robot.base_link,
    kinematics_context.robot.tool_link,
    kinematics_context.robot.tip_frame,
    kinematics_context.robot.tool_to_tip_translation_m,
    kinematics_context.robot.tool_to_tip_quaternion_xyzw,
    kinematics_context.robot.joint_names);

  std::vector<double> command_times;
  command_times.reserve(commanded_trajectory.points.size());
  for (const auto & point : commanded_trajectory.points) {
    command_times.push_back(pointTimeSeconds(point));
  }
  std::vector<RigidPose> source_poses_in_base;
  source_poses_in_base.reserve(source_samples.size());
  for (const auto & source : source_samples) {
    const auto frame = base_frames.find(source.frame_id);
    if (frame == base_frames.end()) {
      throw std::runtime_error(
              "setup.yaml does not resolve source frame '" + source.frame_id + "'");
    }
    source_poses_in_base.push_back(composePose(frame->second, source.frame_to_tip));
    (void)nearestTimeIndex(command_times, source.trajectory_time_s, 0.5e-9);
  }
  CanonicalTipPoseMetrics metrics;
  std::size_t source_interval = 0U;
  for (std::size_t command_index = 0U;
    command_index < commanded_trajectory.points.size(); ++command_index)
  {
    const double command_time_s = command_times[command_index];
    while (source_interval + 1U < source_samples.size() &&
      source_samples[source_interval + 1U].trajectory_time_s < command_time_s - 0.5e-9)
    {
      ++source_interval;
    }
    if (source_interval + 1U >= source_samples.size()) {
      source_interval = source_samples.size() - 2U;
    }
    const double start_time_s = source_samples[source_interval].trajectory_time_s;
    const double end_time_s = source_samples[source_interval + 1U].trajectory_time_s;
    if (command_time_s < start_time_s - 0.5e-9 || command_time_s > end_time_s + 0.5e-9) {
      throw std::runtime_error("canonical trajectory time lies outside authoritative source poses");
    }
    const double alpha = std::clamp(
      (command_time_s - start_time_s) / (end_time_s - start_time_s), 0.0, 1.0);
    RigidPose expected;
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
      expected.position_m[axis] =
        source_poses_in_base[source_interval].position_m[axis] + alpha *
        (source_poses_in_base[source_interval + 1U].position_m[axis] -
        source_poses_in_base[source_interval].position_m[axis]);
    }
    expected.quaternion_xyzw = slerpQuaternion(
      source_poses_in_base[source_interval].quaternion_xyzw,
      source_poses_in_base[source_interval + 1U].quaternion_xyzw,
      alpha);
    const auto & point = commanded_trajectory.points[command_index];
    const auto tip = kinematics.evaluate(point.positions, point.velocities);
    const RigidPose actual{
      tip.position_m,
      normalizedQuaternion(tip.quaternion_xyzw, "canonical FK tip")};
    metrics.max_tip_position_error_m = std::max(
      metrics.max_tip_position_error_m, positionError(actual, expected));
    metrics.max_tip_orientation_error_deg = std::max(
      metrics.max_tip_orientation_error_deg, orientationErrorDeg(actual, expected));
  }
  if (metrics.max_tip_position_error_m >
    kinematics_context.max_tip_source_position_error_m + 1.0e-12 ||
    metrics.max_tip_orientation_error_deg >
    kinematics_context.max_tip_source_orientation_error_deg + 1.0e-12)
  {
    throw std::runtime_error(
            "canonical joint FK does not satisfy authoritative source tip poses");
  }
  return metrics;
}

SimulationRecordingMetrics SimulationRecordingValidator::validate(
  const SimulationRecordingPaths & paths,
  const std::vector<std::string> & joint_names,
  double expected_duration_s,
  bool action_succeeded,
  const SimulationValidationConfig & config)
{
  if (joint_names.empty() ||
    std::unordered_set<std::string>(joint_names.begin(), joint_names.end()).size() !=
    joint_names.size())
  {
    throw std::invalid_argument("simulation validation requires unique joint names");
  }
  if (!std::isfinite(expected_duration_s) || expected_duration_s <= 0.0) {
    throw std::invalid_argument("simulation validation expected duration must be positive");
  }
  if (config.require_action_success && !action_succeeded) {
    throw std::runtime_error("recording.yaml requires successful simulation action completion");
  }

  const std::size_t joint_count = joint_names.size();
  const auto actual_rows = readStrictCsv(paths.actual_joint, actualHeader(joint_names));
  std::vector<double> actual_times;
  std::vector<std::vector<double>> actual_positions;
  std::vector<std::vector<double>> actual_velocities;
  actual_times.reserve(actual_rows.size());
  actual_positions.reserve(actual_rows.size());
  actual_velocities.reserve(actual_rows.size());
  for (std::size_t row_index = 0U; row_index < actual_rows.size(); ++row_index) {
    const auto & row = actual_rows[row_index];
    const std::size_t line_number = row_index + 2U;
    (void)parseFinite(row[0], paths.actual_joint, line_number, "ros_time_s");
    actual_times.push_back(
      parseFinite(row[1], paths.actual_joint, line_number, "trajectory_time_s"));
    std::vector<double> positions;
    std::vector<double> velocities;
    positions.reserve(joint_count);
    velocities.reserve(joint_count);
    for (std::size_t joint = 0U; joint < joint_count; ++joint) {
      positions.push_back(parseFinite(
          row[2U + joint], paths.actual_joint, line_number, "joint position"));
      velocities.push_back(parseFinite(
          row[2U + joint_count + joint], paths.actual_joint, line_number, "joint velocity"));
    }
    actual_positions.push_back(std::move(positions));
    actual_velocities.push_back(std::move(velocities));
  }

  const auto controller_rows = readStrictCsv(
    paths.controller_state, controllerHeader(joint_names));
  std::vector<double> controller_times;
  controller_times.reserve(controller_rows.size());
  double max_speed_scaling_error = 0.0;
  double max_position_error = 0.0;
  double max_velocity_error = 0.0;
  std::vector<std::vector<double>> controller_feedback_positions;
  std::vector<std::vector<double>> controller_feedback_velocities;
  controller_feedback_positions.reserve(controller_rows.size());
  controller_feedback_velocities.reserve(controller_rows.size());
  const std::size_t reference_position_offset = 3U;
  const std::size_t reference_velocity_offset = reference_position_offset + joint_count;
  const std::size_t feedback_position_offset = reference_velocity_offset + joint_count;
  const std::size_t feedback_velocity_offset = feedback_position_offset + joint_count;
  const std::size_t error_position_offset = feedback_velocity_offset + joint_count;
  const std::size_t error_velocity_offset = error_position_offset + joint_count;
  for (std::size_t row_index = 0U; row_index < controller_rows.size(); ++row_index) {
    const auto & row = controller_rows[row_index];
    const std::size_t line_number = row_index + 2U;
    (void)parseFinite(row[0], paths.controller_state, line_number, "ros_time_s");
    controller_times.push_back(
      parseFinite(row[1], paths.controller_state, line_number, "trajectory_time_s"));
    const double speed_scaling = parseFinite(
      row[2], paths.controller_state, line_number, "speed_scaling");
    max_speed_scaling_error = std::max(
      max_speed_scaling_error, std::abs(speed_scaling - 1.0));
    std::vector<double> feedback_positions;
    std::vector<double> feedback_velocities;
    feedback_positions.reserve(joint_count);
    feedback_velocities.reserve(joint_count);
    for (std::size_t joint = 0U; joint < joint_count; ++joint) {
      const double reference_position = parseFinite(
        row[reference_position_offset + joint], paths.controller_state, line_number,
        "reference position");
      const double reference_velocity = parseFinite(
        row[reference_velocity_offset + joint], paths.controller_state, line_number,
        "reference velocity");
      const double feedback_position = parseFinite(
        row[feedback_position_offset + joint], paths.controller_state, line_number,
        "feedback position");
      const double feedback_velocity = parseFinite(
        row[feedback_velocity_offset + joint], paths.controller_state, line_number,
        "feedback velocity");
      const double reported_position_error = parseFinite(
        row[error_position_offset + joint], paths.controller_state, line_number,
        "reported position error");
      const double reported_velocity_error = parseFinite(
        row[error_velocity_offset + joint], paths.controller_state, line_number,
        "reported velocity error");
      feedback_positions.push_back(feedback_position);
      feedback_velocities.push_back(feedback_velocity);
      max_position_error = std::max(
        max_position_error,
        std::max(
          std::abs(reference_position - feedback_position),
          std::abs(reported_position_error)));
      max_velocity_error = std::max(
        max_velocity_error,
        std::max(
          std::abs(reference_velocity - feedback_velocity),
          std::abs(reported_velocity_error)));
    }
    controller_feedback_positions.push_back(std::move(feedback_positions));
    controller_feedback_velocities.push_back(std::move(feedback_velocities));
  }

  const auto frame_rows = readStrictCsv(paths.joint_frames, jointFramesHeader());
  std::map<std::string, PoseStreamData> frame_streams;
  for (std::size_t row_index = 0U; row_index < frame_rows.size(); ++row_index) {
    const auto & row = frame_rows[row_index];
    const std::size_t line_number = row_index + 2U;
    if (row[2].empty()) {
      throw std::runtime_error(
              "joint_frames.csv contains an empty frame_id at line " +
              std::to_string(line_number));
    }
    appendPoseSample(
      row, paths.joint_frames, line_number, 3U, 6U, frame_streams[row[2]]);
  }

  const auto tip_rows = readStrictCsv(paths.actual_tip, actualTipHeader());
  PoseStreamData tip_stream;
  for (std::size_t row_index = 0U; row_index < tip_rows.size(); ++row_index) {
    appendPoseSample(
      tip_rows[row_index], paths.actual_tip, row_index + 2U, 2U, 5U, tip_stream);
  }

  SimulationRecordingMetrics metrics;
  metrics.actual_joint = analyzeSampling(
    actual_times, expected_duration_s, config, "sim_actual_joint.csv");
  metrics.controller_state = analyzeSampling(
    controller_times, expected_duration_s, config, "sim_controller_state.csv");
  metrics.joint_frame_sample_count = frame_rows.size();
  metrics.joint_frame_streams.reserve(frame_streams.size());
  for (const auto & [frame_id, frame_stream] : frame_streams) {
    FrameValidationMetrics frame_metrics;
    frame_metrics.frame_id = frame_id;
    frame_metrics.pose = analyzePoseStream(
      frame_stream, expected_duration_s, config,
      "joint_frames.csv frame '" + frame_id + "'");
    metrics.max_joint_frame_quaternion_norm_error = std::max(
      metrics.max_joint_frame_quaternion_norm_error,
      frame_metrics.pose.max_quaternion_norm_error);
    metrics.joint_frame_streams.push_back(std::move(frame_metrics));
  }
  metrics.actual_tip = analyzePoseStream(
    tip_stream, expected_duration_s, config, "sim_actual_tip.csv");
  metrics.max_speed_scaling_error = max_speed_scaling_error;
  metrics.max_joint_position_tracking_error_rad = max_position_error;
  metrics.max_joint_velocity_tracking_error_rad_s = max_velocity_error;
  std::size_t nearest_controller_index = 0U;
  const double actual_controller_match_tolerance_s =
    config.sample_dt_s + config.max_time_jitter_s;
  for (std::size_t actual_index = 0U; actual_index < actual_times.size(); ++actual_index) {
    while (nearest_controller_index + 1U < controller_times.size() &&
      std::abs(controller_times[nearest_controller_index + 1U] - actual_times[actual_index]) <=
      std::abs(controller_times[nearest_controller_index] - actual_times[actual_index]))
    {
      ++nearest_controller_index;
    }
    if (std::abs(controller_times[nearest_controller_index] - actual_times[actual_index]) >
      actual_controller_match_tolerance_s + 1.0e-12)
    {
      throw std::runtime_error(
              "sim_actual_joint.csv has no nearby controller feedback sample");
    }
    for (std::size_t joint = 0U; joint < joint_count; ++joint) {
      metrics.max_actual_controller_position_error_rad = std::max(
        metrics.max_actual_controller_position_error_rad,
        std::abs(
          actual_positions[actual_index][joint] -
          controller_feedback_positions[nearest_controller_index][joint]));
      metrics.max_actual_controller_velocity_error_rad_s = std::max(
        metrics.max_actual_controller_velocity_error_rad_s,
        std::abs(
          actual_velocities[actual_index][joint] -
          controller_feedback_velocities[nearest_controller_index][joint]));
    }
  }
  if (max_speed_scaling_error > kSpeedScalingTolerance) {
    throw std::runtime_error(
            "sim_controller_state.csv speed_scaling must remain 1 during simulation recording");
  }
  if (max_position_error > config.max_joint_position_tracking_error_rad + 1.0e-12) {
    throw std::runtime_error(
            "simulation exceeds max_joint_position_tracking_error_rad");
  }
  if (max_velocity_error > config.max_joint_velocity_tracking_error_rad_s + 1.0e-12) {
    throw std::runtime_error(
            "simulation exceeds max_joint_velocity_tracking_error_rad_s");
  }
  if (metrics.max_actual_controller_position_error_rad >
    config.max_actual_controller_position_error_rad + 1.0e-12)
  {
    throw std::runtime_error(
            "sim_actual_joint.csv positions differ from controller feedback");
  }
  if (metrics.max_actual_controller_velocity_error_rad_s >
    config.max_actual_controller_velocity_error_rad_s + 1.0e-12)
  {
    throw std::runtime_error(
            "sim_actual_joint.csv velocities differ from controller feedback");
  }
  return metrics;
}

SimulationRecordingMetrics SimulationRecordingValidator::validate(
  const SimulationRecordingPaths & paths,
  const trajectory_msgs::msg::JointTrajectory & commanded_trajectory,
  bool action_succeeded,
  const SimulationValidationConfig & config)
{
  if (commanded_trajectory.points.size() < 2U) {
    throw std::invalid_argument(
            "simulation validation requires a complete commanded trajectory");
  }
  const double expected_duration_s = pointTimeSeconds(commanded_trajectory.points.back());
  auto metrics = validate(
    paths, commanded_trajectory.joint_names, expected_duration_s, action_succeeded, config);
  const auto reference = compareCommandedControllerReference(
    paths, commanded_trajectory, config);
  metrics.max_controller_reference_position_error_rad = reference.max_position_error_rad;
  metrics.max_controller_reference_velocity_error_rad_s = reference.max_velocity_error_rad_s;
  if (reference.max_position_error_rad >
    config.max_controller_reference_position_error_rad + 1.0e-12)
  {
    throw std::runtime_error(
            "simulation controller reference differs from canonical commanded positions");
  }
  if (reference.max_velocity_error_rad_s >
    config.max_controller_reference_velocity_error_rad_s + 1.0e-12)
  {
    throw std::runtime_error(
            "simulation controller reference differs from canonical commanded velocities");
  }
  return metrics;
}

SimulationRecordingMetrics SimulationRecordingValidator::validate(
  const SimulationRecordingPaths & paths,
  const trajectory_msgs::msg::JointTrajectory & commanded_trajectory,
  bool action_succeeded,
  const SimulationValidationConfig & config,
  const SimulationKinematicsValidationContext & kinematics_context)
{
  auto metrics = validate(paths, commanded_trajectory, action_succeeded, config);
  if (commanded_trajectory.joint_names != kinematics_context.robot.joint_names ||
    kinematics_context.robot.joint_limits.size() != commanded_trajectory.joint_names.size())
  {
    throw std::runtime_error("simulation FK context does not match canonical joint_names");
  }

  const auto actual_rows = readStrictCsv(
    paths.actual_joint, actualHeader(commanded_trajectory.joint_names));
  const std::size_t joint_count = commanded_trajectory.joint_names.size();
  std::vector<double> actual_times;
  std::vector<std::vector<double>> actual_positions;
  actual_times.reserve(actual_rows.size());
  actual_positions.reserve(actual_rows.size());
  for (std::size_t row_index = 0U; row_index < actual_rows.size(); ++row_index) {
    const auto & row = actual_rows[row_index];
    const std::size_t line_number = row_index + 2U;
    actual_times.push_back(parseFinite(
        row[1], paths.actual_joint, line_number, "trajectory_time_s"));
    std::vector<double> positions;
    positions.reserve(joint_count);
    for (std::size_t joint = 0U; joint < joint_count; ++joint) {
      const double position = parseFinite(
        row[2U + joint], paths.actual_joint, line_number, "joint position");
      const auto & limit = kinematics_context.robot.joint_limits[joint];
      if (position < limit.lower_position_rad - 1.0e-9 ||
        position > limit.upper_position_rad + 1.0e-9)
      {
        throw std::runtime_error("sim_actual_joint.csv exceeds hash-bound URDF position limits");
      }
      positions.push_back(position);
    }
    actual_positions.push_back(std::move(positions));
  }

  const auto frame_rows = readStrictCsv(paths.joint_frames, jointFramesHeader());
  std::map<std::string, PoseStreamData> frame_streams;
  for (std::size_t row_index = 0U; row_index < frame_rows.size(); ++row_index) {
    const auto & row = frame_rows[row_index];
    if (row[2].empty()) {
      throw std::runtime_error("joint_frames.csv contains an empty frame_id");
    }
    appendPoseSample(
      row, paths.joint_frames, row_index + 2U, 3U, 6U, frame_streams[row[2]]);
  }
  const std::set<std::string> required_frames(
    kRequiredSimulationFrameNames.begin(), kRequiredSimulationFrameNames.end());
  std::set<std::string> recorded_frames;
  for (const auto & [frame_id, unused] : frame_streams) {
    (void)unused;
    recorded_frames.insert(frame_id);
  }
  if (required_frames.size() != kRequiredSimulationFrameNames.size() ||
    recorded_frames != required_frames)
  {
    throw std::runtime_error(
            "joint_frames.csv must contain exactly the required simulation frame set");
  }

  const auto tip_rows = readStrictCsv(paths.actual_tip, actualTipHeader());
  PoseStreamData tip_stream;
  for (std::size_t row_index = 0U; row_index < tip_rows.size(); ++row_index) {
    appendPoseSample(
      tip_rows[row_index], paths.actual_tip, row_index + 2U, 2U, 5U, tip_stream);
  }

  TipKinematics kinematics(
    kinematics_context.robot.urdf_path,
    kinematics_context.robot.base_link,
    kinematics_context.robot.tool_link,
    kinematics_context.robot.tip_frame,
    kinematics_context.robot.tool_to_tip_translation_m,
    kinematics_context.robot.tool_to_tip_quaternion_xyzw,
    kinematics_context.robot.joint_names,
    kRequiredSimulationFrameNames);
  std::unordered_map<std::string, std::size_t> frame_indices;
  for (std::size_t index = 0U; index < kRequiredSimulationFrameNames.size(); ++index) {
    frame_indices.emplace(kRequiredSimulationFrameNames[index], index);
  }
  std::vector<std::vector<FrameKinematicsResult>> predicted_frames(actual_times.size());
  std::vector<bool> predicted(actual_times.size(), false);
  const auto prediction_at = [&](std::size_t actual_index) ->
    const std::vector<FrameKinematicsResult> & {
      if (!predicted[actual_index]) {
        predicted_frames[actual_index] = kinematics.evaluateFrames(actual_positions[actual_index]);
        if (predicted_frames[actual_index].size() != kRequiredSimulationFrameNames.size()) {
          throw std::runtime_error("KDL did not return every required simulation frame");
        }
        predicted[actual_index] = true;
      }
      return predicted_frames[actual_index];
    };

  for (const auto & [frame_id, stream] : frame_streams) {
    const auto frame_index = frame_indices.find(frame_id);
    if (frame_index == frame_indices.end() ||
      stream.trajectory_times.size() != stream.positions_m.size() ||
      stream.trajectory_times.size() != stream.quaternions_xyzw.size())
    {
      throw std::runtime_error("joint frame pose stream is internally inconsistent");
    }
    for (std::size_t sample = 0U; sample < stream.trajectory_times.size(); ++sample) {
      const std::size_t actual_index = nearestTimeIndex(
        actual_times, stream.trajectory_times[sample], 1.0e-12);
      const auto & expected_frame = prediction_at(actual_index)[frame_index->second];
      if (expected_frame.frame_id != frame_id) {
        throw std::runtime_error("KDL required frame order is inconsistent");
      }
      const RigidPose actual_pose{stream.positions_m[sample], stream.quaternions_xyzw[sample]};
      const RigidPose expected_pose{
        expected_frame.position_m,
        normalizedQuaternion(expected_frame.quaternion_xyzw, "frame FK")};
      metrics.max_frame_fk_position_error_m = std::max(
        metrics.max_frame_fk_position_error_m, positionError(actual_pose, expected_pose));
      metrics.max_frame_fk_orientation_error_deg = std::max(
        metrics.max_frame_fk_orientation_error_deg,
        orientationErrorDeg(actual_pose, expected_pose));
    }
  }

  const auto rod_tip_index = frame_indices.find(kinematics_context.robot.tip_frame);
  if (rod_tip_index == frame_indices.end() ||
    tip_stream.trajectory_times.size() != tip_stream.positions_m.size() ||
    tip_stream.trajectory_times.size() != tip_stream.quaternions_xyzw.size())
  {
    throw std::runtime_error("actual tip stream or configured tip frame is invalid");
  }
  for (std::size_t sample = 0U; sample < tip_stream.trajectory_times.size(); ++sample) {
    const std::size_t actual_index = nearestTimeIndex(
      actual_times, tip_stream.trajectory_times[sample], 1.0e-12);
    const auto & expected_frame = prediction_at(actual_index)[rod_tip_index->second];
    const RigidPose actual_pose{
      tip_stream.positions_m[sample], tip_stream.quaternions_xyzw[sample]};
    const RigidPose expected_pose{
      expected_frame.position_m,
      normalizedQuaternion(expected_frame.quaternion_xyzw, "tip FK")};
    metrics.max_tip_fk_position_error_m = std::max(
      metrics.max_tip_fk_position_error_m, positionError(actual_pose, expected_pose));
    metrics.max_tip_fk_orientation_error_deg = std::max(
      metrics.max_tip_fk_orientation_error_deg,
      orientationErrorDeg(actual_pose, expected_pose));
  }

  const auto base_frames = loadBaseFrameTransforms(
    kinematics_context.setup_config_path, kinematics_context.robot);
  const auto source_samples = loadSourcePoseSamples(
    kinematics_context.source_trajectory_path);
  const double source_match_tolerance_s = config.sample_dt_s + config.max_time_jitter_s;
  for (const auto & source : source_samples) {
    const auto frame = base_frames.find(source.frame_id);
    if (frame == base_frames.end()) {
      throw std::runtime_error(
              "setup.yaml does not resolve source frame '" + source.frame_id + "'");
    }
    // Telemetry is sampled independently from the authoritative source grid. Comparing a
    // nearest 125 Hz sample directly with an exact source timestamp adds a path-speed-dependent
    // phase error of up to half a controller period. Interpolate the recorded pose to the exact
    // source time instead; the fixed threshold and every source timestamp remain unchanged.
    const RigidPose actual_pose = interpolatePoseStreamAtTime(
      tip_stream, source.trajectory_time_s, source_match_tolerance_s);
    const RigidPose expected_pose = composePose(frame->second, source.frame_to_tip);
    metrics.max_tip_source_position_error_m = std::max(
      metrics.max_tip_source_position_error_m, positionError(actual_pose, expected_pose));
    metrics.max_tip_source_orientation_error_deg = std::max(
      metrics.max_tip_source_orientation_error_deg,
      orientationErrorDeg(actual_pose, expected_pose));
  }

  if (metrics.max_frame_fk_position_error_m > config.max_frame_fk_position_error_m + 1.0e-12 ||
    metrics.max_frame_fk_orientation_error_deg >
    config.max_frame_fk_orientation_error_deg + 1.0e-12 ||
    metrics.max_tip_fk_position_error_m > config.max_frame_fk_position_error_m + 1.0e-12 ||
    metrics.max_tip_fk_orientation_error_deg >
    config.max_frame_fk_orientation_error_deg + 1.0e-12 ||
    metrics.max_tip_source_position_error_m >
    kinematics_context.max_tip_source_position_error_m + 1.0e-12 ||
    metrics.max_tip_source_orientation_error_deg >
    kinematics_context.max_tip_source_orientation_error_deg + 1.0e-12)
  {
    throw std::runtime_error(
            "simulation FK or authoritative source pose validation exceeded its limit");
  }
  return metrics;
}

}  // namespace hx_waypoint
