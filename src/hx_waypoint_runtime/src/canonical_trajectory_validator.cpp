#include "hx_waypoint/canonical_trajectory_validator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace hx_waypoint
{
namespace
{

constexpr double kRootTimeTolerance = 1.0e-12;
constexpr double kJointBoundTolerance = 1.0e-9;
constexpr double kDynamicRatioTolerance = 1.0e-9;

[[nodiscard]] double pointTimeSeconds(
  const trajectory_msgs::msg::JointTrajectoryPoint & point)
{
  return static_cast<double>(point.time_from_start.sec) +
         static_cast<double>(point.time_from_start.nanosec) * 1.0e-9;
}

void trimPolynomial(std::vector<double> & coefficients)
{
  double scale = 0.0;
  for (const double coefficient : coefficients) {
    scale = std::max(scale, std::abs(coefficient));
  }
  const double tolerance = std::max(1.0, scale) * 1.0e-14;
  while (coefficients.size() > 1U && std::abs(coefficients.back()) <= tolerance) {
    coefficients.pop_back();
  }
}

[[nodiscard]] double evaluatePolynomial(
  const std::vector<double> & coefficients,
  double value)
{
  double result = 0.0;
  for (auto iterator = coefficients.rbegin(); iterator != coefficients.rend(); ++iterator) {
    result = result * value + *iterator;
  }
  return result;
}

[[nodiscard]] std::vector<double> derivative(
  const std::vector<double> & coefficients)
{
  if (coefficients.size() <= 1U) {
    return {0.0};
  }
  std::vector<double> result(coefficients.size() - 1U, 0.0);
  for (std::size_t index = 1U; index < coefficients.size(); ++index) {
    result[index - 1U] = static_cast<double>(index) * coefficients[index];
  }
  trimPolynomial(result);
  return result;
}

void appendUniqueRoot(std::vector<double> & roots, double root)
{
  root = std::clamp(root, 0.0, 1.0);
  if (std::none_of(roots.begin(), roots.end(), [root](double existing) {
      return std::abs(existing - root) <= kRootTimeTolerance;
    }))
  {
    roots.push_back(root);
  }
}

[[nodiscard]] std::vector<double> realRootsOnUnitInterval(
  std::vector<double> coefficients)
{
  trimPolynomial(coefficients);
  const std::size_t degree = coefficients.size() - 1U;
  std::vector<double> roots;
  if (degree == 0U) {
    return roots;
  }
  if (degree == 1U) {
    const double root = -coefficients[0] / coefficients[1];
    if (std::isfinite(root) && root >= -kRootTimeTolerance &&
      root <= 1.0 + kRootTimeTolerance)
    {
      appendUniqueRoot(roots, root);
    }
    return roots;
  }

  auto critical = realRootsOnUnitInterval(derivative(coefficients));
  std::sort(critical.begin(), critical.end());
  std::vector<double> boundaries{0.0};
  boundaries.insert(boundaries.end(), critical.begin(), critical.end());
  boundaries.push_back(1.0);

  double value_scale = 0.0;
  for (const double coefficient : coefficients) {
    value_scale += std::abs(coefficient);
  }
  const double value_tolerance = std::max(1.0, value_scale) * 1.0e-12;
  for (const double critical_root : critical) {
    if (std::abs(evaluatePolynomial(coefficients, critical_root)) <= value_tolerance) {
      appendUniqueRoot(roots, critical_root);
    }
  }
  for (std::size_t interval = 1U; interval < boundaries.size(); ++interval) {
    double left = boundaries[interval - 1U];
    double right = boundaries[interval];
    double left_value = evaluatePolynomial(coefficients, left);
    const double right_value = evaluatePolynomial(coefficients, right);
    if (std::abs(left_value) <= value_tolerance) {
      appendUniqueRoot(roots, left);
    }
    if (std::abs(right_value) <= value_tolerance) {
      appendUniqueRoot(roots, right);
    }
    if ((left_value < 0.0 && right_value < 0.0) ||
      (left_value > 0.0 && right_value > 0.0) ||
      std::abs(left_value) <= value_tolerance ||
      std::abs(right_value) <= value_tolerance)
    {
      continue;
    }
    for (int iteration = 0; iteration < 100; ++iteration) {
      const double midpoint = 0.5 * (left + right);
      const double midpoint_value = evaluatePolynomial(coefficients, midpoint);
      if (std::abs(midpoint_value) <= value_tolerance ||
        right - left <= kRootTimeTolerance)
      {
        left = midpoint;
        right = midpoint;
        break;
      }
      if ((left_value < 0.0) != (midpoint_value < 0.0)) {
        right = midpoint;
      } else {
        left = midpoint;
        left_value = midpoint_value;
      }
    }
    appendUniqueRoot(roots, 0.5 * (left + right));
  }
  std::sort(roots.begin(), roots.end());
  return roots;
}

[[nodiscard]] std::vector<double> extremaCandidates(
  const std::vector<double> & polynomial)
{
  std::vector<double> candidates{0.0, 1.0};
  for (const double root : realRootsOnUnitInterval(derivative(polynomial))) {
    appendUniqueRoot(candidates, root);
  }
  return candidates;
}

[[nodiscard]] std::vector<double> quinticCoefficients(
  double q0,
  double qd0,
  double qdd0,
  double q1,
  double qd1,
  double qdd1,
  double duration_s)
{
  const double scaled_velocity_0 = qd0 * duration_s;
  const double scaled_velocity_1 = qd1 * duration_s;
  const double scaled_acceleration_0 = qdd0 * duration_s * duration_s;
  const double scaled_acceleration_1 = qdd1 * duration_s * duration_s;
  const double c0 = q0;
  const double c1 = scaled_velocity_0;
  const double c2 = 0.5 * scaled_acceleration_0;
  const double delta_position = q1 - (c0 + c1 + c2);
  const double delta_velocity = scaled_velocity_1 - (c1 + 2.0 * c2);
  const double delta_acceleration = scaled_acceleration_1 - 2.0 * c2;
  return {
    c0, c1, c2,
    10.0 * delta_position - 4.0 * delta_velocity + 0.5 * delta_acceleration,
    -15.0 * delta_position + 7.0 * delta_velocity - delta_acceleration,
    6.0 * delta_position - 3.0 * delta_velocity + 0.5 * delta_acceleration};
}

[[nodiscard]] double maximumAbsoluteValue(
  const std::vector<double> & polynomial,
  const std::vector<double> & candidates)
{
  double maximum = 0.0;
  for (const double candidate : candidates) {
    maximum = std::max(maximum, std::abs(evaluatePolynomial(polynomial, candidate)));
  }
  return maximum;
}

}  // namespace

CanonicalTrajectoryMetrics validateCanonicalTrajectoryDynamics(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const RobotValidationModel & model)
{
  if (trajectory.joint_names != model.joint_names ||
    model.joint_limits.size() != model.joint_names.size() ||
    trajectory.points.size() < 2U)
  {
    throw std::runtime_error(
            "canonical trajectory and robot validation model dimensions do not match");
  }

  CanonicalTrajectoryMetrics metrics;
  double previous_time_s = pointTimeSeconds(trajectory.points.front());
  for (std::size_t point_index = 0U; point_index < trajectory.points.size(); ++point_index) {
    const auto & point = trajectory.points[point_index];
    if (point.positions.size() != model.joint_names.size() ||
      point.velocities.size() != model.joint_names.size() ||
      point.accelerations.size() != model.joint_names.size())
    {
      throw std::runtime_error("canonical trajectory q/qd/qdd dimensions are incomplete");
    }
    if (point_index == 0U) {
      if (previous_time_s != 0.0) {
        throw std::runtime_error("canonical trajectory must start at time zero");
      }
      continue;
    }
    const double current_time_s = pointTimeSeconds(point);
    const double duration_s = current_time_s - previous_time_s;
    if (!std::isfinite(duration_s) || duration_s <= 0.0) {
      throw std::runtime_error("canonical trajectory segment duration must be positive");
    }
    const auto & previous = trajectory.points[point_index - 1U];
    for (std::size_t joint = 0U; joint < model.joint_names.size(); ++joint) {
      const std::vector<double> position = quinticCoefficients(
        previous.positions[joint], previous.velocities[joint], previous.accelerations[joint],
        point.positions[joint], point.velocities[joint], point.accelerations[joint], duration_s);
      if (!std::all_of(position.begin(), position.end(), [](double value) {
          return std::isfinite(value);
        }))
      {
        throw std::runtime_error("canonical quintic coefficients are non-finite");
      }
      const auto velocity_scaled = derivative(position);
      const auto acceleration_scaled = derivative(velocity_scaled);
      const auto jerk_scaled = derivative(acceleration_scaled);
      const auto position_candidates = extremaCandidates(position);
      for (const double candidate : position_candidates) {
        const double value = evaluatePolynomial(position, candidate);
        const auto & limit = model.joint_limits[joint];
        if (value < limit.lower_position_rad - kJointBoundTolerance ||
          value > limit.upper_position_rad + kJointBoundTolerance)
        {
          throw std::runtime_error(
                  "canonical quintic exceeds URDF position bounds for joint '" +
                  model.joint_names[joint] + "'");
        }
      }
      const auto & limit = model.joint_limits[joint];
      metrics.max_joint_velocity_ratio = std::max(
        metrics.max_joint_velocity_ratio,
        maximumAbsoluteValue(velocity_scaled, extremaCandidates(velocity_scaled)) /
        duration_s / limit.max_velocity_rad_s);
      metrics.max_joint_acceleration_ratio = std::max(
        metrics.max_joint_acceleration_ratio,
        maximumAbsoluteValue(acceleration_scaled, extremaCandidates(acceleration_scaled)) /
        (duration_s * duration_s) / limit.max_acceleration_rad_s2);
      metrics.max_joint_jerk_ratio = std::max(
        metrics.max_joint_jerk_ratio,
        maximumAbsoluteValue(jerk_scaled, extremaCandidates(jerk_scaled)) /
        (duration_s * duration_s * duration_s) / limit.max_jerk_rad_s3);
    }
    previous_time_s = current_time_s;
  }
  if (!std::isfinite(metrics.max_joint_velocity_ratio) ||
    !std::isfinite(metrics.max_joint_acceleration_ratio) ||
    !std::isfinite(metrics.max_joint_jerk_ratio) ||
    metrics.max_joint_velocity_ratio > 1.0 + kDynamicRatioTolerance ||
    metrics.max_joint_acceleration_ratio > 1.0 + kDynamicRatioTolerance ||
    metrics.max_joint_jerk_ratio > 1.0 + kDynamicRatioTolerance)
  {
    throw std::runtime_error(
            "canonical quintic violates robot_model velocity/acceleration/jerk limits");
  }
  return metrics;
}

}  // namespace hx_waypoint
