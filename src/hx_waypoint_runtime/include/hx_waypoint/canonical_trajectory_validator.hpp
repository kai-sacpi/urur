#pragma once

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "hx_waypoint/robot_validation_model.hpp"

namespace hx_waypoint
{

struct CanonicalTrajectoryMetrics
{
  double max_joint_velocity_ratio{0.0};
  double max_joint_acceleration_ratio{0.0};
  double max_joint_jerk_ratio{0.0};
};

[[nodiscard]] CanonicalTrajectoryMetrics validateCanonicalTrajectoryDynamics(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const RobotValidationModel & model);

}  // namespace hx_waypoint
