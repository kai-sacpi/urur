#!/usr/bin/env python3
"""Plan a collision-checked joint-space prepare with MoveIt and execute exact output."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient


JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def _seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", type=float, nargs=6, required=True)
    parser.add_argument("--workspace", default="/home/huaxin/Codes/ur-waypoint")
    parser.add_argument("--allowed-planning-time", type=float, default=15.0)
    parser.add_argument("--planning-attempts", type=int, default=10)
    parser.add_argument("--velocity-scale", type=float, default=0.15)
    parser.add_argument("--acceleration-scale", type=float, default=0.15)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    target = [float(value) for value in args.target]
    if not all(math.isfinite(value) for value in target):
        parser.error("--target values must be finite")

    rclpy.init()
    node = rclpy.create_node("hx_moveit_joint_prepare")
    move_group = ActionClient(node, MoveGroup, "/move_action")
    controller = ActionClient(
        node,
        FollowJointTrajectory,
        "/joint_trajectory_controller/follow_joint_trajectory",
    )
    try:
        if not move_group.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveIt /move_action is unavailable")

        constraints = Constraints(name="authoritative_home_joint_goal")
        constraints.joint_constraints = [
            JointConstraint(
                joint_name=name,
                position=position,
                tolerance_above=1e-4,
                tolerance_below=1e-4,
                weight=1.0,
            )
            for name, position in zip(JOINT_NAMES, target, strict=True)
        ]
        goal = MoveGroup.Goal()
        goal.request.group_name = "ur_manipulator"
        goal.request.num_planning_attempts = int(args.planning_attempts)
        goal.request.allowed_planning_time = float(args.allowed_planning_time)
        goal.request.max_velocity_scaling_factor = float(args.velocity_scale)
        goal.request.max_acceleration_scaling_factor = float(args.acceleration_scale)
        goal.request.pipeline_id = "ompl"
        goal.request.planner_id = "RRTConnectkConfigDefault"
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True

        future = move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("MoveIt rejected the joint prepare request")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        wrapped = result_future.result()
        result = wrapped.result
        if int(result.error_code.val) != int(MoveItErrorCodes.SUCCESS):
            raise RuntimeError(f"MoveIt planning failed with code {result.error_code.val}")

        trajectory = result.planned_trajectory.joint_trajectory
        if not trajectory.points:
            raise RuntimeError("MoveIt returned an empty trajectory")
        if set(trajectory.joint_names) != set(JOINT_NAMES):
            raise RuntimeError(
                f"unexpected MoveIt joint order/content: {list(trajectory.joint_names)}"
            )
        target_by_name = dict(zip(JOINT_NAMES, target, strict=True))
        final = trajectory.points[-1]
        final_error = max(
            abs(float(value) - target_by_name[name])
            for name, value in zip(trajectory.joint_names, final.positions, strict=True)
        )
        if final_error > 1e-3:
            raise RuntimeError(f"MoveIt endpoint error {final_error} rad exceeds 1e-3")

        times = [_seconds(point.time_from_start) for point in trajectory.points]
        if any(not math.isfinite(value) for value in times):
            raise RuntimeError("MoveIt trajectory contains non-finite timestamps")
        if any(right <= left for left, right in zip(times, times[1:])):
            raise RuntimeError("MoveIt trajectory timestamps are not strictly increasing")
        for point in trajectory.points:
            values = list(point.positions) + list(point.velocities) + list(point.accelerations)
            if any(not math.isfinite(float(value)) for value in values):
                raise RuntimeError("MoveIt trajectory contains non-finite joint values")

        evidence = {
            "planner": "MoveIt OMPL RRTConnectkConfigDefault",
            "collision_checking": True,
            "plan_only_request": True,
            "controller": "/joint_trajectory_controller/follow_joint_trajectory",
            "joint_names": list(trajectory.joint_names),
            "target_positions_rad": target,
            "point_count": len(trajectory.points),
            "duration_s": times[-1],
            "final_error_rad": final_error,
            "executed": False,
        }
        runs = Path(args.workspace).expanduser().resolve() / "runs"
        runs.mkdir(parents=True, exist_ok=True)
        path = runs / f"moveit_prepare_home__{time.strftime('%Y%m%d_%H%M%S')}.json"
        path.write_text(json.dumps(evidence, indent=2) + "\n", encoding="utf-8")
        print(f"planned: {path}", flush=True)

        if not args.execute:
            return 0
        if not controller.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("unscaled joint trajectory controller is unavailable")
        execute_goal = FollowJointTrajectory.Goal()
        execute_goal.trajectory = trajectory
        execute_future = controller.send_goal_async(execute_goal)
        rclpy.spin_until_future_complete(node, execute_future)
        execute_handle = execute_future.result()
        if execute_handle is None or not execute_handle.accepted:
            raise RuntimeError("unscaled controller rejected the MoveIt prepare trajectory")
        done_future = execute_handle.get_result_async()
        rclpy.spin_until_future_complete(node, done_future)
        done = done_future.result().result
        if int(done.error_code) != int(FollowJointTrajectory.Result.SUCCESSFUL):
            raise RuntimeError(
                f"unscaled controller failed: {done.error_code} {done.error_string}"
            )
        evidence["executed"] = True
        evidence["controller_result"] = "SUCCESSFUL"
        path.write_text(json.dumps(evidence, indent=2) + "\n", encoding="utf-8")
        print(f"executed: {path}", flush=True)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
