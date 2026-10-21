from __future__ import annotations

from math import floor, isfinite
from pathlib import Path
from threading import Lock

from .curobo_adapter_v2 import CuroboAdapterV2
from .environment_builder import EnvironmentBuilder
from .models import CompiledTrajectory
from .prepare_planner import PreparePlanner


def _environment_evidence(hashes) -> tuple[list[str], list[str]]:
    ordered = sorted((str(path), str(digest)) for path, digest in hashes.items())
    return [path for path, _ in ordered], [digest for _, digest in ordered]


def _to_ros_trajectory(compiled: CompiledTrajectory):
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    trajectory = JointTrajectory()
    trajectory.joint_names = list(compiled.joint_names)
    for sample in compiled.samples:
        if not isfinite(sample.time_s) or sample.time_s < 0.0:
            raise ValueError("prepare trajectory contains an invalid timestamp")
        seconds = floor(sample.time_s)
        nanoseconds = round((sample.time_s - seconds) * 1_000_000_000)
        if nanoseconds == 1_000_000_000:
            seconds += 1
            nanoseconds = 0
        if seconds > 2_147_483_647:
            raise ValueError("prepare trajectory duration exceeds ROS Duration range")
        point = JointTrajectoryPoint()
        point.positions = list(sample.positions)
        point.velocities = list(sample.velocities)
        point.accelerations = list(sample.accelerations)
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int(nanoseconds)
        trajectory.points.append(point)
    return trajectory


def main(args: list[str] | None = None) -> None:
    import rclpy
    from hx_waypoint_interfaces.srv import PlanJointPath
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    class CuroboServer(Node):
        def __init__(self) -> None:
            super().__init__("hx_waypoint_curobo_server")
            self.declare_parameter("workspace", str(Path.cwd()))
            self.declare_parameter("environment_id", "lab_wall_a")
            workspace = Path(self.get_parameter("workspace").value).expanduser().resolve()
            self.environment_id = str(self.get_parameter("environment_id").value)
            self.environment = EnvironmentBuilder().load(
                workspace / "environments" / self.environment_id
            )
            self.adapter = CuroboAdapterV2(self.environment, workspace / "curobo.lock")
            self.planner = PreparePlanner(self.environment, self.adapter)
            self._planning_lock = Lock()
            self._warmed = False
            self.warmup_service = self.create_service(
                Trigger,
                "/hx_waypoint/curobo/warmup",
                self._warmup,
            )
            self.plan_service = self.create_service(
                PlanJointPath,
                "/hx_waypoint/curobo/plan_joint_path",
                self._plan_joint_path,
            )
            self.get_logger().info(
                f"cuRoboV2 server ready for environment={self.environment_id}, "
                f"revision={self.adapter.revision}"
            )

        def _warmup(self, request, response):
            del request
            try:
                with self._planning_lock:
                    self._ensure_environment_unchanged()
                    self._ensure_warm()
                response.success = True
                response.message = f"cuRoboV2 warm at {self.adapter.revision}"
            except Exception as error:  # ROS service boundary must return the failure.
                response.success = False
                response.message = str(error)
            return response

        def _plan_joint_path(self, request, response):
            response.environment_paths, response.environment_sha256 = _environment_evidence(
                self.environment.hashes
            )
            try:
                if request.environment_id != self.environment_id:
                    raise ValueError(
                        "prepare request environment_id does not match the fixed server environment"
                    )
                with self._planning_lock:
                    self._ensure_environment_unchanged()
                    self._ensure_warm()
                    compiled = self.planner.plan(
                        request.joint_names,
                        request.current_positions,
                        request.target_positions,
                    )
                response.trajectory = _to_ros_trajectory(compiled)
                response.success = True
                response.message = "cuRobo prepare trajectory planned and dynamically validated"
                response.curobo_revision = self.adapter.revision
            except Exception as error:  # ROS service boundary must return the failure.
                response.success = False
                response.message = str(error)
                response.curobo_revision = self.adapter.revision
            return response

        def _ensure_warm(self) -> None:
            if not self._warmed:
                self.adapter.warmup()
                self._warmed = True

        def _ensure_environment_unchanged(self) -> None:
            current = EnvironmentBuilder().load(self.environment.root)
            if current.environment_id != self.environment_id:
                raise ValueError("fixed cuRobo environment_id changed after server startup")
            if dict(current.hashes) != dict(self.environment.hashes):
                raise ValueError(
                    "cuRobo environment files changed after server startup; restart is required"
                )

    rclpy.init(args=args)
    node = CuroboServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
