"""Bounded, read-only live ROS readiness before expensive release validation.

No ROS, GPU, serial, planner goal or controller goal is imported/created at module
import time.  This probe does not replace the frozen release or motion gates.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
import time
from typing import Any, Callable, Mapping, Protocol
from uuid import uuid4

import yaml


class ReadinessError(RuntimeError):
    def __init__(self, report: dict[str, Any]):
        self.report = report
        super().__init__("read-only ROS preflight failed: " + "; ".join(report["errors"]))


@dataclass(frozen=True)
class JointFeedback:
    names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    source_stamp_ns: int
    receipt_monotonic_s: float


class ReadinessBackend(Protocol):
    """poll spins for at most timeout_s and returns newly received feedback.

    Snapshot keys: runtime (parameter mapping or None), controllers (list of
    name/state/type mappings or None), action_ready (bool), joints (new
    JointFeedback list), optional runtime_error/controllers_error/program_running.
    Service requests are sent once, never retried by this probe.
    """

    def poll(self, timeout_s: float) -> Mapping[str, Any]: ...
    def close(self) -> None: ...


def _settings(workspace: Path) -> dict[str, Any]:
    root = workspace / "environments/mla_short"
    model = yaml.safe_load((root / "robot/robot_model.yaml").read_text())
    controllers = yaml.safe_load((root / "controllers.yaml").read_text())["real"]
    names = model["robot"]["joint_names"]
    if (not isinstance(names, list) or len(names) != 6 or len(set(names)) != 6 or
            any(not isinstance(name, str) or not name for name in names)):
        raise ValueError("mla_short must define six unique named robot joints")
    action = controllers["action_name"]
    suffix = "/follow_joint_trajectory"
    if (controllers.get("controller_type") != "joint_trajectory_controller" or
            not isinstance(action, str) or not action.startswith("/") or
            not action.endswith(suffix) or action.count("/") != 2):
        raise ValueError("mla_short real controller must be an explicit joint trajectory action")
    return {"joint_names": tuple(names), "action_name": action,
            "controller_name": action[1:-len(suffix)],
            "runtime": {"execution_mode": "real", "environments_root": str(workspace / "environments"),
                        "telemetry_root": str(workspace / "runs"), "use_sim_time": False}}


def _joint_evidence(sample: JointFeedback, names: tuple[str, ...], now: float,
                    source_now_ns: int) -> dict[str, Any]:
    if (len(sample.names) != 6 or set(sample.names) != set(names) or
            len(set(sample.names)) != len(sample.names) or
            len(sample.positions) != len(sample.names) or
            len(sample.velocities) != len(sample.names) or
            any(name not in sample.names for name in names)):
        raise ValueError("actual /joint_states lacks unique full six-joint position/velocity feedback")
    positions = tuple(sample.positions[sample.names.index(name)] for name in names)
    velocities = tuple(sample.velocities[sample.names.index(name)] for name in names)
    if not all(math.isfinite(value) for value in (*positions, *velocities)):
        raise ValueError("actual /joint_states has nonfinite position/velocity feedback")
    receipt_age = now - sample.receipt_monotonic_s
    source_age = (source_now_ns - sample.source_stamp_ns) / 1e9
    if (not math.isfinite(receipt_age) or not 0 <= receipt_age <= .5 or
            isinstance(sample.source_stamp_ns, bool) or sample.source_stamp_ns <= 0 or
            not 0 <= source_age <= .5):
        raise ValueError("actual /joint_states source or receipt is stale/future (limit 0.5 s)")
    return {"joint_names": list(names), "positions_rad": list(positions),
            "velocities_rad_s": list(velocities), "source_stamp_ns": sample.source_stamp_ns,
            "receipt_monotonic_s": sample.receipt_monotonic_s,
            "source_age_s": source_age, "receipt_age_s": receipt_age}


def check_readiness(workspace: Path | str, *, timeout_s: float = 10., domain_id: int = 0,
                    backend: ReadinessBackend | None = None,
                    monotonic: Callable[[], float] = time.monotonic,
                    wall_time_ns: Callable[[], int] = time.time_ns) -> dict[str, Any]:
    """Return live readiness evidence in one shared <= 20 s deadline.

    An injected backend is owned and closed by this call.  Passing readiness
    authorizes nothing: full release, collision, velocity and serial gates still
    apply.  A negative response fails immediately; discovery alone never passes.
    """
    if isinstance(timeout_s, bool) or not math.isfinite(timeout_s) or not 0 < timeout_s <= 20:
        raise ValueError("preflight timeout must be finite and in (0, 20] seconds")
    if isinstance(domain_id, bool) or not isinstance(domain_id, int) or domain_id != 0:
        raise ValueError("real collection preflight requires ROS_DOMAIN_ID=0")
    started = monotonic()
    deadline = started + timeout_s
    workspace = Path(workspace).expanduser().resolve()
    settings = _settings(workspace)
    report: dict[str, Any] = {
        "schema_version": 1, "passed": False, "read_only": True,
        "scope": "live_ros_readiness_not_motion_or_sensor_authorization",
        "domain_id": domain_id, "timeout_s": timeout_s, "expected": settings,
        "checks": {}, "errors": [], "no_motion_goals": True, "serial_opened": False,
    }
    last_source = 0
    samples = 0
    latest: JointFeedback | None = None
    try:
        if backend is None:
            backend = RosReadinessBackend(settings, domain_id)
        while monotonic() < deadline:
            snapshot = backend.poll(min(.05, max(0., deadline - monotonic())))
            if monotonic() > deadline:
                raise TimeoutError("ROS preflight exhausted its shared deadline")
            for key in ("runtime_error", "controllers_error"):
                if snapshot.get(key):
                    raise RuntimeError(str(snapshot[key]))
            runtime = snapshot.get("runtime")
            if runtime is not None:
                if (dict(runtime) != settings["runtime"] or
                        runtime.get("use_sim_time") is not False):
                    raise ValueError(f"runtime identity mismatch: {runtime!r}; start the mla_short real workspace stack")
                report["checks"]["runtime"] = dict(runtime)
            controllers = snapshot.get("controllers")
            if controllers is not None:
                selected = [item for item in controllers if item.get("name") == settings["controller_name"]]
                if (len(selected) != 1 or selected[0].get("state") != "active" or
                        selected[0].get("type") != "joint_trajectory_controller/JointTrajectoryController"):
                    raise ValueError(f"environment-bound controller {settings['controller_name']} is not uniquely active; "
                                     "start the matching real driver/controller, not a scaled fallback")
                report["checks"]["controller"] = dict(selected[0])
            if snapshot.get("program_running") is False:
                raise RuntimeError("UR robot program is not running; start External Control on the pendant")
            if snapshot.get("program_running") is not None:
                report["checks"]["program_running"] = snapshot["program_running"]
            if snapshot.get("action_ready") is True:
                report["checks"]["controller_action"] = settings["action_name"]
            else:
                report["checks"].pop("controller_action", None)
            for sample in snapshot.get("joints", ()):
                evidence = _joint_evidence(sample, settings["joint_names"], monotonic(), wall_time_ns())
                if last_source and sample.source_stamp_ns <= last_source:
                    raise ValueError("actual /joint_states source clock did not advance")
                last_source, latest = sample.source_stamp_ns, sample
                samples += 1
                report["checks"]["actual_joint_state"] = {**evidence, "live_samples": samples}
            required = {"runtime", "controller", "controller_action", "actual_joint_state"}
            if required.issubset(report["checks"]) and samples >= 2 and latest is not None:
                # A service response must not turn older feedback into a pass.
                final = _joint_evidence(latest, settings["joint_names"], monotonic(), wall_time_ns())
                report["checks"]["actual_joint_state"] = {**final, "live_samples": samples}
                report["passed"] = True
                break
        if not report["passed"]:
            missing = []
            for name in ("runtime", "controller", "controller_action", "actual_joint_state"):
                if name not in report["checks"]:
                    missing.append(name)
            if samples < 2:
                missing.append("two advancing actual JointState samples")
            raise TimeoutError("ROS preflight deadline: missing " + ", ".join(missing) +
                               "; check mla_short stack and DDS loopback UDP environment; NUCLEO was not opened")
    except Exception as exc:
        report["passed"] = False
        report["errors"].append(f"{type(exc).__name__}: {exc}")
    finally:
        if backend is not None:
            try:
                backend.close()
            except Exception as exc:
                report["passed"] = False
                report["errors"].append(f"preflight cleanup failed: {type(exc).__name__}: {exc}")
        report["elapsed_s"] = monotonic() - started
        if report["passed"] and report["elapsed_s"] > timeout_s:
            report["passed"] = False
            report["errors"].append("preflight cleanup exceeded the shared deadline")
    return report


def require_readiness(workspace: Path | str, **kwargs: Any) -> dict[str, Any]:
    report = check_readiness(workspace, **kwargs)
    if not report["passed"]:
        raise ReadinessError(report)
    return report


class RosReadinessBackend:
    """Standalone live client: no planner, serial, restart or goal submission."""

    def __init__(self, settings: Mapping[str, Any], domain_id: int):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from control_msgs.action import FollowJointTrajectory
        from controller_manager_msgs.srv import ListControllers
        from rcl_interfaces.srv import GetParameters
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool

        self.rclpy, self.context = rclpy, Context()
        self.node = self.executor = self.action = None
        try:
            rclpy.init(context=self.context, domain_id=domain_id)
            self.node = rclpy.create_node(f"wm_readonly_preflight_{os.getpid()}_{uuid4().hex[:8]}",
                                         context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.joints: list[JointFeedback] = []
            self.snapshot: dict[str, Any] = {"runtime": None, "controllers": None,
                                             "program_running": None}
            self.GetParameters, self.ListControllers = GetParameters, ListControllers
            self.parameter_client = self.node.create_client(GetParameters, "/hx_waypoint_runtime/get_parameters")
            self.controller_client = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
            self.action = ActionClient(self.node, FollowJointTrajectory, settings["action_name"])
            self.runtime_future = self.controller_future = None
            self.subscription = self.node.create_subscription(JointState, "/joint_states", self._joint,
                                                              qos_profile_sensor_data)
            self.program_subscription = self.node.create_subscription(
                Bool, "/io_and_status_controller/robot_program_running", self._program, 1)
        except BaseException:
            self.close()
            raise

    def _joint(self, message: Any) -> None:
        self.joints.append(JointFeedback(tuple(message.name), tuple(message.position),
                          tuple(message.velocity), message.header.stamp.sec * 1_000_000_000 +
                          message.header.stamp.nanosec, time.monotonic()))

    def _program(self, message: Any) -> None:
        self.snapshot["program_running"] = bool(message.data)

    def poll(self, timeout_s: float) -> Mapping[str, Any]:
        if self.runtime_future is None and self.parameter_client.service_is_ready():
            request = self.GetParameters.Request()
            request.names = ["execution_mode", "environments_root", "telemetry_root", "use_sim_time"]
            self.runtime_future = self.parameter_client.call_async(request)
        if self.controller_future is None and self.controller_client.service_is_ready():
            self.controller_future = self.controller_client.call_async(self.ListControllers.Request())
        self.executor.spin_once(timeout_sec=timeout_s)
        if self.runtime_future is not None and self.runtime_future.done() and self.snapshot["runtime"] is None:
            try:
                response = self.runtime_future.result()
                # ParameterType.STRING=4 and BOOL=1: unset/default fields cannot pass.
                if response is None or len(response.values) != 4 or [p.type for p in response.values] != [4, 4, 4, 1]:
                    raise ValueError("runtime parameter reply has missing/wrongly typed values")
                self.snapshot["runtime"] = {
                    **dict(zip(("execution_mode", "environments_root", "telemetry_root"),
                               (p.string_value for p in response.values[:3]), strict=True)),
                    "use_sim_time": response.values[3].bool_value}
            except Exception as exc:
                self.snapshot["runtime_error"] = f"runtime parameter service failed: {exc}"
        if self.controller_future is not None and self.controller_future.done() and self.snapshot["controllers"] is None:
            try:
                response = self.controller_future.result()
                if response is None:
                    raise RuntimeError("ListControllers returned no response")
                self.snapshot["controllers"] = [{"name": c.name, "state": c.state, "type": c.type}
                                                  for c in response.controller]
            except Exception as exc:
                self.snapshot["controllers_error"] = f"controller manager service failed: {exc}"
        joints, self.joints = self.joints, []
        return {**self.snapshot, "action_ready": self.action.server_is_ready(), "joints": joints}

    def close(self) -> None:
        try:
            if self.action is not None:
                self.action.destroy()
        finally:
            try:
                if self.executor is not None:
                    self.executor.shutdown(timeout_sec=0.)
            finally:
                try:
                    if self.node is not None:
                        self.node.destroy_node()
                finally:
                    if self.context.ok():
                        self.rclpy.shutdown(context=self.context)
