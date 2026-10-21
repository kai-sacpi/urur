"""Explicit, environment-bound prepare motions; no ROS or hardware at import.

Prepare is outside every authoritative drawing CSV. Neither the source CSV nor
the returned q/qd/qdd/t is retimed, clipped, skipped, or otherwise repaired.
"""
from __future__ import annotations

from contextlib import AbstractContextManager
from dataclasses import dataclass
from datetime import datetime, timezone
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time
from typing import Any, Mapping, Protocol, Sequence
from uuid import uuid4

import yaml


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def write_json_new(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def write_joint_csv_exact(path: Path, trajectory: Any) -> None:
    """Round-trip binary64 q/qd/qdd exactly; only time is canonical integer ns."""
    from hx_waypoint_curobo.trajectory_io import joint_trajectory_columns
    with path.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=joint_trajectory_columns(trajectory.joint_names))
        writer.writeheader()
        for sample in trajectory.samples:
            row = {"time_s": f"{sample.time_s:.9f}", "phase": sample.phase}
            for suffix, values in (("pos",sample.positions),("vel",sample.velocities),("acc",sample.accelerations)):
                row.update({f"{name}_{suffix}":repr(value) for name,value in zip(trajectory.joint_names,values,strict=True)})
            writer.writerow(row)


@dataclass(frozen=True)
class JointStateSnapshot:
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    sync_time_ns: int
    ros_stamp_ns: int
    sequence: int


@dataclass(frozen=True)
class PlannerReply:
    trajectory: Any
    environment_hashes: Mapping[str, str]
    curobo_revision: str
    collision_checked: bool


class CancellationUnconfirmed(RuntimeError):
    """A controller goal may still be active; never assume the robot stopped."""


class MotionBackend(Protocol):
    def runtime_identity(self, timeout_s: float) -> Mapping[str, Any]: ...
    def fresh_state(self, timeout_s: float) -> JointStateSnapshot: ...
    def plan(self, environment_id: str, joint_names: Sequence[str], start: Sequence[float],
             target: Sequence[float], timeout_s: float) -> PlannerReply: ...
    def execute(self, trajectory: Any, timeout_s: float) -> Mapping[str, Any]: ...
    def close(self) -> None: ...


def validate_trajectory(environment: Any, trajectory: Any, start: Sequence[float],
                        target: Sequence[float], endpoint_tolerance: float = .001,
                        max_duration_s: float = 120.) -> dict[str, Any]:
    """Verify the controller's actual quintic segments, not finite differences."""
    from hx_waypoint_curobo.robot_kinematics import QuinticHermiteSegment

    if trajectory.joint_names != environment.joint_names:
        raise ValueError("prepare joint order differs from frozen environment")
    samples = trajectory.samples
    if len(samples) < 2 or samples[0].time_s != 0:
        raise ValueError("prepare requires at least two points starting at zero")
    duration = samples[-1].time_s
    if not math.isfinite(duration) or not 0 < duration <= max_duration_s:
        raise ValueError("prepare duration is invalid or exceeds fixed 120s limit")
    for sample in samples:
        if (not math.isfinite(sample.time_s) or
                abs(sample.time_s - round(sample.time_s * 1e9) / 1e9) > 1e-15):
            raise ValueError("prepare time is not on ROS nanosecond grid")
        values = (*sample.positions, *sample.velocities, *sample.accelerations)
        if len(values) != 18 or not all(math.isfinite(x) for x in values):
            raise ValueError("prepare has missing or nonfinite q/qd/qdd")
    if any(b.time_s <= a.time_s for a, b in zip(samples, samples[1:])):
        raise ValueError("prepare timestamps do not strictly increase")
    errors = [max(abs(a-b) for a, b in zip(samples[0].positions, start, strict=True)),
              max(abs(a-b) for a, b in zip(samples[-1].positions, target, strict=True))]
    if max(errors) > endpoint_tolerance:
        raise ValueError("prepare start/target differs from actual/requested joints")
    for sample in (samples[0], samples[-1]):
        if max(abs(x) for x in (*sample.velocities, *sample.accelerations)) > 1e-6:
            raise ValueError("prepare endpoints must have zero velocity and acceleration")
    if not all(environment.canonical_joint_branch_mask([p.positions for p in samples])):
        raise ValueError("prepare leaves canonical joint branch")
    limits = environment.robot_model["robot"]["limits"]
    maxima = [0., 0., 0.]
    position_bounds = environment.joint_position_domains
    branch = environment.planner["ik"].get("canonical_joint_ranges_rad", {})
    for a, b in zip(samples, samples[1:]):
        for j, name in enumerate(environment.joint_names):
            segment = QuinticHermiteSegment.from_boundary_conditions(
                duration_s=b.time_s-a.time_s, start_position=a.positions[j],
                start_velocity=a.velocities[j], start_acceleration=a.accelerations[j],
                end_position=b.positions[j], end_velocity=b.velocities[j],
                end_acceleration=b.accelerations[j])
            lower, upper = position_bounds[j].lower, position_bounds[j].upper
            if name in branch:
                lower, upper = max(lower, branch[name][0]), min(upper, branch[name][1])
            if any(not lower-1e-9 <= x.value <= upper+1e-9
                   for x in segment.candidate_extrema(0)):
                raise ValueError("prepare interpolation leaves joint/branch limits")
            for order, key in enumerate(("velocity_rad_s", "acceleration_rad_s2", "jerk_rad_s3"), 1):
                maximum = segment.maximum_absolute(order).absolute_value
                maxima[order-1] = max(maxima[order-1], maximum / limits[key][j])
    if max(maxima) > 1.+1e-7:
        raise ValueError(f"prepare violates joint dynamics; no retiming allowed: {maxima}")
    return {"passed": True, "duration_s": duration, "point_count": len(samples),
            "start_error_rad": errors[0], "target_error_rad": errors[1],
            "uniform_time_scale": 1., "continuous_quintic_extrema_checked": True,
            "max_joint_velocity_ratio": maxima[0], "max_joint_acceleration_ratio": maxima[1],
            "max_joint_jerk_ratio": maxima[2], "limits": limits}


class MotionController:
    """Fresh actual state -> environment-bound GPU planner -> exact FJT -> settle."""

    def __init__(self, workspace: Path | str, config_path: Path | str | None = None, *,
                 mode: str = "simulation", real_release: bool = False,
                 domain_id: int | None = None, backend: MotionBackend | None = None,
                 evidence_root: Path | str | None = None,
                 prepared_parking: Path | str | None = None):
        from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
        from hx_waypoint_curobo.robot_kinematics import Pose, RobotKinematics

        self.workspace = Path(workspace).expanduser().resolve()
        self.config_path = Path(config_path or self.workspace / "collection/config.yaml").resolve()
        self.config = yaml.safe_load(self.config_path.read_text())
        if self.config.get("environment_id") != "mla_short":
            raise ValueError("motion requires mla_short")
        self.mode = mode
        if mode not in {"simulation", "real"}:
            raise ValueError("mode must explicitly be simulation or real")
        if mode == "real" and (not real_release or
                self.config.get("execution", {}).get("real_execution_authorized") is not True or
                self.config.get("execution", {}).get("simulation_only") is not False):
            raise PermissionError("real motion requires an explicit accepted installation release")
        self.domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0")) if domain_id is None else domain_id
        if (not isinstance(self.domain_id, int) or not 0 <= self.domain_id <= 232 or
                self.domain_id in {219, 220} or (mode == "simulation" and self.domain_id == 0)):
            raise ValueError("collection simulation must use an isolated nonzero domain, not 219/220")
        self.environment = EnvironmentBuilder().load(self.workspace / "environments/mla_short")
        self.environment_hashes = dict(self.environment.hashes)
        self.lock_sha256 = sha256(self.workspace / "curobo.lock")
        self.config_sha256 = sha256(self.config_path)
        self.revision = str(yaml.safe_load((self.workspace / "curobo.lock").read_text())["commit"])
        parking = self.config["parking"]
        if (parking.get("operator_confirmed") is not True or
                tuple(parking["joint_names"]) != self.environment.joint_names or
                len(parking["joint_positions_deg"]) != 6 or
                not all(math.isfinite(float(x)) for x in parking["joint_positions_deg"])):
            raise ValueError("parking requires six operator-confirmed joints in exact environment order")
        self.park = tuple(math.radians(float(x)) for x in parking["joint_positions_deg"])
        self.home = self.environment.home_joint_positions
        # The bundle already validates the whole frozen environment. Also name
        # the installation-bound rod and clearance explicitly in this adapter.
        if (abs(float(self.environment.tcp["rod"]["length_m"])-.190) > 1e-12 or
                abs(float(self.environment.tcp["rod"]["radius_m"])-.0025) > 1e-12 or
                abs(float(self.environment.planner["validation"]["minimum_collision_distance_m"])-.005) > 1e-12):
            raise ValueError("parking requires unchanged rod190/d5 and hard 5mm clearance")
        self.options = {"planning_timeout_s": 120., "settle_timeout_s": 10.,
                        "max_state_age_s": .5, "settle_velocity_rad_s": .01,
                        "endpoint_tolerance_rad": .001}
        self.options.update(self.config.get("motion", {}))
        if any(isinstance(v, bool) or not math.isfinite(float(v)) or float(v) <= 0
               for v in self.options.values()):
            raise ValueError("all motion limits must be finite positive numbers")
        self.backend = backend
        self._last_state_source_ns = 0
        self._last_state_sequence = 0
        self.evidence_root = Path(evidence_root or self.workspace / "collection/motion_runs")
        self.last_move: dict[str, Any] | None = None
        tool = self.environment.setup_config.tool
        transform = tool.parent_to_tip
        self.tool_pose = Pose(transform.translation_m, transform.quaternion_xyzw)
        self.model = RobotKinematics.from_urdf_bytes(
            (self.environment.root / self.environment.asset_provenance["generated_urdf"]["path"]).read_bytes(),
            base_link=self.environment.base_frame, joint_names=self.environment.joint_names)
        self.prepared_parking = None
        if prepared_parking is not None:
            from .prepared_parking import PreparedParkingBundle
            self.prepared_parking = PreparedParkingBundle(self.workspace,prepared_parking,self.config_path)

    def _unchanged(self) -> None:
        from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
        if (dict(EnvironmentBuilder().load(self.environment.root).hashes) != self.environment_hashes or
                sha256(self.workspace / "curobo.lock") != self.lock_sha256 or
                sha256(self.config_path) != self.config_sha256):
            raise ValueError("environment, curobo lock, or parking configuration changed")
        if self.prepared_parking is not None:
            self.prepared_parking.verify()

    def _backend(self) -> MotionBackend:
        if self.backend is None:
            self.backend = RosMotionBackend(self.environment, self.mode, self.domain_id)
        return self.backend

    def verify_runtime_identity(self) -> dict[str, Any]:
        self._unchanged()
        identity = dict(self._backend().runtime_identity(self.options["planning_timeout_s"]))
        expected = {"execution_mode": self.mode,
                    "environments_root": str(self.workspace / "environments"),
                    "telemetry_root": str(self.workspace / "runs")}
        if any(identity.get(k) != v for k, v in expected.items()) or identity.get("use_sim_time") is not False:
            raise ValueError(f"runtime identity mismatch: {identity!r} != {expected!r}")
        return {**identity, "domain_id": self.domain_id, "verified": True}

    def _state(self, timeout: float | None = None, target: Sequence[float] | None = None) -> JointStateSnapshot:
        state = self._backend().fresh_state(timeout or self.options["max_state_age_s"])
        names = state.joint_names
        if len(names) != 6 or len(set(names)) != 6 or set(names) != set(self.environment.joint_names):
            raise ValueError("actual JointState names do not match environment")
        if len(state.positions) != 6 or len(state.velocities) != 6:
            raise ValueError("actual JointState lacks positions or velocities")
        if any(not math.isfinite(x) for x in (*state.positions, *state.velocities)):
            raise ValueError("actual JointState contains nonfinite q/qd")
        age = (time.monotonic_ns()-state.sync_time_ns)*1e-9
        if not 0 <= age <= self.options["max_state_age_s"]:
            raise ValueError("actual JointState is stale or from a different monotonic clock")
        # Runtime identity requires system ROS time (use_sim_time=false).
        # Receipt time alone cannot make an old message in a DDS queue fresh.
        source_age = (time.time_ns()-state.ros_stamp_ns)*1e-9
        if state.ros_stamp_ns <= 0 or not 0 <= source_age <= self.options["max_state_age_s"]:
            raise ValueError("actual JointState source header is stale, zero, or future-dated")
        if state.ros_stamp_ns <= self._last_state_source_ns or state.sequence <= self._last_state_sequence:
            raise ValueError("actual JointState source header/sequence did not advance")
        self._last_state_source_ns, self._last_state_sequence = state.ros_stamp_ns, state.sequence
        order = [names.index(name) for name in self.environment.joint_names]
        state = JointStateSnapshot(self.environment.joint_names,
                                   tuple(state.positions[i] for i in order),
                                   tuple(state.velocities[i] for i in order),
                                   state.sync_time_ns, state.ros_stamp_ns, state.sequence)
        if max(abs(x) for x in state.velocities) > self.options["settle_velocity_rad_s"]:
            raise ValueError("actual robot is not stationary")
        if target is not None and max(abs(a-b) for a,b in zip(state.positions,target,strict=True)) > self.options["endpoint_tolerance_rad"]:
            raise ValueError("actual joints left requested stationary pose")
        return state

    def tip_fk(self, q: Sequence[float]) -> dict[str, Any]:
        import numpy as np
        from hx_waypoint_curobo.math3d import pose_matrix, matrix_pose
        tool = self.environment.setup_config.tool
        pose = self.model.forward_kinematics(q, (tool.tip_frame,), tool_to_tip=self.tool_pose,
                                           tool_frame=tool.parent_link, tip_frame=tool.tip_frame)[tool.tip_frame]
        local = np.linalg.inv(self.environment._frame_to_base("plane1")) @ pose_matrix(pose.position_m, pose.quaternion_xyzw)
        p, quat = matrix_pose(local)
        return {"base_link_position_m": list(pose.position_m), "base_link_quaternion_xyzw": list(pose.quaternion_xyzw),
                "plane1_position_m": list(p), "plane1_quaternion_xyzw": list(quat)}

    def move_home(self, *, from_trajectory_end: bool = False) -> dict[str, Any]:
        return self._move("home", self.home, from_trajectory_end=from_trajectory_end)

    def move_park(self) -> dict[str, Any]:
        return self._move("park", self.park)

    def _move(self, name: str, target: Sequence[float], *, from_trajectory_end: bool = False) -> dict[str, Any]:
        from .geometry import screen_joint_positions
        directory = self.evidence_root / f"{time.time_ns()}_{name}_{uuid4().hex[:8]}"
        directory.mkdir(parents=True, exist_ok=False)
        evidence: dict[str, Any] = {"passed": False, "stage": name, "execution_mode": self.mode,
            "started_utc": utc_now(), "target_positions_rad": list(target),
            "environment_id": "mla_short", "environment_hashes": self.environment_hashes,
            "curobo_lock_sha256": self.lock_sha256, "config_sha256": self.config_sha256,
            "source_sha256": sha256(Path(__file__)), "automatic_return_on_failure": False}
        try:
            evidence["runtime_identity"] = self.verify_runtime_identity()
            actual = self._state()
            evidence["start_positions_rad"] = list(actual.positions)
            evidence["start_state_sync_time_ns"] = actual.sync_time_ns
            prepared_bytes = None
            trajectory = None
            if self.prepared_parking is not None:
                direction = "home_to_park" if name=="park" else "park_to_home"
                candidate,asset,candidate_bytes = self.prepared_parking.load_asset(direction)
                matches = max(abs(a-b) for a,b in zip(actual.positions,candidate.samples[0].positions,strict=True)) <= self.options["endpoint_tolerance_rad"]
                if matches:
                    trajectory,prepared_bytes = candidate,candidate_bytes
                    evidence.update(motion_source="prepared_parking_bundle",planning_performed=False,
                                    prepared_asset=asset)
                    evidence["gpu_full_path_validation"] = {"passed":True,"source":"immutable original offline GPU certificate",
                        "source_validation_sha256":asset["source_validation_sha256"],
                        "curobo_revision":self.revision,"required_world_self_clearance_m":.005}
                elif name!="home" or not from_trajectory_end:
                    raise ValueError(f"prepared {direction} fresh start mismatch; no implicit replanning or input modification")
            if trajectory is None:
                reply = self._backend().plan("mla_short", self.environment.joint_names, actual.positions, target,
                                            self.options["planning_timeout_s"])
                self._unchanged()
                if (dict(reply.environment_hashes) != self.environment_hashes or
                        reply.curobo_revision != self.revision or reply.collision_checked is not True):
                    raise ValueError("planner lacks exact frozen environment/revision/full-path GPU evidence")
                trajectory = reply.trajectory
                evidence.update(motion_source="dynamic_collision_checked_prepare",planning_performed=True,
                                from_trajectory_end=from_trajectory_end)
                evidence["gpu_full_path_validation"] = {"passed": True, "source": "fixed-environment PlanJointPath service",
                    "curobo_revision": reply.curobo_revision, "required_world_self_clearance_m": .005}
            evidence["validation"] = validate_trajectory(self.environment, trajectory, actual.positions,
                target, self.options["endpoint_tolerance_rad"])
            evidence["cpu_sampled_geometry"] = screen_joint_positions(self.environment, [s.positions for s in trajectory.samples])
            if evidence["cpu_sampled_geometry"]["status"] == "rejected":
                raise ValueError("CPU emitted-point geometry rejected returned GPU path")
            path = directory / "joint_trajectory.csv"
            if prepared_bytes is None:
                write_joint_csv_exact(path, trajectory)
            else:
                with path.open("xb") as stream:
                    stream.write(prepared_bytes)
            evidence["joint_trajectory_path"] = str(path)
            evidence["joint_trajectory_sha256"] = sha256(path)
            # No state may change while planning; never submit a stale start.
            self._state(target=trajectory.samples[0].positions)
            evidence["controller"] = dict(self._backend().execute(trajectory, trajectory.duration_s+25.))
            if evidence["controller"].get("success") is not True:
                raise RuntimeError("controller did not report successful exact-trajectory execution")
            deadline = time.monotonic()+self.options["settle_timeout_s"]
            while True:
                try:
                    settled = self._state(target=target)
                    break
                except ValueError:
                    if time.monotonic() >= deadline:
                        raise
            self._unchanged()
            evidence.update(passed=True, finished_utc=utc_now(), settled_sync_time_ns=settled.sync_time_ns,
                            settled_positions_rad=list(settled.positions), tip_fk=self.tip_fk(settled.positions))
            self.last_move = evidence
            return evidence
        except BaseException as error:
            evidence.update(finished_utc=utc_now(), error=f"{type(error).__name__}: {error}")
            raise
        finally:
            write_json_new(directory / "evidence.json", evidence)

    def stationary_monitor(self, output_path: Path | str, duration_s: float = 30.) -> StationaryMonitor:
        return StationaryMonitor(self, Path(output_path), duration_s)

    def close(self) -> None:
        if self.backend is not None:
            self.backend.close()


class StationaryMonitor(AbstractContextManager):
    """Continuously verify actual parked joints/FK while the sensor reads."""

    def __init__(self, motion: MotionController, path: Path, duration_s: float):
        if not math.isfinite(duration_s) or duration_s <= 0:
            raise ValueError("stationary monitoring needs a positive duration")
        self.motion, self.path, self.duration_s = motion, path, duration_s
        self.stop = threading.Event()
        self.error: BaseException | None = None
        self.evidence: dict[str, Any] = {"passed": False}
        self.rows = 0
        self.previous_ns = 0
        self.max_joint_error = self.max_velocity = self.max_tip_drift = 0.

    def __enter__(self) -> StationaryMonitor:
        self.motion.verify_runtime_identity()
        initial = self.motion._state(target=self.motion.park)
        self.reference_tip = self.motion.tip_fk(self.motion.park)["plane1_position_m"]
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open("x", newline="")
        names = self.motion.environment.joint_names
        self.writer = csv.DictWriter(self.stream, fieldnames=["sync_time_ns", "ros_stamp_ns", "sequence",
            *[f"{j}_pos" for j in names], *[f"{j}_vel" for j in names],
            "tip_x_m", "tip_y_m", "tip_z_m", "tip_qx", "tip_qy", "tip_qz", "tip_qw"])
        self.writer.writeheader()
        self.started_ns = initial.sync_time_ns
        try:
            self._record(initial)
        except BaseException:
            self.stream.close()
            raise
        self.thread = threading.Thread(target=self._loop, name="park-actual-fk-monitor", daemon=True)
        self.thread.start()
        return self

    def _record(self, state: JointStateSnapshot) -> None:
        gap = (state.sync_time_ns-self.previous_ns)*1e-9 if self.previous_ns else 0.
        if self.previous_ns and (gap <= 0 or gap > self.motion.options["max_state_age_s"]):
            raise ValueError("parked actual-state samples have a gap or regressing clock")
        fk = self.motion.tip_fk(state.positions)
        self.max_joint_error = max(self.max_joint_error, max(abs(a-b) for a,b in zip(state.positions,self.motion.park)))
        self.max_velocity = max(self.max_velocity, max(abs(v) for v in state.velocities))
        self.max_tip_drift = max(self.max_tip_drift, math.dist(fk["plane1_position_m"], self.reference_tip))
        if self.max_tip_drift > .001:
            raise ValueError("parked rod-tip FK drift exceeds 1mm")
        row = {"sync_time_ns":state.sync_time_ns, "ros_stamp_ns":state.ros_stamp_ns, "sequence":state.sequence}
        row.update({f"{j}_pos":q for j,q in zip(state.joint_names,state.positions)})
        row.update({f"{j}_vel":v for j,v in zip(state.joint_names,state.velocities)})
        row.update(dict(zip(("tip_x_m","tip_y_m","tip_z_m"),fk["plane1_position_m"])))
        row.update(dict(zip(("tip_qx","tip_qy","tip_qz","tip_qw"),fk["plane1_quaternion_xyzw"])))
        self.writer.writerow(row)
        self.stream.flush()
        self.previous_ns = state.sync_time_ns
        self.rows += 1

    def _loop(self) -> None:
        try:
            while not self.stop.is_set():
                self._record(self.motion._state(target=self.motion.park))
        except BaseException as error:
            self.error = error

    def check(self) -> None:
        if self.error is not None:
            raise RuntimeError(f"stationary background invalid: {self.error}") from self.error
        if self.previous_ns and (time.monotonic_ns()-self.previous_ns)*1e-9 > self.motion.options["max_state_age_s"]:
            raise RuntimeError("stationary background invalid: actual JointState became stale")

    def __exit__(self, exception_type, exception, traceback):
        self.stop.set()
        self.thread.join(self.motion.options["max_state_age_s"]+2.)
        if self.thread.is_alive():
            self.error = RuntimeError("stationary monitor did not stop")
        self.stream.close()
        span = (self.previous_ns-self.started_ns)*1e-9
        wall_span = (time.monotonic_ns()-self.started_ns)*1e-9
        if (self.rows < 2 or wall_span < self.duration_s or
                span < self.duration_s-self.motion.options["max_state_age_s"]):
            self.error = self.error or ValueError("actual-state evidence does not cover requested background window")
        try:
            self.motion._unchanged()
        except Exception as error:
            self.error = self.error or error
        self.evidence = {"passed": exception is None and self.error is None,
            "execution_mode": self.motion.mode, "path": str(self.path), "sha256": sha256(self.path),
            "requested_duration_s": self.duration_s, "observed_span_s": span, "sample_count": self.rows,
            "host_window_s": wall_span,
            "started_sync_time_ns": self.started_ns, "stopped_sync_time_ns": self.previous_ns,
            "sync_clock": "host CLOCK_MONOTONIC nanoseconds; same as NUCLEO sensor_rx_sync_time_ns",
            "max_joint_position_error_rad": self.max_joint_error, "max_joint_velocity_rad_s": self.max_velocity,
            "max_tip_fk_drift_m": self.max_tip_drift, "tip_frame": "plane1",
            "error": str(exception or self.error) if exception or self.error else None}
        write_json_new(self.path.with_suffix(self.path.suffix+".json"), self.evidence)
        if exception is None:
            self.check()
        return False


class RosMotionBackend:
    """Connect to an existing fixed runtime; never launches drivers or nodes."""

    def __init__(self, environment: Any, mode: str, domain_id: int):
        import rclpy
        from rclpy.context import Context
        from rclpy.action import ActionClient
        from rclpy.executors import SingleThreadedExecutor
        from rcl_interfaces.srv import GetParameters
        from hx_waypoint_interfaces.srv import PlanJointPath
        from control_msgs.action import FollowJointTrajectory
        from sensor_msgs.msg import JointState
        self.rclpy = rclpy
        self.context = Context()
        rclpy.init(context=self.context, domain_id=domain_id)
        self.node = rclpy.create_node(f"wm_collection_motion_{os.getpid()}", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.environment = environment
        self.latest = None
        self.sequence = 0
        self.subscription = self.node.create_subscription(JointState, "/joint_states", self._joint, 1)
        self.parameters = self.node.create_client(GetParameters, "/hx_waypoint_runtime/get_parameters")
        self.planner = self.node.create_client(PlanJointPath, "/hx_waypoint/curobo/plan_joint_path")
        self.controller_name = environment.controllers[mode]["action_name"]
        self.controller = ActionClient(self.node, FollowJointTrajectory, self.controller_name)
        self.GetParameters, self.PlanJointPath, self.FollowJointTrajectory = GetParameters, PlanJointPath, FollowJointTrajectory

    def _joint(self, message) -> None:
        self.sequence += 1
        self.latest = JointStateSnapshot(tuple(message.name), tuple(message.position), tuple(message.velocity),
            time.monotonic_ns(), message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec, self.sequence)

    def _wait(self, future, timeout_s: float):
        deadline = time.monotonic()+timeout_s
        while not future.done() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=min(.05, max(0., deadline-time.monotonic())))
        if not future.done():
            raise TimeoutError("ROS motion operation timed out")
        response = future.result()
        if response is None:
            raise RuntimeError("ROS motion operation returned no response")
        return response

    def runtime_identity(self, timeout_s: float) -> Mapping[str, Any]:
        if not self.parameters.wait_for_service(timeout_sec=min(timeout_s,20.)):
            raise RuntimeError("fixed hx_waypoint runtime parameter service unavailable")
        request = self.GetParameters.Request()
        request.names = ["execution_mode", "environments_root", "telemetry_root", "use_sim_time"]
        response = self._wait(self.parameters.call_async(request), timeout_s)
        if len(response.values) != 4:
            raise ValueError("runtime identity returned wrong parameter count")
        return {**dict(zip(request.names[:3],(p.string_value for p in response.values[:3]),strict=True)),
                "use_sim_time":response.values[3].bool_value}

    def fresh_state(self, timeout_s: float) -> JointStateSnapshot:
        baseline, deadline = self.sequence, time.monotonic()+timeout_s
        while self.sequence <= baseline and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=min(.02, max(0., deadline-time.monotonic())))
        if self.sequence <= baseline or self.latest is None:
            raise TimeoutError("no new actual JointState before deadline")
        return self.latest

    def plan(self, environment_id, joint_names, start, target, timeout_s) -> PlannerReply:
        from hx_waypoint_curobo.models import CompiledTrajectory, JointSample
        if not self.planner.wait_for_service(timeout_sec=min(20.,timeout_s)):
            raise RuntimeError("cuRobo fixed-environment planner unavailable")
        request = self.PlanJointPath.Request()
        request.environment_id, request.joint_names = environment_id, list(joint_names)
        request.current_positions, request.target_positions = list(start), list(target)
        response = self._wait(self.planner.call_async(request),timeout_s)
        if not response.success:
            raise RuntimeError(f"collision-aware prepare rejected: {response.message}")
        if (len(response.environment_paths) != len(response.environment_sha256) or
                len(set(response.environment_paths)) != len(response.environment_paths)):
            raise ValueError("planner environment evidence has duplicate/missing paths")
        trajectory = CompiledTrajectory(tuple(response.trajectory.joint_names), tuple(
            JointSample(p.time_from_start.sec+p.time_from_start.nanosec*1e-9,
                        tuple(p.positions),tuple(p.velocities),tuple(p.accelerations),"approach")
            for p in response.trajectory.points))
        return PlannerReply(trajectory, dict(zip(response.environment_paths,response.environment_sha256,strict=True)),
                            response.curobo_revision, True)

    def execute(self, trajectory, timeout_s) -> Mapping[str, Any]:
        from hx_waypoint_curobo.curobo_server import _to_ros_trajectory
        if not self.controller.wait_for_server(timeout_sec=20.):
            raise RuntimeError(f"environment-bound controller unavailable: {self.controller_name}")
        goal = self.FollowJointTrajectory.Goal()
        goal.trajectory = _to_ros_trajectory(trajectory)
        pending = handle = None
        terminal = False
        try:
            pending = self.controller.send_goal_async(goal)
            handle = self._wait(pending,20.)
            if not handle.accepted:
                terminal = True
                raise RuntimeError("controller rejected prepare goal")
            result = self._wait(handle.get_result_async(),timeout_s)
            terminal = result.status in (4,5,6)
            if result.status != 4 or result.result.error_code != 0:
                raise RuntimeError(f"prepare controller failed: {result.status}, {result.result.error_string}")
            return {"success":True, "action_name":self.controller_name, "action_status":result.status,
                    "controller_error_code":result.result.error_code}
        except BaseException as original:
            # Losing the action client, including during goal submission, does
            # not cancel an accepted ROS action. Own the future until terminal.
            if pending is not None and not terminal:
                try:
                    if handle is None:
                        handle = self._wait(pending,5.)
                    if handle.accepted:
                        self._wait(handle.cancel_goal_async(),10.)
                        wrapped = self._wait(handle.get_result_async(),10.)
                        if wrapped.status not in (4,5,6):
                            raise RuntimeError("controller goal has no terminal result after cancel")
                except BaseException as cancellation_error:
                    raise CancellationUnconfirmed(
                        "prepare cancellation_unconfirmed; do not assume robot stopped; "
                        f"original={original}; cleanup={cancellation_error}") from original
            raise

    def close(self) -> None:
        self.executor.shutdown(timeout_sec=2.)
        self.node.destroy_node()
        if self.context.ok():
            self.rclpy.shutdown(context=self.context)
