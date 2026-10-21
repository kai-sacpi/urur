from __future__ import annotations

import csv
import hashlib
import io
import math
import os
import re
import tempfile
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml
from hx_waypoint_curobo.curobo_adapter_v2 import CuroboAdapterV2
from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_curobo.errors import PlanningError
from hx_waypoint_curobo.models import IKCandidate, TimedPose
from hx_waypoint_curobo.prepare_planner import PreparePlanner
from hx_waypoint_curobo.robot_tool_builder import RobotToolBuilder
from hx_waypoint_curobo.trajectory_io import SOURCE_COLUMNS

MLA_GRID_ENVIRONMENT_ID = "mla"
MLA_GRID_FRAME_ID = "plane1"
GRID_COORDINATES_M = (-0.05, 0.0, 0.05)
GRID_HEIGHTS_M = (0.07, 0.08, 0.09, 0.10, 0.11, 0.12)
TILT_LEVELS_DEG = (0.0, 5.0, 10.0, 15.0, 20.0, 25.0)
STATIC_DURATION_S = 3.0
STATIC_TIMES_S = (0.0, 1.0, STATIC_DURATION_S)
STATIC_PHASES = ("draw", "hold", "hold")
EXPECTED_TARGET_COUNT = 54
_SAFE_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]*\Z")


class MlaGridError(RuntimeError):
    """The MLA grid search cannot continue without weakening its contract."""


@dataclass(frozen=True)
class GridTarget:
    order: int
    condition_id: str
    x_A_m: float
    y_A_m: float
    z_A_m: float

    @property
    def tip_position_plane1_m(self) -> tuple[float, float, float]:
        # In the verified MLA setup, plane1 +Z points physically into the board.
        # Positive experimental height therefore maps to negative plane1 Z.
        return (self.x_A_m, self.y_A_m, -self.z_A_m)


@dataclass(frozen=True)
class OrientationCandidate:
    tilt_deg: float
    azimuth_deg: float
    quaternion_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class JointChoice:
    orientation: OrientationCandidate
    candidate: IKCandidate
    orientation_change_deg: float
    joint_change_rad: float


@dataclass(frozen=True)
class PathSummary:
    sample_count: int
    duration_s: float
    minimum_collision_distance_m: float
    max_joint_velocity_ratio: float
    max_joint_acceleration_ratio: float
    max_joint_jerk_ratio: float


@dataclass(frozen=True)
class SelectedTarget:
    target: GridTarget
    orientation: OrientationCandidate
    candidate: IKCandidate
    home_path: PathSummary
    sequence_path: PathSummary


def utc_run_id() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def sha256_bytes(content: bytes) -> str:
    return hashlib.sha256(content).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _coordinate_token(value_m: float) -> str:
    millimetres = int(round(abs(float(value_m)) * 1000.0))
    if abs(value_m) <= 5.0e-13:
        return "z000"
    return f"{'p' if value_m > 0.0 else 'm'}{millimetres:03d}"


def ordered_grid_targets() -> tuple[GridTarget, ...]:
    """Return a layer-wise serpentine order with no substituted grid points."""

    layer_pattern: list[tuple[float, float]] = []
    for row_index, y_value in enumerate(GRID_COORDINATES_M):
        x_values = (
            GRID_COORDINATES_M
            if row_index % 2 == 0
            else tuple(reversed(GRID_COORDINATES_M))
        )
        layer_pattern.extend((x_value, y_value) for x_value in x_values)

    targets: list[GridTarget] = []
    order = 1
    for layer_index, height in enumerate(GRID_HEIGHTS_M):
        points = layer_pattern if layer_index % 2 == 0 else list(reversed(layer_pattern))
        for x_value, y_value in points:
            condition_id = (
                f"mla_grid_z{int(round(height * 1000.0)):03d}_"
                f"x{_coordinate_token(x_value)}_y{_coordinate_token(y_value)}"
            )
            targets.append(
                GridTarget(
                    order=order,
                    condition_id=condition_id,
                    x_A_m=x_value,
                    y_A_m=y_value,
                    z_A_m=height,
                )
            )
            order += 1
    if len(targets) != EXPECTED_TARGET_COUNT:
        raise AssertionError("MLA grid definition did not produce exactly 54 targets")
    return tuple(targets)


def tilt_azimuth_quaternion_xyzw(
    tilt_deg: float,
    azimuth_deg: float,
) -> tuple[float, float, float, float]:
    """Orient tool +Z at ``tilt`` from plane1 +Z toward ``azimuth`` in XY.

    The rotation uses the unique minimum-angle realization (zero free roll).
    Azimuth 0 degrees tilts toward +X; 90 degrees tilts toward +Y.
    """

    tilt = float(tilt_deg)
    azimuth = float(azimuth_deg)
    if not math.isfinite(tilt) or not 0.0 <= tilt <= 180.0:
        raise MlaGridError("tilt_deg must be finite and within [0, 180]")
    if not math.isfinite(azimuth):
        raise MlaGridError("azimuth_deg must be finite")
    if abs(tilt) <= 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    azimuth_rad = math.radians(azimuth)
    half_tilt = 0.5 * math.radians(tilt)
    sine = math.sin(half_tilt)
    quaternion = (
        -math.sin(azimuth_rad) * sine,
        math.cos(azimuth_rad) * sine,
        0.0,
        math.cos(half_tilt),
    )
    norm = math.sqrt(sum(value * value for value in quaternion))
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


def orientation_candidates(
    tilt_deg: float,
    *,
    azimuth_step_deg: int,
) -> tuple[OrientationCandidate, ...]:
    if (
        isinstance(azimuth_step_deg, bool)
        or not isinstance(azimuth_step_deg, int)
        or azimuth_step_deg <= 0
        or 360 % azimuth_step_deg != 0
    ):
        raise MlaGridError("azimuth_step_deg must be a positive integer divisor of 360")
    azimuths = (0.0,) if abs(float(tilt_deg)) <= 1.0e-12 else tuple(
        float(value) for value in range(0, 360, azimuth_step_deg)
    )
    return tuple(
        OrientationCandidate(
            tilt_deg=float(tilt_deg),
            azimuth_deg=azimuth,
            quaternion_xyzw=tilt_azimuth_quaternion_xyzw(tilt_deg, azimuth),
        )
        for azimuth in azimuths
    )


def quaternion_angular_distance_deg(
    left_xyzw: Sequence[float],
    right_xyzw: Sequence[float],
) -> float:
    left = np.asarray(left_xyzw, dtype=float)
    right = np.asarray(right_xyzw, dtype=float)
    if left.shape != (4,) or right.shape != (4,):
        raise MlaGridError("quaternion distance requires two four-element quaternions")
    left /= np.linalg.norm(left)
    right /= np.linalg.norm(right)
    dot = float(np.clip(abs(np.dot(left, right)), 0.0, 1.0))
    return math.degrees(2.0 * math.acos(dot))


def _authority_content(
    target: GridTarget,
    orientation: OrientationCandidate,
) -> bytes:
    position = target.tip_position_plane1_m
    pose = {
        "frame_id": MLA_GRID_FRAME_ID,
        "tip_x_m": f"{position[0]:.9f}",
        "tip_y_m": f"{position[1]:.9f}",
        "tip_z_m": f"{position[2]:.9f}",
        "tip_qx": f"{orientation.quaternion_xyzw[0]:.12f}",
        "tip_qy": f"{orientation.quaternion_xyzw[1]:.12f}",
        "tip_qz": f"{orientation.quaternion_xyzw[2]:.12f}",
        "tip_qw": f"{orientation.quaternion_xyzw[3]:.12f}",
    }
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=SOURCE_COLUMNS, lineterminator="\n")
    writer.writeheader()
    for time_s, phase in zip(STATIC_TIMES_S, STATIC_PHASES, strict=True):
        writer.writerow({"time_s": f"{time_s:.9f}", **pose, "phase": phase})
    return stream.getvalue().encode("utf-8")


def _write_new_or_same(path: Path, content: bytes) -> None:
    if path.exists():
        if path.is_symlink() or not path.is_file() or path.read_bytes() != content:
            raise MlaGridError(f"refusing to overwrite a differing authority: {path}")
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _atomic_csv(
    path: Path,
    fieldnames: Sequence[str],
    rows: Sequence[Mapping[str, object]],
) -> None:
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="") as output:
            output.write(stream.getvalue())
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _pose_for(
    target: GridTarget,
    orientation: OrientationCandidate,
) -> TimedPose:
    return TimedPose(
        time_s=0.0,
        frame_id=MLA_GRID_FRAME_ID,
        position_m=target.tip_position_plane1_m,
        quaternion_xyzw=orientation.quaternion_xyzw,
        phase="draw",
    )


def _choice_sort_key(choice: JointChoice) -> tuple[object, ...]:
    # The tilt level is handled by the outer loop.  The remaining order is the
    # user's requested lexicographic preference; joint change is a deterministic
    # final tie-break after the requested posture-change criterion.
    return (
        -choice.candidate.collision_distance_m,
        -choice.candidate.joint_limit_margin_rad,
        choice.candidate.singularity_score,
        choice.orientation_change_deg,
        choice.joint_change_rad,
        choice.orientation.azimuth_deg,
        choice.candidate.positions,
    )


def _path_summary(
    planner: PreparePlanner,
    adapter: CuroboAdapterV2,
    joint_names: Sequence[str],
    start: Sequence[float],
    goal: Sequence[float],
) -> PathSummary:
    trajectory = planner.plan(joint_names, start, goal)
    validation = adapter.validate_joint_trajectory(trajectory)
    metadata = trajectory.metadata.get("prepare", {})
    if not isinstance(metadata, Mapping):
        raise PlanningError("prepare trajectory metadata is not a mapping")
    return PathSummary(
        sample_count=len(trajectory.samples),
        duration_s=float(trajectory.samples[-1].time_s),
        minimum_collision_distance_m=float(validation.minimum_collision_distance_m),
        max_joint_velocity_ratio=float(metadata.get("max_joint_velocity_ratio", 0.0)),
        max_joint_acceleration_ratio=float(metadata.get("max_joint_acceleration_ratio", 0.0)),
        max_joint_jerk_ratio=float(metadata.get("max_joint_jerk_ratio", 0.0)),
    )


def _format_float(value: float) -> str:
    return f"{float(value):.12g}"


REACHABILITY_FIELDS = (
    "order",
    "condition_id",
    "trajectory_id",
    "environment_id",
    "status",
    "x_A_m",
    "y_A_m",
    "z_A_m",
    "tip_x_plane1_m",
    "tip_y_plane1_m",
    "tip_z_plane1_m",
    "tilt_deg",
    "azimuth_deg",
    "tip_qx",
    "tip_qy",
    "tip_qz",
    "tip_qw",
    "collision_distance_m",
    "joint_limit_margin_rad",
    "singularity_condition_number",
    "orientation_change_from_previous_deg",
    "joint_change_from_previous_rad",
    "search_joint_1_rad",
    "search_joint_2_rad",
    "search_joint_3_rad",
    "search_joint_4_rad",
    "search_joint_5_rad",
    "search_joint_6_rad",
    "home_path_sample_count",
    "home_path_duration_s",
    "home_path_minimum_collision_distance_m",
    "sequence_path_sample_count",
    "sequence_path_duration_s",
    "sequence_path_minimum_collision_distance_m",
    "authority_sha256",
    "failure_reason",
)


ATTEMPT_FIELDS = (
    "target_order",
    "condition_id",
    "x_A_m",
    "y_A_m",
    "z_A_m",
    "tip_z_plane1_m",
    "tilt_deg",
    "azimuth_deg",
    "tip_qx",
    "tip_qy",
    "tip_qz",
    "tip_qw",
    "ik_status",
    "ik_candidate_count",
    "best_collision_distance_m",
    "best_joint_limit_margin_rad",
    "best_singularity_condition_number",
    "path_candidates_tested",
    "path_status",
    "message",
)


CAPTURE_FIELDS = (
    "order",
    "condition_id",
    "trajectory_id",
    "capture_start_s",
    "capture_end_s",
    "capture_duration_s",
    "policy",
)


def _empty_reachability_row(target: GridTarget, failure_reason: str) -> dict[str, object]:
    tip = target.tip_position_plane1_m
    row: dict[str, object] = {field: "" for field in REACHABILITY_FIELDS}
    row.update(
        {
            "order": target.order,
            "condition_id": target.condition_id,
            "environment_id": MLA_GRID_ENVIRONMENT_ID,
            "status": "UNREACHABLE",
            "x_A_m": _format_float(target.x_A_m),
            "y_A_m": _format_float(target.y_A_m),
            "z_A_m": _format_float(target.z_A_m),
            "tip_x_plane1_m": _format_float(tip[0]),
            "tip_y_plane1_m": _format_float(tip[1]),
            "tip_z_plane1_m": _format_float(tip[2]),
            "failure_reason": failure_reason.replace("\n", " "),
        }
    )
    return row


def _reachable_row(
    selected: SelectedTarget,
    previous_orientation: Sequence[float],
    previous_joints: Sequence[float],
    authority_sha256: str,
) -> dict[str, object]:
    target = selected.target
    orientation = selected.orientation
    ik = selected.candidate
    tip = target.tip_position_plane1_m
    row: dict[str, object] = {field: "" for field in REACHABILITY_FIELDS}
    row.update(
        {
            "order": target.order,
            "condition_id": target.condition_id,
            "trajectory_id": target.condition_id,
            "environment_id": MLA_GRID_ENVIRONMENT_ID,
            "status": "REACHABLE",
            "x_A_m": _format_float(target.x_A_m),
            "y_A_m": _format_float(target.y_A_m),
            "z_A_m": _format_float(target.z_A_m),
            "tip_x_plane1_m": _format_float(tip[0]),
            "tip_y_plane1_m": _format_float(tip[1]),
            "tip_z_plane1_m": _format_float(tip[2]),
            "tilt_deg": _format_float(orientation.tilt_deg),
            "azimuth_deg": _format_float(orientation.azimuth_deg),
            "tip_qx": f"{orientation.quaternion_xyzw[0]:.12f}",
            "tip_qy": f"{orientation.quaternion_xyzw[1]:.12f}",
            "tip_qz": f"{orientation.quaternion_xyzw[2]:.12f}",
            "tip_qw": f"{orientation.quaternion_xyzw[3]:.12f}",
            "collision_distance_m": _format_float(ik.collision_distance_m),
            "joint_limit_margin_rad": _format_float(ik.joint_limit_margin_rad),
            "singularity_condition_number": _format_float(ik.singularity_score),
            "orientation_change_from_previous_deg": _format_float(
                quaternion_angular_distance_deg(
                    previous_orientation,
                    orientation.quaternion_xyzw,
                )
            ),
            "joint_change_from_previous_rad": _format_float(
                max(
                    abs(current - previous)
                    for current, previous in zip(
                        ik.positions,
                        previous_joints,
                        strict=True,
                    )
                )
            ),
            "home_path_sample_count": selected.home_path.sample_count,
            "home_path_duration_s": _format_float(selected.home_path.duration_s),
            "home_path_minimum_collision_distance_m": _format_float(
                selected.home_path.minimum_collision_distance_m
            ),
            "sequence_path_sample_count": selected.sequence_path.sample_count,
            "sequence_path_duration_s": _format_float(selected.sequence_path.duration_s),
            "sequence_path_minimum_collision_distance_m": _format_float(
                selected.sequence_path.minimum_collision_distance_m
            ),
            "authority_sha256": authority_sha256,
            "failure_reason": "",
        }
    )
    for index, value in enumerate(ik.positions, start=1):
        row[f"search_joint_{index}_rad"] = _format_float(value)
    return row


def run_search(
    *,
    workspace: Path,
    output_directory: Path,
    azimuth_step_deg: int = 30,
    candidates_per_pose: int = 32,
    adapter: CuroboAdapterV2 | None = None,
) -> tuple[Path, Path, Path]:
    """Search all targets, publish immutable authorities, and write evidence."""

    workspace = workspace.expanduser().resolve()
    output_directory = output_directory.expanduser().resolve()
    environment_directory = workspace / "environments" / MLA_GRID_ENVIRONMENT_ID
    environment = EnvironmentBuilder().load(environment_directory)
    if environment.environment_id != MLA_GRID_ENVIRONMENT_ID:
        raise MlaGridError(
            f"environment_id must be {MLA_GRID_ENVIRONMENT_ID}; got {environment.environment_id}"
        )
    if output_directory.exists() and any(output_directory.iterdir()):
        raise MlaGridError(f"output directory is not empty: {output_directory}")
    output_directory.mkdir(parents=True, exist_ok=True)
    if (
        isinstance(candidates_per_pose, bool)
        or not isinstance(candidates_per_pose, int)
        or candidates_per_pose <= 0
    ):
        raise MlaGridError("candidates_per_pose must be a positive integer")
    # Validate the azimuth contract before allocating GPU state.
    orientation_candidates(5.0, azimuth_step_deg=azimuth_step_deg)

    active_adapter = adapter or CuroboAdapterV2(environment, workspace / "curobo.lock")
    if tuple(active_adapter.joint_names) != environment.joint_names:
        raise MlaGridError("adapter joint order does not match the MLA environment")
    print("warming locked cuRobo runtime", flush=True)
    active_adapter.warmup()
    planner = PreparePlanner(environment, active_adapter)
    tool_builder = RobotToolBuilder(environment)
    home = environment.home_joint_positions
    previous_joints = home
    previous_orientation = (0.0, 0.0, 0.0, 1.0)

    reachability_rows: list[dict[str, object]] = []
    attempt_rows: list[dict[str, object]] = []
    capture_rows: list[dict[str, object]] = []
    trajectory_entries: list[dict[str, object]] = []
    trajectories_directory = environment_directory / "trajectories"

    for target in ordered_grid_targets():
        print(
            f"[{target.order:02d}/{EXPECTED_TARGET_COUNT}] {target.condition_id} "
            f"A=({target.x_A_m:+.3f},{target.y_A_m:+.3f},{target.z_A_m:+.3f})",
            flush=True,
        )
        selected: SelectedTarget | None = None
        last_failure = "no orientation candidate was evaluated"
        previous_for_row = previous_joints
        previous_orientation_for_row = previous_orientation

        for tilt in TILT_LEVELS_DEG:
            choices: list[JointChoice] = []
            tilt_attempt_indices: list[int] = []
            for orientation in orientation_candidates(
                tilt,
                azimuth_step_deg=azimuth_step_deg,
            ):
                quaternion = orientation.quaternion_xyzw
                attempt: dict[str, object] = {
                    "target_order": target.order,
                    "condition_id": target.condition_id,
                    "x_A_m": _format_float(target.x_A_m),
                    "y_A_m": _format_float(target.y_A_m),
                    "z_A_m": _format_float(target.z_A_m),
                    "tip_z_plane1_m": _format_float(-target.z_A_m),
                    "tilt_deg": _format_float(tilt),
                    "azimuth_deg": _format_float(orientation.azimuth_deg),
                    "tip_qx": f"{quaternion[0]:.12f}",
                    "tip_qy": f"{quaternion[1]:.12f}",
                    "tip_qz": f"{quaternion[2]:.12f}",
                    "tip_qw": f"{quaternion[3]:.12f}",
                    "ik_status": "FAILED",
                    "ik_candidate_count": 0,
                    "best_collision_distance_m": "",
                    "best_joint_limit_margin_rad": "",
                    "best_singularity_condition_number": "",
                    "path_candidates_tested": 0,
                    "path_status": "NOT_TESTED",
                    "message": "",
                }
                attempt_rows.append(attempt)
                tilt_attempt_indices.append(len(attempt_rows) - 1)
                try:
                    tip_pose = _pose_for(target, orientation)
                    base_tip = environment.transform_pose_to_base(tip_pose)
                    tool_pose = tool_builder.tip_pose_to_tool_pose(base_tip)
                    candidate_set = active_adapter.solve_ik_candidates(
                        (tool_pose,),
                        candidates_per_pose,
                    )[0]
                except (PlanningError, ValueError) as error:
                    last_failure = str(error)
                    attempt["message"] = last_failure.replace("\n", " ")
                    continue
                if not candidate_set:
                    last_failure = "IK returned no candidates"
                    attempt["message"] = last_failure
                    continue
                attempt["ik_status"] = "FEASIBLE"
                attempt["ik_candidate_count"] = len(candidate_set)
                attempt["best_collision_distance_m"] = _format_float(
                    max(item.collision_distance_m for item in candidate_set)
                )
                attempt["best_joint_limit_margin_rad"] = _format_float(
                    max(item.joint_limit_margin_rad for item in candidate_set)
                )
                attempt["best_singularity_condition_number"] = _format_float(
                    min(item.singularity_score for item in candidate_set)
                )
                orientation_change = quaternion_angular_distance_deg(
                    previous_orientation,
                    quaternion,
                )
                for candidate in candidate_set:
                    joint_change = max(
                        abs(current - previous)
                        for current, previous in zip(
                            candidate.positions,
                            previous_joints,
                            strict=True,
                        )
                    )
                    choices.append(
                        JointChoice(
                            orientation=orientation,
                            candidate=candidate,
                            orientation_change_deg=orientation_change,
                            joint_change_rad=joint_change,
                        )
                    )

            for choice in sorted(choices, key=_choice_sort_key):
                attempt = next(
                    attempt_rows[index]
                    for index in tilt_attempt_indices
                    if float(attempt_rows[index]["azimuth_deg"])
                    == choice.orientation.azimuth_deg
                )
                attempt["path_candidates_tested"] = int(
                    attempt["path_candidates_tested"]
                ) + 1
                try:
                    home_path = _path_summary(
                        planner,
                        active_adapter,
                        environment.joint_names,
                        home,
                        choice.candidate.positions,
                    )
                    if np.allclose(
                        np.asarray(previous_joints),
                        np.asarray(home),
                        atol=1.0e-12,
                        rtol=0.0,
                    ):
                        sequence_path = home_path
                    else:
                        sequence_path = _path_summary(
                            planner,
                            active_adapter,
                            environment.joint_names,
                            previous_joints,
                            choice.candidate.positions,
                        )
                except (PlanningError, ValueError) as error:
                    last_failure = str(error)
                    attempt["path_status"] = "FAILED"
                    attempt["message"] = last_failure.replace("\n", " ")
                    continue
                attempt["path_status"] = "SELECTED"
                attempt["message"] = "home and previous-target paths validated"
                selected = SelectedTarget(
                    target=target,
                    orientation=choice.orientation,
                    candidate=choice.candidate,
                    home_path=home_path,
                    sequence_path=sequence_path,
                )
                break

            if selected is not None:
                # The first successful tilt is authoritative; larger tilts are
                # deliberately not searched because minimum tilt is priority 1.
                break
            for index in tilt_attempt_indices:
                attempt = attempt_rows[index]
                if attempt["ik_status"] == "FEASIBLE" and attempt["path_status"] == "NOT_TESTED":
                    attempt["path_status"] = "NO_PATH_SELECTED"

        if selected is None:
            reachability_rows.append(_empty_reachability_row(target, last_failure))
            print(f"  UNREACHABLE: {last_failure}", flush=True)
        else:
            content = _authority_content(target, selected.orientation)
            authority_path = trajectories_directory / f"{target.condition_id}.csv"
            _write_new_or_same(authority_path, content)
            authority_sha256 = sha256_bytes(content)
            reachability_rows.append(
                _reachable_row(
                    selected,
                    previous_orientation_for_row,
                    previous_for_row,
                    authority_sha256,
                )
            )
            capture_rows.append(
                {
                    "order": target.order,
                    "condition_id": target.condition_id,
                    "trajectory_id": target.condition_id,
                    "capture_start_s": "0.000000000",
                    "capture_end_s": "3.000000000",
                    "capture_duration_s": "3.000000000",
                    "policy": "entire_authoritative_strictly_static_playback",
                }
            )
            trajectory_entries.append(
                {
                    "order": target.order,
                    "condition_id": target.condition_id,
                    "path": str(authority_path.relative_to(workspace)),
                    "sha256": authority_sha256,
                    "duration_s": STATIC_DURATION_S,
                    "strictly_static": True,
                }
            )
            previous_joints = selected.candidate.positions
            previous_orientation = selected.orientation.quaternion_xyzw
            print(
                f"  REACHABLE tilt={selected.orientation.tilt_deg:.0f} "
                f"azimuth={selected.orientation.azimuth_deg:.0f} "
                f"clearance={selected.candidate.collision_distance_m:.6f} m",
                flush=True,
            )

        _atomic_csv(
            output_directory / "reachability.csv",
            REACHABILITY_FIELDS,
            reachability_rows,
        )
        _atomic_csv(
            output_directory / "reachability_attempts.csv",
            ATTEMPT_FIELDS,
            attempt_rows,
        )
        _atomic_csv(
            output_directory / "capture_windows.csv",
            CAPTURE_FIELDS,
            capture_rows,
        )

    reachable_count = sum(row["status"] == "REACHABLE" for row in reachability_rows)
    unreachable_count = EXPECTED_TARGET_COUNT - reachable_count
    manifest = {
        "schema_version": 1,
        "manifest_type": "mla_grid_54_reachability",
        "created_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "environment_id": MLA_GRID_ENVIRONMENT_ID,
        "environment_hashes": dict(sorted(environment.hashes.items())),
        "coordinate_contract": {
            "array_frame": MLA_GRID_FRAME_ID,
            "array_origin": "sensor_array_geometric_center",
            "plane1_positive_z_direction": "physically_into_sensor_board",
            "mapping": "tip_x=x_A, tip_y=y_A, tip_z=-z_A",
        },
        "grid": {
            "x_A_m": list(GRID_COORDINATES_M),
            "y_A_m": list(GRID_COORDINATES_M),
            "z_A_m": list(GRID_HEIGHTS_M),
            "target_count": EXPECTED_TARGET_COUNT,
            "order_policy": "layer_ascending_z_A_serpentine_xy_alternating_layers",
        },
        "orientation_search": {
            "tilt_levels_deg": list(TILT_LEVELS_DEG),
            "azimuth_step_deg": azimuth_step_deg,
            "free_roll_deg": 0.0,
            "ranking": [
                "minimum_tilt",
                "maximum_collision_clearance",
                "maximum_joint_limit_margin",
                "minimum_jacobian_condition_number",
                "minimum_orientation_change_from_previous",
                "minimum_joint_change_final_tiebreak",
            ],
        },
        "path_checks": {
            "configured_home_to_target": True,
            "previous_reachable_target_to_target": True,
            "planner": "locked cuRobo c-space PreparePlanner",
            "runtime_prepare_is_outside_authoritative_recording": True,
        },
        "authority": {
            "duration_s": STATIC_DURATION_S,
            "timestamps_s": list(STATIC_TIMES_S),
            "strictly_static": True,
            "runtime_scaling_retiming_cropping_skipping_allowed": False,
        },
        "counts": {
            "reachable": reachable_count,
            "unreachable": unreachable_count,
            "total": EXPECTED_TARGET_COUNT,
        },
        "files": {
            "reachability": {
                "path": "reachability.csv",
                "sha256": sha256_file(output_directory / "reachability.csv"),
            },
            "attempts": {
                "path": "reachability_attempts.csv",
                "sha256": sha256_file(output_directory / "reachability_attempts.csv"),
            },
            "capture_windows": {
                "path": "capture_windows.csv",
                "sha256": sha256_file(output_directory / "capture_windows.csv"),
            },
        },
        "trajectories": trajectory_entries,
    }
    manifest_path = output_directory / "manifest.yaml"
    _write_new_or_same(
        manifest_path,
        yaml.safe_dump(manifest, sort_keys=False).encode("utf-8"),
    )
    return (
        output_directory / "reachability.csv",
        output_directory / "reachability_attempts.csv",
        manifest_path,
    )

