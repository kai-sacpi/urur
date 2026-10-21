from __future__ import annotations

import csv
import math
import os
import shutil
from bisect import bisect_left
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from io import StringIO
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from hx_waypoint_curobo.models import TimedPose
from hx_waypoint_curobo.robot_kinematics import (
    Pose,
    RobotKinematics,
    compose_poses,
    inverse_pose,
)
from hx_waypoint_curobo.setup_config import load_setup_file
from hx_waypoint_curobo.trajectory_io import SOURCE_COLUMNS, read_timed_pose_csv

from .tracker_capture import TrackerCaptureSummary

YAW_CALIBRATION_SCHEMA_VERSION = 1
DEFAULT_SENSOR_DISTANCE_M = 0.0675
DEFAULT_SWEEP_DEG = 360.0
DEFAULT_LEG_DURATION_S = 18.0
DEFAULT_SAMPLE_PERIOD_S = 0.05
DEFAULT_HOLD_DURATION_S = 2.0
DEFAULT_WARMUP_DURATION_S = 6.0
DEFAULT_WARMUP_AMPLITUDE_M = 0.005
DEFAULT_WARMUP_SETTLE_DURATION_S = 2.0

YAW_CALIBRATION_TRAJECTORY_IDS = (
    "yaw_calibration_center_yaw_180deg",
    "yaw_calibration_center_yaw_360deg",
    "yaw_calibration_x50_yaw_180deg",
    "yaw_calibration_x50_yaw_360deg",
)
YAW_CALIBRATION_TRAJECTORY_FILENAMES = tuple(
    f"{trajectory_id}.csv" for trajectory_id in YAW_CALIBRATION_TRAJECTORY_IDS
)

TRACKER_MAPPED_COLUMNS = (
    "sync_time_ns",
    "trajectory_time_s",
    "robot_row_index",
    "tracker_rx_sync_time_ns",
    "sync_offset_ns",
    "tracker_message_ros_time_ns",
    "tracker_x_plane1_m",
    "tracker_y_plane1_m",
    "tracker_z_plane1_m",
    "rod_tip_x_plane1_m",
    "rod_tip_y_plane1_m",
    "rod_tip_z_plane1_m",
    "actual_yaw_rad",
    "delta_x_m",
    "delta_y_m",
    "delta_z_m",
    "used_for_yaw_fit",
)


@dataclass(frozen=True)
class YawCalibrationTarget:
    frame_id: str
    position_m: tuple[float, float, float]
    sensor_plane_local_z_m: float
    sensor_distance_m: float


@dataclass(frozen=True)
class YawOffsetFit:
    board_bias_xy_m: tuple[float, float]
    tool_offset_xy_m: tuple[float, float]
    combined_bz_plus_oz_m: float
    board_bias_radius_m: float
    tool_offset_radius_m: float
    tool_offset_start_direction_deg: float
    xy_rmse_m: float
    xyz_rmse_m: float
    maximum_residual_m: float
    condition_number: float


@dataclass(frozen=True)
class YawTrackerFinalizeResult:
    status: str
    tracker_telemetry_path: Path
    tracker_raw_path: Path
    report_path: Path
    sample_count: int
    maximum_mapping_offset_ns: int
    fit: YawOffsetFit


@dataclass(frozen=True)
class YawSourceMetadata:
    yaw_start_s: float
    turnaround_s: float
    yaw_end_s: float
    source_end_s: float
    intended_sweep_deg: float
    fixed_tip_position_m: tuple[float, float, float]


def minimum_jerk_progress(unit_time: float) -> float:
    value = float(unit_time)
    if not math.isfinite(value) or value < 0.0 or value > 1.0:
        raise ValueError("minimum-jerk unit_time must be within [0, 1]")
    return 10.0 * value**3 - 15.0 * value**4 + 6.0 * value**5


def _normalized_quaternion(
    quaternion_xyzw: Sequence[float],
) -> tuple[float, float, float, float]:
    values = tuple(float(value) for value in quaternion_xyzw)
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion must contain four finite values")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-15 or abs(norm - 1.0) > 1.0e-3:
        raise ValueError("quaternion must be normalized")
    return tuple(value / norm for value in values)  # type: ignore[return-value]


def _quaternion_product_xyzw(
    left: Sequence[float], right: Sequence[float]
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = _normalized_quaternion(left)
    rx, ry, rz, rw = _normalized_quaternion(right)
    return _normalized_quaternion(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def _seconds_to_nanoseconds(value: float, label: str) -> int:
    seconds = float(value)
    if not math.isfinite(seconds) or seconds <= 0.0:
        raise ValueError(f"{label} must be finite and positive")
    nanoseconds = int(round(seconds * 1_000_000_000.0))
    if not math.isclose(
        nanoseconds / 1_000_000_000.0,
        seconds,
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise ValueError(f"{label} must be exactly representable in ROS nanoseconds")
    return nanoseconds


def build_yaw_sweep_trajectory(
    *,
    position_m: Sequence[float],
    base_quaternion_xyzw: Sequence[float],
    frame_id: str = "plane1",
    sweep_deg: float = DEFAULT_SWEEP_DEG,
    leg_duration_s: float = DEFAULT_LEG_DURATION_S,
    sample_period_s: float = DEFAULT_SAMPLE_PERIOD_S,
    hold_duration_s: float = DEFAULT_HOLD_DURATION_S,
) -> tuple[TimedPose, ...]:
    """Build 0->sweep->0 yaw motion with fixed XYZ and a fixed rod axis."""

    position = tuple(float(value) for value in position_m)
    if len(position) != 3 or not all(math.isfinite(value) for value in position):
        raise ValueError("yaw calibration position_m must contain three finite values")
    if not frame_id or frame_id != frame_id.strip() or any(
        character.isspace() for character in frame_id
    ):
        raise ValueError("yaw calibration frame_id must be a ROS frame")
    sweep = float(sweep_deg)
    if not math.isfinite(sweep) or sweep not in (180.0, 360.0):
        raise ValueError("yaw calibration sweep_deg must be exactly 180 or 360")
    leg_ns = _seconds_to_nanoseconds(leg_duration_s, "leg_duration_s")
    if not 15.0 <= leg_duration_s <= 20.0:
        raise ValueError("yaw calibration leg_duration_s must remain within 15..20 seconds")
    sample_ns = _seconds_to_nanoseconds(sample_period_s, "sample_period_s")
    hold_ns = _seconds_to_nanoseconds(hold_duration_s, "hold_duration_s")
    if leg_ns % sample_ns:
        raise ValueError("sample_period_s must divide leg_duration_s exactly")
    base_quaternion = _normalized_quaternion(base_quaternion_xyzw)
    total_ns = 2 * leg_ns
    sweep_rad = math.radians(sweep)
    poses: list[TimedPose] = []
    for time_ns in range(0, total_ns + sample_ns, sample_ns):
        if time_ns <= leg_ns:
            progress = minimum_jerk_progress(time_ns / leg_ns)
            yaw_rad = sweep_rad * progress
        else:
            progress = minimum_jerk_progress((time_ns - leg_ns) / leg_ns)
            yaw_rad = sweep_rad * (1.0 - progress)
        half = 0.5 * yaw_rad
        yaw_quaternion = (0.0, 0.0, math.sin(half), math.cos(half))
        poses.append(
            TimedPose(
                time_s=time_ns / 1_000_000_000.0,
                frame_id=frame_id,
                position_m=position,  # type: ignore[arg-type]
                quaternion_xyzw=_quaternion_product_xyzw(
                    base_quaternion, yaw_quaternion
                ),
                phase="draw",
            )
        )
    final = poses[-1]
    poses.append(
        TimedPose(
            time_s=(total_ns + hold_ns) / 1_000_000_000.0,
            frame_id=frame_id,
            position_m=final.position_m,
            quaternion_xyzw=final.quaternion_xyzw,
            phase="hold",
        )
    )
    return tuple(poses)


def build_yaw_warmup_wave(
    *,
    position_m: Sequence[float],
    quaternion_xyzw: Sequence[float],
    frame_id: str = "plane1",
    duration_s: float = DEFAULT_WARMUP_DURATION_S,
    amplitude_m: float = DEFAULT_WARMUP_AMPLITUDE_M,
    sample_period_s: float = DEFAULT_SAMPLE_PERIOD_S,
    hold_duration_s: float = 1.0,
) -> tuple[TimedPose, ...]:
    """Build a small, independent warmup authority instead of cropping Wave."""

    center = tuple(float(value) for value in position_m)
    if len(center) != 3 or not all(math.isfinite(value) for value in center):
        raise ValueError("warmup position_m must contain three finite values")
    amplitude = float(amplitude_m)
    if not math.isfinite(amplitude) or amplitude <= 0.0 or amplitude > 0.01:
        raise ValueError("warmup amplitude_m must be within (0, 0.01]")
    duration_ns = _seconds_to_nanoseconds(duration_s, "warmup duration_s")
    sample_ns = _seconds_to_nanoseconds(sample_period_s, "warmup sample_period_s")
    hold_ns = _seconds_to_nanoseconds(hold_duration_s, "warmup hold_duration_s")
    if duration_ns % sample_ns:
        raise ValueError("warmup sample_period_s must divide duration_s exactly")
    quaternion = _normalized_quaternion(quaternion_xyzw)
    poses: list[TimedPose] = []
    for time_ns in range(0, duration_ns + sample_ns, sample_ns):
        progress = minimum_jerk_progress(time_ns / duration_ns)
        angle = 2.0 * math.pi * progress
        poses.append(
            TimedPose(
                time_s=time_ns / 1_000_000_000.0,
                frame_id=frame_id,
                position_m=(
                    center[0] + amplitude * math.sin(angle),
                    center[1] + 0.5 * amplitude * math.sin(2.0 * angle),
                    center[2],
                ),
                quaternion_xyzw=quaternion,
                phase="draw",
            )
        )
    final = poses[-1]
    poses.append(
        TimedPose(
            time_s=(duration_ns + hold_ns) / 1_000_000_000.0,
            frame_id=frame_id,
            position_m=final.position_m,
            quaternion_xyzw=final.quaternion_xyzw,
            phase="hold",
        )
    )
    return tuple(poses)


def build_yaw_calibration_trajectory(
    *,
    position_m: Sequence[float],
    base_quaternion_xyzw: Sequence[float],
    frame_id: str = "plane1",
    sweep_deg: float = DEFAULT_SWEEP_DEG,
    leg_duration_s: float = DEFAULT_LEG_DURATION_S,
    sample_period_s: float = DEFAULT_SAMPLE_PERIOD_S,
    hold_duration_s: float = DEFAULT_HOLD_DURATION_S,
    warmup_duration_s: float = DEFAULT_WARMUP_DURATION_S,
    warmup_amplitude_m: float = DEFAULT_WARMUP_AMPLITUDE_M,
    warmup_settle_duration_s: float = DEFAULT_WARMUP_SETTLE_DURATION_S,
) -> tuple[TimedPose, ...]:
    """Build warmup, center settle, and yaw sweep as one immutable authority.

    Keeping every stage in one source CSV is intentional.  The current
    ``home_then_recording_start`` runtime policy prepares through the configured
    startup posture before every recording.  Splitting warmup and yaw into two
    playbacks would therefore insert an unwanted trip home between them.
    """

    warmup = build_yaw_warmup_wave(
        position_m=position_m,
        quaternion_xyzw=base_quaternion_xyzw,
        frame_id=frame_id,
        duration_s=warmup_duration_s,
        amplitude_m=warmup_amplitude_m,
        sample_period_s=sample_period_s,
        hold_duration_s=warmup_settle_duration_s,
    )
    yaw = build_yaw_sweep_trajectory(
        position_m=position_m,
        base_quaternion_xyzw=base_quaternion_xyzw,
        frame_id=frame_id,
        sweep_deg=sweep_deg,
        leg_duration_s=leg_duration_s,
        sample_period_s=sample_period_s,
        hold_duration_s=hold_duration_s,
    )
    offset_s = warmup[-1].time_s
    combined = list(warmup)
    for pose in yaw[1:]:
        combined.append(
            TimedPose(
                time_s=offset_s + pose.time_s,
                frame_id=pose.frame_id,
                position_m=pose.position_m,
                quaternion_xyzw=pose.quaternion_xyzw,
                phase=pose.phase,
            )
        )
    return tuple(combined)


def resolve_yaw_calibration_target(
    environment_path: str | Path,
    *,
    sensor_distance_m: float = DEFAULT_SENSOR_DISTANCE_M,
    offset_xy_m: Sequence[float] = (0.0, 0.0),
) -> YawCalibrationTarget:
    environment = Path(environment_path).expanduser().resolve()
    try:
        value = yaml.safe_load((environment / "environment.yaml").read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f"cannot read environment sensor geometry: {error}") from error
    calibration = value.get("calibration") if isinstance(value, Mapping) else None
    geometry = (
        calibration.get("sensor_geometry")
        if isinstance(calibration, Mapping)
        else None
    )
    if not isinstance(geometry, Mapping):
        raise ValueError("environment.yaml is missing sensor_geometry")
    sensor_z = float(geometry["actual_sensor_plane_local_z_m"])
    center_xy = tuple(float(item) for item in geometry["board_center_in_plane1_xy_m"])
    offsets = tuple(float(item) for item in offset_xy_m)
    distance = float(sensor_distance_m)
    if (
        len(center_xy) != 2
        or len(offsets) != 2
        or not all(math.isfinite(item) for item in (*center_xy, *offsets, sensor_z, distance))
        or distance <= 0.0
    ):
        raise ValueError("environment sensor geometry and calibration offsets must be finite")
    setup = load_setup_file(environment / "setup.yaml")
    return YawCalibrationTarget(
        frame_id=setup.work_frame.child_frame,
        position_m=(
            center_xy[0] + offsets[0],
            center_xy[1] + offsets[1],
            sensor_z - distance,
        ),
        sensor_plane_local_z_m=sensor_z,
        sensor_distance_m=distance,
    )


def _render_timed_pose_csv(poses: Iterable[TimedPose]) -> bytes:
    stream = StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=SOURCE_COLUMNS, lineterminator="\n")
    writer.writeheader()
    for pose in poses:
        writer.writerow(
            {
                "time_s": f"{pose.time_s:.9f}",
                "frame_id": pose.frame_id,
                "tip_x_m": f"{pose.position_m[0]:.9f}",
                "tip_y_m": f"{pose.position_m[1]:.9f}",
                "tip_z_m": f"{pose.position_m[2]:.9f}",
                "tip_qx": f"{pose.quaternion_xyzw[0]:.12f}",
                "tip_qy": f"{pose.quaternion_xyzw[1]:.12f}",
                "tip_qz": f"{pose.quaternion_xyzw[2]:.12f}",
                "tip_qw": f"{pose.quaternion_xyzw[3]:.12f}",
                "phase": pose.phase,
            }
        )
    return stream.getvalue().encode("utf-8")


def write_authoritative_trajectory(
    path: str | Path, poses: Iterable[TimedPose]
) -> Path:
    destination = Path(path)
    payload = _render_timed_pose_csv(tuple(poses))
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.is_symlink() or (destination.exists() and not destination.is_file()):
        raise FileExistsError(f"refusing to replace non-regular authority: {destination}")
    if destination.exists():
        if destination.read_bytes() != payload:
            raise FileExistsError(
                f"refusing to overwrite differing authoritative trajectory: {destination}"
            )
        return destination
    with destination.open("xb") as stream:
        stream.write(payload)
    return destination


def fit_yaw_offsets(
    yaw_rad: Sequence[float],
    deltas_m: Sequence[Sequence[float]],
) -> YawOffsetFit:
    theta = np.asarray(yaw_rad, dtype=float)
    deltas = np.asarray(deltas_m, dtype=float)
    if theta.ndim != 1 or deltas.shape != (theta.size, 3) or theta.size < 4:
        raise ValueError("yaw fit requires N>=4 angles and an Nx3 delta array")
    if not np.all(np.isfinite(theta)) or not np.all(np.isfinite(deltas)):
        raise ValueError("yaw fit inputs must be finite")
    cosine = np.cos(theta)
    sine = np.sin(theta)
    design = np.zeros((2 * theta.size, 4), dtype=float)
    observations = np.zeros(2 * theta.size, dtype=float)
    design[0::2, :] = np.column_stack(
        (np.ones(theta.size), np.zeros(theta.size), cosine, -sine)
    )
    design[1::2, :] = np.column_stack(
        (np.zeros(theta.size), np.ones(theta.size), sine, cosine)
    )
    observations[0::2] = deltas[:, 0]
    observations[1::2] = deltas[:, 1]
    solution, _, rank, singular_values = np.linalg.lstsq(design, observations, rcond=None)
    if rank != 4 or singular_values[-1] <= 1.0e-12:
        raise ValueError("yaw samples do not identify independent XY board/tool offsets")
    bx, by, ox, oy = (float(value) for value in solution)
    predicted_xy = np.column_stack(
        (bx + cosine * ox - sine * oy, by + sine * ox + cosine * oy)
    )
    combined_z = float(np.mean(deltas[:, 2]))
    predicted = np.column_stack(
        (predicted_xy, np.full(theta.size, combined_z, dtype=float))
    )
    residual = deltas - predicted
    residual_norm = np.linalg.norm(residual, axis=1)
    return YawOffsetFit(
        board_bias_xy_m=(bx, by),
        tool_offset_xy_m=(ox, oy),
        combined_bz_plus_oz_m=combined_z,
        board_bias_radius_m=math.hypot(bx, by),
        tool_offset_radius_m=math.hypot(ox, oy),
        tool_offset_start_direction_deg=math.degrees(math.atan2(oy, ox)),
        xy_rmse_m=float(np.sqrt(np.mean(np.sum(residual[:, :2] ** 2, axis=1)))),
        xyz_rmse_m=float(np.sqrt(np.mean(residual_norm**2))),
        maximum_residual_m=float(np.max(residual_norm)),
        condition_number=float(singular_values[0] / singular_values[-1]),
    )


def _parse_nonnegative_int(raw: Any, label: str) -> int:
    try:
        value = int(raw)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label} must be an integer") from error
    if value < 0:
        raise ValueError(f"{label} must be non-negative")
    return value


def _nearest_index(sorted_values: Sequence[int], target: int) -> int:
    insertion = bisect_left(sorted_values, target)
    if insertion == 0:
        return 0
    if insertion == len(sorted_values):
        return len(sorted_values) - 1
    before = sorted_values[insertion - 1]
    after = sorted_values[insertion]
    return insertion - 1 if target - before <= after - target else insertion


def _yaw_from_quaternion(quaternion_xyzw: Sequence[float]) -> float:
    x, y, z, w = _normalized_quaternion(quaternion_xyzw)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _atomic_write_text(path: Path, text: str) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    with temporary.open("x", encoding="utf-8", newline="") as stream:
        stream.write(text)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def _atomic_write_csv(
    path: Path, fieldnames: Sequence[str], rows: Iterable[Mapping[str, Any]]
) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    with temporary.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def _read_robot_rows(
    robot_telemetry_path: Path,
    joint_names: tuple[str, ...],
) -> tuple[list[dict[str, str]], list[int]]:
    with robot_telemetry_path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"sync_time_ns", "trajectory_time_s"} | {
            f"{name}_pos" for name in joint_names
        }
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError("execution telemetry is missing actual FK columns")
        rows = list(reader)
    if len(rows) < 2:
        raise ValueError("execution telemetry contains fewer than two samples")
    sync_times = [
        _parse_nonnegative_int(row["sync_time_ns"], "robot sync_time_ns") for row in rows
    ]
    if any(
        right <= left
        for left, right in zip(sync_times, sync_times[1:], strict=False)
    ):
        raise ValueError("execution telemetry sync_time_ns must be strictly increasing")
    return rows, sync_times


def _actual_plane1_tip_poses(
    *,
    rows: Sequence[Mapping[str, str]],
    indices: Sequence[int],
    joint_names: tuple[str, ...],
    snapshot: Path,
) -> dict[int, Pose]:
    setup = load_setup_file(snapshot / "setup.yaml")
    if setup.work_frame.parent_frame != "base":
        raise ValueError("yaw finalizer currently requires setup parent_frame=base")
    urdf_path = snapshot / "robot" / "assets" / "ur5.urdf"
    model = RobotKinematics.from_urdf_bytes(
        urdf_path.read_bytes(),
        base_link="base_link",
        joint_names=joint_names,
    )
    tool_to_tip = setup.tool.parent_to_tip
    tool_pose = Pose(tool_to_tip.translation_m, tool_to_tip.quaternion_xyzw)
    base_to_plane1 = setup.work_frame.parent_to_child
    base_plane_pose = Pose(base_to_plane1.translation_m, base_to_plane1.quaternion_xyzw)
    result: dict[int, Pose] = {}
    for index in sorted(set(indices)):
        row = rows[index]
        positions = tuple(float(row[f"{name}_pos"]) for name in joint_names)
        frames = model.forward_kinematics(
            positions,
            ("base", setup.tool.tip_frame),
            tool_to_tip=tool_pose,
            tool_frame=setup.tool.parent_link,
            tip_frame=setup.tool.tip_frame,
        )
        base_link_to_plane1 = compose_poses(frames["base"], base_plane_pose)
        result[index] = compose_poses(
            inverse_pose(base_link_to_plane1), frames[setup.tool.tip_frame]
        )
    return result


def read_yaw_calibration_metadata(source_path: str | Path) -> YawSourceMetadata:
    """Validate one source as fixed-XYZ, one-axis yaw out-and-back motion."""

    source = read_timed_pose_csv(source_path)
    if len(source) < 4:
        raise ValueError("yaw calibration source has too few poses")
    initial_orientation = Pose((0.0, 0.0, 0.0), source[0].quaternion_xyzw)
    relative_yaw: list[float] = []
    for pose in source:
        relative = compose_poses(
            inverse_pose(initial_orientation),
            Pose((0.0, 0.0, 0.0), pose.quaternion_xyzw),
        )
        qx, qy, _, _ = _normalized_quaternion(relative.quaternion_xyzw)
        if abs(qx) > 1.0e-7 or abs(qy) > 1.0e-7:
            raise ValueError("yaw calibration source changes the rod-axis direction")
        relative_yaw.append(_yaw_from_quaternion(relative.quaternion_xyzw))
    unwrapped = np.unwrap(np.asarray(relative_yaw, dtype=float))
    changed = np.flatnonzero(np.abs(unwrapped) > math.radians(1.0e-4))
    if changed.size == 0:
        raise ValueError("yaw calibration source contains no yaw motion")
    first_changed_index = int(changed[0])
    preceding_holds = [
        index
        for index, pose in enumerate(source[: first_changed_index + 1])
        if pose.phase == "hold"
    ]
    yaw_start_index = (
        preceding_holds[-1]
        if preceding_holds
        else max(0, first_changed_index - 1)
    )
    yaw_end_index = max(
        index
        for index, pose in enumerate(source)
        if index >= yaw_start_index and pose.phase == "draw"
    )
    if yaw_end_index <= yaw_start_index + 1:
        raise ValueError("yaw calibration source has an incomplete yaw interval")
    fixed_position = source[yaw_start_index].position_m
    for pose in source[yaw_start_index : yaw_end_index + 1]:
        if any(
            abs(current - expected) > 1.0e-9
            for current, expected in zip(
                pose.position_m,
                fixed_position,
                strict=True,
            )
        ):
            raise ValueError("yaw calibration source changes XYZ during the yaw interval")

    yaw = unwrapped[yaw_start_index : yaw_end_index + 1]
    peak_local_index = int(np.argmax(np.abs(yaw)))
    peak_yaw = float(yaw[peak_local_index])
    intended_sweep_deg = abs(math.degrees(peak_yaw))
    matched_sweep = min((180.0, 360.0), key=lambda candidate: abs(candidate - intended_sweep_deg))
    if abs(matched_sweep - intended_sweep_deg) > 0.01:
        raise ValueError("yaw calibration source sweep must be exactly 180 or 360 degrees")
    if abs(math.degrees(float(yaw[0]))) > 0.01 or abs(
        math.degrees(float(yaw[-1]))
    ) > 0.01:
        raise ValueError("yaw calibration source must start and return to zero yaw")
    direction = 1.0 if peak_yaw >= 0.0 else -1.0
    directed = direction * yaw
    monotonic_tolerance = math.radians(1.0e-5)
    if np.any(np.diff(directed[: peak_local_index + 1]) < -monotonic_tolerance):
        raise ValueError("yaw calibration outbound leg is not monotonic")
    if np.any(np.diff(directed[peak_local_index:]) > monotonic_tolerance):
        raise ValueError("yaw calibration return leg is not monotonic")

    peak_index = yaw_start_index + peak_local_index
    return YawSourceMetadata(
        yaw_start_s=float(source[yaw_start_index].time_s),
        turnaround_s=float(source[peak_index].time_s),
        yaw_end_s=float(source[yaw_end_index].time_s),
        source_end_s=float(source[-1].time_s),
        intended_sweep_deg=matched_sweep,
        fixed_tip_position_m=fixed_position,
    )


def finalize_yaw_tracker_capture(
    summary: TrackerCaptureSummary,
    robot_telemetry_path: str | Path,
    recording_path: str | Path,
) -> YawTrackerFinalizeResult:
    """Map tracker samples to actual FK and fit board/tool XY offsets."""

    robot_path = Path(robot_telemetry_path).expanduser().resolve()
    recording = Path(recording_path).expanduser().resolve()
    run_directory = robot_path.parent
    manifest_value = yaml.safe_load((recording / "manifest.yaml").read_text(encoding="utf-8"))
    try:
        joint_names = tuple(str(name) for name in manifest_value["robot"]["joint_names"])
    except (KeyError, TypeError) as error:
        raise ValueError("recording manifest is missing robot joint_names") from error
    robot_rows, robot_sync_times = _read_robot_rows(robot_path, joint_names)
    if not summary.samples:
        raise ValueError("tracker capture contains no valid PoseStamped samples")
    first_sync, last_sync = robot_sync_times[0], robot_sync_times[-1]
    mapped_samples = [
        sample
        for sample in summary.samples
        if first_sync <= sample.tracker_rx_sync_time_ns <= last_sync
    ]
    if len(mapped_samples) < summary.config.minimum_mapped_samples:
        raise ValueError(
            "too few tracker samples overlap robot telemetry "
            f"({len(mapped_samples)} < {summary.config.minimum_mapped_samples})"
        )
    robot_indices = [
        _nearest_index(robot_sync_times, sample.tracker_rx_sync_time_ns)
        for sample in mapped_samples
    ]
    offsets_ns = [
        sample.tracker_rx_sync_time_ns - robot_sync_times[index]
        for sample, index in zip(mapped_samples, robot_indices, strict=True)
    ]
    maximum_mapping_offset_ns = max(abs(value) for value in offsets_ns)
    allowed_offset_ns = int(summary.config.max_mapping_offset_s * 1_000_000_000)
    if maximum_mapping_offset_ns > allowed_offset_ns:
        raise ValueError(
            "tracker-to-robot mapping exceeds allowed offset "
            f"({maximum_mapping_offset_ns} > {allowed_offset_ns} ns)"
        )
    metadata = read_yaw_calibration_metadata(
        recording / "source_tip_trajectory.csv"
    )
    robot_trajectory_times = [
        float(row["trajectory_time_s"]) for row in robot_rows
    ]
    if not all(math.isfinite(value) and value >= 0.0 for value in robot_trajectory_times):
        raise ValueError("execution telemetry trajectory_time_s must be finite and non-negative")
    if any(
        right < left
        for left, right in zip(
            robot_trajectory_times,
            robot_trajectory_times[1:],
            strict=False,
        )
    ):
        raise ValueError("execution telemetry trajectory_time_s must be non-decreasing")
    yaw_reference_index = min(
        range(len(robot_trajectory_times)),
        key=lambda index: abs(robot_trajectory_times[index] - metadata.yaw_start_s),
    )
    actual_poses = _actual_plane1_tip_poses(
        rows=robot_rows,
        indices=(*robot_indices, yaw_reference_index),
        joint_names=joint_names,
        snapshot=recording / "environment_snapshot",
    )
    reference_pose = actual_poses[yaw_reference_index]
    raw_actual_yaw: list[float] = []
    deltas: list[tuple[float, float, float]] = []
    trajectory_times: list[float] = []
    for sample, index in zip(mapped_samples, robot_indices, strict=True):
        actual = actual_poses[index]
        relative = compose_poses(
            inverse_pose(Pose((0.0, 0.0, 0.0), reference_pose.quaternion_xyzw)),
            Pose((0.0, 0.0, 0.0), actual.quaternion_xyzw),
        )
        raw_actual_yaw.append(_yaw_from_quaternion(relative.quaternion_xyzw))
        deltas.append(
            tuple(
                sample.position_m[axis] - actual.position_m[axis] for axis in range(3)
            )  # type: ignore[arg-type]
        )
        trajectory_times.append(float(robot_rows[index]["trajectory_time_s"]))
    actual_yaw = np.unwrap(np.asarray(raw_actual_yaw, dtype=float))
    fit_indices = [
        index
        for index, time_s in enumerate(trajectory_times)
        if metadata.yaw_start_s - 1.0e-9
        <= time_s
        <= metadata.yaw_end_s + 1.0e-9
    ]
    if len(fit_indices) < summary.config.minimum_mapped_samples:
        raise ValueError("too few tracker samples fall inside the yaw draw window")
    fit_times = np.asarray([trajectory_times[index] for index in fit_indices], dtype=float)
    coverage_tolerance_s = 1.0
    if (
        float(np.min(fit_times)) > metadata.yaw_start_s + coverage_tolerance_s
        or float(np.max(fit_times)) < metadata.yaw_end_s - coverage_tolerance_s
        or float(np.min(np.abs(fit_times - metadata.turnaround_s)))
        > coverage_tolerance_s
    ):
        raise ValueError("tracker samples do not cover yaw start, turnaround, and return")
    fit_yaw = actual_yaw[fit_indices]
    actual_sweep_deg = abs(math.degrees(float(fit_yaw[np.argmax(np.abs(fit_yaw))])))
    if actual_sweep_deg < 0.9 * metadata.intended_sweep_deg:
        raise ValueError(
            "actual FK yaw sweep is too small for the authoritative source "
            f"({actual_sweep_deg:.3f} < {0.9 * metadata.intended_sweep_deg:.3f} deg)"
        )
    final_fit_index = fit_indices[int(np.argmax(fit_times))]
    if abs(math.degrees(float(actual_yaw[final_fit_index]))) > 10.0:
        raise ValueError("actual FK yaw did not return close to its starting orientation")
    fit = fit_yaw_offsets(
        fit_yaw,
        np.asarray(deltas, dtype=float)[fit_indices, :],
    )

    mapped_rows: list[dict[str, Any]] = []
    for sample, robot_index, offset_ns, yaw, delta in zip(
        mapped_samples,
        robot_indices,
        offsets_ns,
        actual_yaw,
        deltas,
        strict=True,
    ):
        actual = actual_poses[robot_index]
        mapped_rows.append(
            {
                "sync_time_ns": robot_sync_times[robot_index],
                "trajectory_time_s": robot_rows[robot_index]["trajectory_time_s"],
                "robot_row_index": robot_index,
                "tracker_rx_sync_time_ns": sample.tracker_rx_sync_time_ns,
                "sync_offset_ns": offset_ns,
                "tracker_message_ros_time_ns": sample.tracker_message_ros_time_ns,
                "tracker_x_plane1_m": format(sample.position_m[0], ".17g"),
                "tracker_y_plane1_m": format(sample.position_m[1], ".17g"),
                "tracker_z_plane1_m": format(sample.position_m[2], ".17g"),
                "rod_tip_x_plane1_m": format(actual.position_m[0], ".17g"),
                "rod_tip_y_plane1_m": format(actual.position_m[1], ".17g"),
                "rod_tip_z_plane1_m": format(actual.position_m[2], ".17g"),
                "actual_yaw_rad": format(float(yaw), ".17g"),
                "delta_x_m": format(delta[0], ".17g"),
                "delta_y_m": format(delta[1], ".17g"),
                "delta_z_m": format(delta[2], ".17g"),
                "used_for_yaw_fit": int(
                    metadata.yaw_start_s - 1.0e-9
                    <= float(robot_rows[robot_index]["trajectory_time_s"])
                    <= metadata.yaw_end_s + 1.0e-9
                ),
            }
        )

    tracker_telemetry_path = run_directory / "tracker_telemetry.csv"
    tracker_raw_path = run_directory / "tracker_raw.csv"
    report_path = run_directory / "yaw_calibration_report.yaml"
    for path in (tracker_telemetry_path, tracker_raw_path, report_path):
        if path.exists() or path.is_symlink():
            raise FileExistsError(f"refusing to replace existing tracker artifact: {path}")
    _atomic_write_csv(tracker_telemetry_path, TRACKER_MAPPED_COLUMNS, mapped_rows)
    shutil.copy2(summary.staging_path, tracker_raw_path)

    actual_positions = np.asarray(
        [actual_poses[robot_indices[index]].position_m for index in fit_indices],
        dtype=float,
    )
    center = np.mean(actual_positions, axis=0)
    rod_tip_max_deviation_m = float(
        np.max(np.linalg.norm(actual_positions - center, axis=1))
    )
    classification_threshold_m = 0.002
    observed_fixed_offset_norm_m = math.hypot(
        fit.board_bias_radius_m,
        fit.combined_bz_plus_oz_m,
    )
    fixed_present = observed_fixed_offset_norm_m >= classification_threshold_m
    tool_present = fit.tool_offset_radius_m >= classification_threshold_m
    classification = (
        "combined_fixed_and_tool_offset"
        if fixed_present and tool_present
        else "fixed_offset_dominant"
        if fixed_present
        else "tool_eccentricity_dominant"
        if tool_present
        else "both_offsets_below_2mm_threshold"
    )
    report: dict[str, Any] = {
        "schema_version": YAW_CALIBRATION_SCHEMA_VERSION,
        "status": "complete",
        "frame_id": summary.config.frame_id,
        "equation": "p_tracker(theta)-p_rodtip=b+Rz(theta)o",
        "recording_id": recording.name,
        "samples": {
            "captured": len(summary.samples),
            "mapped": len(mapped_samples),
            "fit": len(fit_indices),
            "invalid_messages": summary.invalid_message_count,
        },
        "timing": {
            "maximum_mapping_offset_ns": maximum_mapping_offset_ns,
            "maximum_allowed_mapping_offset_ns": allowed_offset_ns,
        },
        "sweep": {
            "warmup_and_settle_end_s": metadata.yaw_start_s,
            "yaw_start_s": metadata.yaw_start_s,
            "turnaround_s": metadata.turnaround_s,
            "yaw_end_s": metadata.yaw_end_s,
            "source_end_s": metadata.source_end_s,
            "intended_sweep_deg": metadata.intended_sweep_deg,
            "actual_sweep_deg": actual_sweep_deg,
            "actual_yaw_min_deg": math.degrees(float(np.min(actual_yaw[fit_indices]))),
            "actual_yaw_max_deg": math.degrees(float(np.max(actual_yaw[fit_indices]))),
            "rod_tip_max_position_deviation_m": rod_tip_max_deviation_m,
        },
        "fit": {
            "board_bias_b_xy_m": list(fit.board_bias_xy_m),
            "board_bias_b_z_m": None,
            "board_bias_xy_norm_m": fit.board_bias_radius_m,
            "observed_fixed_offset_norm_m": observed_fixed_offset_norm_m,
            "tool_offset_o_xy_m": list(fit.tool_offset_xy_m),
            "tool_offset_o_z_m": None,
            "tool_offset_radius_m": fit.tool_offset_radius_m,
            "tool_offset_start_direction_deg": fit.tool_offset_start_direction_deg,
            "combined_bz_plus_oz_m": fit.combined_bz_plus_oz_m,
            "xy_rmse_m": fit.xy_rmse_m,
            "xyz_rmse_m": fit.xyz_rmse_m,
            "maximum_residual_m": fit.maximum_residual_m,
            "condition_number": fit.condition_number,
            "classification_threshold_m": classification_threshold_m,
            "classification": classification,
        },
        "identifiability": {
            "xy": "b_xy and o_xy are independently identifiable from yaw rotation",
            "z": "pure yaw identifies only b_z + o_z; b_z and o_z cannot be separated",
        },
        "artifacts": {
            "robot_telemetry_csv": str(robot_path),
            "tracker_raw_csv": str(tracker_raw_path),
            "tracker_telemetry_csv": str(tracker_telemetry_path),
        },
    }
    _atomic_write_text(report_path, yaml.safe_dump(report, sort_keys=False))
    summary.staging_path.unlink()
    return YawTrackerFinalizeResult(
        status="complete",
        tracker_telemetry_path=tracker_telemetry_path,
        tracker_raw_path=tracker_raw_path,
        report_path=report_path,
        sample_count=len(mapped_samples),
        maximum_mapping_offset_ns=maximum_mapping_offset_ns,
        fit=fit,
    )
