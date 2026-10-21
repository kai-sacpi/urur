"""Collision-planned live static-point capture without recording artifacts.

The module keeps ROS imports inside :func:`main` so grid generation and sensor
mapping remain testable on the development Mac.
"""

from __future__ import annotations

import argparse
from bisect import bisect_left
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import time
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_curobo.math3d import inverse_pose, matrix_pose

from .planning_scene_publisher import load_planning_scene_definition
from .tcp_console import (
    AbsolutePoseCommand,
    RosTcpSession,
    _validate_args as validate_tcp_console_args,
    build_argument_parser as build_tcp_console_parser,
)


DEFAULT_COORDINATES_CM = (-15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0)
DEFAULT_HEIGHTS_CM = (7.0, 9.0, 11.0, 13.0, 15.0)
PARSER_COUNTER_COLUMNS = (
    "parser_duplicate_or_reordered_groups",
    "parser_incomplete_groups",
    "parser_duplicate_packets",
    "parser_reordered_packets",
    "parser_channel_sequence_drops",
    "parser_source_sequence_drops",
    "parser_bytes_discarded",
    "parser_malformed_packets",
    "parser_unexpected_channel_packets",
    "parser_pending_evictions",
)


@dataclass(frozen=True)
class StaticGridPoint:
    sequence: int
    point_id: str
    x_m: float
    y_m: float
    height_m: float

    @property
    def plane1_position_m(self) -> tuple[float, float, float]:
        # The verified MLA frame has physical height along plane1 -Z.
        return (self.x_m, self.y_m, -self.height_m)

    @property
    def vertical_command(self) -> AbsolutePoseCommand:
        return AbsolutePoseCommand(
            frame="plane1",
            position_m=self.plane1_position_m,
            rotation_vector_rad=(0.0, 0.0, 0.0),
        )


@dataclass(frozen=True)
class CaptureWindow:
    sequence: int
    point_id: str
    target_x_m: float
    target_y_m: float
    target_height_m: float
    target_plane1_z_m: float
    start_sync_time_ns: int
    end_sync_time_ns: int
    robot_sample_count: int


def _axis_id(value_m: float) -> str:
    millimetres = int(round(value_m * 1000.0))
    if millimetres > 0:
        return f"p{millimetres:03d}"
    if millimetres < 0:
        return f"m{abs(millimetres):03d}"
    return "z000"


def build_serpentine_grid(
    coordinates_cm: Sequence[float] = DEFAULT_COORDINATES_CM,
    heights_cm: Sequence[float] = DEFAULT_HEIGHTS_CM,
    exclude_interior_cm: float | None = None,
) -> tuple[StaticGridPoint, ...]:
    """Build a unique nearest-neighbour grid, starting at the safest height."""

    coordinates = tuple(float(value) / 100.0 for value in coordinates_cm)
    heights = tuple(sorted((float(value) / 100.0 for value in heights_cm), reverse=True))
    if not coordinates or not heights:
        raise ValueError("grid coordinates and heights must not be empty")
    if not all(math.isfinite(value) for value in (*coordinates, *heights)):
        raise ValueError("grid values must be finite")
    if len(set(coordinates)) != len(coordinates) or len(set(heights)) != len(heights):
        raise ValueError("grid axes must contain unique values")
    if any(height <= 0.0 for height in heights):
        raise ValueError("experimental heights must be positive")

    ys = tuple(sorted(coordinates))
    xs = tuple(sorted(coordinates))
    if exclude_interior_cm is None:
        base_layer: list[tuple[float, float]] = []
        for row_index, y_m in enumerate(ys):
            row = xs if row_index % 2 == 0 else tuple(reversed(xs))
            base_layer.extend((x_m, y_m) for x_m in row)
    else:
        interior_m = float(exclude_interior_cm) / 100.0
        if not math.isfinite(interior_m) or interior_m < 0.0:
            raise ValueError("excluded interior half-width must be finite and non-negative")
        remaining = {
            (x_m, y_m)
            for y_m in ys
            for x_m in xs
            if abs(x_m) > interior_m + 1.0e-12
            or abs(y_m) > interior_m + 1.0e-12
        }
        if not remaining:
            raise ValueError("excluded interior removes every requested XY point")
        # Start near the positive-X mid-edge, then greedily walk the perimeter.
        # For the regular grids used here this gives exactly one 5 cm axis step
        # between neighbours and avoids a 40 cm cross-board jump.
        current = min(
            remaining,
            key=lambda item: ((item[0] - max(xs)) ** 2 + item[1] ** 2, item[1], item[0]),
        )
        base_layer = [current]
        remaining.remove(current)
        while remaining:
            current = min(
                remaining,
                key=lambda item: (
                    (item[0] - current[0]) ** 2 + (item[1] - current[1]) ** 2,
                    item[1],
                    item[0],
                ),
            )
            base_layer.append(current)
            remaining.remove(current)

    result: list[StaticGridPoint] = []
    for layer_index, height_m in enumerate(heights):
        layer = base_layer if layer_index % 2 == 0 else list(reversed(base_layer))
        for x_m, y_m in layer:
            sequence = len(result) + 1
            point_id = (
                f"z{int(round(height_m * 1000.0)):03d}_"
                f"x{_axis_id(x_m)}_y{_axis_id(y_m)}"
            )
            result.append(
                StaticGridPoint(
                    sequence=sequence,
                    point_id=point_id,
                    x_m=x_m,
                    y_m=y_m,
                    height_m=height_m,
                )
            )
    return tuple(result)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
        stream.flush()
    temporary.replace(path)


def _write_schedule(path: Path, points: Sequence[StaticGridPoint]) -> None:
    with path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            (
                "sequence",
                "point_id",
                "target_x_m",
                "target_y_m",
                "target_height_m",
                "target_plane1_z_m",
                "tip_qx",
                "tip_qy",
                "tip_qz",
                "tip_qw",
            )
        )
        for point in points:
            writer.writerow(
                (
                    point.sequence,
                    point.point_id,
                    f"{point.x_m:.9f}",
                    f"{point.y_m:.9f}",
                    f"{point.height_m:.9f}",
                    f"{-point.height_m:.9f}",
                    "0",
                    "0",
                    "0",
                    "1",
                )
            )


def _robot_columns(joint_names: Sequence[str]) -> tuple[str, ...]:
    return (
        "point_sequence",
        "point_id",
        "target_x_m",
        "target_y_m",
        "target_height_m",
        "target_plane1_z_m",
        "sync_time_ns",
        "ros_time_ns",
        "capture_elapsed_s",
        "actual_tip_base_x_m",
        "actual_tip_base_y_m",
        "actual_tip_base_z_m",
        "actual_tip_base_qx",
        "actual_tip_base_qy",
        "actual_tip_base_qz",
        "actual_tip_base_qw",
        "actual_tip_plane1_x_m",
        "actual_tip_plane1_y_m",
        "actual_tip_plane1_z_m",
        "actual_tip_plane1_qx",
        "actual_tip_plane1_qy",
        "actual_tip_plane1_qz",
        "actual_tip_plane1_qw",
        *(f"joint_{name}_position_rad" for name in joint_names),
        *(f"joint_{name}_velocity_rad_s" for name in joint_names),
    )


def _capture_robot_window(
    *,
    session: RosTcpSession,
    point: StaticGridPoint,
    writer: csv.DictWriter[str],
    stream: Any,
    base_to_plane1: np.ndarray,
    tool0_to_tip: np.ndarray,
    duration_s: float,
    sample_hz: float,
) -> CaptureWindow:
    period_ns = int(round(1_000_000_000.0 / sample_hz))
    duration_ns = int(round(duration_s * 1_000_000_000.0))
    start_ns = time.monotonic_ns()
    deadline_ns = start_ns + duration_ns
    next_ns = start_ns
    count = 0
    while True:
        now_ns = time.monotonic_ns()
        if now_ns >= deadline_ns:
            break
        if now_ns < next_ns:
            time.sleep((next_ns - now_ns) / 1_000_000_000.0)
        snapshot = session.latest_joints()
        if snapshot is None:
            raise RuntimeError("actual JointState disappeared during static capture")
        if time.monotonic() - snapshot.received_monotonic_s > session.args.state_freshness_s:
            raise RuntimeError("actual JointState became stale during static capture")
        base_to_tip = session.current_tip_matrix(
            timeout_s=min(0.2, session.args.service_timeout_s),
            tool0_to_tip=tool0_to_tip,
        )
        plane1_to_tip = inverse_pose(base_to_plane1) @ base_to_tip
        base_position, base_quaternion = matrix_pose(base_to_tip)
        plane_position, plane_quaternion = matrix_pose(plane1_to_tip)
        sync_ns = time.monotonic_ns()
        velocities = snapshot.velocities_rad_s
        if velocities is None:
            raise RuntimeError("actual joint velocities disappeared during static capture")
        row: dict[str, str] = {
            "point_sequence": str(point.sequence),
            "point_id": point.point_id,
            "target_x_m": f"{point.x_m:.9f}",
            "target_y_m": f"{point.y_m:.9f}",
            "target_height_m": f"{point.height_m:.9f}",
            "target_plane1_z_m": f"{-point.height_m:.9f}",
            "sync_time_ns": str(sync_ns),
            "ros_time_ns": str(int(session.node.get_clock().now().nanoseconds)),
            "capture_elapsed_s": f"{(sync_ns - start_ns) / 1e9:.9f}",
        }
        for prefix, position, quaternion in (
            ("actual_tip_base", base_position, base_quaternion),
            ("actual_tip_plane1", plane_position, plane_quaternion),
        ):
            row[f"{prefix}_x_m"] = f"{position[0]:.12f}"
            row[f"{prefix}_y_m"] = f"{position[1]:.12f}"
            row[f"{prefix}_z_m"] = f"{position[2]:.12f}"
            row[f"{prefix}_qx"] = f"{quaternion[0]:.12f}"
            row[f"{prefix}_qy"] = f"{quaternion[1]:.12f}"
            row[f"{prefix}_qz"] = f"{quaternion[2]:.12f}"
            row[f"{prefix}_qw"] = f"{quaternion[3]:.12f}"
        for name, position, velocity in zip(
            session.environment.joint_names,
            snapshot.positions_rad,
            velocities,
            strict=True,
        ):
            row[f"joint_{name}_position_rad"] = f"{position:.12f}"
            row[f"joint_{name}_velocity_rad_s"] = f"{velocity:.12f}"
        writer.writerow(row)
        count += 1
        next_ns += period_ns
        if next_ns < sync_ns:
            next_ns = sync_ns + period_ns
    end_ns = time.monotonic_ns()
    stream.flush()
    session.wait_settled(timeout_s=session.args.state_timeout_s, after_s=start_ns / 1e9)
    return CaptureWindow(
        sequence=point.sequence,
        point_id=point.point_id,
        target_x_m=point.x_m,
        target_y_m=point.y_m,
        target_height_m=point.height_m,
        target_plane1_z_m=-point.height_m,
        start_sync_time_ns=start_ns,
        end_sync_time_ns=end_ns,
        robot_sample_count=count,
    )


def _nearest_index(values: Sequence[int], target: int) -> int:
    insertion = bisect_left(values, target)
    if insertion <= 0:
        return 0
    if insertion >= len(values):
        return len(values) - 1
    before = values[insertion - 1]
    after = values[insertion]
    return insertion - 1 if target - before <= after - target else insertion


def map_sensor_windows(
    *,
    raw_sensor_path: Path,
    robot_path: Path,
    windows: Sequence[CaptureWindow],
    mapped_path: Path,
    quality_path: Path,
) -> dict[str, Any]:
    """Map only in-window sensor frames to the nearest same-point FK sample."""

    robot_rows: dict[int, list[dict[str, str]]] = {}
    with robot_path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        robot_fieldnames = tuple(reader.fieldnames or ())
        for row in reader:
            robot_rows.setdefault(int(row["point_sequence"]), []).append(row)
    robot_times = {
        sequence: tuple(int(row["sync_time_ns"]) for row in rows)
        for sequence, rows in robot_rows.items()
    }

    ordered_windows = tuple(sorted(windows, key=lambda item: item.start_sync_time_ns))
    quality: dict[int, dict[str, Any]] = {
        window.sequence: {
            "window": window,
            "sensor_frames": 0,
            "maximum_mapping_offset_ns": 0,
            "maximum_inter_frame_gap_ns": 0,
            "previous_sensor_ns": None,
            "baseline": None,
            "last": None,
        }
        for window in ordered_windows
    }

    mapped_count = 0
    outside_count = 0
    window_index = 0
    previous_counters = {name: 0 for name in PARSER_COUNTER_COLUMNS}
    with raw_sensor_path.open("r", encoding="utf-8", newline="") as source:
        reader = csv.DictReader(source)
        raw_fieldnames = tuple(reader.fieldnames or ())
        missing = [name for name in PARSER_COUNTER_COLUMNS if name not in raw_fieldnames]
        if missing or "sensor_rx_sync_time_ns" not in raw_fieldnames:
            raise ValueError("raw NUCLEO CSV is missing required columns")
        mapped_fieldnames = (
            "point_sequence",
            "point_id",
            "target_x_m",
            "target_y_m",
            "target_height_m",
            "target_plane1_z_m",
            "robot_sync_time_ns",
            "sync_offset_ns",
            "robot_row_index_within_point",
            *(f"robot_{name}" for name in robot_fieldnames if name not in {
                "point_sequence", "point_id", "target_x_m", "target_y_m",
                "target_height_m", "target_plane1_z_m", "sync_time_ns",
            }),
            *raw_fieldnames,
        )
        with mapped_path.open("x", encoding="utf-8", newline="") as target:
            writer = csv.DictWriter(target, fieldnames=mapped_fieldnames, lineterminator="\n")
            writer.writeheader()
            for sensor_row in reader:
                sensor_ns = int(sensor_row["sensor_rx_sync_time_ns"])
                counters = {
                    name: int(sensor_row[name]) for name in PARSER_COUNTER_COLUMNS
                }
                while (
                    window_index < len(ordered_windows)
                    and sensor_ns > ordered_windows[window_index].end_sync_time_ns
                ):
                    window_index += 1
                if window_index >= len(ordered_windows):
                    outside_count += 1
                    previous_counters = counters
                    continue
                window = ordered_windows[window_index]
                if sensor_ns < window.start_sync_time_ns:
                    outside_count += 1
                    previous_counters = counters
                    continue

                state = quality[window.sequence]
                if state["baseline"] is None:
                    state["baseline"] = dict(previous_counters)
                state["last"] = dict(counters)
                previous_sensor_ns = state["previous_sensor_ns"]
                if previous_sensor_ns is not None:
                    state["maximum_inter_frame_gap_ns"] = max(
                        int(state["maximum_inter_frame_gap_ns"]),
                        sensor_ns - int(previous_sensor_ns),
                    )
                state["previous_sensor_ns"] = sensor_ns

                rows = robot_rows.get(window.sequence, ())
                times = robot_times.get(window.sequence, ())
                if not rows or not times:
                    raise ValueError(f"window {window.point_id} has no robot FK samples")
                robot_index = _nearest_index(times, sensor_ns)
                robot_row = rows[robot_index]
                offset_ns = sensor_ns - times[robot_index]
                state["maximum_mapping_offset_ns"] = max(
                    int(state["maximum_mapping_offset_ns"]), abs(offset_ns)
                )
                state["sensor_frames"] = int(state["sensor_frames"]) + 1

                output: dict[str, str | int] = {
                    "point_sequence": window.sequence,
                    "point_id": window.point_id,
                    "target_x_m": f"{window.target_x_m:.9f}",
                    "target_y_m": f"{window.target_y_m:.9f}",
                    "target_height_m": f"{window.target_height_m:.9f}",
                    "target_plane1_z_m": f"{window.target_plane1_z_m:.9f}",
                    "robot_sync_time_ns": times[robot_index],
                    "sync_offset_ns": offset_ns,
                    "robot_row_index_within_point": robot_index,
                }
                for name in robot_fieldnames:
                    if name not in {
                        "point_sequence", "point_id", "target_x_m", "target_y_m",
                        "target_height_m", "target_plane1_z_m", "sync_time_ns",
                    }:
                        output[f"robot_{name}"] = robot_row[name]
                output.update(sensor_row)
                writer.writerow(output)
                mapped_count += 1
                previous_counters = counters

    quality_columns = (
        "point_sequence",
        "point_id",
        "sensor_frames",
        "maximum_mapping_offset_ns",
        "maximum_inter_frame_gap_ns",
        *PARSER_COUNTER_COLUMNS,
    )
    quality_rows: list[dict[str, str | int]] = []
    with quality_path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=quality_columns, lineterminator="\n")
        writer.writeheader()
        for window in ordered_windows:
            state = quality[window.sequence]
            baseline = state["baseline"] or {name: 0 for name in PARSER_COUNTER_COLUMNS}
            last = state["last"] or baseline
            row: dict[str, str | int] = {
                "point_sequence": window.sequence,
                "point_id": window.point_id,
                "sensor_frames": int(state["sensor_frames"]),
                "maximum_mapping_offset_ns": int(state["maximum_mapping_offset_ns"]),
                "maximum_inter_frame_gap_ns": int(state["maximum_inter_frame_gap_ns"]),
            }
            for name in PARSER_COUNTER_COLUMNS:
                row[name] = int(last[name]) - int(baseline[name])
            writer.writerow(row)
            quality_rows.append(row)

    return {
        "mapped_sensor_frames": mapped_count,
        "outside_capture_window_frames": outside_count,
        "maximum_mapping_offset_ns": max(
            (int(row["maximum_mapping_offset_ns"]) for row in quality_rows),
            default=0,
        ),
        "minimum_sensor_frames_per_point": min(
            (int(row["sensor_frames"]) for row in quality_rows),
            default=0,
        ),
        "maximum_sensor_frames_per_point": max(
            (int(row["sensor_frames"]) for row in quality_rows),
            default=0,
        ),
        "window_quality_totals": {
            name: sum(int(row[name]) for row in quality_rows)
            for name in PARSER_COUNTER_COLUMNS
        },
    }


def _target_rejection(error: Exception) -> bool:
    message = str(error)
    return any(
        marker in message
        for marker in (
            "MoveIt collision-checked IK failed",
            "MoveIt joint-space planning failed",
            "has no exactly equivalent winding",
            "leaves the configured local/canonical interval",
            "no valid local joint interval",
        )
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="static-grid-capture",
        description=(
            "Move the authoritative rod tip through exact static targets with MoveIt "
            "collision checking and one continuous NUCLEO capture."
        ),
    )
    parser.add_argument("--workspace", type=Path, default=Path.cwd())
    parser.add_argument("--env", default="mla")
    parser.add_argument("--coordinates-cm", type=float, nargs="+", default=DEFAULT_COORDINATES_CM)
    parser.add_argument("--heights-cm", type=float, nargs="+", default=DEFAULT_HEIGHTS_CM)
    parser.add_argument(
        "--exclude-interior-cm",
        type=float,
        default=None,
        help="exclude XY points whose |x| and |y| are both at most this value",
    )
    parser.add_argument("--capture-s", type=float, default=3.0)
    parser.add_argument("--robot-sample-hz", type=float, default=100.0)
    parser.add_argument("--velocity-scale", type=float, default=0.15)
    parser.add_argument("--acceleration-scale", type=float, default=0.15)
    parser.add_argument("--planning-time-s", type=float, default=5.0)
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--nucleo-port", required=True)
    sync = parser.add_mutually_exclusive_group(required=True)
    sync.add_argument("--perform-initial-sync", action="store_true")
    sync.add_argument("--reuse-sync", action="store_true")
    parser.add_argument(
        "--output-parent",
        type=Path,
        default=Path("/data/ur-waypoint/mla_vertical_grid_live"),
    )
    parser.add_argument("--start-sequence", type=int, default=1)
    return parser


def _validated_args(argv: list[str] | None) -> argparse.Namespace:
    args = _build_parser().parse_args(argv)
    for name in ("capture_s", "robot_sample_hz", "velocity_scale", "acceleration_scale", "planning_time_s"):
        value = float(getattr(args, name))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"--{name.replace('_', '-')} must be finite and positive")
    if args.velocity_scale > 1.0 or args.acceleration_scale > 1.0:
        raise ValueError("velocity and acceleration scales cannot exceed 1")
    if args.planning_attempts <= 0:
        raise ValueError("--planning-attempts must be positive")
    return args


def _console_args(args: argparse.Namespace, workspace: Path) -> argparse.Namespace:
    values = [
        "--workspace", str(workspace),
        "--env", str(args.env),
        "--enable-motion",
        "--max-translation-mm", "1000",
        "--max-absolute-translation-mm", "1000",
        "--max-rotation-deg", "180",
        "--max-joint-excursion-deg", "180",
        "--planning-attempts", str(args.planning_attempts),
        "--allowed-planning-time", str(args.planning_time_s),
        "--velocity-scale", str(args.velocity_scale),
        "--acceleration-scale", str(args.acceleration_scale),
    ]
    console_args = build_tcp_console_parser().parse_args(values)
    validate_tcp_console_args(console_args)
    return console_args


def main(argv: list[str] | None = None) -> int:
    try:
        args = _validated_args(argv)
        workspace = args.workspace.expanduser().resolve()
        if args.env not in {"mla", "mla_short"}:
            raise ValueError(
                "this production grid command is restricted to --env mla or mla_short"
            )
        points = build_serpentine_grid(
            args.coordinates_cm,
            args.heights_cm,
            exclude_interior_cm=args.exclude_interior_cm,
        )
        if not 1 <= args.start_sequence <= len(points):
            raise ValueError("--start-sequence is outside the generated grid")
        points = tuple(point for point in points if point.sequence >= args.start_sequence)
        output_parent = args.output_parent.expanduser().resolve()
        output_parent.mkdir(parents=True, exist_ok=True)
        run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        output = output_parent / run_id
        output.mkdir(parents=False, exist_ok=False)
        _write_schedule(output / "schedule.csv", points)
        scene = load_planning_scene_definition(workspace, args.env)
        environment = EnvironmentBuilder().load(workspace / "environments" / args.env)
        if scene.environment_id not in {"mla", "mla_short"} or environment.environment_id != scene.environment_id:
            raise RuntimeError(
                "the resolved environment is not an authoritative MLA sensor-centered bundle"
            )
        console_args = _console_args(args, workspace)
    except (OSError, RuntimeError, ValueError) as error:
        print(f"static-grid-capture setup failed: {error}", file=sys.stderr)
        return 2

    try:
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from hx_waypoint_nucleo import NucleoCaptureConfig, NucleoCaptureSession
    except ImportError as error:
        print(f"static-grid-capture requires the sourced ROS workspace: {error}", file=sys.stderr)
        return 2

    result_columns = (
        "sequence", "point_id", "target_x_m", "target_y_m", "target_height_m",
        "target_plane1_z_m", "status", "motion_started_utc", "motion_duration_s",
        "motion_translation_m", "motion_rotation_deg", "capture_start_sync_time_ns",
        "capture_end_sync_time_ns", "robot_sample_count", "nucleo_frame_delta", "message",
    )
    window_columns = (
        "sequence", "point_id", "target_x_m", "target_y_m", "target_height_m",
        "target_plane1_z_m", "start_sync_time_ns", "end_sync_time_ns", "robot_sample_count",
    )
    fatal_error: str | None = None
    capture_summary: Any | None = None
    capture_session: Any | None = None
    session: RosTcpSession | None = None
    windows: list[CaptureWindow] = []
    captured_count = 0
    unreachable_count = 0
    park_status = "not_attempted"
    robot_path = output / "robot_actual_fk.csv"
    results_path = output / "point_results.csv"
    windows_path = output / "capture_windows.csv"

    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    try:
        session = RosTcpSession(environment, scene, console_args)
        session.wait_fresh_joint_state(timeout_s=console_args.state_timeout_s)
        session.wait_settled(timeout_s=console_args.state_timeout_s)
        base_to_plane1, tool0_to_tip = session.live_authoritative_transforms()
        capture_workspace = output_parent / "capture_workspace"
        capture_workspace.mkdir(parents=True, exist_ok=True)
        capture_session = NucleoCaptureSession(
            capture_workspace,
            NucleoCaptureConfig(
                port=args.nucleo_port,
                baudrate=115200,
                channels=(1, 2, 3, 4),
                perform_sync=bool(args.perform_initial_sync),
                preflight_timeout_s=5.0,
            ),
            lambda: int(session.node.get_clock().now().nanoseconds),
        )
        capture_session.start()
        print(
            "NUCLEO ready; "
            + ("SYNC performed once" if args.perform_initial_sync else "existing SYNC reused"),
            flush=True,
        )

        with (
            robot_path.open("x", encoding="utf-8", newline="") as robot_stream,
            results_path.open("x", encoding="utf-8", newline="") as results_stream,
            windows_path.open("x", encoding="utf-8", newline="") as windows_stream,
        ):
            robot_writer = csv.DictWriter(
                robot_stream,
                fieldnames=_robot_columns(environment.joint_names),
                lineterminator="\n",
            )
            result_writer = csv.DictWriter(results_stream, fieldnames=result_columns, lineterminator="\n")
            window_writer = csv.DictWriter(windows_stream, fieldnames=window_columns, lineterminator="\n")
            robot_writer.writeheader()
            result_writer.writeheader()
            window_writer.writeheader()

            for batch_index, point in enumerate(points, start=1):
                print(
                    f"===== [{batch_index}/{len(points)}; grid {point.sequence}] {point.point_id} =====",
                    flush=True,
                )
                motion_started_utc = datetime.now(timezone.utc).isoformat()
                motion_started = time.monotonic()
                try:
                    delta = session.move_absolute(point.vertical_command)
                except Exception as error:
                    if not _target_rejection(error):
                        raise
                    unreachable_count += 1
                    result_writer.writerow(
                        {
                            "sequence": point.sequence,
                            "point_id": point.point_id,
                            "target_x_m": f"{point.x_m:.9f}",
                            "target_y_m": f"{point.y_m:.9f}",
                            "target_height_m": f"{point.height_m:.9f}",
                            "target_plane1_z_m": f"{-point.height_m:.9f}",
                            "status": "UNREACHABLE",
                            "motion_started_utc": motion_started_utc,
                            "motion_duration_s": f"{time.monotonic() - motion_started:.6f}",
                            "message": str(error),
                        }
                    )
                    results_stream.flush()
                    print(f"UNREACHABLE: {error}", flush=True)
                    continue

                motion_duration_s = time.monotonic() - motion_started
                # Count only frames received during the authoritative static
                # window, never frames observed while MoveIt was moving.
                frames_before = int(capture_session.frames_written)
                window = _capture_robot_window(
                    session=session,
                    point=point,
                    writer=robot_writer,
                    stream=robot_stream,
                    base_to_plane1=base_to_plane1,
                    tool0_to_tip=tool0_to_tip,
                    duration_s=args.capture_s,
                    sample_hz=args.robot_sample_hz,
                )
                frames_after = int(capture_session.frames_written)
                frame_delta = frames_after - frames_before
                if capture_session.capture_error:
                    raise RuntimeError(f"NUCLEO reader failed: {capture_session.capture_error}")
                if frame_delta < max(1, int(args.capture_s * 50.0)):
                    raise RuntimeError(
                        f"NUCLEO liveness failed at {point.point_id}: only {frame_delta} "
                        "complete groups observed during the static capture window"
                    )
                windows.append(window)
                captured_count += 1
                window_writer.writerow(asdict(window))
                windows_stream.flush()
                result_writer.writerow(
                    {
                        "sequence": point.sequence,
                        "point_id": point.point_id,
                        "target_x_m": f"{point.x_m:.9f}",
                        "target_y_m": f"{point.y_m:.9f}",
                        "target_height_m": f"{point.height_m:.9f}",
                        "target_plane1_z_m": f"{-point.height_m:.9f}",
                        "status": "CAPTURED",
                        "motion_started_utc": motion_started_utc,
                        "motion_duration_s": f"{motion_duration_s:.6f}",
                        "motion_translation_m": f"{delta.translation_m:.9f}",
                        "motion_rotation_deg": f"{delta.rotation_deg:.9f}",
                        "capture_start_sync_time_ns": window.start_sync_time_ns,
                        "capture_end_sync_time_ns": window.end_sync_time_ns,
                        "robot_sample_count": window.robot_sample_count,
                        "nucleo_frame_delta": frame_delta,
                        "message": "exact target reached and settled; 3 s static window captured",
                    }
                )
                results_stream.flush()
                print(
                    f"CAPTURED: robot_samples={window.robot_sample_count} "
                    f"nucleo_frame_delta={frame_delta}",
                    flush=True,
                )

        capture_summary = capture_session.stop()
        capture_session = None
        # Leave the board only after moving away from its closest plane and then
        # to the central high park, both through the same exact MoveIt gate.
        if windows:
            last = points[[point.sequence for point in points].index(windows[-1].sequence)]
            session.move_absolute(
                AbsolutePoseCommand(
                    frame="plane1",
                    position_m=(last.x_m, last.y_m, -0.15),
                    rotation_vector_rad=(0.0, 0.0, 0.0),
                )
            )
            session.move_absolute(
                AbsolutePoseCommand(
                    frame="plane1",
                    position_m=(0.0, 0.0, -0.15),
                    rotation_vector_rad=(0.0, 0.0, 0.0),
                )
            )
            park_status = "plane1_center_height_15cm"
    except KeyboardInterrupt:
        fatal_error = "operator interrupted the batch"
        if session is not None:
            session.cancel_active()
    except Exception as error:
        fatal_error = f"{type(error).__name__}: {error}"
        if session is not None:
            session.cancel_active()
    finally:
        if capture_session is not None:
            try:
                capture_summary = capture_session.stop()
            except Exception as error:
                fatal_error = fatal_error or f"NUCLEO stop failed: {type(error).__name__}: {error}"
        if session is not None:
            session.close()
        rclpy.shutdown()

    raw_sensor_path = output / "nucleo_raw_telemetry.csv"
    mapping_summary: dict[str, Any] = {}
    if capture_summary is not None and Path(capture_summary.staging_path).is_file():
        shutil.move(str(capture_summary.staging_path), raw_sensor_path)
        if windows and robot_path.is_file():
            try:
                mapping_summary = map_sensor_windows(
                    raw_sensor_path=raw_sensor_path,
                    robot_path=robot_path,
                    windows=windows,
                    mapped_path=output / "nucleo_mapped.csv",
                    quality_path=output / "sensor_window_quality.csv",
                )
            except Exception as error:
                fatal_error = fatal_error or f"sensor mapping failed: {type(error).__name__}: {error}"

    parser_stats = asdict(capture_summary.parser_stats) if capture_summary is not None else {}
    status = "complete"
    if fatal_error:
        status = "failed"
    elif unreachable_count or any(int(parser_stats.get(name, 0)) for name in (
        "incomplete_groups", "duplicate_packets", "reordered_packets",
        "channel_sequence_drops", "source_sequence_drops", "malformed_packets",
    )):
        status = "complete_with_quality_flags"
    manifest = {
        "schema_version": 1,
        "status": status,
        "error": fatal_error,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "environment_id": environment.environment_id,
        "frame_id": "plane1",
        "orientation": {
            "description": "strict vertical rod-tip orientation in plane1",
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
        "grid": {
            "coordinates_cm": [float(value) for value in args.coordinates_cm],
            "heights_cm": [float(value) for value in args.heights_cm],
            "excluded_interior_cm": args.exclude_interior_cm,
            "plane1_z_sign": "negative",
            "scheduled_points": len(points),
            "first_original_sequence": args.start_sequence,
        },
        "capture": {
            "seconds_per_point": args.capture_s,
            "robot_sample_hz": args.robot_sample_hz,
            "captured_points": captured_count,
            "unreachable_points": unreachable_count,
            "nucleo_sync_mode": "performed_once" if args.perform_initial_sync else "reused_existing",
            "nucleo_frames_total": capture_summary.frames_written if capture_summary is not None else 0,
            "nucleo_capture_error": capture_summary.capture_error if capture_summary is not None else None,
            "nucleo_parser": parser_stats,
        },
        "motion": {
            "method": "MoveIt collision-checked IK + plan-only + exact ExecuteTrajectory",
            "velocity_scale": args.velocity_scale,
            "acceleration_scale": args.acceleration_scale,
            "planning_time_s": args.planning_time_s,
            "planning_attempts": args.planning_attempts,
            "recording_artifacts_used": False,
            "park_status": park_status,
        },
        "mapping": mapping_summary,
    }
    _write_json(output / "manifest.json", manifest)

    artifact_names = (
        "schedule.csv", "point_results.csv", "capture_windows.csv",
        "robot_actual_fk.csv", "nucleo_raw_telemetry.csv", "nucleo_mapped.csv",
        "sensor_window_quality.csv", "manifest.json",
    )
    with (output / "sha256sums.txt").open("w", encoding="utf-8") as stream:
        for name in artifact_names:
            path = output / name
            if path.is_file():
                stream.write(f"{_sha256(path)}  {name}\n")

    print(f"output_directory: {output}", flush=True)
    print(f"status: {status}", flush=True)
    print(f"captured_points: {captured_count}", flush=True)
    print(f"unreachable_points: {unreachable_count}", flush=True)
    if fatal_error:
        print(f"fatal_error: {fatal_error}", file=sys.stderr, flush=True)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
