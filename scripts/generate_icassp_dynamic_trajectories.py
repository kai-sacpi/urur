#!/usr/bin/env python3
"""Generate the ICASSP dynamic-route authority CSVs for the ``mla`` scene.

The paper-side trajectories use a sensor-board frame whose positive ``z`` is
the distance *above* the array.  The MLA ``plane1`` frame has the opposite
local ``z`` direction, hence ``tip_z_m = -z_A`` below.  This file deliberately
does not call a runtime planner: it only writes immutable source waypoints and
their authoritative timestamps.  A differing existing file is an error.

All moving pieces use the quintic minimum-jerk progress
``10*u**3 - 15*u**4 + 6*u**5``.  Every piece starts and ends with zero velocity
and acceleration.  The final three seconds of every authority repeat the last
draw pose with ``phase=hold``.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Callable, Iterable


FRAME_ID = "plane1"
DT_S = 0.1
TOTAL_DURATION_S = 60.0
HOLD_DURATION_S = 3.0
LOOP_MOTION_DURATION_S = 54.0
CONCENTRIC_CENTER_HOLD_S = 5.0
CONCENTRIC_TRANSITION_S = 5.0
CONCENTRIC_INNER_LOOP_S = 15.0
CONCENTRIC_OUTER_LOOP_S = 27.0

TRAJECTORY_IDS = (
    "icassp_dynamic_concentric_z07",
    "icassp_dynamic_concentric_z09",
    "icassp_dynamic_concentric_z11",
    "icassp_dynamic_concentric_z13",
    "icassp_dynamic_concentric_z15",
    "icassp_dynamic_figure8_z11",
    "icassp_dynamic_figure8_z15",
    "icassp_dynamic_rounded_square_z15",
    "icassp_dynamic_space_z11",
    "icassp_dynamic_circle_r15_z11",
)

CSV_COLUMNS = (
    "time_s",
    "frame_id",
    "tip_x_m",
    "tip_y_m",
    "tip_z_m",
    "tip_qx",
    "tip_qy",
    "tip_qz",
    "tip_qw",
    "phase",
)
QUATERNION_XYZW = (0.0, 0.0, 0.0, 1.0)

Point = tuple[float, float, float]
Evaluator = Callable[[float], Point]


def minimum_jerk(u: float) -> float:
    """Quintic minimum-jerk progress on the unit interval."""

    if not math.isfinite(u) or not 0.0 <= u <= 1.0:
        raise ValueError(f"normalized time must be in [0,1], got {u!r}")
    if u in (0.0, 1.0):
        return u
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def _lerp(a: Point, b: Point, u: float) -> Point:
    s = minimum_jerk(u)
    return tuple((1.0 - s) * x + s * y for x, y in zip(a, b, strict=True))  # type: ignore[return-value]


def _loop_parameter(u: float) -> float:
    return 2.0 * math.pi * minimum_jerk(u)


def _paper_xy(kind: str, theta: float, radius_m: float) -> tuple[float, float]:
    """Return the paper-frame x/y geometry used by dynamic_crlb.py."""

    if kind in {"circle", "space"}:
        return radius_m * math.cos(theta), radius_m * math.sin(theta)
    if kind == "figure8":
        return radius_m * math.sin(theta), radius_m * math.sin(2.0 * theta)
    if kind == "rounded_square":
        k = 2.5
        normalizer = math.tanh(k)
        return (
            radius_m * math.tanh(k * math.cos(theta)) / normalizer,
            radius_m * math.tanh(k * math.sin(theta)) / normalizer,
        )
    raise ValueError(f"unknown paper geometry: {kind}")


def _append_segment(
    rows: list[dict[str, object]],
    start_tick: int,
    duration_s: float,
    evaluator: Evaluator,
    *,
    phase: str = "draw",
) -> int:
    """Append a uniformly sampled segment, sharing its first boundary knot."""

    ticks = int(round(duration_s / DT_S))
    if ticks <= 0 or not math.isclose(ticks * DT_S, duration_s, abs_tol=1.0e-12):
        raise ValueError(f"segment duration is not on the {DT_S:g}s grid: {duration_s}")
    for local_tick in range(ticks + 1):
        if rows and local_tick == 0:
            continue
        point = evaluator(local_tick / ticks)
        rows.append(
            {
                "time_s": (start_tick + local_tick) * DT_S,
                "frame_id": FRAME_ID,
                "tip_x_m": point[0],
                "tip_y_m": point[1],
                "tip_z_m": point[2],
                "tip_qx": QUATERNION_XYZW[0],
                "tip_qy": QUATERNION_XYZW[1],
                "tip_qz": QUATERNION_XYZW[2],
                "tip_qw": QUATERNION_XYZW[3],
                "phase": phase,
            }
        )
    return start_tick + ticks


def _append_hold(rows: list[dict[str, object]], end_tick: int) -> None:
    if not rows:
        raise ValueError("cannot append a hold to an empty route")
    last = rows[-1]
    hold_tick = int(round(HOLD_DURATION_S / DT_S))
    if end_tick + hold_tick != int(round(TOTAL_DURATION_S / DT_S)):
        raise ValueError("route pieces do not leave exactly three seconds for the final hold")
    # Keep every hold sample explicit.  This makes the static interval
    # auditable and avoids asking a downstream interpolator to infer a long
    # constant segment from only its two endpoints.
    for offset in range(1, hold_tick + 1):
        rows.append(
            {
                **last,
                "time_s": (end_tick + offset) * DT_S,
                "phase": "hold",
            }
        )


def _concentric_route(height_m: float) -> list[dict[str, object]]:
    """Center -> one inner ring -> one outer ring at a fixed paper height."""

    # Paper z_A is positive above the board; plane1's local z is opposite.
    z = -float(height_m)
    center: Point = (0.0, 0.0, z)
    inner_start: Point = (0.025, 0.0, z)
    outer_start: Point = (0.050, 0.0, z)
    rows: list[dict[str, object]] = []
    tick = 0
    tick = _append_segment(rows, tick, CONCENTRIC_CENTER_HOLD_S, lambda _u: center)
    tick = _append_segment(rows, tick, CONCENTRIC_TRANSITION_S, lambda u: _lerp(center, inner_start, u))
    tick = _append_segment(
        rows,
        tick,
        CONCENTRIC_INNER_LOOP_S,
        lambda u: (*_paper_xy("circle", _loop_parameter(u), 0.025), z),
    )
    tick = _append_segment(rows, tick, CONCENTRIC_TRANSITION_S, lambda u: _lerp(inner_start, outer_start, u))
    tick = _append_segment(
        rows,
        tick,
        CONCENTRIC_OUTER_LOOP_S,
        lambda u: (*_paper_xy("circle", _loop_parameter(u), 0.050), z),
    )
    _append_hold(rows, tick)
    return rows


def _simple_loop_route(kind: str, height_m: float, *, radius_m: float = 0.050) -> list[dict[str, object]]:
    z_a = float(height_m)
    z = -z_a

    def point(u: float) -> Point:
        theta = _loop_parameter(u)
        x, y = _paper_xy(kind, theta, radius_m)
        if kind == "space":
            z_local = -(z_a + 0.040 * math.sin(2.0 * theta))
        else:
            z_local = z
        return x, y, z_local

    rows: list[dict[str, object]] = []
    tick = 0
    # The initial dwell makes the first waypoint explicitly stationary while
    # retaining a draw phase; the final row below is the only hold phase.
    first = point(0.0)
    tick = _append_segment(rows, tick, TOTAL_DURATION_S - LOOP_MOTION_DURATION_S - HOLD_DURATION_S, lambda _u: first)
    tick = _append_segment(rows, tick, LOOP_MOTION_DURATION_S, point)
    _append_hold(rows, tick)
    return rows


def build_rows(trajectory_id: str) -> list[dict[str, object]]:
    if trajectory_id.startswith("icassp_dynamic_concentric_z"):
        height_cm = int(trajectory_id.rsplit("z", 1)[1])
        return _concentric_route(height_cm / 100.0)
    if trajectory_id == "icassp_dynamic_figure8_z11":
        return _simple_loop_route("figure8", 0.11)
    if trajectory_id == "icassp_dynamic_figure8_z15":
        return _simple_loop_route("figure8", 0.15)
    if trajectory_id == "icassp_dynamic_rounded_square_z15":
        return _simple_loop_route("rounded_square", 0.15)
    if trajectory_id == "icassp_dynamic_space_z11":
        return _simple_loop_route("space", 0.11)
    if trajectory_id == "icassp_dynamic_circle_r15_z11":
        return _simple_loop_route("circle", 0.11, radius_m=0.15)
    raise ValueError(f"unknown trajectory id: {trajectory_id}")


def _canonical_csv(rows: Iterable[dict[str, object]]) -> bytes:
    from io import StringIO

    stream = StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=CSV_COLUMNS, lineterminator="\n")
    writer.writeheader()
    for row in rows:
        writer.writerow(
            {
                "time_s": f"{float(row['time_s']):.9f}",
                "frame_id": row["frame_id"],
                "tip_x_m": f"{float(row['tip_x_m']):.9f}",
                "tip_y_m": f"{float(row['tip_y_m']):.9f}",
                "tip_z_m": f"{float(row['tip_z_m']):.9f}",
                "tip_qx": f"{float(row['tip_qx']):.12f}",
                "tip_qy": f"{float(row['tip_qy']):.12f}",
                "tip_qz": f"{float(row['tip_qz']):.12f}",
                "tip_qw": f"{float(row['tip_qw']):.12f}",
                "phase": row["phase"],
            }
        )
    return stream.getvalue().encode("ascii")


def _validate_rows(trajectory_id: str, rows: list[dict[str, object]]) -> dict[str, object]:
    expected_last_tick = int(round(TOTAL_DURATION_S / DT_S))
    if len(rows) < 3:
        raise ValueError(f"{trajectory_id}: too few source rows")
    times = [float(row["time_s"]) for row in rows]
    expected_ticks = list(range(expected_last_tick + 1))
    actual_ticks = [round(t / DT_S) for t in times]
    if actual_ticks != expected_ticks:
        raise ValueError(f"{trajectory_id}: timestamps are not a complete {DT_S:g}s grid")
    if times[0] != 0.0 or times[-1] != TOTAL_DURATION_S:
        raise ValueError(f"{trajectory_id}: invalid endpoint time")
    if rows[-1]["phase"] != "hold":
        raise ValueError(f"{trajectory_id}: final row is not hold")
    for column in ("tip_x_m", "tip_y_m", "tip_z_m", "tip_qx", "tip_qy", "tip_qz", "tip_qw"):
        if not all(math.isfinite(float(row[column])) for row in rows):
            raise ValueError(f"{trajectory_id}: non-finite {column}")
    first_pose = tuple(rows[-2][column] for column in CSV_COLUMNS[1:9])
    final_pose = tuple(rows[-1][column] for column in CSV_COLUMNS[1:9])
    if final_pose != first_pose:
        raise ValueError(f"{trajectory_id}: hold row does not repeat the final draw pose")
    if any(row["frame_id"] != FRAME_ID for row in rows):
        raise ValueError(f"{trajectory_id}: wrong frame id")
    if any(row["phase"] not in {"draw", "hold"} for row in rows):
        raise ValueError(f"{trajectory_id}: invalid phase")
    q_norm = math.sqrt(sum(float(rows[0][f"tip_q{axis}"]) ** 2 for axis in ("x", "y", "z")) + float(rows[0]["tip_qw"]) ** 2)
    if abs(q_norm - 1.0) > 1.0e-12:
        raise ValueError(f"{trajectory_id}: quaternion is not normalized")

    points = [(float(row["tip_x_m"]), float(row["tip_y_m"]), float(row["tip_z_m"])) for row in rows]
    # Finite-difference diagnostics are evidence only; no repair is applied.
    velocities = [
        tuple((b[index] - a[index]) / DT_S for index in range(3))
        for a, b in zip(points, points[1:])
    ]
    accelerations = [
        tuple((a[index] - 2.0 * b[index] + c[index]) / (DT_S * DT_S) for index in range(3))
        for a, b, c in zip(points, points[1:], points[2:])
    ]
    jerks = [
        tuple(
            (a[index] - 3.0 * b[index] + 3.0 * c[index] - d[index]) / (DT_S**3)
            for index in range(3)
        )
        for a, b, c, d in zip(points, points[1:], points[2:], points[3:])
    ]

    # Diagnostics are metadata, not authority values.  Round them before
    # serializing so a rerun with a different CPython/libm minor version does
    # not manufacture a byte-different manifest from identical CSV bytes.
    def rounded(value: float) -> float:
        return float(f"{value:.12g}")

    vector_norm = lambda vector: math.sqrt(sum(value * value for value in vector))
    return {
        "trajectory_id": trajectory_id,
        "source_row_count": len(rows),
        "duration_s": TOTAL_DURATION_S,
        "sample_dt_s": DT_S,
        "minimum_jerk": True,
        "endpoint_velocity_norm_m_s": 0.0,
        "endpoint_acceleration_norm_m_s2": 0.0,
        "max_sampled_position_step_m": rounded(max(math.dist(a, b) for a, b in zip(points, points[1:]))),
        "max_forward_difference_speed_m_s": rounded(max(vector_norm(vector) for vector in velocities)),
        "max_second_difference_acceleration_m_s2": rounded(max(vector_norm(vector) for vector in accelerations)),
        "max_third_difference_jerk_m_s3": rounded(max(vector_norm(vector) for vector in jerks)),
        "tip_x_range_m": [rounded(min(p[0] for p in points)), rounded(max(p[0] for p in points))],
        "tip_y_range_m": [rounded(min(p[1] for p in points)), rounded(max(p[1] for p in points))],
        "tip_z_range_m": [rounded(min(p[2] for p in points)), rounded(max(p[2] for p in points))],
    }


def _write_immutable(path: Path, payload: bytes) -> str:
    digest = hashlib.sha256(payload).hexdigest()
    if path.exists():
        if path.is_symlink() or not path.is_file() or path.read_bytes() != payload:
            raise FileExistsError(f"refusing to overwrite differing authority: {path}")
    else:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("xb") as stream:
            stream.write(payload)
    return digest


def main() -> int:
    repository_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-directory",
        type=Path,
        default=repository_root / "environments" / "mla" / "trajectories",
    )
    parser.add_argument("--manifest", type=Path, default=None)
    args = parser.parse_args()
    output_directory = args.output_directory.expanduser().resolve()
    manifest_path = (args.manifest or output_directory / "icassp_dynamic_authority_manifest.json").expanduser().resolve()

    entries: list[dict[str, object]] = []
    for trajectory_id in TRAJECTORY_IDS:
        rows = build_rows(trajectory_id)
        diagnostics = _validate_rows(trajectory_id, rows)
        payload = _canonical_csv(rows)
        path = output_directory / f"{trajectory_id}.csv"
        digest = _write_immutable(path, payload)
        diagnostics["authority_file"] = path.name
        diagnostics["authority_sha256"] = digest
        diagnostics["frame_id"] = FRAME_ID
        diagnostics["orientation_xyzw"] = list(QUATERNION_XYZW)
        diagnostics["coordinate_mapping"] = "tip_z_m = -z_A; x/y unchanged"
        entries.append(diagnostics)
        print(f"{path}: sha256={digest}")

    manifest = {
        "schema_version": 1,
        "environment_id": "mla",
        "frame_id": FRAME_ID,
        "coordinate_convention": {
            "paper_frame": "sensor-board A frame; z_A positive above sensor plane",
            "authority_frame": "plane1; local +Z points physically downward",
            "mapping": "tip_x_m=x_A, tip_y_m=y_A, tip_z_m=-z_A",
        },
        "orientation_xyzw": list(QUATERNION_XYZW),
        "timing": {
            "sample_dt_s": DT_S,
            "duration_s": TOTAL_DURATION_S,
            "final_hold_duration_s": HOLD_DURATION_S,
            "parameterization": "quintic minimum jerk per dwell/transition/loop segment",
        },
        "authority_policy": "waypoints and time_s are immutable hard constraints; no runtime scaling, retiming, clipping, or point dropping",
        "source_index": "/Users/huaxin/Codes/ICASSP2027/TRAJECTORY_GENERATION_INDEX_20260908.md",
        "trajectories": entries,
    }
    manifest_payload = (json.dumps(manifest, indent=2, sort_keys=False) + "\n").encode("utf-8")
    _write_immutable(manifest_path, manifest_payload)
    print(f"manifest: {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
