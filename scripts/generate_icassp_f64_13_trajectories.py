#!/usr/bin/env python3
"""Convert the paper-side F64 dynamic routes into immutable UR5 authorities.

The files in ``paper_f64_v1_source`` are deliberately kept as a verbatim
snapshot of the paper-side export.  Their ``q*`` columns are a unit magnetic
moment direction in the sensor-board frame, *not* a ROS quaternion.  This
bridge applies the frozen MLA registration used by the static experiments:

    rod_tip +Z in ``plane1`` = (-q_x, +q_y, +q_z)

and emits a minimum-roll quaternion whose local +Z axis is that vector.  The
paper frame has +Z above the board while ``plane1`` has +Z into the board, so
``tip_z_m = -z_m``.  Source timestamps, positions, and sample count are
preserved exactly; no scaling, retiming, clipping, or point dropping is done.

The source trajectories are closed 12 s/16 s cycles and have no explicit hold
segment.  Every emitted row is therefore ``draw``; the endpoint is still a
closed pose and the runtime's canonical endpoint-rest contract is checked
during compilation.  A failed route is never silently altered by this script.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from decimal import Decimal, InvalidOperation
from pathlib import Path
from typing import Any, Iterable


FRAME_ID = "plane1"
SOURCE_COLUMNS = (
    "route",
    "sample",
    "time_s",
    "x_m",
    "vx_m_s",
    "y_m",
    "vy_m_s",
    "z_m",
    "vz_m_s",
    "qx",
    "qy",
    "qz",
    "ax_m_s2",
    "ay_m_s2",
    "az_m_s2",
)
AUTHORITY_COLUMNS = (
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

ROUTE_ORDER = (
    "line_x_vertical_z09",
    "line_x_tilt90_z09",
    "line_y_vertical_z09",
    "line_y_tilt90_z09",
    "line_x_vertical_z13",
    "line_x_tilt90_z13",
    "line_y_vertical_z13",
    "line_y_tilt90_z13",
    "line_x_vertical_z15",
    "line_x_tilt90_z15",
    "line_y_vertical_z15",
    "line_y_tilt90_z15",
    "eight_tilt90_z13",
)
ROUTE_DURATIONS_S = {name: (16.0 if name.startswith("eight_") else 12.0) for name in ROUTE_ORDER}
SOURCE_DT_S = 0.02
_NANOSECONDS = Decimal(1_000_000_000)


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _finite(raw: str, field: str) -> float:
    try:
        value = float(raw)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{field} must be numeric") from error
    if not math.isfinite(value):
        raise ValueError(f"{field} must be finite")
    return value


def _time(raw: str, row_number: int) -> tuple[Decimal, float]:
    try:
        value = Decimal(raw)
    except (InvalidOperation, TypeError) as error:
        raise ValueError(f"time_s at row {row_number} is not a decimal") from error
    if not value.is_finite() or value < 0:
        raise ValueError(f"time_s at row {row_number} must be finite and non-negative")
    nanos = value * _NANOSECONDS
    if nanos != nanos.to_integral_value():
        raise ValueError(f"time_s at row {row_number} is not an exact ROS timestamp")
    return value, float(value)


def tip_quaternion_from_moment(qx: float, qy: float, qz: float) -> tuple[float, float, float, float]:
    """Map a board-frame moment direction to the frozen MLA rod-tip pose.

    The installed magnet/rod registration has two Rx(pi) factors.  Thus the
    rod +Z axis in ``plane1`` is ``(-qx, qy, qz)``.  The returned quaternion is
    the shortest/minimum-roll rotation from +Z to that axis, with a canonical
    non-negative scalar component for byte-stable output.
    """

    norm = math.sqrt(qx * qx + qy * qy + qz * qz)
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("moment direction must be finite and non-zero")
    qx, qy, qz = qx / norm, qy / norm, qz / norm
    # The static MLA authority is defined only for the +Z hemisphere.  The
    # supplied F64 routes satisfy qz >= cos(45 deg), but keep this guard here
    # so an accidental opposite-pole input fails closed.
    if qz < -1.0 + 1.0e-12:
        raise ValueError("180-degree moment reversal has no unique zero-roll pose")
    transverse = math.hypot(qx, qy)
    if transverse <= 1.0e-14:
        return (0.0, 0.0, 0.0, 1.0)
    half_sine = math.sqrt(max(0.0, (1.0 - qz) / 2.0))
    scale = half_sine / transverse
    # e_z x (-qx, qy, qz) = (-qy, -qx, 0).
    quat = (-qy * scale, -qx * scale, 0.0, math.sqrt(max(0.0, (1.0 + qz) / 2.0)))
    norm_q = math.sqrt(sum(value * value for value in quat))
    quat = tuple(value / norm_q for value in quat)
    # Keep the sign deterministic.  This does not change the physical pose.
    if quat[3] < 0.0:
        quat = tuple(-value for value in quat)
    return tuple(float(value) for value in quat)


def _canonical_authority(rows: Iterable[dict[str, Any]]) -> bytes:
    lines: list[str] = [",".join(AUTHORITY_COLUMNS)]
    for row in rows:
        lines.append(
            ",".join(
                (
                    f"{float(row['time_s']):.9f}",
                    str(row["frame_id"]),
                    f"{float(row['tip_x_m']):.9f}",
                    f"{float(row['tip_y_m']):.9f}",
                    f"{float(row['tip_z_m']):.9f}",
                    f"{float(row['tip_qx']):.12f}",
                    f"{float(row['tip_qy']):.12f}",
                    f"{float(row['tip_qz']):.12f}",
                    f"{float(row['tip_qw']):.12f}",
                    str(row["phase"]),
                )
            )
        )
    return ("\n".join(lines) + "\n").encode("ascii")


def _write_immutable(path: Path, payload: bytes) -> str:
    digest = sha256_bytes(payload)
    if path.exists():
        if path.is_symlink() or not path.is_file() or path.read_bytes() != payload:
            raise FileExistsError(f"refusing to overwrite differing authority: {path}")
    else:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("xb") as stream:
            stream.write(payload)
    return digest


def _read_route(path: Path, expected_route: str) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    if path.is_symlink() or not path.is_file():
        raise FileNotFoundError(f"source route is missing: {path}")
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != SOURCE_COLUMNS:
            raise ValueError(f"{path.name}: source header does not match the F64 schema")
        source_rows = list(reader)
    if not source_rows:
        raise ValueError(f"{path.name}: source route is empty")

    rows: list[dict[str, Any]] = []
    previous_time: Decimal | None = None
    expected_count = int(round(ROUTE_DURATIONS_S[expected_route] / SOURCE_DT_S)) + 1
    for row_number, source in enumerate(source_rows, start=2):
        if source.get("route") != expected_route:
            raise ValueError(f"{path.name}: row {row_number} route field is inconsistent")
        try:
            sample = int(source["sample"])
        except (TypeError, ValueError) as error:
            raise ValueError(f"{path.name}: invalid sample index at row {row_number}") from error
        if sample != row_number - 2:
            raise ValueError(f"{path.name}: sample indices are not contiguous")
        decimal_time, time_s = _time(source["time_s"], row_number)
        if previous_time is not None and decimal_time <= previous_time:
            raise ValueError(f"{path.name}: time_s is not strictly increasing")
        previous_time = decimal_time
        x = _finite(source["x_m"], "x_m")
        y = _finite(source["y_m"], "y_m")
        z_a = _finite(source["z_m"], "z_m")
        qx = _finite(source["qx"], "qx")
        qy = _finite(source["qy"], "qy")
        qz = _finite(source["qz"], "qz")
        q_norm = math.sqrt(qx * qx + qy * qy + qz * qz)
        if abs(q_norm - 1.0) > 1.0e-6:
            raise ValueError(f"{path.name}: q is not a unit direction at row {row_number}")
        quaternion = tip_quaternion_from_moment(qx, qy, qz)
        rows.append(
            {
                "time_s": time_s,
                "frame_id": FRAME_ID,
                "tip_x_m": x,
                "tip_y_m": y,
                "tip_z_m": -z_a,
                "tip_qx": quaternion[0],
                "tip_qy": quaternion[1],
                "tip_qz": quaternion[2],
                "tip_qw": quaternion[3],
                "phase": "draw",
            }
        )
    if len(rows) != expected_count:
        raise ValueError(f"{path.name}: expected {expected_count} rows, got {len(rows)}")
    if rows[0]["time_s"] != 0.0 or rows[-1]["time_s"] != ROUTE_DURATIONS_S[expected_route]:
        raise ValueError(f"{path.name}: endpoint time does not match the declared period")
    for field in ("tip_x_m", "tip_y_m", "tip_z_m", "tip_qx", "tip_qy", "tip_qz", "tip_qw"):
        if abs(float(rows[0][field]) - float(rows[-1][field])) > 2.0e-8:
            raise ValueError(f"{path.name}: closed route endpoint mismatch in {field}")
    payload = _canonical_authority(rows)
    metadata = {
        "source_file": path.name,
        "source_sha256": sha256_file(path),
        "source_waypoint_count": len(rows),
        "duration_s": ROUTE_DURATIONS_S[expected_route],
        "sample_dt_s": SOURCE_DT_S,
        "authority_sha256": sha256_bytes(payload),
        "xy_bounds_m": {
            "x": [min(float(row["tip_x_m"]) for row in rows), max(float(row["tip_x_m"]) for row in rows)],
            "y": [min(float(row["tip_y_m"]) for row in rows), max(float(row["tip_y_m"]) for row in rows)],
        },
        "z_A_bounds_m": [
            min(-float(row["tip_z_m"]) for row in rows),
            max(-float(row["tip_z_m"]) for row in rows),
        ],
        "phase_policy": "draw_only; source is a closed cycle with no added hold knots",
    }
    return rows, {"payload": payload, **metadata}


def generate(
    source_directory: Path,
    output_directory: Path,
    manifest_path: Path,
) -> tuple[Path, ...]:
    source_directory = source_directory.expanduser().resolve()
    output_directory = output_directory.expanduser().resolve()
    manifest_path = manifest_path.expanduser().resolve()
    source_manifest_path = source_directory / "manifest.json"
    if not source_manifest_path.is_file():
        raise FileNotFoundError(f"source manifest is missing: {source_manifest_path}")
    source_manifest = json.loads(source_manifest_path.read_text(encoding="utf-8"))
    if source_manifest.get("routes") != list(ROUTE_ORDER):
        raise ValueError("source manifest route order differs from the frozen 13-route set")
    if source_manifest.get("coordinate_frame") != "sensor_board_center; z=0 at sensor plane":
        raise ValueError("source manifest coordinate frame is not the expected board frame")
    paths: list[Path] = []
    entries: list[dict[str, Any]] = []
    for order, route in enumerate(ROUTE_ORDER):
        source_path = source_directory / f"{route}.csv"
        rows, metadata = _read_route(source_path, route)
        route_id = f"icassp_f64_{route}"
        authority_path = output_directory / f"{route_id}.csv"
        authority_sha = _write_immutable(authority_path, metadata["payload"])
        if authority_sha != metadata["authority_sha256"]:
            raise AssertionError("authority digest changed while writing")
        paths.append(authority_path)
        entries.append(
            {
                "order": order,
                "trajectory_id": route_id,
                "source_route": route,
                "authority_file": authority_path.name,
                "authority_sha256": authority_sha,
                "source_file": metadata["source_file"],
                "source_sha256": metadata["source_sha256"],
                "source_waypoint_count": metadata["source_waypoint_count"],
                "duration_s": metadata["duration_s"],
                "sample_dt_s": metadata["sample_dt_s"],
                "frame_id": FRAME_ID,
                "coordinate_mapping": "tip_x_m=x_A, tip_y_m=y_A, tip_z_m=-z_A",
                "moment_to_rod_axis": "rod_tip +Z_plane1=(-qx_A,+qy_A,+qz_A)",
                "phase_policy": metadata["phase_policy"],
                "xy_bounds_m": metadata["xy_bounds_m"],
                "z_A_bounds_m": metadata["z_A_bounds_m"],
            }
        )
        print(f"{authority_path}: sha256={authority_sha}")

    manifest = {
        "schema_version": 1,
        "manifest_id": "icassp_f64_13_authority",
        "environment_id": "mla",
        "robot_type": "ur5",
        "frame_id": FRAME_ID,
        "source_snapshot": str(source_directory),
        "source_snapshot_manifest_sha256": sha256_file(source_manifest_path),
        "route_count": len(entries),
        "routes": [entry["trajectory_id"] for entry in entries],
        "nominal_heights_cm": [9, 13, 15],
        "sensor": "F64 full 8x8 board, 64 triaxial nodes",
        "timing": {
            "source_sample_dt_s": SOURCE_DT_S,
            "motion_periods_s": {"line_scans": 12.0, "figure_eight": 16.0},
            "source_timestamps_preserved": True,
            "authority_phase_policy": "draw_only; no synthetic hold or boundary waypoint added",
        },
        "orientation": {
            "input": "q=(qx,qy,qz) magnetic-moment direction in sensor-board frame",
            "output": "minimum-roll tip quaternion xyzw",
            "rod_axis_mapping": "rod_tip +Z in plane1 = (-qx,+qy,+qz)",
            "free_roll": "zero/minimum roll, fixed throughout each source route",
        },
        "authority_policy": (
            "source waypoints and time_s are immutable hard constraints; runtime does not "
            "scale, retime, clip, crop, substitute, or drop points"
        ),
        "routes": entries,
    }
    manifest_payload = (json.dumps(manifest, indent=2) + "\n").encode("utf-8")
    _write_immutable(manifest_path, manifest_payload)
    print(f"manifest: {manifest_path}")
    return tuple(paths)


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-directory",
        type=Path,
        default=root / "environments" / "mla" / "trajectories" / "paper_f64_v1_source",
    )
    parser.add_argument(
        "--output-directory",
        type=Path,
        default=root / "environments" / "mla" / "trajectories",
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=root / "environments" / "mla" / "trajectories" / "icassp_f64_13_authority_manifest.json",
    )
    args = parser.parse_args()
    generate(args.source_directory, args.output_directory, args.manifest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
