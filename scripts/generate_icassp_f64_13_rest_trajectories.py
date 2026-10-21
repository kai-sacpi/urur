#!/usr/bin/env python3
"""Generate boundary-rest execution authorities for the 13 F64 routes.

The paper export is a board-frame *reference* sampled at a sinusoidal/arc
length parameter.  A robot recording starts from a stopped TCP, so the
reference's non-zero initial tilt rate cannot be sent unchanged under the
MLA endpoint-rest contract.  This script is the explicit upstream redesign:
it evaluates the same closed spatial curve and magnetic-direction curve at a
quintic minimum-jerk phase ``s=10u^3-15u^4+6u^5``.  The authority keeps every
paper timestamp, period, spatial envelope, and orientation amplitude while
making both endpoint velocity and acceleration zero.  The runtime still sees
immutable waypoints and times and is not allowed to retime or scale them.

The paper snapshot is never overwritten.  A separate ``icassp_f64_mj_*`` ID
namespace makes the boundary-rest redesign auditable and prevents accidental
mixing with the analysis reference.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from scipy.interpolate import CubicSpline

import generate_icassp_f64_13_trajectories as reference


ID_PREFIX = "icassp_f64_mj"
MANIFEST_NAME = "icassp_f64_13_rest_authority_manifest_v2.json"


def minimum_jerk(u: float) -> float:
    if not math.isfinite(u) or not 0.0 <= u <= 1.0:
        raise ValueError(f"normalized phase must be in [0,1], got {u!r}")
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def _compose_local_x_pi(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    """Compose a fixed local Rx(pi) for the uniformly flipped q realization."""

    x, y, z, w = quaternion
    # q ⊗ (1,0,0,0), in xyzw order.
    result = (w, z, -y, -x)
    norm = math.sqrt(sum(value * value for value in result))
    result = tuple(value / norm for value in result)
    if result[3] < 0.0:
        result = tuple(-value for value in result)
    return tuple(float(value) for value in result)


def tip_quaternion_from_moment(
    qx: float,
    qy: float,
    qz: float,
    *,
    moment_sign: int = 1,
) -> tuple[float, float, float, float]:
    if moment_sign not in (-1, 1):
        raise ValueError("moment_sign must be +1 or -1")
    base_quaternion = reference.tip_quaternion_from_moment(qx, qy, qz)
    if moment_sign == 1:
        return base_quaternion
    # q -> -q reverses the rod axis.  Applying a fixed local Rx(pi) to the
    # nominal realization keeps that reversal continuous through the vertical
    # crossings and uses one sign/roll policy for the whole experiment.
    return _compose_local_x_pi(base_quaternion)


def _finite_array(raw_rows: list[dict[str, str]], field: str) -> np.ndarray:
    values = np.asarray([float(row[field]) for row in raw_rows], dtype=float)
    if values.ndim != 1 or not np.all(np.isfinite(values)):
        raise ValueError(f"source field {field} contains non-finite values")
    return values


def _read_reference_route(path: Path, route: str) -> dict[str, Any]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        expected = (
            "route", "sample", "time_s", "x_m", "vx_m_s", "y_m", "vy_m_s",
            "z_m", "vz_m_s", "qx", "qy", "qz", "ax_m_s2", "ay_m_s2", "az_m_s2",
        )
        if tuple(reader.fieldnames or ()) != expected:
            raise ValueError(f"{path.name}: unexpected paper reference header")
        rows = list(reader)
    if not rows:
        raise ValueError(f"{path.name}: empty paper reference route")
    if any(row["route"] != route for row in rows):
        raise ValueError(f"{path.name}: route column mismatch")
    sample = np.asarray([int(row["sample"]) for row in rows], dtype=int)
    if not np.array_equal(sample, np.arange(len(rows), dtype=int)):
        raise ValueError(f"{path.name}: sample indices are not contiguous")
    times = _finite_array(rows, "time_s")
    if times[0] != 0.0 or np.any(np.diff(times) <= 0.0):
        raise ValueError(f"{path.name}: invalid source times")
    expected_duration = reference.ROUTE_DURATIONS_S[route]
    if times[-1] != expected_duration:
        raise ValueError(f"{path.name}: source period is not {expected_duration:g} s")
    dt = np.diff(times)
    if not np.allclose(dt, reference.SOURCE_DT_S, rtol=0.0, atol=1.0e-12):
        raise ValueError(f"{path.name}: source sampling is not 0.02 s")
    q = np.column_stack([_finite_array(rows, field) for field in ("qx", "qy", "qz")])
    norms = np.linalg.norm(q, axis=1)
    if np.max(np.abs(norms - 1.0)) > 1.0e-6:
        raise ValueError(f"{path.name}: moment directions are not unit vectors")
    return {
        "times": times,
        "x": _finite_array(rows, "x_m"),
        "y": _finite_array(rows, "y_m"),
        "z": _finite_array(rows, "z_m"),
        "q": q / norms[:, None],
        "source_rows": rows,
    }


def _periodic_splines(data: dict[str, Any]) -> tuple[np.ndarray, dict[str, CubicSpline]]:
    times = np.asarray(data["times"], dtype=float)
    phase = times / float(times[-1])
    splines: dict[str, CubicSpline] = {}
    for field in ("x", "y", "z"):
        splines[field] = CubicSpline(phase, np.asarray(data[field], dtype=float), bc_type="periodic")
    q = np.asarray(data["q"], dtype=float)
    for index, field in enumerate(("qx", "qy", "qz")):
        splines[field] = CubicSpline(phase, q[:, index], bc_type="periodic")
    return phase, splines


def _canonical_csv(rows: list[dict[str, Any]]) -> bytes:
    columns = reference.AUTHORITY_COLUMNS
    lines = [",".join(columns)]
    for row in rows:
        lines.append(
            ",".join(
                (
                    f"{float(row['time_s']):.9f}",
                    "plane1",
                    f"{float(row['tip_x_m']):.9f}",
                    f"{float(row['tip_y_m']):.9f}",
                    f"{float(row['tip_z_m']):.9f}",
                    f"{float(row['tip_qx']):.12f}",
                    f"{float(row['tip_qy']):.12f}",
                    f"{float(row['tip_qz']):.12f}",
                    f"{float(row['tip_qw']):.12f}",
                    "draw",
                )
            )
        )
    return ("\n".join(lines) + "\n").encode("ascii")


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_immutable(path: Path, payload: bytes) -> str:
    digest = _sha256(payload)
    if path.exists():
        if path.is_symlink() or not path.is_file() or path.read_bytes() != payload:
            raise FileExistsError(f"refusing to overwrite differing authority: {path}")
    else:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("xb") as stream:
            stream.write(payload)
    return digest


def _build_route(
    source_directory: Path,
    route: str,
    *,
    moment_sign: int,
) -> tuple[bytes, dict[str, Any]]:
    source_path = source_directory / f"{route}.csv"
    data = _read_reference_route(source_path, route)
    phase, splines = _periodic_splines(data)
    source_times = np.asarray(data["times"], dtype=float)
    output_rows: list[dict[str, Any]] = []
    for time_s in source_times:
        u = float(time_s / source_times[-1])
        warped = minimum_jerk(u)
        x = float(splines["x"](warped))
        y = float(splines["y"](warped))
        z_a = float(splines["z"](warped))
        q_values = np.asarray([float(splines[field](warped)) for field in ("qx", "qy", "qz")])
        q_values /= np.linalg.norm(q_values)
        quaternion = tip_quaternion_from_moment(*q_values, moment_sign=moment_sign)
        output_rows.append(
            {
                "time_s": float(time_s),
                "tip_x_m": x,
                "tip_y_m": y,
                "tip_z_m": -z_a,
                "tip_qx": quaternion[0],
                "tip_qy": quaternion[1],
                "tip_qz": quaternion[2],
                "tip_qw": quaternion[3],
            }
        )
    payload = _canonical_csv(output_rows)
    metadata: dict[str, Any] = {
        "source_file": source_path.name,
        "source_sha256": _sha256_file(source_path),
        "authority_sha256": _sha256(payload),
        "source_waypoint_count": len(output_rows),
        "duration_s": float(source_times[-1]),
        "sample_dt_s": reference.SOURCE_DT_S,
        "moment_sign": moment_sign,
        "time_parameterization": "quintic minimum-jerk phase applied to the closed paper curve",
        "phase_policy": "draw_only; no runtime-generated hold or endpoint knot",
        "xy_bounds_m": {
            "x": [min(row["tip_x_m"] for row in output_rows), max(row["tip_x_m"] for row in output_rows)],
            "y": [min(row["tip_y_m"] for row in output_rows), max(row["tip_y_m"] for row in output_rows)],
        },
        "z_A_bounds_m": [
            min(-row["tip_z_m"] for row in output_rows),
            max(-row["tip_z_m"] for row in output_rows),
        ],
    }
    return payload, metadata


def generate(
    source_directory: Path,
    output_directory: Path,
    manifest_path: Path,
    *,
    moment_sign: int = 1,
    id_prefix: str = ID_PREFIX,
) -> tuple[Path, ...]:
    if moment_sign not in (-1, 1):
        raise ValueError("moment_sign must be +1 or -1")
    source_directory = source_directory.expanduser().resolve()
    output_directory = output_directory.expanduser().resolve()
    manifest_path = manifest_path.expanduser().resolve()
    source_manifest_path = source_directory / "manifest.json"
    source_manifest = json.loads(source_manifest_path.read_text(encoding="utf-8"))
    if source_manifest.get("routes") != list(reference.ROUTE_ORDER):
        raise ValueError("source route order differs from the frozen F64 set")
    entries: list[dict[str, Any]] = []
    paths: list[Path] = []
    for order, route in enumerate(reference.ROUTE_ORDER):
        payload, metadata = _build_route(source_directory, route, moment_sign=moment_sign)
        trajectory_id = f"{id_prefix}_{route}"
        path = output_directory / f"{trajectory_id}.csv"
        digest = _write_immutable(path, payload)
        if digest != metadata["authority_sha256"]:
            raise AssertionError("authority digest changed while writing")
        paths.append(path)
        entries.append(
            {
                "order": order,
                "trajectory_id": trajectory_id,
                "source_route": route,
                "authority_file": path.name,
                **metadata,
                "frame_id": "plane1",
                "coordinate_mapping": "tip_x_m=x_A, tip_y_m=y_A, tip_z_m=-z_A",
                "moment_to_rod_axis": (
                    "rod_tip +Z in plane1 = sign*(-qx_A,+qy_A,+qz_A)"
                ),
            }
        )
        print(f"{path}: sha256={digest}")

    manifest = {
        "schema_version": 1,
        "manifest_id": f"{id_prefix}_13_authority",
        "environment_id": "mla",
        "robot_type": "ur5",
        "frame_id": "plane1",
        # Keep the manifest portable between the Mac source checkout and the
        # Ubuntu execution checkout; the content hash below is the authority.
        "source_snapshot": "environments/mla/trajectories/paper_f64_v1_source",
        "source_snapshot_manifest_sha256": _sha256_file(source_manifest_path),
        "route_count": len(entries),
        "routes": [entry["trajectory_id"] for entry in entries],
        "nominal_heights_cm": [9, 13, 15],
        "sensor": "F64 full 8x8 board, 64 triaxial nodes",
        "geometry_and_timing": {
            "line_scan_xy_range_m": [-0.15, 0.15],
            "figure_eight_xy_range_m": [-0.10, 0.10],
            "line_period_s": 12.0,
            "figure_eight_period_s": 16.0,
            "source_sample_dt_s": reference.SOURCE_DT_S,
            "timestamp_policy": "paper timestamps retained exactly",
            "phase_parameterization": "quintic minimum jerk",
        },
        "orientation": {
            "input": "q=(qx,qy,qz) magnetic-moment direction in sensor-board frame",
            "implemented_moment_sign": moment_sign,
            "rod_axis_mapping": "rod_tip +Z in plane1 = sign*(-qx,+qy,+qz)",
            "free_roll": "fixed minimum roll for sign=+1; fixed local Rx(pi) continuation for sign=-1",
        },
        "authority_policy": (
            "source waypoints and time_s are immutable hard constraints; runtime does not "
            "scale, retime, clip, crop, substitute, or drop points"
        ),
        "routes": entries,
    }
    _write_immutable(manifest_path, (json.dumps(manifest, indent=2) + "\n").encode("utf-8"))
    print(f"manifest: {manifest_path}")
    return tuple(paths)


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-directory", type=Path, default=root / "environments/mla/trajectories/paper_f64_v1_source")
    parser.add_argument("--output-directory", type=Path, default=root / "environments/mla/trajectories")
    parser.add_argument("--manifest", type=Path, default=None)
    parser.add_argument("--moment-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--id-prefix", default=ID_PREFIX)
    args = parser.parse_args()
    manifest = args.manifest
    if manifest is None:
        manifest = args.output_directory / (
            MANIFEST_NAME
            if args.id_prefix == ID_PREFIX and args.moment_sign == 1
            else f"{args.id_prefix}_13_authority_manifest.json"
        )
    generate(
        args.source_directory,
        args.output_directory,
        manifest,
        moment_sign=args.moment_sign,
        id_prefix=args.id_prefix,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
