#!/usr/bin/env python3
"""Generate the F64 13-route authority set with an upstream feasible tilt policy.

The frozen paper snapshot and all Cartesian samples/timestamps are retained.
Only the difficult bidirectional tilt realization is redesigned upstream.  For
the X/Y tilt routes (and the figure-eight, whose paper moment varies in X), the
signed rod-axis angle is mapped as::

    alpha' = -30 deg + alpha / 3

Thus the rod stays on the installed, reachable side while varying smoothly
from 15 to 45 degrees.  This avoids the runtime changing a waypoint, time,
or branch: the generated CSV itself is the authority and the runtime remains
fail-closed.
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


ID_PREFIX = "icassp_f64_mj_feasible"
MANIFEST_NAME = "icassp_f64_13_feasible_authority_manifest.json"
BIAS_RAD = math.radians(-30.0)
ANGLE_SCALE = 1.0 / 3.0


def minimum_jerk(u: float) -> float:
    if not math.isfinite(u) or not 0.0 <= u <= 1.0:
        raise ValueError(f"normalized phase must be in [0,1], got {u!r}")
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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


def _finite_array(rows: list[dict[str, str]], field: str) -> np.ndarray:
    values = np.asarray([float(row[field]) for row in rows], dtype=float)
    if values.ndim != 1 or not np.all(np.isfinite(values)):
        raise ValueError(f"source field {field} contains non-finite values")
    return values


def _read_route(path: Path, route: str) -> dict[str, Any]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        expected = (
            "route", "sample", "time_s", "x_m", "vx_m_s", "y_m", "vy_m_s",
            "z_m", "vz_m_s", "qx", "qy", "qz", "ax_m_s2", "ay_m_s2", "az_m_s2",
        )
        if tuple(reader.fieldnames or ()) != expected:
            raise ValueError(f"{path}: unexpected source header")
        rows = list(reader)
    if not rows or any(row["route"] != route for row in rows):
        raise ValueError(f"{path}: invalid route rows")
    sample = np.asarray([int(row["sample"]) for row in rows], dtype=int)
    if not np.array_equal(sample, np.arange(len(rows), dtype=int)):
        raise ValueError(f"{path}: samples are not contiguous")
    times = _finite_array(rows, "time_s")
    if times[0] != 0.0 or np.any(np.diff(times) <= 0.0):
        raise ValueError(f"{path}: invalid timestamps")
    expected_duration = reference.ROUTE_DURATIONS_S[route]
    if times[-1] != expected_duration:
        raise ValueError(f"{path}: duration is not {expected_duration:g} s")
    if not np.allclose(np.diff(times), reference.SOURCE_DT_S, rtol=0.0, atol=1e-12):
        raise ValueError(f"{path}: sampling is not 0.02 s")
    q = np.column_stack([_finite_array(rows, field) for field in ("qx", "qy", "qz")])
    norms = np.linalg.norm(q, axis=1)
    if np.max(np.abs(norms - 1.0)) > 1e-6:
        raise ValueError(f"{path}: moment directions are not unit vectors")
    return {
        "times": times,
        "x": _finite_array(rows, "x_m"),
        "y": _finite_array(rows, "y_m"),
        "z": _finite_array(rows, "z_m"),
        "q": q / norms[:, None],
    }


def _biased_axis(moment: np.ndarray, route: str) -> np.ndarray:
    """Return a smooth reachable-side rod axis for one source sample."""

    qx, qy, qz = (float(value) for value in moment)
    axis = np.asarray((-qx, qy, qz), dtype=float)
    axis /= np.linalg.norm(axis)
    if route.startswith("line_x_tilt") or route.startswith("eight_tilt"):
        alpha = math.atan2(float(axis[0]), float(axis[2]))
        alpha = BIAS_RAD + ANGLE_SCALE * alpha
        return np.asarray((math.sin(alpha), 0.0, math.cos(alpha)), dtype=float)
    if route.startswith("line_y_tilt"):
        beta = math.atan2(float(axis[1]), float(axis[2]))
        beta = BIAS_RAD + ANGLE_SCALE * beta
        return np.asarray((0.0, math.sin(beta), math.cos(beta)), dtype=float)
    return axis


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


def _build_route(source_directory: Path, route: str) -> tuple[bytes, dict[str, Any]]:
    data = _read_route(source_directory / f"{route}.csv", route)
    times = np.asarray(data["times"], dtype=float)
    phase = times / float(times[-1])
    splines = {
        field: CubicSpline(phase, np.asarray(data[field], dtype=float), bc_type="periodic")
        for field in ("x", "y", "z")
    }
    q_splines = {
        field: CubicSpline(phase, np.asarray(data["q"])[:, index], bc_type="periodic")
        for index, field in enumerate(("qx", "qy", "qz"))
    }
    output: list[dict[str, Any]] = []
    tilt_values: list[float] = []
    for time_s in times:
        warped = minimum_jerk(float(time_s / times[-1]))
        moment = np.asarray([float(q_splines[field](warped)) for field in ("qx", "qy", "qz")])
        moment /= np.linalg.norm(moment)
        axis = _biased_axis(moment, route)
        tilt_values.append(math.degrees(math.atan2(math.hypot(axis[0], axis[1]), axis[2])))
        # Invert the frozen MLA registration axis=(-qx,+qy,+qz).
        quaternion = reference.tip_quaternion_from_moment(-axis[0], axis[1], axis[2])
        output.append(
            {
                "time_s": float(time_s),
                "tip_x_m": float(splines["x"](warped)),
                "tip_y_m": float(splines["y"](warped)),
                "tip_z_m": -float(splines["z"](warped)),
                "tip_qx": quaternion[0],
                "tip_qy": quaternion[1],
                "tip_qz": quaternion[2],
                "tip_qw": quaternion[3],
            }
        )
    payload = _canonical_csv(output)
    return payload, {
        "source_file": f"{route}.csv",
        "source_sha256": _sha256_file(source_directory / f"{route}.csv"),
        "authority_sha256": hashlib.sha256(payload).hexdigest(),
        "source_waypoint_count": len(output),
        "duration_s": float(times[-1]),
        "sample_dt_s": reference.SOURCE_DT_S,
        "orientation_policy": "reachable_side_smooth_bias",
        "tilt_mapping": "alpha_prime=-30deg+alpha/3; signed rod-axis angle",
        "tilt_range_deg": [float(min(tilt_values)), float(max(tilt_values))],
        "frame_id": "plane1",
        "coordinate_mapping": "tip_x_m=x_A, tip_y_m=y_A, tip_z_m=-z_A",
    }


def generate(source_directory: Path, output_directory: Path, manifest_path: Path) -> tuple[Path, ...]:
    source_directory = source_directory.expanduser().resolve()
    output_directory = output_directory.expanduser().resolve()
    manifest_path = manifest_path.expanduser().resolve()
    source_manifest = json.loads((source_directory / "manifest.json").read_text(encoding="utf-8"))
    if source_manifest.get("routes") != list(reference.ROUTE_ORDER):
        raise ValueError("source route order differs from frozen F64 set")
    entries: list[dict[str, Any]] = []
    paths: list[Path] = []
    for order, route in enumerate(reference.ROUTE_ORDER):
        payload, metadata = _build_route(source_directory, route)
        trajectory_id = f"{ID_PREFIX}_{route}"
        path = output_directory / f"{trajectory_id}.csv"
        digest = _write_immutable(path, payload)
        if digest != metadata["authority_sha256"]:
            raise AssertionError("authority digest changed while writing")
        paths.append(path)
        entries.append({"order": order, "trajectory_id": trajectory_id, **metadata, "authority_file": path.name})
        print(f"{path}: sha256={digest}")
    manifest = {
        "schema_version": 1,
        "manifest_id": f"{ID_PREFIX}_13_authority",
        "environment_id": "mla",
        "robot_type": "ur5",
        "frame_id": "plane1",
        "source_snapshot": "environments/mla/trajectories/paper_f64_v1_source",
        "source_snapshot_manifest_sha256": _sha256_file(source_directory / "manifest.json"),
        "route_count": len(entries),
        "routes": entries,
        "nominal_heights_cm": [9, 13, 15],
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
            "registration": "rod_tip +Z in plane1=(-qx,+qy,+qz)",
            "policy": "upstream reachable-side smooth bias; no runtime orientation changes",
            "formula": "signed tilt alpha_prime=-30deg+alpha/3",
            "range_deg": [15.0, 45.0],
        },
        "authority_policy": (
            "source waypoints and time_s are immutable hard constraints; runtime does not "
            "scale, retime, clip, crop, substitute, or drop points"
        ),
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
    args = parser.parse_args()
    manifest = args.manifest or args.output_directory / MANIFEST_NAME
    generate(args.source_directory, args.output_directory, manifest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
