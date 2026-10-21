#!/usr/bin/env python3
"""Generate the v2 feasible ICASSP F64 authority set.

The paper-side F64 snapshot contains the Cartesian reference curves and a
12/16-second sample grid.  This generator makes one explicit upstream
minimum-jerk phase redesign (the resulting CSV is the authority; runtime
never changes it) and applies the reachable-side rod registration used by
the MLA environment.

Three nominally vertical high-height scans were not feasible with a fixed
identity orientation under the calibrated UR5 limits.  Their Cartesian
positions and timestamps are retained exactly, while their orientation is
replaced by a smooth 25-percent version of the already-feasible same-axis
tilt waveform.  The resulting rod-axis tilt is 3.75--11.25 degrees, so the
condition remains a near-vertical translation control.  All other routes use
the v1 reachable-side smooth tilt policy.

No runtime scaling, retiming, clipping, cropping, waypoint substitution, or
point dropping is involved.  Existing v1 authorities are never overwritten;
the v2 prefix and manifest are immutable and independently hash-bound.
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
import generate_icassp_f64_13_feasible_trajectories as v1


ID_PREFIX = "icassp_f64_mj_feasible_v2"
MANIFEST_NAME = "icassp_f64_13_feasible_v2_authority_manifest.json"
VERTICAL_FALLBACKS = {
    "line_x_vertical_z15": "line_x_tilt90_z15",
    "line_y_vertical_z13": "line_y_tilt90_z13",
    "line_y_vertical_z15": "line_y_tilt90_z15",
}
VERTICAL_ORIENTATION_SCALE = 0.25


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


def _splines(data: dict[str, Any]) -> tuple[dict[str, CubicSpline], dict[str, CubicSpline]]:
    times = np.asarray(data["times"], dtype=float)
    phase = times / float(times[-1])
    position = {
        field: CubicSpline(phase, np.asarray(data[field], dtype=float), bc_type="periodic")
        for field in ("x", "y", "z")
    }
    orientation = {
        field: CubicSpline(phase, np.asarray(data["q"])[:, index], bc_type="periodic")
        for index, field in enumerate(("qx", "qy", "qz"))
    }
    return position, orientation


def _scaled_axis(moment: np.ndarray, route: str, scale: float) -> np.ndarray:
    """Scale a reachable-side signed tilt while preserving its axis plane."""

    axis = v1._biased_axis(moment, route)
    if route.startswith("line_x_tilt") or route.startswith("eight_tilt"):
        alpha = math.atan2(float(axis[0]), float(axis[2])) * scale
        return np.asarray((math.sin(alpha), 0.0, math.cos(alpha)), dtype=float)
    if route.startswith("line_y_tilt"):
        beta = math.atan2(float(axis[1]), float(axis[2])) * scale
        return np.asarray((0.0, math.sin(beta), math.cos(beta)), dtype=float)
    raise ValueError(f"cannot scale tilt for route {route}")


def _canonical_csv(rows: list[dict[str, Any]]) -> bytes:
    return reference._canonical_authority(rows)


def _build_route(
    source_directory: Path,
    route: str,
    source_cache: dict[str, dict[str, Any]],
) -> tuple[bytes, dict[str, Any]]:
    data = source_cache.setdefault(
        route,
        v1._read_route(source_directory / f"{route}.csv", route),
    )
    position_splines, orientation_splines = _splines(data)
    fallback_route = VERTICAL_FALLBACKS.get(route)
    fallback_orientation_splines = None
    if fallback_route is not None:
        fallback_data = source_cache.setdefault(
            fallback_route,
            v1._read_route(source_directory / f"{fallback_route}.csv", fallback_route),
        )
        _, fallback_orientation_splines = _splines(fallback_data)

    times = np.asarray(data["times"], dtype=float)
    tilt_values: list[float] = []
    rows: list[dict[str, Any]] = []
    for time_s in times:
        warped = v1.minimum_jerk(float(time_s / times[-1]))
        if fallback_orientation_splines is not None:
            moment = np.asarray(
                [
                    float(fallback_orientation_splines[field](warped))
                    for field in ("qx", "qy", "qz")
                ],
                dtype=float,
            )
            moment /= np.linalg.norm(moment)
            axis = _scaled_axis(moment, fallback_route or route, VERTICAL_ORIENTATION_SCALE)
            policy = "near_vertical_scaled_feasible_tilt"
            mapping = "fallback rod-axis tilt = 0.25 * v1 signed tilt"
        else:
            moment = np.asarray(
                [float(orientation_splines[field](warped)) for field in ("qx", "qy", "qz")],
                dtype=float,
            )
            moment /= np.linalg.norm(moment)
            axis = v1._biased_axis(moment, route)
            policy = "reachable_side_smooth_bias"
            mapping = "alpha_prime=-30deg+alpha/3; signed rod-axis angle"
        tilt_values.append(math.degrees(math.atan2(math.hypot(axis[0], axis[1]), axis[2])))
        quaternion = reference.tip_quaternion_from_moment(-axis[0], axis[1], axis[2])
        rows.append(
            {
                "time_s": float(time_s),
                "frame_id": "plane1",
                "tip_x_m": float(position_splines["x"](warped)),
                "tip_y_m": float(position_splines["y"](warped)),
                "tip_z_m": -float(position_splines["z"](warped)),
                "tip_qx": quaternion[0],
                "tip_qy": quaternion[1],
                "tip_qz": quaternion[2],
                "tip_qw": quaternion[3],
                "phase": "draw",
            }
        )

    source_path = source_directory / f"{route}.csv"
    metadata = {
        "source_file": source_path.name,
        "source_sha256": _sha256_file(source_path),
        "source_waypoint_count": len(rows),
        "duration_s": float(times[-1]),
        "sample_dt_s": reference.SOURCE_DT_S,
        "orientation_policy": policy,
        "tilt_mapping": mapping,
        "tilt_range_deg": [float(min(tilt_values)), float(max(tilt_values))],
        "frame_id": "plane1",
        "coordinate_mapping": "tip_x_m=x_A, tip_y_m=y_A, tip_z_m=-z_A",
        "upstream_phase_redesign": "minimum_jerk quintic phase evaluated on the fixed source time grid",
    }
    if fallback_route is not None:
        metadata["fallback_from_route"] = fallback_route
        metadata["orientation_scale"] = VERTICAL_ORIENTATION_SCALE
    payload = _canonical_csv(rows)
    metadata["authority_sha256"] = hashlib.sha256(payload).hexdigest()
    return payload, metadata


def generate(source_directory: Path, output_directory: Path, manifest_path: Path) -> tuple[Path, ...]:
    source_directory = source_directory.expanduser().resolve()
    output_directory = output_directory.expanduser().resolve()
    manifest_path = manifest_path.expanduser().resolve()
    source_manifest = json.loads((source_directory / "manifest.json").read_text(encoding="utf-8"))
    if source_manifest.get("routes") != list(reference.ROUTE_ORDER):
        raise ValueError("source route order differs from frozen F64 set")
    entries: list[dict[str, Any]] = []
    paths: list[Path] = []
    cache: dict[str, dict[str, Any]] = {}
    for order, route in enumerate(reference.ROUTE_ORDER):
        payload, metadata = _build_route(source_directory, route, cache)
        trajectory_id = f"{ID_PREFIX}_{route}"
        path = output_directory / f"{trajectory_id}.csv"
        digest = _write_immutable(path, payload)
        if digest != metadata["authority_sha256"]:
            raise AssertionError("authority digest changed while writing")
        paths.append(path)
        entries.append(
            {
                "order": order,
                "trajectory_id": trajectory_id,
                "authority_file": path.name,
                **metadata,
            }
        )
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
            "timestamp_policy": "fixed source time grid retained exactly",
            "pose_policy": "Cartesian samples are an explicit upstream minimum-jerk redesign; the emitted CSV is authoritative",
            "phase_parameterization": "quintic minimum jerk",
        },
        "orientation": {
            "input": "q=(qx,qy,qz) magnetic-moment direction in sensor-board frame",
            "registration": "rod_tip +Z in plane1=(-qx,+qy,+qz)",
            "policy": "reachable-side smooth bias; three difficult vertical scans use a 0.25-scaled same-axis tilt waveform",
            "fallback_routes": sorted(VERTICAL_FALLBACKS),
            "fallback_orientation_scale": VERTICAL_ORIENTATION_SCALE,
            "tilt_range_deg": [3.75, 45.0],
            "runtime_orientation_changes": False,
        },
        "authority_policy": (
            "emitted CSV waypoints and time_s are immutable hard constraints; runtime does not "
            "scale, retime, clip, crop, substitute, or drop points"
        ),
    }
    _write_immutable(manifest_path, (json.dumps(manifest, indent=2) + "\n").encode("utf-8"))
    print(f"manifest: {manifest_path}")
    return tuple(paths)


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-directory",
        type=Path,
        default=root / "environments/mla/trajectories/paper_f64_v1_source",
    )
    parser.add_argument("--output-directory", type=Path, default=root / "environments/mla/trajectories")
    parser.add_argument("--manifest", type=Path, default=None)
    args = parser.parse_args()
    manifest = args.manifest or args.output_directory / MANIFEST_NAME
    generate(args.source_directory, args.output_directory, manifest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
