#!/usr/bin/env python3
"""Generate the immutable two-site Z-stack + tilt calibration for ``mla_short``.

The distances in this experiment are physical distances from the sensor-board
plane.  ``mla_short`` uses the verified convention that local ``-Z`` points
away from the board, so a distance ``d`` is written as ``sensor_z - d`` in
the plane1 frame.  No timing or spatial post-processing is performed here.
"""

from __future__ import annotations

import argparse
import csv
import math
from datetime import datetime, timezone
from pathlib import Path

import yaml

import hx_waypoint_tools.z_stack_calibration as zstack
from hx_waypoint_tools.yaw_calibration import write_authoritative_trajectory


def _read_geometry(environment: Path) -> tuple[float, float, float, float]:
    env_data = yaml.safe_load((environment / "environment.yaml").read_text())
    geometry = env_data["calibration"]["sensor_geometry"]
    sensor_z = float(geometry["actual_sensor_plane_local_z_m"])
    keepout_min = float(geometry["keepout_local_z_min_m"])
    zones = yaml.safe_load((environment / "zones.yaml").read_text())
    boundary = zones["analysis_zones"]["sensor_keepout_clearance_boundary"]
    boundary_z = float(boundary["normal_xyz"][2]) * float(boundary["offset_m"])
    if not math.isclose(float(boundary["normal_xyz"][2]), -1.0, abs_tol=1e-12):
        raise ValueError("mla_short calibration requires a local -Z clearance boundary")
    minimum_distance = sensor_z - boundary_z
    return sensor_z, keepout_min, boundary_z, minimum_distance


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--environment", type=Path, required=True)
    parser.add_argument("--prefix", default=None)
    args = parser.parse_args()
    environment = args.environment.expanduser().resolve()
    if not environment.is_dir():
        raise SystemExit(f"environment does not exist: {environment}")

    sensor_z, keepout_min, boundary_z, minimum_distance = _read_geometry(environment)
    # Adapt the reusable authority builder to the selected environment without
    # changing its immutable trajectory construction or timing contract.
    zstack.Z_STACK_ENVIRONMENT_ID = environment.name
    zstack.Z_STACK_SENSOR_PLANE_Z_M = sensor_z
    zstack.Z_STACK_COLLISION_FACE_Z_M = keepout_min
    zstack.Z_STACK_CLEARANCE_BOUNDARY_Z_M = boundary_z
    zstack.Z_STACK_MINIMUM_TIP_DISTANCE_M = minimum_distance
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    prefix = args.prefix or f"mla_short_z_stack_tilt_calibration_{stamp}"
    zstack.Z_STACK_SITES = (
        zstack.ZStackSite("center", f"{prefix}_center", 0.0, 0.0),
        zstack.ZStackSite("x50", f"{prefix}_x50", 0.050, 0.0),
    )

    experiments = zstack.build_z_stack_tilt_experiments(environment)
    output = environment / "trajectories"
    paths = tuple(
        write_authoritative_trajectory(output / f"{exp.site.trajectory_id}.csv", exp.poses)
        for exp in experiments
    )
    capture_path = output / f"{prefix}_capture_windows.csv"
    payload = zstack._capture_window_csv(experiments)
    capture_path.parent.mkdir(parents=True, exist_ok=True)
    if capture_path.exists():
        if capture_path.read_bytes() != payload:
            raise FileExistsError(f"refusing to overwrite differing authority: {capture_path}")
    else:
        capture_path.write_bytes(payload)

    # A compact machine-readable index makes the exact source/window pairing
    # unambiguous for playback and later calibration fitting.
    index_path = output / f"{prefix}_index.csv"
    rows = []
    for exp in experiments:
        rows.append(
            {
                "environment_id": environment.name,
                "site_id": exp.site.site_id,
                "trajectory_id": exp.site.trajectory_id,
                "trajectory_path": str(paths[len(rows)]),
                "capture_windows_path": str(capture_path),
                "source_duration_s": f"{exp.poses[-1].time_s:.9f}",
                "source_pose_count": str(len(exp.poses)),
                "capture_window_count": str(len(exp.visits)),
                "x_m": f"{exp.site.x_m:.9f}",
                "y_m": f"{exp.site.y_m:.9f}",
                "distance_sequence_m": "68,76,84,92,84,76,68 mm",
            }
        )
    with index_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    print(f"environment={environment}")
    for path in paths:
        print(f"trajectory={path}")
    print(f"capture_windows={capture_path}")
    print(f"index={index_path}")
    print("tip_z_m=-0.068,-0.076,-0.084,-0.092,-0.084,-0.076,-0.068")
    print("tilt_deg=25; settle_s=1; capture_s=3")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
