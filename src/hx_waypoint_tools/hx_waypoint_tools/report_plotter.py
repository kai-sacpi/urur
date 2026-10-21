from __future__ import annotations

from pathlib import Path

import numpy as np

from hx_waypoint_curobo.models import CompiledTrajectory


def trajectory_summary(trajectory: CompiledTrajectory) -> dict[str, float | int]:
    positions = np.asarray([sample.positions for sample in trajectory.samples], dtype=float)
    velocities = np.asarray([sample.velocities for sample in trajectory.samples], dtype=float)
    accelerations = np.asarray([sample.accelerations for sample in trajectory.samples], dtype=float)
    return {
        "duration_s": trajectory.duration_s,
        "point_count": len(trajectory.samples),
        "max_abs_position_rad": float(np.max(np.abs(positions))),
        "max_abs_velocity_rad_s": float(np.max(np.abs(velocities))),
        "max_abs_acceleration_rad_s2": float(np.max(np.abs(accelerations))),
    }


def save_summary(path: str | Path, trajectory: CompiledTrajectory) -> None:
    import yaml

    Path(path).write_text(
        yaml.safe_dump(trajectory_summary(trajectory), sort_keys=False),
        encoding="utf-8",
    )
