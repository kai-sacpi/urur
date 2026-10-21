from __future__ import annotations

import csv
from pathlib import Path
from typing import Iterable, Mapping, Sequence

from .models import CompiledTrajectory, TimedPose
from .trajectory_io import write_joint_trajectory_csv, write_timed_pose_csv


JOINT_FRAME_COLUMNS = (
    "time_s",
    "frame_id",
    "x_m",
    "y_m",
    "z_m",
    "qx",
    "qy",
    "qz",
    "qw",
)


class TrajectoryExporter:
    def write_source(self, path: str | Path, poses: Iterable[TimedPose]) -> None:
        write_timed_pose_csv(path, poses)

    def write_joint_trajectory(
        self,
        path: str | Path,
        trajectory: CompiledTrajectory,
    ) -> None:
        write_joint_trajectory_csv(path, trajectory)

    def write_joint_frames(
        self,
        path: str | Path,
        rows: Iterable[Mapping[str, object]],
    ) -> None:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        with destination.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=JOINT_FRAME_COLUMNS)
            writer.writeheader()
            count = 0
            for row in rows:
                writer.writerow({column: row[column] for column in JOINT_FRAME_COLUMNS})
                count += 1
        if count == 0:
            raise ValueError("joint_frames.csv cannot be empty")

    def write_rows(
        self,
        path: str | Path,
        columns: Sequence[str],
        rows: Iterable[Mapping[str, object]],
    ) -> None:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        with destination.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=tuple(columns))
            writer.writeheader()
            count = 0
            for row in rows:
                writer.writerow({column: row[column] for column in columns})
                count += 1
        if count == 0:
            raise ValueError(f"{destination.name} cannot be empty")
