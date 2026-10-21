from __future__ import annotations

import csv
import hashlib
from decimal import Decimal, InvalidOperation
from pathlib import Path
from typing import Iterable

from .errors import TrajectoryFormatError
from .models import CompiledTrajectory, JointSample, TimedPose


SOURCE_COLUMNS = (
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

_NANOSECONDS_PER_SECOND = Decimal(1_000_000_000)


def _parse_source_time_s(raw: str, line_number: int) -> float:
    try:
        decimal_time = Decimal(raw)
    except (InvalidOperation, TypeError) as error:
        raise TrajectoryFormatError(
            f"time_s must be a finite decimal at row {line_number}"
        ) from error
    if not decimal_time.is_finite() or decimal_time < 0:
        raise TrajectoryFormatError(
            f"time_s must be finite and non-negative at row {line_number}"
        )
    nanoseconds = decimal_time * _NANOSECONDS_PER_SECOND
    integral_nanoseconds = nanoseconds.to_integral_value()
    if nanoseconds != integral_nanoseconds:
        raise TrajectoryFormatError(
            f"time_s at row {line_number} is not exactly representable in ROS nanoseconds"
        )
    if integral_nanoseconds > Decimal(2_147_483_647_999_999_999):
        raise TrajectoryFormatError(f"time_s is outside ROS Duration range at row {line_number}")
    authoritative_nanoseconds = int(integral_nanoseconds)
    time_s = authoritative_nanoseconds / 1_000_000_000.0
    # The numerical planner currently carries seconds as binary64.  Accept a
    # source timestamp only when that representation round-trips to the same
    # ROS Duration nanosecond; otherwise later interpolation could silently
    # alter an otherwise valid Decimal timestamp at large absolute durations.
    if int(round(time_s * 1_000_000_000.0)) != authoritative_nanoseconds:
        raise TrajectoryFormatError(
            f"time_s at row {line_number} cannot be preserved exactly by the compiler"
        )
    return time_s


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_timed_pose_csv(path: str | Path) -> tuple[TimedPose, ...]:
    csv_path = Path(path).expanduser().resolve()
    if not csv_path.is_file():
        raise TrajectoryFormatError(f"source trajectory does not exist: {csv_path}")
    with csv_path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise TrajectoryFormatError("source trajectory is missing a header")
        if tuple(reader.fieldnames) != SOURCE_COLUMNS:
            raise TrajectoryFormatError(
                "source trajectory header must exactly match: " + ",".join(SOURCE_COLUMNS)
            )
        poses: list[TimedPose] = []
        for line_number, row in enumerate(reader, start=2):
            if None in row:
                raise TrajectoryFormatError(
                    f"source row {line_number} contains extra columns"
                )
            try:
                pose = TimedPose(
                    time_s=_parse_source_time_s(row["time_s"], line_number),
                    frame_id=row["frame_id"],
                    position_m=(
                        float(row["tip_x_m"]),
                        float(row["tip_y_m"]),
                        float(row["tip_z_m"]),
                    ),
                    quaternion_xyzw=(
                        float(row["tip_qx"]),
                        float(row["tip_qy"]),
                        float(row["tip_qz"]),
                        float(row["tip_qw"]),
                    ),
                    phase=row["phase"],
                )
            except (TypeError, ValueError, TrajectoryFormatError) as error:
                raise TrajectoryFormatError(f"invalid source row {line_number}: {error}") from error
            if poses and pose.time_s <= poses[-1].time_s:
                raise TrajectoryFormatError(
                    f"time_s must be strictly increasing at row {line_number}"
                )
            if pose.phase not in ("draw", "hold"):
                raise TrajectoryFormatError(
                    f"source phase must be draw or hold at row {line_number}"
                )
            if not poses and pose.phase == "hold":
                raise TrajectoryFormatError("the first source pose must use phase draw")
            if poses and pose.frame_id != poses[0].frame_id:
                raise TrajectoryFormatError("all source poses must use the same frame_id")
            if pose.phase == "hold" and poses:
                previous = poses[-1]
                same_position = all(
                    abs(current - before) <= 1.0e-9
                    for current, before in zip(pose.position_m, previous.position_m, strict=True)
                )
                same_orientation = all(
                    abs(current - before) <= 1.0e-9
                    for current, before in zip(
                        pose.quaternion_xyzw,
                        previous.quaternion_xyzw,
                        strict=True,
                    )
                )
                if not same_position or not same_orientation:
                    raise TrajectoryFormatError(
                        f"hold row {line_number} must repeat the previous pose"
                    )
            poses.append(pose)
    if len(poses) < 2:
        raise TrajectoryFormatError("source trajectory requires at least two poses")
    if poses[0].time_s != 0.0:
        raise TrajectoryFormatError("source trajectory must start at time_s=0")
    return tuple(poses)


def write_timed_pose_csv(path: str | Path, poses: Iterable[TimedPose]) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=SOURCE_COLUMNS)
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


def joint_trajectory_columns(joint_names: tuple[str, ...]) -> tuple[str, ...]:
    columns = ["time_s"]
    columns.extend(f"{name}_pos" for name in joint_names)
    columns.extend(f"{name}_vel" for name in joint_names)
    columns.extend(f"{name}_acc" for name in joint_names)
    columns.append("phase")
    return tuple(columns)


def write_joint_trajectory_csv(path: str | Path, trajectory: CompiledTrajectory) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = joint_trajectory_columns(trajectory.joint_names)
    with destination.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for sample in trajectory.samples:
            row: dict[str, str] = {"time_s": f"{sample.time_s:.9f}", "phase": sample.phase}
            for name, value in zip(trajectory.joint_names, sample.positions, strict=True):
                row[f"{name}_pos"] = f"{value:.12f}"
            for name, value in zip(trajectory.joint_names, sample.velocities, strict=True):
                row[f"{name}_vel"] = f"{value:.12f}"
            for name, value in zip(trajectory.joint_names, sample.accelerations, strict=True):
                row[f"{name}_acc"] = f"{value:.12f}"
            writer.writerow(row)


def read_joint_trajectory_csv(
    path: str | Path,
    joint_names: tuple[str, ...],
) -> CompiledTrajectory:
    source = Path(path).expanduser().resolve()
    expected = joint_trajectory_columns(joint_names)
    with source.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or any(name not in reader.fieldnames for name in expected):
            raise TrajectoryFormatError("joint trajectory CSV does not match the required schema")
        samples: list[JointSample] = []
        for line_number, row in enumerate(reader, start=2):
            try:
                samples.append(
                    JointSample(
                        time_s=float(row["time_s"]),
                        positions=tuple(float(row[f"{name}_pos"]) for name in joint_names),
                        velocities=tuple(float(row[f"{name}_vel"]) for name in joint_names),
                        accelerations=tuple(float(row[f"{name}_acc"]) for name in joint_names),
                        phase=row["phase"],
                    )
                )
            except (TypeError, ValueError, TrajectoryFormatError) as error:
                raise TrajectoryFormatError(f"invalid joint row {line_number}: {error}") from error
    return CompiledTrajectory(joint_names=joint_names, samples=tuple(samples))
