from __future__ import annotations

import csv
from pathlib import Path

import pytest
from hx_waypoint_curobo.errors import TrajectoryFormatError
from hx_waypoint_curobo.trajectory_io import SOURCE_COLUMNS, read_timed_pose_csv
from hx_waypoint_tools.trajectory_generator import (
    BAFE_DOWNWARD_MJ_TRAJECTORY_FILENAMES,
    BAFE_DOWNWARD_TRAJECTORY_FILENAMES,
    BAFE_FRAME_ID,
    BAFE_STARTUP_TRAJECTORY_FILENAMES,
    NEW_BOARD_CONTINUOUS_TRAJECTORY_FILENAMES,
    NEW_BOARD_CONTINUOUS_ZPLUS26_TRAJECTORY_FILENAMES,
    NEW_BOARD_FIG14_TRAJECTORY_FILENAMES,
    NEW_BOARD_FRAME_ID,
)
from hx_waypoint_tools.yaw_calibration import YAW_CALIBRATION_TRAJECTORY_FILENAMES


def _zero_quaternion(rows: list[str]) -> None:
    values = rows[2].split(",")
    values[5:9] = ["0", "0", "0", "0"]
    rows[2] = ",".join(values)


def _duplicate_previous_time(rows: list[str]) -> None:
    values = rows[2].split(",")
    values[0] = rows[1].split(",")[0]
    rows[2] = ",".join(values)


def _sub_nanosecond_time(rows: list[str]) -> None:
    values = rows[2].split(",")
    values[0] = "0.0000000001"
    rows[2] = ",".join(values)


def _one_nanosecond_start(rows: list[str]) -> None:
    values = rows[1].split(",")
    values[0] = "0.000000001"
    rows[1] = ",".join(values)


def test_source_csv_schema(repository_root: Path) -> None:
    poses = read_timed_pose_csv(
        repository_root / "environments" / "lab_wall_a" / "trajectories" / "circle_01.csv"
    )
    assert poses[0].time_s == 0.0
    assert poses[-1].phase == "hold"
    assert {pose.frame_id for pose in poses} == {"wall_draw_frame"}


def test_all_shipped_test_trajectories_match_the_authoritative_schema(
    repository_root: Path,
) -> None:
    trajectory_root = (
        repository_root / "environments" / "lab_wall_a" / "trajectories"
    )
    paths = sorted(trajectory_root.glob("*.csv"))
    assert paths
    for path in paths:
        poses = read_timed_pose_csv(path)
        expected_frame = (
            BAFE_FRAME_ID
            if path.name
            in (
                *BAFE_STARTUP_TRAJECTORY_FILENAMES,
                *BAFE_DOWNWARD_TRAJECTORY_FILENAMES,
                *BAFE_DOWNWARD_MJ_TRAJECTORY_FILENAMES,
            )
            else "wall_draw_frame"
        )
        assert len(poses) >= 2, path.name
        assert poses[0].time_s == 0.0, path.name
        assert all(pose.frame_id == expected_frame for pose in poses), path.name


def test_new_board_trajectories_match_the_authoritative_schema(
    repository_root: Path,
) -> None:
    trajectory_root = (
        repository_root
        / "environments"
        / "new_board_4cm_rod33365"
        / "trajectories"
    )
    paths = sorted(trajectory_root.glob("*.csv"))

    expected_names = tuple(
        sorted(
            (
                *NEW_BOARD_FIG14_TRAJECTORY_FILENAMES,
                *NEW_BOARD_CONTINUOUS_TRAJECTORY_FILENAMES,
                *NEW_BOARD_CONTINUOUS_ZPLUS26_TRAJECTORY_FILENAMES,
                *YAW_CALIBRATION_TRAJECTORY_FILENAMES,
            )
        )
    )
    for path in paths:
        poses = read_timed_pose_csv(path)
        assert len(poses) >= 2, path.name
        assert poses[0].time_s == 0.0, path.name
        assert all(pose.frame_id == NEW_BOARD_FRAME_ID for pose in poses), path.name

    actual_names = tuple(path.name for path in paths)
    unexpected = set(actual_names) - set(expected_names)
    assert not unexpected, f"Unexpected trajectory assets: {sorted(unexpected)}"
    missing = set(expected_names) - set(actual_names)
    if missing:
        pytest.skip(
            "Generated trajectory assets are external to the source-only release; "
            "restore the matching dataset to run this integration test. Missing: "
            + ", ".join(sorted(missing))
        )
    assert actual_names == expected_names


def test_source_quaternion_is_validated_without_rewriting(tmp_path: Path) -> None:
    path = tmp_path / "near-unit.csv"
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(SOURCE_COLUMNS)
        writer.writerow([0, "wall_draw_frame", 0, 0, 0, 0, 0, 0, 0.9995, "draw"])
        writer.writerow([1, "wall_draw_frame", 0.01, 0, 0, 0, 0, 0, 0.9995, "draw"])

    poses = read_timed_pose_csv(path)

    assert poses[0].quaternion_xyzw == (0.0, 0.0, 0.0, 0.9995)
    assert poses[1].quaternion_xyzw == (0.0, 0.0, 0.0, 0.9995)


@pytest.mark.parametrize(
    ("mutator", "message"),
    [
        (lambda rows: rows.__setitem__(0, rows[0].replace("frame_id,", "")), "header"),
        (lambda rows: rows.__setitem__(2, rows[2].replace("wall_draw_frame", "other")), "frame"),
        (_duplicate_previous_time, "increasing"),
        (_sub_nanosecond_time, "ROS nanoseconds"),
        (_one_nanosecond_start, "start at time_s=0"),
        (_zero_quaternion, "quaternion"),
        (lambda rows: rows.__setitem__(2, rows[2].replace(",draw", ",approach")), "phase"),
    ],
)
def test_invalid_source_is_rejected(
    tmp_path: Path,
    repository_root: Path,
    mutator,
    message: str,
) -> None:
    source = repository_root / "environments" / "lab_wall_a" / "trajectories" / "circle_01.csv"
    rows = source.read_text(encoding="utf-8").splitlines()
    mutator(rows)
    path = tmp_path / "bad.csv"
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")
    with pytest.raises(TrajectoryFormatError, match=message):
        read_timed_pose_csv(path)


def test_hold_must_repeat_previous_pose(tmp_path: Path) -> None:
    path = tmp_path / "hold.csv"
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(SOURCE_COLUMNS)
        writer.writerow([0, "wall_draw_frame", 0, 0, 0, 0, 0, 0, 1, "draw"])
        writer.writerow([1, "wall_draw_frame", 0.01, 0, 0, 0, 0, 0, 1, "hold"])
    with pytest.raises(TrajectoryFormatError, match="repeat the previous pose"):
        read_timed_pose_csv(path)


def test_source_data_row_with_extra_column_is_rejected(tmp_path: Path) -> None:
    path = tmp_path / "extra-column.csv"
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(SOURCE_COLUMNS)
        writer.writerow(
            [0, "wall_draw_frame", 0, 0, 0, 0, 0, 0, 1, "draw", "unexpected"]
        )
        writer.writerow([1, "wall_draw_frame", 0, 0, 0, 0, 0, 0, 1, "draw"])
    with pytest.raises(TrajectoryFormatError, match="extra columns"):
        read_timed_pose_csv(path)


def test_first_source_pose_cannot_be_hold(tmp_path: Path) -> None:
    path = tmp_path / "first-hold.csv"
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(SOURCE_COLUMNS)
        writer.writerow([0, "wall_draw_frame", 0, 0, 0, 0, 0, 0, 1, "hold"])
        writer.writerow([1, "wall_draw_frame", 0, 0, 0, 0, 0, 0, 1, "draw"])
    with pytest.raises(TrajectoryFormatError, match="first source pose must use phase draw"):
        read_timed_pose_csv(path)


def test_source_time_must_round_trip_through_compiler_representation(tmp_path: Path) -> None:
    path = tmp_path / "large-duration.csv"
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(SOURCE_COLUMNS)
        writer.writerow([0, "wall_draw_frame", 0, 0, 0, 0, 0, 0, 1, "draw"])
        writer.writerow(
            [
                "2147483647.000000001",
                "wall_draw_frame",
                0.01,
                0,
                0,
                0,
                0,
                0,
                1,
                "draw",
            ]
        )
    with pytest.raises(TrajectoryFormatError, match="preserved exactly"):
        read_timed_pose_csv(path)
