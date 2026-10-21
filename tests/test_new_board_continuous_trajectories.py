from __future__ import annotations

import csv
from decimal import Decimal
from pathlib import Path

import numpy as np
import pytest

from conftest import FakeCuroboAdapter

from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_curobo.timed_pose_compiler import CuroboTimedPathCompiler
from hx_waypoint_curobo.trajectory_io import read_timed_pose_csv
from hx_waypoint_tools.trajectory_generator import (
    NewBoardOrientationMotion,
    NEW_BOARD_CONTINUOUS_DURATION_NS,
    NEW_BOARD_CONTINUOUS_INTERVAL_NS,
    NEW_BOARD_CONTINUOUS_SAMPLE_COUNT,
    NEW_BOARD_CONTINUOUS_PERSISTENT_MOTION_IDS,
    NEW_BOARD_CONTINUOUS_TRAJECTORY_FILENAMES,
    NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS,
    NEW_BOARD_CONTINUOUS_ZPLUS26_OFFSET_M,
    NEW_BOARD_CONTINUOUS_ZPLUS26_SOURCE_TRAJECTORY_IDS,
    NEW_BOARD_CONTINUOUS_ZPLUS26_TRAJECTORY_FILENAMES,
    NEW_BOARD_CONTINUOUS_ZPLUS26_TRAJECTORY_IDS,
    NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_OFFSET_M,
    NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_SOURCE_TRAJECTORY_IDS,
    NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_TRAJECTORY_FILENAMES,
    NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_TRAJECTORY_IDS,
    NEW_BOARD_FRAME_ID,
    NEW_BOARD_QUATERNION_XYZW,
    build_new_board_continuous_trajectory,
    build_new_board_continuous_zplus26_trajectory,
    generate_new_board_continuous_trajectories,
    generate_new_board_continuous_zplus26_trajectories,
    generate_new_board_continuous_zplus26_up50_trajectories,
    new_board_continuous_geometry_position,
    new_board_continuous_quaternion,
    render_new_board_continuous_trajectory_csv,
    render_new_board_continuous_zplus26_trajectory_csv,
    render_new_board_continuous_zplus26_up50_trajectory_csv,
)


def _trajectory_directory(repository_root: Path) -> Path:
    return (
        repository_root
        / "environments"
        / "new_board_4cm_rod33365"
        / "trajectories"
    )


def test_checked_in_continuous_csvs_byte_match_generator(
    repository_root: Path,
    tmp_path: Path,
) -> None:
    generated_paths = generate_new_board_continuous_trajectories(tmp_path)

    assert tuple(path.name for path in generated_paths) == (
        NEW_BOARD_CONTINUOUS_TRAJECTORY_FILENAMES
    )
    for generated_path in generated_paths:
        checked_in_path = _trajectory_directory(repository_root) / generated_path.name
        assert generated_path.read_bytes() == render_new_board_continuous_trajectory_csv(
            generated_path.stem
        )
        assert checked_in_path.read_bytes() == generated_path.read_bytes()

    assert generate_new_board_continuous_trajectories(tmp_path) == generated_paths


def test_continuous_generator_refuses_to_replace_authority(tmp_path: Path) -> None:
    path = tmp_path / NEW_BOARD_CONTINUOUS_TRAJECTORY_FILENAMES[0]
    path.write_bytes(b"different authoritative bytes\n")

    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        generate_new_board_continuous_trajectories(tmp_path)

    assert path.read_bytes() == b"different authoritative bytes\n"


def test_checked_in_zplus26_csvs_byte_match_generator(
    repository_root: Path,
    tmp_path: Path,
) -> None:
    generated_paths = generate_new_board_continuous_zplus26_trajectories(tmp_path)

    assert tuple(path.name for path in generated_paths) == (
        NEW_BOARD_CONTINUOUS_ZPLUS26_TRAJECTORY_FILENAMES
    )
    for generated_path in generated_paths:
        checked_in_path = _trajectory_directory(repository_root) / generated_path.name
        assert generated_path.read_bytes() == (
            render_new_board_continuous_zplus26_trajectory_csv(
                generated_path.stem
            )
        )
        assert checked_in_path.read_bytes() == generated_path.read_bytes()

    assert (
        generate_new_board_continuous_zplus26_trajectories(tmp_path)
        == generated_paths
    )


def test_zplus26_generator_refuses_to_replace_authority(tmp_path: Path) -> None:
    path = tmp_path / NEW_BOARD_CONTINUOUS_ZPLUS26_TRAJECTORY_FILENAMES[0]
    path.write_bytes(b"different authoritative bytes\n")

    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        generate_new_board_continuous_zplus26_trajectories(tmp_path)

    assert path.read_bytes() == b"different authoritative bytes\n"


def test_checked_in_zplus26_up50_csvs_byte_match_generator(
    repository_root: Path,
    tmp_path: Path,
) -> None:
    generated_paths = generate_new_board_continuous_zplus26_up50_trajectories(
        tmp_path
    )

    assert tuple(path.name for path in generated_paths) == (
        NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_TRAJECTORY_FILENAMES
    )
    for generated_path in generated_paths:
        checked_in_path = _trajectory_directory(repository_root) / generated_path.name
        assert generated_path.read_bytes() == (
            render_new_board_continuous_zplus26_up50_trajectory_csv(
                generated_path.stem
            )
        )
        assert checked_in_path.read_bytes() == generated_path.read_bytes()

    assert (
        generate_new_board_continuous_zplus26_up50_trajectories(tmp_path)
        == generated_paths
    )


def test_zplus26_up50_generator_refuses_to_replace_authority(
    tmp_path: Path,
) -> None:
    path = tmp_path / NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_TRAJECTORY_FILENAMES[0]
    path.write_bytes(b"different authoritative bytes\n")

    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        generate_new_board_continuous_zplus26_up50_trajectories(tmp_path)

    assert path.read_bytes() == b"different authoritative bytes\n"


@pytest.mark.parametrize(
    ("source_trajectory_id", "shifted_trajectory_id"),
    tuple(
        zip(
            NEW_BOARD_CONTINUOUS_ZPLUS26_SOURCE_TRAJECTORY_IDS,
            NEW_BOARD_CONTINUOUS_ZPLUS26_TRAJECTORY_IDS,
            strict=True,
        )
    ),
)
def test_zplus26_authorities_change_only_tip_z(
    repository_root: Path,
    source_trajectory_id: str,
    shifted_trajectory_id: str,
) -> None:
    source_path = _trajectory_directory(repository_root) / f"{source_trajectory_id}.csv"
    shifted_path = _trajectory_directory(repository_root) / f"{shifted_trajectory_id}.csv"
    with source_path.open("r", encoding="utf-8", newline="") as stream:
        source_rows = list(csv.DictReader(stream))
    with shifted_path.open("r", encoding="utf-8", newline="") as stream:
        shifted_rows = list(csv.DictReader(stream))

    assert len(source_rows) == len(shifted_rows) == NEW_BOARD_CONTINUOUS_SAMPLE_COUNT
    unchanged_fields = (
        "time_s",
        "frame_id",
        "tip_x_m",
        "tip_y_m",
        "tip_qx",
        "tip_qy",
        "tip_qz",
        "tip_qw",
        "phase",
    )
    expected_offset = Decimal("0.026000000")
    for source_row, shifted_row in zip(source_rows, shifted_rows, strict=True):
        assert {
            field: shifted_row[field] for field in unchanged_fields
        } == {
            field: source_row[field] for field in unchanged_fields
        }
        assert Decimal(shifted_row["tip_z_m"]) - Decimal(
            source_row["tip_z_m"]
        ) == expected_offset


@pytest.mark.parametrize(
    ("source_trajectory_id", "shifted_trajectory_id"),
    tuple(
        zip(
            NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_SOURCE_TRAJECTORY_IDS,
            NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_TRAJECTORY_IDS,
            strict=True,
        )
    ),
)
def test_zplus26_up50_authorities_change_only_tip_z(
    repository_root: Path,
    source_trajectory_id: str,
    shifted_trajectory_id: str,
) -> None:
    source_path = _trajectory_directory(repository_root) / f"{source_trajectory_id}.csv"
    shifted_path = _trajectory_directory(repository_root) / f"{shifted_trajectory_id}.csv"
    with source_path.open("r", encoding="utf-8", newline="") as stream:
        source_rows = list(csv.DictReader(stream))
    with shifted_path.open("r", encoding="utf-8", newline="") as stream:
        shifted_rows = list(csv.DictReader(stream))

    assert len(source_rows) == len(shifted_rows) == NEW_BOARD_CONTINUOUS_SAMPLE_COUNT
    unchanged_fields = (
        "time_s",
        "frame_id",
        "tip_x_m",
        "tip_y_m",
        "tip_qx",
        "tip_qy",
        "tip_qz",
        "tip_qw",
        "phase",
    )
    expected_offset = Decimal("-0.050000000")
    for source_row, shifted_row in zip(source_rows, shifted_rows, strict=True):
        assert {
            field: shifted_row[field] for field in unchanged_fields
        } == {
            field: source_row[field] for field in unchanged_fields
        }
        assert Decimal(shifted_row["tip_z_m"]) - Decimal(
            source_row["tip_z_m"]
        ) == expected_offset


def test_zplus26_group_keeps_existing_boundary_and_requested_sensor_gap(
    repository_root: Path,
) -> None:
    all_z = []
    for trajectory_id in NEW_BOARD_CONTINUOUS_ZPLUS26_TRAJECTORY_IDS:
        poses = read_timed_pose_csv(
            _trajectory_directory(repository_root) / f"{trajectory_id}.csv"
        )
        all_z.extend(pose.position_m[2] for pose in poses)
        assert poses[-1].position_m == pytest.approx(
            poses[0].position_m,
            abs=1.0e-12,
        )

    assert NEW_BOARD_CONTINUOUS_ZPLUS26_OFFSET_M == pytest.approx(0.026, abs=0.0)
    assert min(all_z) == pytest.approx(-0.032225604, abs=5.1e-10)
    assert max(all_z) == pytest.approx(-0.015529086, abs=5.1e-10)
    assert max(all_z) <= -0.015
    assert -0.015 - max(all_z) == pytest.approx(0.000529086, abs=5.1e-10)

    actual_sensor_plane_z_m = 0.043
    minimum_gap_m = actual_sensor_plane_z_m - max(all_z)
    maximum_gap_m = actual_sensor_plane_z_m - min(all_z)
    assert minimum_gap_m == pytest.approx(0.058529086, abs=5.1e-10)
    assert maximum_gap_m == pytest.approx(0.075225604, abs=5.1e-10)


def test_zplus26_up50_group_has_requested_sensor_gap(
    repository_root: Path,
) -> None:
    all_z = []
    for trajectory_id in NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_TRAJECTORY_IDS:
        poses = read_timed_pose_csv(
            _trajectory_directory(repository_root) / f"{trajectory_id}.csv"
        )
        all_z.extend(pose.position_m[2] for pose in poses)
        assert poses[-1].position_m == pytest.approx(
            poses[0].position_m,
            abs=1.0e-12,
        )

    assert NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_OFFSET_M == pytest.approx(
        -0.050,
        abs=0.0,
    )
    assert min(all_z) == pytest.approx(-0.082225604, abs=5.1e-10)
    assert max(all_z) == pytest.approx(-0.065529086, abs=5.1e-10)

    actual_sensor_plane_z_m = 0.043
    minimum_gap_m = actual_sensor_plane_z_m - max(all_z)
    maximum_gap_m = actual_sensor_plane_z_m - min(all_z)
    assert minimum_gap_m == pytest.approx(0.108529086, abs=5.1e-10)
    assert maximum_gap_m == pytest.approx(0.125225604, abs=5.1e-10)


def test_optional_orientation_motion_is_bounded_and_returns_to_baseline() -> None:
    profile = NewBoardOrientationMotion(
        enabled=True,
        amplitude_deg=15.0,
        cycle_count=4,
    )
    timestamps_s = np.linspace(0.0, 30.0, 3001)
    quaternions = np.asarray(
        [new_board_continuous_quaternion(time_s, profile) for time_s in timestamps_s]
    )

    assert quaternions[0] == pytest.approx(NEW_BOARD_QUATERNION_XYZW, abs=0.0)
    assert quaternions[-1] == pytest.approx(NEW_BOARD_QUATERNION_XYZW, abs=0.0)
    assert np.linalg.norm(quaternions, axis=1) == pytest.approx(1.0, abs=1.0e-12)
    tilt_deg = np.degrees(2.0 * np.arccos(np.clip(quaternions[:, 3], -1.0, 1.0)))
    assert float(np.max(tilt_deg)) == pytest.approx(15.0, abs=0.02)
    assert np.count_nonzero(np.linalg.norm(quaternions[:, :3], axis=1) > 1.0e-6) > 2500


def test_orientation_motion_defaults_off_and_changes_render_only_when_enabled() -> None:
    trajectory_id = NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS[0]
    fixed = render_new_board_continuous_trajectory_csv(trajectory_id)
    disabled = render_new_board_continuous_trajectory_csv(
        trajectory_id,
        orientation_motion=NewBoardOrientationMotion(
            enabled=False,
            amplitude_deg=25.0,
        ),
    )
    moving = render_new_board_continuous_trajectory_csv(
        trajectory_id,
        orientation_motion=NewBoardOrientationMotion(
            enabled=True,
            amplitude_deg=12.0,
        ),
    )

    assert fixed == disabled
    assert fixed != moving


@pytest.mark.parametrize(
    "profile",
    [
        {"amplitude_deg": -0.1},
        {"amplitude_deg": 45.1},
        {"cycle_count": 0},
        {"cycle_count": 21},
        {"cycle_count": 2.5},
    ],
)
def test_orientation_motion_rejects_invalid_controls(profile: dict[str, float]) -> None:
    with pytest.raises(ValueError):
        NewBoardOrientationMotion(**profile)


@pytest.mark.parametrize("trajectory_id", NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS)
def test_continuous_schema_timing_and_static_boundaries_are_exact(
    repository_root: Path,
    trajectory_id: str,
) -> None:
    path = _trajectory_directory(repository_root) / f"{trajectory_id}.csv"
    poses = read_timed_pose_csv(path)
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))

    assert len(rows) == len(poses) == NEW_BOARD_CONTINUOUS_SAMPLE_COUNT
    time_ns = [
        int(Decimal(row["time_s"]) * Decimal(1_000_000_000)) for row in rows
    ]
    assert time_ns == [
        index * NEW_BOARD_CONTINUOUS_INTERVAL_NS
        for index in range(NEW_BOARD_CONTINUOUS_SAMPLE_COUNT)
    ]
    assert time_ns[-1] == NEW_BOARD_CONTINUOUS_DURATION_NS
    assert {row["phase"] for row in rows} == {"draw"}
    assert {pose.frame_id for pose in poses} == {NEW_BOARD_FRAME_ID}
    assert {pose.quaternion_xyzw for pose in poses} == {
        NEW_BOARD_QUATERNION_XYZW
    }
    if trajectory_id == "bafe_continuous_lorenz_3d":
        assert np.linalg.norm(
            np.asarray(poses[-1].position_m) - np.asarray(poses[0].position_m)
        ) > 0.005
    else:
        assert poses[-1].position_m == pytest.approx(
            poses[0].position_m,
            abs=1.0e-12,
        )


def test_continuous_paths_stay_inside_the_board_workspace_and_clear_keepout() -> None:
    timestamps_s = np.linspace(0.0, 30.0, 3001)

    for trajectory_id in NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS:
        positions = np.asarray(
            [
                new_board_continuous_geometry_position(trajectory_id, time_s)
                for time_s in timestamps_s
            ]
        )
        assert float(np.max(np.abs(positions[:, 0]))) <= 0.045 + 1.0e-12
        assert float(np.max(np.abs(positions[:, 1]))) <= 0.034 + 1.0e-12
        assert float(np.min(positions[:, 2])) >= -0.059
        assert float(np.max(positions[:, 2])) <= -0.041

        clearance_to_boundary_m = -positions[:, 2] - 0.015
        assert float(np.min(clearance_to_boundary_m)) >= 0.026


@pytest.mark.parametrize("trajectory_id", NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS)
def test_continuous_motion_is_smooth_and_persistently_informative(
    trajectory_id: str,
) -> None:
    dt_s = 0.01
    timestamps_s = np.arange(0.0, 30.0 + 0.5 * dt_s, dt_s)
    positions = np.asarray(
        [
            new_board_continuous_geometry_position(trajectory_id, time_s)
            for time_s in timestamps_s
        ]
    )
    velocity = np.gradient(positions, dt_s, axis=0, edge_order=2)
    acceleration = np.gradient(velocity, dt_s, axis=0, edge_order=2)
    jerk = np.gradient(acceleration, dt_s, axis=0, edge_order=2)

    assert float(np.linalg.norm(positions[5] - positions[0])) < 1.0e-5
    assert float(np.linalg.norm(positions[-1] - positions[-6])) < 1.0e-5
    assert float(np.max(np.linalg.norm(velocity, axis=1))) < 0.15
    assert float(np.max(np.linalg.norm(acceleration, axis=1))) < 0.8
    assert float(np.max(np.linalg.norm(jerk, axis=1))) < 6.0

    rolling_xy_spans = []
    rolling_z_spans = []
    rolling_path_lengths = []
    for start in range(0, len(positions) - 273 + 1, 10):
        window = positions[start : start + 273]
        spans = np.ptp(window, axis=0)
        rolling_xy_spans.append(float(np.linalg.norm(spans[:2])))
        rolling_z_spans.append(float(spans[2]))
        rolling_path_lengths.append(
            float(np.linalg.norm(np.diff(window, axis=0), axis=1).sum())
        )
    if trajectory_id in NEW_BOARD_CONTINUOUS_PERSISTENT_MOTION_IDS:
        assert min(rolling_xy_spans) > 0.07
        assert min(rolling_z_spans) > 0.010
        assert min(rolling_path_lengths) > 0.14

    interior_path_lengths = []
    for start in range(60, len(positions) - 60 - 15):
        window = positions[start : start + 16]
        interior_path_lengths.append(
            float(np.linalg.norm(np.diff(window, axis=0), axis=1).sum())
        )
    if trajectory_id in NEW_BOARD_CONTINUOUS_PERSISTENT_MOTION_IDS:
        assert min(interior_path_lengths) > 0.0025

    cv_rmse_m = []
    local_time_s = np.arange(32, dtype=float) * dt_s
    design = np.column_stack((np.ones(32), local_time_s))
    for start in range(0, len(positions) - 32 + 1, 16):
        window = positions[start : start + 32]
        coefficients, _, _, _ = np.linalg.lstsq(design, window, rcond=None)
        residual = window - design @ coefficients
        cv_rmse_m.append(float(np.sqrt(np.mean(np.sum(residual**2, axis=1)))))
    assert float(np.percentile(cv_rmse_m, 95.0)) < 0.0021
    assert max(cv_rmse_m) < 0.0022


def test_lorenz_candidate_is_explicitly_a_motion_stress_not_a_startup_default() -> None:
    assert "bafe_continuous_lorenz_3d" in NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS
    assert (
        "bafe_continuous_lorenz_3d"
        not in NEW_BOARD_CONTINUOUS_PERSISTENT_MOTION_IDS
    )


@pytest.mark.parametrize("trajectory_id", NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS)
def test_continuous_assets_pass_the_fixed_time_compiler_contract(
    repository_root: Path,
    trajectory_id: str,
) -> None:
    environment = EnvironmentBuilder().load(
        repository_root / "environments" / "new_board_4cm_rod33365"
    )
    source = read_timed_pose_csv(
        _trajectory_directory(repository_root) / f"{trajectory_id}.csv"
    )
    adapter = FakeCuroboAdapter(environment)

    result = CuroboTimedPathCompiler(environment, adapter).compile(source)

    assert result.source_poses == source
    assert result.metrics.uniform_time_scale == 1.0
    assert result.trajectory.duration_s == pytest.approx(30.0)
    compiled_times = {sample.time_s for sample in result.trajectory.samples}
    assert all(pose.time_s in compiled_times for pose in source)
    assert result.trajectory.samples[0].velocities == (0.0,) * 6
    assert result.trajectory.samples[0].accelerations == (0.0,) * 6
    assert result.trajectory.samples[-1].velocities == (0.0,) * 6
    assert result.trajectory.samples[-1].accelerations == (0.0,) * 6


@pytest.mark.parametrize(
    "trajectory_id", NEW_BOARD_CONTINUOUS_ZPLUS26_TRAJECTORY_IDS
)
def test_zplus26_assets_pass_the_fixed_time_compiler_contract(
    repository_root: Path,
    trajectory_id: str,
) -> None:
    environment = EnvironmentBuilder().load(
        repository_root / "environments" / "new_board_4cm_rod33365"
    )
    source = read_timed_pose_csv(
        _trajectory_directory(repository_root) / f"{trajectory_id}.csv"
    )
    adapter = FakeCuroboAdapter(environment)

    result = CuroboTimedPathCompiler(environment, adapter).compile(source)

    assert result.source_poses == source
    assert result.metrics.uniform_time_scale == 1.0
    assert result.trajectory.duration_s == pytest.approx(30.0)
    compiled_times = {sample.time_s for sample in result.trajectory.samples}
    assert all(pose.time_s in compiled_times for pose in source)
    assert result.trajectory.samples[0].velocities == (0.0,) * 6
    assert result.trajectory.samples[0].accelerations == (0.0,) * 6
    assert result.trajectory.samples[-1].velocities == (0.0,) * 6
    assert result.trajectory.samples[-1].accelerations == (0.0,) * 6


def test_continuous_builder_matches_dense_geometry_at_source_times() -> None:
    for trajectory_id in NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS:
        poses = build_new_board_continuous_trajectory(trajectory_id)
        for pose in poses:
            assert pose.position_m == pytest.approx(
                new_board_continuous_geometry_position(trajectory_id, pose.time_s),
                abs=1.0e-15,
            )


@pytest.mark.parametrize(
    ("source_trajectory_id", "shifted_trajectory_id"),
    tuple(
        zip(
            NEW_BOARD_CONTINUOUS_ZPLUS26_SOURCE_TRAJECTORY_IDS,
            NEW_BOARD_CONTINUOUS_ZPLUS26_TRAJECTORY_IDS,
            strict=True,
        )
    ),
)
def test_zplus26_builder_is_exact_translation_of_source_builder(
    source_trajectory_id: str,
    shifted_trajectory_id: str,
) -> None:
    source = build_new_board_continuous_trajectory(source_trajectory_id)
    shifted = build_new_board_continuous_zplus26_trajectory(
        shifted_trajectory_id
    )

    assert len(source) == len(shifted)
    for source_pose, shifted_pose in zip(source, shifted, strict=True):
        assert shifted_pose.time_s == source_pose.time_s
        assert shifted_pose.frame_id == source_pose.frame_id
        assert shifted_pose.position_m[:2] == source_pose.position_m[:2]
        assert shifted_pose.position_m[2] == pytest.approx(
            source_pose.position_m[2] + NEW_BOARD_CONTINUOUS_ZPLUS26_OFFSET_M,
            abs=0.0,
        )
        assert shifted_pose.quaternion_xyzw == source_pose.quaternion_xyzw
        assert shifted_pose.phase == source_pose.phase
