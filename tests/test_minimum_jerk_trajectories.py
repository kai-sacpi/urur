from __future__ import annotations

import csv
import hashlib
from decimal import Decimal
from pathlib import Path

import numpy as np
import pytest

from conftest import FakeCuroboAdapter

from hx_waypoint_curobo.contact_policy import ContactPolicy
from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_curobo.math3d import pose_matrix
from hx_waypoint_curobo.timed_pose_compiler import CuroboTimedPathCompiler
from hx_waypoint_curobo.trajectory_io import read_timed_pose_csv
from hx_waypoint_tools.trajectory_generator import (
    ACCEPTANCE_TRAJECTORY_FILENAMES,
    ACCEPTANCE_TRAJECTORY_IDS,
    BAFE_DRAW_DURATION_NS,
    BAFE_DRAW_INTERVAL_COUNT,
    BAFE_DRAW_INTERVAL_NS,
    BAFE_DRAW_SAMPLE_COUNT,
    BAFE_DOWNWARD_MJ_DRAW_DURATION_NS,
    BAFE_DOWNWARD_MJ_DRAW_INTERVAL_NS,
    BAFE_DOWNWARD_MJ_TRAJECTORY_FILENAMES,
    BAFE_DOWNWARD_MJ_TRAJECTORY_IDS,
    BAFE_DOWNWARD_MJ_VERTICAL_OFFSETS_M,
    BAFE_DOWNWARD_QUATERNION_XYZW,
    BAFE_DOWNWARD_TRAJECTORY_FILENAMES,
    BAFE_DOWNWARD_TRAJECTORY_IDS,
    BAFE_FRAME_ID,
    BAFE_GROUND_MJ_DRAW_DURATION_NS,
    BAFE_GROUND_MJ_DRAW_INTERVAL_NS,
    BAFE_GROUND_MJ_TRAJECTORY_FILENAMES,
    BAFE_GROUND_MJ_TRAJECTORY_IDS,
    BAFE_GROUND_MJ_V2_DRAW_DURATION_NS,
    BAFE_GROUND_MJ_V2_DRAW_INTERVAL_NS,
    BAFE_GROUND_MJ_V2_TRAJECTORY_FILENAMES,
    BAFE_GROUND_MJ_V2_TRAJECTORY_IDS,
    BAFE_GROUND_QUATERNION_XYZW,
    BAFE_HOLD_DURATION_NS,
    BAFE_STARTUP_TRAJECTORY_FILENAMES,
    BAFE_STARTUP_TRAJECTORY_IDS,
    DRAW_INTERVAL_COUNT,
    DRAW_INTERVAL_NS,
    DRAW_SAMPLE_COUNT,
    FRAME_ID,
    QUATERNION_XYZW,
    RECOVERY_DRAW_DURATION_NS,
    RECOVERY_DRAW_INTERVAL_NS,
    RECOVERY_TRAJECTORY_FILENAMES,
    RECOVERY_TRAJECTORY_IDS,
    acceptance_geometry_position,
    bafe_startup_geometry_position,
    bafe_startup_progress,
    build_acceptance_trajectories,
    build_bafe_downward_mj_trajectories,
    build_bafe_downward_trajectories,
    build_bafe_ground_mj_trajectories,
    build_bafe_ground_mj_v2_trajectories,
    build_bafe_startup_trajectories,
    generate_acceptance_trajectories,
    generate_bafe_downward_mj_trajectories,
    generate_bafe_downward_trajectories,
    generate_bafe_ground_mj_trajectories,
    generate_bafe_ground_mj_v2_trajectories,
    generate_bafe_startup_trajectories,
    generate_recovery_trajectories,
    minimum_jerk_acceleration_factor,
    minimum_jerk_progress,
    minimum_jerk_velocity_factor,
    render_acceptance_trajectory_csv,
    render_bafe_downward_mj_trajectory_csv,
    render_bafe_downward_trajectory_csv,
    render_bafe_ground_mj_trajectory_csv,
    render_bafe_ground_mj_v2_trajectory_csv,
    render_bafe_startup_trajectory_csv,
    render_recovery_trajectory_csv,
)


STRICT_REJECTION_FILENAMES = (
    "circle.csv",
    "figure8.csv",
    "lissajous_xy_z_sweep.csv",
    "rounded_square.csv",
    "spiral_in_out.csv",
    "straight_line.csv",
    "vertical_bob.csv",
    "zigzag_s_curve.csv",
)

LEGACY_TRAJECTORY_SHA256 = {
    "circle.csv": "012dd84049a51207b72924547c528faccd39daeea3519b0a228338abe9a42a41",
    "circle_01.csv": "5aeff5f7e67343322748d3d998fd6b5126e196d3cf866f3b34b732e32cc2a713",
    "figure8.csv": "79966ab1ef70c17e437c786bebd21b063d28a79f0dde61efc9a0397c1528d991",
    "lissajous_xy_z_sweep.csv": (
        "dbf2970e65b7b6cffd37341e7521e426b52c89e045dc64b6be44aaefd0c2cfe3"
    ),
    "rounded_square.csv": (
        "0026ff5625aa6661bbc7b58d271cd3902e50b7fa5a0cd76323936c7547463429"
    ),
    "spiral_in_out.csv": (
        "0595be7c90a07344acc73075ad6367a2812d9a3fed972c996799da33cc4a427d"
    ),
    "straight_line.csv": (
        "46e1d8b3b92fb76cd953ccb5a0bef0dece1f3ad655fc4a5ac161c73d1bde395c"
    ),
    "vertical_bob.csv": (
        "79558751733d3681a69b18eb355967e522180ae78c792832a7e9f416632e2efe"
    ),
    "zigzag_s_curve.csv": (
        "157e5543f5871018fb6fdf64947df44280d7514298758043efe8e1396b1f6389"
    ),
}


def _trajectory_directory(repository_root: Path) -> Path:
    return repository_root / "environments" / "lab_wall_a" / "trajectories"


def _ground_trajectory_directory(repository_root: Path) -> Path:
    return (
        repository_root
        / "environments"
        / "bafe_ground_p10_rod194"
        / "trajectories"
    )


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_checked_in_csvs_byte_match_deterministic_generator(
    repository_root: Path,
    tmp_path: Path,
) -> None:
    generated_paths = generate_acceptance_trajectories(tmp_path)

    assert tuple(path.name for path in generated_paths) == ACCEPTANCE_TRAJECTORY_FILENAMES
    assert tuple(path.name.removesuffix(".csv") for path in generated_paths) == (
        ACCEPTANCE_TRAJECTORY_IDS
    )
    for generated_path in generated_paths:
        trajectory_id = generated_path.stem
        checked_in_path = _trajectory_directory(repository_root) / generated_path.name
        assert generated_path.read_bytes() == render_acceptance_trajectory_csv(trajectory_id)
        assert checked_in_path.read_bytes() == generated_path.read_bytes()


def test_recovery_candidate_byte_matches_non_overwriting_generator(
    repository_root: Path,
    tmp_path: Path,
) -> None:
    generated_paths = generate_recovery_trajectories(tmp_path)

    assert tuple(path.name for path in generated_paths) == RECOVERY_TRAJECTORY_FILENAMES
    for generated_path in generated_paths:
        trajectory_id = generated_path.stem
        checked_in_path = _trajectory_directory(repository_root) / generated_path.name
        assert generated_path.read_bytes() == render_recovery_trajectory_csv(trajectory_id)
        assert checked_in_path.read_bytes() == generated_path.read_bytes()


def test_bafe_startup_csvs_byte_match_deterministic_generator(
    repository_root: Path,
    tmp_path: Path,
) -> None:
    generated_paths = generate_bafe_startup_trajectories(tmp_path)

    assert tuple(path.name for path in generated_paths) == (
        BAFE_STARTUP_TRAJECTORY_FILENAMES
    )
    for generated_path in generated_paths:
        trajectory_id = generated_path.stem
        checked_in_path = _trajectory_directory(repository_root) / generated_path.name
        assert generated_path.read_bytes() == render_bafe_startup_trajectory_csv(
            trajectory_id
        )
        assert checked_in_path.read_bytes() == generated_path.read_bytes()


def test_bafe_downward_csvs_byte_match_non_overwriting_generator(
    repository_root: Path,
    tmp_path: Path,
) -> None:
    generated_paths = generate_bafe_downward_trajectories(tmp_path)

    assert tuple(path.name for path in generated_paths) == (
        BAFE_DOWNWARD_TRAJECTORY_FILENAMES
    )
    for generated_path in generated_paths:
        trajectory_id = generated_path.stem
        checked_in_path = _trajectory_directory(repository_root) / generated_path.name
        assert generated_path.read_bytes() == render_bafe_downward_trajectory_csv(
            trajectory_id
        )
        assert checked_in_path.read_bytes() == generated_path.read_bytes()


def test_bafe_downward_generator_refuses_to_change_existing_authority(
    tmp_path: Path,
) -> None:
    path = tmp_path / BAFE_DOWNWARD_TRAJECTORY_FILENAMES[0]
    path.write_bytes(b"different authoritative bytes\n")

    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        generate_bafe_downward_trajectories(tmp_path)

    assert path.read_bytes() == b"different authoritative bytes\n"


def test_bafe_downward_mj_csvs_byte_match_non_overwriting_generator(
    repository_root: Path,
    tmp_path: Path,
) -> None:
    generated_paths = generate_bafe_downward_mj_trajectories(tmp_path)

    assert tuple(path.name for path in generated_paths) == (
        BAFE_DOWNWARD_MJ_TRAJECTORY_FILENAMES
    )
    for generated_path in generated_paths:
        trajectory_id = generated_path.stem
        checked_in_path = _trajectory_directory(repository_root) / generated_path.name
        assert generated_path.read_bytes() == render_bafe_downward_mj_trajectory_csv(
            trajectory_id
        )
        assert checked_in_path.read_bytes() == generated_path.read_bytes()


def test_bafe_ground_mj_csvs_byte_match_non_overwriting_generator(
    repository_root: Path,
    tmp_path: Path,
) -> None:
    generated_paths = generate_bafe_ground_mj_trajectories(tmp_path)

    assert tuple(path.name for path in generated_paths) == (
        BAFE_GROUND_MJ_TRAJECTORY_FILENAMES
    )
    for generated_path in generated_paths:
        trajectory_id = generated_path.stem
        checked_in_path = _ground_trajectory_directory(repository_root) / generated_path.name
        assert generated_path.read_bytes() == render_bafe_ground_mj_trajectory_csv(
            trajectory_id
        )
        assert checked_in_path.read_bytes() == generated_path.read_bytes()


def test_bafe_ground_mj_v2_csvs_byte_match_non_overwriting_generator(
    repository_root: Path,
    tmp_path: Path,
) -> None:
    generated_paths = generate_bafe_ground_mj_v2_trajectories(tmp_path)

    assert tuple(path.name for path in generated_paths) == (
        BAFE_GROUND_MJ_V2_TRAJECTORY_FILENAMES
    )
    for generated_path in generated_paths:
        trajectory_id = generated_path.stem
        checked_in_path = _ground_trajectory_directory(repository_root) / generated_path.name
        assert generated_path.read_bytes() == render_bafe_ground_mj_v2_trajectory_csv(
            trajectory_id
        )
        assert checked_in_path.read_bytes() == generated_path.read_bytes()


def test_recovery_generator_refuses_to_change_an_existing_authority(tmp_path: Path) -> None:
    path = tmp_path / RECOVERY_TRAJECTORY_FILENAMES[0]
    path.write_bytes(b"different authoritative bytes\n")

    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        generate_recovery_trajectories(tmp_path)

    assert path.read_bytes() == b"different authoritative bytes\n"


def test_generator_does_not_include_or_rewrite_legacy_trajectories(tmp_path: Path) -> None:
    legacy_names = (
        *STRICT_REJECTION_FILENAMES,
        "circle_01.csv",
        *RECOVERY_TRAJECTORY_FILENAMES,
    )
    sentinels = {name: f"sentinel:{name}\n".encode() for name in legacy_names}
    for name, payload in sentinels.items():
        (tmp_path / name).write_bytes(payload)

    generated_paths = generate_acceptance_trajectories(tmp_path)

    assert {path.name for path in generated_paths}.isdisjoint(legacy_names)
    assert {path.name for path in generated_paths} == set(ACCEPTANCE_TRAJECTORY_FILENAMES)
    for name, payload in sentinels.items():
        assert (tmp_path / name).read_bytes() == payload


def test_existing_baseline_and_strict_rejection_assets_are_unchanged(
    repository_root: Path,
) -> None:
    trajectory_directory = _trajectory_directory(repository_root)
    assert set(STRICT_REJECTION_FILENAMES).isdisjoint(ACCEPTANCE_TRAJECTORY_FILENAMES)
    assert {
        name: _sha256(trajectory_directory / name)
        for name in LEGACY_TRAJECTORY_SHA256
    } == LEGACY_TRAJECTORY_SHA256


@pytest.mark.parametrize("trajectory_id", ACCEPTANCE_TRAJECTORY_IDS)
def test_schema_timing_pose_and_hold_contract_are_exact(
    repository_root: Path,
    trajectory_id: str,
) -> None:
    path = _trajectory_directory(repository_root) / f"{trajectory_id}.csv"
    poses = read_timed_pose_csv(path)
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))

    assert len(rows) == len(poses) == DRAW_SAMPLE_COUNT + 1
    draw_time_ns = [
        int(Decimal(row["time_s"]) * Decimal(1_000_000_000))
        for row in rows[:DRAW_SAMPLE_COUNT]
    ]
    assert draw_time_ns == [index * DRAW_INTERVAL_NS for index in range(DRAW_SAMPLE_COUNT)]
    assert [row["phase"] for row in rows] == ["draw"] * DRAW_SAMPLE_COUNT + ["hold"]
    assert Decimal(rows[-2]["time_s"]) == Decimal("30.000000000")
    assert Decimal(rows[-1]["time_s"]) == Decimal("33.000000000")

    pose_columns = (
        "frame_id",
        "tip_x_m",
        "tip_y_m",
        "tip_z_m",
        "tip_qx",
        "tip_qy",
        "tip_qz",
        "tip_qw",
    )
    assert tuple(rows[-1][column] for column in pose_columns) == tuple(
        rows[-2][column] for column in pose_columns
    )
    assert poses[-1].position_m == poses[-2].position_m
    assert poses[-1].quaternion_xyzw == poses[-2].quaternion_xyzw
    assert {pose.frame_id for pose in poses} == {FRAME_ID}
    assert {pose.quaternion_xyzw for pose in poses} == {QUATERNION_XYZW}
    assert {pose.position_m[2] for pose in poses} == {0.0}


def test_recovery_circle_has_slower_authoritative_timing_and_static_hold(
    repository_root: Path,
) -> None:
    path = _trajectory_directory(repository_root) / RECOVERY_TRAJECTORY_FILENAMES[0]
    poses = read_timed_pose_csv(path)
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))

    assert len(rows) == len(poses) == DRAW_SAMPLE_COUNT + 1
    draw_time_ns = [
        int(Decimal(row["time_s"]) * Decimal(1_000_000_000))
        for row in rows[:DRAW_SAMPLE_COUNT]
    ]
    assert draw_time_ns == [
        index * RECOVERY_DRAW_INTERVAL_NS for index in range(DRAW_SAMPLE_COUNT)
    ]
    assert draw_time_ns[-1] == RECOVERY_DRAW_DURATION_NS
    assert Decimal(rows[-2]["time_s"]) == Decimal("60.000000000")
    assert Decimal(rows[-1]["time_s"]) == Decimal("63.000000000")
    assert rows[-1]["phase"] == "hold"
    assert poses[-1].position_m == poses[-2].position_m
    assert poses[-1].quaternion_xyzw == poses[-2].quaternion_xyzw
    assert {pose.frame_id for pose in poses} == {FRAME_ID}
    assert {pose.quaternion_xyzw for pose in poses} == {QUATERNION_XYZW}


@pytest.mark.parametrize("trajectory_id", BAFE_STARTUP_TRAJECTORY_IDS)
def test_bafe_startup_schema_timing_pose_and_hold_contract_are_exact(
    repository_root: Path,
    trajectory_id: str,
) -> None:
    path = _trajectory_directory(repository_root) / f"{trajectory_id}.csv"
    poses = read_timed_pose_csv(path)
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))

    assert len(rows) == len(poses) == BAFE_DRAW_SAMPLE_COUNT + 1
    draw_time_ns = [
        int(Decimal(row["time_s"]) * Decimal(1_000_000_000))
        for row in rows[:BAFE_DRAW_SAMPLE_COUNT]
    ]
    assert draw_time_ns == [
        index * BAFE_DRAW_INTERVAL_NS for index in range(BAFE_DRAW_SAMPLE_COUNT)
    ]
    assert draw_time_ns[-1] == BAFE_DRAW_DURATION_NS
    assert Decimal(rows[-2]["time_s"]) == Decimal("4.800000000")
    assert Decimal(rows[-1]["time_s"]) == Decimal("7.800000000")
    assert [row["phase"] for row in rows] == [
        "draw"
    ] * BAFE_DRAW_SAMPLE_COUNT + ["hold"]
    assert poses[-1].position_m == poses[-2].position_m
    assert poses[-1].quaternion_xyzw == poses[-2].quaternion_xyzw
    assert {pose.frame_id for pose in poses} == {BAFE_FRAME_ID}
    assert {pose.quaternion_xyzw for pose in poses} == {QUATERNION_XYZW}


@pytest.mark.parametrize("trajectory_id", BAFE_DOWNWARD_TRAJECTORY_IDS)
def test_bafe_downward_schema_timing_pose_and_hold_contract_are_exact(
    repository_root: Path,
    trajectory_id: str,
) -> None:
    path = _trajectory_directory(repository_root) / f"{trajectory_id}.csv"
    poses = read_timed_pose_csv(path)
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))

    assert len(rows) == len(poses) == BAFE_DRAW_SAMPLE_COUNT + 1
    draw_time_ns = [
        int(Decimal(row["time_s"]) * Decimal(1_000_000_000))
        for row in rows[:BAFE_DRAW_SAMPLE_COUNT]
    ]
    assert draw_time_ns == [
        index * BAFE_DRAW_INTERVAL_NS for index in range(BAFE_DRAW_SAMPLE_COUNT)
    ]
    assert draw_time_ns[-1] == BAFE_DRAW_DURATION_NS
    assert Decimal(rows[-2]["time_s"]) == Decimal("4.800000000")
    assert Decimal(rows[-1]["time_s"]) == Decimal("7.800000000")
    assert [row["phase"] for row in rows] == [
        "draw"
    ] * BAFE_DRAW_SAMPLE_COUNT + ["hold"]
    assert poses[-1].position_m == poses[-2].position_m
    assert poses[-1].quaternion_xyzw == poses[-2].quaternion_xyzw
    assert {pose.frame_id for pose in poses} == {BAFE_FRAME_ID}
    assert {pose.quaternion_xyzw for pose in poses} == {
        BAFE_DOWNWARD_QUATERNION_XYZW
    }


def test_bafe_downward_orientation_points_rod_axis_to_world_ground(environment) -> None:
    base_from_plane = environment._frame_to_base(BAFE_FRAME_ID)
    plane_from_tip = pose_matrix(
        (0.0, 0.0, 0.0),
        BAFE_DOWNWARD_QUATERNION_XYZW,
    )
    rod_axis_in_base = (base_from_plane @ plane_from_tip)[:3, :3] @ np.array(
        [0.0, 0.0, 1.0]
    )

    assert rod_axis_in_base == pytest.approx((0.0, 0.0, -1.0), abs=1.0e-12)


@pytest.mark.parametrize("trajectory_id", BAFE_DOWNWARD_MJ_TRAJECTORY_IDS)
def test_bafe_downward_mj_timing_geometry_and_hold_are_authoritative(
    repository_root: Path,
    trajectory_id: str,
) -> None:
    path = _trajectory_directory(repository_root) / f"{trajectory_id}.csv"
    poses = read_timed_pose_csv(path)
    geometry_trajectory_id = trajectory_id.removesuffix("_downward_mj")
    vertical_offset_m = BAFE_DOWNWARD_MJ_VERTICAL_OFFSETS_M[geometry_trajectory_id]

    assert len(poses) == BAFE_DRAW_SAMPLE_COUNT + 1
    assert [
        round(pose.time_s * 1_000_000_000)
        for pose in poses[:BAFE_DRAW_SAMPLE_COUNT]
    ] == [
        index * BAFE_DOWNWARD_MJ_DRAW_INTERVAL_NS
        for index in range(BAFE_DRAW_SAMPLE_COUNT)
    ]
    assert poses[-2].time_s == pytest.approx(
        BAFE_DOWNWARD_MJ_DRAW_DURATION_NS / 1_000_000_000
    )
    assert poses[-1].time_s == pytest.approx(
        (BAFE_DOWNWARD_MJ_DRAW_DURATION_NS + BAFE_HOLD_DURATION_NS)
        / 1_000_000_000
    )
    assert poses[-1].phase == "hold"
    assert poses[-1].position_m == poses[-2].position_m
    assert poses[-1].quaternion_xyzw == poses[-2].quaternion_xyzw
    assert {pose.frame_id for pose in poses} == {BAFE_FRAME_ID}
    assert {pose.quaternion_xyzw for pose in poses} == {
        BAFE_DOWNWARD_QUATERNION_XYZW
    }

    for index, pose in enumerate(poses[:BAFE_DRAW_SAMPLE_COUNT]):
        expected = bafe_startup_geometry_position(
            geometry_trajectory_id,
            minimum_jerk_progress(index / BAFE_DRAW_INTERVAL_COUNT),
        )
        assert pose.position_m == pytest.approx(
            (expected[0] + vertical_offset_m, expected[1], expected[2]),
            abs=5.1e-10,
        )


@pytest.mark.parametrize("trajectory_id", BAFE_GROUND_MJ_TRAJECTORY_IDS)
def test_bafe_ground_mj_timing_geometry_and_hold_are_authoritative(
    repository_root: Path,
    trajectory_id: str,
) -> None:
    path = _ground_trajectory_directory(repository_root) / f"{trajectory_id}.csv"
    poses = read_timed_pose_csv(path)
    geometry_trajectory_id = trajectory_id.removesuffix("_ground_mj")

    assert len(poses) == BAFE_DRAW_SAMPLE_COUNT + 1
    assert [
        round(pose.time_s * 1_000_000_000)
        for pose in poses[:BAFE_DRAW_SAMPLE_COUNT]
    ] == [
        index * BAFE_GROUND_MJ_DRAW_INTERVAL_NS
        for index in range(BAFE_DRAW_SAMPLE_COUNT)
    ]
    assert poses[-2].time_s == pytest.approx(
        BAFE_GROUND_MJ_DRAW_DURATION_NS / 1_000_000_000
    )
    assert poses[-1].time_s == pytest.approx(
        (BAFE_GROUND_MJ_DRAW_DURATION_NS + BAFE_HOLD_DURATION_NS)
        / 1_000_000_000
    )
    assert poses[-1].phase == "hold"
    assert poses[-1].position_m == poses[-2].position_m
    assert poses[-1].quaternion_xyzw == poses[-2].quaternion_xyzw
    assert {pose.frame_id for pose in poses} == {BAFE_FRAME_ID}
    assert {pose.quaternion_xyzw for pose in poses} == {
        BAFE_GROUND_QUATERNION_XYZW
    }

    for index, pose in enumerate(poses[:BAFE_DRAW_SAMPLE_COUNT]):
        expected = bafe_startup_geometry_position(
            geometry_trajectory_id,
            minimum_jerk_progress(index / BAFE_DRAW_INTERVAL_COUNT),
        )
        assert pose.position_m == pytest.approx(expected, abs=5.1e-10)


@pytest.mark.parametrize("trajectory_id", BAFE_GROUND_MJ_V2_TRAJECTORY_IDS)
def test_bafe_ground_mj_v2_timing_geometry_and_hold_are_authoritative(
    repository_root: Path,
    trajectory_id: str,
) -> None:
    path = _ground_trajectory_directory(repository_root) / f"{trajectory_id}.csv"
    poses = read_timed_pose_csv(path)
    geometry_trajectory_id = trajectory_id.removesuffix("_ground_mj_v2")

    assert len(poses) == BAFE_DRAW_SAMPLE_COUNT + 1
    assert [
        round(pose.time_s * 1_000_000_000)
        for pose in poses[:BAFE_DRAW_SAMPLE_COUNT]
    ] == [
        index * BAFE_GROUND_MJ_V2_DRAW_INTERVAL_NS
        for index in range(BAFE_DRAW_SAMPLE_COUNT)
    ]
    assert poses[-2].time_s == pytest.approx(
        BAFE_GROUND_MJ_V2_DRAW_DURATION_NS / 1_000_000_000
    )
    assert poses[-1].time_s == pytest.approx(
        (BAFE_GROUND_MJ_V2_DRAW_DURATION_NS + BAFE_HOLD_DURATION_NS)
        / 1_000_000_000
    )
    assert poses[-1].phase == "hold"
    assert poses[-1].position_m == poses[-2].position_m
    assert poses[-1].quaternion_xyzw == poses[-2].quaternion_xyzw
    assert {pose.frame_id for pose in poses} == {BAFE_FRAME_ID}
    assert {pose.quaternion_xyzw for pose in poses} == {
        BAFE_GROUND_QUATERNION_XYZW
    }

    for index, pose in enumerate(poses[:BAFE_DRAW_SAMPLE_COUNT]):
        expected = bafe_startup_geometry_position(
            geometry_trajectory_id,
            minimum_jerk_progress(index / BAFE_DRAW_INTERVAL_COUNT),
        )
        assert pose.position_m == pytest.approx(expected, abs=5.1e-10)


def test_bafe_ground_mj_rod_axis_points_down_and_matches_safety_plane(
    repository_root: Path,
) -> None:
    environment = EnvironmentBuilder().load(
        repository_root / "environments" / "bafe_ground_p10_rod194"
    )
    base_from_plane = environment._frame_to_base(BAFE_FRAME_ID)
    plane_from_tip = pose_matrix(
        (0.0, 0.0, 0.0),
        BAFE_GROUND_QUATERNION_XYZW,
    )
    rod_axis_in_base = (base_from_plane @ plane_from_tip)[:3, :3] @ np.array(
        [0.0, 0.0, 1.0]
    )
    plane_normal_in_base = np.asarray(
        ContactPolicy.from_environment(environment).plane_normal_base,
        dtype=float,
    )

    assert rod_axis_in_base == pytest.approx(-plane_normal_in_base, abs=1.0e-12)
    assert float(rod_axis_in_base @ np.array([0.0, 0.0, -1.0])) > 0.9999


def test_bafe_startup_schedule_reaches_half_path_at_early_checkpoint() -> None:
    assert bafe_startup_progress(0.0) == 0.0
    assert bafe_startup_progress(2.4) == pytest.approx(0.5, abs=1.0e-15)
    assert bafe_startup_progress(4.8) == 1.0


def test_bafe_startup_geometries_match_the_paper_scale_and_cartesian_limits() -> None:
    expected_ranges_m = {
        "bafe_wave_3d": (0.1891, 0.0731, 0.0145),
        "bafe_helix_3d": (0.1904, 0.1722, 0.0263),
        "bafe_rounded_square_3d": (0.1920, 0.1884, 0.0203),
        "bafe_spiral_3d": (0.1973, 0.1921, 0.0255),
    }
    dt_s = BAFE_DRAW_INTERVAL_NS / 1_000_000_000

    for trajectory_id, poses in build_bafe_startup_trajectories().items():
        positions = np.asarray(
            [pose.position_m for pose in poses[:BAFE_DRAW_SAMPLE_COUNT]],
            dtype=float,
        )
        ranges = np.ptp(positions, axis=0)
        assert ranges == pytest.approx(expected_ranges_m[trajectory_id], abs=1.0e-4)

        steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        velocity = np.gradient(positions, dt_s, axis=0, edge_order=2)
        acceleration = np.gradient(velocity, dt_s, axis=0, edge_order=2)
        jerk = np.gradient(acceleration, dt_s, axis=0, edge_order=2)
        assert float(np.max(steps)) < 0.002
        assert float(np.max(np.linalg.norm(velocity, axis=1))) < 0.25
        assert float(np.max(np.linalg.norm(acceleration, axis=1))) < 3.0
        assert float(np.max(np.linalg.norm(jerk, axis=1))) < 10.0

        assert poses[-1].time_s == pytest.approx(
            (BAFE_DRAW_DURATION_NS + BAFE_HOLD_DURATION_NS) / 1_000_000_000
        )


def test_generated_samples_follow_minimum_jerk_scalar_progress() -> None:
    trajectories = build_acceptance_trajectories()
    assert tuple(trajectories) == ACCEPTANCE_TRAJECTORY_IDS

    for trajectory_id, poses in trajectories.items():
        for index, pose in enumerate(poses[:DRAW_SAMPLE_COUNT]):
            progress = minimum_jerk_progress(index / DRAW_INTERVAL_COUNT)
            assert pose.position_m == pytest.approx(
                acceptance_geometry_position(trajectory_id, progress),
                abs=1.0e-15,
            )


def test_acceptance_geometries_have_the_required_centered_dimensions() -> None:
    line_start = acceptance_geometry_position("horizontal_line_mj", 0.0)
    line_end = acceptance_geometry_position("horizontal_line_mj", 1.0)
    assert line_start == (-0.04, 0.0, 0.0)
    assert line_end == (0.04, 0.0, 0.0)

    circle_cardinals = tuple(
        acceptance_geometry_position("circle_mj", progress)
        for progress in (0.0, 0.25, 0.5, 0.75, 1.0)
    )
    expected_circle_cardinals = (
        (0.0, 0.035, 0.0),
        (0.035, 0.0, 0.0),
        (0.0, -0.035, 0.0),
        (-0.035, 0.0, 0.0),
        (0.0, 0.035, 0.0),
    )
    for actual, expected in zip(
        circle_cardinals,
        expected_circle_cardinals,
        strict=True,
    ):
        assert actual == pytest.approx(expected, abs=1.0e-15)

    ellipse_cardinals = tuple(
        acceptance_geometry_position("ellipse_mj", progress)
        for progress in (0.0, 0.25, 0.5, 0.75, 1.0)
    )
    expected_ellipse_cardinals = (
        (0.0, 0.025, 0.0),
        (0.045, 0.0, 0.0),
        (0.0, -0.025, 0.0),
        (-0.045, 0.0, 0.0),
        (0.0, 0.025, 0.0),
    )
    for actual, expected in zip(
        ellipse_cardinals,
        expected_ellipse_cardinals,
        strict=True,
    ):
        assert actual == pytest.approx(expected, abs=1.0e-15)

    shallow_s_cardinals = tuple(
        acceptance_geometry_position("shallow_s_mj", progress)
        for progress in (0.0, 0.25, 0.5, 0.75, 1.0)
    )
    expected_shallow_s_cardinals = (
        (-0.04, 0.0, 0.0),
        (-0.02, 0.015, 0.0),
        (0.0, 0.0, 0.0),
        (0.02, -0.015, 0.0),
        (0.04, 0.0, 0.0),
    )
    for actual, expected in zip(
        shallow_s_cardinals,
        expected_shallow_s_cardinals,
        strict=True,
    ):
        assert actual == pytest.approx(expected, abs=1.0e-15)


def test_minimum_jerk_endpoint_velocity_and_acceleration_factors_are_zero() -> None:
    assert minimum_jerk_progress(0.0) == 0.0
    assert minimum_jerk_progress(1.0) == 1.0
    assert minimum_jerk_velocity_factor(0.0) == 0.0
    assert minimum_jerk_velocity_factor(1.0) == 0.0
    assert minimum_jerk_acceleration_factor(0.0) == 0.0
    assert minimum_jerk_acceleration_factor(1.0) == 0.0


@pytest.mark.parametrize("trajectory_id", ACCEPTANCE_TRAJECTORY_IDS)
def test_acceptance_assets_pass_the_fixed_time_compiler_contract(
    repository_root: Path,
    environment,
    trajectory_id: str,
) -> None:
    source = read_timed_pose_csv(
        _trajectory_directory(repository_root) / f"{trajectory_id}.csv"
    )
    adapter = FakeCuroboAdapter(environment)

    result = CuroboTimedPathCompiler(environment, adapter).compile(source)

    assert adapter.warmup_calls == 1
    assert adapter.ik_calls == 1
    assert result.metrics.uniform_time_scale == 1.0
    assert result.trajectory.duration_s == 33.0
    compiled_times = {sample.time_s for sample in result.trajectory.samples}
    assert all(pose.time_s in compiled_times for pose in source)
    assert result.trajectory.samples[0].velocities == (0.0,) * 6
    assert result.trajectory.samples[0].accelerations == (0.0,) * 6
    assert result.trajectory.samples[-1].velocities == (0.0,) * 6
    assert result.trajectory.samples[-1].accelerations == (0.0,) * 6


@pytest.mark.parametrize("trajectory_id", RECOVERY_TRAJECTORY_IDS)
def test_recovery_asset_passes_the_same_fixed_time_compiler_contract(
    repository_root: Path,
    environment,
    trajectory_id: str,
) -> None:
    source = read_timed_pose_csv(
        _trajectory_directory(repository_root) / f"{trajectory_id}.csv"
    )
    adapter = FakeCuroboAdapter(environment)

    result = CuroboTimedPathCompiler(environment, adapter).compile(source)

    assert result.metrics.uniform_time_scale == 1.0
    assert result.trajectory.duration_s == 63.0
    compiled_times = {sample.time_s for sample in result.trajectory.samples}
    assert all(pose.time_s in compiled_times for pose in source)
    assert result.trajectory.samples[0].velocities == (0.0,) * 6
    assert result.trajectory.samples[0].accelerations == (0.0,) * 6
    assert result.trajectory.samples[-1].velocities == (0.0,) * 6
    assert result.trajectory.samples[-1].accelerations == (0.0,) * 6


@pytest.mark.parametrize("trajectory_id", BAFE_STARTUP_TRAJECTORY_IDS)
def test_bafe_startup_assets_pass_the_fixed_time_compiler_contract(
    repository_root: Path,
    environment,
    trajectory_id: str,
) -> None:
    source = read_timed_pose_csv(
        _trajectory_directory(repository_root) / f"{trajectory_id}.csv"
    )
    adapter = FakeCuroboAdapter(environment)

    result = CuroboTimedPathCompiler(environment, adapter).compile(source)

    assert result.metrics.uniform_time_scale == 1.0
    assert result.trajectory.duration_s == pytest.approx(7.8)
    compiled_times = {sample.time_s for sample in result.trajectory.samples}
    assert all(pose.time_s in compiled_times for pose in source)
    assert result.trajectory.samples[0].velocities == (0.0,) * 6
    assert result.trajectory.samples[0].accelerations == (0.0,) * 6
    assert result.trajectory.samples[-1].velocities == (0.0,) * 6
    assert result.trajectory.samples[-1].accelerations == (0.0,) * 6


@pytest.mark.parametrize("trajectory_id", BAFE_DOWNWARD_TRAJECTORY_IDS)
def test_bafe_downward_assets_pass_the_fixed_time_compiler_contract(
    repository_root: Path,
    environment,
    trajectory_id: str,
) -> None:
    source = read_timed_pose_csv(
        _trajectory_directory(repository_root) / f"{trajectory_id}.csv"
    )
    adapter = FakeCuroboAdapter(environment)

    result = CuroboTimedPathCompiler(environment, adapter).compile(source)

    assert result.metrics.uniform_time_scale == 1.0
    assert result.trajectory.duration_s == pytest.approx(7.8)
    compiled_times = {sample.time_s for sample in result.trajectory.samples}
    assert all(pose.time_s in compiled_times for pose in source)
    assert result.trajectory.samples[0].velocities == (0.0,) * 6
    assert result.trajectory.samples[0].accelerations == (0.0,) * 6
    assert result.trajectory.samples[-1].velocities == (0.0,) * 6
    assert result.trajectory.samples[-1].accelerations == (0.0,) * 6


@pytest.mark.parametrize("trajectory_id", BAFE_DOWNWARD_MJ_TRAJECTORY_IDS)
def test_bafe_downward_mj_assets_pass_the_fixed_time_compiler_contract(
    repository_root: Path,
    environment,
    trajectory_id: str,
) -> None:
    source = read_timed_pose_csv(
        _trajectory_directory(repository_root) / f"{trajectory_id}.csv"
    )
    adapter = FakeCuroboAdapter(environment)

    result = CuroboTimedPathCompiler(environment, adapter).compile(source)

    assert result.metrics.uniform_time_scale == 1.0
    assert result.trajectory.duration_s == pytest.approx(33.0)
    compiled_times = {sample.time_s for sample in result.trajectory.samples}
    assert all(pose.time_s in compiled_times for pose in source)
    assert result.trajectory.samples[0].velocities == (0.0,) * 6
    assert result.trajectory.samples[0].accelerations == (0.0,) * 6
    assert result.trajectory.samples[-1].velocities == (0.0,) * 6
    assert result.trajectory.samples[-1].accelerations == (0.0,) * 6


@pytest.mark.parametrize("trajectory_id", BAFE_GROUND_MJ_TRAJECTORY_IDS)
def test_bafe_ground_mj_assets_pass_the_fixed_time_compiler_contract(
    repository_root: Path,
    trajectory_id: str,
) -> None:
    environment = EnvironmentBuilder().load(
        repository_root / "environments" / "bafe_ground_p10_rod194"
    )
    source = read_timed_pose_csv(
        _ground_trajectory_directory(repository_root) / f"{trajectory_id}.csv"
    )
    adapter = FakeCuroboAdapter(environment)

    result = CuroboTimedPathCompiler(environment, adapter).compile(source)

    assert result.metrics.uniform_time_scale == 1.0
    assert result.trajectory.duration_s == pytest.approx(33.0)
    compiled_times = {sample.time_s for sample in result.trajectory.samples}
    assert all(pose.time_s in compiled_times for pose in source)
    assert result.trajectory.samples[0].velocities == (0.0,) * 6
    assert result.trajectory.samples[0].accelerations == (0.0,) * 6
    assert result.trajectory.samples[-1].velocities == (0.0,) * 6
    assert result.trajectory.samples[-1].accelerations == (0.0,) * 6


@pytest.mark.parametrize("trajectory_id", BAFE_GROUND_MJ_V2_TRAJECTORY_IDS)
def test_bafe_ground_mj_v2_assets_pass_the_fixed_time_compiler_contract(
    repository_root: Path,
    trajectory_id: str,
) -> None:
    environment = EnvironmentBuilder().load(
        repository_root / "environments" / "bafe_ground_p10_rod194"
    )
    source = read_timed_pose_csv(
        _ground_trajectory_directory(repository_root) / f"{trajectory_id}.csv"
    )
    adapter = FakeCuroboAdapter(environment)

    result = CuroboTimedPathCompiler(environment, adapter).compile(source)

    assert result.metrics.uniform_time_scale == 1.0
    assert result.trajectory.duration_s == pytest.approx(123.0)
    compiled_times = {sample.time_s for sample in result.trajectory.samples}
    assert all(pose.time_s in compiled_times for pose in source)
    assert result.trajectory.samples[0].velocities == (0.0,) * 6
    assert result.trajectory.samples[0].accelerations == (0.0,) * 6
    assert result.trajectory.samples[-1].velocities == (0.0,) * 6
    assert result.trajectory.samples[-1].accelerations == (0.0,) * 6
