from __future__ import annotations

import csv
from decimal import Decimal
from pathlib import Path

import numpy as np
import pytest

from conftest import FakeCuroboAdapter

from hx_waypoint_curobo.contact_policy import ContactPolicy
from hx_waypoint_curobo.environment_builder import EnvironmentBuilder, EnvironmentBundle
from hx_waypoint_curobo.math3d import pose_matrix
from hx_waypoint_curobo.timed_pose_compiler import CuroboTimedPathCompiler
from hx_waypoint_curobo.trajectory_io import read_timed_pose_csv
from hx_waypoint_tools.trajectory_generator import (
    NEW_BOARD_DRAW_DURATION_NS,
    NEW_BOARD_DRAW_INTERVAL_COUNT,
    NEW_BOARD_DRAW_INTERVAL_NS,
    NEW_BOARD_DRAW_SAMPLE_COUNT,
    NEW_BOARD_FIG14_TRAJECTORY_FILENAMES,
    NEW_BOARD_FIG14_TRAJECTORY_IDS,
    NEW_BOARD_FRAME_ID,
    NEW_BOARD_HOLD_DURATION_NS,
    NEW_BOARD_QUATERNION_XYZW,
    build_new_board_fig14_trajectory,
    build_new_board_fig14_trajectories,
    generate_new_board_fig14_trajectories,
    minimum_jerk_progress,
    new_board_fig14_geometry_position,
    render_new_board_fig14_trajectory_csv,
)


POSE_COLUMNS = (
    "frame_id",
    "tip_x_m",
    "tip_y_m",
    "tip_z_m",
    "tip_qx",
    "tip_qy",
    "tip_qz",
    "tip_qw",
)


def _trajectory_directory(repository_root: Path) -> Path:
    return (
        repository_root
        / "environments"
        / "new_board_4cm_rod33365"
        / "trajectories"
    )


def _environment(repository_root: Path) -> EnvironmentBundle:
    return EnvironmentBuilder().load(
        repository_root / "environments" / "new_board_4cm_rod33365"
    )


def test_checked_in_new_board_csvs_byte_match_non_overwriting_generator(
    repository_root: Path,
    tmp_path: Path,
) -> None:
    generated_paths = generate_new_board_fig14_trajectories(tmp_path)

    assert tuple(path.name for path in generated_paths) == (
        NEW_BOARD_FIG14_TRAJECTORY_FILENAMES
    )
    for generated_path in generated_paths:
        trajectory_id = generated_path.stem
        checked_in_path = _trajectory_directory(repository_root) / generated_path.name
        assert generated_path.read_bytes() == render_new_board_fig14_trajectory_csv(
            trajectory_id
        )
        assert checked_in_path.read_bytes() == generated_path.read_bytes()

    assert generate_new_board_fig14_trajectories(tmp_path) == generated_paths


def test_new_board_generator_refuses_to_replace_authority(tmp_path: Path) -> None:
    path = tmp_path / NEW_BOARD_FIG14_TRAJECTORY_FILENAMES[0]
    path.write_bytes(b"different authoritative bytes\n")

    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        generate_new_board_fig14_trajectories(tmp_path)

    assert path.read_bytes() == b"different authoritative bytes\n"


def test_new_board_generator_preflights_every_authority_before_writing(
    tmp_path: Path,
) -> None:
    first_path = tmp_path / NEW_BOARD_FIG14_TRAJECTORY_FILENAMES[0]
    second_path = tmp_path / NEW_BOARD_FIG14_TRAJECTORY_FILENAMES[1]
    second_path.write_bytes(b"different authoritative bytes\n")

    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        generate_new_board_fig14_trajectories(tmp_path)

    assert not first_path.exists()
    assert second_path.read_bytes() == b"different authoritative bytes\n"


@pytest.mark.parametrize("trajectory_id", NEW_BOARD_FIG14_TRAJECTORY_IDS)
def test_new_board_schema_timing_minimum_jerk_and_hold_are_exact(
    repository_root: Path,
    trajectory_id: str,
) -> None:
    path = _trajectory_directory(repository_root) / f"{trajectory_id}.csv"
    poses = read_timed_pose_csv(path)
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))

    assert len(rows) == len(poses) == NEW_BOARD_DRAW_SAMPLE_COUNT + 1
    draw_time_ns = [
        int(Decimal(row["time_s"]) * Decimal(1_000_000_000))
        for row in rows[:NEW_BOARD_DRAW_SAMPLE_COUNT]
    ]
    assert draw_time_ns == [
        index * NEW_BOARD_DRAW_INTERVAL_NS
        for index in range(NEW_BOARD_DRAW_SAMPLE_COUNT)
    ]
    assert draw_time_ns[-1] == NEW_BOARD_DRAW_DURATION_NS
    assert Decimal(rows[-2]["time_s"]) == Decimal("36.000000000")
    assert Decimal(rows[-1]["time_s"]) == Decimal("39.000000000")
    assert [row["phase"] for row in rows] == [
        "draw"
    ] * NEW_BOARD_DRAW_SAMPLE_COUNT + ["hold"]
    assert tuple(rows[-1][column] for column in POSE_COLUMNS) == tuple(
        rows[-2][column] for column in POSE_COLUMNS
    )
    assert poses[-1].position_m == poses[-2].position_m
    assert poses[-1].quaternion_xyzw == poses[-2].quaternion_xyzw
    assert {pose.frame_id for pose in poses} == {NEW_BOARD_FRAME_ID}
    assert {pose.quaternion_xyzw for pose in poses} == {
        NEW_BOARD_QUATERNION_XYZW
    }

    for index, pose in enumerate(poses[:NEW_BOARD_DRAW_SAMPLE_COUNT]):
        expected = new_board_fig14_geometry_position(
            trajectory_id,
            minimum_jerk_progress(index / NEW_BOARD_DRAW_INTERVAL_COUNT),
        )
        assert pose.position_m == pytest.approx(expected, abs=5.1e-10)


def test_new_board_geometries_are_clear_simple_and_inside_the_safe_board_region(
    repository_root: Path,
) -> None:
    expected_ranges_m = {
        "fig14_figure8_3d_mj": (0.08, 0.05, 0.016),
        "fig14_zigzag_s_curve_3d_mj": (0.09, 0.06, 0.014),
    }
    environment = _environment(repository_root)
    policy = ContactPolicy.from_environment(environment)
    base_from_plane = environment._frame_to_base(NEW_BOARD_FRAME_ID)
    plane_from_tip = pose_matrix((0.0, 0.0, 0.0), NEW_BOARD_QUATERNION_XYZW)
    rod_axis_in_base = (base_from_plane @ plane_from_tip)[:3, :3] @ np.array(
        [0.0, 0.0, 1.0]
    )

    assert float(rod_axis_in_base @ np.array([0.0, 0.0, -1.0])) > 0.9999
    assert float(rod_axis_in_base @ np.asarray(policy.plane_normal_base)) < -0.9999

    for trajectory_id, poses in build_new_board_fig14_trajectories().items():
        positions = np.asarray(
            [pose.position_m for pose in poses[:NEW_BOARD_DRAW_SAMPLE_COUNT]],
            dtype=float,
        )
        assert np.ptp(positions, axis=0) == pytest.approx(
            expected_ranges_m[trajectory_id],
            abs=1.0e-4,
        )
        assert float(np.max(np.abs(positions[:, 0]))) <= 0.045 + 1.0e-12
        assert float(np.max(np.abs(positions[:, 1]))) <= 0.03 + 1.0e-12
        assert float(np.min(positions[:, 2])) >= -0.058 - 1.0e-12
        assert float(np.max(positions[:, 2])) <= -0.042 + 1.0e-12

        # The authoritative no-cross boundary is plane1 z=-0.015 m. Every
        # sample stays on the robot side with at least 27 mm of tip clearance.
        clearance_to_boundary_m = -positions[:, 2] - 0.015
        assert float(np.min(clearance_to_boundary_m)) >= 0.027 - 1.0e-12


def test_new_board_geometry_topology_remains_figure8_and_short_serpentine() -> None:
    local_progress = np.linspace(0.0, 1.0 / 3.0, 101)
    first_loop = np.asarray(
        [new_board_fig14_geometry_position("fig14_figure8_3d_mj", s) for s in local_progress]
    )
    second_loop = np.asarray(
        [
            new_board_fig14_geometry_position("fig14_figure8_3d_mj", s + 1.0 / 3.0)
            for s in local_progress
        ]
    )
    third_loop = np.asarray(
        [
            new_board_fig14_geometry_position("fig14_figure8_3d_mj", s + 2.0 / 3.0)
            for s in local_progress
        ]
    )
    assert second_loop == pytest.approx(first_loop, abs=1.0e-12)
    assert third_loop == pytest.approx(first_loop, abs=1.0e-12)
    for crossing_index in range(7):
        crossing = new_board_fig14_geometry_position(
            "fig14_figure8_3d_mj",
            crossing_index / 6.0,
        )
        assert crossing[:2] == pytest.approx((0.0, 0.0), abs=1.0e-15)

    zigzag_progress = np.linspace(0.0, 1.0, 1001)
    zigzag = np.asarray(
        [
            new_board_fig14_geometry_position("fig14_zigzag_s_curve_3d_mj", s)
            for s in zigzag_progress
        ]
    )
    assert np.all(np.diff(zigzag[:, 1]) > 0.0)
    extrema = tuple(
        new_board_fig14_geometry_position(
            "fig14_zigzag_s_curve_3d_mj",
            (1 + 2 * index) / 12.0,
        )[0]
        for index in range(6)
    )
    assert extrema == pytest.approx((0.045, -0.045) * 3, abs=1.0e-15)


@pytest.mark.parametrize("trajectory_id", NEW_BOARD_FIG14_TRAJECTORY_IDS)
def test_new_board_cartesian_motion_is_slow_smooth_and_has_no_jump(
    trajectory_id: str,
) -> None:
    poses = build_new_board_fig14_trajectory(trajectory_id)
    positions = np.asarray(
        [pose.position_m for pose in poses[:NEW_BOARD_DRAW_SAMPLE_COUNT]],
        dtype=float,
    )
    dt_s = NEW_BOARD_DRAW_INTERVAL_NS / 1_000_000_000
    steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    velocity = np.gradient(positions, dt_s, axis=0, edge_order=2)
    acceleration = np.gradient(velocity, dt_s, axis=0, edge_order=2)
    jerk = np.gradient(acceleration, dt_s, axis=0, edge_order=2)

    assert float(np.max(steps)) < 0.005
    assert float(steps[0]) < 1.0e-6
    assert float(steps[-1]) < 1.0e-6
    assert float(np.max(np.linalg.norm(velocity, axis=1))) < 0.08
    assert float(np.max(np.linalg.norm(acceleration, axis=1))) < 0.12
    assert float(np.max(np.linalg.norm(jerk, axis=1))) < 0.25


@pytest.mark.parametrize("trajectory_id", NEW_BOARD_FIG14_TRAJECTORY_IDS)
def test_new_board_assets_pass_the_fixed_time_compiler_contract(
    repository_root: Path,
    trajectory_id: str,
) -> None:
    environment = _environment(repository_root)
    source = read_timed_pose_csv(
        _trajectory_directory(repository_root) / f"{trajectory_id}.csv"
    )
    adapter = FakeCuroboAdapter(environment)

    result = CuroboTimedPathCompiler(environment, adapter).compile(source)

    assert adapter.warmup_calls == 1
    assert adapter.ik_calls == 1
    assert result.source_poses == source
    assert result.metrics.uniform_time_scale == 1.0
    assert result.trajectory.duration_s == pytest.approx(
        (NEW_BOARD_DRAW_DURATION_NS + NEW_BOARD_HOLD_DURATION_NS)
        / 1_000_000_000
    )
    compiled_times = {sample.time_s for sample in result.trajectory.samples}
    assert all(pose.time_s in compiled_times for pose in source)
    assert result.trajectory.samples[0].velocities == (0.0,) * 6
    assert result.trajectory.samples[0].accelerations == (0.0,) * 6
    assert result.trajectory.samples[-1].velocities == (0.0,) * 6
    assert result.trajectory.samples[-1].accelerations == (0.0,) * 6
