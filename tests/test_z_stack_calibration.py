from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np
import pytest
from conftest import copy_environment_without_recordings
from hx_waypoint_curobo.math3d import quaternion_to_matrix_xyzw
from hx_waypoint_curobo.trajectory_io import read_timed_pose_csv
from hx_waypoint_tools.z_stack_calibration import (
    Z_STACK_BASE_QUATERNION_XYZW,
    Z_STACK_CAPTURE_DURATION_S,
    Z_STACK_CLEARANCE_BOUNDARY_Z_M,
    Z_STACK_DISTANCES_M,
    Z_STACK_ENVIRONMENT_ID,
    Z_STACK_MINIMUM_TIP_DISTANCE_M,
    Z_STACK_SENSOR_PLANE_Z_M,
    Z_STACK_SETTLE_DURATION_S,
    Z_STACK_TILT_DEG,
    build_z_stack_tilt_experiments,
    fit_rigid_offsets,
    generate_z_stack_tilt_experiments,
    rigid_offset_design_matrix,
    validate_z_stack_tip_distances,
)


def _environment(repository_root: Path) -> Path:
    return repository_root / "environments" / Z_STACK_ENVIRONMENT_ID


def test_z_stack_schedule_respects_new_collision_boundary_and_exact_timing(
    repository_root: Path,
) -> None:
    center, x50 = build_z_stack_tilt_experiments(_environment(repository_root))

    assert center.site.site_id == "center"
    assert x50.site.site_id == "x50"
    assert len(center.poses) == len(x50.poses) == 931
    assert len(center.visits) == len(x50.visits) == 15
    assert center.poses[-1].time_s == x50.poses[-1].time_s == pytest.approx(105.0)
    assert center.visits[0].capture_start_time_s == pytest.approx(1.0)
    assert center.visits[0].capture_end_time_s == pytest.approx(4.0)
    assert center.visits[-1].arrival_time_s == pytest.approx(101.0)
    assert center.visits[-1].capture_start_time_s == pytest.approx(102.0)
    assert center.visits[-1].capture_end_time_s == pytest.approx(105.0)
    assert all(
        visit.settle_end_time_s - visit.arrival_time_s
        == pytest.approx(Z_STACK_SETTLE_DURATION_S)
        for visit in center.visits
    )
    assert all(
        visit.capture_end_time_s - visit.capture_start_time_s
        == pytest.approx(Z_STACK_CAPTURE_DURATION_S)
        for visit in center.visits
    )

    expected_distances = (
        0.068,
        0.076,
        0.084,
        0.092,
        0.092,
        0.092,
        0.092,
        0.092,
        0.092,
        0.092,
        0.092,
        0.092,
        0.084,
        0.076,
        0.068,
    )
    assert tuple(visit.distance_m for visit in center.visits) == expected_distances
    assert Z_STACK_DISTANCES_M == (0.068, 0.076, 0.084, 0.092, 0.084, 0.076, 0.068)
    assert max(pose.position_m[2] for pose in center.poses) == pytest.approx(-0.075)
    assert max(pose.position_m[2] for pose in center.poses) < (
        Z_STACK_CLEARANCE_BOUNDARY_Z_M
    )
    assert all(pose.position_m[0] == 0.0 for pose in center.poses)
    assert all(pose.position_m[1] == 0.0 for pose in center.poses)
    assert all(pose.position_m[0] == 0.050 for pose in x50.poses)
    assert all(pose.position_m[1] == 0.0 for pose in x50.poses)
    assert all(
        Z_STACK_SENSOR_PLANE_Z_M - pose.position_m[2]
        > Z_STACK_MINIMUM_TIP_DISTANCE_M
        for pose in center.poses
    )


def test_z_stack_tilt_set_is_full_rank_and_keeps_tip_fixed(
    repository_root: Path,
) -> None:
    center = build_z_stack_tilt_experiments(_environment(repository_root))[0]
    deepest = center.visits[3:12]

    assert all(visit.distance_m == 0.092 for visit in deepest)
    assert all(
        visit.tip_position_m == pytest.approx((0.0, 0.0, -0.099), abs=1.0e-12)
        for visit in deepest
    )
    assert deepest[0].quaternion_xyzw == Z_STACK_BASE_QUATERNION_XYZW
    sine = math.sin(math.radians(0.5 * Z_STACK_TILT_DEG))
    cosine = math.cos(math.radians(0.5 * Z_STACK_TILT_DEG))
    assert deepest[1].quaternion_xyzw == pytest.approx((sine, 0.0, 0.0, cosine))
    assert deepest[3].quaternion_xyzw == pytest.approx((-sine, 0.0, 0.0, cosine))
    assert deepest[5].quaternion_xyzw == pytest.approx((0.0, sine, 0.0, cosine))
    assert deepest[7].quaternion_xyzw == pytest.approx((0.0, -sine, 0.0, cosine))
    assert all(visit.quaternion_xyzw[2] == 0.0 for visit in center.visits)

    design = rigid_offset_design_matrix(center)
    assert design.shape == (27, 6)
    assert np.linalg.matrix_rank(design) == 6
    assert np.linalg.cond(design) < 10.0

    rotated_axes = tuple(
        quaternion_to_matrix_xyzw(visit.quaternion_xyzw)[:, 2]
        for visit in (deepest[0], deepest[1], deepest[5])
    )
    assert np.linalg.matrix_rank(np.column_stack(rotated_axes)) == 3


def test_rigid_offset_fit_recovers_synthetic_board_and_tool_offsets(
    repository_root: Path,
) -> None:
    center = build_z_stack_tilt_experiments(_environment(repository_root))[0]
    fit_visits = tuple(
        visit
        for visit in center.visits
        if visit.fit_role in {"z_stack_and_rigid", "rigid", "rigid_drift"}
    )
    rotations = np.asarray(
        [quaternion_to_matrix_xyzw(visit.quaternion_xyzw) for visit in fit_visits]
    )
    expected_b = np.asarray((-0.0015, 0.0117, 0.0019))
    expected_o = np.asarray((-0.0011, 0.0019, 0.0032))
    deltas = np.asarray([expected_b + rotation @ expected_o for rotation in rotations])

    fit = fit_rigid_offsets(rotations, deltas)

    assert fit.board_bias_m == pytest.approx(expected_b, abs=1.0e-12)
    assert fit.tool_offset_m == pytest.approx(expected_o, abs=1.0e-12)
    assert fit.rmse_m < 1.0e-14
    assert fit.maximum_residual_m < 1.0e-14
    assert fit.rank == 6
    assert fit.condition_number < 10.0

    with pytest.raises(ValueError, match="do not independently identify"):
        fit_rigid_offsets(
            np.repeat(np.eye(3)[None, :, :], 3, axis=0),
            np.repeat((expected_b + expected_o)[None, :], 3, axis=0),
        )


def test_z_stack_moves_use_minimum_jerk_and_holds_are_exact(
    repository_root: Path,
) -> None:
    center = build_z_stack_tilt_experiments(_environment(repository_root))[0]

    assert center.poses[0].phase == "draw"
    assert center.poses[1].time_s == pytest.approx(1.0)
    assert center.poses[1].phase == "hold"
    assert center.poses[2].time_s == pytest.approx(4.0)
    assert center.poses[2].phase == "hold"
    assert center.poses[1].position_m == center.poses[0].position_m
    assert center.poses[2].position_m == center.poses[1].position_m

    first_move = center.poses[3:33]
    assert first_move[0].time_s == pytest.approx(4.05)
    assert first_move[14].time_s == pytest.approx(4.75)
    assert first_move[-1].time_s == pytest.approx(5.5)
    assert first_move[14].position_m[2] == pytest.approx(-0.079, abs=1.0e-12)
    assert first_move[-1].position_m[2] == pytest.approx(-0.083, abs=1.0e-12)
    assert all(pose.phase == "draw" for pose in first_move)

    for previous, pose in zip(center.poses[:-1], center.poses[1:], strict=True):
        assert pose.time_s > previous.time_s
        if pose.phase == "hold":
            assert pose.position_m == previous.position_m
            assert pose.quaternion_xyzw == previous.quaternion_xyzw


def test_user_proposed_60mm_point_is_rejected_by_new_environment() -> None:
    assert validate_z_stack_tip_distances((0.068, 0.076)) == (0.068, 0.076)
    with pytest.raises(ValueError, match="65 mm hard boundary"):
        validate_z_stack_tip_distances((0.060, 0.068, 0.076, 0.084))


def test_generated_authorities_and_capture_windows_are_deterministic(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    target = copy_environment_without_recordings(
        _environment(repository_root),
        tmp_path / Z_STACK_ENVIRONMENT_ID,
    )
    trajectories = target / "trajectories"
    for path in tuple(trajectories.iterdir()):
        path.unlink()

    paths, capture_path = generate_z_stack_tilt_experiments(target)
    assert len(paths) == 2
    assert capture_path.name == "z_stack_tilt_calibration_capture_windows_v3.csv"
    assert all(len(read_timed_pose_csv(path)) == 931 for path in paths)
    assert generate_z_stack_tilt_experiments(target) == (paths, capture_path)
    checked_in_directory = _environment(repository_root) / "trajectories"
    for path in paths:
        assert path.read_bytes() == (checked_in_directory / path.name).read_bytes()
    assert capture_path.read_bytes() == (
        checked_in_directory / capture_path.name
    ).read_bytes()

    with capture_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 30
    assert {row["site_id"] for row in rows} == {"center", "x50"}
    assert all(row["environment_id"] == Z_STACK_ENVIRONMENT_ID for row in rows)
    assert all(
        float(row["capture_end_time_s"]) - float(row["capture_start_time_s"])
        == pytest.approx(3.0)
        for row in rows
    )

    capture_path.write_text("different\n", encoding="utf-8")
    with pytest.raises(FileExistsError, match="differing authority"):
        generate_z_stack_tilt_experiments(target)
