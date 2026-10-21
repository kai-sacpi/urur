from __future__ import annotations

import csv
import math
import shutil
from pathlib import Path

import numpy as np
import pytest
from conftest import FakeCuroboAdapter
from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_curobo.models import IKCandidate
from hx_waypoint_curobo.trajectory_io import read_timed_pose_csv

from hx_waypoint_tools import mla_grid


def _rotate_z_axis(quaternion_xyzw: tuple[float, float, float, float]) -> np.ndarray:
    x_value, y_value, z_value, w_value = quaternion_xyzw
    return np.asarray(
        (
            2.0 * (x_value * z_value + w_value * y_value),
            2.0 * (y_value * z_value - w_value * x_value),
            1.0 - 2.0 * (x_value * x_value + y_value * y_value),
        )
    )


def test_ordered_grid_has_exact_54_fixed_positions_and_serpentine_steps() -> None:
    targets = mla_grid.ordered_grid_targets()

    assert len(targets) == mla_grid.EXPECTED_TARGET_COUNT == 54
    assert len({target.condition_id for target in targets}) == 54
    assert {
        (target.x_A_m, target.y_A_m, target.z_A_m) for target in targets
    } == {
        (x_value, y_value, z_value)
        for x_value in mla_grid.GRID_COORDINATES_M
        for y_value in mla_grid.GRID_COORDINATES_M
        for z_value in mla_grid.GRID_HEIGHTS_M
    }
    assert all(
        target.tip_position_plane1_m == (target.x_A_m, target.y_A_m, -target.z_A_m)
        for target in targets
    )
    distances = [
        np.linalg.norm(
            np.asarray(right.tip_position_plane1_m)
            - np.asarray(left.tip_position_plane1_m)
        )
        for left, right in zip(targets[:-1], targets[1:], strict=True)
    ]
    assert max(distances) == pytest.approx(0.05, abs=1.0e-12)


@pytest.mark.parametrize(
    ("tilt_deg", "azimuth_deg"),
    ((0.0, 0.0), (5.0, 0.0), (10.0, 90.0), (25.0, 210.0)),
)
def test_tilt_quaternion_points_tool_axis_at_requested_direction(
    tilt_deg: float,
    azimuth_deg: float,
) -> None:
    quaternion = mla_grid.tilt_azimuth_quaternion_xyzw(tilt_deg, azimuth_deg)
    axis = _rotate_z_axis(quaternion)
    tilt_rad = math.radians(tilt_deg)
    azimuth_rad = math.radians(azimuth_deg)
    expected = np.asarray(
        (
            math.sin(tilt_rad) * math.cos(azimuth_rad),
            math.sin(tilt_rad) * math.sin(azimuth_rad),
            math.cos(tilt_rad),
        )
    )

    assert np.linalg.norm(quaternion) == pytest.approx(1.0, abs=1.0e-12)
    assert axis == pytest.approx(expected, abs=1.0e-12)


def test_orientation_catalog_searches_vertical_once_then_all_azimuths() -> None:
    vertical = mla_grid.orientation_candidates(0.0, azimuth_step_deg=30)
    tilted = mla_grid.orientation_candidates(5.0, azimuth_step_deg=30)

    assert len(vertical) == 1
    assert vertical[0].quaternion_xyzw == (0.0, 0.0, 0.0, 1.0)
    assert [candidate.azimuth_deg for candidate in tilted] == list(
        map(float, range(0, 360, 30))
    )


def test_choice_ranking_matches_required_lexicographic_priority() -> None:
    orientation_near = mla_grid.OrientationCandidate(
        5.0,
        0.0,
        mla_grid.tilt_azimuth_quaternion_xyzw(5.0, 0.0),
    )
    orientation_far = mla_grid.OrientationCandidate(
        5.0,
        180.0,
        mla_grid.tilt_azimuth_quaternion_xyzw(5.0, 180.0),
    )
    high_clearance = mla_grid.JointChoice(
        orientation_near,
        IKCandidate((0.0,) * 6, 0.020, 0.2, 50.0),
        orientation_change_deg=5.0,
        joint_change_rad=0.1,
    )
    low_clearance_but_otherwise_better = mla_grid.JointChoice(
        orientation_far,
        IKCandidate((0.1,) * 6, 0.019, 2.0, 1.0),
        orientation_change_deg=1.0,
        joint_change_rad=0.01,
    )

    ranked = sorted(
        (low_clearance_but_otherwise_better, high_clearance),
        key=mla_grid._choice_sort_key,
    )

    assert ranked[0] is high_clearance


def test_authority_is_exact_three_second_static_pose(tmp_path: Path) -> None:
    target = mla_grid.ordered_grid_targets()[0]
    orientation = mla_grid.orientation_candidates(10.0, azimuth_step_deg=30)[2]
    path = tmp_path / "authority.csv"
    path.write_bytes(mla_grid._authority_content(target, orientation))

    poses = read_timed_pose_csv(path)

    assert [pose.time_s for pose in poses] == [0.0, 1.0, 3.0]
    assert [pose.phase for pose in poses] == ["draw", "hold", "hold"]
    assert {pose.frame_id for pose in poses} == {"plane1"}
    assert all(pose.position_m == target.tip_position_plane1_m for pose in poses)
    assert all(pose.quaternion_xyzw == poses[0].quaternion_xyzw for pose in poses)
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert rows[-1]["time_s"] == "3.000000000"


def test_fake_search_publishes_all_54_authorities_without_changing_xyz(
    repository_root: Path,
    tmp_path: Path,
) -> None:
    workspace = tmp_path / "workspace"
    source_environment = repository_root / "environments" / "mla"
    environment_directory = workspace / "environments" / "mla"
    shutil.copytree(source_environment, environment_directory)
    environment = EnvironmentBuilder().load(environment_directory)
    adapter = FakeCuroboAdapter(environment)
    output = workspace / "artifacts" / "test_search"

    reachability, attempts, manifest = mla_grid.run_search(
        workspace=workspace,
        output_directory=output,
        adapter=adapter,
    )

    with reachability.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 54
    assert {row["status"] for row in rows} == {"REACHABLE"}
    assert {float(row["tilt_deg"]) for row in rows} == {0.0}
    assert all(float(row["tip_z_plane1_m"]) == -float(row["z_A_m"]) for row in rows)
    assert len(list((environment_directory / "trajectories").glob("mla_grid_*.csv"))) == 54
    assert attempts.is_file()
    assert manifest.is_file()
