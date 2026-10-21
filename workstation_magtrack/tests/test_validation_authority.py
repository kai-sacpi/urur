"""Meaningful independent checks for time coverage and9DOF observability."""

import csv
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from workstation_magtrack.validation.audit import FIELDS, coarse_rod_clearance, continuous_bounds, observability, read_authority, weighted_bins


def test_time_weighting_splits_crossings_and_includes_final_static_boundary():
    result = weighted_bins(np.array([0., 4., 6.]), np.array([.14, .18, .18]), np.array([.14, .16, .18]))
    np.testing.assert_allclose(result, [2., 4.])


def test_nominal_two_axis_geometry_has9dof_single_axis8_and_static6():
    points = np.array([[x, y, z] for x in [-.1, 0, .1] for y in [-.1, 0, .1] for z in [-.075, -.08, -.1]])
    centres = np.repeat(points, 5, axis=0)
    attitudes = Rotation.from_euler("xy", [[0, 0], [20, 0], [-20, 0], [0, 20], [0, -20]], degrees=True).as_quat()
    q = np.tile(attitudes, (len(points), 1))
    assert observability(centres, q)["rank"] == 9
    yaw = Rotation.from_rotvec(np.column_stack([np.zeros((len(centres), 2)), np.deg2rad(np.linspace(-40, 40, len(centres)))])).as_quat()
    assert observability(centres, yaw)["rank"] == 8
    assert observability(centres, np.tile([0, 0, 0, 1.], (len(centres), 1)))["rank"] == 6


def _csv(tmp_path: Path, *, moving_hold=False, bad_time=False):
    path = tmp_path / "environments/mla_short/trajectories/test.csv"
    path.parent.mkdir(parents=True)
    with path.open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(FIELDS)
        for i in range(5):
            writer.writerow(["0.1000000001" if bad_time and i == 1 else f"{i*.1:.9f}", "plane1", .001 if moving_hold and i == 2 else 0, 0, -.1, 0, 0, 0, 1, "draw" if i == 0 else "hold"])
    return {"trajectory_id": "test", "source_file": str(path)}


def test_hold_must_be_exactly_stationary(tmp_path):
    with pytest.raises(ValueError, match="hold rows move"):
        read_authority(tmp_path, _csv(tmp_path, moving_hold=True))


def test_time_must_roundtrip_ros_nanoseconds_without_retiming(tmp_path):
    with pytest.raises(ValueError, match="exactly representable"):
        read_authority(tmp_path, _csv(tmp_path, bad_time=True))


def test_authority_sha_is_checked(tmp_path):
    record = _csv(tmp_path)
    record["source_sha256"] = "0" * 64
    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        read_authority(tmp_path, record)


def test_unchanged_board_boundary_rejects60mm_under_coarse_rod_envelope():
    attitudes = Rotation.from_euler("xy", [[0., 0.], [20., 0.], [0., 20.]], degrees=True).as_quat()
    too_low = coarse_rod_clearance(np.tile([0., 0., -.060], (3, 1)), attitudes)
    redesigned = coarse_rod_clearance(np.tile([0., 0., -.075], (3, 1)), attitudes)
    assert not too_low["passed"]
    assert redesigned["passed"]
    assert redesigned["unchanged_board_keepout_top_z_m"] == too_low["unchanged_board_keepout_top_z_m"]


def test_turning_bound_includes_centripetal_acceleration_and_curvature_jerk():
    record = {"category": "turn", "nominal_center_speed_m_s": .05,
              "geometry": {"speed_ramp_s": 3., "curvature_per_m": 10.,
                           "geometric_length_m": .2, "sweep_rad": 2.}}
    result = continuous_bounds(record, np.array([0., 1.]), np.zeros((2, 3)), np.tile([0., 0., 0., 1.], (2, 1)))
    expected_tangent_a = .05 * 2.1875 / 3
    assert result["bounds"]["max_acceleration_m_s2"] > expected_tangent_a
    assert result["bounds"]["max_jerk_m_s3"] > .05 * 7.513188404399293 / 9
