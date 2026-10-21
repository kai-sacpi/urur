from __future__ import annotations

import csv
import json
import shutil
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from workstation_magtrack.generation.calibration import generate, low_height_precheck


@pytest.fixture(scope="module")
def generated(tmp_path_factory):
    root = tmp_path_factory.mktemp("workstation_calibration")
    source_env = Path(__file__).resolve().parents[2] / "environments" / "mla_short"
    dest_env = root / "environments" / "mla_short"
    dest_env.mkdir(parents=True)
    for name in ("setup.yaml", "environment.yaml", "zones.yaml"):
        shutil.copyfile(source_env / name, dest_env / name)
    return root, generate(root)


def _csv(root: Path, record: dict):
    with (root / record["source_file"]).open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    t = np.array([float(row["time_s"]) for row in rows])
    p = np.array([[float(row[f"tip_{axis}_m"]) for axis in "xyz"] for row in rows])
    q = np.array([[float(row[f"tip_q{axis}"]) for axis in "xyzw"] for row in rows])
    return rows, t, p, q


def test_unsafe_nominal_height_is_rejected_without_weakening_board():
    for height in (0.060, 0.070):
        assert not low_height_precheck(height)["accepted"]
    for height in (0.075, 0.080, 0.100):
        check = low_height_precheck(height)
        assert check["accepted"]
        assert check["worst_case_rod_surface_to_sensor_keepout_m"] >= 0.005
        assert check["analytical_tip_margin_m"] > 0
    assert low_height_precheck(0.070)["analytical_tip_margin_m"] < 0


def test_calibration_has_135_windows_and_whole_site_holdout_repeats(generated):
    root, records = generated
    calibration = [r for r in records if r["category"] == "calibration"]
    assert len(calibration) == 27
    windows = [w for r in calibration for w in r["capture_windows"]]
    assert len(windows) == 135
    assert sum(w["data_role"] == "geometry_holdout" for w in windows) == 30
    for site_id in {w["site_id"] for w in windows}:
        group = [w for w in windows if w["site_id"] == site_id]
        assert len(group) == 15
        assert len({w["data_role"] for w in group}) == 1
        assert {w["nominal_height_m"] for w in group} == {0.075, 0.080, 0.100}
        axes = Rotation.from_quat([w["quaternion_xyzw"] for w in group[:5]]).apply([0, 0, 1])
        assert np.linalg.matrix_rank(axes) == 3
    schedule = json.loads((root / "planning/calibration_repeat_schedule.json").read_text())
    assert len(schedule) == 54
    for record in calibration:
        repeat = [r for r in schedule if r["trajectory_id"] == record["trajectory_id"]]
        assert {r["repeat_index"] for r in repeat} == {1, 2}
        assert {r["data_role"] for r in repeat} == {record["data_role"]}
        assert {r["source_sha256"] for r in repeat} == {record["source_sha256"]}


def test_capture_and_settle_are_exactly_stationary(generated):
    root, records = generated
    for record in records:
        rows, times, positions, quaternions = _csv(root, record)
        assert times[0] == 0 and np.all(np.diff(times) > 0)
        assert rows[0]["phase"] == "draw"
        for index, row in enumerate(rows[1:], 1):
            if row["phase"] == "hold":
                assert np.array_equal(positions[index], positions[index - 1])
                assert np.array_equal(quaternions[index], quaternions[index - 1])
        for window in record.get("capture_windows", []):
            assert window["capture_start_time_s"] - window["arrival_time_s"] == pytest.approx(0.5)
            assert window["capture_end_time_s"] - window["capture_start_time_s"] == pytest.approx(1.0)
            mask = (times >= window["arrival_time_s"]) & (times <= window["capture_end_time_s"])
            assert np.max(np.abs(positions[mask] - positions[mask][0])) == 0
            assert np.max(np.abs(quaternions[mask] - quaternions[mask][0])) == 0


def test_synchronization_reverses_same_path_at_two_real_peaks(generated):
    root, records = generated
    sync = [r for r in records if r["category"] == "synchronization"]
    assert len(sync) == 6
    for axis in "xyz":
        pair = [r for r in sync if r["axis"] == axis]
        assert {r["specified_peak_speed_m_s"] for r in pair} == {0.005, 0.010}
        for record in pair:
            _, times, positions, quaternions = _csv(root, record)
            assert np.array_equal(positions[0], positions[-1])
            assert np.all(quaternions == np.array([0, 0, 0, 1]))
            moving = [segment for segment in record["phases"] if segment["phase"].startswith("synchronization_")]
            assert len(moving) == 2
            for segment in moving:
                bounds = segment["continuous_analytic_bounds"]
                assert bounds["center_max_speed_m_s"] == pytest.approx(record["specified_peak_speed_m_s"])
                start = np.flatnonzero(times == segment["start_time_s"])[0]
                end = np.flatnonzero(times == segment["end_time_s"])[0]
                assert np.linalg.norm(positions[end] - positions[start]) == pytest.approx(0.040)
            assert -positions[:, 2].max() >= 0.080 - 1e-12


def test_continuous_septic_motion_bounds_satisfy_generation_contract(generated):
    _, records = generated
    for record in records:
        for segment in record["phases"]:
            bounds = segment.get("continuous_analytic_bounds")
            if bounds is None:
                continue
            assert bounds["center_max_acceleration_m_s2"] <= 0.05
            assert bounds["center_max_jerk_m_s3"] <= 0.25
            assert bounds["flange_max_angular_acceleration_deg_s2"] <= 30
            assert bounds["flange_max_angular_jerk_deg_s3"] <= 150
            assert bounds["endpoint_center_velocity_acceleration_jerk_zero"]
            assert bounds["endpoint_angular_velocity_acceleration_jerk_zero"]


def test_generation_is_deterministic_and_authority_overwrite_is_rejected(generated):
    root, records = generated
    assert generate(root) == records
    source = root / records[0]["source_file"]
    original = source.read_bytes()
    source.write_bytes(original + b"\n")
    try:
        with pytest.raises(FileExistsError, match="refusing to overwrite"):
            generate(root)
    finally:
        source.write_bytes(original)
