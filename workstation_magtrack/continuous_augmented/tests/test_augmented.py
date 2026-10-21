"""Independent source contract and SO(3) regression tests (no ROS/device)."""
from __future__ import annotations

import copy
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("augmented_source_audit", ROOT/"audit.py")
assert SPEC is not None and SPEC.loader is not None
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


def metadata() -> dict:
    x = np.linspace(-.06, .06, 8)
    y = np.array([-.015, .005, .021, -.018, -.011, .023, .008, -.01])
    controls = np.c_[x, y, np.full(8, -.12)]
    euler = np.array([[.01, .06, .02], [.03, .08, .04], [-.01, .04, .08],
                      [.04, -.03, .10], [.06, .01, .04], [.02, .09, -.02],
                      [-.02, .04, -.07], [.01, .02, -.03]])
    return {"trajectory_id": "test_open", "seed": 42, "group_id": "fixed_height",
            "environment_id": "mla_short", "frame_id": "plane1", "position_units": "m",
            "quaternion_order": "xyzw", "height_definition": "height_m=-plane1_z_m",
            "source_sample_period_s": .05,
            "safety_status": {"simulation_only": True, "real_execution_authorized": False,
                              "source_authority_may_be_modified_by_runtime": False,
                              "hard_collision_clearance_m": .005, "rod_length_m": .190, "rod_radius_m": .0025},
            "family_id": "elastic_open", "height_profile": {"mode": "fixed", "lower_m": .12, "upper_m": .12},
            "curve": {"position_control_points_m": controls.tolist(), "longitudinal_axis_xyz": [1., 0., 0.], "degree": 7},
            "time_warp": {"density_control_points_positive": [.8, 1.2, .9, 1.1, 1.]},
            "orientation": {"mode": "smooth_multi_axis", "euler_xyz_control_points_rad": euler.tolist(),
                            "composition": "Rz(yaw) Ry(pitch) Rx(roll)"},
            "draw_duration_s": 30., "hold_start_s": .5, "hold_end_s": .5}


def write_fixture(root: Path, meta: dict | None = None) -> dict:
    meta = metadata() if meta is None else meta
    (root/"csv").mkdir(exist_ok=True)
    (root/"metadata").mkdir(exist_ok=True)
    duration = meta["draw_duration_s"]
    t = np.arange(round((duration+1.)*20)+1)/20.
    s = np.clip((t-.5)/duration, 0., 1.)
    model = audit.reconstruct(meta, s)
    positions = model["position_derivatives"][0]
    quaternions = model["quaternion"]
    # Copy original serialized endpoint strings into hold rows exactly.
    pose_fields = [[f"{v:.15f}" for v in np.r_[p, q]] for p, q in zip(positions, quaternions)]
    for i in np.flatnonzero(t <= .5+1e-12):
        pose_fields[int(i)] = list(pose_fields[0])
    for i in np.flatnonzero(t >= duration+.5-1e-12):
        pose_fields[int(i)] = list(pose_fields[-1])
    csv_path = root/"csv/test.csv"
    with csv_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(audit.FIELDS)
        for i, time in enumerate(t):
            writer.writerow([f"{time:.2f}", "plane1", *pose_fields[i],
                             "draw" if i == 0 or .5 < time <= duration+.5 else "hold"])
    meta_path = root/"metadata/test.json"
    meta["authority"] = {"csv_file": "csv/test.csv", "sha256": audit.sha256(csv_path),
                         "source_pose_count": len(t), "duration_s": duration+1.}
    meta_path.write_text(json.dumps(meta, indent=2)+"\n")
    return {"trajectory_id": meta["trajectory_id"], "csv_file": "csv/test.csv", "metadata_file": "metadata/test.json",
            "sha256": audit.sha256(csv_path), "metadata_sha256": audit.sha256(meta_path),
            "duration_s": duration+1., "seed": meta["seed"], "group_id": meta["group_id"]}


class AugmentedMathematicsTests(unittest.TestCase):
    def test_positive_density_is_required_not_grid_tested(self):
        meta = metadata()
        meta["time_warp"]["density_control_points_positive"][2] = -.1
        with self.assertRaisesRegex(ValueError, "strictly positive"):
            audit.reconstruct(meta, np.linspace(0., 1., 20))

    def test_progress_is_positive_interior_and_C3_stationary_at_endpoints(self):
        model = audit.reconstruct(metadata(), np.linspace(0., 1., 4097))
        self.assertTrue(np.all(np.diff(model["progress"]) > -1e-14))
        self.assertTrue(np.all(model["progress_derivative"][1:-1] > 0.))
        endpoints = audit.reconstruct(metadata(), np.array([0., 1.]))
        for value in endpoints["position_derivatives"][1:4]+endpoints["angular_derivatives_normalized"][:3]:
            np.testing.assert_allclose(value, 0., atol=1e-13)

    def test_SO3_matrix_jets_match_independent_rotation_increments(self):
        result = audit.so3_finite_difference_diagnostic(metadata())
        self.assertTrue(result["formula_cross_check_pass"], result)
        self.assertEqual(len(result["max_vector_errors_rad_s_orders_1_3"]), 3)

    def test_noncommutative_SO3_is_not_Euler_angle_derivative(self):
        meta = metadata()
        controls = np.asarray(meta["orientation"]["euler_xyz_control_points_rad"])*10.
        meta["orientation"]["euler_xyz_control_points_rad"] = controls.tolist()
        s = np.array([.35, .55, .75])
        model = audit.reconstruct(meta, s)
        u = audit.timing_model(meta, s)[0]
        euler_derivative = audit._compose(controls, u)[1]
        self.assertGreater(np.max(np.abs(model["angular_derivatives_normalized"][0]-euler_derivative)), .01)
        euler_angles = audit._compose(controls, u)[0]
        expected = Rotation.from_euler("xyz", euler_angles)
        error = (expected*Rotation.from_quat(model["quaternion"]).inv()).magnitude()
        np.testing.assert_allclose(error, 0., atol=1e-12)

    def test_Bernstein_projection_proof_rejects_closed_or_backward_path(self):
        meta = metadata()
        self.assertTrue(audit.path_certificate(meta)["passed"])
        meta["curve"]["position_control_points_m"][-1] = meta["curve"]["position_control_points_m"][0]
        self.assertFalse(audit.path_certificate(meta)["strictly_monotone_projection"])

    def test_height_and_XY_box_are_continuous_control_hull_checks(self):
        meta = metadata()
        meta["curve"]["position_control_points_m"][3][0] = .14
        self.assertFalse(audit.path_certificate(meta)["xy_box_pass"])
        meta = metadata()
        meta["curve"]["position_control_points_m"][3][2] = -.13
        self.assertFalse(audit.path_certificate(meta)["height_contract_pass"])

    def test_full_height_is_one_way_not_repeated_traversal(self):
        meta = metadata()
        z = -np.linspace(.07, .20, 8)
        for point, height in zip(meta["curve"]["position_control_points_m"], z):
            point[2] = float(height)
        meta["height_profile"] = {"mode": "full_height_one_way", "lower_m": .07, "upper_m": .20}
        self.assertTrue(audit.path_certificate(meta)["passed"])
        meta["curve"]["position_control_points_m"][4][2] = -.08
        self.assertFalse(audit.path_certificate(meta)["height_contract_pass"])

    def test_continuous_limits_reject_unsafe_duration_without_retiming(self):
        meta = metadata()
        self.assertTrue(audit.continuous_certificate(meta, samples=1025)["passed"])
        meta["draw_duration_s"] = .5
        before = copy.deepcopy(meta)
        self.assertFalse(audit.continuous_certificate(meta, samples=1025)["passed"])
        self.assertEqual(meta, before)


class AugmentedAuthorityTests(unittest.TestCase):
    def test_source_contract_and_authority_pass_without_robot_claim(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            result = audit.audit_record(root, write_fixture(root), samples=1025)
            self.assertTrue(result["source_level_pass"], result["errors"])
            self.assertEqual(result["robot_simulation"], "not_run")
            self.assertFalse(result["runtime_ready"])
            self.assertFalse(result["real_execution_authorized"])

    def test_hash_mutation_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            record = write_fixture(root)
            path = root/record["csv_file"]
            path.write_bytes(path.read_bytes()+b"\n")
            result = audit.audit_record(root, record, samples=1025)
            self.assertFalse(result["source_level_pass"])
            self.assertIn("CSV SHA256 mismatch", result["errors"])

    def test_hold_value_must_be_serially_identical_not_merely_numeric_equal(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            record = write_fixture(root)
            path = root/record["csv_file"]
            lines = path.read_text().splitlines()
            fields = lines[3].split(",")
            fields[2] += "0"
            lines[3] = ",".join(fields)
            path.write_text("\n".join(lines)+"\n")
            record["sha256"] = audit.sha256(path)
            result = audit.audit_record(root, record, samples=1025)
            self.assertFalse(result["byte_exact_holds_pass"])
            self.assertFalse(result["source_level_pass"])

    def test_time_grid_mutation_is_rejected_even_when_hash_updated(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            record = write_fixture(root)
            path = root/record["csv_file"]
            content = path.read_text().replace("1.25,plane1", "1.26,plane1", 1)
            path.write_text(content)
            record["sha256"] = audit.sha256(path)
            result = audit.audit_record(root, record, samples=1025)
            self.assertIn("time grid must start at zero and advance exactly 0.05 s", result["errors"])

    def test_moving_hold_and_wrong_frame_metadata_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            meta = metadata()
            meta["frame_id"] = "base"
            record = write_fixture(root, meta)
            path = root/record["csv_file"]
            lines = path.read_text().splitlines()
            fields = lines[100].split(",")
            fields[-1] = "hold"
            lines[100] = ",".join(fields)
            path.write_text("\n".join(lines)+"\n")
            record["sha256"] = audit.sha256(path)
            result = audit.audit_record(root, record, samples=1025)
            self.assertFalse(result["phase_contract_pass"])
            self.assertFalse(result["source_level_pass"])
            self.assertTrue(any("context mismatch" in error for error in result["errors"]))

    def test_path_escape_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            record = write_fixture(root)
            record["csv_file"] = "../outside.csv"
            result = audit.audit_record(root, record, samples=1025)
            self.assertFalse(result["source_level_pass"])
            self.assertTrue(any("escapes batch" in err for err in result["errors"]))


if __name__ == "__main__":
    unittest.main()
