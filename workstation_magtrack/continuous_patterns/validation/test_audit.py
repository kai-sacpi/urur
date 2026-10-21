"""Small invariant fixtures for the independent source auditor (no batch run)."""
from __future__ import annotations

import csv
import hashlib
import importlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

try:
    from . import audit
except ImportError:
    import audit


class AuthorityFixtures(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="continuous_audit_fixture_")
        self.root = Path(self.temporary.name).resolve()
        self.csv_path = self.root / "curve.csv"
        self.metadata_path = self.root / "curve.json"
        self.metadata = {"trajectory_id": "fixture", "draw_start_s": .5,
                         "draw_end_s": 1.5, "draw_duration_s": 1.}
        self.metadata_path.write_text(json.dumps(self.metadata))
        output = io.StringIO(newline="")
        writer = csv.writer(output, lineterminator="\n")
        writer.writerow(audit.FIELDS)
        for i in range(41):
            t = i/20.
            s = float(np.clip(t-.5, 0., 1.))
            x = .01*(35*s**4-84*s**5+70*s**6-20*s**7)
            phase = "draw" if i == 0 or .5 < t <= 1.5 else "hold"
            writer.writerow([f"{t:.2f}", "plane1", f"{x:.12f}", "0.000000000000", "-0.100000000000",
                             "0.000000000000", "0.000000000000", "0.000000000000", "1.000000000000", phase])
        self.csv_path.write_text(output.getvalue())
        self.record = {"trajectory_id": "fixture", "csv_file": "curve.csv", "metadata_file": "curve.json",
                       "sha256": hashlib.sha256(self.csv_path.read_bytes()).hexdigest(),
                       "metadata_sha256": hashlib.sha256(self.metadata_path.read_bytes()).hexdigest()}

    def tearDown(self):
        self.temporary.cleanup()

    def resha(self):
        self.record["sha256"] = hashlib.sha256(self.csv_path.read_bytes()).hexdigest()

    def test_valid_authority_keeps_literal_holds(self):
        metadata, t, p, q, phase, info = audit._read_source(self.root, self.record)
        self.assertEqual(info["source_pose_count"], 41)
        self.assertTrue(info["hold_seven_pose_fields_byte_exact"])
        self.assertEqual(float(t[0]), 0.)
        self.assertAlmostEqual(float(p[-1, 0]), .01)

    def test_hash_tampering_rejected(self):
        self.csv_path.write_text(self.csv_path.read_text().replace("plane1", "plane2", 1))
        with self.assertRaisesRegex(ValueError, "SHA-256"):
            audit._read_source(self.root, self.record)

    def test_metadata_tampering_rejected(self):
        self.metadata_path.write_text(self.metadata_path.read_text()+" ")
        with self.assertRaisesRegex(ValueError, "metadata SHA-256"):
            audit._read_source(self.root, self.record)

    def test_equal_float_hold_with_different_field_bytes_rejected(self):
        rows = self.csv_path.read_text().splitlines()
        fields = rows[32].split(",")  # t=1.55 s, an end hold row
        self.assertEqual(fields[-1], "hold")
        self.assertEqual(float(fields[2]), .01)
        fields[2] = ".01"
        rows[32] = ",".join(fields)
        self.csv_path.write_text("\n".join(rows)+"\n")
        self.resha()
        with self.assertRaisesRegex(ValueError, "hold seven-pose field bytes"):
            audit._read_source(self.root, self.record)

    def test_duplicate_time_rejected(self):
        rows = self.csv_path.read_text().splitlines()
        fields = rows[20].split(",")
        fields[0] = rows[19].split(",")[0]
        rows[20] = ",".join(fields)
        self.csv_path.write_text("\n".join(rows)+"\n")
        self.resha()
        with self.assertRaisesRegex(ValueError, "strictly increase"):
            audit._read_source(self.root, self.record)

    def test_single_constant_layer_end_to_end_source_audit(self):
        code_root = Path(__file__).resolve().parents[1]
        sys.path.insert(0, str(code_root.parent))
        trajectory = importlib.import_module(f"{code_root.name}.src.trajectory")
        patterns = importlib.import_module(f"{code_root.name}.src.patterns")
        meta = {"trajectory_id": "fixture", "group_id": "stratified",
                "geometry_id": "fixture_xy", "family_id": patterns.FAMILY_NAMES[0],
                "seed": 42, "direction": "forward", "v_tier": "V1", "w_tier": "W0",
                "nominal_center_speed_m_s": .01, "nominal_axis_rate_deg_s": 0.,
                "orientation_mode": "fixed", "pattern_parameters": patterns.sample_parameters(patterns.FAMILY_NAMES[0], 42),
                "height_profile": {"mode": "constant", "lower_m": .10, "upper_m": .10},
                "xy": {"center_m": [0., 0.], "scale_m": [.04, .05],
                       "rotation_rad": .2, "parameter_offset": .3, "cycles": 1},
                "orientation": {"base_rotvec_rad": [.1, 0., 0.], "axis_in_base_tool_xyz": [1., 0., 0.],
                                "cycles": 0, "phase_rad": .2, "amplitude_rad": 0.}}
        certificate = trajectory.derivative_certificate(meta, samples=1025)
        duration = math.ceil(certificate["normalized_position_derivative_bounds"][1]/.01*10)/10
        meta.update({"draw_duration_s": duration, "draw_start_s": .5, "draw_end_s": duration+.5})
        t = np.arange(round((duration+1.)/.05)+1)*.05
        s = np.clip((t-.5)/duration, 0., 1.)
        p, q = trajectory.position_derivatives(meta, s)[0], trajectory.orientation(meta, s)
        stream = io.StringIO(newline="")
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(audit.FIELDS)
        for index, (tt, pp, qq) in enumerate(zip(t, p, q)):
            phase = "draw" if index == 0 or .5 < tt <= duration+.5+1e-10 else "hold"
            writer.writerow([f"{tt:.9f}", "plane1", *[f"{v:.12f}" for v in np.r_[pp, qq]], phase])
        self.csv_path.write_text(stream.getvalue())
        self.metadata_path.write_text(json.dumps(meta))
        self.resha()
        self.record["metadata_sha256"] = hashlib.sha256(self.metadata_path.read_bytes()).hexdigest()
        source = audit.audit_source(self.root, self.record, trajectory, None, 1025, 101)
        self.assertTrue(source.audit["source_level_pass"])
        self.assertEqual(source.audit["geometry"]["height_reversal_count"], 0)
        self.assertLess(source.audit["geometry"]["SVD_nonplanarity_rms_m"], 1e-12)
        self.assertEqual(source.audit["nominal_geometry_precheck"]["status"], "not_available")


class GeometryFixtures(unittest.TestCase):
    def test_full_vector_acceleration_includes_circle_centripetal_term(self):
        t = np.linspace(0., 10., 1001)
        radius, angular_rate = .05, .4
        p = np.c_[radius*np.cos(angular_rate*t), radius*np.sin(angular_rate*t), np.full_like(t, -.1)]
        q = np.tile([0., 0., 0., 1.], (len(t), 1))
        series = audit.motion_series(t, p, q)
        self.assertAlmostEqual(float(np.median(series["acceleration_m_s2"])), radius*angular_rate**2, places=6)
        self.assertLess(float(np.ptp(series["speed"][3:-3])), 1e-9)

    def test_full_SO3_rate_detects_roll_when_tool_axis_rate_is_zero(self):
        t = np.linspace(0., 10., 1001)
        angle = np.deg2rad(3.)*t
        q = Rotation.from_rotvec(np.c_[np.zeros_like(t), np.zeros_like(t), angle]).as_quat()
        p = np.tile([0., 0., -.1], (len(t), 1))
        series = audit.motion_series(t, p, q)
        self.assertAlmostEqual(float(np.median(series["angular_speed_deg_s"])), 3., places=10)
        self.assertEqual(float(series["axis_rate_deg_s"].max()), 0.)
        self.assertLess(float(series["angular_acceleration_deg_s2"].max()), 1e-8)

    def test_height_bins_split_crossing_intervals_and_include_200mm(self):
        result = audit.weighted_bins(np.array([0., 1., 2.]), np.array([.07, .20, .20]), audit.HEIGHT_EDGES_M)
        self.assertAlmostEqual(float(result.sum()), 2.)
        self.assertAlmostEqual(float(result[0]), 1./13.)
        self.assertAlmostEqual(float(result[-1]), 1.+1./13.)
        draw = audit.weighted_bins(np.array([0., 1., 2.]), np.array([.07, .20, .20]),
                                   audit.HEIGHT_EDGES_M, np.array([True, False]))
        self.assertAlmostEqual(float(draw.sum()), 1.)

    def test_crossing_interpolates_real_height_difference(self):
        p = np.array([[-.05, -.05, -.10], [.05, .05, -.10], [.05, -.05, -.18], [-.05, .05, -.18]])
        result = audit._projection_crossings(np.arange(len(p), dtype=float), p, 101)
        self.assertEqual(result["count"], 1)
        self.assertAlmostEqual(result["events"][0]["height_separation_m"], .08)

    def test_crossing_exactly_at_sampled_vertices_is_not_lost(self):
        p = np.array([[-.05, -.05, -.10], [0., 0., -.10], [.05, .05, -.10],
                      [.05, -.05, -.18], [0., 0., -.18], [-.05, .05, -.18]])
        result = audit._projection_crossings(np.arange(len(p), dtype=float), p, 101)
        self.assertEqual(result["count"], 1)
        self.assertAlmostEqual(result["events"][0]["height_separation_m"], .08)

    def test_tangent_touch_is_not_a_transverse_crossing(self):
        p = np.array([[-.05, 0., -.10], [.05, 0., -.10], [.05, .05, -.18],
                      [0., 0., -.18], [-.05, .05, -.18]])
        result = audit._projection_crossings(np.arange(len(p), dtype=float), p, 101)
        self.assertEqual(result["count"], 0)


if __name__ == "__main__":
    unittest.main()
