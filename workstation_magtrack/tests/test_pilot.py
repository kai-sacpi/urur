"""Numerical and research-design contracts for the pre-calibration pilot."""

from collections import Counter
import csv
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from workstation_magtrack.generation import pilot


class PilotContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        cls.records = pilot.generate(cls.root)
        cls.pilot = [r for r in cls.records if r["stage"] == "pilot"]

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_counts_and_session_margins(self):
        self.assertEqual(len(self.records), 63)
        self.assertEqual(Counter(r["category"] for r in self.pilot), dict.fromkeys(["static", "translation", "turn", "orientation", "mixed"], 12))
        for session in (1, 2):
            records = [r for r in self.pilot if r["session_number"] == session]
            self.assertEqual(len(records), 30)
            self.assertTrue(all(n == 6 for n in Counter(r["category"] for r in records).values()))
            mixed = [r for r in records if r["category"] == "mixed"]
            self.assertEqual(Counter(r["v_tier"] for r in mixed), {"V1": 2, "V2": 2, "V3": 2})
            self.assertEqual(Counter(r["w_tier"] for r in mixed), {"W1": 2, "W2": 2, "W3": 2})
            for tier_name in ("v_tier", "w_tier"):
                for tier in (1, 2, 3):
                    self.assertEqual(Counter(r["height_group"] for r in mixed if r[tier_name] == f"{tier_name[0].upper()}{tier}"), {"high": 1, "low": 1})
        for category in ("translation", "turn", "orientation", "mixed"):
            self.assertEqual(Counter(r["height_group"] for r in self.pilot if r["category"] == category), {"high": 6, "low": 6})
        mixed_pairs = Counter((r["v_tier"], r["w_tier"]) for r in self.pilot if r["category"] == "mixed")
        self.assertEqual(len(mixed_pairs), 9)
        self.assertEqual({p for p, n in mixed_pairs.items() if n == 2}, {("V1", "W1"), ("V2", "W3"), ("V3", "W2")})

    def test_declared_speed_tiers_reached_with_bounded_dynamics(self):
        for r in self.records:
            d = r["measured_source_diagnostics"]
            if r["nominal_center_speed_m_s"]:
                self.assertGreaterEqual(d["center_speed_max_m_s"], .985 * r["nominal_center_speed_m_s"], r["trajectory_id"])
            if r["nominal_axis_rate_deg_s"]:
                self.assertGreaterEqual(d["nominal_axis_rate_max_deg_s"], .985 * r["nominal_axis_rate_deg_s"], r["trajectory_id"])
            self.assertLessEqual(d["center_acceleration_max_m_s2"], .05)
            self.assertLessEqual(d["center_jerk_max_m_s3"], .25)
            self.assertLessEqual(d["flange_angular_acceleration_max_deg_s2"], 30)
            self.assertLessEqual(d["flange_angular_jerk_max_deg_s3"], 150)
            self.assertLessEqual(d["xy_absolute_max_m"], .13)
            self.assertGreaterEqual(d["height_min_m"], .08 - 1e-10)
            self.assertLessEqual(d["height_max_m"], .20 + 1e-10)

    def test_routes_have_rest_and_exact_holds(self):
        for r in self.records:
            with open(r["source_file"], newline="") as stream:
                csv_rows = list(csv.DictReader(stream))
            for previous, current in zip(csv_rows[:-1], csv_rows[1:]):
                if current["phase"] == "hold":
                    self.assertEqual([current[key] for key in pilot.CSV_FIELDS[2:9]], [previous[key] for key in pilot.CSV_FIELDS[2:9]])
            data = np.genfromtxt(r["source_file"], delimiter=",", names=True, dtype=None, encoding="utf-8")
            p = np.column_stack([data[k] for k in ("tip_x_m", "tip_y_m", "tip_z_m")])
            q = np.column_stack([data[k] for k in ("tip_qx", "tip_qy", "tip_qz", "tip_qw")])
            self.assertTrue(np.array_equal(p[0], p[1]))
            self.assertTrue(np.array_equal(p[-1], p[-2]))
            self.assertTrue(np.array_equal(q[0], q[1]))
            self.assertTrue(np.array_equal(q[-1], q[-2]))
            t = data["time_s"]
            for window in r["phase_windows"]:
                if window["phase"] != "hold":
                    continue
                ix = (t >= window["start_s"] + 1e-8) & (t <= window["end_s"] - 1e-8)
                if ix.sum():
                    self.assertTrue(np.all(p[ix] == p[ix][0]))
                    self.assertTrue(np.all(q[ix] == q[ix][0]))
            if r["category"] in ("static", "orientation"):
                self.assertTrue(np.all(p == p[0]))

    def test_pair_family_roles_and_real_gate(self):
        for family in {r["family_id"] for r in self.pilot}:
            records = [r for r in self.pilot if r["family_id"] == family]
            self.assertEqual(len({r["data_role"] for r in records}), 1)
        for r in self.records:
            self.assertTrue(r["simulation_only"])
            self.assertFalse(r["real_execution_authorized"])
            self.assertEqual(r["real_gate"], "calibration+sync+commissioning+tier_pending")
            self.assertTrue(all(value is False for value in r["runtime_changes_permitted"].values()))
            if r["duration_s"] > 20.01:
                self.assertIsNotNone(r["duration_exception_reason"])

    def test_fixed_axis_not_axial_yaw(self):
        records = [r for r in self.pilot if r["category"] == "orientation"]
        axes = [r["geometry"]["axis_in_plane1_xyz"] for r in records]
        self.assertGreater(np.linalg.matrix_rank(axes), 1)
        for r in records:
            self.assertFalse(r["geometry"]["pure_axial_yaw"])
            self.assertGreater(r["measured_source_diagnostics"]["nominal_axis_rate_max_deg_s"], 2.9)

    def test_deterministic_generation_and_time_coverage_report(self):
        regenerated = pilot.generate(self.root)
        self.assertEqual([r["source_sha256"] for r in self.records], [r["source_sha256"] for r in regenerated])
        coverage = json.loads((self.root / "planning" / "pilot_coverage_v4.json").read_text())
        self.assertAlmostEqual(sum(coverage["height_bin_time_s"]), coverage["planned_high_15_20cm_time_s"], places=6)
        self.assertGreater(coverage["planned_high_15_20cm_fraction"], .50)
        self.assertLess(coverage["planned_high_15_20cm_fraction"], .70)
        for fraction in coverage["height_bin_fraction_of_all_time"]:
            self.assertGreater(fraction, .10)
            self.assertLess(fraction, .14)
        self.assertEqual(len(json.loads((self.root / "planning" / "pilot_execution_order_v4.json").read_text())["records"]), 60)


if __name__ == "__main__":
    unittest.main()
