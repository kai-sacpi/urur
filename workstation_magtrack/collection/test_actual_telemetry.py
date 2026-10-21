import csv
import hashlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from .actual_telemetry import audit_actual_telemetry, native_columns


JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")


class ActualTelemetryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.raw = self.root / "execution_telemetry.csv"
        self.output = self.root / "audit"
        self.motion = SimpleNamespace(environment=SimpleNamespace(joint_names=JOINTS),
                                      tip_fk=self.fk)
        self.columns = native_columns(JOINTS)
        self.rows = []
        for i in range(3):
            row = dict.fromkeys(self.columns, "0")
            row.update(ros_time_s=str(1. + i * .01), ros_time_ns=str(1_000_000_000 + i * 10_000_000),
                       sync_time_ns=str(100_000_000 + i * 10_000_000), trajectory_time_s=str(i * .01),
                       trajectory_time_ns=str(i * 10_000_000), state="EXECUTING",
                       shoulder_pan_joint_pos=str(i * .001), tip_x=str(i * .001), speed_scaling="1")
            # A different plan proves the audit really passes actual q to FK.
            row.update({f"desired_{j}_pos": "9" for j in JOINTS})
            self.rows.append(row)

    @staticmethod
    def fk(q):
        return {"base_link_position_m": q[:3], "plane1_position_m": [-q[0], q[1], q[2]],
                "base_link_quaternion_xyzw": [0., 0., 0., 1.],
                "plane1_quaternion_xyzw": [0., 0., 0., 1.]}

    def write(self):
        stream = io.StringIO(newline="")
        writer = csv.DictWriter(stream, fieldnames=self.columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(self.rows)
        self.raw.write_bytes(stream.getvalue().encode())
        return self.raw.read_bytes()

    def audit(self):
        self.write()
        return audit_actual_telemetry(self.raw, self.motion, self.output)

    def assert_rejected(self, message):
        before = self.write()
        with self.assertRaisesRegex((ValueError, csv.Error), message):
            audit_actual_telemetry(self.raw, self.motion, self.output)
        self.assertEqual(before, self.raw.read_bytes())
        report = json.loads((self.output / "actual_telemetry_audit.json").read_text())
        self.assertFalse(report["passed"])
        self.assertEqual(report["raw_sha256"], hashlib.sha256(before).hexdigest())
        self.assertFalse((self.output / "actual_fk_reference.csv").exists())

    def test_success_keeps_all_native_rows_clocks_and_raw(self):
        before = self.write()
        result = audit_actual_telemetry(self.raw, self.motion, self.output)
        self.assertTrue(result["passed"])
        self.assertEqual(result["sample_count"], 3)
        self.assertEqual(before, self.raw.read_bytes())
        self.assertEqual(result["raw_sha256"], hashlib.sha256(before).hexdigest())
        self.assertIn("not_magnetic_ground_truth", result["label_reference_type"])
        self.assertIn("not source header", result["sync_clock"])
        with Path(result["reference_csv"]).open(newline="") as stream:
            derived = list(csv.DictReader(stream))
        self.assertEqual([r["sync_time_ns"] for r in self.rows], [r["sync_time_ns"] for r in derived])
        self.assertEqual(derived[-1]["plane1_tip_x_m"], "-0.002")
        self.assertEqual(derived[-1]["base_link_tip_qw"], "1.0")
        self.assertEqual(derived[-1]["shoulder_pan_joint_pos"], "0.002")

    def test_no_planned_fallback_when_actual_column_missing(self):
        self.columns.remove("shoulder_pan_joint_pos")
        self.assert_rejected("no planned fallback")

    def test_renamed_plan_cannot_copy_into_actual_schema(self):
        self.columns[self.columns.index("desired_shoulder_pan_joint_pos")] = "shoulder_pan_joint_pos"
        self.assert_rejected("schema/order")

    def test_actual_equal_to_plan_is_valid_for_hold(self):
        for row in self.rows:
            for joint in JOINTS:
                row[f"desired_{joint}_pos"] = row[f"{joint}_pos"]
        self.assertTrue(self.audit()["passed"])

    def test_reordered_actual_joint_columns_rejected(self):
        a, b = (self.columns.index(f"{j}_pos") for j in JOINTS[:2])
        self.columns[a], self.columns[b] = self.columns[b], self.columns[a]
        self.assert_rejected("schema/order")

    def test_nonfinite_actual_joint_rejected(self):
        self.rows[1]["shoulder_pan_joint_pos"] = "nan"
        self.assert_rejected("nonfinite")

    def test_nonfinite_actual_velocity_rejected(self):
        self.rows[1]["wrist_3_joint_vel"] = "inf"
        self.assert_rejected("nonfinite")

    def test_nonfinite_native_tip_rejected(self):
        self.rows[1]["tip_x"] = "nan"
        self.assert_rejected("nonfinite")

    def test_duplicate_sync_time_rejected(self):
        self.rows[1]["sync_time_ns"] = self.rows[0]["sync_time_ns"]
        self.assert_rejected("clock progression")

    def test_zero_or_decimal_sync_rejected(self):
        self.rows[1]["sync_time_ns"] = "0"
        self.assert_rejected("positive integer ns")

    def test_trajectory_time_repeats_are_valid(self):
        self.rows[1]["trajectory_time_s"] = self.rows[0]["trajectory_time_s"]
        self.rows[1]["trajectory_time_ns"] = self.rows[0]["trajectory_time_ns"]
        self.assertTrue(self.audit()["passed"])

    def test_regressing_trajectory_time_rejected(self):
        self.rows[2]["trajectory_time_s"] = "0"
        self.rows[2]["trajectory_time_ns"] = "0"
        self.assert_rejected("clock progression")

    def test_inconsistent_trajectory_time_rejected(self):
        self.rows[1]["trajectory_time_ns"] = "11"
        self.assert_rejected("clock progression")

    def test_actual_fk_disagrees_with_native_tip_rejected(self):
        self.rows[1]["tip_x"] = "0.02"
        self.assert_rejected("differs from actual-joint FK")

    def test_derived_fk_nonfinite_rejected(self):
        self.motion.tip_fk = lambda q: {**self.fk(q), "plane1_position_m": [float("nan"), 0, 0]}
        self.assert_rejected("nonfinite")

    def test_single_row_rejected(self):
        self.rows = self.rows[:1]
        self.assert_rejected("at least two")

    def test_wrong_row_width_rejected(self):
        self.write()
        self.raw.write_bytes(self.raw.read_bytes() + b"0,1,2\n")
        with self.assertRaisesRegex(ValueError, "incorrect width"):
            audit_actual_telemetry(self.raw, self.motion, self.output)

    def test_existing_outputs_never_overwritten(self):
        self.audit()
        originals = {p: p.read_bytes() for p in self.output.iterdir()}
        with self.assertRaisesRegex(FileExistsError, "refusing overwrite"):
            audit_actual_telemetry(self.raw, self.motion, self.output)
        self.assertEqual(originals, {p: p.read_bytes() for p in self.output.iterdir()})


if __name__ == "__main__":
    unittest.main()
