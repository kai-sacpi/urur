"""Verifier regression checks using synthetic fixtures only, never campaign data."""
import csv
from dataclasses import asdict
import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

SPEC = importlib.util.spec_from_file_location("verify_background", Path(__file__).with_name("verify_background_batch.py"))
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)
from hx_waypoint_nucleo.protocol import ParserStats


class VerificationTests(unittest.TestCase):
    def counters(self, **updates):
        values = {key: 0 for key in audit.ZERO_COUNTERS}
        values.update(incomplete_groups=0)
        values.update(updates)
        return values

    def test_exact_policy_boundary_is_accepted(self):
        self.assertEqual(audit.check_quality(11940, 119.9, self.counters(incomplete_groups=60)), .005)

    def test_policy_is_not_relaxed(self):
        for count, coverage, counters in [
            (11939, 119.99, self.counters()),
            (12000, 119.899999, self.counters()),
            (11940, 119.9, self.counters(incomplete_groups=61)),
        ]:
            with self.subTest(count=count, coverage=coverage, counters=counters):
                with self.assertRaises(ValueError):
                    audit.check_quality(count, coverage, counters)

    def test_all_forbidden_counters_fail(self):
        for name in audit.ZERO_COUNTERS:
            with self.subTest(name=name), self.assertRaises(ValueError):
                audit.check_quality(12000, 119.99, self.counters(**{name: 1}))

    def test_half_open_abutting_windows_are_valid(self):
        self.assertEqual(len(audit.check_nonoverlap([(120, 240, "b"), (0, 120, "a")])), 2)

    def test_cross_session_overlap_fails(self):
        with self.assertRaises(ValueError):
            audit.check_nonoverlap([(0, 120, "session1"), (119, 239, "session2")])

    def fixture(self, path, *, corrupt=None, equal_receive=False):
        baseline = asdict(ParserStats())
        end = dict(baseline, hub_packets=12, completed_groups=3, bytes_received=1368)
        with path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=audit.RAW_CAPTURE_COLUMNS)
            writer.writeheader()
            for i, offset in enumerate([0, 0 if equal_receive else 60_000_000_000, 119_990_000_000]):
                row = {name: 0 for name in audit.RAW_CAPTURE_COLUMNS}
                row.update(sensor_rx_sync_time_ns=1_000_000_000 + offset,
                           parser_host_monotonic_ns=1_000_000_000 + offset,
                           sensor_rx_ros_time_ns=1_700_000_000_000_000_000 + offset,
                           logical_seq=i, src_seq=i, channel_ids="1;2;3;4",
                           parser_hub_packets=4 * (i + 1), parser_completed_groups=i + 1)
                if i == 1 and corrupt:
                    row.update(corrupt)
                writer.writerow(row)
        return baseline, end

    def test_every_row_is_scanned_and_equal_batch_timestamps_are_allowed(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / "fixture.csv"
            baseline, end = self.fixture(path, equal_receive=True)
            result = audit.scan_csv(path, 1_000_000_000, 121_000_000_000, baseline, end)
            self.assertEqual(result["frames"], 3)
            self.assertAlmostEqual(result["actual_sample_coverage_s"], 119.99)

    def test_nonfinite_or_incomplete_or_out_of_window_rows_fail(self):
        cases = [
            {"sensor_63_z_ut": "nan"},
            {"sensor_00_x_ut": "inf"},
            {"channel_ids": "2;3;4"},
            {"logical_seq": 0},
            {"sensor_rx_sync_time_ns": 121_000_000_000},
            {"parser_timeout_groups": 1},
        ]
        for corrupt in cases:
            with self.subTest(corrupt=corrupt), TemporaryDirectory() as folder:
                path = Path(folder) / "fixture.csv"
                baseline, end = self.fixture(path, corrupt=corrupt)
                with self.assertRaises(ValueError):
                    audit.scan_csv(path, 1_000_000_000, 121_000_000_000, baseline, end)


if __name__ == "__main__":
    unittest.main()
