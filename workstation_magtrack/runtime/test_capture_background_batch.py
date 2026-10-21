"""Acceptance and segmentation checks; no hardware is opened by these tests."""

import csv
from dataclasses import asdict
import importlib.util
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

SPEC = importlib.util.spec_from_file_location("background_batch", Path(__file__).with_name("capture_background_batch.py"))
batch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(batch)
from hx_waypoint_nucleo.protocol import ParserStats


class BackgroundQualityTests(unittest.TestCase):
    def counters(self, **updates):
        values = {key: 0 for key in batch.ZERO_COUNTERS}
        values.update(incomplete_groups=0)
        values.update(updates)
        return values

    def test_accepts_small_number_of_incomplete_groups(self):
        self.assertEqual(batch.quality_reasons(11970, 120, 119.98, self.counters(incomplete_groups=30)), [])

    def test_rejects_too_few_complete_groups(self):
        self.assertTrue(batch.quality_reasons(11939, 120, 119.98, self.counters()))

    def test_rejects_incomplete_ratio_above_threshold(self):
        self.assertTrue(batch.quality_reasons(12000, 120, 119.98, self.counters(incomplete_groups=61)))

    def test_each_forbidden_counter_causes_rejection(self):
        for key in batch.ZERO_COUNTERS:
            with self.subTest(counter=key):
                self.assertTrue(batch.quality_reasons(12000, 120, 119.98, self.counters(**{key: 1})))

    def test_rejects_short_actual_temporal_coverage(self):
        self.assertTrue(batch.quality_reasons(12000, 120, 119.8, self.counters()))

    def test_window_uses_counter_deltas_and_preserves_raw_cumulative_values(self):
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            for name in ("in_progress", "accepted", "rejected"):
                (root / name).mkdir()
            baseline = asdict(ParserStats(completed_groups=500, reordered_packets=32, bytes_discarded=70))
            window = batch.Window(root, 1, 10_000_000_000, 1_000_000_000, baseline)
            for index in range(100):
                timestamp = 10_000_000_000 + index * 10_000_000
                frame = SimpleNamespace(host_monotonic_ns=timestamp)
                row = {key: "" for key in batch.RAW_CAPTURE_COLUMNS}
                row.update(sensor_rx_ros_time_ns=timestamp, parser_reordered_packets=32, parser_bytes_discarded=70)
                window.append(frame, row)
            end = dict(baseline, completed_groups=600)
            session = SimpleNamespace(config=SimpleNamespace(port="test", baudrate=115200),
                                      _sync_attempts=0, capture_error=None)
            result = window.finish(root, end, session, "reused_existing_stream")
            self.assertTrue(result["accepted"], result)
            folder = Path(result["directory"])
            manifest = json.loads((folder / "nucleo_capture.yaml").read_text())
            self.assertEqual(manifest["counts"]["parser"]["reordered_packets"], 0)
            with (folder / "nucleo_raw_telemetry.csv").open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 100)
            self.assertTrue(all(row["parser_reordered_packets"] == "32" for row in rows))
            self.assertEqual(batch.digest(folder / "nucleo_raw_telemetry.csv"), result["raw_sha256"])

    def test_existing_stream_never_sends_sync(self):
        passive = MagicMock()
        with patch.object(batch, "ObservedSession", return_value=passive) as constructor:
            session, mode = batch.open_session(Path("unused"), "test")
            self.assertIs(session, passive)
            self.assertEqual(mode, "reused_existing_stream")
            self.assertFalse(constructor.call_args.args[1].perform_sync)
            self.assertEqual(constructor.call_count, 1)

    def test_silent_board_receives_only_one_initial_sync(self):
        with TemporaryDirectory() as tmp:
            passive, active = MagicMock(), MagicMock()
            passive.start.side_effect = batch.NucleoCaptureError("Nucleo preflight timed out")
            passive.stop.return_value = SimpleNamespace(parser_stats=ParserStats(), staging_path=Path(tmp) / "probe.csv")
            with patch.object(batch, "ObservedSession", side_effect=[passive, active]) as constructor:
                session, mode = batch.open_session(Path(tmp), "test")
                self.assertIs(session, active)
                self.assertEqual(mode, "performed_once")
                self.assertEqual(constructor.call_args_list[1].args[1].sync_retries, 1)
                active.start.assert_called_once()

    def test_partial_existing_stream_does_not_retry_handshake(self):
        with TemporaryDirectory() as tmp:
            passive = MagicMock()
            passive.start.side_effect = batch.NucleoCaptureError("Nucleo preflight timed out")
            passive.stop.return_value = SimpleNamespace(parser_stats=ParserStats(bytes_received=50), staging_path=Path(tmp) / "probe.csv")
            with patch.object(batch, "ObservedSession", return_value=passive) as constructor:
                with self.assertRaisesRegex(batch.NucleoCaptureError, "refusing another SYNC"):
                    batch.open_session(Path(tmp), "test")
                self.assertEqual(constructor.call_count, 1)


if __name__ == "__main__":
    unittest.main()
