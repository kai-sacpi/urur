"""Pure CPU checks for the simulated-wire isolation diagnostic; no serial I/O."""
from __future__ import annotations

import csv
from pathlib import Path
import tempfile
import unittest

from hx_waypoint_nucleo.protocol import HubStreamParser

from ops.assess_capture_isolation import _reserve_output, hub_group, receive_metrics


class IsolationDiagnosticTests(unittest.TestCase):
    def test_actual_wire_format_groups_are_complete_and_loss_free(self) -> None:
        parser = HubStreamParser()
        for index in range(25):
            wire = hub_group(index)
            self.assertEqual(len(wire), 456)
            frames = parser.feed(wire, 1_000_000_000 + index * 10_000_000)
            self.assertEqual(len(frames), 1)
            self.assertEqual(frames[0].src_seq, index % 16)
            self.assertEqual(frames[0].source_time_ms_u32, 1000 + index * 10)
            self.assertEqual(tuple(record.channel_id for record in frames[0].records), (1, 2, 3, 4))
            self.assertEqual(frames[0].records[0].payload.raw_xyz[0], (100, 101, 102))
            self.assertEqual(frames[0].records[3].payload.raw_xyz[-1], (445, 446, 447))
        self.assertEqual(parser.stats.completed_groups, 25)
        self.assertEqual(parser.stats.incomplete_groups, 0)
        self.assertEqual(parser.stats.channel_sequence_drops, 0)
        self.assertEqual(parser.stats.source_sequence_drops, 0)
        self.assertEqual(parser.stats.malformed_packets, 0)

    def test_simulated_uint32_wire_clock_wrap_is_explicit(self) -> None:
        parser = HubStreamParser()
        index = (2**32 - 1000) // 10 + 1
        frame = parser.feed(hub_group(index), 1_000_000_000)[0]
        self.assertEqual(frame.source_time_ms_u32, (1000 + index * 10) & 0xFFFFFFFF)
        self.assertEqual(frame.records[0].nucleo_time_ms_u32, frame.source_time_ms_u32)

    def test_invalid_indices_rejected(self) -> None:
        for invalid in (-1, True, 1.5, None):
            with self.subTest(value=invalid), self.assertRaises(ValueError):
                hub_group(invalid)

    def test_exclusive_output_preserves_existing_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            output = _reserve_output(workspace, workspace / "runs/probe")
            sentinel = output / "original.json"
            sentinel.write_text("original")
            with self.assertRaises(FileExistsError):
                _reserve_output(workspace, output)
            self.assertEqual(sentinel.read_text(), "original")

    def test_output_cannot_escape_runs_or_be_runs_itself(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            for path in (workspace / "outside", workspace / "runs", workspace / "runs/../outside"):
                with self.subTest(path=path), self.assertRaisesRegex(ValueError, "beneath workspace/runs"):
                    _reserve_output(workspace, path)

    def test_metrics_preserve_zero_intervals_and_workload_bounds(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            raw = Path(directory) / "raw.csv"
            with raw.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(("sensor_rx_sync_time_ns",))
                writer.writerows((stamp,) for stamp in (10_000_000, 20_000_000, 20_000_000, 45_000_000))
            original = raw.read_bytes()
            report = receive_metrics(raw, [{"start_monotonic_ns": 20_000_000,
                                            "end_monotonic_ns": 45_000_000}])
            self.assertEqual(report["complete_raw"]["max_receive_gap_ms"], 25)
            self.assertEqual(report["complete_raw"]["zero_interval_arrivals"], 1)
            self.assertEqual(report["complete_raw"]["gaps_over_20ms"], 1)
            self.assertEqual(report["verification_intervals"][0]["frames"], 2)
            self.assertEqual(raw.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
