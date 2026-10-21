import csv
from dataclasses import asdict
import io
import struct
import unittest

from diagnose_nucleo_stream import HubStreamParser, TracedParser, read_chunk


def packet(channel, sample, arrival_ms):
    return (b"\x55\xaa" + bytes([channel])
            + struct.pack("<II", sample, arrival_ms)
            + b"L5" + bytes([sample % 16])
            + struct.pack(">48hI", *range(48), 1000 + sample * 10))


def ordered_wire_with_27ms_channel_skew():
    records = []
    for sample in range(30):
        for channel in (1, 2, 3, 4):
            ms = sample * 10 + (27 if channel == 1 else 0)
            records.append((ms, channel, packet(channel, sample, ms)))
    return sorted(records)


def replay(records, quantize_ms=0, parser=None):
    parser = parser if parser is not None else HubStreamParser()
    batches = {}
    for ms, _, value in records:
        stamp = ((ms + quantize_ms - 1) // quantize_ms * quantize_ms
                 if quantize_ms else ms)
        batches.setdefault(stamp, bytearray()).extend(value)
    for ms, batch in sorted(batches.items()):
        parser.feed(batch, 1_000_000_000 + ms * 1_000_000)
    return parser.stats


class DiagnosticTests(unittest.TestCase):
    def test_read_strategy_never_writes_to_serial(self):
        class Port:
            in_waiting = 114
            def __init__(self):
                self.calls = []
            def read(self, size):
                self.calls.append(size)
                return b"x" * size
            def write(self, value):
                raise AssertionError("diagnosis must never transmit")
        p = Port()
        read_chunk(p, "blocking4096")
        read_chunk(p, "drain_available")
        p.in_waiting = 0
        read_chunk(p, "drain_available")
        self.assertEqual(p.calls, [4096, 114, 1])

    def test_same_ordered_wire_batched_reads_can_create_false_timeout_and_reorder(self):
        wire = ordered_wire_with_27ms_channel_skew()
        immediate = replay(wire)
        batched = replay(wire, 20)
        self.assertEqual(immediate.completed_groups, 30)
        self.assertEqual(immediate.timeout_groups, 0)
        self.assertEqual(immediate.reordered_packets, 0)
        self.assertGreater(batched.timeout_groups, 0)
        self.assertGreater(batched.reordered_packets, 0)
        self.assertTrue(all(c.reordered_packets == 0 for c in batched.channel_stats))

    def test_trace_instrumentation_does_not_change_parser_results(self):
        output = io.StringIO()
        writer = csv.DictWriter(output, fieldnames=("phase", "channel", "channel_seq", "src_seq", "source_ms", "nucleo_ms", "host_ns"))
        events = io.StringIO()
        traced = TracedParser(writer, events)
        wire = ordered_wire_with_27ms_channel_skew()
        original = replay(wire, 20)
        instrumented = replay(wire, 20, traced)
        self.assertEqual(asdict(original), asdict(instrumented))
        self.assertIn('"kind": "expire_before_feed"', events.getvalue())
        self.assertIn('"kind": "reordered_rejection"', events.getvalue())


if __name__ == "__main__":
    unittest.main()
