"""Serial batching must not manufacture group deadline failures."""

from __future__ import annotations

import struct

import pytest

from hx_waypoint_nucleo import NucleoCaptureConfig, NucleoCaptureSession
from hx_waypoint_nucleo import capture


def _session(tmp_path):
    return NucleoCaptureSession(
        tmp_path, NucleoCaptureConfig(port="virtual"), lambda: 1
    )


@pytest.mark.parametrize("available,expected", [(0, 1), (37, 37), (8192, 4096)])
def test_stream_read_uses_available_bytes_or_waits_for_one(
    tmp_path, available, expected
):
    class Port:
        in_waiting = available

        def read(self, size):
            assert size == expected
            return b"received"

        def write(self, data):
            pytest.fail("stream reads must never send SYNC or other bytes")

        def reset_input_buffer(self):
            pytest.fail("stream reads must never discard queued samples")

    assert _session(tmp_path)._read_stream_chunk(Port()) == b"received"


def test_stream_read_supports_ports_without_in_waiting(tmp_path):
    class Port:
        def read(self, size):
            assert size == 1
            return b"x"

    assert _session(tmp_path)._read_stream_chunk(Port()) == b"x"


def _packet(channel, group, arrival_ms):
    payload = (
        b"L5"
        + bytes((group % 16,))
        + struct.pack(">48h", *range(48))
        + struct.pack(">I", 1000 + group * 10)
    )
    return (
        b"\x55\xaa"
        + bytes((channel,))
        + struct.pack("<II", group, 1000 + arrival_ms)
        + payload
    )


class TimedSerial:
    """Model pyserial: read(size) waits for size bytes or its 20 ms timeout."""

    def __init__(self, events, stop_event):
        self.now_ns = 1_000_000_000
        self.events = sorted(
            (self.now_ns + ms * 1_000_000, data) for ms, data in events
        )
        self.pending = bytearray()
        self.stop_event = stop_event
        self.stop_ns = max((t for t, _ in self.events), default=self.now_ns) + 80_000_000
        self.requests = []

    def _enqueue_due(self):
        while self.events and self.events[0][0] <= self.now_ns:
            _, data = self.events.pop(0)
            self.pending.extend(data)

    @property
    def in_waiting(self):
        self._enqueue_due()
        return len(self.pending)

    def read(self, size):
        self.requests.append(size)
        deadline = self.now_ns + 20_000_000
        self._enqueue_due()
        while len(self.pending) < size:
            if not self.events or self.events[0][0] > deadline:
                self.now_ns = deadline
                break
            self.now_ns = self.events[0][0]
            self._enqueue_due()
        result = bytes(self.pending[:size])
        del self.pending[:size]
        if self.now_ns >= self.stop_ns:
            self.stop_event.set()
        return result


def _run_reader(tmp_path, monkeypatch, events, *, legacy=False):
    session = _session(tmp_path)
    port = TimedSerial(events, session._stop_event)
    session._serial = port
    frames = []
    session._write_frames = frames.extend
    monkeypatch.setattr(capture.time, "monotonic_ns", lambda: port.now_ns)
    if legacy:
        session._read_stream_chunk = lambda serial_port: serial_port.read(4096)
    session._reader_loop()
    assert session.capture_error is None
    assert session._reader_done_event.is_set()
    return frames, session._parser.stats, port


def test_receive_buffering_does_not_create_false_40ms_timeouts(tmp_path, monkeypatch):
    # Each physical group spans only 27 ms. Large timed reads can expose it to
    # the parser 40 ms apart, causing expiration followed by apparent reordering.
    events = [
        (group * 10 + delay, _packet(channel, group, group * 10 + delay))
        for group in range(100)
        for channel, delay in ((2, 0), (3, 0), (4, 0), (1, 27))
    ]
    _, before, _ = _run_reader(tmp_path, monkeypatch, events, legacy=True)
    assert before.timeout_groups > 0
    assert before.reordered_packets > 0

    frames, after, port = _run_reader(tmp_path, monkeypatch, events)
    assert len(frames) == 100
    assert [frame.src_seq for frame in frames] == [n % 16 for n in range(100)]
    assert after.hub_packets == 400
    assert after.bytes_received == 400 * 114
    for name in (
        "timeout_groups", "reordered_packets", "incomplete_groups",
        "channel_sequence_drops", "source_sequence_drops", "bytes_discarded",
        "malformed_packets", "duplicate_packets", "pending_evictions",
    ):
        assert getattr(after, name) == 0, name
    assert max(port.requests) < 4096


@pytest.mark.parametrize("delay_ms", [40, 45])
def test_genuinely_late_group_is_still_rejected(tmp_path, monkeypatch, delay_ms):
    events = [(0, _packet(channel, 0, 0)) for channel in (2, 3, 4)]
    events.append((delay_ms, _packet(1, 0, delay_ms)))
    frames, stats, _ = _run_reader(tmp_path, monkeypatch, events)
    assert not frames
    assert stats.timeout_groups == 1
    assert stats.incomplete_groups == 1


def test_idle_reads_still_expire_incomplete_groups(tmp_path, monkeypatch):
    frames, stats, _ = _run_reader(tmp_path, monkeypatch, [(0, _packet(1, 0, 0))])
    assert not frames
    assert stats.timeout_groups == 1


def test_stream_read_failure_is_reported(tmp_path):
    class DisconnectedPort:
        @property
        def in_waiting(self):
            raise OSError("USB disconnected")

    session = _session(tmp_path)
    session._serial = DisconnectedPort()
    session._reader_loop()
    assert "OSError: USB disconnected" in session.capture_error
    assert session._reader_done_event.is_set()
