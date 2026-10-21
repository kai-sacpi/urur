# SPDX-License-Identifier: Apache-2.0
"""Pure-Python decoding and grouping for the Nucleo magnetic sensor stream.

This is an independent implementation of the documented wire format.  The
protocol provenance recorded by the surrounding project names hospital branch
commit b80f4ebee945c6ac07be7732621fa8cd1dc056cd.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import struct
import time
from types import MappingProxyType
from typing import Iterable, Mapping, Sequence


HUB_HEADER = b"\x55\xaa"
HUB_PACKET_SIZE = 114
L5_HEADER = b"L5"
L5_PAYLOAD_SIZE = 103
SENSORS_PER_CHANNEL = 16
SENSOR_AXIS_COUNT = 3
MICROTESLA_PER_COUNT = 0.15

CONTROL_HEADER = b"\xa5\x5a"
CONTROL_FRAME_SIZE = 8
SYNC_COMMAND = 0x01
SYNC_ACK_COMMAND = 0x81

UINT32_MASK = 0xFFFFFFFF
UINT32_HALF_RANGE = 0x80000000
SOURCE_SEQUENCE_MODULUS = 16
SOURCE_SEQUENCE_HALF_RANGE = SOURCE_SEQUENCE_MODULUS // 2


class ProtocolError(ValueError):
    """Raised when a complete protocol unit is structurally invalid."""


class ControlFrameError(ProtocolError):
    """Raised when a control frame is invalid."""


class HubPacketError(ProtocolError):
    """Raised when a HUB packet or its nested L5 payload is invalid."""


@dataclass(frozen=True)
class L5Payload:
    """Decoded 103-byte L5 payload."""

    src_seq: int
    raw_xyz: tuple[tuple[int, int, int], ...]
    source_time_ms_u32: int

    @property
    def src_ts_ms(self) -> int:
        """Compatibility spelling for the source timestamp."""

        return self.source_time_ms_u32

    @property
    def source_timestamp_ms(self) -> int:
        return self.source_time_ms_u32

    @property
    def xyz_ut(self) -> tuple[tuple[float, float, float], ...]:
        return tuple(
            tuple(axis * MICROTESLA_PER_COUNT for axis in sample)
            for sample in self.raw_xyz
        )


@dataclass(frozen=True)
class HubRecord:
    """One channel record carried by a HUB packet."""

    channel_id: int
    channel_seq: int
    nucleo_time_ms_u32: int
    payload: L5Payload
    host_monotonic_ns: int

    @property
    def src_seq(self) -> int:
        return self.payload.src_seq

    @property
    def source_time_ms_u32(self) -> int:
        return self.payload.source_time_ms_u32

    @property
    def nucleo_ts_ms(self) -> int:
        return self.nucleo_time_ms_u32

    @property
    def xyz_ut(self) -> tuple[tuple[float, float, float], ...]:
        return self.payload.xyz_ut

    @property
    def first_sensor_index(self) -> int:
        return (self.channel_id - 1) * SENSORS_PER_CHANNEL


@dataclass(frozen=True)
class ChannelSequenceStats:
    """Sequence accounting for one HUB channel."""

    channel_id: int
    accepted_packets: int = 0
    sequence_drops: int = 0
    duplicate_packets: int = 0
    reordered_packets: int = 0
    last_sequence: int | None = None


@dataclass(frozen=True)
class ParserStats:
    """Immutable parser/grouping statistics snapshot."""

    bytes_received: int = 0
    bytes_discarded: int = 0
    hub_packets: int = 0
    malformed_packets: int = 0
    unexpected_channel_packets: int = 0
    duplicate_packets: int = 0
    reordered_packets: int = 0
    channel_sequence_drops: int = 0
    source_sequence_drops: int = 0
    completed_groups: int = 0
    duplicate_or_reordered_groups: int = 0
    incomplete_groups: int = 0
    timeout_groups: int = 0
    pending_evictions: int = 0
    channel_stats: tuple[ChannelSequenceStats, ...] = ()

    @property
    def dropped_packets(self) -> int:
        return self.channel_sequence_drops

    @property
    def dropped_groups(self) -> int:
        return (
            self.source_sequence_drops
            + self.incomplete_groups
            + self.duplicate_or_reordered_groups
        )


@dataclass(frozen=True)
class GroupedFrame:
    """A complete logical sensor frame assembled from expected channels."""

    records: tuple[HubRecord, ...]
    logical_seq: int
    source_time_ms_u32: int
    host_monotonic_ns: int
    src_seq: int
    channel_sequences: Mapping[int, int]
    channel_nucleo_timestamps_ms: Mapping[int, int]
    stats: ParserStats

    @property
    def records_by_channel(self) -> Mapping[int, HubRecord]:
        return MappingProxyType({record.channel_id: record for record in self.records})

    @property
    def per_channel_sequences(self) -> Mapping[int, int]:
        return self.channel_sequences

    @property
    def per_channel_nucleo_timestamps_ms(self) -> Mapping[int, int]:
        return self.channel_nucleo_timestamps_ms

    @property
    def source_timestamp_ms(self) -> int:
        return self.source_time_ms_u32

    @property
    def sensor_values_ut(
        self,
    ) -> Mapping[int, tuple[float, float, float]]:
        values: dict[int, tuple[float, float, float]] = {}
        for record in self.records:
            for local_index, xyz in enumerate(record.xyz_ut):
                values[record.first_sensor_index + local_index] = xyz
        return MappingProxyType(values)


@dataclass(frozen=True)
class ControlFrame:
    command: int
    sync_time_ms_u32: int
    checksum: int

    @property
    def cmd(self) -> int:
        return self.command

    @property
    def sync_time(self) -> int:
        return self.sync_time_ms_u32


def _require_uint(value: int, bits: int, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{label} must be an integer")
    maximum = (1 << bits) - 1
    if not 0 <= value <= maximum:
        raise ValueError(f"{label} must be in [0, {maximum}]")
    return value


def control_checksum(data: bytes | bytearray | memoryview) -> int:
    """Return the bytewise XOR checksum used by control frames."""

    checksum = 0
    for value in bytes(data):
        checksum ^= value
    return checksum


def build_control_frame(command: int, sync_time_ms_u32: int) -> bytes:
    command = _require_uint(command, 8, "command")
    sync_time_ms_u32 = _require_uint(sync_time_ms_u32, 32, "sync_time_ms_u32")
    body = CONTROL_HEADER + bytes((command,)) + struct.pack("<I", sync_time_ms_u32)
    return body + bytes((control_checksum(body),))


def build_sync_frame(sync_time_ms_u32: int) -> bytes:
    return build_control_frame(SYNC_COMMAND, sync_time_ms_u32)


def parse_control_frame(frame: bytes | bytearray | memoryview) -> ControlFrame:
    data = bytes(frame)
    if len(data) != CONTROL_FRAME_SIZE:
        raise ControlFrameError(
            f"control frame must be {CONTROL_FRAME_SIZE} bytes, got {len(data)}"
        )
    if data[:2] != CONTROL_HEADER:
        raise ControlFrameError("invalid control frame header")
    expected = control_checksum(data[:-1])
    if data[-1] != expected:
        raise ControlFrameError(
            f"invalid control checksum 0x{data[-1]:02x}; expected 0x{expected:02x}"
        )
    return ControlFrame(
        command=data[2],
        sync_time_ms_u32=struct.unpack_from("<I", data, 3)[0],
        checksum=data[-1],
    )


def is_sync_ack(
    frame: ControlFrame | bytes | bytearray | memoryview,
    expected_sync_time_ms_u32: int | None = None,
) -> bool:
    parsed = frame if isinstance(frame, ControlFrame) else parse_control_frame(frame)
    if parsed.command != SYNC_ACK_COMMAND:
        return False
    if expected_sync_time_ms_u32 is None:
        return True
    return parsed.sync_time_ms_u32 == _require_uint(
        expected_sync_time_ms_u32, 32, "expected_sync_time_ms_u32"
    )


def parse_sync_ack(
    frame: bytes | bytearray | memoryview,
    expected_sync_time_ms_u32: int | None = None,
) -> ControlFrame:
    parsed = parse_control_frame(frame)
    if not is_sync_ack(parsed, expected_sync_time_ms_u32):
        raise ControlFrameError("control frame is not the expected SYNC ACK")
    return parsed


def parse_l5_payload(payload: bytes | bytearray | memoryview) -> L5Payload:
    data = bytes(payload)
    if len(data) != L5_PAYLOAD_SIZE:
        raise HubPacketError(
            f"L5 payload must be {L5_PAYLOAD_SIZE} bytes, got {len(data)}"
        )
    if data[:2] != L5_HEADER:
        raise HubPacketError("invalid L5 payload header")
    src_seq = data[2]
    if src_seq >= SOURCE_SEQUENCE_MODULUS:
        raise HubPacketError("L5 src_seq must be in [0, 15]")
    axes = struct.unpack_from(">48h", data, 3)
    raw_xyz = tuple(
        (axes[index], axes[index + 1], axes[index + 2])
        for index in range(0, len(axes), SENSOR_AXIS_COUNT)
    )
    source_time_ms_u32 = struct.unpack_from(">I", data, 99)[0]
    return L5Payload(
        src_seq=src_seq,
        raw_xyz=raw_xyz,
        source_time_ms_u32=source_time_ms_u32,
    )


def parse_hub_packet(
    packet: bytes | bytearray | memoryview,
    host_monotonic_ns: int | None = None,
) -> HubRecord:
    data = bytes(packet)
    if len(data) != HUB_PACKET_SIZE:
        raise HubPacketError(
            f"HUB packet must be {HUB_PACKET_SIZE} bytes, got {len(data)}"
        )
    if data[:2] != HUB_HEADER:
        raise HubPacketError("invalid HUB packet header")
    channel_id = data[2]
    if channel_id not in (1, 2, 3, 4):
        raise HubPacketError("HUB channel_id must be in [1, 4]")
    if host_monotonic_ns is None:
        host_monotonic_ns = time.monotonic_ns()
    host_monotonic_ns = _require_nonnegative_int(host_monotonic_ns, "host_monotonic_ns")
    return HubRecord(
        channel_id=channel_id,
        channel_seq=struct.unpack_from("<I", data, 3)[0],
        nucleo_time_ms_u32=struct.unpack_from("<I", data, 7)[0],
        payload=parse_l5_payload(data[11:]),
        host_monotonic_ns=host_monotonic_ns,
    )


def _require_nonnegative_int(value: int, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{label} must be an integer")
    if value < 0:
        raise ValueError(f"{label} must be non-negative")
    return value


def signed_u32_delta(value: int, reference: int) -> int:
    """Return the shortest signed modular delta from ``reference`` to ``value``."""

    value = _require_uint(value, 32, "value")
    reference = _require_uint(reference, 32, "reference")
    delta = (value - reference) & UINT32_MASK
    if delta == UINT32_HALF_RANGE:
        raise ValueError("u32 values are exactly half a range apart")
    return delta if delta < UINT32_HALF_RANGE else delta - (UINT32_MASK + 1)


def median_u32(values: Iterable[int]) -> int:
    """Compute an integer median after unwrapping u32 values around the first."""

    checked = tuple(_require_uint(value, 32, "timestamp") for value in values)
    if not checked:
        raise ValueError("at least one u32 value is required")
    anchor = checked[0]
    relative = sorted(signed_u32_delta(value, anchor) for value in checked)
    if relative[-1] - relative[0] >= UINT32_HALF_RANGE:
        raise ValueError("u32 values do not occupy an unambiguous half-range")
    middle = len(relative) // 2
    if len(relative) % 2:
        median_delta = relative[middle]
    else:
        lower = relative[middle - 1]
        upper = relative[middle]
        quotient, remainder = divmod(lower + upper, 2)
        # Match numpy/Python round-to-nearest-even used by the pinned hospital
        # parser without converting the relative integers through float.
        median_delta = quotient + (1 if remainder and quotient % 2 else 0)
    return (anchor + median_delta) & UINT32_MASK


class ControlStreamParser:
    """Resynchronizing parser for control frames embedded in a byte stream."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes | bytearray | memoryview) -> tuple[ControlFrame, ...]:
        self._buffer.extend(bytes(data))
        frames: list[ControlFrame] = []
        while True:
            header_at = self._buffer.find(CONTROL_HEADER)
            if header_at < 0:
                keep = 1 if self._buffer.endswith(CONTROL_HEADER[:1]) else 0
                if len(self._buffer) > keep:
                    del self._buffer[: len(self._buffer) - keep]
                break
            if header_at:
                del self._buffer[:header_at]
            if len(self._buffer) < CONTROL_FRAME_SIZE:
                break
            candidate = bytes(self._buffer[:CONTROL_FRAME_SIZE])
            try:
                frame = parse_control_frame(candidate)
            except ControlFrameError:
                del self._buffer[0]
                continue
            del self._buffer[:CONTROL_FRAME_SIZE]
            frames.append(frame)
        return tuple(frames)

    def reset(self) -> None:
        self._buffer.clear()


@dataclass
class _MutableChannelStats:
    accepted_packets: int = 0
    sequence_drops: int = 0
    duplicate_packets: int = 0
    reordered_packets: int = 0
    last_sequence: int | None = None


@dataclass
class _PendingGroup:
    logical_seq: int
    src_seq: int
    first_seen_ns: int
    records: dict[int, HubRecord] = field(default_factory=dict)


class HubStreamParser:
    """Resynchronize HUB packets and group channels into complete frames.

    Source sequence numbers are unwrapped with the modular forward window
    1..7.  Delta 8 is deliberately rejected as ambiguous.  Existing pending
    groups are consulted first so interleaved channel delivery is supported.
    """

    def __init__(
        self,
        expected_channels: Sequence[int] = (1, 2, 3, 4),
        *,
        incomplete_group_timeout_s: float = 0.04,
        pending_limit: int = 8,
    ) -> None:
        channels = tuple(expected_channels)
        if not channels:
            raise ValueError("expected_channels must not be empty")
        if len(set(channels)) != len(channels):
            raise ValueError("expected_channels must be unique")
        if any(
            isinstance(channel, bool)
            or not isinstance(channel, int)
            or channel not in (1, 2, 3, 4)
            for channel in channels
        ):
            raise ValueError("expected_channels must contain only channels 1..4")
        if (
            isinstance(incomplete_group_timeout_s, bool)
            or not isinstance(incomplete_group_timeout_s, (int, float))
            or incomplete_group_timeout_s <= 0.0
        ):
            raise ValueError("incomplete_group_timeout_s must be positive")
        if (
            isinstance(pending_limit, bool)
            or not isinstance(pending_limit, int)
            or not 1 <= pending_limit <= SOURCE_SEQUENCE_HALF_RANGE
        ):
            raise ValueError("pending_limit must be in [1, 8]")

        self.expected_channels = channels
        self._expected_channel_set = frozenset(channels)
        self.incomplete_group_timeout_ns = int(
            float(incomplete_group_timeout_s) * 1_000_000_000
        )
        if self.incomplete_group_timeout_ns <= 0:
            raise ValueError("incomplete_group_timeout_s is below nanosecond precision")
        self.pending_limit = pending_limit
        self._buffer = bytearray()
        self._pending: dict[int, _PendingGroup] = {}
        self._latest_logical_seq: int | None = None
        self._latest_src_seq: int | None = None
        self._last_emitted_logical_seq: int | None = None
        self._last_emitted_source_time_ms_u32: int | None = None
        self._channel_stats = {channel: _MutableChannelStats() for channel in channels}

        self._bytes_received = 0
        self._bytes_discarded = 0
        self._hub_packets = 0
        self._malformed_packets = 0
        self._unexpected_channel_packets = 0
        self._duplicate_packets = 0
        self._reordered_packets = 0
        self._channel_sequence_drops = 0
        self._source_sequence_drops = 0
        self._completed_groups = 0
        self._duplicate_or_reordered_groups = 0
        self._incomplete_groups = 0
        self._timeout_groups = 0
        self._pending_evictions = 0

    @property
    def stats(self) -> ParserStats:
        return self._snapshot_stats()

    @property
    def pending_group_count(self) -> int:
        return len(self._pending)

    def feed(
        self,
        data: bytes | bytearray | memoryview,
        host_monotonic_ns: int | None = None,
    ) -> tuple[GroupedFrame, ...]:
        if host_monotonic_ns is None:
            host_monotonic_ns = time.monotonic_ns()
        now_ns = _require_nonnegative_int(host_monotonic_ns, "host_monotonic_ns")
        incoming = bytes(data)
        self._bytes_received += len(incoming)
        self._expire_due(now_ns)
        self._buffer.extend(incoming)

        completed: list[GroupedFrame] = []
        while True:
            header_at = self._buffer.find(HUB_HEADER)
            if header_at < 0:
                keep = 1 if self._buffer.endswith(HUB_HEADER[:1]) else 0
                discard = len(self._buffer) - keep
                if discard:
                    del self._buffer[:discard]
                    self._bytes_discarded += discard
                break
            if header_at:
                del self._buffer[:header_at]
                self._bytes_discarded += header_at
            if len(self._buffer) < HUB_PACKET_SIZE:
                break
            candidate = bytes(self._buffer[:HUB_PACKET_SIZE])
            try:
                record = parse_hub_packet(candidate, now_ns)
            except HubPacketError:
                del self._buffer[0]
                self._bytes_discarded += 1
                self._malformed_packets += 1
                continue
            del self._buffer[:HUB_PACKET_SIZE]
            self._hub_packets += 1
            frame = self._accept_record(record)
            if frame is not None:
                completed.append(frame)
        return tuple(completed)

    def poll(self, host_monotonic_ns: int | None = None) -> ParserStats:
        """Expire timed-out groups even when no serial bytes arrive."""

        self.feed(b"", host_monotonic_ns)
        return self.stats

    def close(self) -> ParserStats:
        """Retire all incomplete state and account for any trailing bytes."""

        if self._pending:
            self._incomplete_groups += len(self._pending)
            self._pending.clear()
        if self._buffer:
            self._bytes_discarded += len(self._buffer)
            self._buffer.clear()
        return self.stats

    def reset(self) -> None:
        """Reset all stream, grouping, and statistics state."""

        timeout_s = self.incomplete_group_timeout_ns / 1_000_000_000
        self.__init__(
            self.expected_channels,
            incomplete_group_timeout_s=timeout_s,
            pending_limit=self.pending_limit,
        )

    def _accept_record(self, record: HubRecord) -> GroupedFrame | None:
        channel = record.channel_id
        if channel not in self._expected_channel_set:
            self._unexpected_channel_packets += 1
            return None

        channel_stats = self._channel_stats[channel]
        sequence_drop_count = 0
        if channel_stats.last_sequence is not None:
            sequence_delta = (
                record.channel_seq - channel_stats.last_sequence
            ) & UINT32_MASK
            if sequence_delta == 0:
                channel_stats.duplicate_packets += 1
                self._duplicate_packets += 1
                return None
            if sequence_delta >= UINT32_HALF_RANGE:
                channel_stats.reordered_packets += 1
                self._reordered_packets += 1
                return None
            sequence_drop_count = sequence_delta - 1

        pending = self._pending_for_src_seq(record.src_seq)
        is_new_group = pending is None
        source_drop_count = 0
        if pending is None:
            if self._latest_logical_seq is None:
                logical_seq = record.src_seq
            else:
                assert self._latest_src_seq is not None
                source_delta = (record.src_seq - self._latest_src_seq) & 0x0F
                if source_delta == 0:
                    self._duplicate_packets += 1
                    return None
                if source_delta >= SOURCE_SEQUENCE_HALF_RANGE:
                    self._reordered_packets += 1
                    return None
                logical_seq = self._latest_logical_seq + source_delta
                source_drop_count = source_delta - 1
            if (
                self._last_emitted_logical_seq is not None
                and logical_seq <= self._last_emitted_logical_seq
            ):
                self._reordered_packets += 1
                return None
        else:
            logical_seq = pending.logical_seq
            if channel in pending.records:
                self._duplicate_packets += 1
                return None
            if (
                self._last_emitted_logical_seq is not None
                and logical_seq <= self._last_emitted_logical_seq
            ):
                self._reordered_packets += 1
                return None

        # Commit sequence baselines only after every rejection check succeeds.
        channel_stats.last_sequence = record.channel_seq
        channel_stats.accepted_packets += 1
        channel_stats.sequence_drops += sequence_drop_count
        self._channel_sequence_drops += sequence_drop_count

        if is_new_group:
            while len(self._pending) >= self.pending_limit:
                self._evict_oldest_pending()
            pending = _PendingGroup(
                logical_seq=logical_seq,
                src_seq=record.src_seq,
                first_seen_ns=record.host_monotonic_ns,
            )
            self._pending[logical_seq] = pending
            self._latest_logical_seq = logical_seq
            self._latest_src_seq = record.src_seq
            self._source_sequence_drops += source_drop_count

        assert pending is not None
        pending.records[channel] = record
        if set(pending.records) != self._expected_channel_set:
            return None

        del self._pending[logical_seq]
        records = tuple(
            pending.records[channel] for channel in sorted(self.expected_channels)
        )
        source_time_ms_u32 = median_u32(record.source_time_ms_u32 for record in records)
        if self._last_emitted_source_time_ms_u32 is not None:
            try:
                source_time_delta = signed_u32_delta(
                    source_time_ms_u32,
                    self._last_emitted_source_time_ms_u32,
                )
            except ValueError:
                source_time_delta = 0
            if source_time_delta <= 0:
                self._duplicate_or_reordered_groups += 1
                return None

        older = [key for key in self._pending if key < logical_seq]
        if older:
            self._incomplete_groups += len(older)
            for key in older:
                del self._pending[key]
        self._last_emitted_logical_seq = logical_seq
        self._last_emitted_source_time_ms_u32 = source_time_ms_u32
        self._completed_groups += 1
        output_logical_seq = self._completed_groups - 1

        return GroupedFrame(
            records=records,
            logical_seq=output_logical_seq,
            source_time_ms_u32=source_time_ms_u32,
            host_monotonic_ns=max(record.host_monotonic_ns for record in records),
            src_seq=pending.src_seq,
            channel_sequences=MappingProxyType(
                {record.channel_id: record.channel_seq for record in records}
            ),
            channel_nucleo_timestamps_ms=MappingProxyType(
                {record.channel_id: record.nucleo_time_ms_u32 for record in records}
            ),
            stats=self._snapshot_stats(),
        )

    def _pending_for_src_seq(self, src_seq: int) -> _PendingGroup | None:
        matches = [
            pending for pending in self._pending.values() if pending.src_seq == src_seq
        ]
        if not matches:
            return None
        return max(matches, key=lambda pending: pending.logical_seq)

    def _expire_due(self, now_ns: int) -> None:
        expired = [
            logical_seq
            for logical_seq, pending in self._pending.items()
            if now_ns - pending.first_seen_ns >= self.incomplete_group_timeout_ns
        ]
        if expired:
            self._incomplete_groups += len(expired)
            self._timeout_groups += len(expired)
            for logical_seq in expired:
                del self._pending[logical_seq]

    def _evict_oldest_pending(self) -> None:
        if not self._pending:
            return
        logical_seq, _ = min(
            self._pending.items(),
            key=lambda item: (item[1].first_seen_ns, item[0]),
        )
        del self._pending[logical_seq]
        self._incomplete_groups += 1
        self._pending_evictions += 1

    def _snapshot_stats(self) -> ParserStats:
        return ParserStats(
            bytes_received=self._bytes_received,
            bytes_discarded=self._bytes_discarded,
            hub_packets=self._hub_packets,
            malformed_packets=self._malformed_packets,
            unexpected_channel_packets=self._unexpected_channel_packets,
            duplicate_packets=self._duplicate_packets,
            reordered_packets=self._reordered_packets,
            channel_sequence_drops=self._channel_sequence_drops,
            source_sequence_drops=self._source_sequence_drops,
            completed_groups=self._completed_groups,
            duplicate_or_reordered_groups=self._duplicate_or_reordered_groups,
            incomplete_groups=self._incomplete_groups,
            timeout_groups=self._timeout_groups,
            pending_evictions=self._pending_evictions,
            channel_stats=tuple(
                ChannelSequenceStats(
                    channel_id=channel,
                    accepted_packets=stats.accepted_packets,
                    sequence_drops=stats.sequence_drops,
                    duplicate_packets=stats.duplicate_packets,
                    reordered_packets=stats.reordered_packets,
                    last_sequence=stats.last_sequence,
                )
                for channel, stats in sorted(self._channel_stats.items())
            ),
        )


# Short compatibility spellings for callers that treat the nested L5 payload
# and the stream parser as the primary protocol types.
L5Frame = L5Payload
ChannelRecord = HubRecord
NucleoStreamParser = HubStreamParser
StreamParser = HubStreamParser
parse_l5 = parse_l5_payload
decode_control_frame = parse_control_frame
make_sync_frame = build_sync_frame


__all__ = [
    "CONTROL_FRAME_SIZE",
    "CONTROL_HEADER",
    "ChannelRecord",
    "ChannelSequenceStats",
    "ControlFrame",
    "ControlFrameError",
    "ControlStreamParser",
    "GroupedFrame",
    "HUB_HEADER",
    "HUB_PACKET_SIZE",
    "HubPacketError",
    "HubRecord",
    "HubStreamParser",
    "L5Frame",
    "L5Payload",
    "L5_PAYLOAD_SIZE",
    "MICROTESLA_PER_COUNT",
    "NucleoStreamParser",
    "ParserStats",
    "ProtocolError",
    "SENSORS_PER_CHANNEL",
    "SOURCE_SEQUENCE_MODULUS",
    "SYNC_ACK_COMMAND",
    "SYNC_COMMAND",
    "StreamParser",
    "build_control_frame",
    "build_sync_frame",
    "control_checksum",
    "decode_control_frame",
    "is_sync_ack",
    "make_sync_frame",
    "median_u32",
    "parse_control_frame",
    "parse_hub_packet",
    "parse_l5",
    "parse_l5_payload",
    "parse_sync_ack",
    "signed_u32_delta",
]
