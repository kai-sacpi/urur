# SPDX-License-Identifier: Apache-2.0
"""Optional serial capture and run-artifact finalization for Nucleo data."""

from __future__ import annotations

from bisect import bisect_left
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import statistics
import tempfile
import threading
import time
from typing import Any, Callable, Mapping, Sequence, TextIO
import uuid

from .protocol import (
    CONTROL_FRAME_SIZE,
    CONTROL_HEADER,
    SYNC_ACK_COMMAND,
    GroupedFrame,
    HubStreamParser,
    ParserStats,
    build_sync_frame,
    parse_control_frame,
)


PARSER_PROVENANCE_COMMIT = "b80f4ebee945c6ac07be7732621fa8cd1dc056cd"
PARSER_PROVENANCE_DEFAULTS: Mapping[str, Any] = {
    "schema_version": 1,
    "repository": "https://github.com/hxhou0207/magnet_localization_real.git",
    "branch": "hospital",
    "commit": PARSER_PROVENANCE_COMMIT,
    "protocol_path": "magloc/sources/hardware/nucleo/protocol.py",
    "protocol_blob": "9408a3043199c7338f2a6f26b43db4c4b7e5089d",
    "protocol_sha256": (
        "fe6510dfe3b819faeb5fad3f7c6d8535a6af603ea8eb08e1a2359590c428bbb3"
    ),
    "integration": "independent_wire_protocol_reimplementation",
}

_AXES = ("x", "y", "z")
_CHANNEL_IDS = (1, 2, 3, 4)
_SENSOR_COUNT = 64


class NucleoCaptureError(RuntimeError):
    """Raised when capture cannot be started or stopped without corrupting evidence."""


@dataclass(frozen=True)
class NucleoCaptureConfig:
    """Validated serial and grouping settings for one capture session."""

    port: str
    baudrate: int = 115200
    channels: tuple[int, ...] = (1, 2, 3, 4)
    perform_sync: bool = True
    settle_time_s: float = 0.2
    serial_timeout_s: float = 0.02
    write_timeout_s: float = 1.0
    sync_timeout_s: float = 1.0
    sync_retry_interval_s: float = 0.05
    sync_retries: int = 20
    preflight_timeout_s: float = 5.0
    liveness_timeout_s: float = 0.5
    max_mapping_offset_s: float = 0.05
    incomplete_group_timeout_s: float = 0.04
    pending_group_limit: int = 8
    read_size: int = 4096

    def __post_init__(self) -> None:
        if not isinstance(self.port, str) or not self.port.strip():
            raise ValueError("port must be a non-empty string")
        object.__setattr__(self, "port", self.port.strip())
        if (
            isinstance(self.baudrate, bool)
            or not isinstance(self.baudrate, int)
            or self.baudrate <= 0
        ):
            raise ValueError("baudrate must be a positive integer")

        channels = tuple(self.channels)
        if not channels:
            raise ValueError("channels must not be empty")
        if len(set(channels)) != len(channels):
            raise ValueError("channels must be unique")
        if any(
            isinstance(channel, bool)
            or not isinstance(channel, int)
            or channel not in _CHANNEL_IDS
            for channel in channels
        ):
            raise ValueError("channels must contain only integer channel IDs 1..4")
        if channels != tuple(sorted(channels)):
            raise ValueError("channels must be in ascending order")
        object.__setattr__(self, "channels", channels)

        if not isinstance(self.perform_sync, bool):
            raise ValueError("perform_sync must be a boolean")

        for name in (
            "settle_time_s",
            "serial_timeout_s",
            "write_timeout_s",
            "sync_timeout_s",
            "sync_retry_interval_s",
            "preflight_timeout_s",
            "liveness_timeout_s",
            "max_mapping_offset_s",
            "incomplete_group_timeout_s",
        ):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                or float(value) <= 0.0
            ):
                raise ValueError(f"{name} must be finite and positive")
            object.__setattr__(self, name, float(value))

        if (
            isinstance(self.sync_retries, bool)
            or not isinstance(self.sync_retries, int)
            or self.sync_retries <= 0
        ):
            raise ValueError("sync_retries must be a positive integer")
        if (
            isinstance(self.pending_group_limit, bool)
            or not isinstance(self.pending_group_limit, int)
            or not 1 <= self.pending_group_limit <= 8
        ):
            raise ValueError("pending_group_limit must be in [1, 8]")
        if (
            isinstance(self.read_size, bool)
            or not isinstance(self.read_size, int)
            or self.read_size < CONTROL_FRAME_SIZE
        ):
            raise ValueError(
                f"read_size must be an integer of at least {CONTROL_FRAME_SIZE}"
            )

    @property
    def expected_channels(self) -> tuple[int, ...]:
        return self.channels

    @property
    def read_timeout_s(self) -> float:
        return self.serial_timeout_s


@dataclass(frozen=True)
class NucleoCaptureSummary:
    """Stopped capture state consumed by :func:`finalize_nucleo_capture`."""

    staging_path: Path
    config: NucleoCaptureConfig
    workspace: Path | None = None
    started_sync_time_ns: int = 0
    stopped_sync_time_ns: int = 0
    sync_time_ms_u32: int | None = None
    sync_rtt_ns: int | None = None
    sync_attempts: int = 0
    frames_written: int = 0
    parser_stats: ParserStats = ParserStats()
    capture_error: str | None = None

    @property
    def error(self) -> str | None:
        return self.capture_error

    @property
    def status(self) -> str:
        return "incomplete" if self.capture_error else "stopped"


CaptureSummary = NucleoCaptureSummary


@dataclass(frozen=True)
class NucleoFinalizeResult:
    sensor_telemetry_path: Path
    manifest_path: Path
    status: str
    message: str
    mapped_frame_count: int = 0
    outside_window_frame_count: int = 0

    @property
    def complete(self) -> bool:
        return self.status == "complete"


FinalizeResult = NucleoFinalizeResult


_RAW_BASE_COLUMNS = (
    "sensor_rx_sync_time_ns",
    "sensor_rx_ros_time_ns",
    "parser_host_monotonic_ns",
    "logical_seq",
    "src_seq",
    "source_time_ms_u32",
    "channel_ids",
)
_RAW_STATS_COLUMNS = (
    "parser_hub_packets",
    "parser_completed_groups",
    "parser_duplicate_or_reordered_groups",
    "parser_incomplete_groups",
    "parser_timeout_groups",
    "parser_duplicate_packets",
    "parser_reordered_packets",
    "parser_channel_sequence_drops",
    "parser_source_sequence_drops",
    "parser_bytes_discarded",
    "parser_malformed_packets",
    "parser_unexpected_channel_packets",
    "parser_pending_evictions",
)

_PARSER_INTEGRITY_COUNTERS: Mapping[str, str] = {
    "parser_duplicate_or_reordered_groups": "duplicate/reordered groups",
    "parser_incomplete_groups": "incomplete groups",
    "parser_duplicate_packets": "duplicate packets",
    "parser_reordered_packets": "reordered packets",
    "parser_channel_sequence_drops": "channel sequence drops",
    "parser_source_sequence_drops": "source sequence drops",
    "parser_bytes_discarded": "discarded stream bytes",
    "parser_malformed_packets": "malformed packets",
    "parser_unexpected_channel_packets": "unexpected-channel packets",
    "parser_pending_evictions": "pending-group evictions",
}


def raw_capture_columns() -> tuple[str, ...]:
    channel_columns = tuple(
        column
        for channel in _CHANNEL_IDS
        for column in (
            f"channel_{channel}_seq",
            f"channel_{channel}_nucleo_time_ms_u32",
            f"channel_{channel}_source_time_ms_u32",
        )
    )
    sensor_columns = tuple(
        f"sensor_{sensor_index:02d}_{axis}_ut"
        for sensor_index in range(_SENSOR_COUNT)
        for axis in _AXES
    )
    return _RAW_BASE_COLUMNS + channel_columns + _RAW_STATS_COLUMNS + sensor_columns


RAW_CAPTURE_COLUMNS = raw_capture_columns()


class NucleoCaptureSession:
    """A SYNC-gated serial capture session with a background CSV writer."""

    def __init__(
        self,
        workspace: str | os.PathLike[str],
        config: NucleoCaptureConfig,
        ros_time_ns_factory: Callable[[], int],
    ) -> None:
        if not isinstance(config, NucleoCaptureConfig):
            raise TypeError("config must be a NucleoCaptureConfig")
        if not callable(ros_time_ns_factory):
            raise TypeError("ros_time_ns_factory must be callable")
        self.workspace = Path(workspace).expanduser().resolve()
        self.config = config
        self.ros_time_ns_factory = ros_time_ns_factory

        self._parser = HubStreamParser(
            expected_channels=config.channels,
            incomplete_group_timeout_s=config.incomplete_group_timeout_s,
            pending_limit=config.pending_group_limit,
        )
        self._serial: Any | None = None
        self._stream: TextIO | None = None
        self._writer: csv.DictWriter[str] | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._first_frame_event = threading.Event()
        self._reader_done_event = threading.Event()
        self._state_lock = threading.Lock()
        self._started = False
        self._stopped = False
        self._summary: NucleoCaptureSummary | None = None
        self._capture_error: str | None = None
        self._frames_written = 0
        self._staging_path: Path | None = None
        self._started_sync_time_ns = 0
        self._stopped_sync_time_ns = 0
        self._sync_time_ms_u32: int | None = None
        self._sync_rtt_ns: int | None = None
        self._sync_attempts = 0
        self._initial_stream_bytes = b""

    @property
    def staging_path(self) -> Path | None:
        return self._staging_path

    @property
    def capture_error(self) -> str | None:
        return self._capture_error

    @property
    def frames_written(self) -> int:
        return self._frames_written

    def start(self) -> "NucleoCaptureSession":
        with self._state_lock:
            if self._started:
                raise NucleoCaptureError("capture session has already been started")
            self._started = True

        runs_directory = self.workspace / "runs"
        runs_directory.mkdir(parents=True, exist_ok=True)
        serial_port: Any | None = None
        try:
            serial_port = self._open_serial()
            self._serial = serial_port
            time.sleep(self.config.settle_time_s)
            # A reused SYNC session may have accumulated bytes in the USB CDC
            # input queue while the previous capture was being finalized.  Drop
            # those stale bytes before establishing this session's parser
            # baseline.  This is deliberately independent of ``perform_sync``:
            # clearing the host-side buffer does not transmit a second SYNC to
            # the board and therefore preserves the one-handshake contract.
            serial_port.reset_input_buffer()
            if self.config.perform_sync:
                serial_port.reset_output_buffer()
                (
                    self._sync_time_ms_u32,
                    self._sync_rtt_ns,
                    self._sync_attempts,
                    self._initial_stream_bytes,
                ) = self._synchronize(serial_port)

            self._staging_path = self._new_staging_path(runs_directory)
            self._stream = self._staging_path.open("x", encoding="utf-8", newline="")
            self._writer = csv.DictWriter(
                self._stream, fieldnames=RAW_CAPTURE_COLUMNS, lineterminator="\n"
            )
            self._writer.writeheader()
            self._stream.flush()
            self._started_sync_time_ns = time.monotonic_ns()
            self._thread = threading.Thread(
                target=self._reader_loop,
                name="hx-waypoint-nucleo-capture",
                daemon=True,
            )
            self._thread.start()
        except Exception as exc:
            if serial_port is not None:
                _best_effort_close(serial_port)
            if self._stream is not None:
                _best_effort_close(self._stream)
            self._record_error(f"capture setup failed: {type(exc).__name__}: {exc}")
            raise NucleoCaptureError(self._capture_error) from exc

        deadline = time.monotonic() + self.config.preflight_timeout_s
        while not self._first_frame_event.is_set():
            if self._reader_done_event.is_set():
                message = (
                    self._capture_error or "capture reader stopped before preflight"
                )
                self._stop_after_failed_preflight(message)
                raise NucleoCaptureError(message)
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                message = (
                    "Nucleo preflight timed out without a complete grouped frame "
                    f"after {self.config.preflight_timeout_s:.3f} s"
                )
                self._stop_after_failed_preflight(message)
                raise NucleoCaptureError(message)
            self._first_frame_event.wait(min(remaining, 0.05))
        return self

    def stop(self) -> NucleoCaptureSummary:
        with self._state_lock:
            if not self._started:
                raise NucleoCaptureError("capture session has not been started")
            if self._summary is not None:
                return self._summary
            self._stopped = True

        self._stop_event.set()
        serial_port = self._serial
        if serial_port is not None:
            cancel_read = getattr(serial_port, "cancel_read", None)
            if callable(cancel_read):
                try:
                    cancel_read()
                except Exception:
                    pass

        thread = self._thread
        if thread is not None:
            thread.join(timeout=max(1.0, self.config.serial_timeout_s * 4.0))
            if thread.is_alive() and serial_port is not None:
                _best_effort_close(serial_port)
                thread.join(timeout=1.0)
            if thread.is_alive():
                message = "capture reader did not stop within the join timeout"
                self._record_error(message)
                # The reader still owns the CSV writer, parser, and ROS clock
                # callback.  Closing/finalizing them concurrently would create a
                # corrupt artifact and a use-after-destroy race with the ROS node.
                raise NucleoCaptureError(message)

        if serial_port is not None:
            try:
                serial_port.close()
            except Exception as exc:
                self._record_error(f"serial close failed: {type(exc).__name__}: {exc}")
        if self._stream is not None:
            try:
                self._stream.flush()
                self._stream.close()
            except Exception as exc:
                self._record_error(
                    f"staging CSV close failed: {type(exc).__name__}: {exc}"
                )

        self._stopped_sync_time_ns = time.monotonic_ns()
        parser_stats = self._parser.close()
        if self._staging_path is None:
            # Only reachable after a setup failure, where stop() is rarely useful.
            self._staging_path = self.workspace / "runs" / "nucleo_capture_missing.csv"
        summary = NucleoCaptureSummary(
            staging_path=self._staging_path,
            config=self.config,
            workspace=self.workspace,
            started_sync_time_ns=self._started_sync_time_ns,
            stopped_sync_time_ns=self._stopped_sync_time_ns,
            sync_time_ms_u32=self._sync_time_ms_u32,
            sync_rtt_ns=self._sync_rtt_ns,
            sync_attempts=self._sync_attempts,
            frames_written=self._frames_written,
            parser_stats=parser_stats,
            capture_error=self._capture_error,
        )
        with self._state_lock:
            self._summary = summary
        return summary

    def __enter__(self) -> "NucleoCaptureSession":
        return self.start()

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.stop()

    def _open_serial(self) -> Any:
        try:
            serial_module = importlib.import_module("serial")
        except ImportError as exc:
            raise NucleoCaptureError(
                "pyserial is required only when Nucleo capture is enabled"
            ) from exc
        return serial_module.Serial(
            port=self.config.port,
            baudrate=self.config.baudrate,
            bytesize=getattr(serial_module, "EIGHTBITS", 8),
            parity=getattr(serial_module, "PARITY_NONE", "N"),
            stopbits=getattr(serial_module, "STOPBITS_ONE", 1),
            timeout=self.config.serial_timeout_s,
            write_timeout=self.config.write_timeout_s,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )

    def _synchronize(self, serial_port: Any) -> tuple[int, int, int, bytes]:
        started_ns = time.monotonic_ns()
        deadline_ns = started_ns + int(self.config.sync_timeout_s * 1_000_000_000)
        retry_interval_ns = int(self.config.sync_retry_interval_s * 1_000_000_000)
        next_send_ns = started_ns
        attempts = 0
        sent_at_by_sync_time: dict[int, int] = {}
        received = bytearray()

        while time.monotonic_ns() < deadline_ns:
            now_ns = time.monotonic_ns()
            if now_ns >= next_send_ns and attempts < self.config.sync_retries:
                sync_time_ms_u32 = (now_ns // 1_000_000) & 0xFFFFFFFF
                frame = build_sync_frame(sync_time_ms_u32)
                serial_port.write(frame)
                serial_port.flush()
                sent_at_by_sync_time[sync_time_ms_u32] = time.monotonic_ns()
                attempts += 1
                next_send_ns = now_ns + retry_interval_ns

            chunk = serial_port.read(self.config.read_size)
            if chunk:
                received.extend(bytes(chunk))
                match = _find_matching_sync_ack(received, sent_at_by_sync_time)
                if match is not None:
                    frame_start, frame_end, sync_time_ms_u32 = match
                    ack_ns = time.monotonic_ns()
                    sent_ns = sent_at_by_sync_time[sync_time_ms_u32]
                    preserved = bytes(received[:frame_start] + received[frame_end:])
                    return sync_time_ms_u32, ack_ns - sent_ns, attempts, preserved
            elif attempts >= self.config.sync_retries and now_ns >= next_send_ns:
                break

        raise NucleoCaptureError(
            "Nucleo SYNC did not receive a valid matching ACK within "
            f"{self.config.sync_timeout_s:.3f} s ({attempts} attempts)"
        )

    def _read_stream_chunk(self, serial_port: Any) -> bytes:
        # pyserial read(size) waits for size bytes or the serial timeout. Asking
        # for 4096 at this stream rate adds up to 20 ms of host-side batching,
        # which can turn an on-time group into a false 40 ms parser timeout.
        # Drain bytes already queued; when idle, wait only for the first byte.
        # Keep the normal read timeout so an idle stream still gets polled for
        # genuine group expiry. No deadlines, timestamps or samples are altered.
        available = int(getattr(serial_port, "in_waiting", 0))
        size = min(self.config.read_size, max(1, available))
        return serial_port.read(size)

    def _reader_loop(self) -> None:
        try:
            if self._initial_stream_bytes:
                self._consume_stream_bytes(self._initial_stream_bytes)
            while not self._stop_event.is_set():
                serial_port = self._serial
                if serial_port is None:
                    raise RuntimeError("serial port disappeared during capture")
                chunk = self._read_stream_chunk(serial_port)
                now_ns = time.monotonic_ns()
                if self._stop_event.is_set():
                    break
                if chunk:
                    frames = self._parser.feed(bytes(chunk), now_ns)
                    self._write_frames(frames)
                else:
                    self._parser.poll(now_ns)
        except BaseException as exc:
            if not self._stop_event.is_set():
                self._record_error(
                    f"capture reader failed: {type(exc).__name__}: {exc}"
                )
        finally:
            self._reader_done_event.set()

    def _consume_stream_bytes(self, data: bytes) -> None:
        frames = self._parser.feed(data, time.monotonic_ns())
        self._write_frames(frames)

    def _write_frames(self, frames: Sequence[GroupedFrame]) -> None:
        writer = self._writer
        stream = self._stream
        if writer is None or stream is None:
            raise RuntimeError("staging CSV writer is unavailable")
        for frame in frames:
            if self._stop_event.is_set():
                break
            # The parser timestamp is captured immediately after the serial read.
            # Reusing it avoids adding per-row CSV flush latency to synchronization.
            sensor_rx_sync_time_ns = frame.host_monotonic_ns
            sensor_rx_ros_time_ns = _require_clock_ns(
                self.ros_time_ns_factory(), "ros_time_ns_factory result"
            )
            writer.writerow(
                _raw_row(
                    frame,
                    sensor_rx_sync_time_ns=sensor_rx_sync_time_ns,
                    sensor_rx_ros_time_ns=sensor_rx_ros_time_ns,
                )
            )
            self._frames_written += 1
            self._first_frame_event.set()

    def _new_staging_path(self, runs_directory: Path) -> Path:
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        nonce = uuid.uuid4().hex[:8]
        return runs_directory / f"nucleo_capture__{timestamp}__{nonce}.staging.csv"

    def _record_error(self, message: str) -> None:
        with self._state_lock:
            if self._capture_error is None:
                self._capture_error = message

    def _stop_after_failed_preflight(self, message: str) -> None:
        self._record_error(message)
        try:
            self.stop()
        except Exception:
            pass


def _find_matching_sync_ack(
    buffer: bytearray,
    sent_at_by_sync_time: Mapping[int, int],
) -> tuple[int, int, int] | None:
    search_from = 0
    while True:
        frame_start = buffer.find(CONTROL_HEADER, search_from)
        if frame_start < 0:
            return None
        frame_end = frame_start + CONTROL_FRAME_SIZE
        if frame_end > len(buffer):
            return None
        candidate = bytes(buffer[frame_start:frame_end])
        try:
            parsed = parse_control_frame(candidate)
        except ValueError:
            search_from = frame_start + 1
            continue
        if (
            parsed.command == SYNC_ACK_COMMAND
            and parsed.sync_time_ms_u32 in sent_at_by_sync_time
        ):
            return frame_start, frame_end, parsed.sync_time_ms_u32
        search_from = frame_end


def _require_clock_ns(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{label} must be a non-negative integer")
    return value


def _raw_row(
    frame: GroupedFrame,
    *,
    sensor_rx_sync_time_ns: int,
    sensor_rx_ros_time_ns: int,
) -> dict[str, Any]:
    row: dict[str, Any] = {column: "" for column in RAW_CAPTURE_COLUMNS}
    row.update(
        {
            "sensor_rx_sync_time_ns": sensor_rx_sync_time_ns,
            "sensor_rx_ros_time_ns": sensor_rx_ros_time_ns,
            "parser_host_monotonic_ns": frame.host_monotonic_ns,
            "logical_seq": frame.logical_seq,
            "src_seq": frame.src_seq,
            "source_time_ms_u32": frame.source_time_ms_u32,
            "channel_ids": ";".join(str(record.channel_id) for record in frame.records),
            "parser_hub_packets": frame.stats.hub_packets,
            "parser_completed_groups": frame.stats.completed_groups,
            "parser_duplicate_or_reordered_groups": (
                frame.stats.duplicate_or_reordered_groups
            ),
            "parser_incomplete_groups": frame.stats.incomplete_groups,
            "parser_timeout_groups": frame.stats.timeout_groups,
            "parser_duplicate_packets": frame.stats.duplicate_packets,
            "parser_reordered_packets": frame.stats.reordered_packets,
            "parser_channel_sequence_drops": frame.stats.channel_sequence_drops,
            "parser_source_sequence_drops": frame.stats.source_sequence_drops,
            "parser_bytes_discarded": frame.stats.bytes_discarded,
            "parser_malformed_packets": frame.stats.malformed_packets,
            "parser_unexpected_channel_packets": (
                frame.stats.unexpected_channel_packets
            ),
            "parser_pending_evictions": frame.stats.pending_evictions,
        }
    )
    for record in frame.records:
        channel = record.channel_id
        row[f"channel_{channel}_seq"] = record.channel_seq
        row[f"channel_{channel}_nucleo_time_ms_u32"] = record.nucleo_time_ms_u32
        row[f"channel_{channel}_source_time_ms_u32"] = record.source_time_ms_u32
        for local_index, xyz in enumerate(record.xyz_ut):
            sensor_index = (channel - 1) * 16 + local_index
            for axis, value in zip(_AXES, xyz):
                row[f"sensor_{sensor_index:02d}_{axis}_ut"] = format(value, ".17g")
    return row


def finalize_nucleo_capture(
    summary: NucleoCaptureSummary,
    robot_telemetry_path: str | os.PathLike[str],
) -> NucleoFinalizeResult:
    """Map stopped sensor frames onto exact robot synchronization timestamps.

    Expected finalization failures are returned as ``status='failed'`` so that
    optional sensor evidence cannot replace or cancel the robot's outcome.
    A mid-run reader error maps all usable rows and returns ``incomplete``.
    """

    if not isinstance(summary, NucleoCaptureSummary):
        raise TypeError("summary must be a stopped NucleoCaptureSummary")
    robot_path = Path(robot_telemetry_path).expanduser().resolve()
    sensor_path = robot_path.with_name("nucleo_telemetry.csv")
    manifest_path = robot_path.with_name("nucleo_capture.yaml")
    staging_input_path = Path(summary.staging_path).expanduser()
    staging_input_is_symlink = staging_input_path.is_symlink()
    staging_path = staging_input_path.resolve()
    workspace = (
        Path(summary.workspace).expanduser().resolve()
        if summary.workspace is not None
        else _infer_workspace(staging_path)
    )

    mapped_count = 0
    outside_count = 0
    temporary_sensor_path: Path | None = None
    destination_allowed = False
    try:
        _require_robot_path_in_workspace_runs(robot_path, workspace)
        _require_staging_path_in_workspace_runs(
            staging_path,
            workspace,
            input_is_symlink=staging_input_is_symlink,
        )
        destination_allowed = True
        robot_fieldnames, robot_rows, robot_sync_times = _read_robot_rows(robot_path)
        staging_fieldnames, staging_rows = _read_staging_rows(staging_path)
        first_robot_sync_ns = robot_sync_times[0]
        last_robot_sync_ns = robot_sync_times[-1]
        output_fieldnames = _final_columns(robot_fieldnames, staging_fieldnames)

        output_rows: list[dict[str, str | int]] = []
        offsets: list[int] = []
        mapped_sensor_sync_times: list[int] = []
        for sensor_row_number, sensor_row in enumerate(staging_rows, start=2):
            sensor_sync_ns = _parse_nonnegative_int(
                sensor_row.get("sensor_rx_sync_time_ns", ""),
                f"staging row {sensor_row_number} sensor_rx_sync_time_ns",
            )
            if not first_robot_sync_ns <= sensor_sync_ns <= last_robot_sync_ns:
                outside_count += 1
                continue
            robot_index = _nearest_index(robot_sync_times, sensor_sync_ns)
            robot_row = robot_rows[robot_index]
            robot_sync_ns = robot_sync_times[robot_index]
            offset_ns = sensor_sync_ns - robot_sync_ns
            offsets.append(offset_ns)
            mapped_sensor_sync_times.append(sensor_sync_ns)
            output_rows.append(
                _mapped_row(
                    output_fieldnames=output_fieldnames,
                    robot_fieldnames=robot_fieldnames,
                    robot_row=robot_row,
                    robot_index=robot_index,
                    staging_fieldnames=staging_fieldnames,
                    sensor_row=sensor_row,
                    offset_ns=offset_ns,
                )
            )
        mapped_count = len(output_rows)

        temporary_sensor_path = _write_csv_atomic(
            sensor_path, output_fieldnames, output_rows
        )
        # _write_csv_atomic returns the installed target for a uniform cleanup path.
        assert temporary_sensor_path == sensor_path

        messages: list[str] = []
        if summary.capture_error:
            messages.append(summary.capture_error)
        if len(staging_rows) != summary.frames_written:
            messages.append(
                "staging row count differs from the capture summary "
                f"({len(staging_rows)} != {summary.frames_written})"
            )
        if mapped_count == 0:
            messages.append("no sensor frames fell within the robot telemetry window")
        coverage, coverage_messages = _sensor_coverage(
            mapped_sensor_sync_times,
            first_robot_sync_ns=first_robot_sync_ns,
            last_robot_sync_ns=last_robot_sync_ns,
            liveness_timeout_ns=int(summary.config.liveness_timeout_s * 1_000_000_000),
        )
        messages.extend(coverage_messages)
        max_mapping_offset_ns = int(summary.config.max_mapping_offset_s * 1_000_000_000)
        maximum_absolute_offset_ns = max((abs(value) for value in offsets), default=0)
        if maximum_absolute_offset_ns > max_mapping_offset_ns:
            messages.append(
                "maximum absolute robot/sensor mapping offset is "
                f"{maximum_absolute_offset_ns} ns, exceeding the "
                f"{max_mapping_offset_ns} ns limit"
            )
        parser_window_deltas, parser_messages = _parser_integrity_deltas(
            staging_rows,
            first_robot_sync_ns=first_robot_sync_ns,
            last_robot_sync_ns=last_robot_sync_ns,
        )
        messages.extend(parser_messages)
        status = "incomplete" if messages else "complete"
        message = "; ".join(messages) if messages else "Nucleo capture finalized"

        robot_sha256 = _sha256_file(robot_path)
        sensor_sha256 = _sha256_file(sensor_path)
        staging_sha256 = _sha256_file(staging_path)
        manifest = _build_manifest(
            status=status,
            message=message,
            summary=summary,
            workspace=workspace,
            robot_path=robot_path,
            sensor_path=sensor_path,
            staging_path=staging_path,
            robot_sha256=robot_sha256,
            sensor_sha256=sensor_sha256,
            staging_sha256=staging_sha256,
            raw_frame_count=len(staging_rows),
            mapped_count=mapped_count,
            outside_count=outside_count,
            offsets=offsets,
            robot_sample_count=len(robot_rows),
            first_robot_sync_ns=first_robot_sync_ns,
            last_robot_sync_ns=last_robot_sync_ns,
            coverage=coverage,
            parser_window_deltas=parser_window_deltas,
            staging_removed=False,
        )
        _write_json_yaml_atomic(manifest_path, manifest)

        if status == "complete":
            try:
                staging_path.unlink()
                _fsync_directory(staging_path.parent)
            except Exception as exc:
                status = "incomplete"
                message = (
                    "synchronized CSV was written, but staging evidence could not be "
                    f"removed: {type(exc).__name__}: {exc}"
                )
                manifest["status"] = status
                manifest["message"] = message
                manifest["artifacts"]["staging_raw_csv"]["removed"] = False
                _write_json_yaml_atomic(manifest_path, manifest)
            else:
                manifest["artifacts"]["staging_raw_csv"]["removed"] = True
                _write_json_yaml_atomic(manifest_path, manifest)

        return NucleoFinalizeResult(
            sensor_telemetry_path=sensor_path,
            manifest_path=manifest_path,
            status=status,
            message=message,
            mapped_frame_count=mapped_count,
            outside_window_frame_count=outside_count,
        )
    except Exception as exc:
        message = f"Nucleo finalization failed: {type(exc).__name__}: {exc}"
        if destination_allowed:
            try:
                failure_manifest = _build_failure_manifest(
                    message=message,
                    summary=summary,
                    workspace=workspace,
                    robot_path=robot_path,
                    sensor_path=sensor_path,
                    staging_path=staging_path,
                )
                manifest_path.parent.mkdir(parents=True, exist_ok=True)
                _write_json_yaml_atomic(manifest_path, failure_manifest)
            except Exception:
                pass
        return NucleoFinalizeResult(
            sensor_telemetry_path=sensor_path,
            manifest_path=manifest_path,
            status="failed",
            message=message,
            mapped_frame_count=mapped_count,
            outside_window_frame_count=outside_count,
        )


def _read_robot_rows(
    path: Path,
) -> tuple[list[str], list[dict[str, str]], list[int]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        fieldnames = list(reader.fieldnames or ())
        required = {"sync_time_ns", "trajectory_time_s"}
        missing = sorted(required - set(fieldnames))
        if missing:
            raise ValueError(
                "execution_telemetry.csv is missing required columns: "
                + ", ".join(missing)
            )
        if None in fieldnames or len(fieldnames) != len(set(fieldnames)):
            raise ValueError("execution_telemetry.csv has invalid or duplicate columns")
        rows = list(reader)
    if not rows:
        raise ValueError("execution_telemetry.csv contains no samples")

    sync_times: list[int] = []
    for row_number, row in enumerate(rows, start=2):
        if None in row:
            raise ValueError(
                f"execution_telemetry.csv row {row_number} has extra columns"
            )
        sync_ns = _parse_nonnegative_int(
            row.get("sync_time_ns", ""),
            f"execution_telemetry.csv row {row_number} sync_time_ns",
        )
        try:
            trajectory_time_s = float(row.get("trajectory_time_s", ""))
        except ValueError as exc:
            raise ValueError(
                f"execution_telemetry.csv row {row_number} trajectory_time_s is invalid"
            ) from exc
        if not math.isfinite(trajectory_time_s) or trajectory_time_s < 0.0:
            raise ValueError(
                f"execution_telemetry.csv row {row_number} trajectory_time_s "
                "must be finite and non-negative"
            )
        if "trajectory_time_ns" in fieldnames:
            _parse_nonnegative_int(
                row.get("trajectory_time_ns", ""),
                f"execution_telemetry.csv row {row_number} trajectory_time_ns",
            )
        if sync_times and sync_ns < sync_times[-1]:
            raise ValueError("execution_telemetry.csv sync_time_ns must be monotonic")
        sync_times.append(sync_ns)
    return fieldnames, rows, sync_times


def _read_staging_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        fieldnames = list(reader.fieldnames or ())
        if "sensor_rx_sync_time_ns" not in fieldnames:
            raise ValueError("staging sensor CSV is missing sensor_rx_sync_time_ns")
        if "sensor_rx_ros_time_ns" not in fieldnames:
            raise ValueError("staging sensor CSV is missing sensor_rx_ros_time_ns")
        if None in fieldnames or len(fieldnames) != len(set(fieldnames)):
            raise ValueError("staging sensor CSV has invalid or duplicate columns")
        rows = list(reader)
    previous_sync_ns: int | None = None
    for row_number, row in enumerate(rows, start=2):
        if None in row:
            raise ValueError(f"staging sensor CSV row {row_number} has extra columns")
        sync_ns = _parse_nonnegative_int(
            row.get("sensor_rx_sync_time_ns", ""),
            f"staging row {row_number} sensor_rx_sync_time_ns",
        )
        if previous_sync_ns is not None and sync_ns < previous_sync_ns:
            raise ValueError("staging sensor_rx_sync_time_ns must be monotonic")
        previous_sync_ns = sync_ns
        _parse_nonnegative_int(
            row.get("sensor_rx_ros_time_ns", ""),
            f"staging row {row_number} sensor_rx_ros_time_ns",
        )
    return fieldnames, rows


def _final_columns(
    robot_fieldnames: Sequence[str], staging_fieldnames: Sequence[str]
) -> list[str]:
    columns = ["sync_time_ns", "trajectory_time_s"]
    if "trajectory_time_ns" in robot_fieldnames:
        columns.append("trajectory_time_ns")
    if "ros_time_ns" in robot_fieldnames:
        columns.append("robot_ros_time_ns")
    columns.extend(
        [
            "sensor_rx_sync_time_ns",
            "sensor_rx_ros_time_ns",
            "sync_offset_ns",
            "robot_row_index",
        ]
    )
    columns.extend(
        column
        for column in staging_fieldnames
        if column not in {"sensor_rx_sync_time_ns", "sensor_rx_ros_time_ns"}
        and column not in columns
    )
    return columns


def _mapped_row(
    *,
    output_fieldnames: Sequence[str],
    robot_fieldnames: Sequence[str],
    robot_row: Mapping[str, str],
    robot_index: int,
    staging_fieldnames: Sequence[str],
    sensor_row: Mapping[str, str],
    offset_ns: int,
) -> dict[str, str | int]:
    row: dict[str, str | int] = {column: "" for column in output_fieldnames}
    # These values are copied as their original CSV strings, including the exact
    # primary integer synchronization timestamp selected from the robot row.
    row["sync_time_ns"] = robot_row["sync_time_ns"]
    row["trajectory_time_s"] = robot_row["trajectory_time_s"]
    if "trajectory_time_ns" in robot_fieldnames:
        row["trajectory_time_ns"] = robot_row["trajectory_time_ns"]
    if "ros_time_ns" in robot_fieldnames:
        row["robot_ros_time_ns"] = robot_row["ros_time_ns"]
    row["sensor_rx_sync_time_ns"] = sensor_row["sensor_rx_sync_time_ns"]
    row["sensor_rx_ros_time_ns"] = sensor_row["sensor_rx_ros_time_ns"]
    row["sync_offset_ns"] = offset_ns
    row["robot_row_index"] = robot_index
    for column in staging_fieldnames:
        if column not in {"sensor_rx_sync_time_ns", "sensor_rx_ros_time_ns"}:
            row[column] = sensor_row[column]
    return row


def _nearest_index(sorted_values: Sequence[int], target: int) -> int:
    insertion = bisect_left(sorted_values, target)
    if insertion == 0:
        return 0
    if insertion == len(sorted_values):
        return len(sorted_values) - 1
    before = sorted_values[insertion - 1]
    after = sorted_values[insertion]
    return insertion - 1 if target - before <= after - target else insertion


def _sensor_coverage(
    mapped_sensor_sync_times: Sequence[int],
    *,
    first_robot_sync_ns: int,
    last_robot_sync_ns: int,
    liveness_timeout_ns: int,
) -> tuple[dict[str, int | None], list[str]]:
    coverage: dict[str, int | None] = {
        "liveness_timeout_ns": liveness_timeout_ns,
        "first_sensor_rx_sync_time_ns": None,
        "last_sensor_rx_sync_time_ns": None,
        "leading_gap_ns": None,
        "trailing_gap_ns": None,
        "maximum_inter_frame_gap_ns": None,
    }
    if not mapped_sensor_sync_times:
        return coverage, []

    first_sensor_sync_ns = mapped_sensor_sync_times[0]
    last_sensor_sync_ns = mapped_sensor_sync_times[-1]
    gaps = [
        current - previous
        for previous, current in zip(
            mapped_sensor_sync_times,
            mapped_sensor_sync_times[1:],
        )
    ]
    leading_gap_ns = first_sensor_sync_ns - first_robot_sync_ns
    trailing_gap_ns = last_robot_sync_ns - last_sensor_sync_ns
    maximum_inter_frame_gap_ns = max(gaps, default=0)
    coverage.update(
        {
            "first_sensor_rx_sync_time_ns": first_sensor_sync_ns,
            "last_sensor_rx_sync_time_ns": last_sensor_sync_ns,
            "leading_gap_ns": leading_gap_ns,
            "trailing_gap_ns": trailing_gap_ns,
            "maximum_inter_frame_gap_ns": maximum_inter_frame_gap_ns,
        }
    )

    messages: list[str] = []
    for label, gap_ns in (
        ("leading sensor coverage gap", leading_gap_ns),
        ("trailing sensor coverage gap", trailing_gap_ns),
        ("maximum sensor inter-frame gap", maximum_inter_frame_gap_ns),
    ):
        if gap_ns > liveness_timeout_ns:
            messages.append(
                f"{label} is {gap_ns} ns, exceeding the "
                f"{liveness_timeout_ns} ns liveness limit"
            )
    return coverage, messages


def _parser_integrity_deltas(
    staging_rows: Sequence[Mapping[str, str]],
    *,
    first_robot_sync_ns: int,
    last_robot_sync_ns: int,
) -> tuple[dict[str, int], list[str]]:
    baseline: dict[str, int] = {}
    last_in_window: dict[str, int] | None = None
    previous: dict[str, int] = {}

    for row_number, row in enumerate(staging_rows, start=2):
        sensor_sync_ns = _parse_nonnegative_int(
            row.get("sensor_rx_sync_time_ns", ""),
            f"staging row {row_number} sensor_rx_sync_time_ns",
        )
        counters: dict[str, int] = {}
        for column in _PARSER_INTEGRITY_COUNTERS:
            raw_value = row.get(column)
            if raw_value is None or not str(raw_value).strip():
                continue
            value = _parse_nonnegative_int(
                raw_value,
                f"staging row {row_number} {column}",
            )
            if column in previous and value < previous[column]:
                raise ValueError(f"staging parser counter {column} must be monotonic")
            previous[column] = value
            counters[column] = value
        if sensor_sync_ns < first_robot_sync_ns:
            baseline.update(counters)
        elif sensor_sync_ns <= last_robot_sync_ns:
            if last_in_window is None:
                last_in_window = dict(baseline)
            last_in_window.update(counters)

    if last_in_window is None:
        return {}, []

    deltas: dict[str, int] = {}
    messages: list[str] = []
    for column, label in _PARSER_INTEGRITY_COUNTERS.items():
        if column not in last_in_window:
            continue
        delta = last_in_window[column] - baseline.get(column, 0)
        if delta < 0:
            raise ValueError(f"staging parser counter {column} decreased in playback")
        deltas[column] = delta
        if delta:
            messages.append(f"parser observed {delta} {label} during playback")
    return deltas, messages


def _parse_nonnegative_int(value: Any, label: str) -> int:
    text = str(value).strip()
    if not text:
        raise ValueError(f"{label} is empty")
    try:
        parsed = int(text, 10)
    except ValueError as exc:
        raise ValueError(f"{label} is not an integer") from exc
    if parsed < 0:
        raise ValueError(f"{label} must be non-negative")
    return parsed


def _write_csv_atomic(
    path: Path,
    fieldnames: Sequence[str],
    rows: Sequence[Mapping[str, Any]],
) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary_path = Path(stream.name)
            writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
        _fsync_directory(path.parent)
        return path
    except Exception:
        if temporary_path is not None:
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass
        raise


def _write_json_yaml_atomic(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="\n",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary_path = Path(stream.name)
            # JSON is a strict subset of YAML 1.2 and avoids a mandatory YAML
            # dependency in this optional sidecar package.
            json.dump(payload, stream, indent=2, ensure_ascii=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
        _fsync_directory(path.parent)
    except Exception:
        if temporary_path is not None:
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass
        raise


def _build_manifest(
    *,
    status: str,
    message: str,
    summary: NucleoCaptureSummary,
    workspace: Path,
    robot_path: Path,
    sensor_path: Path,
    staging_path: Path,
    robot_sha256: str,
    sensor_sha256: str,
    staging_sha256: str,
    raw_frame_count: int,
    mapped_count: int,
    outside_count: int,
    offsets: Sequence[int],
    robot_sample_count: int,
    first_robot_sync_ns: int,
    last_robot_sync_ns: int,
    coverage: Mapping[str, int | None],
    parser_window_deltas: Mapping[str, int],
    staging_removed: bool,
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "status": status,
        "message": message,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "license": "Apache-2.0",
        "parser_provenance": _load_parser_provenance(workspace),
        "capture": {
            "port": summary.config.port,
            "baudrate": summary.config.baudrate,
            "channels": list(summary.config.channels),
            "serial_format": "8N1",
            "flow_control": False,
            "settle_time_s": summary.config.settle_time_s,
            "serial_timeout_s": summary.config.serial_timeout_s,
            "preflight_timeout_s": summary.config.preflight_timeout_s,
            "liveness_timeout_s": summary.config.liveness_timeout_s,
            "max_mapping_offset_s": summary.config.max_mapping_offset_s,
            "incomplete_group_timeout_s": (summary.config.incomplete_group_timeout_s),
            "pending_group_limit": summary.config.pending_group_limit,
            "started_sync_time_ns": summary.started_sync_time_ns,
            "stopped_sync_time_ns": summary.stopped_sync_time_ns,
            "capture_error": summary.capture_error,
            "sync": {
                "mode": (
                    "performed" if summary.config.perform_sync else "reused_existing"
                ),
                "command": "0x01",
                "ack_command": "0x81",
                "echoed_time_ms_u32": summary.sync_time_ms_u32,
                "round_trip_ns": summary.sync_rtt_ns,
                "attempts": summary.sync_attempts,
                "timeout_s": summary.config.sync_timeout_s,
                "retry_interval_s": summary.config.sync_retry_interval_s,
            },
        },
        "counts": {
            "frames_reported_by_capture": summary.frames_written,
            "raw_sensor_frames": raw_frame_count,
            "mapped_sensor_frames": mapped_count,
            "outside_robot_window_frames": outside_count,
            "robot_samples": robot_sample_count,
            "parser": asdict(summary.parser_stats),
        },
        "mapping": {
            "method": "nearest_robot_sync_time_ns",
            "tie_break": "earlier_robot_sample",
            "offset_definition": (
                "sync_offset_ns = sensor_rx_sync_time_ns - "
                "selected_robot_sync_time_ns"
            ),
            "robot_first_sync_time_ns": first_robot_sync_ns,
            "robot_last_sync_time_ns": last_robot_sync_ns,
            "offset_ns": _offset_statistics(offsets),
            "maximum_allowed_absolute_offset_ns": int(
                summary.config.max_mapping_offset_s * 1_000_000_000
            ),
            "sensor_coverage": dict(coverage),
            "parser_integrity_deltas": dict(parser_window_deltas),
        },
        "artifacts": {
            "robot_telemetry_csv": {
                "path": str(robot_path),
                "sha256": robot_sha256,
            },
            "sensor_telemetry_csv": {
                "path": str(sensor_path),
                "sha256": sensor_sha256,
            },
            "staging_raw_csv": {
                "path": str(staging_path),
                "sha256": staging_sha256,
                "removed": staging_removed,
            },
        },
    }


def _build_failure_manifest(
    *,
    message: str,
    summary: NucleoCaptureSummary,
    workspace: Path,
    robot_path: Path,
    sensor_path: Path,
    staging_path: Path,
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "status": "failed",
        "message": message,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "license": "Apache-2.0",
        "parser_provenance": _load_parser_provenance(workspace),
        "capture": {
            "port": summary.config.port,
            "baudrate": summary.config.baudrate,
            "channels": list(summary.config.channels),
            "liveness_timeout_s": summary.config.liveness_timeout_s,
            "max_mapping_offset_s": summary.config.max_mapping_offset_s,
            "capture_error": summary.capture_error,
            "sync_round_trip_ns": summary.sync_rtt_ns,
        },
        "counts": {
            "frames_reported_by_capture": summary.frames_written,
            "parser": asdict(summary.parser_stats),
        },
        "artifacts": {
            "robot_telemetry_csv": {
                "path": str(robot_path),
                "sha256": _sha256_if_regular_file(robot_path),
            },
            "sensor_telemetry_csv": {
                "path": str(sensor_path),
                "sha256": _sha256_if_regular_file(sensor_path),
            },
            "staging_raw_csv": {
                "path": str(staging_path),
                "sha256": _sha256_if_regular_file(staging_path),
                "removed": False,
            },
        },
    }


def _offset_statistics(offsets: Sequence[int]) -> dict[str, int | float | None]:
    if not offsets:
        return {
            "minimum": None,
            "maximum": None,
            "mean": None,
            "median": None,
            "mean_absolute": None,
            "maximum_absolute": None,
        }
    absolute = [abs(value) for value in offsets]
    return {
        "minimum": min(offsets),
        "maximum": max(offsets),
        "mean": statistics.fmean(offsets),
        "median": statistics.median(offsets),
        "mean_absolute": statistics.fmean(absolute),
        "maximum_absolute": max(absolute),
    }


def _load_parser_provenance(workspace: Path) -> dict[str, Any]:
    provenance = dict(PARSER_PROVENANCE_DEFAULTS)
    lock_path = workspace / "nucleo.lock"
    try:
        for raw_line in lock_path.read_text(encoding="utf-8").splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            key, raw_value = line.split(":", 1)
            key = key.strip()
            value: Any = raw_value.strip()
            if key == "schema_version":
                value = int(value)
            provenance[key] = value
        provenance["lock_path"] = str(lock_path)
        provenance["lock_sha256"] = _sha256_file(lock_path)
    except (OSError, ValueError):
        provenance["lock_path"] = str(lock_path)
        provenance["lock_sha256"] = None
    provenance["implementation"] = "independent_wire_protocol_reimplementation"
    provenance["implementation_module_sha256"] = _sha256_file(
        Path(__file__).with_name("protocol.py")
    )
    return provenance


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def _sha256_if_regular_file(path: Path) -> str | None:
    try:
        if not path.is_file() or path.is_symlink():
            return None
        return _sha256_file(path)
    except OSError:
        return None


def _fsync_directory(directory: Path) -> None:
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    descriptor = os.open(directory, flags)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _infer_workspace(staging_path: Path) -> Path:
    if staging_path.parent.name == "runs":
        return staging_path.parent.parent
    return staging_path.parent


def _require_robot_path_in_workspace_runs(robot_path: Path, workspace: Path) -> None:
    runs_root = (workspace / "runs").resolve()
    try:
        robot_path.relative_to(runs_root)
    except ValueError as exc:
        raise ValueError(
            "robot telemetry path must be located under workspace/runs"
        ) from exc
    if robot_path.name != "execution_telemetry.csv":
        raise ValueError("robot telemetry path must name execution_telemetry.csv")


def _require_staging_path_in_workspace_runs(
    staging_path: Path,
    workspace: Path,
    *,
    input_is_symlink: bool,
) -> None:
    runs_root = (workspace / "runs").resolve()
    try:
        staging_path.relative_to(runs_root)
    except ValueError as exc:
        raise ValueError(
            "NUCLEO staging path must be located under workspace/runs"
        ) from exc
    if input_is_symlink:
        raise ValueError("NUCLEO staging path must not be a symlink")
    if not (
        staging_path.name.startswith("nucleo_capture__")
        and staging_path.name.endswith(".staging.csv")
    ):
        raise ValueError(
            "NUCLEO staging filename must match nucleo_capture__*.staging.csv"
        )
    if not staging_path.is_file():
        raise ValueError("NUCLEO staging path must be an existing regular file")


def _best_effort_close(resource: Any) -> None:
    try:
        resource.close()
    except Exception:
        pass


__all__ = [
    "CaptureSummary",
    "FinalizeResult",
    "NucleoCaptureConfig",
    "NucleoCaptureError",
    "NucleoCaptureSession",
    "NucleoCaptureSummary",
    "NucleoFinalizeResult",
    "PARSER_PROVENANCE_COMMIT",
    "RAW_CAPTURE_COLUMNS",
    "finalize_nucleo_capture",
    "raw_capture_columns",
]
