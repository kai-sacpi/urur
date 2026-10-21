from __future__ import annotations

import csv
import hashlib
import io
import json
from pathlib import Path
import struct
import threading
import time

import pytest

from hx_waypoint_nucleo import (
    HubStreamParser,
    NucleoCaptureConfig,
    NucleoCaptureError,
    NucleoCaptureSession,
    NucleoCaptureSummary,
    ParserStats,
    RAW_CAPTURE_COLUMNS,
    build_control_frame,
    build_sync_frame,
    control_checksum,
    finalize_nucleo_capture,
    median_u32,
    parse_control_frame,
    parse_l5_payload,
)
from hx_waypoint_nucleo.protocol import SYNC_ACK_COMMAND


def _l5_payload(
    *,
    src_seq: int,
    source_time_ms_u32: int,
    axes: list[int] | None = None,
) -> bytes:
    if axes is None:
        axes = list(range(48))
    assert len(axes) == 48
    return (
        b"L5"
        + bytes((src_seq,))
        + struct.pack(">48h", *axes)
        + struct.pack(">I", source_time_ms_u32)
    )


def _hub_packet(
    *,
    channel: int,
    channel_seq: int,
    nucleo_time_ms_u32: int,
    src_seq: int,
    source_time_ms_u32: int,
    axes: list[int] | None = None,
) -> bytes:
    packet = (
        b"\x55\xaa"
        + bytes((channel,))
        + struct.pack("<I", channel_seq)
        + struct.pack("<I", nucleo_time_ms_u32)
        + _l5_payload(
            src_seq=src_seq,
            source_time_ms_u32=source_time_ms_u32,
            axes=axes,
        )
    )
    assert len(packet) == 114
    return packet


def test_control_frame_checksum_and_sync_ack_validation() -> None:
    sync = build_sync_frame(0x12345678)
    assert len(sync) == 8
    assert sync[:3] == b"\xa5\x5a\x01"
    assert sync[3:7] == b"\x78\x56\x34\x12"
    assert sync[-1] == control_checksum(sync[:-1])

    ack = build_control_frame(SYNC_ACK_COMMAND, 0x12345678)
    parsed = parse_control_frame(ack)
    assert parsed.command == SYNC_ACK_COMMAND
    assert parsed.sync_time_ms_u32 == 0x12345678

    corrupted = bytearray(ack)
    corrupted[-1] ^= 0x01
    with pytest.raises(ValueError, match="checksum"):
        parse_control_frame(corrupted)


def test_stream_parser_resynchronizes_noise_and_split_packets() -> None:
    parser = HubStreamParser(expected_channels=(1, 2))
    channel_1 = _hub_packet(
        channel=1,
        channel_seq=10,
        nucleo_time_ms_u32=100,
        src_seq=7,
        source_time_ms_u32=200,
    )
    channel_2 = _hub_packet(
        channel=2,
        channel_seq=20,
        nucleo_time_ms_u32=101,
        src_seq=7,
        source_time_ms_u32=202,
    )

    assert parser.feed(b"noise" + channel_1[:37], 1_000_000_000) == ()
    frames = parser.feed(channel_1[37:] + channel_2[:51], 1_001_000_000)
    assert frames == ()
    frames = parser.feed(channel_2[51:], 1_002_000_000)

    assert len(frames) == 1
    frame = frames[0]
    assert frame.logical_seq == 0
    assert frame.src_seq == 7
    assert frame.source_time_ms_u32 == 201
    assert [record.channel_id for record in frame.records] == [1, 2]
    assert frame.channel_sequences == {1: 10, 2: 20}
    assert frame.channel_nucleo_timestamps_ms == {1: 100, 2: 101}
    assert frame.stats.bytes_discarded == len(b"noise")


def test_l5_big_endian_signed_extremes_are_scaled_to_microtesla() -> None:
    axes = [-32768, 32767, -1, 0, 1, 2] + [0] * 42
    payload = parse_l5_payload(
        _l5_payload(src_seq=15, source_time_ms_u32=0xFEDCBA98, axes=axes)
    )

    assert payload.src_seq == 15
    assert payload.source_time_ms_u32 == 0xFEDCBA98
    assert payload.raw_xyz[0] == (-32768, 32767, -1)
    assert payload.raw_xyz[1] == (0, 1, 2)
    assert payload.xyz_ut[0] == pytest.approx((-4915.2, 4915.05, -0.15))
    assert payload.xyz_ut[1] == pytest.approx((0.0, 0.15, 0.3))


@pytest.mark.parametrize("channels", [(1, 2), (1, 2, 3, 4)])
def test_two_and_four_channel_grouping_maps_global_sensor_indices(
    channels: tuple[int, ...],
) -> None:
    parser = HubStreamParser(expected_channels=channels)
    packets = []
    for channel in channels:
        axes = [channel] * 48
        packets.append(
            _hub_packet(
                channel=channel,
                channel_seq=channel * 100,
                nucleo_time_ms_u32=500 + channel,
                src_seq=3,
                source_time_ms_u32=700 + channel,
                axes=axes,
            )
        )

    frames = parser.feed(b"".join(packets), 10_000)

    assert len(frames) == 1
    frame = frames[0]
    assert len(frame.records) == len(channels)
    for channel in channels:
        first_sensor = (channel - 1) * 16
        assert frame.sensor_values_ut[first_sensor] == pytest.approx(
            (channel * 0.15,) * 3
        )
    assert len(frame.sensor_values_ut) == len(channels) * 16


def test_duplicate_reorder_drop_stats_and_u32_wrap() -> None:
    parser = HubStreamParser(expected_channels=(1, 2))
    first = parser.feed(
        _hub_packet(
            channel=1,
            channel_seq=0xFFFFFFFF,
            nucleo_time_ms_u32=0xFFFFFFFE,
            src_seq=15,
            source_time_ms_u32=0xFFFFFFFE,
        )
        + _hub_packet(
            channel=2,
            channel_seq=0xFFFFFFFF,
            nucleo_time_ms_u32=1,
            src_seq=15,
            source_time_ms_u32=1,
        ),
        1_000,
    )
    assert len(first) == 1
    assert first[0].logical_seq == 0
    assert first[0].source_time_ms_u32 == 0

    second = parser.feed(
        _hub_packet(
            channel=1,
            channel_seq=1,
            nucleo_time_ms_u32=2,
            src_seq=0,
            source_time_ms_u32=0,
        )
        + _hub_packet(
            channel=2,
            channel_seq=0,
            nucleo_time_ms_u32=3,
            src_seq=0,
            source_time_ms_u32=2,
        ),
        2_000,
    )
    assert len(second) == 1
    assert second[0].logical_seq == 1
    assert second[0].source_time_ms_u32 == 1
    assert second[0].stats.channel_sequence_drops == 1

    duplicate = _hub_packet(
        channel=1,
        channel_seq=1,
        nucleo_time_ms_u32=4,
        src_seq=0,
        source_time_ms_u32=3,
    )
    reordered = _hub_packet(
        channel=1,
        channel_seq=0,
        nucleo_time_ms_u32=5,
        src_seq=15,
        source_time_ms_u32=4,
    )
    assert parser.feed(duplicate + reordered, 3_000) == ()
    assert parser.stats.duplicate_packets == 1
    assert parser.stats.reordered_packets == 1
    assert parser.stats.channel_sequence_drops == 1


def test_wrap_aware_median_and_incomplete_timeout() -> None:
    assert median_u32((0xFFFFFFFE, 0xFFFFFFFF, 0, 1)) == 0
    assert median_u32((101, 102)) == 101
    parser = HubStreamParser(expected_channels=(1, 2), incomplete_group_timeout_s=0.04)
    parser.feed(
        _hub_packet(
            channel=1,
            channel_seq=1,
            nucleo_time_ms_u32=1,
            src_seq=1,
            source_time_ms_u32=1,
        ),
        100,
    )
    parser.poll(40_000_100)
    assert parser.pending_group_count == 0
    assert parser.stats.incomplete_groups == 1
    assert parser.stats.timeout_groups == 1


def test_group_source_timestamp_must_increase_like_hospital_parser() -> None:
    parser = HubStreamParser(expected_channels=(1, 2))

    def group(channel_sequence: int, src_seq: int, source_time_ms: int) -> bytes:
        return b"".join(
            _hub_packet(
                channel=channel,
                channel_seq=channel_sequence,
                nucleo_time_ms_u32=source_time_ms,
                src_seq=src_seq,
                source_time_ms_u32=source_time_ms,
            )
            for channel in (1, 2)
        )

    assert len(parser.feed(group(1, 1, 100), 1_000)) == 1
    assert parser.feed(group(2, 2, 99), 2_000) == ()
    resumed = parser.feed(group(3, 3, 101), 3_000)

    assert len(resumed) == 1
    assert resumed[0].logical_seq == 1
    assert parser.stats.duplicate_or_reordered_groups == 1


@pytest.mark.parametrize(
    "kwargs",
    [
        {"port": ""},
        {"port": "/dev/ttyACM0", "baudrate": 0},
        {"port": "/dev/ttyACM0", "channels": (1, 1)},
        {"port": "/dev/ttyACM0", "channels": (2, 1)},
        {"port": "/dev/ttyACM0", "channels": (1, 5)},
        {"port": "/dev/ttyACM0", "preflight_timeout_s": 0.0},
        {"port": "/dev/ttyACM0", "liveness_timeout_s": 0.0},
        {"port": "/dev/ttyACM0", "max_mapping_offset_s": 0.0},
        {"port": "/dev/ttyACM0", "pending_group_limit": 9},
        {"port": "/dev/ttyACM0", "read_size": 7},
        {"port": "/dev/ttyACM0", "perform_sync": 1},
    ],
)
def test_capture_config_validation(kwargs: dict[str, object]) -> None:
    with pytest.raises(ValueError):
        NucleoCaptureConfig(**kwargs)


def test_capture_config_hospital_aligned_defaults() -> None:
    config = NucleoCaptureConfig(port="/dev/ttyACM0")
    assert config.baudrate == 115200
    assert config.perform_sync is True
    assert config.settle_time_s == 0.2
    assert config.serial_timeout_s == 0.02
    assert config.sync_timeout_s == 1.0
    assert config.sync_retry_interval_s == 0.05
    assert config.preflight_timeout_s == 5.0
    assert config.liveness_timeout_s == 0.5
    assert config.max_mapping_offset_s == 0.05
    assert config.incomplete_group_timeout_s == 0.04
    assert config.pending_group_limit == 8


def test_capture_reuses_sync_with_one_host_input_clear_and_no_control_traffic(
    tmp_path: Path,
) -> None:
    payload = b"".join(
        _hub_packet(
            channel=channel,
            channel_seq=1,
            nucleo_time_ms_u32=10,
            src_seq=1,
            source_time_ms_u32=10,
        )
        for channel in (1, 2)
    )

    class StreamingSerial:
        def __init__(self) -> None:
            self.payload = b"stale host input"
            self.input_reset_calls = 0
            self.output_reset_calls = 0
            self.writes: list[bytes] = []
            self.closed = False

        def reset_input_buffer(self) -> None:
            self.input_reset_calls += 1
            # Fresh stream bytes arrive after the stale host queue is cleared.
            self.payload = payload

        def reset_output_buffer(self) -> None:
            self.output_reset_calls += 1

        def write(self, data: bytes) -> int:
            self.writes.append(bytes(data))
            raise AssertionError("reuse mode must not send a second handshake")

        def read(self, size: int) -> bytes:
            assert size > 0
            assert self.input_reset_calls == 1
            if self.payload:
                result = self.payload
                self.payload = b""
                return result
            time.sleep(0.001)
            return b""

        def cancel_read(self) -> None:
            pass

        def close(self) -> None:
            self.closed = True

    serial_port = StreamingSerial()
    session = NucleoCaptureSession(
        tmp_path,
        NucleoCaptureConfig(
            port="/dev/ttyACM0",
            channels=(1, 2),
            perform_sync=False,
            settle_time_s=0.001,
            preflight_timeout_s=1.0,
        ),
        lambda: 123,
    )
    session._open_serial = lambda: serial_port

    session.start()
    summary = session.stop()

    assert summary.frames_written == 1
    assert summary.sync_attempts == 0
    assert summary.sync_time_ms_u32 is None
    assert summary.sync_rtt_ns is None
    assert serial_port.input_reset_calls == 1
    assert serial_port.output_reset_calls == 0
    assert serial_port.writes == []
    assert serial_port.closed is True


def test_capture_rows_use_serial_read_timestamp_not_writer_latency(
    tmp_path: Path,
) -> None:
    class FlushCountingStream(io.StringIO):
        def __init__(self) -> None:
            super().__init__()
            self.flush_calls = 0

        def flush(self) -> None:
            self.flush_calls += 1
            super().flush()

    parser = HubStreamParser(expected_channels=(1, 2))
    payload = b"".join(
        _hub_packet(
            channel=channel,
            channel_seq=channel,
            nucleo_time_ms_u32=10,
            src_seq=1,
            source_time_ms_u32=10,
        )
        for channel in (1, 2)
    )
    frames = parser.feed(payload, 123_456_789)
    assert len(frames) == 1

    session = NucleoCaptureSession(
        tmp_path,
        NucleoCaptureConfig(port="/dev/ttyACM0", channels=(1, 2)),
        lambda: 987_654_321,
    )
    stream = FlushCountingStream()
    session._stream = stream
    session._writer = csv.DictWriter(stream, fieldnames=RAW_CAPTURE_COLUMNS)
    session._writer.writeheader()
    session._write_frames(frames)

    assert stream.flush_calls == 0

    stream.seek(0)
    row = next(csv.DictReader(stream))
    assert row["sensor_rx_sync_time_ns"] == "123456789"
    assert row["parser_host_monotonic_ns"] == "123456789"


def test_reader_does_not_touch_ros_clock_after_stop_during_blocking_read(
    tmp_path: Path,
) -> None:
    entered_read = threading.Event()
    release_read = threading.Event()
    payload = b"".join(
        _hub_packet(
            channel=channel,
            channel_seq=channel,
            nucleo_time_ms_u32=10,
            src_seq=1,
            source_time_ms_u32=10,
        )
        for channel in (1, 2)
    )

    class BlockingSerial:
        def read(self, size: int) -> bytes:
            assert size > 0
            entered_read.set()
            assert release_read.wait(timeout=2.0)
            return payload

    ros_clock_calls = 0

    def ros_time_ns() -> int:
        nonlocal ros_clock_calls
        ros_clock_calls += 1
        return 123

    session = NucleoCaptureSession(
        tmp_path,
        NucleoCaptureConfig(port="/dev/ttyACM0", channels=(1, 2)),
        ros_time_ns,
    )
    stream = io.StringIO()
    session._stream = stream
    session._writer = csv.DictWriter(stream, fieldnames=RAW_CAPTURE_COLUMNS)
    session._writer.writeheader()
    session._serial = BlockingSerial()
    thread = threading.Thread(target=session._reader_loop)
    session._thread = thread
    thread.start()
    assert entered_read.wait(timeout=2.0)

    session._stop_event.set()
    release_read.set()
    thread.join(timeout=2.0)

    assert not thread.is_alive()
    assert ros_clock_calls == 0
    assert session.frames_written == 0


def test_final_mapping_copies_exact_robot_sync_time_ns(tmp_path: Path) -> None:
    workspace = tmp_path / "workspace"
    run_directory = workspace / "runs" / "demo__20260803_120000" / "real"
    run_directory.mkdir(parents=True)
    robot_path = run_directory / "execution_telemetry.csv"
    with robot_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "sync_time_ns",
                "ros_time_ns",
                "trajectory_time_s",
                "trajectory_time_ns",
            ],
            lineterminator="\n",
        )
        writer.writeheader()
        writer.writerow(
            {
                "sync_time_ns": "1000",
                "ros_time_ns": "9001000",
                "trajectory_time_s": "0.000000000",
                "trajectory_time_ns": "0",
            }
        )
        writer.writerow(
            {
                "sync_time_ns": "1100",
                "ros_time_ns": "9001100",
                "trajectory_time_s": "0.100000000",
                "trajectory_time_ns": "100000000",
            }
        )
        writer.writerow(
            {
                "sync_time_ns": "1200",
                "ros_time_ns": "9001200",
                "trajectory_time_s": "0.200000000",
                "trajectory_time_ns": "200000000",
            }
        )

    staging_path = workspace / "runs" / "nucleo_capture__mapping.staging.csv"
    with staging_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "sensor_rx_sync_time_ns",
                "sensor_rx_ros_time_ns",
                "logical_seq",
                "src_seq",
                "source_time_ms_u32",
                "channel_1_seq",
                "sensor_00_x_ut",
                "sensor_00_y_ut",
                "sensor_00_z_ut",
            ],
            lineterminator="\n",
        )
        writer.writeheader()
        for values in (
            ("1010", "8001010", "0", "5", "100", "50", "1", "2", "3"),
            ("1150", "8001150", "1", "6", "101", "51", "4", "5", "6"),
            ("1201", "8001201", "2", "7", "102", "52", "7", "8", "9"),
        ):
            writer.writerow(dict(zip(writer.fieldnames, values)))

    summary = NucleoCaptureSummary(
        staging_path=staging_path,
        config=NucleoCaptureConfig(port="/dev/ttyACM0", channels=(1, 2)),
        workspace=workspace,
        sync_time_ms_u32=123,
        sync_rtt_ns=456,
        sync_attempts=1,
        frames_written=3,
        parser_stats=ParserStats(completed_groups=3),
    )

    result = finalize_nucleo_capture(summary, robot_path)

    assert result.status == "complete"
    assert result.mapped_frame_count == 2
    assert result.outside_window_frame_count == 1
    assert not staging_path.exists()
    with result.sensor_telemetry_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert [row["sync_time_ns"] for row in rows] == ["1000", "1100"]
    assert [row["trajectory_time_ns"] for row in rows] == ["0", "100000000"]
    assert [row["sensor_rx_sync_time_ns"] for row in rows] == ["1010", "1150"]
    assert [row["sync_offset_ns"] for row in rows] == ["10", "50"]
    assert rows[1]["logical_seq"] == "1"
    assert rows[1]["sensor_00_z_ut"] == "6"

    manifest = json.loads(result.manifest_path.read_text(encoding="utf-8"))
    assert manifest["status"] == "complete"
    assert manifest["capture"]["sync"]["round_trip_ns"] == 456
    assert manifest["mapping"]["offset_ns"]["maximum"] == 50
    assert manifest["artifacts"]["robot_telemetry_csv"]["sha256"] == (
        hashlib.sha256(robot_path.read_bytes()).hexdigest()
    )
    assert manifest["artifacts"]["sensor_telemetry_csv"]["sha256"] == (
        hashlib.sha256(result.sensor_telemetry_path.read_bytes()).hexdigest()
    )
    assert manifest["artifacts"]["staging_raw_csv"]["removed"] is True


def test_midrun_capture_error_finalizes_incomplete_and_keeps_staging(
    tmp_path: Path,
) -> None:
    workspace = tmp_path / "workspace"
    run_directory = workspace / "runs" / "demo" / "simulation"
    run_directory.mkdir(parents=True)
    robot_path = run_directory / "execution_telemetry.csv"
    robot_path.write_text(
        "sync_time_ns,trajectory_time_s\n100,0.0\n200,0.1\n",
        encoding="utf-8",
    )
    staging_path = workspace / "runs" / "nucleo_capture__reader_error.staging.csv"
    staging_path.write_text(
        "sensor_rx_sync_time_ns,sensor_rx_ros_time_ns,logical_seq\n" "150,1000,0\n",
        encoding="utf-8",
    )
    summary = NucleoCaptureSummary(
        staging_path=staging_path,
        config=NucleoCaptureConfig(port="/dev/ttyACM0"),
        workspace=workspace,
        frames_written=1,
        capture_error="serial read failed mid-run",
    )

    result = finalize_nucleo_capture(summary, robot_path)

    assert result.status == "incomplete"
    assert staging_path.exists()
    assert result.sensor_telemetry_path.exists()
    manifest = json.loads(result.manifest_path.read_text(encoding="utf-8"))
    assert manifest["status"] == "incomplete"
    assert manifest["artifacts"]["staging_raw_csv"]["removed"] is False


def test_silent_sensor_gap_marks_capture_incomplete(tmp_path: Path) -> None:
    workspace = tmp_path / "workspace"
    run_directory = workspace / "runs" / "demo" / "simulation"
    run_directory.mkdir(parents=True)
    robot_path = run_directory / "execution_telemetry.csv"
    robot_path.write_text(
        "sync_time_ns,trajectory_time_s\n"
        "1000000000,0.0\n"
        "1500000000,0.5\n"
        "2000000000,1.0\n",
        encoding="utf-8",
    )
    staging_path = workspace / "runs" / "nucleo_capture__silent_gap.staging.csv"
    staging_path.write_text(
        "sensor_rx_sync_time_ns,sensor_rx_ros_time_ns,logical_seq\n"
        "1010000000,5000000000,0\n"
        "1200000000,5190000000,1\n",
        encoding="utf-8",
    )
    summary = NucleoCaptureSummary(
        staging_path=staging_path,
        config=NucleoCaptureConfig(
            port="/dev/ttyACM0",
            liveness_timeout_s=0.5,
        ),
        workspace=workspace,
        frames_written=2,
    )

    result = finalize_nucleo_capture(summary, robot_path)

    assert result.status == "incomplete"
    assert "trailing sensor coverage gap" in result.message
    assert staging_path.exists()
    manifest = json.loads(result.manifest_path.read_text(encoding="utf-8"))
    assert manifest["mapping"]["sensor_coverage"]["trailing_gap_ns"] == 800000000


def test_parser_integrity_loss_inside_playback_marks_incomplete(
    tmp_path: Path,
) -> None:
    workspace = tmp_path / "workspace"
    run_directory = workspace / "runs" / "demo" / "real"
    run_directory.mkdir(parents=True)
    robot_path = run_directory / "execution_telemetry.csv"
    robot_path.write_text(
        "sync_time_ns,trajectory_time_s\n100,0.0\n300,0.2\n",
        encoding="utf-8",
    )
    staging_path = workspace / "runs" / "nucleo_capture__parser_loss.staging.csv"
    staging_path.write_text(
        "sensor_rx_sync_time_ns,sensor_rx_ros_time_ns,"
        "parser_duplicate_or_reordered_groups\n"
        "50,950,0\n"
        "150,1050,0\n"
        "250,1150,1\n",
        encoding="utf-8",
    )
    summary = NucleoCaptureSummary(
        staging_path=staging_path,
        config=NucleoCaptureConfig(port="/dev/ttyACM0"),
        workspace=workspace,
        frames_written=3,
    )

    result = finalize_nucleo_capture(summary, robot_path)

    assert result.status == "incomplete"
    assert "duplicate/reordered groups" in result.message
    manifest = json.loads(result.manifest_path.read_text(encoding="utf-8"))
    assert (
        manifest["mapping"]["parser_integrity_deltas"][
            "parser_duplicate_or_reordered_groups"
        ]
        == 1
    )


def test_sparse_robot_telemetry_mapping_marks_capture_incomplete(
    tmp_path: Path,
) -> None:
    workspace = tmp_path / "workspace"
    run_directory = workspace / "runs" / "demo" / "simulation"
    run_directory.mkdir(parents=True)
    robot_path = run_directory / "execution_telemetry.csv"
    robot_path.write_text(
        "sync_time_ns,trajectory_time_s\n" "1000000000,0.0\n" "2000000000,1.0\n",
        encoding="utf-8",
    )
    staging_path = workspace / "runs" / "nucleo_capture__sparse_robot.staging.csv"
    with staging_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "sensor_rx_sync_time_ns",
                "sensor_rx_ros_time_ns",
                "logical_seq",
            ],
            lineterminator="\n",
        )
        writer.writeheader()
        for index in range(1, 10):
            sensor_sync_ns = 1_000_000_000 + index * 100_000_000
            writer.writerow(
                {
                    "sensor_rx_sync_time_ns": sensor_sync_ns,
                    "sensor_rx_ros_time_ns": 5_000_000_000 + sensor_sync_ns,
                    "logical_seq": index - 1,
                }
            )
    summary = NucleoCaptureSummary(
        staging_path=staging_path,
        config=NucleoCaptureConfig(port="/dev/ttyACM0"),
        workspace=workspace,
        frames_written=9,
    )

    result = finalize_nucleo_capture(summary, robot_path)

    assert result.status == "incomplete"
    assert "maximum absolute robot/sensor mapping offset" in result.message
    manifest = json.loads(result.manifest_path.read_text(encoding="utf-8"))
    assert manifest["mapping"]["offset_ns"]["maximum_absolute"] == 500000000
    assert manifest["mapping"]["maximum_allowed_absolute_offset_ns"] == 50000000


def test_stop_refuses_to_finalize_while_reader_thread_is_alive(
    tmp_path: Path,
) -> None:
    class StuckThread:
        def join(self, timeout: float) -> None:
            del timeout

        def is_alive(self) -> bool:
            return True

    class SerialPort:
        def __init__(self) -> None:
            self.closed = False

        def cancel_read(self) -> None:
            pass

        def close(self) -> None:
            self.closed = True

    class Stream:
        def __init__(self) -> None:
            self.closed = False

        def flush(self) -> None:
            pass

        def close(self) -> None:
            self.closed = True

    session = NucleoCaptureSession(
        tmp_path,
        NucleoCaptureConfig(port="/dev/ttyACM0"),
        lambda: 0,
    )
    serial_port = SerialPort()
    stream = Stream()
    session._started = True
    session._thread = StuckThread()
    session._serial = serial_port
    session._stream = stream
    session._staging_path = tmp_path / "runs" / "nucleo_capture__stuck.staging.csv"

    with pytest.raises(NucleoCaptureError, match="reader did not stop"):
        session.stop()

    assert serial_port.closed is True
    assert stream.closed is False
    assert (
        session.capture_error == "capture reader did not stop within the join timeout"
    )


def test_finalizer_rejects_robot_destination_outside_workspace_runs(
    tmp_path: Path,
) -> None:
    workspace = tmp_path / "workspace"
    (workspace / "runs").mkdir(parents=True)
    staging_path = workspace / "runs" / "nucleo_capture__outside.staging.csv"
    staging_path.write_text(
        "sensor_rx_sync_time_ns,sensor_rx_ros_time_ns\n150,1000\n",
        encoding="utf-8",
    )
    outside = tmp_path / "outside"
    outside.mkdir()
    robot_path = outside / "execution_telemetry.csv"
    robot_path.write_text(
        "sync_time_ns,trajectory_time_s\n100,0.0\n200,0.1\n",
        encoding="utf-8",
    )
    summary = NucleoCaptureSummary(
        staging_path=staging_path,
        config=NucleoCaptureConfig(port="/dev/ttyACM0"),
        workspace=workspace,
        frames_written=1,
    )

    result = finalize_nucleo_capture(summary, robot_path)

    assert result.status == "failed"
    assert "workspace/runs" in result.message
    assert staging_path.exists()
    assert not (outside / "nucleo_telemetry.csv").exists()
    assert not (outside / "nucleo_capture.yaml").exists()


def test_finalizer_rejects_staging_path_outside_workspace_runs(
    tmp_path: Path,
) -> None:
    workspace = tmp_path / "workspace"
    run_directory = workspace / "runs" / "demo" / "real"
    run_directory.mkdir(parents=True)
    robot_path = run_directory / "execution_telemetry.csv"
    robot_path.write_text(
        "sync_time_ns,trajectory_time_s\n100,0.0\n200,0.1\n",
        encoding="utf-8",
    )
    outside = tmp_path / "outside"
    outside.mkdir()
    staging_path = outside / "nucleo_capture__outside.staging.csv"
    staging_path.write_text(
        "sensor_rx_sync_time_ns,sensor_rx_ros_time_ns\n150,1000\n",
        encoding="utf-8",
    )
    summary = NucleoCaptureSummary(
        staging_path=staging_path,
        config=NucleoCaptureConfig(port="/dev/ttyACM0"),
        workspace=workspace,
        frames_written=1,
    )

    result = finalize_nucleo_capture(summary, robot_path)

    assert result.status == "failed"
    assert "staging path must be located under workspace/runs" in result.message
    assert staging_path.exists()
    assert not result.sensor_telemetry_path.exists()
    assert not result.manifest_path.exists()


def test_finalizer_rejects_staging_symlink(tmp_path: Path) -> None:
    workspace = tmp_path / "workspace"
    run_directory = workspace / "runs" / "demo" / "real"
    run_directory.mkdir(parents=True)
    robot_path = run_directory / "execution_telemetry.csv"
    robot_path.write_text(
        "sync_time_ns,trajectory_time_s\n100,0.0\n200,0.1\n",
        encoding="utf-8",
    )
    target = workspace / "runs" / "nucleo_capture__target.staging.csv"
    target.write_text(
        "sensor_rx_sync_time_ns,sensor_rx_ros_time_ns\n150,1000\n",
        encoding="utf-8",
    )
    staging_link = workspace / "runs" / "nucleo_capture__link.staging.csv"
    staging_link.symlink_to(target)
    summary = NucleoCaptureSummary(
        staging_path=staging_link,
        config=NucleoCaptureConfig(port="/dev/ttyACM0"),
        workspace=workspace,
        frames_written=1,
    )

    result = finalize_nucleo_capture(summary, robot_path)

    assert result.status == "failed"
    assert "must not be a symlink" in result.message
    assert staging_link.is_symlink()
    assert target.exists()
    assert not result.sensor_telemetry_path.exists()
    assert not result.manifest_path.exists()
