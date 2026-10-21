#!/usr/bin/env python3
"""Expose a deterministic pseudo-terminal that emulates the NUCLEO L5 stream."""

from __future__ import annotations

import argparse
import errno
import os
import pty
import select
import signal
import struct
import time
from pathlib import Path


CONTROL_MAGIC = b"\xA5\x5A"
SYNC_COMMAND = 0x01
SYNC_ACK_COMMAND = 0x81
CONTROL_FRAME_SIZE = 8
HUB_MAGIC = b"\x55\xAA"


def _checksum(payload: bytes) -> int:
    value = 0
    for byte in payload:
        value ^= byte
    return value


def _control_frame(command: int, sync_time_ms: int) -> bytes:
    body = CONTROL_MAGIC + bytes((command,)) + struct.pack("<I", sync_time_ms)
    return body + bytes((_checksum(body),))


def _hub_packet(channel: int, channel_seq: int, source_seq: int, time_ms: int) -> bytes:
    l5 = bytearray(b"L5" + bytes((source_seq,)))
    first_sensor = (channel - 1) * 16
    for local_sensor in range(16):
        sensor = first_sensor + local_sensor
        base = 100 + sensor * 3 + (channel_seq % 100)
        l5.extend(struct.pack(">hhh", base, -base, base // 2))
    l5.extend(struct.pack(">I", time_ms & 0xFFFFFFFF))
    packet = (
        HUB_MAGIC
        + bytes((channel,))
        + struct.pack("<I", channel_seq & 0xFFFFFFFF)
        + struct.pack("<I", time_ms & 0xFFFFFFFF)
        + bytes(l5)
    )
    if len(packet) != 114:
        raise RuntimeError(f"mock HUB packet has invalid length {len(packet)}")
    return packet


def _pop_sync(buffer: bytearray) -> int | None:
    while True:
        index = buffer.find(CONTROL_MAGIC)
        if index < 0:
            if len(buffer) > 1:
                del buffer[:-1]
            return None
        if index:
            del buffer[:index]
        if len(buffer) < CONTROL_FRAME_SIZE:
            return None
        candidate = bytes(buffer[:CONTROL_FRAME_SIZE])
        if candidate[2] == SYNC_COMMAND and candidate[-1] == _checksum(candidate[:-1]):
            del buffer[:CONTROL_FRAME_SIZE]
            return struct.unpack_from("<I", candidate, 3)[0]
        del buffer[0]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--path-file", type=Path)
    args = parser.parse_args(argv)
    if not 0.1 <= args.rate_hz <= 1000.0:
        parser.error("--rate-hz must be in [0.1, 1000]")

    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)
    os.close(slave_fd)
    os.set_blocking(master_fd, False)
    if args.path_file is not None:
        args.path_file.write_text(slave_path + "\n", encoding="utf-8")
    print(slave_path, flush=True)

    running = True

    def stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    buffer = bytearray()
    synced = False
    channel_sequences = {channel: 0 for channel in range(1, 5)}
    source_seq = 0
    source_time_ms = 0
    period_s = 1.0 / args.rate_hz
    period_ms = max(1, round(period_s * 1000.0))
    next_frame = time.monotonic()
    try:
        while running:
            timeout = (
                max(0.0, min(0.05, next_frame - time.monotonic())) if synced else 0.05
            )
            readable, _, _ = select.select([master_fd], [], [], timeout)
            if readable:
                try:
                    chunk = os.read(master_fd, 4096)
                except BlockingIOError:
                    chunk = b""
                except OSError as error:
                    if error.errno != errno.EIO:
                        raise
                    chunk = b""
                if chunk:
                    buffer.extend(chunk)
                    sync_time_ms = _pop_sync(buffer)
                    if sync_time_ms is not None:
                        try:
                            os.write(
                                master_fd,
                                _control_frame(SYNC_ACK_COMMAND, sync_time_ms),
                            )
                        except BlockingIOError:
                            synced = False
                        else:
                            synced = True
                            next_frame = time.monotonic()

            now = time.monotonic()
            if synced and now >= next_frame:
                payload = bytearray()
                for channel in range(1, 5):
                    payload.extend(
                        _hub_packet(
                            channel,
                            channel_sequences[channel],
                            source_seq,
                            source_time_ms,
                        )
                    )
                    channel_sequences[channel] = (
                        channel_sequences[channel] + 1
                    ) & 0xFFFFFFFF
                try:
                    os.write(master_fd, payload)
                except BlockingIOError:
                    synced = False
                except OSError as error:
                    if error.errno != errno.EIO:
                        raise
                    synced = False
                source_seq = (source_seq + 1) & 0x0F
                source_time_ms = (source_time_ms + period_ms) & 0xFFFFFFFF
                next_frame += period_s
                if next_frame < now - period_s:
                    next_frame = now + period_s
    finally:
        os.close(master_fd)
        if args.path_file is not None:
            try:
                args.path_file.unlink()
            except FileNotFoundError:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
