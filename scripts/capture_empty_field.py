#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Capture a named NUCLEO empty-field run without commanding the robot."""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import time
import uuid


DEFAULT_PORT = (
    "/dev/serial/by-id/"
    "usb-STMicroelectronics_STM32_Virtual_ComPort_315437643330-if00"
)
QUALITY_COUNTERS = (
    "incomplete_groups",
    "timeout_groups",
    "duplicate_or_reordered_groups",
    "duplicate_packets",
    "reordered_packets",
    "channel_sequence_drops",
    "source_sequence_drops",
    "bytes_discarded",
    "malformed_packets",
    "unexpected_channel_packets",
    "pending_evictions",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Record raw NUCLEO magnetic data without starting UR control or "
            "commanding robot motion. A timestamp and nonce are appended to --name."
        )
    )
    parser.add_argument(
        "--name",
        required=True,
        help="run label used as the output-directory prefix",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=30.0,
        help="capture duration in seconds (default: 30)",
    )
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--reuse-sync",
        action="store_true",
        help=(
            "do not send another SYNC handshake; use this when the current board "
            "boot has already been synchronized"
        ),
    )
    parser.add_argument(
        "--workspace",
        type=Path,
        default=Path(__file__).resolve().parents[1],
    )
    return parser.parse_args()


def normalized_label(value: str) -> str:
    label = value.strip()
    if not label:
        raise ValueError("--name must not be empty")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]*", label):
        raise ValueError(
            "--name may contain only letters, digits, '.', '_' and '-', and must "
            "start with a letter or digit"
        )
    return label


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    args = parse_args()
    label = normalized_label(args.name)
    if not math.isfinite(args.duration_s) or args.duration_s <= 0.0:
        raise ValueError("--duration-s must be finite and positive")
    if args.baudrate <= 0:
        raise ValueError("--baudrate must be positive")

    from hx_waypoint_nucleo import NucleoCaptureConfig, NucleoCaptureSession

    workspace = args.workspace.expanduser().resolve()
    runs = workspace / "runs"
    runs.mkdir(parents=True, exist_ok=True)
    created_at = datetime.now(timezone.utc)
    run_id = (
        f"{label}__{created_at.strftime('%Y%m%dT%H%M%S.%fZ')}__"
        f"{uuid.uuid4().hex[:8]}"
    )
    output_dir = runs / run_id
    output_dir.mkdir(parents=False, exist_ok=False)

    config = NucleoCaptureConfig(
        port=args.port,
        baudrate=args.baudrate,
        channels=(1, 2, 3, 4),
        perform_sync=not args.reuse_sync,
        preflight_timeout_s=5.0,
    )
    session = NucleoCaptureSession(workspace, config, time.time_ns)
    try:
        session.start()
    except BaseException:
        output_dir.rmdir()
        raise

    timer_started = time.monotonic()
    wall_started = datetime.now(timezone.utc)
    try:
        deadline = timer_started + args.duration_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            time.sleep(min(remaining, 0.1))
    finally:
        summary = session.stop()
        timer_finished = time.monotonic()
        wall_finished = datetime.now(timezone.utc)

    raw_path = output_dir / "nucleo_raw_telemetry.csv"
    shutil.move(str(summary.staging_path), raw_path)
    parser_stats = asdict(summary.parser_stats)
    quality = {name: int(parser_stats[name]) for name in QUALITY_COUNTERS}
    if summary.capture_error:
        status = "incomplete"
    elif any(quality.values()):
        status = "complete_with_quality_flags"
    else:
        status = "complete"

    manifest = {
        "schema_version": 1,
        "status": status,
        "experiment": "empty_sensor_field",
        "run_label": label,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "ur_control_started": False,
        "robot_motion_commanded": False,
        "capture": {
            "port": args.port,
            "baudrate": args.baudrate,
            "channels": [1, 2, 3, 4],
            "sync_mode": "reused_existing_stream" if args.reuse_sync else "performed",
            "sync_attempts": summary.sync_attempts,
            "sync_time_ms_u32": summary.sync_time_ms_u32,
            "sync_round_trip_ns": summary.sync_rtt_ns,
            "requested_duration_s": args.duration_s,
            "timer_duration_s": timer_finished - timer_started,
            "capture_clock_duration_s": (
                summary.stopped_sync_time_ns - summary.started_sync_time_ns
            )
            / 1e9,
            "started_at_utc": wall_started.isoformat(),
            "finished_at_utc": wall_finished.isoformat(),
            "started_sync_time_ns": summary.started_sync_time_ns,
            "stopped_sync_time_ns": summary.stopped_sync_time_ns,
            "capture_error": summary.capture_error,
        },
        "counts": {
            "frames_written": summary.frames_written,
            "parser": parser_stats,
            "quality_counters": quality,
        },
        "artifacts": {
            "raw_sensor_csv": {
                "path": str(raw_path),
                "sha256": sha256(raw_path),
                "size_bytes": raw_path.stat().st_size,
            }
        },
    }
    manifest_path = output_dir / "nucleo_capture.yaml"
    with manifest_path.open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2, ensure_ascii=False)
        stream.write("\n")

    checksums_path = output_dir / "sha256sums.txt"
    with checksums_path.open("x", encoding="utf-8") as stream:
        stream.write(
            f"{manifest['artifacts']['raw_sensor_csv']['sha256']}  "
            "nucleo_raw_telemetry.csv\n"
        )
        stream.write(f"{sha256(manifest_path)}  nucleo_capture.yaml\n")

    print(f"output_directory: {output_dir}")
    print(f"status: {status}")
    print(f"frames_written: {summary.frames_written}")
    print(f"sync_mode: {manifest['capture']['sync_mode']}")
    print(f"raw_sha256: {manifest['artifacts']['raw_sensor_csv']['sha256']}")
    return 0 if not summary.capture_error else 2


if __name__ == "__main__":
    raise SystemExit(main())
