#!/usr/bin/env python3
"""Collect fixed-duration empty fields using one uninterrupted NUCLEO connection."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import queue
import signal
import sys
import time

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT / "src" / "hx_waypoint_nucleo"))
from hx_waypoint_nucleo.capture import (  # noqa: E402
    NucleoCaptureConfig,
    NucleoCaptureError,
    NucleoCaptureSession,
    RAW_CAPTURE_COLUMNS,
    _raw_row,
)

ZERO_COUNTERS = (
    "malformed_packets", "bytes_discarded", "source_sequence_drops",
    "reordered_packets", "duplicate_packets", "timeout_groups",
    "pending_evictions", "unexpected_channel_packets",
    "duplicate_or_reordered_groups",
)


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def save_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")
    temporary.replace(path)


def quality_reasons(count, duration, coverage, counters):
    minimum = math.ceil(duration * 100 * 0.995)
    reasons = []
    if count < minimum:
        reasons.append(f"complete_groups={count} < {minimum}")
    denominator = count + counters["incomplete_groups"]
    ratio = counters["incomplete_groups"] / denominator if denominator else 1.0
    if ratio > 0.005:
        reasons.append(f"incomplete_group_ratio={ratio:.6f} > 0.005")
    if coverage < duration - 0.1:
        reasons.append(f"actual_sample_coverage_s={coverage:.6f} < {duration - 0.1}")
    for name in ZERO_COUNTERS:
        if counters[name]:
            reasons.append(f"{name}={counters[name]}")
    return reasons


class ObservedSession(NucleoCaptureSession):
    """Write the original full-session CSV and enqueue the identical raw rows."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.rows = queue.Queue(maxsize=20000)

    def _write_frames(self, frames):
        for frame in frames:
            if self._stop_event.is_set():
                break
            row = _raw_row(
                frame,
                sensor_rx_sync_time_ns=frame.host_monotonic_ns,
                sensor_rx_ros_time_ns=time.time_ns(),
            )
            self._writer.writerow(row)
            self._frames_written += 1
            self.rows.put_nowait((frame, row))
            self._first_frame_event.set()


def open_session(root, port):
    # Passive probe sends no SYNC. A board already streaming must never be
    # handshaken again. Only an entirely silent, unsynchronized board falls back
    # to one SYNC transmission; an invalid/partial stream stops the campaign.
    passive = ObservedSession(
        root / "session", NucleoCaptureConfig(
            port=port, perform_sync=False, preflight_timeout_s=2.0,
        ), time.time_ns,
    )
    try:
        passive.start()
        return passive, "reused_existing_stream"
    except NucleoCaptureError as exc:
        summary = passive.stop()
        save_json(root / "passive_probe.json", {
            "error": str(exc), "parser": asdict(summary.parser_stats),
            "staging_path": str(summary.staging_path),
        })
        if summary.parser_stats.bytes_received:
            raise NucleoCaptureError("Existing bytes did not yield complete frames; refusing another SYNC") from exc
        if "preflight timed out" not in str(exc):
            raise
    print("No existing stream; sending the initial SYNC once", flush=True)
    active = ObservedSession(
        root / "session", NucleoCaptureConfig(
            port=port, perform_sync=True, sync_retries=1,
        ), time.time_ns,
    )
    active.start()
    return active, "performed_once"


class Window:
    def __init__(self, root, attempt, start_ns, duration_ns, baseline):
        self.start_ns, self.end_ns = start_ns, start_ns + duration_ns
        self.baseline = baseline
        self.path = root / "in_progress" / f"background_120s_attempt_{attempt:04d}"
        self.path.mkdir()
        self.stream = (self.path / "nucleo_raw_telemetry.csv").open("x", newline="")
        self.writer = csv.DictWriter(self.stream, fieldnames=RAW_CAPTURE_COLUMNS, lineterminator="\n")
        self.writer.writeheader()
        self.count = 0
        self.first_ns = self.last_ns = None
        self.first_wall_ns = self.last_wall_ns = None

    def append(self, frame, row):
        self.writer.writerow(row)
        self.count += 1
        if self.first_ns is None:
            self.first_ns, self.first_wall_ns = frame.host_monotonic_ns, row["sensor_rx_ros_time_ns"]
        self.last_ns, self.last_wall_ns = frame.host_monotonic_ns, row["sensor_rx_ros_time_ns"]

    def finish(self, root, stats, session, sync_mode, interrupted=False):
        self.stream.flush()
        self.stream.close()
        counters = {
            name: value - self.baseline[name]
            for name, value in stats.items() if name != "channel_stats"
        }
        coverage = (self.last_ns - self.first_ns) / 1e9 if self.count else 0.0
        duration = (self.end_ns - self.start_ns) / 1e9
        reasons = quality_reasons(self.count, duration, coverage, counters)
        if interrupted:
            reasons.append("interrupted_before_window_end")
        accepted = not reasons
        target = root / ("accepted" if accepted else "rejected") / self.path.name
        self.path.rename(target)
        raw = target / "nucleo_raw_telemetry.csv"
        raw_sha = digest(raw)
        manifest = {
            "schema_version": 1,
            "status": "complete_with_quality_flags" if accepted else "rejected_by_quality",
            "experiment": "empty_sensor_field",
            "run_label": target.name,
            "generated_at_utc": utc_now(),
            "ur_control_started": False, "robot_motion_commanded": False,
            "capture": {
                "port": session.config.port, "baudrate": session.config.baudrate,
                "channels": [1, 2, 3, 4], "requested_duration_s": duration,
                "capture_clock_duration_s": duration if not interrupted else coverage,
                "window_start_monotonic_ns": self.start_ns,
                "window_end_monotonic_ns": self.end_ns,
                "actual_sample_coverage_s": coverage,
                "first_sample_wall_time_ns": self.first_wall_ns,
                "last_sample_wall_time_ns": self.last_wall_ns,
                "sync_mode": sync_mode,
                "sync_attempts_in_whole_session": session._sync_attempts,
                "capture_error": session.capture_error,
            },
            "counts": {"frames_written": self.count, "parser": counters},
            "parser_counters": {
                "raw_csv_scope": "cumulative_full_continuous_session",
                "manifest_scope": "per_window_delta",
                "baseline": self.baseline,
                "end": stats,
            },
            "quality": {"accepted": accepted, "reasons": reasons},
            "artifacts": {"raw_sensor_csv": {"path": str(raw), "sha256": raw_sha,
                                                "size_bytes": raw.stat().st_size}},
        }
        save_json(target / "nucleo_capture.yaml", manifest)
        (target / "sha256sums.txt").write_text(
            f"{raw_sha}  nucleo_raw_telemetry.csv\n"
            f"{digest(target / 'nucleo_capture.yaml')}  nucleo_capture.yaml\n"
        )
        return {"directory": str(target), "frames": self.count,
                "accepted": accepted, "reasons": reasons,
                "incomplete_groups": counters["incomplete_groups"], "raw_sha256": raw_sha}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--port", required=True)
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument("--duration-s", type=float, default=120.0)
    parser.add_argument("--max-attempts", type=int, default=150)
    parser.add_argument("--max-consecutive-rejections", type=int, default=5)
    args = parser.parse_args()
    if args.count < 1 or args.duration_s < 1 or args.max_attempts < args.count:
        parser.error("invalid count/duration/max-attempts")
    root = args.output.expanduser().resolve()
    root.mkdir(parents=True, exist_ok=False)
    for name in ("accepted", "rejected", "in_progress"):
        (root / name).mkdir()
    status = {
        "state": "starting", "started_at_utc": utc_now(),
        "output_directory": str(root), "target_accepted": args.count,
        "duration_s": args.duration_s, "max_attempts": args.max_attempts,
        "accepted": 0, "rejected": 0, "attempts": 0,
        "ur_control_started": False, "robot_motion_commanded": False,
        "quality_policy": {"minimum_frames": math.ceil(args.duration_s * 99.5),
                           "max_incomplete_group_ratio": 0.005,
                           "required_zero_counters": ZERO_COUNTERS},
    }
    window = session = None
    sync_mode = None
    stopped = False

    def request_stop(signum, frame):
        nonlocal stopped
        stopped = True

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, request_stop)

    def publish():
        status["updated_at_utc"] = utc_now()
        if session is not None:
            status["full_session_frames_written"] = session.frames_written
        if window is not None:
            status["current_window_frames"] = window.count
            status["current_window_elapsed_s"] = min(
                args.duration_s, max(0.0, (time.monotonic_ns() - window.start_ns) / 1e9)
            )
        save_json(root / "status.json", status)

    publish()
    try:
        session, sync_mode = open_session(root, args.port)
        status["state"], status["sync_mode"] = "recording", sync_mode
        previous_stats = None
        start_ns = None
        duration_ns = int(args.duration_s * 1e9)
        rejected_streak = 0
        last_publish = 0.0
        while not stopped:
            if session.capture_error:
                raise NucleoCaptureError(session.capture_error)
            try:
                frame, row = session.rows.get(timeout=0.5)
            except queue.Empty:
                if session._reader_done_event.is_set():
                    raise NucleoCaptureError("Serial reader stopped")
                if time.monotonic() - last_publish > 5:
                    publish()
                    last_publish = time.monotonic()
                if start_ns and time.monotonic_ns() - start_ns > duration_ns + 5_000_000_000:
                    raise NucleoCaptureError("No complete frame for more than five seconds after window end")
                continue
            stats = asdict(frame.stats)
            if start_ns is None:
                start_ns = frame.host_monotonic_ns + 5_000_000_000
                status["warmup_s"] = 5
                print(f"Stream ready ({sync_mode}); target={args.count} x {args.duration_s:g}s", flush=True)
            if frame.host_monotonic_ns < start_ns:
                previous_stats = stats
                continue
            if previous_stats is None:
                raise NucleoCaptureError("Missing pre-window parser baseline")
            if window is None:
                status["attempts"] += 1
                window = Window(root, status["attempts"], start_ns, duration_ns, previous_stats)
            if frame.host_monotonic_ns >= window.end_ns:
                result = window.finish(root, previous_stats, session, sync_mode)
                key = "accepted" if result["accepted"] else "rejected"
                status[key] += 1
                rejected_streak = 0 if result["accepted"] else rejected_streak + 1
                with (root / "attempts.jsonl").open("a") as stream:
                    stream.write(json.dumps(result, ensure_ascii=False) + "\n")
                print(json.dumps({"attempt": status["attempts"], **result}, ensure_ascii=False), flush=True)
                start_ns = window.end_ns
                window = None
                publish()
                if status["accepted"] >= args.count:
                    status["state"] = "complete"
                    break
                if status["attempts"] >= args.max_attempts:
                    raise RuntimeError("Attempt limit reached before target accepted count")
                if rejected_streak >= args.max_consecutive_rejections:
                    raise RuntimeError("Consecutive rejection limit reached; board/stream needs attention")
                status["attempts"] += 1
                window = Window(root, status["attempts"], start_ns, duration_ns, previous_stats)
            window.append(frame, row)
            previous_stats = stats
            if time.monotonic() - last_publish > 5:
                publish()
                last_publish = time.monotonic()
        if stopped:
            status["state"] = "interrupted"
    except BaseException as exc:
        status["state"], status["error"] = "needs_attention", f"{type(exc).__name__}: {exc}"
        print(status["error"], flush=True)
    finally:
        if window is not None:
            result = window.finish(root, previous_stats, session, sync_mode, interrupted=True)
            status["rejected"] += 1
            with (root / "attempts.jsonl").open("a") as stream:
                stream.write(json.dumps(result, ensure_ascii=False) + "\n")
            window = None
        if session is not None:
            summary = session.stop()
            raw = root / "session" / "nucleo_raw_telemetry.csv"
            summary.staging_path.rename(raw)
            save_json(root / "session" / "capture_summary.json", {
                "config": asdict(summary.config), "parser": asdict(summary.parser_stats),
                "frames_written": summary.frames_written, "sync_attempts": summary.sync_attempts,
                "capture_error": summary.capture_error, "raw_csv": str(raw),
                "raw_sha256": digest(raw),
            })
        status["finished_at_utc"] = utc_now()
        publish()
    return 0 if status["state"] == "complete" else 2


if __name__ == "__main__":
    raise SystemExit(main())
