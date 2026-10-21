#!/usr/bin/env python3
"""Passive forensic A/B/A capture; never sends SYNC or accepts backgrounds.

Only this diagnostic reader changes its read strategy. The production parser,
its 40 ms deadline, existing capture files and quality policy stay unchanged.
All received bytes, read boundaries and decoded packet headers are retained.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys
import time

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT / "src" / "hx_waypoint_nucleo"))
from hx_waypoint_nucleo.protocol import HubStreamParser  # noqa: E402


def save_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n")
    temporary.replace(path)


def sha256(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def read_chunk(port, strategy):
    if strategy == "blocking4096":
        return port.read(4096)
    if strategy == "drain_available":
        # Wait for one byte, not a 4096-byte block. Drain bytes already queued
        # without adding a full read timeout to every nonempty batch.
        return port.read(min(65536, max(1, port.in_waiting)))
    raise ValueError(strategy)


def header(record):
    return {
        "channel": record.channel_id, "channel_seq": record.channel_seq,
        "src_seq": record.src_seq, "source_ms": record.source_time_ms_u32,
        "nucleo_ms": record.nucleo_time_ms_u32,
        "host_ns": record.host_monotonic_ns,
    }


class TracedParser(HubStreamParser):
    def __init__(self, packets, events):
        super().__init__()  # Unchanged production settings: 40 ms, eight groups.
        self.packets, self.events = packets, events
        self.phase = "startup"

    def event(self, kind, **values):
        self.events.write(json.dumps({"kind": kind, "phase": self.phase, **values}) + "\n")

    def _expire_due(self, now_ns):
        for pending in self._pending.values():
            age_ns = now_ns - pending.first_seen_ns
            if age_ns >= self.incomplete_group_timeout_ns:
                self.event("expire_before_feed", host_ns=now_ns,
                           age_ms=age_ns / 1e6, src_seq=pending.src_seq,
                           logical_seq=pending.logical_seq,
                           records=[header(r) for r in pending.records.values()])
        super()._expire_due(now_ns)

    def _accept_record(self, record):
        values = header(record)
        self.packets.writerow({"phase": self.phase, **values})
        before_reordered = self._reordered_packets
        before = {"latest_src_seq": self._latest_src_seq,
                  "latest_logical_seq": self._latest_logical_seq,
                  "last_emitted_logical_seq": self._last_emitted_logical_seq}
        frame = super()._accept_record(record)
        if self._reordered_packets > before_reordered:
            self.event("reordered_rejection", packet=values, state_before=before)
        return frame


def delta_stats(end, start):
    return {key: value - start[key] for key, value in end.items()
            if key != "channel_stats"}


def main():
    import serial

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--phase-s", type=float, default=30)
    args = ap.parse_args()
    if args.phase_s < 10:
        ap.error("Each phase must be at least ten seconds")
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    provenance = root / "provenance"
    provenance.mkdir()
    for source in (Path(__file__), PROJECT / "src/hx_waypoint_nucleo/hx_waypoint_nucleo/protocol.py",
                   PROJECT / "src/hx_waypoint_nucleo/hx_waypoint_nucleo/capture.py"):
        shutil.copy2(source, provenance / source.name)
    phases = ["blocking4096", "drain_available", "blocking4096"]
    status = {"state": "starting", "started_utc": datetime.now(timezone.utc).isoformat(),
              "purpose": "diagnosis_only_not_accepted_backgrounds", "sync_sent": 0,
              "serial_write_calls": 0, "baudrate": 115200, "timeout_s": .02,
              "group_timeout_s": .04, "port": args.port,
              "phases": [], "phase_duration_s": args.phase_s}
    save_json(root / "status.json", status)
    try:
        with (root / "wire.bin").open("xb") as raw, \
             (root / "reads.csv").open("x", newline="") as reads_file, \
             (root / "packets.csv").open("x", newline="") as packets_file, \
             (root / "parser_events.jsonl").open("x") as events, \
             serial.Serial(args.port, 115200, timeout=.02, write_timeout=1,
                           exclusive=True, xonxoff=False, rtscts=False,
                           dsrdtr=False) as port:
            reads = csv.DictWriter(reads_file, fieldnames=("phase", "read_start_ns", "host_ns", "wall_ns", "offset", "length"))
            packets = csv.DictWriter(packets_file, fieldnames=("phase", "channel", "channel_seq", "src_seq", "source_ms", "nucleo_ms", "host_ns"))
            reads.writeheader()
            packets.writeheader()
            decoder = TracedParser(packets, events)
            offset, first_byte_ns = 0, None
            status["state"] = "recording"
            for index, strategy in enumerate(phases):
                label = f"{index + 1}_{strategy}"
                decoder.phase = label
                begin = time.monotonic_ns()
                end = begin + int(args.phase_s * 1e9)
                baseline = None
                phase = {"label": label, "strategy": strategy, "start_ns": begin,
                         "excluded_startup_s": 5}
                status["current_phase"] = label
                save_json(root / "status.json", status)
                print(f"phase={label}; passive read only", flush=True)
                while time.monotonic_ns() < end:
                    read_start_ns = time.monotonic_ns()
                    chunk = read_chunk(port, strategy)
                    now_ns, wall_ns = time.monotonic_ns(), time.time_ns()
                    raw.write(chunk)
                    reads.writerow({"phase": label, "read_start_ns": read_start_ns,
                                    "host_ns": now_ns, "wall_ns": wall_ns,
                                    "offset": offset, "length": len(chunk)})
                    offset += len(chunk)
                    decoder.feed(chunk, now_ns)
                    if chunk and first_byte_ns is None:
                        first_byte_ns = now_ns
                    if first_byte_ns is None and now_ns - begin > 2_000_000_000:
                        raise RuntimeError("No existing stream; no SYNC was sent")
                    if baseline is None and now_ns - begin >= 5_000_000_000:
                        baseline = asdict(decoder.stats)
                        phase["measurement_start_ns"] = now_ns
                phase["end_ns"] = time.monotonic_ns()
                phase["measurement_s"] = (phase["end_ns"] - phase["measurement_start_ns"]) / 1e9
                phase["parser_delta"] = delta_stats(asdict(decoder.stats), baseline)
                status["phases"].append(phase)
                print(json.dumps(phase), flush=True)
                save_json(root / "status.json", status)
            status["parser_before_close"] = asdict(decoder.stats)
            decoder.close()
            status["bytes_saved"] = offset
            status["state"] = "complete"
    except BaseException as exc:
        status["state"] = "failed"
        status["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        status["finished_utc"] = datetime.now(timezone.utc).isoformat()
        status["sha256"] = {p.name: sha256(p) for p in root.iterdir()
                            if p.is_file() and p.name != "status.json"}
        save_json(root / "status.json", status)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
