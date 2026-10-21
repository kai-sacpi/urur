#!/usr/bin/env python3
"""Audit one completed 54-point MLA real-robot + NUCLEO batch."""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import yaml


EXPECTED_COUNT = 54
MAX_INTEGRITY = {
    "parser_incomplete_groups": 10,
    "parser_channel_sequence_drops": 10,
    "parser_duplicate_packets": 10,
    "parser_reordered_packets": 10,
}
ALLOWED_NONZERO_DELTAS = frozenset(MAX_INTEGRITY)
FINAL_FIELDS = (
    "sequence",
    "condition_id",
    "recording_id",
    "robot_status",
    "sensor_status",
    "quality_decision",
    "robot_rows",
    "sensor_rows",
    "mapped_sensor_frames",
    "incomplete_groups",
    "channel_sequence_drops",
    "duplicate_packets",
    "reordered_packets",
    "maximum_mapping_offset_ms",
    "sync_mode",
    "run_id",
    "durable_robot_telemetry",
    "durable_sensor_telemetry",
    "durable_nucleo_manifest",
    "robot_telemetry_sha256",
    "sensor_telemetry_sha256",
    "nucleo_manifest_sha256",
)


class AuditError(RuntimeError):
    pass


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise AuditError(f"missing CSV header: {path}")
        rows = [dict(row) for row in reader]
    return list(reader.fieldnames), rows


def require_file(path: Path) -> Path:
    if path.is_symlink() or not path.is_file():
        raise AuditError(f"required regular file is missing: {path}")
    return path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, required=True)
    args = parser.parse_args()
    root = args.run_root.expanduser().resolve()
    schedule_path = require_file(root / "execution_schedule.csv")
    _, schedule = read_csv(schedule_path)
    if len(schedule) != EXPECTED_COUNT:
        raise AuditError(f"expected {EXPECTED_COUNT} schedule rows, got {len(schedule)}")
    sequences = [int(row["sequence"]) for row in schedule]
    if sequences != list(range(1, EXPECTED_COUNT + 1)):
        raise AuditError("schedule sequence is not exactly 1..54")
    if len({row["condition_id"] for row in schedule}) != EXPECTED_COUNT:
        raise AuditError("condition identities are not unique")
    if len({row["recording_id"] for row in schedule}) != EXPECTED_COUNT:
        raise AuditError("recording identities are not unique")

    results: list[dict[str, object]] = []
    status_counts: Counter[str] = Counter()
    sync_counts: Counter[str] = Counter()
    integrity_totals: Counter[str] = Counter()
    integrity_maxima: Counter[str] = Counter()
    sensor_row_counts: list[int] = []
    robot_row_counts: list[int] = []
    maximum_mapping_offset_ns = 0
    maximum_robot_position_error_m = 0.0
    maximum_actual_tip_speed_mps = 0.0

    for row in schedule:
        sequence = int(row["sequence"])
        condition = row["condition_id"]
        recording_id = row["recording_id"]
        source_manifest = require_file(Path(row["nucleo_manifest"]))
        source_run = source_manifest.parent.parent
        run_id = source_run.name
        if not run_id.startswith(f"{recording_id}__"):
            raise AuditError(f"run/recording identity mismatch at sequence {sequence}")
        durable_real = root / "artifacts" / run_id / "real"
        robot_path = require_file(durable_real / "execution_telemetry.csv")
        sensor_path = require_file(durable_real / "nucleo_telemetry.csv")
        manifest_path = require_file(durable_real / "nucleo_capture.yaml")
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
        if not isinstance(manifest, dict):
            raise AuditError(f"invalid NUCLEO manifest: {manifest_path}")
        capture = manifest.get("capture")
        mapping = manifest.get("mapping")
        counts = manifest.get("counts")
        artifacts = manifest.get("artifacts")
        if not all(isinstance(value, dict) for value in (capture, mapping, counts, artifacts)):
            raise AuditError(f"incomplete NUCLEO manifest structure at sequence {sequence}")
        if capture.get("capture_error") is not None:
            raise AuditError(f"NUCLEO capture error at sequence {sequence}: {capture['capture_error']}")

        sensor_status = str(manifest.get("status"))
        if sensor_status not in {"complete", "incomplete"}:
            raise AuditError(f"invalid sensor status at sequence {sequence}: {sensor_status}")
        deltas = mapping.get("parser_integrity_deltas")
        if not isinstance(deltas, dict):
            raise AuditError(f"missing parser integrity deltas at sequence {sequence}")
        normalized_deltas = {key: int(value) for key, value in deltas.items()}
        for key, value in normalized_deltas.items():
            if value < 0:
                raise AuditError(f"negative integrity delta {key} at sequence {sequence}")
            if key not in ALLOWED_NONZERO_DELTAS and value != 0:
                raise AuditError(f"unaccepted integrity delta {key}={value} at sequence {sequence}")
        for key, limit in MAX_INTEGRITY.items():
            value = normalized_deltas.get(key, 0)
            if value > limit:
                raise AuditError(f"integrity delta {key}={value} exceeds {limit} at sequence {sequence}")
            integrity_totals[key] += value
            integrity_maxima[key] = max(integrity_maxima[key], value)

        offset_ns = int(mapping.get("offset_ns", {}).get("maximum_absolute", -1))
        if offset_ns < 0 or offset_ns > 50_000_000:
            raise AuditError(f"timestamp mapping offset out of bounds at sequence {sequence}: {offset_ns}")
        maximum_mapping_offset_ns = max(maximum_mapping_offset_ns, offset_ns)

        robot_fields, robot_rows = read_csv(robot_path)
        required_robot = {
            "sync_time_ns",
            "trajectory_time_s",
            "state",
            "speed_scaling",
            "position_error_m",
            "actual_tip_speed_mps",
        }
        if not required_robot.issubset(robot_fields) or not robot_rows:
            raise AuditError(f"robot telemetry schema/data invalid at sequence {sequence}")
        trajectory_times = [float(item["trajectory_time_s"]) for item in robot_rows]
        if min(trajectory_times) < -1e-9 or max(trajectory_times) < 2.9 or max(trajectory_times) > 3.05:
            raise AuditError(f"robot trajectory-time coverage invalid at sequence {sequence}")
        if any(item["state"] != "EXECUTING" for item in robot_rows):
            raise AuditError(f"unexpected robot telemetry state at sequence {sequence}")
        if any(abs(float(item["speed_scaling"]) - 1.0) > 1e-9 for item in robot_rows):
            raise AuditError(f"speed scaling changed at sequence {sequence}")
        position_errors = [abs(float(item["position_error_m"])) for item in robot_rows]
        tip_speeds = [abs(float(item["actual_tip_speed_mps"])) for item in robot_rows]
        if not all(math.isfinite(value) for value in position_errors + tip_speeds):
            raise AuditError(f"non-finite robot tracking metric at sequence {sequence}")
        maximum_robot_position_error_m = max(
            maximum_robot_position_error_m,
            max(position_errors),
        )
        maximum_actual_tip_speed_mps = max(
            maximum_actual_tip_speed_mps,
            max(tip_speeds),
        )

        sensor_fields, sensor_rows = read_csv(sensor_path)
        required_sensor = {
            "sync_time_ns",
            "trajectory_time_s",
            "robot_ros_time_ns",
            "sensor_rx_sync_time_ns",
            "robot_row_index",
        }
        expected_measurements = {
            f"sensor_{index:02d}_{axis}_ut"
            for index in range(64)
            for axis in ("x", "y", "z")
        }
        if not required_sensor.issubset(sensor_fields) or not expected_measurements.issubset(sensor_fields):
            raise AuditError(f"sensor telemetry does not contain common timestamps and 192 channels at sequence {sequence}")
        mapped_count = int(counts.get("mapped_sensor_frames", -1))
        robot_count = int(counts.get("robot_samples", -1))
        if len(sensor_rows) != mapped_count or len(robot_rows) != robot_count:
            raise AuditError(f"manifest/CSV row count mismatch at sequence {sequence}")
        if mapped_count < 250:
            raise AuditError(f"too few mapped sensor frames at sequence {sequence}: {mapped_count}")

        robot_hash = sha256(robot_path)
        sensor_hash = sha256(sensor_path)
        expected_robot_hash = artifacts.get("robot_telemetry_csv", {}).get("sha256")
        expected_sensor_hash = artifacts.get("sensor_telemetry_csv", {}).get("sha256")
        if robot_hash != expected_robot_hash or sensor_hash != expected_sensor_hash:
            raise AuditError(f"telemetry hash mismatch at sequence {sequence}")

        sync_mode = str(capture.get("sync", {}).get("mode"))
        expected_sync = "performed" if sequence == 1 else "reused_existing"
        if sync_mode != expected_sync:
            raise AuditError(f"unexpected SYNC mode at sequence {sequence}: {sync_mode}")
        sync_counts[sync_mode] += 1

        play_rc = int(row["play_exit_code"])
        log_path = require_file(root / f"{sequence:03d}__{condition}.log")
        log_text = log_path.read_text(encoding="utf-8", errors="replace")
        if play_rc not in {0, 2}:
            raise AuditError(f"unexpected play exit code at sequence {sequence}: {play_rc}")
        if play_rc == 2 and "robot playback succeeded" not in log_text:
            raise AuditError(f"nonzero playback did not prove robot success at sequence {sequence}")

        decision = "COMPLETE" if sensor_status == "complete" else "ACCEPTED_BOUNDED_SENSOR_LOSS"
        status_counts[decision] += 1
        robot_row_counts.append(len(robot_rows))
        sensor_row_counts.append(len(sensor_rows))
        results.append(
            {
                "sequence": sequence,
                "condition_id": condition,
                "recording_id": recording_id,
                "robot_status": "SUCCEEDED",
                "sensor_status": sensor_status,
                "quality_decision": decision,
                "robot_rows": len(robot_rows),
                "sensor_rows": len(sensor_rows),
                "mapped_sensor_frames": mapped_count,
                "incomplete_groups": normalized_deltas.get("parser_incomplete_groups", 0),
                "channel_sequence_drops": normalized_deltas.get("parser_channel_sequence_drops", 0),
                "duplicate_packets": normalized_deltas.get("parser_duplicate_packets", 0),
                "reordered_packets": normalized_deltas.get("parser_reordered_packets", 0),
                "maximum_mapping_offset_ms": f"{offset_ns / 1e6:.6f}",
                "sync_mode": sync_mode,
                "run_id": run_id,
                "durable_robot_telemetry": str(robot_path),
                "durable_sensor_telemetry": str(sensor_path),
                "durable_nucleo_manifest": str(manifest_path),
                "robot_telemetry_sha256": robot_hash,
                "sensor_telemetry_sha256": sensor_hash,
                "nucleo_manifest_sha256": sha256(manifest_path),
            }
        )

    index_path = root / "final_artifact_index.csv"
    with index_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FINAL_FIELDS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(results)

    start_utc = schedule[0]["start_utc"]
    end_utc = schedule[-1]["end_utc"]
    start = datetime.fromisoformat(start_utc.replace("Z", "+00:00"))
    end = datetime.fromisoformat(end_utc.replace("Z", "+00:00"))
    summary = {
        "schema_version": 1,
        "manifest_type": "mla_grid_54_real_nucleo_audit",
        "generated_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "environment_id": "mla",
        "run_root": str(root),
        "experiment_window": {
            "start_utc": start_utc,
            "end_utc": end_utc,
            "wall_duration_s": (end - start).total_seconds(),
            "authoritative_hold_duration_s": EXPECTED_COUNT * 3,
        },
        "counts": {
            "conditions": EXPECTED_COUNT,
            "robot_succeeded": EXPECTED_COUNT,
            "sensor_complete": status_counts["COMPLETE"],
            "sensor_accepted_bounded_loss": status_counts["ACCEPTED_BOUNDED_SENSOR_LOSS"],
            "sensor_rejected": 0,
            "sync_performed": sync_counts["performed"],
            "sync_reused": sync_counts["reused_existing"],
            "robot_rows_min": min(robot_row_counts),
            "robot_rows_max": max(robot_row_counts),
            "mapped_sensor_rows_min": min(sensor_row_counts),
            "mapped_sensor_rows_max": max(sensor_row_counts),
        },
        "integrity_totals": dict(sorted(integrity_totals.items())),
        "integrity_per_condition_maxima": dict(sorted(integrity_maxima.items())),
        "maximum_mapping_offset_ms": maximum_mapping_offset_ns / 1e6,
        "robot_tracking": {
            "maximum_tip_position_error_mm": maximum_robot_position_error_m * 1000.0,
            "maximum_actual_tip_speed_mps": maximum_actual_tip_speed_mps,
            "speed_scaling": 1.0,
        },
        "contracts": {
            "sensor_channels": 192,
            "position_frame": "plane1",
            "target_positions_changed": False,
            "scaling_retiming_cropping_skipping_allowed": False,
            "first_sync_only_once": True,
            "subsequent_sync_reused": True,
            "final_return_home_executed": require_file(root / "final_return_home.log").is_file(),
        },
        "files": {
            "schedule": str(schedule_path),
            "final_index": str(index_path),
            "batch_log": str(require_file(root / "batch.log")),
            "final_return_home_log": str(root / "final_return_home.log"),
        },
    }
    summary_path = root / "audit_summary.yaml"
    summary_path.write_text(yaml.safe_dump(summary, sort_keys=False), encoding="utf-8")
    print(summary_path)
    print(index_path)
    print(yaml.safe_dump(summary["counts"], sort_keys=False).strip())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
