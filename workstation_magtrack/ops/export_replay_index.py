#!/usr/bin/env python3
"""Export an honest, portable replay inventory and verify local sealed bytes."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
from typing import Any


PHASES = ("compile", "build", "inspect", "play")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_seal(recording: Path, expected_source_hash: str) -> dict[str, Any]:
    if not recording.is_dir():
        return {"status": "not_available_on_this_host"}
    try:
        if sha256(recording / "source_tip_trajectory.csv") != expected_source_hash:
            raise ValueError("sealed source does not match authority")
        count = 0
        for line in (recording / "sha256sums.txt").read_text().splitlines():
            expected, separator, filename = line.partition("  ")
            path = recording / filename
            if not separator or len(expected) != 64 or not path.resolve().is_relative_to(recording.resolve()):
                raise ValueError("invalid or escaping checksum entry")
            if sha256(path) != expected:
                raise ValueError(f"sealed checksum mismatch: {filename}")
            count += 1
        if count == 0:
            raise ValueError("empty sealed checksum list")
        return {"status": "passed", "checked_sealed_files": count}
    except (OSError, ValueError) as error:
        return {"status": "failed", "error": str(error)}


def export(root: Path, status_path: Path) -> dict[str, Any]:
    root = root.resolve()
    active = json.loads((root / "planning/authority_index_v4.json").read_text())
    status = json.loads(status_path.read_text())
    rows = []
    for authority in active["trajectories"]:
        name = authority["trajectory_id"]
        state = status["trajectories"].get(name, {})
        if state.get("source", {}).get("sha256") not in (None, authority["source_sha256"]):
            raise ValueError(f"state belongs to a different authority: {name}")
        rid = state.get("build", {}).get("recording_id", "")
        if rid and (Path(rid).name != rid or rid in {".", ".."}):
            raise ValueError("unsafe recording ID")
        relative = f"environments/mla_short/recordings/{rid}" if rid else ""
        local = check_seal(root / relative, authority["source_sha256"]) if rid else {"status": "not_built"}
        phases = {p: state.get(p, {}).get("status", "pending") for p in PHASES}
        witness = state.get("play", {}).get("playback_audit", {}).get("controller_witness", {})
        ready = (all(v == "passed" for v in phases.values()) and
                 witness.get("status") == "passed" and local["status"] == "passed")
        compile_report = state.get("compile", {}).get("report", {})
        rows.append({
            "trajectory_id": name, "category": authority["category"],
            "session_id": authority.get("session_id", "calibration_or_sync"),
            "data_role": authority.get("data_role", ""),
            "duration_s": authority["duration_s"], "source_file": authority["source_file"],
            "source_sha256": authority["source_sha256"], **phases,
            "recording_id": rid, "recording_directory": relative,
            "local_sealed_checksum_status": local["status"],
            "local_sealed_checksum_error": local.get("error", ""),
            "coherent_playback_witness": witness.get("status", "pending"),
            "ready_for_simulation_replay_on_this_host": ready,
            "minimum_collision_distance_m": compile_report.get("minimum_collision_distance_m"),
            "max_joint_velocity_ratio": compile_report.get("max_joint_velocity_ratio"),
            "max_joint_acceleration_ratio": compile_report.get("max_joint_acceleration_ratio"),
            "max_joint_jerk_ratio": compile_report.get("max_joint_jerk_ratio"),
            "max_tip_position_error_m": compile_report.get("max_tip_position_error_m"),
            "max_tip_orientation_error_deg": compile_report.get("max_tip_orientation_error_deg"),
            "max_independent_fk_position_error_m": witness.get("max_coherent_actual_source_position_error_m"),
            "max_independent_fk_orientation_error_deg": witness.get("max_coherent_actual_source_orientation_error_deg"),
            "real_execution_authorized": False,
        })
    result = {"schema_version": 1, "environment_id": "mla_short", "simulation_only": True,
              "trajectory_count": len(rows),
              "ready_on_this_host": sum(r["ready_for_simulation_replay_on_this_host"] for r in rows),
              "status_input": str(status_path), "recordings": rows}
    reports = root / "reports"
    reports.mkdir(exist_ok=True)
    (reports / "replay_index.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    with (reports / "replay_index.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--status", type=Path)
    parser.add_argument("--require-all", action="store_true")
    args = parser.parse_args()
    root = args.workspace.resolve()
    result = export(root, args.status or root / "runs/authority_v4/status.json")
    print(json.dumps({k: v for k, v in result.items() if k != "recordings"}))
    return 0 if not args.require_all or result["ready_on_this_host"] == result["trajectory_count"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
