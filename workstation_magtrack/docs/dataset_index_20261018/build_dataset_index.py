#!/usr/bin/env python3
"""Reconstruct accepted336 from named immutable metadata, never raw CSV contents.

Usage: python3 build_dataset_index.py --workspace /path/to/workstation_magtrack
Outputs are new derived documentation only. A different --output is required if
any output exists. All stored artifact paths are relative to workspace root.
"""
import argparse
import collections
import csv
import hashlib
import json
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

ORIGINAL = "reports/overnight_delivery_20261017_v1/collection_manifest.json"
SOURCES = [
    ("reports/raw_continuation_20261017_1904_v1/continuation_evidence.json", "accepted_completed_prefix", 0, 28),
    ("runs/collection_dependency_fix_20261017_v1/post_fix_real_session_acceptance_v1.json", "accepted_drawings", 28, 54),
    ("reports/raw_resume11_partial_20261017_v1/real_session_evidence_v2.json", "accepted_drawings", 54, 72),
    ("reports/raw_resume12_session_20261017_v1/real_session_evidence_v2.json", "accepted_drawings", 72, 89),
    ("collection/raw_authorizations/raw_continuation_20261017_2221_v1/continuation_evidence.json", "accepted_drawings", 89, 94),
    ("reports/raw_resume15_session_20261017_v1/real_session_evidence.json", "accepted_drawings", 94, 118),
    ("reports/raw_resume15_session2_20261017_v1/real_session_evidence.json", "accepted_drawings", 118, 141),
    ("reports/raw_resume17_session_20261017_v1/real_session_evidence.json", "accepted_drawings", 141, 165),
    ("reports/raw_resume18_session_20261018_v1/real_session_evidence.json", "accepted_drawings", 165, 177),
    ("reports/raw_resume19_session_20261018_v1/real_session_evidence.json", "accepted_drawings", 177, 185),
    ("reports/raw_resume20_session_20261018_v1/real_session_evidence.json", "accepted_drawings", 185, 206),
    ("reports/raw_resume21_session_20261018_v1/real_session_evidence.json", "accepted_drawings", 206, 230),
    ("reports/raw_resume21_boot2_session_20261018_v1/real_session_evidence.json", "accepted_drawings", 230, 263),
    ("reports/raw_resume22_session_20261018_v1/real_session_evidence.json", "accepted_drawings", 263, 286),
    ("reports/raw_resume23_session_20261018_v1/real_session_evidence.json", "accepted_drawings", 286, 307),
    ("reports/raw_resume24_session_20261018_v1/real_session_evidence.json", "accepted_drawings", 307, 321),
    ("reports/raw_resume25_session_20261018_v1/real_session_evidence.json", "accepted_drawings", 321, 333),
    ("reports/raw_resume26_session_20261018_v1/real_session_evidence.json", "accepted_drawings", 333, 336),
]


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = args.workspace.resolve()
    out = args.output or root / "docs/dataset_index_20261018"
    names = ["accepted_trajectories.json", "accepted_trajectories.csv", "accepted_backgrounds.json", "index_build_report.json"]
    require(all(not (out / name).exists() for name in names), "Refusing to overwrite derived index outputs")
    small_metadata = {}
    checked_files = set()

    def rel(value):
        if isinstance(value, dict):
            value = value["path"]
        p = Path(value)
        # Original metadata is Ubuntu24-rooted. Relocate its root when rebuilding
        # from an extracted archive, retaining the exact workspace-relative path.
        old = "/home/huaxin/Codes/ur-waypoint/workstation_magtrack/"
        if str(p).startswith(old):
            p = Path(str(p)[len(old):])
        elif p.is_absolute():
            p = p.relative_to(root)
        require(".." not in p.parts, "Artifact escaped workspace: " + str(value))
        return p.as_posix()

    def exists(value):
        p = rel(value)
        require((root / p).is_file(), "Missing artifact: " + p)
        checked_files.add(p)
        return p

    def read(value):
        p = exists(value)
        require(Path(p).suffix == ".json", "Only small JSON metadata may be read: " + p)
        content = (root / p).read_bytes()
        digest = hashlib.sha256(content).hexdigest()
        if isinstance(value, dict) and value.get("sha256"):
            require(value["sha256"] == digest, "Small JSON hash mismatch: " + p)
        small_metadata[p] = digest
        return json.loads(content)

    original = read(ORIGINAL)
    recordings = original["recordings"]
    require(len(recordings) == 336, "Original manifest must contain exactly336 records")
    rows = []
    backgrounds = []
    source_summary = []
    seen_backgrounds = set()

    def quality_fields(op, win):
        # strict-complete acceptance documents historically omit count fields.
        # Preserve that absence rather than inventing expected groups from duration.
        return {
            "operational_accepted": op["accepted"],
            "operational_status": op.get("status"),
            "policy_id": op.get("policy_id"),
            "strict_status": win.get("status"),
            "strict_accepted": op.get("strict_accepted"),
            "complete_groups_from_operational": op.get("complete_groups"),
            "expected_groups_from_operational": op.get("expected_groups"),
            "missing_groups_from_operational": op.get("missing_groups"),
            "window_complete_frames": win.get("frames"),
            "window_clock_error_at_finalization": win.get("clock_drift", {}).get("error"),
            "window_session_sensor_guard_error": win.get("session_sensor_guard_error"),
        }

    def sensor_paths(op):
        require(op.get("schema") == "raw_operational_acceptance_v1", "Unknown operational schema")
        require(op.get("accepted") is True, "Source names an operationally rejected action")
        require(op.get("policy_id") == "bounded_group_loss_10_per_3000_v1", "Unknown loss policy")
        window_path = rel(op["strict_window_manifest"])
        win = read(window_path)
        raw = exists(win["raw_csv"])
        mapping = win.get("mapping", {})
        mapped = exists(mapping["path"]) if mapping.get("path") else None
        capture = exists(mapping["manifest"]) if mapping.get("manifest") else None
        if op.get("raw_sha256"):
            require(op["raw_sha256"] == win.get("raw_sha256"), "Stored raw-sensor hash disagreement")
        return win, {
            "strict_sensor_window": window_path,
            "raw_sensor_csv": raw,
            "raw_sensor_sha256_from_existing_metadata": win.get("raw_sha256"),
            "mapped_sensor_csv": mapped,
            "mapped_sensor_manifest": capture,
            "session_id": win["session_id"],
            "epoch_id": win["epoch_id"],
            "hardware_boot_id": win["hardware_boot_id"],
            "host_boot_id": win["host_boot_id"],
            "time_mapping": win.get("time_mapping"),
            **quality_fields(op, win),
        }

    for source, key, begin, end in SOURCES:
        report = read(source)
        accepted = report[key]
        require(len(accepted) == end - begin, "Source slice length mismatch: " + source)
        for offset, item in enumerate(accepted):
            i = item.get("original_trajectory_index")
            if i is None:
                i = item["original_trajectory_ordinal"] - 1
            require(i == begin + offset, "Source not in exact ordered original slice: " + source)
            origin = recordings[i]
            require(item["trajectory_id"] == origin["trajectory_id"], "Trajectory ID differs from original manifest")
            require(item["recording_id"] == origin["recording_id"], "Recording ID differs from original manifest")
            require(item.get("original_trajectory_ordinal", i + 1) == i + 1, "Ordinal/index disagreement")
            op_ref = item.get("operational_acceptance", item.get("operational_quality_report"))
            require(op_ref is not None, "Missing operational evidence")
            op = read(op_ref)
            audit = read(item["actual_fk_audit"])
            require(audit.get("passed") is True and audit.get("status") == "passed", "Actual FK audit not passed")
            require(audit["label_reference_type"] == "nominal_rod_tip_not_magnetic_ground_truth", "Unknown reference semantics")
            win, paths = sensor_paths(op)
            if item.get("strict_sensor_window"):
                require(rel(item["strict_sensor_window"]) == paths["strict_sensor_window"], "Report/window binding differs")
                read(item["strict_sensor_window"])
            require(win.get("kind") == "trajectory", "Unknown trajectory window kind: " + str(win.get("kind")))
            require(win["metadata"]["trajectory_id"] == origin["trajectory_id"], "Window trajectory ID disagreement")
            require(win["metadata"]["recording_id"] == origin["recording_id"], "Window recording ID disagreement")
            robot = exists(audit["raw_csv"])
            fk = exists(audit["reference_csv"])
            require(rel(win["robot_telemetry"]) == robot, "Window and actual audit reference different robot telemetry")
            require(win.get("robot_telemetry_sha256") == audit.get("raw_sha256"), "Stored robot hash disagreement")
            source_csv = exists(origin["source_file"])
            recording_dir = rel(origin["recording_directory"])
            require((root / recording_dir).is_dir(), "Missing original recording directory")
            rows.append({
                "original_trajectory_index": i,
                "original_trajectory_ordinal": i + 1,
                "trajectory_id": origin["trajectory_id"],
                "recording_id": origin["recording_id"],
                "duration_s_planned": origin["duration_s"],
                "action_id": item["action_id"],
                "campaign_directory": str(Path(rel(op_ref)).parent),
                "acceptance_source": source,
                "acceptance_source_key": key,
                "acceptance_source_row": offset,
                "planned_source_csv": source_csv,
                "planned_source_sha256_from_original_manifest": origin["source_sha256"],
                "planned_recording_directory": recording_dir,
                "robot_actual_telemetry_csv": robot,
                "robot_actual_sha256_from_existing_audit": audit["raw_sha256"],
                "actual_fk_reference_csv": fk,
                "actual_fk_reference_sha256_from_existing_audit": audit["reference_sha256"],
                "actual_fk_audit": rel(item["actual_fk_audit"]),
                "actual_fk_audit_passed": True,
                "label_reference_type": audit["label_reference_type"],
                "actual_joint_source": audit["actual_joint_source"],
                "actual_rows_from_existing_audit": audit["sample_count"],
                "first_sync_time_ns": audit["first_sync_time_ns"],
                "last_sync_time_ns": audit["last_sync_time_ns"],
                "operational_acceptance": rel(op_ref),
                **paths,
            })
        bg_count = 0
        for offset, item in enumerate(report.get("accepted_backgrounds", [])):
            require(item["action_id"] not in seen_backgrounds, "Duplicate accepted background action")
            seen_backgrounds.add(item["action_id"])
            op_ref = item["operational_acceptance"]
            op = read(op_ref)
            win, paths = sensor_paths(op)
            stationary_ref = item["stationary"]
            stationary = read(stationary_ref)
            require(stationary.get("passed") is True, "Background stationary audit not passed")
            require(win.get("kind") == "background", "Unknown background kind")
            robot = exists(stationary["path"])
            exists(item["background_report"])
            backgrounds.append({
                "action_id": item["action_id"],
                "acceptance_source": source,
                "acceptance_source_row": offset,
                "campaign_directory": str(Path(rel(op_ref)).parent),
                "operational_acceptance": rel(op_ref),
                "stationary_audit": rel(stationary_ref),
                "stationary_passed": True,
                "robot_stationary_telemetry_csv": robot,
                "background_report": rel(item["background_report"]),
                **paths,
            })
            bg_count += 1
        source_summary.append({"path": source, "sha256": small_metadata[source], "records_key": key,
                               "original_index_begin_inclusive": begin, "original_index_end_exclusive": end,
                               "accepted_trajectories": len(accepted), "accepted_backgrounds_in_this_source": bg_count})

    require([r["original_trajectory_index"] for r in rows] == list(range(336)), "Accepted index not exactly0..335")
    for key in ["trajectory_id", "recording_id", "action_id", "raw_sensor_csv", "robot_actual_telemetry_csv"]:
        require(len({r[key] for r in rows}) == 336, "Duplicate binding: " + key)
    generated = datetime.now(ZoneInfo("Asia/Shanghai")).isoformat()
    common = {
        "schema_version": 1,
        "created_at_Asia_Shanghai": generated,
        "artifact_paths_relative_to": "workstation_magtrack root (archive top-level directory)",
        "scope": "derived_existing_operational_acceptance_index_not_new_raw_quality_or_physical_calibration_audit",
        "original_manifest": ORIGINAL,
        "original_manifest_sha256": small_metadata[ORIGINAL],
        "raw_csv_contents_read_or_rehashed": False,
        "original_runs_reports_or_data_modified": False,
        "stored_csv_sha256_values_are_existing_evidence_not_new_verification": True,
        "independently_calibrated_magnet_center_ground_truth": False,
        "whole_sessions_clock_clean_certified": False,
    }
    payload = {**common, "count": len(rows), "accepted_trajectories": rows}
    bg_payload = {**common, "count": len(backgrounds),
                  "coverage": "Only accepted_backgrounds explicitly listed by the18 named evidence sources; not claimed to exhaust all backgrounds, particularly early campaigns before resume10.",
                  "accepted_backgrounds": backgrounds}
    report = {**common, "passed": True, "count": len(rows), "accepted_backgrounds_indexed": len(backgrounds),
              "index_is_exact_unique_original336_order": True,
              "checks": ["original manifest trajectory and recording identity", "exact source slice order0..335",
                         "operational accepted=true", "actual FK audit passed=true", "artifact file existence",
                         "stored robot and sensor metadata binding consistency", "small metadata JSON sha256 where source supplies it"],
              "not_performed": ["raw CSV content scan or rehash", "new clock mapping or quality audit", "GPU/ROS/serial access",
                                "independent physical magnet/TCP/array calibration", "background completeness reconstruction"],
              "sources": source_summary,
              "checked_artifact_file_count": len(checked_files),
              "small_json_metadata_sha256": small_metadata,
              "strict_window_status_counts": dict(collections.Counter(r["strict_status"] for r in rows)),
              "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    out.mkdir(parents=True, exist_ok=True)
    for name, data in [("accepted_trajectories.json", payload), ("accepted_backgrounds.json", bg_payload), ("index_build_report.json", report)]:
        with (out / name).open("x", encoding="utf-8") as handle:
            json.dump(data, handle, ensure_ascii=False, indent=2, sort_keys=True)
            handle.write("\n")
    with (out / "accepted_trajectories.csv").open("x", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps({"passed": True, "accepted_trajectories": len(rows), "accepted_backgrounds_indexed": len(backgrounds),
                      "output_directory": str(out), "raw_csv_contents_read": False}, ensure_ascii=False))


if __name__ == "__main__":
    main()
