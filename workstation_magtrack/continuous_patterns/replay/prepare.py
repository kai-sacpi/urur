#!/usr/bin/env python3
"""Freeze the audited candidate subset without altering any authoritative input."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any


BATCH = "cp_layers_and_full_h070_200_20261016_v1"
SAFE_ID = re.compile(r"^[A-Za-z0-9_-]+$")
HEADER = ["time_s", "frame_id", "tip_x_m", "tip_y_m", "tip_z_m",
          "tip_qx", "tip_qy", "tip_qz", "tip_qw", "phase"]
FROZEN_PARENT = {
    "planning/authority_index_v4.json": "62df51b9d225030f871620159abf3b3ccb00718e9004985fc9049b5163e28550",
    "environments/mla_short/setup.yaml": "eb8f79e46fffc50a72069c4531642737e45f0889c9962a741c2d904087f7684d",
    "environments/mla_short/zones.yaml": "3de7d095bde6693d13a39eda552945fb5ec9ff4fda05b81f7f962ce87c9a46a1",
    "environments/mla_short/robot/assets/ur5.urdf": "2ef10ba4989720ebcc9e49c64b0ecb49ac94f20ef20147ad1e841d058258c9e6",
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_json(path: Path) -> Any:
    return json.loads(path.read_text())


def json_bytes(value: Any) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False) + "\n").encode()


def contained(root: Path, relative: str) -> Path:
    path = root / relative
    if Path(relative).is_absolute() or not path.resolve().is_relative_to(root.resolve()):
        raise ValueError(f"unsafe or escaping relative path: {relative}")
    return path


def keyed(rows: list[dict[str, Any]], label: str) -> dict[str, dict[str, Any]]:
    result = {}
    for row in rows:
        name = row.get("trajectory_id", "")
        if not SAFE_ID.fullmatch(name) or name in result:
            raise ValueError(f"invalid or duplicate ID in {label}: {name}")
        result[name] = row
    return result


def protected_inputs(root: Path) -> dict[str, str]:
    """Prove the old active 96 sources and the environment are still untouched."""
    for relative, expected in FROZEN_PARENT.items():
        if sha256(root / relative) != expected:
            raise ValueError(f"protected parent input differs: {relative}")
    old = read_json(root / "planning/authority_index_v4.json")["trajectories"]
    if len(keyed(old, "active-v4")) != 96:
        raise ValueError("active-v4 must still contain exactly 96 original sources")
    result = {}
    for row in old:
        relative = row["source_file"]
        actual = sha256(contained(root, relative))
        if actual != row["source_sha256"]:
            raise ValueError(f"old active authority differs: {row['trajectory_id']}")
        result[relative] = actual
    for path in sorted((root / "planning").glob("*.json")):
        result[path.relative_to(root).as_posix()] = sha256(path)
    environment = root / "environments/mla_short"
    for path in sorted(environment.rglob("*")):
        if path.is_file() and path.relative_to(environment).parts[0] not in ("trajectories", "recordings"):
            result[path.relative_to(root).as_posix()] = sha256(path)
    result["curobo.lock"] = sha256(root / "curobo.lock")
    return result


def validate_bank(root: Path, *, expected_counts: tuple[int, int, int] = (156, 72, 84)) -> tuple[dict[str, Any], dict[str, Any]]:
    """Join all manifest/audit rows exactly once and verify their actual bytes."""
    batch = root / "continuous_patterns/batches" / BATCH
    manifest_path = batch / "manifest.json"
    audit_path = batch / "reports/source_audit.json"
    manifest, audit = read_json(manifest_path), read_json(audit_path)
    if manifest.get("batch_id") != BATCH or audit.get("manifest_sha256") != sha256(manifest_path):
        raise ValueError("source audit/manifest SHA or batch mismatch")
    records = keyed(manifest["records"], "manifest")
    audited = keyed(audit["trajectories"], "audit")
    total, accepted_count, rejected_count = expected_counts
    if len(records) != total or manifest.get("trajectory_count") != total or records.keys() != audited.keys():
        raise ValueError("manifest/audit IDs are missing, extra, or do not match exact candidate count")
    if audit.get("source_contract_failures") or audit.get("design_findings"):
        raise ValueError("source audit contains source/design failures")
    accepted, rejected = [], []
    for name, entry in records.items():
        row = audited[name]
        authority = row["authority"]
        for key in ("csv_file", "metadata_file", "sha256", "metadata_sha256"):
            if entry.get(key) != authority.get(key):
                raise ValueError(f"manifest/audit {key} mismatch: {name}")
        source = contained(batch, entry["csv_file"])
        metadata_path = contained(batch, entry["metadata_file"])
        if sha256(source) != entry["sha256"] or sha256(metadata_path) != entry["metadata_sha256"]:
            raise ValueError(f"source or metadata SHA mismatch: {name}")
        metadata = read_json(metadata_path)
        for key in ("trajectory_id", "group_id", "family_id", "geometry_id", "seed", "direction", "orientation_mode", "height_profile"):
            if metadata.get(key) != row.get(key):
                raise ValueError(f"metadata/audit {key} mismatch: {name}")
        if metadata.get("source_sha256") != entry["sha256"] or entry["group_id"] != row["group_id"]:
            raise ValueError(f"metadata/source identity mismatch: {name}")
        if metadata.get("environment_id") != "mla_short" or metadata.get("frame_id") != "plane1":
            raise ValueError(f"wrong environment/frame: {name}")
        for flag in ("source_contract_pass", "source_grid_motion_limits_pass", "source_level_pass"):
            if row.get(flag) is not True:
                raise ValueError(f"source audit {flag} failed: {name}")
        formula = row.get("formula", {})
        for flag in ("source_reconstruction_pass", "zero_endpoint_velocity_and_acceleration_pass", "zero_endpoint_jerk_pass", "continuous_motion_limits_pass", "continuous_XY_main_range_pass", "declared_XY_coordinate_bound_matches_recomputed"):
            if formula.get(flag) is not True:
                raise ValueError(f"continuous source audit {flag} failed: {name}")
        if row.get("source_grid_motion_limit_violations") or formula.get("continuous_motion_limit_violations"):
            raise ValueError(f"source dynamics violation: {name}")
        for flag in ("speed_tier_achieved", "axis_rate_tier_achieved"):
            if row.get("tiers", {}).get(flag) is not True:
                raise ValueError(f"source tier mismatch: {name}")
        for key in ("v_tier", "w_tier"):
            if metadata.get(key) != row["tiers"].get(key):
                raise ValueError(f"metadata/audit {key} mismatch: {name}")
        with source.open(newline="") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames != HEADER:
                raise ValueError(f"authority schema mismatch: {name}")
            csv_rows = list(reader)
        times = [float(r["time_s"]) for r in csv_rows]
        if (len(times) < 2 or times[0] != 0 or any(not math.isfinite(t) for t in times)
                or any(b <= a for a, b in zip(times, times[1:]))):
            raise ValueError(f"authority times invalid: {name}")
        if (times[-1] != authority["duration_s"] or times[-1] != metadata["duration_s"]
                or len(times) != authority["source_pose_count"] or len(times) != metadata["source_pose_count"]):
            raise ValueError(f"source duration/count mismatch: {name}")
        precheck = row["nominal_geometry_precheck"]
        clearance = entry["clearance_status"]
        if (clearance != precheck.get("status") or clearance != metadata.get("clearance_precheck", {}).get("status")
                or precheck.get("metadata_precheck_status_matches_recomputed") is not True):
            raise ValueError(f"geometry status disagreement: {name}")
        record = {"trajectory_id": name, "source_sha256": entry["sha256"],
                  "metadata_sha256": entry["metadata_sha256"], "duration_s": times[-1],
                  "waypoint_count": len(times), "category": f"continuous_patterns/{entry['group_id']}",
                  "profile": metadata["height_profile"], "group_id": entry["group_id"],
                  "seed": metadata["seed"], "family_id": metadata["family_id"],
                  "v_tier": metadata["v_tier"], "w_tier": metadata["w_tier"],
                  "original_source_file": source.relative_to(root).as_posix(),
                  "metadata_file": metadata_path.relative_to(root).as_posix(),
                  "clearance_status": clearance}
        if clearance == "passed_rod_subset_only" and precheck.get("passed") is True:
            if precheck.get("board", {}).get("status") != "passed" or any(x.get("status") != "passed" for x in precheck.get("external_cuboids", [])):
                raise ValueError(f"geometry sub-check failed despite passed status: {name}")
            record["source_file"] = f"environments/mla_short/trajectories/{name}.csv"
            accepted.append(record)
        elif clearance == "rejected_rod_subset" and precheck.get("passed") is False:
            record.update(status="rejected_upstream_rod_envelope", evidence=precheck)
            rejected.append(record)
        else:
            raise ValueError(f"unsupported/contradictory geometry classification: {name}")
    if (len(accepted), len(rejected)) != (accepted_count, rejected_count):
        raise ValueError(f"accepted/rejected count mismatch: {len(accepted)}/{len(rejected)}")
    if expected_counts == (156, 72, 84):
        groups = {g: sum(r["group_id"] == g for r in accepted) for g in {r["group_id"] for r in accepted}}
        if groups != {"stratified": 52, "guarded_full_height": 20}:
            raise ValueError(f"accepted group distribution mismatch: {groups}")
    provenance = {"batch_id": BATCH, "manifest_file": manifest_path.relative_to(root).as_posix(),
                  "manifest_sha256": sha256(manifest_path), "source_audit_file": audit_path.relative_to(root).as_posix(),
                  "source_audit_sha256": sha256(audit_path)}
    execution = {"schema_version": 1, "environment_id": "mla_short", "simulation_only": True,
                 "real_execution_authorized": False, "runtime_mutation_permitted": False,
                 "authority_scope": "accepted_subset_only_not_parent_active_v4", **provenance,
                 "candidate_count": total, "trajectory_count": len(accepted), "trajectories": accepted}
    rejections = {"schema_version": 1, **provenance, "rejected_count": len(rejected), "trajectories": rejected}
    return execution, rejections


def immutable_outputs(outputs: dict[Path, bytes]) -> None:
    # Validate the entire plan before creating a single destination. Never replace
    # a frozen authority or a prior incompatible preparation snapshot.
    for path, content in outputs.items():
        if path.exists() and (not path.is_file() or path.read_bytes() != content):
            raise ValueError(f"immutable destination differs; refusing overwrite: {path}")
    for path, content in outputs.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        if not path.exists():
            with path.open("xb") as stream:
                stream.write(content)


def prepare(root: Path) -> dict[str, Any]:
    root = root.resolve()
    before = protected_inputs(root)
    execution, rejections = validate_bank(root)
    directory = root / "continuous_patterns/replay/planning"
    outputs = {directory / "execution_index.json": json_bytes(execution),
               directory / "rejections.json": json_bytes(rejections),
               directory / "protected_inputs.json": json_bytes({"schema_version": 1, "sha256": before})}
    for row in execution["trajectories"]:
        outputs[contained(root, row["source_file"])] = contained(root, row["original_source_file"]).read_bytes()
    immutable_outputs(outputs)
    if protected_inputs(root) != before:
        raise ValueError("protected parent inputs changed during preparation")
    return execution


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[2])
    args = parser.parse_args()
    result = prepare(args.workspace)
    print(json.dumps({"accepted": result["trajectory_count"], "rejected": result["candidate_count"] - result["trajectory_count"],
                      "execution_index": "continuous_patterns/replay/planning/execution_index.json", "simulation_only": True}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
