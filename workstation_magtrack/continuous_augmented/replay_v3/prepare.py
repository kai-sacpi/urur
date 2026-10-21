#!/usr/bin/env python3
"""Prepare the 120 immutable open-route sources for isolated simulation only."""
from __future__ import annotations

import argparse
import csv
from decimal import Decimal
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any

BATCH = "ca_open_elastic_batch120_20261016_v3"
PRIOR_BANKS = {"v2": "ca_open_elastic_batch120_20261016_v2", "preview": "ca_open_elastic_preview_20261016_v1"}
ID_RE = re.compile(r"^[A-Za-z0-9_-]+$")
FIELDS = ["time_s", "frame_id", "tip_x_m", "tip_y_m", "tip_z_m", "tip_qx", "tip_qy", "tip_qz", "tip_qw", "phase"]
FAMILIES = {"open_line", "gentle_s", "strong_s", "c_curve", "slanted_s", "tube_bend"}
ORIGINAL = {"planning/authority_index_v4.json": "62df51b9d225030f871620159abf3b3ccb00718e9004985fc9049b5163e28550",
            "environments/mla_short/setup.yaml": "eb8f79e46fffc50a72069c4531642737e45f0889c9962a741c2d904087f7684d",
            "environments/mla_short/zones.yaml": "3de7d095bde6693d13a39eda552945fb5ec9ff4fda05b81f7f962ce87c9a46a1",
            "environments/mla_short/robot/assets/ur5.urdf": "2ef10ba4989720ebcc9e49c64b0ecb49ac94f20ef20147ad1e841d058258c9e6"}


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path: Path) -> Any:
    return json.loads(path.read_text())


def encoded(value: Any) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False) + "\n").encode()


def safe_path(root: Path, relative: str) -> Path:
    path = root / relative
    if Path(relative).is_absolute() or not path.resolve().is_relative_to(root.resolve()):
        raise ValueError(f"unsafe relative path: {relative}")
    return path


def unique(rows: list[dict[str, Any]], label: str) -> dict[str, dict[str, Any]]:
    result = {}
    for row in rows:
        name = row.get("trajectory_id", "")
        if not ID_RE.fullmatch(name) or name in result:
            raise ValueError(f"unsafe or duplicate ID in {label}: {name}")
        result[name] = row
    return result


def protected(root: Path) -> dict[str, str]:
    for name, expected in ORIGINAL.items():
        if sha(root / name) != expected:
            raise ValueError(f"original environment/active authority changed: {name}")
    old = unique(read(root / "planning/authority_index_v4.json")["trajectories"], "original-v4")
    if len(old) != 96:
        raise ValueError("original v4 must retain exactly 96 sources")
    values = {}
    for row in old.values():
        source = safe_path(root, row["source_file"])
        if sha(source) != row["source_sha256"]:
            raise ValueError(f"original source changed: {row['trajectory_id']}")
        values[source.relative_to(root).as_posix()] = sha(source)
    for path in sorted((root / "planning").glob("*.json")):
        values[path.relative_to(root).as_posix()] = sha(path)
    environment = root / "environments/mla_short"
    for path in sorted(environment.rglob("*")):
        if path.is_file() and path.relative_to(environment).parts[0] not in ("trajectories", "recordings"):
            values[path.relative_to(root).as_posix()] = sha(path)
    values["curobo.lock"] = sha(root / "curobo.lock")
    # Freeze immutable source authorities only. v2 is still running, so its
    # status, logs, replay index and recordings must remain free to accumulate.
    for label, batch_id in PRIOR_BANKS.items():
        bank = root / "continuous_augmented/batches" / batch_id
        manifest_path = bank / "manifest.json"
        manifest = read(manifest_path)
        records = unique(manifest["records"], label)
        if manifest.get("batch_id") != batch_id or len(records) != (120 if label == "v2" else 12):
            raise ValueError(f"prior immutable bank identity/count differs: {label}")
        values[manifest_path.relative_to(root).as_posix()] = sha(manifest_path)
        for row in records.values():
            for filename, digest in ((row["csv_file"], row["sha256"]), (row["metadata_file"], row["metadata_sha256"])):
                path = safe_path(bank, filename)
                if sha(path) != digest:
                    raise ValueError(f"prior immutable source bytes differ: {label}/{filename}")
                values[path.relative_to(root).as_posix()] = digest
    return values


def bank_signatures(root: Path, batch_id: str, expected_count: int) -> dict[str, Any]:
    bank = root / "continuous_augmented/batches" / batch_id
    manifest_path = bank / "manifest.json"
    manifest = read(manifest_path)
    records = unique(manifest["records"], batch_id)
    if manifest.get("batch_id") != batch_id or len(records) != expected_count:
        raise ValueError(f"cross-bank manifest identity/count differs: {batch_id}")
    values = {"trajectory_ids": set(records), "csv_sha256": set(), "authority_seeds": set(), "seed_streams": set()}
    for name, row in records.items():
        source, metadata_path = safe_path(bank, row["csv_file"]), safe_path(bank, row["metadata_file"])
        if sha(source) != row["sha256"] or sha(metadata_path) != row["metadata_sha256"]:
            raise ValueError(f"cross-bank actual CSV/metadata hash differs: {batch_id}/{name}")
        meta = read(metadata_path)
        if meta.get("trajectory_id") != name or meta.get("seed") != row.get("seed"):
            raise ValueError(f"cross-bank metadata identity/seed differs: {batch_id}/{name}")
        streams = meta.get("seed_streams", {})
        if set(streams) != {"space", "speed", "orientation", "duration"} or any(not isinstance(v, int) or isinstance(v, bool) for v in streams.values()):
            raise ValueError(f"cross-bank seed streams missing/invalid: {batch_id}/{name}")
        values["csv_sha256"].add(row["sha256"])
        values["authority_seeds"].add(row["seed"])
        values["seed_streams"].update(streams.values())
    return {"manifest_sha256": sha(manifest_path), "signatures": values}


def cross_bank_evidence(root: Path, expected_counts: dict[str, int] | None = None) -> dict[str, Any]:
    counts = expected_counts or {"v3": 120, "v2": 120, "preview": 12}
    names = {"v3": BATCH, **PRIOR_BANKS}
    banks = {label: bank_signatures(root, batch_id, counts[label]) for label, batch_id in names.items()}
    for field in ("trajectory_ids", "csv_sha256", "authority_seeds", "seed_streams"):
        prior = banks["v2"]["signatures"][field] | banks["preview"]["signatures"][field]
        if banks["v3"]["signatures"][field] & prior:
            raise ValueError(f"new v3 bank intersects prior immutable authorities: {field}")
    report_path = root / "continuous_augmented/batches" / BATCH / "reports/cross_bank_diversity.json"
    report = read(report_path)
    bindings = {label: result["manifest_sha256"] for label, result in banks.items()}
    if (report.get("manifest_sha256") != bindings or report.get("all_disjoint") is not True
            or report.get("passed") is not True or report.get("not_robot_pass") is not True
            or report.get("integrity_errors") or report.get("collisions") != {
                "trajectory_ids": [], "csv_sha256": [], "authority_seeds": [], "seed_streams": []}):
        raise ValueError("cross-bank report does not bind three actual manifests and wholly disjoint source IDs/hashes/seeds/streams")
    return {"cross_bank_diversity_file": report_path.relative_to(root).as_posix(),
            "cross_bank_diversity_sha256": sha(report_path), "cross_bank_manifest_sha256": bindings}


def environment_hashes(root: Path) -> dict[str, str]:
    environment = root / "environments/mla_short"
    return {path.relative_to(environment).as_posix(): sha(path) for path in sorted(environment.rglob("*"))
            if path.is_file() and path.relative_to(environment).parts[0] not in ("trajectories", "recordings")}


def verify_environment_fingerprint(root: Path, manifest: dict[str, Any]) -> None:
    fingerprint = manifest.get("environment_fingerprint", {})
    values = environment_hashes(root)
    canonical = json.dumps(values, sort_keys=True, separators=(",", ":")).encode("utf-8")
    if (fingerprint.get("files") != values or fingerprint.get("sha256") != hashlib.sha256(canonical).hexdigest()
            or fingerprint.get("lock_sha256") != sha(root / "curobo.lock")
            or manifest.get("provenance", {}).get("environment_fingerprint") != fingerprint):
        raise ValueError("source batch environment fingerprint/lock differs from unchanged environment")


def verify_provenance(root: Path, manifest: dict[str, Any]) -> dict[str, str]:
    """Verify frozen generation bytes and historical references without imports."""
    batch = root / "continuous_augmented/batches" / BATCH
    inventory_path = batch / "provenance/source_inventory.json"
    inventory = read(inventory_path)
    if inventory != manifest.get("provenance"):
        raise ValueError("provenance inventory differs from manifest")
    sources = inventory.get("sources", [])
    if not sources:
        raise ValueError("provenance sources are missing")
    frozen = {}
    for entry in sources:
        name, digest = entry.get("frozen_file", ""), entry.get("sha256", "")
        if (not name.startswith("provenance/") or name in frozen
                or re.fullmatch(r"[0-9a-f]{64}", digest) is None
                or sha(safe_path(batch, name)) != digest):
            raise ValueError(f"frozen provenance source SHA/path differs: {name}")
        frozen[name] = entry
    actual = {name for name, entry in frozen.items() if entry.get("source_role") == "actual_generation_source"}
    required = {f"provenance/generator_sources/{name}.py" for name in
                ("__init__", "core", "generate", "generate_batch", "generate_extension", "common")}
    if actual != required:
        raise ValueError("actual generation source inventory differs or omits generate_extension.py")
    parent_batch = root / "continuous_augmented/batches" / PRIOR_BANKS["v2"]
    parent_path = parent_batch / "manifest.json"
    parent_manifest = read(parent_path)
    binding = {"reference_parent_batch": PRIOR_BANKS["v2"],
               "reference_parent_manifest_sha256": sha(parent_path),
               "reference_parent_manifest_origin": str(parent_path),
               "reference_sources_are_execution_imports": False}
    if (parent_manifest.get("batch_id") != PRIOR_BANKS["v2"]
            or any(inventory.get(key) != value for key, value in binding.items())):
        raise ValueError("historical reference parent provenance binding differs")
    parent_sources = {entry["frozen_file"]: entry for entry in parent_manifest["provenance"]["sources"]}
    references = [entry for entry in sources if entry.get("source_role") == "historical_open_route_reference_only_not_imported"]
    expected_references = {"provenance/reference_sources/trajectory_generator.py",
                           "provenance/reference_sources/generate_new_board_esophageal_up50_trajectories.py"}
    if {entry["frozen_file"] for entry in references} != expected_references:
        raise ValueError("historical reference inventory differs")
    for entry in references:
        name, digest = entry["frozen_file"], entry["sha256"]
        original = parent_sources.get(name, {})
        if (any(entry.get(key) != value for key, value in binding.items())
                or entry.get("reference_parent_frozen_file") != name
                or entry.get("reference_parent_source_sha256") != digest
                or entry.get("origin_path") != str(parent_batch / name)
                or original.get("sha256") != digest or sha(safe_path(parent_batch, name)) != digest):
            raise ValueError(f"historical reference source provenance differs: {name}")
    return {"provenance_inventory_file": inventory_path.relative_to(root).as_posix(),
            "provenance_inventory_sha256": sha(inventory_path)}


def validate_sources(root: Path, expected_count: int = 120, check_distribution: bool = True,
                     check_cross_bank: bool = True, check_provenance: bool = True) -> dict[str, Any]:
    batch = root / "continuous_augmented/batches" / BATCH
    manifest_path, audit_path = batch / "manifest.json", batch / "reports/source_audit.json"
    manifest, audit = read(manifest_path), read(audit_path)
    if (manifest.get("batch_id") != BATCH or manifest.get("environment_id") != "mla_short"
            or manifest.get("simulation_only") is not True or manifest.get("real_execution_authorized") is not False
            or audit.get("manifest_sha256") != sha(manifest_path)):
        raise ValueError("manifest identity or audit/manifest SHA mismatch")
    records, audits = unique(manifest["records"], "manifest"), unique(audit["trajectories"], "source-audit")
    if (len(records) != expected_count or manifest.get("trajectory_count") != expected_count
            or records.keys() != audits.keys() or audit.get("count") != expected_count
            or audit.get("passed") != expected_count or audit.get("all_source_level_pass") is not True
            or audit.get("batch_contract_errors")):
        raise ValueError("source audit is incomplete, failing, or does not join every manifest ID exactly once")
    if manifest.get("source_contract", {}).get("closed_pattern_families_allowed") is not False or manifest.get("source_contract", {}).get("repeated_laps_allowed") is not False:
        raise ValueError("closed or repeated-lap sources are forbidden")
    verify_environment_fingerprint(root, manifest)
    provenance = verify_provenance(root, manifest) if check_provenance else {}
    output = []
    for name, row in records.items():
        result = audits[name]
        source, metadata_path = safe_path(batch, row["csv_file"]), safe_path(batch, row["metadata_file"])
        actual_source, actual_metadata = sha(source), sha(metadata_path)
        if (actual_source != row["sha256"] or actual_metadata != row["metadata_sha256"]
                or result.get("authority") != {"csv_sha256": actual_source, "metadata_sha256": actual_metadata}):
            raise ValueError(f"CSV/metadata/audit SHA mismatch: {name}")
        meta = read(metadata_path)
        if check_provenance and (meta.get("provenance_inventory_file") != "provenance/source_inventory.json"
                                 or meta.get("provenance_inventory_sha256") != provenance["provenance_inventory_sha256"]):
            raise ValueError(f"metadata provenance inventory SHA binding differs: {name}")
        for key in ("trajectory_id", "group_id", "family_id", "seed", "height_profile", "draw_duration_s"):
            if meta.get(key) != row.get(key):
                raise ValueError(f"metadata/manifest {key} mismatch: {name}")
        if row.get("family_id") not in FAMILIES or row.get("orientation_mode") != "smooth_multi_axis" or meta.get("orientation", {}).get("mode") != "smooth_multi_axis":
            raise ValueError(f"unexpected family/orientation mode: {name}")
        if (result.get("source_level_pass") is not True or result.get("errors")
                or any(result.get(k) is not True for k in ("byte_exact_holds_pass", "phase_contract_pass", "analytical_endpoint_v_a_jerk_zero_pass"))
                or result.get("path", {}).get("passed") is not True
                or result.get("path", {}).get("strictly_monotone_projection") is not True
                or result.get("continuous_motion", {}).get("passed") is not True
                or result.get("so3_finite_difference", {}).get("formula_cross_check_pass") is not True):
            raise ValueError(f"independent source/model audit failed: {name}")
        safety = meta.get("safety_status", {})
        if (meta.get("environment_id") != "mla_short" or meta.get("frame_id") != "plane1"
                or meta.get("quaternion_order") != "xyzw" or meta.get("source_sample_period_s") != .05
                or safety.get("simulation_only") is not True or safety.get("real_execution_authorized") is not False
                or safety.get("source_authority_may_be_modified_by_runtime") is not False
                or safety.get("hard_collision_clearance_m") != .005 or safety.get("rod_length_m") != .190
                or safety.get("rod_radius_m") != .0025 or safety.get("provisional_position_error_envelope_m") != 0
                or safety.get("provisional_orientation_error_envelope_deg") != 0):
            raise ValueError(f"nominal simulation rod190/frame/hard-clearance contract differs: {name}")
        authority = meta["authority"]
        with source.open(newline="") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames != FIELDS:
                raise ValueError(f"CSV schema differs: {name}")
            csv_rows = list(reader)
        times = [Decimal(r["time_s"]) for r in csv_rows]
        if (len(times) < 25 or times[0] != 0 or any(not t.is_finite() for t in times)
                or any(b-a != Decimal("0.05") for a, b in zip(times, times[1:]))
                or float(times[-1]) != row["duration_s"] or float(times[-1]) != authority["duration_s"]
                or len(times) != authority["source_pose_count"] or authority.get("sha256") != actual_source
                or authority.get("csv_file") != row["csv_file"]):
            raise ValueError(f"authoritative times/count/source binding differs: {name}")
        for r in csv_rows:
            if r["frame_id"] != "plane1" or not all(math.isfinite(float(r[k])) for k in FIELDS[2:9]):
                raise ValueError(f"nonfinite pose/wrong frame: {name}")
        output.append({"trajectory_id": name, "source_file": f"environments/mla_short/trajectories/{name}.csv",
                       "source_sha256": actual_source, "metadata_sha256": actual_metadata,
                       "original_source_file": source.relative_to(root).as_posix(),
                       "metadata_file": metadata_path.relative_to(root).as_posix(), "waypoint_count": len(times),
                       "duration_s": float(times[-1]), "category": f"continuous_augmented/{row['group_id']}",
                       "group_id": row["group_id"], "family_id": row["family_id"], "seed": row["seed"],
                       "profile": row["height_profile"], "orientation_mode": row["orientation_mode"]})
    if check_distribution:
        fixed = [r for r in output if r["group_id"] == "fixed_height"]
        full = [r for r in output if r["group_id"] == "full_height_one_way"]
        if len(fixed) != 84 or len(full) != 36:
            raise ValueError("batch must contain 84 fixed-height plus 36 full-height sources")
        cells = {(round(r["profile"]["lower_m"]*1000), r["family_id"]) for r in fixed
                 if r["profile"]["lower_m"] == r["profile"]["upper_m"]}
        if cells != {(h, family) for h in range(70, 201, 10) for family in FAMILIES}:
            raise ValueError("fixed-height/family coverage differs from 14 x 6 contract")
        for family in FAMILIES:
            selected = [r for r in full if r["family_id"] == family]
            directions = [r["profile"]["direction"] for r in selected]
            if (len(selected) != 6 or directions.count("ascending_height") != 3 or directions.count("descending_height") != 3
                    or any(r["profile"]["lower_m"] != .07 or r["profile"]["upper_m"] != .2 for r in selected)):
                raise ValueError(f"full-height family/up/down coverage differs: {family}")
    cross = cross_bank_evidence(root) if check_cross_bank else {}
    return {"schema_version": 1, "source_batch_id": BATCH, "environment_id": "mla_short",
            "simulation_only": True, "real_execution_authorized": False,
            "runtime_source_or_time_mutation_permitted": False, "trajectory_count": len(output),
            "manifest_file": manifest_path.relative_to(root).as_posix(), "manifest_sha256": sha(manifest_path),
            "source_audit_file": audit_path.relative_to(root).as_posix(), "source_audit_sha256": sha(audit_path),
            "environment_fingerprint": manifest["environment_fingerprint"],
            "trajectories": output, **cross, **provenance}


def immutable_outputs(outputs: dict[Path, bytes]) -> None:
    for path, value in outputs.items():
        if path.exists() and (not path.is_file() or path.read_bytes() != value):
            raise ValueError(f"immutable destination differs; refusing overwrite: {path}")
    for path, value in outputs.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        if not path.exists():
            with path.open("xb") as stream:
                stream.write(value)


def prepare(root: Path) -> dict[str, Any]:
    root = root.resolve()
    before = protected(root)
    index = validate_sources(root)
    directory = root / "continuous_augmented/replay_v3/planning"
    outputs = {directory / "execution_index.json": encoded(index),
               directory / "protected_inputs.json": encoded({"sha256": before})}
    for row in index["trajectories"]:
        outputs[safe_path(root, row["source_file"])] = safe_path(root, row["original_source_file"]).read_bytes()
    immutable_outputs(outputs)
    if protected(root) != before:
        raise ValueError("protected inputs changed during preparation")
    return index


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[2])
    args = parser.parse_args()
    index = prepare(args.workspace)
    print(json.dumps({"trajectory_count": index["trajectory_count"], "index": "continuous_augmented/replay_v3/planning/execution_index.json", "simulation_only": True}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

