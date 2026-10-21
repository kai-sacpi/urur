#!/usr/bin/env python3
"""Generate the next 120 open routes using verified historical references.

Run this entry point on Ubuntu24.  It imports the unchanged bank mathematics
and reads the two design-reference files from the preceding batch's frozen
provenance, never from an assumed current parent-repository revision.  Those
references are not executed or imported.  Its only outputs are new immutable
upstream authorities and their truthful provenance; it cannot run a robot.
"""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import platform
import sys
from typing import Any

import numpy as np
import scipy

if __package__:
    from . import generate_batch as bank
else:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import generate_batch as bank

ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parent
DEFAULT_BATCH = "ca_open_elastic_batch120_20261016_v3"
MASTER_SEED = 20261026120
REFERENCE_PARENT_BATCH = "ca_open_elastic_batch120_20261016_v2"
REFERENCE_PARENT_MANIFEST_SHA256 = "19c5d1009f4c917eddfb8e545f7fd9e0c5635b10b3d2c03dd549f667db69406c"
GENERATOR_VERSION = "open_elastic_extension_v1"
REFERENCE_NAMES = ("trajectory_generator.py", "generate_new_board_esophageal_up50_trajectories.py")


def checked_reference_sources(parent: Path) -> tuple[dict[str, Any], list[tuple[Path, str, dict[str, Any]]]]:
    """Resolve only hash-pinned frozen references, with their true origins."""
    parent = Path(parent).resolve()
    manifest_path = parent/"manifest.json"
    manifest_bytes = manifest_path.read_bytes()
    manifest_sha = bank.preview.digest(manifest_bytes)
    if manifest_sha != REFERENCE_PARENT_MANIFEST_SHA256:
        raise ValueError("reference parent manifest is not the reviewed, hash-pinned v2 authority")
    manifest = json.loads(manifest_bytes)
    inventory = json.loads((parent/"provenance/source_inventory.json").read_bytes())
    if manifest["provenance"] != inventory:
        raise ValueError("reference parent inventory differs from its frozen manifest")
    context = {"reference_parent_batch": manifest["batch_id"],
               "reference_parent_manifest_sha256": manifest_sha,
               "reference_parent_manifest_origin": str(manifest_path),
               "reference_sources_are_execution_imports": False}
    refs = []
    for name in REFERENCE_NAMES:
        relative = f"provenance/reference_sources/{name}"
        matches = [r for r in inventory["sources"] if r["frozen_file"] == relative]
        if len(matches) != 1:
            raise ValueError(f"expected exactly one frozen parent reference: {name}")
        source = parent/relative
        if source.is_symlink() or not source.is_file():
            raise ValueError(f"reference must be a real frozen file: {source}")
        if bank.preview.digest(source.read_bytes()) != matches[0]["sha256"]:
            raise ValueError(f"reference parent SHA256 mismatch: {name}")
        extra = {**context, "reference_parent_frozen_file": relative,
                 "reference_parent_source_sha256": matches[0]["sha256"],
                 "source_role": "historical_open_route_reference_only_not_imported",
                 "not_current_parent_repository_source": True}
        refs.append((source, f"reference_sources/{name}", extra))
    return context, refs


def prior_authorities(parent: Path) -> list[dict[str, Any]]:
    """Read the old 120 and preview for disjointness without changing them."""
    parents = [Path(parent)]
    preview = Path(parent).parent/"ca_open_elastic_preview_20261016_v1"
    if (preview/"manifest.json").is_file():
        parents.append(preview)
    records = []
    for old_batch in parents:
        manifest = json.loads((old_batch/"manifest.json").read_bytes())
        for item in manifest["records"]:
            csv_path, meta_path = old_batch/item["csv_file"], old_batch/item["metadata_file"]
            if (bank.preview.digest(csv_path.read_bytes()) != item["sha256"] or
                bank.preview.digest(meta_path.read_bytes()) != item["metadata_sha256"]):
                raise ValueError("existing authority hash changed before extension generation")
            meta = json.loads(meta_path.read_bytes())
            records.append({"trajectory_id": meta["trajectory_id"], "seed": meta["seed"],
                            "seed_streams": meta["seed_streams"], "sha256": item["sha256"]})
    return records


def assert_disjoint(designs: list[dict[str, Any]], old: list[dict[str, Any]]) -> dict[str, Any]:
    """Pre-write proof of new authority IDs/seeds and all four seed streams."""
    ids, seeds = [m["trajectory_id"] for m in designs], [m["seed"] for m in designs]
    streams = [value for m in designs for value in m["seed_streams"].values()]
    oldids, oldseeds = {m["trajectory_id"] for m in old}, {m["seed"] for m in old}
    oldstreams = {value for m in old for value in m["seed_streams"].values()}
    overlap = {"trajectory_id_overlap": len(set(ids) & oldids),
               "authority_seed_overlap": len(set(seeds) & oldseeds),
               "seed_stream_overlap": len(set(streams) & oldstreams)}
    if len(ids) != 120 or len(set(ids)) != 120 or len(set(seeds)) != 120 or len(set(streams)) != 480 or any(overlap.values()):
        raise ValueError(f"extension authority IDs/seeds/streams are not fully disjoint: {overlap}")
    return {"prior_authority_count": len(old), "new_authority_count": len(designs),
            "new_unique_seed_stream_count": len(set(streams)), **overlap,
            "seed_streams": ["space", "speed", "orientation", "duration"],
            "generation_stage": "before_authority_write"}


def freeze_provenance(batch: Path, parent: Path) -> dict[str, Any]:
    """Freeze actual executing source and separate verified historical refs."""
    context, historical_refs = checked_reference_sources(parent)
    fingerprint, environment = bank._environment_snapshot()
    refs: list[tuple[Path, str, dict[str, Any]]] = [
        (ROOT/name, f"generator_sources/{name}", {"source_role": "actual_generation_source"})
        for name in ("__init__.py", "core.py", "generate.py", "generate_batch.py", "generate_extension.py")]
    refs += [(WORKSPACE/"generation/common.py", "generator_sources/common.py",
              {"source_role": "actual_generation_source"})]
    refs += historical_refs
    refs += [(WORKSPACE/"inputs"/name, f"protocol/{name}", {"source_role": "frozen_protocol"})
             for name in ("collection_task_v2_20261016.txt", "real_trajectory_collection_plan_20261013.md")]
    refs += [(source, relative, {"source_role": "frozen_environment_or_dependency_lock"})
             for source, relative in environment]
    entries = []
    for source, relative, extra in refs:
        payload = source.read_bytes()
        frozen = batch/"provenance"/relative
        bank.preview._write_immutable(frozen, payload)
        entries.append({"origin_path": str(source), "frozen_file": frozen.relative_to(batch).as_posix(),
                        "sha256": bank.preview.digest(payload), **extra})
    result = {"sources": entries, "environment_fingerprint": fingerprint, **context,
              "python": platform.python_version(), "numpy": np.__version__, "scipy": scipy.__version__}
    bank.preview.write_json_immutable(batch/"provenance/source_inventory.json", result)
    return result


def generate(batch_directory: Path | None = None, master_seed: int = MASTER_SEED,
             reference_parent: Path | None = None) -> Path:
    bank.preview._check_environment()
    batch = Path(batch_directory) if batch_directory else ROOT/"batches"/DEFAULT_BATCH
    parent = Path(reference_parent) if reference_parent else ROOT/"batches"/REFERENCE_PARENT_BATCH
    if batch.is_symlink() or batch.resolve() == parent.resolve():
        raise ValueError("extension batch must be a distinct non-symlink directory")
    # Check every source dependency and cross-bank seed before writing anything.
    context, _ = checked_reference_sources(parent)
    old = prior_authorities(parent)
    designs = bank.design_records(master_seed)
    diversity = assert_disjoint(designs, old)
    for meta in designs:
        meta["batch_id"] = batch.name
        meta["generator_version"] = GENERATOR_VERSION
        meta["historical_reference_provenance"] = context
    provenance = freeze_provenance(batch, parent)
    inventory_sha = bank.preview.digest((batch/"provenance/source_inventory.json").read_bytes())
    records = [bank._write_record(batch, meta, inventory_sha) for meta in designs]
    duplicate_csv = {r["sha256"] for r in records} & {r["sha256"] for r in old}
    if duplicate_csv:
        raise ValueError("a generated extension authority duplicates an old CSV SHA")
    diversity["authority_csv_sha256_overlap"] = 0
    contract = {"closed_pattern_families_allowed": False, "repeated_laps_allowed": False,
                "fixed_height_count": 84, "full_height_one_way_count": 36,
                "fixed_height_layers_mm": list(bank.HEIGHT_LAYERS_MM), "families": list(bank.FAMILIES),
                "fixed_routes_per_layer": 6, "full_height_variants_per_family": 6,
                "full_height_direction_counts": {"ascending_height": 18, "descending_height": 18},
                "all_orientation_mode": "smooth_multi_axis",
                "provisional_3mm_2deg_envelope_removed_by_user": True,
                "unchanged_hard_limits": copy.deepcopy(bank.preview.LIMITS),
                "source_time_position_orientation_are_immutable_after_write": True,
                "space_speed_orientation_duration_seed_streams_are_independent": True,
                "discrete_V_W_quota_claimed": False}
    manifest = {"schema_version": 1, "batch_id": batch.name, "generator_version": GENERATOR_VERSION,
                "master_seed": master_seed, "trajectory_count": 120, "records": records,
                "environment_id": "mla_short", "simulation_only": True, "real_execution_authorized": False,
                "authority_contract": "every written CSV row/pose/time_s is immutable; runtime scaling, retiming, clipping, skipping and automatic slowing are forbidden",
                "source_contract": contract, "cross_bank_generation_check": diversity,
                "total_authority_duration_s": sum(r["duration_s"] for r in records),
                "environment_fingerprint": provenance["environment_fingerprint"], "provenance": provenance,
                "historical_reference_provenance": context,
                "full_robot_validation_status": "not_run; source authorities are not executable robot recordings"}
    bank.preview.write_json_immutable(batch/"manifest.json", manifest)
    return batch/"manifest.json"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="new immutable batch directory, normally on Ubuntu24")
    parser.add_argument("--seed", type=int, default=MASTER_SEED)
    parser.add_argument("--reference-parent", type=Path, help="reviewed v2 frozen batch, never the current parent repository")
    args = parser.parse_args()
    print(generate(args.output, args.seed, args.reference_parent))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
