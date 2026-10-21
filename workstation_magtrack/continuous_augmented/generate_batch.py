#!/usr/bin/env python3
"""Author the approved 120-route open elastic bank, upstream and immutable.

This entry point reuses the independently reviewed preview mathematics and
priors without changing its frozen files.  It adds 14 fixed-height layers
times 6 open route families, plus 6 one-way full-height variants per family.
Every new source has independently seeded smooth speed and multi-axis pose.
It never invokes ROS, a GPU, playback, a sensor, or an actual robot.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
import platform
import sys
from typing import Any

import numpy as np
import scipy

if __package__:
    from . import generate as preview
else:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import generate as preview

ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parent
REPO = WORKSPACE.parent
DEFAULT_BATCH = "ca_open_elastic_batch120_20261016_v2"
MASTER_SEED = 20261016120
HEIGHT_LAYERS_MM = tuple(range(70, 201, 10))
FAMILIES = preview.FAMILIES
FULL_HEIGHT_VARIANTS = 6
GENERATOR_VERSION = "open_elastic_batch_v2"


def _multi_axis_orientation(seed_stream: int) -> list[list[float]]:
    """The same approved preview prior, without the two fixed controls."""
    rng = np.random.default_rng(seed_stream)
    yaw0 = float(rng.uniform(-math.pi, math.pi))
    roll = np.deg2rad(rng.uniform(-10., 10., 6))
    pitch = np.deg2rad(rng.uniform(-12., 12., 6))
    yaw_offset = np.deg2rad(rng.uniform(-48., 48., 6))
    return np.c_[roll, pitch, yaw0+yaw_offset].tolist()


def design_batch_metadata(family_index: int, *, height_mm: int | None = None,
                          variant_index: int = 1, direction: str | None = None,
                          master_seed: int = MASTER_SEED) -> dict[str, Any]:
    """Design one new authority, without selecting duration or writing files.

    ``height_mm=None`` means a one-way full-height route.  Its variants 1..6
    alternate ascent/descent independently of family.  Fixed layers use one
    route per family, with seeds independent across all 84 layer/family pairs.
    The constructed seed slots are disjoint, rather than probabilistically
    relying on truncated hashes not to collide.
    """
    if not isinstance(master_seed, int) or master_seed < 0:
        raise ValueError("master_seed must be a nonnegative integer")
    if family_index not in range(len(FAMILIES)):
        raise ValueError("unknown open route family index")
    full_height = height_mm is None
    if full_height:
        if variant_index not in range(1, FULL_HEIGHT_VARIANTS+1):
            raise ValueError("full-height variants must be 1..6")
        intended_direction = "ascending_height" if variant_index % 2 else "descending_height"
        if direction is not None and direction != intended_direction:
            raise ValueError("direction must match the predeclared balanced variant schedule")
        direction = intended_direction
        seed = master_seed + 1000 + family_index*FULL_HEIGHT_VARIANTS + variant_index-1
    else:
        if height_mm not in HEIGHT_LAYERS_MM or variant_index != 1:
            raise ValueError("fixed routes require one of 14 height layers and variant 1")
        if direction not in (None, "constant"):
            raise ValueError("fixed-height route direction must be constant")
        direction = "constant"
        seed = master_seed + HEIGHT_LAYERS_MM.index(height_mm)*len(FAMILIES) + family_index
    # The preview helper derives its seed from base + family slot + kind slot.
    # Offset only that seed constructor; its approved priors are unchanged.
    helper_master_seed = seed - family_index - (1000 if full_height else 0)
    meta = preview.design_metadata(family_index, full_height, helper_master_seed)
    if meta["seed"] != seed:
        raise AssertionError("unexpected preview helper seed mapping")
    meta["master_seed"] = master_seed
    meta["generator_version"] = GENERATOR_VERSION
    meta["batch_id"] = DEFAULT_BATCH
    meta["variant_index"] = variant_index
    meta["height_layer_mm"] = height_mm
    meta["direction"] = direction
    curve = meta["curve"]
    selected = np.asarray(curve["position_control_points_m"], float)
    baseline = np.asarray(curve["baseline_control_points_m"], float)
    if full_height:
        heights = np.sort(-selected[:, 2])
        if direction == "descending_height":
            heights = heights[::-1]
        selected[:, 2] = -heights
        baseline[:, 2] = -np.linspace(heights[0], heights[-1], len(heights))
        meta["height_profile"] = {
            "mode": "full_height_one_way", "lower_m": .070, "upper_m": .200,
            "direction": direction, "depth_control_heights_m": heights.tolist(),
            "depth_positive_stretch_weights": np.abs(np.diff(heights)).tolist(),
            "depth_weight_units": "m of height per adjacent Bernstein control increment"}
        tag = "full_h070_200"
    else:
        assert height_mm is not None
        height = height_mm / 1000.
        selected[:, 2] = baseline[:, 2] = -height
        meta["height_profile"] = {"mode": "fixed", "lower_m": height, "upper_m": height,
                                  "direction": "constant"}
        tag = f"fixed_h{height_mm:03d}"
    curve["position_control_points_m"] = selected.tolist()
    curve["prototype_control_points_m"] = baseline.tolist()
    curve["baseline_control_points_m"] = baseline.tolist()
    curve["deformation_control_points_m"] = (selected-baseline).tolist()
    meta["orientation"]["mode"] = "smooth_multi_axis"
    meta["orientation"]["euler_xyz_control_points_rad"] = _multi_axis_orientation(meta["seed_streams"]["orientation"])
    meta["trajectory_id"] = f"cae2_{tag}_{FAMILIES[family_index]}_v{variant_index:02d}_s{seed}"
    meta["bank_role"] = "offline_open_elastic_development_bank; no train/test independence claimed"
    meta["preview_source_reused_or_mutated"] = False
    return meta


def design_records(master_seed: int = MASTER_SEED) -> list[dict[str, Any]]:
    """Return the complete predeclared 84+36 quota before writing any source."""
    records = [design_batch_metadata(i, height_mm=h, master_seed=master_seed)
               for h in HEIGHT_LAYERS_MM for i in range(len(FAMILIES))]
    records += [design_batch_metadata(i, variant_index=v, master_seed=master_seed)
                for i in range(len(FAMILIES)) for v in range(1, FULL_HEIGHT_VARIANTS+1)]
    if len({m["trajectory_id"] for m in records}) != 120 or len({m["seed"] for m in records}) != 120:
        raise AssertionError("bank IDs and authority seeds must each be 120 unique values")
    return records


def _environment_snapshot() -> tuple[dict[str, Any], list[tuple[Path, str]]]:
    environment = WORKSPACE/"environments/mla_short"
    entries = []
    files = {}
    for source in sorted(environment.rglob("*")):
        relative = source.relative_to(environment)
        if relative.parts[0] in ("trajectories", "recordings") or not source.is_file():
            continue
        if source.is_symlink():
            raise ValueError(f"environment authority must not be a symlink: {source}")
        name = relative.as_posix()
        files[name] = preview.digest(source.read_bytes())
        entries.append((source, f"environment_snapshot/{name}"))
    payload = json.dumps(files, sort_keys=True, separators=(",", ":")).encode("utf-8")
    lock = WORKSPACE/"curobo.lock"
    result = {"schema": "mla_short_environment_sha256_v1", "files": files,
              "sha256": preview.digest(payload), "lock_sha256": preview.digest(lock.read_bytes()),
              "lock_frozen_file": "provenance/curobo.lock",
              "snapshot_directory": "provenance/environment_snapshot",
              "excluded_top_level_directories": ["trajectories", "recordings"]}
    entries.append((lock, "curobo.lock"))
    return result, entries


def freeze_provenance(batch: Path) -> dict[str, Any]:
    """Freeze all used design code, source references, protocols and environment."""
    fingerprint, environment_sources = _environment_snapshot()
    refs = [(ROOT/name, f"generator_sources/{name}")
            for name in ("__init__.py", "core.py", "generate.py", "generate_batch.py")]
    refs += [(WORKSPACE/"generation/common.py", "generator_sources/common.py"),
             (REPO/"src/hx_waypoint_tools/hx_waypoint_tools/trajectory_generator.py", "reference_sources/trajectory_generator.py"),
             (REPO/"scripts/generate_new_board_esophageal_up50_trajectories.py", "reference_sources/generate_new_board_esophageal_up50_trajectories.py")]
    refs += [(WORKSPACE/"inputs"/name, f"protocol/{name}")
             for name in ("collection_task_v2_20261016.txt", "real_trajectory_collection_plan_20261013.md")]
    refs += environment_sources
    entries = []
    for source, relative in refs:
        payload = source.read_bytes()
        frozen = batch/"provenance"/relative
        preview._write_immutable(frozen, payload)
        entries.append({"origin_path": str(source), "frozen_file": frozen.relative_to(batch).as_posix(),
                        "sha256": preview.digest(payload)})
    result = {"sources": entries, "environment_fingerprint": fingerprint,
              "python": platform.python_version(), "numpy": np.__version__, "scipy": scipy.__version__}
    preview.write_json_immutable(batch/"provenance/source_inventory.json", result)
    return result


def _write_record(batch: Path, meta: dict[str, Any], inventory_sha256: str) -> dict[str, Any]:
    meta = preview.finalize_duration(meta)
    meta["provenance_inventory_file"] = "provenance/source_inventory.json"
    meta["provenance_inventory_sha256"] = inventory_sha256
    source = batch/"csv"/f"{meta['trajectory_id']}.csv"
    authority = preview.write_authority(source, *preview.sampled_authority(meta))
    meta["authority"] = {"csv_file": source.relative_to(batch).as_posix(),
                         "sha256": authority["source_sha256"], "source_pose_count": authority["source_pose_count"],
                         "duration_s": authority["duration_s"]}
    meta["source_grid_diagnostics"] = authority["diagnostics"]
    metadata_file = batch/"metadata"/f"{meta['trajectory_id']}.json"
    preview.write_json_immutable(metadata_file, meta)
    return {"trajectory_id": meta["trajectory_id"], "csv_file": source.relative_to(batch).as_posix(),
            "metadata_file": metadata_file.relative_to(batch).as_posix(),
            "sha256": authority["source_sha256"], "metadata_sha256": preview.digest(metadata_file.read_bytes()),
            "duration_s": authority["duration_s"], "draw_duration_s": meta["draw_duration_s"],
            "group_id": meta["group_id"], "family_id": meta["family_id"], "seed": meta["seed"],
            "height_profile": meta["height_profile"], "orientation_mode": meta["orientation"]["mode"],
            "variant_index": meta["variant_index"], "height_layer_mm": meta["height_layer_mm"],
            "direction": meta["direction"], "robot_simulation": "not_run", "runtime_ready": False}


def generate(batch_directory: Path | None = None, master_seed: int = MASTER_SEED) -> Path:
    preview._check_environment()
    batch = Path(batch_directory) if batch_directory else ROOT/"batches"/DEFAULT_BATCH
    if batch.is_symlink():
        raise ValueError("batch root cannot be a symlink")
    designs = design_records(master_seed)
    for meta in designs:
        meta["batch_id"] = batch.name
    provenance = freeze_provenance(batch)
    inventory_sha256 = preview.digest((batch/"provenance/source_inventory.json").read_bytes())
    records = [_write_record(batch, meta, inventory_sha256) for meta in designs]
    contract = {"closed_pattern_families_allowed": False, "repeated_laps_allowed": False,
                "fixed_height_count": 84, "full_height_one_way_count": 36,
                "fixed_height_layers_mm": list(HEIGHT_LAYERS_MM), "families": list(FAMILIES),
                "fixed_routes_per_layer": len(FAMILIES), "full_height_variants_per_family": FULL_HEIGHT_VARIANTS,
                "full_height_direction_counts": {"ascending_height": 18, "descending_height": 18},
                "all_orientation_mode": "smooth_multi_axis",
                "provisional_3mm_2deg_envelope_removed_by_user": True,
                "unchanged_hard_limits": copy.deepcopy(preview.LIMITS),
                "source_time_position_orientation_are_immutable_after_write": True,
                "space_speed_orientation_duration_seed_streams_are_independent": True,
                "discrete_V_W_quota_claimed": False}
    manifest = {"schema_version": 1, "batch_id": batch.name, "generator_version": GENERATOR_VERSION,
                "master_seed": master_seed, "trajectory_count": len(records), "records": records,
                "environment_id": "mla_short", "simulation_only": True, "real_execution_authorized": False,
                "authority_contract": "every written CSV row/pose/time_s is immutable; runtime scaling, retiming, clipping, skipping and automatic slowing are forbidden",
                "source_contract": contract, "total_authority_duration_s": sum(r["duration_s"] for r in records),
                "environment_fingerprint": provenance["environment_fingerprint"], "provenance": provenance,
                "full_robot_validation_status": "not_run; source authorities are not executable robot recordings"}
    preview.write_json_immutable(batch/"manifest.json", manifest)
    return batch/"manifest.json"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="new immutable batch directory")
    parser.add_argument("--seed", type=int, default=MASTER_SEED)
    args = parser.parse_args()
    print(generate(args.output, args.seed))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
