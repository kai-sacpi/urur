"""Independent, source-only audit of the 120-route augmented authority bank.

No generator, runtime, ROS, GPU or device imports are used.  Each authority is
checked by the immutable preview auditor; this adapter independently adds the
120-route quotas, seed independence, frozen-code and environment identity gates.
Passing this report is deliberately NOT a robot or executable-replay approval.
"""
from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import math
from pathlib import Path
import platform
import sys
from typing import Any

import numpy as np
import scipy

if __package__:
    from . import audit
else:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import audit

ROOT = Path(__file__).resolve().parent
FAMILIES = ("open_line", "gentle_s", "strong_s", "c_curve", "slanted_s", "tube_bend")
LAYERS_MM = tuple(range(70, 201, 10))
STREAM_NAMES = ("space", "speed", "orientation", "duration")


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def safe_path(root: Path, relative: str) -> Path:
    path = (root/relative).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError(f"path escapes authority batch: {relative}")
    return path


def canonical_digest(value: dict[str, str]) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def environment_identity(environment: Path) -> dict[str, Any]:
    files = {str(path.relative_to(environment)): audit.sha256(path)
             for path in sorted(environment.rglob("*"))
             if path.is_file() and path.relative_to(environment).parts[0]
             not in ("trajectories", "recordings")}
    lock = environment.parent.parent/"curobo.lock"
    return {"schema": "mla_short_environment_sha256_v1", "files": files,
            "sha256": canonical_digest(files), "lock_sha256": audit.sha256(lock),
            "lock_frozen_file": "provenance/curobo.lock", "snapshot_directory": "provenance/environment_snapshot",
            "excluded_top_level_directories": ["trajectories", "recordings"]}


def quota_check(records: list[dict[str, Any]], metadata: list[dict[str, Any]],
                preview_ids: set[str]) -> dict[str, Any]:
    """Check quotas against actual metadata, not manifest assertions alone."""
    errors: list[str] = []
    ids = [record.get("trajectory_id") for record in records]
    if len(records) != 120 or len(metadata) != 120:
        errors.append("bank must contain exactly 120 records and metadata documents")
    if len(ids) != len(set(ids)):
        errors.append("duplicate trajectory IDs")
    overlap = sorted(set(ids) & preview_ids)
    if overlap:
        errors.append("new IDs collide with immutable preview IDs")
    fixed_counts: Counter[tuple[int, str]] = Counter()
    full_counts: Counter[tuple[str, int]] = Counter()
    direction_counts: Counter[tuple[str, str]] = Counter()
    orientation_counts: Counter[str] = Counter()
    all_seeds: list[int] = []
    all_streams: list[int] = []
    for record, meta in zip(records, metadata):
        identifier = str(record.get("trajectory_id"))
        if meta.get("trajectory_id") != identifier:
            errors.append(f"{identifier}: metadata ID mismatch")
        family = meta.get("family_id")
        group = meta.get("group_id")
        profile = meta.get("height_profile", {})
        if family not in FAMILIES or family != record.get("family_id"):
            errors.append(f"{identifier}: unknown/inconsistent family")
        if group != record.get("group_id"):
            errors.append(f"{identifier}: group mismatch")
        if (meta.get("variant_index") != record.get("variant_index")
                or meta.get("height_layer_mm") != record.get("height_layer_mm")
                or meta.get("direction") != record.get("direction")):
            errors.append(f"{identifier}: variant/layer/direction metadata disagree with manifest")
        orientation = meta.get("orientation", {})
        if (record.get("height_profile") != profile
                or record.get("orientation_mode") != orientation.get("mode")
                or record.get("draw_duration_s") != meta.get("draw_duration_s")):
            errors.append(f"{identifier}: height/orientation/draw-duration labels disagree with metadata")
        orientation_counts[str(orientation.get("mode"))] += 1
        if orientation.get("mode") != "smooth_multi_axis":
            errors.append(f"{identifier}: all 120 routes must use smooth_multi_axis orientation")
        euler = np.asarray(orientation.get("euler_xyz_control_points_rad", []), float)
        if euler.ndim != 2 or euler.shape[1:] != (3,) or len(euler) < 2 or np.count_nonzero(np.ptp(euler, axis=0) > 1e-6) < 2:
            errors.append(f"{identifier}: multi-axis controls must vary on at least two axes")
        density = np.asarray(meta.get("time_warp", {}).get("density_control_points_positive", []), float)
        if len(density) < 2 or not np.all(np.isfinite(density)) or np.min(density) <= 0. or np.ptp(density) <= 1e-6:
            errors.append(f"{identifier}: random speed field must have nonconstant positive controls")
        try:
            seed = int(meta["seed"])
            streams = meta["seed_streams"]
            children = np.random.SeedSequence(seed).spawn(4)
            expected_streams = {name: int(child.generate_state(1, dtype=np.uint64)[0])
                                for name, child in zip(STREAM_NAMES, children, strict=True)}
            if streams != expected_streams:
                errors.append(f"{identifier}: seed streams do not reproduce independent SeedSequence children")
            if record.get("seed") != seed:
                errors.append(f"{identifier}: record seed mismatch")
            all_seeds.append(seed)
            all_streams.extend(int(streams[name]) for name in STREAM_NAMES)
        except (KeyError, TypeError, ValueError) as exc:
            errors.append(f"{identifier}: invalid seed/stream metadata: {exc}")
        try:
            lower = float(profile["lower_m"])
            upper = float(profile["upper_m"])
            if group == "fixed_height":
                layer = round(lower*1000)
                fixed_counts[(layer, str(family))] += 1
                if profile.get("mode") != "fixed" or abs(lower-upper) > 1e-12 or abs(lower-layer/1000.) > 1e-12:
                    errors.append(f"{identifier}: fixed-height contract mismatch")
                if record.get("height_layer_mm") != layer:
                    errors.append(f"{identifier}: manifest fixed-height layer mismatch")
            elif group == "full_height_one_way":
                variant = int(record["variant_index"])
                full_counts[(str(family), variant)] += 1
                direction = str(profile.get("direction"))
                direction_counts[(str(family), direction)] += 1
                if (profile.get("mode") != "full_height_one_way" or abs(lower-.07) > 1e-12
                        or abs(upper-.20) > 1e-12 or record.get("direction") != direction):
                    errors.append(f"{identifier}: full-height direction/range mismatch")
                controls = np.asarray(meta["curve"]["position_control_points_m"], float)
                actual_direction = "ascending_height" if -controls[-1, 2] > -controls[0, 2] else "descending_height"
                if actual_direction != direction:
                    errors.append(f"{identifier}: direction label differs from analytical endpoints")
            else:
                errors.append(f"{identifier}: unknown trajectory group")
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            errors.append(f"{identifier}: malformed height/variant metadata: {exc}")
    expected_fixed = Counter({(height, family): 1 for height in LAYERS_MM for family in FAMILIES})
    expected_full = Counter({(family, variant): 1 for family in FAMILIES for variant in range(1, 7)})
    expected_directions = Counter({(family, direction): 3 for family in FAMILIES
                                   for direction in ("ascending_height", "descending_height")})
    if fixed_counts != expected_fixed:
        errors.append("fixed-height quota must equal 14 layers (70..200 mm) × 6 families, once each")
    if full_counts != expected_full:
        errors.append("full-height quota must equal 6 families × variants 1..6, once each")
    if direction_counts != expected_directions:
        errors.append("every full-height family must have three ascending and three descending routes")
    if len(all_seeds) != len(set(all_seeds)) or len(all_streams) != len(set(all_streams)):
        errors.append("bank seeds and independent random streams must be globally unique")
    return {"passed": not errors, "errors": errors, "count": len(records),
            "group_counts": dict(Counter(meta.get("group_id") for meta in metadata)),
            "orientation_counts": dict(orientation_counts), "preview_ID_overlap": overlap,
            "independent_seed_count": len(set(all_seeds)), "independent_stream_count": len(set(all_streams)),
            "fixed_layers_mm": list(LAYERS_MM),
            "full_height_direction_counts": {f"{family}:{direction}": count for (family, direction), count in sorted(direction_counts.items())}}


def provenance_check(batch: Path, manifest: dict[str, Any], metadata: list[dict[str, Any]],
                     environment: Path) -> dict[str, Any]:
    errors: list[str] = []
    inventory_path = batch/"provenance/source_inventory.json"
    inventory = read_json(inventory_path)
    inventory_sha = audit.sha256(inventory_path)
    frozen = inventory.get("sources", [])
    seen: set[str] = set()
    for entry in frozen:
        relative = entry["frozen_file"]
        if relative in seen:
            errors.append(f"duplicate frozen provenance path: {relative}")
        seen.add(relative)
        path = safe_path(batch, relative)
        if not path.is_file() or audit.sha256(path) != entry["sha256"]:
            errors.append(f"frozen provenance SHA256 mismatch: {relative}")
    required = {"core.py", "generate.py", "generate_batch.py", "common.py"}
    actual_names = {Path(entry["frozen_file"]).name for entry in frozen
                    if "generator_sources" in Path(entry["frozen_file"]).parts}
    if not required.issubset(actual_names):
        errors.append("frozen source inventory is missing required generator/core/common code")
    for meta in metadata:
        if (meta.get("provenance_inventory_file") != "provenance/source_inventory.json"
                or meta.get("provenance_inventory_sha256") != inventory_sha):
            errors.append(f"{meta.get('trajectory_id')}: frozen provenance inventory binding mismatch")
    expected = manifest.get("environment_fingerprint", {})
    if inventory.get("environment_fingerprint") != expected:
        errors.append("frozen source inventory and manifest environment fingerprints disagree")
    if manifest.get("provenance") is not None and manifest["provenance"] != inventory:
        errors.append("manifest provenance differs from frozen source inventory")
    live = environment_identity(environment)
    if expected != live:
        errors.append("live mla_short environment/curobo.lock differs from frozen bank fingerprint")
    for relative, expected_sha in expected.get("files", {}).items():
        snapshot = safe_path(batch, f"provenance/environment_snapshot/{relative}")
        if not snapshot.is_file() or audit.sha256(snapshot) != expected_sha:
            errors.append(f"frozen environment snapshot differs from fingerprint: {relative}")
    lock_snapshot = batch/"provenance/curobo.lock"
    if not lock_snapshot.is_file() or audit.sha256(lock_snapshot) != expected.get("lock_sha256"):
        errors.append("frozen curobo.lock differs from fingerprint")
    return {"passed": not errors, "errors": errors, "inventory_sha256": inventory_sha,
            "frozen_file_count": len(frozen), "live_environment_fingerprint": live,
            "environment_file_count": len(live["files"])}


def _audit_one(arguments: tuple[Path, dict[str, Any], int]) -> dict[str, Any]:
    return audit.audit_record(*arguments)


def audit_bank(manifest_path: Path, *, samples: int = 32769, jobs: int = 1,
               environment: Path | None = None, preview_manifest: Path | None = None) -> dict[str, Any]:
    batch = manifest_path.resolve().parent
    manifest = read_json(manifest_path)
    records = manifest["records"]
    metadata = [read_json(safe_path(batch, record["metadata_file"])) for record in records]
    preview_manifest = preview_manifest or ROOT/"batches/ca_open_elastic_preview_20261016_v1/manifest.json"
    preview = read_json(preview_manifest)
    preview_ids = {record["trajectory_id"] for record in preview["records"]}
    quotas = quota_check(records, metadata, preview_ids)
    provenance = provenance_check(batch, manifest, metadata, environment or ROOT.parent/"environments/mla_short")
    arguments = [(batch, record, samples) for record in records]
    if jobs == 1:
        trajectories = [_audit_one(arg) for arg in arguments]
    else:
        with ProcessPoolExecutor(max_workers=jobs) as executor:
            trajectories = list(executor.map(_audit_one, arguments))
    top_errors = []
    if (manifest.get("environment_id") != "mla_short" or manifest.get("simulation_only") is not True
            or manifest.get("real_execution_authorized") is not False):
        top_errors.append("manifest must declare mla_short and simulation-only without real-hardware authorization")
    if manifest.get("trajectory_count") != len(records):
        top_errors.append("manifest trajectory_count disagrees with actual records")
    total = sum(float(record["duration_s"]) for record in records)
    if not math.isfinite(total) or abs(total-float(manifest.get("total_authority_duration_s", -1.))) > 1e-8:
        top_errors.append("manifest total duration differs from authoritative record durations")
    count_passed = sum(record["source_level_pass"] for record in trajectories)
    return {"schema_version": "continuous_augmented_batch120_source_audit_v1",
            "batch_id": manifest.get("batch_id"), "manifest_sha256": audit.sha256(manifest_path),
            "source_only": True, "not_robot_pass": True, "robot_simulation": "not_run",
            "runtime_ready": False, "real_execution_authorized": False,
            "scope": "Nominal rod_tip sources only. No robot IK/collision/joint dynamics/compile/mock/inspect/playback claim. Every authoritative CSV waypoint and time_s remains unchanged.",
            "validation_provenance": {"batch_audit_sha256": audit.sha256(Path(__file__).resolve()),
                                      "independent_math_audit_sha256": audit.sha256(Path(audit.__file__).resolve()),
                                      "python": platform.python_version(), "numpy": np.__version__, "scipy": scipy.__version__,
                                      "preview_manifest_sha256": audit.sha256(preview_manifest),
                                      "continuous_certificate_samples": samples,
                                      "independence": "No generator/core imports. Reused independent Bernstein reconstruction, SO3 matrix jets and rigorous continuous next-derivative enclosures; FD is a consistency diagnostic only."},
            "limits": audit.LIMITS, "count": len(trajectories), "passed": count_passed,
            "total_authority_duration_s": total, "batch_contract_errors": top_errors,
            "quotas": quotas, "provenance": provenance,
            "all_source_level_pass": (not top_errors and quotas["passed"] and provenance["passed"]
                                      and count_passed == len(trajectories)),
            "trajectories": trajectories}


def write_reports(report: dict[str, Any], output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    (output/"source_audit.json").write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)+"\n")
    lines = ["# Batch-120 independent source audit", "", f"Source rows passed: {report['passed']}/{report['count']}.",
             f"Overall source-only result: {'PASS' if report['all_source_level_pass'] else 'FAIL'}.", "",
             "NOT a robot pass: full IK, collisions, joint dynamics, GPU compile, mock recording, inspect and independent playback are not performed by this auditor.", "",
             "Authority files are read-only. No scaling, retiming, clipping, point omission, repair or automatic slowdown is performed.", "",
             f"Quota check: {'PASS' if report['quotas']['passed'] else 'FAIL'}; frozen provenance/environment: {'PASS' if report['provenance']['passed'] else 'FAIL'}.", ""]
    for section in (report["batch_contract_errors"], report["quotas"]["errors"], report["provenance"]["errors"]):
        lines.extend(f"- {error}" for error in section)
    lines += ["", "| Trajectory | Source | Errors |", "| --- | --- | --- |"]
    lines.extend(f"| {record['trajectory_id']} | {'PASS' if record['source_level_pass'] else 'FAIL'} | {'; '.join(record['errors'])} |"
                 for record in report["trajectories"])
    (output/"source_audit.md").write_text("\n".join(lines)+"\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--environment", type=Path)
    parser.add_argument("--preview-manifest", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--samples", type=int, default=32769)
    parser.add_argument("--jobs", type=int, default=1, choices=range(1, 9))
    args = parser.parse_args()
    report = audit_bank(args.manifest.resolve(), samples=args.samples, jobs=args.jobs,
                        environment=args.environment, preview_manifest=args.preview_manifest)
    write_reports(report, args.output or args.manifest.resolve().parent/"reports")
    print(f"batch-120 source audit: {report['passed']}/{report['count']}; overall {'PASS' if report['all_source_level_pass'] else 'FAIL'}; NOT a robot pass")
    return 0 if report["all_source_level_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
