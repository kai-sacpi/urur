#!/usr/bin/env python3
"""Verify actual bytes and export honest open-route replay readiness; no ROS use."""
from __future__ import annotations

import argparse
import csv
import importlib.util
import math
from pathlib import Path
from typing import Any

try:
    from .prepare import BATCH, ID_RE, encoded, environment_hashes, protected, read, safe_path, sha, validate_sources
except ImportError:
    from prepare import BATCH, ID_RE, encoded, environment_hashes, protected, read, safe_path, sha, validate_sources

PHASES = ("compile", "build", "inspect", "play")
COMPILE_FILES = {"source_tip_trajectory.csv", "joint_trajectory.csv", "compile_report.yaml"}
SEALED_FILES = COMPILE_FILES | {"manifest.yaml", "validation_report.yaml", "sha256sums.txt", "joint_frames.csv",
                               "sim_actual_joint.csv", "sim_actual_tip.csv", "sim_controller_state.csv"}
# Exact current mla_short EnvironmentBuilder reproducibility contract: its
# ROOT_CONFIG_FILES, ROBOT_CONFIG_FILES, ROBOT_DATA_FILES and the existing
# meshes/ + robot/assets/ trees. README.md is source-provenance-bound separately,
# but the runtime deliberately does not include documentation in this map.
# Never accept an arbitrary report-chosen subset of the environment.
REPRODUCIBLE_ENV_FILES = frozenset({"environment.yaml", "setup.yaml", "zones.yaml", "planner.yaml",
    "recording.yaml", "controllers.yaml", "robot/robot_model.yaml", "robot/calibrated_kinematics.yaml",
    "robot/asset_provenance.yaml", "robot/curobo_robot.yaml", "robot/collision_spheres.yaml",
    "robot/driver_kinematics.yaml", "robot/assets/ur5.urdf", "meshes/.gitkeep"})


def reproducible_environment_hashes(root: Path) -> dict[str, str]:
    return {relative: sha(root / "environments/mla_short" / relative) for relative in sorted(REPRODUCIBLE_ENV_FILES)}


def sealed_checker(root: Path) -> Any:
    spec = importlib.util.spec_from_file_location("existing_wm_seal_check", root / "ops/export_replay_index.py")
    if spec is None or spec.loader is None:
        raise ValueError("existing sealed-recording checker unavailable")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.check_seal


def rebase(root: Path, value: str, origin: str) -> Path:
    path = Path(value)
    if path.is_absolute():
        try:
            path = path.relative_to(Path(origin))
        except ValueError as error:
            raise ValueError(f"status artifact escaped declared workspace: {value}") from error
    return safe_path(root, path.as_posix())


def load_states(paths: list[Path], allowed_ids: set[str]) -> dict[str, tuple[dict[str, Any], str]]:
    states = {}
    for path in paths:
        data = read(path)
        origin = data.get("workspace", "")
        if not Path(origin).is_absolute() or Path(origin).name != "workstation_magtrack" or data.get("environment_id") != "mla_short":
            raise ValueError(f"wrong status workspace/environment: {path}")
        for name, state in data["trajectories"].items():
            if name not in allowed_ids or name in states:
                raise ValueError(f"nonauthority or duplicate cross-shard trajectory ID: {name}")
            states[name] = (state, origin)
    return states


def fingerprints(root: Path, stage: dict[str, Any], origin: str, required: set[Path]) -> dict[Path, str]:
    values = stage.get("artifact_hashes", {})
    if not values:
        raise ValueError("passed stage has no actual artifact fingerprints")
    checked = {}
    for name, expected in values.items():
        path = rebase(root, name, origin)
        if path in checked or not path.is_file() or path.stat().st_size == 0 or sha(path) != expected:
            raise ValueError(f"missing, duplicate or changed artifact bytes: {name}")
        checked[path] = expected
    if not required.issubset(checked):
        raise ValueError(f"missing required artifact fingerprints: {required - checked.keys()}")
    return checked


def source_identity(authority: dict[str, Any]) -> dict[str, Any]:
    return {"sha256": authority["source_sha256"], "duration_s": authority["duration_s"], "waypoint_count": authority["waypoint_count"]}


def verify_complete(root: Path, authority: dict[str, Any], state: dict[str, Any], origin: str, check_seal: Any) -> dict[str, Any]:
    import yaml

    if state.get("source") != source_identity(authority):
        raise ValueError("status source SHA/duration/count differs from authority")
    if not all(state.get(p, {}).get("status") == "passed" for p in PHASES):
        raise ValueError("not all four stages passed")
    compile_stage, build, inspect, play = (state[p] for p in PHASES)
    name, source_sha = authority["trajectory_id"], authority["source_sha256"]
    rid = build.get("recording_id", "")
    if not ID_RE.fullmatch(rid):
        raise ValueError("unsafe/missing recording ID")
    recording = root / "environments/mla_short/recordings" / rid
    for stage in (build, inspect):
        if stage.get("recording_id") != rid or rebase(root, stage.get("recording_path", ""), origin) != recording:
            raise ValueError("build/inspect disagree on actual sealed recording")
    seal = check_seal(recording, source_sha)
    if seal.get("status") != "passed":
        raise ValueError(f"sealed checksum verification failed: {seal}")
    compiled = rebase(root, compile_stage.get("compiled_path", ""), origin)
    if compiled.parent != root / "runs":
        raise ValueError("compiled artifacts must be direct children of controlled runs directory")
    chashes = fingerprints(root, compile_stage, origin, {compiled / f for f in COMPILE_FILES})
    required_sealed = {recording / f for f in SEALED_FILES}
    for stage in (build, inspect):
        fingerprints(root, stage, origin, required_sealed)
    report = yaml.safe_load((compiled / "compile_report.yaml").read_text())
    required_report = {"schema_version": 2, "compiler_contract_revision": "fixed_time_waypoint_v2",
                       "environment_id": "mla_short", "trajectory_id": name, "source_trajectory_sha256": source_sha,
                       "uniform_time_scale": 1.0, "source_waypoint_count": authority["waypoint_count"],
                       "matched_source_waypoint_count": authority["waypoint_count"], "source_duration_s": authority["duration_s"],
                       "duration_s": authority["duration_s"], "max_source_time_error_s": 0.0}
    if report != compile_stage.get("report") or any(report.get(k) != v for k, v in required_report.items()):
        raise ValueError("compile report/source/fixed-time contract differs from status or authority")
    if report.get("environment_hashes") != reproducible_environment_hashes(root):
        raise ValueError("compile report belongs to different environment bytes")
    for filename in COMPILE_FILES:
        if sha(recording / filename) != chashes[compiled / filename]:
            raise ValueError(f"sealed recording is not from proved compile: {filename}")
    if sha(compiled / "source_tip_trajectory.csv") != source_sha or sha(compiled / "joint_trajectory.csv") != report.get("joint_trajectory_sha256"):
        raise ValueError("compile artifact hashes differ from source or report")
    planner = yaml.safe_load((root / "environments/mla_short/planner.yaml").read_text())["validation"]
    checks = [("minimum_collision_distance_m", .005, True),
              ("max_tip_position_error_m", float(planner["tip_position_tolerance_m"]), False),
              ("max_tip_orientation_error_deg", float(planner["tip_orientation_tolerance_deg"]), False)]
    checks += [(key, 1., False) for key in ("max_joint_velocity_ratio", "max_joint_acceleration_ratio", "max_joint_jerk_ratio")]
    for key, limit, minimum in checks:
        value = report.get(key)
        if (not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0
                or (minimum and value < limit) or (not minimum and value > limit)):
            raise ValueError(f"compile original hard collision/FK/dynamics gate failed: {key}")
    manifest = yaml.safe_load((recording / "manifest.yaml").read_text())
    validation = yaml.safe_load((recording / "validation_report.yaml").read_text())
    if (manifest.get("recording_id") != rid or manifest.get("trajectory_id") != name or manifest.get("environment_id") != "mla_short"
            or manifest.get("status") != "validated" or validation.get("simulation_completed") is not True
            or validation.get("controller_result") != "succeeded"):
        raise ValueError("sealed identity or mock validation is incomplete")
    for stage, domain in ((build, 219), (play, 220)):
        runtime = stage.get("runtime", {})
        if (stage.get("action_status") != 4 or stage.get("success") is not True or runtime.get("domain") != domain
                or runtime.get("execution_mode") != "simulation" or rebase(root, runtime.get("telemetry_root", ""), origin) != root / "runs"
                or rebase(root, runtime.get("environments_root", ""), origin) != root / "environments"):
            raise ValueError("successful isolated mock action/runtime identity missing")
    audit = play.get("playback_audit", {})
    witness = audit.get("controller_witness", {})
    if (audit.get("status") != "passed" or audit.get("canonical_recording_inspector_passed") is not True
            or audit.get("authoritative_duration_s") != authority["duration_s"] or audit.get("source_waypoint_count") != authority["waypoint_count"]
            or witness.get("status") != "passed" or witness.get("source_timing_changed") is not False
            or witness.get("resampling_or_synthetic_rows") is not False):
        raise ValueError("independent coherent controller witness missing/failed")
    extras = {}
    for key in ("controller_witness_path", "telemetry_path", "playback_audit_path"):
        if not play.get(key):
            raise ValueError(f"missing independent playback {key}")
        extras[key] = rebase(root, play[key], origin)
        if not extras[key].is_relative_to(root / "runs"):
            raise ValueError("independent playback evidence escaped controlled runs")
    fingerprints(root, play, origin, required_sealed | set(extras.values()))
    if read(extras["playback_audit_path"]) != audit:
        raise ValueError("actual audit artifact differs from status audit")
    metrics = {"max_independent_fk_position_error_m": witness.get("max_coherent_actual_source_position_error_m"),
               "max_independent_fk_orientation_error_deg": witness.get("max_coherent_actual_source_orientation_error_deg")}
    for key, limit in (("max_independent_fk_position_error_m", planner["tip_position_tolerance_m"]),
                       ("max_independent_fk_orientation_error_deg", planner["tip_orientation_tolerance_deg"])):
        value = metrics[key]
        if not isinstance(value, (float, int)) or not math.isfinite(value) or value < 0 or value > float(limit) + 1e-9:
            raise ValueError(f"coherent FK gate failed: {key}")
    return {"recording_id": rid, "recording_directory": recording.relative_to(root).as_posix(),
            "sealed_checksum_status": "passed", "checked_sealed_files": seal["checked_sealed_files"],
            "artifact_fingerprints_verified": True, "coherent_controller_witness": "passed",
            "minimum_collision_distance_m": report["minimum_collision_distance_m"],
            "max_tip_position_error_m": report["max_tip_position_error_m"],
            "max_tip_orientation_error_deg": report["max_tip_orientation_error_deg"], **metrics}


def export(root: Path, status_paths: list[Path]) -> dict[str, Any]:
    root = root.resolve()
    planning = root / "continuous_augmented/replay/planning"
    index = read(planning / "execution_index.json")
    if index != validate_sources(root) or read(planning / "protected_inputs.json").get("sha256") != protected(root):
        raise ValueError("frozen index/protected inputs differ from source authority or current environment")
    states = load_states(status_paths, {row["trajectory_id"] for row in index["trajectories"]})
    check_seal = sealed_checker(root)
    rows = []
    for authority in index["trajectories"]:
        if sha(safe_path(root, authority["source_file"])) != authority["source_sha256"]:
            raise ValueError(f"staged source changed: {authority['trajectory_id']}")
        state, origin = states.get(authority["trajectory_id"], ({}, str(root)))
        if state and state.get("source") != source_identity(authority):
            raise ValueError(f"state belongs to different source: {authority['trajectory_id']}")
        phases = {phase: state.get(phase, {}).get("status", "pending") for phase in PHASES}
        row = {k: authority[k] for k in ("trajectory_id", "source_file", "source_sha256", "duration_s", "group_id", "family_id", "seed", "orientation_mode")}
        row.update(**phases, ready_for_simulation_replay_on_this_host=False, recording_id=state.get("build", {}).get("recording_id", ""),
                   recording_directory="", sealed_checksum_status="not_verified", artifact_fingerprints_verified=False,
                   coherent_controller_witness="pending", evidence_error="", last_error=state.get("last_error", ""), real_execution_authorized=False)
        if all(value == "passed" for value in phases.values()):
            try:
                row.update(verify_complete(root, authority, state, origin, check_seal))
                row["ready_for_simulation_replay_on_this_host"] = True
            except (OSError, ValueError, KeyError, TypeError) as error:
                row.update(evidence_error=str(error), sealed_checksum_status="failed_or_unavailable")
        rows.append(row)
    result = {"schema_version": 1, "source_batch_id": BATCH, "environment_id": "mla_short", "simulation_only": True,
              "real_execution_authorized": False, "trajectory_count": len(rows),
              "ready_on_this_host": sum(row["ready_for_simulation_replay_on_this_host"] for row in rows),
              "execution_index_sha256": sha(planning / "execution_index.json"),
              "status_inputs": [str(path.resolve()) for path in status_paths], "recordings": rows}
    reports = root / "continuous_augmented/batches" / BATCH / "reports"
    reports.mkdir(parents=True, exist_ok=True)
    (reports / "replay_index.json").write_bytes(encoded(result))
    with (reports / "replay_index.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, list(dict.fromkeys(k for row in rows for k in row)))
        writer.writeheader()
        writer.writerows(rows)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--status", type=Path, action="append", default=[])
    parser.add_argument("--require-all", action="store_true")
    args = parser.parse_args()
    result = export(args.workspace, args.status)
    print({key: value for key, value in result.items() if key != "recordings"})
    return 0 if not args.require_all or result["ready_on_this_host"] == result["trajectory_count"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
