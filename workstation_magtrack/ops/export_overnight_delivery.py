#!/usr/bin/env python3
"""Audit the exact 336-route bank and publish one immutable simulation handoff.

This is a read-only evidence audit until every original/augmented replay and
both independent parking prepare directions pass. It never launches a replay,
ROS node, GPU job, robot action, or serial capture. --wait observes already-live
processes; neither a lock nor a stale state file counts as a live process.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import csv
import ctypes
import errno
import hashlib
import importlib
import io
import json
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import time
from typing import Any, Callable

CODE_ROOT = Path(__file__).resolve().parents[1]
PHASES = ("compile", "build", "inspect", "play")
DELIVERY_ID = "overnight_delivery_20261017_v1"
PARKING_VALIDATION = "runs/parking_gpu_validation_20261017_v2/validation.json"
PARKING_MOCK = "runs/parking_mock_validation_20261017_v2/evidence.json"
PARKING_BUNDLE = "collection/prepared_parking/parking_20261017_v2"
ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*\Z")


@dataclass(frozen=True)
class Campaign:
    source_batch_id: str
    run_batch_id: str
    count: int
    authority_index: str
    final_inventory: str
    module: str


CAMPAIGNS = (
    Campaign("authority_v4", "authority_v4", 96, "planning/authority_index_v4.json",
             "reports/replay_index.json", "continuous_augmented.replay"),
    Campaign("ca_open_elastic_batch120_20261016_v2", "ca_batch120_20261016_v2", 120,
             "continuous_augmented/replay/planning/execution_index.json",
             "continuous_augmented/batches/ca_open_elastic_batch120_20261016_v2/reports/replay_index.json",
             "continuous_augmented.replay"),
    Campaign("ca_open_elastic_batch120_20261016_v3", "ca_batch120_20261016_v3", 120,
             "continuous_augmented/replay_v3/planning/execution_index.json",
             "continuous_augmented/batches/ca_open_elastic_batch120_20261016_v3/reports/replay_index.json",
             "continuous_augmented.replay_v3"),
)


def encoded(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, ensure_ascii=False, indent=2,
                       allow_nan=False) + "\n").encode()


def read(path: Path) -> Any:
    return json.loads(path.read_text())


def sha(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def contained(root: Path, relative: str) -> Path:
    path = root / relative
    if (not relative or Path(relative).is_absolute() or ".." in Path(relative).parts
            or not path.resolve().is_relative_to(root.resolve())):
        raise ValueError(f"unsafe workspace-relative path: {relative}")
    return path


def exact_count(value: Any, expected: int) -> bool:
    return type(value) is int and value == expected


def verify_campaign_summary(summary: dict[str, Any], batch_id: str, expected_count: int) -> None:
    """Check full terminal workflow, not a mid-phase status or ready counter."""
    if summary.get("workflow_passed") is not True:
        raise ValueError(f"campaign has not passed its complete workflow: {batch_id}")
    counts = summary.get("phase_counts", {})
    if set(counts) != set(PHASES) or any(
            set(counts[p]) != {"passed"} or not exact_count(counts[p]["passed"], expected_count)
            for p in PHASES):
        raise ValueError(f"campaign phase counts are not exact: {batch_id}")
    if batch_id == "authority_v4":
        if (summary.get("batch") != batch_id or summary.get("state") != "finished"
                or not exact_count(summary.get("trajectory_count"), expected_count)
                or not exact_count(summary.get("complete_passed"), expected_count)
                or summary.get("real_robot_used") is not False or summary.get("sensor_used") is not False):
            raise ValueError("original 96-route terminal identity/count/simulation mismatch")
        exits = summary.get("phase_exit_codes", {})
        required = {"build", "inspect", "play", "inventory", "independent_source_audit"}
        if not required.issubset(exits) or any(not exact_count(code, 0) for code in exits.values()):
            raise ValueError("original campaign has missing/failing phase or independent postcheck")
    else:
        if (batch_id not in {c.source_batch_id for c in CAMPAIGNS[1:]}
                or summary.get("source_batch_id") != batch_id or summary.get("state") != "complete"
                or any(not exact_count(summary.get(key), expected_count) for key in
                       ("expected_trajectory_count", "observed_trajectory_count", "complete_four_stage_passed", "ready_on_this_host"))
                or summary.get("robot_used") is not False or summary.get("sensor_used") is not False
                or summary.get("simulation_only") is not True or summary.get("real_execution_authorized") is not False):
            raise ValueError("augmented campaign terminal identity/count/simulation mismatch")
        attempts = summary.get("command_attempts", [])
        required_pairs = {(label, phase) for label in ("canary", "remaining117") for phase in PHASES}
        observed = [(item.get("label"), item.get("phase")) for item in attempts]
        if (len(attempts) != 8 or set(observed) != required_pairs or len(set(observed)) != 8
                or any(not exact_count(item.get("exit_code"), 0) for item in attempts)):
            raise ValueError("augmented campaign requires all eight exact successful child stages")


def _rename_directory_exclusive(source: Path, destination: Path) -> None:
    """Native atomic rename that cannot replace even a racing empty directory."""
    libc = ctypes.CDLL(None, use_errno=True)
    if sys.platform.startswith("linux") and hasattr(libc, "renameat2"):
        rename = libc.renameat2
        rename.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
        rename.restype = ctypes.c_int
        result = rename(-100, os.fsencode(source), -100, os.fsencode(destination), 1)
    elif sys.platform == "darwin" and hasattr(libc, "renamex_np"):
        rename = libc.renamex_np
        rename.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint]
        rename.restype = ctypes.c_int
        result = rename(os.fsencode(source), os.fsencode(destination), 4)
    else:
        raise OSError(errno.ENOSYS, "atomic exclusive directory publication is unavailable")
    if result != 0:
        code = ctypes.get_errno()
        raise OSError(code, os.strerror(code), str(destination))


def immutable_bundle(destination: Path, outputs: dict[str, bytes]) -> None:
    """Publish all files atomically; an exact existing bundle is idempotent.

    Differing, incomplete, extra, or symlinked existing content is never repaired
    or overwritten. A temporary sibling directory cannot be mistaken for final.
    """
    if not outputs:
        raise ValueError("empty delivery output")
    names: set[str] = set()
    for name, data in outputs.items():
        if (not isinstance(name, str) or name in {"", "."} or Path(name).is_absolute()
                or ".." in Path(name).parts or Path(name).as_posix() != name
                or not isinstance(data, bytes)):
            raise ValueError(f"invalid immutable output name/bytes: {name!r}")
        names.add(name)
    if any(parent.as_posix() in names for name in names for parent in Path(name).parents if parent != Path(".")):
        raise ValueError("output file conflicts with a parent directory")
    if destination.exists() or destination.is_symlink():
        if not destination.is_dir() or destination.is_symlink():
            raise ValueError("immutable destination is not a normal directory")
        paths = list(destination.rglob("*"))
        if any(path.is_symlink() for path in paths):
            raise ValueError("immutable destination contains a symlink")
        actual = {p.relative_to(destination).as_posix() for p in paths if p.is_file()}
        expected_dirs = {p.as_posix() for name in names for p in Path(name).parents if p != Path(".")}
        actual_dirs = {p.relative_to(destination).as_posix() for p in paths if p.is_dir()}
        if (actual != names or actual_dirs != expected_dirs
                or any((destination / name).read_bytes() != data for name, data in outputs.items())):
            raise ValueError(f"immutable delivery differs; refusing overwrite: {destination}")
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=f".{destination.name}.staging-", dir=destination.parent))
    try:
        for name, data in outputs.items():
            path = temporary / name
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("xb") as stream:
                stream.write(data)
                stream.flush()
                os.fsync(stream.fileno())
        # Exclusive native rename never replaces even a racing empty directory.
        # A concurrent winner is accepted only when its complete bytes are exact.
        if destination.exists() or destination.is_symlink():
            immutable_bundle(destination, outputs)
        else:
            try:
                _rename_directory_exclusive(temporary, destination)
            except OSError:
                if destination.exists() or destination.is_symlink():
                    immutable_bundle(destination, outputs)
                else:
                    raise
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)


def module(name: str) -> Any:
    # Code is imported from this checkout; data are read only from --workspace.
    if str(CODE_ROOT) not in sys.path:
        sys.path.insert(0, str(CODE_ROOT))
    return importlib.import_module(name)


def python_invocation(tokens: list[str]) -> tuple[str, list[str]] | None:
    """Extract only the actual Python script/module, never nested command text."""
    if not tokens or not re.fullmatch(r"python(?:\d+(?:\.\d+)*)?", Path(tokens[0]).name):
        return None
    i = 1
    while i < len(tokens):
        token = tokens[i]
        if token in {"-c", "-", "--help", "-h", "--version", "-V"}:
            return None
        if token == "-m":
            return (tokens[i + 1], tokens[i + 2:]) if i + 1 < len(tokens) else None
        if token == "--":
            return (tokens[i + 1], tokens[i + 2:]) if i + 1 < len(tokens) else None
        if token in {"-W", "-X", "--check-hash-based-pycs"}:
            i += 2
            continue
        if token.startswith(("-W", "-X", "--check-hash-based-pycs=")):
            i += 1
            continue
        if token.startswith("-"):
            if not re.fullmatch(r"-[uBEsSIPqOvdbrxR]+", token):
                return None
            i += 1
            continue
        return token, tokens[i + 1:]
    return None


def command_option(arguments: list[str], option: str) -> str | None:
    """Read a single CLI option; ambiguous duplicate options do not prove identity."""
    values = []
    for i, argument in enumerate(arguments):
        if argument == option and i + 1 < len(arguments):
            values.append(arguments[i + 1])
        elif argument.startswith(option + "="):
            values.append(argument.partition("=")[2])
    return values[0] if len(values) == 1 else None


def verified_live_pids(root: Path, target_batch_ids: set[str],
                       process_runner: Callable[..., Any] = subprocess.run, *,
                       parking_producers: dict[str, Path] | None = None) -> list[int]:
    """Observe only actual Python producers of the currently missing prerequisites.

    In particular a tmux server can retain its original launch command in argv
    for hours after the child exits. Such a server, shell, `python -c`, or other
    nested command is not evidence that the claimed producer is alive.
    """
    if parking_producers is None:
        parking_producers = {"gpu": root / PARKING_VALIDATION, "mock": root / PARKING_MOCK,
                             "bundle": root / PARKING_BUNDLE}
    result = process_runner(["ps", "-eo", "pid=,stat=,args="], check=True,
                            capture_output=True, text=True)
    found: set[int] = set()
    for line in result.stdout.splitlines():
        fields = line.strip().split(None, 2)
        if len(fields) != 3 or not fields[0].isdigit() or "Z" in fields[1]:
            continue
        pid, _, command = fields
        if int(pid) == os.getpid():
            continue
        try:
            tokens = shlex.split(command)
        except ValueError:
            continue
        invocation = python_invocation(tokens)
        if invocation is None:
            continue
        script, arguments = invocation
        if any(argument in {"--help", "-h", "--version"} for argument in arguments):
            continue
        path = Path(script)
        script_rooted = path.is_absolute() and path.is_relative_to(root)
        workspace = command_option(arguments, "--workspace")
        if workspace is None and any(t == "--workspace" or t.startswith("--workspace=") for t in arguments):
            continue
        if workspace is not None and workspace != str(root):
            continue
        if not script_rooted and workspace != str(root):
            continue
        relative = path.relative_to(root).as_posix() if script_rooted else script
        campaigns = {"continuous_augmented/replay/run_campaign.py", "continuous_augmented/replay_v3/run_campaign.py",
                     "runtime/batch_recordings.py", "continuous_augmented.replay.run_campaign",
                     "continuous_augmented.replay_v3.run_campaign"}
        relevant = relative in campaigns and command_option(arguments, "--batch") in target_batch_ids
        if relative == "ops/finish_campaign.py":
            relevant = "authority_v4" in target_batch_ids
        kinds = {"collection/validate_parking.py": "gpu", "collection.validate_parking": "gpu",
                 "collection/mock_parking.py": "mock", "collection.mock_parking": "mock",
                 "collection/prepared_parking.py": "bundle", "collection.prepared_parking": "bundle"}
        kind = kinds.get(relative)
        parking = False
        if kind in parking_producers:
            output = command_option(arguments, "--output")
            expected = parking_producers[kind] if kind == "bundle" else parking_producers[kind].parent
            parking = output == str(expected) and (kind != "bundle" or arguments[:1] == ["create"])
            if (kind == "gpu" and "--gpu" not in arguments) or (kind == "mock" and "--verify-only" in arguments):
                parking = False
        if relevant or parking:
            found.add(int(pid))
    return sorted(found)


def probe(root: Path, parking_validation: Path, parking_mock: Path, parking_bundle: Path) -> dict[str, Any]:
    """Cheap truthful progress, without hashing gigabytes during active play."""
    rows, terminal_failure = [], False
    for campaign in CAMPAIGNS:
        path = root / "runs" / campaign.run_batch_id / "campaign_summary.json"
        item: dict[str, Any] = {"batch_id": campaign.source_batch_id, "expected": campaign.count,
                               "accepted_terminal_workflow": False}
        if path.is_file():
            value = read(path)
            item.update(state=value.get("state"), phase_counts=value.get("phase_counts"))
            try:
                verify_campaign_summary(value, campaign.source_batch_id, campaign.count)
                item["accepted_terminal_workflow"] = True
            except ValueError as error:
                item["reason"] = str(error)
                if value.get("state") in {"failed", "interrupted", "finished", "complete"}:
                    terminal_failure = True
        else:
            item.update(state="missing", reason="campaign summary unavailable")
        rows.append(item)
    parking = {"gpu_validation": "missing", "independent_mock": "missing",
               "prepared_bundle": "present" if (parking_bundle / "bundle.json").is_file() else "missing"}
    for key, path in (("gpu_validation", parking_validation), ("independent_mock", parking_mock)):
        if path.is_file():
            parking[key] = read(path).get("status", "unknown")
            terminal_failure |= parking[key] in {"failed", "rejected"}
    prerequisite_ready = (all(r["accepted_terminal_workflow"] for r in rows)
                          and parking["gpu_validation"] == "passed" and parking["independent_mock"] == "passed"
                          and parking["prepared_bundle"] == "present")
    return {"accepted": False, "prerequisites_ready_for_full_byte_audit": prerequisite_ready,
            "terminal_failure": terminal_failure, "campaigns": rows, "parking": parking}


def unique_rows(rows: list[dict[str, Any]], expected_count: int, label: str) -> dict[str, dict[str, Any]]:
    values: dict[str, dict[str, Any]] = {}
    for row in rows:
        name = row.get("trajectory_id", "")
        if not ID.fullmatch(name) or name in values:
            raise ValueError(f"duplicate/unsafe trajectory ID in {label}: {name}")
        values[name] = row
    if len(values) != expected_count:
        raise ValueError(f"incorrect exact count in {label}: {len(values)} != {expected_count}")
    return values


def verify_campaign(root: Path, campaign: Campaign) -> tuple[list[dict[str, Any]], dict[str, str]]:
    summary_path = root / "runs" / campaign.run_batch_id / "campaign_summary.json"
    state_path = summary_path.with_name("status.json")
    index_path, inventory_path = root / campaign.authority_index, root / campaign.final_inventory
    inputs = (summary_path, state_path, index_path, inventory_path)
    provenance = {path.relative_to(root).as_posix(): sha(path) for path in inputs}
    summary, state, index, inventory = (read(path) for path in (summary_path, state_path, index_path, inventory_path))
    verify_campaign_summary(summary, campaign.source_batch_id, campaign.count)
    authorities = unique_rows(index["trajectories"], campaign.count, "authority index")
    final = unique_rows(inventory["recordings"], campaign.count, "final replay inventory")
    if (set(state.get("trajectories", {})) != set(authorities) or set(final) != set(authorities)
            or index.get("environment_id") != "mla_short" or inventory.get("environment_id") != "mla_short"
            or not exact_count(index.get("trajectory_count"), campaign.count)
            or not exact_count(inventory.get("trajectory_count"), campaign.count)
            or not exact_count(inventory.get("ready_on_this_host"), campaign.count)
            or inventory.get("simulation_only") is not True):
        raise ValueError(f"source/state/final-inventory exact join mismatch: {campaign.source_batch_id}")
    verifier = module(campaign.module + ".export_index")
    if campaign.source_batch_id != "authority_v4":
        preparation = module(campaign.module + ".prepare")
        if (index != preparation.validate_sources(root)
                or read(index_path.with_name("protected_inputs.json")).get("sha256") != preparation.protected(root)
                or inventory.get("source_batch_id") != campaign.source_batch_id
                or inventory.get("execution_index_sha256") != sha(index_path)):
            raise ValueError("augmented source audit/frozen environment/final inventory provenance differs")
    origin = state.get("workspace", "")
    if not Path(origin).is_absolute() or Path(origin).name != "workstation_magtrack" or state.get("environment_id") != "mla_short":
        raise ValueError("invalid state workspace/environment")
    check_seal, rows = verifier.sealed_checker(root), []
    for name, authority in authorities.items():
        authority = dict(authority)
        if campaign.source_batch_id == "authority_v4":
            authority["waypoint_count"] = authority["source_pose_count"]
        source = contained(root, authority["source_file"])
        if sha(source) != authority["source_sha256"]:
            raise ValueError(f"authoritative source bytes changed: {name}")
        final_row, record_state = final[name], state["trajectories"][name]
        proof = verifier.verify_complete(root, authority, record_state, origin, check_seal)
        for key in ("source_file", "source_sha256", "duration_s"):
            if final_row.get(key) != authority[key]:
                raise ValueError(f"final inventory {key} differs from source authority: {name}")
        witness_values = [final_row[key] for key in ("coherent_controller_witness", "coherent_playback_witness") if key in final_row]
        if (any(final_row.get(phase) != "passed" for phase in PHASES)
                or final_row.get("ready_for_simulation_replay_on_this_host") is not True
                or not witness_values or any(value != "passed" for value in witness_values)
                or final_row.get("recording_id") != proof["recording_id"]
                or final_row.get("recording_directory") != proof["recording_directory"]
                or final_row.get("real_execution_authorized") is not False):
            raise ValueError(f"final inventory readiness disagrees with independently checked evidence: {name}")
        rows.append({"trajectory_id": name, "source_batch_id": campaign.source_batch_id,
                     "category": authority.get("category", ""), "source_file": authority["source_file"],
                     "source_sha256": authority["source_sha256"], "duration_s": authority["duration_s"],
                     "waypoint_count": authority["waypoint_count"], **{phase: "passed" for phase in PHASES},
                     **proof, "real_execution_authorized": False})
    if any(sha(root / name) != digest for name, digest in provenance.items()):
        raise ValueError("campaign evidence inputs changed during full-byte audit")
    return rows, provenance


def verify_parking(root: Path, validation_path: Path, mock_path: Path, bundle_path: Path) -> dict[str, Any]:
    """Reuse independent strong prepare and raw mock verifiers, not status flags."""
    prepared = module("collection.prepared_parking")
    mock = module("collection.mock_parking")
    bundle = prepared.PreparedParkingBundle(root, bundle_path)
    bundle.verify()
    validation, config = read(validation_path), module("collection.manifest").config_at(root / "collection/config.yaml")
    if (validation.get("status") != "passed" or validation.get("gpu_full_path_validation") != "passed_both_directions"
            or validation.get("runtime_ready") is not True or validation.get("real_execution_authorized") is not False
            or validation.get("simulation_only") is not True or validation.get("authority_mutated") is not False
            or validation.get("uniform_time_scale") != 1.0
            or validation.get("hard_world_self_clearance_m") != .005 or validation.get("rod_length_m") != .190
            or validation.get("rod_diameter_m") != .005 or validation.get("config_sha256") != sha(root / "collection/config.yaml")
            or validation.get("parking_joint_positions_deg") != config["parking"]["joint_positions_deg"]
            or set(validation.get("directions", {})) != {"home_to_park", "park_to_home"}):
        raise ValueError("parking full-path proof/config/immutable nominal contract differs")
    source_validation = bundle.manifest["source_validation"]
    if source_validation.get("sha256") != sha(validation_path):
        raise ValueError("prepared parking bundle belongs to another GPU validation")
    for direction, item in validation["directions"].items():
        path = Path(item["joint_trajectory_path"])
        if not path.is_absolute():
            path = validation_path.parent / direction / path.name
        if not path.resolve().is_relative_to(root / "runs"):
            raise ValueError("GPU parking source path escaped controlled runs")
        if (item.get("status") != "passed" or sha(path) != item.get("joint_trajectory_sha256")
                or bundle.manifest["directions"][direction].get("joint_trajectory_sha256") != sha(path)):
            raise ValueError("parking prepared CSV differs from proved GPU CSV")
    checked = mock.verify_mock_evidence(root, mock_path, bundle_path)
    if checked.get("status") != "passed":
        raise ValueError("parking independent mock raw-evidence verifier did not pass")
    return {"asset_type": "collision_checked_prepare_joint_bundle", "tipdraw_recording": False,
            "gpu_validation_file": validation_path.relative_to(root).as_posix(),
            "gpu_validation_sha256": sha(validation_path),
            "independent_mock_evidence_file": mock_path.relative_to(root).as_posix(),
            "independent_mock_evidence_sha256": sha(mock_path),
            "prepared_bundle": bundle_path.relative_to(root).as_posix(),
            "prepared_bundle_manifest_sha256": bundle.manifest_sha256,
            "directions": bundle.manifest["directions"], "independent_mock_status": "passed",
            "real_execution_authorized": False,
            "magnetic_interference_status": "operator_expected_low_interference_not_measured"}


def csv_bytes(rows: list[dict[str, Any]]) -> bytes:
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=list(dict.fromkeys(key for row in rows for key in row)))
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue().encode()


def export(root: Path, destination: Path, validation_path: Path, mock_path: Path, bundle_path: Path) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    provenance: dict[str, str] = {}
    for campaign in CAMPAIGNS:
        batch_rows, fingerprints = verify_campaign(root, campaign)
        rows.extend(batch_rows)
        provenance.update(fingerprints)
    unique_rows(rows, 336, "all three accepted banks")
    # Commissioning and Pilot intentionally share three original source hashes.
    # Enforce no duplicate source across enhanced banks or against original;
    # do not incorrectly reject the protocol's original replication design.
    original = {r["source_sha256"] for r in rows if r["source_batch_id"] == "authority_v4"}
    enhanced = [r["source_sha256"] for r in rows if r["source_batch_id"] != "authority_v4"]
    if len(set(enhanced)) != 240 or set(enhanced) & original:
        raise ValueError("augmented sources overlap a formal source bank")
    parking = verify_parking(root, validation_path, mock_path, bundle_path)
    collection = module("collection.manifest")
    indexes = [root / campaign.final_inventory for campaign in CAMPAIGNS]
    manifest = collection.create(root, indexes, root / "collection/config.yaml")
    collection.verify(root, manifest)
    if [r["trajectory_id"] for r in manifest["recordings"]] != [r["trajectory_id"] for r in rows]:
        raise ValueError("collection manifest does not preserve all 336 authority IDs in order")
    if any(sha(root / name) != digest for name, digest in provenance.items()):
        raise ValueError("campaign evidence inputs changed before handoff publication")
    manifest["parking_prepare_bundle"] = parking
    report = {"schema_version": 1, "delivery_id": DELIVERY_ID, "accepted": True,
              "acceptance_scope": "all_336_simulation_recordings_and_both_parking_prepare_directions",
              "environment_id": "mla_short", "simulation_only": True, "real_execution_authorized": False,
              "trajectory_count": 336, "ready_for_simulation_replay_on_this_host": 336,
              "batch_counts": dict(Counter(r["source_batch_id"] for r in rows)),
              "excluded": {"obsolete_closed_pattern_bank": 156, "preview_not_formal": 12},
              "runtime_authority_mutation_permitted": False,
              "background_measured": False, "real_robot_used": False,
              "pending_real_release": ["accepted physical calibration and installation binding", "active TCP binding",
                                       "separate real-execution release; no simulation acceptance grants motion permission"],
              "provenance_sha256": provenance, "parking_prepare": parking, "recordings": rows}
    readme = ("# Overnight simulation replay delivery\n\n"
              "Accepted: all 336 exact authoritative drawing CSVs completed GPU compile, mock recording, inspect, "
              "and independent native-clock playback with controller witness. The original 96 and both additional "
              "120-route open/elastic banks are included; the obsolete 156 closed patterns and 12 previews are excluded.\n\n"
              "Each source, compiled asset, sealed recording, independent audit/telemetry artifact and final inventory "
              "was rechecked against actual file bytes. No waypoint, orientation or time_s was scaled, retimed, cropped or skipped.\n\n"
              "Parking home_to_park and park_to_home are separate collision-checked joint prepare assets, not Cartesian "
              "tipdraw sealed recordings. Both GPU paths and independent mock raw controller evidence were verified.\n\n"
              "The ordered collection manifest is collection_manifest.json. It retains the 600 s complete-trajectory-boundary "
              "schedule, 30 s background windows, home returns, and pause after three backgrounds until an explicit hardware-restart resume.\n\n"
              "SIMULATION ONLY. This delivery does not authorize real robot motion. Installation/calibration and active TCP "
              "acceptance remain separate. No real robot, serial capture or background measurement was run by this exporter. "
              "Low magnetic interference at parking is an operator expectation, not a measured result.\n").encode()
    outputs = {"delivery.json": encoded(report), "delivery.csv": csv_bytes(rows),
               "collection_manifest.json": encoded(manifest), "README.md": readme}
    outputs["sha256sums.txt"] = "".join(f"{hashlib.sha256(data).hexdigest()}  {name}\n"
                                       for name, data in sorted(outputs.items())).encode()
    immutable_bundle(destination, outputs)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=CODE_ROOT)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--parking-validation", type=Path)
    parser.add_argument("--parking-mock-evidence", type=Path)
    parser.add_argument("--parking-bundle", type=Path)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--once", action="store_true", help="audit current state once; never start missing stages")
    group.add_argument("--wait", action="store_true", help="wait only for matching currently live processes, then audit")
    parser.add_argument("--poll-s", type=float, default=30.)
    args = parser.parse_args()
    if sys.platform != "linux":
        parser.error("final delivery export runs only on Ubuntu24; Mac is for code and temporary-fixture tests")
    root = args.workspace.expanduser().resolve()
    if not math.isfinite(args.poll_s) or not 1 <= args.poll_s <= 60:
        parser.error("--poll-s must be finite and between 1 and 60 seconds")
    if root.name != "workstation_magtrack":
        parser.error("--workspace must be the workstation_magtrack directory")
    destination = (args.output or root / "reports" / DELIVERY_ID).resolve()
    validation = (args.parking_validation or root / PARKING_VALIDATION).resolve()
    mock = (args.parking_mock_evidence or root / PARKING_MOCK).resolve()
    bundle = (args.parking_bundle or root / PARKING_BUNDLE).resolve()
    if any(not path.is_relative_to(root) for path in (destination, validation, mock, bundle)):
        parser.error("all output/evidence paths must stay inside --workspace")
    try:
        while True:
            state = probe(root, validation, mock, bundle)
            if state["prerequisites_ready_for_full_byte_audit"]:
                report = export(root, destination, validation, mock, bundle)
                print(json.dumps({key: value for key, value in report.items() if key not in {"recordings", "parking_prepare", "provenance_sha256"}}, ensure_ascii=False), flush=True)
                print(f"Immutable delivery: {destination}", flush=True)
                return 0
            missing_batches = {c.run_batch_id for c, row in zip(CAMPAIGNS, state["campaigns"])
                               if not row["accepted_terminal_workflow"]}
            missing_parking = {}
            if state["parking"]["gpu_validation"] != "passed":
                missing_parking["gpu"] = validation
            if state["parking"]["independent_mock"] != "passed":
                missing_parking["mock"] = mock
            if state["parking"]["prepared_bundle"] != "present":
                missing_parking["bundle"] = bundle
            state["live_pids"] = verified_live_pids(root, missing_batches, parking_producers=missing_parking)
            print(json.dumps(state, ensure_ascii=False), flush=True)
            if not args.wait or state["terminal_failure"] or not state["live_pids"]:
                return 2
            time.sleep(args.poll_s)
    except (OSError, ValueError, KeyError, TypeError, ImportError, subprocess.SubprocessError) as error:
        print(json.dumps({"accepted": False, "error": f"{type(error).__name__}: {error}"}), flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
