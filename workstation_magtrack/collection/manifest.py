"""Freeze an ordered collection list without touching any authority/recording."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any

import yaml

ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*\Z")
PHASES = ("compile", "build", "inspect", "play")


def sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def encoded(value: Any) -> bytes:
    return (json.dumps(value, indent=2, ensure_ascii=False, sort_keys=True, allow_nan=False) + "\n").encode()


def immutable_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(encoded(value))


def contained(root: Path, name: str) -> Path:
    path = (root / name).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError(f"path escapes workspace: {name}")
    return path


def config_at(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text())
    if value.get("schema_version") != 1 or value.get("environment_id") != "mla_short":
        raise ValueError("collection requires mla_short config schema 1")
    parking, schedule = value["parking"], value["schedule"]
    if parking.get("operator_confirmed") is not True:
        raise ValueError("parking joints are not operator-confirmed")
    if (len(parking["joint_names"]) != 6 or len(set(parking["joint_names"])) != 6 or
            len(parking["joint_positions_deg"]) != 6 or
            any(not math.isfinite(float(q)) for q in parking["joint_positions_deg"])):
        raise ValueError("parking requires six finite named joints")
    for key in ("interval_s", "background_duration_s"):
        if not math.isfinite(float(schedule[key])) or schedule[key] <= 0:
            raise ValueError(f"invalid {key}")
    if schedule["pause_after_backgrounds"] != 3 or schedule["trigger"] != "complete_trajectory_boundary":
        raise ValueError("policy must pause after three complete boundary-triggered captures")
    if schedule.get("final_return_home") is not True or schedule.get("restart_requires_explicit_confirmation") is not True:
        raise ValueError("final home and explicit restart are mandatory")
    return value


def environment_hashes(root: Path) -> dict[str, str]:
    env = root / "environments/mla_short"
    result = {p.relative_to(root).as_posix(): sha(p) for p in sorted(env.rglob("*"))
              if p.is_file() and p.relative_to(env).parts[0] not in {"trajectories", "recordings"}}
    if not result:
        raise ValueError("empty environment")
    result["curobo.lock"] = sha(root / "curobo.lock")
    return result


def recording_hashes(root: Path, directory: Path, expected: str) -> dict[str, str]:
    if sha(directory / "source_tip_trajectory.csv") != expected:
        raise ValueError("sealed source differs from authority")
    files: dict[str, str] = {}
    names: set[str] = set()
    for line in (directory / "sha256sums.txt").read_text().splitlines():
        digest, separator, name = line.partition("  ")
        if not separator or not re.fullmatch(r"[0-9a-f]{64}", digest):
            raise ValueError("invalid sealed checksum entry")
        path = contained(directory, name)
        relative = path.relative_to(directory).as_posix()
        if relative in names or relative == "sha256sums.txt":
            raise ValueError("duplicate/self-referential sealed checksum entry")
        names.add(relative)
        if not path.is_file() or sha(path) != digest:
            raise ValueError(f"sealed checksum mismatch: {name}")
        files[path.relative_to(root).as_posix()] = digest
    required = {"source_tip_trajectory.csv", "joint_trajectory.csv", "manifest.yaml"}
    if not required.issubset(names):
        raise ValueError("sealed checksum list misses mandatory files")
    actual = {p.relative_to(directory).as_posix() for p in directory.rglob("*")
              if p.is_file() and p.relative_to(directory).as_posix() != "sha256sums.txt"}
    if names != actual:
        raise ValueError("sealed checksum list does not protect every recording file")
    files[(directory / "sha256sums.txt").relative_to(root).as_posix()] = sha(directory / "sha256sums.txt")
    return files


def create(root: Path, indexes: list[Path], config: Path,
           ids: list[str] | None = None) -> dict[str, Any]:
    root, config = root.resolve(), config.resolve()
    policy = config_at(config)
    entries: dict[str, dict[str, Any]] = {}
    for index in indexes:
        data = json.loads(index.read_text())
        if data.get("environment_id") != "mla_short":
            raise ValueError("inventory is not mla_short")
        for row in data["recordings"]:
            name = row["trajectory_id"]
            if name in entries:
                raise ValueError(f"duplicate trajectory across inventories: {name}")
            entries[name] = row
    names = list(entries) if ids is None else ids
    if not names or len(set(names)) != len(names):
        raise ValueError("collection list must be nonempty and unique")
    rows = []
    for name in names:
        if not ID.fullmatch(name) or name not in entries:
            raise ValueError(f"unknown/unsafe trajectory ID: {name}")
        row = entries[name]
        # The original 96-route exporter and the later enhanced-bank exporter
        # use two names for the same independently verified controller witness.
        # An explicitly present failing alias must never be hidden by a passed one.
        witness_keys = [key for key in ("coherent_playback_witness", "coherent_controller_witness")
                        if key in row]
        if (not all(row.get(p) == "passed" for p in PHASES) or
                row.get("ready_for_simulation_replay_on_this_host") is not True or
                not witness_keys or any(row[key] != "passed" for key in witness_keys)):
            raise ValueError(f"not four-stage ready (no silent skipping): {name}")
        rid = row["recording_id"]
        if not ID.fullmatch(rid):
            raise ValueError("unsafe recording ID")
        source_rel = f"environments/mla_short/trajectories/{name}.csv"
        source = contained(root, source_rel)
        if sha(source) != row["source_sha256"]:
            raise ValueError(f"source changed: {name}")
        rec_rel = f"environments/mla_short/recordings/{rid}"
        hashes = recording_hashes(root, contained(root, rec_rel), row["source_sha256"])
        duration = float(row["duration_s"])
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError("invalid recording duration")
        rows.append({"trajectory_id": name, "recording_id": rid, "duration_s": duration,
                     "source_file": source_rel, "source_sha256": row["source_sha256"],
                     "recording_directory": rec_rel, "sealed_hashes": hashes,
                     "coherent_witness_provenance": {key: row[key] for key in witness_keys},
                     "real_execution_authorized": row.get("real_execution_authorized") is True})
    return {"schema_version": 1, "environment_id": "mla_short", "config": policy,
            "config_sha256": sha(config), "environment_hashes": environment_hashes(root),
            "inventory_provenance": [{"path": str(p), "sha256": sha(p)} for p in indexes],
            "real_execution_authorized": policy["execution"].get("real_execution_authorized") is True
            and all(r["real_execution_authorized"] for r in rows), "recordings": rows}


def verify(root: Path, value: dict[str, Any]) -> None:
    if value.get("schema_version") != 1 or value.get("environment_id") != "mla_short":
        raise ValueError("invalid collection manifest")
    if environment_hashes(root) != value["environment_hashes"]:
        raise ValueError("frozen environment changed")
    if not value["recordings"]:
        raise ValueError("empty collection manifest")
    for row in value["recordings"]:
        if sha(contained(root, row["source_file"])) != row["source_sha256"]:
            raise ValueError("authoritative source changed")
        for path, expected in row["sealed_hashes"].items():
            if sha(contained(root, path)) != expected:
                raise ValueError(f"sealed file changed: {path}")


def require_real_release(value: dict[str, Any], root: Path | None = None, *,
                         verify_recordings: bool = True) -> None:
    # A separately issued, evidence-bound release preserves the original
    # nominal delivery unchanged. Its historic booleans are not an execution
    # permit and must not be rewritten to manufacture one.
    if "formal_release" in value:
        if root is None:
            raise ValueError("formal release verification requires the current workspace")
        from .release import verify_release
        verify_release(root, value, verify_recordings=verify_recordings)
        return
    # Never convert a simulation acceptance into a real-machine permission.
    if (value.get("real_execution_authorized") is not True or
            value["config"]["execution"].get("real_execution_authorized") is not True or
            value["config"]["execution"].get("simulation_only") is not False or
            not all(r.get("real_execution_authorized") is True for r in value["recordings"])):
        raise ValueError("simulation_only: current installation/trajectories are not released for real collection")
    release = value.get("accepted_installation_release", {})
    required = ("installation_id", "calibration_acceptance_sha256", "active_tcp_binding_sha256",
                "parking_validation_sha256")
    if release.get("accepted") is not True or any(not release.get(k) for k in required):
        raise ValueError("missing accepted calibration, installation/TCP and parking release")
    # Legacy nonempty SHA fields do not identify evidence to read/validate.
    raise ValueError("missing hash-bound formal release; legacy flags are not physical acceptance")
