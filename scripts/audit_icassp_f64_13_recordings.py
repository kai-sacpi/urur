#!/usr/bin/env python3
"""Fail-closed audit of the 13-route F64 authority/recording bundle."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import yaml


ROUTES = (
    "line_x_vertical_z09",
    "line_x_tilt90_z09",
    "line_y_vertical_z09",
    "line_y_tilt90_z09",
    "line_x_vertical_z13",
    "line_x_tilt90_z13",
    "line_y_vertical_z13",
    "line_y_tilt90_z13",
    "line_x_vertical_z15",
    "line_x_tilt90_z15",
    "line_y_vertical_z15",
    "line_y_tilt90_z15",
    "eight_tilt90_z13",
)
DEFAULT_ID_PREFIX = "icassp_f64"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def audit(
    workspace: Path,
    manifest_path: Path,
    *,
    id_prefix: str = DEFAULT_ID_PREFIX,
) -> list[dict[str, str]]:
    workspace = workspace.expanduser().resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    entries = manifest.get("routes")
    if not isinstance(entries, list) or len(entries) != len(ROUTES):
        raise ValueError("F64 recording manifest must contain exactly 13 routes")
    expected_ids = [f"{id_prefix}_{route}" for route in ROUTES]
    if [entry.get("trajectory_id") for entry in entries] != expected_ids:
        raise ValueError("F64 recording manifest route order/content mismatch")
    output: list[dict[str, str]] = []
    for entry in entries:
        trajectory_id = str(entry["trajectory_id"])
        authority = workspace / "environments" / "mla" / "trajectories" / str(entry["authority_file"])
        recording = workspace / str(entry["recording_relative_path"])
        if authority.is_symlink() or not authority.is_file():
            raise FileNotFoundError(f"authority missing: {authority}")
        if recording.is_symlink() or not recording.is_dir():
            raise FileNotFoundError(f"recording missing: {recording}")
        authority_sha = sha256_file(authority)
        if authority_sha != entry["authority_sha256"]:
            raise ValueError(f"authority hash mismatch: {authority}")
        recording_manifest_path = recording / "manifest.yaml"
        recording_manifest = yaml.safe_load(recording_manifest_path.read_text(encoding="utf-8"))
        if not isinstance(recording_manifest, dict):
            raise ValueError(f"recording manifest is invalid: {recording}")
        trajectory = recording_manifest.get("trajectory")
        validation = recording_manifest.get("validation")
        if not isinstance(trajectory, dict) or not isinstance(validation, dict):
            raise ValueError(f"recording evidence is incomplete: {recording}")
        expected_duration = 16.0 if trajectory_id.endswith("eight_tilt90_z13") else 12.0
        if (
            recording_manifest.get("status") != "validated"
            or recording_manifest.get("environment_id") != "mla"
            or recording_manifest.get("trajectory_id") != trajectory_id
            or trajectory.get("uniform_time_scale") != 1
            or float(trajectory.get("duration_s")) != expected_duration
            or float(trajectory.get("source_duration_s")) != expected_duration
            or trajectory.get("source_waypoint_count") != (801 if expected_duration == 16.0 else 601)
            or validation.get("simulation_completed") is not True
            or validation.get("controller_result") != "succeeded"
        ):
            raise ValueError(f"recording validation gate failed: {recording}")
        hashes = recording_manifest.get("hashes")
        if not isinstance(hashes, dict) or hashes.get("source_trajectory") != authority_sha:
            raise ValueError(f"source hash is not bound in recording: {recording}")
        recording_manifest_sha = sha256_file(recording_manifest_path)
        if recording_manifest_sha != entry["recording_manifest_sha256"]:
            raise ValueError(f"recording manifest hash mismatch: {recording}")
        output.append(
            {
                "order": str(entry["order"]),
                "trajectory_id": trajectory_id,
                "recording_id": str(recording_manifest["recording_id"]),
                "recording_relative_path": recording.relative_to(workspace).as_posix(),
                "status": "PASS",
            }
        )
    return output


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=root)
    parser.add_argument("--id-prefix", default=DEFAULT_ID_PREFIX)
    parser.add_argument("--manifest", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    default_root = args.workspace / "environments/mla/trajectories"
    manifest_path = args.manifest or default_root / f"{args.id_prefix}_13_validated_recordings_manifest.json"
    output_path = args.output or args.workspace / "runs" / f"{args.id_prefix}_13_recordings_audit.csv"
    rows = audit(args.workspace, manifest_path, id_prefix=args.id_prefix)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"PASS: {len(rows)}/13")
    print(f"audit: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
