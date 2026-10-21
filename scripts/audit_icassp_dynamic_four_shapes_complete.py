#!/usr/bin/env python3
"""Audit the immutable complete dynamic-trajectory/recording bundle."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
from typing import Any

import yaml


EXPECTED_SHAPES = ("circle", "rounded_square", "figure8", "depth_loop")
EXPECTED_HEIGHTS = (7, 9, 11, 13, 15)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def audit(workspace: Path, manifest_path: Path) -> list[dict[str, Any]]:
    workspace = workspace.expanduser().resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    entries = manifest.get("entries")
    if not isinstance(entries, list) or len(entries) != 20:
        raise ValueError("complete manifest must contain exactly 20 entries")
    expected = [
        (shape, height) for shape in EXPECTED_SHAPES for height in EXPECTED_HEIGHTS
    ]
    actual = [(entry.get("shape"), entry.get("height_cm")) for entry in entries]
    if actual != expected:
        raise ValueError(f"entry order/content mismatch: {actual!r}")

    result: list[dict[str, Any]] = []
    for entry in entries:
        authority = (
            workspace
            / "environments"
            / "mla"
            / "trajectories"
            / str(entry["authority_file"])
        )
        recording = workspace / str(entry["recording_relative_path"])
        if authority.is_symlink() or not authority.is_file():
            raise FileNotFoundError(f"authority missing: {authority}")
        if recording.is_symlink() or not recording.is_dir():
            raise FileNotFoundError(f"recording missing: {recording}")
        authority_hash = sha256_file(authority)
        if authority_hash != entry["authority_sha256"]:
            raise ValueError(f"authority hash mismatch: {authority.name}")
        manifest_file = recording / "manifest.yaml"
        recording_manifest = yaml.safe_load(manifest_file.read_text(encoding="utf-8"))
        if not isinstance(recording_manifest, dict):
            raise ValueError(f"recording manifest is not a mapping: {recording}")
        trajectory = recording_manifest.get("trajectory")
        validation = recording_manifest.get("validation")
        hashes = recording_manifest.get("hashes")
        if (
            recording_manifest.get("status") != "validated"
            or recording_manifest.get("trajectory_id") != entry["trajectory_id"]
            or not isinstance(trajectory, dict)
            or trajectory.get("uniform_time_scale") != 1
            or trajectory.get("source_waypoint_count") != 601
            or trajectory.get("point_count") != 6001
            or not isinstance(validation, dict)
            or validation.get("simulation_completed") is not True
            or validation.get("controller_result") != "succeeded"
            or not isinstance(hashes, dict)
            or hashes.get("source_trajectory") != authority_hash
        ):
            raise ValueError(f"recording validation/hash gate failed: {recording}")
        if sha256_file(manifest_file) != entry["recording_manifest_sha256"]:
            raise ValueError(f"recording manifest hash mismatch: {recording}")
        result.append(
            {
                "order": entry["order"],
                "shape": entry["shape"],
                "height_cm": entry["height_cm"],
                "trajectory_id": entry["trajectory_id"],
                "authority_sha256": authority_hash,
                "recording_id": recording_manifest["recording_id"],
                "recording_relative_path": recording.relative_to(workspace).as_posix(),
                "status": "PASS",
            }
        )
    return result


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=root)
    parser.add_argument("--manifest", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    workspace = args.workspace.expanduser().resolve()
    manifest = (
        (
            args.manifest
            or workspace
            / "environments"
            / "mla"
            / "trajectories"
            / "icassp_dynamic_four_shapes_complete_manifest.json"
        )
        .expanduser()
        .resolve()
    )
    rows = audit(workspace, manifest)
    output = (
        (
            args.output
            or workspace / "runs" / "icassp_dynamic_four_shapes_complete_audit.csv"
        )
        .expanduser()
        .resolve()
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"PASS: {len(rows)}/20")
    print(f"audit: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
