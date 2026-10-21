#!/usr/bin/env python3
"""Assemble the validated recording manifest/index for the MLA height sweep."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from io import StringIO
from pathlib import Path
from typing import Any

import yaml


INDEX_COLUMNS = (
    "order",
    "kind",
    "axis",
    "height_cm",
    "trajectory_id",
    "authority_file",
    "authority_sha256",
    "recording_id",
    "recording_relative_path",
    "recording_manifest_sha256",
    "duration_s",
    "source_waypoint_count",
    "recording_point_count",
    "status",
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_immutable(path: Path, payload: bytes) -> None:
    if path.exists():
        if path.is_symlink() or not path.is_file() or path.read_bytes() != payload:
            raise FileExistsError(f"refusing to overwrite differing output: {path}")
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)


def latest_validated_recording(
    recordings_root: Path,
    trajectory_id: str,
    authority_sha256: str,
) -> tuple[Path, dict[str, Any]]:
    candidates = sorted(
        (
            path
            for path in recordings_root.glob(f"{trajectory_id}__*")
            if path.is_dir() and not path.is_symlink()
        ),
        reverse=True,
    )
    for path in candidates:
        manifest_path = path / "manifest.yaml"
        if not manifest_path.is_file() or manifest_path.is_symlink():
            continue
        try:
            manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError):
            continue
        if not isinstance(manifest, dict):
            continue
        trajectory = manifest.get("trajectory")
        validation = manifest.get("validation")
        hashes = manifest.get("hashes")
        if (
            manifest.get("status") == "validated"
            and manifest.get("environment_id") == "mla"
            and manifest.get("trajectory_id") == trajectory_id
            and isinstance(trajectory, dict)
            and isinstance(validation, dict)
            and isinstance(hashes, dict)
            and hashes.get("source_trajectory") == authority_sha256
            and trajectory.get("uniform_time_scale") == 1
            and validation.get("simulation_completed") is True
            and validation.get("controller_result") == "succeeded"
        ):
            return path, manifest
    raise FileNotFoundError(f"no exact validated recording found for {trajectory_id}")


def assemble(workspace: Path, output_manifest: Path, output_index: Path) -> tuple[Path, Path]:
    workspace = workspace.expanduser().resolve()
    trajectory_root = workspace / "environments" / "mla" / "trajectories"
    recordings_root = workspace / "environments" / "mla" / "recordings"
    authority_manifest_path = (
        trajectory_root / "icassp_f64_mj_height_sweep_feasible_v2_authority_manifest.json"
    )
    authority_index_path = (
        trajectory_root / "icassp_f64_mj_height_sweep_feasible_v2_authority_index.csv"
    )
    authority_manifest = json.loads(authority_manifest_path.read_text(encoding="utf-8"))
    with authority_index_path.open("r", encoding="utf-8", newline="") as stream:
        authority_rows = list(csv.DictReader(stream))
    if authority_manifest.get("route_count") != 20 or len(authority_rows) != 20:
        raise ValueError("height-sweep authority set is not exactly 20 routes")

    entries: list[dict[str, Any]] = []
    index_rows: list[dict[str, str]] = []
    manifest_routes = authority_manifest.get("routes")
    if not isinstance(manifest_routes, list) or len(manifest_routes) != len(authority_rows):
        raise ValueError("authority manifest/index route count differs")
    for order, (authority_row, source_entry) in enumerate(
        zip(authority_rows, manifest_routes, strict=True)
    ):
        if not isinstance(source_entry, dict):
            raise ValueError("authority manifest route is not an object")
        authority_file = str(authority_row["authority_file"])
        trajectory_id = str(authority_row["trajectory_id"])
        if Path(authority_file).stem != trajectory_id:
            raise ValueError(f"trajectory/file ID mismatch: {authority_file}")
        authority_path = trajectory_root / authority_file
        authority_sha = sha256_file(authority_path)
        if authority_sha != authority_row["authority_sha256"]:
            raise ValueError(f"authority hash mismatch: {authority_file}")
        recording_path, recording_manifest = latest_validated_recording(
            recordings_root, trajectory_id, authority_sha
        )
        trajectory = recording_manifest["trajectory"]
        expected_duration = float(authority_row["duration_s"])
        expected_count = int(source_entry["source_row_count"])
        if (
            float(trajectory["duration_s"]) != expected_duration
            or float(trajectory["source_duration_s"]) != expected_duration
            or int(trajectory["source_waypoint_count"]) != expected_count
        ):
            raise ValueError(f"recording trajectory contract mismatch: {recording_path}")
        entry = {
            "order": order,
            "kind": authority_row["kind"],
            "axis": authority_row["axis"],
            "height_cm": int(authority_row["height_cm"]),
            "trajectory_id": trajectory_id,
            "authority_file": authority_file,
            "authority_sha256": authority_sha,
            "recording_id": recording_manifest["recording_id"],
            "recording_relative_path": recording_path.relative_to(workspace).as_posix(),
            "recording_manifest_sha256": sha256_file(recording_path / "manifest.yaml"),
            "duration_s": expected_duration,
            "source_waypoint_count": expected_count,
            "recording_point_count": int(trajectory["point_count"]),
            "status": "validated",
        }
        entries.append(entry)
        index_rows.append({field: str(entry[field]) for field in INDEX_COLUMNS})

    output = {
        "schema_version": 1,
        "manifest_id": "icassp_f64_mj_height_sweep_validated_recordings",
        "environment_id": "mla",
        "robot_type": "ur5",
        "frame_id": "plane1",
        "route_count": len(entries),
        "routes": entries,
        "authority_manifest": authority_manifest_path.relative_to(workspace).as_posix(),
        "authority_manifest_sha256": sha256_file(authority_manifest_path),
        "recording_policy": (
            "sealed exact recording; no runtime scaling, retiming, clipping, "
            "cropping, waypoint substitution, or point dropping"
        ),
    }
    write_immutable(output_manifest, (json.dumps(output, indent=2) + "\n").encode("utf-8"))
    stream = StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=INDEX_COLUMNS, lineterminator="\n")
    writer.writeheader()
    writer.writerows(index_rows)
    write_immutable(output_index, stream.getvalue().encode("utf-8"))
    return output_manifest, output_index


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=root)
    parser.add_argument("--manifest", type=Path, default=None)
    parser.add_argument("--index", type=Path, default=None)
    args = parser.parse_args()
    trajectory_root = args.workspace / "environments" / "mla" / "trajectories"
    manifest = args.manifest or (
        trajectory_root / "icassp_f64_mj_height_sweep_validated_recordings_manifest.json"
    )
    index = args.index or (
        trajectory_root / "icassp_f64_mj_height_sweep_validated_recordings_index.csv"
    )
    assembled = assemble(args.workspace, manifest, index)
    print(f"manifest: {assembled[0]}")
    print(f"index: {assembled[1]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
