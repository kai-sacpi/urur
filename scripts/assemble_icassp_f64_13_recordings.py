#!/usr/bin/env python3
"""Assemble the validated recording index for the 13-route F64 experiment."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
from typing import Any

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
# Authority generators keep the paper-facing manifest name (without the
# ``_mj`` recording prefix).  Keep this mapping here so selecting the v2
# recording prefix alone is sufficient; callers may still override it.
DEFAULT_AUTHORITY_MANIFEST_NAMES = {
    "icassp_f64_mj_feasible": "icassp_f64_13_feasible_authority_manifest.json",
    "icassp_f64_mj_feasible_v2": "icassp_f64_13_feasible_v2_authority_manifest.json",
}
CSV_COLUMNS = (
    "order",
    "trajectory_id",
    "source_route",
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


def _write_immutable(path: Path, payload: bytes) -> None:
    if path.exists():
        if path.is_symlink() or not path.is_file() or path.read_bytes() != payload:
            raise FileExistsError(f"refusing to overwrite differing index: {path}")
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(payload)


def _latest_valid_recording(recordings_root: Path, trajectory_id: str) -> tuple[Path, dict[str, Any]]:
    candidates = sorted(
        (path for path in recordings_root.glob(f"{trajectory_id}__*") if path.is_dir() and not path.is_symlink()),
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
        if (
            manifest.get("status") == "validated"
            and manifest.get("environment_id") == "mla"
            and manifest.get("trajectory_id") == trajectory_id
            and isinstance(trajectory, dict)
            and isinstance(validation, dict)
            and trajectory.get("uniform_time_scale") == 1
            and validation.get("simulation_completed") is True
            and validation.get("controller_result") == "succeeded"
        ):
            return path, manifest
    raise FileNotFoundError(f"no validated recording found for {trajectory_id}")


def _authority_route_ids(source_manifest: dict[str, Any]) -> tuple[str, ...]:
    """Return the ordered trajectory IDs from either supported authority schema.

    The original F64 authority manifest used a list of strings in ``routes``;
    the feasible/v2 generators use a list of metadata objects.  Assembly only
    needs the immutable order, so accept both representations while rejecting
    malformed or mixed entries.
    """

    routes = source_manifest.get("routes")
    if not isinstance(routes, list):
        raise ValueError("authority manifest must contain a routes list")
    ids: list[str] = []
    for route in routes:
        if isinstance(route, str):
            trajectory_id = route
        elif isinstance(route, dict) and isinstance(route.get("trajectory_id"), str):
            trajectory_id = route["trajectory_id"]
        else:
            raise ValueError("authority manifest routes must be IDs or metadata objects")
        ids.append(trajectory_id)
    return tuple(ids)


def assemble(
    workspace: Path,
    output_manifest: Path,
    output_index: Path,
    *,
    id_prefix: str = DEFAULT_ID_PREFIX,
    authority_manifest_name: str | None = None,
) -> tuple[Path, Path]:
    workspace = workspace.expanduser().resolve()
    environment = workspace / "environments" / "mla"
    trajectories = environment / "trajectories"
    recordings = environment / "recordings"
    source_manifest_name = authority_manifest_name or DEFAULT_AUTHORITY_MANIFEST_NAMES.get(
        id_prefix, f"{id_prefix}_13_authority_manifest.json"
    )
    source_manifest_path = trajectories / source_manifest_name
    source_manifest = json.loads(source_manifest_path.read_text(encoding="utf-8"))
    expected_route_ids = tuple(f"{id_prefix}_{route}" for route in ROUTES)
    if _authority_route_ids(source_manifest) != expected_route_ids:
        raise ValueError("authority manifest route order does not match the 13-route F64 contract")

    entries: list[dict[str, Any]] = []
    rows: list[dict[str, str]] = []
    for order, route in enumerate(ROUTES):
        trajectory_id = f"{id_prefix}_{route}"
        authority = trajectories / f"{trajectory_id}.csv"
        if authority.is_symlink() or not authority.is_file():
            raise FileNotFoundError(f"authority missing: {authority}")
        recording, manifest = _latest_valid_recording(recordings, trajectory_id)
        trajectory = manifest["trajectory"]
        expected_duration = 16.0 if route.startswith("eight_") else 12.0
        if float(trajectory["duration_s"]) != expected_duration or float(trajectory["source_duration_s"]) != expected_duration:
            raise ValueError(f"{recording}: duration does not match the paper route period")
        authority_sha = sha256_file(authority)
        recording_manifest_path = recording / "manifest.yaml"
        recording_manifest_sha = sha256_file(recording_manifest_path)
        relative = recording.relative_to(workspace).as_posix()
        entry = {
            "order": order,
            "trajectory_id": trajectory_id,
            "source_route": route,
            "authority_file": authority.name,
            "authority_sha256": authority_sha,
            "recording_id": manifest["recording_id"],
            "recording_relative_path": relative,
            "recording_path": str(recording),
            "recording_manifest_sha256": recording_manifest_sha,
            "duration_s": float(trajectory["duration_s"]),
            "source_waypoint_count": int(trajectory["source_waypoint_count"]),
            "recording_point_count": int(trajectory["point_count"]),
            "status": "validated",
        }
        entries.append(entry)
        rows.append({key: str(entry[key]) for key in CSV_COLUMNS})

    manifest = {
        "schema_version": 1,
        "manifest_id": f"{id_prefix}_13_validated_recordings",
        "environment_id": "mla",
        "robot_type": "ur5",
        "frame_id": "plane1",
        "route_count": len(entries),
        "routes": entries,
        "authority_manifest": source_manifest_path.relative_to(workspace).as_posix(),
        "authority_manifest_sha256": sha256_file(source_manifest_path),
        "recording_policy": "sealed canonical recording; no runtime scaling, retiming, clipping, or point dropping",
    }
    _write_immutable(output_manifest, (json.dumps(manifest, indent=2) + "\n").encode("utf-8"))
    from io import StringIO

    stream = StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=CSV_COLUMNS, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    _write_immutable(output_index, stream.getvalue().encode("utf-8"))
    return output_manifest, output_index


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=root)
    parser.add_argument("--id-prefix", default=DEFAULT_ID_PREFIX)
    parser.add_argument("--authority-manifest-name", default=None)
    parser.add_argument("--manifest", type=Path, default=None)
    parser.add_argument("--index", type=Path, default=None)
    args = parser.parse_args()
    default_root = args.workspace / "environments/mla/trajectories"
    manifest_path = args.manifest or default_root / f"{args.id_prefix}_13_validated_recordings_manifest.json"
    index_path = args.index or default_root / f"{args.id_prefix}_13_validated_recordings_index.csv"
    manifest, index = assemble(
        args.workspace,
        manifest_path,
        index_path,
        id_prefix=args.id_prefix,
        authority_manifest_name=args.authority_manifest_name,
    )
    print(f"manifest: {manifest}")
    print(f"index: {index}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
