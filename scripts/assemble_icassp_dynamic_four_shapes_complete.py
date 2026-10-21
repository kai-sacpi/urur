#!/usr/bin/env python3
"""Assemble the complete 4-shape × 5-height simulation authority index.

The original four-shape manifest remains immutable, including its three failed
centred depth-loop authorities.  The complete set uses the explicitly
redesigned one-sided ``depth_loop_safe10mm`` authorities for all five depth
loop heights and reuses the already validated circle, rounded-square, and
figure-eight recordings byte-for-byte.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
from typing import Any

import yaml


HEIGHTS_CM = (7, 9, 11, 13, 15)
SHAPES = ("circle", "rounded_square", "figure8", "depth_loop")
CSV_COLUMNS = (
    "shape",
    "height_cm",
    "trajectory_id",
    "authority_file",
    "authority_sha256",
    "recording_id",
    "recording_relative_path",
    "recording_manifest_sha256",
    "status",
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_immutable(path: Path, payload: bytes) -> str:
    digest = hashlib.sha256(payload).hexdigest()
    if path.exists():
        if path.is_symlink() or not path.is_file() or path.read_bytes() != payload:
            raise FileExistsError(f"refusing to overwrite differing index: {path}")
    else:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("xb") as stream:
            stream.write(payload)
    return digest


def _latest_valid_recording(
    recordings_root: Path, trajectory_id: str
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
        if (
            manifest.get("status") == "validated"
            and manifest.get("environment_id") == "mla"
            and manifest.get("trajectory_id") == trajectory_id
            and isinstance(trajectory, dict)
            and trajectory.get("uniform_time_scale") == 1
            and isinstance(validation, dict)
            and validation.get("simulation_completed") is True
            and validation.get("controller_result") == "succeeded"
        ):
            return path, manifest
    raise FileNotFoundError(f"no validated recording found for {trajectory_id}")


def _source_id(shape: str, height_cm: int) -> tuple[str, str]:
    if shape == "depth_loop":
        return (
            f"icassp_dynamic_depth_loop_safe10mm_z{height_cm:02d}",
            "upstream_redesign_depth_loop_safe10mm",
        )
    return (f"icassp_dynamic_{shape}_z{height_cm:02d}", "original_four_shapes")


def assemble(
    workspace: Path,
    output_manifest: Path,
    output_index: Path,
) -> tuple[Path, Path]:
    workspace = workspace.expanduser().resolve()
    environment = workspace / "environments" / "mla"
    trajectories_root = environment / "trajectories"
    recordings_root = environment / "recordings"
    entries: list[dict[str, Any]] = []
    csv_rows: list[dict[str, str]] = []

    for shape in SHAPES:
        for height_cm in HEIGHTS_CM:
            trajectory_id, source_revision = _source_id(shape, height_cm)
            authority = trajectories_root / f"{trajectory_id}.csv"
            if authority.is_symlink() or not authority.is_file():
                raise FileNotFoundError(f"authority is missing: {authority}")
            recording, recording_manifest = _latest_valid_recording(
                recordings_root, trajectory_id
            )
            recording_manifest_path = recording / "manifest.yaml"
            authority_sha256 = sha256_file(authority)
            recording_manifest_sha256 = sha256_file(recording_manifest_path)
            relative_recording = recording.relative_to(workspace).as_posix()
            entry = {
                "order": len(entries),
                "shape": shape,
                "height_cm": height_cm,
                "trajectory_id": trajectory_id,
                "source_revision": source_revision,
                "authority_file": authority.name,
                "authority_sha256": authority_sha256,
                "recording_id": recording_manifest["recording_id"],
                "recording_relative_path": relative_recording,
                "recording_path": str(recording),
                "recording_manifest_sha256": recording_manifest_sha256,
                "recording_status": recording_manifest["status"],
                "duration_s": recording_manifest["trajectory"]["duration_s"],
                "source_waypoint_count": recording_manifest["trajectory"][
                    "source_waypoint_count"
                ],
                "recording_point_count": recording_manifest["trajectory"][
                    "point_count"
                ],
                "uniform_time_scale": recording_manifest["trajectory"][
                    "uniform_time_scale"
                ],
            }
            entries.append(entry)
            csv_rows.append(
                {
                    "shape": shape,
                    "height_cm": str(height_cm),
                    "trajectory_id": trajectory_id,
                    "authority_file": authority.name,
                    "authority_sha256": authority_sha256,
                    "recording_id": str(entry["recording_id"]),
                    "recording_relative_path": relative_recording,
                    "recording_manifest_sha256": recording_manifest_sha256,
                    "status": "validated",
                }
            )

    manifest = {
        "schema_version": 1,
        "manifest_id": "icassp_dynamic_four_shapes_five_heights_complete",
        "environment_id": "mla",
        "robot_type": "ur5",
        "frame_id": "plane1",
        "shapes": list(SHAPES),
        "nominal_heights_cm": list(HEIGHTS_CM),
        "condition_count": len(entries),
        "authority_policy": (
            "all waypoints and time_s are immutable hard constraints; runtime "
            "does not scale, retime, clip, or drop points"
        ),
        "timing": {
            "source_sample_dt_s": 0.1,
            "source_duration_s": 60.0,
            "final_hold_duration_s": 3.0,
            "parameterization": "quintic minimum jerk",
        },
        "depth_loop_redesign": {
            "original_authority": "icassp_dynamic_space_z07/z09/z15 remain preserved as failed historical inputs",
            "selected_authority": "icassp_dynamic_depth_loop_safe10mm_z{height}",
            "xy_radius_m": 0.05,
            "depth_amplitude_m": 0.01,
            "policy": "one-sided farther from sensor; local plane1 z <= -nominal height",
            "reason": "centred 40 mm excursion violated the calibrated sensor keepout or reachability at 7/9/15 cm",
        },
        "entries": entries,
    }
    manifest_payload = (json.dumps(manifest, indent=2) + "\n").encode("utf-8")
    _write_immutable(output_manifest, manifest_payload)

    from io import StringIO

    stream = StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=CSV_COLUMNS, lineterminator="\n")
    writer.writeheader()
    writer.writerows(csv_rows)
    _write_immutable(output_index, stream.getvalue().encode("utf-8"))
    return output_manifest, output_index


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=root)
    parser.add_argument(
        "--output-manifest",
        type=Path,
        default=None,
    )
    parser.add_argument("--output-index", type=Path, default=None)
    args = parser.parse_args()
    workspace = args.workspace.expanduser().resolve()
    default_root = workspace / "environments" / "mla" / "trajectories"
    output_manifest = (
        (
            args.output_manifest
            or default_root / "icassp_dynamic_four_shapes_complete_manifest.json"
        )
        .expanduser()
        .resolve()
    )
    output_index = (
        (
            args.output_index
            or default_root / "icassp_dynamic_four_shapes_complete_index.csv"
        )
        .expanduser()
        .resolve()
    )
    manifest, index = assemble(workspace, output_manifest, output_index)
    print(f"manifest: {manifest}")
    print(f"index: {index}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
