#!/usr/bin/env python3
"""Generate the four-shape, five-height ICASSP dynamic authorities.

This is a separate authority set from the earlier exploratory ten-route
manifest.  The geometry is the paper-side definition used by
``dynamic_crlb.py``: a 5 cm circle, rounded square, figure eight, and the
non-planar ``space``/depth loop with a 4 cm depth amplitude.  Each nominal
height is expressed as positive distance above the sensor plane; the MLA
``plane1`` authority uses ``tip_z_m = -z_A``.

The imported generator deliberately remains the single implementation of the
minimum-jerk sampler, explicit dwell/hold handling, and finite-difference
diagnostics.  Existing authority bytes are never replaced with different
bytes.  A new manifest is used so the earlier exploratory manifest remains a
reproducible historical record.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import generate_icassp_dynamic_trajectories as base


HEIGHTS_CM = (7, 9, 11, 13, 15)
SHAPES = ("circle", "rounded_square", "figure8", "depth_loop")
MANIFEST_NAME = "icassp_dynamic_four_shapes_authority_manifest.json"


def trajectory_id(shape: str, height_cm: int) -> str:
    """Return the stable CLI/file id for one shape and nominal height."""

    if shape not in SHAPES:
        raise ValueError(f"unknown shape: {shape}")
    # Keep the established ``space`` spelling for the depth-loop route so
    # existing z11 authority/recording artifacts can be reused byte-for-byte.
    route_name = "space" if shape == "depth_loop" else shape
    return f"icassp_dynamic_{route_name}_z{height_cm:02d}"


def build_rows(shape: str, height_cm: int):
    """Build one immutable route using the shared minimum-jerk primitives."""

    height_m = height_cm / 100.0
    if shape == "depth_loop":
        return base._simple_loop_route("space", height_m, radius_m=0.05)
    return base._simple_loop_route(shape, height_m, radius_m=0.05)


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _write_immutable(path: Path, payload: bytes) -> str:
    digest = _sha256(payload)
    if path.exists():
        if path.is_symlink() or not path.is_file() or path.read_bytes() != payload:
            raise FileExistsError(
                f"refusing to overwrite differing authority: {path}"
            )
    else:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("xb") as stream:
            stream.write(payload)
    return digest


def generate(output_directory: Path, manifest_path: Path) -> tuple[Path, ...]:
    output_directory = output_directory.expanduser().resolve()
    manifest_path = manifest_path.expanduser().resolve()
    entries: list[dict[str, object]] = []
    paths: list[Path] = []
    for shape in SHAPES:
        for height_cm in HEIGHTS_CM:
            route_id = trajectory_id(shape, height_cm)
            rows = build_rows(shape, height_cm)
            diagnostics = base._validate_rows(route_id, rows)
            payload = base._canonical_csv(rows)
            path = output_directory / f"{route_id}.csv"
            digest = _write_immutable(path, payload)
            diagnostics.update(
                {
                    "shape": shape,
                    "nominal_height_cm": height_cm,
                    "authority_file": path.name,
                    "authority_sha256": digest,
                    "frame_id": base.FRAME_ID,
                    "orientation_xyzw": list(base.QUATERNION_XYZW),
                    "coordinate_mapping": "tip_z_m = -z_A; x/y unchanged",
                    "xy_radius_m": 0.05,
                    "depth_amplitude_m": 0.04 if shape == "depth_loop" else 0.0,
                }
            )
            entries.append(diagnostics)
            paths.append(path)
            print(f"{path}: sha256={digest}")

    manifest = {
        "schema_version": 1,
        "manifest_id": "icassp_dynamic_four_shapes_five_heights",
        "environment_id": "mla",
        "frame_id": base.FRAME_ID,
        "shapes": list(SHAPES),
        "nominal_heights_cm": list(HEIGHTS_CM),
        "geometry": {
            "circle_radius_m": 0.05,
            "rounded_square_half_extent_m": 0.05,
            "figure8_half_extent_m": 0.05,
            "depth_loop_xy_radius_m": 0.05,
            "depth_loop_amplitude_m": 0.04,
        },
        "coordinate_convention": {
            "paper_frame": "sensor-board A frame; z_A positive above sensor plane",
            "authority_frame": "plane1; local +Z points physically downward",
            "mapping": "tip_x_m=x_A, tip_y_m=y_A, tip_z_m=-z_A",
        },
        "orientation_xyzw": list(base.QUATERNION_XYZW),
        "timing": {
            "sample_dt_s": base.DT_S,
            "duration_s": base.TOTAL_DURATION_S,
            "final_hold_duration_s": base.HOLD_DURATION_S,
            "parameterization": "quintic minimum jerk per dwell/loop segment",
        },
        "authority_policy": (
            "waypoints and time_s are immutable hard constraints; no runtime "
            "scaling, retiming, clipping, or point dropping"
        ),
        "source_index": "/Users/huaxin/Codes/ICASSP2027/TRAJECTORY_GENERATION_INDEX_20260908.md",
        "trajectories": entries,
    }
    manifest_payload = (json.dumps(manifest, indent=2, sort_keys=False) + "\n").encode(
        "utf-8"
    )
    _write_immutable(manifest_path, manifest_payload)
    print(f"manifest: {manifest_path}")
    return tuple(paths)


def main() -> int:
    repository_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-directory",
        type=Path,
        default=repository_root / "environments" / "mla" / "trajectories",
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=None,
    )
    args = parser.parse_args()
    output_directory = args.output_directory.expanduser().resolve()
    manifest_path = (
        args.manifest or output_directory / MANIFEST_NAME
    ).expanduser().resolve()
    generate(output_directory, manifest_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
