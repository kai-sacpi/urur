#!/usr/bin/env python3
"""Generate a clearance-aware depth-loop authority set.

The first four-shape authority manifest used a centred 40 mm vertical
excursion for the ``space`` route.  In the calibrated ``mla`` scene that
excursion enters the protected sensor column at 7/9 cm and reaches an
unreachable wrist configuration at 15 cm.  This generator is an explicit
upstream redesign: the loop keeps the same 50 mm XY circle and minimum-jerk
timing, but its depth excursion is one-sided (farther from the board) and is
parameterised by an immutable amplitude.  Runtime never changes these CSVs.

The output IDs include the design tag so the original failed authorities stay
available as historical evidence and cannot be silently overwritten.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import generate_icassp_dynamic_trajectories as base


HEIGHTS_CM = (7, 9, 11, 13, 15)
FRAME_ID = base.FRAME_ID
CSV_COLUMNS = base.CSV_COLUMNS


def _safe_route(height_m: float, amplitude_m: float) -> list[dict[str, object]]:
    """Build a one-sided, downward-from-board depth loop.

    ``plane1`` has local -Z physically above the sensor.  The nominal point
    therefore starts at ``-height_m`` and the loop only moves toward more
    negative local Z.  This preserves the requested minimum board distance at
    every sample while retaining a genuine non-planar loop.
    """

    if amplitude_m <= 0.0:
        raise ValueError("amplitude_m must be positive")

    def point(u: float) -> base.Point:
        theta = base._loop_parameter(u)
        x, y = base._paper_xy("space", theta, 0.05)
        # One-sided excursion: z ∈ [-(height+amplitude), -height].
        z = -(height_m + amplitude_m * (0.5 - 0.5 * math.sin(2.0 * theta)))
        return x, y, z

    rows: list[dict[str, object]] = []
    tick = 0
    first = point(0.0)
    tick = base._append_segment(
        rows,
        tick,
        base.TOTAL_DURATION_S - base.LOOP_MOTION_DURATION_S - base.HOLD_DURATION_S,
        lambda _u: first,
    )
    tick = base._append_segment(rows, tick, base.LOOP_MOTION_DURATION_S, point)
    base._append_hold(rows, tick)
    return rows


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _write_immutable(path: Path, payload: bytes) -> str:
    digest = _sha256(payload)
    if path.exists():
        if path.is_symlink() or not path.is_file() or path.read_bytes() != payload:
            raise FileExistsError(f"refusing to overwrite differing authority: {path}")
    else:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("xb") as stream:
            stream.write(payload)
    return digest


def generate(
    output_directory: Path, manifest_path: Path, *, amplitude_m: float, tag: str
) -> tuple[Path, ...]:
    output_directory = output_directory.expanduser().resolve()
    manifest_path = manifest_path.expanduser().resolve()
    paths: list[Path] = []
    entries: list[dict[str, object]] = []
    for height_cm in HEIGHTS_CM:
        trajectory_id = f"icassp_dynamic_depth_loop_{tag}_z{height_cm:02d}"
        rows = _safe_route(height_cm / 100.0, amplitude_m)
        diagnostics = base._validate_rows(trajectory_id, rows)
        payload = base._canonical_csv(rows)
        path = output_directory / f"{trajectory_id}.csv"
        digest = _write_immutable(path, payload)
        diagnostics.update(
            {
                "authority_file": path.name,
                "authority_sha256": digest,
                "shape": "depth_loop",
                "nominal_height_cm": height_cm,
                "frame_id": FRAME_ID,
                "orientation_xyzw": list(base.QUATERNION_XYZW),
                "xy_radius_m": 0.05,
                "depth_amplitude_m": amplitude_m,
                "depth_policy": "one_sided_farther_from_sensor",
                "coordinate_mapping": "tip_z_m = -z_A; x/y unchanged",
            }
        )
        entries.append(diagnostics)
        paths.append(path)
        print(f"{path}: sha256={digest}")

    manifest = {
        "schema_version": 1,
        "manifest_id": f"icassp_dynamic_depth_loop_{tag}",
        "environment_id": "mla",
        "frame_id": FRAME_ID,
        "shape": "depth_loop",
        "nominal_heights_cm": list(HEIGHTS_CM),
        "geometry": {
            "xy_radius_m": 0.05,
            "depth_amplitude_m": amplitude_m,
            "depth_policy": "one_sided_farther_from_sensor",
        },
        "timing": {
            "sample_dt_s": base.DT_S,
            "duration_s": base.TOTAL_DURATION_S,
            "final_hold_duration_s": base.HOLD_DURATION_S,
            "parameterization": "quintic minimum jerk per loop segment",
        },
        "authority_policy": (
            "waypoints and time_s are immutable hard constraints; no runtime "
            "scaling, retiming, clipping, or point dropping"
        ),
        "trajectories": entries,
    }
    _write_immutable(
        manifest_path,
        (json.dumps(manifest, indent=2) + "\n").encode("utf-8"),
    )
    print(f"manifest: {manifest_path}")
    return tuple(paths)


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-directory",
        type=Path,
        default=root / "environments" / "mla" / "trajectories",
    )
    parser.add_argument("--manifest", type=Path, default=None)
    parser.add_argument("--amplitude-m", type=float, default=0.01)
    parser.add_argument("--tag", default="safe10mm")
    args = parser.parse_args()
    output = args.output_directory.expanduser().resolve()
    manifest = (
        (
            args.manifest
            or output / f"icassp_dynamic_depth_loop_{args.tag}_manifest.json"
        )
        .expanduser()
        .resolve()
    )
    generate(output, manifest, amplitude_m=args.amplitude_m, tag=args.tag)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
