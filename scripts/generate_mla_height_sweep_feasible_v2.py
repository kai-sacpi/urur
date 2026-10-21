#!/usr/bin/env python3
"""Build the final MLA height-sweep authority index after compile gating.

Sixteen exact-vertical candidate routes are reused unchanged.  Four candidates
that failed the compile gate are replaced by z-only clones of the last
validated F64 *near-vertical* controls.  This preserves x/y, time, quaternion,
and phase from an existing validated route; it is an upstream new authority,
never a runtime trajectory edit.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from io import StringIO
from pathlib import Path
from typing import Any

import generate_mla_height_sweep_variants as base


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TRAJECTORY_DIR = ROOT / "environments" / "mla" / "trajectories"
BASE_INDEX = "icassp_f64_mj_height_sweep_authority_index.csv"
BASE_MANIFEST = "icassp_f64_mj_height_sweep_authority_manifest.json"
OUTPUT_INDEX = "icassp_f64_mj_height_sweep_feasible_v2_authority_index.csv"
OUTPUT_MANIFEST = "icassp_f64_mj_height_sweep_feasible_v2_authority_manifest.json"

FALLBACKS = {
    ("x", 14): (
        "icassp_f64_mj_feasible_v2_line_x_vertical_z15.csv",
        "authoritative timing violated joint acceleration/jerk with exact-vertical template",
    ),
    ("y", 12): (
        "icassp_f64_mj_feasible_v2_line_y_vertical_z13.csv",
        "no clearance-valid continuous IK with exact-vertical template",
    ),
    ("y", 13): (
        "icassp_f64_mj_feasible_v2_line_y_vertical_z13.csv",
        "no clearance-valid continuous IK with exact-vertical template",
    ),
    ("y", 14): (
        "icassp_f64_mj_feasible_v2_line_y_vertical_z13.csv",
        "no clearance-valid continuous IK with exact-vertical template",
    ),
}


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def generate(trajectory_root: Path, manifest_path: Path, index_path: Path) -> tuple[Path, ...]:
    trajectory_root = trajectory_root.expanduser().resolve()
    base_index_path = trajectory_root / BASE_INDEX
    base_manifest_path = trajectory_root / BASE_MANIFEST
    with base_index_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != 20:
        raise ValueError(f"base height-sweep index must contain 20 routes, got {len(rows)}")

    entries: list[dict[str, Any]] = []
    outputs: list[Path] = []
    for order, row in enumerate(rows):
        kind = row["kind"]
        axis = row["axis"]
        height_cm = int(row["height_cm"])
        fallback = FALLBACKS.get((axis, height_cm)) if kind == "straight_scan" else None
        if fallback is None:
            authority = trajectory_root / row["authority_file"]
            if sha256_file(authority) != row["authority_sha256"]:
                raise ValueError(f"base authority hash mismatch: {authority}")
            entry: dict[str, Any] = {
                "order": order,
                "kind": kind,
                "axis": axis,
                "height_cm": height_cm,
                "trajectory_id": authority.stem,
                "authority_file": authority.name,
                "authority_sha256": row["authority_sha256"],
                "source_template": row["source_template"],
                "source_sha256": sha256_file(trajectory_root / row["source_template"]),
                "duration_s": float(row["duration_s"]),
                "sample_dt_s": float(row["sample_dt_s"]),
                "source_row_count": len(authority.read_text(encoding="ascii").splitlines()) - 1,
                "orientation_policy": (
                    "inherited_exactly_from_last_validated_F64_figure8"
                    if kind == "horizontal_figure8"
                    else "exact_vertical"
                ),
                "candidate_reused_unchanged": True,
            }
        else:
            template_name, failure_reason = fallback
            destination = trajectory_root / (
                "icassp_f64_mj_height_sweep_feasible_v2_"
                f"line_{axis}_near_vertical_z{height_cm:02d}.csv"
            )
            clone = base._clone_at_height(
                trajectory_root / template_name,
                destination,
                height_cm,
            )
            entry = {
                "order": order,
                "kind": kind,
                "axis": axis,
                "height_cm": height_cm,
                "trajectory_id": destination.stem,
                **clone,
                "orientation_policy": (
                    "inherited 3.75--11.25 degree smooth near-vertical waveform "
                    "from last validated F64 control"
                ),
                "candidate_reused_unchanged": False,
                "replaces_failed_candidate": row["authority_file"],
                "replacement_reason": failure_reason,
            }
            outputs.append(destination)
        entries.append(entry)

    manifest = {
        "schema_version": 1,
        "manifest_id": "icassp_f64_mj_height_sweep_feasible_v2",
        "environment_id": "mla",
        "frame_id": "plane1",
        "route_count": len(entries),
        "coordinate_mapping": "tip_x_m=x_A, tip_y_m=y_A, tip_z_m=-z_A",
        "base_candidate_manifest": BASE_MANIFEST,
        "base_candidate_manifest_sha256": sha256_file(base_manifest_path),
        "policy": (
            "each route is either the original z-only candidate or a z-only clone of an "
            "already-validated near-vertical F64 template; runtime must not scale, retime, "
            "clip, crop, substitute, or drop waypoints"
        ),
        "routes": entries,
    }
    base.write_immutable(manifest_path, (json.dumps(manifest, indent=2) + "\n").encode("utf-8"))

    fields = (
        "order",
        "kind",
        "axis",
        "height_cm",
        "trajectory_id",
        "authority_file",
        "authority_sha256",
        "source_template",
        "duration_s",
        "sample_dt_s",
        "orientation_policy",
    )
    stream = StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
    writer.writeheader()
    writer.writerows({field: entry[field] for field in fields} for entry in entries)
    base.write_immutable(index_path, stream.getvalue().encode("utf-8"))
    return tuple(outputs)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory-directory", type=Path, default=DEFAULT_TRAJECTORY_DIR)
    parser.add_argument("--manifest", type=Path, default=None)
    parser.add_argument("--index", type=Path, default=None)
    args = parser.parse_args()
    root = args.trajectory_directory.expanduser().resolve()
    manifest = args.manifest or root / OUTPUT_MANIFEST
    index = args.index or root / OUTPUT_INDEX
    outputs = generate(root, manifest, index)
    for output in outputs:
        print(output)
    print(f"manifest: {manifest}")
    print(f"index: {index}")
    print("route_count: 20; new near-vertical replacements: 4")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
