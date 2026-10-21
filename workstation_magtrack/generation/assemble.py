"""Freeze a portable authority index without rewriting any generated CSV."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from .common import PROTOCOL, write_json_immutable


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def assemble(root: Path, active_manifest: Path, revision: str = "") -> Path:
    root = root.resolve()
    calibration = json.loads((root / "planning/calibration_manifest.json").read_text())
    active = json.loads(active_manifest.read_text())["active_records"]
    # Inspect conservative center/high poses before lower and outer poses.
    calibration.sort(key=lambda r: (
        r["category"] != "calibration", -r.get("nominal_height_m", 0.0),
        sum(abs(v) for v in r.get("diagnostics", {}).get("position_min_m", [])[:2]),
        r["trajectory_id"],
    ))
    records = []
    seen: set[str] = set()
    for original in calibration + active:
        record = dict(original)
        name = record["trajectory_id"]
        if name in seen:
            raise ValueError(f"duplicate authority ID: {name}")
        seen.add(name)
        source = root / "environments/mla_short/trajectories" / f"{name}.csv"
        if sha256(source) != record["source_sha256"]:
            raise ValueError(f"source hash differs from generator manifest: {name}")
        record["source_file"] = str(source.relative_to(root))
        record.update(candidate_lifecycle_state="active", simulation_only=True,
                      real_execution_authorized=False,
                      accepted_calibration_id=None,
                      installation_binding_state="pending_physical_acceptance",
                      coordinate_frame="plane1", quaternion_order_csv="xyzw")
        records.append(record)
    if len(records) != 96:
        raise ValueError(f"expected 27 calibration + 6 sync + 3 commissioning + 60 pilot, got {len(records)}")
    index = {
        "schema_version": 1, "protocol_version": PROTOCOL,
        "environment_id": "mla_short", "active_manifest": str(active_manifest.relative_to(root)),
        "trajectory_count": len(records), "trajectories": records,
        "planned_unique_duration_s": sum(r["duration_s"] for r in records),
        "calibration_repeat_schedule": "planning/calibration_repeat_schedule.json",
        "authority_contract": "All source rows and time_s are hard constraints; no runtime scaling, retiming, cropping, skipping",
        "real_execution_authorized": False,
    }
    if revision and (not revision.startswith("_v") or not revision[2:].isdigit()):
        raise ValueError("revision must be empty or _vN")
    output = root / f"planning/authority_index{revision}.json"
    write_json_immutable(output, index)
    snapshot = {
        "protocol_version": PROTOCOL, "environment_id": "mla_short",
        "environment_files_sha256": {
            str(p.relative_to(root)): sha256(p)
            for p in sorted((root / "environments/mla_short").rglob("*"))
            if p.is_file() and p.relative_to(root / "environments/mla_short").parts[0]
            not in {"trajectories", "recordings"}
        },
        "inputs_sha256": {str(p.relative_to(root)): sha256(p)
                           for p in sorted((root / "inputs").glob("*")) if p.is_file()},
        "curobo_lock_sha256": sha256(root / "curobo.lock"),
        "authority_index_sha256": sha256(output),
        "nominal_tool": {"rod_length_m": .190, "rod_diameter_m": .005},
        "height_convention": "plane1 negative Z physically upward, h=-z",
        "label_center_type": "nominal rod_tip, not yet accepted magnet reference center",
        "array_frame_mapping": "pending; plane1 visual board center not automatically hospital S origin",
        "accepted_R_S_B_t_S_B_r_F_C": None,
        "actual_controller_TCP": "not queried; no real robot accessed",
        "coarse_uncertainty_not_measured": {"position_m": .003, "orientation_deg": 2.0},
        "simulation_host": "ubuntu24",
        "simulation_parent_commit": "a258cfeb4fa3ed8f1c78878f927e3ba4dfec399a",
        "simulation_software": {"python": "3.12.3", "torch": "2.11.0+cu128",
                                "numpy": "2.1.3", "scipy": "1.15.3", "ros": "jazzy",
                                "gpu": "RTX 3090 24GB", "driver": "570.172.08"},
        "execution_scope": "isolated mock simulation only; no sensor, background, or physical robot",
    }
    write_json_immutable(root / f"planning/frozen_nominal_snapshot{revision}.json", snapshot)
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path("workstation_magtrack"))
    parser.add_argument("--active-manifest", required=True, type=Path)
    parser.add_argument("--revision", default="", help="new immutable index suffix, e.g. _v4")
    args = parser.parse_args()
    print(assemble(args.root, args.active_manifest.resolve(), args.revision))


if __name__ == "__main__":
    main()
