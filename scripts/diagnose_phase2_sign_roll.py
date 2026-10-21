#!/usr/bin/env python3
"""Diagnose q-sign/free-roll feasibility without writing authorities.

This is deliberately a diagnostic companion to ``search_icassp_phase2_realizations``:
it evaluates candidates and emits a CSV of evidence, but never changes a condition,
authority, registration, environment, or recording.  A candidate is marked pass
only when the same sign/roll works at every height belonging to that direction.
"""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

from hx_waypoint_curobo.curobo_adapter_v2 import CuroboAdapterV2
from hx_waypoint_curobo.environment_builder import EnvironmentBuilder

# Import private helpers from the existing search implementation so this tool
# uses exactly the production transform and screening semantics.
from search_icassp_phase2_realizations import (  # type: ignore[import-not-found]
    _array_magnet_rotation,
    _read_csv,
    _screen_candidate,
)
from hx_waypoint_tools.icassp_phase2 import _load_registration


def _candidate_list(step: int) -> list[tuple[int, float]]:
    if step <= 0 or 360 % step:
        raise ValueError("roll step must be a positive divisor of 360")
    # Test the requested equivalence first, then progressively enumerate roll.
    values = [0.0]
    for magnitude in range(step, 180, step):
        values.extend((float(magnitude), float(-magnitude)))
    values.append(180.0)
    output: list[tuple[int, float]] = [(1, 0.0), (-1, 0.0)]
    for sign in (1, -1):
        for roll in values:
            if roll == 0.0:
                continue
            candidate = (sign, roll)
            if candidate not in output:
                output.append(candidate)
    return output


def _load_rows(path: Path) -> list[dict[str, str]]:
    _, rows = _read_csv(path)
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--environment-directory", type=Path, required=True)
    parser.add_argument("--conditions", type=Path, required=True)
    parser.add_argument("--registration", type=Path, required=True)
    parser.add_argument("--build-index", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--roll-step-deg", type=int, default=15)
    parser.add_argument(
        "--sign-only",
        action="store_true",
        help="test only q and -q (roll fixed at zero)",
    )
    parser.add_argument("--all-directions", action="store_true")
    parser.add_argument(
        "--direction",
        action="append",
        dest="directions",
        help="limit the diagnostic to one or more direction_id values",
    )
    args = parser.parse_args()

    condition_rows = _load_rows(args.conditions)
    _, events = _read_csv(args.build_index)
    latest = {row["condition_id"]: row for row in events}
    rows_by_direction: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in condition_rows:
        rows_by_direction[row["direction_id"]].append(row)
    if args.directions:
        unknown = sorted(set(args.directions).difference(rows_by_direction))
        if unknown:
            raise ValueError("unknown direction_id: " + ",".join(unknown))
        directions = sorted(set(args.directions))
    elif args.all_directions:
        directions = sorted(rows_by_direction)
    else:
        failed = {
            row["direction_id"]
            for row in condition_rows
            if latest.get(row["condition_id"], {}).get("status")
            not in {"success", "recovered"}
        }
        directions = sorted(failed)

    registration_content = args.registration.read_bytes()
    transform_P_A, transform_R_M, _ = _load_registration(
        registration_content, exploratory=True
    )
    environment = EnvironmentBuilder().load(args.environment_directory)
    adapter = CuroboAdapterV2(environment, args.workspace / "curobo.lock")
    print("warming locked cuRobo runtime", flush=True)
    adapter.warmup()

    candidates = [(1, 0.0), (-1, 0.0)] if args.sign_only else _candidate_list(args.roll_step_deg)
    fieldnames = (
        "direction_id", "sign", "roll_deg", "passed", "minimum_clearance_m",
        "minimum_joint_margin_rad", "maximum_singularity_score", "failure",
    )
    output_rows: list[dict[str, str]] = []
    for direction_id in directions:
        direction_rows = rows_by_direction[direction_id]
        original_quaternion = [
            float(direction_rows[0][f"quat_A_M_{axis}"]) for axis in "xyzw"
        ]
        from hx_waypoint_tools.icassp_phase2 import _quaternion_xyzw_to_rotation

        original_rotation = _quaternion_xyzw_to_rotation(original_quaternion)
        found = False
        for sign, roll_deg in candidates:
            rotation = _array_magnet_rotation(original_rotation, sign, roll_deg)
            passed, clearance, margin, singularity, failure = _screen_candidate(
                direction_rows, rotation, transform_P_A, transform_R_M, environment, adapter
            )
            row = {
                "direction_id": direction_id,
                "sign": f"{sign:+d}",
                "roll_deg": f"{roll_deg:.6f}",
                "passed": str(bool(passed)).lower(),
                "minimum_clearance_m": "" if not math.isfinite(clearance) else f"{clearance:.12g}",
                "minimum_joint_margin_rad": "" if not math.isfinite(margin) else f"{margin:.12g}",
                "maximum_singularity_score": "" if not math.isfinite(singularity) else f"{singularity:.12g}",
                "failure": failure.replace("\n", " "),
            }
            output_rows.append(row)
            print(
                f"{direction_id}: sign={sign:+d} roll={roll_deg:+.0f} pass={passed}",
                flush=True,
            )
            if passed:
                found = True
                break
        if not found:
            print(f"{direction_id}: no common sign/roll candidate", flush=True)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(output_rows)
    print(f"wrote diagnostic evidence: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
