#!/usr/bin/env python3
"""Find one collision-valid q-sign/roll realization per failed Phase-2 direction."""

from __future__ import annotations

import argparse
import csv
import io
import math
import os
import tempfile
from collections import defaultdict
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np

from hx_waypoint_curobo.curobo_adapter_v2 import CuroboAdapterV2
from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_curobo.errors import PlanningError
from hx_waypoint_curobo.models import TimedPose
from hx_waypoint_curobo.robot_tool_builder import RobotToolBuilder
from hx_waypoint_tools.icassp_phase2 import (
    PHASE2_FRAME_ID,
    PHASE2_TCP_FLOOR_Z_M,
    _inverse_se3,
    _load_registration,
    _quaternion_xyzw_to_rotation,
    _rotation_to_quaternion_xyzw,
)


class RealizationSearchError(RuntimeError):
    """The realization search cannot proceed without weakening its contract."""


def _read_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or len(reader.fieldnames) != len(set(reader.fieldnames)):
            raise RealizationSearchError(f"invalid CSV header: {path}")
        rows = []
        for line_number, row in enumerate(reader, start=2):
            if None in row or any(value is None for value in row.values()):
                raise RealizationSearchError(f"invalid CSV row {line_number}: {path}")
            rows.append({str(key): str(value) for key, value in row.items()})
    if not rows:
        raise RealizationSearchError(f"empty CSV: {path}")
    return list(reader.fieldnames), rows


def _latest_failures(
    build_index: Path,
    conditions_by_id: Mapping[str, Mapping[str, str]],
) -> set[str]:
    _, events = _read_csv(build_index)
    latest: dict[str, dict[str, str]] = {}
    for event in events:
        latest[event["condition_id"]] = event
    if set(latest) != set(conditions_by_id):
        missing = sorted(set(conditions_by_id).difference(latest))
        raise RealizationSearchError(
            "first-pass index does not cover every condition: " + ",".join(missing)
        )
    return {
        conditions_by_id[condition_id]["direction_id"]
        for condition_id, event in latest.items()
        if event["status"] not in {"success", "recovered"}
    }


def _rotation_x_pi() -> np.ndarray:
    return np.diag((1.0, -1.0, -1.0))


def _rotation_z(degrees: float) -> np.ndarray:
    angle = math.radians(float(degrees))
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.asarray(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=float,
    )


def _candidate_order(step_deg: int) -> tuple[tuple[int, float], ...]:
    if step_deg <= 0 or 360 % step_deg != 0:
        raise RealizationSearchError("--roll-step-deg must be a positive divisor of 360")
    # Prefer preserving the measured polarity, then try the theoretically
    # equivalent -q family. Within each family test small positive/negative
    # roll changes first and include 180 degrees only once.
    signed_rolls: list[float] = [0.0]
    for magnitude in range(step_deg, 180, step_deg):
        signed_rolls.extend((float(magnitude), float(-magnitude)))
    signed_rolls.append(180.0)
    output = []
    for sign in (1, -1):
        for roll in signed_rolls:
            if sign == 1 and roll == 0:
                continue  # the first-pass bundle already tested this realization
            output.append((sign, float(roll)))
    return tuple(output)


def _array_magnet_rotation(
    original_rotation_A_M: np.ndarray,
    sign: int,
    roll_deg: float,
) -> np.ndarray:
    sign_flip = np.eye(3) if sign == 1 else _rotation_x_pi()
    rotation = original_rotation_A_M @ sign_flip @ _rotation_z(roll_deg)
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-9, rtol=0.0):
        raise RealizationSearchError("candidate rotation is not orthonormal")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1.0e-9):
        raise RealizationSearchError("candidate rotation is not proper")
    return rotation


def _tip_pose(
    row: Mapping[str, str],
    rotation_A_M: np.ndarray,
    transform_P_A: np.ndarray,
    transform_R_M: np.ndarray,
) -> TimedPose:
    transform_A_M = np.eye(4)
    transform_A_M[:3, :3] = rotation_A_M
    transform_A_M[:3, 3] = [
        float(row["p_A_x_m"]),
        float(row["p_A_y_m"]),
        float(row["p_A_z_m"]),
    ]
    transform_P_R = transform_P_A @ transform_A_M @ _inverse_se3(transform_R_M)
    position = tuple(float(value) for value in transform_P_R[:3, 3])
    if position[2] > PHASE2_TCP_FLOOR_Z_M + 1.0e-12:
        raise RealizationSearchError(
            f"{row['condition_id']} violates z_rodtip <= {PHASE2_TCP_FLOOR_Z_M}"
        )
    quaternion = _rotation_to_quaternion_xyzw(transform_P_R[:3, :3])
    return TimedPose(
        time_s=0.0,
        frame_id=PHASE2_FRAME_ID,
        position_m=position,  # type: ignore[arg-type]
        quaternion_xyzw=quaternion,
        phase="draw",
    )


def _screen_candidate(
    rows: Sequence[Mapping[str, str]],
    rotation_A_M: np.ndarray,
    transform_P_A: np.ndarray,
    transform_R_M: np.ndarray,
    environment,
    adapter: CuroboAdapterV2,
) -> tuple[bool, float, float, float, str]:
    tool_builder = RobotToolBuilder(environment)
    minimum_clearance = math.inf
    minimum_joint_margin = math.inf
    maximum_singularity = -math.inf
    # Test the historically failed/farthest rows first to reject bad candidates quickly.
    ordered_rows = sorted(rows, key=lambda row: float(row["p_A_z_m"]), reverse=True)
    for row in ordered_rows:
        tip = _tip_pose(row, rotation_A_M, transform_P_A, transform_R_M)
        base_tip = environment.transform_pose_to_base(tip)
        tool_pose = tool_builder.tip_pose_to_tool_pose(base_tip)
        try:
            candidates = adapter.solve_ik_candidates((tool_pose,), 32)[0]
        except PlanningError as error:
            return False, math.nan, math.nan, math.nan, str(error)
        if not candidates:
            return False, math.nan, math.nan, math.nan, "no IK candidates"
        best = candidates[0]
        minimum_clearance = min(minimum_clearance, best.collision_distance_m)
        minimum_joint_margin = min(minimum_joint_margin, best.joint_limit_margin_rad)
        maximum_singularity = max(maximum_singularity, best.singularity_score)
    return True, minimum_clearance, minimum_joint_margin, maximum_singularity, ""


def _render_csv(fieldnames: Sequence[str], rows: Sequence[Mapping[str, str]]) -> bytes:
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue().encode("utf-8")


def _write_new_or_same(path: Path, content: bytes) -> None:
    if path.exists():
        if path.is_symlink() or not path.is_file() or path.read_bytes() != content:
            raise RealizationSearchError(f"refusing to overwrite differing output: {path}")
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def search(args: argparse.Namespace) -> None:
    condition_fields, condition_rows = _read_csv(args.conditions)
    conditions_by_id = {row["condition_id"]: row for row in condition_rows}
    if len(conditions_by_id) != len(condition_rows):
        raise RealizationSearchError("condition_id values must be unique")
    failed_directions = _latest_failures(args.build_index, conditions_by_id)
    rows_by_direction: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in condition_rows:
        rows_by_direction[row["direction_id"]].append(row)

    registration_content = args.registration.read_bytes()
    transform_P_A, transform_R_M, _ = _load_registration(
        registration_content,
        exploratory=True,
    )
    environment = EnvironmentBuilder().load(args.environment_directory)
    adapter = CuroboAdapterV2(environment, args.workspace / "curobo.lock")
    print("warming locked cuRobo runtime", flush=True)
    adapter.warmup()

    realization: dict[str, tuple[int, float, np.ndarray, dict[str, str]]] = {}
    catalog_rows: list[dict[str, str]] = []
    candidate_order = _candidate_order(args.roll_step_deg)
    for direction_id in sorted(rows_by_direction):
        direction_rows = rows_by_direction[direction_id]
        original_quaternion = [
            float(direction_rows[0][f"quat_A_M_{axis}"]) for axis in "xyzw"
        ]
        original_rotation = _quaternion_xyzw_to_rotation(original_quaternion)
        if direction_id not in failed_directions:
            rotation = original_rotation
            metrics = {
                "candidate_attempts": "0",
                "minimum_clearance_m": "",
                "minimum_joint_margin_rad": "",
                "maximum_singularity_score": "",
                "last_failure": "",
            }
            realization[direction_id] = (1, 0.0, rotation, metrics)
            continue

        chosen = None
        last_failure = ""
        for attempt, (sign, roll_deg) in enumerate(candidate_order, start=1):
            rotation = _array_magnet_rotation(original_rotation, sign, roll_deg)
            passed, clearance, joint_margin, singularity, failure = _screen_candidate(
                direction_rows,
                rotation,
                transform_P_A,
                transform_R_M,
                environment,
                adapter,
            )
            print(
                f"{direction_id}: attempt={attempt} sign={sign:+d} "
                f"roll={roll_deg:+.0f} pass={passed}",
                flush=True,
            )
            if passed:
                chosen = (
                    sign,
                    roll_deg,
                    rotation,
                    {
                        "candidate_attempts": str(attempt),
                        "minimum_clearance_m": f"{clearance:.12g}",
                        "minimum_joint_margin_rad": f"{joint_margin:.12g}",
                        "maximum_singularity_score": f"{singularity:.12g}",
                        "last_failure": last_failure,
                    },
                )
                break
            last_failure = failure
        if chosen is None:
            raise RealizationSearchError(
                f"no common sign/roll realization found for {direction_id}: {last_failure}"
            )
        realization[direction_id] = chosen

    extra_fields = ["implemented_q_sign", "free_roll_deg", "realization_policy"]
    output_fields = [*condition_fields, *(name for name in extra_fields if name not in condition_fields)]
    output_rows = []
    for source in condition_rows:
        row = dict(source)
        sign, roll_deg, rotation, metrics = realization[row["direction_id"]]
        quaternion = _rotation_to_quaternion_xyzw(rotation)
        moment = rotation[:, 2]
        for axis_index, axis in enumerate("xyz"):
            row[f"moment_q_A_{axis}"] = f"{moment[axis_index]:.12f}"
        for axis_index, axis in enumerate("xyzw"):
            row[f"quat_A_M_{axis}"] = f"{quaternion[axis_index]:.12f}"
        row["implemented_q_sign"] = f"{sign:+d}"
        row["free_roll_deg"] = f"{roll_deg:.6f}"
        row["realization_policy"] = "one_sign_roll_per_direction_all_heights"
        output_rows.append(row)

    for direction_id in sorted(rows_by_direction):
        sign, roll_deg, _, metrics = realization[direction_id]
        catalog_rows.append(
            {
                "direction_id": direction_id,
                "condition_count": str(len(rows_by_direction[direction_id])),
                "first_pass_had_failure": str(direction_id in failed_directions).lower(),
                "implemented_q_sign": f"{sign:+d}",
                "free_roll_deg": f"{roll_deg:.6f}",
                "candidate_attempts": metrics["candidate_attempts"],
                "minimum_clearance_m": metrics["minimum_clearance_m"],
                "minimum_joint_margin_rad": metrics["minimum_joint_margin_rad"],
                "maximum_singularity_score": metrics["maximum_singularity_score"],
                "last_failure": metrics["last_failure"].replace("\n", " "),
                "status": "EXPLORATORY_REALIZATION_SELECTED",
            }
        )

    catalog_fields = (
        "direction_id",
        "condition_count",
        "first_pass_had_failure",
        "implemented_q_sign",
        "free_roll_deg",
        "candidate_attempts",
        "minimum_clearance_m",
        "minimum_joint_margin_rad",
        "maximum_singularity_score",
        "last_failure",
        "status",
    )
    _write_new_or_same(args.output_conditions, _render_csv(output_fields, output_rows))
    _write_new_or_same(args.output_catalog, _render_csv(catalog_fields, catalog_rows))
    print(f"wrote conditions: {args.output_conditions}")
    print(f"wrote catalog: {args.output_catalog}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--environment-directory", type=Path, required=True)
    parser.add_argument("--conditions", type=Path, required=True)
    parser.add_argument("--registration", type=Path, required=True)
    parser.add_argument("--build-index", type=Path, required=True)
    parser.add_argument("--output-conditions", type=Path, required=True)
    parser.add_argument("--output-catalog", type=Path, required=True)
    parser.add_argument("--roll-step-deg", type=int, default=45)
    args = parser.parse_args()
    try:
        search(args)
    except (OSError, ValueError, RealizationSearchError) as error:
        parser.exit(2, f"search_icassp_phase2_realizations.py: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
