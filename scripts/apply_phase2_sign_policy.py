#!/usr/bin/env python3
"""Apply only validated q-sign/free-roll realizations to a Phase-2 input CSV.

The diagnostic CSV is treated as evidence, not as a source of new waypoints.
Only a candidate explicitly marked ``passed=true`` is applied; directions with
no common candidate retain their original quaternion and are marked unresolved.
The output is written atomically and an existing different file is never
overwritten.  This produces an exploratory input bundle for offline screening;
it does not generate or mutate authorities itself.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import math
import os
import tempfile
from collections import defaultdict
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np

from hx_waypoint_tools.icassp_phase2 import (
    _quaternion_xyzw_to_rotation,
    _rotation_to_quaternion_xyzw,
)


EXTRA_FIELDS = (
    "implemented_q_sign",
    "free_roll_deg",
    "realization_status",
    "realization_diagnostic_sha256",
)


def _read(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError(f"CSV is missing a header: {path}")
        fields = list(reader.fieldnames)
        if len(fields) != len(set(fields)):
            raise ValueError(f"CSV header contains duplicate fields: {path}")
        rows: list[dict[str, str]] = []
        for row in reader:
            if None in row or any(value is None for value in row.values()):
                raise ValueError(f"malformed CSV row in {path}")
            rows.append({str(k): str(v) for k, v in row.items()})
    return fields, rows


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _rotation_x_pi() -> np.ndarray:
    return np.diag((1.0, -1.0, -1.0))


def _rotation_z(degrees: float) -> np.ndarray:
    angle = math.radians(float(degrees))
    c, s = math.cos(angle), math.sin(angle)
    return np.asarray(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)), dtype=float)


def _apply_rotation(rotation: np.ndarray, sign: int, roll_deg: float) -> np.ndarray:
    if sign not in {-1, 1}:
        raise ValueError(f"q sign must be +/-1, got {sign}")
    result = rotation @ (_rotation_x_pi() if sign == -1 else np.eye(3)) @ _rotation_z(roll_deg)
    if not np.allclose(result.T @ result, np.eye(3), atol=1e-9, rtol=0.0):
        raise ValueError("realization rotation is not orthonormal")
    if not math.isclose(float(np.linalg.det(result)), 1.0, abs_tol=1e-9):
        raise ValueError("realization rotation is not proper")
    return result


def _render(fields: Sequence[str], rows: Sequence[Mapping[str, str]]) -> bytes:
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=list(fields), lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue().encode("utf-8")


def _write_new_or_same(path: Path, content: bytes) -> None:
    if path.exists():
        if path.is_symlink() or not path.is_file() or path.read_bytes() != content:
            raise ValueError(f"refusing to overwrite differing output: {path}")
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--conditions", type=Path, required=True)
    parser.add_argument("--diagnostic", type=Path, required=True)
    parser.add_argument("--output-conditions", type=Path, required=True)
    parser.add_argument("--output-catalog", type=Path, required=True)
    args = parser.parse_args()

    fields, source_rows = _read(args.conditions)
    if "condition_id" not in fields or "direction_id" not in fields:
        raise ValueError("conditions CSV must contain condition_id and direction_id")
    if len({row["condition_id"] for row in source_rows}) != len(source_rows):
        raise ValueError("conditions contain duplicate condition_id values")
    diagnostic_fields, diagnostic_rows = _read(args.diagnostic)
    required = {"direction_id", "sign", "roll_deg", "passed"}
    missing = required.difference(diagnostic_fields)
    if missing:
        raise ValueError("diagnostic CSV is missing: " + ",".join(sorted(missing)))
    by_direction: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in diagnostic_rows:
        by_direction[row["direction_id"]].append(row)
    source_directions = {row["direction_id"] for row in source_rows}
    if set(by_direction) != source_directions:
        missing_directions = sorted(source_directions.difference(by_direction))
        extra_directions = sorted(set(by_direction).difference(source_directions))
        raise ValueError(
            "diagnostic direction set mismatch; missing="
            + ",".join(missing_directions)
            + "; extra="
            + ",".join(extra_directions)
        )
    diagnostic_sha = _sha256(args.diagnostic)

    selected: dict[str, tuple[int, float]] = {}
    status_by_direction: dict[str, str] = {}
    catalog_rows: list[dict[str, str]] = []
    for direction_id in sorted(by_direction):
        rows = by_direction[direction_id]
        passed = [row for row in rows if row["passed"].strip().lower() == "true"]
        if len(passed) > 1:
            raise ValueError(
                f"diagnostic has multiple passing realizations for {direction_id}; "
                "resolve ambiguity before applying"
            )
        if passed:
            candidate = passed[0]
            try:
                sign = int(candidate["sign"])
                roll = float(candidate["roll_deg"])
            except (TypeError, ValueError) as error:
                raise ValueError(f"invalid passing candidate for {direction_id}") from error
            if sign not in {-1, 1} or not math.isfinite(roll):
                raise ValueError(f"invalid passing sign/roll for {direction_id}")
            selected[direction_id] = (sign, roll)
            status = "COMMON_SIGN_ROLL_SELECTED"
        else:
            status = "UNRESOLVED_RETAIN_ORIGINAL"
        status_by_direction[direction_id] = status
        catalog_rows.append(
            {
                "direction_id": direction_id,
                "candidate_count": str(len(rows)),
                "passing_candidate_count": str(len(passed)),
                "implemented_q_sign": str(selected[direction_id][0]) if passed else "",
                "free_roll_deg": f"{selected[direction_id][1]:.6f}" if passed else "",
                "status": status,
                "diagnostic_sha256": diagnostic_sha,
            }
        )

    output_fields = [*fields, *(field for field in EXTRA_FIELDS if field not in fields)]
    output_rows: list[dict[str, str]] = []
    changed_directions: set[str] = set()
    for source in source_rows:
        row = dict(source)
        direction_id = row["direction_id"]
        original_q = [float(row[f"quat_A_M_{axis}"]) for axis in "xyzw"]
        original_rotation = _quaternion_xyzw_to_rotation(original_q)
        sign, roll = selected.get(direction_id, (1, 0.0))
        rotation = _apply_rotation(original_rotation, sign, roll)
        quaternion = _rotation_to_quaternion_xyzw(rotation)
        moment = rotation[:, 2]
        for index, axis in enumerate("xyz"):
            row[f"moment_q_A_{axis}"] = f"{moment[index]:.12f}"
        for index, axis in enumerate("xyzw"):
            row[f"quat_A_M_{axis}"] = f"{quaternion[index]:.12f}"
        row["implemented_q_sign"] = f"{sign:+d}"
        row["free_roll_deg"] = f"{roll:.6f}"
        row["realization_status"] = status_by_direction[direction_id]
        row["realization_diagnostic_sha256"] = diagnostic_sha
        if sign != 1 or abs(roll) > 1.0e-12:
            changed_directions.add(direction_id)
        output_rows.append(row)

    catalog_fields = (
        "direction_id",
        "candidate_count",
        "passing_candidate_count",
        "implemented_q_sign",
        "free_roll_deg",
        "status",
        "diagnostic_sha256",
    )
    catalog_bytes = _render(catalog_fields, catalog_rows)
    conditions_bytes = _render(output_fields, output_rows)
    _write_new_or_same(args.output_conditions, conditions_bytes)
    _write_new_or_same(args.output_catalog, catalog_bytes)
    print(f"wrote conditions: {args.output_conditions}")
    print(f"wrote catalog: {args.output_catalog}")
    print("changed directions: " + (",".join(sorted(changed_directions)) or "none"))
    print(
        "unresolved directions: "
        + ",".join(sorted(d for d, status in status_by_direction.items() if status != "COMMON_SIGN_ROLL_SELECTED"))
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
