#!/usr/bin/env python3
"""Clone the last validated F64 authorities at new board-frame heights.

Only ``tip_z_m`` is changed.  The source time grid, x/y samples, quaternion,
and phase labels are copied verbatim; this is an upstream authority
transformation, not a runtime correction.  Existing files are never
overwritten with different bytes.

The default set is deliberately small and matches the requested comparison:

* one horizontal F64 figure-eight at z_A = 7, 9, 11, 13 cm;
* orthogonal (X and Y) vertical straight scans at z_A = 7..14 cm.

In the plane1 convention used by ``mla``, z_A is the physical distance above
the sensor and therefore ``tip_z_m = -z_A``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TRAJECTORY_DIR = ROOT / "environments" / "mla" / "trajectories"
CSV_COLUMNS = (
    "time_s",
    "frame_id",
    "tip_x_m",
    "tip_y_m",
    "tip_z_m",
    "tip_qx",
    "tip_qy",
    "tip_qz",
    "tip_qw",
    "phase",
)

FIGURE8_HEIGHTS_CM = (7, 9, 11, 13)
LINE_HEIGHTS_CM = tuple(range(7, 15))


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def write_immutable(path: Path, payload: bytes) -> str:
    digest = sha256_bytes(payload)
    if path.exists():
        if path.is_symlink() or not path.is_file() or path.read_bytes() != payload:
            raise FileExistsError(f"refusing to overwrite differing authority: {path}")
    else:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)
    return digest


def _read_lines(path: Path) -> tuple[list[str], list[list[str]]]:
    raw = path.read_text(encoding="ascii")
    if not raw.endswith("\n"):
        raise ValueError(f"{path.name}: authority must end with a newline")
    lines = raw[:-1].split("\n")
    header = lines[0].split(",")
    if tuple(header) != CSV_COLUMNS:
        raise ValueError(f"{path.name}: unexpected authority header")
    rows: list[list[str]] = []
    previous_time = -math.inf
    for line_number, line in enumerate(lines[1:], start=2):
        fields = line.split(",")
        if len(fields) != len(CSV_COLUMNS):
            raise ValueError(f"{path.name}: malformed row {line_number}")
        try:
            time_s = float(fields[0])
            values = [float(fields[index]) for index in (2, 3, 4, 5, 6, 7, 8)]
        except ValueError as error:
            raise ValueError(f"{path.name}: non-numeric value at row {line_number}") from error
        if not math.isfinite(time_s) or time_s <= previous_time:
            raise ValueError(f"{path.name}: time_s is not strictly increasing")
        previous_time = time_s
        if fields[1] != "plane1":
            raise ValueError(f"{path.name}: frame_id is not plane1")
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{path.name}: non-finite pose at row {line_number}")
        q_norm = math.sqrt(sum(float(fields[index]) ** 2 for index in (5, 6, 7, 8)))
        if abs(q_norm - 1.0) > 2.0e-9:
            raise ValueError(f"{path.name}: quaternion is not normalized at row {line_number}")
        if fields[9] not in {"draw", "hold"}:
            raise ValueError(f"{path.name}: invalid phase at row {line_number}")
        rows.append(fields)
    if len(rows) < 2:
        raise ValueError(f"{path.name}: too few rows")
    if float(rows[0][0]) != 0.0:
        raise ValueError(f"{path.name}: first time_s is not zero")
    # A source hold, if present, must already be static.  We do not repair it.
    for previous, current in zip(rows, rows[1:]):
        if current[9] == "hold" and current[2:9] != previous[2:9]:
            raise ValueError(f"{path.name}: hold row is not an exact pose repeat")
    return lines, rows


def _clone_at_height(
    template: Path,
    destination: Path,
    height_cm: int,
) -> dict[str, object]:
    lines, rows = _read_lines(template)
    source_z = {row[4] for row in rows}
    if len(source_z) != 1:
        raise ValueError(f"{template.name}: source z is not constant")
    tip_z_m = -height_cm / 100.0
    replacement = f"{tip_z_m:.9f}"
    cloned = [lines[0]]
    for row in rows:
        out = list(row)
        out[4] = replacement
        cloned.append(",".join(out))
    payload = ("\n".join(cloned) + "\n").encode("ascii")
    authority_sha256 = write_immutable(destination, payload)
    return {
        "authority_file": destination.name,
        "authority_sha256": authority_sha256,
        "source_template": template.name,
        "source_sha256": sha256_bytes(template.read_bytes()),
        "height_cm": height_cm,
        "tip_z_m": replacement,
        "source_row_count": len(rows),
        "duration_s": float(rows[-1][0]),
        "sample_dt_s": float(rows[1][0]) - float(rows[0][0]),
        "frame_id": "plane1",
        "z_only_change": True,
    }


def generate(output_directory: Path, manifest_path: Path, index_path: Path) -> tuple[Path, ...]:
    output_directory = output_directory.expanduser().resolve()
    manifest_path = manifest_path.expanduser().resolve()
    index_path = index_path.expanduser().resolve()

    figure8_template = output_directory / "icassp_f64_mj_feasible_v2_eight_tilt90_z13.csv"
    line_x_template = output_directory / "icassp_f64_mj_feasible_v2_line_x_vertical_z09.csv"
    line_y_template = output_directory / "icassp_f64_mj_feasible_v2_line_y_vertical_z09.csv"
    for template in (figure8_template, line_x_template, line_y_template):
        if not template.is_file():
            raise FileNotFoundError(f"missing validated template: {template}")

    entries: list[dict[str, object]] = []
    paths: list[Path] = []
    for height_cm in FIGURE8_HEIGHTS_CM:
        destination = output_directory / (
            f"icassp_f64_mj_height_sweep_eight_tilt90_z{height_cm:02d}.csv"
        )
        entry = _clone_at_height(figure8_template, destination, height_cm)
        entry.update({"kind": "horizontal_figure8", "axis": "xy"})
        entries.append(entry)
        paths.append(destination)
    for axis, template in (("x", line_x_template), ("y", line_y_template)):
        for height_cm in LINE_HEIGHTS_CM:
            destination = output_directory / (
                f"icassp_f64_mj_height_sweep_line_{axis}_vertical_z{height_cm:02d}.csv"
            )
            entry = _clone_at_height(template, destination, height_cm)
            entry.update({"kind": "straight_scan", "axis": axis})
            entries.append(entry)
            paths.append(destination)

    manifest = {
        "schema_version": 1,
        "manifest_id": "icassp_f64_mj_height_sweep_z07_z14",
        "environment_id": "mla",
        "frame_id": "plane1",
        "coordinate_mapping": "tip_x_m=x_A, tip_y_m=y_A, tip_z_m=-z_A",
        "source_policy": (
            "clone the last validated F64 authority and change only tip_z_m; "
            "time_s, x/y, quaternion and phase are immutable"
        ),
        "figure8_heights_cm": list(FIGURE8_HEIGHTS_CM),
        "line_heights_cm": list(LINE_HEIGHTS_CM),
        "route_count": len(entries),
        "routes": entries,
    }
    manifest_payload = (json.dumps(manifest, indent=2) + "\n").encode("utf-8")
    write_immutable(manifest_path, manifest_payload)

    index_fields = (
        "order",
        "kind",
        "axis",
        "height_cm",
        "authority_file",
        "authority_sha256",
        "source_template",
        "duration_s",
        "sample_dt_s",
    )
    index_lines = [",".join(index_fields)]
    for order, entry in enumerate(entries):
        index_lines.append(
            ",".join(
                str(order) if field == "order" else str(entry[field])
                for field in index_fields
            )
        )
    write_immutable(index_path, ("\n".join(index_lines) + "\n").encode("utf-8"))
    return tuple(paths)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-directory", type=Path, default=DEFAULT_TRAJECTORY_DIR)
    parser.add_argument("--manifest", type=Path, default=None)
    parser.add_argument("--index", type=Path, default=None)
    args = parser.parse_args()
    output_directory = args.output_directory.expanduser().resolve()
    manifest = args.manifest or output_directory / "icassp_f64_mj_height_sweep_authority_manifest.json"
    index = args.index or output_directory / "icassp_f64_mj_height_sweep_authority_index.csv"
    paths = generate(output_directory, manifest, index)
    for path in paths:
        print(path)
    print(f"manifest: {manifest}")
    print(f"index: {index}")
    print(f"route_count: {len(paths)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
