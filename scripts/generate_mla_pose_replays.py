#!/usr/bin/env python3
"""Generate immutable 3 s MLA static-pose authorities from magnetic directions.

The supplied ``q`` values are magnetic-moment directions in the sensor-board
frame, not quaternions.  The installed magnet has ``+Z_M`` opposite the rod
tip ``+Z``; the conversion below matches the frozen registration used by the
previous MLA/Phase-2 experiments.  Existing authorities are never overwritten
with different bytes.
"""

from __future__ import annotations

import csv
import hashlib
import math
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ENVIRONMENT = ROOT / "environments" / "mla"
TRAJECTORIES = ENVIRONMENT / "trajectories"
FRAME_ID = "plane1"
STATIC_ROWS = ((0.0, "draw"), (1.0, "hold"), (3.0, "hold"))


@dataclass(frozen=True)
class Condition:
    condition_id: str
    height_cm: float
    x_cm: float
    y_cm: float
    sensor_class: str
    qx: float
    qy: float
    qz: float
    tilt_deg: float
    azimuth_deg: float


CONDITIONS = (
    Condition("mla_z070_s32_f64_xp050_y000", 7.0, 5.0, 0.0, "S32/F64", 0.02799, 0.0, 0.99961, 1.60, 0.0),
    Condition("mla_z070_d16_xp050_ym050", 7.0, 5.0, -5.0, "D16", 0.27281, -0.23018, 0.93412, 20.91, -40.2),
    Condition("mla_z070_c16_xm050_yp050", 7.0, -5.0, 5.0, "C16", -0.23282, 0.23282, 0.94424, 19.22, 135.0),
    Condition("mla_z100_s32_f64_xp050_y000", 10.0, 5.0, 0.0, "S32/F64", 0.05837, 0.0, 0.99830, 3.35, 0.0),
    Condition("mla_z100_d16_xp050_ym050", 10.0, 5.0, -5.0, "D16", 0.32005, -0.24201, 0.91597, 23.66, -37.1),
    Condition("mla_z100_c16_xp050_yp050", 10.0, 5.0, 5.0, "C16", 0.29220, 0.29215, 0.91064, 24.41, 45.0),
    Condition("mla_z120_s32_f64_xp050_y000", 12.0, 5.0, 0.0, "S32/F64", 0.08116, 0.0, 0.99670, 4.66, 0.0),
    Condition("mla_z120_d16_xp050_ym050", 12.0, 5.0, -5.0, "D16", 0.32401, -0.24735, 0.91315, 24.06, -37.4),
    Condition("mla_z120_c16_xm050_yp050", 12.0, -5.0, 5.0, "C16", -0.29674, 0.29673, 0.90769, 24.81, 135.0),
    # User-provided 180-degree symmetric C16 candidates for the previously
    # unreachable (-5,+5) cm condition.  These remain separate authorities.
    Condition("mla_z120_c16_sym_xp050_ym050", 12.0, 5.0, -5.0, "C16", 0.29673, -0.29673, 0.90769, 24.81, -45.0),
    Condition("mla_z120_c16_sym_xp050_yp050", 12.0, 5.0, 5.0, "C16", 0.29673, 0.29673, 0.90769, 24.81, 45.0),
)


def sha256_bytes(content: bytes) -> str:
    return hashlib.sha256(content).hexdigest()


def _tip_quaternion_from_moment(qx: float, qy: float, qz: float) -> tuple[float, float, float, float]:
    """Return the minimum-roll rod-tip quaternion for a board-frame moment.

    The frozen registration is ``R_plane1_from_array=Rx(pi)`` and
    ``R_rod_tip_from_magnet=Rx(pi)``.  Therefore the rod +Z axis in plane1 is
    ``(-qx, qy, qz)``.  The minimum-angle quaternion maps +Z to that axis.
    """

    norm = math.sqrt(qx * qx + qy * qy + qz * qz)
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("moment direction must be finite and nonzero")
    qx, qy, qz = qx / norm, qy / norm, qz / norm
    transverse = math.hypot(qx, qy)
    if transverse <= 1.0e-14:
        if qz < 0.0:
            raise ValueError("180 degree moment reversal has no unique zero-roll realization")
        return (0.0, 0.0, 0.0, 1.0)
    # e_z cross (-qx, qy, qz) = (-qy, -qx, 0).
    half_sine = math.sqrt(max(0.0, (1.0 - qz) / 2.0))
    scale = half_sine / transverse
    quaternion = (-qy * scale, -qx * scale, 0.0, math.sqrt(max(0.0, (1.0 + qz) / 2.0)))
    norm_q = math.sqrt(sum(value * value for value in quaternion))
    return tuple(value / norm_q for value in quaternion)


def _authority_bytes(condition: Condition, quaternion: tuple[float, float, float, float]) -> bytes:
    output: list[str] = []
    output.append("time_s,frame_id,tip_x_m,tip_y_m,tip_z_m,tip_qx,tip_qy,tip_qz,tip_qw,phase")
    values = (
        condition.x_cm / 100.0,
        condition.y_cm / 100.0,
        -condition.height_cm / 100.0,
        *quaternion,
    )
    for time_s, phase in STATIC_ROWS:
        output.append(
            f"{time_s:.9f},{FRAME_ID},{values[0]:.9f},{values[1]:.9f},{values[2]:.9f},"
            f"{values[3]:.12f},{values[4]:.12f},{values[5]:.12f},{values[6]:.12f},{phase}"
        )
    return ("\n".join(output) + "\n").encode("ascii")


def _write_immutable(path: Path, content: bytes) -> None:
    if path.exists():
        if path.is_symlink() or not path.is_file() or path.read_bytes() != content:
            raise FileExistsError(f"refusing to overwrite differing authority: {path}")
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)


def main() -> int:
    metadata_path = TRAJECTORIES / "mla_pose_conditions_z070_z100_z120.csv"
    metadata_fields = (
        "condition_id", "sensor_class", "height_cm", "x_cm", "y_cm",
        "moment_q_x", "moment_q_y", "moment_q_z", "tilt_deg", "azimuth_deg",
        "frame_id", "tip_z_m", "tip_qx", "tip_qy", "tip_qz", "tip_qw", "authority_sha256",
    )
    rows: list[dict[str, str]] = []
    for condition in CONDITIONS:
        quaternion = _tip_quaternion_from_moment(condition.qx, condition.qy, condition.qz)
        content = _authority_bytes(condition, quaternion)
        authority_path = TRAJECTORIES / f"{condition.condition_id}.csv"
        _write_immutable(authority_path, content)
        rows.append({
            "condition_id": condition.condition_id,
            "sensor_class": condition.sensor_class,
            "height_cm": f"{condition.height_cm:.2f}",
            "x_cm": f"{condition.x_cm:.2f}",
            "y_cm": f"{condition.y_cm:.2f}",
            "moment_q_x": f"{condition.qx:.5f}",
            "moment_q_y": f"{condition.qy:.5f}",
            "moment_q_z": f"{condition.qz:.5f}",
            "tilt_deg": f"{condition.tilt_deg:.2f}",
            "azimuth_deg": f"{condition.azimuth_deg:.1f}",
            "frame_id": FRAME_ID,
            "tip_z_m": f"{-condition.height_cm / 100.0:.9f}",
            "tip_qx": f"{quaternion[0]:.12f}",
            "tip_qy": f"{quaternion[1]:.12f}",
            "tip_qz": f"{quaternion[2]:.12f}",
            "tip_qw": f"{quaternion[3]:.12f}",
            "authority_sha256": sha256_bytes(content),
        })
        print(f"{authority_path}: sha256={rows[-1]['authority_sha256']}")
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    with metadata_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=metadata_fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    print(f"metadata: {metadata_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
