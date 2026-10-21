"""Shared immutable CSV writing and nominal Cartesian motion diagnostics."""

from __future__ import annotations

import csv
import hashlib
import io
import json
from pathlib import Path
from typing import Any, Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.spatial.transform import Rotation

DT_S = 0.05
PROTOCOL = "real_collection_v2_20261016"
CSV_FIELDS = (
    "time_s", "frame_id", "tip_x_m", "tip_y_m", "tip_z_m",
    "tip_qx", "tip_qy", "tip_qz", "tip_qw", "phase",
)


def sample_grid(duration_s: float, dt_s: float = DT_S) -> NDArray[np.float64]:
    """Sample a newly designed interval, retaining its final ROS-ns endpoint."""
    if not np.isfinite(duration_s) or duration_s <= 0 or dt_s <= 0:
        raise ValueError("duration and sampling period must be positive")
    duration = round(float(duration_s), 9)
    values = np.arange(int(np.floor(duration / dt_s + 1e-10)) + 1) * dt_s
    if duration - values[-1] > 1e-9:
        values = np.append(values, duration)
    else:
        values[-1] = duration
    return np.round(values, 9)


def smootherstep7(u: ArrayLike) -> NDArray[np.float64]:
    """Septic progress with velocity, acceleration and jerk zero at both ends."""
    v = np.asarray(u, dtype=float)
    if np.any(v < -1e-10) or np.any(v > 1 + 1e-10):
        raise ValueError("progress outside the designed interval")
    v = np.clip(v, 0., 1.)
    return v**4 * (35. + v * (-84. + v * (70. - 20. * v)))


def _gradient(values: NDArray[np.float64], times: NDArray[np.float64]) -> NDArray[np.float64]:
    return np.gradient(values, times, axis=0, edge_order=2 if len(times) > 2 else 1)


def _max_norm(values: NDArray[np.float64]) -> float:
    return float(np.max(np.linalg.norm(values, axis=1)))


def motion_diagnostics(times: ArrayLike, positions: ArrayLike, quaternions: ArrayLike) -> dict[str, Any]:
    """Finite-difference diagnostics; continuous curve constraints remain explicit."""
    t, p, q = np.asarray(times, float), np.asarray(positions, float), np.asarray(quaternions, float)
    if t.ndim != 1 or len(t) < 2 or p.shape != (len(t), 3) or q.shape != (len(t), 4):
        raise ValueError("expected t[N], p[N,3], xyzw[N,4]")
    if not all(np.isfinite(x).all() for x in (t, p, q)) or np.any(np.diff(t) <= 0):
        raise ValueError("non-finite trajectory or non-increasing time")
    if np.max(np.abs(np.linalg.norm(q, axis=1) - 1)) > 1e-9:
        raise ValueError("generated quaternions are not unit length")
    v = _gradient(p, t)
    a, j = _gradient(v, t), _gradient(_gradient(v, t), t)
    rotations = Rotation.from_quat(q)
    mid_t = (t[1:] + t[:-1]) / 2
    omega = (rotations[1:] * rotations[:-1].inv()).as_rotvec() / np.diff(t)[:, None]
    alpha = _gradient(omega, mid_t) if len(mid_t) > 1 else np.zeros_like(omega)
    angular_jerk = _gradient(alpha, mid_t) if len(mid_t) > 1 else np.zeros_like(omega)
    axes = rotations.apply(np.repeat([[0., 0., 1.]], len(t), axis=0))
    axis_angles = np.arctan2(np.linalg.norm(np.cross(axes[:-1], axes[1:]), axis=1),
                             np.einsum("ij,ij->i", axes[:-1], axes[1:]))
    speeds = np.linalg.norm(v, axis=1)
    result: dict[str, Any] = {
        "diagnostic_method": "finite differences of source samples; world-frame SO(3) angular increments",
        "max_speed_m_s": _max_norm(v),
        "mean_speed_m_s": float(np.trapezoid(speeds, t) / (t[-1] - t[0])),
        "p95_speed_m_s": float(np.percentile(speeds, 95)),
        "max_acceleration_m_s2": _max_norm(a), "max_jerk_m_s3": _max_norm(j),
        "max_angular_speed_deg_s": float(np.rad2deg(_max_norm(omega))),
        "max_angular_acceleration_deg_s2": float(np.rad2deg(_max_norm(alpha))),
        "max_angular_jerk_deg_s3": float(np.rad2deg(_max_norm(angular_jerk))),
        "max_axis_rate_deg_s": float(np.rad2deg(np.max(axis_angles / np.diff(t)))),
        "position_min_m": p.min(axis=0).tolist(), "position_max_m": p.max(axis=0).tolist(),
        "height_min_m": float(-p[:, 2].max()), "height_max_m": float(-p[:, 2].min()),
        "axis_definition": "nominal local +Z only; measured magnetization axis pending calibration",
    }
    return result


def _write_immutable(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        if path.is_symlink() or path.read_bytes() != payload:
            raise FileExistsError(f"refusing to overwrite differing authority: {path}")
        return
    with path.open("xb") as stream:
        stream.write(payload)


def write_json_immutable(path: Path, data: Any) -> None:
    _write_immutable(Path(path), (json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n").encode())


def write_authority(path: Path, times: ArrayLike, positions: ArrayLike,
                    quaternions: ArrayLike, phases: Sequence[str]) -> dict[str, Any]:
    t, p, q = np.asarray(times, float), np.asarray(positions, float), np.asarray(quaternions, float)
    diagnostics = motion_diagnostics(t, p, q)
    if t[0] != 0 or len(phases) != len(t) or phases[0] != "draw":
        raise ValueError("authority must start at t=0 with phase=draw")
    stream = io.StringIO(newline="")
    writer = csv.writer(stream, lineterminator="\n")
    writer.writerow(CSV_FIELDS)
    previous_serialized_pose: list[str] | None = None
    for i, time_s in enumerate(t):
        if phases[i] not in ("draw", "hold"):
            raise ValueError("authority phase must be draw/hold")
        if i and phases[i] == "hold" and (np.max(np.abs(p[i] - p[i-1])) > 1e-12 or
                                          np.max(np.abs(q[i] - q[i-1])) > 1e-12):
            raise ValueError(f"hold row {i} is moving")
        serialized_pose = [*(f"{x:.12f}" for x in p[i]), *(f"{x:.12f}" for x in q[i])]
        if phases[i] == "hold" and serialized_pose != previous_serialized_pose:
            raise ValueError(f"hold row {i} changes its serialized authority pose")
        writer.writerow([f"{time_s:.9f}", "plane1", *serialized_pose, phases[i]])
        previous_serialized_pose = serialized_pose
    payload = stream.getvalue().encode()
    _write_immutable(Path(path), payload)
    return {"source_file": str(path), "source_sha256": hashlib.sha256(payload).hexdigest(),
            "source_pose_count": len(t), "duration_s": float(t[-1]), "diagnostics": diagnostics}
