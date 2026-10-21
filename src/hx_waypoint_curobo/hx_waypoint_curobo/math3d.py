from __future__ import annotations

from math import acos, sin
from typing import Sequence

import numpy as np

from .errors import TrajectoryFormatError


def normalize_quaternion_xyzw(values: Sequence[float]) -> np.ndarray:
    quaternion = np.asarray(values, dtype=float)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise TrajectoryFormatError("quaternion must contain four finite xyzw values")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1.0e-12:
        raise TrajectoryFormatError("zero-length quaternion is invalid")
    return quaternion / norm


def quaternion_to_matrix_xyzw(values: Sequence[float]) -> np.ndarray:
    x, y, z, w = normalize_quaternion_xyzw(values)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_to_quaternion_xyzw(matrix: np.ndarray) -> np.ndarray:
    rotation = np.asarray(matrix, dtype=float)
    if rotation.shape != (3, 3):
        raise TrajectoryFormatError("rotation matrix must be 3x3")
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            quaternion = np.array(
                [
                    0.25 * scale,
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[2, 1] - rotation[1, 2]) / scale,
                ]
            )
        elif index == 1:
            scale = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            quaternion = np.array(
                [
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    0.25 * scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    (rotation[0, 2] - rotation[2, 0]) / scale,
                ]
            )
        else:
            scale = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            quaternion = np.array(
                [
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    0.25 * scale,
                    (rotation[1, 0] - rotation[0, 1]) / scale,
                ]
            )
    return normalize_quaternion_xyzw(quaternion)


def pose_matrix(position_m: Sequence[float], quaternion_xyzw: Sequence[float]) -> np.ndarray:
    position = np.asarray(position_m, dtype=float)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise TrajectoryFormatError("position must contain three finite values")
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = quaternion_to_matrix_xyzw(quaternion_xyzw)
    transform[:3, 3] = position
    return transform


def matrix_pose(transform: np.ndarray) -> tuple[tuple[float, ...], tuple[float, ...]]:
    matrix = np.asarray(transform, dtype=float)
    if matrix.shape != (4, 4):
        raise TrajectoryFormatError("pose transform must be 4x4")
    position = tuple(float(value) for value in matrix[:3, 3])
    quaternion = tuple(float(value) for value in matrix_to_quaternion_xyzw(matrix[:3, :3]))
    return position, quaternion


def inverse_pose(transform: np.ndarray) -> np.ndarray:
    matrix = np.asarray(transform, dtype=float)
    rotation = matrix[:3, :3]
    result = np.eye(4, dtype=float)
    result[:3, :3] = rotation.T
    result[:3, 3] = -rotation.T @ matrix[:3, 3]
    return result


def slerp_xyzw(start: Sequence[float], end: Sequence[float], ratio: float) -> np.ndarray:
    q0 = normalize_quaternion_xyzw(start)
    q1 = normalize_quaternion_xyzw(end)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        return normalize_quaternion_xyzw(q0 + float(ratio) * (q1 - q0))
    angle = acos(dot)
    denominator = sin(angle)
    return (
        sin((1.0 - float(ratio)) * angle) / denominator * q0
        + sin(float(ratio) * angle) / denominator * q1
    )


def quaternion_angular_distance_deg(start: Sequence[float], end: Sequence[float]) -> float:
    q0 = normalize_quaternion_xyzw(start)
    q1 = normalize_quaternion_xyzw(end)
    dot = abs(float(np.dot(q0, q1)))
    return float(np.degrees(2.0 * acos(float(np.clip(dot, -1.0, 1.0)))))
