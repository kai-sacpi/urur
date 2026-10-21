from __future__ import annotations

from typing import Sequence

import numpy as np

from hx_waypoint_curobo.math3d import matrix_to_quaternion_xyzw


def point_local_z_at(target_vector: Sequence[float], roll_reference: Sequence[float] = (0, 1, 0)):
    """Return xyzw orientation whose local +Z points along ``target_vector``."""
    z_axis = np.asarray(target_vector, dtype=float)
    z_axis /= np.linalg.norm(z_axis)
    reference = np.asarray(roll_reference, dtype=float)
    if abs(float(np.dot(reference, z_axis))) > 0.99:
        reference = np.array([1.0, 0.0, 0.0])
    x_axis = np.cross(reference, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    return tuple(float(value) for value in matrix_to_quaternion_xyzw(np.column_stack((x_axis, y_axis, z_axis))))
