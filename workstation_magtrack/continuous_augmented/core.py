"""Analytic open Bernstein paths, positive speed warps and SO(3) motion.

The parameter ``s`` is normalized *draw time*, not arc length.  All position
derivatives returned by :func:`evaluate` are with respect to ``s``.  Angular
outputs explicitly carry physical SI units.  A single monotone longitudinal
projection certifies that these paths have no closed loops or retraced laps.

This module has no ROS, GPU, robot, serial, or playback imports.  Its only
purpose is to design new upstream authorities before they are written.
"""

from __future__ import annotations

import math
from typing import Any, Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.spatial.transform import Rotation

FloatArray = NDArray[np.float64]
SEPTIC_CONTROL_POINTS = np.array([0., 0., 0., 0., 1., 1., 1., 1.])


def bernstein(control_points: ArrayLike, u: ArrayLike, derivative: int = 0) -> FloatArray:
    """Stable de Casteljau evaluation, including exact analytic derivatives."""
    cp = np.asarray(control_points, dtype=float)
    x = np.atleast_1d(np.asarray(u, dtype=float))
    if cp.ndim not in (1, 2) or len(cp) < 1 or derivative < 0:
        raise ValueError("expected scalar/vector Bernstein controls and a nonnegative order")
    if not np.isfinite(cp).all() or not np.isfinite(x).all():
        raise ValueError("Bernstein inputs must be finite")
    if derivative >= len(cp):
        return np.zeros((len(x), *cp.shape[1:]))
    n = len(cp) - 1
    for order in range(derivative):
        cp = (n - order) * np.diff(cp, axis=0)
    values = np.broadcast_to(cp, (len(x), *cp.shape)).copy()
    w = x.reshape((len(x),) + (1,) * cp.ndim)
    for width in range(len(cp) - 1, 0, -1):
        values = values[:, :width] * (1. - w) + values[:, 1:width+1] * w
    return values[:, 0]


def _control_derivative_bounds(control_points: ArrayLike, order: int) -> FloatArray:
    """Convex-hull norm bound for all derivatives on the closed unit interval."""
    cp = np.asarray(control_points, float)
    n = len(cp) - 1
    result = []
    for d in range(order + 1):
        if len(cp) == 0:
            result.append(0.)
        elif cp.ndim == 1:
            result.append(float(np.max(np.abs(cp))))
        else:
            result.append(float(np.max(np.linalg.norm(cp, axis=1))))
        cp = (n - d) * np.diff(cp, axis=0)
    # Guard only arithmetic roundoff, never hide a sampled violation.
    return np.asarray(result) * (1. + 2e-12)


def _bell(n: int, k: int, derivatives: Sequence[Any]) -> Any:
    """Partial exponential Bell polynomial, with x[j] the j-th derivative."""
    if n == k == 0:
        return 1.
    if n == 0 or k == 0 or k > n:
        return 0.
    return sum(math.comb(n-1, j-1) * derivatives[j] * _bell(n-j, k-1, derivatives)
               for j in range(1, n-k+2))


def _compose(control_points: ArrayLike, inner: Sequence[FloatArray], order: int) -> list[FloatArray]:
    result = [bernstein(control_points, inner[0])]
    vector = np.asarray(control_points).ndim == 2
    for n in range(1, order+1):
        total = np.zeros_like(result[0])
        for k in range(1, n+1):
            weight = np.asarray(_bell(n, k, inner))
            if vector and weight.ndim:
                weight = weight[:, None]
            total += bernstein(control_points, inner[0], k) * weight
        result.append(total)
    return result


def _compose_bounds(outer: Sequence[float], inner: Sequence[float], order: int) -> FloatArray:
    return np.array([outer[0]] + [sum(outer[k] * _bell(n, k, inner)
                                    for k in range(1, n+1)) for n in range(1, order+1)])


def _time_cdf_controls(meta: dict[str, Any]) -> FloatArray:
    density = np.asarray(meta["time_warp"]["density_control_points_positive"], float)
    if density.ndim != 1 or len(density) < 2 or not np.isfinite(density).all() or np.any(density <= 0):
        raise ValueError("time density must have finite strictly positive Bernstein controls")
    return np.r_[0., np.cumsum(density) / np.sum(density)]


def progress_derivatives(meta: dict[str, Any], s: ArrayLike, order: int = 4) -> list[FloatArray]:
    """u(s)=septic(CDF(s)); analytic Faà di Bruno derivatives, never FD."""
    x = np.atleast_1d(np.asarray(s, float))
    if x.ndim != 1 or not np.isfinite(x).all() or np.any((x < 0.) | (x > 1.)):
        raise ValueError("normalized draw time must be finite and inside [0, 1]")
    cdf_cp = _time_cdf_controls(meta)
    f = [bernstein(cdf_cp, x, d) for d in range(order+1)]
    # Cumulative arithmetic is not allowed to move the exact authority endpoints.
    f[0] = np.where(x == 0., 0., np.where(x == 1., 1., f[0]))
    result = _compose(SEPTIC_CONTROL_POINTS, f, order)
    result[0] = np.where(x == 0., 0., np.where(x == 1., 1., result[0]))
    for d in range(1, min(order, 3)+1):
        result[d] = np.where((x == 0.) | (x == 1.), 0., result[d])
    return result


def _progress_bounds(meta: dict[str, Any], order: int = 4) -> FloatArray:
    return _compose_bounds(_control_derivative_bounds(SEPTIC_CONTROL_POINTS, order),
                           _control_derivative_bounds(_time_cdf_controls(meta), order), order)


def _complex_rotation_jets(angle: Sequence[FloatArray], order: int) -> list[NDArray[np.complex128]]:
    """Exact derivatives of exp(i angle(s)), from z'=i angle' z."""
    result = [np.exp(1j * angle[0])]
    for n in range(1, order+1):
        result.append(sum(math.comb(n-1, k) * 1j * angle[k+1] * result[n-1-k]
                          for k in range(n)))
    return result


def _rotation_norm_bounds(angle_bounds: Sequence[float], order: int) -> FloatArray:
    """Induced 2-norm derivative bound for a fixed-axis rotation matrix."""
    result = [1.]
    for n in range(1, order+1):
        result.append(sum(math.comb(n-1, k) * angle_bounds[k+1] * result[n-1-k]
                          for k in range(n)))
    return np.array(result)


def _so3_normalized(meta: dict[str, Any], u: Sequence[FloatArray]) -> tuple[FloatArray, list[FloatArray], FloatArray]:
    """World-frame omega and its first three s-derivatives for Rz Ry Rx.

    omega = yaw' ez + pitch' Rz ey + roll' Rz Ry ex.  The transported axes
    and product derivatives are differentiated analytically; Euler angle
    derivatives themselves are never mistaken for angular velocity.
    """
    controls = meta["orientation"]["euler_xyz_control_points_rad"]
    angles = _compose(controls, u, 4)
    roll = [v[:, 0] for v in angles]
    pitch = [v[:, 1] for v in angles]
    yaw = [v[:, 2] for v in angles]
    zy, zp = _complex_rotation_jets(yaw, 3), _complex_rotation_jets(pitch, 3)
    y_axis = [np.c_[-v.imag, v.real, np.zeros(len(v))] for v in zy]
    x_axis = []
    for n in range(4):
        xy = sum(math.comb(n, k) * zy[k] * zp[n-k].real for k in range(n+1))
        x_axis.append(np.c_[xy.real, xy.imag, -zp[n].imag])
    omega = []
    for n in range(4):
        w = np.c_[np.zeros_like(yaw[n+1]), np.zeros_like(yaw[n+1]), yaw[n+1]]
        for k in range(n+1):
            w += math.comb(n, k) * (pitch[k+1][:, None] * y_axis[n-k]
                                  + roll[k+1][:, None] * x_axis[n-k])
        omega.append(w)
    rotations = Rotation.from_euler("xyz", angles[0])
    return rotations.as_quat(), omega, rotations.apply([0., 0., 1.])


def _global_bounds(meta: dict[str, Any]) -> tuple[FloatArray, FloatArray]:
    ub = _progress_bounds(meta)
    pb = _compose_bounds(_control_derivative_bounds(meta["curve"]["position_control_points_m"], 4), ub, 4)
    cp = np.asarray(meta["orientation"]["euler_xyz_control_points_rad"], float)
    ab = [_compose_bounds(_control_derivative_bounds(cp[:, i], 4), ub, 4) for i in range(3)]
    rb, ptb, yb = ab
    y_axis = _rotation_norm_bounds(yb, 3)
    pitch_rot = _rotation_norm_bounds(ptb, 3)
    x_axis = np.array([sum(math.comb(n, k) * y_axis[k] * pitch_rot[n-k]
                           for k in range(n+1)) for n in range(4)])
    wb = np.array([yb[n+1] + sum(math.comb(n, k) *
                   (ptb[k+1] * y_axis[n-k] + rb[k+1] * x_axis[n-k])
                   for k in range(n+1)) for n in range(4)])
    return pb * (1. + 2e-12), wb * (1. + 2e-12)


def evaluate(meta: dict[str, Any], normalized_s: ArrayLike) -> dict[str, Any]:
    """Evaluate a designed draw segment without altering any source authority."""
    u = progress_derivatives(meta, normalized_s)
    pd = _compose(meta["curve"]["position_control_points_m"], u, 4)
    q, wd, axis = _so3_normalized(meta, u)
    duration = float(meta["draw_duration_s"])
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError("draw duration must be positive")
    return {"position": pd[0], "quaternion": q, "position_derivatives": pd,
            "progress": u[0], "progress_derivatives": u,
            "omega_rad_s": wd[0] / duration,
            "alpha_rad_s2": wd[1] / duration**2,
            "jerk_rad_s3": wd[2] / duration**3,
            "normalized_angular_velocity_derivatives": wd,
            "axis_xyz": axis, "height_m": -pd[0][:, 2]}


def derivative_certificate(meta: dict[str, Any], samples: int = 32769) -> dict[str, Any]:
    """Continuous curve bounds: analytic grid maximum + global Lipschitz term.

    The norm of the d-th derivative is Lipschitz with constant at most the
    global norm bound of derivative d+1.  The uniform grid includes both
    endpoints, so every point is within half a cell of a checked sample.
    Global bounds follow from Bernstein convex hulls, chain rule Bell
    polynomials, rotation-matrix induced norms and product rules.
    """
    if samples < 3:
        raise ValueError("at least three samples are required for enclosure")
    grid = np.linspace(0., 1., samples)
    values = evaluate(meta, grid)
    pp = np.array([np.linalg.norm(v, axis=1).max() for v in values["position_derivatives"]])
    wp = np.array([np.linalg.norm(v, axis=1).max() for v in values["normalized_angular_velocity_derivatives"]])
    pb, wb = _global_bounds(meta)
    half_cell = float(np.max(np.diff(grid))) / 2.
    duration = float(meta["draw_duration_s"])
    orders = np.arange(1, 4)
    pos = (pp[1:4] + pb[2:5] * half_cell) / duration**orders
    ang = np.rad2deg((wp[:3] + wb[1:4] * half_cell) / duration**orders)
    return {
        "method": "analytic_Bernstein_chain_rule_SO3_transported_axes_grid_max_plus_global_next_derivative_Lipschitz",
        "sample_count": samples, "maximum_normalized_cell_width": 2*half_cell,
        "normalized_position_derivative_global_bounds_orders_0_4": pb.tolist(),
        "normalized_angular_velocity_derivative_global_bounds_orders_0_3": wb.tolist(),
        "normalized_position_derivative_sampled_peaks_orders_0_4": pp.tolist(),
        "normalized_angular_velocity_derivative_sampled_peaks_orders_0_3": wp.tolist(),
        "position_derivative_bounds_m_s_orders_1_3": pos.tolist(),
        "angular_derivative_bounds_deg_s_orders_1_3": ang.tolist(),
        "derivative_convention": "position derivatives in m per normalized s^order; normalized omega in rad per s^(order+1); physical bounds use draw_duration_s",
        "finite_differences_used_for_certificate": False,
    }


def path_certificate(meta: dict[str, Any]) -> dict[str, Any]:
    """Convex-hull and monotone-projection proof, not a whole-robot proof."""
    cp = np.asarray(meta["curve"]["position_control_points_m"], float)
    direction = np.asarray(meta["curve"]["longitudinal_axis_xyz"], float)
    if cp.shape != (8, 3) or direction.shape != (3,) or not np.isclose(np.linalg.norm(direction), 1., atol=1e-12):
        raise ValueError("expected degree-7 3D controls and a unit longitudinal direction")
    increments = np.diff(cp, axis=0) @ direction
    density = np.asarray(meta["time_warp"]["density_control_points_positive"], float)
    ec = np.asarray(meta["orientation"]["euler_xyz_control_points_rad"], float)
    tilt_bound = float(np.rad2deg(np.max(np.abs(ec[:, 0])) + np.max(np.abs(ec[:, 1]))))
    return {
        "method": "positive_Bernstein_projection_derivative_and_positive_time_density",
        "strictly_monotone_projection": bool(np.all(increments > 0.) and np.all(density > 0.)),
        "direction_xyz": direction.tolist(),
        "control_projection_increments_m": increments.tolist(),
        "min_control_projection_increment_m": float(increments.min()),
        "endpoint_distance_m": float(np.linalg.norm(cp[-1]-cp[0])),
        "closed_loop": False if np.all(increments > 0.) else None,
        "repeated_laps": 0,
        "convex_hull_coordinate_min_m": cp.min(axis=0).tolist(),
        "convex_hull_coordinate_max_m": cp.max(axis=0).tolist(),
        "tilt_bound_deg": tilt_bound,
        "tilt_bound_method": "SO3 rotation triangle inequality: |roll|+|pitch|; yaw preserves plane1 Z",
        "source_time_map_strictly_increasing_in_draw_interior": bool(np.all(density > 0.)),
        "robot_collision_ik_or_joint_dynamics_certified": False,
    }
