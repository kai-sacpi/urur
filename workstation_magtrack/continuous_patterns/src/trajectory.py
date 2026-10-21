"""Offline C3 nominal center curves. No robot, runtime, or device imports.

All derivatives are analytic derivatives in normalized draw time s, except
orientation_diagnostics(), which explicitly returns physical-time SI units.
The 7th degree progress polynomial has zero derivatives 1..3 at both ends.
"""
from __future__ import annotations

import math
import numpy as np
from numpy.polynomial import Polynomial
from scipy.spatial.transform import Rotation
from . import patterns

PROGRESS = Polynomial([0., 0., 0., 0., 35., -84., 70., -20.])


def polynomial_bound(poly):
    """Exact extrema candidates for a polynomial over the closed unit interval."""
    roots = poly.deriv().roots()
    points = [0., 1.] + [float(r.real) for r in roots
                         if abs(r.imag) < 1e-9 and 0. < r.real < 1.]
    return float(np.max(np.abs(poly(points)))) * (1. + 1e-12)


S_BOUNDS = np.array([polynomial_bound(PROGRESS.deriv(d)) for d in range(5)])


def progress(s, d=0):
    s = np.asarray(s, dtype=float)
    if np.any(s < -1e-10) or np.any(s > 1. + 1e-10):
        raise ValueError("normalized draw time outside [0,1]")
    result = PROGRESS.deriv(d)(s)
    if d in (1, 2, 3):
        result = np.where((s == 0.) | (s == 1.), 0., result)
    if d == 0:
        result = np.where(s == 0., 0., np.where(s == 1., 1., result))
    return result


def height_segments(meta):
    spec = meta["height_profile"]
    if spec["mode"] == "constant":
        return np.array([0., 1.]), np.array([spec["lower_m"], spec["lower_m"]])
    lo, hi = spec["lower_m"], spec["upper_m"]
    edges = np.unique(np.r_[lo, np.arange(8, 21) / 100., hi])
    edges = edges[(edges >= lo - 1e-12) & (edges <= hi + 1e-12)]
    levels = np.r_[edges[::-1], edges[1:]]
    deltas = np.abs(np.diff(levels))
    middle = (levels[1:] + levels[:-1]) / 2
    weights = np.where(middle >= .15, .60 / (.20 - .15), .40 / (.15 - lo))
    knots = np.r_[0., np.cumsum(deltas * weights / 2.)]
    knots[len(edges) - 1], knots[-1] = .5, 1.
    return knots, levels


def height_derivatives(meta, s):
    s = np.asarray(s, float)
    knots, heights = height_segments(meta)
    if meta["height_profile"]["mode"] == "constant":
        return [np.full_like(s, heights[0])] + [np.zeros_like(s) for _ in range(4)]
    indexes = np.minimum(np.searchsorted(knots, s, side="right") - 1, len(knots) - 2)
    widths = knots[indexes + 1] - knots[indexes]
    local = (s - knots[indexes]) / widths
    delta = heights[indexes + 1] - heights[indexes]
    return [heights[indexes] + delta * progress(local)] + [
        delta * progress(local, d) / widths**d for d in range(1, 5)]


def position_derivatives(meta, s):
    s = np.atleast_1d(np.asarray(s, float))
    backwards = meta["direction"] == "reverse"
    x = 1. - s if backwards else s
    cycles = meta["xy"]["cycles"]
    u = cycles * progress(x) + meta["xy"]["parameter_offset"]
    ud = [cycles * progress(x, d)[:, None] for d in range(1, 5)]
    f = [patterns.evaluate(meta["family_id"], u, meta["pattern_parameters"], d)
         for d in range(5)]
    u1, u2, u3, u4 = ud
    xy = [f[0], f[1] * u1, f[2] * u1**2 + f[1] * u2,
          f[3] * u1**3 + 3 * f[2] * u1 * u2 + f[1] * u3,
          f[4] * u1**4 + 6*f[3]*u1**2*u2 +
          f[2]*(3*u2**2 + 4*u1*u3) + f[1]*u4]
    angle = meta["xy"]["rotation_rad"]
    matrix = np.array([[np.cos(angle), -np.sin(angle)],
                       [np.sin(angle), np.cos(angle)]]) @ np.diag(meta["xy"]["scale_m"])
    h = height_derivatives(meta, x)
    out = [np.c_[v @ matrix.T, -h[d]] for d, v in enumerate(xy)]
    out[0][:, :2] += meta["xy"]["center_m"]
    if backwards:
        for d in (1, 3):
            out[d] *= -1.
    return out


def global_derivative_bounds(meta):
    f = np.asarray(patterns.derivative_bounds(meta["family_id"],
                       meta["pattern_parameters"], max_order=4))
    u1, u2, u3, u4 = meta["xy"]["cycles"] * S_BOUNDS[1:]
    bounds = np.array([f[0], f[1]*u1, f[2]*u1**2 + f[1]*u2,
              f[3]*u1**3 + 3*f[2]*u1*u2 + f[1]*u3,
              f[4]*u1**4 + 6*f[3]*u1**2*u2 +
              f[2]*(3*u2**2 + 4*u1*u3) + f[1]*u4])
    angle = meta["xy"]["rotation_rad"]
    matrix = np.abs(np.array([[np.cos(angle), -np.sin(angle)],
                             [np.sin(angle), np.cos(angle)]])) @ np.diag(meta["xy"]["scale_m"])
    xy = bounds @ matrix.T
    knots, heights = height_segments(meta)
    if meta["height_profile"]["mode"] == "constant":
        hbound = np.r_[heights[0], np.zeros(4)]
    else:
        hbound = np.array([np.max(heights)] + [
            float(np.max(np.abs(np.diff(heights)) * S_BOUNDS[d] / np.diff(knots)**d))
            for d in range(1, 5)])
    xy[0] += np.abs(meta["xy"]["center_m"])
    return np.linalg.norm(np.c_[xy, hbound], axis=1)


def derivative_certificate(meta, samples=32769):
    """A grid maximum plus global next-derivative half-cell bound.

    p is C3 across all height knots. The norm of p^(d) is Lipschitz with
    constant sup|p^(d+1)|. The global 4th derivative bound is piecewise valid.
    Thus the acceleration/jerk bounds cover the entire continuous curve,
    including centripetal terms; this is stronger than finite differences.
    """
    knots, _ = height_segments(meta)
    s = np.unique(np.r_[np.linspace(0., 1., samples), knots])
    if meta["direction"] == "reverse":
        s = np.unique(np.r_[s, 1.-knots])
    values = position_derivatives(meta, s)
    peaks = np.array([np.max(np.linalg.norm(v, axis=1)) for v in values])
    global_bounds = global_derivative_bounds(meta)
    half_cell = float(np.max(np.diff(s))) / 2.
    refined = peaks.copy()
    refined[0] = global_bounds[0]
    refined[1:4] += global_bounds[2:5] * half_cell
    refined[4] = global_bounds[4]
    return {"method": "analytic_chain_rule_grid_max_plus_global_derivative_Lipschitz_enclosure",
            "sample_count": len(s), "maximum_normalized_cell_width": 2*half_cell,
            "normalized_position_derivative_bounds": refined.tolist(),
            "sampled_position_derivative_maxima": peaks.tolist(),
            "global_derivative_bounds": global_bounds.tolist(),
            "derivative_units": "m per normalized draw time s^order"}


def angle_derivatives(meta, s):
    s = np.atleast_1d(np.asarray(s, float))
    spec = meta["orientation"]
    if meta["orientation_mode"] == "fixed":
        return [np.zeros_like(s) for _ in range(5)]
    reverse = meta["direction"] == "reverse"
    x = 1.-s if reverse else s
    w = 2*math.pi*spec["cycles"]
    g = w*progress(x) + spec["phase_rad"]
    g1, g2, g3, g4 = [w*progress(x, d) for d in range(1, 5)]
    a, sn, cs = spec["amplitude_rad"], np.sin(g), np.cos(g)
    values = [a*sn, a*cs*g1, a*(-sn*g1*g1+cs*g2),
              a*(-cs*g1**3-3*sn*g1*g2+cs*g3),
              a*(sn*g1**4-6*cs*g1*g1*g2-sn*(3*g2*g2+4*g1*g3)+cs*g4)]
    if reverse:
        values[1] *= -1.
        values[3] *= -1.
    return values


def orientation(meta, s):
    theta = angle_derivatives(meta, s)[0]
    spec = meta["orientation"]
    base = Rotation.from_rotvec(spec["base_rotvec_rad"])
    axis = np.asarray(spec["axis_in_base_tool_xyz"])
    return (base * Rotation.from_rotvec(theta[:, None]*axis)).as_quat()


def orientation_diagnostics(meta, s):
    derivatives = angle_derivatives(meta, s)
    spec = meta["orientation"]
    base = Rotation.from_rotvec(spec["base_rotvec_rad"])
    axis = base.apply(spec["axis_in_base_tool_xyz"])
    duration = meta["draw_duration_s"]
    q = orientation(meta, s)
    return {"omega_rad_s": derivatives[1][:, None]*axis/duration,
            "alpha_rad_s2": derivatives[2][:, None]*axis/duration**2,
            "jerk_rad_s3": derivatives[3][:, None]*axis/duration**3,
            "axis_xyz": Rotation.from_quat(q).apply([0., 0., 1.])}


def angular_certificate(meta, samples=32769):
    if meta["orientation_mode"] == "fixed":
        return {"angular_derivative_bounds_deg": [0., 0., 0.], "method": "fixed_orientation_exact"}
    spec = meta["orientation"]
    g1, g2, g3, g4 = 2*math.pi*spec["cycles"]*S_BOUNDS[1:]
    a = spec["amplitude_rad"]
    global_bounds = a*np.array([g1, g1*g1+g2,
        g1**3+3*g1*g2+g3, g1**4+6*g1*g1*g2+3*g2*g2+4*g1*g3+g4])
    values = angle_derivatives(meta, np.linspace(0., 1., samples))
    peaks = np.array([np.max(np.abs(values[d])) for d in range(1, 4)])
    enclosed = peaks + global_bounds[1:]/(2*(samples-1))
    physical = np.rad2deg(enclosed / meta["draw_duration_s"]**np.arange(1, 4))
    return {"angular_derivative_bounds_deg": physical.tolist(),
            "normalized_scalar_global_derivative_bounds_rad": global_bounds.tolist(),
            "method": "fixed_rotation_axis_SO3_analytic_derivatives_plus_Lipschitz_enclosure",
            "sample_count": samples}
