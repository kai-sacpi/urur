"""Dimensionless, periodic XY geometry for continuous_patterns candidates.

This module deliberately owns geometry only.  Its outputs are not waypoint
authorities and are not motion commands.  The caller owns metric scale, the
plane1 height contract, direction, orientation, time parameterization and all
release gates.  In particular, no legacy rod geometry or ROS/GPU code is used.

``u`` is the unwrapped cycle parameter: 0..1 traverses one full period.  All
derivatives are with respect to u, not phase radians or physical seconds.
``evaluate`` returns (N, 2), each zeroth-order coordinate bounded by [-1, 1].
``carrier`` is an optional dimensionless depth carrier, never a height in m.
Both support exact analytic derivatives through order four using Taylor jets.
The corresponding bounds use the same expression tree and nonnegative Taylor
envelopes, so they cover the entire continuous parameter domain without a grid.

Run ``python patterns.py --self-check`` for a deterministic CPU-only check.
See ../design_notes/geometry_sources.md for formulas and source provenance.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
from collections.abc import Mapping
from typing import Any

import numpy as np


MODULE_VERSION = "continuous_patterns_geometry_v1"
MAX_DERIVATIVE = 4
_FACTORIAL = np.array([math.factorial(n) for n in range(MAX_DERIVATIVE + 1)])
FAMILY_NAMES = (
    "woven_lissajous",
    "trefoil_projection",
    "torus_knot_2_5",
    "hypotrochoid_3_2",
    "braided_spiral",
    "spherical_sweep",
    "compound_harmonograph",
    "bent_tube_braid",
    "rounded_square_shear",
    "celtic_weave",
)

_COMMON = {"phase_rad": 0.0, "depth_phase_rad": 0.37}
_DEFAULTS = {
    "woven_lissajous": {
        "x_secondary": 0.18, "y_secondary": 0.16,
        "x_phase_rad": 0.4, "secondary_phase_rad": 0.7,
    },
    "trefoil_projection": {"double_harmonic_weight": 2.0},
    "torus_knot_2_5": {"tube_radius_ratio": 0.28},
    "hypotrochoid_3_2": {
        "major_weight": 2.0, "pen_weight": 1.35, "pen_phase_rad": 0.0,
    },
    "braided_spiral": {
        "braid_weight": 0.20, "slow_radial_weight": 0.07,
        "angle_ripple_rad": 0.055, "slow_phase_rad": 0.45,
    },
    "spherical_sweep": {
        "latitude_amplitude_rad": 0.62, "slow_latitude_rad": 0.035,
        "latitude_phase_rad": 0.15,
    },
    "compound_harmonograph": {
        "x_secondary": 0.24, "y_secondary": 0.20,
        "x_phase_rad": 0.65, "x_secondary_phase_rad": -0.35,
        "y_secondary_phase_rad": 1.0,
    },
    "bent_tube_braid": {
        "centerline_weight": 0.55, "lateral_weight": 0.28,
        "secondary_weight": 0.07, "lateral_phase_rad": 0.45,
        "secondary_phase_rad": 0.2,
    },
    "rounded_square_shear": {
        "rounding_weight": 0.11, "shear_weight": 0.16,
        "shear_phase_rad": 0.6,
    },
    "celtic_weave": {
        "x_perturbation": 0.105, "y_sixth_weight": 0.16,
        "x_phase_rad": 0.5, "y_sixth_phase_rad": 0.3,
    },
}

_DESCRIPTIONS = {
    "woven_lissajous": "2:3 weave with independent 5th/7th harmonic detail",
    "trefoil_projection": "trefoil's unequal first/second-harmonic XY projection",
    "torus_knot_2_5": "two azimuth turns with five radial breathing cycles",
    "hypotrochoid_3_2": "counter-rotating 3:2 rolling-circle pen geometry",
    "braided_spiral": "six radial braids, slow breathing and angular ripple",
    "spherical_sweep": "three azimuth turns with four latitude oscillations",
    "compound_harmonograph": "2+3 by 3+5 coupled pendulum harmonics",
    "bent_tube_braid": "sinusoidal axial return along an S tube plus local braid",
    "rounded_square_shear": "four smooth bends with asymmetric second-harmonic shear",
    "celtic_weave": "2:3 crossings with independent 7th/6th-harmonic weave",
}

_SOURCE_REFS = {
    "woven_lissajous": ["trajectory_generator.py:new_board_continuous_geometry_position", "motions.py:wave_sweep_3d"],
    "trefoil_projection": ["trajectory_generator.py:_candidate_raw_curve:trefoil_knot"],
    "torus_knot_2_5": ["trajectory_generator.py:_candidate_raw_curve:torus_knot_2_5"],
    "hypotrochoid_3_2": ["trajectory_generator.py:_candidate_raw_curve:spirograph"],
    "braided_spiral": ["trajectory_generator.py:_candidate_raw_curve:braided_helix", "motions.py:spiral_circle_3d"],
    "spherical_sweep": ["trajectory_generator.py:_candidate_raw_curve:spherical_spiral"],
    "compound_harmonograph": ["trajectory_generator.py:_candidate_raw_curve:harmonograph"],
    "bent_tube_braid": ["trajectory_generator.py:new_board_esophageal_geometry_position", "trajectory_generator.py:_esophageal_startup_centerline_offsets", "generate_new_board_esophageal_up50_trajectories.py:delegating_entrypoint"],
    "rounded_square_shear": ["trajectory_generator.py:_bafe_local_shape:rounded_square", "motions.py:rounded_square_3d"],
    "celtic_weave": ["trajectory_generator.py:_candidate_raw_curve:celtic_knot"],
}


class _Jet:
    """Exact Taylor coefficients f^(n)/n!, vectorized over samples."""

    def __init__(self, coefficients: np.ndarray):
        self.c = np.asarray(coefficients, dtype=float)

    def _constant(self, value: float) -> _Jet:
        c = np.zeros_like(self.c)
        c[0] = float(value)
        return _Jet(c)

    def _coerce(self, other: Any) -> _Jet:
        return other if isinstance(other, _Jet) else self._constant(float(other))

    def __add__(self, other: Any) -> _Jet:
        return _Jet(self.c + self._coerce(other).c)

    __radd__ = __add__

    def __neg__(self) -> _Jet:
        return _Jet(-self.c)

    def __sub__(self, other: Any) -> _Jet:
        return self + (-self._coerce(other))

    def __rsub__(self, other: Any) -> _Jet:
        return self._coerce(other) - self

    def __mul__(self, other: Any) -> _Jet:
        other = self._coerce(other)
        out = np.zeros_like(self.c)
        for n in range(MAX_DERIVATIVE + 1):
            for k in range(n + 1):
                out[n] += self.c[k] * other.c[n - k]
        return _Jet(out)

    __rmul__ = __mul__

    def __truediv__(self, value: float) -> _Jet:
        return _Jet(self.c / float(value))

    def sincos(self) -> tuple[_Jet, _Jet]:
        sine = np.zeros_like(self.c)
        cosine = np.zeros_like(self.c)
        sine[0], cosine[0] = np.sin(self.c[0]), np.cos(self.c[0])
        for n in range(1, MAX_DERIVATIVE + 1):
            for k in range(1, n + 1):
                sine[n] += k * self.c[k] * cosine[n - k] / n
                cosine[n] -= k * self.c[k] * sine[n - k] / n
        return _Jet(sine), _Jet(cosine)


class _BoundJet:
    """Global |f^(n)|/n! envelope; no sample-derived normalization."""

    def __init__(self, coefficients: np.ndarray):
        self.c = np.asarray(coefficients, dtype=float)

    @staticmethod
    def _constant(value: float) -> _BoundJet:
        c = np.zeros(MAX_DERIVATIVE + 1)
        c[0] = abs(float(value))
        return _BoundJet(c)

    def _coerce(self, other: Any) -> _BoundJet:
        return other if isinstance(other, _BoundJet) else self._constant(float(other))

    def __add__(self, other: Any) -> _BoundJet:
        return _BoundJet(self.c + self._coerce(other).c)

    __radd__ = __add__

    def __neg__(self) -> _BoundJet:
        return _BoundJet(self.c.copy())

    def __sub__(self, other: Any) -> _BoundJet:
        return self + self._coerce(other)

    def __rsub__(self, other: Any) -> _BoundJet:
        return self._coerce(other) + self

    def __mul__(self, other: Any) -> _BoundJet:
        other = self._coerce(other)
        out = np.zeros_like(self.c)
        for n in range(MAX_DERIVATIVE + 1):
            for k in range(n + 1):
                out[n] += self.c[k] * other.c[n - k]
        return _BoundJet(out)

    __rmul__ = __mul__

    def __truediv__(self, value: float) -> _BoundJet:
        return _BoundJet(self.c / abs(float(value)))

    def sincos(self) -> tuple[_BoundJet, _BoundJet]:
        out = np.zeros_like(self.c)
        out[0] = 1.0
        for n in range(1, MAX_DERIVATIVE + 1):
            for k in range(1, n + 1):
                out[n] += k * self.c[k] * out[n - k] / n
        return _BoundJet(out), _BoundJet(out.copy())


def _sin(value: _Jet | _BoundJet) -> _Jet | _BoundJet:
    return value.sincos()[0]


def _cos(value: _Jet | _BoundJet) -> _Jet | _BoundJet:
    return value.sincos()[1]


def _parameters(family: str, supplied: Mapping[str, Any] | None) -> dict[str, float]:
    if family not in FAMILY_NAMES:
        raise ValueError(f"unknown geometry family: {family!r}")
    p = {**_COMMON, **_DEFAULTS[family]}
    supplied = {} if supplied is None else dict(supplied)
    unknown = set(supplied) - set(p) - {"parameter_seed", "parameter_variant"}
    if unknown:
        raise ValueError(f"unsupported {family} parameters: {sorted(unknown)}")
    for key in p:
        if key in supplied:
            p[key] = float(supplied[key])
        if not math.isfinite(p[key]):
            raise ValueError(f"{key} must be finite")
        if not key.endswith("_rad") and p[key] < 0.0:
            raise ValueError(f"{key} must be nonnegative")
    if family == "hypotrochoid_3_2" and p["major_weight"] + p["pen_weight"] <= 0.0:
        raise ValueError("rolling-circle weights must have positive sum")
    if family == "bent_tube_braid" and sum(p[key] for key in ("centerline_weight", "lateral_weight", "secondary_weight")) <= 0.0:
        raise ValueError("bent-tube weights must have positive sum")
    if family == "spherical_sweep":
        lat = p["latitude_amplitude_rad"] + p["slow_latitude_rad"]
        if not (0.0 < lat < math.pi / 2.0):
            raise ValueError("sum of latitude amplitudes must be in (0, pi/2)")
        if p["latitude_amplitude_rad"] < 0.0 or p["slow_latitude_rad"] < 0.0:
            raise ValueError("latitude amplitudes must be nonnegative")
    return p


def _formula(family: str, t: _Jet | _BoundJet, p: Mapping[str, float]) -> tuple[Any, Any]:
    if family == "woven_lissajous":
        a, b, q = p["x_secondary"], p["y_secondary"], p["secondary_phase_rad"]
        x = (_sin(2 * t + p["x_phase_rad"]) + a * _sin(5 * t + q)) / (1 + a)
        y = (_sin(3 * t) + b * _sin(7 * t + q + 0.55)) / (1 + b)
    elif family == "trefoil_projection":
        a = p["double_harmonic_weight"]
        x = (_sin(t) + a * _sin(2 * t)) / (1 + a)
        y = (_cos(t) - a * _cos(2 * t)) / (1 + a)
    elif family == "torus_knot_2_5":
        a = p["tube_radius_ratio"]
        r = 1 + a * _cos(5 * t)
        x, y = r * _cos(2 * t) / (1 + a), r * _sin(2 * t) / (1 + a)
    elif family == "hypotrochoid_3_2":
        a, b, q = p["major_weight"], p["pen_weight"], p["pen_phase_rad"]
        x = (a * _cos(3 * t) + b * _cos(2 * t + q)) / (a + b)
        y = (a * _sin(3 * t) - b * _sin(2 * t + q)) / (a + b)
    elif family == "braided_spiral":
        a, b = p["braid_weight"], p["slow_radial_weight"]
        r = 1 + a * _cos(6 * t) + b * _cos(t + p["slow_phase_rad"])
        az = t + p["angle_ripple_rad"] * _sin(2 * t)
        x, y = r * _cos(az) / (1 + a + b), r * _sin(az) / (1 + a + b)
    elif family == "spherical_sweep":
        lat = p["latitude_amplitude_rad"] * _sin(4 * t + p["latitude_phase_rad"]) + p["slow_latitude_rad"] * _sin(t)
        x, y = _cos(lat) * _cos(3 * t), _cos(lat) * _sin(3 * t)
    elif family == "compound_harmonograph":
        a, b = p["x_secondary"], p["y_secondary"]
        x = (_sin(2 * t + p["x_phase_rad"]) + a * _sin(3 * t + p["x_secondary_phase_rad"])) / (1 + a)
        y = (_sin(3 * t) + b * _sin(5 * t + p["y_secondary_phase_rad"])) / (1 + b)
    elif family == "bent_tube_braid":
        a, b, c = p["centerline_weight"], p["lateral_weight"], p["secondary_weight"]
        x = _sin(t)
        y = (a * _sin(math.pi * x) + b * _sin(4 * t + p["lateral_phase_rad"]) + c * _sin(7 * t + p["secondary_phase_rad"])) / (a + b + c)
    elif family == "rounded_square_shear":
        a, b, q = p["rounding_weight"], p["shear_weight"], p["shear_phase_rad"]
        x = (_cos(t) - a * _cos(3 * t) + b * _sin(2 * t)) / (1 + a + b)
        y = (_sin(t) + a * _sin(3 * t) + 0.7 * b * _sin(2 * t + q)) / (1 + a + 0.7 * b)
    elif family == "celtic_weave":
        a, b = p["x_perturbation"], p["y_sixth_weight"]
        x = (_sin(2 * t + p["x_phase_rad"]) + a * _sin(7 * t)) / (1 + a)
        y = (_sin(3 * t) + b * _sin(6 * t + p["y_sixth_phase_rad"])) / (1 + b)
    else:  # Family validation is done before formula evaluation.
        raise AssertionError(family)
    return x, y


def _carrier_formula(family: str, t: _Jet | _BoundJet, p: Mapping[str, float]) -> Any:
    q = p["depth_phase_rad"]
    if family == "woven_lissajous":
        return (_sin(4 * t + q) + 0.18 * _sin(t)) / 1.18
    if family == "trefoil_projection":
        return -_sin(3 * t + q)
    if family == "torus_knot_2_5":
        return _sin(5 * t + q)
    if family == "hypotrochoid_3_2":
        return (_sin(3 * t + q) + 0.16 * _sin(5 * t + 0.3)) / 1.16
    if family == "braided_spiral":
        return (_sin(6 * t + q) + 0.25 * _sin(t + 0.8)) / 1.25
    if family == "spherical_sweep":
        a, b = p["latitude_amplitude_rad"], p["slow_latitude_rad"]
        return _sin(a * _sin(4 * t + q) + b * _sin(t)) / math.sin(a + b)
    if family == "compound_harmonograph":
        return (_sin(4 * t + q) + 0.12 * _sin(t)) / 1.12
    if family == "bent_tube_braid":
        return (_sin(5 * t + q) + 0.12 * _cos(math.pi * _sin(t))) / 1.12
    if family == "rounded_square_shear":
        return (_sin(2 * t + q) + 0.12 * _sin(5 * t + 0.4)) / 1.12
    if family == "celtic_weave":
        return (_sin(4 * t + q) + 0.16 * _sin(t)) / 1.16
    raise AssertionError(family)


def _order(value: int) -> int:
    if isinstance(value, bool) or int(value) != value or not 0 <= int(value) <= MAX_DERIVATIVE:
        raise ValueError("derivative order must be an integer in [0, 4]")
    return int(value)


def _phase_jet(u: np.ndarray, phase_rad: float) -> _Jet:
    u = np.atleast_1d(np.asarray(u, dtype=float))
    if u.ndim != 1 or not np.all(np.isfinite(u)):
        raise ValueError("u must be a finite one-dimensional array")
    c = np.zeros((MAX_DERIVATIVE + 1, len(u)), dtype=float)
    c[0] = 2.0 * math.pi * u + phase_rad
    c[1] = 2.0 * math.pi
    return _Jet(c)


def _phase_bounds() -> _BoundJet:
    c = np.zeros(MAX_DERIVATIVE + 1)
    # The affine phase only occurs inside sin/cos, so its zeroth-order bound
    # is unused; derivative bounds are valid for every real u, not just 0..1.
    c[1] = 2.0 * math.pi
    return _BoundJet(c)


def evaluate(family: str, u: np.ndarray, parameters: Mapping[str, Any] | None = None, derivative: int = 0) -> np.ndarray:
    """Return N×2 dimensionless XY or its derivative d^order/du^order."""
    n = _order(derivative)
    p = _parameters(family, parameters)
    x, y = _formula(family, _phase_jet(u, p["phase_rad"]), p)
    return np.column_stack((x.c[n], y.c[n])) * _FACTORIAL[n]


def carrier(family: str, u: np.ndarray, parameters: Mapping[str, Any] | None = None, derivative: int = 0) -> np.ndarray:
    """Return optional bounded latent depth (N,), without choosing any height."""
    n = _order(derivative)
    p = _parameters(family, parameters)
    out = _carrier_formula(family, _phase_jet(u, p["phase_rad"]), p)
    return out.c[n] * _FACTORIAL[n]


def derivative_bounds(family: str, parameters: Mapping[str, Any] | None = None, max_order: int = 4) -> np.ndarray:
    """Return (max_order+1,2) global |d^n XY/du^n| bounds, n starting at 0."""
    n = _order(max_order)
    p = _parameters(family, parameters)
    x, y = _formula(family, _phase_bounds(), p)
    bounds = np.column_stack((x.c, y.c)) * _FACTORIAL[:, None]
    # The spherical XY product has |cos(latitude)| <= 1.  Every family has
    # an analytic zeroth-order coordinate bound of one by construction.
    bounds[0] = 1.0
    return bounds[: n + 1]


def carrier_derivative_bounds(family: str, parameters: Mapping[str, Any] | None = None, max_order: int = 4) -> np.ndarray:
    """Return (max_order+1,) continuous global bounds for the depth carrier."""
    n = _order(max_order)
    p = _parameters(family, parameters)
    out = _carrier_formula(family, _phase_bounds(), p)
    bounds = out.c * _FACTORIAL
    # For spherical_sweep, the tighter value follows from |latitude| < pi/2.
    bounds[0] = 1.0
    return bounds[: n + 1]


def sample_parameters(family: str, seed: int, variant: int = 0) -> dict[str, Any]:
    """Deterministically sample geometry, independent of any process RNG state.

    The frozen family order and explicit PCG64 seed recipe are part of v1.
    Caller metadata should retain the entire returned dictionary and NumPy
    version, alongside MODULE_VERSION and this source file's SHA256.
    """
    if family not in FAMILY_NAMES:
        raise ValueError(f"unknown geometry family: {family!r}")
    if isinstance(seed, bool) or int(seed) != seed or int(seed) < 0:
        raise ValueError("seed must be a nonnegative integer")
    if isinstance(variant, bool) or int(variant) != variant or int(variant) < 0:
        raise ValueError("variant must be a nonnegative integer")
    rng = np.random.Generator(np.random.PCG64(np.random.SeedSequence([int(seed), FAMILY_NAMES.index(family) + 1, int(variant)])))
    p = {**_COMMON, **_DEFAULTS[family]}
    p["phase_rad"] = float(rng.uniform(0, 2 * math.pi))
    p["depth_phase_rad"] = float(rng.uniform(0.2, 1.3))
    def draw(key: str, low: float, high: float) -> None:
        p[key] = float(rng.uniform(low, high))
    if family == "woven_lissajous":
        draw("x_secondary", 0.13, 0.22); draw("y_secondary", 0.12, 0.20)
        draw("x_phase_rad", 0.25, 0.65); draw("secondary_phase_rad", 0.2, 1.1)
    elif family == "trefoil_projection":
        draw("double_harmonic_weight", 1.8, 2.2)
    elif family == "torus_knot_2_5":
        draw("tube_radius_ratio", 0.22, 0.36)
    elif family == "hypotrochoid_3_2":
        draw("major_weight", 1.8, 2.2); draw("pen_weight", 1.15, 1.55)
        draw("pen_phase_rad", -0.35, 0.35)
    elif family == "braided_spiral":
        draw("braid_weight", 0.16, 0.24); draw("slow_radial_weight", 0.04, 0.09)
        draw("angle_ripple_rad", 0.035, 0.075); draw("slow_phase_rad", 0.25, 0.9)
    elif family == "spherical_sweep":
        draw("latitude_amplitude_rad", 0.48, 0.72); draw("slow_latitude_rad", 0.02, 0.05)
        draw("latitude_phase_rad", 0.0, 0.4)
    elif family == "compound_harmonograph":
        draw("x_secondary", 0.18, 0.30); draw("y_secondary", 0.15, 0.25)
        draw("x_phase_rad", 0.4, 0.85); draw("x_secondary_phase_rad", -0.6, -0.15)
        draw("y_secondary_phase_rad", 0.7, 1.3)
    elif family == "bent_tube_braid":
        draw("centerline_weight", 0.45, 0.65); draw("lateral_weight", 0.23, 0.33)
        draw("secondary_weight", 0.05, 0.09); draw("lateral_phase_rad", 0.25, 0.7)
        draw("secondary_phase_rad", -0.2, 0.5)
    elif family == "rounded_square_shear":
        draw("rounding_weight", 0.08, 0.13); draw("shear_weight", 0.12, 0.20)
        draw("shear_phase_rad", 0.3, 1.0)
    elif family == "celtic_weave":
        draw("x_perturbation", 0.075, 0.13); draw("y_sixth_weight", 0.12, 0.20)
        draw("x_phase_rad", 0.3, 0.7); draw("y_sixth_phase_rad", 0.1, 0.5)
    _parameters(family, p)
    return {"parameter_seed": int(seed), "parameter_variant": int(variant), **p}


def catalog() -> dict[str, dict[str, Any]]:
    """Return serializable family descriptions, sources, defaults and bounds."""
    return {
        family: {
            "description": _DESCRIPTIONS[family],
            "dimension": 2,
            "units": "dimensionless",
            "period_in_u": 1.0,
            "derivative_variable": "u",
            "max_derivative_order": MAX_DERIVATIVE,
            "xy_coordinate_absolute_bound": [1.0, 1.0],
            "closed_periodic_geometry": True,
            "optional_depth_carrier": "dimensionless in [-1,1]; caller owns height",
            "sources": list(_SOURCE_REFS[family]),
            "default_parameters": copy.deepcopy({**_COMMON, **_DEFAULTS[family]}),
            "default_derivative_absolute_bounds": derivative_bounds(family).tolist(),
            "seed_recipe": "PCG64(SeedSequence([seed, frozen_family_index_1based, variant]))",
        }
        for family in FAMILY_NAMES
    }


def self_check() -> dict[str, Any]:
    """Check derivatives, periodic seams, analytic envelopes and RNG repeatability."""
    u = np.linspace(0.0, 1.0, 2001)
    probe = np.linspace(-0.11, 1.13, 113)
    delta = 2.0e-6
    result: dict[str, Any] = {"module_version": MODULE_VERSION, "numpy_version": np.__version__, "families": {}}
    for family in FAMILY_NAMES:
        p = sample_parameters(family, 20261016)
        assert p == sample_parameters(family, 20261016)
        assert p != sample_parameters(family, 20261016, 1)
        bound = derivative_bounds(family, p)
        cbound = carrier_derivative_bounds(family, p)
        errors = []
        ratios = []
        for n in range(MAX_DERIVATIVE + 1):
            actual = evaluate(family, u, p, n)
            depth = carrier(family, u, p, n)
            assert np.all(np.isfinite(actual)) and np.all(np.isfinite(depth))
            ratio = float(np.max(np.abs(actual) / bound[n]))
            cratio = float(np.max(np.abs(depth) / cbound[n]))
            assert ratio <= 1.0 + 1.0e-12, (family, n, ratio)
            assert cratio <= 1.0 + 1.0e-12, (family, n, cratio)
            ratios.append(max(ratio, cratio))
            for fn in (evaluate, carrier):
                ends = fn(family, np.array([0.0, 1.0]), p, n)
                assert np.max(np.abs(ends[0] - ends[1])) <= 2.0e-11 * max(1.0, float(np.max(np.abs(ends))))
            if n:
                for fn in (evaluate, carrier):
                    diff = (fn(family, probe + delta, p, n - 1) - fn(family, probe - delta, p, n - 1)) / (2 * delta)
                    analytic = fn(family, probe, p, n)
                    error = float(np.max(np.abs(diff - analytic)) / max(1.0, float(np.max(np.abs(analytic)))))
                    assert error < 3.0e-7, (family, n, error)
                    errors.append(error)
        result["families"][family] = {
            "passed": True,
            "max_normalized_derivative_error": max(errors),
            "max_dense_sample_to_global_bound_ratio": max(ratios),
        }
    result["passed"] = True
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    print(json.dumps(self_check() if args.self_check else catalog(), indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
