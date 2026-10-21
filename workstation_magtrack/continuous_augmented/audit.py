"""Independent source-only audit for open, elastically augmented trajectories.

This module never edits authoritative CSV/metadata or invokes a robot/runtime.
Bernstein coefficients independently reconstruct space and timing; rotation-matrix
jets independently recover spatial SO(3) velocity, acceleration and jerk.  CSV
finite differences are diagnostics, never continuous-motion certificates.
"""
from __future__ import annotations

import argparse
import csv
from decimal import Decimal
import hashlib
import json
import math
from pathlib import Path
import platform
from typing import Any

import numpy as np
from numpy.polynomial import Polynomial
from scipy.spatial.transform import Rotation
import scipy

FIELDS = ("time_s", "frame_id", "tip_x_m", "tip_y_m", "tip_z_m",
          "tip_qx", "tip_qy", "tip_qz", "tip_qw", "phase")
LIMITS = {"acceleration_m_s2": .05, "jerk_m_s3": .25,
          "angular_acceleration_deg_s2": 30., "angular_jerk_deg_s3": 150.}
SEPTIC = Polynomial([0., 0., 0., 0., 35., -84., 70., -20.])


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def bernstein_polynomial(control: np.ndarray) -> Polynomial:
    """Convert scalar Bernstein coefficients to a power polynomial."""
    values = np.asarray(control, float)
    degree = len(values) - 1
    x = Polynomial([0., 1.])
    one_minus_x = Polynomial([1., -1.])
    return sum((float(v)*math.comb(degree, i)*x**i*one_minus_x**(degree-i)
                for i, v in enumerate(values)), Polynomial([0.]))


def _bezier(control: np.ndarray, x: np.ndarray, order: int = 0) -> np.ndarray:
    values = np.asarray(control, float)
    n = len(values)-1
    for k in range(order):
        values = np.diff(values, axis=0)*(n-k)
    n = len(values)-1
    if n < 0:
        return np.zeros((len(x),) + control.shape[1:])
    weights = np.array([math.comb(n, i)*x**i*(1.-x)**(n-i)
                        for i in range(n+1)]).T
    return weights @ values


def _control_bound(control: np.ndarray, order: int) -> float:
    values = np.asarray(control, float)
    n = len(values)-1
    for k in range(order):
        values = np.diff(values, axis=0)*(n-k)
    if not len(values):
        return 0.
    if values.ndim == 1:
        return float(np.max(np.abs(values)))
    return float(np.max(np.linalg.norm(values, axis=-1)))


def _power_to_bernstein(poly: Polynomial) -> np.ndarray:
    n = poly.degree()
    return np.array([sum(poly.coef[k]*math.comb(i, k)/math.comb(n, k)
                         for k in range(i+1)) for i in range(n+1)])


def _poly_global_abs(poly: Polynomial) -> float:
    """Convex-hull enclosure, with floating arithmetic safety allowance."""
    return float(np.max(np.abs(_power_to_bernstein(poly))))*(1.+1e-10)+1e-12


def timing_model(meta: dict[str, Any], s: np.ndarray) -> tuple[list[np.ndarray], np.ndarray]:
    density = np.asarray(meta["time_warp"]["density_control_points_positive"], float)
    if density.ndim != 1 or len(density) < 2 or not np.all(np.isfinite(density)):
        raise ValueError("invalid speed-density controls")
    if np.min(density) <= 0.:
        raise ValueError("speed-density controls must all be strictly positive")
    # The integral of a Bernstein polynomial has adjacent control differences
    # density/(degree+1), so this CDF is monotone on the whole interval.
    cdf_controls = np.r_[0., np.cumsum(density)/len(density)]
    cdf_controls /= cdf_controls[-1]
    cdf = [_bezier(cdf_controls[:, None], s, k)[:, 0] for k in range(5)]
    hcontrols = np.array([0., 0., 0., 0., 1., 1., 1., 1.])[:, None]
    u = [v[:, 0] for v in _compose(hcontrols, cdf)]
    # Avoid cancellation at exact endpoint, where these values are analytical.
    for k in (1, 2, 3):
        u[k] = np.where((s == 0.) | (s == 1.), 0., u[k])
    u[0] = np.where(s == 0., 0., np.where(s == 1., 1., u[0]))
    cdf_bounds = np.array([_control_bound(cdf_controls, k) for k in range(1, 5)])
    bounds = _compose_bounds(hcontrols, cdf_bounds)
    return u, bounds


def _compose(control: np.ndarray, u: list[np.ndarray]) -> list[np.ndarray]:
    f = [_bezier(control, u[0], k) for k in range(5)]
    a, b, c, d = [x[:, None] for x in u[1:]]
    return [f[0], f[1]*a, f[2]*a*a+f[1]*b,
            f[3]*a**3+3*f[2]*a*b+f[1]*c,
            f[4]*a**4+6*f[3]*a*a*b+f[2]*(3*b*b+4*a*c)+f[1]*d]


def _compose_bounds(control: np.ndarray, warp: np.ndarray) -> np.ndarray:
    f = [_control_bound(control, k) for k in range(1, 5)]
    a, b, c, d = warp
    return np.array([f[0]*a, f[1]*a*a+f[0]*b,
                     f[2]*a**3+3*f[1]*a*b+f[0]*c,
                     f[3]*a**4+6*f[2]*a*a*b+f[1]*(3*b*b+4*a*c)+f[0]*d])


def _trig_jets(theta: list[np.ndarray]) -> tuple[list[np.ndarray], list[np.ndarray]]:
    x, a, b, c, d = theta
    sn, cs = np.sin(x), np.cos(x)
    return ([sn, cs*a, -sn*a*a+cs*b,
             -cs*a**3-3*sn*a*b+cs*c,
             sn*a**4-6*cs*a*a*b-3*sn*b*b-4*sn*a*c+cs*d],
            [cs, -sn*a, -cs*a*a-sn*b,
             sn*a**3-3*cs*a*b-sn*c,
             cs*a**4+6*sn*a*a*b-3*cs*b*b-4*cs*a*c-sn*d])


def _axis_rotation_jets(theta: list[np.ndarray], axis: int) -> list[np.ndarray]:
    sn, cs = _trig_jets(theta)
    result = []
    i, j = ((1, 2), (2, 0), (0, 1))[axis]
    for k in range(5):
        matrix = np.zeros((len(theta[0]), 3, 3))
        if k == 0:
            matrix[:, axis, axis] = 1.
        matrix[:, i, i] = cs[k]
        matrix[:, j, j] = cs[k]
        matrix[:, i, j] = -sn[k]
        matrix[:, j, i] = sn[k]
        result.append(matrix)
    return result


def _product_jets(left: list[np.ndarray], right: list[np.ndarray]) -> list[np.ndarray]:
    return [sum((math.comb(k, i)*(left[i] @ right[k-i])
                 for i in range(k+1)), np.zeros_like(left[0])) for k in range(5)]


def _rotation_bounds(euler_global: np.ndarray) -> np.ndarray:
    product = np.r_[1., np.zeros(4)]
    for axis in range(3):
        a, b, c, d = euler_global[:, axis]
        bound = [1., a, b+a*a, c+3*a*b+a**3,
                 d+4*a*c+3*b*b+6*a*a*b+a**4]
        product = np.array([sum(math.comb(k, i)*product[i]*bound[k-i]
                                for i in range(k+1)) for k in range(5)])
    return product


def _angular_global_bounds(euler_global: np.ndarray) -> np.ndarray:
    """Cancellation-aware SO(3) bound, separate from matrix-jet evaluation.

    The identity omega = yaw' ez + pitch' Rz ey + roll' Rz Ry ex
    avoids bounding mutually cancelling R'R^T product terms. Fixed-axis
    rotation derivative norms are bounded by the explicit Bell expressions
    below. This is an analytic whole-interval bound, not sampled or FD.
    """
    roll, pitch, yaw = euler_global.T

    def axis_bounds(first_three: np.ndarray) -> np.ndarray:
        a, b, c = first_three
        return np.array([1., a, b+a*a, c+3*a*b+a**3])

    y_axis = axis_bounds(yaw[:3])
    # Product Rz(yaw) Ry(pitch) has the same Bell upper bound as the
    # componentwise sum of their absolute scalar derivative bounds.
    x_axis = axis_bounds(yaw[:3]+pitch[:3])
    return np.array([yaw[n]+sum(math.comb(n, k)*(pitch[k]*y_axis[n-k]+roll[k]*x_axis[n-k])
                               for k in range(n+1)) for n in range(4)])


def reconstruct(meta: dict[str, Any], s: np.ndarray) -> dict[str, Any]:
    """Reconstruct analytical motion independently of the generator module."""
    s = np.atleast_1d(np.asarray(s, float))
    if np.any(s < 0.) or np.any(s > 1.):
        raise ValueError("normalized time must be in [0, 1]")
    u, warp_global = timing_model(meta, s)
    controls = np.asarray(meta["curve"]["position_control_points_m"], float)
    euler_controls = np.asarray(meta["orientation"]["euler_xyz_control_points_rad"], float)
    position = _compose(controls, u)
    euler = _compose(euler_controls, u)
    axes = [_axis_rotation_jets([v[:, i] for v in euler], i) for i in range(3)]
    rotation = _product_jets(_product_jets(axes[2], axes[1]), axes[0])
    angular = []
    for k in range(4):
        skew = sum((math.comb(k, i)*(rotation[i+1] @ rotation[k-i].transpose(0, 2, 1))
                    for i in range(k+1)), np.zeros_like(rotation[0]))
        angular.append(np.stack([skew[:, 2, 1]-skew[:, 1, 2],
                                 skew[:, 0, 2]-skew[:, 2, 0],
                                 skew[:, 1, 0]-skew[:, 0, 1]], axis=-1)/2.)
    euler_global = np.column_stack([_compose_bounds(euler_controls[:, i:i+1], warp_global)
                                   for i in range(3)])
    angular_global = _angular_global_bounds(euler_global)
    return {"position_derivatives": position, "quaternion": Rotation.from_matrix(rotation[0]).as_quat(),
            "angular_derivatives_normalized": angular,
            "position_global_bounds_normalized": _compose_bounds(controls, warp_global),
            "angular_global_bounds_normalized": angular_global,
            "progress": u[0], "progress_derivative": u[1]}


def continuous_certificate(meta: dict[str, Any], samples: int = 32769) -> dict[str, Any]:
    """Grid max + analytic next-derivative Lipschitz enclosure covers all time."""
    if samples < 1025:
        raise ValueError("continuous audit requires at least 1025 grid samples")
    model = reconstruct(meta, np.linspace(0., 1., samples))
    duration = float(meta["draw_duration_s"])
    half_cell = 1./(2.*(samples-1))
    pb = model["position_global_bounds_normalized"]
    ab = model["angular_global_bounds_normalized"]
    ppeaks = np.array([np.linalg.norm(v, axis=1).max()
                      for v in model["position_derivatives"][1:4]])
    apeaks = np.array([np.linalg.norm(v, axis=1).max()
                      for v in model["angular_derivatives_normalized"][:3]])
    position = (ppeaks+half_cell*pb[1:])/duration**np.arange(1, 4)
    angular = np.rad2deg((apeaks+half_cell*ab[1:])/duration**np.arange(1, 4))
    return {"method": "independent_Bernstein_chain_rule_and_SO3_matrix_jets_with_global_next_derivative_Lipschitz_enclosure",
            "sample_count": samples, "maximum_normalized_cell_width": 2*half_cell,
            "position_derivative_bounds_m_s_orders_1_3": position.tolist(),
            "angular_derivative_bounds_deg_s_orders_1_3": angular.tolist(),
            "normalized_position_global_bounds_orders_1_4": pb.tolist(),
            "normalized_angular_global_bounds_orders_1_4": ab.tolist(),
            "passed": bool(position[1] <= .05 and position[2] <= .25
                           and angular[1] <= 30. and angular[2] <= 150.)}


def path_certificate(meta: dict[str, Any]) -> dict[str, Any]:
    controls = np.asarray(meta["curve"]["position_control_points_m"], float)
    direction = np.asarray(meta["curve"]["longitudinal_axis_xyz"], float)
    if controls.ndim != 2 or controls.shape[1] != 3 or len(controls) < 5:
        raise ValueError("position controls must be Nx3, with degree >= 4")
    if not np.all(np.isfinite(controls)) or not np.all(np.isfinite(direction)):
        raise ValueError("non-finite curve controls")
    if abs(np.linalg.norm(direction)-1.) > 1e-10:
        raise ValueError("longitudinal axis must be a unit vector")
    increments = np.diff(controls, axis=0) @ direction
    monotone = bool(np.all(increments > 0.))
    height = meta["height_profile"]
    h = -controls[:, 2]
    if height["mode"] == "fixed":
        height_pass = bool(np.max(np.abs(h-float(height["lower_m"]))) < 1e-11
                           and abs(float(height["upper_m"])-float(height["lower_m"])) < 1e-11)
    elif height["mode"] == "full_height_one_way":
        dh = np.diff(h)
        height_pass = bool((np.all(dh > 0.) or np.all(dh < 0.))
                           and abs(h.min()-float(height["lower_m"])) < 1e-11
                           and abs(h.max()-float(height["upper_m"])) < 1e-11)
    else:
        height_pass = False
    xy_pass = bool(np.all(np.abs(controls[:, :2]) <= .13+1e-12))
    range_pass = bool(h.min() >= .07-1e-12 and h.max() <= .20+1e-12)
    euler_controls = np.asarray(meta["orientation"]["euler_xyz_control_points_rad"], float)
    tilt_bound = float(np.rad2deg(np.max(np.abs(euler_controls[:, 0]))+
                                 np.max(np.abs(euler_controls[:, 1]))))
    tilt_pass = tilt_bound <= 25.
    return {"method": "strict_positive_Bernstein_derivative_projection_controls_plus_positive_time_density",
            "strictly_monotone_projection": monotone,
            "minimum_control_projection_increment_m": float(increments.min()),
            "direction_xyz": direction.tolist(),
            "endpoint_distance_m": float(np.linalg.norm(controls[-1]-controls[0])),
            "coordinate_control_min_m": controls.min(axis=0).tolist(),
            "coordinate_control_max_m": controls.max(axis=0).tolist(),
            "height_contract_pass": height_pass, "xy_box_pass": xy_pass,
            "height_range_pass": range_pass,
            "tilt_bound_deg": tilt_bound, "tilt_bound_pass": tilt_pass,
            "tilt_method": "whole-curve Euler-control convex hull plus rotation triangle inequality; yaw preserves plane1 Z",
            "passed": monotone and height_pass and xy_pass and range_pass and tilt_pass}


def so3_finite_difference_diagnostic(meta: dict[str, Any], samples: int = 4097) -> dict[str, Any]:
    s = np.linspace(0., 1., samples)
    model = reconstruct(meta, s)
    duration = float(meta["draw_duration_s"])
    t = s*duration
    rotations = Rotation.from_quat(model["quaternion"])
    tm = (t[:-1]+t[1:])/2.
    omega = (rotations[1:]*rotations[:-1].inv()).as_rotvec()/np.diff(t)[:, None]
    alpha = np.gradient(omega, tm, axis=0, edge_order=2)
    jerk = np.gradient(alpha, tm, axis=0, edge_order=2)
    exact = reconstruct(meta, (s[:-1]+s[1:])/2.)["angular_derivatives_normalized"]
    margins = slice(4, -4)
    errors = [float(np.max(np.linalg.norm(fd[margins]-a[margins]/duration**(k+1), axis=1)))
              for k, (fd, a) in enumerate(zip([omega, alpha, jerk], exact))]
    scales = [float(np.max(np.linalg.norm(a[margins]/duration**(k+1), axis=1)))
              for k, a in enumerate(exact[:3])]
    # FD is only an independent analytical implementation consistency check.
    tolerances = [2e-6+3e-4*x for x in scales]
    return {"method": "spatial_rotvec_increment_then_time_derivatives_not_Euler_differences",
            "role": "diagnostic_and_independent_formula_cross_check_not_continuous_certificate",
            "sample_count": samples, "max_vector_errors_rad_s_orders_1_3": errors,
            "comparison_tolerances": tolerances,
            "formula_cross_check_pass": all(e <= tol for e, tol in zip(errors, tolerances))}


def _resolve(root: Path, relative: str) -> Path:
    path = (root/relative).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError("authority path escapes batch")
    return path


def audit_record(root: Path, record: dict[str, Any], samples: int = 32769) -> dict[str, Any]:
    result: dict[str, Any] = {"trajectory_id": record.get("trajectory_id"),
                              "source_level_pass": False, "errors": [],
                              "robot_simulation": "not_run", "runtime_ready": False,
                              "real_execution_authorized": False}
    errors = result["errors"]
    try:
        csv_path = _resolve(root, record["csv_file"])
        metadata_path = _resolve(root, record["metadata_file"])
        result["authority"] = {"csv_sha256": sha256(csv_path), "metadata_sha256": sha256(metadata_path)}
        if result["authority"]["csv_sha256"] != record["sha256"]:
            errors.append("CSV SHA256 mismatch")
        if result["authority"]["metadata_sha256"] != record["metadata_sha256"]:
            errors.append("metadata SHA256 mismatch")
        meta = json.loads(metadata_path.read_text())
        if meta["trajectory_id"] != record["trajectory_id"]:
            errors.append("trajectory ID mismatch")
        if meta.get("seed") != record.get("seed"):
            errors.append("seed mismatch")
        if meta.get("group_id") != record.get("group_id"):
            errors.append("group ID mismatch")
        authority = meta.get("authority", {})
        if (authority.get("csv_file") != record["csv_file"]
                or authority.get("sha256") != result["authority"]["csv_sha256"]
                or abs(float(authority.get("duration_s", -1.))-float(record["duration_s"])) > 1e-10):
            errors.append("metadata authority and manifest/CSV disagree")
        context = (meta.get("environment_id") == "mla_short" and meta.get("frame_id") == "plane1"
                   and meta.get("position_units") == "m" and meta.get("quaternion_order") == "xyzw"
                   and meta.get("height_definition") == "height_m=-plane1_z_m"
                   and meta.get("source_sample_period_s") == .05)
        safety = meta.get("safety_status", {})
        safety_pass = (safety.get("simulation_only") is True
                       and safety.get("real_execution_authorized") is False
                       and safety.get("source_authority_may_be_modified_by_runtime") is False
                       and safety.get("hard_collision_clearance_m") == .005
                       and safety.get("rod_length_m") == .190
                       and safety.get("rod_radius_m") == .0025)
        if not context:
            errors.append("metadata environment/frame/unit/sample-grid context mismatch")
        if not safety_pass:
            errors.append("metadata nominal-only rod190/5mm-clearance safety contract mismatch")
        if meta.get("orientation", {}).get("composition") != "Rz(yaw) Ry(pitch) Rx(roll)":
            errors.append("orientation composition is not the frozen SO3 formula")
        with csv_path.open(newline="") as stream:
            reader = csv.DictReader(stream)
            if tuple(reader.fieldnames or ()) != FIELDS:
                raise ValueError("CSV header differs from authoritative waypoint contract")
            rows = list(reader)
        if len(rows) < 25:
            raise ValueError("too few source rows")
        if authority.get("source_pose_count") != len(rows):
            errors.append("metadata authority source pose count mismatch")
        decimal_times = [Decimal(row["time_s"]) for row in rows]
        if decimal_times[0] != 0 or any(b-a != Decimal("0.05")
                for a, b in zip(decimal_times[:-1], decimal_times[1:])):
            errors.append("time grid must start at zero and advance exactly 0.05 s")
        t = np.array([float(v) for v in decimal_times])
        p = np.array([[float(row[k]) for k in FIELDS[2:5]] for row in rows])
        q = np.array([[float(row[k]) for k in FIELDS[5:9]] for row in rows])
        if not (np.all(np.isfinite(p)) and np.all(np.isfinite(q)) and np.all(np.isfinite(t))):
            raise ValueError("CSV contains non-finite numeric values")
        if set(row["frame_id"] for row in rows) != {"plane1"}:
            errors.append("frame_id must be plane1")
        if np.max(np.abs(np.linalg.norm(q, axis=1)-1.)) > 2e-10:
            errors.append("non-unit quaternion")
        if np.min(np.sum(q[1:]*q[:-1], axis=1)) <= 0.:
            errors.append("quaternion hemisphere discontinuity")
        duration = float(meta["draw_duration_s"])
        hold_start, hold_end = float(meta["hold_start_s"]), float(meta["hold_end_s"])
        if hold_start != .5 or hold_end != .5:
            errors.append("hold durations must be exactly 0.5 s")
        if abs(t[-1]-(duration+hold_start+hold_end)) > 1e-10:
            errors.append("source duration differs from metadata")
        if abs(float(record["duration_s"])-t[-1]) > 1e-10:
            errors.append("source duration differs from manifest")
        first_hold = np.flatnonzero(t <= hold_start+1e-12)
        last_hold = np.flatnonzero(t >= hold_start+duration-1e-12)
        holds_pass = True
        for indices in [first_hold, last_hold]:
            reference = tuple(rows[int(indices[0])][k] for k in FIELDS[2:9])
            if len(indices) != 11 or any(tuple(rows[int(i)][k] for k in FIELDS[2:9]) != reference
                                         for i in indices):
                holds_pass = False
        if not holds_pass:
            errors.append("hold pose fields must be byte-exact and contain 11 samples per 0.5 s")
        result["byte_exact_holds_pass"] = holds_pass
        phase_pass = True
        for i, row in enumerate(rows):
            expected_phase = ("draw" if i == 0 or hold_start+1e-12 < t[i] <= hold_start+duration+1e-12
                              else "hold")
            if row["phase"] != expected_phase:
                phase_pass = False
            if row["phase"] == "hold" and (i == 0 or any(row[k] != rows[i-1][k] for k in FIELDS[2:9])):
                phase_pass = False
        result["phase_contract_pass"] = phase_pass
        if not phase_pass:
            errors.append("phase contract failed: first row draw; hold rows must be strictly byte-exact stationary")
        s = np.clip((t-hold_start)/duration, 0., 1.)
        model = reconstruct(meta, s)
        position_error = float(np.max(np.linalg.norm(p-model["position_derivatives"][0], axis=1)))
        rotation_error = float(np.max((Rotation.from_quat(q)*Rotation.from_quat(model["quaternion"]).inv()).magnitude()))
        result["formula_reconstruction_error"] = {"position_m": position_error, "orientation_rad": rotation_error}
        if position_error > 2e-10 or rotation_error > 2e-9:
            errors.append("CSV does not reproduce frozen analytical motion")
        endpoints = reconstruct(meta, np.array([0., 1.]))
        endpoint_errors = [float(np.max(np.abs(v))) for v in endpoints["position_derivatives"][1:4]]
        endpoint_errors += [float(np.max(np.abs(v))) for v in endpoints["angular_derivatives_normalized"][:3]]
        result["analytical_endpoint_v_a_jerk_zero_pass"] = bool(max(endpoint_errors) < 1e-8)
        if not result["analytical_endpoint_v_a_jerk_zero_pass"]:
            errors.append("analytical endpoint velocity/acceleration/jerk are nonzero")
        result["path"] = path_certificate(meta)
        result["continuous_motion"] = continuous_certificate(meta, samples)
        result["so3_finite_difference"] = so3_finite_difference_diagnostic(meta)
        for key in ["path", "continuous_motion"]:
            if not result[key]["passed"]:
                errors.append(f"{key} failed")
        if not result["so3_finite_difference"]["formula_cross_check_pass"]:
            errors.append("SO3 independent finite-difference formula cross-check failed")
        result["source_level_pass"] = not errors
    except (ValueError, KeyError, TypeError, OSError, ArithmeticError) as exc:
        errors.append(f"{type(exc).__name__}: {exc}")
    return result


def audit_batch(manifest_path: Path, samples: int = 32769) -> dict[str, Any]:
    manifest = json.loads(manifest_path.read_text())
    records = manifest["records"]
    ids = [record["trajectory_id"] for record in records]
    if len(ids) != len(set(ids)):
        raise ValueError("manifest trajectory IDs are not unique")
    group_counts = {group: sum(record.get("group_id") == group for record in records)
                    for group in ("fixed_height", "full_height_one_way")}
    batch_errors = []
    if len(records) != 12 or group_counts != {"fixed_height": 6, "full_height_one_way": 6}:
        batch_errors.append("preview must contain exactly 6 fixed-height plus 6 one-way full-height records")
    fixed_heights = []
    for record in records:
        if record.get("group_id") == "fixed_height":
            meta = json.loads(_resolve(manifest_path.parent, record["metadata_file"]).read_text())
            fixed_heights.append(float(meta["height_profile"]["lower_m"]))
    if not np.array_equal(np.round(sorted(fixed_heights), 12), np.array([.07, .09, .12, .15, .18, .20])):
        batch_errors.append("fixed-height preview layers must be 70/90/120/150/180/200 mm exactly once")
    results = [audit_record(manifest_path.parent, record, samples) for record in records]
    return {"schema_version": "continuous_augmented_source_audit_v1", "manifest_sha256": sha256(manifest_path),
            "source_only": True, "robot_simulation": "not_run", "runtime_ready": False,
            "real_execution_authorized": False,
            "validation_provenance": {"audit_script_sha256": sha256(Path(__file__).resolve()),
                                      "python": platform.python_version(), "numpy": np.__version__,
                                      "scipy": scipy.__version__,
                                      "independence": "No imports from generator/core/runtime; independent explicit Bernstein chain-rule reconstruction and matrix-jet SO3 implementation."},
            "scope": "Nominal rod_tip in plane1, h=-z; not magnet-centre truth or robot feedback. 70 mm is eligible for nominal simulation only; full robot/rod collision checks remain required.",
            "limits": LIMITS, "count": len(results),
            "group_counts": group_counts, "batch_contract_errors": batch_errors,
            "passed": sum(record["source_level_pass"] for record in results),
            "all_source_level_pass": not batch_errors and all(record["source_level_pass"] for record in results),
            "trajectories": results}


def write_reports(report: dict[str, Any], output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    (output/"source_audit.json").write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)+"\n")
    lines = ["# Open augmented trajectory source audit", "", f"Source-level passed: {report['passed']}/{report['count']}.",
             "", "This is not robot IK, full collision, dynamics, mock playback or real-hardware approval.", "",
             "All CSVs and metadata were read-only. Continuous limits use analytic SO(3)/position derivatives and Lipschitz enclosure; finite differences are diagnostics only.", "",
             "| Trajectory | Source | Errors |", "| --- | --- | --- |"]
    lines.extend(f"| {row['trajectory_id']} | {'PASS' if row['source_level_pass'] else 'FAIL'} | {'; '.join(row['errors'])} |"
                 for row in report["trajectories"])
    (output/"source_audit.md").write_text("\n".join(lines)+"\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--samples", type=int, default=32769)
    args = parser.parse_args()
    manifest_path = args.manifest.resolve()
    report = audit_batch(manifest_path, args.samples)
    write_reports(report, args.output or manifest_path.parent/"reports")
    print(f"source-only audit: {report['passed']}/{report['count']} passed; robot simulation not run")
    return 0 if report["all_source_level_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
