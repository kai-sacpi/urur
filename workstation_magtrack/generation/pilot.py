"""Generate the pre-calibration 60-record pilot and three commissioning plans.

All positions are nominal rod-tip targets in the sensor-centred plane1 frame.
They are deliberately *not* called calibrated magnet-centre execution plans.
CSV time and pose rows become immutable authority at creation.  Nothing here
changes an existing runtime recording or relaxes the mla_short environment.
"""

from __future__ import annotations

import argparse
from collections import Counter
import itertools
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

try:
    from .common import CSV_FIELDS, DT_S, motion_diagnostics, sample_grid, smootherstep7, write_authority, write_json_immutable
except ImportError:  # Direct invocation from the generation directory.
    from common import CSV_FIELDS, DT_S, motion_diagnostics, sample_grid, smootherstep7, write_authority, write_json_immutable


PROTOCOL = "real_collection_v2_20261016"
GENERATOR = "workstation_magtrack.pilot.v4"
DEFAULT_SEED = 20261016
V_TIERS = {0: 0.0, 1: 0.01, 2: 0.02, 3: 0.05}
W_TIERS = {0: 0.0, 1: 3.0, 2: 10.0, 3: 20.0}
LIMITS = {
    "center_acceleration_m_s2": 0.05,
    "center_jerk_m_s3": 0.25,
    "flange_angular_acceleration_deg_s2": 30.0,
    "flange_angular_jerk_deg_s3": 150.0,
}


def _integrated_smootherstep7(u):
    u = np.clip(np.asarray(u, dtype=float), 0.0, 1.0)
    return 7 * u**5 - 14 * u**6 + 10 * u**7 - 2.5 * u**8


def _speed_path(t, length, speed, ramp_s=3.0):
    """Analytical distance with a C3 speed ramp and an exact cruise plateau."""
    cruise_s = length / speed - ramp_s
    if cruise_s < 0.1:
        raise ValueError("Full route is too short to attain the declared cruise speed")
    duration_s = 2.0 * ramp_s + cruise_s
    tt = np.clip(np.asarray(t, dtype=float), 0.0, duration_s)
    s = np.empty_like(tt)
    accelerating = tt < ramp_s
    cruising = (tt >= ramp_s) & (tt <= ramp_s + cruise_s)
    braking = tt > ramp_s + cruise_s
    s[accelerating] = speed * ramp_s * _integrated_smootherstep7(tt[accelerating] / ramp_s)
    s[cruising] = 0.5 * speed * ramp_s + speed * (tt[cruising] - ramp_s)
    remaining = duration_s - tt[braking]
    s[braking] = length - speed * ramp_s * _integrated_smootherstep7(remaining / ramp_s)
    s[tt <= 0.0] = 0.0
    s[tt >= duration_s] = length
    return s, duration_s, cruise_s


def _padding(motion_s, target_s=20.0, minimum_hold_s=0.5):
    duration_s = max(target_s, motion_s + 2 * minimum_hold_s)
    hold_s = (duration_s - motion_s) / 2
    return duration_s, hold_s


def _windows(hold_s, motion_s, duration_s, extra=None):
    result = [
        {"phase": "hold", "purpose": "pre_target_rest", "start_s": 0.0, "end_s": hold_s},
        {"phase": "draw", "purpose": "effective_target_motion", "start_s": hold_s, "end_s": hold_s + motion_s},
        {"phase": "hold", "purpose": "post_target_rest", "start_s": hold_s + motion_s, "end_s": duration_s},
    ]
    if extra:
        result.extend(extra)
    return result


def _phases(t, windows, p, q):
    phases = np.full(len(t), "hold", dtype=object)
    for window in windows:
        if window["phase"] == "draw":
            phases[(t > window["start_s"]) & (t < window["end_s"])] = "draw"
    phases[0] = "draw"
    # The first sampled point after a continuous motion boundary can still
    # differ from its predecessor; that boundary row remains draw.  All hold
    # rows must be exact repetitions according to the existing CSV contract.
    changed = (np.max(np.abs(np.diff(p, axis=0)), axis=1) > 0.0) | (np.max(np.abs(np.diff(q, axis=0)), axis=1) > 0.0)
    serialized = [tuple(f"{value:.12f}" for value in pose) for pose in np.column_stack((p, q))]
    changed |= np.asarray([a != b for a, b in zip(serialized[:-1], serialized[1:])])
    phases[1:][changed] = "draw"
    return phases.tolist()


def _constant_orientation(n, axis=(1.0, 0.0, 0.0), angle_deg=0.0):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    q = Rotation.from_rotvec(np.deg2rad(angle_deg) * axis).as_quat()
    return np.tile(q, (n, 1))


def _position_line(family):
    # Every route has sufficient length for a genuine 5 cm/s cruise while
    # obeying the total centre-acceleration bound, including the high routes.
    configurations = [
        ([-0.060, -0.058, -0.153], [0.060, 0.058, -0.198]),
        ([-0.062, 0.057, -0.084], [0.062, -0.057, -0.139]),
        ([0.062, -0.060, -0.196], [-0.062, 0.060, -0.152]),
        ([0.063, 0.058, -0.142], [-0.063, -0.058, -0.092]),
    ]
    return tuple(np.asarray(p, dtype=float) for p in configurations[family])


def _line(family, v_tier):
    p0, p1 = _position_line(family)
    length = float(np.linalg.norm(p1 - p0))
    speed = V_TIERS[v_tier]
    _, moving_s, cruise_s = _speed_path(np.array([0.0]), length, speed)
    duration_s, hold_s = _padding(moving_s)
    t = sample_grid(duration_s)
    s, _, _ = _speed_path(t - hold_s, length, speed)
    p = p0 + s[:, None] * (p1 - p0) / length
    q = _constant_orientation(len(t), axis=(1, 0, 0) if family < 2 else (0, 1, 0), angle_deg=(-1)**family * 6)
    windows = _windows(hold_s, moving_s, duration_s)
    windows.append({"phase": "cruise", "start_s": hold_s + 3, "end_s": hold_s + 3 + cruise_s})
    return t, p, q, windows, {
        "geometry": "three_dimensional_straight_segment",
        "p0_m": p0.tolist(), "p1_m": p1.tolist(), "geometric_length_m": length,
        "motion_duration_s": moving_s, "cruise_duration_s": cruise_s,
        "speed_ramp_s": 3.0, "continuous_height_change": True,
    }


def _turn(family, v_tier):
    radius = 0.085
    sweep = 2.12
    high = family % 2 == 0
    # The revised high turn families place planned time in the upper two
    # centimetre bins.  Other high dynamic families span the full high band.
    # This is a new upstream design, not a runtime shift of the v1 sources.
    h0, h1 = ((0.181, 0.199) if family == 0 else
              (0.199, 0.181) if family == 2 else
              (0.083, 0.141) if family == 1 else (0.142, 0.089))
    slope = -(h1 - h0) / sweep
    path_radius = math.hypot(radius, slope)
    length = path_radius * sweep
    speed = V_TIERS[v_tier]
    _, moving_s, cruise_s = _speed_path(np.array([0.0]), length, speed)
    duration_s, hold_s = _padding(moving_s)
    t = sample_grid(duration_s)
    s, _, _ = _speed_path(t - hold_s, length, speed)
    theta = s / path_radius
    azimuth = [-0.70, 0.65, 2.25, 3.65][family]
    direction = 1.0 if family < 2 else -1.0
    phi = azimuth + direction * theta
    centers = [(0.010, 0.005), (-0.014, 0.005), (0.008, -0.010), (-0.005, -0.010)]
    cx, cy = centers[family]
    p = np.column_stack((cx + radius * np.cos(phi), cy + radius * np.sin(phi), -h0 + slope * theta))
    q = _constant_orientation(len(t), axis=(0, 1, 0) if family < 2 else (1, 1, 0), angle_deg=(-1)**family * 7)
    windows = _windows(hold_s, moving_s, duration_s)
    windows.append({"phase": "cruise", "start_s": hold_s + 3, "end_s": hold_s + 3 + cruise_s})
    return t, p, q, windows, {
        "geometry": "constant_pitch_circular_arc", "radius_xy_m": radius,
        "sweep_rad": sweep, "azimuth_rad": azimuth, "direction_sign": direction,
        "center_xy_m": [cx, cy], "height_endpoints_m": [h0, h1],
        "geometric_length_m": length, "curvature_per_m": radius / path_radius**2,
        "cruise_centripetal_acceleration_m_s2": speed**2 * radius / path_radius**2,
        "motion_duration_s": moving_s, "cruise_duration_s": cruise_s,
        "speed_ramp_s": 3.0, "continuous_height_change": True,
        "height_group": "high" if high else "low",
    }


def _angle_curve(t, w_tier, family, total_target_s=20.0):
    """One complete C3 tilt/reversal, with a nominal transverse-axis peak."""
    amplitude_deg = 22.0
    # max(smootherstep7') = 35/16 = 2.1875.  The peak is analytical,
    # independent of the source sample grid and the dwell time.
    leg_s = amplitude_deg * 2.1875 / W_TIERS[w_tier]
    motion_s = 2 * leg_s
    duration_s, hold_s = _padding(motion_s, total_target_s)
    tt = np.asarray(t, dtype=float) - hold_s
    forward = amplitude_deg * smootherstep7(np.clip(tt / leg_s, 0, 1))
    reverse = amplitude_deg * (1.0 - smootherstep7(np.clip((tt - leg_s) / leg_s, 0, 1)))
    angle_deg = np.where(tt <= leg_s, forward, reverse)
    angle_deg[(tt <= 0) | (tt >= motion_s)] = 0.0
    axes = np.asarray([(1, 0, 0), (0, 1, 0), (1, 1, 0), (1, -1, 0)], dtype=float)
    axis = axes[family % 4]
    axis /= np.linalg.norm(axis)
    sign = 1 if family % 2 == 0 else -1
    q = Rotation.from_rotvec(np.deg2rad(sign * angle_deg)[:, None] * axis).as_quat()
    return q, duration_s, hold_s, motion_s, {
        "axis_in_plane1_xyz": axis.tolist(), "angle_sign": sign,
        "one_way_tilt_deg": amplitude_deg, "one_way_duration_s": leg_s,
        "motion_duration_s": motion_s,
        "nominal_magnetic_axis_in_tool_xyz": [0.0, 0.0, 1.0],
        "magnetic_axis_status": "nominal_only_pending_low_height_axis_calibration",
        "pure_axial_yaw": False,
        "analytic_peak_axis_rate_deg_s": W_TIERS[w_tier],
        "analytic_peak_angular_acceleration_deg_s2": amplitude_deg * 7.513188404399293 / leg_s**2,
        "analytic_peak_angular_jerk_deg_s3": amplitude_deg * 52.5 / leg_s**3,
    }


def _orientation(family, w_tier):
    _, duration_s, hold_s, motion_s, geometry = _angle_curve(np.array([0.0]), w_tier, family)
    t = sample_grid(duration_s)
    q, _, _, _, _ = _angle_curve(t, w_tier, family)
    xyz = [(0.0, 0.0, -0.165), (0.030, -0.025, -0.105),
           (-0.025, 0.025, -0.175), (-0.035, -0.025, -0.135)][family]
    p = np.tile(np.asarray(xyz), (len(t), 1))
    geometry.update({"geometry": "fixed_nominal_center_transverse_tilt_and_return", "center_m": list(xyz), "continuous_height_change": False})
    return t, p, q, _windows(hold_s, motion_s, duration_s), geometry


def _mixed(index, session, v_tier, w_tier, high):
    # Two complete straight 3D legs provide an explicit, smooth reversal;
    # orientation evolves independently, rather than binding V and W tiers.
    dh = 0.035 if high else 0.050
    h0 = 0.152 if high else 0.087 + (index % 2) * 0.006
    h1 = h0 + dh
    length = 0.170
    xy_distance = math.sqrt(length**2 - dh**2)
    azimuth = (index * 0.71 + session * 0.29) % (2 * math.pi)
    d = np.asarray([math.cos(azimuth), math.sin(azimuth)])
    center_xy = np.asarray([0.01 * (-1)**index, -0.008 * (-1)**session])
    p0 = np.r_[center_xy - d * xy_distance / 2, -h0]
    p1 = np.r_[center_xy + d * xy_distance / 2, -h1]
    _, leg_s, cruise_s = _speed_path(np.array([0.0]), length, V_TIERS[v_tier])
    moving_position_s = 2 * leg_s
    _, angular_duration_s, _, moving_angle_s, _ = _angle_curve(np.array([0.0]), w_tier, index % 4)
    duration_s, hold_s = _padding(max(moving_position_s, moving_angle_s), max(20.0, angular_duration_s))
    # Each channel is centred within the full record independently.  This
    # preserves both requested peaks without time-scaling either channel.
    position_hold_s = (duration_s - moving_position_s) / 2
    t = sample_grid(duration_s)
    tt = t - position_hold_s
    outward, _, _ = _speed_path(tt, length, V_TIERS[v_tier])
    inward, _, _ = _speed_path(tt - leg_s, length, V_TIERS[v_tier])
    s = np.where(tt <= leg_s, outward, length - inward)
    s[(tt <= 0) | (tt >= 2 * leg_s)] = 0.0
    p = p0 + s[:, None] * (p1 - p0) / length
    q, _, angular_hold_s, _, angular_geometry = _angle_curve(t, w_tier, index % 4, duration_s)
    first = min(position_hold_s, angular_hold_s)
    last = max(position_hold_s + moving_position_s, angular_hold_s + moving_angle_s)
    windows = _windows(first, last - first, duration_s)
    windows += [
        {"phase": "center_motion", "start_s": position_hold_s, "end_s": position_hold_s + moving_position_s},
        {"phase": "axis_motion", "start_s": angular_hold_s, "end_s": angular_hold_s + moving_angle_s},
        {"phase": "cruise", "start_s": position_hold_s + 3, "end_s": position_hold_s + 3 + cruise_s},
        {"phase": "cruise", "start_s": position_hold_s + leg_s + 3, "end_s": position_hold_s + leg_s + 3 + cruise_s},
    ]
    return t, p, q, windows, {
        "geometry": "three_dimensional_straight_out_and_back_with_independent_transverse_tilt",
        "p0_m": p0.tolist(), "p1_m": p1.tolist(), "one_way_length_m": length,
        "geometric_length_m": 2 * length, "motion_duration_s": max(moving_position_s, moving_angle_s),
        "position_motion_duration_s": moving_position_s, "axis_motion_duration_s": moving_angle_s,
        "one_way_cruise_duration_s": cruise_s, "speed_ramp_s": 3.0,
        "continuous_height_change": True, "smooth_center_reversal": True,
        "orientation": angular_geometry,
    }


def _static(session, height_index):
    t = sample_grid(20.0)
    height = 0.15 + height_index * 0.01
    theta = (height_index * 1.17 + session * 0.83)
    radius = [0.0, 0.045, 0.070, 0.085, 0.055, 0.095][height_index]
    # The central position changes between sessions too.
    xy = [radius * math.cos(theta) + (0.008 if session == 2 else 0.0),
          radius * math.sin(theta) + (-0.006 if session == 2 else 0.0)]
    p = np.tile(np.r_[xy, -height], (len(t), 1))
    q = _constant_orientation(len(t), axis=(math.cos(theta), math.sin(theta), 0), angle_deg=6 + 2 * height_index + 3 * (session - 1))
    return t, p, q, [{"phase": "hold", "purpose": "effective_static_target", "start_s": 0.0, "end_s": 20.0}], {
        "geometry": "single_static_pose", "center_m": p[0].tolist(), "height_m": height,
        "continuous_height_change": False,
    }


def _measure(t, p, q):
    velocity = np.gradient(p, t, axis=0, edge_order=2)
    acceleration = np.gradient(velocity, t, axis=0, edge_order=2)
    jerk = np.gradient(acceleration, t, axis=0, edge_order=2)
    axes = Rotation.from_quat(q).apply(np.tile([0, 0, 1.0], (len(t), 1)))
    axis_rate = np.linalg.norm(np.gradient(axes, t, axis=0, edge_order=2), axis=1) * 180 / math.pi
    relative = Rotation.from_quat(q[1:]) * Rotation.from_quat(q[:-1]).inv()
    omega = relative.as_rotvec() / np.diff(t)[:, None]
    midpoint_t = (t[1:] + t[:-1]) / 2
    angular_accel = np.gradient(omega, midpoint_t, axis=0, edge_order=2)
    angular_jerk = np.gradient(angular_accel, midpoint_t, axis=0, edge_order=2)
    speeds = np.linalg.norm(velocity, axis=1)
    result = {
        "center_speed_max_m_s": float(speeds.max()),
        "center_speed_mean_m_s": float(np.trapezoid(speeds, t) / t[-1]),
        "center_speed_p95_m_s": float(np.percentile(speeds, 95)),
        "center_acceleration_max_m_s2": float(np.linalg.norm(acceleration, axis=1).max()),
        "center_jerk_max_m_s3": float(np.linalg.norm(jerk, axis=1).max()),
        "nominal_axis_rate_max_deg_s": float(axis_rate.max()),
        "flange_angular_speed_max_deg_s": float(np.linalg.norm(omega, axis=1).max() * 180 / math.pi),
        "flange_angular_acceleration_max_deg_s2": float(np.linalg.norm(angular_accel, axis=1).max() * 180 / math.pi),
        "flange_angular_jerk_max_deg_s3": float(np.linalg.norm(angular_jerk, axis=1).max() * 180 / math.pi),
        "height_min_m": float(-p[:, 2].max()), "height_max_m": float(-p[:, 2].min()),
        "xy_absolute_max_m": float(np.abs(p[:, :2]).max()),
        "derivative_method": "source_grid_finite_difference_diagnostics; analytic_time_law_also_saved",
    }
    if result["height_min_m"] < .08 - 1e-10 or result["height_max_m"] > .20 + 1e-10 or result["xy_absolute_max_m"] > .13 + 1e-10:
        raise ValueError(f"Candidate leaves registered target workspace: {result}")
    for key, upper in [
        ("center_acceleration_max_m_s2", .05), ("center_jerk_max_m_s3", .25),
        ("flange_angular_acceleration_max_deg_s2", 30), ("flange_angular_jerk_max_deg_s3", 150),
    ]:
        if result[key] > upper + 1e-8:
            raise ValueError(f"Candidate exceeds {key}: {result[key]} > {upper}")
    return result


def _record(root, ident, category, session, family, high, v_tier, w_tier, arrays, seed, role, stage="pilot"):
    t, p, q, windows, geometry = arrays
    measured = _measure(t, p, q)
    for key, target in [("center_speed_max_m_s", V_TIERS[v_tier]), ("nominal_axis_rate_max_deg_s", W_TIERS[w_tier])]:
        if target and not (0.985 * target <= measured[key] <= 1.002 * target):
            raise ValueError(f"Declared tier not reached: {ident} {key}={measured[key]}, requested={target}")
    path = root / "environments" / "mla_short" / "trajectories" / f"{ident}.csv"
    authority = write_authority(path, t, p, q, _phases(t, windows, p, q))
    predecessor_id = ident.removesuffix("_v4") + "_v3"
    record = {
        "trajectory_id": ident, "stage": stage, "category": category,
        "authority_revision": 4, "candidate_lifecycle_state": "active",
        "replaces_trajectory_id": predecessor_id,
        "replacement_reason": "Upstream v4 requires bit-exact serialized pose repetition before a row is labelled hold; all poses, times, balanced V/W×height design and complete routes retained",
        "protocol_version": PROTOCOL, "generator_version": GENERATOR,
        "session_id": f"pilot_session_{session:02d}" if stage == "pilot" else "commissioning",
        "session_number": session, "data_role": role, "family_id": family, "pair_id": family,
        "seed": seed, "height_group": "high" if high else "low",
        "v_tier": f"V{v_tier}", "w_tier": f"W{w_tier}",
        "nominal_center_speed_m_s": V_TIERS[v_tier],
        "nominal_axis_rate_deg_s": W_TIERS[w_tier],
        "linear_speed_definition": "cruise" if category in ("translation", "turn") else "intended_peak" if category == "mixed" else "zero",
        "angular_speed_definition": "intended_peak_of_nominal_transverse_magnetic_axis" if w_tier else "zero",
        "coordinate_frame": "plane1", "quaternion_order_csv": "xyzw",
        "height_definition": "h=-tip_z_m; origin=existing_mla_short_sensor_center",
        "reference_center": "nominal_rod_tip_pending_new_magnet_center_calibration",
        "nominal_tool_length_m": .190, "nominal_tool_diameter_m": .005,
        "magnetic_axis_status": "nominal_tool_axis_pending_joint_low_height_calibration",
        "installation_id": "pending_new_physical_installation_verification",
        "geometry_calibration_id": None, "synchronization_id": None,
        "tcp_configuration_binding": "nominal_mla_short_rod190_only_not_physical_controller_verification",
        "background_id": None, "background_measurement_requested_now": False,
        "label_reference_type": "planned_nominal_pose_only_no_actual_reference_labels",
        "simulation_only": True, "real_execution_authorized": False,
        "real_gate": "calibration+sync+commissioning+tier_pending",
        "tier_state": "generated_candidate_not_opened_on_real_robot",
        "time_parameterization": "analytical_C3_speed_ramps_and_C3_septic_position_tilt; source_timestamp_authority",
        "runtime_changes_permitted": {"scaling": False, "retiming": False, "cropping": False, "waypoint_skipping": False},
        "candidate_limits": LIMITS, "phase_windows": windows, "geometry": geometry,
        "measured_source_diagnostics": measured,
        "duration_exception_reason": "Full paired geometry and exact declared speed tiers preserved; slow complete route exceeds nominal 20s" if t[-1] > 20.01 else None,
        "hold_policy": "Exact constant XYZ and quaternion outside motion windows",
        "ingress_egress_parking": "Must use collision-checked nominal runtime home-to-first and return-home transitions; physical calibration rebinding pending",
        "ik_clearance_state": "pending_parent_full_path_simulation",
        "coarse_uncertainty_policy": "Parent precheck includes nominal board/tool uncertainty; no uncertainty claimed as new measured calibration",
        **authority,
    }
    # Keep an explicit diagnostic schema independent of common's summaries.
    record["duration_s"] = float(t[-1])
    write_json_immutable(root / "planning" / f"{ident}.json", record)
    return record


def _coverage(records):
    bins = np.asarray([.15, .16, .17, .18, .19, .20])
    histogram = np.zeros(5)
    total = 0.0
    high_s = 0.0
    all_types = Counter()
    type_session = Counter()
    for record in records:
        path = Path(record["source_file"])
        data = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")
        t = np.asarray(data["time_s"], dtype=float)
        h = -np.asarray(data["tip_z_m"], dtype=float)
        dt = np.diff(t)
        # Left-continuous interval accounting is explicit and deterministic.
        height = h[:-1]
        high = (height >= .15 - 1e-10) & (height <= .20 + 1e-10)
        high_s += float(dt[high].sum())
        total += float(dt.sum())
        for i in range(5):
            if i == 4:
                mask = (height >= bins[i] - 1e-10) & (height <= bins[i + 1] + 1e-10)
            else:
                mask = (height >= bins[i] - 1e-10) & (height < bins[i + 1] - 1e-10)
            histogram[i] += float(dt[mask].sum())
        all_types[record["category"]] += 1
        type_session[f"{record['session_id']}:{record['category']}"] += 1
    return {
        "planned_source_time_s": total, "planned_high_15_20cm_time_s": high_s,
        "planned_high_15_20cm_fraction": high_s / total,
        "height_1cm_bins_m": bins.tolist(), "height_bin_time_s": histogram.tolist(),
        "height_bin_fraction_of_all_time": (histogram / total).tolist(),
        "height_bin_target_fraction_of_all_time": [.12] * 5,
        "height_time_accounting": "left sample held over each authoritative source interval; includes all declared rests and motion",
        "category_counts": dict(all_types), "type_by_session_counts": dict(type_session),
        "coverage_limitations": [
            "Source candidates use nominal tool axis and nominal rod_tip; final magnet-centre coverage requires accepted calibration and new authority files.",
            "Time and height histogram deviations are reported, not hidden by weighting or deleting source samples.",
            "Complete slow paired routes can exceed20s; the explicit duration exception is retained per record.",
            "This is a finite 60-record pilot, not complete speed×height×direction×environment factorial coverage.",
        ],
    }


def generate(root: Path, config: dict | None = None) -> list[dict]:
    root = Path(root).resolve()
    config = config or {}
    seed = int(config.get("seed", DEFAULT_SEED))
    records = []
    for session in (1, 2):
        role = "pilot_development" if session == 1 else "pilot_cross_session_development"
        for i in range(6):
            ident = f"pilot_s{session:02d}_static_h{150 + 10*i:03d}_v0_w0_v4"
            records.append(_record(root, ident, "static", session, f"static_s{session:02d}_h{150+10*i:03d}", True, 0, 0, _static(session, i), seed, role))
        for local_family in range(2):
            family_index = 2 * (session - 1) + local_family
            high = family_index % 2 == 0
            for category, tiers, builder in [("translation", (1, 2, 3), _line), ("turn", (1, 2, 3), _turn), ("orientation", (1, 2, 3), _orientation)]:
                for tier in tiers:
                    vt, wt = (0, tier) if category == "orientation" else (tier, 0)
                    family = f"{category}_family_{family_index+1:02d}"
                    ident = f"pilot_s{session:02d}_{category}_f{family_index+1:02d}_{'high' if high else 'low'}_v{vt}_w{wt}_v4"
                    records.append(_record(root, ident, category, session, family, high, vt, wt, builder(family_index, tier), seed, role))

    assignments = {
        1: [(1, 1), (1, 2), (2, 1), (2, 3), (3, 2), (3, 3)],
        2: [(1, 1), (1, 3), (2, 2), (2, 3), (3, 1), (3, 2)],
    }
    for session, combinations in assignments.items():
        role = "pilot_development" if session == 1 else "pilot_cross_session_development"
        for i, (vt, wt) in enumerate(combinations):
            # Each session has one high and one low record at every V tier
            # and every W tier.  Alternating list indices binds W3 to low
            # heights for this particular non-diagonal V×W assignment.
            high = i in ({1, 2, 5} if session == 1 else {1, 2, 4})
            index = (session - 1) * 6 + i
            family = f"mixed_family_{index+1:02d}"
            ident = f"pilot_s{session:02d}_mixed_f{index+1:02d}_{'high' if high else 'low'}_v{vt}_w{wt}_v4"
            records.append(_record(root, ident, "mixed", session, family, high, vt, wt, _mixed(index, session, vt, wt, high), seed, role))

    commissioning = [
        ("commissioning_01_static_v0_w0_v4", "static", 0, 0, _static(1, 2)),
        ("commissioning_02_translation_v2_w0_v4", "translation", 2, 0, _line(0, 2)),
        ("commissioning_03_orientation_v0_w1_v4", "orientation", 0, 1, _orientation(0, 1)),
    ]
    commissioning_records = [
        _record(root, ident, category, 0, ident, True, vt, wt, arrays, seed, "engineering_commissioning", stage="commissioning")
        for ident, category, vt, wt, arrays in commissioning
    ]
    rng = np.random.default_rng(seed)
    order = []
    for session in (1, 2):
        group = [r for r in records if r["session_number"] == session]
        for i in rng.permutation(len(group)):
            order.append({"order_index": len(order) + 1, "session_order_index": sum(x["session_id"] == f"pilot_session_{session:02d}" for x in order) + 1,
                          "trajectory_id": group[i]["trajectory_id"], "session_id": group[i]["session_id"],
                          "data_role": group[i]["data_role"], "family_id": group[i]["family_id"],
                          "background_batch_proposal": f"pilot_s{session:02d}_batch_{sum(x['session_id'] == f'pilot_session_{session:02d}' for x in order)//6+1:02d}",
                          "real_gate": "calibration+sync+commissioning+tier_pending"})
    coverage = _coverage(records)
    replaced = [{"trajectory_id": r["replaces_trajectory_id"], "state": "replaced", "new_trajectory_id": r["trajectory_id"],
                 "reason": "strict serialized-pose hold boundary correction; previous source authority retained unchanged"} for r in commissioning_records + records]
    write_json_immutable(root / "planning" / "pilot_manifest_v4.json", {"protocol_version": PROTOCOL, "generator_version": GENERATOR, "seed": seed, "simulation_only": True, "state": "active", "records": records, "coverage": coverage, "replaced_records": replaced[3:]})
    write_json_immutable(root / "planning" / "pilot_execution_order_v4.json", {"seed": seed, "randomization": "pre_registered_numpy_default_rng_by_session", "records": order})
    write_json_immutable(root / "planning" / "pilot_coverage_v4.json", coverage)
    write_json_immutable(root / "planning" / "commissioning_manifest_v4.json", {"protocol_version": PROTOCOL, "simulation_only": True, "state": "active", "records": commissioning_records, "replaced_records": replaced[:3]})
    write_json_immutable(root / "planning" / "active_pilot_manifest_v4.json", {"protocol_version": PROTOCOL, "revision": 4, "active_records": commissioning_records + records, "replaced_records": replaced,
                          "do_not_compile": "v1, v2 and v3 predecessor IDs; retained solely for source provenance"})
    return commissioning_records + records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    result = generate(args.root)
    print(f"Generated {len(result)} immutable candidates: 3 commissioning +60pilot")


if __name__ == "__main__":
    main()
