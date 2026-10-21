"""Audit immutable source CSVs and produce planned, nominal coverage figures.

Run from the parent repository with::

    python -m workstation_magtrack.validation.audit --workspace workstation_magtrack

Only ``reports/`` is written.  CSVs and planning manifests are never modified.
Sampled Cartesian diagnostics do not establish continuous robot feasibility.
The runtime evidence join is deliberately separate from nominal source checks.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation


FIELDS = ("time_s", "frame_id", "tip_x_m", "tip_y_m", "tip_z_m",
          "tip_qx", "tip_qy", "tip_qz", "tip_qw", "phase")
TYPES = ("static", "translation", "turn", "orientation", "mixed")
TYPE_LABELS = {"static": "Static", "translation": "Translation", "turn": "Turning",
               "orientation": "Centre-fixed tilt", "mixed": "Mixed 3D"}
LIMITS = {"max_acceleration_m_s2": .05, "max_jerk_m_s3": .25,
          "max_angular_acceleration_deg_s2": 30., "max_angular_jerk_deg_s3": 150.}
HEIGHT_EDGES_M = np.asarray([.08, .15, .16, .17, .18, .19, .20])
AUTHORITY_WARNING = (
    "Planned nominal rod_tip in plane1; not an accepted magnetic-centre trajectory in S. "
    "Accepted R_S_B, t_S_B, r_F_C, magnetic axis, effective TCP and installation ID are pending. "
    "The hospital S origin may differ by about 10 mm from the board visual origin. "
    "No physical feasibility, actual feedback or absolute reference accuracy is inferred."
)


@dataclass
class Authority:
    metadata: dict[str, Any]
    path: Path
    t: np.ndarray
    p: np.ndarray
    q: np.ndarray
    phases: np.ndarray
    series: dict[str, np.ndarray]
    audit: dict[str, Any]


def _json(path: Path) -> Any:
    return json.loads(path.read_text())


def _write_json(path: Path, value: Any) -> None:
    def scalar(item: Any) -> Any:
        if isinstance(item, np.generic):
            return item.item()
        raise TypeError(f"Unsupported report value: {type(item).__name__}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False,
                               default=scalar) + "\n")


def _norm(x: np.ndarray) -> np.ndarray:
    return np.linalg.norm(x, axis=1)


def motion_series(t: np.ndarray, p: np.ndarray, q: np.ndarray) -> dict[str, np.ndarray]:
    """Independent vector derivatives and geodesic nominal-axis rates."""
    def grad(x: np.ndarray, tt: np.ndarray = t) -> np.ndarray:
        return np.gradient(x, tt, axis=0, edge_order=2 if len(tt) > 2 else 1)

    v = grad(p)
    a = grad(v)
    j = grad(a)
    rotations = Rotation.from_quat(q)
    axes = rotations.apply(np.tile([0., 0., 1.], (len(q), 1)))
    tm = (t[:-1] + t[1:]) / 2
    omega = (rotations[1:] * rotations[:-1].inv()).as_rotvec() / np.diff(t)[:, None]
    alpha = grad(omega, tm)
    angular_jerk = grad(alpha, tm)
    axis_rate = np.arctan2(_norm(np.cross(axes[:-1], axes[1:])),
                           np.einsum("ij,ij->i", axes[:-1], axes[1:])) / np.diff(t)
    flange = p - rotations.apply(np.tile([0., 0., .19], (len(q), 1)))
    return {"t": t, "p": p, "q": q, "v": v, "a": a, "j": j,
            "speed": _norm(v), "acceleration": _norm(a), "jerk": _norm(j),
            "tm": tm, "omega": np.rad2deg(_norm(omega)),
            "alpha": np.rad2deg(_norm(alpha)), "angular_jerk": np.rad2deg(_norm(angular_jerk)),
            "axis_rate": np.rad2deg(axis_rate), "axes": axes, "flange": flange}


def _resolve_source(workspace: Path, record: dict[str, Any]) -> Path:
    path = Path(record["source_file"])
    if path.is_absolute() and path.exists():
        return path
    if not path.is_absolute() and (workspace / path).exists():
        return workspace / path
    # Manifests made remotely retain their original absolute workspace paths.
    local = workspace / "environments" / "mla_short" / "trajectories" / path.name
    if local.exists():
        return local
    raise FileNotFoundError(path)


def read_authority(workspace: Path, record: dict[str, Any]) -> Authority:
    path = _resolve_source(workspace, record)
    payload = path.read_bytes()
    reader = csv.DictReader(payload.decode("utf-8").splitlines())
    if tuple(reader.fieldnames or ()) != FIELDS:
        raise ValueError(f"{path.name}: noncanonical CSV header")
    rows = list(reader)
    if len(rows) < 4:
        raise ValueError(f"{path.name}: too few rows for motion audit")
    if any(None in row or any(value is None for value in row.values()) for row in rows):
        raise ValueError(f"{path.name}: malformed row")
    ns: list[int] = []
    try:
        for row in rows:
            value = Decimal(row["time_s"]) * Decimal(1_000_000_000)
            if not value.is_finite() or value != value.to_integral_value():
                raise ValueError("timestamp is not exactly representable in ROS nanoseconds")
            ns.append(int(value))
    except (InvalidOperation, ValueError) as error:
        raise ValueError(f"{path.name}: invalid authoritative time: {error}") from error
    if ns[0] != 0 or any(b <= a for a, b in zip(ns, ns[1:])):
        raise ValueError(f"{path.name}: time must start at zero and strictly increase")
    t = np.asarray(ns, dtype=np.int64) / 1e9
    p = np.asarray([[float(row[key]) for key in FIELDS[2:5]] for row in rows])
    q = np.asarray([[float(row[key]) for key in FIELDS[5:9]] for row in rows])
    phases = np.asarray([row["phase"] for row in rows])
    if any(row["frame_id"] != "plane1" for row in rows):
        raise ValueError(f"{path.name}: mixed or unsupported frame")
    if not np.isfinite(p).all() or not np.isfinite(q).all():
        raise ValueError(f"{path.name}: non-finite pose")
    norm_error = float(np.max(np.abs(_norm(q) - 1)))
    if norm_error > 2e-10:
        raise ValueError(f"{path.name}: quaternion norm error {norm_error}")
    if phases[0] != "draw" or set(phases) - {"draw", "hold"}:
        raise ValueError(f"{path.name}: unsupported phase or initial phase")
    moving_hold = [i for i in range(1, len(rows)) if phases[i] == "hold" and
                   any(rows[i][key] != rows[i-1][key] for key in FIELDS[2:9])]
    if moving_hold:
        raise ValueError(f"{path.name}: hold rows move: {moving_hold[:5]}")
    sha = hashlib.sha256(payload).hexdigest()
    if record.get("source_sha256") and record["source_sha256"] != sha:
        raise ValueError(f"{path.name}: authority SHA-256 mismatch")
    if record.get("source_pose_count") and record["source_pose_count"] != len(t):
        raise ValueError(f"{path.name}: authority row-count mismatch")
    if abs(record.get("duration_s", t[-1]) - t[-1]) > 5e-10:
        raise ValueError(f"{path.name}: duration metadata mismatch")
    series = motion_series(t, p, q)
    peaks = {"max_speed_m_s": float(series["speed"].max()),
             "mean_speed_m_s": float(np.trapezoid(series["speed"], t) / t[-1]),
             "p95_speed_m_s": float(np.percentile(series["speed"], 95)),
             "max_acceleration_m_s2": float(series["acceleration"].max()),
             "max_jerk_m_s3": float(series["jerk"].max()),
             "max_angular_speed_deg_s": float(series["omega"].max()),
             "max_axis_rate_deg_s": float(series["axis_rate"].max()),
             "max_angular_acceleration_deg_s2": float(series["alpha"].max()),
             "max_angular_jerk_deg_s3": float(series["angular_jerk"].max())}
    violations = {key: {"observed": peaks[key], "limit": limit}
                  for key, limit in LIMITS.items() if peaks[key] > limit + 1e-7}
    endpoint_equal = all(np.array_equal(array[0], array[1]) and np.array_equal(array[-1], array[-2])
                         for array in (p, q))
    audit = {"trajectory_id": record["trajectory_id"], "source_file": str(path.relative_to(workspace)),
             "source_sha256": sha, "source_pose_count": len(t), "duration_s": float(t[-1]),
             "schema_valid": True, "exact_ros_nanoseconds": True,
             "time_ns_first": ns[0], "time_ns_last": ns[-1],
             "quaternion_max_unit_norm_error": norm_error, "hold_rows_stationary_exact": True,
             "hold_row_count": int(np.count_nonzero(phases == "hold")),
             "sampled_endpoints_have_exact_stationary_neighbours": bool(endpoint_equal),
             "sampled_motion_limits_pass": not violations, "motion_limit_violations": violations,
             "diagnostics": peaks,
             "nominal_height_range_m": [float(-p[:, 2].max()), float(-p[:, 2].min())],
             "nominal_axis_min_xyz": series["axes"].min(axis=0).tolist(),
             "nominal_axis_max_xyz": series["axes"].max(axis=0).tolist(),
             "diagnostic_scope": "source-grid vector-norm finite differences; separate continuous formula and full robot checks required"}
    audit["coarse_rod_sensor_face_precheck"] = coarse_rod_clearance(p, q)
    audit["continuous_cartesian_bounds"] = continuous_bounds(record, t, p, q)
    if record.get("stage") in ("pilot", "commissioning"):
        tier_findings = []
        for key, target in (("max_speed_m_s", record["nominal_center_speed_m_s"]),
                            ("max_axis_rate_deg_s", record["nominal_axis_rate_deg_s"])):
            if target > 0 and not .985 * target <= peaks[key] <= 1.002 * target:
                tier_findings.append(f"{key} source peak {peaks[key]} does not reach declared {target}")
            elif target == 0 and peaks[key] > 1e-6:
                tier_findings.append(f"{key} violates declared zero tier")
        category = record["category"]
        if category in ("static", "orientation") and not np.all(p == p[0]):
            tier_findings.append("fixed centre changes position")
        if category in ("static", "translation", "turn") and not np.all(q == q[0]):
            tier_findings.append("fixed nominal axis changes orientation")
        audit["declared_source_tiers_pass"] = not tier_findings
        audit["declared_source_tier_violations"] = tier_findings
    return Authority(record, path, t, p, q, phases, series, audit)


def coarse_rod_clearance(p: np.ndarray, q: np.ndarray) -> dict[str, Any]:
    """Conservative board-face envelope; this is not full-scene collision IK."""
    axes = Rotation.from_quat(q).apply(np.tile([0., 0., 1.], (len(q), 1)))
    nz = axes[:, 2]
    worst_nz = np.cos(np.maximum(np.arccos(np.clip(nz, -1., 1.)) - np.deg2rad(2.), 0.))
    # A linear height function along the rod attains its maximum at an end.
    flange_z = p[:, 2] - .19 * nz + .003
    tip_z = p[:, 2] + .003 + .19 * (worst_nz - nz)
    rod_surface_z = np.maximum(flange_z, tip_z) + .0025
    analytic_margin = -.065 - tip_z.max()
    physical_margin = -.060 - rod_surface_z.max() - .005
    return {"scope": "boardfaceonly/allsourceposes;robotworldcollisionandIKseparate",
            "flange_position_uncertainty_radius_m": .003, "orientation_cone_half_angle_deg": 2.,
            "nominal_rod_length_m": .190, "rod_radius_m": .0025,
            "unchanged_board_keepout_top_z_m": -.060, "unchanged_analytical_tip_boundary_z_m": -.065,
            "minimum_analytical_tip_margin_m": float(analytic_margin),
            "minimum_non_tip_clearance_margin_m_after_required5mm": float(physical_margin),
            "passed": bool(min(analytic_margin, physical_margin) >= -1e-10)}


def continuous_bounds(record: dict[str, Any], t: np.ndarray, p: np.ndarray, q: np.ndarray) -> dict[str, Any]:
    """Independent sufficient Cartesian bounds from declared analytic laws.

    The source grid cannot prove between-sample properties. These bounds use
    the separately declared speed-ramp / septic-rotation law and exact endpoint
    displacement. They neither change authoritative times nor replace IK checks.
    """
    d1, d2, d3 = 2.1875, 7.513188404399293, 52.5
    linear_accel, linear_jerk, angular_accel, angular_jerk = [], [], [], []
    geometry = record.get("geometry", {})
    if record.get("category") == "static":
        linear_accel.append(0.)
        linear_jerk.append(0.)
        angular_accel.append(0.)
        angular_jerk.append(0.)
    if record.get("category") in ("translation", "turn", "mixed"):
        speed = float(record["nominal_center_speed_m_s"])
        ramp = float(geometry["speed_ramp_s"])
        tangent_a, tangent_j = speed * d1 / ramp, speed * d2 / ramp**2
        curvature = float(geometry.get("curvature_per_m", 0.))
        if curvature:
            path_radius = float(geometry["geometric_length_m"]) / float(geometry["sweep_rad"])
            linear_accel.append(float(np.hypot(tangent_a, curvature * speed**2)))
            linear_jerk.append(tangent_j + 3 * curvature * speed * tangent_a + curvature * speed**3 / path_radius)
        else:
            linear_accel.append(tangent_a)
            linear_jerk.append(tangent_j)
    angular = geometry.get("orientation", geometry)
    if "one_way_tilt_deg" in angular:
        angle, duration = float(angular["one_way_tilt_deg"]), float(angular["one_way_duration_s"])
        angular_accel.append(angle * d2 / duration**2)
        angular_jerk.append(angle * d3 / duration**3)
    if record.get("category") in ("static", "translation", "turn"):
        angular_accel.append(0.)
        angular_jerk.append(0.)
    if record.get("category") == "orientation":
        linear_accel.append(0.)
        linear_jerk.append(0.)
    # Calibration and synchronization record each analytic segment separately.
    for phase in record.get("phases", []):
        if "continuous_analytic_bounds" not in phase:
            continue
        t0, t1 = phase["start_time_s"], phase["end_time_s"]
        i, k = int(np.argmin(abs(t - t0))), int(np.argmin(abs(t - t1)))
        if abs(t[i] - t0) > 1e-8 or abs(t[k] - t1) > 1e-8:
            raise ValueError("Analytic phase endpoint is absent from authority")
        duration = t1 - t0
        distance = float(np.linalg.norm(p[k] - p[i]))
        angle = np.rad2deg((Rotation.from_quat(q[k]) * Rotation.from_quat(q[i]).inv()).magnitude())
        linear_accel.append(distance * d2 / duration**2)
        linear_jerk.append(distance * d3 / duration**3)
        angular_accel.append(float(angle) * d2 / duration**2)
        angular_jerk.append(float(angle) * d3 / duration**3)
    values = {"max_acceleration_m_s2": max(linear_accel, default=0.),
              "max_jerk_m_s3": max(linear_jerk, default=0.),
              "max_angular_acceleration_deg_s2": max(angular_accel, default=0.),
              "max_angular_jerk_deg_s3": max(angular_jerk, default=0.)}
    return {"available": bool(linear_accel and angular_accel), "bounds": values,
            "limits_pass": bool(all(values[key] <= upper + 1e-9 for key, upper in LIMITS.items())),
            "scope": "sufficientCartesianboundsfor_declared_analyticlaw;3Dhelicalcurvatureandtorsionincluded;notjointdynamicsorfeedback",
            "rest_boundary_design": "declaredsepticlawzeroendpointvelocity/acceleration/jerk;allCSVendpointneighboursindependentlychecked"}


def weighted_bins(t: np.ndarray, heights: np.ndarray, edges: np.ndarray) -> np.ndarray:
    """Exact duration in each half-open bin for linearly connected samples.

    A stationary value exactly at the final edge belongs to the final bin.
    Crossing intervals are split at every bin edge, not counted by frame count.
    """
    result = np.zeros(len(edges) - 1)
    for t0, t1, h0, h1 in zip(t[:-1], t[1:], heights[:-1], heights[1:]):
        h0, h1 = round(float(h0), 11), round(float(h1), 11)
        cuts = [0., 1.]
        if h1 != h0:
            cuts.extend((edge - h0) / (h1 - h0) for edge in edges if 0 < (edge - h0) / (h1 - h0) < 1)
        cuts = sorted(cuts)
        for lo, hi in zip(cuts[:-1], cuts[1:]):
            height = h0 + (h1 - h0) * (lo + hi) / 2
            index = int(np.searchsorted(edges, height, side="right") - 1)
            if abs(height - edges[-1]) < 1e-10:
                index = len(result) - 1
            if 0 <= index < len(result):
                result[index] += (t1 - t0) * (hi - lo)
    return result


def _counts(values: list[str]) -> dict[str, int]:
    return dict(sorted(Counter(values).items()))


def pilot_coverage(authorities: list[Authority]) -> dict[str, Any]:
    records = [a for a in authorities if a.metadata.get("stage") == "pilot"]
    if not records:
        return {"status": "pilot_manifest_not_available"}
    total_s = sum(a.t[-1] for a in records)
    bins = sum((weighted_bins(a.t, -a.p[:, 2], HEIGHT_EDGES_M) for a in records), np.zeros(6))
    matrix = {f"{session}:{category}": sum(a.metadata["session_id"] == session and
                a.metadata["category"] == category for a in records)
              for session in sorted({a.metadata["session_id"] for a in records}) for category in TYPES}
    findings = []
    if len(records) != 60:
        findings.append(f"Expected60pilotrecords;observed{len(records)}")
    if any(value != 12 for value in Counter(a.metadata["category"] for a in records).values()):
        findings.append("Five type quotas are not all12")
    if len(matrix) != 10 or any(value != 6 for value in matrix.values()):
        findings.append("Each session/type quota must be6")
    counts = {}
    for category in TYPES:
        group = [a for a in records if a.metadata["category"] == category]
        counts[category] = {"v_tiers": _counts([a.metadata["v_tier"] for a in group]),
                            "w_tiers": _counts([a.metadata["w_tier"] for a in group]),
                            "height_groups": _counts([a.metadata["height_group"] for a in group]),
                            "family_ids": _counts([a.metadata["family_id"] for a in group])}
        if category in ("translation", "turn", "mixed"):
            if counts[category]["v_tiers"] != {"V1": 4, "V2": 4, "V3": 4}:
                findings.append(f"{category}:V1/V2/V3quota")
        if category in ("orientation", "mixed"):
            if counts[category]["w_tiers"] != {"W1": 4, "W2": 4, "W3": 4}:
                findings.append(f"{category}:W1/W2/W3quota")
        if category != "static" and counts[category]["height_groups"] != {"high": 6, "low": 6}:
            findings.append(f"{category}:high/lowquota")
    mixed = [a for a in records if a.metadata["category"] == "mixed"]
    mixed_pairs = _counts([f"{a.metadata['v_tier']}:{a.metadata['w_tier']}" for a in mixed])
    for v in range(1, 4):
        for w in range(1, 4):
            expected = 2 if (v, w) in ((1, 1), (2, 3), (3, 2)) else 1
            if mixed_pairs.get(f"V{v}:W{w}", 0) != expected:
                findings.append(f"mixedV{v}:W{w}quota")
    for session in sorted({a.metadata["session_id"] for a in records}):
        group = [a for a in mixed if a.metadata["session_id"] == session]
        for key, prefix in (("v_tier", "V"), ("w_tier", "W")):
            if Counter(a.metadata[key] for a in group) != {f"{prefix}{i}": 2 for i in range(1, 4)}:
                findings.append(f"mixed:{session}:{key}marginal")
    static_layers = {}
    for session in sorted({a.metadata["session_id"] for a in records}):
        layers = sorted(round(float(-a.p[0, 2]), 6) for a in records
                        if a.metadata["session_id"] == session and a.metadata["category"] == "static")
        static_layers[session] = layers
        if not np.allclose(layers, [.15, .16, .17, .18, .19, .20], atol=1e-10):
            findings.append(f"{session}:staticheightlayers")
    continuous_high = {category: sum(a.metadata["category"] == category and
                        a.metadata["height_group"] == "high" and
                        np.ptp(-a.p[:, 2]) > .001 for a in records)
                       for category in ("translation", "turn", "mixed")}
    if any(value < 3 for value in continuous_high.values()):
        findings.append("At least3highroutes per translatingtype need continuousheightvariation")
    roles = defaultdict(set)
    for a in records:
        roles[a.metadata["family_id"]].add(a.metadata["data_role"])
    if any(len(values) > 1 for values in roles.values()):
        findings.append("Routefamilycrossesdataroles")
    return {"status": "source_candidates_only", "record_count": len(records),
            "total_effective_target_time_s_including_rests": float(total_s),
            "height_edges_m": HEIGHT_EDGES_M.tolist(), "height_bin_time_s": bins.tolist(),
            "height_bin_fraction_all_time": (bins / total_s).tolist(),
            "high_15_20cm_fraction": float(bins[1:].sum() / total_s),
            "five_high_bin_target_fraction": [.12] * 5,
            "five_high_bin_deviation_percentage_points": ((bins[1:] / total_s - .12) * 100).tolist(),
            "height_integration": "exact segment/bin intersection for source-linearly-interpolated height; static boundary bins right-inclusive only at20cm",
            "category_counts": _counts([a.metadata["category"] for a in records]),
            "type_by_session_counts": matrix, "quotas_by_type": counts,
            "mixed_v_by_w_counts": mixed_pairs, "static_layers_m": static_layers,
            "continuous_height_high_routes_by_type": continuous_high,
            "discrete_quota_violations": findings, "discrete_quotas_pass": not findings,
            "duration_exception_count": sum(a.t[-1] > 20.01 for a in records),
            "duration_range_s": [float(min(a.t[-1] for a in records)), float(max(a.t[-1] for a in records))],
            "all_source_time_retained": True, "warning": AUTHORITY_WARNING}


def _skew(v: np.ndarray) -> np.ndarray:
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def observability(centres: np.ndarray, quaternions: np.ndarray) -> dict[str, Any]:
    """Nominal local9DOF J for c=R(a+Qr)+t, with proper SO(3) only.

    At R=I,t=0,r=(0,0,.19), nominal flange a=c-Qr. Columns are
    [-[c]x,I,Q]. Parameters use0.1rad/0.01m/0.01m comparison scales.
    """
    rotations = Rotation.from_quat(quaternions).as_matrix()
    scale = np.array([.1] * 3 + [.01] * 6)
    jacobian = np.vstack([np.hstack([-_skew(c), np.eye(3), q]) for c, q in zip(centres, rotations)])
    singular = np.linalg.svd(jacobian * scale, compute_uv=False)
    rank = int(np.count_nonzero(singular > singular[0] * 1e-9))
    return {"rank": rank, "parameter_count": 9, "pose_count": len(centres),
            "scaled_singular_values": singular.tolist(),
            "scaled_condition_number": float(singular[0] / singular[-1]) if rank == 9 else None,
            "parameter_scales_rotation_rad_translation_m_tool_m": scale.tolist(),
            "model": "p_C_S=R_S_B(a_F_B+Q_B_F*r_F_C)+t_S_B; nominalR=I,t=0,r=(0,0,.190)",
            "jacobian_columns": "[-skew(p_C),I,Q]underleftSO3perturbation",
            "rotation_domain": "properSO3only;determinant+1;nozreflection/noarrayscaling",
            "interpretation": "Desired-pose local observability only. Full rank/condition do not prove measured precision or absolute label accuracy."}


def calibration_audit(workspace: Path, authorities: list[Authority]) -> dict[str, Any]:
    path = workspace / "planning" / "calibration_capture_windows.json"
    if not path.exists():
        return {"status": "calibration_windows_not_available"}
    windows = _json(path)
    by_id = {a.metadata["trajectory_id"]: a for a in authorities}
    records = [a for a in authorities if a.metadata.get("category") == "calibration"]
    violations = []
    for a in records:
        for window in a.metadata["capture_windows"]:
            mask = (a.t >= window["capture_start_time_s"] - 1e-10) & (a.t <= window["capture_end_time_s"] + 1e-10)
            if window["capture_end_time_s"] - window["capture_start_time_s"] < .5:
                violations.append(window["window_id"] + ":shortcapture")
            if window["capture_start_time_s"] - window["arrival_time_s"] < .3:
                violations.append(window["window_id"] + ":shortsettle")
            if not np.all(a.p[mask] == a.p[mask][0]) or not np.all(a.q[mask] == a.q[mask][0]):
                violations.append(window["window_id"] + ":capturemoving")
    sites = defaultdict(list)
    for window in windows:
        sites[window["site_id"]].append(window)
    for site, site_windows in sites.items():
        if len(site_windows) != 15 or len({w["data_role"] for w in site_windows}) != 1:
            violations.append(site + ":splitorgroupsize")
        if len({w["orientation_id"] for w in site_windows}) != 5:
            violations.append(site + ":orientationcoverage")
    schedule_path = workspace / "planning" / "calibration_repeat_schedule.json"
    schedule = _json(schedule_path) if schedule_path.exists() else []
    for item in schedule:
        source = by_id.get(item["trajectory_id"])
        if source is None or item["source_sha256"] != source.audit["source_sha256"]:
            violations.append(item["execution_id"] + ":repeatmissingorchangedsource")
        elif item["data_role"] != source.metadata["data_role"]:
            violations.append(item["execution_id"] + ":repeatroleleakage")
    repeats = Counter(item["trajectory_id"] for item in schedule)
    if len(schedule) != 54 or len(repeats) != 27 or set(repeats.values()) != {2}:
        violations.append("repeatmustretain27authoritiesin2independentlogicalexecutions")
    centres = np.asarray([w["nominal_rod_tip_position_plane1_m"] for w in windows])
    q = np.asarray([w["quaternion_xyzw"] for w in windows])
    fit = np.asarray([w["data_role"] == "geometry_fit" for w in windows])
    yaw = Rotation.from_rotvec(np.column_stack([np.zeros((len(windows), 2)), np.deg2rad(np.linspace(-40., 40., len(windows)))])).as_quat()
    static = np.tile([0., 0., 0., 1.], (len(windows), 1))
    observations = {"two_axis_all": observability(centres, q),
                    "two_axis_fit_only": observability(centres[fit], q[fit]),
                    "yaw_only_comparison": observability(centres, yaw),
                    "fixed_orientation_comparison": observability(centres, static)}
    if observations["two_axis_fit_only"]["rank"] != 9:
        violations.append("fitgeometrynotfullrank9")
    return {"status": "nominal_design_audited", "unique_pose_windows": len(windows),
            "site_count": len(sites), "heights_m": sorted({w["nominal_height_m"] for w in windows}),
            "fit_pose_count": int(fit.sum()), "holdout_pose_count": int((~fit).sum()),
            "holdout_site_count": sum(values[0]["data_role"] == "geometry_holdout" for values in sites.values()),
            "split_unit": "wholeXYsite/allheights/allorientations/acrossbothrepeats",
            "logical_repeat_execution_count": len(schedule), "scheduled_capture_window_count": len(schedule) * 5,
            "holdout_fraction": float((~fit).sum() / len(windows)), "observability": observations,
            "violations": violations, "passed": not violations,
            "unmeasured_items": ["LMstability/saturation", "realpose/FK", "rawmagneticcapture", "acceptedR,t,r", "physicalinstallationID", "actualtiminguncertainty"],
            "low_height_adjustment": "60mmproposalrejectedupstreamagainstunchanged60mmkeepoutand65mmanalyticboundary;authorityuses75/80/100mmwith3mmposition+2degangularcoarseuncertainty",
            "warning": AUTHORITY_WARNING}


def _load_records(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    data = _json(path)
    return data if isinstance(data, list) else data.get("trajectories", data.get("active_records", data.get("records", [])))


def load_metadata(workspace: Path, authority_index: Path | None = None) -> tuple[list[dict[str, Any]], list[str]]:
    if authority_index:
        path = authority_index.resolve()
        return _load_records(path), [str(path)]
    # An explicit active manifest lets the author retain earlier source versions.
    active = workspace / "planning" / "active_manifest.json"
    if active.exists():
        records = _load_records(active)
        return records, [str(active.relative_to(workspace))]
    pilot_versions = list((workspace / "planning").glob("active_pilot_manifest_v*.json"))
    pilot_active = max(pilot_versions, key=lambda item: int(item.stem.rsplit("v", 1)[1])) if pilot_versions else None
    if pilot_active:
        records = _load_records(workspace / "planning" / "calibration_manifest.json")
        records.extend(_json(pilot_active)["active_records"])
        return records, ["planning/calibration_manifest.json", str(pilot_active.relative_to(workspace))]
    names = ["calibration_manifest.json", "commissioning_manifest.json", "pilot_manifest.json"]
    records = []
    used = []
    for name in names:
        path = workspace / "planning" / name
        if path.exists():
            records.extend(_load_records(path))
            used.append(str(path.relative_to(workspace)))
    unique = {record["trajectory_id"]: record for record in records}
    return list(unique.values()), used


def runtime_join(workspace: Path, authorities: list[Authority], index_path: Path | list[Path] | None) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    index_paths = index_path if isinstance(index_path, list) else [index_path] if index_path else []
    for path in index_paths:
        if not path.exists():
            continue
        data = _json(path)
        if isinstance(data, list):
            rows.extend(data)
        elif isinstance(data, dict):
            source = data.get("trajectories", data.get("records", data.get("results", [])))
            rows.extend([{"trajectory_id": key, **value} for key, value in source.items()] if isinstance(source, dict) else source)
    indexed = {row.get("trajectory_id", row.get("source_id")): row for row in rows}
    records = []
    for authority in authorities:
        ident = authority.metadata["trajectory_id"]
        source = indexed.get(ident)
        if source is None:
            records.append({"trajectory_id": ident, "status": "not_simulated", "source_sha256": authority.audit["source_sha256"]})
        else:
            phase_status = {phase: source.get(phase, {}).get("status", "pending") for phase in ("compile", "build", "inspect", "play")}
            failures = [key for key, value in phase_status.items() if value == "failed"]
            evidence_sha = source.get("source", {}).get("sha256", source.get("source_sha256"))
            if evidence_sha and evidence_sha != authority.audit["source_sha256"]:
                status = "rejected:evidence_source_sha256_mismatch"
            elif failures:
                status = "failed:" + ",".join(failures)
            elif all(value == "passed" for value in phase_status.values()):
                status = "compile_record_inspect_playback_passed"
            elif any(value == "running" for value in phase_status.values()):
                status = "running:" + ",".join(key for key, value in phase_status.items() if value == "running")
            elif any(value == "passed" for value in phase_status.values()):
                status = "passed:" + ",".join(key for key, value in phase_status.items() if value == "passed")
            else:
                status = source.get("status", "not_simulated")
            records.append({"trajectory_id": ident, "status": status, "phase_status": phase_status,
                            "source_sha256": authority.audit["source_sha256"], "runtime_evidence": source})
    return {"index_paths": [str(path) for path in index_paths],
            "status_counts": _counts([r["status"] for r in records]), "records": records,
            "interpretation": "Source checks do not imply reachable. Only explicit runtime evidence can support compile/record/inspect/independentplayback claims."}


def _save(fig: Any, directory: Path, stem: str) -> list[str]:
    import matplotlib.pyplot as plt
    result = []
    for suffix in ("png", "pdf"):
        path = directory / f"{stem}.{suffix}"
        fig.savefig(path, dpi=220, bbox_inches="tight", facecolor="white")
        result.append(path.name)
    plt.close(fig)
    return result


def figures(directory: Path, authorities: list[Authority], coverage: dict[str, Any], runtime: dict[str, Any]) -> list[str]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                         "pdf.fonttype": 42})
    directory.mkdir(parents=True, exist_ok=True)
    palette = dict(zip(TYPES, ("#0072B2", "#E69F00", "#009E73", "#CC79A7", "#D55E00")))
    pilot = [a for a in authorities if a.metadata.get("stage") == "pilot"]
    paths = []
    if pilot:
        fig = plt.figure(figsize=(14, 7.5), constrained_layout=True)
        for i, category in enumerate(TYPES):
            ax = fig.add_subplot(2, 3, i + 1, projection="3d")
            for a in pilot:
                if a.metadata["category"] != category:
                    continue
                p = a.p * 100
                ax.plot(p[:, 0], p[:, 1], -p[:, 2], color=palette[category], alpha=.55, lw=.8)
                ax.scatter(p[0, 0], p[0, 1], -p[0, 2], s=10, color=palette[category])
                select = np.linspace(0, len(a.t) - 1, 4, dtype=int)
                axis = a.series["axes"][select].copy()
                axis[:, 2] *= -1
                ax.quiver(p[select, 0], p[select, 1], -p[select, 2], axis[:, 0], axis[:, 1], axis[:, 2], length=1.7, color="black", alpha=.5, linewidth=.5)
            ax.set(xlabel="plane1 X (cm)", ylabel="plane1 Y (cm)", zlabel="h=-Z (cm)",
                   xlim=(-14, 14), ylim=(-14, 14), zlim=(7, 21), title=TYPE_LABELS[category])
            ax.view_init(25, -58)
        ax = fig.add_subplot(2, 3, 6)
        ax.axis("off")
        ax.text(0, .95, "60 nominal pilot candidates\n2 sessions / 5 types\nBlack arrows: local +Z axis\n\nCentre: nominal rod_tip\nAxis: uncalibrated tool axis\nS transform / magnet offset pending\n\nThese source plots do not prove\nrobot reachability or physical labels.", va="top", fontsize=11, linespacing=1.5)
        fig.suptitle("Planned 3D paths and nominal axes — no actual robot measurements")
        paths += _save(fig, directory, "planned_paths_nominal_axes")
        fig, axes = plt.subplots(5, 2, figsize=(13, 12), constrained_layout=True)
        for i, category in enumerate(TYPES):
            for a in pilot:
                if a.metadata["category"] != category:
                    continue
                s = a.series
                style = "-" if a.metadata["session_number"] == 1 else "--"
                axes[i, 0].plot(a.t, s["speed"] * 100, style, color=palette[category], alpha=.6, lw=.7)
                axes[i, 1].plot(s["tm"], s["axis_rate"], style, color=palette[category], alpha=.6, lw=.7)
            axes[i, 0].set(ylabel=f"{TYPE_LABELS[category]}\nSpeed(cm/s)")
            axes[i, 1].set(ylabel="Nominal axis rate(deg/s)")
            for ax in axes[i]:
                ax.grid(alpha=.2)
                ax.set_ylim(bottom=-.1)
        axes[-1, 0].set_xlabel("Authority time (s); all rests retained")
        axes[-1, 1].set_xlabel("Authority time (s); solid=session1, dashed=session2")
        fig.suptitle("Planned vector speed and nominal direction-change rate")
        paths += _save(fig, directory, "planned_speed_axis_rate")
        fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
        for a in pilot:
            s = a.series
            color = palette[a.metadata["category"]]
            for ax, tt, yy in ((axes[0, 0], a.t, s["acceleration"]), (axes[0, 1], a.t, s["jerk"]),
                               (axes[1, 0], s["tm"], s["alpha"]), (axes[1, 1], s["tm"], s["angular_jerk"])):
                ax.plot(tt, yy, color=color, lw=.6, alpha=.35)
        for ax, title, limit in zip(axes.flat, ("Centre acceleration(m/s2)", "Centre jerk(m/s3)", "Flange angular acceleration(deg/s2)", "Flange angular jerk(deg/s3)"), (.05, .25, 30, 150)):
            ax.axhline(limit, color="black", ls="--", lw=1, label=f"Candidate bound {limit:g}")
            ax.set(title=title, xlabel="Authority time(s)", ylim=(-limit * .03, limit * 1.08))
            ax.grid(alpha=.2)
            ax.legend(fontsize=8)
        fig.suptitle("Source-grid vector-norm derivative diagnostics — sampled, not a continuous feasibility proof")
        paths += _save(fig, directory, "planned_acceleration_jerk")
        fig, axes = plt.subplots(1, 2, figsize=(13, 4.5), constrained_layout=True)
        speed_height = np.zeros((4, 6))
        for a in pilot:
            speed_height[int(a.metadata["v_tier"][1:])] += weighted_bins(a.t, -a.p[:, 2], HEIGHT_EDGES_M)
        image = axes[0].imshow(speed_height / coverage["total_effective_target_time_s_including_rests"] * 100, cmap="viridis", aspect="auto")
        axes[0].set(xticks=range(6), xticklabels=["8–15", "15–16", "16–17", "17–18", "18–19", "19–20"], yticks=range(4), yticklabels=["V0", "V1", "V2", "V3"], xlabel="Nominal height (cm)", ylabel="Declared centre-speed tier", title="Time-weighted speed×height (% of all target time)")
        for i in range(4):
            for j in range(6):
                axes[0].text(j, i, f"{speed_height[i,j]/coverage['total_effective_target_time_s_including_rests']*100:.1f}", ha="center", va="center", color="white" if image.norm(speed_height[i,j]/coverage['total_effective_target_time_s_including_rests']*100)<.5 else "black")
        fig.colorbar(image, ax=axes[0], label="% of target time")
        fraction = np.asarray(coverage["height_bin_fraction_all_time"]) * 100
        axes[1].bar(range(6), fraction, color="#0072B2")
        axes[1].plot(range(1, 6), [12] * 5, "k_", ms=30, mew=2, label="High-bin target:12% each")
        axes[1].set(xticks=range(6), xticklabels=["8–15", "15–16", "16–17", "17–18", "18–19", "19–20"], xlabel="Nominal height (cm)", ylabel="% of all target time", title=f"High-zone share {coverage['high_15_20cm_fraction']*100:.2f}% (target≈60%)")
        axes[1].legend(fontsize=8)
        paths += _save(fig, directory, "planned_speed_height_coverage")
        fig, ax = plt.subplots(figsize=(9, 4), constrained_layout=True)
        sessions = sorted({a.metadata["session_id"] for a in pilot})
        matrix = np.array([[coverage["type_by_session_counts"][f"{session}:{category}"] for category in TYPES] for session in sessions])
        ax.imshow(matrix, cmap="Blues", vmin=0, vmax=8, aspect="auto")
        ax.set(xticks=range(5), xticklabels=[TYPE_LABELS[t] for t in TYPES], yticks=range(2), yticklabels=sessions, title="Type×session candidate count (target:6 each)")
        for i in range(2):
            for j in range(5):
                ax.text(j, i, str(matrix[i, j]), ha="center", va="center", color="black")
        paths += _save(fig, directory, "planned_type_session")
    calibration = [a for a in authorities if a.metadata.get("category") == "calibration"]
    if calibration:
        fig = plt.figure(figsize=(13, 5.5), constrained_layout=True)
        ax = fig.add_subplot(1, 2, 1, projection="3d")
        for authority in calibration:
            windows = authority.metadata["capture_windows"]
            for window in windows:
                point = np.asarray(window["nominal_rod_tip_position_plane1_m"]) * 100
                axis = Rotation.from_quat(window["quaternion_xyzw"]).apply([0., 0., 1.])
                color = "#D55E00" if window["data_role"] == "geometry_holdout" else "#0072B2"
                ax.scatter(*[point[0], point[1], -point[2]], s=12, color=color)
                ax.quiver(point[0], point[1], -point[2], axis[0], axis[1], -axis[2], length=.5, color=color, linewidth=.6, alpha=.65)
        ax.set(xlabel="plane1 X (cm)", ylabel="plane1 Y (cm)", zlabel="h=-Z (cm)", zlim=(6.8, 10.4), title="135 static windows /9 sites /3 heights /5 attitudes\nBlue=fit; orange=whole-site holdout")
        ax.view_init(24, -52)
        ax = fig.add_subplot(1, 2, 2)
        windows = [window for a in calibration for window in a.metadata["capture_windows"]]
        centres = np.asarray([w["nominal_rod_tip_position_plane1_m"] for w in windows])
        q = np.asarray([w["quaternion_xyzw"] for w in windows])
        for label, quats in (("Two-axis tilt", q), ("Yaw only", Rotation.from_rotvec(np.column_stack([np.zeros((len(q), 2)), np.deg2rad(np.linspace(-40, 40, len(q)))])).as_quat()), ("Fixed attitude", np.tile([0., 0., 0., 1.], (len(q), 1)))):
            result = observability(centres, quats)
            singular = np.asarray(result["scaled_singular_values"])
            ax.semilogy(range(1, 10), np.maximum(singular / singular[0], 1e-16), "o-", label=f"{label}: rank {result['rank']}/9")
        ax.set(xticks=range(1, 10), xlabel="Singular-value index", ylabel="Scaled singular value /maximum", title="Nominal local observability (not measured precision)")
        ax.grid(alpha=.2)
        ax.legend()
        paths += _save(fig, directory, "calibration_design_observability")
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5), constrained_layout=True)
    ax = axes[0]
    counts = runtime["status_counts"]
    statuses = sorted(counts)
    ax.barh(statuses, [counts[s] for s in statuses], color="#0072B2")
    ax.set(xlabel="Number of authorities", title="Runtime evidence status — pending candidates are not declared reachable")
    for i, key in enumerate(statuses):
        ax.text(counts[key] + .2, i, str(counts[key]), va="center")
    ax = axes[1]
    precheck_path = directory.parent / "planning" / "calibration_report.json"
    rejected = _json(precheck_path).get("rejected_upstream_candidates", []) if precheck_path.exists() else []
    if rejected:
        heights = [f"{item['nominal_height_m']*1000:g}mm" for item in rejected]
        margins = [item["non_tip_clearance_margin_m"] * 1000 for item in rejected]
        ax.bar(heights, margins, color="#D55E00", label="Rejected upstream proposals")
        ax.axhline(0, color="black", lw=1)
        ax.set(xlabel="Proposed nominal calibration height", ylabel="Clearance margin after required 5mm (mm)", title="Unchanged board keepout +coarse 3mm/2deg envelope")
        ax.legend(fontsize=8)
    else:
        ax.axis("off")
        ax.text(0, .6, "No upstream-rejection index available.\nRuntime failures remain in JSON evidence.")
    paths += _save(fig, directory, "simulation_status_rejection")
    return paths


def run(workspace: Path, runtime_index: Path | list[Path] | None = None, plots: bool = True, authority_index: Path | None = None) -> dict[str, Any]:
    workspace = workspace.resolve()
    records, manifest_paths = load_metadata(workspace, authority_index)
    if not records:
        raise RuntimeError("No generation manifest is available")
    authorities = [read_authority(workspace, record) for record in records]
    coverage = pilot_coverage(authorities)
    calibration = calibration_audit(workspace, authorities)
    runtime = runtime_join(workspace, authorities, runtime_index)
    directory = workspace / "reports"
    directory.mkdir(parents=True, exist_ok=True)
    report = {"protocol_version": "real_collection_v2_20261016", "report_scope": "independentnominalsourceaudit+explicitruntimeevidencejoin",
              "input_manifests": manifest_paths, "authority_count": len(authorities),
              "source_csv_schema_hash_time_hold_checks_pass": True,
              "sampled_motion_limits_pass": all(a.audit["sampled_motion_limits_pass"] for a in authorities),
              "declared_continuous_cartesian_bounds_available_and_pass": all(a.audit["continuous_cartesian_bounds"]["available"] and a.audit["continuous_cartesian_bounds"]["limits_pass"] for a in authorities),
              "coarse_rod_sensor_face_precheck_pass": all(a.audit["coarse_rod_sensor_face_precheck"]["passed"] for a in authorities),
              "declared_source_tiers_pass": all(a.audit.get("declared_source_tiers_pass", True) for a in authorities),
              "all_sampled_endpoint_neighbours_stationary": all(a.audit["sampled_endpoints_have_exact_stationary_neighbours"] for a in authorities),
              "calibration": calibration, "pilot": coverage, "runtime": runtime,
              "authorities": [a.audit for a in authorities], "warning": AUTHORITY_WARNING,
              "not_claimed": ["acceptedmagnetcentre/Sframe", "physicalrobotvalidation", "magneticdata", "realfeedbackcoverage", "backgroundcapture", "absolutetruthprecision"]}
    if plots:
        report["figures"] = figures(directory, authorities, coverage, runtime)
    _write_json(directory / "independent_source_audit.json", report)
    _write_json(directory / "calibration_observability.json", calibration)
    _write_json(directory / "planned_coverage_audit.json", coverage)
    _write_json(directory / "runtime_evidence_join.json", runtime)
    lines = ["# Independent planned-source audit", "", AUTHORITY_WARNING, "",
             f"Authorities audited: {len(authorities)}. Exact ROS-ns times, immutable SHA-256, strict CSV schema and stationary hold rows verified.",
             f"Sampled Cartesian derivative limits pass: {report['sampled_motion_limits_pass']}.",
             f"Independently recomputed sufficient continuous Cartesian bounds pass: {report['declared_continuous_cartesian_bounds_available_and_pass']}.",
             f"Coarse 3mm/+2deg rod/board-face envelope passes: {report['coarse_rod_sensor_face_precheck_pass']}; declared source speed/axis tiers pass: {report['declared_source_tiers_pass']}.",
             "Whole-robot collision, IK, joint dynamics and independent playback are separate runtime checks. Source-grid plots alone do not establish them.", "",
             "## Nominal calibration design", "",
             f"Unique static windows: {calibration.get('unique_pose_windows')}; logical repeated executions: {calibration.get('logical_repeat_execution_count')}. Whole XY sites are kept together for fit/holdout and repeated acquisition."]
    if "observability" in calibration:
        lines += ["", "| Design | Nominal rank | Scaled condition |", "|---|---:|---:|"]
        for key, value in calibration["observability"].items():
            lines.append(f"| {key} | {value['rank']}/9 | {value['scaled_condition_number']} |")
        lines += ["", "These are desired-pose Jacobian diagnostics, not a measured calibration or an accuracy claim. 60mm calibration was rejected upstream against the unchanged protective volume; 75/80/100mm candidates include coarse3mm/+2deg envelope checks."]
    if coverage.get("record_count"):
        lines += ["", "## Planned pilot coverage", "",
                  f"{coverage['record_count']} candidates, {coverage['total_effective_target_time_s_including_rests']:.3f}s target time including declared rests;15–20cm={coverage['high_15_20cm_fraction']*100:.3f}%.",
                  f"Discrete quotas pass: {coverage['discrete_quotas_pass']}. Violations: {coverage['discrete_quota_violations']}.", "",
                  "| Height(cm) | Time(s) | %alltargettime |", "|---|---:|---:|"]
        for label, seconds, fraction in zip(["8–15", "15–16", "16–17", "17–18", "18–19", "19–20"], coverage["height_bin_time_s"], coverage["height_bin_fraction_all_time"]):
            lines.append(f"| {label} | {seconds:.3f} | {fraction*100:.3f} |")
    lines += ["", "## Runtime evidence", "", json.dumps(runtime["status_counts"], ensure_ascii=False), "",
              "Only explicit simulation evidence can upgrade a candidate. Real-feedback plots, magnetic quality, calibration acceptance and synchronization uncertainty remain future acquisition outputs.", "",
              "## Reproduce", "", "```bash", "python -m workstation_magtrack.validation.audit --workspace workstation_magtrack", "```", ""]
    (directory / "independent_source_audit.md").write_text("\n".join(lines))
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--runtime-index", type=Path, action="append", help="May repeat for separate calibration/pilot evidence batches")
    parser.add_argument("--authority-index", type=Path, help="Explicit immutable mastermanifest with trajectories list")
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()
    report = run(args.workspace, args.runtime_index, not args.no_plots, args.authority_index)
    print(json.dumps({"authority_count": report["authority_count"], "sampled_motion_limits_pass": report["sampled_motion_limits_pass"],
                      "pilot_high_fraction": report["pilot"].get("high_15_20cm_fraction"),
                      "runtime_status_counts": report["runtime"]["status_counts"]}, indent=2))


if __name__ == "__main__":
    main()
