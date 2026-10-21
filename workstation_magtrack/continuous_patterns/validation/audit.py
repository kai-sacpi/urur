"""Independent CPU audit of the continuous-pattern authority files.

Only reports beneath ``continuous_patterns/reports`` are written.  The CSVs
are immutable authority: no scaling, retiming, clipping, omission or repair is
performed here.  A source pass is deliberately separate from the unchanged
nominal scene precheck and from robot IK/collision/playback, which are not run.

Example::

    python validation/audit.py --root . --manifest batches/<batch_id>/manifest.json

NumPy and SciPy are the only non-standard dependencies.  Formula reconstruction
and continuous certificates are checked through this batch's ``src.trajectory``;
the CSV finite differences, geometry, bins and control comparisons are computed
independently below, without importing any previous runtime or validator.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
import hashlib
import importlib
import io
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation


FIELDS = ("time_s", "frame_id", "tip_x_m", "tip_y_m", "tip_z_m",
          "tip_qx", "tip_qy", "tip_qz", "tip_qw", "phase")
LIMITS = {"acceleration_m_s2": .05, "jerk_m_s3": .25,
          "angular_acceleration_deg_s2": 30., "angular_jerk_deg_s3": 150.}
V_TIERS = {"V1": .01, "V2": .02, "V3": .05}
W_TIERS = {"W0": 0., "W1": 3., "W2": 10., "W3": 20.}
HEIGHT_EDGES_M = np.arange(7, 21, dtype=float) / 100.
EXPECTED_GROUP_COUNTS = {"stratified": 56, "full_height": 60,
                         "full_height_reverse": 20, "guarded_full_height": 20}
AUTHORITY_WARNING = (
    "Planned nominal rod_tip in plane1, h=-z; quaternion order xyzw. "
    "Accepted plane1-to-sensor registration, flange-to-magnet centre, magnetic "
    "axis, effective TCP and installation ID remain pending. These are not "
    "formal magnet-centre truth labels or physical robot feedback."
)


@dataclass
class Source:
    record: dict[str, Any]
    metadata: dict[str, Any]
    t: np.ndarray
    p: np.ndarray
    q: np.ndarray
    phase: np.ndarray
    series: dict[str, np.ndarray]
    audit: dict[str, Any]


def _norm(values: np.ndarray) -> np.ndarray:
    return np.linalg.norm(values, axis=-1)


def _json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _clean(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return _clean(value.tolist())
    if isinstance(value, np.generic):
        return _clean(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _clean(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_clean(item) for item in value]
    return value


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_clean(value), ensure_ascii=False, indent=2,
                               allow_nan=False) + "\n", encoding="utf-8")


def _resolve(root: Path, name: str | Path) -> Path:
    root = root.resolve()
    path = Path(name)
    path = path.resolve() if path.is_absolute() else (root / path).resolve()
    if not path.is_relative_to(root):
        raise ValueError(f"authority path is outside this continuous_patterns batch: {name}")
    return path


def _sha(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _gradient(values: np.ndarray, t: np.ndarray) -> np.ndarray:
    return np.gradient(values, t, axis=0, edge_order=2 if len(t) > 2 else 1)


def motion_series(t: np.ndarray, p: np.ndarray, q: np.ndarray) -> dict[str, np.ndarray]:
    """Full vector derivatives; SO(3) spatial angular rates, including roll.

    Adjacent rotations give midpoint spatial angular velocity, not Euler-angle
    differences and not only a magnetic/tool-axis direction rate.  Consecutive
    derivatives are in one fixed plane1 basis.  Centre acceleration therefore
    includes turning/centripetal terms rather than just scalar speed changes.
    """
    v = _gradient(p, t)
    a = _gradient(v, t)
    j = _gradient(a, t)
    speed = _norm(v)
    moving = speed > 1e-10
    tangential_acceleration = np.zeros(len(t))
    normal_acceleration = np.zeros(len(t))
    tangential_acceleration[moving] = np.einsum("ij,ij->i", v[moving], a[moving])/speed[moving]
    normal_acceleration[moving] = _norm(np.cross(v[moving], a[moving]))/speed[moving]
    rotations = Rotation.from_quat(q)
    dt = np.diff(t)
    tm = (t[1:] + t[:-1]) / 2.
    omega = (rotations[1:] * rotations[:-1].inv()).as_rotvec() / dt[:, None]
    alpha = _gradient(omega, tm)
    angular_jerk = _gradient(alpha, tm)
    axes = rotations.apply(np.tile([0., 0., 1.], (len(t), 1)))
    dots = np.einsum("ij,ij->i", axes[:-1], axes[1:])
    axis_rate = np.arctan2(_norm(np.cross(axes[:-1], axes[1:])),
                           np.clip(dots, -1., 1.)) / dt
    return {"v": v, "a": a, "j": j, "speed": speed,
            "signed_tangential_acceleration_m_s2": tangential_acceleration,
            "normal_centripetal_acceleration_m_s2": normal_acceleration,
            "acceleration_m_s2": _norm(a), "jerk_m_s3": _norm(j),
            "tm": tm, "omega_vector_rad_s": omega, "alpha_vector_rad_s2": alpha,
            "angular_jerk_vector_rad_s3": angular_jerk,
            "angular_speed_deg_s": np.rad2deg(_norm(omega)),
            "angular_acceleration_deg_s2": np.rad2deg(_norm(alpha)),
            "angular_jerk_deg_s3": np.rad2deg(_norm(angular_jerk)),
            "axis_rate_deg_s": np.rad2deg(axis_rate), "axes": axes}


def _weighted_quantile(values: np.ndarray, weights: np.ndarray, fraction: float) -> float:
    order = np.argsort(values)
    sums = np.cumsum(weights[order])
    return float(values[order[min(int(np.searchsorted(sums, fraction * sums[-1])), len(order)-1)]])


def _stats(values: np.ndarray, times: np.ndarray | None = None,
           weights: np.ndarray | None = None) -> dict[str, float | int]:
    values = np.asarray(values, float)
    if not len(values):
        return {"count": 0, "mean": 0., "p95": 0., "max": 0.}
    if weights is None:
        if times is None or len(times) == 1:
            weights = np.ones(len(values))
        else:
            dt = np.diff(times)
            weights = np.r_[dt[0]/2., (dt[:-1]+dt[1:])/2., dt[-1]/2.]
    return {"count": len(values), "mean": float(np.dot(values, weights) / weights.sum()),
            "p95": _weighted_quantile(values, weights, .95), "max": float(values.max())}


def weighted_bins(t: np.ndarray, heights: np.ndarray, edges: np.ndarray,
                  selected_intervals: np.ndarray | None = None) -> np.ndarray:
    """Exact segment/bin durations for source-linearly-interpolated height.

    Bins are [lo,hi), with exactly 200 mm included in the final bin. A per-
    interval mask permits draw-only coverage without counting stationary holds.
    """
    result = np.zeros(len(edges)-1)
    if selected_intervals is None:
        selected_intervals = np.ones(len(t)-1, dtype=bool)
    for index in np.flatnonzero(selected_intervals):
        t0, t1 = t[index:index+2]
        h0, h1 = [round(float(x), 11) for x in heights[index:index+2]]
        cuts = [0., 1.]
        if h1 != h0:
            cuts += [float((edge-h0)/(h1-h0)) for edge in edges
                     if 0. < (edge-h0)/(h1-h0) < 1.]
        cuts.sort()
        for lo, hi in zip(cuts[:-1], cuts[1:]):
            h = h0 + (h1-h0)*(lo+hi)/2.
            k = len(result)-1 if abs(h-edges[-1]) < 1e-10 else int(np.searchsorted(edges, h, side="right")-1)
            if 0 <= k < len(result):
                result[k] += (t1-t0)*(hi-lo)
    return result


def _read_source(root: Path, record: dict[str, Any]) -> tuple[dict, np.ndarray, np.ndarray,
                                                            np.ndarray, np.ndarray, dict]:
    path = _resolve(root, record["csv_file"])
    metadata_path = _resolve(root, record["metadata_file"])
    payload = path.read_bytes()
    metadata_bytes = metadata_path.read_bytes()
    expected_sha = record.get("sha256", record.get("csv_sha256"))
    if not isinstance(expected_sha, str) or expected_sha != _sha(payload):
        raise ValueError("CSV SHA-256 is missing or differs from the manifest")
    metadata_sha = record.get("metadata_sha256")
    if not isinstance(metadata_sha, str) or metadata_sha != _sha(metadata_bytes):
        raise ValueError("metadata SHA-256 is missing or differs from the manifest")
    metadata = json.loads(metadata_bytes.decode("utf-8"))
    if metadata.get("trajectory_id") != record["trajectory_id"]:
        raise ValueError("metadata trajectory_id differs from manifest")
    reader = csv.DictReader(io.StringIO(payload.decode("utf-8")), strict=True)
    if tuple(reader.fieldnames or ()) != FIELDS:
        raise ValueError("CSV header is not the exact authority schema")
    rows = list(reader)
    raw_lines = payload.splitlines()
    if len(rows) < 5 or len(raw_lines) != len(rows)+1:
        raise ValueError("too few rows or noncanonical multiline CSV")
    # Numeric/phase fields contain no commas. Requiring one physical row per
    # waypoint lets hold checks compare the literal field bytes, not floats.
    raw_rows = [line.split(b",") for line in raw_lines[1:]]
    if any(len(row) != 10 for row in raw_rows):
        raise ValueError("noncanonical quoted/comma-containing authority fields")
    if any(None in row or any(value is None for value in row.values()) for row in rows):
        raise ValueError("malformed CSV record")
    ns = []
    try:
        for row in rows:
            value = Decimal(row["time_s"]) * Decimal(1_000_000_000)
            if not value.is_finite() or value != value.to_integral_value():
                raise ValueError("timestamp is not an exact ROS nanosecond")
            ns.append(int(value))
    except (InvalidOperation, ValueError) as error:
        raise ValueError(f"invalid authoritative time: {error}") from error
    if ns[0] != 0 or any(b <= a for a, b in zip(ns, ns[1:])):
        raise ValueError("time must start at zero and strictly increase")
    if ns[-1] >= np.iinfo(np.int64).max:
        raise ValueError("authoritative time is outside int64 nanoseconds")
    t = np.asarray(ns, np.int64) / 1e9
    p = np.array([[float(row[key]) for key in FIELDS[2:5]] for row in rows])
    q = np.array([[float(row[key]) for key in FIELDS[5:9]] for row in rows])
    phase = np.array([row["phase"] for row in rows])
    if any(row["frame_id"] != "plane1" for row in rows):
        raise ValueError("every frame_id must be plane1")
    if not np.isfinite(p).all() or not np.isfinite(q).all():
        raise ValueError("non-finite pose field")
    norm_error = float(np.abs(_norm(q)-1.).max())
    if norm_error > 2e-10:
        raise ValueError(f"xyzw quaternion unit norm error {norm_error:.6g}")
    if phase[0] != "draw" or set(phase)-{"draw", "hold"}:
        raise ValueError("phase must be draw/hold and the initial row must be draw")
    moving_holds = [i for i in range(1, len(rows)) if phase[i] == "hold"
                    and raw_rows[i][2:9] != raw_rows[i-1][2:9]]
    if moving_holds:
        raise ValueError(f"hold seven-pose field bytes change at rows {moving_holds[:8]}")
    if float(np.max(np.abs(p[:, :2]))) > .13+2e-10:
        raise ValueError("XY exceeds the unchanged +/-130 mm main range")
    if float((-p[:, 2]).min()) < .07-2e-10 or float((-p[:, 2]).max()) > .20+2e-10:
        raise ValueError("height exceeds the requested 70--200 mm source range")
    begin, end = float(metadata["draw_start_s"]), float(metadata["draw_end_s"])
    duration = float(metadata["draw_duration_s"])
    if abs((end-begin)-duration) > 1e-8 or begin <= 0. or end >= t[-1]:
        raise ValueError("draw/hold metadata schedule is inconsistent")
    if min(abs(t-begin)) > 1e-9 or min(abs(t-end)) > 1e-9:
        raise ValueError("draw endpoints are absent from authority time")
    before, after = t <= begin+1e-10, t >= end-1e-10
    if not (np.all(p[before] == p[0]) and np.all(q[before] == q[0]) and
            np.all(p[after] == p[-1]) and np.all(q[after] == q[-1])):
        raise ValueError("declared initial/final hold contains a changing pose")
    if abs(begin-.5) > 1e-9 or abs(t[-1]-end-.5) > 1e-9:
        raise ValueError("this version requires exactly 0.5 s initial and final holds")
    interval_draw = (t[:-1] >= begin-1e-10) & (t[1:] <= end+1e-10)
    if np.any((phase == "hold") & (t > begin+1e-10) & (t <= end+1e-10)):
        raise ValueError("a draw-interval source row is mislabeled hold")
    if np.any(np.abs(np.diff(t)-.05) > 1e-9):
        raise ValueError("this version requires the declared 0.05 s source grid")
    info = {"csv_file": str(path.relative_to(root)), "metadata_file": str(metadata_path.relative_to(root)),
            "sha256": _sha(payload), "metadata_sha256": _sha(metadata_bytes),
            "source_pose_count": len(rows), "duration_s": float(t[-1]),
            "draw_duration_s": duration, "hold_duration_s": float(t[-1]-duration),
            "quaternion_max_unit_norm_error": norm_error,
            "quaternion_lexical_sign_change_count": int(np.count_nonzero(np.einsum("ij,ij->i", q[:-1], q[1:]) < 0.)),
            "time_start_zero_and_strictly_increasing": True, "exact_ros_nanoseconds": True,
            "source_grid_dt_s": .05, "schema_valid": True, "frame_id": "plane1",
            "quaternion_order": "xyzw", "hold_seven_pose_fields_byte_exact": True,
            "hold_row_count": int(np.count_nonzero(phase == "hold")),
            "initial_and_final_hold_s": [begin, float(t[-1]-end)],
            "sampled_endpoint_velocity_and_acceleration_zero": True,
            "xy_min_m": p[:, :2].min(axis=0).tolist(), "xy_max_m": p[:, :2].max(axis=0).tolist(),
            "height_min_m": float((-p[:, 2]).min()), "height_max_m": float((-p[:, 2]).max()),
            "draw_interval_mask": interval_draw}
    return metadata, t, p, q, phase, info


def _reversals(values: np.ndarray, threshold: float = 1e-6) -> int:
    differences = np.diff(values)
    signs = np.sign(differences[np.abs(differences) > threshold])
    return int(np.count_nonzero(signs[1:] != signs[:-1])) if len(signs) else 0


def _range_stats(values: np.ndarray) -> dict[str, Any]:
    finite = np.asarray(values, float)
    finite = finite[np.isfinite(finite)]
    if not len(finite):
        return {"sample_count": 0, "min": None, "median": None, "p95": None, "max": None}
    return {"sample_count": len(finite), "min": float(finite.min()),
            "median": float(np.median(finite)), "p95": float(np.percentile(finite, 95)),
            "max": float(finite.max())}


def _projection_crossings(t: np.ndarray, p: np.ndarray, max_points: int) -> dict[str, Any]:
    """Transverse, nonlocal intersections of a bounded-size XY source polyline.

    Spatial bins avoid a quadratic all-segment scan. Endpoint intersections are
    deduplicated in arclength, so exact sampled figure-eight crossings survive.
    Near-collinear/tangent events are excluded, as are adjacent segments and
    events with less than 5 mm centreline arclength separation.
    """
    keep = np.unique(np.linspace(0, len(p)-1, min(len(p), max_points), dtype=int))
    sampled, times = p[keep], t[keep]
    maximum_xy_error, maximum_height_error = 0., 0.
    for i, j in zip(keep[:-1], keep[1:]):
        fraction = (t[i:j+1]-t[i])/(t[j]-t[i])
        chord = p[i] + fraction[:, None]*(p[j]-p[i])
        error = p[i:j+1]-chord
        maximum_xy_error = max(maximum_xy_error, float(_norm(error[:, :2]).max()))
        maximum_height_error = max(maximum_height_error, float(np.abs(error[:, 2]).max()))
    xy = sampled[:, :2]
    a, d = xy[:-1], np.diff(xy, axis=0)
    xyz_lengths = _norm(np.diff(sampled, axis=0))
    arc = np.r_[0., np.cumsum(xyz_lengths)]
    bins: dict[tuple[int, int], list[int]] = defaultdict(list)
    cell = max(.005, float(np.ptp(xy, axis=0).max())/32.)
    candidates: set[tuple[int, int]] = set()
    for i in range(len(a)):
        if np.linalg.norm(d[i]) < 1e-10:
            continue
        lower = np.floor(np.minimum(xy[i], xy[i+1])/cell).astype(int)
        upper = np.floor(np.maximum(xy[i], xy[i+1])/cell).astype(int)
        for ix in range(lower[0], upper[0]+1):
            for iy in range(lower[1], upper[1]+1):
                bucket = bins[ix, iy]
                for j in bucket:
                    if i-j > 1:
                        candidates.add((j, i))
                bucket.append(i)
    def cross(x, y):
        return float(x[0]*y[1]-x[1]*y[0])
    events = []
    for i, j in sorted(candidates):
        determinant = cross(d[i], d[j])
        length_product = float(np.linalg.norm(d[i])*np.linalg.norm(d[j]))
        if length_product < 1e-18 or abs(determinant)/length_product < math.sin(math.radians(1.)):
            continue
        offset = a[j]-a[i]
        u, v = cross(offset, d[j])/determinant, cross(offset, d[i])/determinant
        if not (-1e-8 <= u <= 1.+1e-8 and -1e-8 <= v <= 1.+1e-8):
            continue
        u, v = float(np.clip(u, 0., 1.)), float(np.clip(v, 0., 1.))
        arc_i, arc_j = float(arc[i]+u*xyz_lengths[i]), float(arc[j]+v*xyz_lengths[j])
        if arc_j-arc_i < .005:
            continue
        # Closing branches are adjacent in a geometrically closed polyline.
        if np.linalg.norm(sampled[0]-sampled[-1]) < 1e-7 and arc[-1]-(arc_j-arc_i) < .005:
            continue
        # A vertex must pass across the other branch, not merely touch it.
        transverse = True
        for k, fraction, other_a, other_d in ((i, u, a[j], d[j]), (j, v, a[i], d[i])):
            vertex = k if fraction < 1e-7 else k+1 if fraction > 1.-1e-7 else None
            if vertex is not None and 0 < vertex < len(xy)-1:
                side_before = cross(other_d, xy[vertex-1]-other_a)
                side_after = cross(other_d, xy[vertex+1]-other_a)
                if side_before*side_after >= 0.:
                    transverse = False
        if not transverse:
            continue
        if any(abs(event["arc_positions_m"][0]-arc_i) < .0002 and
               abs(event["arc_positions_m"][1]-arc_j) < .0002 for event in events):
            continue
        h_i = float(-(sampled[i, 2]+u*(sampled[i+1, 2]-sampled[i, 2])))
        h_j = float(-(sampled[j, 2]+v*(sampled[j+1, 2]-sampled[j, 2])))
        cosine = abs(float(np.dot(d[i], d[j])))/length_product
        events.append({"xy_m": (a[i]+u*d[i]).tolist(), "heights_m": [h_i, h_j],
                       "height_separation_m": abs(h_i-h_j), "arc_positions_m": [arc_i, arc_j],
                       "arc_separation_m": arc_j-arc_i,
                       "times_s": [float(times[i]+u*(times[i+1]-times[i])),
                                   float(times[j]+v*(times[j+1]-times[j]))],
                       "crossing_angle_deg": math.degrees(math.acos(np.clip(cosine, -1., 1.)))})
    separations = np.array([event["height_separation_m"] for event in events])
    return {"method": "transverse_nonadjacent_XY_source_polyline_segment_intersections",
            "sample_points": len(sampled), "maximum_sample_interval_s": float(np.diff(times).max()),
            "maximum_measured_XY_chord_error_against_all_source_vertices_m": maximum_xy_error,
            "maximum_measured_height_chord_error_against_all_source_vertices_m": maximum_height_error,
            "height_difference_chord_error_bound_m": 2.*maximum_height_error,
            "minimum_transverse_crossing_angle_deg": 1., "minimum_arclength_separation_m": .005,
            "count": len(events), "count_height_separation_at_least_1mm": int(np.count_nonzero(separations >= .001)),
            "count_height_separation_at_least_5mm": int(np.count_nonzero(separations >= .005)),
            "count_height_separation_at_least_10mm": int(np.count_nonzero(separations >= .010)),
            "height_separation_m": _range_stats(separations), "events": events,
            "scope": "Sampled CSV polyline diagnostic. Chord error is measured against all authoritative vertices; no completeness or continuous curve-intersection proof is asserted."}


def geometry_diagnostics(t: np.ndarray, p: np.ndarray, series: dict[str, np.ndarray],
                         draw: np.ndarray, max_points: int) -> dict[str, Any]:
    pp, tt = p[draw], t[draw]
    differences = np.diff(pp, axis=0)
    lengths = _norm(differences)
    nonzero = lengths > 1e-9
    tangents = differences[nonzero]/lengths[nonzero, None]
    turning = np.arctan2(_norm(np.cross(tangents[:-1], tangents[1:])),
                         np.clip(np.einsum("ij,ij->i", tangents[:-1], tangents[1:]), -1., 1.))
    arclength = np.r_[0., np.cumsum(lengths)]
    unique = np.r_[True, np.diff(arclength) > 1e-12]
    uniform_arc = np.linspace(0., arclength[-1], min(2001, len(pp)))
    equal_arc = np.column_stack([np.interp(uniform_arc, arclength[unique], pp[unique, k]) for k in range(3)])
    singular = np.linalg.svd(equal_arc-equal_arc.mean(axis=0), compute_uv=False)/math.sqrt(len(equal_arc))
    v, a, j = [series[key][draw] for key in ("v", "a", "j")]
    speed = _norm(v)
    va = np.cross(v, a)
    va_norm = _norm(va)
    usable = speed > max(1e-8, float(speed.max())*1e-3)
    usable[:2], usable[-2:] = False, False
    curvature = va_norm[usable]/speed[usable]**3
    torsion_ok = usable & (va_norm > max(1e-12, float(va_norm.max())*1e-7))
    torsion = np.einsum("ij,ij->i", va[torsion_ok], j[torsion_ok])/va_norm[torsion_ok]**2
    headings = np.sign(tangents[np.any(np.abs(tangents) > .01, axis=1)]).astype(int)
    octants = Counter(",".join(str(x) for x in row) for row in headings)
    return {"scope": "Independent CSV finite differences and equal-arclength source-polyline geometry; no robot feasibility claim.",
            "path_length_m": float(arclength[-1]), "net_displacement_m": float(np.linalg.norm(pp[-1]-pp[0])),
            "xyz_span_m": np.ptp(pp, axis=0).tolist(),
            "curvature_per_m": _range_stats(curvature), "signed_torsion_per_m": _range_stats(torsion),
            "absolute_torsion_per_m": _range_stats(np.abs(torsion)),
            "torsion_definition_status": "defined_on_curved_moving_samples" if len(torsion) else "undefined_no_curved_moving_samples",
            "planar_constant_height": bool(np.ptp(pp[:, 2]) <= 2e-10),
            "torsion_interpretation": "A planar curved trajectory has defined zero torsion; straight/near-rest singularities are excluded and empty statistics are null. Nonzero torsion is not a requirement for constant-height layers.",
            "curvature_speed_exclusion_fraction_of_draw_rows": float(1.-usable.mean()),
            "torsion_valid_fraction_of_draw_rows": float(torsion_ok.mean()),
            "curvature_torsion_method": "kappa=|v cross a|/|v|^3; tau=(v cross a) dot j/|v cross a|^2, from complete vector source-grid derivatives. Near-rest and near-straight singularities excluded.",
            "SVD_equal_arclength_rms_axes_m": singular.tolist(),
            "SVD_nonplanarity_rms_m": float(singular[-1]),
            "SVD_smallest_to_largest_ratio": float(singular[-1]/singular[0]) if singular[0] else 0.,
            "tangent_total_turn_deg": float(np.rad2deg(turning.sum())),
            "tangent_turn_per_segment_deg": _range_stats(np.rad2deg(turning)),
            "coordinate_reversal_counts_xyz": [_reversals(pp[:, k]) for k in range(3)],
            "height_reversal_count": _reversals(-pp[:, 2]),
            "reversal_coordinate_increment_threshold_m": 1e-6,
            "tangent_octant_counts": dict(sorted(octants.items())),
            "XY_projection_crossings": _projection_crossings(tt, pp, max_points)}


def _formula_audit(module: Any, metadata: dict, t: np.ndarray, p: np.ndarray,
                   q: np.ndarray, samples: int) -> dict[str, Any]:
    duration = metadata["draw_duration_s"]
    s = np.clip((t-metadata["draw_start_s"])/duration, 0., 1.)
    expected_p = module.position_derivatives(metadata, s)[0]
    expected_q = module.orientation(metadata, s)
    p_error = float(_norm(p-expected_p).max())
    rotation_error = float(np.rad2deg((Rotation.from_quat(q)*Rotation.from_quat(expected_q).inv()).magnitude()).max())
    endpoints = module.position_derivatives(metadata, np.array([0., 1.]))
    angular_endpoints = module.orientation_diagnostics(metadata, np.array([0., 1.]))
    endpoint_values = {"max_speed_m_s": float(_norm(endpoints[1]).max()/duration),
                       "max_acceleration_m_s2": float(_norm(endpoints[2]).max()/duration**2),
                       "max_jerk_m_s3": float(_norm(endpoints[3]).max()/duration**3),
                       "max_angular_speed_deg_s": float(np.rad2deg(_norm(angular_endpoints["omega_rad_s"])).max()),
                       "max_angular_acceleration_deg_s2": float(np.rad2deg(_norm(angular_endpoints["alpha_rad_s2"])).max()),
                       "max_angular_jerk_deg_s3": float(np.rad2deg(_norm(angular_endpoints["jerk_rad_s3"])).max())}
    position_certificate = module.derivative_certificate(metadata, samples=samples)
    angular_certificate = module.angular_certificate(metadata, samples=samples)
    bound = position_certificate["normalized_position_derivative_bounds"]
    angular_bound = angular_certificate["angular_derivative_bounds_deg"]
    physical = {"speed_m_s": bound[1]/duration, "acceleration_m_s2": bound[2]/duration**2,
                "jerk_m_s3": bound[3]/duration**3, "angular_speed_deg_s": angular_bound[0],
                "angular_acceleration_deg_s2": angular_bound[1], "angular_jerk_deg_s3": angular_bound[2]}
    violations = {key: {"observed_bound": physical[key], "limit": limit}
                  for key, limit in LIMITS.items() if physical[key] > limit+1e-8}
    grid = np.linspace(0., 1., min(samples, 8193))
    pd = module.position_derivatives(metadata, grid)
    od = module.orientation_diagnostics(metadata, grid)
    xy = metadata["xy"]
    angle = xy["rotation_rad"]
    rotation_abs = np.abs(np.array([[math.cos(angle), -math.sin(angle)],
                                    [math.sin(angle), math.cos(angle)]]))
    unit_bound = np.asarray(module.patterns.derivative_bounds(metadata["family_id"], metadata["pattern_parameters"], max_order=0))[0]
    xy_abs_bound = np.abs(xy["center_m"])+rotation_abs@(np.asarray(xy["scale_m"])*unit_bound)
    declared_xy_bound = xy.get("analytic_coordinate_abs_bound_m")
    declared_xy_matches = bool(np.max(np.abs(xy_abs_bound-np.asarray(declared_xy_bound))) <= 1e-12) if declared_xy_bound is not None else None
    return {"source_reconstruction_pass": p_error <= 5e-10 and rotation_error <= 5e-8,
            "maximum_position_reconstruction_error_m": p_error,
            "maximum_SO3_reconstruction_error_deg": rotation_error,
            "endpoint_analytic_derivatives": endpoint_values,
            "zero_endpoint_velocity_and_acceleration_pass": all(value <= 1e-8 for key, value in endpoint_values.items() if "jerk" not in key),
            "zero_endpoint_jerk_pass": all(value <= 1e-7 for key, value in endpoint_values.items() if "jerk" in key),
            "continuous_position_certificate": position_certificate,
            "continuous_angular_certificate": angular_certificate,
            "physical_continuous_bounds": physical,
            "continuous_motion_limits_pass": not violations, "continuous_motion_limit_violations": violations,
            "continuous_XY_coordinate_abs_bound_m": xy_abs_bound.tolist(),
            "continuous_XY_main_range_pass": bool(np.max(xy_abs_bound) <= .13+1e-12),
            "declared_XY_coordinate_bound_matches_recomputed": declared_xy_matches,
            "analytic_grid_peaks": {"speed_m_s": float(_norm(pd[1]).max()/duration),
                                     "angular_speed_deg_s": float(np.rad2deg(_norm(od["omega_rad_s"])).max())},
            "independence_scope": "CSV reconstruction and endpoint evaluation use this batch's explicit source formulas. CSV finite differences remain independently implemented; continuous certificates are recomputed from the source analytic derivative/Lipschitz method, rather than trusting metadata numbers. Zeroth-order position norm and per-axis XY envelopes use global analytic bounds; sampled_position_derivative_maxima remains diagnostic."}


def _nominal_precheck(clearance_module: Any | None, metadata: dict,
                       p: np.ndarray, q: np.ndarray, t: np.ndarray,
                       geometry: dict | None, contract: dict | None,
                       formula: dict) -> dict[str, Any]:
    if clearance_module is None or geometry is None or contract is None:
        return {"status": "not_available", "passed": None,
                "scope": "No nominal geometry precheck was available; full robot collision and IK remain not_run."}
    # Only this in-memory diagnostic copy receives the freshly recomputed
    # certificates. Authoritative metadata is never rewritten or repaired.
    diagnostic_metadata = dict(metadata)
    diagnostic_metadata["continuous_position_certificate"] = formula["continuous_position_certificate"]
    diagnostic_metadata["continuous_angular_certificate"] = formula["continuous_angular_certificate"]
    result = clearance_module.precheck(diagnostic_metadata, p, q, t, geometry, contract)
    result = dict(result)
    result.setdefault("passed", str(result.get("status", "")).startswith("passed"))
    result.setdefault("status", "passed_nominal_geometry_precheck" if result.get("passed") else "rejected_nominal_geometry_precheck")
    result["uses_recomputed_continuous_motion_certificates"] = True
    result["metadata_precheck_status_matches_recomputed"] = metadata.get("clearance_precheck", {}).get("status") == result["status"]
    result["full_robot_collision_IK_playback"] = "not_run"
    return result


def audit_source(root: Path, record: dict, trajectory_module: Any,
                  clearance_module: Any | None, samples: int, max_geometry_points: int,
                  geometry: dict | None = None, contract: dict | None = None) -> Source:
    metadata, t, p, q, phase, base = _read_source(root, record)
    series = motion_series(t, p, q)
    begin, end = metadata["draw_start_s"], metadata["draw_end_s"]
    draw = (t >= begin-1e-10) & (t <= end+1e-10)
    angular_draw = (series["tm"] >= begin-1e-10) & (series["tm"] <= end+1e-10)
    peaks = {key: float(series[key].max()) for key in LIMITS}
    peaks["max_tangential_acceleration_m_s2"] = float(np.abs(series["signed_tangential_acceleration_m_s2"]).max())
    peaks["max_normal_centripetal_acceleration_m_s2"] = float(series["normal_centripetal_acceleration_m_s2"].max())
    violations = {key: {"observed": peaks[key], "limit": limit}
                  for key, limit in LIMITS.items() if peaks[key] > limit+1e-7}
    dt = np.diff(t)
    rates = {"full": {"speed_m_s": _stats(series["speed"], t),
                       "angular_speed_deg_s": _stats(series["angular_speed_deg_s"], weights=dt),
                       "nominal_tool_axis_rate_deg_s": _stats(series["axis_rate_deg_s"], weights=dt)},
             "draw": {"speed_m_s": _stats(series["speed"][draw], t[draw]),
                       "angular_speed_deg_s": _stats(series["angular_speed_deg_s"][angular_draw], weights=dt[angular_draw]),
                       "nominal_tool_axis_rate_deg_s": _stats(series["axis_rate_deg_s"][angular_draw], weights=dt[angular_draw])}}
    nominal_v, nominal_w = float(metadata["nominal_center_speed_m_s"]), float(metadata["nominal_axis_rate_deg_s"])
    speed_peak = rates["draw"]["speed_m_s"]["max"]
    axis_peak = rates["draw"]["nominal_tool_axis_rate_deg_s"]["max"]
    tiers = {"v_tier": metadata["v_tier"], "nominal_center_speed_m_s": nominal_v,
             "w_tier": metadata["w_tier"], "nominal_tool_axis_rate_deg_s": nominal_w,
             "V_definition": "predetermined peak centre speed for these mixed/continuous candidates",
             "W_definition": "nominal tool +Z axis direction-change peak; physical magnet axis remains uncalibrated",
             "source_peak_speed_ratio_to_nominal": speed_peak/nominal_v if nominal_v else None,
             "source_peak_axis_rate_ratio_to_nominal": axis_peak/nominal_w if nominal_w else None,
             "speed_tier_achieved": bool(.985*nominal_v <= speed_peak <= 1.002*nominal_v) if nominal_v else speed_peak <= 1e-6,
             "axis_rate_tier_achieved": bool(.985*nominal_w <= axis_peak <= 1.002*nominal_w) if nominal_w else axis_peak <= 1e-6,
             "tier_peak_acceptance_ratio": [.985, 1.002]}
    if metadata["v_tier"] not in V_TIERS or abs(nominal_v-V_TIERS.get(metadata["v_tier"], -1)) > 1e-12:
        raise ValueError("declared V tier is inconsistent with the 10/20/50 mm/s contract")
    if metadata["w_tier"] not in W_TIERS or abs(nominal_w-W_TIERS.get(metadata["w_tier"], -1)) > 1e-12:
        raise ValueError("declared W tier is inconsistent with the 0/3/10/20 deg/s contract")
    fixed = metadata["orientation_mode"] == "fixed"
    if fixed and not np.all(q == q[0]):
        raise ValueError("fixed-orientation source changes quaternion")
    if fixed != (metadata["w_tier"] == "W0"):
        raise ValueError("fixed/varying orientation and W0 declaration disagree")
    profile = metadata["height_profile"]
    if profile["mode"] == "constant" and np.ptp(p[:, 2]) > 2e-10:
        raise ValueError("stratified constant-height source changes height")
    if abs(float((-p[:, 2]).min())-profile["lower_m"]) > 2e-7 or abs(float((-p[:, 2]).max())-profile["upper_m"]) > 2e-7:
        raise ValueError("source height extrema do not cover its declared height profile")
    formula = _formula_audit(trajectory_module, metadata, t, p, q, samples)
    interval_draw = base.pop("draw_interval_mask")
    bins_full = weighted_bins(t, -p[:, 2], HEIGHT_EDGES_M)
    bins_draw = weighted_bins(t, -p[:, 2], HEIGHT_EDGES_M, interval_draw)
    endpoint_derivatives = {key: float(_norm(series[key][[0, -1]]).max()) for key in ("v", "a", "j")}
    base["sampled_endpoint_velocity_and_acceleration_zero"] = endpoint_derivatives["v"] < 1e-12 and endpoint_derivatives["a"] < 1e-12
    audit = {"trajectory_id": record["trajectory_id"], "group_id": metadata["group_id"],
             "family_id": metadata["family_id"], "geometry_id": metadata.get("geometry_id", metadata["family_id"]),
             "direction": metadata["direction"], "orientation_mode": metadata["orientation_mode"],
             "seed": metadata["seed"], "height_profile": profile,
             "source_contract_pass": True, "authority": base,
             "source_grid_motion_limits_pass": not violations, "source_grid_peaks": peaks,
             "source_grid_motion_limit_violations": violations,
             "finite_difference_method": "numpy.gradient with authoritative time, second-order edges; spatial midpoint SO3 log increments, then fixed-basis vector gradients; full holds retained",
             "sampled_endpoint_derivatives": endpoint_derivatives, "rates": rates, "tiers": tiers,
             "formula": formula, "geometry": geometry_diagnostics(t, p, series, draw, max_geometry_points),
             "height_histogram": {"edges_m": HEIGHT_EDGES_M.tolist(), "full_time_s": bins_full.tolist(),
                                  "draw_time_s": bins_draw.tolist(), "full_total_s": float(t[-1]), "draw_total_s": float(end-begin)},
             "nominal_geometry_precheck": _nominal_precheck(clearance_module, metadata, p, q, t, geometry, contract, formula),
             "robot_simulation": {"GPU_compile": "not_run", "IK": "not_run", "full_robot_collision": "not_run", "mock_recording": "not_run", "independent_playback": "not_run"}}
    audit["source_level_pass"] = bool(not violations and formula["source_reconstruction_pass"]
                                      and formula["continuous_motion_limits_pass"]
                                      and formula["continuous_XY_main_range_pass"]
                                      and formula["declared_XY_coordinate_bound_matches_recomputed"] is not False
                                      and formula["zero_endpoint_velocity_and_acceleration_pass"]
                                      and base["sampled_endpoint_velocity_and_acceleration_zero"])
    return Source(record, metadata, t, p, q, phase, series, audit)


def _counts(values) -> dict[str, int]:
    return dict(sorted(Counter(str(value) for value in values).items()))


def _coverage(sources: list[Source]) -> dict[str, Any]:
    full = sum((np.asarray(s.audit["height_histogram"]["full_time_s"]) for s in sources), np.zeros(len(HEIGHT_EDGES_M)-1))
    draw = sum((np.asarray(s.audit["height_histogram"]["draw_time_s"]) for s in sources), np.zeros(len(HEIGHT_EDGES_M)-1))
    full_total = sum(s.t[-1] for s in sources)
    draw_total = sum(s.metadata["draw_duration_s"] for s in sources)
    high = HEIGHT_EDGES_M[:-1] >= .15-1e-10
    def percentages(bins, total):
        fractions = bins/total if total else np.zeros_like(bins)
        return {"total_time_s": float(total), "bin_time_s": bins.tolist(), "bin_fraction": fractions.tolist(),
                "high_150_200mm_fraction": float(fractions[high].sum()),
                "high_fraction_deviation_percentage_points_from_60pct": float((fractions[high].sum()-.6)*100.),
                "five_high_bin_fraction": fractions[high].tolist(),
                "five_high_bin_deviation_percentage_points_from_12pct": ((fractions[high]-.12)*100.).tolist()}
    return {"count": len(sources), "height_edges_m": HEIGHT_EDGES_M.tolist(),
            "full": percentages(full, full_total), "draw": percentages(draw, draw_total),
            "v_tier_counts": _counts(s.metadata["v_tier"] for s in sources),
            "w_tier_counts": _counts(s.metadata["w_tier"] for s in sources),
            "V_by_W_counts": _counts(f"{s.metadata['v_tier']}:{s.metadata['w_tier']}" for s in sources),
            "family_V_W_counts": _counts(f"{s.metadata['family_id']}:{s.metadata['v_tier']}:{s.metadata['w_tier']}" for s in sources),
            "orientation_counts": _counts(s.metadata["orientation_mode"] for s in sources),
            "direction_counts": _counts(s.metadata["direction"] for s in sources),
            "source_level_pass_count": sum(s.audit["source_level_pass"] for s in sources),
            "speed_tier_achieved_count": sum(s.audit["tiers"]["speed_tier_achieved"] for s in sources),
            "axis_rate_tier_achieved_count": sum(s.audit["tiers"]["axis_rate_tier_achieved"] for s in sources),
            "nominal_precheck_status_counts": _counts(s.audit["nominal_geometry_precheck"]["status"] for s in sources),
            "histogram_method": "Exact interval/bin splits for the CSV piecewise-linear height; draw excludes both 0.5 s holds. Bins are [lo,hi), with 200 mm in the final bin; fixed layers also have their own per-layer table."}


def _geometry_coverage(sources: list[Source]) -> dict[str, Any]:
    """Aggregate measured geometry without treating planar layers as failures."""
    groups = defaultdict(list)
    for source in sources:
        groups[f"{source.metadata['group_id']}:{source.metadata['family_id']}"].append(source)
    result = {}
    for key, group in sorted(groups.items()):
        geometries = {source.metadata.get("geometry_id", source.metadata["family_id"]): source for source in group}
        representatives = list(geometries.values())
        records = [s.audit["geometry"] for s in representatives]
        positions = [s.metadata.get("xy", {}) for s in representatives]
        result[key] = {"source_count": len(group), "distinct_seeded_geometry_count": len(representatives),
                       "seed_count": len({s.metadata["seed"] for s in representatives}),
                       "geometry_ids": sorted(geometries),
                       "path_length_m": _range_stats(np.array([g["path_length_m"] for g in records])),
                       "nonplanarity_rms_m": _range_stats(np.array([g["SVD_nonplanarity_rms_m"] for g in records])),
                       "curvature_p95_per_m": _range_stats(np.array([g["curvature_per_m"]["p95"] for g in records], dtype=float)),
                       "absolute_torsion_p95_per_m": _range_stats(np.array([g["absolute_torsion_per_m"]["p95"] for g in records], dtype=float)),
                       "tangent_total_turn_deg": _range_stats(np.array([g["tangent_total_turn_deg"] for g in records])),
                       "XYZ_reversal_counts_by_geometry": {identifier: source.audit["geometry"]["coordinate_reversal_counts_xyz"] for identifier, source in geometries.items()},
                       "height_reversal_counts_by_geometry": {identifier: source.audit["geometry"]["height_reversal_count"] for identifier, source in geometries.items()},
                       "XY_projection_crossing_count_by_geometry": {identifier: source.audit["geometry"]["XY_projection_crossings"]["count"] for identifier, source in geometries.items()},
                       "XY_projection_5mm_separated_crossing_count_by_geometry": {identifier: source.audit["geometry"]["XY_projection_crossings"]["count_height_separation_at_least_5mm"] for identifier, source in geometries.items()},
                       "maximum_crossing_height_difference_m": max((g["XY_projection_crossings"]["height_separation_m"]["max"] or 0. for g in records), default=0.),
                       "XY_center_m_by_geometry": [value.get("center_m") for value in positions],
                       "XY_scale_m_by_geometry": [value.get("scale_m") for value in positions],
                       "XY_rotation_rad_by_geometry": [value.get("rotation_rad") for value in positions],
                       "XY_parameter_offset_by_geometry": [value.get("parameter_offset") for value in positions]}
    return result


def _normalized_position(source: Source, points: int = 1001) -> np.ndarray:
    normalized = np.clip((source.t-source.metadata["draw_start_s"])/source.metadata["draw_duration_s"], 0., 1.)
    unique, indexes = np.unique(normalized, return_index=True)
    return np.column_stack([np.interp(np.linspace(0., 1., points), unique, source.p[indexes, k]) for k in range(3)])


def control_pairs(sources: list[Source]) -> dict[str, Any]:
    """Compare controls using CSV interpolation in normalized draw time.

    The 50 um tolerance exceeds two 0.05 s chord interpolation bounds at the
    unchanged 0.05 m/s² acceleration limit (2*a*dt²/8 = 31.25 um).
    It is a source pairing diagnostic, not an execution tracking tolerance.
    """
    tolerance = 5e-5
    cache: dict[str, np.ndarray] = {}
    def position(source):
        key = source.record["trajectory_id"]
        if key not in cache:
            cache[key] = _normalized_position(source)
        return cache[key]
    def geometry_id(source):
        return source.metadata.get("geometry_id", source.metadata["family_id"])
    pose_groups = defaultdict(list)
    for source in sources:
        m = source.metadata
        pose_groups[(m["group_id"], geometry_id(source), m["v_tier"], m["direction"], m["height_profile"]["lower_m"])].append(source)
    pose = []
    for key, group in sorted(pose_groups.items()):
        fixed = [s for s in group if s.metadata["orientation_mode"] == "fixed"]
        varying = [s for s in group if s.metadata["orientation_mode"] != "fixed"]
        if len(fixed) != 1 or len(varying) != 1:
            pose.append({"key": list(key), "pass": False, "reason": "expected exactly one fixed and one varying pose member", "members": [s.record["trajectory_id"] for s in group]})
            continue
        error = float(_norm(position(fixed[0])-position(varying[0])).max())
        pose.append({"key": list(key), "members": [fixed[0].record["trajectory_id"], varying[0].record["trajectory_id"]],
                     "maximum_position_difference_m": error, "pass": error <= tolerance})
    lookup = {(s.metadata["group_id"], geometry_id(s), s.metadata["v_tier"], s.metadata["orientation_mode"]): s for s in sources}
    speed, reverse, guarded = [], [], []
    for source in sources:
        m = source.metadata
        if m["group_id"] == "full_height" and m["v_tier"] == "V1":
            for tier in ("V2", "V3"):
                other = lookup.get(("full_height", geometry_id(source), tier, m["orientation_mode"]))
                error = float(_norm(position(source)-position(other)).max()) if other else None
                speed.append({"members": [source.record["trajectory_id"], other.record["trajectory_id"] if other else None],
                              "maximum_position_difference_m": error, "pass": error is not None and error <= tolerance})
        if m["group_id"] in ("full_height_reverse", "guarded_full_height"):
            comparison_id = m.get("comparison_geometry_id", geometry_id(source))
            other = lookup.get(("full_height", comparison_id, "V2", m["orientation_mode"]))
            expected = position(other).copy() if other else None
            if expected is not None and m["group_id"] == "full_height_reverse":
                expected = expected[::-1]
            elif expected is not None:
                expected[:, 2] = -(.075+(-expected[:, 2]-.07)*(.20-.075)/(.20-.07))
            # Guarded height scheduling may preserve bin time proportions rather
            # than affine heights. XY is the paired geometry; height is separately
            # required to cover its declared 75--200 mm profile.
            dimensions = slice(None) if m["group_id"] == "full_height_reverse" else slice(0, 2)
            error = float(_norm(position(source)[:, dimensions]-expected[:, dimensions]).max()) if expected is not None else None
            item = {"members": [other.record["trajectory_id"] if other else None, source.record["trajectory_id"]],
                    "compared_coordinates": "XYZ_reversed" if m["group_id"] == "full_height_reverse" else "XY_same_normalized_time",
                    "maximum_position_difference_m": error, "pass": error is not None and error <= tolerance}
            (reverse if m["group_id"] == "full_height_reverse" else guarded).append(item)
    return {"position_comparison_tolerance_m": tolerance, "comparison_grid_points": 1001,
            "scope": "Controls compared from CSV in normalized draw time, retaining original CSV times. No source retiming or mutation occurs.",
            "fixed_varying_pose_pairs": pose, "cross_speed_pairs": speed,
            "forward_reverse_pairs": reverse, "70mm_75mm_XY_control_pairs": guarded,
            "pair_counts": {"fixed_varying": len(pose), "cross_speed": len(speed), "forward_reverse": len(reverse), "70mm_75mm": len(guarded)},
            "all_observed_pair_comparisons_pass": all(item["pass"] for items in (pose, speed, reverse, guarded) for item in items)}


def _metrics_row(source: Source) -> dict[str, Any]:
    a, m = source.audit, source.metadata
    row = {"trajectory_id": m["trajectory_id"], "group_id": m["group_id"], "family_id": m["family_id"],
           "geometry_id": m.get("geometry_id", m["family_id"]), "seed": m["seed"], "v_tier": m["v_tier"],
           "w_tier": m["w_tier"], "orientation_mode": m["orientation_mode"], "direction": m["direction"],
           "height_profile_mode": m["height_profile"]["mode"], "height_min_m": a["authority"]["height_min_m"],
           "height_max_m": a["authority"]["height_max_m"], "duration_s": source.t[-1], "draw_duration_s": m["draw_duration_s"],
           "source_level_pass": a["source_level_pass"], "CSV_sha256": a["authority"]["sha256"],
           "metadata_sha256": a["authority"]["metadata_sha256"], "source_grid_motion_limits_pass": a["source_grid_motion_limits_pass"],
           "continuous_motion_limits_pass": a["formula"]["continuous_motion_limits_pass"],
           "nominal_geometry_precheck_status": a["nominal_geometry_precheck"]["status"],
           "robot_IK_collision_simulation": "not_run", "speed_tier_achieved": a["tiers"]["speed_tier_achieved"],
           "axis_rate_tier_achieved": a["tiers"]["axis_rate_tier_achieved"],
           "nominal_center_speed_m_s": m["nominal_center_speed_m_s"], "nominal_tool_axis_rate_deg_s": m["nominal_axis_rate_deg_s"]}
    for section in ("full", "draw"):
        for kind, stats in a["rates"][section].items():
            for stat in ("mean", "p95", "max"):
                row[f"{section}_{kind}_{stat}"] = stats[stat]
    row.update({f"source_grid_{key}": value for key, value in a["source_grid_peaks"].items()})
    row.update({f"continuous_bound_{key}": value for key, value in a["formula"]["physical_continuous_bounds"].items()})
    g = a["geometry"]
    row.update({"path_length_m": g["path_length_m"], "SVD_nonplanarity_rms_m": g["SVD_nonplanarity_rms_m"],
                "SVD_smallest_to_largest_ratio": g["SVD_smallest_to_largest_ratio"],
                "tangent_total_turn_deg": g["tangent_total_turn_deg"], "height_reversal_count": g["height_reversal_count"],
                "X_reversal_count": g["coordinate_reversal_counts_xyz"][0], "Y_reversal_count": g["coordinate_reversal_counts_xyz"][1],
                "Z_reversal_count": g["coordinate_reversal_counts_xyz"][2], "XY_crossing_count": g["XY_projection_crossings"]["count"],
                "XY_crossing_5mm_height_count": g["XY_projection_crossings"]["count_height_separation_at_least_5mm"],
                "XY_crossing_max_height_difference_m": g["XY_projection_crossings"]["height_separation_m"]["max"]})
    for name in ("curvature_per_m", "signed_torsion_per_m", "absolute_torsion_per_m"):
        for stat in ("median", "p95", "max"):
            row[f"{name}_{stat}"] = g[name][stat]
    return row


def _write_csv(path: Path, rows: list[dict], fields: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fields is None:
        fields = list(rows[0]) if rows else []
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fields)
        writer.writeheader()
        writer.writerows(_clean(rows))


def _markdown(report: dict[str, Any]) -> str:
    s = report["summary"]
    lines = ["# Continuous patterns: independent source audit", "",
             f"Source-level pass: **{s['source_level_pass_count']}/{s['manifest_record_count']}**. "
             f"CSV contract failures: {s['source_contract_failure_count']}. "
             f"Nominal geometry prechecks rejected: {s['nominal_geometry_rejected_count']}. "
             "**Full robot collision, IK, GPU compile and robot/mock playback: not run.**", "",
             AUTHORITY_WARNING, "",
             "The CSV waypoint poses and time_s are retained exactly. Source checks do not authorize real execution. "
             "Source passing and collision/precheck rejection are independent statuses.", "",
             "## Groups and height coverage", "",
             "| Group | Count | Source pass | Draw seconds | Draw 15–20 cm | Full 15–20 cm | Nominal precheck |",
             "|---|---:|---:|---:|---:|---:|---|"]
    for group, c in report["coverage_by_group"].items():
        lines.append(f"| {group} | {c['count']} | {c['source_level_pass_count']} | {c['draw']['total_time_s']:.2f} | "
                     f"{100*c['draw']['high_150_200mm_fraction']:.2f}% | {100*c['full']['high_150_200mm_fraction']:.2f}% | "
                     f"{json.dumps(c['nominal_precheck_status_counts'], ensure_ascii=False)} |")
    overall = report["coverage_all"]
    lines += ["", f"All draw time in 15–20 cm: {100*overall['draw']['high_150_200mm_fraction']:.2f}%; "
              f"deviation from 60%: {overall['draw']['high_fraction_deviation_percentage_points_from_60pct']:+.2f} percentage points. "
              "Each group's actual histogram is kept in source_audit.json and height_histograms.csv; constant-height layers are listed independently below.", "",
              "| Constant layer (mm) | Count | Draw seconds | Source pass | Nominal precheck |",
              "|---|---:|---:|---:|---|"]
    for layer, c in report["coverage_by_constant_layer_mm"].items():
        lines.append(f"| {layer} | {c['count']} | {c['draw']['total_time_s']:.2f} | {c['source_level_pass_count']} | {json.dumps(c['nominal_precheck_status_counts'])} |")
    lines += ["", "## Limits, geometry and controls", "",
              "Full XYZ acceleration and jerk use independent authoritative-time finite differences. "
              "SO(3) log increments provide full flange angular velocity; vector gradients provide angular acceleration and jerk. "
              "The source formulas independently reconstruct every CSV pose and supply recomputed continuous derivative/Lipschitz enclosures. "
              "Limits are 0.05 m/s², 0.25 m/s³, 30 °/s² and 150 °/s³. Both 0.5 s holds remain in full statistics; draw statistics exclude them.", "",
              f"Achieved V peak tiers: {s['speed_tier_achieved_count']}/{s['valid_source_count']}; "
              f"achieved W direction-rate tiers: {s['axis_rate_tier_achieved_count']}/{s['valid_source_count']}. "
              "Mean, time-weighted P95 and maximum are recorded for speed, full angular rate and nominal tool-axis rate. "
              "Tier acceptance is 98.5–100.2% of the declared peak, with exact zero for fixed pose; gaps are retained in metadata/reporting.", "",
              f"Control counts: `{json.dumps(report['control_pairs']['pair_counts'])}`; "
              f"all observed control comparisons pass: `{report['control_pairs']['all_observed_pair_comparisons_pass']}`. "
              "Position comparisons interpolate source CSVs only for diagnostics at normalized draw time, with a 50 µm chord-error allowance; authoritative CSV times are unchanged.", "",
              "Curvature and torsion use complete vector derivatives, omitting near-rest/straight singularities. "
              "SVD nonplanarity uses equal-arclength source samples; tangent turns and XYZ/height reversals quantify geometric richness. "
              "XY crossings are transverse nonadjacent polyline segment intersections with ≥5 mm arclength separation. "
              "Each result includes measured decimation chord error against all original CSV vertices and interpolated crossing-height differences. "
              "These crossings are sampled diagnostics, not a completeness proof for the continuous curve.", "",
              "## 70 mm request and 75 mm controls", "",
              f"70 mm candidates retained: {s['seven_cm_candidate_count']}; nominal geometry rejected: {s['seven_cm_geometry_rejected_count']}. "
              f"75 mm controls retained: {s['seventy_five_mm_control_count']}; nominal geometry rejected: {s['seventy_five_mm_geometry_rejected_count']}.", "",
              "The existing nominal scene and protection thresholds are unchanged. Rejections do not silently move a 70 mm candidate upward. "
              "The geometry precheck covers only the stated rod/obstacle envelope; calibration acceptance and full robot feasibility remain outstanding."]
    if report["source_contract_failures"]:
        lines += ["", "## Source contract failures", ""]
        lines += [f"- `{item['trajectory_id']}`: {item['error']}" for item in report["source_contract_failures"]]
    if report["design_findings"]:
        lines += ["", "## Coverage or design findings", ""]
        lines += [f"- {finding}" for finding in report["design_findings"]]
    return "\n".join(lines) + "\n"


def run(root: Path, manifest_path: Path, samples: int = 32769,
        max_geometry_points: int = 1601) -> dict[str, Any]:
    root = root.resolve()
    manifest_path = _resolve(root, manifest_path)
    batch_root = manifest_path.parent
    code_root = root if (root / "src" / "trajectory.py").is_file() else Path(__file__).resolve().parents[1]
    if not batch_root.is_relative_to(code_root):
        raise ValueError("batch and reports must remain within this continuous_patterns tree")
    manifest = _json(manifest_path)
    records = manifest["records"] if isinstance(manifest, dict) else manifest
    if not isinstance(records, list):
        raise ValueError("manifest records must be a list")
    if not records:
        raise ValueError("an empty manifest is not an auditable candidate bank")
    if isinstance(manifest, dict) and manifest.get("trajectory_count", len(records)) != len(records):
        raise ValueError("manifest trajectory_count differs from its records")
    if len({record["trajectory_id"] for record in records}) != len(records):
        raise ValueError("manifest contains duplicate trajectory IDs")
    sys.path.insert(0, str(code_root.parent))
    trajectory_module = importlib.import_module(f"{code_root.name}.src.trajectory")
    try:
        clearance_module = importlib.import_module(f"{code_root.name}.src.clearance")
    except ModuleNotFoundError as error:
        if error.name != f"{code_root.name}.src.clearance":
            raise
        clearance_module = None
    geometry_path, contract_path = batch_root / "frozen_geometry.json", batch_root / "contract.json"
    geometry = _json(geometry_path) if geometry_path.exists() else None
    contract = _json(contract_path) if contract_path.exists() else None
    geometry_contract_checks = {}
    if geometry is not None and contract is not None:
        rod = geometry["setup"]["tool"]["rod"]
        board = geometry["zones"]["curobo_scene"]["cuboid"]["sensor_board_keepout"]
        boundary = geometry["zones"]["analysis_zones"]["sensor_keepout_clearance_boundary"]
        geometry_contract_checks = {"rod_length_m": [rod["length_m"], .190], "rod_radius_m": [rod["radius_m"], .0025],
                                    "board_keepout_top_z_m": [board["pose"][2]-board["dims"][2]/2., -.060],
                                    "analytical_tip_boundary_offset_m": [boundary["offset_m"], .065],
                                    "minimum_non_tip_clearance_m": [geometry["zones"]["contact_policy"]["minimum_non_tip_clearance_m"], .005],
                                    "provisional_position_uncertainty_radius_m": [contract["provisional_position_uncertainty_radius_m"], .003],
                                    "provisional_orientation_uncertainty_half_angle_deg": [contract["provisional_orientation_uncertainty_half_angle_deg"], 2.]}
        if any(abs(actual-expected) > 1e-12 for actual, expected in geometry_contract_checks.values()):
            raise ValueError("frozen nominal rod/keepout/uncertainty contract differs from the unchanged 190 mm contract")
    source_provenance = []
    for name in ("trajectory.py", "patterns.py", "clearance.py"):
        live, frozen = code_root / "src" / name, batch_root / "provenance" / "generator_sources" / "src" / name
        if not live.exists():
            continue
        item = {"source_file": str(live.relative_to(code_root)), "sha256": _sha(live.read_bytes()),
                "frozen_source_file": str(frozen.relative_to(batch_root)), "matches_frozen_generation_source": None}
        if frozen.exists():
            item["frozen_sha256"] = _sha(frozen.read_bytes())
            item["matches_frozen_generation_source"] = item["sha256"] == item["frozen_sha256"]
            if not item["matches_frozen_generation_source"]:
                raise ValueError(f"source formula {name} differs from the frozen generation source")
        source_provenance.append(item)
    sources, failures = [], []
    for index, record in enumerate(records):
        try:
            source = audit_source(batch_root, record, trajectory_module, clearance_module, samples, max_geometry_points,
                                  geometry, contract)
            sources.append(source)
        except (KeyError, ValueError, TypeError, OSError, csv.Error, json.JSONDecodeError) as error:
            failures.append({"trajectory_id": record.get("trajectory_id", f"record_{index}"),
                             "source_contract_pass": False, "error": f"{type(error).__name__}: {error}"})
        if (index+1) % 10 == 0 or index+1 == len(records):
            print(f"audited {index+1}/{len(records)} source records", flush=True)
    groups = {group: [s for s in sources if s.metadata["group_id"] == group]
              for group in sorted({s.metadata["group_id"] for s in sources})}
    layered = {f"{1000*height:.0f}": [s for s in sources if s.metadata["height_profile"]["mode"] == "constant"
                                        and abs(s.metadata["height_profile"]["lower_m"]-height) < 1e-9]
               for height in sorted({s.metadata["height_profile"]["lower_m"] for s in sources if s.metadata["height_profile"]["mode"] == "constant"})}
    seven = [s for s in sources if abs(s.metadata["height_profile"]["lower_m"]-.07) < 1e-9]
    guarded = [s for s in sources if abs(s.metadata["height_profile"]["lower_m"]-.075) < 1e-9]
    def rejected(source):
        precheck = source.audit["nominal_geometry_precheck"]
        return precheck.get("passed") is False or str(precheck.get("status", "")).startswith("reject")
    findings = []
    for group, count in EXPECTED_GROUP_COUNTS.items():
        if len(groups.get(group, [])) != count:
            findings.append(f"{group}: expected {count} valid sources, observed {len(groups.get(group, []))}")
    for height in np.arange(7, 21)/100.:
        layer = layered.get(f"{height*1000:.0f}", [])
        if len(layer) != 4:
            findings.append(f"stratified layer {height*1000:.0f} mm: expected 4 sources, observed {len(layer)}")
    pairs = control_pairs(sources)
    if not pairs["all_observed_pair_comparisons_pass"]:
        findings.append("One or more fixed/varying, speed, direction or 70/75 mm source-control comparisons failed; inspect control_pairs in JSON")
    tier_gaps = [s.metadata["trajectory_id"] for s in sources if not (s.audit["tiers"]["speed_tier_achieved"] and s.audit["tiers"]["axis_rate_tier_achieved"])]
    if tier_gaps:
        findings.append(f"{len(tier_gaps)} source records do not reach both declared V and W peak tiers; see tier_gap_ids (no hidden speed reduction)")
    nonplanar = [s for s in sources if s.audit["geometry"]["SVD_nonplanarity_rms_m"] >= .001]
    crossings = [s for s in sources if s.audit["geometry"]["XY_projection_crossings"]["count_height_separation_at_least_5mm"] > 0]
    report = {"report_schema": "continuous_patterns_source_audit_v1", "manifest_file": str(manifest_path.relative_to(code_root)),
              "manifest_sha256": _sha(manifest_path.read_bytes()), "authority_warning": AUTHORITY_WARNING,
              "source_formula_provenance": source_provenance,
              "auditor_file": str(Path(__file__).resolve().relative_to(code_root)),
              "auditor_sha256": _sha(Path(__file__).read_bytes()),
              "frozen_geometry_sha256": _sha(geometry_path.read_bytes()) if geometry_path.exists() else None,
              "contract_sha256": _sha(contract_path.read_bytes()) if contract_path.exists() else None,
              "unchanged_nominal_geometry_contract_actual_expected": geometry_contract_checks,
              "limits": LIMITS, "requested_height_range_m": [.07, .20], "main_xy_range_m": [-.13, .13],
              "candidate_only": True, "runtime_mutation_permitted": False,
              "robot_simulation_status": "not_run", "real_execution_authorized": False,
              "summary": {"manifest_record_count": len(records), "valid_source_count": len(sources),
                          "source_contract_failure_count": len(failures), "source_contract_pass_count": len(sources),
                          "source_level_pass_count": sum(s.audit["source_level_pass"] for s in sources),
                          "source_grid_motion_limits_pass_count": sum(s.audit["source_grid_motion_limits_pass"] for s in sources),
                          "continuous_motion_limits_pass_count": sum(s.audit["formula"]["continuous_motion_limits_pass"] for s in sources),
                          "speed_tier_achieved_count": sum(s.audit["tiers"]["speed_tier_achieved"] for s in sources),
                          "axis_rate_tier_achieved_count": sum(s.audit["tiers"]["axis_rate_tier_achieved"] for s in sources),
                          "nominal_geometry_rejected_count": sum(rejected(s) for s in sources),
                          "seven_cm_candidate_count": len(seven), "seven_cm_geometry_rejected_count": sum(rejected(s) for s in seven),
                          "seventy_five_mm_control_count": len(guarded), "seventy_five_mm_geometry_rejected_count": sum(rejected(s) for s in guarded),
                          "geometrically_nonplanar_at_least_1mm_count": len(nonplanar),
                          "projected_crossings_separated_by_at_least_5mm_record_count": len(crossings)},
              "coverage_all": _coverage(sources), "coverage_by_group": {group: _coverage(items) for group, items in groups.items()},
              "coverage_by_constant_layer_mm": {layer: _coverage(items) for layer, items in layered.items()},
              "geometry_coverage_by_group_family": _geometry_coverage(sources),
              "control_pairs": pairs, "tier_gap_ids": tier_gaps, "design_findings": findings,
              "source_contract_failures": failures, "trajectories": [s.audit for s in sources]}
    reports = batch_root / "reports"
    _write_json(reports / "source_audit.json", report)
    (reports / "source_audit.md").write_text(_markdown(report), encoding="utf-8")
    _write_csv(reports / "metrics.csv", [_metrics_row(source) for source in sources])
    histogram_rows = []
    collections = {"all": report["coverage_all"], **{f"group:{k}": v for k, v in report["coverage_by_group"].items()},
                   **{f"layer_mm:{k}": v for k, v in report["coverage_by_constant_layer_mm"].items()}}
    for collection, coverage in collections.items():
        for section in ("draw", "full"):
            for index, (lower, upper) in enumerate(zip(HEIGHT_EDGES_M[:-1], HEIGHT_EDGES_M[1:])):
                histogram_rows.append({"collection": collection, "time_scope": section, "height_lower_m": lower,
                                       "height_upper_m": upper, "upper_inclusive": index == len(HEIGHT_EDGES_M)-2,
                                       "time_s": coverage[section]["bin_time_s"][index], "fraction": coverage[section]["bin_fraction"][index]})
    _write_csv(reports / "height_histograms.csv", histogram_rows)
    crossing_rows = []
    for source in sources:
        for event in source.audit["geometry"]["XY_projection_crossings"]["events"]:
            crossing_rows.append({"trajectory_id": source.metadata["trajectory_id"], "group_id": source.metadata["group_id"],
                                  "x_m": event["xy_m"][0], "y_m": event["xy_m"][1],
                                  "time_a_s": event["times_s"][0], "time_b_s": event["times_s"][1],
                                  "height_a_m": event["heights_m"][0], "height_b_m": event["heights_m"][1],
                                  "height_separation_m": event["height_separation_m"], "crossing_angle_deg": event["crossing_angle_deg"],
                                  "arc_separation_m": event["arc_separation_m"]})
    _write_csv(reports / "XY_crossings.csv", crossing_rows, ["trajectory_id", "group_id", "x_m", "y_m", "time_a_s", "time_b_s",
                                                          "height_a_m", "height_b_m", "height_separation_m", "crossing_angle_deg", "arc_separation_m"])
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--manifest", type=Path, default=Path("manifest.json"))
    parser.add_argument("--certificate-samples", type=int, default=32769)
    parser.add_argument("--max-geometry-points", type=int, default=1601)
    args = parser.parse_args()
    if args.certificate_samples < 257 or args.max_geometry_points < 101:
        parser.error("certificate/geometry sample counts are too small for this audit")
    report = run(args.root, args.manifest, args.certificate_samples, args.max_geometry_points)
    print(json.dumps(report["summary"], ensure_ascii=False, indent=2))
    return 0 if report["summary"]["source_level_pass_count"] == report["summary"]["manifest_record_count"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
