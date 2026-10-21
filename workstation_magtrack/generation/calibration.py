"""Author low-height geometry-calibration and independent timing motions.

All centers are *nominal rod_tip* targets in the frozen ``plane1`` frame.
An accepted magnetic-center/tool/S-frame calibration has not been applied.
Generation is offline; this module cannot start ROS or a physical device.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from .common import DT_S, motion_diagnostics, sample_grid, smootherstep7, write_authority, write_json_immutable


PROTOCOL = "real_collection_v2_20261016"
HEIGHTS_M = (0.075, 0.080, 0.100)
TILT_DEG = 20.0
SETTLE_S = 0.5
CAPTURE_S = 1.0
ROTATION_DURATION_S = 5.0
TOOL_LENGTH_M = 0.190
TOOL_RADIUS_M = 0.0025
POSITION_UNCERTAINTY_M = 0.003
ORIENTATION_UNCERTAINTY_DEG = 2.0
SENSOR_KEEP_OUT_FACE_Z_M = -0.060
ANALYTIC_TIP_BOUNDARY_Z_M = -0.065
NON_TIP_CLEARANCE_M = 0.005
S7_MAX_D1 = 2.1875
S7_MAX_D2 = 7.513188404399293
S7_MAX_D3 = 52.5


def orientations() -> list[dict[str, Any]]:
    result = []
    for name, axis, degrees in (
        ("normal", "x", 0.0),
        ("local_x_plus20", "x", TILT_DEG),
        ("local_x_minus20", "x", -TILT_DEG),
        ("local_y_plus20", "y", TILT_DEG),
        ("local_y_minus20", "y", -TILT_DEG),
    ):
        result.append({"orientation_id": name, "quaternion_xyzw": Rotation.from_euler(axis, degrees, degrees=True).as_quat().tolist()})
    return result


def sites() -> list[dict[str, Any]]:
    """Whole sites, including all heights, are inseparable split units."""
    result = []
    for y in (-0.1, 0.0, 0.1):
        for x in (-0.1, 0.0, 0.1):
            def label(value: float) -> str:
                return ("m" if value < 0 else "p") + f"{round(abs(value) * 1000):03d}"
            site_id = f"x{label(x)}_y{label(y)}"
            holdout = (x, y) in ((-0.1, 0.1), (0.1, -0.1))
            result.append({"site_id": site_id, "xy_m": [x, y], "split_group_id": f"calibration_site_{site_id}", "data_role": "geometry_holdout" if holdout else "geometry_fit"})
    return result


def low_height_precheck(height_m: float, quaternions: np.ndarray | None = None) -> dict[str, Any]:
    """Conservative board clearance for a rigid rod with provisional errors.

    A flange-position ball of radius 3 mm and an orientation cone of 2 deg
    propagate to every sphere along the 190 mm rod.  For the horizontal
    sensor face the exact worst positive-Z cone displacement is used rather
    than an isotropic L*sin(delta) approximation.  The tip sphere is included
    in the non-tip test conservatively even though contact policy treats the
    analytical rod_tip point separately.  Whole-robot/world collision checks
    remain a separate mandatory simulation stage.
    """
    if height_m <= 0 or not math.isfinite(height_m):
        raise ValueError("height must be finite and positive")
    if quaternions is None:
        quaternions = np.asarray([item["quaternion_xyzw"] for item in orientations()])
    axes = Rotation.from_quat(quaternions).apply(np.array([0.0, 0.0, 1.0]))
    nz = axes[:, 2]
    alpha = np.arccos(np.clip(nz, -1.0, 1.0))
    cone_cos_max = np.cos(np.maximum(0.0, alpha - math.radians(ORIENTATION_UNCERTAINTY_DEG)))
    distances_from_flange = np.linspace(0.0, TOOL_LENGTH_M, 39)
    nominal_centers_z = -height_m - (TOOL_LENGTH_M - distances_from_flange[None, :]) * nz[:, None]
    positive_z_rotation_error = distances_from_flange[None, :] * (cone_cos_max - nz)[:, None]
    worst_centers_z = nominal_centers_z + POSITION_UNCERTAINTY_M + positive_z_rotation_error
    worst_sphere_z = float(np.max(worst_centers_z) + TOOL_RADIUS_M)
    inflated_physical_clearance = SENSOR_KEEP_OUT_FACE_Z_M - worst_sphere_z
    tip_z_worst = float(np.max(worst_centers_z[:, -1]))
    analytic_margin = ANALYTIC_TIP_BOUNDARY_Z_M - tip_z_worst
    non_tip_margin = inflated_physical_clearance - NON_TIP_CLEARANCE_M
    passed = min(analytic_margin, non_tip_margin) >= -1.0e-12
    return {
        "nominal_height_m": height_m,
        "accepted": bool(passed),
        "scope": "sensor_keepout_rod_uncertainty_precheck_only",
        "position_uncertainty_radius_m": POSITION_UNCERTAINTY_M,
        "orientation_uncertainty_half_angle_deg": ORIENTATION_UNCERTAINTY_DEG,
        "maximum_rotation_induced_tip_z_error_m": float(np.max(positive_z_rotation_error[:, -1])),
        "worst_case_tip_z_m": tip_z_worst,
        "analytical_tip_boundary_z_m": ANALYTIC_TIP_BOUNDARY_Z_M,
        "analytical_tip_margin_m": analytic_margin,
        "worst_case_rod_sphere_surface_z_m": worst_sphere_z,
        "worst_case_rod_surface_to_sensor_keepout_m": inflated_physical_clearance,
        "required_non_tip_clearance_m": NON_TIP_CLEARANCE_M,
        "non_tip_clearance_margin_m": non_tip_margin,
        "reason": "passes provisional rod/board envelope; full-scene robot simulation still required" if passed else "rejected upstream: provisional flange-position/orientation envelope violates unchanged board clearance",
    }


class _Timeline:
    def __init__(self, position: np.ndarray, quaternion: np.ndarray):
        self.times = [0.0]
        self.positions = [np.asarray(position, dtype=float)]
        self.quaternions = [np.asarray(quaternion, dtype=float)]
        self.csv_phases = ["draw"]
        self.segments: list[dict[str, Any]] = []

    @property
    def t(self) -> float:
        return self.times[-1]

    def hold(self, duration_s: float, phase: str, **metadata: Any) -> tuple[float, float]:
        start = self.t
        for elapsed in sample_grid(duration_s)[1:]:
            self.times.append(round(start + float(elapsed), 9))
            self.positions.append(self.positions[-1].copy())
            self.quaternions.append(self.quaternions[-1].copy())
            self.csv_phases.append("hold")
        self.segments.append({"phase": phase, "start_time_s": start, "end_time_s": self.t, **metadata})
        return start, self.t

    def move(self, position: np.ndarray, quaternion: np.ndarray, duration_s: float, phase: str, **metadata: Any) -> tuple[float, float]:
        start = self.t
        p0, q0 = self.positions[-1].copy(), self.quaternions[-1].copy()
        p1, q1 = np.asarray(position, dtype=float), np.asarray(quaternion, dtype=float)
        r0, r1 = Rotation.from_quat(q0), Rotation.from_quat(q1)
        delta = (r0.inv() * r1).as_rotvec()
        elapsed = sample_grid(duration_s)
        progress = smootherstep7(elapsed / duration_s)
        for seconds, u in zip(elapsed[1:], progress[1:], strict=True):
            self.times.append(round(start + float(seconds), 9))
            self.positions.append(p0 + float(u) * (p1 - p0))
            self.quaternions.append((r0 * Rotation.from_rotvec(float(u) * delta)).as_quat())
            self.csv_phases.append("draw")
        # CSV hold semantics require bit-identical endpoint coordinates.
        self.positions[-1], self.quaternions[-1] = p1.copy(), q1.copy()
        distance = float(np.linalg.norm(p1 - p0))
        angle = float(np.linalg.norm(delta))
        omega = angle * S7_MAX_D1 / duration_s
        alpha = angle * S7_MAX_D2 / duration_s**2
        angular_jerk = angle * S7_MAX_D3 / duration_s**3
        self.segments.append({"phase": phase, "start_time_s": start, "end_time_s": self.t, "parameterization": "seventh_degree_rest_to_rest_zero_endpoint_velocity_acceleration_jerk", "continuous_analytic_bounds": {
            "method": "exact scalar septic derivative extrema and rigid-tool norm inequalities",
            "center_max_speed_m_s": distance * S7_MAX_D1 / duration_s,
            "center_max_acceleration_m_s2": distance * S7_MAX_D2 / duration_s**2,
            "center_max_jerk_m_s3": distance * S7_MAX_D3 / duration_s**3,
            "flange_max_angular_speed_deg_s": math.degrees(omega),
            "flange_max_angular_acceleration_deg_s2": math.degrees(alpha),
            "flange_max_angular_jerk_deg_s3": math.degrees(angular_jerk),
            "flange_linear_speed_upper_bound_m_s": distance * S7_MAX_D1 / duration_s + TOOL_LENGTH_M * omega,
            "flange_linear_acceleration_upper_bound_m_s2": distance * S7_MAX_D2 / duration_s**2 + TOOL_LENGTH_M * (alpha + omega**2),
            "flange_linear_jerk_upper_bound_m_s3": distance * S7_MAX_D3 / duration_s**3 + TOOL_LENGTH_M * (angular_jerk + 3 * omega * alpha + omega**3),
            "endpoint_center_velocity_acceleration_jerk_zero": True,
            "endpoint_angular_velocity_acceleration_jerk_zero": True,
        }, **metadata})
        return start, self.t

    def arrays(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        return np.asarray(self.times), np.asarray(self.positions), np.asarray(self.quaternions)


def _relative_paths(value: dict[str, Any], root: Path) -> dict[str, Any]:
    result = dict(value)
    for name in ("source_file", "trajectory_path"):
        if name in result:
            path = Path(str(result[name]))
            try:
                result[name] = str(path.relative_to(root))
            except ValueError:
                pass
    return result


def _validate_frozen_geometry(root: Path) -> None:
    environment = root / "environments" / "mla_short"
    setup = yaml.safe_load((environment / "setup.yaml").read_text())
    zones = yaml.safe_load((environment / "zones.yaml").read_text())
    environment_data = yaml.safe_load((environment / "environment.yaml").read_text())
    rod = setup["tool"]["rod"]
    protected = zones["curobo_scene"]["cuboid"]["sensor_board_keepout"]
    boundary = zones["analysis_zones"]["sensor_keepout_clearance_boundary"]
    required = (
        (float(rod["length_m"]), TOOL_LENGTH_M),
        (float(rod["radius_m"]), TOOL_RADIUS_M),
        (float(protected["pose"][2]) - float(protected["dims"][2]) / 2, SENSOR_KEEP_OUT_FACE_Z_M),
        (float(boundary["normal_xyz"][2]) * float(boundary["offset_m"]), ANALYTIC_TIP_BOUNDARY_Z_M),
        (float(zones["contact_policy"]["minimum_non_tip_clearance_m"]), NON_TIP_CLEARANCE_M),
        (float(environment_data["calibration"]["sensor_geometry"]["actual_sensor_plane_local_z_m"]), 0.0),
    )
    if any(not math.isclose(actual, expected, abs_tol=1e-12, rel_tol=0) for actual, expected in required):
        raise ValueError("calibration design requires the unchanged frozen mla_short geometry")
    if list(rod["axis_in_parent_xyz"]) != [0.0, 0.0, 1.0] or protected["frame_id"] != "plane1":
        raise ValueError("calibration design requires +Z rod axis and plane1 board geometry")


def _write(timeline: _Timeline, root: Path, trajectory_id: str) -> dict[str, Any]:
    arrays = timeline.arrays()
    metadata = write_authority(root / "environments" / "mla_short" / "trajectories" / f"{trajectory_id}.csv", *arrays, timeline.csv_phases)
    return {"trajectory_id": trajectory_id, **_relative_paths(metadata, root), "phases": timeline.segments}


def _calibration(root: Path, site: dict[str, Any], height_m: float) -> dict[str, Any]:
    targets = orientations()
    p = np.asarray([*site["xy_m"], -height_m])
    timeline = _Timeline(p, np.asarray(targets[0]["quaternion_xyzw"]))
    windows = []
    for i, target in enumerate(targets):
        q = np.asarray(target["quaternion_xyzw"])
        if i:
            timeline.move(p, q, ROTATION_DURATION_S, "orientation_transition", target_orientation_id=target["orientation_id"])
        arrival = timeline.t
        timeline.hold(SETTLE_S, "settle", orientation_id=target["orientation_id"])
        capture_start, capture_end = timeline.hold(CAPTURE_S, "capture", orientation_id=target["orientation_id"])
        windows.append({"window_id": f"{site['site_id']}_h{round(height_m * 1000):03d}_{target['orientation_id']}", "site_id": site["site_id"], "split_group_id": site["split_group_id"], "data_role": site["data_role"], "nominal_height_m": height_m, "nominal_rod_tip_position_plane1_m": p.tolist(), **target, "arrival_time_s": arrival, "settle_end_time_s": capture_start, "capture_start_time_s": capture_start, "capture_end_time_s": capture_end})
    timeline.move(p, np.asarray(targets[0]["quaternion_xyzw"]), ROTATION_DURATION_S, "return_to_normal_orientation")
    timeline.hold(SETTLE_S, "final_settle")
    times, positions, quaternions = timeline.arrays()
    precheck = low_height_precheck(height_m, quaternions)
    if not precheck["accepted"]:
        raise ValueError(f"refusing to author infeasible low calibration height: {precheck}")
    result = _write(timeline, root, f"wm_cal_{site['site_id']}_h{round(height_m * 1000):03d}_v1")
    result.update({"category": "calibration", "environment_id": "mla_short", "protocol_version": PROTOCOL, **site, "nominal_height_m": height_m, "nominal_center_type": "rod_tip_before_accepted_magnet_calibration", "frame_id": "plane1", "capture_windows": windows, "capture_window_count": len(windows), "sensor_keepout_uncertainty_precheck": precheck, "diagnostics": motion_diagnostics(times, positions, quaternions), "real_execution_status": "not_run_simulation_only"})
    return result


def _synchronization(root: Path, axis: str, peak_speed_m_s: float) -> dict[str, Any]:
    # With s7'(1/2)=2.1875, these durations realize exact 5/10 mm/s
    # peaks on one unchanged 40 mm path and both lie on the 50 ms grid.
    distance_m = 0.040
    duration_s = 2.1875 * distance_m / peak_speed_m_s
    center = np.asarray([0.0, 0.0, -0.100])
    delta = np.eye(3)["xyz".index(axis)] * distance_m
    p0, p1 = center - delta / 2.0, center + delta / 2.0
    q = np.array([0.0, 0.0, 0.0, 1.0])
    timeline = _Timeline(p0, q)
    timeline.hold(SETTLE_S, "settle_before_forward")
    timeline.hold(CAPTURE_S, "stationary_anchor_before_forward")
    forward_start, forward_end = timeline.move(p1, q, duration_s, "synchronization_forward", axis=axis, direction=1, specified_peak_speed_m_s=peak_speed_m_s)
    timeline.hold(SETTLE_S, "settle_after_forward")
    timeline.hold(CAPTURE_S, "stationary_anchor_before_reverse")
    reverse_start, reverse_end = timeline.move(p0, q, duration_s, "synchronization_reverse", axis=axis, direction=-1, specified_peak_speed_m_s=peak_speed_m_s)
    timeline.hold(SETTLE_S, "settle_after_reverse")
    timeline.hold(CAPTURE_S, "stationary_anchor_after_reverse")
    times, positions, quaternions = timeline.arrays()
    result = _write(timeline, root, f"wm_sync_{axis}_peak{round(peak_speed_m_s * 1000):03d}mmps_h100_v1")
    result.update({"category": "synchronization", "environment_id": "mla_short", "protocol_version": PROTOCOL, "axis": axis, "specified_peak_speed_m_s": peak_speed_m_s, "speed_semantics": "planned_rod_tip_peak_not_constant_cruise", "path_length_one_way_m": distance_m, "nominal_height_range_m": [float(-positions[:, 2].max()), float(-positions[:, 2].min())], "frame_id": "plane1", "nominal_center_type": "rod_tip_before_accepted_magnet_calibration", "orientation_policy": "identity_quaternion_fixed", "data_role": "independent_low_height_synchronization", "dynamic_windows": [{"direction": "forward", "start_time_s": forward_start, "end_time_s": forward_end}, {"direction": "reverse", "start_time_s": reverse_start, "end_time_s": reverse_end}], "diagnostics": motion_diagnostics(times, positions, quaternions), "time_alignment_policy": "retain_device_controller_host_times; estimate_time_offset_only_from_these_low_height_motions", "synchronization_status": "not_measured_offline_plan_only", "real_execution_status": "not_run_simulation_only"})
    return result


def generate(root: Path, config: dict[str, Any] | None = None) -> list[dict[str, Any]]:
    """Create 27 authority calibration files and six timing files immutably."""
    root = Path(root).expanduser().resolve()
    _validate_frozen_geometry(root)
    config = dict(config or {})
    if config.get("height_levels_m") is not None and tuple(config["height_levels_m"]) != HEIGHTS_M:
        raise ValueError("new height levels require a new upstream design/version")
    planning = root / "planning"
    rejected = [low_height_precheck(h) for h in (0.060, 0.070)]
    if any(row["accepted"] for row in rejected):
        raise AssertionError("expected initial nominal 60/70 mm proposals to fail conservative envelope")
    calibrations = [_calibration(root, site, height) for height in HEIGHTS_M for site in sites()]
    synchronization = [_synchronization(root, axis, speed) for axis in "xyz" for speed in (0.005, 0.010)]
    windows = [{"trajectory_id": record["trajectory_id"], **window} for record in calibrations for window in record["capture_windows"]]
    schedule = []
    for repeat in (1, 2):
        rng = np.random.default_rng(20261016 + repeat)
        for height in sorted(HEIGHTS_M, reverse=True):
            records = [record for record in calibrations if math.isclose(record["nominal_height_m"], height)]
            for index in rng.permutation(len(records)):
                record = records[int(index)]
                schedule.append({"schedule_index": len(schedule) + 1, "repeat_index": repeat, "execution_id": f"{record['trajectory_id']}__repeat{repeat:02d}", "trajectory_id": record["trajectory_id"], "source_file": record["source_file"], "source_sha256": record["source_sha256"], "site_id": record["site_id"], "split_group_id": record["split_group_id"], "data_role": record["data_role"], "nominal_height_m": height, "capture_window_count": 5})
    report = {
        "protocol_version": PROTOCOL,
        "environment_id": "mla_short",
        "scope": "offline_authoring_and_simulation_only_no_sensor_background_or_robot_execution",
        "nominal_center_type": "rod_tip_before_accepted_magnet_calibration",
        "coordinate_convention": "plane1 metres; negative Z is physically above sensor sensitive plane; h=-z; quaternion xyzw",
        "array_frame_mapping_status": "pending_accepted_S_frame_mapping_do_not_equate_visual_board_center_with_hospital_array_origin",
        "provisional_error_model": {"flange_position_ball_radius_m": POSITION_UNCERTAINTY_M, "flange_orientation_cone_half_angle_deg": ORIENTATION_UNCERTAINTY_DEG, "measured_or_accepted_calibration": False},
        "tool": {"rod_length_m": TOOL_LENGTH_M, "rod_radius_m": TOOL_RADIUS_M, "axis_in_tool0_xyz": [0.0, 0.0, 1.0]},
        "accepted_nominal_height_levels_m": list(HEIGHTS_M),
        "rejected_upstream_candidates": rejected,
        "height_redesign_reason": "60 mm intersects unchanged 65 mm analytical boundary; 70 mm fails provisional full-rod sphere and flange-error envelope; new 75/80/100 mm authority files designed upstream",
        "xy_site_count": 9,
        "xy_levels_m": [-0.1, 0.0, 0.1],
        "orientation_count_per_site_height": 5,
        "unique_capture_windows": 135,
        "repeat_count": 2,
        "scheduled_capture_windows": 270,
        "fit_site_count": 7,
        "holdout_site_count": 2,
        "holdout_site_fraction": 2.0 / 9.0,
        "holdout_policy": "two_whole_xy_sites_all_heights_and_orientations_preserved_across_repeats",
        "settle_duration_s": SETTLE_S,
        "capture_duration_s": CAPTURE_S,
        "rotation_transition_duration_s": ROTATION_DURATION_S,
        "synchronization_files": 6,
        "synchronization_axes": ["x", "y", "z"],
        "synchronization_peak_speeds_m_s": [0.005, 0.010],
        "waypoint_time_contract": "every CSV row and time_s authoritative; runtime scaling, retiming, clipping and skipping forbidden",
        "physical_and_magnetic_acceptance": "not_performed; LM anchor stability saturation and accepted fixed calibration remain later work",
    }
    write_json_immutable(planning / "calibration_report.json", report)
    write_json_immutable(planning / "calibration_capture_windows.json", windows)
    write_json_immutable(planning / "calibration_repeat_schedule.json", schedule)
    write_json_immutable(planning / "calibration_manifest.json", calibrations + synchronization)
    return calibrations + synchronization


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    outputs = generate(args.root)
    print(f"Created/verified {len(outputs)} immutable calibration/synchronization authorities in {args.root}")


if __name__ == "__main__":
    main()
