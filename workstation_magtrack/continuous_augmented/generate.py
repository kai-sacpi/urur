#!/usr/bin/env python3
"""Author 12 diverse *open*, one-pass nominal routes as immutable CSVs.

CV-style local stretch, shear and elastic displacement are applied upstream
to adapted open line/S/C/tube centerline prototypes.  A strictly positive
longitudinal derivative prevents loops, reversals and repeated laps.  New
time and multi-axis orientation controls are independently seeded.  No
runtime trajectory adjustment, robot, ROS or sensor use occurs here.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import platform
import sys
from typing import Any

import numpy as np
import scipy
import yaml

if __package__:
    from . import core
else:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import core

ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parent
REPO = WORKSPACE.parent
sys.path.insert(0, str(WORKSPACE))
from generation.common import sample_grid, write_authority, write_json_immutable, _write_immutable

DEFAULT_BATCH = "ca_open_elastic_preview_20261016_v1"
MASTER_SEED = 2026101607
FAMILIES = ("open_line", "gentle_s", "strong_s", "c_curve", "slanted_s", "tube_bend")
LIMITS = {"position_acceleration_m_s2": .05, "position_jerk_m_s3": .25,
          "angular_acceleration_deg_s2": 30., "angular_jerk_deg_s3": 150.,
          "xy_abs_m": .130, "tilt_deg": 25., "height_lower_m": .070,
          "height_upper_m": .200, "minimum_collision_clearance_m": .005}


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _seed_streams(seed: int) -> dict[str, int]:
    spawned = np.random.SeedSequence(seed).spawn(4)
    return {name: int(child.generate_state(1, dtype=np.uint64)[0])
            for name, child in zip(("space", "speed", "orientation", "duration"), spawned, strict=True)}


def _prototype(family: str) -> np.ndarray:
    """Degree-7 open centerline prototypes, intentionally with no tube laps."""
    presets = {
        "open_line": [0., 0., 0., 0., 0., 0., 0., 0.],
        "gentle_s": [0., -.014, -.032, -.020, .020, .032, .014, 0.],
        "strong_s": [0., -.027, -.052, -.032, .032, .052, .027, 0.],
        "c_curve": [0., .027, .048, .058, .058, .048, .027, 0.],
        "slanted_s": [-.020, -.033, -.046, -.018, .018, .046, .033, .020],
        "tube_bend": [-.025, -.035, -.031, -.002, .027, .040, .026, .010],
    }
    return np.asarray(presets[family], float)


def design_metadata(index: int, full_height: bool, master_seed: int = MASTER_SEED) -> dict[str, Any]:
    seed = master_seed + (1000 if full_height else 0) + index
    streams = _seed_streams(seed)
    rng = np.random.default_rng(streams["space"])
    family = FAMILIES[index]
    degree = 7
    grid = np.linspace(0., 1., degree+1)
    length = float(rng.uniform(.145, .177))
    rotation = float(rng.uniform(-math.pi, math.pi))
    direction = np.array([math.cos(rotation), math.sin(rotation), 0.])
    normal = np.array([-math.sin(rotation), math.cos(rotation), 0.])
    center = np.r_[rng.uniform(-.009, .009, 2), 0.]
    prototype_lateral = _prototype(family)
    baseline_longitudinal = length * (grid-.5)
    baseline = center + baseline_longitudinal[:, None]*direction + prototype_lateral[:, None]*normal
    # Positive local increments perform a random local longitudinal stretch.
    stretch_weights = rng.uniform(.55, 1.6, degree)
    stretched = length * np.r_[0., np.cumsum(stretch_weights)/stretch_weights.sum()] - length/2.
    shear = float(rng.uniform(-.016, .016))
    # A degree-3 smooth displacement field, sampled into degree-7 controls.
    # These are new geometric design controls, not modifications to an old CSV.
    elastic_controls = rng.uniform(-.023, .023, 4)
    elastic = core.bernstein(elastic_controls, grid)
    lateral_scale = float(rng.uniform(.75, 1.10))
    lateral = prototype_lateral*lateral_scale + shear*(2*grid-1) + elastic
    selected = center + stretched[:, None]*direction + lateral[:, None]*normal
    if full_height:
        depth_weights = rng.uniform(.45, 1.8, degree)
        heights = .070 + .130*np.r_[0., np.cumsum(depth_weights)/depth_weights.sum()]
        reverse_depth = bool(index % 2)
        if reverse_depth:
            heights = heights[::-1]
        baseline[:, 2] = -np.linspace(heights[0], heights[-1], degree+1)
        selected[:, 2] = -heights
        profile = {"mode": "full_height_one_way", "lower_m": .070, "upper_m": .200,
                   "direction": "descending_height" if reverse_depth else "ascending_height",
                   "depth_positive_stretch_weights": depth_weights.tolist()}
        group = "full_height_one_way"
        height_tag = "h070_200"
    else:
        height = (.070, .090, .120, .150, .180, .200)[index]
        baseline[:, 2] = selected[:, 2] = -height
        profile = {"mode": "fixed", "lower_m": height, "upper_m": height, "direction": "constant"}
        group = "fixed_height"
        height_tag = f"h{round(height*1000):03d}"
    if np.max(np.abs(selected[:, :2])) > .129:
        raise ValueError("upstream sampled control hull exceeds XY design box; choose a new batch/seed, never clip")
    speed_rng = np.random.default_rng(streams["speed"])
    density = np.exp(speed_rng.uniform(-1.0, 1.0, 5))
    # Independent, deliberately low-frequency multi-axis orientation geometry.
    orient_rng = np.random.default_rng(streams["orientation"])
    fixed = not full_height and index in (0, 5)
    yaw0 = float(orient_rng.uniform(-math.pi, math.pi))
    roll = np.deg2rad(orient_rng.uniform(-10., 10., 6))
    pitch = np.deg2rad(orient_rng.uniform(-12., 12., 6))
    yaw_offsets = np.deg2rad(orient_rng.uniform(-48., 48., 6))
    ec = np.c_[roll, pitch, yaw0+yaw_offsets]
    if fixed:
        ec[:] = ec[0]
    # A new upstream seed is a distinct authority, never the same runtime ID.
    trajectory_id = f"cae1_{group}_{index+1:02d}_{family}_{height_tag}_s{seed}"
    return {
        "metadata_schema_version": 1, "generator_version": "open_elastic_v1",
        "trajectory_id": trajectory_id, "group_id": group, "family_id": family,
        "seed": seed, "master_seed": master_seed, "seed_streams": streams,
        "environment_id": "mla_short", "frame_id": "plane1", "position_units": "m",
        "quaternion_order": "xyzw", "height_definition": "height_m=-plane1_z_m",
        "height_profile": profile,
        "curve": {"degree": degree, "position_control_points_m": selected.tolist(),
                  "prototype_control_points_m": baseline.tolist(),
                  "baseline_control_points_m": baseline.tolist(),
                  "deformation_control_points_m": (selected-baseline).tolist(),
                  "longitudinal_axis_xyz": direction.tolist(),
                  "augmentation": {"style": "CV_style_affine_shear_local_stretch_and_smooth_elastic_displacement",
                                   "rotation_rad": rotation, "center_m": center.tolist(), "length_m": length,
                                   "local_positive_stretch_weights": stretch_weights.tolist(),
                                   "normal_elastic_field_degree": 3,
                                   "normal_elastic_control_points_m": elastic_controls.tolist(),
                                   "shear_endpoint_amplitude_m": shear, "lateral_scale": lateral_scale,
                                   "all_changes_occur_before_new_authority_write": True}},
        "source_adaptation": {
            "reference": "src/hx_waypoint_tools/hx_waypoint_tools/trajectory_generator.py",
            "open_route_reference_functions": ["_esophageal_startup_centerline_offsets", "new_board_esophageal_startup_centerline_position"],
            "adaptation": "new degree-7 open centerline prototype inspired by existing line/S/C/tube centerlines; deliberately omits old repeat_count and local periodic tube harmonics; not an old CSV or exact copy",
            "source_prototype_is_open": True, "requested_laps": 0},
        "time_warp": {"density_control_points_positive": density.tolist(),
                      "formula": "septic_smootherstep(positive_Bernstein_density_CDF(s))",
                      "density_degree": len(density)-1, "backwards_time_allowed": False,
                      "velocity_acceleration_jerk_zero_at_draw_endpoints": True},
        "orientation": {"mode": "fixed" if fixed else "smooth_multi_axis",
                        "euler_xyz_control_points_rad": ec.tolist(),
                        "composition": "Rz(yaw) Ry(pitch) Rx(roll)",
                        "parameter": "same_positive_warped_progress_u_as_position",
                        "tool_axis": "local +Z toward plane1 +Z; rod extends from tip along -axis",
                        "magnet_axis_measured": False},
        "draw_duration_s": 1., "hold_start_s": .5, "hold_end_s": .5,
        "source_sample_period_s": .05,
        "hard_limits": LIMITS,
        "safety_status": {"simulation_only": True, "real_execution_authorized": False,
                          "whole_robot_collision_ik_joint_dynamics": "not_run",
                          "source_authority_may_be_modified_by_runtime": False,
                          "provisional_position_error_envelope_m": 0.,
                          "provisional_orientation_error_envelope_deg": 0.,
                          "seven_cm_user_authorized_nominal_geometry_only": True,
                          "hard_collision_clearance_m": .005,
                          "rod_length_m": .190, "rod_radius_m": .0025},
    }


def finalize_duration(meta: dict[str, Any], samples: int = 32769) -> dict[str, Any]:
    """Select a new upstream time law exactly once, before any CSV exists."""
    result = copy.deepcopy(meta)
    normalized = core.derivative_certificate(result, samples=samples)
    pos = normalized["position_derivative_bounds_m_s_orders_1_3"]
    ang = normalized["angular_derivative_bounds_deg_s_orders_1_3"]
    rng = np.random.default_rng(result["seed_streams"]["duration"])
    target_speed = float(rng.uniform(.023, .040))
    selected = max(float(rng.uniform(20., 30.)), pos[0]/target_speed,
                   math.sqrt(pos[1]/(.90*LIMITS["position_acceleration_m_s2"])),
                   (pos[2]/(.90*LIMITS["position_jerk_m_s3"]))**(1/3),
                   math.sqrt(ang[1]/(.90*LIMITS["angular_acceleration_deg_s2"])),
                   (ang[2]/(.90*LIMITS["angular_jerk_deg_s3"]))**(1/3))
    selected = math.ceil(selected/.05-1e-10)*.05
    result["draw_duration_s"] = round(selected, 9)
    result["duration_selection"] = {
        "upstream_before_any_csv_write": True, "target_peak_speed_m_s": target_speed,
        "selected_draw_duration_s": result["draw_duration_s"],
        "cycle_count_increase_or_repeated_laps_used": False,
        "dynamics_design_fraction_of_hard_limit": .90,
        "note": "independent random requested pace plus continuous safety certificate; no V/W discrete labels claimed"}
    result["certificate"] = core.derivative_certificate(result, samples=samples)
    result["path_certificate"] = core.path_certificate(result)
    c = result["certificate"]
    if (c["position_derivative_bounds_m_s_orders_1_3"][1] > LIMITS["position_acceleration_m_s2"] or
        c["position_derivative_bounds_m_s_orders_1_3"][2] > LIMITS["position_jerk_m_s3"] or
        c["angular_derivative_bounds_deg_s_orders_1_3"][1] > LIMITS["angular_acceleration_deg_s2"] or
        c["angular_derivative_bounds_deg_s_orders_1_3"][2] > LIMITS["angular_jerk_deg_s3"] or
        result["path_certificate"]["tilt_bound_deg"] > LIMITS["tilt_deg"] or
        not result["path_certificate"]["strictly_monotone_projection"]):
        raise ValueError("new source design failed unchanged motion or open-path contract")
    return result


def sampled_authority(meta: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[str]]:
    duration = meta["draw_duration_s"]
    start, end = meta["hold_start_s"], meta["hold_end_s"]
    times = sample_grid(duration+start+end, .05)
    s = np.clip((times-start)/duration, 0., 1.)
    values = core.evaluate(meta, s)
    p, q = values["position"], values["quaternion"]
    # Byte-exact repeated poses in strict hold intervals, not approximate motion.
    p[times <= start], q[times <= start] = p[0], q[0]
    p[times >= start+duration], q[times >= start+duration] = p[-1], q[-1]
    phases = ["hold" if t <= start or t > start+duration+1e-10 else "draw" for t in times]
    phases[0] = "draw"  # Existing parser contract: first authority row is draw.
    return times, p, q, phases


def _freeze_provenance(batch: Path) -> dict[str, Any]:
    refs = [(ROOT/"core.py", "generator_sources/core.py"),
            (ROOT/"generate.py", "generator_sources/generate.py"),
            (WORKSPACE/"generation/common.py", "generator_sources/common.py"),
            (REPO/"src/hx_waypoint_tools/hx_waypoint_tools/trajectory_generator.py", "reference_sources/trajectory_generator.py"),
            (REPO/"scripts/generate_new_board_esophageal_up50_trajectories.py", "reference_sources/generate_new_board_esophageal_up50_trajectories.py")]
    refs += [(WORKSPACE/f"environments/mla_short/{name}.yaml", f"environment/{name}.yaml")
             for name in ("setup", "zones", "planner", "environment")]
    refs.append((WORKSPACE/"environments/mla_short/robot/assets/ur5.urdf", "environment/ur5.urdf"))
    entries = []
    for source, name in refs:
        payload = source.read_bytes()
        target = batch/"provenance"/name
        _write_immutable(target, payload)
        entries.append({"origin_path": str(source), "frozen_file": str(target.relative_to(batch)),
                        "sha256": digest(payload)})
    result = {"sources": entries, "python": platform.python_version(),
              "numpy": np.__version__, "scipy": scipy.__version__}
    write_json_immutable(batch/"provenance/source_inventory.json", result)
    return result


def _check_environment() -> None:
    env = WORKSPACE/"environments/mla_short"
    setup = yaml.safe_load((env/"setup.yaml").read_text())
    zones = yaml.safe_load((env/"zones.yaml").read_text())
    rod = setup["tool"]["rod"]
    board = zones["curobo_scene"]["cuboid"]["sensor_board_keepout"]
    if (abs(rod["length_m"]-.190) > 1e-12 or abs(rod["radius_m"]-.0025) > 1e-12 or
        abs(board["pose"][2]-board["dims"][2]/2+.060) > 1e-12 or
        abs(zones["contact_policy"]["minimum_non_tip_clearance_m"]-.005) > 1e-12):
        raise ValueError("unchanged mla_short geometry does not match rod190/board/5mm contract")


def generate(batch_directory: Path | None = None, master_seed: int = MASTER_SEED) -> Path:
    _check_environment()
    batch = Path(batch_directory) if batch_directory else ROOT/"batches"/DEFAULT_BATCH
    provenance = _freeze_provenance(batch)
    records = []
    for full_height in (False, True):
        for index in range(6):
            meta = finalize_duration(design_metadata(index, full_height, master_seed))
            meta["provenance_inventory_file"] = "provenance/source_inventory.json"
            meta["provenance_inventory_sha256"] = digest((batch/"provenance/source_inventory.json").read_bytes())
            path = batch/"csv"/f"{meta['trajectory_id']}.csv"
            authority = write_authority(path, *sampled_authority(meta))
            meta["authority"] = {"csv_file": str(path.relative_to(batch)), "sha256": authority["source_sha256"],
                                 "source_pose_count": authority["source_pose_count"],
                                 "duration_s": authority["duration_s"]}
            meta["source_grid_diagnostics"] = authority["diagnostics"]
            metadata_file = batch/"metadata"/f"{meta['trajectory_id']}.json"
            write_json_immutable(metadata_file, meta)
            records.append({"trajectory_id": meta["trajectory_id"], "csv_file": str(path.relative_to(batch)),
                            "metadata_file": str(metadata_file.relative_to(batch)),
                            "sha256": authority["source_sha256"], "metadata_sha256": digest(metadata_file.read_bytes()),
                            "duration_s": authority["duration_s"], "draw_duration_s": meta["draw_duration_s"],
                            "group_id": meta["group_id"], "family_id": meta["family_id"], "seed": meta["seed"],
                            "height_profile": meta["height_profile"], "orientation_mode": meta["orientation"]["mode"],
                            "robot_simulation": "not_run", "runtime_ready": False})
    manifest = {"schema_version": 1, "batch_id": batch.name, "generator_version": "open_elastic_v1",
                "master_seed": master_seed, "trajectory_count": len(records), "records": records,
                "environment_id": "mla_short", "simulation_only": True, "real_execution_authorized": False,
                "authority_contract": "every written CSV row/pose/time_s is immutable; runtime scaling, retiming, clipping, skipping and automatic slowing are forbidden",
                "source_contract": {"closed_pattern_families_allowed": False, "repeated_laps_allowed": False,
                                    "fixed_height_count": 6, "full_height_one_way_count": 6,
                                    "provisional_3mm_2deg_envelope_removed_by_user": True,
                                    "unchanged_hard_limits": LIMITS},
                "total_authority_duration_s": sum(r["duration_s"] for r in records),
                "provenance": provenance,
                "full_robot_validation_status": "not_run; preview authorities are not executable robot recordings"}
    write_json_immutable(batch/"manifest.json", manifest)
    return batch/"manifest.json"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="new immutable batch directory")
    parser.add_argument("--seed", type=int, default=MASTER_SEED)
    args = parser.parse_args()
    manifest = generate(args.output, args.seed)
    print(manifest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
