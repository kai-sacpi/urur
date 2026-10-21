#!/usr/bin/env python3
"""Author isolated, immutable layered and full-height nominal authorities."""
from __future__ import annotations
import argparse
import copy
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import platform
import sys
import xml.etree.ElementTree as ET
import numpy as np
import scipy
import yaml
from src import patterns, trajectory, clearance

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent.parent
FIELDS = ["time_s", "frame_id", "tip_x_m", "tip_y_m", "tip_z_m",
          "tip_qx", "tip_qy", "tip_qz", "tip_qw", "phase"]


def digest(data):
    return hashlib.sha256(data).hexdigest()


def json_bytes(value):
    return (json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n").encode()


def immutable(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        if path.is_symlink() or path.read_bytes() != payload:
            raise FileExistsError(f"refusing to overwrite differing authority: {path}")
        return
    with path.open("xb") as stream:
        stream.write(payload)


def read_geometry():
    env = ROOT.parent / "environments/mla_short"
    result = {name: yaml.safe_load((env/f"{name}.yaml").read_text())
              for name in ("setup", "zones", "planner")}
    rod = result["setup"]["tool"]["rod"]
    board = result["zones"]["curobo_scene"]["cuboid"]["sensor_board_keepout"]
    boundary = result["zones"]["analysis_zones"]["sensor_keepout_clearance_boundary"]
    policy = result["zones"]["contact_policy"]
    expected = [(rod["length_m"], .190), (rod["radius_m"], .0025),
                (board["pose"][2]-board["dims"][2]/2, -.060),
                (boundary["offset_m"], .065), (policy["minimum_non_tip_clearance_m"], .005)]
    if any(abs(a-b) > 1e-12 for a, b in expected):
        raise ValueError("nominal geometry differs from the unchanged rod190/board contract")
    urdf = env/"robot/assets/ur5.urdf"
    tree = ET.fromstring(urdf.read_bytes())
    joint = tree.find("./joint[@name='base_link-base_fixed_joint']")
    if joint is None or joint.attrib["type"] != "fixed" or joint.find("parent").attrib["link"] != "base_link" or joint.find("child").attrib["link"] != "base":
        raise ValueError("cannot resolve fixed UR base_link -> controller base from frozen URDF")
    origin = joint.find("origin")
    result["base_link_to_base"] = {
        "parent_frame":"base_link", "child_frame":"base",
        "translation_m":[float(x) for x in origin.attrib.get("xyz", "0 0 0").split()],
        "rpy_rad":[float(x) for x in origin.attrib.get("rpy", "0 0 0").split()],
        "urdf_sha256":digest(urdf.read_bytes()), "urdf_joint":joint.attrib["name"]}
    return result


def provenance(batch, config):
    refs = [
        (REPO/"src/hx_waypoint_tools/hx_waypoint_tools/trajectory_generator.py", "trajectory_generator.py"),
        (Path("/Users/huaxin/Codes/paper_test/BAFE-code-trajectory-perf/experiments/simulation_mechanism/motions.py"), "bafe_motions.py"),
        (REPO/"scripts/generate_new_board_esophageal_up50_trajectories.py", "generate_new_board_esophageal_up50_trajectories.py"),
        (ROOT.parent/"README.md", "workstation_README.md"),
        (ROOT.parent/"docs/calibration_and_release_gate.md", "calibration_and_release_gate.md"),
        (ROOT.parent/"inputs/collection_task_v2_20261016.txt", "collection_task_v2_20261016.txt"),
        (ROOT.parent/"inputs/real_trajectory_collection_plan_20261013.md", "real_trajectory_collection_plan_20261013.md"),
        (ROOT.parent/"generation/calibration.py", "old_calibration_precheck.py"),
    ]
    refs += [(ROOT.parent/f"environments/mla_short/{n}.yaml", f"mla_short_{n}.yaml")
             for n in ("setup", "zones", "planner", "environment")]
    refs += [(ROOT.parent/"environments/mla_short/robot/assets/ur5.urdf", "mla_short_ur5.urdf")]
    records = []
    for original, name in refs:
        target = batch/"provenance/reference_sources"/name
        # Recovery/reproduction may use the already frozen copy of external BAFE.
        fallback = ROOT/"batches"/config["batch_id"]/"provenance/reference_sources"/name
        payload = original.read_bytes() if original.exists() else fallback.read_bytes()
        immutable(target, payload)
        records.append({"origin_path": str(original), "frozen_file": str(target.relative_to(batch)),
                        "sha256": digest(payload)})
    generation_files = [ROOT/"generate.py", ROOT/"config/contract_v1.json",
                        *sorted((ROOT/"src").glob("*.py")),
                        *sorted((ROOT/"validation").glob("*.py")),
                        ROOT/"visualization/plot_batch.py", ROOT/"design_notes/geometry_sources.md"]
    for original in generation_files:
        target = batch/"provenance/generator_sources"/original.relative_to(ROOT)
        payload = original.read_bytes()
        immutable(target, payload)
        records.append({"origin_path": str(original.relative_to(ROOT)),
                        "frozen_file": str(target.relative_to(batch)), "sha256": digest(payload)})
    immutable(batch/"provenance/source_inventory.json", json_bytes(records))
    return records


def new_geometry(family, seed, variant, geometry_id, height_profile):
    rng = np.random.default_rng(seed)
    parameters = patterns.sample_parameters(family, seed, variant)
    rotation = float(rng.uniform(-math.pi, math.pi))
    center = rng.uniform(-.022, .022, 2)
    scale = rng.uniform(.045, .070, 2) if variant % 2 == 0 else rng.uniform(.080, .115, 2)
    # Select the entire geometry upstream using an analytic envelope. This
    # never clips points and never modifies a previously authored CSV.
    r = np.abs([[math.cos(rotation), -math.sin(rotation)],
                [math.sin(rotation), math.cos(rotation)]])
    b0 = np.asarray(patterns.derivative_bounds(family, parameters, max_order=0))[0]
    selected_factor = min(1., float(np.min((.129-np.abs(center))/(r@(scale*b0)))))
    scale *= selected_factor
    axis_angle = float(rng.uniform(-math.pi, math.pi))
    axis = np.array([math.cos(axis_angle), math.sin(axis_angle), 0.])
    return {"metadata_schema_version": 1, "generator_version": "layers_full_v1",
            "protocol_version": "real_collection_v2_20261016_plus_explicit_h070_200_layers",
            "family_id": family, "geometry_id": geometry_id,
            "split_group_id": f"family_{family}", "data_role": "offline_design_development",
            "seed": seed, "pattern_parameters": parameters,
            "height_profile": height_profile,
            "min_height_m": height_profile["lower_m"], "max_height_m": height_profile["upper_m"],
            "direction": "forward",
            "xy": {"center_m": center.tolist(), "scale_m": scale.tolist(),
                   "rotation_rad": rotation, "parameter_offset": float(rng.uniform(0., 1.)),
                   "cycles": 1, "upstream_analytic_fit_factor": selected_factor,
                   "analytic_coordinate_abs_bound_m": (np.abs(center)+r@(scale*b0)).tolist(),
                   "sampling_policy": "fixed_seed_parameters_then_analytic_whole_curve_envelope_no_clipping"},
            "orientation": {"base_rotvec_rad": (math.radians(8.)*axis).tolist(),
                            "axis_in_base_tool_xyz": axis.tolist(),
                            "phase_rad": float(rng.uniform(-math.pi, math.pi)),
                            "cycles": 0, "amplitude_rad": 0.}}


def select_cycles(base, config, initial=1, design_peak_speed=.05):
    attempts = []
    for cycles in range(initial, 9):
        base["xy"]["cycles"] = cycles
        cert = trajectory.derivative_certificate(base)
        t = math.ceil(cert["normalized_position_derivative_bounds"][1]/design_peak_speed*10-1e-10)/10
        a = cert["normalized_position_derivative_bounds"][2]/t**2
        j = cert["normalized_position_derivative_bounds"][3]/t**3
        attempts.append({"xy_cycles": cycles, "design_peak_speed_m_s": design_peak_speed, "duration_s": t,
                         "acceleration_upper_bound_m_s2": a, "jerk_upper_bound_m_s3": j})
        if a <= config["center_acceleration_limit_m_s2"] and j <= config["center_jerk_limit_m_s3"]:
            base["upstream_cycle_design_attempts"] = attempts
            base["continuous_position_certificate"] = cert
            return
    raise ValueError(f"unresolved source motion limits, no CSV created: {base['geometry_id']}")


def full_speed_geometry_design(base, force_large_envelope=False):
    """Explicit upstream shape selection for the demanding 50 mm/s peak.

    Tight small-radius ripples need a different upstream geometry to retain
    this peak under the Cartesian limits. The original sampled parameters,
    selection recipe and changed values remain in metadata. Layer routes
    retain the full original random parameter ranges at their declared tiers.
    """
    family = base["family_id"]
    if not force_large_envelope and family not in ("torus_knot_2_5", "braided_spiral", "rounded_square_shear"):
        return
    before = {"xy": copy.deepcopy(base["xy"]), "parameters": copy.deepcopy(base["pattern_parameters"])}
    rng = np.random.default_rng(base["seed"]+40000)
    xy = base["xy"]
    xy["rotation_rad"] = float(rng.uniform(-.05, .05))
    xy["center_m"] = rng.uniform(-.003, .003, 2).tolist()
    xy["scale_m"] = rng.uniform(.115, .118, 2).tolist()
    if family == "torus_knot_2_5":
        base["pattern_parameters"]["tube_radius_ratio"] = float(rng.uniform(.23, .25))
    elif family == "braided_spiral":
        base["pattern_parameters"]["braid_weight"] = float(rng.uniform(.035, .040))
    r = np.abs([[math.cos(xy["rotation_rad"]), -math.sin(xy["rotation_rad"])],
                [math.sin(xy["rotation_rad"]), math.cos(xy["rotation_rad"])]])
    xy["analytic_coordinate_abs_bound_m"] = (np.abs(xy["center_m"])+r@xy["scale_m"]).tolist()
    base["upstream_geometry_design"] = {
        "reason":"retain 50 mm/s peak and complete route under unchanged acceleration/jerk limits",
        "before_authoring_any_CSV":True,"initial_seeded_design":before,
        "selection_seed":base["seed"]+40000,
        "rule":"larger random XY envelope; torus ratio .23-.25, braid radial weight .035-.040; no point clipping or runtime change"}


def timed_variant(base, config, group, v, mode, direction="forward", w="W1"):
    meta = copy.deepcopy(base)
    meta.update({"group_id": group, "v_tier": v, "orientation_mode": mode,
                 "w_tier": "W0" if mode == "fixed" else w, "direction": direction,
                 "nominal_center_speed_m_s": config["center_speed_tiers_m_s"][v],
                 "nominal_speed_definition": "planned_peak_of_complete_vector_center_speed_not_mean_or_cruise",
                 "nominal_axis_rate_deg_s": 0. if mode == "fixed" else config["axis_rate_tiers_deg_s"][w]})
    cert = meta["continuous_position_certificate"]
    duration = math.ceil(cert["normalized_position_derivative_bounds"][1]/meta["nominal_center_speed_m_s"]*10-1e-10)/10
    meta.update({"draw_duration_s": duration, "draw_start_s": config["endpoint_hold_s"],
                 "draw_end_s": config["endpoint_hold_s"]+duration,
                 "duration_s": 2*config["endpoint_hold_s"]+duration,
                 "duration_reason": "retain complete rich curve and speed tier with continuous C3 start/stop; no cropping to 20 s",
                 "time_parameterization": "C3 septic XY progress; constant layer or separately scheduled C3 per-height-bin sweep; globally selected duration before CSV creation"})
    if mode == "varying":
        spec = meta["orientation"]
        target = math.radians(meta["nominal_axis_rate_deg_s"])
        spec["cycles"] = max(1, math.ceil(target*duration/(math.radians(18.)*2*math.pi*trajectory.S_BOUNDS[1])))
        spec["amplitude_rad"] = 1.
        unit = trajectory.angular_certificate(meta)["angular_derivative_bounds_deg"][0]
        spec["amplitude_rad"] = meta["nominal_axis_rate_deg_s"]/unit
    angular = trajectory.angular_certificate(meta)
    meta["continuous_angular_certificate"] = angular
    pb = cert["normalized_position_derivative_bounds"]
    ab = angular["angular_derivative_bounds_deg"]
    if pb[2]/duration**2 > config["center_acceleration_limit_m_s2"] or pb[3]/duration**3 > config["center_jerk_limit_m_s3"]:
        raise ValueError("time design does not meet center acceleration/jerk contract")
    if ab[1] > config["flange_angular_acceleration_limit_deg_s2"] or ab[2] > config["flange_angular_jerk_limit_deg_s3"]:
        raise ValueError(f"angular source design rejected: {meta['geometry_id']} {v} {w} {ab}")
    meta.update({"environment_id": "mla_short", "robot_model": "ur5",
                 "frame_id": "plane1", "height_definition": "h=-tip_z_m", "quaternion_order": "xyzw",
                 "reference_center": config["reference_center"],
                 "label_reference_type": "planned_nominal_pose_only_no_actual_reference_labels",
                 "calibration_status": "R_S_B_t_S_B_r_F_C_magnetization_axis_installation_and_active_TCP_not_accepted",
                 "source_design_status": "passed_continuous_Cartesian_limits",
                 "independent_source_validation": "see_separate_source_audit_report",
                 "robot_simulation": "not_run", "gpu_compile": "not_run", "mock_playback": "not_run",
                 "joint_limits_ik_arm_collision_entry_exit_parking": "not_evaluated",
                 "runtime_ready": False, "real_execution_authorized": False,
                 "runtime_authority": config["runtime_authority"],
                 "background_id": None, "synchronization_id": None, "installation_id": None,
                 "axis_definition": "nominal tool local +Z; changing axis not pure axial spin; magnetization binding pending",
                 "paired_geometry_id": meta["geometry_id"]})
    suffix = f"{v.lower()}_{mode}_{'rev' if direction=='reverse' else 'fwd'}"
    meta["trajectory_id"] = f"cp_{group}_{meta['geometry_id']}_{suffix}_v1"
    return meta


def build_plan(config):
    result = []
    seed = config["seed"]
    families = patterns.FAMILY_NAMES
    full = []
    for i, family in enumerate(families):
        base = new_geometry(family, seed+1000+i, i % 2, f"full_f{i:02d}",
                            {"mode":"sweep", "lower_m":.070, "upper_m":.200})
        full_speed_geometry_design(base)
        select_cycles(base, config, initial=2 if family == "braided_spiral" else 1)
        full.append(base)
        for vi, v in enumerate(("V1", "V2", "V3")):
            for mode in ("fixed", "varying"):
                result.append(timed_variant(base, config, "full_height", v, mode, w=f"W{1+(i+vi)%3}"))
        for mode in ("fixed", "varying"):
            result.append(timed_variant(base, config, "full_height_reverse", "V2", mode,
                                        direction="reverse", w=f"W{1+(i+1)%3}"))
        guarded = copy.deepcopy(base)
        guarded["height_profile"]["lower_m"] = .075
        guarded["min_height_m"] = .075
        guarded["comparison_geometry_id"] = base["geometry_id"]
        guarded["geometry_id"] += "_h075"
        guarded["explicit_height_difference_from_requested_full_range_m"] = .005
        # Same XY route/cycles. A fresh design/time/certificate and new ID;
        # the 70 mm authorities remain unchanged and explicitly rejected.
        guarded["continuous_position_certificate"] = trajectory.derivative_certificate(guarded)
        for mode in ("fixed", "varying"):
            result.append(timed_variant(guarded, config, "guarded_full_height", "V2", mode,
                                        w=f"W{1+(i+1)%3}"))
    for hi, height in enumerate(config["height_layers_m"]):
        for variant in (0, 1):
            family = families[(hi*3+variant*4) % len(families)]
            base = new_geometry(family, seed+2000+hi*10+variant, variant,
                                f"layer_h{round(height*1000):03d}_r{variant}",
                                {"mode":"constant", "lower_m":height, "upper_m":height})
            # More independent XY loops in high layers aim for about 60%
            # high-area time, while the speed tier retains its exact meaning.
            v = f"V{1+(hi+variant)%3}"
            if v == "V3":
                full_speed_geometry_design(base, force_large_envelope=True)
            select_cycles(base, config, initial=2 if height >= .15 else 1,
                          design_peak_speed=config["center_speed_tiers_m_s"][v])
            for mode in ("fixed", "varying"):
                result.append(timed_variant(base, config, "stratified", v, mode,
                                            w=f"W{1+(2*hi+variant)%3}"))
    return result


def arrays(meta, config):
    duration, dt, hold = meta["draw_duration_s"], config["source_sample_dt_s"], config["endpoint_hold_s"]
    local = np.linspace(0., duration, round(duration/dt)+1)
    p = trajectory.position_derivatives(meta, local/duration)[0]
    q = trajectory.orientation(meta, local/duration)
    nh = round(hold/dt)
    times = np.r_[np.arange(nh+1)*dt, hold+local[1:], hold+duration+np.arange(1, nh+1)*dt]
    positions = np.r_[np.repeat(p[:1], nh+1, axis=0), p[1:], np.repeat(p[-1:], nh, axis=0)]
    quaternions = np.r_[np.repeat(q[:1], nh+1, axis=0), q[1:], np.repeat(q[-1:], nh, axis=0)]
    phases = ["draw"]+["hold"]*nh+["draw"]*(len(local)-1)+["hold"]*nh
    return times, positions, quaternions, phases


def csv_payload(times, p, q, phases):
    stream = io.StringIO(newline="")
    writer = csv.writer(stream, lineterminator="\n")
    writer.writerow(FIELDS)
    previous = None
    for t, xyz, quat, phase in zip(times, p, q, phases, strict=True):
        pose = [f"{v:.12f}" for v in np.r_[xyz, quat]]
        if phase == "hold" and pose != previous:
            raise ValueError("serialized hold changes one of the seven authoritative pose fields")
        writer.writerow([f"{t:.9f}", "plane1", *pose, phase])
        previous = pose
    return stream.getvalue().encode()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch", type=Path)
    parser.add_argument("--plan-only", action="store_true")
    args = parser.parse_args()
    config = json.loads((ROOT/"config/contract_v1.json").read_text())
    geometry = read_geometry()
    plan = build_plan(config)
    print(json.dumps({"count":len(plan), "groups":{g:sum(m["group_id"]==g for m in plan)
          for g in sorted({m["group_id"] for m in plan})},
          "duration_range_s":[min(m["duration_s"] for m in plan),max(m["duration_s"] for m in plan)],
          "planned_draw_hours":sum(m["draw_duration_s"] for m in plan)/3600}, indent=2), flush=True)
    if args.plan_only:
        return
    batch = (args.batch or ROOT/"batches"/config["batch_id"]).resolve()
    if not batch.is_relative_to(ROOT):
        raise ValueError("all new work must remain inside continuous_patterns/")
    immutable(batch/"contract.json", json_bytes(config))
    immutable(batch/"frozen_geometry.json", json_bytes(geometry))
    provenance(batch, config)
    records = []
    for i, meta in enumerate(plan):
        t, p, q, phases = arrays(meta, config)
        meta["clearance_precheck"] = clearance.precheck(meta, p, q, t, geometry, config)
        csv_name = f"csv/{meta['trajectory_id']}.csv"
        meta_name = f"metadata/{meta['trajectory_id']}.json"
        payload = csv_payload(t, p, q, phases)
        meta["csv_file"], meta["source_sha256"] = csv_name, digest(payload)
        meta["source_pose_count"] = len(t)
        meta_payload = json_bytes(meta)
        immutable(batch/csv_name, payload)
        immutable(batch/meta_name, meta_payload)
        records.append({"trajectory_id":meta["trajectory_id"], "group_id":meta["group_id"],
                        "csv_file":csv_name, "metadata_file":meta_name, "sha256":digest(payload),
                        "metadata_sha256":digest(meta_payload),
                        "clearance_status":meta["clearance_precheck"]["status"],
                        "robot_simulation":"not_run", "runtime_ready":False})
        if (i+1) % 10 == 0:
            print(f"authored {i+1}/{len(plan)}", flush=True)
    order = np.random.default_rng(config["seed"]+9999).permutation(len(records)).tolist()
    manifest = {"schema_version":1,"batch_id":config["batch_id"],"seed":config["seed"],
        "trajectory_count":len(records), "manifest_scope":"this_isolated_candidate_bank_only_not_parent_active_v4",
        "primary_delivery_host":"ubuntu24", "primary_delivery_root":"/home/huaxin/Codes/ur-waypoint/workstation_magtrack/continuous_patterns",
        "records":records, "pre_generated_review_order":[records[i]["trajectory_id"] for i in order],
        "review_order_is_execution_authorization":False,
        "generator_environment":{"python":sys.version,"numpy":np.__version__,"scipy":scipy.__version__,"platform":platform.platform()},
        "real_execution_authorized":False,"robot_simulation":"not_run","runtime_ready":False}
    immutable(batch/"manifest.json", json_bytes(manifest))
    print(f"authority manifest: {batch/'manifest.json'}", flush=True)


if __name__ == "__main__":
    main()
