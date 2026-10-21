"""Conservative CPU rod/obstacle precheck; deliberately has no arm IK.

The board uses the existing exact horizontal-face orientation-cone model.
Other cuboids use an enclosing sphere chain, provisional pose errors and a
continuous-time Lipschitz margin between checked CSV times. This proves only
the modeled rod subset, never a complete robot/scene collision certificate.
"""
from __future__ import annotations
import math
import numpy as np
from scipy.spatial.transform import Rotation


def maximum_tilt(meta):
    spec = meta["orientation"]
    base = Rotation.from_rotvec(spec["base_rotvec_rad"])
    axis = np.asarray(spec["axis_in_base_tool_xyz"])
    amp = spec["amplitude_rad"] if meta["orientation_mode"] == "varying" else 0.
    a = base.apply([0., 0., 1.])[2]
    b = base.apply(np.cross(axis, [0., 0., 1.]))[2]
    candidates = [-amp, amp]
    critical = math.atan2(b, a)
    for n in range(-3, 4):
        value = critical + n*math.pi
        if -amp <= value <= amp:
            candidates.append(value)
    minimum_nz = min(a*math.cos(v)+b*math.sin(v) for v in candidates)
    return math.acos(float(np.clip(minimum_nz, -1., 1.)))


def rod_board_envelope(meta, geometry, contract):
    zones = geometry["zones"]
    box = zones["curobo_scene"]["cuboid"]["sensor_board_keepout"]
    boundary = zones["analysis_zones"]["sensor_keepout_clearance_boundary"]
    rod = geometry["setup"]["tool"]["rod"]
    length, radius = float(rod["length_m"]), float(rod["radius_m"])
    face_z = float(box["pose"][2] - box["dims"][2]/2)
    boundary_z = float(boundary["normal_xyz"][2]*boundary["offset_m"])
    required = float(zones["contact_policy"]["minimum_non_tip_clearance_m"])
    uncertainty = contract["provisional_position_uncertainty_radius_m"]
    delta = math.radians(contract["provisional_orientation_uncertainty_half_angle_deg"])
    alpha = maximum_tilt(meta)
    if alpha + delta >= math.pi/2:
        raise ValueError("rod model must point toward positive local-Z with conservative cone")
    error = length*(math.cos(max(0., alpha-delta))-math.cos(alpha))
    worst_tip_z = -meta["min_height_m"] + uncertainty + error
    tip_margin = boundary_z - worst_tip_z
    surface_clearance = face_z - (worst_tip_z + radius)
    margin = min(tip_margin, surface_clearance-required)
    return {"status": "passed" if margin >= -1e-12 else "rejected",
            "scope": "continuous_global_rigid_rod_sensor_face_envelope_only",
            "height_lower_bound_m": meta["min_height_m"],
            "maximum_nominal_tilt_deg": math.degrees(alpha),
            "provisional_position_uncertainty_radius_m": uncertainty,
            "provisional_orientation_uncertainty_half_angle_deg": math.degrees(delta),
            "rotation_induced_tip_positive_z_bound_m": error,
            "unchanged_sensor_keepout_face_z_m": face_z,
            "unchanged_analytic_tip_boundary_z_m": boundary_z,
            "unchanged_required_non_tip_clearance_m": required,
            "analytic_tip_margin_m": tip_margin,
            "rod_surface_clearance_lower_bound_m": surface_clearance,
            "non_tip_clearance_margin_m": surface_clearance-required,
            "minimum_clearance_margin_m": margin,
            "minimum_height_required_by_this_conservative_envelope_m":
                max(-boundary_z+uncertainty+error, -face_z+required+radius+uncertainty+error),
            "reason": "unchanged board/rod envelope accepted; arm and IK pending" if margin >= -1e-12
                      else "retained candidate rejected by unchanged board/rod provisional error envelope"}


def cuboid_signed_distance(points, dims):
    d = np.abs(points) - np.asarray(dims)/2
    return np.linalg.norm(np.maximum(d, 0.), axis=-1) + np.minimum(np.max(d, axis=-1), 0.)


def precheck(meta, positions, quaternions, times, geometry, contract):
    board = rod_board_envelope(meta, geometry, contract)
    rod = geometry["setup"]["tool"]["rod"]
    length, radius = rod["length_m"], rod["radius_m"]
    indices = np.unique(np.linspace(0, len(times)-1, min(len(times), 1001)).astype(int))
    p, q, t = positions[indices], quaternions[indices], times[indices]
    n = Rotation.from_quat(q).apply([0., 0., 1.])
    # Distance d is measured from flange. sqrt(r^2+(spacing/2)^2)
    # encloses the entire cylindrical surface between neighboring centers.
    d = np.linspace(0., length, 39)
    cover_radius = math.sqrt(radius**2 + (length/38/2)**2)
    centers = p[:, None, :] - (length-d)[None, :, None]*n[:, None, :]
    work = geometry["setup"]["work_frame"]
    raw = work["teach_pendant_pose_xyz_rotvec"]
    r = Rotation.from_rotvec(raw[3:])
    translation = np.asarray(raw[:3]) + r.apply(work["feature_to_work_frame_translation_m"])
    centers_controller_base = r.apply(centers.reshape(-1, 3)).reshape(centers.shape) + translation
    # setup.yaml is expressed in UR controller `base`, whereas the fixed
    # world cuboids use ROS REP-103 `base_link`. Resolve their fixed URDF
    # transform explicitly; for this UR5 it includes a pi rotation about Z.
    binding = geometry["base_link_to_base"]
    base_rotation = Rotation.from_euler("xyz", binding["rpy_rad"])
    centers_base = base_rotation.apply(centers_controller_base.reshape(-1, 3)).reshape(centers.shape) + binding["translation_m"]
    delta = math.radians(contract["provisional_orientation_uncertainty_half_angle_deg"])
    inflation = cover_radius + contract["provisional_position_uncertainty_radius_m"] + 2*d*math.sin(delta/2)
    vbound = meta["continuous_position_certificate"]["normalized_position_derivative_bounds"][1] / meta["draw_duration_s"]
    wbound = math.radians(meta["continuous_angular_certificate"]["angular_derivative_bounds_deg"][0])
    half_gap = float(np.max(np.diff(t)))/2.
    temporal_margin = (vbound+(length-d)*wbound)*half_gap
    required = geometry["zones"]["contact_policy"]["minimum_non_tip_clearance_m"]
    obstacles = []
    for name, box in geometry["zones"]["curobo_scene"]["cuboid"].items():
        if name == "sensor_board_keepout":
            continue
        pose = box["pose"]
        # cuRobo configuration uses wxyz; source CSV uses xyzw.
        rotation = Rotation.from_quat([pose[4], pose[5], pose[6], pose[3]])
        points = centers_base if box["frame_id"] == "base_link" else centers
        local = rotation.inv().apply((points-np.asarray(pose[:3])).reshape(-1, 3)).reshape(points.shape)
        distances = cuboid_signed_distance(local, box["dims"]) - inflation - temporal_margin
        lower = float(distances.min())
        obstacles.append({"obstacle_id": name, "surface_clearance_lower_bound_m": lower,
                          "required_clearance_m": required,
                          "status": "passed" if lower >= required else "rejected"})
    passed = board["status"] == "passed" and all(x["status"] == "passed" for x in obstacles)
    return {"status": "passed_rod_subset_only" if passed else "rejected_rod_subset",
            "board": board, "external_cuboids": obstacles,
            "time_sample_count": len(indices), "rod_center_count": len(d),
            "rod_covering_sphere_radius_m": cover_radius,
            "maximum_time_gap_s": 2*half_gap,
            "frame_chain": "plane1 -> controller base -> URDF fixed base_link",
            "base_link_to_base_urdf_sha256": binding["urdf_sha256"],
            "continuous_time_motion_bound_applied": True,
            "arm_self_collision_ik_joint_dynamics": "not_evaluated",
            "robot_simulation": "not_run",
            "runtime_ready": False}
