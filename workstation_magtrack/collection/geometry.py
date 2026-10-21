"""CPU-only, discrete nominal collision screening for collection parking.

This deliberately does not plan a motion, attach a clock to joint samples, or
certify swept/continuous safety.  A passed screen still needs the existing GPU
motion planner/validator before it can be used by a controller.
"""
from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
from typing import Any

import numpy as np

from hx_waypoint_curobo.environment_builder import EnvironmentBuilder, EnvironmentBundle
from hx_waypoint_curobo.math3d import inverse_pose, matrix_pose, pose_matrix
from hx_waypoint_curobo.robot_kinematics import Pose, RobotKinematics
from hx_waypoint_curobo.robot_tool_builder import RobotToolBuilder
from hx_waypoint_curobo.scene_config import resolve_curobo_cuboids


def _finite_nonnegative(value: object, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be a finite nonnegative number")
    try:
        number = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label} must be a finite nonnegative number") from error
    if not np.isfinite(number) or number < 0.0:
        raise ValueError(f"{label} must be a finite nonnegative number")
    return number


def _sphere_box_clearances(
    centers: np.ndarray, radii: np.ndarray, box_pose: np.ndarray, dimensions: np.ndarray,
) -> np.ndarray:
    """Signed surface clearance, including sphere penetration inside a box."""
    # box_pose maps object -> environment base, so row-vector inverse is * R.
    local = (centers - box_pose[:3, 3]) @ box_pose[:3, :3]
    outside = np.abs(local) - dimensions / 2.0
    signed_distance = np.linalg.norm(np.maximum(outside, 0.0), axis=1)
    signed_distance += np.minimum(np.max(outside, axis=1), 0.0)
    return signed_distance - radii


def _self_pairs(links: Sequence[str], ignored: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    """Same-link and explicitly ignored pairs are excluded, in both directions."""
    first, second = [], []
    for i, link in enumerate(links):
        for j in range(i + 1, len(links)):
            other = links[j]
            if (link == other or other in ignored.get(link, ())
                    or link in ignored.get(other, ())):
                continue
            first.append(i)
            second.append(j)
    if not first:
        raise ValueError("collision model has no enabled self-collision pairs")
    return np.asarray(first, dtype=int), np.asarray(second, dtype=int)


def screen_joint_positions(
    environment: EnvironmentBundle | str | Path,
    positions: Sequence[Sequence[float]],
) -> dict[str, Any]:
    """Screen each supplied row against the unchanged environment geometry.

    Uses every configured robot sphere plus RobotToolBuilder's nominal rod
    chain, shared frame-resolved world cuboids, and the cuRobo self-ignore and
    radius-buffer policy.  No interpolation, clamping, branch substitution,
    waypoint editing, dynamics check, or controller/GPU access occurs here.
    """
    env = environment if isinstance(environment, EnvironmentBundle) else EnvironmentBuilder().load(environment)
    values = np.asarray(positions, dtype=float)
    if (values.ndim != 2 or values.shape[0] == 0
            or values.shape[1] != len(env.joint_names) or not np.all(np.isfinite(values))):
        raise ValueError("positions must be a nonempty finite N x joint_count matrix")

    required = float(env.planner["validation"]["minimum_collision_distance_m"])
    if not np.isfinite(required) or required < 0.005:
        raise ValueError("CPU screen requires the unchanged minimum clearance of at least 5 mm")
    config = env.curobo_robot["robot_cfg"]["kinematics"]
    if config["base_link"] != env.base_frame:
        raise ValueError("cuRobo and EnvironmentBundle base frames must match")
    global_buffer = _finite_nonnegative(config.get("collision_sphere_buffer", 0.0), "collision_sphere_buffer")
    self_buffers = {name: _finite_nonnegative(value, f"self_collision_buffer.{name}")
                    for name, value in config.get("self_collision_buffer", {}).items()}
    configured_links = tuple(config["collision_link_names"])
    source_spheres = env.collision_spheres["collision_spheres"]
    if set(source_spheres) != set(configured_links):
        raise ValueError("configured collision links and robot sphere links differ")

    names: list[str] = []
    links: list[str] = []
    local_centers: list[tuple[float, ...]] = []
    radii: list[float] = []
    for link in configured_links:
        for index, sphere in enumerate(source_spheres[link]):
            names.append(f"{link}/robot_{index:03d}")
            links.append(link)
            local_centers.append(tuple(sphere["center"]))
            radii.append(float(sphere["radius"]) + global_buffer)
    robot_sphere_count = len(names)
    tool = RobotToolBuilder(env)
    rod_spheres = tool.collision_spheres()
    if tool.parent_link not in configured_links:
        raise ValueError("rod parent link is missing from collision_link_names")
    for sphere in rod_spheres:
        names.append(f"{sphere.link_name}/{sphere.name}")
        links.append(sphere.link_name)
        local_centers.append(sphere.center_m)
        radii.append(sphere.radius_m + global_buffer)
    local_array, world_radii = np.asarray(local_centers), np.asarray(radii)
    if (local_array.shape != (len(names), 3) or not np.all(np.isfinite(local_array))
            or not np.all(np.isfinite(world_radii)) or np.any(world_radii <= 0)):
        raise ValueError("invalid nominal robot/rod collision spheres")
    self_radii = world_radii + np.asarray([self_buffers.get(link, 0.0) for link in links])
    pair_i, pair_j = _self_pairs(links, config.get("self_collision_ignore", {}))

    urdf = env.root / env.asset_provenance["generated_urdf"]["path"]
    kinematics = RobotKinematics.from_urdf_bytes(
        urdf.read_bytes(), base_link=env.base_frame, joint_names=env.joint_names,
    )
    tip_position, tip_quaternion = matrix_pose(tool.tool0_to_tip)
    tool_to_tip = Pose(tip_position, tip_quaternion)
    frames = tuple(dict.fromkeys((*links, tool.tip_frame)))
    base_link_from_base = inverse_pose(env._frame_to_base("base_link"))
    plane_from_base = inverse_pose(env._frame_to_base("plane1"))
    cuboids = resolve_curobo_cuboids(env)
    boxes = [(box, pose_matrix(box.position_m, box.quaternion_xyzw), np.asarray(box.dimensions_m))
             for box in cuboids]
    branch_mask = env.canonical_joint_branch_mask(values)
    domains = env.joint_position_domains
    lower = np.asarray([domain.lower for domain in domains])
    upper = np.asarray([domain.upper for domain in domains])
    in_limits = np.all((values >= lower) & (values <= upper), axis=1)

    world_minimum: dict[str, Any] | None = None
    self_minimum: dict[str, Any] | None = None
    rows: list[dict[str, Any]] = []
    reasons: list[str] = []
    for row_index, q in enumerate(values):
        poses = kinematics.forward_kinematics(q, frames, tool_to_tip=tool_to_tip,
                                             tool_frame=tool.parent_link, tip_frame=tool.tip_frame)
        transforms = {frame: pose_matrix(pose.position_m, pose.quaternion_xyzw)
                      for frame, pose in poses.items()}
        centers = np.empty_like(local_array)
        for link in configured_links:
            selected = np.asarray([index for index, item in enumerate(links) if item == link])
            centers[selected] = local_array[selected] @ transforms[link][:3, :3].T + transforms[link][:3, 3]

        row_world = float("inf")
        for box, transform, dimensions in boxes:
            clearances = _sphere_box_clearances(centers, world_radii, transform, dimensions)
            nearest = int(np.argmin(clearances))
            clearance = float(clearances[nearest])
            row_world = min(row_world, clearance)
            if world_minimum is None or clearance < world_minimum["clearance_m"]:
                world_minimum = {"clearance_m": clearance, "row_index": row_index,
                                 "obstacle_id": box.object_id, "link_name": links[nearest],
                                 "sphere_name": names[nearest]}
        self_distances = np.linalg.norm(centers[pair_i] - centers[pair_j], axis=1)
        self_distances -= self_radii[pair_i] + self_radii[pair_j]
        nearest_pair = int(np.argmin(self_distances))
        row_self = float(self_distances[nearest_pair])
        if self_minimum is None or row_self < self_minimum["clearance_m"]:
            i, j = int(pair_i[nearest_pair]), int(pair_j[nearest_pair])
            self_minimum = {"clearance_m": row_self, "row_index": row_index,
                            "link_names": [links[i], links[j]], "sphere_names": [names[i], names[j]]}

        tip_base = transforms[tool.tip_frame]
        row = {"row_index": row_index, "joint_positions_rad": q.tolist(),
               "canonical_branch_passed": bool(branch_mask[row_index]),
               "joint_limits_passed": bool(in_limits[row_index]),
               "violating_joint_names": [env.joint_names[j] for j in range(len(q))
                                          if q[j] < lower[j] or q[j] > upper[j]],
               "minimum_world_clearance_m": row_world, "minimum_self_clearance_m": row_self,
               "tip_position_base_link_m": (base_link_from_base @ tip_base)[:3, 3].tolist(),
               "tip_position_plane1_m": (plane_from_base @ tip_base)[:3, 3].tolist()}
        rows.append(row)
        if not row["joint_limits_passed"]:
            reasons.append(f"row {row_index}: joint position limit violation")
        if not row["canonical_branch_passed"]:
            reasons.append(f"row {row_index}: canonical joint branch violation")
        if row_world < required:
            reasons.append(f"row {row_index}: world clearance {row_world:.12g} < {required:.12g} m")
        if row_self < required:
            reasons.append(f"row {row_index}: self clearance {row_self:.12g} < {required:.12g} m")

    assert world_minimum is not None and self_minimum is not None
    return {"schema_version": 1, "status": "rejected" if reasons else "CPU_sampled_passed",
            "environment_id": env.environment_id, "base_frame": env.base_frame,
            "sample_count": len(values), "required_minimum_clearance_m": required,
            "minimum_world_clearance_m": world_minimum["clearance_m"],
            "minimum_self_clearance_m": self_minimum["clearance_m"],
            "world_minimum": world_minimum, "self_minimum": self_minimum,
            "robot_sphere_count": robot_sphere_count, "rod_sphere_count": len(rod_spheres),
            "self_pair_count": len(pair_i), "world_cuboid_count": len(boxes),
            "rows": rows, "end_tip_position_base_link_m": rows[-1]["tip_position_base_link_m"],
            "end_tip_position_plane1_m": rows[-1]["tip_position_plane1_m"],
            "rejection_reasons": reasons, "sampling_only": True, "gpu_validated": False,
            "continuous_collision_checked": False, "dynamics_checked": False,
            "real_execution_authorized": False,
            "scope": "nominal_configured_robot_and_rod_spheres_at_supplied_joint_rows_only",
            "environment_hashes": dict(env.hashes)}


def screen_home_parking(
    environment: EnvironmentBundle | str | Path,
    parking_rad: Sequence[float],
    sample_count: int = 201,
) -> dict[str, Any]:
    """Diagnose a straight joint-space sample grid, not an executable motion.

    Reverse travel visits the identical geometric samples.  Neither direction
    has a swept-volume certificate, timing, acceleration, or jerk approval.
    """
    env = environment if isinstance(environment, EnvironmentBundle) else EnvironmentBuilder().load(environment)
    if isinstance(sample_count, bool) or not isinstance(sample_count, int) or sample_count < 2:
        raise ValueError("sample_count must be an integer of at least 2")
    target = np.asarray(parking_rad, dtype=float)
    if target.shape != (len(env.joint_names),) or not np.all(np.isfinite(target)):
        raise ValueError("parking_rad must contain exactly joint_count finite values")
    samples = np.linspace(np.asarray(env.home_joint_positions), target, sample_count)
    result = screen_joint_positions(env, samples)
    result.update({"diagnostic_path": "linear_joint_samples_home_to_parking",
                   "reverse_direction_has_same_geometric_samples": True,
                   "home_joint_positions_rad": list(env.home_joint_positions),
                   "parking_joint_positions_rad": target.tolist(), "motion_planned": False})
    return result
