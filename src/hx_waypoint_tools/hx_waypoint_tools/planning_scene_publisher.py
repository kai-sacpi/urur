from __future__ import annotations

import math
import re
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_curobo.errors import ConfigurationError
from hx_waypoint_curobo.scene_config import resolve_curobo_cuboids
from hx_waypoint_curobo.setup_config import rotvec_to_quaternion_xyzw


APPLY_PLANNING_SCENE_SERVICE = "/apply_planning_scene"
GET_PLANNING_SCENE_SERVICE = "/get_planning_scene"
ATTACHED_ROD_OBJECT_ID = "hx_waypoint_attached_rod"
ENVIRONMENT_MARKER_TOPIC = "/hx_waypoint/environment_markers"
_VISUALIZATION_PLANE_THICKNESS_M = 0.002
_SAFE_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


class PlanningSceneConfigurationError(ValueError):
    """Raised when an environment cannot be represented faithfully in MoveIt."""


@dataclass(frozen=True)
class CollisionCuboid:
    """ROS-neutral representation of one cuRobo cuboid.

    cuRobo stores a cuboid pose as ``[x, y, z, qw, qx, qy, qz]``.  The
    orientation exposed here is deliberately converted to ROS' ``xyzw`` order.
    """

    object_id: str
    frame_id: str
    dimensions_m: tuple[float, float, float]
    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class AttachedCollisionCylinder:
    """ROS-neutral description of the environment's tool0-attached rod."""

    object_id: str
    link_name: str
    length_m: float
    radius_m: float
    axis_in_link_xyz: tuple[float, float, float]
    center_in_link_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    touch_links: tuple[str, ...]


@dataclass(frozen=True)
class VisualizationPlane:
    """Non-collision RViz plane derived from optional environment calibration data."""

    namespace: str
    marker_id: int
    frame_id: str
    size_xy_m: tuple[float, float]
    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    color_rgba: tuple[float, float, float, float]
    label: str
    label_position_m: tuple[float, float, float]


@dataclass(frozen=True)
class PlanningSceneDefinition:
    environment_id: str
    frame_id: str
    joint_names: tuple[str, ...]
    remove_object_ids: tuple[str, ...]
    cuboids: tuple[CollisionCuboid, ...]
    attached_rod: AttachedCollisionCylinder
    visualization_planes: tuple[VisualizationPlane, ...]


@dataclass(frozen=True)
class PlanningSceneRequestStep:
    """One ordered request in the fail-closed scene transition."""

    label: str
    request: Any
    success_required: bool


def _finite_vector(value: Any, size: int, label: str) -> tuple[float, ...]:
    if (
        isinstance(value, (str, bytes))
        or not isinstance(value, Sequence)
        or len(value) != size
    ):
        raise PlanningSceneConfigurationError(
            f"{label} must contain exactly {size} finite numbers"
        )
    result: list[float] = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise PlanningSceneConfigurationError(
                f"{label} must contain exactly {size} finite numbers"
            )
        number = float(item)
        if not math.isfinite(number):
            raise PlanningSceneConfigurationError(
                f"{label} must contain exactly {size} finite numbers"
            )
        result.append(number)
    return tuple(result)


def _positive_size_xy(value: Any, label: str) -> tuple[float, float]:
    size = _finite_vector(value, 2, label)
    if any(component <= 0.0 for component in size):
        raise PlanningSceneConfigurationError(f"{label} must contain positive dimensions")
    return size  # type: ignore[return-value]


def _quaternion_align_z_to_axis(
    axis_xyz: tuple[float, float, float],
) -> tuple[float, float, float, float]:
    """Return the canonical xyzw rotation from cylinder-local +Z to ``axis_xyz``."""

    axis_norm = math.sqrt(sum(component * component for component in axis_xyz))
    if not math.isfinite(axis_norm) or axis_norm <= 1.0e-15:
        raise PlanningSceneConfigurationError("attached rod axis must be a finite unit vector")
    x, y, z = (component / axis_norm for component in axis_xyz)
    if abs(axis_norm - 1.0) > 1.0e-9:
        raise PlanningSceneConfigurationError("attached rod axis must be a finite unit vector")

    # The shortest-arc quaternion is proportional to
    # ``cross(+Z, axis), 1 + dot(+Z, axis)``.  Only the exactly antiparallel
    # case is degenerate; choose +X deterministically there for stable diffs.
    quaternion = (-y, x, 0.0, 1.0 + z)
    quaternion_norm = math.sqrt(sum(component * component for component in quaternion))
    if quaternion_norm <= 1.0e-15:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(
        component / quaternion_norm for component in quaternion
    )  # type: ignore[return-value]


def _rotate_vector_xyzw(
    quaternion_xyzw: tuple[float, float, float, float],
    vector_xyz: tuple[float, float, float],
) -> tuple[float, float, float]:
    qx, qy, qz, qw = quaternion_xyzw
    vx, vy, vz = vector_xyz
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def _visualization_planes_from_environment(environment: Any) -> tuple[VisualizationPlane, ...]:
    """Resolve optional board planes without adding them to the collision scene."""

    calibration = environment.environment.get("calibration")
    if not isinstance(calibration, Mapping):
        return ()
    dashboard = calibration.get("dashboard")
    sensor_geometry = calibration.get("sensor_geometry")
    if not isinstance(dashboard, Mapping) or not isinstance(sensor_geometry, Mapping):
        return ()
    raw_pose_value = dashboard.get("raw_base_to_feature_pose_xyz_rotvec")
    board_size_value = sensor_geometry.get("board_size_xy_m")
    if raw_pose_value is None and board_size_value is None:
        return ()
    if raw_pose_value is None or board_size_value is None:
        raise PlanningSceneConfigurationError(
            "calibration visualization requires both raw feature pose and board_size_xy_m"
        )
    if dashboard.get("pose_semantics") != "base_to_feature":
        raise PlanningSceneConfigurationError(
            "calibration.dashboard.pose_semantics must be base_to_feature for visualization"
        )
    feature_name = dashboard.get("variable")
    if not isinstance(feature_name, str) or not feature_name.strip():
        raise PlanningSceneConfigurationError(
            "calibration.dashboard.variable must name the visualized UR Feature"
        )
    board_matches_feature = (
        sensor_geometry.get("board_center_matches_feature_origin") is True
    )
    board_matches_work_frame = (
        sensor_geometry.get("board_center_matches_work_frame_origin") is True
    )
    if board_matches_feature == board_matches_work_frame:
        raise PlanningSceneConfigurationError(
            "calibration visualization must identify exactly one board-center reference: "
            "Feature origin or work-frame origin"
        )
    board_center_xy = _finite_vector(
        sensor_geometry.get("board_center_in_plane1_xy_m"),
        2,
        "calibration.sensor_geometry.board_center_in_plane1_xy_m",
    )
    if board_center_xy != (0.0, 0.0):
        raise PlanningSceneConfigurationError(
            "confirmed board center in plane1 must be exactly [0.0, 0.0]"
        )

    raw_pose = _finite_vector(
        raw_pose_value,
        6,
        "calibration.dashboard.raw_base_to_feature_pose_xyz_rotvec",
    )
    board_size = _positive_size_xy(
        board_size_value,
        "calibration.sensor_geometry.board_size_xy_m",
    )
    raw_feature_reference_size = tuple(min(component, 0.10) for component in board_size)
    analysis_zones = environment.zones.get("analysis_zones")
    if not isinstance(analysis_zones, Mapping):
        raise PlanningSceneConfigurationError(
            "calibration visualization requires zones.analysis_zones"
        )
    actual_surface = analysis_zones.get("actual_sensor_plane")
    if not isinstance(actual_surface, Mapping):
        raise PlanningSceneConfigurationError(
            "calibration visualization requires analysis_zones.actual_sensor_plane"
        )
    frame_id = actual_surface.get("frame_id")
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise PlanningSceneConfigurationError(
            "analysis_zones.actual_sensor_plane.frame_id must be non-empty"
        )
    normal = _finite_vector(
        actual_surface.get("normal_xyz"),
        3,
        "analysis_zones.actual_sensor_plane.normal_xyz",
    )
    normal_norm = math.sqrt(sum(component * component for component in normal))
    if abs(normal_norm - 1.0) > 1.0e-9:
        raise PlanningSceneConfigurationError(
            "analysis_zones.actual_sensor_plane.normal_xyz must be a unit vector"
        )
    offset = _finite_vector(
        [actual_surface.get("offset_m")],
        1,
        "analysis_zones.actual_sensor_plane.offset_m",
    )[0]
    actual_position = (
        board_center_xy[0] + normal[0] * offset,
        board_center_xy[1] + normal[1] * offset,
        normal[2] * offset,
    )
    actual_label_position = (
        actual_position[0] + 0.25,
        actual_position[1] - 0.25,
        actual_position[2] - 0.03,
    )
    raw_position = raw_pose[:3]
    raw_orientation = rotvec_to_quaternion_xyzw(raw_pose[3:])
    raw_label_offset = _rotate_vector_xyzw(
        raw_orientation,
        (-0.15, 0.15, -0.03),
    )
    raw_label_position = tuple(
        raw_position[index] + raw_label_offset[index] for index in range(3)
    )
    if board_matches_feature:
        board_label = "PURPLE_Feature_reference_z=0_not_sensor"
    elif abs(offset) <= 1.0e-12:
        board_label = "PURPLE_work_frame_sensor_center_z=0"
    else:
        board_label = "PURPLE_MLA_work_frame_origin_z=0_not_sensor"

    return (
        VisualizationPlane(
            namespace="raw_ur_feature",
            marker_id=0,
            frame_id=environment.setup_config.work_frame.parent_frame,
            size_xy_m=raw_feature_reference_size,  # type: ignore[arg-type]
            position_m=raw_position,  # type: ignore[arg-type]
            quaternion_xyzw=raw_orientation,
            color_rgba=(1.0, 0.55, 0.0, 0.55),
            label=f"ORANGE_raw_Feature_reference:{feature_name}",
            label_position_m=raw_label_position,  # type: ignore[arg-type]
        ),
        VisualizationPlane(
            namespace="calibration_board_plane",
            marker_id=0,
            frame_id=environment.setup_config.work_frame.child_frame,
            size_xy_m=board_size,
            position_m=(board_center_xy[0], board_center_xy[1], 0.0),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            color_rgba=(0.85, 0.2, 1.0, 0.55),
            label=board_label,
            label_position_m=(
                board_center_xy[0] - 0.25,
                board_center_xy[1] - 0.25,
                -0.03,
            ),
        ),
        VisualizationPlane(
            namespace="actual_sensor_plane",
            marker_id=0,
            frame_id=frame_id,
            size_xy_m=board_size,
            position_m=actual_position,  # type: ignore[arg-type]
            quaternion_xyzw=_quaternion_align_z_to_axis(normal),  # type: ignore[arg-type]
            color_rgba=(0.0, 0.75, 1.0, 0.75),
            label=f"CYAN_sensor_z={actual_position[2] * 1000.0:+.0f}mm",
            label_position_m=actual_label_position,
        ),
    )


def load_planning_scene_definition(
    workspace: str | Path,
    environment_id: str,
) -> PlanningSceneDefinition:
    """Load one environment scene without importing ROS packages."""

    normalized_id = str(environment_id).strip()
    if not _SAFE_IDENTIFIER.fullmatch(normalized_id) or normalized_id in {".", ".."}:
        raise PlanningSceneConfigurationError(f"invalid environment_id: {environment_id!r}")

    workspace_root = Path(workspace).expanduser().resolve()
    environments_root = (workspace_root / "environments").resolve()
    if not environments_root.is_dir():
        raise PlanningSceneConfigurationError(
            f"workspace does not contain an environments directory: {workspace_root}"
        )
    environment_root = (environments_root / normalized_id).resolve()
    if environment_root.parent != environments_root or not environment_root.is_dir():
        raise PlanningSceneConfigurationError(
            f"environment directory does not exist inside workspace: {normalized_id}"
        )

    try:
        environment = EnvironmentBuilder().load(environment_root)
        resolved = resolve_curobo_cuboids(environment)
    except ConfigurationError as error:
        raise PlanningSceneConfigurationError(str(error)) from error
    if environment.environment_id != normalized_id:
        raise PlanningSceneConfigurationError(
            "environment.yaml environment_id does not match the requested environment: "
            f"{environment.environment_id!r} != {normalized_id!r}"
        )
    cuboids = tuple(
        CollisionCuboid(
            object_id=cuboid.object_id,
            frame_id=cuboid.base_frame_id,
            dimensions_m=cuboid.dimensions_m,
            position_m=cuboid.position_m,
            quaternion_xyzw=cuboid.quaternion_xyzw,
        )
        for cuboid in resolved
    )
    remove_object_ids: tuple[str, ...] = ()
    workspace_boundaries = environment.environment.get("workspace_boundaries")
    if workspace_boundaries is not None:
        if not isinstance(workspace_boundaries, Mapping):
            raise PlanningSceneConfigurationError(
                "environment.yaml workspace_boundaries must be a mapping"
            )
        raw_remove_ids = workspace_boundaries.get("remove_collision_object_ids", [])
        if (
            isinstance(raw_remove_ids, (str, bytes))
            or not isinstance(raw_remove_ids, Sequence)
        ):
            raise PlanningSceneConfigurationError(
                "workspace_boundaries.remove_collision_object_ids must be a sequence"
            )
        normalized_remove_ids: list[str] = []
        for raw_object_id in raw_remove_ids:
            if (
                not isinstance(raw_object_id, str)
                or not _SAFE_IDENTIFIER.fullmatch(raw_object_id)
            ):
                raise PlanningSceneConfigurationError(
                    "workspace_boundaries.remove_collision_object_ids contains an invalid id"
                )
            normalized_remove_ids.append(raw_object_id)
        if len(set(normalized_remove_ids)) != len(normalized_remove_ids):
            raise PlanningSceneConfigurationError(
                "workspace_boundaries.remove_collision_object_ids contains duplicates"
            )
        configured_ids = {cuboid.object_id for cuboid in cuboids}
        overlap = sorted(configured_ids.intersection(normalized_remove_ids))
        if overlap:
            raise PlanningSceneConfigurationError(
                "collision object ids cannot be both removed and added: "
                + ", ".join(overlap)
            )
        remove_object_ids = tuple(normalized_remove_ids)
    tool = environment.setup_config.tool
    axis = tool.axis_in_parent_xyz
    attached_rod = AttachedCollisionCylinder(
        object_id=ATTACHED_ROD_OBJECT_ID,
        link_name=tool.parent_link,
        length_m=tool.length_m,
        radius_m=tool.radius_m,
        axis_in_link_xyz=axis,
        center_in_link_m=tuple(
            component * tool.length_m * 0.5 for component in axis
        ),  # type: ignore[arg-type]
        quaternion_xyzw=_quaternion_align_z_to_axis(axis),
        touch_links=(tool.parent_link,),
    )
    visualization_planes = _visualization_planes_from_environment(environment)
    return PlanningSceneDefinition(
        environment_id=normalized_id,
        frame_id=environment.base_frame,
        joint_names=environment.joint_names,
        remove_object_ids=remove_object_ids,
        cuboids=cuboids,
        attached_rod=attached_rod,
        visualization_planes=visualization_planes,
    )


def _complete_joint_positions(
    message: Any,
    required_joint_names: Sequence[str],
) -> tuple[float, ...] | None:
    """Return one complete finite ordered JointState, otherwise ``None``.

    The planning-scene service can appear before MoveGroup has received a
    usable robot state.  Applying an object attached to ``tool0`` during that
    interval makes MoveIt try to transform an incomplete robot tree and may
    leave the service request unanswered.  Keep this predicate ROS-neutral so
    the exact readiness contract can be unit tested without ROS installed.
    """

    names = tuple(getattr(message, "name", ()))
    positions = tuple(getattr(message, "position", ()))
    required = tuple(str(name) for name in required_joint_names)
    if (
        not required
        or len(set(required)) != len(required)
        or len(names) != len(positions)
        or len(set(names)) != len(names)
    ):
        return None
    by_name = dict(zip(names, positions, strict=True))
    if any(name not in by_name for name in required):
        return None
    try:
        ordered = tuple(float(by_name[name]) for name in required)
    except (TypeError, ValueError):
        return None
    return ordered if all(math.isfinite(value) for value in ordered) else None


def _wait_for_attached_scene_inputs(
    *,
    node: Any,
    rclpy: Any,
    JointState: Any,
    qos_profile_sensor_data: Any,
    tf_buffer: Any,
    Time: Any,
    definition: PlanningSceneDefinition,
    timeout_s: float,
) -> None:
    """Wait for a post-service JointState and the robot-link TF used by ADD.

    The subscription is intentionally created only after
    ``/apply_planning_scene`` is available.  JointState uses volatile QoS, so
    a sample received here was published after MoveGroup exposed its service;
    this prevents a queued pre-start sample from satisfying the gate.
    """

    ready_positions: tuple[float, ...] | None = None

    def observe_joint_state(message: Any) -> None:
        nonlocal ready_positions
        positions = _complete_joint_positions(message, definition.joint_names)
        if positions is not None:
            ready_positions = positions

    subscription = node.create_subscription(
        JointState,
        "/joint_states",
        observe_joint_state,
        qos_profile_sensor_data,
    )
    deadline = time.monotonic() + timeout_s
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(
                node,
                timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())),
            )
            if ready_positions is None:
                continue
            if tf_buffer.can_transform(
                definition.frame_id,
                definition.attached_rod.link_name,
                Time(),
            ):
                return
    finally:
        node.destroy_subscription(subscription)
    missing: list[str] = []
    if ready_positions is None:
        missing.append("a complete finite JointState")
    if not tf_buffer.can_transform(
        definition.frame_id,
        definition.attached_rod.link_name,
        Time(),
    ):
        missing.append(
            f"TF {definition.frame_id}->{definition.attached_rod.link_name}"
        )
    raise RuntimeError(
        "robot state was not ready for the attached collision scene within "
        f"{timeout_s:.1f} s; missing " + " and ".join(missing)
    )


def _build_apply_request(definition: PlanningSceneDefinition, ros_types: Mapping[str, Any]) -> Any:
    ApplyPlanningScene = ros_types["ApplyPlanningScene"]
    AttachedCollisionObject = ros_types["AttachedCollisionObject"]
    CollisionObject = ros_types["CollisionObject"]
    PlanningScene = ros_types["PlanningScene"]
    Pose = ros_types["Pose"]
    SolidPrimitive = ros_types["SolidPrimitive"]

    request = ApplyPlanningScene.Request()
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    for cuboid in definition.cuboids:
        collision_object = CollisionObject()
        collision_object.header.frame_id = cuboid.frame_id
        collision_object.id = cuboid.object_id
        collision_object.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(cuboid.dimensions_m)

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = cuboid.position_m
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = cuboid.quaternion_xyzw
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        scene.world.collision_objects.append(collision_object)

    rod = definition.attached_rod
    rod_object = CollisionObject()
    rod_object.header.frame_id = rod.link_name
    rod_object.id = rod.object_id
    rod_object.operation = CollisionObject.ADD

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.CYLINDER
    primitive.dimensions = [rod.length_m, rod.radius_m]

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = rod.center_in_link_m
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = rod.quaternion_xyzw
    rod_object.primitives.append(primitive)
    rod_object.primitive_poses.append(pose)

    attached = AttachedCollisionObject()
    attached.link_name = rod.link_name
    attached.object = rod_object
    attached.touch_links = list(rod.touch_links)
    scene.robot_state.attached_collision_objects.append(attached)
    request.scene = scene
    return request


def _build_remove_request(
    definition: PlanningSceneDefinition,
    ros_types: Mapping[str, Any],
) -> Any:
    """Build a cleanup diff separate from authoritative scene additions.

    MoveIt may report ``success=false`` when a REMOVE target is already absent.
    Keeping cleanup separate prevents that benign cold-start result from
    rejecting the current collision scene.
    """

    ApplyPlanningScene = ros_types["ApplyPlanningScene"]
    CollisionObject = ros_types["CollisionObject"]
    PlanningScene = ros_types["PlanningScene"]

    request = ApplyPlanningScene.Request()
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    for object_id in definition.remove_object_ids:
        collision_object = CollisionObject()
        collision_object.header.frame_id = definition.frame_id
        collision_object.id = object_id
        collision_object.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(collision_object)
    request.scene = scene
    return request


def _build_application_steps(
    definition: PlanningSceneDefinition,
    ros_types: Mapping[str, Any],
    *,
    first_application: bool,
) -> tuple[PlanningSceneRequestStep, ...]:
    """Build an ADD-first transition that never removes the last safe scene.

    The current authoritative collision objects are always installed strictly.
    Legacy cleanup is an initial-only, best-effort response step after that ADD;
    an absent legacy object may produce ``success=false`` without weakening the
    already-active scene.
    """

    steps = [
        PlanningSceneRequestStep(
            label="current planning scene",
            request=_build_apply_request(definition, ros_types),
            success_required=True,
        )
    ]
    if first_application and definition.remove_object_ids:
        steps.append(
            PlanningSceneRequestStep(
                label="legacy-object cleanup",
                request=_build_remove_request(definition, ros_types),
                success_required=False,
            )
        )
    return tuple(steps)


def _build_world_object_names_request(ros_types: Mapping[str, Any]) -> Any:
    """Request the authoritative MoveIt world-object names after cleanup."""

    GetPlanningScene = ros_types["GetPlanningScene"]
    PlanningSceneComponents = ros_types["PlanningSceneComponents"]
    request = GetPlanningScene.Request()
    request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
    return request


def _remaining_legacy_object_ids(
    response: Any,
    remove_object_ids: Sequence[str],
) -> tuple[str, ...]:
    """Return configured legacy IDs still present in a GetPlanningScene response."""

    try:
        collision_objects = response.scene.world.collision_objects
    except AttributeError as error:
        raise PlanningSceneConfigurationError(
            "MoveIt world-object verification response is malformed"
        ) from error
    present_ids: set[str] = set()
    for collision_object in collision_objects:
        object_id = getattr(collision_object, "id", None)
        if not isinstance(object_id, str):
            raise PlanningSceneConfigurationError(
                "MoveIt world-object verification returned an invalid object id"
            )
        present_ids.add(object_id)
    return tuple(sorted(present_ids.intersection(remove_object_ids)))


def _build_marker_messages(definition: PlanningSceneDefinition, Marker: Any) -> tuple[Any, ...]:
    """Build an idempotent clear-and-add marker sequence for one environment."""

    clear = Marker()
    clear.header.frame_id = definition.frame_id
    clear.action = Marker.DELETEALL
    messages = [clear]
    for plane in definition.visualization_planes:
        marker = Marker()
        marker.header.frame_id = plane.frame_id
        marker.ns = plane.namespace
        marker.id = plane.marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
            plane.position_m
        )
        (
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w,
        ) = plane.quaternion_xyzw
        marker.scale.x, marker.scale.y = plane.size_xy_m
        marker.scale.z = _VISUALIZATION_PLANE_THICKNESS_M
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = plane.color_rgba
        marker.frame_locked = True
        messages.append(marker)

        label = Marker()
        label.header.frame_id = plane.frame_id
        label.ns = plane.namespace
        label.id = plane.marker_id + 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x, label.pose.position.y, label.pose.position.z = (
            plane.label_position_m
        )
        label.pose.orientation.w = 1.0
        label.scale.z = 0.018
        label.color.r, label.color.g, label.color.b, _ = plane.color_rgba
        label.color.a = 1.0
        label.text = plane.label
        label.frame_locked = True
        messages.append(label)
    return tuple(messages)


def main(args: list[str] | None = None) -> int:
    """Apply the selected collision scene and publish its non-collision RViz markers."""

    try:
        import rclpy
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import (
            AttachedCollisionObject,
            CollisionObject,
            PlanningScene,
            PlanningSceneComponents,
        )
        from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from rclpy.time import Time
        from sensor_msgs.msg import JointState
        from shape_msgs.msg import SolidPrimitive
        from tf2_ros import Buffer, TransformListener
        from visualization_msgs.msg import Marker
    except ImportError as error:
        print(f"hx-waypoint planning scene requires ROS 2 MoveIt messages: {error}")
        return 2

    rclpy.init(args=args)
    node = Node("hx_waypoint_planning_scene")
    exit_code = 0
    try:
        workspace = str(node.declare_parameter("workspace", "").value).strip()
        environment_id = str(node.declare_parameter("environment_id", "").value).strip()
        service_timeout_s = float(node.declare_parameter("service_timeout_s", 30.0).value)
        response_timeout_s = float(node.declare_parameter("response_timeout_s", 10.0).value)
        reapply_period_s = float(node.declare_parameter("reapply_period_s", 10.0).value)
        for parameter_name, value in (
            ("service_timeout_s", service_timeout_s),
            ("response_timeout_s", response_timeout_s),
            ("reapply_period_s", reapply_period_s),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise PlanningSceneConfigurationError(
                    f"parameter {parameter_name} must be finite and greater than zero"
                )

        definition = load_planning_scene_definition(workspace, environment_id)
        ros_types = {
            "ApplyPlanningScene": ApplyPlanningScene,
            "GetPlanningScene": GetPlanningScene,
            "AttachedCollisionObject": AttachedCollisionObject,
            "CollisionObject": CollisionObject,
            "PlanningScene": PlanningScene,
            "PlanningSceneComponents": PlanningSceneComponents,
            "Pose": Pose,
            "SolidPrimitive": SolidPrimitive,
        }
        client = node.create_client(ApplyPlanningScene, APPLY_PLANNING_SCENE_SERVICE)
        verification_client = node.create_client(
            GetPlanningScene,
            GET_PLANNING_SCENE_SERVICE,
        )
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node, spin_thread=False)
        marker_messages = _build_marker_messages(definition, Marker)
        marker_publisher = node.create_publisher(
            Marker,
            ENVIRONMENT_MARKER_TOPIC,
            QoSProfile(
                depth=max(10, len(marker_messages)),
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        first_application = True
        while rclpy.ok():
            if not client.wait_for_service(timeout_sec=service_timeout_s):
                raise RuntimeError(
                    f"MoveIt service {APPLY_PLANNING_SCENE_SERVICE} was unavailable for "
                    f"{service_timeout_s:.1f} s"
                )
            if first_application:
                _wait_for_attached_scene_inputs(
                    node=node,
                    rclpy=rclpy,
                    JointState=JointState,
                    qos_profile_sensor_data=qos_profile_sensor_data,
                    tf_buffer=tf_buffer,
                    Time=Time,
                    definition=definition,
                    timeout_s=service_timeout_s,
                )
            for step in _build_application_steps(
                definition,
                ros_types,
                first_application=first_application,
            ):
                future = client.call_async(step.request)
                rclpy.spin_until_future_complete(
                    node,
                    future,
                    timeout_sec=response_timeout_s,
                )
                if not future.done():
                    future.cancel()
                    raise RuntimeError(
                        f"MoveIt {step.label} did not respond within "
                        f"{response_timeout_s:.1f} s"
                    )
                response = future.result()
                if response is None:
                    raise RuntimeError(f"MoveIt {step.label} returned no response")
                if not bool(response.success):
                    if step.success_required:
                        raise RuntimeError(
                            f"MoveIt rejected the {definition.environment_id} "
                            "planning scene"
                        )
                    node.get_logger().info(
                        "legacy collision objects were already absent or retained; "
                        "verifying the authoritative world state"
                    )
            if first_application:
                if definition.remove_object_ids:
                    if not verification_client.wait_for_service(
                        timeout_sec=service_timeout_s
                    ):
                        raise RuntimeError(
                            f"MoveIt service {GET_PLANNING_SCENE_SERVICE} was unavailable "
                            f"for {service_timeout_s:.1f} s"
                        )
                    verification_future = verification_client.call_async(
                        _build_world_object_names_request(ros_types)
                    )
                    rclpy.spin_until_future_complete(
                        node,
                        verification_future,
                        timeout_sec=response_timeout_s,
                    )
                    if not verification_future.done():
                        verification_future.cancel()
                        raise RuntimeError(
                            "MoveIt world-object verification did not respond within "
                            f"{response_timeout_s:.1f} s"
                        )
                    verification_response = verification_future.result()
                    if verification_response is None:
                        raise RuntimeError(
                            "MoveIt world-object verification returned no response"
                        )
                    remaining_legacy_ids = _remaining_legacy_object_ids(
                        verification_response,
                        definition.remove_object_ids,
                    )
                    if remaining_legacy_ids:
                        raise RuntimeError(
                            "MoveIt retained legacy collision objects after cleanup: "
                            + ", ".join(remaining_legacy_ids)
                        )
                for marker in marker_messages:
                    marker_publisher.publish(marker)
                node.get_logger().info(
                    f"applied {len(definition.cuboids)} collision cuboids for "
                    f"environment {definition.environment_id!r} in frame "
                    f"{definition.frame_id!r} and attached rod "
                    f"{definition.attached_rod.object_id!r} to "
                    f"{definition.attached_rod.link_name!r}; published "
                    f"{len(definition.visualization_planes)} non-collision RViz planes"
                )
                first_application = False

            # Keep the listener alive for the lifetime of the node.  The name
            # also documents that scene reapplication shares this TF buffer.
            _ = tf_listener

            deadline = time.monotonic() + reapply_period_s
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(
                    node,
                    timeout_sec=min(0.25, max(0.0, deadline - time.monotonic())),
                )
    except KeyboardInterrupt:
        pass
    except (PlanningSceneConfigurationError, RuntimeError, OSError, ValueError) as error:
        node.get_logger().fatal(str(error))
        exit_code = 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
