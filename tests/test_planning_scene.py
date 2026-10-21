from __future__ import annotations

import copy
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from conftest import copy_environment_without_recordings

from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_curobo.scene_config import (
    SceneConfigurationError,
    resolve_curobo_cuboids,
    resolved_curobo_scene,
)

from hx_waypoint_tools.planning_scene_publisher import (
    ATTACHED_ROD_OBJECT_ID,
    ENVIRONMENT_MARKER_TOPIC,
    GET_PLANNING_SCENE_SERVICE,
    PlanningSceneConfigurationError,
    _build_application_steps,
    _build_apply_request,
    _build_marker_messages,
    _build_remove_request,
    _build_world_object_names_request,
    _complete_joint_positions,
    _remaining_legacy_object_ids,
    load_planning_scene_definition,
)


def test_environment_cuboids_match_curobo_scene(repository_root: Path) -> None:
    definition = load_planning_scene_definition(repository_root, "lab_wall_a")

    assert definition.environment_id == "lab_wall_a"
    assert definition.frame_id == "base_link"
    assert definition.joint_names == (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    )
    assert definition.remove_object_ids == ()
    assert [cuboid.object_id for cuboid in definition.cuboids] == [
        "floor",
        "rear_keepout",
    ]
    floor = definition.cuboids[0]
    assert floor.dimensions_m == (2.0, 2.0, 0.10)
    assert floor.position_m == pytest.approx((0.0, 0.0, -0.10), abs=1.0e-12)
    assert floor.quaternion_xyzw == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1.0e-12)
    assert definition.visualization_planes == ()


def test_planning_scene_joint_state_gate_requires_all_finite_named_joints() -> None:
    required = ("joint_a", "joint_b", "joint_c")

    assert _complete_joint_positions(
        SimpleNamespace(
            name=["extra", "joint_c", "joint_a", "joint_b"],
            position=[9.0, 3.0, 1.0, 2.0],
        ),
        required,
    ) == (1.0, 2.0, 3.0)
    assert _complete_joint_positions(
        SimpleNamespace(name=["joint_a", "joint_b"], position=[1.0, 2.0]),
        required,
    ) is None
    assert _complete_joint_positions(
        SimpleNamespace(
            name=["joint_a", "joint_b", "joint_c"],
            position=[1.0, float("nan"), 3.0],
        ),
        required,
    ) is None
    assert _complete_joint_positions(
        SimpleNamespace(
            name=["joint_a", "joint_a", "joint_c"],
            position=[1.0, 2.0, 3.0],
        ),
        required,
    ) is None
    assert _complete_joint_positions(
        SimpleNamespace(
            name=["joint_a", "joint_b", "joint_c"],
            position=[1.0, 2.0],
        ),
        required,
    ) is None


def test_new_board_attached_rod_is_derived_from_authoritative_setup(
    repository_root: Path,
) -> None:
    definition = load_planning_scene_definition(
        repository_root,
        "new_board_4cm_rod33365",
    )

    rod = definition.attached_rod
    assert definition.remove_object_ids == ("floor", "rear_keepout")
    assert rod.object_id == ATTACHED_ROD_OBJECT_ID
    assert rod.link_name == "tool0"
    assert rod.length_m == pytest.approx(0.33365, abs=1.0e-12)
    assert rod.radius_m == pytest.approx(0.0025, abs=1.0e-12)
    assert rod.axis_in_link_xyz == pytest.approx((0.0, 0.0, 1.0), abs=1.0e-12)
    assert rod.center_in_link_m == pytest.approx(
        (0.0, 0.0, 0.33365 / 2.0),
        abs=1.0e-12,
    )
    assert rod.quaternion_xyzw == pytest.approx(
        (0.0, 0.0, 0.0, 1.0),
        abs=1.0e-12,
    )
    assert rod.touch_links == ("tool0",)

    raw_plane, board_plane, actual_plane = definition.visualization_planes
    assert raw_plane.namespace == "raw_ur_feature"
    assert raw_plane.frame_id == "base"
    assert raw_plane.size_xy_m == (0.1, 0.1)
    assert raw_plane.position_m == pytest.approx(
        (-0.06528, -0.46973, 0.18244),
        abs=1.0e-12,
    )
    assert raw_plane.quaternion_xyzw == pytest.approx(
        (
            0.3692189854076563,
            0.9293397736045651,
            -5.1001500185811107e-05,
            0.0022188758422972935,
        ),
        abs=1.0e-12,
    )
    assert raw_plane.color_rgba == (1.0, 0.55, 0.0, 0.55)
    assert raw_plane.label == "ORANGE_raw_Feature_reference:new_board_4cm"
    assert raw_plane.label_position_m != raw_plane.position_m
    assert board_plane.namespace == "calibration_board_plane"
    assert board_plane.frame_id == "plane1"
    assert board_plane.size_xy_m == (0.4, 0.4)
    assert board_plane.position_m == (0.0, 0.0, 0.0)
    assert board_plane.quaternion_xyzw == (0.0, 0.0, 0.0, 1.0)
    assert board_plane.color_rgba == (0.85, 0.2, 1.0, 0.55)
    assert board_plane.label == "PURPLE_Feature_reference_z=0_not_sensor"
    assert board_plane.label_position_m == (-0.25, -0.25, -0.03)
    assert actual_plane.namespace == "actual_sensor_plane"
    assert actual_plane.frame_id == "plane1"
    assert actual_plane.size_xy_m == (0.4, 0.4)
    assert actual_plane.position_m == pytest.approx((0.0, 0.0, 0.043), abs=1.0e-12)
    assert actual_plane.quaternion_xyzw == pytest.approx(
        (1.0, 0.0, 0.0, 0.0),
        abs=1.0e-12,
    )
    assert actual_plane.color_rgba == (0.0, 0.75, 1.0, 0.75)
    assert actual_plane.label == "CYAN_sensor_z=+43mm"
    assert actual_plane.label_position_m == pytest.approx(
        (0.25, -0.25, 0.013),
        abs=1.0e-12,
    )


def test_attached_rod_cylinder_orientation_follows_a_general_setup_axis(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    environment_id = "planning_scene_rotated_rod"
    target = tmp_path / "environments" / environment_id
    copy_environment_without_recordings(
        repository_root / "environments" / "lab_wall_a",
        target,
    )
    environment_path = target / "environment.yaml"
    environment = yaml.safe_load(environment_path.read_text(encoding="utf-8"))
    environment["environment_id"] = environment_id
    environment_path.write_text(yaml.safe_dump(environment), encoding="utf-8")
    setup_path = target / "setup.yaml"
    setup = yaml.safe_load(setup_path.read_text(encoding="utf-8"))
    setup["tool"]["rod"]["axis_in_parent_xyz"] = [1.0, 0.0, 0.0]
    setup_path.write_text(yaml.safe_dump(setup), encoding="utf-8")

    rod = load_planning_scene_definition(tmp_path, environment_id).attached_rod

    assert rod.axis_in_link_xyz == (1.0, 0.0, 0.0)
    assert rod.center_in_link_m == pytest.approx((0.132, 0.0, 0.0), abs=1.0e-12)
    assert rod.quaternion_xyzw == pytest.approx(
        (0.0, 2.0**-0.5, 0.0, 2.0**-0.5),
        abs=1.0e-12,
    )


def _environment_with_zones(tmp_path: Path, repository_root: Path, zones: dict):
    target = tmp_path / "lab_wall_a"
    copy_environment_without_recordings(
        repository_root / "environments" / "lab_wall_a",
        target,
    )
    zones_path = target / "zones.yaml"
    payload = yaml.safe_load(zones_path.read_text(encoding="utf-8"))
    payload["curobo_scene"] = zones["curobo_scene"]
    zones_path.write_text(yaml.safe_dump(payload), encoding="utf-8")
    return EnvironmentBuilder().load(target)


def test_curobo_wxyz_pose_is_converted_to_ros_xyzw(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    environment = _environment_with_zones(
        tmp_path,
        repository_root,
        {
            "schema_version": 1,
            "curobo_scene": {
                "cuboid": {
                    "rotated_box": {
                        "frame_id": "base_link",
                        "dims": [0.1, 0.2, 0.3],
                        "pose": [1.0, 2.0, 3.0, 0.5, 0.5, -0.5, 0.5],
                    }
                }
            },
        },
    )

    first = resolve_curobo_cuboids(environment)
    second = resolve_curobo_cuboids(environment)

    assert first == second
    assert first[0].position_m == (1.0, 2.0, 3.0)
    assert first[0].quaternion_xyzw == (0.5, -0.5, 0.5, 0.5)


@pytest.mark.parametrize(
    ("dims", "pose", "match"),
    [
        ([0.1, 0.0, 0.3], [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], "positive"),
        ([0.1, 0.2], [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], "exactly 3"),
        ([0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0], "normalized"),
        ([0.1, 0.2, 0.3], [0.0, 0.0, float("nan"), 1.0, 0.0, 0.0, 0.0], "non-finite"),
    ],
)
def test_malformed_cuboid_is_rejected(
    dims,
    pose,
    match: str,
    tmp_path: Path,
    repository_root: Path,
) -> None:
    zones = {
        "schema_version": 1,
        "curobo_scene": {
            "cuboid": {
                "unsafe": {"frame_id": "base_link", "dims": dims, "pose": pose}
            },
        }
    }

    environment = _environment_with_zones(tmp_path, repository_root, zones)
    with pytest.raises(SceneConfigurationError, match=match):
        resolve_curobo_cuboids(environment)


def test_unsupported_curobo_geometry_is_rejected(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    zones = {
        "schema_version": 1,
        "curobo_scene": {
            "cuboid": {
                "floor": {
                    "frame_id": "base_link",
                    "dims": [1.0, 1.0, 0.1],
                    "pose": [0.0, 0.0, -0.1, 1.0, 0.0, 0.0, 0.0],
                }
            },
            "mesh": {"fixture": {"file_path": "fixture.stl"}},
        }
    }

    environment = _environment_with_zones(tmp_path, repository_root, zones)
    with pytest.raises(SceneConfigurationError, match="mesh"):
        resolve_curobo_cuboids(environment)


def test_environment_path_and_identity_are_fail_closed(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    with pytest.raises(PlanningSceneConfigurationError, match="invalid environment_id"):
        load_planning_scene_definition(repository_root, "../lab_wall_a")

    environment_root = tmp_path / "environments" / "copied_environment"
    source = repository_root / "environments" / "lab_wall_a"
    copy_environment_without_recordings(source, environment_root)
    with pytest.raises(PlanningSceneConfigurationError, match="does not match"):
        load_planning_scene_definition(tmp_path, "copied_environment")


def test_plane1_objects_are_resolved_once_for_moveit_and_curobo(
    repository_root: Path,
) -> None:
    environment = EnvironmentBuilder().load(repository_root / "environments" / "lab_wall_a")
    resolved = resolve_curobo_cuboids(environment)
    curobo = resolved_curobo_scene(environment)
    moveit = load_planning_scene_definition(repository_root, "lab_wall_a")

    assert all(cuboid.source_frame_id == "plane1" for cuboid in resolved)
    for resolved_cuboid, moveit_cuboid in zip(resolved, moveit.cuboids, strict=True):
        curobo_cuboid = curobo["cuboid"][resolved_cuboid.object_id]
        assert moveit_cuboid.frame_id == environment.base_frame
        assert moveit_cuboid.position_m == pytest.approx(
            resolved_cuboid.position_m,
            abs=1.0e-12,
        )
        assert curobo_cuboid["pose"] == pytest.approx(
            resolved_cuboid.curobo_pose_wxyz,
            abs=1.0e-12,
        )


class _ApplyPlanningScene:
    class Request:
        def __init__(self) -> None:
            self.scene = None


class _PlanningSceneComponents:
    WORLD_OBJECT_NAMES = 8

    def __init__(self) -> None:
        self.components = 0


class _GetPlanningScene:
    class Request:
        def __init__(self) -> None:
            self.components = _PlanningSceneComponents()


class _PlanningScene:
    def __init__(self) -> None:
        self.is_diff = False
        self.robot_state = SimpleNamespace(
            is_diff=False,
            attached_collision_objects=[],
        )
        self.world = SimpleNamespace(collision_objects=[])


class _CollisionObject:
    ADD = 0
    REMOVE = 1

    def __init__(self) -> None:
        self.header = SimpleNamespace(frame_id="")
        self.id = ""
        self.operation = -1
        self.primitives = []
        self.primitive_poses = []


class _AttachedCollisionObject:
    def __init__(self) -> None:
        self.link_name = ""
        self.object = None
        self.touch_links = []


class _SolidPrimitive:
    BOX = 1
    CYLINDER = 3

    def __init__(self) -> None:
        self.type = 0
        self.dimensions = []


class _Pose:
    def __init__(self) -> None:
        self.position = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0)


class _Marker:
    ADD = 0
    CUBE = 1
    DELETEALL = 3
    TEXT_VIEW_FACING = 9

    def __init__(self) -> None:
        self.header = SimpleNamespace(frame_id="")
        self.ns = ""
        self.id = -1
        self.type = -1
        self.action = -1
        self.pose = _Pose()
        self.scale = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.color = SimpleNamespace(r=0.0, g=0.0, b=0.0, a=0.0)
        self.text = ""
        self.frame_locked = False


def _fake_ros_types():
    return {
        "ApplyPlanningScene": _ApplyPlanningScene,
        "GetPlanningScene": _GetPlanningScene,
        "AttachedCollisionObject": _AttachedCollisionObject,
        "CollisionObject": _CollisionObject,
        "PlanningScene": _PlanningScene,
        "PlanningSceneComponents": _PlanningSceneComponents,
        "Pose": _Pose,
        "SolidPrimitive": _SolidPrimitive,
    }


def test_new_board_apply_request_contains_the_configured_attached_rod(
    repository_root: Path,
) -> None:
    definition = load_planning_scene_definition(
        repository_root,
        "new_board_4cm_rod33365",
    )

    request = _build_apply_request(definition, _fake_ros_types())

    assert [item.id for item in request.scene.world.collision_objects] == [
        "base_ground_fixed_base_support",
        "base_ground_x_negative",
        "base_ground_x_positive",
        "base_ground_y_negative",
        "base_ground_y_positive",
        "sensor_board_keepout",
        "top_negative_z_panel",
        "x_positive_limit_wall",
        "y_positive_limit_wall",
    ]
    assert [item.operation for item in request.scene.world.collision_objects] == [
        _CollisionObject.ADD,
        _CollisionObject.ADD,
        _CollisionObject.ADD,
        _CollisionObject.ADD,
        _CollisionObject.ADD,
        _CollisionObject.ADD,
        _CollisionObject.ADD,
        _CollisionObject.ADD,
        _CollisionObject.ADD,
    ]
    attached = request.scene.robot_state.attached_collision_objects
    assert len(attached) == 1
    assert attached[0].link_name == "tool0"
    assert attached[0].object.primitives[0].dimensions == [0.33365, 0.0025]
    pose = attached[0].object.primitive_poses[0]
    assert (pose.position.x, pose.position.y, pose.position.z) == pytest.approx(
        (0.0, 0.0, 0.33365 / 2.0),
        abs=1.0e-12,
    )
    assert all(
        item.id
        not in {"raw_ur_feature", "calibration_board_plane", "actual_sensor_plane"}
        for item in request.scene.world.collision_objects
    )


def test_new_board_remove_request_is_separate_and_idempotent(
    repository_root: Path,
) -> None:
    definition = load_planning_scene_definition(
        repository_root,
        "new_board_4cm_rod33365",
    )

    request = _build_remove_request(definition, _fake_ros_types())

    assert [item.id for item in request.scene.world.collision_objects] == [
        "floor",
        "rear_keepout",
    ]
    assert all(
        item.operation == _CollisionObject.REMOVE
        for item in request.scene.world.collision_objects
    )
    assert request.scene.robot_state.attached_collision_objects == []


def test_new_board_scene_transition_adds_current_scene_before_legacy_cleanup(
    repository_root: Path,
) -> None:
    definition = load_planning_scene_definition(
        repository_root,
        "new_board_4cm_rod33365",
    )

    initial_steps = _build_application_steps(
        definition,
        _fake_ros_types(),
        first_application=True,
    )
    assert [step.label for step in initial_steps] == [
        "current planning scene",
        "legacy-object cleanup",
    ]
    assert [step.success_required for step in initial_steps] == [True, False]
    assert all(
        item.operation == _CollisionObject.ADD
        for item in initial_steps[0].request.scene.world.collision_objects
    )
    assert all(
        item.operation == _CollisionObject.REMOVE
        for item in initial_steps[1].request.scene.world.collision_objects
    )

    reapply_steps = _build_application_steps(
        definition,
        _fake_ros_types(),
        first_application=False,
    )
    assert [step.label for step in reapply_steps] == ["current planning scene"]


def test_new_board_cleanup_is_confirmed_from_the_authoritative_moveit_world(
    repository_root: Path,
) -> None:
    definition = load_planning_scene_definition(
        repository_root,
        "new_board_4cm_rod33365",
    )
    request = _build_world_object_names_request(_fake_ros_types())

    assert GET_PLANNING_SCENE_SERVICE == "/get_planning_scene"
    assert request.components.components == _PlanningSceneComponents.WORLD_OBJECT_NAMES

    absent_response = SimpleNamespace(
        scene=SimpleNamespace(
            world=SimpleNamespace(
                collision_objects=[SimpleNamespace(id="sensor_board_keepout")]
            )
        )
    )
    assert _remaining_legacy_object_ids(
        absent_response,
        definition.remove_object_ids,
    ) == ()

    retained_response = SimpleNamespace(
        scene=SimpleNamespace(
            world=SimpleNamespace(
                collision_objects=[
                    SimpleNamespace(id="rear_keepout"),
                    SimpleNamespace(id="floor"),
                ]
            )
        )
    )
    assert _remaining_legacy_object_ids(
        retained_response,
        definition.remove_object_ids,
    ) == ("floor", "rear_keepout")


def test_new_board_visualization_markers_are_transient_topic_payloads_only(
    repository_root: Path,
) -> None:
    definition = load_planning_scene_definition(
        repository_root,
        "new_board_4cm_rod33365",
    )

    (
        clear,
        raw,
        raw_label,
        board,
        board_label,
        actual,
        actual_label,
    ) = _build_marker_messages(definition, _Marker)

    assert ENVIRONMENT_MARKER_TOPIC == "/hx_waypoint/environment_markers"
    assert clear.header.frame_id == "base_link"
    assert clear.action == _Marker.DELETEALL
    assert raw.action == board.action == actual.action == _Marker.ADD
    assert raw.type == board.type == actual.type == _Marker.CUBE
    assert (
        raw_label.type
        == board_label.type
        == actual_label.type
        == _Marker.TEXT_VIEW_FACING
    )
    assert (raw.ns, raw.id, raw.header.frame_id) == (
        "raw_ur_feature",
        0,
        "base",
    )
    assert (board.ns, board.id, board.header.frame_id) == (
        "calibration_board_plane",
        0,
        "plane1",
    )
    assert (actual.ns, actual.id, actual.header.frame_id) == (
        "actual_sensor_plane",
        0,
        "plane1",
    )
    assert (raw.scale.x, raw.scale.y, raw.scale.z) == (0.1, 0.1, 0.002)
    assert (board.scale.x, board.scale.y, board.scale.z) == (
        0.4,
        0.4,
        0.002,
    )
    assert (actual.scale.x, actual.scale.y, actual.scale.z) == (0.4, 0.4, 0.002)
    assert (actual.pose.position.x, actual.pose.position.y, actual.pose.position.z) == (
        0.0,
        0.0,
        0.043,
    )
    assert (raw.color.r, raw.color.g, raw.color.b, raw.color.a) == (
        1.0,
        0.55,
        0.0,
        0.55,
    )
    assert (actual.color.r, actual.color.g, actual.color.b, actual.color.a) == (
        0.0,
        0.75,
        1.0,
        0.75,
    )
    assert (board.color.r, board.color.g, board.color.b, board.color.a) == (
        0.85,
        0.2,
        1.0,
        0.55,
    )
    assert raw.frame_locked is board.frame_locked is actual.frame_locked is True
    assert raw_label.text == "ORANGE_raw_Feature_reference:new_board_4cm"
    assert board_label.text == "PURPLE_Feature_reference_z=0_not_sensor"
    assert actual_label.text == "CYAN_sensor_z=+43mm"
    assert raw_label.scale.z == board_label.scale.z == actual_label.scale.z == 0.018
    assert (
        actual_label.pose.position.x,
        actual_label.pose.position.y,
        actual_label.pose.position.z,
    ) == pytest.approx((0.25, -0.25, 0.013), abs=1.0e-12)
    assert (board_label.pose.position.x, board_label.pose.position.y) == (
        -0.25,
        -0.25,
    )
    assert (actual_label.pose.position.x, actual_label.pose.position.y) == (0.25, -0.25)
    assert raw_label.color.a == board_label.color.a == actual_label.color.a == 1.0
    assert raw_label.frame_locked is board_label.frame_locked is actual_label.frame_locked


def test_apply_request_is_an_idempotent_planning_scene_diff(repository_root: Path) -> None:
    definition = load_planning_scene_definition(repository_root, "lab_wall_a")
    ros_types = _fake_ros_types()

    first = _build_apply_request(definition, ros_types)
    second = _build_apply_request(copy.deepcopy(definition), ros_types)

    assert first.scene.is_diff is True
    assert first.scene.robot_state.is_diff is True
    assert [item.id for item in first.scene.world.collision_objects] == [
        item.id for item in second.scene.world.collision_objects
    ] == ["floor", "rear_keepout"]
    floor = first.scene.world.collision_objects[0]
    assert floor.header.frame_id == "base_link"
    assert floor.operation == _CollisionObject.ADD
    assert floor.primitives[0].type == _SolidPrimitive.BOX
    assert floor.primitives[0].dimensions == [2.0, 2.0, 0.1]
    pose = floor.primitive_poses[0]
    assert (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1.0e-12)

    assert len(first.scene.robot_state.attached_collision_objects) == 1
    assert len(second.scene.robot_state.attached_collision_objects) == 1
    first_attached = first.scene.robot_state.attached_collision_objects[0]
    second_attached = second.scene.robot_state.attached_collision_objects[0]
    assert first_attached.link_name == second_attached.link_name == "tool0"
    assert first_attached.touch_links == second_attached.touch_links == ["tool0"]
    assert (
        first_attached.object.id
        == second_attached.object.id
        == ATTACHED_ROD_OBJECT_ID
    )
    assert first_attached.object.header.frame_id == "tool0"
    assert first_attached.object.operation == _CollisionObject.ADD
    assert first_attached.object.primitives[0].type == _SolidPrimitive.CYLINDER
    assert first_attached.object.primitives[0].dimensions == [0.264, 0.008]
    rod_pose = first_attached.object.primitive_poses[0]
    assert (rod_pose.position.x, rod_pose.position.y, rod_pose.position.z) == pytest.approx(
        (0.0, 0.0, 0.132),
        abs=1.0e-12,
    )
    assert (
        rod_pose.orientation.x,
        rod_pose.orientation.y,
        rod_pose.orientation.z,
        rod_pose.orientation.w,
    ) == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1.0e-12)


@pytest.mark.parametrize("launch_name", ["simulation.launch.py", "real.launch.py"])
def test_bringup_launches_fail_closed_planning_scene_publisher(
    launch_name: str,
    repository_root: Path,
) -> None:
    launch_path = (
        repository_root / "src" / "hx_waypoint_bringup" / "launch" / launch_name
    )
    text = launch_path.read_text(encoding="utf-8")

    assert 'executable="hx_waypoint_planning_scene"' in text
    assert '"environment_id": environment_id' in text
    assert "OnProcessExit" in text
    assert "incomplete MoveIt scene" in text


def test_simulation_owns_rviz_marker_display_and_uses_transient_local_qos(
    repository_root: Path,
) -> None:
    publisher = (
        repository_root
        / "src"
        / "hx_waypoint_tools"
        / "hx_waypoint_tools"
        / "planning_scene_publisher.py"
    ).read_text(encoding="utf-8")
    launch = (
        repository_root
        / "src"
        / "hx_waypoint_bringup"
        / "launch"
        / "simulation.launch.py"
    ).read_text(encoding="utf-8")
    rviz = (
        repository_root
        / "src"
        / "hx_waypoint_bringup"
        / "config"
        / "hx_waypoint_moveit.rviz"
    ).read_text(encoding="utf-8")

    assert "DurabilityPolicy.TRANSIENT_LOCAL" in publisher
    assert 'ENVIRONMENT_MARKER_TOPIC = "/hx_waypoint/environment_markers"' in publisher
    assert "SetLaunchConfiguration" in launch
    assert '"launch_hx_waypoint_rviz"' in launch
    assert "moveit_group =" not in launch
    assert launch.count('"launch_rviz": "false"') >= 2
    assert 'package="rviz2"' in launch
    assert '"hx_waypoint_moveit.rviz"' in launch
    assert 'name="wait_for_hx_waypoint_rviz_description"' in launch
    assert "target_action=rviz_description_waiter" in launch
    assert "on_exit=[rviz]" in launch
    assert "Class: rviz_default_plugins/Marker" in rviz
    assert "      Topic:\n" in rviz
    assert "Marker Topic:" not in rviz
    assert "Durability Policy: Transient Local" in rviz
    assert "Value: /hx_waypoint/environment_markers" in rviz
    assert "raw_ur_feature: true" in rviz
    assert "calibration_board_plane: true" in rviz
    assert "actual_sensor_plane: true" in rviz
    assert "Scene Alpha: 0.25" in rviz
    assert "Distance: 2" in rviz
    assert "Y: 0.25" in rviz
    assert "Z: 0.25" in rviz
