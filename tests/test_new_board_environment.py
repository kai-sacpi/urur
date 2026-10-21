from __future__ import annotations

from xml.etree import ElementTree

import numpy as np
import pytest
import yaml

from conftest import copy_environment_without_recordings

from hx_waypoint_curobo.contact_policy import ContactPolicy
from hx_waypoint_curobo.curobo_adapter_v2 import CuroboAdapterV2
from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_curobo.errors import ConfigurationError
from hx_waypoint_curobo.math3d import pose_matrix
from hx_waypoint_curobo.robot_tool_builder import RobotToolBuilder
from hx_waypoint_curobo.scene_config import resolve_curobo_cuboids
from hx_waypoint_curobo.setup_config import rotvec_to_quaternion_xyzw


ENVIRONMENT_ID = "new_board_4cm_rod33365"


def _load(repository_root):
    return EnvironmentBuilder().load(
        repository_root / "environments" / ENVIRONMENT_ID
    )


def _copy_new_board(tmp_path, repository_root):
    return copy_environment_without_recordings(
        repository_root / "environments" / ENVIRONMENT_ID,
        tmp_path / ENVIRONMENT_ID,
    )


def test_new_board_environment_locks_the_installed_side_start_branch(
    repository_root,
) -> None:
    environment = _load(repository_root)
    expected = (
        -2.264215638612,
        -1.362229481182,
        -1.056971395008,
        -2.253045531399,
        1.573239787748,
        1.570796326795,
    )

    assert environment.home_joint_positions == pytest.approx(expected, abs=1.0e-12)
    assert environment.curobo_robot["robot_cfg"]["kinematics"]["cspace"][
        "default_joint_position"
    ] == pytest.approx(expected, abs=1.0e-12)
    assert environment.home_joint_positions[0] < 0.0
    assert environment.home_joint_positions[2] < 0.0
    assert environment.home_joint_positions[4] > 0.0
    assert environment.planner["ik"]["canonical_joint_ranges_rad"] == {
        "shoulder_pan_joint": [-3.1415926536, 0.0],
    }
    assert environment.recording["start_state"]["prepare_sequence"] == (
        "home_then_recording_start"
    )

    adapter = CuroboAdapterV2(environment, repository_root / "curobo.lock")
    opposite_branch = (
        1.1960208416,
        -1.5790282488,
        1.2391915321,
        -1.2310768366,
        -1.5663589239,
        0.3815635741,
    )
    installed_branch = expected
    installed_branch_positive_winding = (
        expected[0] + 2.0 * np.pi,
        *expected[1:],
    )
    mask = adapter._canonical_ik_branch_mask(
        np.asarray(
            [
                opposite_branch,
                installed_branch,
                installed_branch_positive_winding,
            ]
        )
    )
    assert mask.tolist() == [False, True, True]


def test_new_board_installed_side_branch_constraint_cannot_be_removed(
    tmp_path,
    repository_root,
) -> None:
    target = _copy_new_board(tmp_path, repository_root)
    planner_path = target / "planner.yaml"
    planner = yaml.safe_load(planner_path.read_text(encoding="utf-8"))
    del planner["ik"]["canonical_joint_ranges_rad"]
    planner_path.write_text(yaml.safe_dump(planner, sort_keys=False), encoding="utf-8")

    with pytest.raises(ConfigurationError, match="installed shoulder-pan IK branch"):
        EnvironmentBuilder().load(target)


def test_new_board_mandatory_startup_sequence_cannot_be_removed(
    tmp_path,
    repository_root,
) -> None:
    target = _copy_new_board(tmp_path, repository_root)
    recording_path = target / "recording.yaml"
    recording = yaml.safe_load(recording_path.read_text(encoding="utf-8"))
    recording["start_state"]["prepare_sequence"] = "direct_to_recording_start"
    recording_path.write_text(
        yaml.safe_dump(recording, sort_keys=False),
        encoding="utf-8",
    )

    with pytest.raises(ConfigurationError, match="home_then_recording_start"):
        EnvironmentBuilder().load(target)


def test_new_board_environment_uses_the_ur_feature_pose_directly(
    repository_root,
) -> None:
    environment = _load(repository_root)
    calibration = environment.environment["calibration"]

    assert environment.environment_id == ENVIRONMENT_ID
    assert calibration["dashboard"] == {
        "robot_ip": "192.168.1.99",
        "variable": "new_board_4cm",
        "source_type": "ur_installation_plane_feature",
        "pose_semantics": "base_to_feature",
        "raw_base_to_feature_pose_xyz_rotvec": [
            -0.06528,
            -0.46973,
            0.18244,
            1.1583,
            2.91549,
            -0.00016,
        ],
    }
    assert calibration["tf_binding"] == {
        "policy": "direct_ur_feature_pose",
        "parent_frame": "base",
        "child_frame": "plane1",
        "translation_policy": "unchanged",
        "rotation_policy": "ur_rotation_vector_to_quaternion_xyzw",
        "orientation_correction": "none",
        "wall_used_as_pose_reference": False,
        "setup_pose_must_match_dashboard_pose": True,
    }
    raw_pose = tuple(calibration["dashboard"]["raw_base_to_feature_pose_xyz_rotvec"])
    assert environment.setup_config.work_frame.teach_pendant_pose_xyz_rotvec == (
        -0.06528,
        -0.46973,
        0.18244,
        1.1583,
        2.91549,
        -0.00016,
    )
    assert environment.setup_config.work_frame.teach_pendant_pose_xyz_rotvec == raw_pose
    direct_transform = environment.setup_config.work_frame.parent_to_child
    assert direct_transform.translation_m == raw_pose[:3]
    assert direct_transform.quaternion_xyzw == pytest.approx(
        (
            0.3692189854076563,
            0.9293397736045651,
            -5.1001500185811107e-05,
            0.0022188758422972935,
        ),
        abs=1.0e-15,
    )
    assert pose_matrix(
        direct_transform.translation_m,
        direct_transform.quaternion_xyzw,
    ) == pytest.approx(
        pose_matrix(raw_pose[:3], rotvec_to_quaternion_xyzw(raw_pose[3:])),
        abs=1.0e-15,
    )
    base_link_matrix = environment._frame_to_base("plane1")
    assert base_link_matrix[:3, 3] == pytest.approx(
        (0.06528, 0.46973, 0.18244),
        abs=1.0e-12,
    )
    assert base_link_matrix[:3, 2] == pytest.approx(
        (-0.004086517701568664, 0.001733297619749481, -0.9999901479776869),
        abs=1.0e-12,
    )
    assert set(environment.frames["frames"]) == {
        "plane1",
        "new_board_4cm",
    }
    assert environment.frames["frames"]["new_board_4cm"] == {
        "parent": "plane1",
        "translation_m": [0.0, 0.0, 0.0],
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    sensor_geometry = calibration["sensor_geometry"]
    assert sensor_geometry["board_center_matches_feature_origin"] is True
    assert sensor_geometry["board_center_in_plane1_xy_m"] == [0.0, 0.0]
    assert (
        sensor_geometry["model_status"]
        == "board_center_and_workspace_boundaries_confirmed"
    )
    assert environment.environment["workspace_boundaries"] == {
        "frame_id": "new_board_4cm",
        "representation": "finite_collision_slabs_on_prohibited_side",
        "slab_thickness_m": 0.01,
        "finite_span_m": 2.0,
        "remove_collision_object_ids": ["floor", "rear_keepout"],
        "ground": {
            "frame_id": "base_link",
            "size_xy_m": [3.0, 3.0],
            "top_z_m": 0.0,
            "thickness_m": 0.10,
            "ur5_nominal_reach_m": 0.85,
            "fixed_base_cutout_size_xy_m": [0.24, 0.24],
            "fixed_base_support_recess_m": 0.0015,
        },
        "positive_x_limit_m": 0.60,
        "positive_y_limit_m": 0.70,
        "top_panel": {
            "boundary_z_m": -0.72,
            "x_range_m": [0.129, 0.294],
            "y_max_m": -0.10,
            "finite_y_min_m": -2.10,
        },
    }


def test_new_board_sensor_height_and_keepout_geometry_are_exact(
    repository_root,
) -> None:
    environment = _load(repository_root)
    base_from_plane = environment._frame_to_base("plane1")

    sensor_point = base_from_plane @ np.asarray([0.0, 0.0, 0.043, 1.0])
    assert sensor_point[:3] == pytest.approx(
        (0.0651042797388326, 0.4698045317976492, 0.13944042363695946),
        abs=1.0e-12,
    )

    cuboids = {item.object_id: item for item in resolve_curobo_cuboids(environment)}
    assert set(cuboids) == {
        "base_ground_fixed_base_support",
        "base_ground_x_negative",
        "base_ground_x_positive",
        "base_ground_y_negative",
        "base_ground_y_positive",
        "sensor_board_keepout",
        "top_negative_z_panel",
        "x_positive_limit_wall",
        "y_positive_limit_wall",
    }
    raw_cuboids = environment.zones["curobo_scene"]["cuboid"]
    assert "floor" not in raw_cuboids
    assert "rear_keepout" not in raw_cuboids
    expected_boundaries = {
        "base_ground_fixed_base_support": {
            "frame_id": "base_link",
            "dims": [0.24, 0.24, 0.0985],
            "pose": [0.0, 0.0, -0.05075, 1.0, 0.0, 0.0, 0.0],
        },
        "base_ground_x_negative": {
            "frame_id": "base_link",
            "dims": [1.38, 3.0, 0.10],
            "pose": [-0.81, 0.0, -0.05, 1.0, 0.0, 0.0, 0.0],
        },
        "base_ground_x_positive": {
            "frame_id": "base_link",
            "dims": [1.38, 3.0, 0.10],
            "pose": [0.81, 0.0, -0.05, 1.0, 0.0, 0.0, 0.0],
        },
        "base_ground_y_negative": {
            "frame_id": "base_link",
            "dims": [0.24, 1.38, 0.10],
            "pose": [0.0, -0.81, -0.05, 1.0, 0.0, 0.0, 0.0],
        },
        "base_ground_y_positive": {
            "frame_id": "base_link",
            "dims": [0.24, 1.38, 0.10],
            "pose": [0.0, 0.81, -0.05, 1.0, 0.0, 0.0, 0.0],
        },
        "x_positive_limit_wall": {
            "frame_id": "new_board_4cm",
            "dims": [0.01, 2.0, 2.0],
            "pose": [0.605, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        },
        "y_positive_limit_wall": {
            "frame_id": "new_board_4cm",
            "dims": [2.0, 0.01, 2.0],
            "pose": [0.0, 0.705, 0.0, 1.0, 0.0, 0.0, 0.0],
        },
        "top_negative_z_panel": {
            "frame_id": "new_board_4cm",
            "dims": [0.165, 2.0, 0.01],
            "pose": [0.2115, -1.10, -0.725, 1.0, 0.0, 0.0, 0.0],
        },
    }
    for name, expected in expected_boundaries.items():
        assert raw_cuboids[name] == expected
        resolved = cuboids[name]
        assert resolved.source_frame_id == expected["frame_id"]
        assert resolved.dimensions_m == tuple(expected["dims"])
        resolved_matrix = pose_matrix(
            resolved.position_m,
            resolved.quaternion_xyzw,
        )
        local_pose = expected["pose"]
        base_from_boundaries = environment._frame_to_base(expected["frame_id"])
        expected_matrix = base_from_boundaries @ pose_matrix(
            local_pose[:3],
            (local_pose[4], local_pose[5], local_pose[6], local_pose[3]),
        )
        assert resolved_matrix == pytest.approx(expected_matrix, abs=1.0e-12)
    board = cuboids["sensor_board_keepout"]
    assert board.source_frame_id == "plane1"
    assert board.dimensions_m == (0.40, 0.40, 0.16)
    assert board.position_m == pytest.approx(
        (0.06499394376089025, 0.46985133083338243, 0.1124406896415619),
        abs=1.0e-12,
    )
    board_matrix = pose_matrix(board.position_m, board.quaternion_xyzw)
    expected_board_matrix = base_from_plane @ pose_matrix(
        (0.0, 0.0, 0.07),
        (0.0, 0.0, 0.0, 1.0),
    )
    assert board_matrix == pytest.approx(
        expected_board_matrix,
        abs=1.0e-12,
    )


def test_new_board_tool_and_contact_boundaries_are_hash_bound(
    repository_root,
) -> None:
    environment = _load(repository_root)
    rod = environment.setup_config.tool

    assert rod.length_m == pytest.approx(0.33365)
    assert rod.radius_m == pytest.approx(0.0025)
    assert rod.sphere_spacing_m == pytest.approx(0.005)
    spheres = RobotToolBuilder(environment).collision_spheres()
    assert len(spheres) == 68
    centers = np.asarray([sphere.center_m[2] for sphere in spheres])
    assert np.max(np.diff(centers)) <= 0.005 + 1.0e-12
    assert centers[-1] + spheres[-1].radius_m == pytest.approx(
        0.33365,
        abs=1.0e-12,
    )

    policy = ContactPolicy.from_environment(environment)
    assert policy.allowed_surface == "sensor_keepout_clearance_boundary"
    assert policy.maximum_tip_penetration_m == 0.0
    assert policy.plane_normal_base == pytest.approx(
        (0.004086517701568664, -0.001733297619749481, 0.9999901479776869),
        abs=1.0e-12,
    )
    assert policy.plane_offset_base_m == pytest.approx(
        0.19689078858168266,
        abs=1.0e-12,
    )
    assert policy.surface_x_range_m == (-0.20, 0.20)
    assert policy.surface_y_range_m == (-0.20, 0.20)
    boundary = environment.zones["analysis_zones"][
        "sensor_keepout_clearance_boundary"
    ]
    assert boundary == {
        "frame_id": "plane1",
        "normal_xyz": [0.0, 0.0, -1.0],
        "offset_m": 0.015,
        "x_range_m": [-0.20, 0.20],
        "y_range_m": [-0.20, 0.20],
    }
    actual = environment.zones["analysis_zones"]["actual_sensor_plane"]
    assert actual == {
        "frame_id": "plane1",
        "normal_xyz": [0.0, 0.0, -1.0],
        "offset_m": -0.043,
    }
    assert {"environment.yaml", "setup.yaml", "zones.yaml"} <= set(
        environment.hashes
    )


def test_direct_feature_binding_rejects_setup_pose_drift(
    tmp_path,
    repository_root,
) -> None:
    target = _copy_new_board(tmp_path, repository_root)
    setup_path = target / "setup.yaml"
    setup = yaml.safe_load(setup_path.read_text(encoding="utf-8"))
    setup["work_frame"]["teach_pendant_pose_xyz_rotvec"][0] += 0.001
    setup_path.write_text(yaml.safe_dump(setup), encoding="utf-8")

    with pytest.raises(ConfigurationError, match="must exactly match"):
        EnvironmentBuilder().load(target)


def test_direct_feature_contract_cannot_be_removed_to_restore_legacy_pose(
    tmp_path,
    repository_root,
) -> None:
    target = _copy_new_board(tmp_path, repository_root)
    environment_path = target / "environment.yaml"
    environment = yaml.safe_load(environment_path.read_text(encoding="utf-8"))
    del environment["calibration"]["tf_binding"]
    environment_path.write_text(yaml.safe_dump(environment), encoding="utf-8")
    setup_path = target / "setup.yaml"
    setup = yaml.safe_load(setup_path.read_text(encoding="utf-8"))
    setup["work_frame"]["teach_pendant_pose_xyz_rotvec"] = [
        -0.06528,
        -0.46973,
        0.18244,
        0.0,
        3.141592653589793,
        0.0,
    ]
    setup_path.write_text(yaml.safe_dump(setup), encoding="utf-8")

    with pytest.raises(ConfigurationError, match="requires calibration.tf_binding"):
        EnvironmentBuilder().load(target)


def test_direct_feature_identity_metadata_cannot_drift(
    tmp_path,
    repository_root,
) -> None:
    target = _copy_new_board(tmp_path, repository_root)
    environment_path = target / "environment.yaml"
    environment = yaml.safe_load(environment_path.read_text(encoding="utf-8"))
    environment["environment_id"] = "renamed_board_environment"
    environment_path.write_text(yaml.safe_dump(environment), encoding="utf-8")

    with pytest.raises(ConfigurationError, match="environment_id must remain"):
        EnvironmentBuilder().load(target)

    environment["environment_id"] = ENVIRONMENT_ID
    environment["calibration"]["dashboard"]["variable"] = "other_feature"
    environment_path.write_text(yaml.safe_dump(environment), encoding="utf-8")

    with pytest.raises(ConfigurationError, match="variable must remain"):
        EnvironmentBuilder().load(target)

    environment["calibration"]["dashboard"]["variable"] = "new_board_4cm"
    environment["workspace_boundaries"]["frame_id"] = "base_link"
    environment_path.write_text(yaml.safe_dump(environment), encoding="utf-8")

    with pytest.raises(ConfigurationError, match="frame_id must remain"):
        EnvironmentBuilder().load(target)

    environment["workspace_boundaries"]["frame_id"] = "new_board_4cm"
    environment_path.write_text(yaml.safe_dump(environment), encoding="utf-8")
    setup_path = target / "setup.yaml"
    setup = yaml.safe_load(setup_path.read_text(encoding="utf-8"))
    setup["work_frame"]["aliases"] = []
    setup_path.write_text(yaml.safe_dump(setup), encoding="utf-8")

    with pytest.raises(ConfigurationError, match="aliases must retain"):
        EnvironmentBuilder().load(target)


def test_confirmed_board_center_and_sensor_offset_cannot_drift(
    tmp_path,
    repository_root,
) -> None:
    target = _copy_new_board(tmp_path, repository_root)
    environment_path = target / "environment.yaml"
    environment = yaml.safe_load(environment_path.read_text(encoding="utf-8"))
    environment["calibration"]["sensor_geometry"][
        "board_center_in_plane1_xy_m"
    ] = [0.01, 0.0]
    environment_path.write_text(yaml.safe_dump(environment), encoding="utf-8")

    with pytest.raises(ConfigurationError, match="must be exactly"):
        EnvironmentBuilder().load(target)

    environment["calibration"]["sensor_geometry"][
        "board_center_in_plane1_xy_m"
    ] = [0.0, 0.0]
    environment_path.write_text(yaml.safe_dump(environment), encoding="utf-8")
    zones_path = target / "zones.yaml"
    zones = yaml.safe_load(zones_path.read_text(encoding="utf-8"))
    zones["analysis_zones"]["actual_sensor_plane"]["offset_m"] = -0.04
    zones_path.write_text(yaml.safe_dump(zones), encoding="utf-8")

    with pytest.raises(ConfigurationError, match=r"configured local \+Z"):
        EnvironmentBuilder().load(target)


def test_sensor_geometry_contract_rejects_coherent_dimension_changes(
    tmp_path,
    repository_root,
) -> None:
    target = _copy_new_board(tmp_path, repository_root)
    environment_path = target / "environment.yaml"
    environment = yaml.safe_load(environment_path.read_text(encoding="utf-8"))
    sensor = environment["calibration"]["sensor_geometry"]
    sensor["actual_sensor_plane_local_z_m"] = 0.05
    sensor["keepout_local_z_min_m"] = -0.02
    sensor["keepout_local_z_max_m"] = 0.20
    environment_path.write_text(yaml.safe_dump(environment), encoding="utf-8")
    zones_path = target / "zones.yaml"
    zones = yaml.safe_load(zones_path.read_text(encoding="utf-8"))
    zones["analysis_zones"]["actual_sensor_plane"]["offset_m"] = -0.05
    keepout = zones["curobo_scene"]["cuboid"]["sensor_board_keepout"]
    keepout["dims"] = [0.40, 0.40, 0.22]
    keepout["pose"] = [0.0, 0.0, 0.09, 1.0, 0.0, 0.0, 0.0]
    zones_path.write_text(yaml.safe_dump(zones), encoding="utf-8")

    with pytest.raises(ConfigurationError, match=r"exactly local z=\+0.043 m"):
        EnvironmentBuilder().load(target)

    sensor["actual_sensor_plane_local_z_m"] = 0.043
    sensor["keepout_local_z_min_m"] = -0.01
    sensor["keepout_local_z_max_m"] = 0.15
    sensor["board_size_xy_m"] = [0.50, 0.50]
    environment_path.write_text(yaml.safe_dump(environment), encoding="utf-8")
    keepout["dims"] = [0.50, 0.50, 0.16]
    keepout["pose"] = [0.0, 0.0, 0.07, 1.0, 0.0, 0.0, 0.0]
    zones["analysis_zones"]["actual_sensor_plane"]["offset_m"] = -0.043
    zones_path.write_text(yaml.safe_dump(zones), encoding="utf-8")

    with pytest.raises(ConfigurationError, match=r"exactly \[0.40, 0.40\]"):
        EnvironmentBuilder().load(target)


def test_workspace_boundary_contract_rejects_legacy_or_shifted_walls(
    tmp_path,
    repository_root,
) -> None:
    target = _copy_new_board(tmp_path, repository_root)
    zones_path = target / "zones.yaml"
    zones = yaml.safe_load(zones_path.read_text(encoding="utf-8"))
    cuboids = zones["curobo_scene"]["cuboid"]
    cuboids["floor"] = {
        "frame_id": "base_link",
        "dims": [2.0, 2.0, 0.10],
        "pose": [0.0, 0.0, -0.10, 1.0, 0.0, 0.0, 0.0],
    }
    zones_path.write_text(yaml.safe_dump(zones), encoding="utf-8")

    with pytest.raises(ConfigurationError, match="contain exactly"):
        EnvironmentBuilder().load(target)

    del cuboids["floor"]
    cuboids["x_positive_limit_wall"]["pose"][0] = 0.655
    zones_path.write_text(yaml.safe_dump(zones), encoding="utf-8")

    with pytest.raises(ConfigurationError, match="pose does not match"):
        EnvironmentBuilder().load(target)

    cuboids["x_positive_limit_wall"]["pose"][0] = 0.605
    zones_path.write_text(yaml.safe_dump(zones), encoding="utf-8")
    environment_path = target / "environment.yaml"
    environment = yaml.safe_load(environment_path.read_text(encoding="utf-8"))
    environment["workspace_boundaries"]["positive_x_limit_m"] = 0.65
    environment_path.write_text(yaml.safe_dump(environment), encoding="utf-8")

    with pytest.raises(ConfigurationError, match="positive_x_limit_m must be exactly"):
        EnvironmentBuilder().load(target)


def test_ground_top_face_is_locked_to_the_robot_base_plane(
    tmp_path,
    repository_root,
) -> None:
    target = _copy_new_board(tmp_path, repository_root)
    zones_path = target / "zones.yaml"
    zones = yaml.safe_load(zones_path.read_text(encoding="utf-8"))
    zones["curobo_scene"]["cuboid"]["base_ground_x_negative"]["pose"][2] = -0.10
    zones_path.write_text(yaml.safe_dump(zones), encoding="utf-8")

    with pytest.raises(
        ConfigurationError,
        match="base_ground_x_negative.pose does not match",
    ):
        EnvironmentBuilder().load(target)

    zones["curobo_scene"]["cuboid"]["base_ground_x_negative"]["pose"][2] = -0.05
    zones_path.write_text(yaml.safe_dump(zones), encoding="utf-8")
    environment_path = target / "environment.yaml"
    environment = yaml.safe_load(environment_path.read_text(encoding="utf-8"))
    environment["workspace_boundaries"]["ground"]["top_z_m"] = -0.05
    environment_path.write_text(yaml.safe_dump(environment), encoding="utf-8")

    with pytest.raises(ConfigurationError, match="ground.top_z_m must be exactly 0.0"):
        EnvironmentBuilder().load(target)


def test_ground_support_clears_the_fixed_base_without_creating_free_space(
    repository_root,
) -> None:
    environment = _load(repository_root)
    ground = environment.environment["workspace_boundaries"]["ground"]
    cutout_half_width = 0.5 * ground["fixed_base_cutout_size_xy_m"][0]
    minimum_scene_clearance = environment.planner["validation"][
        "minimum_collision_distance_m"
    ]
    shoulder_sphere = environment.collision_spheres["collision_spheres"][
        "shoulder_link"
    ][0]

    inflated_shoulder_radius = shoulder_sphere["radius"] + minimum_scene_clearance
    urdf_root = ElementTree.parse(
        environment.root / "robot" / "assets" / "ur5.urdf"
    ).getroot()
    shoulder_joint = urdf_root.find("./joint[@name='shoulder_pan_joint']")
    assert shoulder_joint is not None
    shoulder_origin = shoulder_joint.find("origin")
    assert shoulder_origin is not None
    shoulder_origin_z = float(shoulder_origin.attrib["xyz"].split()[2])
    shoulder_origin_rpy = tuple(
        float(value) for value in shoulder_origin.attrib["rpy"].split()
    )
    assert shoulder_origin_rpy[:2] == pytest.approx((0.0, 0.0), abs=1.0e-12)
    assert abs(shoulder_origin_rpy[2]) > 1.0e-12
    support_recess = ground["fixed_base_support_recess_m"]
    shoulder_to_support_clearance = (
        shoulder_origin_z - shoulder_sphere["radius"] + support_recess
    )
    maximum_configured_sphere_radius = max(
        sphere["radius"]
        for spheres in environment.collision_spheres["collision_spheres"].values()
        for sphere in spheres
    )
    required_ground_half_span = (
        ground["ur5_nominal_reach_m"]
        + environment.setup_config.tool.length_m
        + maximum_configured_sphere_radius
        + minimum_scene_clearance
    )

    # A solid plane at z=0 would reject every joint state because the fixed
    # shoulder sphere overlaps it after applying the authoritative 5 mm
    # clearance probe. The installation recess clears that fixed geometry,
    # while the support slab plus the same 5 mm probe still keeps every moving
    # sphere strictly above the real z=0 plane.
    assert shoulder_origin_z - inflated_shoulder_radius < ground["top_z_m"]
    assert cutout_half_width > inflated_shoulder_radius
    assert shoulder_to_support_clearance > minimum_scene_clearance
    assert minimum_scene_clearance - support_recess > 0.0
    assert 0.5 * min(ground["size_xy_m"]) >= required_ground_half_span
    assert required_ground_half_span == pytest.approx(1.27365, abs=1.0e-12)

    support = environment.zones["curobo_scene"]["cuboid"][
        "base_ground_fixed_base_support"
    ]
    ground_cuboids = environment.zones["curobo_scene"]["cuboid"]
    x_negative = ground_cuboids["base_ground_x_negative"]
    x_positive = ground_cuboids["base_ground_x_positive"]
    y_negative = ground_cuboids["base_ground_y_negative"]
    y_positive = ground_cuboids["base_ground_y_positive"]
    cutout_half_width = 0.5 * support["dims"][0]

    assert support["dims"][:2] == ground["fixed_base_cutout_size_xy_m"]
    assert x_negative["pose"][0] + 0.5 * x_negative["dims"][0] == pytest.approx(
        -cutout_half_width,
        abs=1.0e-12,
    )
    assert x_positive["pose"][0] - 0.5 * x_positive["dims"][0] == pytest.approx(
        cutout_half_width,
        abs=1.0e-12,
    )
    assert y_negative["pose"][1] + 0.5 * y_negative["dims"][1] == pytest.approx(
        -cutout_half_width,
        abs=1.0e-12,
    )
    assert y_positive["pose"][1] - 0.5 * y_positive["dims"][1] == pytest.approx(
        cutout_half_width,
        abs=1.0e-12,
    )
    support_bottom = support["pose"][2] - 0.5 * support["dims"][2]
    support_top = support["pose"][2] + 0.5 * support["dims"][2]
    assert support_bottom == pytest.approx(-ground["thickness_m"], abs=1.0e-12)
    assert support_top == pytest.approx(-support_recess, abs=1.0e-12)


def test_ground_support_rejects_a_clearance_larger_than_the_fixed_base_gap(
    tmp_path,
    repository_root,
) -> None:
    target = _copy_new_board(tmp_path, repository_root)
    planner_path = target / "planner.yaml"
    planner = yaml.safe_load(planner_path.read_text(encoding="utf-8"))
    planner["validation"]["minimum_collision_distance_m"] = 0.006
    planner_path.write_text(yaml.safe_dump(planner), encoding="utf-8")

    with pytest.raises(
        ConfigurationError,
        match="fixed shoulder sphere does not preserve the configured clearance",
    ):
        EnvironmentBuilder().load(target)


def test_ground_rejects_a_robot_envelope_larger_than_its_half_span(
    tmp_path,
    repository_root,
) -> None:
    target = _copy_new_board(tmp_path, repository_root)
    spheres_path = target / "robot" / "collision_spheres.yaml"
    spheres = yaml.safe_load(spheres_path.read_text(encoding="utf-8"))
    spheres["collision_spheres"]["wrist_3_link"][0]["radius"] = 0.40
    spheres_path.write_text(yaml.safe_dump(spheres), encoding="utf-8")

    with pytest.raises(
        ConfigurationError,
        match="base ground does not cover the conservative robot/tool reach envelope",
    ):
        EnvironmentBuilder().load(target)
