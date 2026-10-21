from __future__ import annotations

import hashlib
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from conftest import copy_environment_without_recordings

from hx_waypoint_curobo.contact_policy import ContactPolicy
from hx_waypoint_curobo.environment_builder import EnvironmentBuilder, EnvironmentBundle
from hx_waypoint_curobo.errors import ConfigurationError
from hx_waypoint_curobo.math3d import pose_matrix
from hx_waypoint_curobo.scene_config import resolve_curobo_cuboids
from hx_waypoint_tools.planning_scene_publisher import (
    _build_marker_messages,
    load_planning_scene_definition,
)


LEGACY_ENVIRONMENT_ID = "new_board_4cm_rod33365"
ENVIRONMENT_ID = "new_board_4cm_rod33365_up50_icassp_phase2"


def _load(
    repository_root: Path,
    environment_id: str = ENVIRONMENT_ID,
) -> EnvironmentBundle:
    return EnvironmentBuilder().load(
        repository_root / "environments" / environment_id
    )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


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
        self.pose = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
        )
        self.scale = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.color = SimpleNamespace(r=0.0, g=0.0, b=0.0, a=0.0)
        self.text = ""
        self.frame_locked = False


def test_up50_phase2_sensor_geometry_is_exact_and_legacy_is_unchanged(
    repository_root: Path,
) -> None:
    up50 = _load(repository_root)
    legacy = _load(repository_root, LEGACY_ENVIRONMENT_ID)

    assert up50.environment_id == ENVIRONMENT_ID
    assert up50.setup["setup_id"] == (
        "new_board_4cm_direct_feature_rod333p65_d5_up50_icassp_phase2"
    )
    assert up50.setup_config.work_frame == legacy.setup_config.work_frame
    assert up50.setup_config.tool == legacy.setup_config.tool
    assert up50.robot_model == legacy.robot_model

    sensor = up50.environment["calibration"]["sensor_geometry"]
    assert sensor == {
        "original_sensor_plane_local_z_m": 0.043,
        "sensor_translation_local_z_m": -0.050,
        "actual_sensor_plane_local_z_m": -0.007,
        "protected_height_above_sensor_m": 0.060,
        "board_size_xy_m": [0.40, 0.40],
        "board_center_matches_feature_origin": True,
        "board_center_in_plane1_xy_m": [0.0, 0.0],
        "keepout_local_z_min_m": -0.067,
        "keepout_local_z_max_m": 0.100,
        "model_status": (
            "up50_board_and_workspace_boundaries_confirmed_for_icassp_phase2"
        ),
    }
    assert sensor["original_sensor_plane_local_z_m"] + sensor[
        "sensor_translation_local_z_m"
    ] == pytest.approx(sensor["actual_sensor_plane_local_z_m"], abs=1.0e-12)
    assert sensor["actual_sensor_plane_local_z_m"] - sensor[
        "protected_height_above_sensor_m"
    ] == pytest.approx(sensor["keepout_local_z_min_m"], abs=1.0e-12)

    keepout = up50.zones["curobo_scene"]["cuboid"]["sensor_board_keepout"]
    assert keepout == {
        "frame_id": "plane1",
        "dims": [0.40, 0.40, 0.167],
        "pose": [0.0, 0.0, 0.0165, 1.0, 0.0, 0.0, 0.0],
    }
    assert keepout["pose"][2] - keepout["dims"][2] / 2.0 == pytest.approx(
        -0.067,
        abs=1.0e-12,
    )
    assert keepout["pose"][2] + keepout["dims"][2] / 2.0 == pytest.approx(
        0.100,
        abs=1.0e-12,
    )
    assert up50.zones["analysis_zones"] == {
        "sensor_keepout_clearance_boundary": {
            "frame_id": "plane1",
            "normal_xyz": [0.0, 0.0, -1.0],
            "offset_m": 0.072,
            "x_range_m": [-0.20, 0.20],
            "y_range_m": [-0.20, 0.20],
        },
        "actual_sensor_plane": {
            "frame_id": "plane1",
            "normal_xyz": [0.0, 0.0, -1.0],
            "offset_m": 0.007,
        },
    }
    assert up50.planner["validation"]["minimum_collision_distance_m"] == 0.005
    assert 0.072 - abs(-0.067) == pytest.approx(0.005, abs=1.0e-12)

    policy = ContactPolicy.from_environment(up50)
    assert policy.maximum_tip_penetration_m == 0.0
    assert policy.minimum_non_tip_clearance_m == 0.005
    assert policy.plane_normal_base == pytest.approx(
        (0.004086517701568664, -0.001733297619749481, 0.9999901479776869),
        abs=1.0e-12,
    )
    assert policy.plane_offset_base_m == pytest.approx(
        0.25389078858168266,
        abs=1.0e-12,
    )

    legacy_sensor = legacy.environment["calibration"]["sensor_geometry"]
    assert legacy_sensor["actual_sensor_plane_local_z_m"] == 0.043
    assert legacy_sensor["keepout_local_z_min_m"] == -0.01
    assert legacy_sensor["keepout_local_z_max_m"] == 0.15
    assert legacy.zones["curobo_scene"]["cuboid"]["sensor_board_keepout"] == {
        "frame_id": "plane1",
        "dims": [0.40, 0.40, 0.16],
        "pose": [0.0, 0.0, 0.07, 1.0, 0.0, 0.0, 0.0],
    }
    assert legacy.zones["analysis_zones"]["actual_sensor_plane"]["offset_m"] == (
        -0.043
    )


def test_up50_phase2_planning_scene_uses_the_shifted_keepout(
    repository_root: Path,
) -> None:
    environment = _load(repository_root)
    definition = load_planning_scene_definition(repository_root, ENVIRONMENT_ID)
    curobo_cuboids = {
        cuboid.object_id: cuboid for cuboid in resolve_curobo_cuboids(environment)
    }
    moveit_cuboids = {cuboid.object_id: cuboid for cuboid in definition.cuboids}

    assert set(moveit_cuboids) == set(curobo_cuboids)
    assert definition.environment_id == ENVIRONMENT_ID
    assert definition.remove_object_ids == ("floor", "rear_keepout")
    assert definition.attached_rod.length_m == pytest.approx(0.33365, abs=1.0e-12)
    assert definition.attached_rod.radius_m == pytest.approx(0.0025, abs=1.0e-12)

    board = curobo_cuboids["sensor_board_keepout"]
    assert board.source_frame_id == "plane1"
    assert board.dimensions_m == (0.40, 0.40, 0.167)
    expected_matrix = environment._frame_to_base("plane1") @ pose_matrix(
        (0.0, 0.0, 0.0165),
        (0.0, 0.0, 0.0, 1.0),
    )
    assert pose_matrix(board.position_m, board.quaternion_xyzw) == pytest.approx(
        expected_matrix,
        abs=1.0e-12,
    )
    assert board.position_m == pytest.approx(
        (0.0652125724579239, 0.46975859941072556, 0.16594016255836817),
        abs=1.0e-12,
    )
    moveit_board = moveit_cuboids["sensor_board_keepout"]
    assert moveit_board.dimensions_m == board.dimensions_m
    assert moveit_board.position_m == pytest.approx(board.position_m, abs=1.0e-12)
    assert moveit_board.quaternion_xyzw == pytest.approx(
        board.quaternion_xyzw,
        abs=1.0e-12,
    )

    legacy = _load(repository_root, LEGACY_ENVIRONMENT_ID)
    new_non_sensor = dict(environment.zones["curobo_scene"]["cuboid"])
    old_non_sensor = dict(legacy.zones["curobo_scene"]["cuboid"])
    new_non_sensor.pop("sensor_board_keepout")
    old_non_sensor.pop("sensor_board_keepout")
    assert new_non_sensor == old_non_sensor


def test_up50_phase2_visualization_marker_shows_the_current_sensor_plane(
    repository_root: Path,
) -> None:
    definition = load_planning_scene_definition(repository_root, ENVIRONMENT_ID)
    planes = {plane.namespace: plane for plane in definition.visualization_planes}

    assert planes["calibration_board_plane"].position_m == (0.0, 0.0, 0.0)
    actual = planes["actual_sensor_plane"]
    assert actual.frame_id == "plane1"
    assert actual.position_m == pytest.approx((0.0, 0.0, -0.007), abs=1.0e-12)
    assert actual.label == "CYAN_sensor_z=-7mm"

    messages = _build_marker_messages(definition, _Marker)
    actual_marker = next(
        marker
        for marker in messages
        if marker.ns == "actual_sensor_plane" and marker.type == _Marker.CUBE
    )
    actual_label = next(
        marker
        for marker in messages
        if marker.ns == "actual_sensor_plane"
        and marker.type == _Marker.TEXT_VIEW_FACING
    )
    assert actual_marker.header.frame_id == "plane1"
    assert (
        actual_marker.pose.position.x,
        actual_marker.pose.position.y,
        actual_marker.pose.position.z,
    ) == pytest.approx((0.0, 0.0, -0.007), abs=1.0e-12)
    assert (actual_marker.scale.x, actual_marker.scale.y, actual_marker.scale.z) == (
        0.40,
        0.40,
        0.002,
    )
    assert actual_label.text == "CYAN_sensor_z=-7mm"


def test_up50_phase2_environment_hashes_close_over_every_reproducible_input(
    repository_root: Path,
) -> None:
    environment = _load(repository_root)
    expected_paths = {
        "controllers.yaml",
        "environment.yaml",
        "meshes/.gitkeep",
        "planner.yaml",
        "recording.yaml",
        "robot/asset_provenance.yaml",
        "robot/assets/ur5.urdf",
        "robot/calibrated_kinematics.yaml",
        "robot/collision_spheres.yaml",
        "robot/curobo_robot.yaml",
        "robot/driver_kinematics.yaml",
        "robot/robot_model.yaml",
        "setup.yaml",
        "zones.yaml",
    }

    assert set(environment.hashes) == expected_paths
    assert environment.hashes == {
        relative: _sha256(environment.root / relative)
        for relative in sorted(expected_paths)
    }
    assert {path.name for path in (environment.root / "trajectories").iterdir()} == {
        ".gitkeep",
        "z_stack_tilt_calibration_center_up50_phase2.csv",
        "z_stack_tilt_calibration_x50_up50_phase2.csv",
        "z_stack_tilt_calibration_capture_windows.csv",
        "z_stack_tilt_calibration_center_up50_phase2_v2.csv",
        "z_stack_tilt_calibration_x50_up50_phase2_v2.csv",
        "z_stack_tilt_calibration_capture_windows_v2.csv",
        "z_stack_tilt_calibration_center_up50_phase2_v3.csv",
        "z_stack_tilt_calibration_x50_up50_phase2_v3.csv",
        "z_stack_tilt_calibration_capture_windows_v3.csv",
    }
    recording_entries = tuple((environment.root / "recordings").iterdir())
    assert environment.root / "recordings" / ".gitkeep" in recording_entries
    assert all(
        path.name == ".gitkeep"
        or (
            path.is_dir()
            and path.name.startswith("z_stack_tilt_calibration_")
            and (path / "manifest.yaml").is_file()
        )
        for path in recording_entries
    )

    legacy_root = repository_root / "environments" / LEGACY_ENVIRONMENT_ID
    for relative in expected_paths - {"environment.yaml", "setup.yaml", "zones.yaml"}:
        assert (environment.root / relative).read_bytes() == (
            legacy_root / relative
        ).read_bytes()


def test_up50_phase2_geometry_metadata_and_clearance_are_fail_closed(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    target = copy_environment_without_recordings(
        repository_root / "environments" / ENVIRONMENT_ID,
        tmp_path / ENVIRONMENT_ID,
    )
    environment_path = target / "environment.yaml"
    payload = yaml.safe_load(environment_path.read_text(encoding="utf-8"))
    payload["calibration"]["sensor_geometry"]["protected_height_above_sensor_m"] = (
        0.050
    )
    environment_path.write_text(
        yaml.safe_dump(payload, sort_keys=False),
        encoding="utf-8",
    )

    with pytest.raises(ConfigurationError, match="protected_height_above_sensor_m"):
        EnvironmentBuilder().load(target)

    payload["calibration"]["sensor_geometry"]["protected_height_above_sensor_m"] = (
        0.060
    )
    environment_path.write_text(
        yaml.safe_dump(payload, sort_keys=False),
        encoding="utf-8",
    )
    zones_path = target / "zones.yaml"
    zones = yaml.safe_load(zones_path.read_text(encoding="utf-8"))
    zones["analysis_zones"]["sensor_keepout_clearance_boundary"]["offset_m"] = (
        0.071
    )
    zones_path.write_text(yaml.safe_dump(zones, sort_keys=False), encoding="utf-8")

    with pytest.raises(ConfigurationError, match="exactly 5 mm outside"):
        EnvironmentBuilder().load(target)


def test_direct_feature_policy_cannot_bypass_contract_by_renaming_both_ids(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    renamed_id = "renamed_direct_feature_environment"
    target = copy_environment_without_recordings(
        repository_root / "environments" / ENVIRONMENT_ID,
        tmp_path / renamed_id,
    )
    environment_path = target / "environment.yaml"
    payload = yaml.safe_load(environment_path.read_text(encoding="utf-8"))
    payload["environment_id"] = renamed_id
    environment_path.write_text(
        yaml.safe_dump(payload, sort_keys=False),
        encoding="utf-8",
    )

    with pytest.raises(ConfigurationError, match="registered authority"):
        EnvironmentBuilder().load(target)
