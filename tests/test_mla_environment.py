from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml

from conftest import copy_environment_without_recordings

from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_curobo.errors import ConfigurationError
from hx_waypoint_curobo.scene_config import resolve_curobo_cuboids
from hx_waypoint_tools.planning_scene_publisher import load_planning_scene_definition


MLA_ID = "mla"
PARENT_ID = "new_board_4cm_rod33365_up50_icassp_phase2"
NEW_FEATURE_POSE = (-0.08533, -0.48683, 0.24284, 1.16388, 2.91348, 0.00029)
FEATURE_TO_SENSOR_OFFSET_M = (0.0, 0.0, 0.053)
SENSOR_LOCAL_Z_M = 0.0
KEEPOUT_LOCAL_Z_MIN_M = -0.060
KEEPOUT_LOCAL_Z_MAX_M = 0.19096485659512854
PLANE1_PARENT_TRANSLATION_M = (
    -0.08511768728206212,
    -0.4869042639951418,
    0.1898404772835752,
)
SENSOR_CENTER_BASE_LINK_M = (
    0.08511768728206218,
    0.4869042639951418,
    0.1898404772835752,
)


def _load(root: Path, environment_id: str):
    return EnvironmentBuilder().load(root / "environments" / environment_id)


def _cuboids(environment):
    return {item.object_id: item for item in resolve_curobo_cuboids(environment)}


def test_mla_plane1_is_sensor_center_derived_from_new_board_mla(
    repository_root: Path,
) -> None:
    mla = _load(repository_root, MLA_ID)
    work_frame = mla.setup_config.work_frame

    assert work_frame.teach_pendant_pose_xyz_rotvec == NEW_FEATURE_POSE
    assert work_frame.feature_to_work_frame_translation_m == FEATURE_TO_SENSOR_OFFSET_M
    assert work_frame.aliases == ()
    assert work_frame.parent_to_child.translation_m == pytest.approx(
        PLANE1_PARENT_TRANSLATION_M,
        abs=1.0e-12,
    )

    calibration = mla.environment["calibration"]
    assert calibration["dashboard"]["variable"] == "new_board_mla"
    assert calibration["tf_binding"]["policy"] == "feature_local_translation"
    assert calibration["tf_binding"]["translation_policy"] == (
        "compose_feature_with_local_offset"
    )
    assert calibration["tf_binding"]["local_translation_frame"] == "feature"
    assert calibration["tf_binding"]["local_translation_m"] == list(
        FEATURE_TO_SENSOR_OFFSET_M
    )
    assert calibration["tf_binding"]["setup_pose_must_match_dashboard_pose"] is True


def test_mla_sensor_column_covers_sixty_mm_above_sensor_to_tabletop(
    repository_root: Path,
) -> None:
    mla = _load(repository_root, MLA_ID)
    sensor = mla.environment["calibration"]["sensor_geometry"]
    assert sensor["sensor_center_local_z_m"] == pytest.approx(
        SENSOR_LOCAL_Z_M, abs=1.0e-12
    )
    assert sensor["actual_sensor_plane_local_z_m"] == pytest.approx(
        SENSOR_LOCAL_Z_M, abs=1.0e-12
    )
    assert sensor["protected_height_above_sensor_m"] == pytest.approx(
        0.060, abs=1.0e-12
    )
    assert sensor["keepout_local_z_min_m"] == pytest.approx(
        SENSOR_LOCAL_Z_M - 0.060, abs=1.0e-12
    )
    assert sensor["keepout_local_z_max_m"] == pytest.approx(
        KEEPOUT_LOCAL_Z_MAX_M, abs=1.0e-12
    )

    source = mla.zones["curobo_scene"]["cuboid"]["sensor_board_keepout"]
    assert source["frame_id"] == "plane1"
    assert source["dims"] == pytest.approx(
        (0.40, 0.40, KEEPOUT_LOCAL_Z_MAX_M - KEEPOUT_LOCAL_Z_MIN_M),
        abs=1.0e-12,
    )
    assert source["pose"][:3] == pytest.approx(
        (0.0, 0.0, 0.5 * (KEEPOUT_LOCAL_Z_MIN_M + KEEPOUT_LOCAL_Z_MAX_M)),
        abs=1.0e-12,
    )

    base_from_plane = mla._frame_to_base("plane1")
    sensor_center_base = base_from_plane @ np.array(
        [0.0, 0.0, SENSOR_LOCAL_Z_M, 1.0]
    )
    assert sensor_center_base[:3] == pytest.approx(
        SENSOR_CENTER_BASE_LINK_M,
        abs=1.0e-12,
    )

    tabletop_corner_heights = []
    for x_m in (-0.20, 0.20):
        for y_m in (-0.20, 0.20):
            corner_base = base_from_plane @ np.array(
                [x_m, y_m, KEEPOUT_LOCAL_Z_MAX_M, 1.0]
            )
            tabletop_corner_heights.append(float(corner_base[2]))
    assert max(tabletop_corner_heights) == pytest.approx(0.0, abs=1.0e-12)
    assert all(height <= 1.0e-12 for height in tabletop_corner_heights)

    analysis = mla.zones["analysis_zones"]
    assert analysis["actual_sensor_plane"]["offset_m"] == pytest.approx(
        0.0, abs=1.0e-12
    )
    assert analysis["sensor_keepout_clearance_boundary"][
        "offset_m"
    ] == pytest.approx(0.065, abs=1.0e-12)


def test_mla_keeps_every_non_sensor_collision_at_the_parent_base_pose(
    repository_root: Path,
) -> None:
    parent = _cuboids(_load(repository_root, PARENT_ID))
    mla = _cuboids(_load(repository_root, MLA_ID))
    assert set(mla) == set(parent) - {"base_ground_fixed_base_support"}
    for object_id in mla:
        if object_id == "sensor_board_keepout":
            continue
        assert mla[object_id].source_frame_id == "base_link"
        assert mla[object_id].dimensions_m == parent[object_id].dimensions_m
        assert mla[object_id].position_m == pytest.approx(
            parent[object_id].position_m,
            abs=1.0e-12,
        )
        assert mla[object_id].quaternion_xyzw == pytest.approx(
            parent[object_id].quaternion_xyzw,
            abs=1.0e-12,
        )


def test_mla_moveit_scene_matches_curobo_and_uses_new_sensor_volume(
    repository_root: Path,
) -> None:
    mla = _load(repository_root, MLA_ID)
    curobo = _cuboids(mla)
    moveit = {
        item.object_id: item
        for item in load_planning_scene_definition(repository_root, MLA_ID).cuboids
    }
    assert set(moveit) == set(curobo)
    for object_id in curobo:
        assert moveit[object_id].dimensions_m == curobo[object_id].dimensions_m
        assert moveit[object_id].position_m == pytest.approx(
            curobo[object_id].position_m,
            abs=1.0e-12,
        )
        assert moveit[object_id].quaternion_xyzw == pytest.approx(
            curobo[object_id].quaternion_xyzw,
            abs=1.0e-12,
        )
    assert curobo["sensor_board_keepout"].dimensions_m[2] == pytest.approx(
        KEEPOUT_LOCAL_Z_MAX_M - KEEPOUT_LOCAL_Z_MIN_M,
        abs=1.0e-12,
    )


def test_mla_rviz_provenance_distinguishes_feature_from_sensor_centered_plane1(
    repository_root: Path,
) -> None:
    definition = load_planning_scene_definition(repository_root, MLA_ID)
    planes = {item.namespace: item for item in definition.visualization_planes}
    assert planes["raw_ur_feature"].position_m == pytest.approx(
        NEW_FEATURE_POSE[:3],
        abs=1.0e-12,
    )
    assert planes["raw_ur_feature"].label.endswith(":new_board_mla")
    assert planes["calibration_board_plane"].frame_id == "plane1"
    assert planes["calibration_board_plane"].label == (
        "PURPLE_work_frame_sensor_center_z=0"
    )
    assert planes["actual_sensor_plane"].position_m == pytest.approx(
        (0.0, 0.0, SENSOR_LOCAL_Z_M),
        abs=1.0e-12,
    )


@pytest.mark.parametrize(
    ("target_file", "mutate", "message"),
    [
        (
            "setup.yaml",
            lambda data: data["work_frame"]["teach_pendant_pose_xyz_rotvec"].__setitem__(
                2, 0.24283
            ),
            "new_board_mla Feature pose",
        ),
        (
            "environment.yaml",
            lambda data: data["calibration"]["sensor_geometry"].__setitem__(
                "keepout_local_z_min_m", -0.059
            ),
            "sensor-centered z=0 to tabletop contract",
        ),
    ],
)
def test_mla_contract_rejects_calibration_or_sensor_drift(
    target_file: str,
    mutate,
    message: str,
    tmp_path: Path,
    repository_root: Path,
) -> None:
    target = copy_environment_without_recordings(
        repository_root / "environments" / MLA_ID,
        tmp_path / MLA_ID,
    )
    config_path = target / target_file
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    mutate(config)
    config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")

    with pytest.raises(ConfigurationError, match=message):
        EnvironmentBuilder().load(target)
