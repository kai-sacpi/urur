from __future__ import annotations

import shutil
from math import pi, tau
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from conftest import copy_environment_without_recordings

from hx_waypoint_curobo.curobo_adapter_v2 import CuroboAdapterV2
from hx_waypoint_curobo.curobo_server import _environment_evidence
from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_curobo.errors import ConfigurationError, CuroboUnavailableError
from hx_waypoint_curobo.models import TimedPose
from hx_waypoint_curobo.robot_tool_builder import RobotToolBuilder
from hx_waypoint_curobo.setup_config import rotvec_to_quaternion_xyzw


def test_environment_loads_fail_closed(environment) -> None:
    assert environment.environment_id == "lab_wall_a"
    assert environment.robot_type == "ur5"
    assert len(environment.joint_names) == 6
    assert environment.controllers["real"]["allow_fallback"] is False
    assert environment.planner["limits"] == {
        "on_violation": "reject",
        "allow_uniform_time_scale": False,
        "max_uniform_time_scale": 1.0,
    }
    assert environment.planner["curobo"]["interpolation_buffer_size"] == 65_536
    assert environment.planner["ik"]["random_seed_reserve"] == 8
    assert environment.planner["validation"]["internal_tolerance_guard_ratio"] == 0.8
    assert environment.recording["recording"]["first_point"] == {
        "time_s": 0.0,
        "require_zero_velocity": True,
        "require_zero_acceleration": True,
    }
    assert environment.home_joint_positions == pytest.approx(
        environment.robot_model["robot"]["home_joint_positions"],
        abs=1.0e-12,
    )


def test_curobo_default_position_must_match_environment_home(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    target = tmp_path / "lab_wall_a"
    copy_environment_without_recordings(
        repository_root / "environments" / "lab_wall_a",
        target,
    )
    curobo_path = target / "robot" / "curobo_robot.yaml"
    curobo = yaml.safe_load(curobo_path.read_text(encoding="utf-8"))
    curobo["robot_cfg"]["kinematics"]["cspace"]["default_joint_position"][0] += 0.1
    curobo_path.write_text(yaml.safe_dump(curobo, sort_keys=False), encoding="utf-8")

    with pytest.raises(ConfigurationError, match="must exactly match"):
        EnvironmentBuilder().load(target)


def test_setup_is_the_single_authority_for_work_frame_and_rod(
    environment,
    repository_root: Path,
) -> None:
    setup_path = repository_root / "environments" / "lab_wall_a" / "setup.yaml"
    raw_pose = environment.setup_config.work_frame.teach_pendant_pose_xyz_rotvec

    assert environment.hashes["setup.yaml"]
    assert "tcp.yaml" not in environment.hashes
    assert "frames.yaml" not in environment.hashes
    assert not (setup_path.parent / "tcp.yaml").exists()
    assert not (setup_path.parent / "frames.yaml").exists()
    assert raw_pose == pytest.approx(
        (-0.65, 0.0, 0.30, -2.221441469079183, 0.0, -2.221441469079183)
    )
    assert environment.setup_config.work_frame.parent_to_child.quaternion_xyzw == pytest.approx(
        rotvec_to_quaternion_xyzw(raw_pose[3:]),
        abs=1.0e-15,
    )
    assert environment.tcp["tool_id"] == "rod_264mm_nominal"
    assert environment.tcp["tool"]["translation_m"] == pytest.approx((0.0, 0.0, 0.264))
    assert set(environment.frames["frames"]) == {"plane1", "wall_draw_frame"}


def test_setup_change_updates_every_derived_view_and_hash(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    target = tmp_path / "lab_wall_a"
    copy_environment_without_recordings(
        repository_root / "environments" / "lab_wall_a",
        target,
    )
    original = EnvironmentBuilder().load(target)
    setup_path = target / "setup.yaml"
    setup = yaml.safe_load(setup_path.read_text(encoding="utf-8"))
    setup["work_frame"]["teach_pendant_pose_xyz_rotvec"][0] += 0.01
    setup["tool"]["tool_id"] = "rod_200mm_test"
    setup["tool"]["rod"]["length_m"] = 0.20
    setup_path.write_text(yaml.safe_dump(setup, sort_keys=False), encoding="utf-8")

    changed = EnvironmentBuilder().load(target)
    assert changed.hashes["setup.yaml"] != original.hashes["setup.yaml"]
    assert changed._frame_to_base("plane1")[0, 3] != pytest.approx(
        original._frame_to_base("plane1")[0, 3]
    )
    assert changed.tcp["tool_id"] == "rod_200mm_test"
    assert changed.tcp["tool"]["translation_m"] == pytest.approx((0.0, 0.0, 0.20))
    spheres = RobotToolBuilder(changed).collision_spheres()
    assert spheres[-1].center_m[2] + spheres[-1].radius_m == pytest.approx(
        0.20,
        abs=1.0e-12,
    )


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (
            lambda setup: setup["work_frame"].__setitem__("parent_frame", "base_link"),
            "base -> plane1",
        ),
        (
            lambda setup: setup["tool"]["rod"].__setitem__(
                "axis_in_parent_xyz", [0.0, 0.0, 2.0]
            ),
            "unit vector",
        ),
        (
            lambda setup: setup["tool"]["rod"].__setitem__("length_m", float("nan")),
            "finite positive",
        ),
        (
            lambda setup: setup["tool"].__setitem__("unexpected", 1),
            "unsupported fields",
        ),
    ],
)
def test_invalid_setup_fails_closed(
    tmp_path: Path,
    repository_root: Path,
    mutate,
    message: str,
) -> None:
    target = tmp_path / "lab_wall_a"
    copy_environment_without_recordings(
        repository_root / "environments" / "lab_wall_a",
        target,
    )
    setup_path = target / "setup.yaml"
    setup = yaml.safe_load(setup_path.read_text(encoding="utf-8"))
    mutate(setup)
    setup_path.write_text(yaml.safe_dump(setup, sort_keys=False), encoding="utf-8")

    with pytest.raises(ConfigurationError, match=message):
        EnvironmentBuilder().load(target)


def test_generated_urdf_defines_bounded_periodic_joint_domains(environment) -> None:
    domains = environment.joint_position_domains

    assert tuple(domain.name for domain in domains) == environment.joint_names
    assert all(domain.period == tau for domain in domains)
    assert domains[0].lower == -tau
    assert domains[0].upper == tau
    assert domains[2].lower == -pi
    assert domains[2].upper == pi


def test_prepare_environment_evidence_is_complete_and_deterministic(environment) -> None:
    paths, hashes = _environment_evidence(environment.hashes)
    assert paths == sorted(environment.hashes)
    assert hashes == [environment.hashes[path] for path in paths]
    assert len(paths) == len(hashes) == len(environment.hashes)


def test_environment_missing_file_is_rejected(tmp_path: Path, repository_root: Path) -> None:
    source = repository_root / "environments" / "lab_wall_a"
    target = tmp_path / "lab_wall_a"
    copy_environment_without_recordings(source, target)
    (target / "zones.yaml").unlink()
    with pytest.raises(ConfigurationError, match="zones.yaml"):
        EnvironmentBuilder().load(target)


def test_non_curobo_backend_is_rejected(tmp_path: Path, repository_root: Path) -> None:
    source = repository_root / "environments" / "lab_wall_a"
    target = tmp_path / "lab_wall_a"
    copy_environment_without_recordings(source, target)
    planner_path = target / "planner.yaml"
    planner = yaml.safe_load(planner_path.read_text(encoding="utf-8"))
    planner["backend"]["type"] = "moveit_ik"
    planner_path.write_text(yaml.safe_dump(planner), encoding="utf-8")
    with pytest.raises(ConfigurationError, match="curobo_v2"):
        EnvironmentBuilder().load(target)


def test_timestamp_preservation_cannot_be_disabled(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    source = repository_root / "environments" / "lab_wall_a"
    target = tmp_path / "lab_wall_a"
    copy_environment_without_recordings(source, target)
    planner_path = target / "planner.yaml"
    planner = yaml.safe_load(planner_path.read_text(encoding="utf-8"))
    planner["timed_path"]["preserve_input_timestamps"] = False
    planner_path.write_text(yaml.safe_dump(planner), encoding="utf-8")
    with pytest.raises(ConfigurationError, match="preserve_input_timestamps must be true"):
        EnvironmentBuilder().load(target)


def test_previous_solution_ik_seed_cannot_be_disabled(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    source = repository_root / "environments" / "lab_wall_a"
    target = tmp_path / "lab_wall_a"
    copy_environment_without_recordings(source, target)
    planner_path = target / "planner.yaml"
    planner = yaml.safe_load(planner_path.read_text(encoding="utf-8"))
    planner["ik"]["use_previous_solution_seed"] = False
    planner_path.write_text(yaml.safe_dump(planner), encoding="utf-8")
    with pytest.raises(ConfigurationError, match="use_previous_solution_seed must be true"):
        EnvironmentBuilder().load(target)


@pytest.mark.parametrize("reserve", [True, 0, -1, 32, 33, 1.5, "8"])
def test_invalid_ik_random_seed_reserve_is_rejected(
    tmp_path: Path,
    repository_root: Path,
    reserve: object,
) -> None:
    source = repository_root / "environments" / "lab_wall_a"
    target = tmp_path / "lab_wall_a"
    copy_environment_without_recordings(source, target)
    planner_path = target / "planner.yaml"
    planner = yaml.safe_load(planner_path.read_text(encoding="utf-8"))
    planner["ik"]["random_seed_reserve"] = reserve
    planner_path.write_text(yaml.safe_dump(planner), encoding="utf-8")

    with pytest.raises(ConfigurationError, match="random_seed_reserve"):
        EnvironmentBuilder().load(target)


@pytest.mark.parametrize("ratio", [True, 0.0, -0.1, 1.0, 1.1, float("inf"), "0.8"])
def test_invalid_internal_tolerance_guard_ratio_is_rejected(
    tmp_path: Path,
    repository_root: Path,
    ratio: object,
) -> None:
    source = repository_root / "environments" / "lab_wall_a"
    target = tmp_path / "lab_wall_a"
    copy_environment_without_recordings(source, target)
    planner_path = target / "planner.yaml"
    planner = yaml.safe_load(planner_path.read_text(encoding="utf-8"))
    planner["validation"]["internal_tolerance_guard_ratio"] = ratio
    planner_path.write_text(yaml.safe_dump(planner), encoding="utf-8")

    with pytest.raises(ConfigurationError, match="internal_tolerance_guard_ratio"):
        EnvironmentBuilder().load(target)


def test_curobo_cspace_joint_order_must_match_robot_model(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    source = repository_root / "environments" / "lab_wall_a"
    target = tmp_path / "lab_wall_a"
    copy_environment_without_recordings(source, target)
    curobo_path = target / "robot" / "curobo_robot.yaml"
    curobo_robot = yaml.safe_load(curobo_path.read_text(encoding="utf-8"))
    joint_names = curobo_robot["robot_cfg"]["kinematics"]["cspace"][
        "joint_names"
    ]
    joint_names[0], joint_names[1] = joint_names[1], joint_names[0]
    curobo_path.write_text(yaml.safe_dump(curobo_robot), encoding="utf-8")

    with pytest.raises(ConfigurationError, match="exactly match.*order"):
        EnvironmentBuilder().load(target)


@pytest.mark.parametrize("maximum", [0.5, 1.01, 3.0, float("inf")])
def test_invalid_uniform_time_scale_limit_is_rejected(
    tmp_path: Path,
    repository_root: Path,
    maximum: float,
) -> None:
    source = repository_root / "environments" / "lab_wall_a"
    target = tmp_path / "lab_wall_a"
    copy_environment_without_recordings(source, target)
    planner_path = target / "planner.yaml"
    planner = yaml.safe_load(planner_path.read_text(encoding="utf-8"))
    planner["limits"]["max_uniform_time_scale"] = maximum
    planner_path.write_text(yaml.safe_dump(planner), encoding="utf-8")
    with pytest.raises(ConfigurationError, match="max_uniform_time_scale"):
        EnvironmentBuilder().load(target)


@pytest.mark.parametrize(
    ("key", "value", "message"),
    [
        ("on_violation", "uniform_scale", "on_violation must be reject"),
        (
            "allow_uniform_time_scale",
            True,
            "allow_uniform_time_scale must be false",
        ),
    ],
)
def test_any_time_scaling_configuration_is_rejected(
    tmp_path: Path,
    repository_root: Path,
    key: str,
    value: object,
    message: str,
) -> None:
    source = repository_root / "environments" / "lab_wall_a"
    target = tmp_path / "lab_wall_a"
    copy_environment_without_recordings(source, target)
    planner_path = target / "planner.yaml"
    planner = yaml.safe_load(planner_path.read_text(encoding="utf-8"))
    planner["limits"][key] = value
    planner_path.write_text(yaml.safe_dump(planner), encoding="utf-8")
    with pytest.raises(ConfigurationError, match=message):
        EnvironmentBuilder().load(target)


@pytest.mark.parametrize(
    ("mode", "action"),
    [
        (
            "simulation",
            "/joint_trajectory_controller/follow_joint_trajectory_extra",
        ),
        (
            "real",
            "/prefix/passthrough_trajectory_controller/follow_joint_trajectory",
        ),
    ],
)
def test_controller_action_names_must_match_exactly(
    tmp_path: Path,
    repository_root: Path,
    mode: str,
    action: str,
) -> None:
    source = repository_root / "environments" / "lab_wall_a"
    target = tmp_path / "lab_wall_a"
    copy_environment_without_recordings(source, target)
    controllers_path = target / "controllers.yaml"
    controllers = yaml.safe_load(controllers_path.read_text(encoding="utf-8"))
    controllers[mode]["action_name"] = action
    controllers_path.write_text(yaml.safe_dump(controllers), encoding="utf-8")
    with pytest.raises(ConfigurationError, match="exact .* action"):
        EnvironmentBuilder().load(target)


def test_simulation_controller_state_topic_must_match_exactly(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    source = repository_root / "environments" / "lab_wall_a"
    target = tmp_path / "lab_wall_a"
    copy_environment_without_recordings(source, target)
    controllers_path = target / "controllers.yaml"
    controllers = yaml.safe_load(controllers_path.read_text(encoding="utf-8"))
    controllers["simulation"]["state_topic"] = "/scaled_controller/controller_state"
    controllers_path.write_text(yaml.safe_dump(controllers), encoding="utf-8")

    with pytest.raises(ConfigurationError, match="state topic"):
        EnvironmentBuilder().load(target)


@pytest.mark.parametrize(
    "dotted_field",
    [
        "output_fields.positions",
        "output_fields.velocities",
        "output_fields.accelerations",
        "first_point.require_zero_velocity",
        "first_point.require_zero_acceleration",
        "last_point.require_zero_velocity",
        "last_point.require_zero_acceleration",
    ],
)
def test_recording_derivative_contract_cannot_be_disabled(
    tmp_path: Path,
    repository_root: Path,
    dotted_field: str,
) -> None:
    source = repository_root / "environments" / "lab_wall_a"
    target = tmp_path / "lab_wall_a"
    copy_environment_without_recordings(source, target)
    recording_path = target / "recording.yaml"
    recording = yaml.safe_load(recording_path.read_text(encoding="utf-8"))
    value = recording["recording"]
    keys = dotted_field.split(".")
    for key in keys[:-1]:
        value = value[key]
    value[keys[-1]] = False
    recording_path.write_text(yaml.safe_dump(recording), encoding="utf-8")

    with pytest.raises(ConfigurationError, match="must be true"):
        EnvironmentBuilder().load(target)


def test_tip_to_tool0_transform(environment) -> None:
    builder = RobotToolBuilder(environment)
    tip = TimedPose(
        time_s=0.0,
        frame_id="base",
        position_m=(0.0, 0.0, 0.264),
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        phase="draw",
    )
    tool = builder.tip_pose_to_tool_pose(tip)
    assert np.allclose(tool.position_m, (0.0, 0.0, 0.0), atol=1.0e-12)
    spheres = builder.collision_spheres()
    assert spheres
    assert all(sphere.link_name == "tool0" for sphere in spheres)
    assert max(sphere.center_m[2] for sphere in spheres) <= 0.264


def test_curobo_robot_paths_are_resolved_inside_environment(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    environment_path = tmp_path / "lab_wall_a"
    copy_environment_without_recordings(
        repository_root / "environments" / "lab_wall_a",
        environment_path,
    )
    asset_root = environment_path / "robot" / "assets"
    urdf_path = asset_root / "ur5.urdf"

    environment = EnvironmentBuilder().load(environment_path)
    adapter = CuroboAdapterV2(environment, repository_root / "curobo.lock")
    robot = adapter._robot_config()
    kinematics = robot["robot_cfg"]["kinematics"]

    assert kinematics["asset_root_path"] == str(asset_root.resolve())
    assert kinematics["urdf_path"] == str(urdf_path.resolve())
    assert kinematics["collision_spheres"]["tool0"]
    assert environment.hashes["robot/assets/ur5.urdf"]


def test_calibration_source_and_generated_urdf_are_hash_bound(environment) -> None:
    provenance = environment.asset_provenance
    source_path = provenance["source_kinematics"]["path"]
    urdf_path = provenance["generated_urdf"]["path"]
    assert environment.hashes[source_path] == provenance["source_kinematics"]["sha256"]
    assert environment.hashes[urdf_path] == provenance["generated_urdf"]["sha256"]
    assert (
        environment.calibrated_kinematics["driver_kinematics_sha256"]
        == provenance["source_kinematics"]["sha256"]
    )


@pytest.mark.parametrize(
    ("relative_path", "expected_message"),
    [
        ("robot/driver_kinematics.yaml", "controlled driver kinematics hash"),
        ("robot/assets/ur5.urdf", "generated URDF hash"),
    ],
)
def test_asset_provenance_rejects_tampered_inputs(
    tmp_path: Path,
    repository_root: Path,
    relative_path: str,
    expected_message: str,
) -> None:
    target = tmp_path / "lab_wall_a"
    copy_environment_without_recordings(
        repository_root / "environments" / "lab_wall_a",
        target,
    )
    path = target / relative_path
    path.write_bytes(path.read_bytes() + b"\n# tampered\n")
    with pytest.raises(ConfigurationError, match=expected_message):
        EnvironmentBuilder().load(target)


def test_asset_provenance_rejects_wrong_robot_or_source_path(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    target = tmp_path / "lab_wall_a"
    copy_environment_without_recordings(
        repository_root / "environments" / "lab_wall_a",
        target,
    )
    provenance_path = target / "robot" / "asset_provenance.yaml"
    provenance = yaml.safe_load(provenance_path.read_text(encoding="utf-8"))
    provenance["robot_type"] = "ur5e"
    provenance_path.write_text(yaml.safe_dump(provenance), encoding="utf-8")
    with pytest.raises(ConfigurationError, match="types must match"):
        EnvironmentBuilder().load(target)

    provenance["robot_type"] = "ur5"
    provenance["source_kinematics"]["path"] = "/tmp/arbitrary.yaml"
    provenance_path.write_text(yaml.safe_dump(provenance), encoding="utf-8")
    with pytest.raises(ConfigurationError, match="must be exactly"):
        EnvironmentBuilder().load(target)


def test_curobo_runtime_identity_requires_locked_verified_source(
    tmp_path: Path,
    repository_root: Path,
    environment,
) -> None:
    lock_path = tmp_path / "curobo.lock"
    shutil.copy2(repository_root / "curobo.lock", lock_path)
    adapter = CuroboAdapterV2(environment, lock_path)
    commit = adapter.revision
    source = tmp_path / ".deps" / f"curobo-{commit}"
    module_file = source / "curobo" / "__init__.py"
    module_file.parent.mkdir(parents=True)
    module_file.write_text("", encoding="utf-8")
    (source / ".ur-waypoint-curobo-commit").write_text(commit + "\n", encoding="utf-8")

    adapter._verify_runtime_identity(SimpleNamespace(__file__=str(module_file)))


def test_curobo_runtime_identity_rejects_unlocked_source(
    tmp_path: Path,
    repository_root: Path,
    environment,
) -> None:
    lock_path = tmp_path / "curobo.lock"
    shutil.copy2(repository_root / "curobo.lock", lock_path)
    adapter = CuroboAdapterV2(environment, lock_path)
    foreign_module = tmp_path / "foreign" / "curobo" / "__init__.py"
    foreign_module.parent.mkdir(parents=True)
    foreign_module.write_text("", encoding="utf-8")

    with pytest.raises(CuroboUnavailableError, match="verified locked source"):
        adapter._verify_runtime_identity(SimpleNamespace(__file__=str(foreign_module)))
