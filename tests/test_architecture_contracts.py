from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


def test_only_adapter_imports_curobo(repository_root: Path) -> None:
    offenders = []
    package = repository_root / "src" / "hx_waypoint_curobo" / "hx_waypoint_curobo"
    for path in package.glob("*.py"):
        if path.name == "curobo_adapter_v2.py":
            continue
        text = path.read_text(encoding="utf-8")
        if "import curobo" in text or "from curobo" in text:
            offenders.append(path.name)
    assert offenders == []


def test_no_moveit_ik_in_compiler(repository_root: Path) -> None:
    package = repository_root / "src" / "hx_waypoint_curobo" / "hx_waypoint_curobo"
    text = "\n".join(path.read_text(encoding="utf-8") for path in package.glob("*.py"))
    assert "/compute_ik" not in text
    assert "GetPositionIK" not in text


def test_curobo_lock_is_immutable(repository_root: Path) -> None:
    lock = yaml.safe_load((repository_root / "curobo.lock").read_text(encoding="utf-8"))
    assert lock["repository"] == "https://github.com/NVlabs/curobo.git"
    assert lock["commit"] == "a35a708ecfbb26eb9ab2d7ef22c65919c4fae4a9"
    assert lock["source_tree"] == "369c492056ba8a735c00f11342062dfb22af2d90"
    assert lock["pytorch_version"] == "2.11.0+cu128"
    assert "source_archive_sha256" not in lock


def test_setup_uses_verified_git_checkout(repository_root: Path) -> None:
    text = (repository_root / "scripts" / "setup_ubuntu24.sh").read_text(
        encoding="utf-8"
    )
    assert 'fetch --depth 1 --no-tags origin "${CUROBO_COMMIT}"' in text
    assert 'remote get-url origin' in text
    assert '[[ "${actual_head}" == "${CUROBO_COMMIT}" ]]' in text
    assert "rev-parse --verify 'HEAD^{tree}'" in text
    assert '[[ "${actual_tree}" == "${CUROBO_SOURCE_TREE}" ]]' in text
    assert "status --porcelain=v1 --untracked-files=all" in text
    assert ".ur-waypoint-curobo-commit" in text
    assert ".ur-waypoint-curobo-tree" in text
    assert '"torch==${PYTORCH_VERSION}"' in text
    assert '"${SOURCE_DIR}[cu12-torch]"' in text
    assert 'str(torch.version.cuda or "") != expected_cuda_toolkit' in text
    assert 'version("cuda-core")' in text
    assert "source_archive_sha256" not in text
    assert "/archive/" not in text


def test_build_script_separates_colcon_gtest_from_repository_pytest(
    repository_root: Path,
) -> None:
    text = (repository_root / "scripts" / "build_workspace.sh").read_text(
        encoding="utf-8"
    )
    assert "--packages-select hx_waypoint_runtime" in text
    assert "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1" in text
    assert "python -m pytest -p no:cacheprovider tests -q" in text


def test_cmake_ros_packages_export_ament_build_type(repository_root: Path) -> None:
    for package_name in (
        "hx_waypoint_bringup",
        "hx_waypoint_interfaces",
        "hx_waypoint_runtime",
    ):
        root = ET.parse(
            repository_root / "src" / package_name / "package.xml"
        ).getroot()
        assert root.findtext("./export/build_type") == "ament_cmake"


def test_runtime_csv_writers_preserve_epoch_subsecond_precision(
    repository_root: Path,
) -> None:
    runtime_source = repository_root / "src" / "hx_waypoint_runtime" / "src"
    for filename in ("trajectory_recorder.cpp", "telemetry_recorder.cpp"):
        text = (runtime_source / filename).read_text(encoding="utf-8")
        assert "std::numeric_limits<double>::max_digits10" in text
        assert "std::setprecision(12)" not in text

    telemetry = (runtime_source / "telemetry_recorder.cpp").read_text(encoding="utf-8")
    assert (
        '"ros_time_s,sync_time_ns,ros_time_ns,trajectory_time_s,trajectory_time_ns,state"'
        in telemetry
    )
    assert "std::chrono::steady_clock::now().time_since_epoch()" in telemetry


def test_nucleo_protocol_provenance_is_pinned(repository_root: Path) -> None:
    lock = yaml.safe_load((repository_root / "nucleo.lock").read_text(encoding="utf-8"))
    assert lock == {
        "schema_version": 1,
        "repository": "https://github.com/hxhou0207/magnet_localization_real.git",
        "branch": "hospital",
        "commit": "b80f4ebee945c6ac07be7732621fa8cd1dc056cd",
        "protocol_path": "magloc/sources/hardware/nucleo/protocol.py",
        "protocol_blob": "9408a3043199c7338f2a6f26b43db4c4b7e5089d",
        "protocol_sha256": (
            "fe6510dfe3b819faeb5fad3f7c6d8535a6af603ea8eb08e1a2359590c428bbb3"
        ),
        "integration": "independent_wire_protocol_reimplementation",
    }


def test_unscaled_simulation_records_effective_unit_speed_scaling(
    repository_root: Path,
) -> None:
    text = (
        repository_root
        / "src"
        / "hx_waypoint_runtime"
        / "src"
        / "trajectory_recorder.cpp"
    ).read_text(encoding="utf-8")
    assert "sample.speed_scaling = 1.0;" in text
    assert "sample.speed_scaling = message->speed_scaling_factor;" not in text


def test_simulation_recorder_time_aligns_joint_state_controller_and_tf(
    repository_root: Path,
) -> None:
    text = (
        repository_root
        / "src"
        / "hx_waypoint_runtime"
        / "src"
        / "trajectory_recorder.cpp"
    ).read_text(encoding="utf-8")
    assert text.count("messageTimeOrNow(message->header.stamp, node_)") == 2
    assert "configureKinematics(const RobotValidationModel & robot)" in text
    assert "kinematics_->evaluateFrames(actual.positions)" in text
    assert "sample.transform.header.stamp = message->header.stamp" in text
    assert "lookupTransform" not in text


def test_ur_speed_scaling_percent_is_normalized_at_subscription_boundary(
    repository_root: Path,
) -> None:
    text = (
        repository_root
        / "src"
        / "hx_waypoint_runtime"
        / "src"
        / "telemetry_recorder.cpp"
    ).read_text(encoding="utf-8")
    assert "setSpeedScaling(urSpeedScalingPercentToRatio(message->data));" in text
    assert "return speed_scaling_percent * 0.01;" in text
    assert "stopped publishing fresh samples" in text


def test_urdf_generation_uses_only_the_environment_controlled_calibration(
    repository_root: Path,
) -> None:
    text = (repository_root / "scripts" / "generate_ur_assets.sh").read_text(
        encoding="utf-8"
    )
    assert "--kinematics" not in text
    assert 'KINEMATICS_FILE="${ROBOT_DIRECTORY}/driver_kinematics.yaml"' in text
    assert "asset_provenance.yaml" in text


def test_bringup_scopes_child_launch_arguments_and_preserves_moveit_rviz(
    repository_root: Path,
) -> None:
    for launch_name in ("simulation.launch.py", "real.launch.py"):
        text = (
            repository_root
            / "src"
            / "hx_waypoint_bringup"
            / "launch"
            / launch_name
        ).read_text(encoding="utf-8")

        assert "GroupAction" in text
        assert "driver_group = GroupAction(actions=[driver], scoped=True)" in text
        assert '"launch_rviz": "false"' in text
        if launch_name == "simulation.launch.py":
            assert text.count('"launch_rviz": "false"') >= 2
            assert "SetLaunchConfiguration" in text
            assert '"launch_hx_waypoint_rviz"' in text
            assert "moveit_group =" not in text
            assert 'package="rviz2"' in text
            assert 'name="rviz2_hx_waypoint"' in text
            assert 'condition=IfCondition(launch_hx_waypoint_rviz)' in text
            assert '"hx_waypoint_moveit.rviz"' in text
            assert 'name="wait_for_hx_waypoint_rviz_description"' in text
            assert "target_action=rviz_description_waiter" in text
            assert "on_exit=[rviz]" in text
            assert '"launch_rviz": launch_rviz' not in text
            assert '"simulation_robot_description.launch.py"' in text
        else:
            assert '"launch_rviz": launch_rviz' in text
            assert "EnvironmentBuilder" in text
            assert "SetLaunchConfiguration" in text
            assert '"initial_joint_controller": real_controller_name' in text
            assert '"real_action_name": real_action_name' in text
            assert 'LaunchConfiguration("manual_control")' in text
            assert text.count("UnlessCondition(manual_control)") == 3
            assert '"scaled_joint_trajectory_controller"' in text
            assert 'LaunchConfiguration("hx_waypoint_robot_type")' in text
            assert '"hx_waypoint_driver_kinematics_params_file"' in text
            assert '"hx_waypoint_robot_type", bundle.robot_type' in text
            assert (
                'str(driver_kinematics)' in text
                and 'environment / "robot" / "driver_kinematics.yaml"' in text
            )
            assert 'DeclareLaunchArgument("ur_type"' not in text
            assert 'DeclareLaunchArgument("robot_ip")' in text
            assert (
                'DeclareLaunchArgument(\n                "kinematics_params_file",'
                not in text
            )
            environment_declaration = text.split(
                'DeclareLaunchArgument(\n                "environment_id",', 1
            )[1].split("),", 1)[0]
            assert "default_value" not in environment_declaration
        assert "load_setup_file" in text
        assert '"plane1_transform"' in text
        assert '"rod_tip_transform"' in text
        assert "tcp.yaml" not in text
        assert "frames.yaml" not in text


def test_real_launch_commands_select_only_ip_environment_and_workspace(
    repository_root: Path,
) -> None:
    for relative_path in ("README.md", "docs/tcp-console.md"):
        text = (repository_root / relative_path).read_text(encoding="utf-8")
        command = text.split(
            "ros2 launch hx_waypoint_bringup real.launch.py", 1
        )[1].split("```", 1)[0]
        assert "robot_ip:=" in command
        assert "environment_id:=" in command
        assert "workspace:=" in command
        assert "ur_type:=" not in command
        assert "kinematics_params_file:=" not in command


def test_simulation_robot_description_uses_environment_start_state(
    repository_root: Path,
) -> None:
    text = (
        repository_root
        / "src"
        / "hx_waypoint_bringup"
        / "launch"
        / "simulation_robot_description.launch.py"
    ).read_text(encoding="utf-8")

    assert "EnvironmentBuilder().load(environment_root)" in text
    assert "environment.home_joint_positions" in text
    assert "environment.joint_names" in text
    assert (
        '"kinematics_params": str(environment_root / "robot/driver_kinematics.yaml")'
        in text
    )
    assert '"use_mock_hardware": "true"' in text
    assert '"mock_sensor_commands": "true"' in text
    assert "_replace_mock_initial_positions" in text
    assert 'parameter.getAttribute("name") == "initial_value"' in text

    simulation = (
        repository_root
        / "src"
        / "hx_waypoint_bringup"
        / "launch"
        / "simulation.launch.py"
    ).read_text(encoding="utf-8")
    assert 'SetEnvironmentVariable("HX_WAYPOINT_WORKSPACE", workspace)' in simulation
    assert (
        'SetEnvironmentVariable("HX_WAYPOINT_ENVIRONMENT_ID", environment_id)'
        in simulation
    )


def test_runtime_fails_closed_when_live_setup_tf_differs(
    repository_root: Path,
) -> None:
    source = (
        repository_root
        / "src"
        / "hx_waypoint_runtime"
        / "src"
        / "runtime_node.cpp"
    ).read_text(encoding="utf-8")

    assert source.count("requireLiveSetupTransforms(") >= 4
    assert '"live TF "' in source
    assert '" differs from setup.yaml' in source
    assert 'artifact->directory / "environment_snapshot"' in source


def test_python_ros_nodes_handle_ctrl_c_without_double_shutdown(
    repository_root: Path,
) -> None:
    sources = (
        repository_root
        / "src"
        / "hx_waypoint_curobo"
        / "hx_waypoint_curobo"
        / "curobo_server.py",
        repository_root
        / "src"
        / "hx_waypoint_tools"
        / "hx_waypoint_tools"
        / "planning_scene_publisher.py",
    )
    for source in sources:
        text = source.read_text(encoding="utf-8")
        assert "except KeyboardInterrupt:" in text
        assert "if rclpy.ok():\n            rclpy.shutdown()" in text
