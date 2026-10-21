import re
from pathlib import Path
from xml.dom import Node as XmlNode

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from hx_waypoint_curobo.environment_builder import EnvironmentBuilder


def _replace_mock_initial_positions(document, joint_names, positions):
    controls = document.getElementsByTagName("ros2_control")
    if len(controls) != 1:
        raise RuntimeError("UR robot description must contain exactly one ros2_control block")

    requested = dict(zip(joint_names, positions, strict=True))
    replaced = set()
    for child in controls[0].childNodes:
        if child.nodeType != XmlNode.ELEMENT_NODE or child.tagName != "joint":
            continue
        joint_name = child.getAttribute("name")
        if joint_name not in requested:
            continue
        position_interfaces = [
            interface
            for interface in child.getElementsByTagName("state_interface")
            if interface.getAttribute("name") == "position"
        ]
        if len(position_interfaces) != 1:
            raise RuntimeError(
                f"mock joint {joint_name} must contain exactly one position state interface"
            )
        initial_values = [
            parameter
            for parameter in position_interfaces[0].getElementsByTagName("param")
            if parameter.getAttribute("name") == "initial_value"
        ]
        if len(initial_values) != 1:
            raise RuntimeError(
                f"mock joint {joint_name} must contain exactly one initial_value parameter"
            )
        parameter = initial_values[0]
        while parameter.firstChild is not None:
            parameter.removeChild(parameter.firstChild)
        parameter.appendChild(document.createTextNode(format(float(requested[joint_name]), ".12g")))
        replaced.add(joint_name)

    missing = sorted(set(requested).difference(replaced))
    if missing:
        raise RuntimeError(
            "UR robot description did not expose mock initial positions for: " + ", ".join(missing)
        )


def _robot_state_publisher(context):
    workspace = Path(LaunchConfiguration("workspace").perform(context)).expanduser().resolve()
    environment_id = LaunchConfiguration("environment_id").perform(context)
    if not re.fullmatch(r"[A-Za-z0-9_-]+", environment_id):
        raise RuntimeError("environment_id is not path-safe")
    environments_root = (workspace / "environments").resolve()
    environment_root = (environments_root / environment_id).resolve()
    if environment_root.parent != environments_root:
        raise RuntimeError("environment path escapes environments/")

    environment = EnvironmentBuilder().load(environment_root)
    ur_type = LaunchConfiguration("ur_type").perform(context)
    if environment.robot_type != ur_type:
        raise RuntimeError(
            f"environment robot type {environment.robot_type!r} does not match ur_type "
            f"{ur_type!r}"
        )

    ur_driver_share = Path(get_package_share_directory("ur_robot_driver"))
    ur_description_share = Path(get_package_share_directory("ur_description"))
    ur_client_share = Path(get_package_share_directory("ur_client_library"))
    type_config = ur_description_share / "config" / ur_type
    mappings = {
        "name": ur_type,
        "ur_type": ur_type,
        "robot_ip": LaunchConfiguration("robot_ip").perform(context),
        "joint_limit_params": str(type_config / "joint_limits.yaml"),
        "kinematics_params": str(environment_root / "robot/driver_kinematics.yaml"),
        "physical_params": str(type_config / "physical_parameters.yaml"),
        "visual_params": str(type_config / "visual_parameters.yaml"),
        "safety_limits": "true",
        "safety_pos_margin": "0.15",
        "safety_k_position": "20",
        "script_filename": str(ur_client_share / "resources/external_control.urscript"),
        "input_recipe_filename": str(ur_driver_share / "resources/rtde_input_recipe.txt"),
        "output_recipe_filename": str(ur_driver_share / "resources/rtde_output_recipe.txt"),
        "tf_prefix": "",
        "use_mock_hardware": "true",
        "mock_sensor_commands": "true",
        "headless_mode": "false",
        "use_tool_communication": "false",
    }
    document = xacro.process_file(
        str(ur_driver_share / "urdf/ur.urdf.xacro"),
        mappings=mappings,
    )
    _replace_mock_initial_positions(
        document,
        environment.joint_names,
        environment.home_joint_positions,
    )
    robot_description = ParameterValue(document.toxml(), value_type=str)
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[{"robot_description": robot_description}],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type"),
            DeclareLaunchArgument("robot_ip"),
            DeclareLaunchArgument(
                "workspace",
                default_value=EnvironmentVariable("HX_WAYPOINT_WORKSPACE"),
            ),
            DeclareLaunchArgument(
                "environment_id",
                default_value=EnvironmentVariable("HX_WAYPOINT_ENVIRONMENT_ID"),
            ),
            OpaqueFunction(function=_robot_state_publisher),
        ]
    )
