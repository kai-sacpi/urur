import os
import re
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_curobo.setup_config import RigidTransform, load_setup_file


def _deployment_contract_from_environment(context):
    workspace = Path(LaunchConfiguration("workspace").perform(context)).expanduser().resolve()
    environment_id = LaunchConfiguration("environment_id").perform(context)
    if not re.fullmatch(r"[A-Za-z0-9_-]+", environment_id):
        raise RuntimeError("environment_id is not path-safe")
    environments_root = (workspace / "environments").resolve()
    environment = (environments_root / environment_id).resolve()
    if environment.parent != environments_root:
        raise RuntimeError("environment path escapes environments/")
    bundle = EnvironmentBuilder().load(environment)
    real = bundle.controllers["real"]
    manual_value = LaunchConfiguration("manual_control").perform(context).strip().lower()
    if manual_value not in {"true", "false"}:
        raise RuntimeError("manual_control must be true or false")
    initial_controller = (
        "scaled_joint_trajectory_controller"
        if manual_value == "true"
        else real["controller_type"]
    )
    driver_kinematics = environment / "robot" / "driver_kinematics.yaml"
    return [
        SetLaunchConfiguration("hx_waypoint_robot_type", bundle.robot_type),
        SetLaunchConfiguration(
            "hx_waypoint_driver_kinematics_params_file", str(driver_kinematics)
        ),
        SetLaunchConfiguration("hx_waypoint_real_controller_name", initial_controller),
        SetLaunchConfiguration("hx_waypoint_real_action_name", real["action_name"]),
    ]


def _static_tfs_from_environment(context):
    workspace = Path(LaunchConfiguration("workspace").perform(context)).expanduser().resolve()
    environment_id = LaunchConfiguration("environment_id").perform(context)
    if not re.fullmatch(r"[A-Za-z0-9_-]+", environment_id):
        raise RuntimeError("environment_id is not path-safe")
    environments_root = (workspace / "environments").resolve()
    environment = (environments_root / environment_id).resolve()
    if environment.parent != environments_root:
        raise RuntimeError("environment path escapes environments/")
    setup = load_setup_file(environment / "setup.yaml")

    def publisher(name, transform, parent_frame, child_frame):
        values = (*transform.translation_m, *transform.quaternion_xyzw)
        return Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=name,
            arguments=[
                "--x", str(values[0]), "--y", str(values[1]), "--z", str(values[2]),
                "--qx", str(values[3]), "--qy", str(values[4]), "--qz", str(values[5]),
                "--qw", str(values[6]), "--frame-id", parent_frame,
                "--child-frame-id", child_frame,
            ],
        )

    work = setup.work_frame
    actions = [
        publisher(
            "plane1_transform",
            work.parent_to_child,
            work.parent_frame,
            work.child_frame,
        )
    ]
    for alias in work.aliases:
        actions.append(
            publisher(
                f"{alias}_transform",
                RigidTransform(
                    translation_m=(0.0, 0.0, 0.0),
                    quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                ),
                work.child_frame,
                alias,
            )
        )
    actions.append(
        publisher(
            "rod_tip_transform",
            setup.tool.parent_to_tip,
            setup.tool.parent_link,
            setup.tool.tip_frame,
        )
    )
    return actions


def generate_launch_description():
    ur_type = LaunchConfiguration("hx_waypoint_robot_type")
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")
    workspace = LaunchConfiguration("workspace")
    environment_id = LaunchConfiguration("environment_id")
    manual_control = LaunchConfiguration("manual_control")
    kinematics_params_file = LaunchConfiguration(
        "hx_waypoint_driver_kinematics_params_file"
    )
    real_controller_name = LaunchConfiguration("hx_waypoint_real_controller_name")
    real_action_name = LaunchConfiguration("hx_waypoint_real_action_name")
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(
            Path("srdf") / "ur.srdf.xacro", {"name": ur_type}
        )
        .to_moveit_configs()
    )
    runtime_config = PathJoinSubstitution(
        [FindPackageShare("hx_waypoint_bringup"), "config", "runtime_real.yaml"]
    )
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "kinematics_params_file": kinematics_params_file,
            "initial_joint_controller": real_controller_name,
            "launch_rviz": "false",
        }.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": launch_rviz,
            # The environment-bound manual stack starts its own Servo node with
            # a trajectory output matched to the already-active scaled JTC.
            "launch_servo": "false",
        }.items(),
    )
    # Keep the driver child launch arguments isolated.  Its private
    # ``launch_rviz=false`` must not suppress the MoveIt RViz selected by the
    # parent launch argument.
    driver_group = GroupAction(actions=[driver], scoped=True)
    runtime = Node(
        package="hx_waypoint_runtime",
        executable="runtime_node",
        name="hx_waypoint_runtime",
        output="screen",
        parameters=[
            runtime_config,
            {
                "environments_root": PathJoinSubstitution([workspace, "environments"]),
                "telemetry_root": PathJoinSubstitution([workspace, "runs"]),
                "robot_type": ur_type,
                "driver_kinematics_params_file": kinematics_params_file,
                "real_action_name": real_action_name,
            },
        ],
        condition=UnlessCondition(manual_control),
    )
    curobo_server = Node(
        package="hx_waypoint_curobo",
        executable="curobo_server",
        name="hx_waypoint_curobo_server",
        output="screen",
        parameters=[{"workspace": workspace, "environment_id": environment_id}],
        condition=UnlessCondition(manual_control),
    )
    curobo_server_guard = RegisterEventHandler(
        OnProcessExit(
            target_action=curobo_server,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="cuRobo prepare server exited; refusing unplanned auto-prepare"
                    )
                )
            ],
        ),
        condition=UnlessCondition(manual_control),
    )
    planning_scene = Node(
        package="hx_waypoint_tools",
        executable="hx_waypoint_planning_scene",
        name="hx_waypoint_planning_scene",
        output="screen",
        parameters=[
            {
                "workspace": workspace,
                "environment_id": environment_id,
            }
        ],
    )
    planning_scene_guard = RegisterEventHandler(
        OnProcessExit(
            target_action=planning_scene,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="hx_waypoint planning scene publisher exited; refusing to run "
                        "with an incomplete MoveIt scene"
                    )
                )
            ],
        )
    )
    manual_servo = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            PathJoinSubstitution(
                [FindPackageShare("hx_waypoint_bringup"), "config", "manual_servo.yaml"]
            ),
        ],
        condition=IfCondition(manual_control),
    )
    manual_servo_guard = RegisterEventHandler(
        OnProcessExit(
            target_action=manual_servo,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="MoveIt Servo exited; refusing responsive manual TCP control"
                    )
                )
            ],
        ),
        condition=IfCondition(manual_control),
    )
    setup_tfs = OpaqueFunction(function=_static_tfs_from_environment)
    deployment_contract = OpaqueFunction(function=_deployment_contract_from_environment)
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument(
                "workspace",
                default_value=os.getcwd(),
                description="ur-waypoint repository root containing environments/ and runs/",
            ),
            DeclareLaunchArgument(
                "environment_id",
                description=(
                    "Required environment directory; one selection binds setup TFs, "
                    "TCP/rod geometry, collision scene, robot model/kinematics, "
                    "planner constraints, and the formal playback controller"
                ),
            ),
            DeclareLaunchArgument(
                "manual_control",
                default_value="false",
                description=(
                    "Start the collision-checked manual TCP console stack without the "
                    "recording runtime or cuRobo prepare server"
                ),
            ),
            deployment_contract,
            driver_group,
            moveit,
            setup_tfs,
            planning_scene_guard,
            planning_scene,
            manual_servo_guard,
            manual_servo,
            curobo_server_guard,
            curobo_server,
            runtime,
        ]
    )
