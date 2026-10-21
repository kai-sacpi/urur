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
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

from hx_waypoint_curobo.setup_config import RigidTransform, load_setup_file


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
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_hx_waypoint_rviz = LaunchConfiguration("launch_hx_waypoint_rviz")
    workspace = LaunchConfiguration("workspace")
    environment_id = LaunchConfiguration("environment_id")
    runtime_config = PathJoinSubstitution(
        [FindPackageShare("hx_waypoint_bringup"), "config", "runtime_simulation.yaml"]
    )
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": "0.0.0.0",
            "use_mock_hardware": "true",
            "mock_sensor_commands": "true",
            "description_launchfile": PathJoinSubstitution(
                [
                    FindPackageShare("hx_waypoint_bringup"),
                    "launch",
                    "simulation_robot_description.launch.py",
                ]
            ),
            "initial_joint_controller": "joint_trajectory_controller",
            "launch_dashboard_client": "false",
            "launch_rviz": "false",
        }.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
            )
        ),
        launch_arguments={"ur_type": ur_type, "launch_rviz": "false"}.items(),
    )
    # The driver include can be scoped because it starts all of its actions
    # immediately.  The MoveIt include cannot: its wait-for-description event
    # handler resolves launch arguments after the include returns.  Preserve
    # the user's RViz choice under an internal name, then let the unscoped
    # MoveIt include set its own launch_rviz argument to false.
    driver_group = GroupAction(actions=[driver], scoped=True)
    preserve_rviz_choice = SetLaunchConfiguration(
        "launch_hx_waypoint_rviz",
        launch_rviz,
    )
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": ur_type})
        .to_moveit_configs()
    )
    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
    }
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_hx_waypoint",
        condition=IfCondition(launch_hx_waypoint_rviz),
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("hx_waypoint_bringup"),
                    "config",
                    "hx_waypoint_moveit.rviz",
                ]
            ),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {"use_sim_time": False},
        ],
    )
    rviz_description_waiter = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        name="wait_for_hx_waypoint_rviz_description",
        condition=IfCondition(launch_hx_waypoint_rviz),
        output="screen",
    )
    rviz_start_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz_description_waiter,
            on_exit=[rviz],
        )
    )
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
            },
        ],
    )
    curobo_server = Node(
        package="hx_waypoint_curobo",
        executable="curobo_server",
        name="hx_waypoint_curobo_server",
        output="screen",
        parameters=[{"workspace": workspace, "environment_id": environment_id}],
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
        )
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
    setup_tfs = OpaqueFunction(function=_static_tfs_from_environment)
    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur5"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument(
                "workspace",
                default_value=os.getcwd(),
                description="ur-waypoint repository root containing environments/ and runs/",
            ),
            DeclareLaunchArgument(
                "environment_id",
                default_value="lab_wall_a",
                description="Environment directory whose zones.yaml is applied to MoveIt",
            ),
            SetEnvironmentVariable("HX_WAYPOINT_WORKSPACE", workspace),
            SetEnvironmentVariable("HX_WAYPOINT_ENVIRONMENT_ID", environment_id),
            preserve_rviz_choice,
            driver_group,
            moveit,
            rviz_start_handler,
            rviz_description_waiter,
            setup_tfs,
            planning_scene_guard,
            planning_scene,
            curobo_server_guard,
            curobo_server,
            runtime,
        ]
    )
