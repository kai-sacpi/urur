cmake_minimum_required(VERSION 3.8)
project(hx_waypoint_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/BuildRecording.action"
  "action/PlayRecording.action"
  "msg/ExecutionTelemetry.msg"
  "srv/PlanJointPath.srv"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs sensor_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
