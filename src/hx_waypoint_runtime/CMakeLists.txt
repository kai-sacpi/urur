cmake_minimum_required(VERSION 3.8)
project(hx_waypoint_runtime)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hx_waypoint_interfaces REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_core SHARED
  src/canonical_trajectory_validator.cpp
  src/playback_executor.cpp
  src/prepare_workflow.cpp
  src/recording_store.cpp
  src/robot_validation_model.cpp
  src/runtime_policy.cpp
  src/safety_gate.cpp
  src/setup_config.cpp
  src/simulation_recording_validator.cpp
  src/simulation_runner.cpp
  src/telemetry_recorder.cpp
  src/tip_kinematics.cpp
  src/trajectory_recorder.cpp
)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core
  builtin_interfaces
  control_msgs
  controller_manager_msgs
  geometry_msgs
  hx_waypoint_interfaces
  kdl_parser
  orocos_kdl_vendor
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  tf2_ros
  trajectory_msgs
  urdf
)
target_link_libraries(${PROJECT_NAME}_core yaml-cpp)

add_executable(runtime_node src/runtime_node.cpp)
target_link_libraries(runtime_node ${PROJECT_NAME}_core yaml-cpp)
ament_target_dependencies(runtime_node
  control_msgs
  controller_manager_msgs
  hx_waypoint_interfaces
  kdl_parser
  orocos_kdl_vendor
  rclcpp
  rclcpp_action
  sensor_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}_core runtime_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_runtime_core test/test_runtime_core.cpp)
  if(TARGET test_runtime_core)
    target_link_libraries(test_runtime_core ${PROJECT_NAME}_core)
    ament_target_dependencies(
      test_runtime_core control_msgs controller_manager_msgs rclcpp rclcpp_action sensor_msgs
      trajectory_msgs)
  endif()
  ament_add_gtest(test_simulation_contract test/test_simulation_contract.cpp)
  if(TARGET test_simulation_contract)
    target_link_libraries(test_simulation_contract ${PROJECT_NAME}_core)
    ament_target_dependencies(test_simulation_contract controller_manager_msgs rclcpp)
  endif()
  ament_add_gtest(test_setup_config test/test_setup_config.cpp)
  if(TARGET test_setup_config)
    target_link_libraries(test_setup_config ${PROJECT_NAME}_core yaml-cpp)
  endif()
  ament_add_gtest(test_callback_group_responsiveness test/test_callback_group_responsiveness.cpp)
  if(TARGET test_callback_group_responsiveness)
    target_link_libraries(test_callback_group_responsiveness ${PROJECT_NAME}_core)
    ament_target_dependencies(
      test_callback_group_responsiveness control_msgs rclcpp rclcpp_action trajectory_msgs)
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  builtin_interfaces
  control_msgs
  controller_manager_msgs
  geometry_msgs
  hx_waypoint_interfaces
  kdl_parser
  orocos_kdl_vendor
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  tf2_ros
  trajectory_msgs
  urdf
  yaml-cpp
)
ament_package()
