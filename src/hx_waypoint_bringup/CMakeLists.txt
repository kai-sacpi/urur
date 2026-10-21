cmake_minimum_required(VERSION 3.8)
project(hx_waypoint_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN ".DS_Store" EXCLUDE
)

ament_package()
