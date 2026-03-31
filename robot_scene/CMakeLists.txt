cmake_minimum_required(VERSION 3.8)
project(robot_scene)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch maps worlds models xacro scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()