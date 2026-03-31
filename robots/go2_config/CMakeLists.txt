cmake_minimum_required(VERSION 3.5)
project(go2_config)

find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()