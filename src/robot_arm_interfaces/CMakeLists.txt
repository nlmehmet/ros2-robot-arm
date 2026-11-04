cmake_minimum_required(VERSION 3.8)
project(robot_arm_interfaces)

find_package(ament_cmake REQUIRED)

# Add find_package(rosidl_default_generators REQUIRED) and
# rosidl_generate_interfaces(...) when the first reviewed interface is added.
# An empty interface package intentionally performs no code generation.
install(DIRECTORY msg srv action
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
