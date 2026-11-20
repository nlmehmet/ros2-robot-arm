cmake_minimum_required(VERSION 3.8)
project(robot_arm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(reference_arm_launch test/test_reference_arm_launch.py TIMEOUT 60)
endif()

ament_package()
