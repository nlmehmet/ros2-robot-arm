cmake_minimum_required(VERSION 3.8)
project(robot_arm_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

find_package(controller_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(guarded_trajectory_controller SHARED src/guarded_trajectory_controller.cpp)
ament_target_dependencies(guarded_trajectory_controller
  controller_interface joint_trajectory_controller pluginlib rclcpp rclcpp_action trajectory_msgs
)
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)
install(TARGETS guarded_trajectory_controller LIBRARY DESTINATION lib)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
