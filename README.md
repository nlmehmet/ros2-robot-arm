# ROS 2 Robot Arm

A modular starting point for a six-DOF robotic arm. Target: **Ubuntu 22.04 LTS, ROS 2 Humble, Gazebo Fortress (Gazebo Sim 6), Python 3.10, C++17, colcon, and ament**.

This repository currently contains package skeletons and offline validation only. Robot geometry, simulation, controllers, joystick nodes, and STM32 communication will be implemented in later phases.

## Workspace

The repository root is the colcon workspace. `src/` contains:

- `robot_arm_description` — ament_cmake; future URDF/Xacro, meshes, and RViz assets.
- `robot_arm_bringup` — ament_cmake; future launch composition and environment configuration.
- `robot_arm_control` — ament_cmake; future controller configuration and C++ hardware plugin.
- `robot_arm_teleop` — ament_python; future joystick command translation.
- `robot_arm_interfaces` — ament_cmake; reserved message/service/action contracts, with no generated interfaces yet.

See [architecture](docs/architecture.md), [development phases](docs/development_plan.md), [Ubuntu installation and validation](docs/ubuntu_setup.md), and [engineering rules](AGENTS.md).

## Offline validation

From the repository root, with Python 3 installed:

```bash
python3 -m unittest discover -s tests -v
```

These tests validate package metadata, local dependency boundaries, install rules, and Python packaging without downloading dependencies. They do not replace a ROS build.

## Target machine setup

Install ROS 2 Humble using the official Ubuntu deb instructions at <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>. Keep repository signatures and TLS verification enabled. Then install the scaffold development tools:

```bash
sudo apt update
sudo apt install build-essential cmake python3-setuptools python3-colcon-common-extensions python3-rosdep ros-humble-ros-base
# Run once per machine, only if rosdep has not already been initialized:
sudo rosdep init
rosdep update
```

With this checkout as the current directory:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon list
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
ros2 pkg prefix robot_arm_description
ros2 pkg prefix robot_arm_bringup
ros2 pkg prefix robot_arm_control
ros2 pkg prefix robot_arm_teleop
ros2 pkg prefix robot_arm_interfaces
python3 -m unittest discover -s tests -v
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

All five packages must build and resolve from this workspace's install directory. The scaffold has no ROS runtime tests yet; `colcon test` may report zero tests. The root structural suite is run separately and must execute nonzero tests.

For phase 2, install the simulator integration separately:

```bash
sudo apt install ros-humble-ros-gz ros-humble-ign-ros2-control
```

There is no launch command at this stage. Fortress is the Gazebo Sim generation paired with ROS 2 Humble; Humble-era integration packages and APIs may retain Ignition/`ign` names. See the Ubuntu guide for package availability checks before installing. Joystick drivers, STM32 toolchains, physical devices, and GUI tools are not required for phase 0.

## Validation limitations

The initial scaffold was checked offline in a Debian environment without ROS 2 or colcon. A Humble build, Gazebo startup, and controller behavior have not been validated there. Run the target-machine checks above before treating the ROS workspace as build-verified.

Package maintainer contact is provisional. Confirm contact details and the proposed Apache-2.0 license metadata before distribution.
