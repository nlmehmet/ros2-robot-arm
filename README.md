# ROS 2 Robot Arm

A modular starting point for a six-DOF robotic arm. Target: **Ubuntu 22.04 LTS, ROS 2 Humble, Gazebo Fortress (Gazebo Sim 6), Python 3.10, C++17, colcon, and ament**.

The workspace includes an illustrative six-joint Xacro reference arm, headless Fortress simulation, and automated motion and fault tests. Phase 4 adds opt-in [simulation joystick control](docs/phase4_sim_joystick.md).

## Workspace

The repository root is the colcon workspace. `src/` contains:

- `robot_arm_description` — ament_cmake package with the illustrative six-joint Xacro model and its RViz configuration. Measured geometry and meshes are planned.
- `robot_arm_bringup` — ament_cmake; reference arm launch composition and environment configuration.
- `robot_arm_control` — ament_cmake; future controller configuration and C++ hardware plugin.
- `robot_arm_teleop` — ament_python; joystick command translation for the reference arm simulation.
- `robot_arm_interfaces` — ament_cmake; reserved message/service/action contracts, with no generated interfaces yet.

See [architecture](docs/architecture.md), [development phases](docs/development_plan.md), [reference model](docs/reference_robot_model.md), [Ubuntu installation and validation](docs/ubuntu_setup.md), and [engineering rules](AGENTS.md).

## Offline validation

From the repository root, with Python 3 installed:

```bash
python3 -m unittest discover -s tests -v
```

These tests validate package metadata, local dependency boundaries, install rules, Python packaging, and the reference model's structure. A Xacro expansion test runs only if xacro is available. They do not replace a ROS build.

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

All five packages must build and resolve from this workspace's install directory. Colcon exercises the installed headless TF and Fortress launches, including trajectory faults and synthetic joystick input. Run the root structural suite separately.

For phase 2, install the simulator integration separately:

```bash
sudo apt install ros-humble-ros-gz ros-humble-ign-ros2-control
```

See the [reference arm launch guide](src/robot_arm_bringup/launch/README.md) for Phase 1B bringup. Fortress is the Gazebo Sim generation paired with ROS 2 Humble; Humble-era integration packages and APIs may retain Ignition/`ign` names. See the Ubuntu guide for package availability checks before installing. Joystick drivers, STM32 toolchains, physical devices, and GUI tools are not required for phase 0.

## Validation limitations

Phase 1B was built and tested on Ubuntu 22.04 with ROS 2 Humble. Both headless launch modes passed TF and launch shutdown checks. Interactive RViz rendering remains to be verified for the full Phase 1 gate.

Package maintainer contact is provisional. Confirm contact details and the proposed Apache-2.0 license metadata before distribution.
