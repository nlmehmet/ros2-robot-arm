# Ubuntu 22.04 / ROS 2 Humble setup and validation

This guide targets a native Ubuntu 22.04 LTS (Jammy) machine with system Python 3.10. It prepares the five package skeletons only. No robot model, movement, Gazebo launch, or hardware integration exists yet. Commands below are target-machine instructions, not results from the current Debian review environment.

## 1. Install the supported toolchain

Check the host before installing ROS:

```bash
. /etc/os-release
test "$ID" = ubuntu && test "$VERSION_ID" = 22.04
/usr/bin/python3 --version
locale
```

Require Ubuntu 22.04 and a UTF-8 locale. Follow the official [ROS 2 Humble Ubuntu deb installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) to configure locale, Ubuntu Universe, the signed ROS apt repository, and system updates. Use the currently documented ROS signing/repository mechanism rather than an old copied key command. Do not disable signature or TLS verification. Stop on apt errors; ROS apt repositories must be configured before running the commands below.

For the headless scaffold, install ROS base and development tools:

```bash
sudo apt update
sudo apt install ros-humble-ros-base build-essential cmake \
  python3-setuptools python3-colcon-common-extensions python3-rosdep \
  python3-catkin-pkg-modules
source /opt/ros/humble/setup.bash
printf 'ROS distribution: %s\n' "$ROS_DISTRO"
command -v ros2
command -v colcon
cmake --version
c++ --version
```

Require `ROS_DISTRO=humble`. Use system Python for ROS binary compatibility; do not build inside an unrelated Python virtual environment or source another ROS distribution. Python 3.10 and C++17 are the project targets. `ros-humble-desktop` can replace ROS base if RViz and desktop tooling are needed later.

Initialize rosdep only if its default configuration does not exist:

```bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
```

Use your normal user for `rosdep update` and colcon; only system installation needs sudo. Apt, rosdep initialization, and rosdep index updates require network access. Relevant destinations include Ubuntu mirrors, `packages.ros.org`, and GitHub-hosted rosdep data; use reported failures to identify any additional required domains. Do not claim an offline install on a machine without cached packages and indexes.

## 2. Resolve workspace dependencies

Open a fresh Bash shell and change to the existing checkout root (the directory containing `src/`, `tests/`, and this README). Do not create a second nested workspace or an extra worktree.

```bash
source /opt/ros/humble/setup.bash
workspace_root="$(pwd -P)"
test -d src && test -f tests/test_workspace.py
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
rosdep check --from-paths src --ignore-src --rosdistro humble
```

Both rosdep commands must succeed. Internal `robot_arm_*` dependencies are provided by this checkout; `--ignore-src` prevents installing them from apt. External runtime dependencies include xacro, robot_state_publisher, launch/launch_ros, ros2_control controller packages, rclpy, sensor_msgs, trajectory_msgs, and setuptools. The interfaces scaffold reserves rosidl generator/runtime dependencies but does not yet generate interfaces. C++ library and export dependencies must be added when a library is introduced.

## 3. Validate manifests and build

From the same workspace root:

```bash
python3 -m unittest discover -s tests -v
python3 - <<'PY'
from pathlib import Path
from catkin_pkg.package import parse_package
manifests = sorted(Path('src').glob('*/package.xml'))
assert len(manifests) == 5, manifests
for manifest in manifests:
    warnings = []
    package = parse_package(str(manifest), warnings=warnings)
    package.validate(warnings=warnings)
    print(f'{package.name}: manifest valid')
    for warning in warnings:
        print(f'  warning: {warning}')
PY
colcon list --base-paths src
colcon build --base-paths src --symlink-install --event-handlers console_direct+
```

Stop if any command fails. `colcon list` must identify exactly these five packages: robot_arm_description, robot_arm_bringup, robot_arm_control, robot_arm_teleop, and robot_arm_interfaces. Four use ament_cmake and teleop uses ament_python. A successful build must report all five packages finished, with no failed or aborted packages. Preserve error output for diagnosis; do not infer success from generated directories alone.

CMake installs description, bringup, control configuration, and reserved interface asset directories under `share/<package>`. `ament_package()` installs the four CMake package manifests and ament index entries. Teleop's setup.py installs its manifest, resource index marker, Python module, launch assets, and config assets; setup.cfg places future executable scripts in `lib/robot_arm_teleop`. It currently defines no executable entry points.

## 4. Verify the installed overlay

Only after a successful build, in the same shell:

```bash
source "$workspace_root/install/setup.bash"
ros2 pkg list
for package in robot_arm_description robot_arm_bringup robot_arm_control robot_arm_teleop robot_arm_interfaces; do
  prefix="$(ros2 pkg prefix "$package")" || exit 1
  printf '%s: %s\n' "$package" "$prefix"
  test "$prefix" = "$workspace_root/install/$package" || exit 1
done
python3 - <<'PY'
from pathlib import Path
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import robot_arm_teleop

root = Path.cwd().resolve()
assets = {
    'robot_arm_description': ('urdf', 'meshes', 'rviz'),
    'robot_arm_bringup': ('launch', 'config'),
    'robot_arm_control': ('config',),
    'robot_arm_teleop': ('launch', 'config'),
    'robot_arm_interfaces': ('msg', 'srv', 'action'),
}
for package, directories in assets.items():
    prefix = Path(get_package_prefix(package)).resolve()
    assert prefix == root / 'install' / package, (package, prefix)
    share = Path(get_package_share_directory(package))
    assert (share / 'package.xml').is_file(), package
    for directory in directories:
        assert (share / directory).is_dir(), (package, directory)
    print(f'{package}: overlay discovery and installed assets verified')
print('robot_arm_teleop: Python import verified')
PY
```

This guide uses colcon's default isolated install layout; do not add `--merge-install` without adjusting prefix checks. A prefix in `/opt/ros/humble` or another checkout does not validate this workspace. With `--symlink-install`, the imported teleop source may resolve into `src/`; this is expected. The assertions verify this overlay's ament registration and installed assets, not robot behavior.

## 5. Run tests and inspect results

```bash
python3 -m unittest discover -s tests -v
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Run the root Python suite explicitly: it is not registered as a per-package colcon test. The optional manifest-parser test skips when catkin_pkg is absent; on this target machine it must execute after installing the tool above. Inspect test counts and failures. An empty colcon test run is expected for this scaffold and provides no runtime coverage. There are no generated custom interfaces to inspect and no node or launch command to run yet.

## Later simulation prerequisites

Only when starting the simulation phase:

```bash
apt-cache policy ros-humble-ros-gz ros-humble-ign-ros2-control
sudo apt install ros-humble-ros-gz ros-humble-ign-ros2-control
ign gazebo --versions
```

Target **Gazebo Fortress (Gazebo Sim 6)** with **ROS 2 Humble**. Check that apt reports an installable Candidate for both packages before installation. The Humble/Fortress ros2_control integration uses the historical `ign_ros2_control` name; Fortress also uses the `ign gazebo` CLI. Do not replace these with packages for a newer ROS distribution or an incompatible Gazebo release. The plugin creates the simulation controller manager inside Gazebo; do not additionally launch ros2_control_node for the same simulated robot. Joystick drivers, device access, STM32 tooling, and physical hardware validation belong to later phases.

Require a reported Gazebo Sim major version of 6. Package candidates and these simulator commands have not been verified in this Debian environment. If a package has no Candidate, stop and diagnose the configured Humble apt repository and architecture rather than switching ROS distributions. Consult the official [Humble/Gazebo pairing guide](https://gazebosim.org/docs/fortress/ros_installation/) and the [Humble control integration source](https://github.com/ros-controls/gz_ros2_control/tree/humble) when resolving version-specific names. External references could not be fetched during this review because the configured network proxy was unreachable.

Future bringup must bridge the simulation clock through ros_gz_bridge, set `use_sim_time` on simulation consumers, spawn the robot through ros_gz_sim, and load controllers through the plugin-owned manager. Installing dependencies or querying a simulator version does not validate these runtime behaviors.

## Evidence and remaining checks

| Check | Current review environment | Required Ubuntu result |
| --- | --- | --- |
| Five manifests: XML, metadata, dependency structure | Covered by offline tests | catkin_pkg parser/validation also passes |
| CMake/ament and Python install declarations | Static checks; setuptools metadata inspected | Real colcon configure/build/install passes |
| Python syntax | Parsed offline on Python 3.12 | Re-run suite using system Python 3.10 |
| ROS package discovery, installed assets, teleop import | Not run; ROS unavailable | All five resolve from this overlay; asset/import assertions pass |
| ROS dependency resolution | Not run | rosdep install/check succeeds |
| Gazebo, trajectories, joystick, STM32 | Not implemented or tested | Validate in their respective future phases |

Offline tests cannot establish ament configuration, ROS dependency availability, ABI compatibility, physics, or hardware safety. Maintainer email and proposed Apache-2.0 license metadata remain provisional pending release review.
