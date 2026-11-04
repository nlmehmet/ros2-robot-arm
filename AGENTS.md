# Engineering rules

- Target Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress (Gazebo Sim 6), Python 3.10, and C++17.
- Use the existing isolated checkout. Do not create a Git worktree unless requested.
- Keep packages under `src/`; keep bringup, description, control, teleop, and interfaces separate.
- Build with colcon and ament. Source `/opt/ros/humble/setup.bash` before building and `install/setup.bash` afterward.
- Use standard ROS interfaces (especially FollowJointTrajectory) before adding custom contracts.
- Use SI units, radians, REP-103 frames, explicit joint limits, and stable joint names.
- Keep simulation and physical hardware launch paths separate. Hardware must default to disabled actuation.
- In Gazebo Fortress, use the controller manager created by the Humble/Fortress ign_ros2_control plugin; do not also launch ros2_control_node. Use ros2_control_node only for the separate hardware backend.
- Follow `docs/ubuntu_setup.md` for target-machine checks. Declare direct dependencies in the consuming package; distinguish runtime-only dependencies from future C++ build/export dependencies.
- Never command hardware without limits, watchdogs, a stop path, and a reviewed safety procedure.
- Keep teleop independent of the transport; do not bypass ros2_control from joystick nodes.
- Keep STM32 protocol handling behind a future ros2_control hardware plugin; isolate transport tests from real devices.
- Prefer small, testable changes. Do not add a complete simulation or speculative hardware protocol during scaffolding.
- Run `python3 -m unittest discover -s tests -v` for structural changes; with ROS installed also build and run colcon tests.
- Report passed, failed, skipped, and unavailable checks separately. Offline validation is not evidence of a successful ROS build.
- Never disable TLS, signature, or checksum verification to install dependencies. Do not commit credentials or generated build outputs.
- Update architecture and acceptance criteria when package responsibilities or external dependencies change.
- Maintainer contact metadata is provisional and must be replaced before release; confirm project licensing before distribution.
