# Development plan

Each phase depends on completion of the previous phase unless noted. Run structural tests throughout; add behavior tests alongside each implementation.

| Phase | Scope and dependencies | Acceptance criteria |
| --- | --- | --- |
| 0 — Scaffold | Five ament packages; Python standard library for offline checks; Humble and colcon for build checks | Structural tests pass; on the target host all five packages build and are discoverable. No robot runtime is claimed. |
| 1 — Robot description | Reviewed six-joint geometry, axes, limits, inertias; xacro and robot_state_publisher | Xacro expands to a valid URDF; six independently movable joints; connected TF tree; positive inertias; mesh resources resolve after install. Validate in RViz when a display is available. |
| 2 — Simulation and control | Phase 1; Gazebo Fortress (Gazebo Sim 6), ros_gz_sim, ros_gz_bridge, ign_ros2_control, controller_manager, joint_state_broadcaster, joint_trajectory_controller | Headless world starts; robot spawns once; controllers activate; six joint states publish; FollowJointTrajectory goal reaches tolerance and returns success; out-of-limit commands are rejected. |
| 3 — Repeatable motion tests | Phase 2; control_msgs action client and launch_testing | Automated startup, nominal motion, invalid goal, timeout, and shutdown tests pass; clean restart works; results include joint errors and controller state. |
| 4 — Joystick control | Phase 3; joy, sensor_msgs, teleop adapter; choose mapping and command mode | Configurable deadman and scaling; bounded commands; disconnect and stale input stop commands safely; tests use synthetic Joy messages without a device. |
| 5 — STM32 integration | Reviewed electrical/safety design and protocol; C++17 ros2_control hardware plugin; selected serial or CAN transport | Mock transport passes framing, checksum, timeout, reconnect, and error tests; simulation still passes; supervised bench test verifies limits, watchdog, stop path, and feedback before powered motion. |

## Decisions before implementation

- Confirm robot geometry, actuator limits, payload, and joint naming before phase 1.
- Pin compatible Humble package versions in deployment/CI once a target machine is available. Use Fortress-compatible Humble binaries and Ignition-era plugin APIs; do not substitute packages for another ROS distribution or Gazebo generation.
- Choose joystick mapping during phase 4 and STM32 transport/protocol during phase 5; neither blocks the scaffold.
- Use standard trajectory actions first. Add custom interfaces only with documented consumers and compatibility requirements.
- Confirm maintainer contact and licensing before release.

## Validation gates

Offline structural tests do not exercise ROS dependency resolution, ament configuration, generated interfaces, controllers, physics, or device access. Required phase-0 target checks are `colcon build`, package discovery, and `colcon test` with results inspected. Later phases must supply runtime tests; an empty colcon test run cannot validate simulation behavior. Hardware acceptance always requires separate supervised testing.
