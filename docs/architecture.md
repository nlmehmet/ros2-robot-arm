# Architecture

Target: Ubuntu 22.04 LTS / ROS 2 Humble / Gazebo Fortress (Gazebo Sim 6). Phase 1B provides a headless launch for the illustrative six-joint reference model. The diagram shows the planned simulation and hardware architecture. ROS 2 remains Humble. Use its Fortress-compatible integration, including the Ignition names retained in Humble-era APIs.

```mermaid
flowchart TB
    Bringup[robot_arm_bringup: launch composition]
    Description[robot_arm_description: URDF / Xacro / meshes]
    RSP[robot_state_publisher]
    Gazebo["Gazebo Fortress (Gazebo Sim 6)"]
    Bridge[ros_gz_bridge: simulation clock]
    Sim[ign_ros2_control simulated hardware]
    CM[ros2_control controller_manager]
    JTC[joint_trajectory_controller]
    JSB[joint_state_broadcaster]
    Teleop[robot_arm_teleop: Fortress joystick adapter]
    Joy[joy: joystick driver]
    Client[Trajectory action client]
    HW[Future ros2_control hardware plugin]
    MCU[STM32 motor firmware]
    Interfaces[robot_arm_interfaces: future custom contracts]
    Control[robot_arm_control: controller configuration / future C++ plugin]
    Bringup --> RSP
    Bringup --> Gazebo
    Bringup --> Bridge
    Bringup -->|hardware mode only: ros2_control_node| CM
    Description --> RSP
    Description --> Gazebo
    Gazebo -->|loads plugin| Sim
    Gazebo -->|Gazebo Transport clock| Bridge
    Bridge -->|ROS /clock| RSP
    Sim -->|simulation mode: creates manager| CM
    CM -->|reads and writes joints| Sim
    Control --> CM
    CM --> JTC
    CM --> JSB
    Client -->|FollowJointTrajectory| JTC
    Joy -->|sensor_msgs/Joy| Teleop
    Teleop -->|bounded waypoints in simulation| JTC
    JSB -->|joint_states| RSP
    CM --> HW
    HW -->|reviewed transport protocol| MCU
    Interfaces -.-> HW
```

Simulation and physical hardware are mutually exclusive backends for the same six-joint controller contract. Description owns geometry, inertias, limits, and ros2_control declarations. Control owns controller configuration and the future C++17 hardware plugin. Bringup owns composition and startup ordering. Teleop translates joystick input into standard trajectory commands. Interfaces reserves custom contracts for cases where standard ROS messages are insufficient.

In Gazebo Fortress mode, the `ign_ros2_control` plugin creates the controller manager inside Gazebo. Bringup starts Gazebo, spawns the robot, then loads controllers through that manager's services; it must not start a second `ros2_control_node`. In physical hardware mode, bringup instead starts `ros2_control_node` with the future hardware plugin. The shared manager node in the diagram represents these mutually exclusive alternatives.

Fortress is Gazebo Sim major version 6. Use `ros_gz_sim` for spawning and simulator launch integration, and `ros_gz_bridge` for selected Gazebo Transport topics such as the simulation clock. Joint states and trajectory commands use the ros2_control ROS interfaces directly. In simulation, enable `use_sim_time` consistently for ROS nodes and bridge `/clock`; the diagram shows robot_state_publisher as one consumer. The Humble/Fortress control integration retains the historical `ign_ros2_control` package name. Plugin class and library identifiers must be checked against the installed Humble/Fortress version when the model is implemented; the Phase 2B simulation Xacro records the identifiers validated on Humble and Fortress.

The control package builds a C++17 trajectory controller plugin for simulation and installs its library and pluginlib registration. Bringup declares the simulator and controller integration as runtime dependencies and uses `control_msgs` in its integration tests. Packages install assets under `share/<package>`. Teleop also installs a Python console entry point.

The Phase 1A reference model uses `joint_1` through `joint_6` with illustrative Z/Y/Y/X/Y/Z axes. Dimensions, limits, inertias, and frames are not physical hardware specifications; update rates and controller tuning still require review. No physical dimensions or motor capabilities are assumed here.

The `joy` driver is an optional input source. The adapter consumes `sensor_msgs/Joy`, uses `rosgraph_msgs/Clock` and joint states to check simulation feedback, and publishes `trajectory_msgs/JointTrajectory`. RViz is an optional visualization tool.

The Phase 1B launch in `robot_arm_bringup` expands the Xacro from `robot_arm_description` and passes it to `robot_state_publisher`. By default, `joint_state_publisher` supplies the reference pose on `/joint_states`. External joint states can replace that source for kinematic inspection. RViz is opt-in and uses `base_link` as the fixed frame. See the [reference model guide](reference_robot_model.md).

Phase 2A adds a Fortress world in bringup and a standalone system test for gravity,
conversion and spawning, the Gazebo-to-ROS clock bridge, and shutdown/restart.
Bringup declares the ROS/Gazebo integration as test dependencies for this phase.
The test uses the existing illustrative arm with a free base and checks its
spawned joint inventory. See [Phase 2A validation](phase2a_smoke.md).

Phase 2B adds a simulation Xacro that anchors the illustrative arm to the
world, a controller configuration in control, and a dedicated headless
launch in bringup. Bringup depends on the Humble `gz_ros2_control` package,
which ships the Fortress `ign_ros2_control` compatibility plugin, and uses
`control_msgs` in its standalone trajectory test. The test gates a six-joint
goal on a successful single-joint goal and fresh measured state. The controller
guards action and topic waypoints using command bounds derived from the
expanded URDF, with clearance from the simulated hard stops. See
[Phase 2B control](phase2b_control.md).

Phase 3 registers a simulation integration test in bringup. It reuses the
Phase 2B launch, controller readiness checks, and scoped process cleanup.
The verifier sends standard trajectory actions and checks measured joint state
after success, rejection, timeout, and cancellation. See the
[Phase 3 test guide](phase3_motion_tests.md).

Phase 4 adds an opt-in joystick node to the Fortress launch. Bringup supplies
the same interior position limits used by the trajectory guard and the selected
joint's URDF velocity limit. Teleop uses steady time for the input and feedback
watchdogs. The launch connects its private output to the simulation controller.
See the [Phase 4 joystick guide](phase4_sim_joystick.md).

Phase 5 introduces a host-only STM32 communication mock in `robot_arm_control`.
Its in-memory packet transport and probe state machine build separately from
the Fortress controller plugin. See the [mock communication design](stm32_mock.md).
