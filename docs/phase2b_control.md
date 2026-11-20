# Phase 2B headless simulation control

Use the [Ubuntu setup guide](ubuntu_setup.md) and the
[Phase 2A smoke test](phase2a_smoke.md) to verify the Humble and Fortress
environment. This launch uses the Humble Fortress controller integration,
controller manager, joint state broadcaster and joint trajectory controller.
The standalone test also uses the `control_msgs` action interface.

Build the workspace, then run the test from the checkout root. Select an unused
ROS domain. The runner sets ROS and Ignition loopback configuration and a
unique Ignition partition.

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
set -o pipefail
python3 src/robot_arm_bringup/test/phase2b_trajectory.py --domain 93 |& tee /tmp/phase2b-trajectory.log
```

The runner launches `robot_arm_bringup reference_arm_sim.launch.py`. The
simulation Xacro fixes the base to the world and exposes the six position
command interfaces and their position and velocity states. Fortress loads
`libign_ros2_control-system.so` and the `ign_ros2_control/IgnitionSystem`
hardware class through the compatibility names shipped by the Humble
`gz_ros2_control` package. The plugin creates the sole controller manager.
The ROS clock bridge and robot state publisher provide simulation time and TF.

The trajectory test waits for both controllers to be active and checks that
the joint state broadcaster reports all six joints. A partial goal moves
`joint_1` to 0.20 rad. The test requires action success, a fresh measured
position within 0.02 rad, and the other five joints within 0.02 rad of their
starting positions. Only then does it send the six-joint goal and check all
six measured positions. The runner stops the launch process group and checks
for remaining endpoints. Its transcript includes the launch log location.

## Simulation command bounds

The simulation loads a guard around Humble's joint trajectory controller.
It reads the waypoint bounds generated from the expanded simulation URDF,
with 0.02 rad of clearance inside each physical stop. Both the
`FollowJointTrajectory` action and the controller's trajectory topic use
the guard. The guard accepts position-only waypoints because derivatives
can produce spline extrema between waypoints. The original controller still
handles interpolation, execution, feedback, cancellation, and result reporting.

The unguarded controller reproduced the Fortress/DART hard-stop lockup.
During recovery, `joint_6` stayed at +3.14 rad while controller feedback
showed the desired position reaching zero. The position interface drives
Gazebo joint velocity commands, and this symptom matches
[Gazebo's documented DART issue](https://github.com/gazebosim/gz-sim/issues/1684).
The simulation retains its URDF hard stops and existing physics engine.
The guard prevents commands into the stops through these two ROS interfaces.
A return-to-zero goal failed after the joint latched. Starting a fresh
simulation world restored nominal motion.

The regression checks malformed action goals, including duplicate joint names,
and out-of-range waypoints. It verifies that an out-of-range topic command
leaves all six joints in place and that a valid topic command still moves
the requested joint. Valid goals at +3.10 and −3.10 rad must each return to
zero within 0.02 rad. Velocity and effort enforcement and external disturbances
remain outside these checks.

See the [T03 engineering report](phase2b_t03_report.md) for measured results
and the controller reactivation probe.
