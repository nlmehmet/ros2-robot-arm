# Phase 3 motion and fault checks

Use Ubuntu 22.04, ROS 2 Humble, and Gazebo Fortress with the dependencies
listed in the [Phase 2B guide](phase2b_control.md). Run from the checkout root.

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
colcon test --packages-select robot_arm_bringup --event-handlers console_direct+
colcon test-result --verbose
```

The colcon test picks a ROS domain from 20–89 and checks that it is empty.
Set `ROBOT_ARM_PHASE3_DOMAIN` to a known unused domain if needed. The runner
uses localhost ROS discovery and a fresh Ignition partition for each launch.
For an isolated run with a specific unused domain, use:

```bash
python3 src/robot_arm_bringup/test/test_phase3_motion.py --domain 92
```

The test runs the installed headless Fortress launch and checks:

- Exactly one reference arm spawns, both controllers activate, and fresh joint
  states describe all six joints at the initial pose.
- A six-joint action succeeds and each measured joint reaches its target within
  0.02 rad. The runner prints controller states and per-joint errors.
- An out-of-range action is rejected and all six joints hold their prior
  positions within 0.02 rad for 0.5 simulated seconds.
- An accepted in-range goal with tight position and time tolerances terminates
  as `ABORTED` with `GOAL_TOLERANCE_VIOLATED`.
- A separate slow goal starts moving. The test client waits 0.35 wall-clock
  seconds for a result, then cancels the still-active goal. The action reports
  `CANCELED` and all six joints hold within 0.02 rad for 0.5 simulated seconds.
- A fresh six-joint command succeeds after the faults. Both runs shut down
  cleanly and clear ROS nodes and Ignition endpoints. The second run confirms the
  initial pose and repeats nominal motion in a fresh world.

The timeout and cancellation logic belongs to the test client. The test prints
each launch log directory. Hardware and GUI checks are outside its scope.
