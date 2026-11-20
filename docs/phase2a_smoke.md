# Phase 2A headless Gazebo smoke test

Use Ubuntu 22.04 with ROS 2 Humble, Gazebo Fortress (Gazebo Sim 6), and
`ros_gz_sim` and `ros_gz_bridge` installed. Build the five packages as
described in the [Ubuntu setup guide](ubuntu_setup.md).

From the checkout root, select an unused ROS domain in the Linux UDP-safe
range 0–101 and run the standalone system test.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
set -o pipefail
python3 src/robot_arm_bringup/test/phase2a_smoke.py --domain 97 |& tee /tmp/phase2a-smoke.log
```

The test starts the installed world paused and reads cube poses at the initial
state and after confirming both step intervals against Gazebo world statistics.
Once gravity and contact pass,
it converts the installed reference Xacro to SDF, spawns one arm, and checks
the live joint inventory. The reference arm has a free base and may move
under gravity. The bridge publishes the Fortress clock on ROS `/clock`.

The runner uses `ROS_LOCALHOST_ONLY=1`, sets `IGN_IP=127.0.0.1`, and
shares a fresh Ignition partition across processes in each run. It logs active
socket addresses. Ignition discovery can use wildcard UDP sockets, and a
partition is a discovery scope, so use a dedicated network namespace if
network isolation is required.

The runner sends SIGINT to its bridge and server process groups, including
when the runner receives SIGTERM. On a full run it checks that
their endpoints disappear, and repeats the test from a fresh world. A forced
kill fails the test. The transcript prints a temporary evidence directory
containing each run's logs and converted model. This test runs separately from
the Phase 1B colcon launch suite so the caller can choose an unused ROS domain.
