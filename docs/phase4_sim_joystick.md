# Phase 4 simulation joystick

The adapter controls one joint at a time in the reference-arm Fortress
simulation. After building and sourcing the workspace, start it with:

```bash
ros2 launch robot_arm_bringup reference_arm_sim.launch.py teleop:=true
```

Supply `/joy` at 10 Hz or faster. If a joystick and the optional `joy` package
are available, run the driver in another sourced shell on the same ROS domain:

```bash
ros2 run joy joy_node --ros-args -p autorepeat_rate:=20.0
```

| Launch argument | Default | Meaning |
| --- | --- | --- |
| `joy_joint` | `joint_1` | Joint moved by the axis |
| `joy_axis` | `0` | Zero-based index in `Joy.axes` |
| `joy_deadman_button` | `0` | Zero-based index in `Joy.buttons` |
| `joy_scale` | `0.4` | Full-stick command rate in rad/s, capped by the selected joint's URDF velocity limit |
| `joy_timeout` | `0.25` | Maximum Joy age in wall-clock seconds, greater than 0.05 and at most 1.0 |

With current joint feedback and an advancing simulation clock, release the
deadman before pressing it to enable motion. Each command is a position-only
waypoint 0.20 simulated seconds ahead. The adapter limits the offset from
measured joint position by stick magnitude and configured rate, then clamps to
the same interior position limits as the trajectory guard. A stick magnitude
of 0.10 or less produces no waypoint.

Deadman release, invalid input, or expired Joy or simulation feedback stops
publication. After timeout or invalid input, release and press again to rearm.
The controller can finish the last bounded waypoint already accepted. Let the
adapter finish its current command before sending commands from another client.

## Automated validation

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
colcon test --packages-select robot_arm_bringup --ctest-args -R sim_joystick
colcon test-result --verbose
```

The test uses synthetic `Joy` messages and measured Fortress joint states. It
checks startup with the button held, a nondefault mapping, position clamping,
motion in both directions, release, disconnect, rearming, malformed input,
and shutdown cleanup. Set `ROBOT_ARM_JOYSTICK_DOMAIN` to an unused ROS domain
if needed.
