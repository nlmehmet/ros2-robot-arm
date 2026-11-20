# Reference six-joint model

This is a **teaching reference**, not a measured robot or a safe hardware design.
The Xacro model in `src/robot_arm_description/urdf/reference_arm.urdf.xacro`
contains seven cylindrical rigid links and six revolute joints.

| Link | Length (m) | Radius (m) | Mass (kg) |
| --- | ---: | ---: | ---: |
| base_link | 0.10 | 0.090 | 2.0 |
| link_1 | 0.15 | 0.060 | 1.4 |
| link_2 | 0.30 | 0.050 | 1.2 |
| link_3 | 0.25 | 0.045 | 0.9 |
| link_4 | 0.15 | 0.040 | 0.6 |
| link_5 | 0.12 | 0.035 | 0.4 |
| link_6 | 0.10 | 0.030 | 0.3 |

- Kinematic chain: `base_link → link_1 → … → link_6`.
- Joint axes: Z / Y / Y / X / Y / Z (yaw, pitch, pitch, roll, pitch, roll).
- Cylinder link frames start at the near end of +Z; their centres of mass are
  halfway along the length. Inertias use the uniform-cylinder formulas
  `Ixx=Iyy=m(3r²+L²)/12`, `Izz=mr²/2`.
- Angles are radians; dimensions metres; masses kilograms. The example limits,
  masses, dimensions, and effort values have **not** been verified for any motors.
- Phase 1B runs kinematic inspection only. No physical actuator may be enabled from this model.

## Verify on Ubuntu 22.04 with ROS 2 Humble

From the repository root:

```bash
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s tests -v
ros2 run xacro xacro src/robot_arm_description/urdf/reference_arm.urdf.xacro > /tmp/reference_arm.urdf
check_urdf /tmp/reference_arm.urdf  # if installed
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
colcon test --packages-select robot_arm_bringup --event-handlers console_direct+
colcon test-result --verbose
```

With `joint_state_publisher` installed, launch the default reference pose with:

```bash
ros2 launch robot_arm_bringup reference_arm.launch.py
```

For external joint input, pass `publish_joint_states:=false`. This mode can run
without `joint_state_publisher`; publish all six positions on `/joint_states` to
populate the moving transforms. Add `rviz:=true` on a graphical desktop with
`rviz2` installed. See the [launch guide](../src/robot_arm_bringup/launch/README.md).

The root structural suite checks Xacro expansion when Xacro is installed. The
colcon test starts the installed headless launch on a localhost-only ROS domain
and checks all six parent-child transforms and the base-to-tip chain. It exercises
zero and signed nonzero angles with external input. Both that case and the
default zero-pose case passed on Ubuntu 22.04 with Humble, including a clean
launch exit after SIGINT. Interactive RViz rendering remains a manual check
for the full Phase 1 gate.

## Before continuing to motion

Expand and check the model on the target VM; inspect all six joints and TF in
RViz. Review actual geometry, link inertias, axes, motor and gearbox limits,
payload, stopping behavior, and collision clearance before treating the model
as a real robot specification. Gazebo and hardware remain separate next phases.
