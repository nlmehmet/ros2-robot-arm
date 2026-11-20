# Reference six-joint model — Phase 1A

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
- No ROS controller, Gazebo plugin, launch file, STM32 interface, or motor driver
  is included. No physical actuator may be enabled from this model.

## Verify on Ubuntu 22.04 with ROS 2 Humble

From the repository root:

```bash
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s tests -v
ros2 run xacro xacro src/robot_arm_description/urdf/reference_arm.urdf.xacro > /tmp/reference_arm.urdf
# Optional if installed: check_urdf /tmp/reference_arm.urdf
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
ros2 pkg prefix robot_arm_description
```

The pure-Python structural test can run anywhere. The expanded-URDF test is
skipped when `xacro` is unavailable. Neither a static test nor colcon build
validates RViz visualization, moving TF frames, Gazebo physics, or hardware.

## Before continuing to motion

Expand and check the model on the target VM; inspect all six joints and TF in
RViz. Review actual geometry, link inertias, axes, motor and gearbox limits,
payload, stopping behavior, and collision clearance before treating the model
as a real robot specification. Gazebo and hardware remain separate next phases.
