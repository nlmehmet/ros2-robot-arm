# Reference arm launch

After building and sourcing the Humble workspace, run:

```bash
ros2 launch robot_arm_bringup reference_arm.launch.py
```

The default headless launch expands the installed Xacro, starts
`robot_state_publisher`, and uses `joint_state_publisher` to publish the six
reference positions at zero radians. `base_link` is the TF root.

Use `publish_joint_states:=false` to supply your own stamped
`sensor_msgs/msg/JointState` messages on `/joint_states`. Include `joint_1`
through `joint_6` and their positions in radians. TF follows these reported
positions. Use one joint-state source at a time.

On a graphical desktop with `rviz2` installed, add `rviz:=true` to load the
included robot model and TF displays.
