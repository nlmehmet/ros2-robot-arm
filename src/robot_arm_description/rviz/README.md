# Reference arm visualization

`reference_arm.rviz` uses `base_link` as the fixed frame and subscribes to
`/robot_description` with transient local durability. Start it with
`ros2 launch robot_arm_bringup reference_arm.launch.py rviz:=true` on a graphical
desktop with RViz installed.
