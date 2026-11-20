# Phase 2B T03 engineering report

## Scope

Validated the guarded trajectory controller in headless ROS 2 Humble and
Gazebo Fortress on `feat/phase2b-gazebo-control`, based on `3c43acb`.
The guarded regression used ROS domain 94. The recovery probe used domain
95 and a temporary launch wrapper that substituted the standard Humble
joint trajectory controller. Each run used loopback transport and a unique
Ignition partition. All commands targeted simulation.

The guard checks position-only waypoints against bounds derived from the
expanded URDF, with 0.02 rad of clearance from each physical stop. T03
extended the common action and topic validation to reject duplicate joint
names. The regression exercises both rejection and valid motion on the
trajectory topic.

## Passed

- The simulation controller manager activated both controllers. The
  single-joint action reached 0.194095 rad for a 0.20 rad target. The
  six-joint action had a maximum measured error of 0.009667 rad.
- The action rejected unknown and duplicate names, dimension mismatch,
  empty trajectories, decreasing waypoint times, derivative fields,
  nonfinite positions, out-of-range positions, and a later unsafe
  waypoint in a multi-point goal.
- An out-of-range topic command left all six joints within 0.02 rad of
  their starting positions in the 0.5 s observation window. A subsequent
  valid topic command moved `joint_6` to within 0.02 rad of 0.297 rad.
- The near-stop actions succeeded, with fresh measured positions after
  each action result.

| Target for `joint_6` (rad) | Measured (rad) |
| ---: | ---: |
| +3.10 | +3.082648 |
| 0 after positive approach | +0.017071 |
| −3.10 | −3.082758 |
| 0 after negative approach | −0.017071 |

- The guarded launch exited without forced termination. ROS nodes and
  Ignition endpoints cleared.
- All 14 structural tests passed. The colcon build completed for all five
  packages. Colcon reported 3 tests, 0 errors, 0 failures, and 0 skips.

## Recovery investigation

The unguarded controller accepted a 7 s goal to +3.24 rad and drove
`joint_6` to its +3.14 rad stop. The action finished with
`STATUS_ABORTED` (6) and `GOAL_TOLERANCE_VIOLATED` (−5). An in-range
goal to zero was accepted and returned the same result. Action feedback
showed the desired position reach zero while measured position stayed at
+3.14 rad and sampled speed stayed at zero to six decimal places.

The controller manager successfully deactivated and reactivated the
trajectory controller. The next in-range goal again aborted with −5.
The joint stayed at +3.14 rad across 840 state samples. Cycling the
controller in the same world did not release the joint. The earlier T02
fresh-world run demonstrated restored nominal motion after starting a new
simulation. The current guard passed both retreats while staying inside
the physical stops. These results establish prevention for the tested
commands and a fresh-world workaround for the reproduced latch.

The first lifecycle probe reached the stop but could not cycle the
controller because the optional `ros2 control` CLI was unavailable. The
repeat used the controller manager's `SwitchController` service directly
and verified each lifecycle state through `ListControllers`.

## Outside scope

Dynamic near-stop tests covered `joint_6`. Velocity and effort
enforcement, forced contact or external disturbances, in-place simulator
resets, and physical hardware were outside the test scope.

## Evidence

- Guarded regression and cleanup — `/tmp/phase2b-t03-guarded.log`
- Isolated hard-stop and reactivation probe — `/tmp/phase2b-t03-recovery.log`
- Earlier fresh-world nominal regression — `/tmp/phase2b-t02-nominal-regression.log`
- Structural, build and colcon results — `/tmp/phase2b-t03-structural.log`,
  `/tmp/phase2b-t03-build.log`, and `/tmp/phase2b-t03-colcon.log`

The transcripts are temporary files. The measurements and verdicts above
are recorded here for future reference.
