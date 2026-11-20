"""Exercise the opt-in joystick adapter with synthetic Joy and Fortress feedback."""

import os
import secrets
import signal
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
import rclpy
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from phase2a_smoke import require, terminate
from phase2b_trajectory import run_simulation, trajectory_session, wait_for_state, wait_future


JOINT = 'joint_6'
BOUND = 3.12  # The 3.14 rad URDF stop minus the controller's 0.02 rad clearance.


def validate_joystick():
    with trajectory_session() as (node, client, read_state):
        joy = node.create_publisher(Joy, '/joy', 1)
        received = []
        node.create_subscription(JointTrajectory, '/arm_trajectory_controller/joint_trajectory',
                                 received.append, 10)
        wait_for_state(node, lambda: joy.get_subscription_count() == 1)

        def position():
            state = read_state()
            return state.position[state.name.index(JOINT)]

        def observe(seconds, message=None):
            deadline = time.monotonic() + seconds
            next_send = 0.0
            while time.monotonic() < deadline:
                now = time.monotonic()
                if message is not None and now >= next_send:
                    joy.publish(message)
                    next_send = now + 0.05
                rclpy.spin_once(node, timeout_sec=0.01)

        released = Joy(axes=[0.0, 1.0], buttons=[1, 0])
        forward = Joy(axes=[0.0, 1.0], buttons=[0, 1])
        reverse = Joy(axes=[0.0, -1.0], buttons=[0, 1])

        def require_quiet(label, message=None):
            count = len(received)
            before = position()
            observe(0.35, message)
            require(len(received) == count, f'{label}: unexpected joystick command')
            require(abs(position() - before) < 0.015, f'{label}: simulated joint moved')
            print(f'PASS {label}', flush=True)

        require_quiet('held deadman at startup does not arm', forward)
        require_quiet('released deadman inhibits an active configured axis', released)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [JOINT]
        point = JointTrajectoryPoint(positions=[3.05])
        point.time_from_start.sec = 7
        goal.trajectory.points = [point]
        handle = wait_future(node, client.send_goal_async(goal), 5)
        require(handle.accepted, 'Near-bound setup goal rejected')
        result = wait_future(node, handle.get_result_async(), 12)
        require(result.status == GoalStatus.STATUS_SUCCEEDED, f'Near-bound setup goal failed: {result}')
        wait_for_state(node, lambda: abs(position() - 3.05) <= 0.02)

        observe(0.20, released)
        start = position()
        observe(0.90, forward)
        require(received and any(abs(msg.points[0].positions[0] - BOUND) < 1e-6 for msg in received),
                'Full-stick command did not clamp at the controller bound')
        require(position() > start + 0.012,
                f'Bounded joystick commands did not move the simulated joint: {start} -> {position()}')
        for message in received:
            require(message.joint_names == [JOINT] and len(message.points) == 1,
                    f'Unexpected joystick waypoint: {message}')
            point = message.points[0]
            require(-BOUND <= point.positions[0] <= BOUND and not point.velocities
                    and not point.accelerations and not point.effort
                    and point.time_from_start.sec == 0 and point.time_from_start.nanosec == 200_000_000,
                    f'Unsafe joystick waypoint: {point}')
        print('PASS full stick clamps at the shared bound and moves the intended joint', flush=True)

        observe(0.30, released)

        count = len(received)
        start = position()
        observe(0.50, reverse)
        require(len(received) > count and position() < start - 0.025, 'Reverse input did not move the joint')
        first_target = received[count].points[0].positions[0]
        require(abs((start - first_target) - 0.10) < 0.015,
                f'Configured rate did not produce a 0.10 rad waypoint offset: {start} -> {first_target}')
        print('PASS reverse input and configured command rate', flush=True)

        observe(0.30, released)
        require_quiet('deadman release stops command publication', released)
        count = len(received)
        observe(0.30, reverse)
        require(len(received) > count, 'No active input before disconnect test')
        # The last command may finish its bounded horizon after publication stops.
        observe(0.34)
        count = len(received)
        observe(0.31)
        require(len(received) == count, 'Command published after the input timeout')
        require_quiet('Joy disconnect expires input and stops commanding')
        require_quiet('held deadman after reconnect does not rearm', reverse)

        observe(0.15, released)
        count = len(received)
        observe(0.30, reverse)
        require(len(received) > count, 'Deadman release and press did not rearm')
        malformed = Joy(axes=[0.0], buttons=[0, 1])
        observe(0.30, malformed)
        require_quiet('malformed input disarms and requires a new release', reverse)


def test_sim_joystick(capsys):
    domain = os.environ.get('ROBOT_ARM_JOYSTICK_DOMAIN')
    previous_handler = signal.signal(signal.SIGTERM, terminate)
    try:
        with capsys.disabled():
            run_simulation(int(domain) if domain is not None else 20 + secrets.randbelow(70),
                           validate_joystick, 'phase4-joystick',
                           ('teleop:=true', f'joy_joint:={JOINT}', 'joy_axis:=1',
                            'joy_deadman_button:=1', 'joy_scale:=0.5', 'joy_timeout:=0.20'))
    finally:
        signal.signal(signal.SIGTERM, previous_handler)
