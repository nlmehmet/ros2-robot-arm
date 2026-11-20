#!/usr/bin/env python3
"""Validate single-joint motion before sending a six-joint simulation goal."""

import argparse
from contextlib import contextmanager
import math
import os
from pathlib import Path
import signal
import tempfile
import time
import uuid

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from phase2a_smoke import JOINTS, background, command, require, terminate


CONTROLLERS = {"joint_state_broadcaster", "arm_trajectory_controller"}


def wait_future(node, future, timeout):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    require(future.done(), "ROS request timed out")
    return future.result()


def wait_for_state(node, predicate):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return
    raise RuntimeError("Timed out waiting for a fresh six-joint state within tolerance")


@contextmanager
def trajectory_session():
    rclpy.init()
    node = rclpy.create_node("phase2b_trajectory_verifier")
    client = None
    try:
        latest = None

        def on_state(message):
            nonlocal latest
            if len(message.name) == len(message.position) == len(JOINTS) and sorted(message.name) == JOINTS:
                latest = message

        node.create_subscription(JointState, "/joint_states", on_state, qos_profile_sensor_data)
        manager = node.create_client(ListControllers, "/controller_manager/list_controllers")
        require(manager.wait_for_service(timeout_sec=20), "Gazebo controller manager did not start")
        deadline = time.monotonic() + 20
        while True:
            response = wait_future(node, manager.call_async(ListControllers.Request()), 3)
            states = {controller.name: controller.state for controller in response.controller}
            if states == dict.fromkeys(CONTROLLERS, "active"):
                break
            require(time.monotonic() < deadline, f"Controllers failed to activate: {states}")
            time.sleep(0.2)
        print(f"PASS Gazebo controller activation: {states}", flush=True)

        client = ActionClient(node, FollowJointTrajectory, "/arm_trajectory_controller/follow_joint_trajectory")
        require(client.wait_for_server(timeout_sec=5), "Trajectory action unavailable")
        wait_for_state(node, lambda: latest is not None)
        yield node, client, lambda: latest
    finally:
        if client is not None:
            client.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


def validate_trajectory():
    with trajectory_session() as (node, client, read_state):

        def stamp():
            latest = read_state()
            return latest.header.stamp.sec * 10**9 + latest.header.stamp.nanosec

        def positions():
            latest = read_state()
            return dict(zip(latest.name, latest.position))

        def verify_goal(targets, duration=2):
            before = positions()
            require(all(math.isfinite(position) for position in before.values()), "Invalid starting state")
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(targets)
            point = JointTrajectoryPoint(positions=list(targets.values()))
            point.time_from_start.sec = duration
            goal.trajectory.points = [point]

            handle = wait_future(node, client.send_goal_async(goal), 5)
            require(handle.accepted, f"Trajectory rejected: {targets}")
            result = wait_future(node, handle.get_result_async(), duration + 5)
            require(result.status == GoalStatus.STATUS_SUCCEEDED
                    and result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL,
                    f"Trajectory failed: {result.status}, {result.result}")
            after_result = stamp()
            expected = before | targets

            def matches():
                measured = positions()
                return stamp() > after_result and all(
                    abs(measured[joint] - target) <= 0.02 for joint, target in expected.items()
                )

            wait_for_state(node, matches)
            measured = positions()
            require(all(abs(measured[joint] - before[joint]) >= 0.045 for joint in targets),
                    f"Commanded joints did not move far enough: {before} -> {measured}")
            print(f"PASS {len(targets)}-joint trajectory: targets={targets}, measured={measured}", flush=True)

        verify_goal({"joint_1": 0.20})
        verify_goal(dict(zip(JOINTS, [-0.10, 0.10, -0.10, 0.10, -0.10, 0.10])))

        def reject_unsafe_commands():
            def reject(names, points, label):
                goal = FollowJointTrajectory.Goal()
                goal.trajectory.joint_names = names
                goal.trajectory.points = points
                handle = wait_future(node, client.send_goal_async(goal), 5)
                require(not handle.accepted, f"Invalid goal accepted: {label}")

            def waypoint(positions, seconds):
                point = JointTrajectoryPoint(positions=positions)
                point.time_from_start.sec = seconds
                return point

            names = ["joint_6"]
            for position in (3.24, -3.24, 3.14, -3.14, math.nan):
                reject(names, [waypoint([position], 7)], str(position))
            reject(names, [waypoint([0.1], 1), waypoint([3.24], 8)], "later unsafe waypoint")
            reject(["unknown_joint"], [waypoint([0.1], 1)], "unknown joint")
            reject(["joint_6", "joint_6"], [waypoint([0.1, 0.2], 1)], "duplicate joint")
            reject(names, [waypoint([0.1, 0.2], 1)], "position count mismatch")
            reject(names, [], "empty trajectory")
            reject(names, [waypoint([0.1], 2), waypoint([0.2], 1)], "decreasing times")
            derivative = waypoint([0.1], 1)
            derivative.velocities = [0.0]
            reject(names, [derivative], "derivative waypoint")
            print("PASS action rejects malformed, out-of-range, hard-stop, and nonfinite goals", flush=True)

            publisher = node.create_publisher(
                JointTrajectory, "/arm_trajectory_controller/joint_trajectory", 10,
            )
            wait_for_state(node, lambda: publisher.get_subscription_count() > 0)
            before = positions()
            start = stamp()
            # A short unsafe command would cause detectable motion during the observation window.
            trajectory = JointTrajectory(joint_names=names, points=[waypoint([3.24], 1)])
            publisher.publish(trajectory)

            def held_position():
                measured = positions()
                require(all(abs(measured[joint] - before[joint]) <= 0.02 for joint in JOINTS),
                        "Unsafe topic command moved the arm")
                return stamp() - start >= 500_000_000

            wait_for_state(node, held_position)
            print("PASS out-of-range topic command leaves all six joints in place", flush=True)

            target = before["joint_6"] + 0.20
            sent_at = stamp()
            publisher.publish(JointTrajectory(joint_names=names, points=[waypoint([target], 2)]))
            expected = before | {"joint_6": target}

            def reached_topic_target():
                measured = positions()
                return stamp() > sent_at and all(
                    abs(measured[joint] - expected[joint]) <= 0.02 for joint in JOINTS
                )

            wait_for_state(node, reached_topic_target)
            print(f"PASS valid topic command moves joint_6 to {target:.3f} rad", flush=True)

        reject_unsafe_commands()
        for position in (3.10, -3.10):
            verify_goal({"joint_6": position}, duration=7)
            verify_goal({"joint_6": 0.0}, duration=7)


def run_simulation(domain, validate, label, launch_arguments=()):
    require(0 <= domain <= 101, "Choose an unused ROS domain in 0–101")
    os.environ.update(
        ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1", IGN_IP="127.0.0.1",
        IGN_PARTITION=f"{label}-{uuid.uuid4().hex}",
    )
    for key in ("DISPLAY", "WAYLAND_DISPLAY", "IGN_RELAY"):
        os.environ.pop(key, None)
    topics, _ = command("ros2", "topic", "list", "--no-daemon", "--spin-time", "1")
    require(set(topics.splitlines()) <= {"/parameter_events", "/rosout"}, "ROS domain has other topics")
    nodes, _ = command("ros2", "node", "list", "--no-daemon", "--spin-time", "1")
    require(not nodes.strip(), "ROS domain has other nodes")

    evidence = Path(tempfile.mkdtemp(prefix=f"ros-{label}-"))
    print(f"Evidence: {evidence}; IGN_PARTITION={os.environ['IGN_PARTITION']}; domain={domain}",
          flush=True)
    with background(evidence / "launch.log", "ros2", "launch", "robot_arm_bringup",
                    "reference_arm_sim.launch.py", *launch_arguments):
        validate()

    # Allow discovery to retire the verifier's participant after rclpy shutdown.
    deadline = time.monotonic() + 8
    while True:
        nodes, _ = command("ros2", "node", "list", "--no-daemon", "--spin-time", "1")
        if not nodes.strip():
            break
        require(time.monotonic() < deadline, f"ROS nodes remain: {nodes}")
    services, _ = command("ign", "service", "-l")
    topics, _ = command("ign", "topic", "-l")
    require(not services.strip() and not topics.strip(), "Ignition endpoints remain")
    print("PASS simulation cleanup", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, required=True, help="Unused ROS domain (0–101)")
    args = parser.parse_args()
    run_simulation(args.domain, validate_trajectory, "phase2b")


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, terminate)
    main()
