#!/usr/bin/env python3
"""Exercise motion, action faults, and restart in headless Fortress."""

import argparse
import math
import os
import secrets
import signal
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
import rclpy
from trajectory_msgs.msg import JointTrajectoryPoint

from phase2a_smoke import ARM, JOINTS, model_names, require, terminate
from phase2b_trajectory import run_simulation, trajectory_session, wait_for_state, wait_future


TOLERANCE_RAD = 0.02


def goal_for(targets, seconds, nanoseconds=0):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(targets)
    goal.trajectory.points = [JointTrajectoryPoint(
        positions=list(targets.values()),
        time_from_start=Duration(sec=seconds, nanosec=nanoseconds),
    )]
    return goal


def verify_startup(node, sample):
    inventory = sorted(model_names())
    require(inventory == ["drop_box", "ground", ARM], f"Unexpected model inventory: {inventory}")
    initial_stamp, _ = sample()

    def at_zero():
        stamp, positions = sample()
        return stamp > initial_stamp and all(abs(value) <= TOLERANCE_RAD for value in positions.values())

    wait_for_state(node, at_zero)
    print("PASS one reference arm with fresh six-joint zero state", flush=True)


def require_hold(node, sample, label, anchor=None):
    start, _ = sample()
    wait_for_state(node, lambda: sample()[0] > start)
    start, positions = sample()
    if anchor is None:
        anchor = positions

    def held():
        stamp, positions = sample()
        require(all(abs(positions[joint] - anchor[joint]) <= TOLERANCE_RAD for joint in JOINTS),
                f"{label}: joints moved away from the hold pose: {anchor} -> {positions}")
        return stamp - start >= 500_000_000

    wait_for_state(node, held)
    print(f"PASS {label}: held all six joints for 0.5 simulated seconds", flush=True)


def verify_faults(node, client, sample):
    _, before = sample()
    invalid = goal_for({"joint_6": 3.24}, 1)
    handle = wait_future(node, client.send_goal_async(invalid), 5)
    require(not handle.accepted, "Out-of-range goal was accepted")
    require_hold(node, sample, "out-of-range action rejected", anchor=before)

    # A valid waypoint with a short deadline and tight tolerance cannot settle in time.
    tight = goal_for({"joint_6": 0.65}, 0, 100_000_000)
    tight.goal_tolerance = [JointTolerance(name="joint_6", position=1e-8)]
    tight.goal_time_tolerance = Duration(nanosec=100_000_000)
    handle = wait_future(node, client.send_goal_async(tight), 5)
    require(handle.accepted, "Tolerance-timeout goal was rejected")
    result = wait_future(node, handle.get_result_async(), 5)
    require(result.status == GoalStatus.STATUS_ABORTED and
            result.result.error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
            f"Expected goal tolerance timeout, got {result}")
    print("PASS controller aborted an accepted goal after its tolerance deadline", flush=True)

    # Confirm the slow goal is moving before exercising the client's wall-clock timeout.
    _, before = sample()
    slow = goal_for({"joint_6": -0.85}, 10)
    handle = wait_future(node, client.send_goal_async(slow), 5)
    require(handle.accepted, "Cancellation exercise goal was rejected")
    result_future = handle.get_result_async()
    wait_for_state(node, lambda: before["joint_6"] - sample()[1]["joint_6"] >= 0.03)
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=0.35)
    require(not result_future.done(), "Slow goal finished before the client deadline")
    response = wait_future(node, handle.cancel_goal_async(), 5)
    require(any(goal.goal_id == handle.goal_id for goal in response.goals_canceling),
            "Server did not acknowledge cancellation of the slow goal")
    result = wait_future(node, result_future, 5)
    require(result.status == GoalStatus.STATUS_CANCELED, f"Expected canceled goal, got {result}")
    require_hold(node, sample, "client deadline expired and active goal canceled")


def verify_motion(node, client, sample, targets):
    _, before = sample()
    handle = wait_future(node, client.send_goal_async(goal_for(targets, 2)), 5)
    require(handle.accepted, f"Nominal goal rejected: {targets}")
    result = wait_future(node, handle.get_result_async(), 8)
    require(result.status == GoalStatus.STATUS_SUCCEEDED and
            result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL,
            f"Nominal goal failed: {result}")
    result_stamp, _ = sample()
    expected = before | targets

    def reached():
        stamp, positions = sample()
        return stamp > result_stamp and all(
            abs(positions[joint] - expected[joint]) <= TOLERANCE_RAD for joint in JOINTS
        )

    wait_for_state(node, reached)
    _, measured = sample()
    require(all(abs(measured[joint] - before[joint]) > TOLERANCE_RAD for joint in targets),
            f"A commanded joint did not move: {before} -> {measured}")
    errors = {joint: round(abs(measured[joint] - expected[joint]), 6) for joint in JOINTS}
    print(f"PASS nominal goal: targets={targets}, joint errors (rad)={errors}", flush=True)


def validate_motion(exercise_faults):
    with trajectory_session() as (node, client, read_state):
        def sample():
            state = read_state()
            stamp = state.header.stamp.sec * 10**9 + state.header.stamp.nanosec
            positions = dict(zip(state.name, state.position))
            require(all(math.isfinite(value) for value in positions.values()), "Nonfinite joint state")
            return stamp, positions

        verify_startup(node, sample)
        targets = dict(zip(JOINTS, [0.15, -0.12, 0.10, -0.15, 0.12, -0.10]))
        verify_motion(node, client, sample, targets)
        if exercise_faults:
            verify_faults(node, client, sample)
            verify_motion(node, client, sample, {joint: -target for joint, target in targets.items()})


def run_checkpoint(domain):
    run_simulation(domain, lambda: validate_motion(exercise_faults=True), "phase3-faults")
    run_simulation(domain, lambda: validate_motion(exercise_faults=False), "phase3-restart")
    print("PASS fresh simulation restart and motion", flush=True)


def test_phase3_motion(capsys):
    domain = os.environ.get("ROBOT_ARM_PHASE3_DOMAIN")
    previous_handler = signal.signal(signal.SIGTERM, terminate)
    try:
        # Keep measured errors and controller states in successful colcon logs.
        with capsys.disabled():
            run_checkpoint(int(domain) if domain is not None else 20 + secrets.randbelow(70))
    finally:
        signal.signal(signal.SIGTERM, previous_handler)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, required=True, help="Unused ROS domain (0–101)")
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, terminate)
    run_checkpoint(args.domain)
