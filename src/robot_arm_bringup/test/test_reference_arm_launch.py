"""Exercise the installed launch and the six reference joint transforms."""
import math
import os
import secrets
import signal
import subprocess
import time

from ament_index_python.packages import PackageNotFoundError, get_package_prefix
import pytest
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


JOINT_NAMES = [f"joint_{index}" for index in range(1, 7)]
FRAMES = ["base_link", *(f"link_{index}" for index in range(1, 7))]
AXES = [(0, 0, 1), (0, 1, 0), (0, 1, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)]
OFFSETS = [0.10, 0.15, 0.30, 0.25, 0.15, 0.12]
ZERO = [0.0] * 6
MOVED = [0.25, -0.35, 0.45, -0.55, 0.65, -0.75]


def quaternion_matches(rotation, axis, position):
    expected = [component * math.sin(position / 2) for component in axis]
    expected.append(math.cos(position / 2))
    actual = [rotation.x, rotation.y, rotation.z, rotation.w]
    # Both quaternion signs represent the same rotation.
    sign = 1 if rotation.w >= 0 else -1
    return actual == pytest.approx([sign * value for value in expected], abs=1e-6)


def wait_for_pose(process, node, buffer, publisher, positions):
    deadline = time.monotonic() + 8
    detail = "waiting for transforms"
    while time.monotonic() < deadline:
        assert process.poll() is None, f"Launch exited with status {process.returncode}"
        if publisher is not None:
            state = JointState()
            state.header.stamp = node.get_clock().now().to_msg()
            # Publish in reverse chain order to exercise name-based mapping.
            state.name = JOINT_NAMES[::-1]
            state.position = positions[::-1]
            publisher.publish(state)
        rclpy.spin_once(node, timeout_sec=0.1)

        unmatched = []
        try:
            for parent, child, axis, offset, position in zip(
                FRAMES[:-1], FRAMES[1:], AXES, OFFSETS, positions
            ):
                transform = buffer.lookup_transform(parent, child, Time()).transform
                translation = transform.translation
                assert [translation.x, translation.y, translation.z] == pytest.approx(
                    [0.0, 0.0, offset], abs=1e-6
                ), f"{parent} -> {child}"
                if not quaternion_matches(transform.rotation, axis, position):
                    unmatched.append(f"{parent} -> {child}")
            tip = buffer.lookup_transform(FRAMES[0], FRAMES[-1], Time())
        except TransformException as error:
            detail = str(error)
            continue

        if not unmatched:
            if positions == ZERO:
                translation = tip.transform.translation
                assert [translation.x, translation.y, translation.z] == pytest.approx(
                    [0.0, 0.0, sum(OFFSETS)], abs=1e-6
                )
                assert quaternion_matches(tip.transform.rotation, (0, 0, 1), 0)
            return
        detail = f"unexpected rotations: {', '.join(unmatched)}"
    pytest.fail(f"Timed out waiting for {positions}: {detail}")


@pytest.mark.parametrize("use_default_input", [False, True], ids=["external", "default"])
def test_reference_arm_tf(use_default_input, monkeypatch, tmp_path):
    if use_default_input:
        try:
            get_package_prefix("joint_state_publisher")
        except PackageNotFoundError:
            pytest.skip("joint_state_publisher is unavailable")

    monkeypatch.setenv("ROS_DOMAIN_ID", str(100 + secrets.randbelow(100)))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    command = ["ros2", "launch", "robot_arm_bringup", "reference_arm.launch.py"]
    if not use_default_input:
        command.append("publish_joint_states:=false")
    with subprocess.Popen(command, start_new_session=True) as process:
        try:
            rclpy.init()
            try:
                node = rclpy.create_node("reference_arm_tf_test")
                try:
                    buffer = Buffer()
                    listener = TransformListener(buffer, node)
                    publisher = None
                    if not use_default_input:
                        publisher = node.create_publisher(JointState, "/joint_states", 10)
                    wait_for_pose(process, node, buffer, publisher, ZERO)
                    if publisher is not None:
                        wait_for_pose(process, node, buffer, publisher, MOVED)
                    expected_nodes = {node.get_name(), "robot_state_publisher"}
                    if use_default_input:
                        expected_nodes.add("joint_state_publisher")
                    # TF data can arrive before discovery of its upstream node.
                    deadline = time.monotonic() + 5
                    discovered = set(node.get_node_names())
                    while discovered < expected_nodes and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.1)
                        discovered = set(node.get_node_names())
                    assert discovered == expected_nodes
                    listener.unregister()
                finally:
                    node.destroy_node()
            finally:
                rclpy.shutdown()
        finally:
            try:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2)
                pytest.fail("Reference arm launch did not shut down after SIGINT")
    assert process.returncode == 0
