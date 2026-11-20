#!/usr/bin/env python3
"""Run the bounded Fortress smoke test and print evidence for both runs."""

import argparse
from contextlib import contextmanager
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import tempfile
import time
import uuid
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


WORLD = "/world/phase2a"
ARM = "reference_arm"
JOINTS = sorted(f"joint_{index}" for index in range(1, 7))


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def command(*args, timeout=8):
    print(f"$ {shlex.join(args)}", flush=True)
    result = subprocess.run(
        args, timeout=timeout, check=False, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
    for output in (result.stdout, result.stderr):
        if output.strip():
            print(output.rstrip(), flush=True)
    result.check_returncode()
    return result.stdout, result.stderr


def group_exists(pid):
    try:
        os.killpg(pid, 0)
    except ProcessLookupError:
        return False
    return True


@contextmanager
def background(log, *args):
    print(f"$ {shlex.join(args)} > {log}", flush=True)
    with log.open("w") as output:
        process = subprocess.Popen(
            args, start_new_session=True, stdout=output, stderr=subprocess.STDOUT,
        )
        try:
            yield process
        finally:
            start = time.monotonic()
            running = process.poll() is None
            if group_exists(process.pid):
                os.killpg(process.pid, signal.SIGINT)
            deadline = start + 8
            while group_exists(process.pid) and time.monotonic() < deadline:
                process.poll()
                time.sleep(0.05)
            forced = group_exists(process.pid)
            if forced:
                os.killpg(process.pid, signal.SIGKILL)
            status = process.wait(timeout=2)
            print(log.read_text(), end="", flush=True)
            print(
                f"Shutdown {log.stem}: pid={process.pid} exit={status} "
                f"elapsed={time.monotonic() - start:.3f}s forced={forced}",
                flush=True,
            )
            require(running and not forced and status in (0, -signal.SIGINT, 130),
                    f"{log.stem} did not shut down cleanly")


def step(count, target_ns):
    output, _ = command(
        "ign", "service", "-s", f"{WORLD}/control", "--reqtype",
        "ignition.msgs.WorldControl", "--reptype", "ignition.msgs.Boolean",
        "--timeout", "3000", "--req", f"pause: true multi_step: {count}",
    )
    require(output.strip() == "data: true", "World step was rejected")
    # The service acknowledges scheduling. Confirm the simulator reached the target.
    deadline = time.monotonic() + 5
    while True:
        stats, _ = command(
            "ign", "topic", "-e", "-t", f"{WORLD}/stats", "-n", "1", "--json-output",
            timeout=3,
        )
        state = json.loads(stats)
        stamp = state.get("simTime", {})
        elapsed_ns = int(stamp.get("sec", 0)) * 10**9 + int(stamp.get("nsec", 0))
        if elapsed_ns == target_ns and state.get("paused"):
            return
        require(elapsed_ns <= target_ns and time.monotonic() < deadline,
                f"Expected paused simulation at {target_ns} ns, got {state}")


def box_height():
    output, _ = command("ign", "model", "-m", "drop_box", "-p")
    match = re.search(r"Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*\[([^]]+)\]", output)
    require(match, "Cube pose missing")
    return float(match[1].split()[2])


def model_names():
    output, _ = command("ign", "model", "--list")
    return re.findall(r"^\s*-\s+(\S+)\s*$", output, re.MULTILINE)


def socket_scope(*processes):
    output = subprocess.check_output(("ss", "-luntp"), text=True)
    for process in processes:
        sockets = [line for line in output.splitlines() if f"pid={process.pid}," in line]
        require(sockets, f"No socket evidence for pid {process.pid}")
        print("\n".join(sockets), flush=True)
        require(all(row.split()[4].startswith("127.") for row in sockets if row.startswith("tcp")),
                "A test process has a TCP listener outside IPv4 loopback")


def check_clock():
    stamps = []
    for _ in range(2):
        output, _ = command(
            "ros2", "topic", "echo", "--once", "--qos-reliability", "best_effort",
            "/clock", "rosgraph_msgs/msg/Clock",
        )
        sec = re.search(r"^\s+sec: (\d+)$", output, re.MULTILINE)
        nsec = re.search(r"^\s+nanosec: (\d+)$", output, re.MULTILINE)
        require(sec and nsec, "ROS clock sample missing")
        stamps.append(int(sec[1]) * 10**9 + int(nsec[1]))
    require(0 < stamps[0] < stamps[1], f"ROS clock did not advance: {stamps}")
    print(f"PASS ROS clock (ns): {stamps}", flush=True)


def run_world(world, xacro, evidence):
    evidence.mkdir()
    os.environ["IGN_PARTITION"] = f"phase2a-{uuid.uuid4().hex}"
    print(f"IGN_PARTITION={os.environ['IGN_PARTITION']}", flush=True)
    with background(evidence / "gazebo.log", "ign", "gazebo", "-s", "-v", "3", str(world)) as server:
        deadline = time.monotonic() + 15
        while True:
            require(server.poll() is None, "Gazebo server exited during startup")
            services, _ = command("ign", "service", "-l", timeout=4)
            if f"{WORLD}/create" in services.splitlines():
                break
            require(time.monotonic() < deadline, "Gazebo create service deadline exceeded")
        print("PASS server-only startup", flush=True)
        require(sorted(model_names()) == ["drop_box", "ground"], "Unexpected initial world inventory")

        initial = box_height()
        step(200, 200_000_000)
        falling = box_height()
        step(1000, 1_200_000_000)
        settled = box_height()
        require(abs(initial - 1.0) < 0.01 and 0.70 < falling < 0.90
                and abs(settled - 0.10) < 0.02, f"Unexpected cube heights: {initial}, {falling}, {settled}")
        print(f"PASS gravity at 0, 0.2, 1.2 simulated seconds: {initial}, {falling}, {settled} m")

        urdf = evidence / "reference_arm.urdf"
        sdf = evidence / "reference_arm.sdf"
        with urdf.open("w") as output:
            subprocess.run(("xacro", str(xacro)), stdout=output, check=True, timeout=8)
        with sdf.open("w") as output:
            subprocess.run(("ign", "sdf", "-p", str(urdf)), stdout=output, check=True, timeout=8)
        converted = ET.parse(sdf).findall("./model/joint")
        require(sorted(joint.attrib["name"] for joint in converted) == JOINTS,
                "Unexpected joint inventory after URDF conversion")

        # Use the installed executable directly so cleanup owns the actual PID.
        sim_prefix, _ = command("ros2", "pkg", "prefix", "ros_gz_sim")
        create = Path(sim_prefix.strip()) / "lib/ros_gz_sim/create"
        stdout, stderr = command(
            str(create), "-world", "phase2a", "-file", str(sdf), "-name", ARM, "-z", "0.5",
        )
        require("OK creation of entity." in stdout + stderr, "Create success response missing")
        require(sorted(model_names()) == ["drop_box", "ground", ARM],
                "Expected exactly one spawned reference arm")
        joints, _ = command("ign", "model", "-m", ARM, "-j")
        names = re.findall(r"Joint\s*\[\d+\]\s*- Name:\s*(\S+)", joints)
        require(sorted(names) == JOINTS, f"Unexpected spawned joint inventory: {names}")
        print(f"PASS create response and joint inventory: {names}", flush=True)

        topics, _ = command("ign", "topic", "-l")
        world_clock = f"{WORLD}/clock"
        clock = world_clock if world_clock in topics.splitlines() else "/clock"
        require(clock in topics.splitlines(), "Gazebo clock topic missing")
        info, _ = command("ign", "topic", "-i", "-t", clock)
        require("ignition.msgs.Clock" in info, "Unexpected Gazebo clock type")
        bridge_prefix, _ = command("ros2", "pkg", "prefix", "ros_gz_bridge")
        bridge = Path(bridge_prefix.strip()) / "lib/ros_gz_bridge/parameter_bridge"
        with background(
            evidence / "bridge.log", str(bridge),
            f"{clock}@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "--ros-args", "-r", f"{clock}:=/clock",
        ) as clock_bridge:
            output, _ = command(
                "ign", "service", "-s", f"{WORLD}/control", "--reqtype",
                "ignition.msgs.WorldControl", "--reptype", "ignition.msgs.Boolean",
                "--timeout", "3000", "--req", "pause: false",
            )
            require(output.strip() == "data: true", "Unpause was rejected")
            check_clock()
            socket_scope(server, clock_bridge)
        topics, _ = command("ros2", "topic", "list", "--no-daemon", "--spin-time", "1")
        require("/clock" not in topics.splitlines(), "ROS clock publisher remains after shutdown")

    services, _ = command("ign", "service", "-l")
    topics, _ = command("ign", "topic", "-l")
    require(not services.strip() and not topics.strip(), "Ignition endpoints remain after shutdown")
    print("PASS clean shutdown and endpoint removal", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, required=True, help="Unused ROS domain (0–101)")
    args = parser.parse_args()
    require(0 <= args.domain <= 101, "Choose an unused ROS domain in 0–101")
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY="1",
                      IGN_IP="127.0.0.1")
    os.environ.pop("DISPLAY", None)
    os.environ.pop("WAYLAND_DISPLAY", None)
    os.environ.pop("IGN_RELAY", None)
    evidence = Path(tempfile.mkdtemp(prefix="ros-phase2a-"))
    print(f"Evidence: {evidence}; ROS_DOMAIN_ID={args.domain}; IGN_IP=127.0.0.1", flush=True)
    versions, _ = command("ign", "gazebo", "--versions")
    require(any(line.startswith("6.") for line in versions.splitlines()), "Gazebo Fortress required")
    command("ip", "-brief", "addr")
    command("ip", "maddr", "show")
    topics, _ = command("ros2", "topic", "list", "--no-daemon", "--spin-time", "1")
    require(set(topics.splitlines()) <= {"/parameter_events", "/rosout"}, "ROS domain has other topics")
    world = Path(get_package_share_directory("robot_arm_bringup")) / "worlds/phase2a.sdf"
    xacro = Path(get_package_share_directory("robot_arm_description")) / "urdf/reference_arm.urdf.xacro"
    run_world(world, xacro, evidence / "first")
    run_world(world, xacro, evidence / "restart")
    print("PASS fresh restart", flush=True)


def terminate(signum, _frame):
    # Unwind the process-group context managers on an external timeout.
    raise SystemExit(128 + signum)


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, terminate)
    main()
