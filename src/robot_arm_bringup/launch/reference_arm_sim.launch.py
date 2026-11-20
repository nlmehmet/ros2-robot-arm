"""Run the anchored reference arm in the Fortress server."""

from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml


# Keep commanded waypoints inside the model's physical stops.
HARD_STOP_CLEARANCE_RAD = 0.02


def write_controller_config(urdf, template, destination):
    config = yaml.safe_load(template.read_text())
    trajectory = config["arm_trajectory_controller"]["ros__parameters"]
    limits = {joint.attrib["name"]: joint.find("limit") for joint in ET.fromstring(urdf).findall("joint")}
    trajectory["position_bounds"] = {
        name: [
            float(limits[name].attrib["lower"]) + HARD_STOP_CLEARANCE_RAD,
            float(limits[name].attrib["upper"]) - HARD_STOP_CLEARANCE_RAD,
        ]
        for name in trajectory["joints"]
    }
    destination.write_text(yaml.safe_dump(config))


def generate_launch_description():
    description = Path(get_package_share_directory("robot_arm_description"))
    bringup = Path(get_package_share_directory("robot_arm_bringup"))
    control = Path(get_package_share_directory("robot_arm_control"))
    plugin = Path(get_package_prefix("gz_ros2_control")) / "lib/libign_ros2_control-system.so"
    config_directory = tempfile.TemporaryDirectory(prefix="reference-arm-sim-")
    controller_config = Path(config_directory.name) / "controllers.yaml"
    urdf = xacro.process_file(
        str(description / "urdf/reference_arm_sim.urdf.xacro"),
        mappings={
            "controller_config": str(controller_config),
            "control_plugin": str(plugin),
        },
    ).toxml()
    write_controller_config(urdf, control / "config/reference_arm_sim.yaml", controller_config)

    server = ExecuteProcess(
        cmd=["ign", "gazebo", "-s", "-r", str(bringup / "worlds/phase2a.sdf")],
        output="screen",
    )
    publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": ParameterValue(urdf, value_type=str), "use_sim_time": True}],
        output="screen",
    )
    clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/world/phase2a/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        remappings=[("/world/phase2a/clock", "/clock")],
        output="screen",
    )
    spawn = Node(
        package="ros_gz_sim", executable="create",
        arguments=["-world", "phase2a", "-topic", "/robot_description", "-name", "reference_arm", "-x", "2"],
        output="screen",
    )
    controllers = Node(
        package="controller_manager", executable="spawner",
        arguments=[
            "joint_state_broadcaster", "arm_trajectory_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "15",
        ],
        output="screen",
    )

    def continue_or_shutdown(event, context, label, next_action=None):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason=f"{label} exited with {event.returncode}"))]
        return [next_action] if next_action is not None else []

    def clean_controller_config(_event, _context):
        config_directory.cleanup()
        return []

    return LaunchDescription([
        RegisterEventHandler(OnShutdown(on_shutdown=clean_controller_config)),
        RegisterEventHandler(OnProcessExit(
            target_action=server,
            on_exit=[EmitEvent(event=Shutdown(reason="Gazebo server exited"))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=spawn,
            on_exit=lambda event, context: continue_or_shutdown(event, context, "Robot spawn", controllers),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=controllers,
            on_exit=lambda event, context: continue_or_shutdown(event, context, "Controller spawner"),
        )),
        publisher, clock, server, spawn,
    ])
