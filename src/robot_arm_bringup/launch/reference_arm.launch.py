"""Publish the reference arm model and transforms for inspection."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    description_share = FindPackageShare("robot_arm_description")
    model = PathJoinSubstitution([description_share, "urdf", "reference_arm.urdf.xacro"])
    rviz_config = PathJoinSubstitution([description_share, "rviz", "reference_arm.rviz"])

    def start_state_publisher(context):
        urdf = xacro.process_file(model.perform(context)).toxml()
        return [Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": ParameterValue(urdf, value_type=str)}],
            output="screen",
        )]

    return LaunchDescription([
        DeclareLaunchArgument(
            "publish_joint_states", default_value="true",
            description="Publish the reference pose. Disable when supplying /joint_states.",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="Start RViz on a graphical desktop.",
        ),
        OpaqueFunction(function=start_state_publisher),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=IfCondition(LaunchConfiguration("publish_joint_states")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
