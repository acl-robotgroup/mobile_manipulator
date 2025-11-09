from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # arguments
    arguments = []

    # initialize substitutions
    rviz_config = PathJoinSubstitution([
        FindPackageShare("mobile_manipulator_bringup"),
        "rviz",
        "display.rviz",
    ])

    # nodes
    nodes = []
    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d",
                rviz_config,
            ],
        ))

    return LaunchDescription(arguments + nodes)
