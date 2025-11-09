from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # arguments
    arguments = []

    # includes
    includes = []
    includes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("mobile_manipulator_description"),
                    "launch",
                    "description.launch.py",
                ]),
            ]),
            launch_arguments={}.items(),
        ))
    includes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("tracer_ros2"),
                    "launch",
                    "tracer_base.launch.py",
                ]),
            ]),
            launch_arguments={}.items(),
        ))

    # nodes
    nodes = []

    return LaunchDescription(arguments + includes + nodes)
