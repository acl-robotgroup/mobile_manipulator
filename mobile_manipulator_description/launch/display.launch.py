from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # initialize substitutions
    rviz_config = PathJoinSubstitution([
        FindPackageShare("mobile_manipulator_description"),
        "rviz",
        "display.rviz",
    ])

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

    # nodes
    nodes = []
    nodes.append(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ))
    nodes.append(
        Node(
            name="rviz2",
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                rviz_config,
            ],
        ))

    return LaunchDescription(includes + nodes)
