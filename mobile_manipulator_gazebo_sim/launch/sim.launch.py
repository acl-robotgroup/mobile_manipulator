from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():
    # arguments
    arguments = []
    arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([
                FindPackageShare("mobile_manipulator_gazebo_sim"),
                "worlds",
                "depo.sdf",
            ]),
            # default_value="empty.sdf",
            description="World file to load",
        ))

    # initialize substitutions
    world = LaunchConfiguration("world")
    gz_bridge_config = PathJoinSubstitution([
        FindPackageShare("mobile_manipulator_gazebo_sim"),
        "config",
        "gz_bridge.yaml",
    ])

    # includes
    includes = []
    includes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]),
            ]),
            launch_arguments={
                "gz_args": [
                    "-r",
                    " ",
                    world,
                ],
                "on_exit_shutdown": "True",
            }.items(),
        ))
    includes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("mobile_manipulator_gazebo_sim"),
                    "launch",
                    "spawn.launch.py",
                ]),
            ]),
            launch_arguments={}.items(),
        ))
    includes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("mobile_manipulator_bringup"),
                    "launch",
                    "visualize.launch.py",
                ]),
            ]),
            launch_arguments={}.items(),
        ))

    nodes = []
    nodes.append(
        RosGzBridge(
            bridge_name="ros_gz_bridge",
            config_file=gz_bridge_config,
        ))

    return LaunchDescription(arguments + includes + nodes)
