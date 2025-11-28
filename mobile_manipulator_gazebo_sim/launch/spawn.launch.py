from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # arguments
    arguments = []
    arguments.append(
        DeclareLaunchArgument(
            "x",
            default_value="0",
            description="X coordinate to spawn the robot at",
        ))
    arguments.append(
        DeclareLaunchArgument(
            "y",
            default_value="0",
            description="Y coordinate to spawn the robot at",
        ))
    arguments.append(
        DeclareLaunchArgument(
            "z",
            default_value="0",
            description="Z coordinate to spawn the robot at",
        ))
    arguments.append(
        DeclareLaunchArgument(
            "yaw",
            default_value="0",
            description="Yaw orientation to spawn the robot at",
        ))

    # initialize substitutions
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    gazebo_xacro = PathJoinSubstitution([
        FindPackageShare("mobile_manipulator_gazebo_sim"),
        "urdf",
        "gazebo.xacro",
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
            launch_arguments={
                "xacro_args": [
                    "extra_xacro:=",
                    gazebo_xacro,
                ],
                "use_sim_time": "True",
            }.items(),
        ))

    # nodes
    nodes = []
    nodes.append(
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-topic",
                "robot_description",
                "-name",
                "mobile_manipulator",
                "-x",
                x,
                "-y",
                y,
                "-z",
                z,
                "-Y",
                yaw,
            ],
        ))

    return LaunchDescription(arguments + includes + nodes)
