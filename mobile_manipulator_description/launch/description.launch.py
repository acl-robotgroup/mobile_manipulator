from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description():
    # arguments
    arguments = []
    arguments.append(
        DeclareLaunchArgument(
            "xacro_args",
            default_value="",
            description=
            "Extra args passed to xacro, e.g. \"use_sim_time:=true\"",
        ))
    arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Whether to use simulation time",
        ))

    # initialize substitutions
    xacro_args = LaunchConfiguration("xacro_args")
    use_sim_time = LaunchConfiguration("use_sim_time")
    model_file = PathJoinSubstitution([
        FindPackageShare("mobile_manipulator_description"),
        "urdf",
        "mobile_manipulator.xacro",
    ])
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            model_file,
            " ",
            xacro_args,
        ]))

    # nodes
    nodes = []
    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }],
        ))

    return LaunchDescription(arguments + nodes)
