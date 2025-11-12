import os
import re
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro


def _create_robot_state_publisher(context: LaunchContext):
    xacro_args = LaunchConfiguration("xacro_args")
    use_sim_time = LaunchConfiguration("use_sim_time")

    xacro_file = os.path.join(
        get_package_share_directory("mobile_manipulator_description"),
        "urdf",
        "mobile_manipulator.xacro",
    )

    xacro_args_str = xacro_args.perform(context)

    mappings = {}
    if xacro_args_str:
        try:
            tokens = shlex.split(xacro_args_str)
        except Exception:
            tokens = xacro_args_str.split()

        for token in tokens:
            if ":=" in token:
                key, val = token.split(":=", 1)
                mappings[key] = val
            else:
                # treat bare tokens as boolean flags
                mappings[token] = "true"

    urdf = xacro.process_file(xacro_file, mappings=mappings).toxml()

    # replace package://... with file://<package_share_dir>/...
    def repl(m):
        pkg_name = m.group(1)
        rest = m.group(2)
        try:
            pkg_dir = get_package_share_directory(pkg_name)
        except Exception:
            return m.group(0)

        return "file://" + os.path.join(pkg_dir, rest.lstrip("/"))

    urdf = re.sub(r"package://([^/]+)(/[^\"' >]+)", repl, urdf)

    node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(urdf),
            "use_sim_time": use_sim_time,
        }],
    )

    return [node]


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

    # nodes
    nodes = []
    nodes.append(OpaqueFunction(function=_create_robot_state_publisher))

    return LaunchDescription(arguments + nodes)
