from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushROSNamespace
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
    # includes.append(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([
    #             PathJoinSubstitution([
    #                 FindPackageShare("tracer_ros2"),
    #                 "launch",
    #                 "tracer_base.launch.py",
    #             ]),
    #         ]),
    #         launch_arguments={
    #             "port_name": "can2",
    #         }.items(),
    #     ))
    includes.append(
        GroupAction(actions=[
            PushROSNamespace("lidar_front"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("sllidar_ros2"),
                        "launch",
                        "sllidar_s3_launch.py",
                    ]),
                ]),
                launch_arguments={
                    "serial_port": "/dev/rplidar_front",
                    "frame_id": "lidar_front",
                }.items(),
            ),
        ]))
    includes.append(
        GroupAction(actions=[
            PushROSNamespace("lidar_rear"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("sllidar_ros2"),
                        "launch",
                        "sllidar_s3_launch.py",
                    ]),
                ]),
                launch_arguments={
                    "serial_port": "/dev/rplidar_rear",
                    "frame_id": "lidar_rear",
                }.items(),
            ),
        ]))
    camera_front = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("orbbec_camera"),
                "launch",
                "femto_bolt.launch.py",
            ]),
        ]),
        launch_arguments={
            "camera_name": "camera_front",
            "serial_number": "CL8T754005T",
            "device_num": "4",
        }.items(),
    )
    includes.append(TimerAction(
        period=20.0,
        actions=[camera_front],
    ))

    camera_front_lower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("orbbec_camera"),
                "launch",
                "gemini2.launch.py",
            ]),
        ]),
        launch_arguments={
            "camera_name": "camera_front_lower",
            "serial_number": "AY3794300RP",
            "device_num": "4",
        }.items(),
    )
    includes.append(TimerAction(
        period=5.0,
        actions=[camera_front_lower],
    ))

    camera_left = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("orbbec_camera"),
                "launch",
                "gemini2.launch.py",
            ]),
        ]),
        launch_arguments={
            "camera_name": "camera_left",
            "serial_number": "AY3794301EW",
            "device_num": "4",
        }.items(),
    )
    includes.append(TimerAction(
        period=10.0,
        actions=[camera_left],
    ))

    camera_right = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("orbbec_camera"),
                "launch",
                "gemini2.launch.py",
            ]),
        ]),
        launch_arguments={
            "camera_name": "camera_right",
            "serial_number": "AY37943017E",
            "device_num": "4",
        }.items(),
    )
    includes.append(TimerAction(
        period=15.0,
        actions=[camera_right],
    ))

    # nodes
    nodes = []

    return LaunchDescription(arguments + includes + nodes)
