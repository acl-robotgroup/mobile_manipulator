from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetUseSimTime
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # arguments
    arguments = []
    arguments.append(
        DeclareLaunchArgument(
            "visualize",
            default_value="false",
            description="Whether to visualize the simulated robot",
        ))
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
    visualize = LaunchConfiguration("visualize")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")

    gazebo_xacro = PathJoinSubstitution([
        FindPackageShare("mobile_manipulator_gazebo_sim"),
        "urdf",
        "gazebo.xacro",
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("mobile_manipulator_bringup"),
        "rviz",
        "visualize.rviz",
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
                "use_sim_time": "true",
            }.items(),
        ))

    # nodes
    nodes = []

    # gz_ros2_control::GazeboSimROS2ControlPlugin in the robot description seems
    # to cause gazebo run a built-in controller manager, so no need to run a
    # controller manager node here.
    gz_spawn_entity = Node(
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
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--switch-timeout",
            "60",
        ],
    )
    diff_drive_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_base_controller",
            "--switch-timeout",
            "60",
            "--controller-ros-args",
            "-r diff_drive_base_controller/cmd_vel:=cmd_vel",
            # "--controller-ros-args",
            # "-r /diff_drive_base_controller/odom:=odom",
        ],
    )
    nodes.append(gz_spawn_entity)

    nodes.append(
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )))
    nodes.append(
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_base_controller_spawner],
        )))
    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(visualize),
        ))

    use_sim_time_group = GroupAction([SetUseSimTime(True)] + nodes)

    return LaunchDescription(arguments + includes + [use_sim_time_group])
