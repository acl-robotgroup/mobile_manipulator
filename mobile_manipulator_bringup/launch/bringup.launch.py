from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # arguments
    arguments = []
    arguments.append(
        DeclareLaunchArgument(
            "visualize",
            default_value="false",
            description="Whether to visualize robot perception",
        ))

    # initialize substitutions
    visualize = LaunchConfiguration("visualize")

    config = PathJoinSubstitution([
        FindPackageShare("mobile_manipulator_bringup"),
        "config",
        "bringup.yaml",
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
            launch_arguments={}.items(),
        ))

    # composable nodes
    composable_nodes = []
    composable_nodes.append(
        ComposableNode(
            name="orbbec_camera_node",
            namespace="camera_top",
            package="orbbec_camera",
            plugin="orbbec_camera::OBCameraNodeDriver",
            parameters=[config],
        ))

    # nodes
    nodes = []
    nodes.append(
        Node(
            name="tracer_base_node",
            package="tracer_base",
            executable="tracer_base_node",
            emulate_tty=True,  # requires tty to work properly
            parameters=[config],
        ))

    nodes.append(
        ComposableNodeContainer(
            name="sensors_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=composable_nodes,
        ))

    # the topic names of sllidar_node do not consider the node's name, hence
    # namespace is necessary
    nodes.append(
        Node(
            name="sllidar_node",
            namespace="lidar_front",
            package="sllidar_ros2",
            executable="sllidar_node",
            parameters=[config],
        ))
    nodes.append(
        Node(
            name="sllidar_node",
            namespace="lidar_rear",
            package="sllidar_ros2",
            executable="sllidar_node",
            parameters=[config],
        ))

    nodes.append(
        Node(
            name="rviz2",
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(visualize),
        ))

    events = []

    # load camera nodes with delay due to device access conflict
    camera_front_node = ComposableNode(
        name="orbbec_camera_node",
        namespace="camera_front",
        package="orbbec_camera",
        plugin="orbbec_camera::OBCameraNodeDriver",
        parameters=[config],
    )
    load_camera_front_node = LoadComposableNodes(
        target_container="sensors_container",
        composable_node_descriptions=[camera_front_node],
    )
    events.append(TimerAction(
        period=5.0,
        actions=[load_camera_front_node],
    ))

    camera_left_node = ComposableNode(
        name="orbbec_camera_node",
        namespace="camera_left",
        package="orbbec_camera",
        plugin="orbbec_camera::OBCameraNodeDriver",
        parameters=[config],
    )
    load_camera_left_node = LoadComposableNodes(
        target_container="sensors_container",
        composable_node_descriptions=[camera_left_node],
    )
    events.append(TimerAction(
        period=10.0,
        actions=[load_camera_left_node],
    ))

    camera_right_node = ComposableNode(
        name="orbbec_camera_node",
        namespace="camera_right",
        package="orbbec_camera",
        plugin="orbbec_camera::OBCameraNodeDriver",
        parameters=[config],
    )
    load_camera_right_node = LoadComposableNodes(
        target_container="sensors_container",
        composable_node_descriptions=[camera_right_node],
    )
    events.append(TimerAction(
        period=15.0,
        actions=[load_camera_right_node],
    ))

    return LaunchDescription(arguments + includes + nodes + events)
