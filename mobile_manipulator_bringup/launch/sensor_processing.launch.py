from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # arguments
    arguments = []

    # initialize substitutions
    config = PathJoinSubstitution([
        FindPackageShare("mobile_manipulator_bringup"),
        "config",
        "sensor_processing.yaml",
    ])

    # composable nodes
    composable_nodes = []
    composable_nodes.append(
        ComposableNode(
            name="laser_filter",
            namespace="lidar_front",
            package="laser_filters",
            plugin="ScanToScanFilterChain",
            parameters=[config],
        ))

    composable_nodes.append(
        ComposableNode(
            name="laser_filter",
            namespace="lidar_rear",
            package="laser_filters",
            plugin="ScanToScanFilterChain",
            parameters=[config],
        ))

    composable_nodes.append(
        ComposableNode(
            name="dual_laser_merger",
            package="dual_laser_merger",
            plugin="merger_node::MergerNode",
            parameters=[config],
        ))

    # nodes
    nodes = []
    nodes.append(
        ComposableNodeContainer(
            name="sensor_processing_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=composable_nodes,
        ))

    return LaunchDescription(arguments + nodes)
