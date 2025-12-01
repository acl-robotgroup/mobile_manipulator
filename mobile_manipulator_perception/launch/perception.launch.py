from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, GroupAction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LifecycleNode, Node, SetParameter
from launch_ros.descriptions import ComposableNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # artuments
    arguments = []
    arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Whether to use simulation time",
        ))
    arguments.append(
        DeclareLaunchArgument(
            "visualize",
            default_value="false",
            description="Whether to visualize robot perception",
        ))

    # initialize substitutions
    use_sim_time = LaunchConfiguration("use_sim_time")
    visualize = LaunchConfiguration("visualize")

    config = PathJoinSubstitution([
        FindPackageShare("mobile_manipulator_perception"),
        "config",
        "perception.yaml",
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("mobile_manipulator_perception"),
        "rviz",
        "visualize.rviz",
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

    composable_nodes.append(
        ComposableNode(
            name="nvblox_node",
            package="nvblox_ros",
            plugin="nvblox::NvbloxNode",
            parameters=[config],
            remappings=[
                ("camera_0/color/image", "camera_front/color/image_raw"),
                ("camera_0/color/camera_info",
                 "camera_front/color/camera_info"),
                ("camera_0/depth/image", "camera_front/depth/image_raw"),
                ("camera_0/depth/camera_info",
                 "camera_front/depth/camera_info"),
                ("camera_1/color/image", "camera_left/color/image_raw"),
                ("camera_1/color/camera_info",
                 "camera_left/color/camera_info"),
                ("camera_1/depth/image", "camera_left/depth/image_raw"),
                ("camera_1/depth/camera_info",
                 "camera_left/depth/camera_info"),
                ("camera_2/color/image", "camera_right/color/image_raw"),
                ("camera_2/color/camera_info",
                 "camera_right/color/camera_info"),
                ("camera_2/depth/image", "camera_right/depth/image_raw"),
                ("camera_2/depth/camera_info",
                 "camera_right/depth/camera_info"),
            ],
        ))

    # nodes
    nodes = []
    nodes.append(
        ComposableNodeContainer(
            name="perception_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=composable_nodes,
        ))
    slam_toolbox = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        parameters=[config],
    )
    nodes.append(slam_toolbox)
    nodes.append(
        Node(
            name="rviz2",
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(visualize),
        ))

    # SetUseSimTime only allows to set to true
    use_sim_time_group = GroupAction([
        SetParameter("use_sim_time", use_sim_time),
    ] + nodes)

    # events
    events = []
    events.append(
        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox),
            transition_id=Transition.TRANSITION_CONFIGURE)))
    events.append(
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=slam_toolbox,
                start_state="configuring",
                goal_state="inactive",
                entities=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox),
                        transition_id=Transition.TRANSITION_ACTIVATE))
                ])))

    return LaunchDescription(arguments + [use_sim_time_group] + events)
