from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, RegisterEventHandler, LogInfo
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration, IfElseSubstitution, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, SetParameter
from launch_ros.descriptions import ComposableNode


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
            "bag_path",
            default_value=".",
            description="Path to store the recorded rosbag",
        ))

    # initialize substitutions
    use_sim_time = LaunchConfiguration("use_sim_time")
    bag_path = LaunchConfiguration("bag_path")

    sim_time_flag = IfElseSubstitution(
        condition=use_sim_time,
        if_value="--use-sim-time",
        else_value="",
    )
    camera_bag = PathJoinSubstitution([bag_path, "camera"])
    pose_bag = PathJoinSubstitution([bag_path, "pose"])

    # composable nodes
    composable_nodes = []
    composable_nodes.append(
        ComposableNode(
            name="infra1_encoder_node",
            package="isaac_ros_h264_encoder",
            plugin="nvidia::isaac_ros::h264_encoder::EncoderNode",
            parameters=[{
                "input_width": 1280,
                "input_height": 800,
            }],
            remappings=[
                ("image_raw", "camera_front/infra1/image_raw"),
                ("image_compressed", "camera_front/infra1/image_compressed"),
            ],
        ))
    composable_nodes.append(
        ComposableNode(
            name="infra2_encoder_node",
            package="isaac_ros_h264_encoder",
            plugin="nvidia::isaac_ros::h264_encoder::EncoderNode",
            parameters=[{
                "input_width": 1280,
                "input_height": 800,
            }],
            remappings=[
                ("image_raw", "camera_front/infra2/image_raw"),
                ("image_compressed", "camera_front/infra2/image_compressed"),
            ],
        ))

    # processes
    processes = []

    # camera bag
    processes.append(
        ExecuteProcess(cmd=[
            "ros2",
            "bag",
            "record",
            "/tf",
            "/tf_static",
            "camera_front/infra1/camera_info",
            "camera_front/infra1/image_compressed",
            "camera_front/infra2/camera_info",
            "camera_front/infra2/image_compressed",
            sim_time_flag,
            "-o",
            camera_bag,
        ]))

    # pose bag
    processes.append(
        ExecuteProcess(cmd=[
            "ros2",
            "bag",
            "record",
            "odom",
            sim_time_flag,
            "-o",
            pose_bag,
        ]))

    # nodes
    nodes = []
    nodes.append(
        ComposableNodeContainer(
            name="encoder_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=composable_nodes,
        ))

    # SetUseSimTime only allows to set to true
    use_sim_time_group = GroupAction([
        SetParameter("use_sim_time", use_sim_time),
    ] + nodes)

    events = []
    events.append(
        RegisterEventHandler(
            OnShutdown(on_shutdown=ExecuteProcess(cmd=[
                "sleep",
                "100",
            ]))))
    events.append(
        RegisterEventHandler(
            OnShutdown(on_shutdown=LogInfo(msg=[
                "shutting down",
            ]))))

    return LaunchDescription(arguments + processes + [use_sim_time_group])
