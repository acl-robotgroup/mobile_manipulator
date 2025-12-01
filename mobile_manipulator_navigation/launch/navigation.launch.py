from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # arguments
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
            description="Whether to visualize the navigation stack",
        ))

    # initialize substitutions
    use_sim_time = LaunchConfiguration("use_sim_time")
    visualize = LaunchConfiguration("visualize")

    config = PathJoinSubstitution([
        FindPackageShare("mobile_manipulator_navigation"),
        "config",
        "navigation.yaml",
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("mobile_manipulator_navigation"),
        "rviz",
        "visualize.rviz",
    ])

    # nodes
    # note: tried to use composable nodes but parameters for global and local
    # costmaps were not being set correctly
    nodes = []
    nodes.append(
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            parameters=[{
                "autostart":
                True,
                "node_names": [
                    "behavior_server",
                    "bt_navigator",
                    "planner_server",
                    "controller_server",
                ],
            }],
        ))
    nodes.append(
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            parameters=[config],
        ))
    nodes.append(
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            parameters=[config],
        ))
    nodes.append(
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            parameters=[config],
        ))
    nodes.append(
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
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
    
    # SetUseSimTime only allows to set to true
    use_sim_time_group = GroupAction([
        SetParameter("use_sim_time", use_sim_time),
    ] + nodes)

    return LaunchDescription(arguments + [use_sim_time_group])
