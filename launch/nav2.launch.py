import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    navigation_share = get_package_share_directory("rmcs-navigation")
    local_map_share = get_package_share_directory("rmcs_local_map")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    local_map_params_file = LaunchConfiguration("local_map_params_file")
    local_map_topic = LaunchConfiguration("local_map_topic")
    global_map_topic = LaunchConfiguration("global_map_topic")
    local_map_grid_frame = LaunchConfiguration("local_map_grid_frame")
    local_map_publish_cloud = LaunchConfiguration("local_map_publish_cloud")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    enable_local_map_node = LaunchConfiguration("enable_local_map_node")

    controller_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            "map_topic": local_map_topic,
        },
        convert_types=True,
    )

    planner_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            "map_topic": global_map_topic,
        },
        convert_types=True,
    )

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("use_lifecycle_manager", default_value="true"),
        DeclareLaunchArgument("enable_local_map_node", default_value="true"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                navigation_share, "config", "nav2.yaml"),
        ),
        DeclareLaunchArgument(
            "local_map_params_file",
            default_value=os.path.join(
                local_map_share,
                "config",
                "local_map.yaml",
            ),
        ),
        DeclareLaunchArgument(
            "local_map_topic",
            default_value="/local_map",
        ),
        DeclareLaunchArgument(
            "global_map_topic",
            default_value="/map",
        ),
        DeclareLaunchArgument(
            "local_map_grid_frame",
            default_value="base_link",
        ),
        DeclareLaunchArgument(
            "local_map_publish_cloud",
            default_value="false",
        ),
        Node(
            package="rmcs_local_map",
            executable="local_map",
            name="rmcs_map",
            output="screen",
            condition=IfCondition(enable_local_map_node),
            parameters=[
                local_map_params_file,
                {
                    "use_sim_time": use_sim_time,
                    "name.grid": local_map_topic,
                    "name.frame": local_map_grid_frame,
                    "switch.publish_cloud": local_map_publish_cloud,
                },
            ],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[controller_params, {"use_sim_time": use_sim_time}],
            remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[planner_params, {"use_sim_time": use_sim_time}],
            remappings=remappings,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[planner_params, {"use_sim_time": use_sim_time}],
            remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[planner_params, {"use_sim_time": use_sim_time}],
            remappings=remappings,
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[planner_params, {"use_sim_time": use_sim_time}],
            remappings=remappings,
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[planner_params, {"use_sim_time": use_sim_time}],
            remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": autostart},
                {"node_names": lifecycle_nodes},
            ],
            condition=IfCondition(use_lifecycle_manager),
        ),
    ])
