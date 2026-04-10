import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_config(config_file):
    with open(config_file, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _build_actions(context):
    navigation_share = get_package_share_directory("rmcs-navigation")
    odin_share = get_package_share_directory("odin_ros_driver")
    config_name = LaunchConfiguration("config_name").perform(context)
    odin_config_file = LaunchConfiguration("odin_config_file").perform(context)
    if config_name == "":
        raise RuntimeError("Launch argument 'config_name' must not be empty")

    odin_launch = os.path.join(
        odin_share,
        "launch",
        "odin1_ros2.launch.py",
    )
    nav2_launch = os.path.join(navigation_share, "launch", "nav2.launch.py")
    nav2_params = os.path.join(navigation_share, "config", "nav2.yaml")

    navigation_config = _load_config(
        os.path.join(navigation_share, "config", f"{config_name}.yaml")
    )
    map_yaml_config = navigation_config["navigation"]["map_yaml"]
    if map_yaml_config is None or map_yaml_config == "":
        raise RuntimeError(
            "Missing required key 'navigation.map_yaml' in "
            f"config/{config_name}.yaml"
        )

    if os.path.isabs(map_yaml_config):
        map_yaml = map_yaml_config
    else:
        map_yaml = os.path.join(navigation_share, map_yaml_config)

    local_map_topic = "/map"
    global_map_topic = "/map"

    odin_launch_arguments = {}
    if odin_config_file != "":
        odin_launch_arguments["config_file"] = odin_config_file

    nav2_actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                "use_sim_time": "false",
                "autostart": "true",
                "params_file": nav2_params,
                "local_map_topic": local_map_topic,
                "local_map_transient_local": "true",
                "global_map_topic": global_map_topic,
                "use_lifecycle_manager": "true",
                "enable_local_map_node": "false",
            }.items(),
        ),
        Node(
            package="rmcs-navigation",
            executable="goal_topic_bridge.py",
            name="goal_topic_bridge",
            output="screen",
        ),
    ]

    actions = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "yaml_filename": map_yaml,
                "topic_name": global_map_topic,
                "frame_id": "world",
            }],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            output="screen",
            sigterm_timeout="2",
            sigkill_timeout="4",
            parameters=[
                {"use_sim_time": False},
                {"autostart": True},
                {"node_names": ["map_server"]},
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(odin_launch),
            launch_arguments=odin_launch_arguments.items(),
        ),
        TimerAction(period=3.0, actions=nav2_actions),
        TimerAction(
            period=0.1,
            actions=[
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="world_to_odom_tf",
                    arguments=[
                        "0.18", "0", "0.25",
                        "0", "0", "0", "world", "odom"
                    ],
                    output="screen",
                ),
            ],
        ),
        TimerAction(
            period=0.1,
            actions=[
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="sensor_to_chassis_tf",
                    arguments=[
                        "-0.18", "0", "-0.25",
                        "0", "0", "0", "odin1_base_link", "odin1_chassis_link"
                    ],
                    output="screen",
                ),
            ],
        ),
    ]

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config_name"),
        DeclareLaunchArgument("odin_config_file", default_value=""),
        OpaqueFunction(function=_build_actions),
    ])
