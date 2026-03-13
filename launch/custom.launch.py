import os
import yaml

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


MODE_MAP = {
    "online": {
        "use_livox_driver": True,
        "use_point_lio": True,
        "use_bag": False,
        "use_sim_time": False,
        "nav2_delay_sec": 0.0,
    },
    "bag": {
        "use_livox_driver": False,
        "use_point_lio": True,
        "use_bag": True,
        "use_sim_time": True,
        "nav2_delay_sec": 6.0,
    },
    "static": {
        "use_livox_driver": False,
        "use_point_lio": False,
        "use_bag": False,
        "use_sim_time": False,
        "nav2_delay_sec": 0.0,
    },
}

MAP_SOURCE_TO_MODE = {
    "empty": "empty",
    "demo": "demo",
    "external": None,
}


def _build_mode_actions(
    context,
    livox_launch,
    point_lio_launch,
    nav2_launch,
    goal_bridge,
    static_grid_publisher,
):
    mode = LaunchConfiguration("mode").perform(context)
    bag_path = LaunchConfiguration("bag_path")
    bag_use_clock = LaunchConfiguration("bag_use_clock").perform(context)
    local_map_topic = LaunchConfiguration("local_map_topic")
    global_map_topic = LaunchConfiguration("global_map_topic")
    map_source = LaunchConfiguration("map_source").perform(context)
    pose_fallback = LaunchConfiguration("pose_fallback").perform(context)

    if mode not in MODE_MAP:
        raise RuntimeError(
            f"Invalid mode '{mode}', expected one of {sorted(MODE_MAP)}"
        )

    mode_cfg = MODE_MAP[mode]
    if map_source not in MAP_SOURCE_TO_MODE:
        raise RuntimeError(
            f"Invalid map_source '{map_source}', expected one of "
            f"{sorted(MAP_SOURCE_TO_MODE)}"
        )

    use_pose_fallback = pose_fallback.lower() in {"1", "true", "yes", "on"}
    map_mode = MAP_SOURCE_TO_MODE[map_source]

    nav2_actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                "use_sim_time": str(mode_cfg["use_sim_time"]).lower(),
                "autostart": "true",
                "local_map_topic": local_map_topic,
                "global_map_topic": global_map_topic,
                "use_lifecycle_manager": "true",
            }.items(),
        ),
        ExecuteProcess(
            cmd=["python3", goal_bridge],
            output="screen",
        ),
    ]

    actions = []

    if mode_cfg["use_livox_driver"]:
        actions.insert(
            0,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(livox_launch),
            ),
        )

    if mode_cfg["use_point_lio"] and not use_pose_fallback:
        actions.insert(
            0,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(point_lio_launch),
            ),
        )

    if map_mode is not None:
        actions.append(
            ExecuteProcess(
                cmd=[
                    "python3", static_grid_publisher, "--ros-args",
                    "-p", "frame_id:=world", "-p",
                    ["topic:=",
                        global_map_topic], "-p", f"map_mode:={map_mode}"
                ],
                output="screen",
            )
        )
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_odom_tf",
                arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
                output="screen",
            )
        )

    if mode == "static" or use_pose_fallback:
        actions.extend(
            [
                ExecuteProcess(
                    cmd=[
                        "python3", static_grid_publisher, "--ros-args", "-p",
                        "frame_id:=base_link", "-p",
                        ["topic:=", local_map_topic], "-p", "map_mode:=empty"
                    ],
                    output="screen",
                ),
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="world_to_base_link_tf",
                    arguments=["0", "0", "0", "0", "0",
                               "0", "world", "base_link", ],
                    output="screen",
                ),
            ]
        )

    if mode_cfg["nav2_delay_sec"] > 0.0:
        actions.append(
            TimerAction(
                period=mode_cfg["nav2_delay_sec"],
                actions=nav2_actions,
            )
        )
    else:
        actions.extend(nav2_actions)

    if mode_cfg["use_bag"]:
        bag_cmd = ["ros2", "bag", "play", bag_path]
        if bag_use_clock.lower() in {"1", "true", "yes", "on"}:
            bag_cmd.append("--clock")
        actions.append(
            ExecuteProcess(
                cmd=bag_cmd,
                output="screen",
            )
        )

    return actions


def generate_launch_description():
    navigation_share = get_package_share_directory("rmcs-navigation")
    livox_share = get_package_share_directory("livox_ros_driver2")
    point_lio_share = get_package_share_directory("point_lio")
    navigation_prefix = get_package_prefix("rmcs-navigation")

    livox_launch = os.path.join(
        livox_share,
        "launch",
        "msg_MID360_launch.py",
    )
    point_lio_launch = os.path.join(
        point_lio_share,
        "launch",
        "point_lio.launch.py",
    )
    nav2_launch = os.path.join(navigation_share, "launch", "nav2.launch.py")
    goal_bridge = os.path.join(
        navigation_prefix,
        "lib",
        "rmcs-navigation",
        "goal_topic_bridge.py",
    )
    static_grid_publisher = os.path.join(
        navigation_prefix,
        "lib",
        "rmcs-navigation",
        "static_grid_publisher.py",
    )
    custom_config = _load_custom_config(
        os.path.join(navigation_share, "config", "custom.yaml")
    )
    bag_config = custom_config.get("bag", {})
    bag_path_default = str(
        bag_config.get("path", "/workspaces/data/bag/battlefield_1")
    )
    bag_use_clock_default = str(
        bag_config.get("use_clock", True)
    ).lower()

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="online"),
        DeclareLaunchArgument(
            "bag_path",
            default_value=bag_path_default,
        ),
        DeclareLaunchArgument(
            "bag_use_clock", default_value=bag_use_clock_default),
        DeclareLaunchArgument(
            "local_map_topic",
            default_value="/local_map",
        ),
        DeclareLaunchArgument(
            "global_map_topic",
            default_value="/map",
        ),
        DeclareLaunchArgument("map_source", default_value="empty"),
        DeclareLaunchArgument("pose_fallback", default_value="false"),
        OpaqueFunction(
            function=_build_mode_actions,
            kwargs={
                "livox_launch": livox_launch,
                "point_lio_launch": point_lio_launch,
                "nav2_launch": nav2_launch,
                "goal_bridge": goal_bridge,
                "static_grid_publisher": static_grid_publisher,
            },
        ),
    ])


def _load_custom_config(config_file):
    with open(config_file, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}
