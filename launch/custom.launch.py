import os
import yaml

from ament_index_python.packages import (
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
        "use_local_mock": False,
        "use_initial_pose": True,
    },
    "bag": {
        "use_livox_driver": False,
        "use_point_lio": True,
        "use_bag": True,
        "use_sim_time": True,
        "use_local_mock": False,
        "use_initial_pose": False,
    },
    "static": {
        "use_livox_driver": False,
        "use_point_lio": False,
        "use_bag": False,
        "use_sim_time": False,
        "use_local_mock": True,
        "use_initial_pose": False,
    },
}


def _build_mode_actions(
    context,
    livox_launch,
    point_lio_launch,
    nav2_launch,
    map_yaml,
    local_map_yaml,
    local_map_mock_topic,
):
    mode = LaunchConfiguration("mode").perform(context)
    bag_path = LaunchConfiguration("bag_path")
    bag_use_clock = LaunchConfiguration("bag_use_clock").perform(context)
    local_map_topic = LaunchConfiguration("local_map_topic")
    global_map_topic = LaunchConfiguration("global_map_topic")

    if mode not in MODE_MAP:
        raise RuntimeError(
            f"Invalid mode '{mode}', expected one of {sorted(MODE_MAP)}"
        )

    config = MODE_MAP[mode]

    if config["use_local_mock"]:
        nav2_local_map_topic = local_map_mock_topic
    else:
        nav2_local_map_topic = local_map_topic

    result_actions = []

    # Basic Node
    use_local_mock = str(config["use_local_mock"]).lower()
    if use_local_mock == "true":
        use_local_mode = "false"
    else:
        use_local_mode = "true"
    nav2_actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                "use_sim_time": str(config["use_sim_time"]).lower(),
                "autostart": "true",
                "local_map_topic": nav2_local_map_topic,
                "local_map_transient_local": use_local_mock,
                "global_map_topic": global_map_topic,
                "use_lifecycle_manager": "true",
                "enable_local_map_node": use_local_mode,
            }.items(),
        ),
        Node(
            package="rmcs-navigation",
            executable="goal_topic_bridge.py",
            name="goal_topic_bridge",
            output="screen",
        ),
    ]

    # Map Source
    map_node_names = ["map_server"]
    if config["use_local_mock"]:
        map_node_names.append("local_map_server")

    result_actions.append(Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "use_sim_time": config["use_sim_time"],
            "yaml_filename": map_yaml,
            "topic_name": global_map_topic,
            "frame_id": "world",
        }],
    ))
    if config["use_local_mock"]:
        result_actions.append(Node(
            package="nav2_map_server",
            executable="map_server",
            name="local_map_server",
            output="screen",
            parameters=[{
                "use_sim_time": config["use_sim_time"],
                "yaml_filename": local_map_yaml,
                "topic_name": local_map_mock_topic,
                "frame_id": "base_link",
            }],
        ))
    result_actions.append(Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        sigterm_timeout="2",
        sigkill_timeout="4",
        parameters=[
            {"use_sim_time": config["use_sim_time"]},
            {"autostart": True},
            {"node_names": map_node_names},
        ],
    ))

    result_actions.append(
        TimerAction(period=1.0, actions=nav2_actions))

    if config["use_livox_driver"]:
        result_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(livox_launch),
            )
        )
    if config["use_point_lio"]:
        result_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(point_lio_launch),
            )
        )
    if config["use_bag"]:
        bag_cmd = ["ros2", "bag", "play", bag_path]
        if bag_use_clock.lower() in {"1", "true", "yes", "on"}:
            bag_cmd.append("--clock")
        result_actions.append(ExecuteProcess(cmd=bag_cmd, output="screen"))

    # Transform Initialization
    initial_world_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
        output="screen",
    )
    result_actions.append(
        TimerAction(
            period=1.0,
            actions=[initial_world_link]
        )
    )

    if config["use_initial_pose"]:
        initial_base_link = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_base_link_tf",
            arguments=[
                "1.2", "6.3", "0",
                "0", "0", "0",
                "world", "odom"
            ],
            output="screen",
        )
        result_actions.append(
            TimerAction(
                period=1.5,
                actions=[initial_base_link]
            )
        )

    return result_actions


def generate_launch_description():
    navigation_share = get_package_share_directory("rmcs-navigation")
    livox_share = get_package_share_directory("livox_ros_driver2")
    point_lio_share = get_package_share_directory("point_lio")

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
    custom_config = _load_custom_config(
        os.path.join(navigation_share, "config", "custom.yaml")
    )
    nav_runtime_config = custom_config.get("navigation", {})
    map_yaml_config = nav_runtime_config.get("map_yaml", "maps/rmul.yaml")
    local_map_yaml_config = nav_runtime_config.get(
        "local_map_yaml", "maps/local_mock.yaml")
    local_map_mock_topic = nav_runtime_config.get(
        "local_map_mock_topic", "/local_map_mock")

    if os.path.isabs(map_yaml_config):
        map_yaml = map_yaml_config
    else:
        map_yaml = os.path.join(navigation_share, map_yaml_config)

    if os.path.isabs(local_map_yaml_config):
        local_map_yaml = local_map_yaml_config
    else:
        local_map_yaml = os.path.join(navigation_share, local_map_yaml_config)

    bag_config = custom_config.get("bag", {})
    bag_path_default = str(bag_config.get("path"))
    bag_use_clock_default = str(bag_config.get("use_clock", True)).lower()

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="online"),
        DeclareLaunchArgument("bag_path", default_value=bag_path_default),
        DeclareLaunchArgument("local_map_topic", default_value="/local_map"),
        DeclareLaunchArgument("global_map_topic", default_value="/map"),
        DeclareLaunchArgument(
            "bag_use_clock", default_value=bag_use_clock_default),
        OpaqueFunction(
            function=_build_mode_actions,
            kwargs={
                "livox_launch": livox_launch,
                "point_lio_launch": point_lio_launch,
                "nav2_launch": nav2_launch,
                "map_yaml": map_yaml,
                "local_map_yaml": local_map_yaml,
                "local_map_mock_topic": local_map_mock_topic,
            },
        ),
    ])


def _load_custom_config(config_file):
    with open(config_file, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}
