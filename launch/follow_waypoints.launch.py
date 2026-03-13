import math
import os
import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _build_actions(context, nav2_launch, goal_bridge):
    follow_waypoints_file = LaunchConfiguration("follow_waypoints_file").perform(context)
    follow_start_delay_sec = LaunchConfiguration("follow_start_delay_sec")
    use_sim_time = LaunchConfiguration("use_sim_time")
    local_map_topic = LaunchConfiguration("local_map_topic")
    global_map_topic = LaunchConfiguration("global_map_topic")

    goal_text = _build_follow_waypoints_goal(follow_waypoints_file)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
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
        TimerAction(
            period=follow_start_delay_sec,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "action",
                        "send_goal",
                        "/follow_waypoints",
                        "nav2_msgs/action/FollowWaypoints",
                        goal_text,
                    ],
                    output="screen",
                )
            ],
        ),
    ]


def generate_launch_description():
    navigation_share = get_package_share_directory("rmcs-navigation")
    navigation_prefix = get_package_prefix("rmcs-navigation")

    nav2_launch = os.path.join(navigation_share, "launch", "nav2.launch.py")
    goal_bridge = os.path.join(
        navigation_prefix,
        "lib",
        "rmcs-navigation",
        "goal_topic_bridge.py",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "follow_waypoints_file",
            default_value=os.path.join(navigation_share, "config", "follow_waypoints.yaml"),
        ),
        DeclareLaunchArgument("follow_start_delay_sec", default_value="2.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("local_map_topic", default_value="/local_map"),
        DeclareLaunchArgument("global_map_topic", default_value="/map"),
        OpaqueFunction(
            function=_build_actions,
            kwargs={
                "nav2_launch": nav2_launch,
                "goal_bridge": goal_bridge,
            },
        ),
    ])


def _build_follow_waypoints_goal(config_file):
    with open(config_file, "r", encoding="utf-8") as stream:
        content = yaml.safe_load(stream) or {}

    follow = content.get("follow", {})
    frame_id = follow.get("frame_id", "odom")
    waypoints = follow.get("poses", [])

    if not waypoints:
        raise RuntimeError(f"No waypoints in follow config: {config_file}")

    pose_entries = []
    for point in waypoints:
        x, y, yaw = _parse_waypoint(point)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        pose_entries.append(
            "{header: {frame_id: '%s'}, pose: {position: {x: %s, y: %s, "
            "z: 0.0}, orientation: {x: 0.0, y: 0.0, z: %s, w: %s}}}"
            % (frame_id, x, y, qz, qw)
        )

    return "{poses: [%s]}" % ", ".join(pose_entries)


def _parse_waypoint(point):
    if isinstance(point, list) and len(point) >= 2:
        x = float(point[0])
        y = float(point[1])
        yaw = float(point[2]) if len(point) >= 3 else 0.0
        return x, y, yaw

    if isinstance(point, dict):
        x = float(point.get("x", 0.0))
        y = float(point.get("y", 0.0))
        yaw = float(point.get("yaw", 0.0))
        return x, y, yaw

    raise RuntimeError(f"Invalid waypoint format: {point}")
