import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    navigation_share = get_package_share_directory("rmcs-navigation")
    navigation_prefix = get_package_prefix("rmcs-navigation")

    runner = os.path.join(
        navigation_prefix,
        "lib",
        "rmcs-navigation",
        "follow_waypoints_runner.py",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "follow_waypoints_file",
            default_value=os.path.join(
                navigation_share,
                "config",
                "follow_waypoints.yaml",
            ),
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/aft_mapped_to_init",
        ),
        DeclareLaunchArgument("distance_tolerance", default_value="0.25"),
        DeclareLaunchArgument("yaw_tolerance", default_value="0.35"),
        ExecuteProcess(
            cmd=[
                "python3",
                runner,
                "--ros-args",
                "-p",
                [
                    "config_file:=",
                    LaunchConfiguration("follow_waypoints_file"),
                ],
                "-p",
                ["odom_topic:=", LaunchConfiguration("odom_topic")],
                "-p",
                [
                    "distance_tolerance:=",
                    LaunchConfiguration("distance_tolerance"),
                ],
                "-p",
                [
                    "yaw_tolerance:=",
                    LaunchConfiguration("yaw_tolerance"),
                ],
            ],
            output="screen",
        ),
    ])
