#!/usr/bin/env python3

import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action.client import ActionClient
from rclpy.node import Node


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _quaternion_to_yaw(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


class FollowWaypointsRunner(Node):
    def __init__(self) -> None:
        super().__init__("follow_waypoints_runner")

        self.declare_parameter("config_file", "")
        self.declare_parameter("odom_topic", "/aft_mapped_to_init")
        self.declare_parameter("distance_tolerance", 0.25)
        self.declare_parameter("yaw_tolerance", 0.35)

        config_file = (
            self.get_parameter("config_file")
            .get_parameter_value()
            .string_value
        )
        if not config_file:
            raise RuntimeError("Parameter 'config_file' is required")

        self.odom_topic = (
            self.get_parameter("odom_topic").get_parameter_value().string_value
        )
        self.distance_tolerance = (
            self.get_parameter("distance_tolerance")
            .get_parameter_value()
            .double_value
        )
        self.yaw_tolerance = (
            self.get_parameter("yaw_tolerance")
            .get_parameter_value()
            .double_value
        )

        self.frame_id, self.waypoints = self._load_waypoints(Path(config_file))
        self.path_pub = self.create_publisher(
            NavPath,
            "/follow_waypoints_path",
            1,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._on_odom,
            10,
        )
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.current_index = -1
        self.current_pose = None
        self.goal_in_flight = False
        self.goal_handle = None
        self.finished = False

        self._publish_path()
        self.create_timer(0.5, self._ensure_started)

    def _load_waypoints(
        self,
        config_path: Path,
    ) -> tuple[str, list[tuple[float, float, float]]]:
        with config_path.open("r", encoding="utf-8") as stream:
            content = yaml.safe_load(stream) or {}

        follow = content.get("follow", {})
        frame_id = follow.get("frame_id", "odom")
        poses = follow.get("poses", [])
        if not poses:
            raise RuntimeError(f"No waypoints in follow config: {config_path}")

        waypoints: list[tuple[float, float, float]] = []
        for point in poses:
            if isinstance(point, list) and len(point) >= 2:
                x = float(point[0])
                y = float(point[1])
                yaw = float(point[2]) if len(point) >= 3 else 0.0
            elif isinstance(point, dict):
                x = float(point.get("x", 0.0))
                y = float(point.get("y", 0.0))
                yaw = float(point.get("yaw", 0.0))
            else:
                raise RuntimeError(f"Invalid waypoint format: {point}")
            waypoints.append((x, y, yaw))
        return frame_id, waypoints

    def _publish_path(self) -> None:
        path = NavPath()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        poses: list[PoseStamped] = []
        for x, y, yaw in self.waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            (
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ) = _yaw_to_quaternion(yaw)
            poses.append(pose)
        path.poses = poses
        self.path_pub.publish(path)

    def _ensure_started(self) -> None:
        if self.current_index >= 0 or self.finished:
            return
        if not self.client.wait_for_server(timeout_sec=0.0):
            self.get_logger().info(
                "Waiting for /navigate_to_pose action server..."
            )
            return
        self._send_next_goal()

    def _send_next_goal(self) -> None:
        self.current_index += 1
        if self.current_index >= len(self.waypoints):
            self.finished = True
            self.goal_in_flight = False
            self.get_logger().info("All waypoints reached")
            return

        x, y, yaw = self.waypoints[self.current_index]
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        (
            goal.pose.pose.orientation.z,
            goal.pose.pose.orientation.w,
        ) = _yaw_to_quaternion(yaw)

        self.goal_in_flight = True
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self.get_logger().info(
            f"Send waypoint {self.current_index + 1}/{len(self.waypoints)}: "
            f"x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )

    def _on_goal_response(self, future) -> None:
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.goal_in_flight = False
            self.get_logger().warning(
                f"Waypoint {self.current_index + 1} was rejected"
            )
            return

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        status = future.result().status
        self.goal_handle = None
        if self.finished:
            return

        if status == 4:
            self.goal_in_flight = False
            self.get_logger().info(
                f"Waypoint {self.current_index + 1} completed by action result"
            )
            self._send_next_goal()
            return

        self.goal_in_flight = False
        self.get_logger().warning(
            f"Waypoint {self.current_index + 1} ended with status {status}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self.current_pose = msg.pose.pose
        if self.finished or not self.goal_in_flight or self.current_index < 0:
            return

        target_x, target_y, target_yaw = self.waypoints[self.current_index]
        dx = target_x - self.current_pose.position.x
        dy = target_y - self.current_pose.position.y
        distance = math.hypot(dx, dy)

        current_yaw = _quaternion_to_yaw(
            self.current_pose.orientation.z,
            self.current_pose.orientation.w,
        )
        yaw_error = abs(_normalize_angle(target_yaw - current_yaw))

        if distance > self.distance_tolerance:
            return
        if yaw_error > self.yaw_tolerance:
            return

        self.goal_in_flight = False
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.get_logger().info(
            f"Waypoint {self.current_index + 1} reached: "
            f"distance={distance:.3f}, yaw_error={yaw_error:.3f}"
        )
        self._send_next_goal()


def main() -> None:
    rclpy.init()
    node = FollowWaypointsRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
