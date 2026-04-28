#!/usr/bin/python3

from __future__ import annotations

import copy
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


class MockPickPathPublisher(Node):
    def __init__(self) -> None:
        super().__init__("medipick_mock_pick_path_publisher")

        self.declare_parameter("target_pose_topic", "/medipick/mock_target_pose")
        self.declare_parameter("path_topic", "/medipick/mock_pick_path")
        self.declare_parameter("publish_rate", 2.0)
        self.declare_parameter("approach_distance", 0.18)
        self.declare_parameter("retreat_distance", 0.15)

        target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        path_topic = str(self.get_parameter("path_topic").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self._approach_distance = float(self.get_parameter("approach_distance").value)
        self._retreat_distance = float(self.get_parameter("retreat_distance").value)

        self._target_pose: PoseStamped | None = None
        self._path_publisher = self.create_publisher(Path, path_topic, 10)
        self.create_subscription(PoseStamped, target_pose_topic, self._handle_target_pose, 10)
        self.create_timer(1.0 / max(publish_rate, 0.2), self._publish_path)

        self.get_logger().info(
            f"Mock pick path publisher ready. Target pose: {target_pose_topic}, path: {path_topic}"
        )

    def _handle_target_pose(self, message: PoseStamped) -> None:
        self._target_pose = message

    def _publish_path(self) -> None:
        if self._target_pose is None:
            return

        now = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = now
        path.header.frame_id = self._target_pose.header.frame_id

        target = copy.deepcopy(self._target_pose)
        target.header.stamp = now

        approach_direction = self._compute_approach_direction(target)

        pre_grasp = copy.deepcopy(target)
        pre_grasp.pose.position.x -= approach_direction[0] * self._approach_distance
        pre_grasp.pose.position.y -= approach_direction[1] * self._approach_distance
        pre_grasp.pose.position.z -= approach_direction[2] * self._approach_distance

        retreat = copy.deepcopy(target)
        retreat.pose.position.x -= approach_direction[0] * (self._approach_distance * 0.5)
        retreat.pose.position.y -= approach_direction[1] * (self._approach_distance * 0.5)
        retreat.pose.position.z -= approach_direction[2] * (self._approach_distance * 0.5)
        retreat.pose.position.z += self._retreat_distance

        path.poses = [pre_grasp, target, retreat]
        self._path_publisher.publish(path)

    def _compute_approach_direction(self, target: PoseStamped) -> tuple[float, float, float]:
        q = target.pose.orientation
        xx = q.x * q.x
        yy = q.y * q.y
        zz = q.z * q.z
        xy = q.x * q.y
        xz = q.x * q.z
        yz = q.y * q.z
        wx = q.w * q.x
        wy = q.w * q.y
        wz = q.w * q.z

        # Use the end-effector local +X axis so the preview path matches the target pose
        # semantics used by the planner and RViz pose arrows.
        direction = (
            1.0 - 2.0 * (yy + zz),
            2.0 * (xy + wz),
            2.0 * (xz - wy),
        )
        length = math.sqrt(direction[0] ** 2 + direction[1] ** 2 + direction[2] ** 2)
        if length < 1e-6:
            return (1.0, 0.0, 0.0)

        return (direction[0] / length, direction[1] / length, direction[2] / length)


def main() -> None:
    rclpy.init()
    node = MockPickPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
