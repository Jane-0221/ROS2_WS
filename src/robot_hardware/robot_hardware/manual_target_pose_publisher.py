#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class ManualTargetPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("medipick_manual_target_pose_publisher")

        self.declare_parameter("topic", "/medipick/task/target_pose")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("x", 0.72)
        self.declare_parameter("y", -0.22)
        self.declare_parameter("z", 1.05)
        self.declare_parameter("qx", 0.0)
        self.declare_parameter("qy", 0.70710678)
        self.declare_parameter("qz", 0.0)
        self.declare_parameter("qw", 0.70710678)
        self.declare_parameter("publish_rate", 1.0)

        self._publisher = self.create_publisher(PoseStamped, str(self.get_parameter("topic").value), 10)
        publish_rate = max(float(self.get_parameter("publish_rate").value), 0.1)
        self.create_timer(1.0 / publish_rate, self._publish_target_pose)
        self.get_logger().info("Manual target pose publisher ready.")

    def _publish_target_pose(self) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("frame_id").value)
        message.pose.position.x = float(self.get_parameter("x").value)
        message.pose.position.y = float(self.get_parameter("y").value)
        message.pose.position.z = float(self.get_parameter("z").value)
        message.pose.orientation.x = float(self.get_parameter("qx").value)
        message.pose.orientation.y = float(self.get_parameter("qy").value)
        message.pose.orientation.z = float(self.get_parameter("qz").value)
        message.pose.orientation.w = float(self.get_parameter("qw").value)
        self._publisher.publish(message)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ManualTargetPosePublisher()
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
