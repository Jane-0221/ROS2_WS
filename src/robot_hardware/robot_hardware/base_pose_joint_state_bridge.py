#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class BasePoseJointStateBridge(Node):
    def __init__(self) -> None:
        super().__init__("medipick_base_pose_joint_state_bridge")

        self.declare_parameter("input_topic", "/odometry/filtered")
        self.declare_parameter("output_topic", "/medipick/hardware/base_joint_states")
        self.declare_parameter("joint_names", ["base_x", "base_y", "base_theta"])
        self.declare_parameter("publish_rate", 20.0)

        self._joint_names = [str(name) for name in self.get_parameter("joint_names").value]
        if len(self._joint_names) != 3:
            raise ValueError("joint_names must contain exactly ['base_x', 'base_y', 'base_theta'].")
        self._current_positions = [0.0, 0.0, 0.0]
        self._last_stamp = self.get_clock().now().to_msg()

        self._publisher = self.create_publisher(
            JointState,
            str(self.get_parameter("output_topic").value),
            20,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("input_topic").value),
            self._handle_odometry,
            20,
        )
        publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self.create_timer(1.0 / publish_rate, self._publish_joint_state)

        self.get_logger().info(
            "Base pose joint-state bridge ready. "
            f"{self.get_parameter('input_topic').value} -> {self.get_parameter('output_topic').value}"
        )

    def _handle_odometry(self, message: Odometry) -> None:
        pose = message.pose.pose
        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        self._current_positions = [
            float(pose.position.x),
            float(pose.position.y),
            float(yaw),
        ]
        self._last_stamp = message.header.stamp

    def _publish_joint_state(self) -> None:
        joint_state = JointState()
        joint_state.header.stamp = self._last_stamp
        joint_state.name = list(self._joint_names)
        joint_state.position = list(self._current_positions)
        self._publisher.publish(joint_state)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = BasePoseJointStateBridge()
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
