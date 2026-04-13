#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMux(Node):
    def __init__(self) -> None:
        super().__init__("medipick_joint_state_mux")

        self.declare_parameter(
            "input_topics",
            [
                "/medipick/hardware/arm_joint_states",
                "/medipick/hardware/base_joint_states",
            ],
        )
        self.declare_parameter("output_topic", "/joint_states")
        self.declare_parameter(
            "ordered_joint_names",
            [
                "base_x",
                "base_y",
                "base_theta",
                "raise_joint",
                "r1_joint",
                "r2_joint",
                "r3_joint",
                "r4_joint",
                "r5_joint",
                "r6_joint",
                "sucker_joint",
                "h1_joint",
                "h2_joint",
            ],
        )
        self.declare_parameter("publish_rate", 30.0)

        self._ordered_joint_names = [str(name) for name in self.get_parameter("ordered_joint_names").value]
        self._joint_positions: dict[str, float] = {}
        self._have_input = False

        self._publisher = self.create_publisher(
            JointState,
            str(self.get_parameter("output_topic").value),
            20,
        )

        input_topics = [str(topic) for topic in self.get_parameter("input_topics").value]
        for topic in input_topics:
            self.create_subscription(
                JointState,
                topic,
                lambda message, source_topic=topic: self._handle_joint_state(message, source_topic),
                20,
            )

        publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self.create_timer(1.0 / publish_rate, self._publish_joint_state)

        self.get_logger().info(
            f"Joint-state mux ready. Inputs={input_topics}, output={self.get_parameter('output_topic').value}"
        )

    def _handle_joint_state(self, message: JointState, _source_topic: str) -> None:
        joint_count = min(len(message.name), len(message.position))
        for index in range(joint_count):
            self._joint_positions[message.name[index]] = float(message.position[index])
        self._have_input = True

    def _publish_joint_state(self) -> None:
        if not self._have_input:
            return

        known_names = [name for name in self._ordered_joint_names if name in self._joint_positions]
        extra_names = sorted(name for name in self._joint_positions if name not in set(known_names))
        joint_names = known_names + extra_names
        if not joint_names:
            return

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = joint_names
        message.position = [self._joint_positions[name] for name in joint_names]
        self._publisher.publish(message)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = JointStateMux()
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
