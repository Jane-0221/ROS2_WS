#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class PumpCommandAdapter(Node):
    def __init__(self) -> None:
        super().__init__("medipick_pump_command_adapter")

        self.declare_parameter("input_command_topic", "/medipick/task/pump_command")
        self.declare_parameter("output_command_topic", "stm32/pump_control")
        self.declare_parameter("raw_suction_topic", "/medipick/hardware/raw_suction_state")
        self.declare_parameter("suction_topic", "/medipick/hardware/suction_state")

        self._pump_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("output_command_topic").value),
            10,
        )
        self._suction_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("suction_topic").value),
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("input_command_topic").value),
            self._on_pump_command,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("raw_suction_topic").value),
            self._on_suction_state,
            10,
        )
        self.get_logger().info("Pump command adapter ready.")

    def _on_pump_command(self, msg: Bool) -> None:
        self._pump_publisher.publish(Bool(data=bool(msg.data)))

    def _on_suction_state(self, msg: Bool) -> None:
        self._suction_publisher.publish(Bool(data=bool(msg.data)))


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PumpCommandAdapter()
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
