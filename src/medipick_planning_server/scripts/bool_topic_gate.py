#!/usr/bin/python3

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class BoolTopicGate(Node):
    def __init__(self) -> None:
        super().__init__("medipick_bool_topic_gate")

        self.declare_parameter("topic", "/medipick/controllers_ready")
        self.declare_parameter("required_value", True)
        self.declare_parameter("log_period_sec", 5.0)
        self.declare_parameter("durability_transient_local", False)

        self._topic = str(self.get_parameter("topic").value).strip()
        self._required_value = bool(self.get_parameter("required_value").value)
        self._log_period_sec = max(0.5, float(self.get_parameter("log_period_sec").value))
        self._durability_transient_local = bool(
            self.get_parameter("durability_transient_local").value
        )
        self._last_log_time = self.get_clock().now()
        self._matched = False
        self._should_exit = False

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = (
            DurabilityPolicy.TRANSIENT_LOCAL
            if self._durability_transient_local
            else DurabilityPolicy.VOLATILE
        )
        self.create_subscription(Bool, self._topic, self._on_bool, qos)
        self.create_timer(0.2, self._on_timer)

        self.get_logger().info(
            f"Bool gate 已启动：topic={self._topic}, required_value={self._required_value}, "
            f"transient_local={self._durability_transient_local}"
        )

    def _on_bool(self, msg: Bool) -> None:
        if bool(msg.data) != self._required_value:
            return
        if self._matched:
            return
        self._matched = True
        self._should_exit = True
        self.get_logger().info(f"Bool gate 收到满足条件的消息，准备退出：topic={self._topic}")

    def _on_timer(self) -> None:
        if self._matched:
            return
        now = self.get_clock().now()
        if (now - self._last_log_time).nanoseconds >= int(self._log_period_sec * 1e9):
            self._last_log_time = now
            self.get_logger().info(f"Bool gate 等待条件消息：topic={self._topic}")

    @property
    def should_exit(self) -> bool:
        return self._should_exit


def main() -> None:
    rclpy.init()
    node = BoolTopicGate()
    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
