#!/usr/bin/python3

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray


class GazeboDiscreteLiftController(Node):
    def __init__(self) -> None:
        super().__init__("medipick_gazebo_discrete_lift_controller")

        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("target_height_topic", "/medipick/task/lift_target_height")
        self.declare_parameter("command_topic", "/lift_velocity_controller/commands")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("height_tolerance", 0.008)
        self.declare_parameter("slow_zone", 0.04)
        self.declare_parameter("max_up_velocity", 0.10)
        self.declare_parameter("max_down_velocity", 0.10)
        self.declare_parameter("proportional_gain", 3.0)
        self.declare_parameter("publish_zero_when_idle", True)

        self._joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self._target_height_topic = str(self.get_parameter("target_height_topic").value)
        self._command_topic = str(self.get_parameter("command_topic").value)
        self._control_rate_hz = max(5.0, float(self.get_parameter("control_rate_hz").value))
        self._height_tolerance = max(0.002, float(self.get_parameter("height_tolerance").value))
        self._slow_zone = max(self._height_tolerance, float(self.get_parameter("slow_zone").value))
        self._max_up_velocity = max(0.01, float(self.get_parameter("max_up_velocity").value))
        self._max_down_velocity = max(0.01, float(self.get_parameter("max_down_velocity").value))
        self._proportional_gain = max(0.1, float(self.get_parameter("proportional_gain").value))
        self._publish_zero_when_idle = bool(self.get_parameter("publish_zero_when_idle").value)

        self._current_height: float | None = None
        self._target_height: float | None = None
        self._last_command = math.nan

        self.create_subscription(JointState, self._joint_state_topic, self._on_joint_state, 20)
        self.create_subscription(Float64, self._target_height_topic, self._on_target_height, 20)
        self._command_pub = self.create_publisher(Float64MultiArray, self._command_topic, 20)
        self.create_timer(1.0 / self._control_rate_hz, self._on_timer)

        self.get_logger().info(
            "Gazebo 离散升降控制器已启动："
            f" target={self._target_height_topic}, command={self._command_topic}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        if "raise_joint" not in msg.name:
            return
        index = msg.name.index("raise_joint")
        if index >= len(msg.position):
            return
        self._current_height = float(msg.position[index])

    def _on_target_height(self, msg: Float64) -> None:
        self._target_height = float(msg.data)

    def _publish_velocity(self, velocity: float) -> None:
        if not self._publish_zero_when_idle and abs(velocity) < 1e-6:
            return
        if math.isfinite(self._last_command) and abs(self._last_command - velocity) < 1e-5:
            return
        self._command_pub.publish(Float64MultiArray(data=[velocity]))
        if self._current_height is not None and self._target_height is not None:
            self.get_logger().info(
                "离散升降命令: "
                f"current={self._current_height:.4f}, "
                f"target={self._target_height:.4f}, "
                f"command={velocity:.4f}"
            )
        self._last_command = velocity

    def _on_timer(self) -> None:
        if self._current_height is None or self._target_height is None:
            return

        error = self._target_height - self._current_height
        if abs(error) <= self._height_tolerance:
            self._publish_velocity(0.0)
            return

        command = error * self._proportional_gain
        if error > 0.0:
            command = min(command, self._max_up_velocity)
            command = max(command, 0.03)
        else:
            command = max(command, -self._max_down_velocity)
            command = min(command, -0.03)

        if abs(error) < self._slow_zone:
            scale = max(0.25, abs(error) / self._slow_zone)
            command *= scale

        self._publish_velocity(command)


def main() -> None:
    rclpy.init()
    node = GazeboDiscreteLiftController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
