#!/usr/bin/env python3

from __future__ import annotations

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener


class ImuReadyRelay(Node):
    def __init__(self) -> None:
        super().__init__("medipick_imu_ready_relay")

        self.declare_parameter("input_topic", "/camera/imu/data")
        self.declare_parameter("output_topic", "/camera/imu/ready")
        self.declare_parameter("source_frame", "base_link")
        self.declare_parameter("release_delay_sec", 1.0)
        self.declare_parameter("log_interval_sec", 5.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._source_frame = str(self.get_parameter("source_frame").value)
        self._release_delay = Duration(seconds=max(float(self.get_parameter("release_delay_sec").value), 0.0))
        self._log_interval_sec = max(float(self.get_parameter("log_interval_sec").value), 0.5)

        self._publisher = self.create_publisher(Imu, output_topic, qos_profile_sensor_data)
        self._subscription = self.create_subscription(
            Imu,
            input_topic,
            self._handle_imu,
            qos_profile_sensor_data,
        )
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self, spin_thread=False)
        self._ready_since = None
        self._released = False
        self._last_wait_log_time = self.get_clock().now()

        self.get_logger().info(
            f"IMU relay waiting for TF before forwarding {input_topic} -> {output_topic} from {self._source_frame}"
        )

    def _handle_imu(self, message: Imu) -> None:
        if self._released:
            self._publisher.publish(message)
            return

        frame_id = message.header.frame_id
        if not frame_id:
            self._log_wait("incoming IMU frame_id is empty")
            return

        if not self._buffer.can_transform(self._source_frame, frame_id, Time()):
            self._ready_since = None
            self._log_wait(f"waiting for TF {self._source_frame} -> {frame_id}")
            return

        now = self.get_clock().now()
        if self._ready_since is None:
            self._ready_since = now
            self.get_logger().info(
                f"IMU TF is ready for {self._source_frame} -> {frame_id}, holding for "
                f"{self._release_delay.nanoseconds / 1e9:.1f}s before forwarding"
            )
            return

        if now - self._ready_since < self._release_delay:
            return

        self._released = True
        self.get_logger().info(f"IMU relay is now forwarding messages from {frame_id}")
        self._publisher.publish(message)

    def _log_wait(self, detail: str) -> None:
        now = self.get_clock().now()
        if (now - self._last_wait_log_time).nanoseconds >= int(self._log_interval_sec * 1e9):
            self.get_logger().info(detail)
            self._last_wait_log_time = now


def main() -> int:
    rclpy.init()
    node = ImuReadyRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
