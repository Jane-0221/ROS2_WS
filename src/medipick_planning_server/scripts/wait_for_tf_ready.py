#!/usr/bin/env python3

from __future__ import annotations

from typing import Sequence

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class WaitForTfReady(Node):
    def __init__(self) -> None:
        super().__init__("medipick_wait_for_tf_ready")

        self.declare_parameter("source_frame", "base_link")
        self.declare_parameter("required_frames", [""])
        self.declare_parameter("poll_period_sec", 0.2)
        self.declare_parameter("log_interval_sec", 5.0)

        self._source_frame = str(self.get_parameter("source_frame").value)
        self._required_frames = [str(frame) for frame in self.get_parameter("required_frames").value if str(frame)]
        self._poll_period_sec = max(float(self.get_parameter("poll_period_sec").value), 0.05)
        self._log_interval_sec = max(float(self.get_parameter("log_interval_sec").value), self._poll_period_sec)

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self, spin_thread=False)
        self._last_wait_log_time = self.get_clock().now()
        self._ready = False

        if not self._required_frames:
            self.get_logger().warn("No required TF frames configured. Continuing without waiting.")
            self._ready = True
            return

        self.get_logger().info(
            f"Waiting for TF chain from {self._source_frame} to: {', '.join(self._required_frames)}"
        )

    @property
    def ready(self) -> bool:
        return self._ready

    def poll(self) -> None:
        if self._ready:
            return

        missing_frames = [frame for frame in self._required_frames if not self._has_transform(frame)]
        if not missing_frames:
            self._ready = True
            self.get_logger().info(
                f"All required TF frames are ready for {self._source_frame}: {', '.join(self._required_frames)}"
            )
            return

        now = self.get_clock().now()
        if (now - self._last_wait_log_time).nanoseconds >= int(self._log_interval_sec * 1e9):
            self.get_logger().info(
                f"Still waiting for TF frames from {self._source_frame}: {', '.join(missing_frames)}"
            )
            self._last_wait_log_time = now

    def _has_transform(self, frame: str) -> bool:
        try:
            return self._buffer.can_transform(self._source_frame, frame, Time())
        except Exception:
            return False


def main() -> int:
    rclpy.init()
    node = WaitForTfReady()

    try:
        while rclpy.ok() and not node.ready:
            rclpy.spin_once(node, timeout_sec=node._poll_period_sec)
            node.poll()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
