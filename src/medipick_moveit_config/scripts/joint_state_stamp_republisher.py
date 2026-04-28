#!/usr/bin/env python3

import copy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateStampRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("medipick_joint_state_stamp_republisher")

        self.declare_parameter("input_topic", "/joint_states")
        self.declare_parameter("output_topic", "/medipick/joint_states_sim")
        self.declare_parameter("warn_on_large_stamp_gap_sec", 5.0)
        self.declare_parameter("republish_rate_hz", 60.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._warn_on_large_stamp_gap_sec = max(0.0, float(self.get_parameter("warn_on_large_stamp_gap_sec").value))
        self._republish_rate_hz = max(1.0, float(self.get_parameter("republish_rate_hz").value))

        self._publisher = self.create_publisher(JointState, output_topic, 20)
        self.create_subscription(JointState, input_topic, self._on_joint_state, 20)
        self.create_timer(1.0 / self._republish_rate_hz, self._on_timer)
        self._warned_stamp_mismatch = False
        self._published_once = False
        self._latest_msg: JointState | None = None
        self._last_publish_ns = -1

        self.get_logger().info(
            "joint_states 时间戳重发布器已启动："
            f"input={input_topic}, output={output_topic}, republish_rate_hz={self._republish_rate_hz:.1f}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_msg = msg

        if not self._warned_stamp_mismatch and self._warn_on_large_stamp_gap_sec > 0.0:
            input_stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            now = self.get_clock().now().to_msg()
            output_stamp_sec = float(now.sec) + float(now.nanosec) * 1e-9
            if abs(input_stamp_sec - output_stamp_sec) > self._warn_on_large_stamp_gap_sec:
                self._warned_stamp_mismatch = True
                self.get_logger().warning(
                    "检测到 joint_states 时间戳与当前 ROS 时钟差异过大，"
                    f"将强制改写为 sim time：input={input_stamp_sec:.3f}, now={output_stamp_sec:.3f}"
                )

        self._publish_latest()

    def _on_timer(self) -> None:
        self._publish_latest()

    def _publish_latest(self) -> None:
        if self._latest_msg is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= self._last_publish_ns:
            return
        self._last_publish_ns = now_ns

        republished = copy.deepcopy(self._latest_msg)
        now = self.get_clock().now().to_msg()
        republished.header.stamp = now
        self._publisher.publish(republished)

        if not self._published_once:
            self._published_once = True
            self.get_logger().info("已开始输出 sim-time 对齐后的 joint_states。")


def main() -> None:
    rclpy.init()
    node = JointStateStampRepublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        if rclpy.ok() and "Unable to convert call argument" not in str(exc):
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
