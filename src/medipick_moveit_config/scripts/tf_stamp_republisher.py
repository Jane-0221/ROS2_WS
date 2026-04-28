#!/usr/bin/env python3

import copy

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def dynamic_tf_qos(depth: int = 100) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def static_tf_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class TFStampRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("medipick_tf_stamp_republisher")

        self.declare_parameter("input_tf_topic", "/medipick/raw_tf")
        self.declare_parameter("input_tf_static_topic", "/medipick/raw_tf_static")
        self.declare_parameter("output_tf_topic", "/tf")
        self.declare_parameter("output_tf_static_topic", "/tf_static")
        self.declare_parameter("dynamic_republish_rate_hz", 60.0)

        input_tf_topic = str(self.get_parameter("input_tf_topic").value)
        input_tf_static_topic = str(self.get_parameter("input_tf_static_topic").value)
        output_tf_topic = str(self.get_parameter("output_tf_topic").value)
        output_tf_static_topic = str(self.get_parameter("output_tf_static_topic").value)
        self._dynamic_republish_rate_hz = max(1.0, float(self.get_parameter("dynamic_republish_rate_hz").value))

        self._tf_pub = self.create_publisher(TFMessage, output_tf_topic, dynamic_tf_qos())
        self._tf_static_pub = self.create_publisher(TFMessage, output_tf_static_topic, static_tf_qos())

        self.create_subscription(TFMessage, input_tf_topic, self._on_dynamic_tf, dynamic_tf_qos())
        self.create_subscription(TFMessage, input_tf_static_topic, self._on_static_tf, static_tf_qos())
        self.create_timer(1.0 / self._dynamic_republish_rate_hz, self._on_dynamic_timer)

        self._logged_dynamic = False
        self._logged_static = False
        self._dynamic_cache: dict[tuple[str, str], TransformStamped] = {}
        self._last_dynamic_publish_ns = -1

        self.get_logger().info(
            "TF 时间戳重发布器已启动："
            f" tf={input_tf_topic}->{output_tf_topic}, "
            f"tf_static={input_tf_static_topic}->{output_tf_static_topic}, "
            f"dynamic_republish_rate_hz={self._dynamic_republish_rate_hz:.1f}"
        )

    def _restamp(self, message: TFMessage, stamp_msg) -> TFMessage:
        restamped = copy.deepcopy(message)
        for transform in restamped.transforms:
            transform.header.stamp = stamp_msg
        return restamped

    def _on_dynamic_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            key = (transform.header.frame_id, transform.child_frame_id)
            self._dynamic_cache[key] = copy.deepcopy(transform)
        self._publish_dynamic_cache()
        if not self._logged_dynamic:
            self._logged_dynamic = True
            self.get_logger().info("已开始输出 sim-time 对齐后的 /tf。")

    def _on_dynamic_timer(self) -> None:
        self._publish_dynamic_cache()

    def _publish_dynamic_cache(self) -> None:
        if not self._dynamic_cache:
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= self._last_dynamic_publish_ns:
            return
        self._last_dynamic_publish_ns = now_ns
        message = TFMessage(transforms=[copy.deepcopy(transform) for transform in self._dynamic_cache.values()])
        self._tf_pub.publish(self._restamp(message, self.get_clock().now().to_msg()))

    def _on_static_tf(self, message: TFMessage) -> None:
        self._tf_static_pub.publish(self._restamp(message, self.get_clock().now().to_msg()))
        if not self._logged_static:
            self._logged_static = True
            self.get_logger().info("已开始输出 sim-time 对齐后的 /tf_static。")


def main() -> None:
    rclpy.init()
    node = TFStampRepublisher()
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
