#!/usr/bin/env python3

import copy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image


def sensor_qos(depth: int = 20) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class ImageStampRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("medipick_image_stamp_republisher")

        self.declare_parameter("input_topic", "/medipick/depth_camera/image")
        self.declare_parameter("output_topic", "/medipick/depth_camera/image_synced")
        self.declare_parameter("stamp_backoff_sec", 0.20)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._stamp_backoff_sec = max(0.0, float(self.get_parameter("stamp_backoff_sec").value))

        self._publisher = self.create_publisher(Image, output_topic, sensor_qos())
        self.create_subscription(Image, input_topic, self._on_image, sensor_qos())
        self._published_once = False

        self.get_logger().info(
            "图像时间戳重发布器已启动："
            f"input={input_topic}, output={output_topic}, stamp_backoff_sec={self._stamp_backoff_sec:.3f}"
        )

    def _on_image(self, msg: Image) -> None:
        synced = copy.deepcopy(msg)
        now = self.get_clock().now()
        backoff_ns = int(self._stamp_backoff_sec * 1e9)
        if backoff_ns > 0 and now.nanoseconds > backoff_ns:
            now = rclpy.time.Time(
                nanoseconds=now.nanoseconds - backoff_ns,
                clock_type=now.clock_type,
            )
        synced.header.stamp = now.to_msg()
        self._publisher.publish(synced)

        if not self._published_once:
            self._published_once = True
            self.get_logger().info(
                f"已开始输出同步后的图像，frame={synced.header.frame_id}"
            )


def main() -> None:
    rclpy.init()
    node = ImageStampRepublisher()
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
