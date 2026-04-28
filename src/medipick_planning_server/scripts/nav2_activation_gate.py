#!/usr/bin/python3

from __future__ import annotations

import time

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class Nav2ActivationGate(Node):
    def __init__(self) -> None:
        super().__init__("medipick_nav2_activation_gate")

        self.declare_parameter("mapping_done_topic", "/medipick/slam_mapping_route_done")
        self.declare_parameter("lifecycle_service", "/lifecycle_manager_navigation/manage_nodes")
        self.declare_parameter("post_mapping_delay_sec", 2.0)
        self.declare_parameter("retry_period_sec", 1.0)
        self.declare_parameter("ready_topic", "")
        self.declare_parameter("ready_publish_count", 3)

        self._mapping_done_topic = str(self.get_parameter("mapping_done_topic").value).strip()
        self._lifecycle_service = str(self.get_parameter("lifecycle_service").value).strip()
        self._post_mapping_delay_sec = max(0.0, float(self.get_parameter("post_mapping_delay_sec").value))
        self._retry_period_sec = max(0.2, float(self.get_parameter("retry_period_sec").value))
        self._ready_topic = str(self.get_parameter("ready_topic").value).strip()
        self._ready_publish_count = max(1, int(self.get_parameter("ready_publish_count").value))

        self._mapping_done = False
        self._mapping_done_at = 0.0
        self._startup_requested = False
        self._startup_completed = False
        self._startup_future = None
        self._last_wait_log = 0.0
        self._last_retry_at = 0.0

        done_qos = QoSProfile(depth=1)
        done_qos.reliability = ReliabilityPolicy.RELIABLE
        done_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Bool, self._mapping_done_topic, self._on_mapping_done, done_qos)
        self._ready_publisher = self.create_publisher(Bool, self._ready_topic, done_qos) if self._ready_topic else None

        self._client = self.create_client(ManageLifecycleNodes, self._lifecycle_service)
        self.create_timer(0.2, self._on_timer)

        if self._ready_publisher is not None:
            self._ready_publisher.publish(Bool(data=False))

        self.get_logger().info(
            "Nav2 激活门已启动："
            f" mapping_done_topic={self._mapping_done_topic},"
            f" lifecycle_service={self._lifecycle_service},"
            f" post_mapping_delay_sec={self._post_mapping_delay_sec:.1f}"
        )

    def _on_mapping_done(self, msg: Bool) -> None:
        if self._mapping_done or not bool(msg.data):
            return
        self._mapping_done = True
        self._mapping_done_at = time.monotonic()
        self.get_logger().info("收到建图完成信号，准备激活 Nav2 生命周期。")

    def _on_timer(self) -> None:
        now = time.monotonic()

        if self._startup_completed:
            return

        if self._startup_future is not None:
            if not self._startup_future.done():
                return
            try:
                response = self._startup_future.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"Nav2 生命周期启动调用失败：{exc!r}")
                self._startup_future = None
                self._startup_requested = False
                self._last_retry_at = now
                return

            if bool(response.success):
                self._startup_completed = True
                if self._ready_publisher is not None:
                    ready_msg = Bool(data=True)
                    for _ in range(self._ready_publish_count):
                        self._ready_publisher.publish(ready_msg)
                self.get_logger().info("Nav2 生命周期已按 mapping_done 触发启动。")
                return

            self.get_logger().warning("Nav2 生命周期启动调用返回 success=false，将继续重试。")
            self._startup_future = None
            self._startup_requested = False
            self._last_retry_at = now
            return

        if not self._mapping_done:
            if now - self._last_wait_log >= 5.0:
                self._last_wait_log = now
                self.get_logger().info("Nav2 激活门等待建图完成信号。")
            return

        if now < self._mapping_done_at + self._post_mapping_delay_sec:
            return

        if not self._client.wait_for_service(timeout_sec=0.1):
            if now - self._last_wait_log >= 5.0:
                self._last_wait_log = now
                self.get_logger().info("Nav2 激活门等待 lifecycle_manager 服务就绪。")
            return

        if self._startup_requested and now - self._last_retry_at < self._retry_period_sec:
            return

        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        self._startup_future = self._client.call_async(request)
        self._startup_requested = True
        self._last_retry_at = now
        self.get_logger().info("Nav2 激活门发起 lifecycle startup 调用。")


def main() -> None:
    rclpy.init()
    node = Nav2ActivationGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
