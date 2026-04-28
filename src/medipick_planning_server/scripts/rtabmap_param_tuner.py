#!/usr/bin/python3

from __future__ import annotations

import time

import rclpy
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class RtabmapParamTuner(Node):
    def __init__(self) -> None:
        super().__init__("medipick_rtabmap_param_tuner")

        self.declare_parameter("target_node", "/rtabmap/rtabmap")
        self.declare_parameter("tf_delay", 0.25)
        self.declare_parameter("tf_tolerance", 0.30)
        self.declare_parameter("wait_timeout_sec", 20.0)
        self.declare_parameter("poll_period_sec", 0.5)
        self.declare_parameter("ready_topic", "/medipick/rtabmap_ready")
        self.declare_parameter("ready_publish_count", 10)
        self.declare_parameter("ready_publish_interval_sec", 0.2)

        self._target_node = str(self.get_parameter("target_node").value).strip()
        self._tf_delay = float(self.get_parameter("tf_delay").value)
        self._tf_tolerance = float(self.get_parameter("tf_tolerance").value)
        self._wait_timeout_sec = max(1.0, float(self.get_parameter("wait_timeout_sec").value))
        self._poll_period_sec = max(0.1, float(self.get_parameter("poll_period_sec").value))
        self._ready_topic = str(self.get_parameter("ready_topic").value).strip()
        self._ready_publish_count = max(1, int(self.get_parameter("ready_publish_count").value))
        self._ready_publish_interval_sec = max(
            0.05, float(self.get_parameter("ready_publish_interval_sec").value)
        )

        service_prefix = self._target_node.rstrip("/")
        self._set_parameters_client = self.create_client(
            SetParameters,
            f"{service_prefix}/set_parameters",
        )
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._ready_publisher = self.create_publisher(Bool, self._ready_topic, qos)
        self._started_at = time.monotonic()
        self._done = False
        self._request_in_flight = False
        self._last_wait_log = 0.0
        self._timer = self.create_timer(self._poll_period_sec, self._tick)

    def done(self) -> bool:
        return self._done

    def _tick(self) -> None:
        if self._done:
            return

        elapsed = time.monotonic() - self._started_at
        if elapsed > self._wait_timeout_sec:
            self.get_logger().error(
                f"等待 RTAB-Map 参数服务超时：target={self._target_node}, elapsed={elapsed:.1f}s"
            )
            self._done = True
            return

        if self._request_in_flight:
            return

        if not self._set_parameters_client.service_is_ready():
            if time.monotonic() - self._last_wait_log >= 2.0:
                self._last_wait_log = time.monotonic()
                self.get_logger().info(f"等待 RTAB-Map 参数服务：{self._target_node}")
            self._set_parameters_client.wait_for_service(timeout_sec=0.1)
            return

        self._request_in_flight = True
        request = SetParameters.Request()
        request.parameters = [
            Parameter("tf_delay", Parameter.Type.DOUBLE, self._tf_delay).to_parameter_msg(),
            Parameter("tf_tolerance", Parameter.Type.DOUBLE, self._tf_tolerance).to_parameter_msg(),
        ]
        future = self._set_parameters_client.call_async(request)
        future.add_done_callback(self._on_set_done)

    def _on_set_done(self, future) -> None:
        self._request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"设置 RTAB-Map TF 参数失败：{exc}")
            self._done = True
            return

        results = [] if response is None else response.results
        if results is None or len(results) == 0:
            self.get_logger().error("RTAB-Map 参数设置返回空结果。")
            self._done = True
            return

        failures = [result.reason for result in results if not result.successful]
        if failures:
            self.get_logger().error("RTAB-Map 参数设置失败：" + "; ".join(reason or "<no reason>" for reason in failures))
            self._done = True
            return

        self.get_logger().info(
            f"已设置 RTAB-Map TF 参数：target={self._target_node}, "
            f"tf_delay={self._tf_delay:.3f}, tf_tolerance={self._tf_tolerance:.3f}"
        )
        ready_msg = Bool(data=True)
        for publish_index in range(self._ready_publish_count):
            self._ready_publisher.publish(ready_msg)
            if publish_index == 0:
                self.get_logger().info(f"已开始发布 RTAB-Map 就绪信号：topic={self._ready_topic}")
            time.sleep(self._ready_publish_interval_sec)
        self.get_logger().info(
            f"已完成 RTAB-Map 就绪信号发布：topic={self._ready_topic}, "
            f"count={self._ready_publish_count}, interval={self._ready_publish_interval_sec:.2f}s"
        )
        self._done = True


def main() -> None:
    rclpy.init()
    node = RtabmapParamTuner()
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
