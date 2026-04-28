#!/usr/bin/python3

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class LatchedOccupancyGridRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("medipick_latched_occupancy_grid_republisher")

        self.declare_parameter("input_topic", "/rtabmap/grid_prob_map")
        self.declare_parameter("output_topic", "/medipick/slam_map_latched")
        self.declare_parameter("log_every_n_maps", 20)
        self.declare_parameter("ready_topic", "")
        self.declare_parameter("ready_publish_count", 3)

        self._input_topic = str(self.get_parameter("input_topic").value).strip()
        self._output_topic = str(self.get_parameter("output_topic").value).strip()
        self._log_every_n_maps = max(1, int(self.get_parameter("log_every_n_maps").value))
        self._ready_topic = str(self.get_parameter("ready_topic").value).strip()
        self._ready_publish_count = max(1, int(self.get_parameter("ready_publish_count").value))

        self._latest_map: Optional[OccupancyGrid] = None
        self._map_count = 0
        self._ready_published = False

        input_qos = QoSProfile(depth=1)
        input_qos.reliability = ReliabilityPolicy.RELIABLE
        input_qos.durability = DurabilityPolicy.VOLATILE

        output_qos = QoSProfile(depth=1)
        output_qos.reliability = ReliabilityPolicy.RELIABLE
        output_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._publisher = self.create_publisher(OccupancyGrid, self._output_topic, output_qos)
        self._ready_publisher = (
            self.create_publisher(Bool, self._ready_topic, output_qos) if self._ready_topic else None
        )
        self.create_subscription(OccupancyGrid, self._input_topic, self._on_map, input_qos)

        if self._ready_publisher is not None:
            self._ready_publisher.publish(Bool(data=False))

        self.get_logger().info(
            f"占据栅格缓存重发布器已启动：input={self._input_topic}, output={self._output_topic}"
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg
        self._publisher.publish(msg)
        self._map_count += 1

        if self._ready_publisher is not None and not self._ready_published:
            ready_msg = Bool(data=True)
            for _ in range(self._ready_publish_count):
                self._ready_publisher.publish(ready_msg)
            self._ready_published = True
            self.get_logger().info(f"已发布地图就绪信号：topic={self._ready_topic}")

        if self._map_count == 1 or self._map_count % self._log_every_n_maps == 0:
            resolution = float(msg.info.resolution)
            width = int(msg.info.width)
            height = int(msg.info.height)
            origin = msg.info.origin.position
            known_cells = sum(1 for value in msg.data if value >= 0)
            self.get_logger().info(
                "已缓存并重发布地图："
                f"seq={self._map_count}, size={width}x{height}, resolution={resolution:.3f}, "
                f"known_cells={known_cells}, origin=({origin.x:.2f}, {origin.y:.2f})"
            )


def main() -> None:
    rclpy.init()
    node = LatchedOccupancyGridRepublisher()
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
