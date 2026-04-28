#!/usr/bin/python3

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


@dataclass
class Footprint:
    min_x: float
    max_x: float
    min_y: float
    max_y: float


@dataclass
class WorldObstacle:
    name: str
    x: float
    y: float
    yaw: float
    footprint: Footprint


def parse_pose_text(pose_text: str) -> tuple[float, float, float, float, float, float]:
    x, y, z, roll, pitch, yaw = map(float, pose_text.split())
    return x, y, z, roll, pitch, yaw


class GazeboMapPublisher(Node):
    def __init__(self) -> None:
        super().__init__("medipick_gazebo_map_publisher")

        default_world = (
            Path(get_package_share_directory("medipick_simple3_description"))
            / "worlds"
            / "medipick_pharmacy_textured.world.sdf"
        )

        self.declare_parameter("world_file", str(default_world))
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("padding", 0.60)
        self.declare_parameter("publish_period", 1.0)

        self._world_file = Path(str(self.get_parameter("world_file").value))
        self._map_topic = str(self.get_parameter("map_topic").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._resolution = max(0.02, float(self.get_parameter("resolution").value))
        self._padding = max(0.20, float(self.get_parameter("padding").value))
        self._publish_period = max(0.2, float(self.get_parameter("publish_period").value))

        self._footprint_cache: dict[str, Footprint] = {}
        self._map_msg = self._build_map()
        self._map_pub = self.create_publisher(OccupancyGrid, self._map_topic, 10)
        self.create_timer(self._publish_period, self._on_timer)

        self.get_logger().info(
            "Gazebo 二维地图发布器已启动："
            f" topic={self._map_topic}, frame={self._map_frame}, "
            f"size={self._map_msg.info.width}x{self._map_msg.info.height}, resolution={self._resolution:.3f}"
        )

    def _parse_world_includes(self) -> list[dict]:
        root = ET.parse(self._world_file).getroot()
        result: list[dict] = []
        for include in root.findall(".//include"):
            name = (include.findtext("name") or "").strip()
            uri = (include.findtext("uri") or "").strip()
            if not uri.startswith("model://"):
                continue
            pose = parse_pose_text((include.findtext("pose") or "0 0 0 0 0 0").strip())
            result.append({"name": name, "uri": uri.replace("model://", "", 1), "pose": pose})
        return result

    def _load_model_footprint(self, model_name: str) -> Footprint:
        if model_name in self._footprint_cache:
            return self._footprint_cache[model_name]

        sdf_path = (
            Path(get_package_share_directory("medipick_simple3_description"))
            / "models"
            / model_name
            / "model.sdf"
        )
        root = ET.parse(sdf_path).getroot()

        min_x = math.inf
        max_x = -math.inf
        min_y = math.inf
        max_y = -math.inf
        for collision in root.findall(".//collision"):
            pose_text = (collision.findtext("pose") or "0 0 0 0 0 0").strip()
            cx, cy, _cz, _roll, _pitch, cyaw = parse_pose_text(pose_text)
            box = collision.find("./geometry/box/size")
            if box is None or box.text is None:
                continue
            size_x, size_y, _size_z = map(float, box.text.split())
            half_x = size_x / 2.0
            half_y = size_y / 2.0
            corners = [
                (+half_x, +half_y),
                (+half_x, -half_y),
                (-half_x, +half_y),
                (-half_x, -half_y),
            ]
            cos_yaw = math.cos(cyaw)
            sin_yaw = math.sin(cyaw)
            for local_x, local_y in corners:
                rot_x = cx + local_x * cos_yaw - local_y * sin_yaw
                rot_y = cy + local_x * sin_yaw + local_y * cos_yaw
                min_x = min(min_x, rot_x)
                max_x = max(max_x, rot_x)
                min_y = min(min_y, rot_y)
                max_y = max(max_y, rot_y)

        if not math.isfinite(min_x):
            raise RuntimeError(f"无法从 {sdf_path} 解析模型 footprint")

        footprint = Footprint(min_x=min_x, max_x=max_x, min_y=min_y, max_y=max_y)
        self._footprint_cache[model_name] = footprint
        return footprint

    def _build_obstacles(self) -> list[WorldObstacle]:
        obstacle_models = {
            "pharmacy_shelf",
            "pharmacy_shelf_dense",
            "pharmacy_shelf_compact",
            "pharmacy_endcap_display",
        }
        obstacles: list[WorldObstacle] = []
        for include in self._parse_world_includes():
            if include["uri"] not in obstacle_models:
                continue
            x, y, _z, _roll, _pitch, yaw = include["pose"]
            obstacles.append(
                WorldObstacle(
                    name=include["name"],
                    x=x,
                    y=y,
                    yaw=yaw,
                    footprint=self._load_model_footprint(include["uri"]),
                )
            )
        return obstacles

    def _point_inside_obstacle(self, x: float, y: float, obstacle: WorldObstacle) -> bool:
        dx = x - obstacle.x
        dy = y - obstacle.y
        cos_yaw = math.cos(-obstacle.yaw)
        sin_yaw = math.sin(-obstacle.yaw)
        local_x = dx * cos_yaw - dy * sin_yaw
        local_y = dx * sin_yaw + dy * cos_yaw
        return (
            obstacle.footprint.min_x <= local_x <= obstacle.footprint.max_x
            and obstacle.footprint.min_y <= local_y <= obstacle.footprint.max_y
        )

    def _build_map(self) -> OccupancyGrid:
        obstacles = self._build_obstacles()
        if not obstacles:
            raise RuntimeError("地图生成失败：没有从 world 中找到货架或端头架")

        world_min_x = min(item.x + item.footprint.min_x for item in obstacles) - self._padding
        world_max_x = max(item.x + item.footprint.max_x for item in obstacles) + self._padding
        world_min_y = min(item.y + item.footprint.min_y for item in obstacles) - self._padding
        world_max_y = max(item.y + item.footprint.max_y for item in obstacles) + self._padding

        width = int(math.ceil((world_max_x - world_min_x) / self._resolution))
        height = int(math.ceil((world_max_y - world_min_y) / self._resolution))
        data = [0] * (width * height)

        for row in range(height):
            cell_y = world_min_y + (row + 0.5) * self._resolution
            for col in range(width):
                cell_x = world_min_x + (col + 0.5) * self._resolution
                occupied = False
                for obstacle in obstacles:
                    if self._point_inside_obstacle(cell_x, cell_y, obstacle):
                        occupied = True
                        break
                if occupied:
                    data[row * width + col] = 100

        msg = OccupancyGrid()
        msg.header.frame_id = self._map_frame
        msg.info.resolution = self._resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = world_min_x
        msg.info.origin.position.y = world_min_y
        msg.info.origin.orientation.w = 1.0
        msg.data = data
        return msg

    def _on_timer(self) -> None:
        self._map_msg.header.stamp = self.get_clock().now().to_msg()
        self._map_pub.publish(self._map_msg)


def main() -> None:
    rclpy.init()
    node = GazeboMapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
