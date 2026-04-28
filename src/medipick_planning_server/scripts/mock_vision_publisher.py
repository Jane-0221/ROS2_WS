#!/usr/bin/python3

from __future__ import annotations

from dataclasses import dataclass
import random
import struct
from typing import List, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


PointXYZ = Tuple[float, float, float]


@dataclass(frozen=True)
class BoxSpec:
    center: PointXYZ
    size: PointXYZ


def build_box_surface_points(
    center: PointXYZ,
    size: PointXYZ,
    spacing: float,
) -> List[PointXYZ]:
    cx, cy, cz = center
    sx, sy, sz = size

    x_min = cx - sx / 2.0
    x_max = cx + sx / 2.0
    y_min = cy - sy / 2.0
    y_max = cy + sy / 2.0
    z_min = cz - sz / 2.0
    z_max = cz + sz / 2.0

    xs = frange(x_min, x_max, spacing)
    ys = frange(y_min, y_max, spacing)
    zs = frange(z_min, z_max, spacing)

    points: List[PointXYZ] = []

    for x in xs:
        for y in ys:
            points.append((x, y, z_min))
            points.append((x, y, z_max))

    for x in xs:
        for z in zs:
            points.append((x, y_min, z))
            points.append((x, y_max, z))

    for y in ys:
        for z in zs:
            points.append((x_min, y, z))
            points.append((x_max, y, z))

    return points


def frange(start: float, end: float, step: float) -> List[float]:
    values = []
    current = start
    while current <= end + 1e-9:
        values.append(round(current, 6))
        current += step
    return values


class MockVisionPublisher(Node):
    def __init__(self) -> None:
        super().__init__("medipick_mock_vision_publisher")

        self.declare_parameter("frame_id", "world")
        self.declare_parameter("pointcloud_topic", "/medipick/mock_pointcloud")
        self.declare_parameter("target_pose_topic", "/medipick/mock_target_pose")
        self.declare_parameter("task_target_pose_topic", "/medipick/task/target_pose")
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("point_spacing", 0.025)
        self.declare_parameter("noise_amplitude", 0.0025)
        self.declare_parameter("min_point_z", 0.20)
        self.declare_parameter("random_seed", 42)
        self.declare_parameter("shelf_levels", 5)
        self.declare_parameter("shelf_center_x", 0.88)
        self.declare_parameter("shelf_center_y", 0.0)
        self.declare_parameter("shelf_depth", 0.24)
        self.declare_parameter("shelf_width", 0.72)
        self.declare_parameter("shelf_bottom_z", 0.48)
        self.declare_parameter("shelf_level_gap", 0.24)
        self.declare_parameter("shelf_board_thickness", 0.03)
        self.declare_parameter("shelf_side_thickness", 0.04)
        self.declare_parameter("shelf_back_thickness", 0.03)
        self.declare_parameter("clutter_count", 8)
        self.declare_parameter("target_box_size", [0.08, 0.12, 0.08])
        self.declare_parameter("target_box_size_x", 0.08)
        self.declare_parameter("target_box_size_y", 0.12)
        self.declare_parameter("target_box_size_z", 0.08)
        self.declare_parameter("target_level", -1)
        self.declare_parameter("target_gap_index", -1)
        self.declare_parameter("target_depth_ratio", 0.05)
        self.declare_parameter("target_lateral_margin", 0.10)
        self.declare_parameter("target_lateral_span", 0.06)
        self.declare_parameter("publish_target_box_points", False)
        self.declare_parameter("publish_clutter_points", False)
        self.declare_parameter("publish_floor_points", False)

        self._frame_id = str(self.get_parameter("frame_id").value)
        pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        task_target_pose_topic = str(self.get_parameter("task_target_pose_topic").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self._point_spacing = float(self.get_parameter("point_spacing").value)
        self._noise_amplitude = float(self.get_parameter("noise_amplitude").value)
        self._min_point_z = float(self.get_parameter("min_point_z").value)
        self._rng = random.Random(int(self.get_parameter("random_seed").value))

        self._shelf_levels = max(2, int(self.get_parameter("shelf_levels").value))
        self._shelf_center_x = float(self.get_parameter("shelf_center_x").value)
        self._shelf_center_y = float(self.get_parameter("shelf_center_y").value)
        self._shelf_depth = float(self.get_parameter("shelf_depth").value)
        self._shelf_width = float(self.get_parameter("shelf_width").value)
        self._shelf_bottom_z = float(self.get_parameter("shelf_bottom_z").value)
        self._shelf_level_gap = float(self.get_parameter("shelf_level_gap").value)
        self._shelf_board_thickness = float(self.get_parameter("shelf_board_thickness").value)
        self._shelf_side_thickness = float(self.get_parameter("shelf_side_thickness").value)
        self._shelf_back_thickness = float(self.get_parameter("shelf_back_thickness").value)
        self._clutter_count = max(0, int(self.get_parameter("clutter_count").value))
        self._target_depth_ratio = min(0.95, max(0.05, float(self.get_parameter("target_depth_ratio").value)))
        self._target_lateral_margin = max(0.02, float(self.get_parameter("target_lateral_margin").value))
        self._target_lateral_span = max(0.0, float(self.get_parameter("target_lateral_span").value))
        self._publish_target_box_points = bool(self.get_parameter("publish_target_box_points").value)
        self._publish_clutter_points = bool(self.get_parameter("publish_clutter_points").value)
        self._publish_floor_points = bool(self.get_parameter("publish_floor_points").value)

        target_box_size_values = self.get_parameter("target_box_size").value
        scalar_target_box_size = (
            float(self.get_parameter("target_box_size_x").value),
            float(self.get_parameter("target_box_size_y").value),
            float(self.get_parameter("target_box_size_z").value),
        )
        list_target_box_size = (
            float(target_box_size_values[0]),
            float(target_box_size_values[1]),
            float(target_box_size_values[2]),
        ) if len(target_box_size_values) >= 3 else scalar_target_box_size
        self._target_box_size = scalar_target_box_size if any(
            abs(scalar - default) > 1e-9
            for scalar, default in zip(scalar_target_box_size, (0.08, 0.12, 0.08))
        ) else list_target_box_size
        requested_target_level = int(self.get_parameter("target_level").value)
        if requested_target_level < 0:
            self._target_level = min(self._shelf_levels - 1, self._shelf_levels // 2)
        else:
            self._target_level = max(0, min(self._shelf_levels - 1, requested_target_level))
        requested_gap_index = int(self.get_parameter("target_gap_index").value)
        if requested_gap_index < 0:
            self._target_gap_index = max(0, min(self._shelf_levels - 2, self._shelf_levels // 2))
        else:
            self._target_gap_index = max(0, min(self._shelf_levels - 2, requested_gap_index))

        self._pointcloud_publisher = self.create_publisher(PointCloud2, pointcloud_topic, 10)
        self._target_pose_publisher = self.create_publisher(PoseStamped, target_pose_topic, 10)
        self._task_target_pose_publisher = self.create_publisher(PoseStamped, task_target_pose_topic, 10)

        self._shelf_boxes = self._build_shelf_boxes()
        self._target_box = self._build_target_box()
        self._clutter_boxes = self._build_clutter_boxes()
        self._target_pose = self._build_target_pose()
        self._cloud_points = self._build_scene_cloud()

        self.create_timer(1.0 / max(publish_rate, 0.2), self._publish_scene)
        self.get_logger().info(
            "Mock environment point cloud ready. "
            f"levels={self._shelf_levels}, clutter={self._clutter_count}, gap={self._target_gap_index}, "
            f"center=({self._shelf_center_x:.3f}, {self._shelf_center_y:.3f}), "
            f"size=({self._shelf_depth:.3f}, {self._shelf_width:.3f}), "
            f"boards=({self._shelf_board_thickness:.3f}, {self._shelf_side_thickness:.3f}, {self._shelf_back_thickness:.3f}), "
            f"target_box=({self._target_box_size[0]:.3f}, {self._target_box_size[1]:.3f}, {self._target_box_size[2]:.3f}), "
            f"point cloud={pointcloud_topic}, target pose={target_pose_topic}"
        )

    def _level_z(self, level_index: int) -> float:
        return self._shelf_bottom_z + level_index * self._shelf_level_gap

    def _back_panel_front_x(self) -> float:
        return self._shelf_center_x + self._shelf_depth / 2.0 - self._shelf_back_thickness

    def _build_shelf_boxes(self) -> List[BoxSpec]:
        top_board_z = self._level_z(self._shelf_levels - 1)
        top_board_top_z = top_board_z + self._shelf_board_thickness / 2.0
        bottom_board_bottom_z = self._level_z(0) - self._shelf_board_thickness / 2.0
        shelf_height = top_board_top_z - bottom_board_bottom_z
        shelf_center_z = bottom_board_bottom_z + shelf_height / 2.0
        side_y = self._shelf_center_y + self._shelf_width / 2.0 - self._shelf_side_thickness / 2.0
        side_size = (
            self._shelf_depth,
            self._shelf_side_thickness,
            shelf_height,
        )
        boxes = [
            BoxSpec(center=(self._shelf_center_x, side_y, shelf_center_z), size=side_size),
            BoxSpec(center=(self._shelf_center_x, 2.0 * self._shelf_center_y - side_y, shelf_center_z), size=side_size),
            BoxSpec(
                center=(self._back_panel_front_x() + self._shelf_back_thickness / 2.0, self._shelf_center_y, shelf_center_z),
                size=(self._shelf_back_thickness, self._shelf_width, shelf_height),
            ),
        ]
        for level_index in range(self._shelf_levels):
            boxes.append(
                BoxSpec(
                    center=(self._shelf_center_x, self._shelf_center_y, self._level_z(level_index)),
                    size=(self._shelf_depth, self._shelf_width, self._shelf_board_thickness),
                )
            )
        return boxes

    def _build_target_box(self) -> BoxSpec:
        # The target medicine box sits on one shelf layer and is reachable from the front face.
        box_size = self._target_box_size
        shelf_top_z = self._level_z(self._target_level) + self._shelf_board_thickness / 2.0
        back_panel_front_x = self._back_panel_front_x()
        box_center = (
            back_panel_front_x - box_size[0] / 2.0 - 0.01,
            self._shelf_center_y + min(0.08, 0.18 * self._shelf_width),
            shelf_top_z + box_size[2] / 2.0,
        )
        return BoxSpec(center=box_center, size=box_size)

    def _build_clutter_boxes(self) -> List[BoxSpec]:
        clutter_boxes: List[BoxSpec] = []
        y_limit = self._shelf_width / 2.0 - self._shelf_side_thickness - 0.08
        back_panel_front_x = self._back_panel_front_x()

        for index in range(self._clutter_count):
            level_index = index % self._shelf_levels
            box_size = (
                self._rng.uniform(0.05, 0.10),
                self._rng.uniform(0.06, 0.12),
                self._rng.uniform(0.05, 0.09),
            )
            y = self._rng.uniform(-max(0.05, y_limit), max(0.05, y_limit))
            if level_index == self._target_level and abs(y - self._target_box.center[1]) < 0.16:
                y = -y if abs(-y) <= y_limit else (0.0 if y > 0 else 0.14)

            shelf_top_z = self._level_z(level_index) + self._shelf_board_thickness / 2.0
            center = (
                back_panel_front_x - box_size[0] / 2.0 - self._rng.uniform(0.015, 0.04),
                self._shelf_center_y + y,
                shelf_top_z + box_size[2] / 2.0,
            )
            clutter_boxes.append(BoxSpec(center=center, size=box_size))
        return clutter_boxes

    def _build_target_pose(self) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = self._frame_id
        inner_front_x = self._shelf_center_x - self._shelf_depth / 2.0
        inner_back_x = self._back_panel_front_x() - 0.03
        lower_board_top_z = self._level_z(self._target_gap_index) + self._shelf_board_thickness / 2.0
        upper_board_bottom_z = self._level_z(self._target_gap_index + 1) - self._shelf_board_thickness / 2.0
        lateral_limit = self._shelf_width / 2.0 - self._shelf_side_thickness - self._target_lateral_margin
        target_lateral_span = min(lateral_limit, self._target_lateral_span)

        message.pose.position.x = inner_front_x + self._target_depth_ratio * max(0.02, inner_back_x - inner_front_x)
        message.pose.position.y = self._shelf_center_y + self._rng.uniform(-target_lateral_span, target_lateral_span)
        message.pose.position.z = 0.5 * (lower_board_top_z + upper_board_bottom_z)
        # RViz pose arrows point along local +X. Keep +X pointing from shelf opening to the back panel.
        message.pose.orientation.x = 0.0
        message.pose.orientation.y = 0.0
        message.pose.orientation.z = 0.0
        message.pose.orientation.w = 1.0
        return message

    def _build_scene_cloud(self) -> List[PointXYZ]:
        shelf_points: List[PointXYZ] = []
        for shelf_box in self._shelf_boxes:
            shelf_points.extend(build_box_surface_points(shelf_box.center, shelf_box.size, self._point_spacing))

        target_points: List[PointXYZ] = []
        if self._publish_target_box_points:
            target_points = build_box_surface_points(
                self._target_box.center,
                self._target_box.size,
                max(0.012, self._point_spacing * 0.6),
            )

        clutter_points: List[PointXYZ] = []
        if self._publish_clutter_points:
            for box in self._clutter_boxes:
                clutter_points.extend(
                    build_box_surface_points(
                        box.center,
                        box.size,
                        max(0.015, self._point_spacing * 0.75),
                    )
                )

        floor_points: List[PointXYZ] = []
        if self._publish_floor_points:
            floor_margin = 0.35
            for x in frange(
                self._shelf_center_x - self._shelf_depth - floor_margin,
                self._shelf_center_x + self._shelf_depth + floor_margin,
                0.05,
            ):
                for y in frange(
                    self._shelf_center_y - self._shelf_width / 2.0 - floor_margin,
                    self._shelf_center_y + self._shelf_width / 2.0 + floor_margin,
                    0.05,
                ):
                    floor_points.append((x, y, 0.0))

        # Keep shelf structure intact, and only remove low-lying non-shelf points.
        filtered_non_shelf_points = [
            point for point in (clutter_points + target_points + floor_points) if point[2] >= self._min_point_z
        ]
        cloud = shelf_points + filtered_non_shelf_points
        return [add_noise(point, self._rng, self._noise_amplitude) for point in cloud]

    def _publish_scene(self) -> None:
        now = self.get_clock().now().to_msg()

        target_pose = PoseStamped()
        target_pose.header.stamp = now
        target_pose.header.frame_id = self._target_pose.header.frame_id
        target_pose.pose = self._target_pose.pose
        self._target_pose_publisher.publish(target_pose)
        self._task_target_pose_publisher.publish(target_pose)

        pointcloud = make_pointcloud2(self._frame_id, now, self._cloud_points)
        self._pointcloud_publisher.publish(pointcloud)


def add_noise(point: PointXYZ, rng: random.Random, amplitude: float) -> PointXYZ:
    return (
        point[0] + rng.uniform(-amplitude, amplitude),
        point[1] + rng.uniform(-amplitude, amplitude),
        point[2] + rng.uniform(-amplitude, amplitude),
    )


def make_pointcloud2(frame_id: str, stamp, points: Sequence[PointXYZ]) -> PointCloud2:
    message = PointCloud2()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.point_step * len(points)
    message.is_dense = True
    message.data = b"".join(struct.pack("fff", *point) for point in points)
    return message


def main() -> None:
    rclpy.init()
    node = MockVisionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
