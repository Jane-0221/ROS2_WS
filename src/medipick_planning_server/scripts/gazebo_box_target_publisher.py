#!/usr/bin/python3

from __future__ import annotations

import json
import math
import random
import re
from pathlib import Path
from typing import Dict, List, Tuple
from xml.etree import ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener


def quaternion_from_yaw(yaw: float) -> Quaternion:
    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(yaw / 2.0)
    quat.w = math.cos(yaw / 2.0)
    return quat


def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    quat = Quaternion()
    quat.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y
    quat.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x
    quat.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    quat.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    return quat


def quaternion_conjugate(q: Quaternion) -> Quaternion:
    quat = Quaternion()
    quat.x = -q.x
    quat.y = -q.y
    quat.z = -q.z
    quat.w = q.w
    return quat


def rotate_vector_by_quaternion(q: Quaternion, vector: tuple[float, float, float]) -> tuple[float, float, float]:
    vx, vy, vz = vector
    pure = Quaternion()
    pure.x = vx
    pure.y = vy
    pure.z = vz
    pure.w = 0.0
    rotated = quaternion_multiply(quaternion_multiply(q, pure), quaternion_conjugate(q))
    return rotated.x, rotated.y, rotated.z


def parse_pose_text(pose_text: str) -> Tuple[float, float, float, float, float, float]:
    x, y, z, roll, pitch, yaw = map(float, pose_text.split())
    return x, y, z, roll, pitch, yaw


def clone_pose_stamped(pose: PoseStamped) -> PoseStamped:
    result = PoseStamped()
    result.header = pose.header
    result.pose = pose.pose
    return result


class GazeboBoxTargetPublisher(Node):
    def __init__(self) -> None:
        super().__init__("medipick_gazebo_box_target_publisher")

        default_world = (
            Path(get_package_share_directory("medipick_simple3_description"))
            / "worlds"
            / "medipick_pharmacy_textured.world.sdf"
        )

        self.declare_parameter("world_file", str(default_world))
        self.declare_parameter("target_pose_topic", "/medipick/task/target_pose")
        self.declare_parameter("target_pose_world_topic", "/medipick/gazebo_target_box/pose_world")
        self.declare_parameter("target_pose_camera_topic", "/medipick/gazebo_target_box/pose_camera")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("camera_frame", "cam_link")
        self.declare_parameter("publish_period", 0.5)
        self.declare_parameter("start_delay", 6.0)
        self.declare_parameter("target_seed", 0)
        self.declare_parameter("target_entity_name", "")
        self.declare_parameter("target_contact_inset", 0.002)
        self.declare_parameter("publish_world_pose", True)
        self.declare_parameter("publish_camera_pose", True)
        self.declare_parameter("transform_timeout_sec", 0.10)

        world_file = Path(str(self.get_parameter("world_file").value))
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._world_frame = str(self.get_parameter("world_frame").value)
        self._camera_frame = str(self.get_parameter("camera_frame").value)
        self._publish_period = float(self.get_parameter("publish_period").value)
        self._start_delay = float(self.get_parameter("start_delay").value)
        self._target_seed = int(self.get_parameter("target_seed").value)
        self._target_entity_name = str(self.get_parameter("target_entity_name").value).strip()
        self._target_contact_inset = float(self.get_parameter("target_contact_inset").value)
        self._publish_world_pose = bool(self.get_parameter("publish_world_pose").value)
        self._publish_camera_pose = bool(self.get_parameter("publish_camera_pose").value)
        self._transform_timeout = Duration(seconds=max(0.01, float(self.get_parameter("transform_timeout_sec").value)))
        self._last_tf_warning_time = 0.0

        topic = str(self.get_parameter("target_pose_topic").value)
        world_topic = str(self.get_parameter("target_pose_world_topic").value)
        camera_topic = str(self.get_parameter("target_pose_camera_topic").value)
        self._target_pose_pub = self.create_publisher(PoseStamped, topic, 10)
        self._target_pose_world_pub = self.create_publisher(PoseStamped, world_topic, 10)
        self._target_pose_camera_pub = self.create_publisher(PoseStamped, camera_topic, 10)
        self._target_name_pub = self.create_publisher(String, "/medipick/gazebo_target_box/name", 10)
        self._target_shelf_info_pub = self.create_publisher(String, "/medipick/task/target_shelf_info", 10)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self._start_time = self.get_clock().now()
        self._target_pose_world, self._target_name, self._neighbor_names, self._target_shelf_info = self._select_target_pose(
            world_file
        )
        self._timer = self.create_timer(self._publish_period, self._on_timer)

        self.get_logger().info(
            "Gazebo 药盒目标发布器就绪。"
            f" 目标={self._target_name}, 邻居={self._neighbor_names}, "
            f"task_topic={topic}, world_topic={world_topic}, camera_topic={camera_topic}, "
            f"task_frame={self._frame_id}, world_frame={self._world_frame}, camera_frame={self._camera_frame}"
        )

    def _parse_world(self, world_file: Path):
        root = ET.parse(world_file).getroot()
        includes = []
        for include in root.findall(".//include"):
            name = include.findtext("name", default="").strip()
            uri = include.findtext("uri", default="").strip()
            pose_text = include.findtext("pose", default="0 0 0 0 0 0").strip()
            includes.append(
                {
                    "name": name,
                    "uri": uri,
                    "pose": parse_pose_text(pose_text),
                }
            )
        return includes

    def _extract_level_slot(self, entity_name: str) -> tuple[int, int] | None:
        match = re.search(r"_l(\d+)_s(\d+)$", entity_name)
        if not match:
            return None
        return int(match.group(1)), int(match.group(2))

    def _load_model_size(self, model_name: str) -> tuple[float, float, float]:
        sdf_path = (
            Path(get_package_share_directory("medipick_simple3_description"))
            / "models"
            / model_name
            / "model.sdf"
        )
        text = sdf_path.read_text(encoding="utf-8")
        match = re.search(r"<size>([0-9.eE+-]+)\s+([0-9.eE+-]+)\s+([0-9.eE+-]+)</size>", text)
        if not match:
            raise RuntimeError(f"无法从 {sdf_path} 读取药盒尺寸")
        return float(match.group(1)), float(match.group(2)), float(match.group(3))

    def _load_model_bounds(self, model_name: str) -> dict[str, float]:
        sdf_path = (
            Path(get_package_share_directory("medipick_simple3_description"))
            / "models"
            / model_name
            / "model.sdf"
        )
        root = ET.parse(sdf_path).getroot()
        min_x = math.inf
        min_y = math.inf
        min_z = math.inf
        max_x = -math.inf
        max_y = -math.inf
        max_z = -math.inf

        for collision in root.findall(".//collision"):
            size_text = collision.findtext("./geometry/box/size", default="").strip()
            if not size_text:
                continue
            sx, sy, sz = map(float, size_text.split())
            pose_text = collision.findtext("pose", default="0 0 0 0 0 0").strip()
            px, py, pz, _rr, _rp, _ry = parse_pose_text(pose_text)
            half_x = sx * 0.5
            half_y = sy * 0.5
            half_z = sz * 0.5
            min_x = min(min_x, px - half_x)
            min_y = min(min_y, py - half_y)
            min_z = min(min_z, pz - half_z)
            max_x = max(max_x, px + half_x)
            max_y = max(max_y, py + half_y)
            max_z = max(max_z, pz + half_z)

        if not math.isfinite(min_x):
            raise RuntimeError(f"无法从 {sdf_path} 读取模型碰撞包围盒")

        return {
            "min_x": min_x,
            "min_y": min_y,
            "min_z": min_z,
            "max_x": max_x,
            "max_y": max_y,
            "max_z": max_z,
            "depth": max_x - min_x,
            "width": max_y - min_y,
            "height": max_z - min_z,
        }

    def _select_target_pose(self, world_file: Path) -> tuple[PoseStamped, str, list[str], dict]:
        includes = self._parse_world(world_file)
        shelf_map: Dict[str, dict] = {}
        drug_entities: List[dict] = []

        for item in includes:
            uri = item["uri"]
            if uri.startswith("model://textured_drug_"):
                drug_entities.append(item)
            elif uri.startswith("model://") and not uri.startswith("model://medipick"):
                shelf_map[item["name"]] = item

        if not drug_entities:
            raise RuntimeError("当前 world 中没有找到可作为目标的独立药盒实体")

        if self._target_entity_name:
            matches = [item for item in drug_entities if item["name"] == self._target_entity_name]
            if not matches:
                raise RuntimeError(f"指定目标药盒不存在：{self._target_entity_name}")
            selected = matches[0]
        else:
            rng = random.Random(self._target_seed)
            selected = rng.choice(drug_entities)

        entity_name = selected["name"]
        model_name = selected["uri"].replace("model://", "", 1)
        x, y, z, _roll, _pitch, _entity_yaw = selected["pose"]
        shelf_name = re.sub(r"_l\d+_s\d+$", "", entity_name)
        if shelf_name not in shelf_map:
            raise RuntimeError(f"找不到目标药盒对应的货架：{shelf_name}")
        shelf_model_name = shelf_map[shelf_name]["uri"].replace("model://", "", 1)
        sx, sy, sz, _sroll, _spitch, shelf_yaw = shelf_map[shelf_name]["pose"]

        _width, depth, _height = self._load_model_size(model_name)
        inward_x = math.cos(shelf_yaw)
        inward_y = math.sin(shelf_yaw)
        target_x = x - inward_x * max(0.0, depth / 2.0 - self._target_contact_inset)
        target_y = y - inward_y * max(0.0, depth / 2.0 - self._target_contact_inset)

        target_pose = PoseStamped()
        target_pose.header.frame_id = self._world_frame
        target_pose.pose.position.x = target_x
        target_pose.pose.position.y = target_y
        target_pose.pose.position.z = z
        target_pose.pose.orientation = quaternion_from_yaw(shelf_yaw)

        selected_level_slot = self._extract_level_slot(entity_name)
        neighbor_names: list[str] = []
        if selected_level_slot is not None:
            level_index, slot_index = selected_level_slot
            for item in drug_entities:
                candidate = self._extract_level_slot(item["name"])
                if candidate is None:
                    continue
                candidate_level, candidate_slot = candidate
                candidate_shelf = re.sub(r"_l\d+_s\d+$", "", item["name"])
                if candidate_shelf != shelf_name:
                    continue
                if abs(candidate_level - level_index) <= 1 and abs(candidate_slot - slot_index) <= 1:
                    neighbor_names.append(item["name"])

        shelf_bounds = self._load_model_bounds(shelf_model_name)
        local_center_x = 0.5 * (shelf_bounds["min_x"] + shelf_bounds["max_x"])
        local_center_y = 0.5 * (shelf_bounds["min_y"] + shelf_bounds["max_y"])
        local_entry_x = shelf_bounds["min_x"]
        cos_yaw = math.cos(shelf_yaw)
        sin_yaw = math.sin(shelf_yaw)

        center_world_x = sx + cos_yaw * local_center_x - sin_yaw * local_center_y
        center_world_y = sy + sin_yaw * local_center_x + cos_yaw * local_center_y
        entry_world_x = sx + cos_yaw * local_entry_x - sin_yaw * local_center_y
        entry_world_y = sy + sin_yaw * local_entry_x + cos_yaw * local_center_y

        shelf_info = {
            "shelf_name": shelf_name,
            "shelf_model_name": shelf_model_name,
            "shelf_origin_x": sx,
            "shelf_origin_y": sy,
            "shelf_origin_z": sz,
            "shelf_center_x": center_world_x,
            "shelf_center_y": center_world_y,
            "shelf_entry_x": entry_world_x,
            "shelf_entry_y": entry_world_y,
            "shelf_yaw": shelf_yaw,
            "shelf_inward_axis_x": inward_x,
            "shelf_inward_axis_y": inward_y,
            "shelf_lateral_axis_x": -inward_y,
            "shelf_lateral_axis_y": inward_x,
            "shelf_depth": shelf_bounds["depth"],
            "shelf_width": shelf_bounds["width"],
            "shelf_height": shelf_bounds["height"],
            "target_name": entity_name,
            "target_contact_x": target_x,
            "target_contact_y": target_y,
            "target_contact_z": z,
        }

        return target_pose, entity_name, sorted(neighbor_names), shelf_info

    def _transform_pose(self, pose: PoseStamped, target_frame: str) -> PoseStamped | None:
        if target_frame == pose.header.frame_id:
            result = clone_pose_stamped(pose)
            result.header.stamp = self.get_clock().now().to_msg()
            return result

        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                pose.header.frame_id,
                rclpy.time.Time(),
                timeout=self._transform_timeout,
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec - self._last_tf_warning_time > 2.0:
                self._last_tf_warning_time = now_sec
                message = str(exc)
                if target_frame == "map" and "target_frame does not exist" in message:
                    self.get_logger().info("RTAB-Map 的 map 坐标系还没建立出来，目标发布器继续等待。")
                else:
                    self.get_logger().warning(
                        f"目标位姿 TF 变换失败：{pose.header.frame_id} -> {target_frame}，原因：{exc}"
                    )
            return None

        transformed = PoseStamped()
        transformed.header.stamp = self.get_clock().now().to_msg()
        transformed.header.frame_id = target_frame
        rotated = rotate_vector_by_quaternion(
            transform.transform.rotation,
            (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ),
        )
        transformed.pose.position.x = transform.transform.translation.x + rotated[0]
        transformed.pose.position.y = transform.transform.translation.y + rotated[1]
        transformed.pose.position.z = transform.transform.translation.z + rotated[2]
        transformed.pose.orientation = quaternion_multiply(transform.transform.rotation, pose.pose.orientation)
        return transformed

    def _on_timer(self) -> None:
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        if elapsed < self._start_delay:
            return

        world_pose = clone_pose_stamped(self._target_pose_world)
        world_pose.header.stamp = self.get_clock().now().to_msg()

        if self._publish_world_pose:
            self._target_pose_world_pub.publish(world_pose)

        task_pose = self._transform_pose(world_pose, self._frame_id)
        if task_pose is not None:
            self._target_pose_pub.publish(task_pose)

        if self._publish_camera_pose:
            camera_pose = self._transform_pose(world_pose, self._camera_frame)
            if camera_pose is not None:
                self._target_pose_camera_pub.publish(camera_pose)

        name_msg = String()
        name_msg.data = self._target_name
        self._target_name_pub.publish(name_msg)

        shelf_info_msg = String()
        shelf_info_msg.data = json.dumps(self._target_shelf_info, ensure_ascii=False)
        self._target_shelf_info_pub.publish(shelf_info_msg)


def main() -> None:
    rclpy.init()
    node = GazeboBoxTargetPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
