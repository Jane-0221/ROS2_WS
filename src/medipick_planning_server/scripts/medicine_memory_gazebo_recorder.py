#!/usr/bin/python3

from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from medicine_memory_store import MedicineObservation, default_db_path, upsert_observation


class MedicineMemoryGazeboRecorder(Node):
    def __init__(self) -> None:
        super().__init__("medipick_medicine_memory_gazebo_recorder")

        self.declare_parameter("db_path", str(default_db_path()))
        self.declare_parameter("medicine_id", "")
        self.declare_parameter("display_name", "")
        self.declare_parameter("location_key", "")
        self.declare_parameter("confidence", 1.0)
        self.declare_parameter("source", "gazebo_target_recorder")
        self.declare_parameter("pose_topic", "/medipick/task/target_pose")
        self.declare_parameter("target_name_topic", "/medipick/gazebo_target_box/name")
        self.declare_parameter("target_shelf_info_topic", "/medipick/task/target_shelf_info")
        self.declare_parameter("exit_after_record", True)

        self._db_path = Path(str(self.get_parameter("db_path").value)).expanduser()
        self._medicine_id = str(self.get_parameter("medicine_id").value).strip()
        self._display_name = str(self.get_parameter("display_name").value).strip()
        self._location_key = str(self.get_parameter("location_key").value).strip()
        self._confidence = float(self.get_parameter("confidence").value)
        self._source = str(self.get_parameter("source").value).strip()
        self._pose_topic = str(self.get_parameter("pose_topic").value).strip()
        self._target_name_topic = str(self.get_parameter("target_name_topic").value).strip()
        self._target_shelf_info_topic = str(self.get_parameter("target_shelf_info_topic").value).strip()
        self._exit_after_record = bool(self.get_parameter("exit_after_record").value)

        self._latest_pose: Optional[PoseStamped] = None
        self._latest_entity_name = ""
        self._latest_shelf_info = ""
        self._recorded = False

        self.create_subscription(PoseStamped, self._pose_topic, self._on_pose, 10)
        self.create_subscription(String, self._target_name_topic, self._on_target_name, 10)
        self.create_subscription(String, self._target_shelf_info_topic, self._on_shelf_info, 10)

        self.get_logger().info(
            "Gazebo 目标写入药品记忆库节点已启动："
            f" pose_topic={self._pose_topic}, target_name_topic={self._target_name_topic},"
            f" db_path={self._db_path}"
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        self._latest_pose = msg
        self._try_record()

    def _on_target_name(self, msg: String) -> None:
        self._latest_entity_name = msg.data.strip()
        self._try_record()

    def _on_shelf_info(self, msg: String) -> None:
        self._latest_shelf_info = msg.data.strip()
        self._try_record()

    def _try_record(self) -> None:
        if self._recorded or self._latest_pose is None:
            return

        entity_name = self._latest_entity_name.strip()
        medicine_id = self._medicine_id or entity_name
        if not medicine_id:
            return

        location_key = self._location_key or entity_name or medicine_id
        shelf_info = {}
        if self._latest_shelf_info:
            try:
                payload = json.loads(self._latest_shelf_info)
                if isinstance(payload, dict):
                    shelf_info = payload
            except json.JSONDecodeError:
                shelf_info = {"raw": self._latest_shelf_info}

        observation = MedicineObservation(
            medicine_id=medicine_id,
            location_key=location_key,
            display_name=self._display_name or medicine_id,
            entity_name=entity_name,
            frame_id=self._latest_pose.header.frame_id or "map",
            position_x=float(self._latest_pose.pose.position.x),
            position_y=float(self._latest_pose.pose.position.y),
            position_z=float(self._latest_pose.pose.position.z),
            orientation_x=float(self._latest_pose.pose.orientation.x),
            orientation_y=float(self._latest_pose.pose.orientation.y),
            orientation_z=float(self._latest_pose.pose.orientation.z),
            orientation_w=float(self._latest_pose.pose.orientation.w),
            confidence=self._confidence,
            source=self._source,
            shelf_info_json=json.dumps(shelf_info, ensure_ascii=False, sort_keys=True) if shelf_info else "",
            extra_json="",
        )
        record = upsert_observation(self._db_path, observation)
        self._recorded = True
        self.get_logger().info(
            "已将 Gazebo 目标写入药品记忆库："
            f" medicine_id={record.get('medicine_id', '')},"
            f" location_key={record.get('location_key', '')},"
            f" entity_name={record.get('entity_name', '')},"
            f" position=({float(record.get('position_x', 0.0)):.3f}, "
            f"{float(record.get('position_y', 0.0)):.3f}, "
            f"{float(record.get('position_z', 0.0)):.3f})"
        )
        if self._exit_after_record:
            self.create_timer(0.2, self._shutdown_once)

    def _shutdown_once(self) -> None:
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MedicineMemoryGazeboRecorder()
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
