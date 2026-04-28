#!/usr/bin/python3

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from medicine_memory_store import default_db_path, get_best_record


class MedicineMemoryTargetPublisher(Node):
    def __init__(self) -> None:
        super().__init__("medipick_medicine_memory_target_publisher")

        self.declare_parameter("db_path", str(default_db_path()))
        self.declare_parameter("medicine_id", "")
        self.declare_parameter("location_key", "")
        self.declare_parameter("publish_period_sec", 0.5)
        self.declare_parameter("target_pose_topic", "/medipick/task/target_pose")
        self.declare_parameter("target_name_topic", "/medipick/gazebo_target_box/name")
        self.declare_parameter("target_shelf_info_topic", "/medipick/task/target_shelf_info")

        self._db_path = Path(str(self.get_parameter("db_path").value)).expanduser()
        self._medicine_id = str(self.get_parameter("medicine_id").value).strip()
        self._location_key = str(self.get_parameter("location_key").value).strip()
        self._publish_period_sec = max(0.1, float(self.get_parameter("publish_period_sec").value))
        self._target_pose_topic = str(self.get_parameter("target_pose_topic").value).strip()
        self._target_name_topic = str(self.get_parameter("target_name_topic").value).strip()
        self._target_shelf_info_topic = str(self.get_parameter("target_shelf_info_topic").value).strip()

        if not self._medicine_id:
            raise RuntimeError("medicine_id 不能为空。")

        self._target_pose_pub = self.create_publisher(PoseStamped, self._target_pose_topic, 10)
        self._target_name_pub = self.create_publisher(String, self._target_name_topic, 10)
        self._target_shelf_info_pub = self.create_publisher(String, self._target_shelf_info_topic, 10)

        self._last_signature = ""
        self.create_timer(self._publish_period_sec, self._on_timer)

        self.get_logger().info(
            "药品记忆库目标发布器已启动："
            f" medicine_id={self._medicine_id}, location_key={self._location_key or '<latest>'},"
            f" db_path={self._db_path}"
        )

    def _on_timer(self) -> None:
        record = get_best_record(self._db_path, self._medicine_id, self._location_key)
        if not record:
            self.get_logger().warning(
                f"药品记忆库中尚未找到记录：medicine_id={self._medicine_id}, "
                f"location_key={self._location_key or '<latest>'}",
                throttle_duration_sec=5.0,
            )
            return

        pose = PoseStamped()
        pose.header.frame_id = str(record.get("frame_id", "map"))
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(record.get("position_x", 0.0))
        pose.pose.position.y = float(record.get("position_y", 0.0))
        pose.pose.position.z = float(record.get("position_z", 0.0))
        pose.pose.orientation.x = float(record.get("orientation_x", 0.0))
        pose.pose.orientation.y = float(record.get("orientation_y", 0.0))
        pose.pose.orientation.z = float(record.get("orientation_z", 0.0))
        pose.pose.orientation.w = float(record.get("orientation_w", 1.0))
        self._target_pose_pub.publish(pose)

        entity_name = str(record.get("entity_name", "")).strip() or str(record.get("location_key", "")).strip()
        if entity_name:
            self._target_name_pub.publish(String(data=entity_name))

        shelf_info = record.get("shelf_info", {})
        if shelf_info:
            self._target_shelf_info_pub.publish(String(data=json.dumps(shelf_info, ensure_ascii=False, sort_keys=True)))

        signature = "|".join(
            [
                str(record.get("medicine_id", "")),
                str(record.get("location_key", "")),
                str(record.get("entity_name", "")),
                str(record.get("last_seen_at", "")),
            ]
        )
        if signature != self._last_signature:
            self._last_signature = signature
            self.get_logger().info(
                "已从药品记忆库发布目标位姿："
                f" medicine_id={record.get('medicine_id', '')},"
                f" location_key={record.get('location_key', '')},"
                f" entity_name={record.get('entity_name', '') or '<none>'},"
                f" frame={record.get('frame_id', '')},"
                f" position=({float(record.get('position_x', 0.0)):.3f}, "
                f"{float(record.get('position_y', 0.0)):.3f}, "
                f"{float(record.get('position_z', 0.0)):.3f})"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MedicineMemoryTargetPublisher()
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
