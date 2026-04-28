#!/usr/bin/python3

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from medicine_memory_store import MedicineObservation, default_db_path, upsert_observation


class MedicineMemoryRegistry(Node):
    def __init__(self) -> None:
        super().__init__("medipick_medicine_memory_registry")

        self.declare_parameter("detection_topic", "/medipick/vision/medicine_detection_json")
        self.declare_parameter("db_path", str(default_db_path()))
        self.declare_parameter("log_success_every_n", 1)

        self._detection_topic = str(self.get_parameter("detection_topic").value).strip()
        self._db_path = Path(str(self.get_parameter("db_path").value)).expanduser()
        self._log_success_every_n = max(1, int(self.get_parameter("log_success_every_n").value))
        self._received_count = 0

        self.create_subscription(String, self._detection_topic, self._on_detection, 50)

        self.get_logger().info(
            "药品记忆库注册节点已启动："
            f" detection_topic={self._detection_topic}, db_path={self._db_path}"
        )

    def _on_detection(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("检测消息必须是 JSON object。")
            observation = MedicineObservation.from_payload(payload)
            record = upsert_observation(self._db_path, observation)
        except Exception as exc:
            self.get_logger().error(f"写入药品记忆库失败：{exc}")
            return

        self._received_count += 1
        if self._received_count % self._log_success_every_n == 0:
            self.get_logger().info(
                "药品记忆库已更新："
                f" medicine_id={record.get('medicine_id', '')},"
                f" location_key={record.get('location_key', '')},"
                f" seen_count={record.get('seen_count', 0)},"
                f" frame={record.get('frame_id', '')},"
                f" position=({float(record.get('position_x', 0.0)):.3f}, "
                f"{float(record.get('position_y', 0.0)):.3f}, "
                f"{float(record.get('position_z', 0.0)):.3f})"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MedicineMemoryRegistry()
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
