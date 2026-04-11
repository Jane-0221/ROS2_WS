#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from medipick_planning_interfaces.srv import GetMedicineLocation, StoreMedicineLocation


@dataclass
class MedicineLocationRecord:
    medicine_id: str
    item_pose_map: PoseStamped
    shelf_face_pose_map: PoseStamped
    confidence: float
    updated_at: TimeMsg
    dock_tag_frame: str


class MedicineLocationRegistry(Node):
    def __init__(self) -> None:
        super().__init__("medipick_medicine_location_registry")

        self.declare_parameter("get_service_name", "/medipick/semantic/get_medicine_location")
        self.declare_parameter("store_service_name", "/medipick/semantic/store_medicine_location")
        self.declare_parameter("seed_file", "")
        self.declare_parameter("autosave_file", "")

        self._records: dict[str, MedicineLocationRecord] = {}
        self._autosave_file = str(self.get_parameter("autosave_file").value).strip()

        seed_file = str(self.get_parameter("seed_file").value).strip()
        if seed_file:
            self._load_seed_file(seed_file)

        self.create_service(
            GetMedicineLocation,
            str(self.get_parameter("get_service_name").value),
            self._handle_get_location,
        )
        self.create_service(
            StoreMedicineLocation,
            str(self.get_parameter("store_service_name").value),
            self._handle_store_location,
        )

        self.get_logger().info(
            "Medicine location registry ready. "
            f"Loaded {len(self._records)} records."
        )

    def _handle_get_location(
        self,
        request: GetMedicineLocation.Request,
        response: GetMedicineLocation.Response,
    ) -> GetMedicineLocation.Response:
        medicine_id = request.medicine_id.strip()
        record = self._records.get(medicine_id)
        if record is None:
            response.success = False
            response.message = f"Medicine '{medicine_id}' is not registered."
            return response

        response.success = True
        response.message = "ok"
        response.item_pose_map = record.item_pose_map
        response.shelf_face_pose_map = record.shelf_face_pose_map
        response.confidence = float(record.confidence)
        response.updated_at = record.updated_at
        response.dock_tag_frame = record.dock_tag_frame
        return response

    def _handle_store_location(
        self,
        request: StoreMedicineLocation.Request,
        response: StoreMedicineLocation.Response,
    ) -> StoreMedicineLocation.Response:
        medicine_id = request.medicine_id.strip()
        if not medicine_id:
            response.success = False
            response.message = "medicine_id must not be empty."
            return response
        if not request.item_pose_map.header.frame_id:
            response.success = False
            response.message = "item_pose_map.header.frame_id must not be empty."
            return response
        if not request.shelf_face_pose_map.header.frame_id:
            response.success = False
            response.message = "shelf_face_pose_map.header.frame_id must not be empty."
            return response

        updated_at = request.updated_at
        if updated_at.sec == 0 and updated_at.nanosec == 0:
            updated_at = self.get_clock().now().to_msg()

        record = MedicineLocationRecord(
            medicine_id=medicine_id,
            item_pose_map=request.item_pose_map,
            shelf_face_pose_map=request.shelf_face_pose_map,
            confidence=float(request.confidence),
            updated_at=updated_at,
            dock_tag_frame=request.dock_tag_frame.strip(),
        )
        self._records[medicine_id] = record
        self._autosave_if_needed()

        response.success = True
        response.message = f"Stored medicine location for '{medicine_id}'."
        self.get_logger().info(response.message)
        return response

    def _load_seed_file(self, seed_file: str) -> None:
        path = Path(seed_file)
        if not path.exists():
            self.get_logger().warning(f"Seed file {seed_file} does not exist; starting empty.")
            return

        with path.open("r", encoding="utf-8") as file:
            data = yaml.safe_load(file) or {}

        for item in data.get("medicines", []):
            medicine_id = str(item.get("medicine_id", "")).strip()
            if not medicine_id:
                continue
            self._records[medicine_id] = MedicineLocationRecord(
                medicine_id=medicine_id,
                item_pose_map=self._pose_from_dict(item.get("item_pose_map", {})),
                shelf_face_pose_map=self._pose_from_dict(item.get("shelf_face_pose_map", {})),
                confidence=float(item.get("confidence", 0.0)),
                updated_at=self._time_from_dict(item.get("updated_at", {})),
                dock_tag_frame=str(item.get("dock_tag_frame", "")).strip(),
            )

    def _autosave_if_needed(self) -> None:
        if not self._autosave_file:
            return

        path = Path(self._autosave_file)
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {"medicines": [self._record_to_dict(record) for record in self._records.values()]}
        with path.open("w", encoding="utf-8") as file:
            yaml.safe_dump(data, file, sort_keys=False)

    @staticmethod
    def _record_to_dict(record: MedicineLocationRecord) -> dict:
        return {
            "medicine_id": record.medicine_id,
            "item_pose_map": MedicineLocationRegistry._pose_to_dict(record.item_pose_map),
            "shelf_face_pose_map": MedicineLocationRegistry._pose_to_dict(record.shelf_face_pose_map),
            "confidence": float(record.confidence),
            "updated_at": {"sec": int(record.updated_at.sec), "nanosec": int(record.updated_at.nanosec)},
            "dock_tag_frame": record.dock_tag_frame,
        }

    @staticmethod
    def _pose_to_dict(message: PoseStamped) -> dict:
        return {
            "frame_id": message.header.frame_id,
            "position": {
                "x": float(message.pose.position.x),
                "y": float(message.pose.position.y),
                "z": float(message.pose.position.z),
            },
            "orientation": {
                "x": float(message.pose.orientation.x),
                "y": float(message.pose.orientation.y),
                "z": float(message.pose.orientation.z),
                "w": float(message.pose.orientation.w),
            },
        }

    @staticmethod
    def _pose_from_dict(data: dict) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = str(data.get("frame_id", "map"))
        position = data.get("position", {})
        orientation = data.get("orientation", {})
        message.pose.position.x = float(position.get("x", 0.0))
        message.pose.position.y = float(position.get("y", 0.0))
        message.pose.position.z = float(position.get("z", 0.0))
        message.pose.orientation.x = float(orientation.get("x", 0.0))
        message.pose.orientation.y = float(orientation.get("y", 0.0))
        message.pose.orientation.z = float(orientation.get("z", 0.0))
        message.pose.orientation.w = float(orientation.get("w", 1.0))
        return message

    @staticmethod
    def _time_from_dict(data: dict) -> TimeMsg:
        message = TimeMsg()
        message.sec = int(data.get("sec", 0))
        message.nanosec = int(data.get("nanosec", 0))
        return message


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MedicineLocationRegistry()
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
