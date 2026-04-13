#!/usr/bin/env python3

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class Anchor:
    frame_id: str
    anchor_type: str
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]


def normalize_quaternion(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def quat_multiply(
    lhs: tuple[float, float, float, float],
    rhs: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quat_conjugate(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def rotate_vector(
    vector: tuple[float, float, float],
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    pure = (vector[0], vector[1], vector[2], 0.0)
    rotated = quat_multiply(quat_multiply(quaternion, pure), quat_conjugate(quaternion))
    return (rotated[0], rotated[1], rotated[2])


def inverse_transform(
    translation: tuple[float, float, float],
    rotation: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    inv_rotation = quat_conjugate(rotation)
    rotated = rotate_vector((-translation[0], -translation[1], -translation[2]), inv_rotation)
    return rotated, inv_rotation


def compose_transform(
    lhs_translation: tuple[float, float, float],
    lhs_rotation: tuple[float, float, float, float],
    rhs_translation: tuple[float, float, float],
    rhs_rotation: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    rotated_rhs = rotate_vector(rhs_translation, lhs_rotation)
    return (
        (
            lhs_translation[0] + rotated_rhs[0],
            lhs_translation[1] + rotated_rhs[1],
            lhs_translation[2] + rotated_rhs[2],
        ),
        normalize_quaternion(quat_multiply(lhs_rotation, rhs_rotation)),
    )


class AprilTagAnchorLocalizer(Node):
    def __init__(self) -> None:
        super().__init__("medipick_apriltag_anchor_localizer")

        self.declare_parameter("anchors_file", "")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("output_topic", "/medipick/navigation/apriltag_pose")
        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("transform_timeout_sec", 0.15)
        self.declare_parameter("position_variance", 0.0025)
        self.declare_parameter("yaw_variance", 0.0076)

        self._anchors = self._load_anchors(str(self.get_parameter("anchors_file").value).strip())
        self._camera_frame = str(self.get_parameter("camera_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._timeout = Duration(seconds=float(self.get_parameter("transform_timeout_sec").value))
        self._position_variance = float(self.get_parameter("position_variance").value)
        self._yaw_variance = float(self.get_parameter("yaw_variance").value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("output_topic").value),
            10,
        )

        publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self.create_timer(1.0 / publish_rate, self._publish_anchor_pose)

        self.get_logger().info(
            "AprilTag anchor localizer ready. "
            f"Loaded {len(self._anchors)} anchors from {self.get_parameter('anchors_file').value or '<none>'}"
        )

    def _publish_anchor_pose(self) -> None:
        if not self._anchors:
            return

        base_to_camera = self._lookup_transform(self._base_frame, self._camera_frame)
        if base_to_camera is None:
            return

        for anchor in self._anchors:
            camera_to_tag = self._lookup_transform(self._camera_frame, anchor.frame_id)
            if camera_to_tag is None:
                continue

            map_to_base = compose_transform(
                anchor.position,
                anchor.orientation,
                *inverse_transform(*camera_to_tag),
            )
            map_to_base = compose_transform(
                map_to_base[0],
                map_to_base[1],
                *inverse_transform(*base_to_camera),
            )
            self._publish_pose(anchor, map_to_base[0], map_to_base[1])
            return

    def _publish_pose(
        self,
        anchor: Anchor,
        translation: tuple[float, float, float],
        rotation: tuple[float, float, float, float],
    ) -> None:
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._map_frame
        message.pose.pose.position.x = translation[0]
        message.pose.pose.position.y = translation[1]
        message.pose.pose.position.z = translation[2]
        message.pose.pose.orientation.x = rotation[0]
        message.pose.pose.orientation.y = rotation[1]
        message.pose.pose.orientation.z = rotation[2]
        message.pose.pose.orientation.w = rotation[3]
        message.pose.covariance[0] = self._position_variance
        message.pose.covariance[7] = self._position_variance
        message.pose.covariance[14] = self._position_variance * 4.0
        message.pose.covariance[21] = self._yaw_variance * 4.0
        message.pose.covariance[28] = self._yaw_variance * 4.0
        message.pose.covariance[35] = self._yaw_variance
        self._publisher.publish(message)
        self.get_logger().debug(f"Published anchor-localized base pose from {anchor.frame_id}")

    def _lookup_transform(
        self,
        target_frame: str,
        source_frame: str,
    ) -> Optional[tuple[tuple[float, float, float], tuple[float, float, float, float]]]:
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=self._timeout,
            )
        except TransformException:
            return None

        translation = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        )
        rotation = normalize_quaternion(
            (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
        )
        return translation, rotation

    def _load_anchors(self, anchors_file: str) -> list[Anchor]:
        if not anchors_file:
            return []

        path = Path(anchors_file)
        if not path.exists():
            self.get_logger().warning(f"Anchor file {anchors_file} does not exist; starting without anchors.")
            return []

        with path.open("r", encoding="utf-8") as file:
            data = yaml.safe_load(file) or {}

        anchors: list[Anchor] = []
        for item in data.get("anchors", []):
            anchors.append(
                Anchor(
                    frame_id=str(item["frame_id"]),
                    anchor_type=str(item.get("anchor_type", "scene")),
                    position=tuple(float(value) for value in item.get("position", [0.0, 0.0, 0.0])),
                    orientation=normalize_quaternion(
                        tuple(float(value) for value in item.get("orientation", [0.0, 0.0, 0.0, 1.0]))
                    ),
                )
            )
        return anchors


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = AprilTagAnchorLocalizer()
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
