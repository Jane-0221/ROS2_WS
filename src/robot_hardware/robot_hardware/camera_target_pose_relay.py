#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class CameraTargetPoseRelay(Node):
    def __init__(self) -> None:
        super().__init__("medipick_camera_target_pose_relay")

        self.declare_parameter("input_topic", "/camera/target_pose")
        self.declare_parameter("output_topic", "/medipick/task/target_pose")
        self.declare_parameter("output_frame", "world")
        self.declare_parameter("replace_stamp_with_now", False)
        self.declare_parameter("transform_timeout_sec", 0.2)
        self.declare_parameter("passthrough_if_tf_unavailable", False)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._output_frame = str(self.get_parameter("output_frame").value)
        self._replace_stamp_with_now = bool(self.get_parameter("replace_stamp_with_now").value)
        self._transform_timeout = Duration(seconds=float(self.get_parameter("transform_timeout_sec").value))
        self._passthrough_if_tf_unavailable = bool(self.get_parameter("passthrough_if_tf_unavailable").value)
        self._last_tf_warning_ns = 0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(PoseStamped, self._output_topic, 10)
        self.create_subscription(PoseStamped, self._input_topic, self._handle_pose, 10)

        self.get_logger().info(
            "Camera target pose relay ready. "
            f"{self._input_topic} -> {self._output_topic}, output_frame='{self._output_frame or '<source>'}'"
        )

    def _handle_pose(self, message: PoseStamped) -> None:
        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose

        if not pose.header.frame_id:
            self.get_logger().warning("Received target pose without frame_id; dropping message.")
            return

        if self._output_frame and pose.header.frame_id != self._output_frame:
            transformed_pose = self._transform_pose(pose, self._output_frame)
            if transformed_pose is None:
                if not self._passthrough_if_tf_unavailable:
                    return
            else:
                pose = transformed_pose

        if self._replace_stamp_with_now or self._stamp_is_zero(pose):
            pose.header.stamp = self.get_clock().now().to_msg()

        self._publisher.publish(pose)

    def _transform_pose(self, pose: PoseStamped, target_frame: str) -> Optional[PoseStamped]:
        lookup_time = Time()
        if not self._stamp_is_zero(pose):
            lookup_time = Time.from_msg(pose.header.stamp)

        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                pose.header.frame_id,
                lookup_time,
                timeout=self._transform_timeout,
            )
        except TransformException as exc:
            self._warn_tf_failure(pose.header.frame_id, target_frame, exc)
            return None

        result = PoseStamped()
        result.header = pose.header
        result.header.frame_id = target_frame

        transform_translation = transform.transform.translation
        transform_rotation = self._normalize_quaternion(
            (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
        )
        pose_position = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        )
        rotated_position = self._rotate_vector(pose_position, transform_rotation)

        result.pose.position.x = rotated_position[0] + transform_translation.x
        result.pose.position.y = rotated_position[1] + transform_translation.y
        result.pose.position.z = rotated_position[2] + transform_translation.z

        pose_rotation = self._normalize_quaternion(
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            )
        )
        result_rotation = self._normalize_quaternion(self._quat_multiply(transform_rotation, pose_rotation))
        result.pose.orientation.x = result_rotation[0]
        result.pose.orientation.y = result_rotation[1]
        result.pose.orientation.z = result_rotation[2]
        result.pose.orientation.w = result_rotation[3]
        return result

    def _warn_tf_failure(self, source_frame: str, target_frame: str, exc: TransformException) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_tf_warning_ns < 2_000_000_000:
            return
        self._last_tf_warning_ns = now_ns
        self.get_logger().warning(
            f"Failed to transform target pose from '{source_frame}' to '{target_frame}': {exc}"
        )

    @staticmethod
    def _stamp_is_zero(pose: PoseStamped) -> bool:
        return pose.header.stamp.sec == 0 and pose.header.stamp.nanosec == 0

    @staticmethod
    def _normalize_quaternion(
        quaternion: tuple[float, float, float, float]
    ) -> tuple[float, float, float, float]:
        x, y, z, w = quaternion
        norm = (x * x + y * y + z * z + w * w) ** 0.5
        if norm < 1e-9:
            return (0.0, 0.0, 0.0, 1.0)
        return (x / norm, y / norm, z / norm, w / norm)

    @staticmethod
    def _quat_multiply(
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

    @staticmethod
    def _quat_conjugate(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        x, y, z, w = quaternion
        return (-x, -y, -z, w)

    @classmethod
    def _rotate_vector(
        cls,
        vector: tuple[float, float, float],
        quaternion: tuple[float, float, float, float],
    ) -> tuple[float, float, float]:
        pure_quaternion = (vector[0], vector[1], vector[2], 0.0)
        rotated = cls._quat_multiply(
            cls._quat_multiply(quaternion, pure_quaternion),
            cls._quat_conjugate(quaternion),
        )
        return (rotated[0], rotated[1], rotated[2])


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = CameraTargetPoseRelay()
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
