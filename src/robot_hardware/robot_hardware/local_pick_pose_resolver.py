#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from medipick_planning_interfaces.srv import ResolveLocalPick
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


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


class LocalPickPoseResolver(Node):
    def __init__(self) -> None:
        super().__init__("medipick_local_pick_pose_resolver")

        self.declare_parameter("service_name", "/medipick/vision/resolve_local_pick")
        self.declare_parameter("input_topic", "/camera/local_pick_pose")
        self.declare_parameter("output_topic", "/medipick/vision/local_pick_pose")
        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter("transform_timeout_sec", 0.2)
        self.declare_parameter("max_pose_age_sec", 2.0)
        self.declare_parameter("nominal_confidence", 0.85)
        self.declare_parameter("allow_reference_pose_fallback", False)
        self.declare_parameter("fallback_confidence", 0.30)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_frame = str(self.get_parameter("output_frame").value)
        self._transform_timeout = Duration(seconds=float(self.get_parameter("transform_timeout_sec").value))
        self._max_pose_age_sec = float(self.get_parameter("max_pose_age_sec").value)
        self._nominal_confidence = float(self.get_parameter("nominal_confidence").value)
        self._allow_reference_pose_fallback = bool(self.get_parameter("allow_reference_pose_fallback").value)
        self._fallback_confidence = float(self.get_parameter("fallback_confidence").value)

        self._latest_pose: Optional[PoseStamped] = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._resolved_pose_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("output_topic").value),
            10,
        )
        self.create_subscription(PoseStamped, self._input_topic, self._handle_pose, 10)
        self.create_service(
            ResolveLocalPick,
            str(self.get_parameter("service_name").value),
            self._handle_resolve_request,
        )

        self.get_logger().info(
            "Local pick pose resolver ready. "
            f"Input={self._input_topic}, service={self.get_parameter('service_name').value}, "
            f"output_frame={self._output_frame}"
        )

    def _handle_pose(self, message: PoseStamped) -> None:
        self._latest_pose = message

    def _handle_resolve_request(
        self,
        request: ResolveLocalPick.Request,
        response: ResolveLocalPick.Response,
    ) -> ResolveLocalPick.Response:
        latest_pose = self._latest_pose
        if latest_pose is not None:
            local_pose = self._transform_pose(latest_pose, self._output_frame)
            if local_pose is not None:
                pose_age_sec = self._pose_age_seconds(latest_pose)
                if pose_age_sec is None or pose_age_sec <= self._max_pose_age_sec:
                    response.success = True
                    response.message = (
                        f"Resolved local pick pose for '{request.medicine_id}' from {self._input_topic}."
                    )
                    response.local_pick_pose = local_pose
                    response.confidence = self._nominal_confidence
                    self._resolved_pose_pub.publish(local_pose)
                    return response

        if self._allow_reference_pose_fallback:
            fallback_pose = self._transform_pose(request.reference_item_pose_map, self._output_frame)
            if fallback_pose is not None:
                response.success = True
                response.message = (
                    "No fresh local visual pose was available; fell back to the reference medicine pose."
                )
                response.local_pick_pose = fallback_pose
                response.confidence = self._fallback_confidence
                self._resolved_pose_pub.publish(fallback_pose)
                return response

        response.success = False
        response.message = (
            f"No fresh local pick pose is available on {self._input_topic}, "
            "and reference fallback is disabled."
        )
        return response

    def _pose_age_seconds(self, pose: PoseStamped) -> Optional[float]:
        stamp = pose.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None
        return (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9

    def _transform_pose(self, pose: PoseStamped, target_frame: str) -> Optional[PoseStamped]:
        if not pose.header.frame_id:
            self.get_logger().warning("Received a pose without frame_id; ignoring it.")
            return None
        if pose.header.frame_id == target_frame:
            return pose

        try:
            if not self._tf_buffer.can_transform(
                target_frame,
                pose.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.0),
            ):
                self.get_logger().warning(
                    f"Transform from {pose.header.frame_id} to {target_frame} is not available in the TF buffer yet."
                )
                return None
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                pose.header.frame_id,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"Failed to transform local pose from {pose.header.frame_id} to {target_frame}: {exc}"
            )
            return None

        transform_rotation = normalize_quaternion(
            (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
        )
        transform_translation = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        )
        pose_rotation = normalize_quaternion(
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            )
        )
        rotated_position = rotate_vector(
            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
            transform_rotation,
        )
        transformed_pose = PoseStamped()
        transformed_pose.header = pose.header
        transformed_pose.header.frame_id = target_frame
        transformed_pose.pose.position.x = rotated_position[0] + transform_translation[0]
        transformed_pose.pose.position.y = rotated_position[1] + transform_translation[1]
        transformed_pose.pose.position.z = rotated_position[2] + transform_translation[2]
        transformed_orientation = normalize_quaternion(quat_multiply(transform_rotation, pose_rotation))
        transformed_pose.pose.orientation.x = transformed_orientation[0]
        transformed_pose.pose.orientation.y = transformed_orientation[1]
        transformed_pose.pose.orientation.z = transformed_orientation[2]
        transformed_pose.pose.orientation.w = transformed_orientation[3]
        return transformed_pose


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LocalPickPoseResolver()
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
