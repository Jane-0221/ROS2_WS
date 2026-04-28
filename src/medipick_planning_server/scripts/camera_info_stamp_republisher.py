#!/usr/bin/env python3

import copy
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
try:
    from rclpy.exceptions import RCLError
except ImportError:  # Humble does not expose RCLError here.
    RCLError = RuntimeError
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


def sensor_qos(depth: int = 20) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class CameraInfoStampRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("medipick_camera_info_stamp_republisher")

        self.declare_parameter("image_topic", "/medipick/depth_camera/image")
        self.declare_parameter("camera_info_topic", "/medipick/depth_camera/camera_info")
        self.declare_parameter("output_topic", "/medipick/depth_camera/camera_info_synced")
        self.declare_parameter("horizontal_fov_rad", 1.047)
        self.declare_parameter("force_recompute_intrinsics", False)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._horizontal_fov = float(self.get_parameter("horizontal_fov_rad").value)
        self._force_recompute_intrinsics = bool(self.get_parameter("force_recompute_intrinsics").value)

        self._latest_camera_info: CameraInfo | None = None
        self._published_once = False
        self._rewrote_intrinsics = False
        self._publisher = self.create_publisher(CameraInfo, output_topic, sensor_qos())
        self.create_subscription(CameraInfo, camera_info_topic, self._on_camera_info, sensor_qos(),)
        self.create_subscription(Image, image_topic, self._on_image, sensor_qos(),)

        self.get_logger().info(
            f"相机内参时间戳重发布器已启动：image={image_topic}, "
            f"camera_info={camera_info_topic}, output={output_topic}"
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._latest_camera_info = msg

    def _needs_intrinsics_fix(self, camera_info: CameraInfo, image: Image) -> bool:
        if self._force_recompute_intrinsics:
            return True
        if camera_info.width != image.width or camera_info.height != image.height:
            return True
        expected_cx = image.width / 2.0
        expected_cy = image.height / 2.0
        return abs(camera_info.k[2] - expected_cx) > 1.0 or abs(camera_info.k[5] - expected_cy) > 1.0

    def _rewrite_intrinsics(self, camera_info: CameraInfo, image: Image) -> None:
        fx = image.width / (2.0 * math.tan(self._horizontal_fov / 2.0))
        fy = fx
        cx = image.width / 2.0
        cy = image.height / 2.0

        camera_info.width = image.width
        camera_info.height = image.height
        camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

    def _on_image(self, msg: Image) -> None:
        if self._latest_camera_info is None:
            return

        synced = copy.deepcopy(self._latest_camera_info)
        synced.header.stamp = msg.header.stamp
        synced.header.frame_id = msg.header.frame_id
        if self._needs_intrinsics_fix(synced, msg):
            self._rewrite_intrinsics(synced, msg)
            if not self._rewrote_intrinsics:
                self._rewrote_intrinsics = True
                self.get_logger().warning(
                    "检测到 camera_info 与图像分辨率不一致，已按图像尺寸重写内参："
                    f" width={msg.width}, height={msg.height}, fx={synced.k[0]:.1f}, fy={synced.k[4]:.1f}"
                )
        self._publisher.publish(synced)

        if not self._published_once:
            self._published_once = True
            self.get_logger().info(
                f"已开始输出同步后的 camera_info，frame={synced.header.frame_id}"
            )


def main() -> None:
    rclpy.init()
    node = CameraInfoStampRepublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        if rclpy.ok() and "Unable to convert call argument" not in str(exc):
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except (RCLError, RuntimeError):
            pass


if __name__ == "__main__":
    main()
