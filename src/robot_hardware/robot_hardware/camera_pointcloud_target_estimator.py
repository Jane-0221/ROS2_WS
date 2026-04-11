#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class CameraPointCloudTargetEstimator(Node):
    def __init__(self) -> None:
        super().__init__("medipick_camera_pointcloud_target_estimator")

        self.declare_parameter("input_topic", "/camera/depth/points")
        self.declare_parameter("output_topic", "/camera/target_pose")
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("sample_step", 2)
        self.declare_parameter("min_x", -0.12)
        self.declare_parameter("max_x", 0.18)
        self.declare_parameter("min_y", -0.20)
        self.declare_parameter("max_y", 0.05)
        self.declare_parameter("min_z", 0.18)
        self.declare_parameter("max_z", 0.60)
        self.declare_parameter("voxel_size", 0.04)
        self.declare_parameter("cluster_radius", 0.06)
        self.declare_parameter("min_cluster_points", 40)
        self.declare_parameter("surface_front_ratio", 0.25)
        self.declare_parameter("depth_weight", 0.35)
        self.declare_parameter("offset_x", 0.0)
        self.declare_parameter("offset_y", 0.0)
        self.declare_parameter("offset_z", 0.07)
        self.declare_parameter("orientation_x", 0.0)
        self.declare_parameter("orientation_y", 0.0)
        self.declare_parameter("orientation_z", 0.70710678)
        self.declare_parameter("orientation_w", 0.70710678)
        self.declare_parameter("publish_debug_cloud", False)
        self.declare_parameter("debug_cloud_topic", "/camera/target_cluster")

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._min_xyz = np.array(
            [
                float(self.get_parameter("min_x").value),
                float(self.get_parameter("min_y").value),
                float(self.get_parameter("min_z").value),
            ],
            dtype=np.float32,
        )
        self._max_xyz = np.array(
            [
                float(self.get_parameter("max_x").value),
                float(self.get_parameter("max_y").value),
                float(self.get_parameter("max_z").value),
            ],
            dtype=np.float32,
        )
        self._publish_period = 1.0 / max(float(self.get_parameter("publish_rate").value), 0.1)
        self._sample_step = max(int(self.get_parameter("sample_step").value), 1)
        self._voxel_size = max(float(self.get_parameter("voxel_size").value), 0.005)
        self._cluster_radius = max(float(self.get_parameter("cluster_radius").value), self._voxel_size)
        self._min_cluster_points = max(int(self.get_parameter("min_cluster_points").value), 8)
        self._surface_front_ratio = min(max(float(self.get_parameter("surface_front_ratio").value), 0.05), 0.9)
        self._depth_weight = max(float(self.get_parameter("depth_weight").value), 0.0)
        self._offset = np.array(
            [
                float(self.get_parameter("offset_x").value),
                float(self.get_parameter("offset_y").value),
                float(self.get_parameter("offset_z").value),
            ],
            dtype=np.float32,
        )
        self._orientation = (
            float(self.get_parameter("orientation_x").value),
            float(self.get_parameter("orientation_y").value),
            float(self.get_parameter("orientation_z").value),
            float(self.get_parameter("orientation_w").value),
        )
        self._publish_debug_cloud = bool(self.get_parameter("publish_debug_cloud").value)
        self._last_publish_time_sec = -1.0
        self._last_warning_time_sec = -1.0

        self._pose_publisher = self.create_publisher(PoseStamped, self._output_topic, 10)
        self._debug_cloud_publisher = None
        if self._publish_debug_cloud:
            self._debug_cloud_publisher = self.create_publisher(
                PointCloud2,
                str(self.get_parameter("debug_cloud_topic").value),
                10,
            )

        self.create_subscription(PointCloud2, self._input_topic, self._on_pointcloud, 10)
        self.get_logger().info(
            "Camera point-cloud target estimator ready. "
            f"{self._input_topic} -> {self._output_topic}, "
            f"roi=({self._min_xyz.tolist()} .. {self._max_xyz.tolist()})"
        )

    def _on_pointcloud(self, message: PointCloud2) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self._last_publish_time_sec > 0.0 and now_sec - self._last_publish_time_sec < self._publish_period:
            return

        points = self._extract_points(message)
        if points.size == 0:
            self._warn_throttled("No valid point-cloud samples received from the camera.")
            return

        roi_mask = np.logical_and(points >= self._min_xyz, points <= self._max_xyz).all(axis=1)
        roi_points = points[roi_mask]
        if roi_points.shape[0] < self._min_cluster_points:
            self._warn_throttled(
                f"ROI produced only {roi_points.shape[0]} points. "
                "Adjust min/max x/y/z to cover the target object."
            )
            return

        cluster = self._select_target_cluster(roi_points)
        if cluster.shape[0] < self._min_cluster_points:
            self._warn_throttled(
                f"Detected cluster too small ({cluster.shape[0]} points). "
                "The object may be outside the ROI or too sparse."
            )
            return

        pose = self._build_target_pose(message, cluster)
        self._pose_publisher.publish(pose)
        if self._debug_cloud_publisher is not None:
            self._debug_cloud_publisher.publish(point_cloud2.create_cloud_xyz32(message.header, cluster.tolist()))
        self._last_publish_time_sec = now_sec

    def _extract_points(self, message: PointCloud2) -> np.ndarray:
        sampled_points = []
        for index, point in enumerate(point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)):
            if self._sample_step > 1 and index % self._sample_step != 0:
                continue
            sampled_points.append((point[0], point[1], point[2]))
        if not sampled_points:
            return np.empty((0, 3), dtype=np.float32)
        return np.asarray(sampled_points, dtype=np.float32)

    def _select_target_cluster(self, roi_points: np.ndarray) -> np.ndarray:
        voxel_indices = np.floor(roi_points / self._voxel_size).astype(np.int32)
        unique_voxels, inverse_indices, counts = np.unique(
            voxel_indices,
            axis=0,
            return_inverse=True,
            return_counts=True,
        )
        voxel_centers_z = (unique_voxels[:, 2].astype(np.float32) + 0.5) * self._voxel_size
        voxel_scores = counts.astype(np.float32) - self._depth_weight * voxel_centers_z
        best_voxel = unique_voxels[int(np.argmax(voxel_scores))]

        neighborhood_mask = np.abs(voxel_indices - best_voxel).max(axis=1) <= 1
        seed_points = roi_points[neighborhood_mask]
        if seed_points.shape[0] == 0:
            return seed_points

        seed_center = np.median(seed_points, axis=0)
        distances = np.linalg.norm(roi_points - seed_center, axis=1)
        cluster = roi_points[distances <= self._cluster_radius]
        return cluster if cluster.shape[0] >= seed_points.shape[0] else seed_points

    def _build_target_pose(self, message: PointCloud2, cluster: np.ndarray) -> PoseStamped:
        surface_cutoff = np.quantile(cluster[:, 2], self._surface_front_ratio)
        surface_points = cluster[cluster[:, 2] <= surface_cutoff]
        if surface_points.shape[0] < max(6, self._min_cluster_points // 4):
            surface_points = cluster

        target = np.median(surface_points, axis=0) + self._offset

        pose = PoseStamped()
        pose.header = message.header
        pose.pose.position.x = float(target[0])
        pose.pose.position.y = float(target[1])
        pose.pose.position.z = float(target[2])
        pose.pose.orientation.x = self._orientation[0]
        pose.pose.orientation.y = self._orientation[1]
        pose.pose.orientation.z = self._orientation[2]
        pose.pose.orientation.w = self._orientation[3]
        return pose

    def _warn_throttled(self, message: str) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self._last_warning_time_sec > 0.0 and now_sec - self._last_warning_time_sec < 2.0:
            return
        self._last_warning_time_sec = now_sec
        self.get_logger().warning(message)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = CameraPointCloudTargetEstimator()
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
