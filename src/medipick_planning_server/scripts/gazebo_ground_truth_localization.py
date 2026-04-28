#!/usr/bin/python3

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw: float) -> Quaternion:
    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(yaw / 2.0)
    quat.w = math.cos(yaw / 2.0)
    return quat


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class BaseSample:
    stamp_sec: float
    x: float
    y: float
    theta: float


class GazeboGroundTruthLocalization(Node):
    def __init__(self) -> None:
        super().__init__("medipick_gazebo_ground_truth_localization")

        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("position_stddev", 0.005)
        self.declare_parameter("yaw_stddev_rad", 0.01)
        self.declare_parameter("publish_static_tf", True)
        self.declare_parameter("publish_map_to_odom_tf", True)
        self.declare_parameter("publish_odom_to_world_tf", True)

        self._joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._amcl_pose_topic = str(self.get_parameter("amcl_pose_topic").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._world_frame = str(self.get_parameter("world_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._publish_rate_hz = max(5.0, float(self.get_parameter("publish_rate_hz").value))
        self._position_stddev = max(1e-4, float(self.get_parameter("position_stddev").value))
        self._yaw_stddev = max(1e-4, float(self.get_parameter("yaw_stddev_rad").value))
        self._publish_static_tf = bool(self.get_parameter("publish_static_tf").value)
        self._publish_map_to_odom_tf = bool(self.get_parameter("publish_map_to_odom_tf").value)
        self._publish_odom_to_world_tf = bool(self.get_parameter("publish_odom_to_world_tf").value)

        self._current_joint_state = JointState()
        self._last_sample: BaseSample | None = None
        self._last_raw_base_theta: float | None = None
        self._continuous_base_theta = 0.0

        self.create_subscription(JointState, self._joint_state_topic, self._on_joint_state, 20)
        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 20)
        self._amcl_pub = self.create_publisher(PoseWithCovarianceStamped, self._amcl_pose_topic, 20)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / self._publish_rate_hz, self._on_timer)

        self.get_logger().info(
            "Gazebo 完美定位节点已启动："
            f" odom={self._odom_topic}, amcl_pose={self._amcl_pose_topic}, "
            f"frames={self._map_frame}->{self._odom_frame}->{self._world_frame}->{self._base_frame}, "
            f"publish_static_tf={self._publish_static_tf}, "
            f"publish_map_to_odom_tf={self._publish_map_to_odom_tf}, "
            f"publish_odom_to_world_tf={self._publish_odom_to_world_tf}"
        )

    @staticmethod
    def _build_planar_covariance(
        linear_xy_variance: float,
        yaw_variance: float,
        ignored_axis_variance: float = 1.0e-3,
    ) -> list[float]:
        covariance = [0.0] * 36
        covariance[0] = linear_xy_variance
        covariance[7] = linear_xy_variance
        covariance[14] = ignored_axis_variance
        covariance[21] = ignored_axis_variance
        covariance[28] = ignored_axis_variance
        covariance[35] = yaw_variance
        return covariance

    def _publish_identity_frames(self, stamp_msg) -> None:
        transforms: list[TransformStamped] = []

        if self._publish_map_to_odom_tf:
            map_to_odom = TransformStamped()
            map_to_odom.header.stamp = stamp_msg
            map_to_odom.header.frame_id = self._map_frame
            map_to_odom.child_frame_id = self._odom_frame
            map_to_odom.transform.rotation.w = 1.0
            transforms.append(map_to_odom)

        if self._publish_odom_to_world_tf:
            odom_to_world = TransformStamped()
            odom_to_world.header.stamp = stamp_msg
            odom_to_world.header.frame_id = self._odom_frame
            odom_to_world.child_frame_id = self._world_frame
            odom_to_world.transform.rotation.w = 1.0
            transforms.append(odom_to_world)

        if transforms:
            self._tf_broadcaster.sendTransform(transforms)

    def _on_joint_state(self, msg: JointState) -> None:
        self._current_joint_state = msg
        if "base_theta" not in msg.name:
            return
        index = msg.name.index("base_theta")
        if index >= len(msg.position):
            return
        raw_theta = float(msg.position[index])
        if self._last_raw_base_theta is None:
            self._continuous_base_theta = raw_theta
        else:
            self._continuous_base_theta += normalize_angle(raw_theta - self._last_raw_base_theta)
        self._last_raw_base_theta = raw_theta

    def _joint_position(self, joint_name: str, default: float = 0.0) -> float:
        if joint_name not in self._current_joint_state.name:
            return default
        index = self._current_joint_state.name.index(joint_name)
        if index >= len(self._current_joint_state.position):
            return default
        return float(self._current_joint_state.position[index])

    def _base_theta(self) -> float:
        if self._last_raw_base_theta is None:
            return self._joint_position("base_theta", 0.0)
        return self._continuous_base_theta

    def _on_timer(self) -> None:
        if len(self._current_joint_state.name) == 0:
            return

        stamp = self.get_clock().now()
        stamp_sec = stamp.nanoseconds / 1e9
        x = self._joint_position("base_x", 0.0)
        y = self._joint_position("base_y", 0.0)
        theta = self._base_theta()

        linear_vx = 0.0
        linear_vy = 0.0
        angular_vz = 0.0
        if self._last_sample is not None:
            dt = max(1e-3, stamp_sec - self._last_sample.stamp_sec)
            linear_vx = (x - self._last_sample.x) / dt
            linear_vy = (y - self._last_sample.y) / dt
            angular_vz = (theta - self._last_sample.theta) / dt

        self._last_sample = BaseSample(stamp_sec=stamp_sec, x=x, y=y, theta=theta)
        quat = quaternion_from_yaw(theta)
        planar_pose_covariance = self._build_planar_covariance(
            linear_xy_variance=self._position_stddev**2,
            yaw_variance=self._yaw_stddev**2,
        )
        planar_twist_covariance = self._build_planar_covariance(
            linear_xy_variance=max(self._position_stddev**2, 1.0e-4),
            yaw_variance=max(self._yaw_stddev**2, 1.0e-4),
        )

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp.to_msg()
        odom_msg.header.frame_id = self._odom_frame
        odom_msg.child_frame_id = self._base_frame
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.orientation = quat
        odom_msg.twist.twist.linear.x = linear_vx
        odom_msg.twist.twist.linear.y = linear_vy
        odom_msg.twist.twist.angular.z = angular_vz
        odom_msg.pose.covariance = planar_pose_covariance
        odom_msg.twist.covariance = planar_twist_covariance
        self._odom_pub.publish(odom_msg)

        if self._publish_static_tf:
            self._publish_identity_frames(odom_msg.header.stamp)

        amcl_msg = PoseWithCovarianceStamped()
        amcl_msg.header.stamp = stamp.to_msg()
        amcl_msg.header.frame_id = self._map_frame
        amcl_msg.pose.pose.position.x = x
        amcl_msg.pose.pose.position.y = y
        amcl_msg.pose.pose.orientation = quat
        amcl_msg.pose.covariance = planar_pose_covariance
        self._amcl_pub.publish(amcl_msg)


def main() -> None:
    rclpy.init()
    node = GazeboGroundTruthLocalization()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
