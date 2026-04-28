#!/usr/bin/python3

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


BASE_JOINTS = ("base_x", "base_y", "base_theta")


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


class GazeboOmniBaseDriver(Node):
    def __init__(self) -> None:
        super().__init__("medipick_gazebo_omni_base_driver")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("trajectory_topic", "/base_controller/joint_trajectory")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("control_rate_hz", 15.0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("command_timeout", 0.40)
        self.declare_parameter("lookahead_time", 0.25)
        self.declare_parameter("max_linear_speed", 0.35)
        self.declare_parameter("max_lateral_speed", 0.35)
        self.declare_parameter("max_angular_speed", 0.80)
        self.declare_parameter("trajectory_linear_gain", 6.0)
        self.declare_parameter("trajectory_lateral_gain", 6.0)
        self.declare_parameter("trajectory_angular_gain", 4.0)
        self.declare_parameter("max_target_linear_speed", 2.0)
        self.declare_parameter("max_target_lateral_speed", 2.0)
        self.declare_parameter("max_target_angular_speed", 2.5)
        self.declare_parameter("publish_zero_when_idle", True)
        self.declare_parameter("position_stddev", 0.005)
        self.declare_parameter("yaw_stddev_rad", 0.01)
        self.declare_parameter("publish_identity_tf", True)
        self.declare_parameter("publish_map_to_odom_tf", False)
        self.declare_parameter("publish_odom_to_world_tf", True)
        self.declare_parameter("publish_amcl_pose", True)
        self.declare_parameter("odom_stamp_backoff_sec", 0.08)

        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self._trajectory_topic = str(self.get_parameter("trajectory_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._amcl_pose_topic = str(self.get_parameter("amcl_pose_topic").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._world_frame = str(self.get_parameter("world_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._control_rate_hz = max(5.0, float(self.get_parameter("control_rate_hz").value))
        self._publish_rate_hz = max(5.0, float(self.get_parameter("publish_rate_hz").value))
        self._command_timeout = max(0.10, float(self.get_parameter("command_timeout").value))
        self._lookahead_time = max(0.10, float(self.get_parameter("lookahead_time").value))
        self._max_linear_speed = max(0.05, float(self.get_parameter("max_linear_speed").value))
        self._max_lateral_speed = max(0.05, float(self.get_parameter("max_lateral_speed").value))
        self._max_angular_speed = max(0.05, float(self.get_parameter("max_angular_speed").value))
        self._trajectory_linear_gain = max(1.0, float(self.get_parameter("trajectory_linear_gain").value))
        self._trajectory_lateral_gain = max(1.0, float(self.get_parameter("trajectory_lateral_gain").value))
        self._trajectory_angular_gain = max(1.0, float(self.get_parameter("trajectory_angular_gain").value))
        self._max_target_linear_speed = max(0.05, float(self.get_parameter("max_target_linear_speed").value))
        self._max_target_lateral_speed = max(0.05, float(self.get_parameter("max_target_lateral_speed").value))
        self._max_target_angular_speed = max(0.05, float(self.get_parameter("max_target_angular_speed").value))
        self._publish_zero_when_idle = bool(self.get_parameter("publish_zero_when_idle").value)
        self._position_stddev = max(1e-4, float(self.get_parameter("position_stddev").value))
        self._yaw_stddev = max(1e-4, float(self.get_parameter("yaw_stddev_rad").value))
        self._publish_identity_tf = bool(self.get_parameter("publish_identity_tf").value)
        self._publish_map_to_odom_tf = bool(self.get_parameter("publish_map_to_odom_tf").value)
        self._publish_odom_to_world_tf = bool(self.get_parameter("publish_odom_to_world_tf").value)
        self._publish_amcl_pose = bool(self.get_parameter("publish_amcl_pose").value)
        self._odom_stamp_backoff_sec = max(0.0, float(self.get_parameter("odom_stamp_backoff_sec").value))

        self._current_joint_state = JointState()
        self._last_cmd = Twist()
        self._last_cmd_time = 0.0
        self._last_sent_idle = False
        self._last_sample: BaseSample | None = None
        self._last_joint_stamp: TimeMsg | None = None
        self._last_raw_base_theta: float | None = None
        self._continuous_base_theta = 0.0

        self.create_subscription(Twist, self._cmd_vel_topic, self._on_cmd_vel, 20)
        self.create_subscription(JointState, self._joint_state_topic, self._on_joint_state, 20)
        self._trajectory_pub = self.create_publisher(JointTrajectory, self._trajectory_topic, 20)
        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 20)
        self._amcl_pub = self.create_publisher(PoseWithCovarianceStamped, self._amcl_pose_topic, 20)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / self._control_rate_hz, self._on_control_timer)
        self.create_timer(1.0 / self._publish_rate_hz, self._on_odom_timer)

        self.get_logger().info(
            "Gazebo omni base driver 已启动："
            f" cmd_vel={self._cmd_vel_topic}, trajectory={self._trajectory_topic},"
            f" odom={self._odom_topic}, frames={self._map_frame}->{self._odom_frame}->{self._world_frame}->{self._base_frame}"
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

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._last_cmd_time = time.monotonic()
        self._last_sent_idle = False

    def _on_joint_state(self, msg: JointState) -> None:
        self._current_joint_state = msg
        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            self._last_joint_stamp = msg.header.stamp
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

    def _on_control_timer(self) -> None:
        if len(self._current_joint_state.name) == 0:
            return

        now = time.monotonic()
        cmd_age = now - self._last_cmd_time
        linear_x = self._last_cmd.linear.x if cmd_age <= self._command_timeout else 0.0
        linear_y = self._last_cmd.linear.y if cmd_age <= self._command_timeout else 0.0
        angular_z = self._last_cmd.angular.z if cmd_age <= self._command_timeout else 0.0

        linear_x = max(-self._max_linear_speed, min(self._max_linear_speed, linear_x))
        linear_y = max(-self._max_lateral_speed, min(self._max_lateral_speed, linear_y))
        angular_z = max(-self._max_angular_speed, min(self._max_angular_speed, angular_z))

        target_linear_x = max(
            -self._max_target_linear_speed,
            min(self._max_target_linear_speed, linear_x * self._trajectory_linear_gain),
        )
        target_linear_y = max(
            -self._max_target_lateral_speed,
            min(self._max_target_lateral_speed, linear_y * self._trajectory_lateral_gain),
        )
        target_angular_z = max(
            -self._max_target_angular_speed,
            min(self._max_target_angular_speed, angular_z * self._trajectory_angular_gain),
        )

        is_idle = abs(linear_x) < 1e-4 and abs(linear_y) < 1e-4 and abs(angular_z) < 1e-4
        if is_idle and not self._publish_zero_when_idle:
            return
        if is_idle and self._last_sent_idle:
            return

        base_x = self._joint_position("base_x", 0.0)
        base_y = self._joint_position("base_y", 0.0)
        base_theta = self._base_theta()

        cos_theta = math.cos(base_theta)
        sin_theta = math.sin(base_theta)
        horizon = self._lookahead_time
        world_vx = target_linear_x * cos_theta - target_linear_y * sin_theta
        world_vy = target_linear_x * sin_theta + target_linear_y * cos_theta
        world_dx = world_vx * horizon
        world_dy = world_vy * horizon
        target_theta = base_theta + target_angular_z * horizon

        trajectory = JointTrajectory()
        trajectory.joint_names = list(BASE_JOINTS)

        point = JointTrajectoryPoint()
        point.positions = [base_x + world_dx, base_y + world_dy, target_theta]
        point.velocities = [world_vx, world_vy, target_angular_z]
        total_ns = int(self._lookahead_time * 1e9)
        point.time_from_start.sec = total_ns // 1_000_000_000
        point.time_from_start.nanosec = total_ns % 1_000_000_000
        trajectory.points = [point]

        self._trajectory_pub.publish(trajectory)
        self._last_sent_idle = is_idle

    def _on_odom_timer(self) -> None:
        now = self.get_clock().now()
        if self._last_joint_stamp is not None:
            stamp = rclpy.time.Time.from_msg(self._last_joint_stamp, clock_type=now.clock_type)
        else:
            stamp = now
        if self._odom_stamp_backoff_sec > 0.0:
            backoff_ns = int(self._odom_stamp_backoff_sec * 1e9)
            if stamp.nanoseconds > backoff_ns:
                stamp = rclpy.time.Time(
                    nanoseconds=stamp.nanoseconds - backoff_ns,
                    clock_type=now.clock_type,
                )
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

        if self._publish_identity_tf:
            self._publish_identity_frames(odom_msg.header.stamp)

        if self._publish_amcl_pose:
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
    node = GazeboOmniBaseDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        if rclpy.ok() and "Unable to convert call argument" not in str(exc):
            raise
    except Exception:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
