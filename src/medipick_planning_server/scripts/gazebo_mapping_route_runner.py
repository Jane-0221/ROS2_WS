#!/usr/bin/python3

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class GazeboMappingRouteRunner(Node):
    def __init__(self) -> None:
        super().__init__("medipick_gazebo_mapping_route_runner")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("done_topic", "/medipick/slam_mapping_route_done")
        self.declare_parameter("start_delay_sec", 2.0)
        self.declare_parameter(
            "route_xs",
            [0.35, 2.15, 2.15, 2.15, 3.50, 4.85, 4.85, 4.85, 6.55, 6.55, 6.55, 4.85, 3.50, 2.15, 2.15, 2.15, 0.35],
        )
        self.declare_parameter(
            "route_ys",
            [0.0, 0.0, 3.35, 0.0, 0.0, 0.0, 3.35, 0.0, 0.0, -3.35, 0.0, 0.0, 0.0, 0.0, -3.35, 0.0, 0.0],
        )
        self.declare_parameter(
            "route_yaws_deg",
            [0.0, 0.0, 90.0, -90.0, 0.0, 0.0, 90.0, -90.0, 0.0, -90.0, 90.0, 180.0, 180.0, 180.0, -90.0, 90.0, 180.0],
        )
        self.declare_parameter("waypoint_position_tolerance", 0.08)
        self.declare_parameter("waypoint_yaw_tolerance_deg", 10.0)
        self.declare_parameter("kp_linear", 0.9)
        self.declare_parameter("kp_lateral", 0.9)
        self.declare_parameter("kp_angular", 1.6)
        self.declare_parameter("max_linear_speed", 0.24)
        self.declare_parameter("max_lateral_speed", 0.24)
        self.declare_parameter("max_angular_speed", 0.65)
        self.declare_parameter("slowdown_radius", 0.30)
        self.declare_parameter("timer_period_sec", 0.05)
        self.declare_parameter("yaw_alignment_waypoint_indices", [-1])
        self.declare_parameter("enable_head_scan", False)
        self.declare_parameter("head_trajectory_topic", "/head_controller/joint_trajectory")
        self.declare_parameter("head_scan_waypoint_indices", [-1])
        self.declare_parameter("head_scan_h1_sequence_deg", [-35.0, 0.0, 35.0, 0.0])
        self.declare_parameter("head_scan_h2_deg", -30.0)
        self.declare_parameter("head_scan_move_sec", 0.60)
        self.declare_parameter("head_scan_hold_sec", 0.80)

        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._done_topic = str(self.get_parameter("done_topic").value)
        self._start_delay_sec = max(0.0, float(self.get_parameter("start_delay_sec").value))
        route_xs = [float(value) for value in self.get_parameter("route_xs").value]
        route_ys = [float(value) for value in self.get_parameter("route_ys").value]
        route_yaws_deg = [float(value) for value in self.get_parameter("route_yaws_deg").value]
        if len(route_xs) == 0 or len(route_xs) != len(route_ys) or len(route_xs) != len(route_yaws_deg):
            raise ValueError("route_xs / route_ys / route_yaws_deg 必须非空且长度一致")

        self._route = [
            (x_value, y_value, math.radians(yaw_deg))
            for x_value, y_value, yaw_deg in zip(route_xs, route_ys, route_yaws_deg)
        ]
        self._waypoint_position_tolerance = max(0.02, float(self.get_parameter("waypoint_position_tolerance").value))
        self._waypoint_yaw_tolerance = math.radians(
            max(2.0, float(self.get_parameter("waypoint_yaw_tolerance_deg").value))
        )
        self._kp_linear = max(0.1, float(self.get_parameter("kp_linear").value))
        self._kp_lateral = max(0.1, float(self.get_parameter("kp_lateral").value))
        self._kp_angular = max(0.1, float(self.get_parameter("kp_angular").value))
        self._max_linear_speed = max(0.05, float(self.get_parameter("max_linear_speed").value))
        self._max_lateral_speed = max(0.05, float(self.get_parameter("max_lateral_speed").value))
        self._max_angular_speed = max(0.05, float(self.get_parameter("max_angular_speed").value))
        self._slowdown_radius = max(0.05, float(self.get_parameter("slowdown_radius").value))
        self._timer_period = max(0.02, float(self.get_parameter("timer_period_sec").value))
        self._yaw_alignment_waypoint_indices = {
            int(value)
            for value in self.get_parameter("yaw_alignment_waypoint_indices").value
            if int(value) >= 0
        }
        self._enable_head_scan = bool(self.get_parameter("enable_head_scan").value)
        self._head_trajectory_topic = str(self.get_parameter("head_trajectory_topic").value)
        self._head_scan_waypoint_indices = {
            int(value)
            for value in self.get_parameter("head_scan_waypoint_indices").value
            if int(value) >= 0
        }
        self._head_scan_h1_sequence = tuple(
            math.radians(float(value)) for value in self.get_parameter("head_scan_h1_sequence_deg").value
        )
        self._head_scan_h2 = math.radians(float(self.get_parameter("head_scan_h2_deg").value))
        self._head_scan_move_sec = max(0.10, float(self.get_parameter("head_scan_move_sec").value))
        self._head_scan_hold_sec = max(0.05, float(self.get_parameter("head_scan_hold_sec").value))

        self._latest_odom: Odometry | None = None
        self._route_index = 0
        self._started_at = time.monotonic()
        self._done = False
        self._started_motion = False
        self._last_wait_log = 0.0
        self._head_scan_active = False
        self._head_scan_step_index = 0
        self._head_scan_step_started_at = 0.0
        self._head_scanned_waypoints: set[int] = set()

        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 20)
        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 20)
        self._head_pub = self.create_publisher(JointTrajectory, self._head_trajectory_topic, 10)

        done_qos = QoSProfile(depth=1)
        done_qos.reliability = ReliabilityPolicy.RELIABLE
        done_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._done_pub = self.create_publisher(Bool, self._done_topic, done_qos)
        self._publish_done(False)
        self.create_timer(self._timer_period, self._on_timer)

        self.get_logger().info(
            "Gazebo 自动建图路线器已启动："
            f" odom={self._odom_topic}, cmd_vel={self._cmd_vel_topic}, "
            f"done_topic={self._done_topic}, route_points={len(self._route)}, "
            f"yaw_alignment_points={sorted(self._yaw_alignment_waypoint_indices)}, "
            f"head_scan_enabled={self._enable_head_scan}, head_scan_points={sorted(self._head_scan_waypoint_indices)}"
        )

    def _publish_done(self, value: bool) -> None:
        msg = Bool()
        msg.data = value
        self._done_pub.publish(msg)

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _current_state(self) -> tuple[float, float, float] | None:
        if self._latest_odom is None:
            return None
        pose = self._latest_odom.pose.pose
        return (
            float(pose.position.x),
            float(pose.position.y),
            quaternion_to_yaw(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )

    def _publish_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        self._cmd_pub.publish(msg)

    def _publish_head_target(self, h1_rad: float, h2_rad: float) -> None:
        trajectory = JointTrajectory()
        trajectory.joint_names = ["h1_joint", "h2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [h1_rad, h2_rad]
        total_ns = int(self._head_scan_move_sec * 1e9)
        point.time_from_start.sec = total_ns // 1_000_000_000
        point.time_from_start.nanosec = total_ns % 1_000_000_000
        trajectory.points = [point]
        self._head_pub.publish(trajectory)

    def _scan_needed_for_waypoint(self, waypoint_index: int) -> bool:
        return (
            self._enable_head_scan
            and waypoint_index in self._head_scan_waypoint_indices
            and waypoint_index not in self._head_scanned_waypoints
            and len(self._head_scan_h1_sequence) > 0
        )

    def _start_head_scan(self, now: float) -> None:
        self._head_scan_active = True
        self._head_scan_step_index = 0
        self._head_scan_step_started_at = now
        self._publish_head_target(self._head_scan_h1_sequence[0], self._head_scan_h2)
        self.get_logger().info(
            f"自动建图路线器开始头部扫描：waypoint={self._route_index + 1}/{len(self._route)}"
        )

    def _advance_head_scan(self, now: float) -> bool:
        if not self._head_scan_active:
            return False

        if now - self._head_scan_step_started_at < self._head_scan_move_sec + self._head_scan_hold_sec:
            return True

        self._head_scan_step_index += 1
        if self._head_scan_step_index >= len(self._head_scan_h1_sequence):
            self._head_scan_active = False
            self._head_scanned_waypoints.add(self._route_index)
            self.get_logger().info(
                f"自动建图路线器完成头部扫描：waypoint={self._route_index + 1}/{len(self._route)}"
            )
            return False

        self._head_scan_step_started_at = now
        self._publish_head_target(self._head_scan_h1_sequence[self._head_scan_step_index], self._head_scan_h2)
        return True

    def _on_timer(self) -> None:
        now = time.monotonic()
        state = self._current_state()
        if state is None:
            if now - self._last_wait_log >= 2.0:
                self._last_wait_log = now
                self.get_logger().info("自动建图路线器仍在等待 /odom。")
            return

        if now < self._started_at + self._start_delay_sec:
            self._publish_cmd_vel(0.0, 0.0, 0.0)
            return

        if self._done:
            self._publish_cmd_vel(0.0, 0.0, 0.0)
            return

        if not self._started_motion:
            self._started_motion = True
            self.get_logger().info("自动建图路线器开始执行巡航路线。")

        target_x, target_y, target_yaw = self._route[self._route_index]
        current_x, current_y, current_yaw = state
        error_x = target_x - current_x
        error_y = target_y - current_y
        require_yaw_alignment = self._route_index in self._yaw_alignment_waypoint_indices
        error_yaw = normalize_angle(target_yaw - current_yaw) if require_yaw_alignment else 0.0
        planar_error = math.hypot(error_x, error_y)

        if self._head_scan_active:
            self._publish_cmd_vel(0.0, 0.0, 0.0)
            self._advance_head_scan(now)
            return

        if planar_error <= self._waypoint_position_tolerance and (
            not require_yaw_alignment or abs(error_yaw) <= self._waypoint_yaw_tolerance
        ):
            self._publish_cmd_vel(0.0, 0.0, 0.0)
            if self._scan_needed_for_waypoint(self._route_index):
                self._start_head_scan(now)
                return
            if self._route_index + 1 < len(self._route):
                self._route_index += 1
                self.get_logger().info(
                    f"自动建图路线器已到达路点，切换下一段：{self._route_index + 1}/{len(self._route)}"
                )
            else:
                self._done = True
                self._publish_done(True)
                self.get_logger().info("自动建图路线已完成。")
            return

        error_forward = math.cos(current_yaw) * error_x + math.sin(current_yaw) * error_y
        error_lateral = -math.sin(current_yaw) * error_x + math.cos(current_yaw) * error_y

        cmd_x = max(-self._max_linear_speed, min(self._max_linear_speed, self._kp_linear * error_forward))
        cmd_y = max(-self._max_lateral_speed, min(self._max_lateral_speed, self._kp_lateral * error_lateral))
        cmd_w = max(-self._max_angular_speed, min(self._max_angular_speed, self._kp_angular * error_yaw))

        if planar_error < self._slowdown_radius:
            scale = max(0.25, planar_error / self._slowdown_radius)
            cmd_x *= scale
            cmd_y *= scale
        if abs(error_yaw) < math.radians(12.0):
            cmd_w *= 0.5

        self._publish_cmd_vel(cmd_x, cmd_y, cmd_w)


def main() -> None:
    rclpy.init()
    node = GazeboMappingRouteRunner()
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
