#!/usr/bin/python3

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


BASE_JOINTS = ("base_x", "base_y", "base_theta")


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class GazeboCmdVelBridge(Node):
    def __init__(self) -> None:
        super().__init__("medipick_gazebo_cmd_vel_bridge")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("trajectory_topic", "/base_controller/joint_trajectory")
        self.declare_parameter("control_rate_hz", 15.0)
        self.declare_parameter("command_timeout", 0.40)
        self.declare_parameter("lookahead_time", 0.25)
        self.declare_parameter("max_linear_speed", 0.35)
        self.declare_parameter("max_lateral_speed", 0.35)
        self.declare_parameter("max_angular_speed", 0.80)
        self.declare_parameter("trajectory_linear_gain", 1.0)
        self.declare_parameter("trajectory_lateral_gain", 1.0)
        self.declare_parameter("trajectory_angular_gain", 1.0)
        self.declare_parameter("max_target_linear_speed", 2.0)
        self.declare_parameter("max_target_lateral_speed", 2.0)
        self.declare_parameter("max_target_angular_speed", 2.5)
        self.declare_parameter("publish_zero_when_idle", True)

        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self._trajectory_topic = str(self.get_parameter("trajectory_topic").value)
        self._control_rate_hz = max(5.0, float(self.get_parameter("control_rate_hz").value))
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

        self._current_joint_state = JointState()
        self._last_cmd = Twist()
        self._last_cmd_time = 0.0
        self._last_tick_time = time.monotonic()
        self._last_sent_idle = False
        self._last_raw_base_theta: float | None = None
        self._continuous_base_theta = 0.0

        self.create_subscription(Twist, self._cmd_vel_topic, self._on_cmd_vel, 20)
        self.create_subscription(JointState, self._joint_state_topic, self._on_joint_state, 20)
        self._trajectory_pub = self.create_publisher(JointTrajectory, self._trajectory_topic, 20)
        self.create_timer(1.0 / self._control_rate_hz, self._on_timer)

        self.get_logger().info(
            "Gazebo cmd_vel 桥已启动："
            f" cmd_vel={self._cmd_vel_topic}, joint_states={self._joint_state_topic},"
            f" trajectory={self._trajectory_topic}"
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._last_cmd_time = time.monotonic()
        self._last_sent_idle = False

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

        now = time.monotonic()
        self._last_tick_time = now

        cmd_age = now - self._last_cmd_time
        linear_x = self._last_cmd.linear.x if cmd_age <= self._command_timeout else 0.0
        linear_y = self._last_cmd.linear.y if cmd_age <= self._command_timeout else 0.0
        angular_z = self._last_cmd.angular.z if cmd_age <= self._command_timeout else 0.0

        linear_x = max(-self._max_linear_speed, min(self._max_linear_speed, linear_x))
        linear_y = max(-self._max_lateral_speed, min(self._max_lateral_speed, linear_y))
        angular_z = max(-self._max_angular_speed, min(self._max_angular_speed, angular_z))

        # Compensate for the conservative position-interface response we see with
        # Gazebo Humble so the simulated base tracks Nav2 cmd_vel more closely.
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
        # 这里要用前视时间生成目标点，而不是用本周期 dt。
        # 否则会出现“位移按很小的 dt 算、执行时间却按较大的 lookahead_time 算”，
        # 实际速度会被无意中压低到原指令速度的一小部分。
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


def main() -> None:
    rclpy.init()
    node = GazeboCmdVelBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
