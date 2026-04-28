#!/usr/bin/python3

from __future__ import annotations

import math
import select
import sys
import termios
import tty
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


MOVE_BINDINGS = {
    "i": (1.0, 0.0, "base forward"),
    ",": (-1.0, 0.0, "base backward"),
    "j": (0.0, 1.0, "base rotate left"),
    "l": (0.0, -1.0, "base rotate right"),
    "u": (1.0, 1.0, "base forward + left"),
    "o": (1.0, -1.0, "base forward + right"),
    "m": (-1.0, -1.0, "base backward + right"),
    ".": (-1.0, 1.0, "base backward + left"),
    "k": (0.0, 0.0, "base stop"),
    " ": (0.0, 0.0, "base stop"),
}

SPEED_BINDINGS = {
    "q": (1.1, 1.1, "increase linear and angular speed"),
    "z": (0.9, 0.9, "decrease linear and angular speed"),
    "w": (1.1, 1.0, "increase linear speed"),
    "x": (0.9, 1.0, "decrease linear speed"),
    "e": (1.0, 1.1, "increase angular speed"),
    "c": (1.0, 0.9, "decrease angular speed"),
}


@dataclass
class HeadState:
    h1: float = 0.0
    h2: float = -0.5236


class MappingKeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("medipick_mapping_keyboard_teleop")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("head_trajectory_topic", "/head_controller/joint_trajectory")
        self.declare_parameter("linear_speed", 0.20)
        self.declare_parameter("angular_speed", 0.50)
        self.declare_parameter("repeat_rate_hz", 10.0)
        self.declare_parameter("idle_timeout_sec", 0.35)
        self.declare_parameter("head_step_rad", 0.10)
        self.declare_parameter("head_duration_sec", 0.40)
        self.declare_parameter("head_reset_h1", 0.0)
        self.declare_parameter("head_reset_h2", -0.5236)
        self.declare_parameter("head_min_rad", -math.pi)
        self.declare_parameter("head_max_rad", math.pi)

        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self._head_trajectory_topic = str(self.get_parameter("head_trajectory_topic").value)
        self._linear_speed = float(self.get_parameter("linear_speed").value)
        self._angular_speed = float(self.get_parameter("angular_speed").value)
        self._repeat_rate_hz = max(2.0, float(self.get_parameter("repeat_rate_hz").value))
        self._idle_timeout_sec = max(0.05, float(self.get_parameter("idle_timeout_sec").value))
        self._head_step_rad = max(0.01, float(self.get_parameter("head_step_rad").value))
        self._head_duration_sec = max(0.10, float(self.get_parameter("head_duration_sec").value))
        self._head_reset = HeadState(
            h1=float(self.get_parameter("head_reset_h1").value),
            h2=float(self.get_parameter("head_reset_h2").value),
        )
        self._head_min_rad = float(self.get_parameter("head_min_rad").value)
        self._head_max_rad = float(self.get_parameter("head_max_rad").value)

        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 20)
        self._head_pub = self.create_publisher(JointTrajectory, self._head_trajectory_topic, 10)
        self.create_subscription(JointState, self._joint_state_topic, self._on_joint_state, 20)

        self._current_head = HeadState(self._head_reset.h1, self._head_reset.h2)
        self._last_head_target = HeadState(self._head_reset.h1, self._head_reset.h2)
        self._has_head_state = False
        self._last_head_send_time = 0.0
        self._linear_sign = 0.0
        self._angular_sign = 0.0
        self._last_base_key_time = 0.0

        self.get_logger().info("建图键盘遥操作已启动。按 h 打印按键说明。")

    def _on_joint_state(self, msg: JointState) -> None:
        if "h1_joint" not in msg.name or "h2_joint" not in msg.name:
            return
        h1_index = msg.name.index("h1_joint")
        h2_index = msg.name.index("h2_joint")
        if h1_index >= len(msg.position) or h2_index >= len(msg.position):
            return
        self._current_head = HeadState(h1=float(msg.position[h1_index]), h2=float(msg.position[h2_index]))
        self._has_head_state = True

    def print_help(self) -> None:
        text = f"""
MediPick mapping teleop

Base keys:
  u i o    forward-left / forward / forward-right
  j k l    rotate-left / stop / rotate-right
  m , .    backward-right / backward / backward-left

Head keys:
  r / f    h1_joint + / -
  t / g    h2_joint + / -
  b        reset head to mapping pose

Speed keys:
  q / z    increase / decrease linear+angular speed
  w / x    increase / decrease linear speed only
  e / c    increase / decrease angular speed only

Other:
  h        print this help again
  Ctrl-C   quit

Current speed:
  linear={self._linear_speed:.2f} m/s
  angular={self._angular_speed:.2f} rad/s

Note:
  If head commands have no visible effect, start mapping with:
  bash scripts/run_gazebo_rtabmap_manual_mapping.sh hold_head:=false
"""
        print(text.strip())
        sys.stdout.flush()

    def _print_action(self, key: str, description: str) -> None:
        print(
            f"key={key} -> {description} | linear={self._linear_speed:.2f} m/s angular={self._angular_speed:.2f} rad/s"
        )
        sys.stdout.flush()

    def _publish_cmd_vel(self) -> None:
        twist = Twist()
        twist.linear.x = self._linear_sign * self._linear_speed
        twist.angular.z = self._angular_sign * self._angular_speed
        self._cmd_pub.publish(twist)

    def _set_base_motion(self, key: str) -> None:
        linear_sign, angular_sign, description = MOVE_BINDINGS[key]
        self._linear_sign = linear_sign
        self._angular_sign = angular_sign
        self._last_base_key_time = self.get_clock().now().nanoseconds / 1e9
        self._publish_cmd_vel()
        self._print_action(
            key,
            f"{description}, cmd_vel=({self._linear_sign * self._linear_speed:.2f}, {self._angular_sign * self._angular_speed:.2f})",
        )

    def _update_speed(self, key: str) -> None:
        linear_scale, angular_scale, description = SPEED_BINDINGS[key]
        self._linear_speed = max(0.02, self._linear_speed * linear_scale)
        self._angular_speed = max(0.05, self._angular_speed * angular_scale)
        self._print_action(key, description)

    def _head_command_base(self) -> HeadState:
        now = self.get_clock().now().nanoseconds / 1e9
        if self._has_head_state and (now - self._last_head_send_time) > 0.8:
            return HeadState(self._current_head.h1, self._current_head.h2)
        return HeadState(self._last_head_target.h1, self._last_head_target.h2)

    def _clamp_head(self, value: float) -> float:
        return max(self._head_min_rad, min(self._head_max_rad, value))

    def _publish_head_target(self, key: str, h1: float, h2: float, description: str) -> None:
        trajectory = JointTrajectory()
        trajectory.joint_names = ["h1_joint", "h2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [h1, h2]
        total_ns = int(self._head_duration_sec * 1e9)
        point.time_from_start.sec = total_ns // 1_000_000_000
        point.time_from_start.nanosec = total_ns % 1_000_000_000
        trajectory.points = [point]
        self._head_pub.publish(trajectory)
        self._last_head_target = HeadState(h1=h1, h2=h2)
        self._last_head_send_time = self.get_clock().now().nanoseconds / 1e9
        self._print_action(key, f"{description}, target=(h1={h1:.3f}, h2={h2:.3f})")

    def _handle_head_key(self, key: str) -> None:
        base = self._head_command_base()
        if key == "r":
            target = HeadState(h1=self._clamp_head(base.h1 + self._head_step_rad), h2=base.h2)
            self._publish_head_target(key, target.h1, target.h2, f"h1_joint += {self._head_step_rad:.2f} rad")
        elif key == "f":
            target = HeadState(h1=self._clamp_head(base.h1 - self._head_step_rad), h2=base.h2)
            self._publish_head_target(key, target.h1, target.h2, f"h1_joint -= {self._head_step_rad:.2f} rad")
        elif key == "t":
            target = HeadState(h1=base.h1, h2=self._clamp_head(base.h2 + self._head_step_rad))
            self._publish_head_target(key, target.h1, target.h2, f"h2_joint += {self._head_step_rad:.2f} rad")
        elif key == "g":
            target = HeadState(h1=base.h1, h2=self._clamp_head(base.h2 - self._head_step_rad))
            self._publish_head_target(key, target.h1, target.h2, f"h2_joint -= {self._head_step_rad:.2f} rad")
        elif key == "b":
            self._publish_head_target(
                key,
                self._head_reset.h1,
                self._head_reset.h2,
                "reset head to mapping pose",
            )

    def _stop_if_idle(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if (self._linear_sign != 0.0 or self._angular_sign != 0.0) and (
            now - self._last_base_key_time > self._idle_timeout_sec
        ):
            self._linear_sign = 0.0
            self._angular_sign = 0.0
            self._publish_cmd_vel()


def read_key(timeout_sec: float) -> str:
    ready, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    if not ready:
        return ""
    return sys.stdin.read(1)


def main() -> None:
    if not sys.stdin.isatty():
        print("mapping_keyboard_teleop.py requires a TTY.", file=sys.stderr)
        raise SystemExit(1)

    rclpy.init()
    node = MappingKeyboardTeleop()
    settings = termios.tcgetattr(sys.stdin)
    timeout_sec = 1.0 / node._repeat_rate_hz

    try:
        tty.setraw(sys.stdin.fileno())
        node.print_help()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key(timeout_sec)
            if key == "\x03":
                break
            if key in MOVE_BINDINGS:
                node._set_base_motion(key)
            elif key in SPEED_BINDINGS:
                node._update_speed(key)
            elif key in {"r", "f", "t", "g", "b"}:
                node._handle_head_key(key)
            elif key == "h":
                node.print_help()
            elif key:
                node._print_action(key, "unmapped key")
            node._stop_if_idle()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        stop_twist = Twist()
        node._cmd_pub.publish(stop_twist)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
