#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

from geometry_msgs.msg import Twist
from medipick_planning_interfaces.srv import SetBaseControlMode
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String


ALLOWED_MODES = ("nav", "stm32", "app")
ZERO_CMD = (0.0, 0.0, 0.0)


class BaseCmdMux(Node):
    def __init__(self) -> None:
        super().__init__("medipick_base_cmd_mux")

        self.declare_parameter("default_mode", "nav")
        self.declare_parameter("nav_cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("stm32_cmd_vel_topic", "/medipick/hardware/stm32_base_cmd_vel")
        self.declare_parameter("app_cmd_vel_topic", "/medipick/app/cmd_vel")
        self.declare_parameter("output_topic", "/medipick/base/cmd_vel_muxed")
        self.declare_parameter("active_mode_topic", "/medipick/base/active_mode")
        self.declare_parameter("mode_service", "/medipick/base/set_control_mode")
        self.declare_parameter("input_timeout_sec", 0.5)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("stm32_identical_nonzero_cmd_guard_timeout", 2.0)
        self.declare_parameter("stm32_identical_nonzero_cmd_guard_epsilon", 1e-4)

        default_mode = str(self.get_parameter("default_mode").value).strip()
        if default_mode not in ALLOWED_MODES:
            raise ValueError(f"default_mode must be one of {ALLOWED_MODES}, got '{default_mode}'")

        self._active_mode = default_mode
        self._input_timeout = Duration(seconds=float(self.get_parameter("input_timeout_sec").value))
        self._guard_timeout = float(self.get_parameter("stm32_identical_nonzero_cmd_guard_timeout").value)
        self._guard_epsilon = float(self.get_parameter("stm32_identical_nonzero_cmd_guard_epsilon").value)
        self._last_cmds = {mode: ZERO_CMD for mode in ALLOWED_MODES}
        self._last_stamps = {}
        self._last_published_cmd = ZERO_CMD

        self._guard_active_cmd = None
        self._guard_active_since = None
        self._guard_latched_cmd = None

        self._publisher = self.create_publisher(
            Twist,
            str(self.get_parameter("output_topic").value),
            20,
        )
        self._mode_publisher = self.create_publisher(
            String,
            str(self.get_parameter("active_mode_topic").value),
            10,
        )

        self.create_subscription(
            Twist,
            str(self.get_parameter("nav_cmd_vel_topic").value),
            lambda message: self._handle_cmd("nav", message),
            20,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("stm32_cmd_vel_topic").value),
            lambda message: self._handle_cmd("stm32", message),
            20,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("app_cmd_vel_topic").value),
            lambda message: self._handle_cmd("app", message),
            20,
        )
        self.create_service(
            SetBaseControlMode,
            str(self.get_parameter("mode_service").value),
            self._set_mode,
        )

        publish_rate = max(5.0, float(self.get_parameter("publish_rate").value))
        self.create_timer(1.0 / publish_rate, self._publish_active_cmd)
        self.create_timer(1.0, self._publish_active_mode)

        self.get_logger().info(
            "Base cmd mux ready. mode=%s, nav=%s, stm32=%s, app=%s, out=%s"
            % (
                self._active_mode,
                self.get_parameter("nav_cmd_vel_topic").value,
                self.get_parameter("stm32_cmd_vel_topic").value,
                self.get_parameter("app_cmd_vel_topic").value,
                self.get_parameter("output_topic").value,
            )
        )
        self._publish_active_mode()

    def _handle_cmd(self, mode: str, message: Twist) -> None:
        self._last_cmds[mode] = self._cmd_to_tuple(message)
        self._last_stamps[mode] = self.get_clock().now()

    def _set_mode(self, request: SetBaseControlMode.Request, response: SetBaseControlMode.Response):
        mode = request.mode.strip().lower()
        if mode not in ALLOWED_MODES:
            response.success = False
            response.message = f"Unsupported base control mode '{request.mode}'. Expected one of {ALLOWED_MODES}."
            response.active_mode = self._active_mode
            return response

        if mode == self._active_mode:
            response.success = True
            response.message = f"Base control mode is already '{mode}'."
            response.active_mode = self._active_mode
            return response

        previous_mode = self._active_mode
        self._active_mode = mode
        self._clear_stm32_guard()
        self._publish_cmd(ZERO_CMD)
        self._publish_active_mode()

        response.success = True
        response.message = f"Switched base control mode from '{previous_mode}' to '{mode}'."
        response.active_mode = self._active_mode
        self.get_logger().info(response.message)
        return response

    def _publish_active_cmd(self) -> None:
        now = self.get_clock().now()
        stamp = self._last_stamps.get(self._active_mode)
        if stamp is None or now - stamp > self._input_timeout:
            if self._active_mode == "stm32":
                self._clear_stm32_guard()
            self._publish_cmd(ZERO_CMD)
            return

        command = self._last_cmds.get(self._active_mode, ZERO_CMD)
        if self._active_mode == "stm32":
            command = self._apply_stm32_guard(command, now)
        else:
            self._clear_stm32_guard()

        self._publish_cmd(command)

    def _publish_active_mode(self) -> None:
        message = String()
        message.data = self._active_mode
        self._mode_publisher.publish(message)

    def _publish_cmd(self, command: tuple[float, float, float]) -> None:
        message = Twist()
        message.linear.x = float(command[0])
        message.linear.y = float(command[1])
        message.angular.z = float(command[2])
        self._publisher.publish(message)
        self._last_published_cmd = command

    def _apply_stm32_guard(self, command: tuple[float, float, float], now) -> tuple[float, float, float]:
        if self._is_zero_cmd(command):
            if self._guard_latched_cmd is not None:
                self.get_logger().info("STM32 identical non-zero guard released by zero command.")
            self._clear_stm32_guard()
            return ZERO_CMD

        if self._guard_latched_cmd is not None:
            if self._same_cmd(command, self._guard_latched_cmd):
                return ZERO_CMD
            self.get_logger().info("STM32 identical non-zero guard released by a new command.")
            self._clear_stm32_guard()

        if not self._same_cmd(command, self._guard_active_cmd):
            self._guard_active_cmd = command
            self._guard_active_since = now
            return command

        if self._guard_active_since is None:
            self._guard_active_since = now
            return command

        elapsed_sec = (now - self._guard_active_since).nanoseconds / 1e9
        if elapsed_sec < self._guard_timeout:
            return command

        self._guard_latched_cmd = command
        self._guard_active_cmd = None
        self._guard_active_since = None
        self.get_logger().error(
            "Identical STM32 non-zero command persisted for %.2fs, forcing stop: %s"
            % (elapsed_sec, self._format_cmd(command))
        )
        return ZERO_CMD

    def _clear_stm32_guard(self) -> None:
        self._guard_active_cmd = None
        self._guard_active_since = None
        self._guard_latched_cmd = None

    def _is_zero_cmd(self, command: tuple[float, float, float]) -> bool:
        return all(abs(value) <= self._guard_epsilon for value in command)

    def _same_cmd(
        self,
        left: Optional[tuple[float, float, float]],
        right: Optional[tuple[float, float, float]],
    ) -> bool:
        if left is None or right is None:
            return False
        return all(abs(left_value - right_value) <= self._guard_epsilon for left_value, right_value in zip(left, right))

    @staticmethod
    def _cmd_to_tuple(message: Twist) -> tuple[float, float, float]:
        return (float(message.linear.x), float(message.linear.y), float(message.angular.z))

    @staticmethod
    def _format_cmd(command: tuple[float, float, float]) -> str:
        return f"vx={command[0]:.4f} m/s, vy={command[1]:.4f} m/s, wz={command[2]:.4f} rad/s"


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = BaseCmdMux()
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
