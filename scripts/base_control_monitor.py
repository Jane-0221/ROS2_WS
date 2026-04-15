#!/usr/bin/env python3

from __future__ import annotations

import datetime as dt
import os
import signal
import sys
from dataclasses import dataclass
from typing import Optional

from geometry_msgs.msg import Twist
from std_msgs.msg import String
import rclpy
from rclpy.node import Node


TOPIC_ROWS = (
    ("nav", "/cmd_vel"),
    ("stm32", "/medipick/hardware/stm32_base_cmd_vel"),
    ("app", "/medipick/app/cmd_vel"),
    ("mux", "/medipick/base/cmd_vel_muxed"),
)


@dataclass
class TwistSnapshot:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    count: int = 0
    last_wall_time: Optional[float] = None

    def update(self, message: Twist, wall_time: float) -> None:
        self.vx = float(message.linear.x)
        self.vy = float(message.linear.y)
        self.wz = float(message.angular.z)
        self.count += 1
        self.last_wall_time = wall_time


class BaseControlMonitor(Node):
    def __init__(self) -> None:
        super().__init__("medipick_base_control_monitor")

        self._rows = {name: TwistSnapshot() for name, _ in TOPIC_ROWS}
        self._active_mode = "unknown"
        self._start_wall_time = self._now()

        for row_name, topic_name in TOPIC_ROWS:
            self.create_subscription(
                Twist,
                topic_name,
                self._make_twist_callback(row_name),
                20,
            )

        self.create_subscription(
            String,
            "/medipick/base/active_mode",
            self._active_mode_callback,
            10,
        )
        self.create_timer(0.2, self._render)

    @staticmethod
    def _now() -> float:
        return dt.datetime.now().timestamp()

    def _make_twist_callback(self, row_name: str):
        def callback(message: Twist) -> None:
            self._rows[row_name].update(message, self._now())

        return callback

    def _active_mode_callback(self, message: String) -> None:
        self._active_mode = message.data.strip() or "unknown"

    def _format_age(self, snapshot: TwistSnapshot, now: float) -> str:
        if snapshot.last_wall_time is None:
            return "waiting"
        age = now - snapshot.last_wall_time
        return f"{age:6.2f}s"

    @staticmethod
    def _format_value(value: float) -> str:
        return f"{value:8.3f}"

    def _render(self) -> None:
        now = self._now()
        uptime = now - self._start_wall_time
        current_time = dt.datetime.fromtimestamp(now).strftime("%Y-%m-%d %H:%M:%S")
        lines = [
            "\033[2J\033[H",
            "MediPick 底盘控制统一监控",
            f"时间: {current_time}    运行时长: {uptime:6.1f}s    当前模式: {self._active_mode}",
            "",
            "链路      Topic                                     linear.x   linear.y  angular.z   Age      Count",
            "-" * 104,
        ]

        for row_name, topic_name in TOPIC_ROWS:
            snapshot = self._rows[row_name]
            lines.append(
                f"{row_name:<8}{topic_name:<42}"
                f"{self._format_value(snapshot.vx)}"
                f"{self._format_value(snapshot.vy)}"
                f"{self._format_value(snapshot.wz)}   "
                f"{self._format_age(snapshot, now):>7}   "
                f"{snapshot.count:6d}"
            )

        lines.extend(
            [
                "",
                "说明:",
                "  nav   = 导航 /cmd_vel 输入",
                "  stm32 = STM32 上报码输入",
                "  app   = 手机 App 输入",
                "  mux   = 经过 base_cmd_mux 后送到底盘前的最终输出",
                "",
                "按 Ctrl+C 退出",
            ]
        )

        sys.stdout.write("\n".join(lines))
        sys.stdout.flush()


def main() -> None:
    rclpy.init()
    node = BaseControlMonitor()

    def handle_signal(signum, frame):  # noqa: ARG001
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
