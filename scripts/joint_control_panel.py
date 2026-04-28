#!/usr/bin/env python3

from __future__ import annotations

import math
import os
import threading
import tkinter as tk
import xml.etree.ElementTree as ET
from pathlib import Path
from tkinter import messagebox, ttk

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

CONTROLLED_JOINTS = [
    "base_x",
    "base_y",
    "base_theta",
    "raise_joint",
    "r1_joint",
    "r2_joint",
    "r3_joint",
    "r4_joint",
    "r5_joint",
    "r6_joint",
    "sucker_joint",
    "h1_joint",
    "h2_joint",
]

DEFAULT_LIMITS = {
    "base_x": (-3.0, 3.0),
    "base_y": (-3.0, 3.0),
    "base_theta": (-math.pi, math.pi),
    "raise_joint": (0.0, 1.5),
    "r1_joint": (-2.0 * math.pi, 2.0 * math.pi),
    "r2_joint": (-2.0 * math.pi, 2.0 * math.pi),
    "r3_joint": (-2.0 * math.pi, 2.0 * math.pi),
    "r4_joint": (-2.0 * math.pi, 2.0 * math.pi),
    "r5_joint": (-2.0 * math.pi, 2.0 * math.pi),
    "r6_joint": (-2.0 * math.pi, 2.0 * math.pi),
    "sucker_joint": (-2.0 * math.pi, 2.0 * math.pi),
    "h1_joint": (-math.pi, math.pi),
    "h2_joint": (-math.pi, math.pi),
}

GROUPS = {
    "head": {
        "title": "Head",
        "controller": "head_controller",
        "joints": ["h1_joint", "h2_joint"],
        "default_duration": 1.0,
    },
    "base": {
        "title": "Base",
        "controller": "mobile_arm_controller",
        "joints": ["base_x", "base_y", "base_theta"],
        "default_duration": 4.0,
    },
    "arm": {
        "title": "Lift + Arm",
        "controller": "mobile_arm_controller",
        "joints": ["raise_joint", "r1_joint", "r2_joint", "r3_joint", "r4_joint", "r5_joint", "r6_joint"],
        "default_duration": 8.0,
    },
    "tool": {
        "title": "Tool",
        "controller": "tool_controller",
        "joints": ["sucker_joint"],
        "default_duration": 1.0,
    },
}

ACTION_SERVER_WAIT_SEC = 2.0


def locate_urdf() -> Path | None:
    try:
        package_share = Path(get_package_share_directory("medipick_simple3_description"))
        candidate = package_share / "urdf" / "simple3_moveit.urdf"
        if candidate.is_file():
            return candidate
    except PackageNotFoundError:
        pass

    workspace_candidate = Path(__file__).resolve().parents[1] / "src" / "medipick_simple3_description" / "urdf" / "simple3_moveit.urdf"
    if workspace_candidate.is_file():
        return workspace_candidate
    return None


def load_joint_limits() -> dict[str, tuple[float, float]]:
    limits = dict(DEFAULT_LIMITS)
    urdf_path = locate_urdf()
    if urdf_path is None:
        return limits

    try:
        root = ET.parse(urdf_path).getroot()
    except ET.ParseError:
        return limits

    for joint in root.findall("joint"):
        name = joint.attrib.get("name", "")
        if name not in limits:
            continue
        limit = joint.find("limit")
        if limit is None:
            continue
        lower = limit.attrib.get("lower")
        upper = limit.attrib.get("upper")
        if lower is None or upper is None:
            continue
        try:
            limits[name] = (float(lower), float(upper))
        except ValueError:
            continue
    return limits


def split_duration(duration: float) -> tuple[int, int]:
    clipped = max(0.1, float(duration))
    sec = int(clipped)
    nanosec = int((clipped - sec) * 1_000_000_000)
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return sec, nanosec


def slider_resolution(joint_name: str) -> float:
    if joint_name in {"base_x", "base_y", "raise_joint"}:
        return 0.001
    return 0.01


class JointControlNode(Node):
    def __init__(self) -> None:
        super().__init__("medipick_joint_control_panel")
        self._lock = threading.Lock()
        self.current_positions: dict[str, float] = {}
        self.status_messages: list[str] = []
        self._goals_in_flight = {
            "mobile_arm_controller": False,
            "head_controller": False,
            "tool_controller": False,
        }
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        self._action_clients = {
            "mobile_arm_controller": ActionClient(
                self, FollowJointTrajectory, "/mobile_arm_controller/follow_joint_trajectory"
            ),
            "head_controller": ActionClient(
                self, FollowJointTrajectory, "/head_controller/follow_joint_trajectory"
            ),
            "tool_controller": ActionClient(
                self, FollowJointTrajectory, "/tool_controller/follow_joint_trajectory"
            ),
        }

    def _on_joint_state(self, msg: JointState) -> None:
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                self.current_positions[name] = position

    def pop_status_messages(self) -> list[str]:
        with self._lock:
            messages = self.status_messages[:]
            self.status_messages.clear()
        return messages

    def current_positions_snapshot(self) -> dict[str, float]:
        with self._lock:
            return dict(self.current_positions)

    def _push_status(self, message: str) -> None:
        with self._lock:
            self.status_messages.append(message)
        self.get_logger().info(message)

    def _auto_duration(
        self,
        controller_name: str,
        joint_names: list[str],
        positions: list[float],
        requested_duration: float,
    ) -> float:
        with self._lock:
            current = [self.current_positions.get(joint_name) for joint_name in joint_names]

        known_deltas = [
            abs(target - current_value)
            for target, current_value in zip(positions, current)
            if current_value is not None
        ]
        if not known_deltas:
            return requested_duration

        max_delta = max(known_deltas)
        if controller_name == "head_controller":
            recommended = 1.2 + 1.6 * max_delta
        elif controller_name == "tool_controller":
            recommended = 0.8 + 0.4 * max_delta
        else:
            recommended = 4.0 + 4.0 * max_delta
        return max(requested_duration, recommended)

    def send_goal(
        self,
        controller_name: str,
        joint_names: list[str],
        positions: list[float],
        duration: float,
        label: str,
    ) -> bool:
        client = self._action_clients.get(controller_name)
        if client is None:
            self._push_status(f"{label}: 未知控制器 {controller_name}")
            return False

        with self._lock:
            if self._goals_in_flight.get(controller_name, False):
                self.status_messages.append(f"{label}: {controller_name} 正在执行上一个目标，请等它完成。")
                latest_message = self.status_messages[-1]
            else:
                latest_message = ""
        if latest_message:
            self.get_logger().info(latest_message)
            return False

        if not client.wait_for_server(timeout_sec=ACTION_SERVER_WAIT_SEC):
            self._push_status(
                f"{label}: 控制器 action 未就绪：{controller_name}。"
                " 这条全开链里 mobile_arm_controller 可能会比 head_controller 晚一会儿激活。"
            )
            return False

        duration = self._auto_duration(controller_name, joint_names, positions, duration)
        sec, nanosec = split_duration(duration)
        trajectory = JointTrajectory()
        trajectory.joint_names = list(joint_names)

        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        with self._lock:
            self._goals_in_flight[controller_name] = True
        future = client.send_goal_async(goal)
        future.add_done_callback(
            lambda result, used_label=label, used_controller=controller_name: self._on_goal_response(
                used_label,
                used_controller,
                result,
            )
        )
        self._push_status(
            f"{label}: 已发送到 {controller_name}，关节={joint_names}，duration={duration:.2f}s"
        )
        return True

    def _clear_goal_in_flight(self, controller_name: str) -> None:
        with self._lock:
            self._goals_in_flight[controller_name] = False

    def _on_goal_response(self, label: str, controller_name: str, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._clear_goal_in_flight(controller_name)
            self._push_status(f"{label}: 发送失败: {exc}")
            return

        if goal_handle is None or not goal_handle.accepted:
            self._clear_goal_in_flight(controller_name)
            self._push_status(f"{label}: 控制器拒绝了目标。")
            return

        self._push_status(f"{label}: 控制器已接受目标。")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, used_label=label, used_controller=controller_name: self._on_goal_result(
                used_label,
                used_controller,
                result,
            )
        )

    def _on_goal_result(self, label: str, controller_name: str, future) -> None:
        try:
            wrapped_result = future.result()
        except Exception as exc:  # noqa: BLE001
            self._clear_goal_in_flight(controller_name)
            self._push_status(f"{label}: 结果获取失败: {exc}")
            return

        self._clear_goal_in_flight(controller_name)
        result = wrapped_result.result
        status = wrapped_result.status
        error_code = getattr(result, "error_code", None)
        error_string = getattr(result, "error_string", "")
        if status == 4 or error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self._push_status(f"{label}: 执行完成。")
            return

        details = error_string or f"status={status}, error_code={error_code}"
        self._push_status(f"{label}: 执行失败: {details}")


class JointControlPanel:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("MediPick Joint Control Panel")
        self.root.geometry("1080x760")

        self.joint_limits = load_joint_limits()

        rclpy.init(args=None)
        self.node = JointControlNode()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._shutdown_event = threading.Event()
        self._executor_thread = threading.Thread(target=self._spin_executor, daemon=True)
        self._executor_thread.start()

        self.slider_vars: dict[str, tk.DoubleVar] = {}
        self.current_labels: dict[str, ttk.Label] = {}
        self.duration_vars: dict[str, tk.StringVar] = {}
        self.status_var = tk.StringVar(value="等待 /joint_states 和控制器...")
        self._closing = False
        self._poll_after_id: str | None = None

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._poll_after_id = self.root.after(60, self._poll_ros)

    def _build_ui(self) -> None:
        container = ttk.Frame(self.root, padding=10)
        container.pack(fill=tk.BOTH, expand=True)

        top_bar = ttk.Frame(container)
        top_bar.pack(fill=tk.X, pady=(0, 8))

        ttk.Button(top_bar, text="Load All From Current", command=self._load_all_from_current).pack(
            side=tk.LEFT, padx=(0, 6)
        )
        ttk.Button(top_bar, text="Load Head Forward Preset", command=self._load_head_forward_preset).pack(
            side=tk.LEFT, padx=(0, 6)
        )
        ttk.Button(top_bar, text="Send Head Forward", command=self._send_head_forward).pack(
            side=tk.LEFT, padx=(0, 6)
        )
        ttk.Button(top_bar, text="Load Debug Home Preset", command=self._load_debug_home_preset).pack(
            side=tk.LEFT, padx=(0, 6)
        )
        ttk.Button(top_bar, text="Send Debug Home", command=self._send_debug_home).pack(side=tk.LEFT)

        notebook = ttk.Notebook(container)
        notebook.pack(fill=tk.BOTH, expand=True)

        for group_key, config in GROUPS.items():
            frame = ttk.Frame(notebook, padding=10)
            notebook.add(frame, text=config["title"])
            self._build_group_tab(frame, group_key, config)

        status_frame = ttk.LabelFrame(container, text="Status", padding=8)
        status_frame.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(status_frame, textvariable=self.status_var, anchor="w").pack(fill=tk.X)

    def _build_group_tab(self, parent: ttk.Frame, group_key: str, config: dict) -> None:
        toolbar = ttk.Frame(parent)
        toolbar.pack(fill=tk.X, pady=(0, 8))

        self.duration_vars[group_key] = tk.StringVar(value=f"{config['default_duration']:.1f}")
        ttk.Label(toolbar, text="Duration (s)").pack(side=tk.LEFT)
        ttk.Entry(toolbar, width=8, textvariable=self.duration_vars[group_key]).pack(side=tk.LEFT, padx=(6, 12))
        ttk.Button(
            toolbar,
            text="Load Current",
            command=lambda joints=config["joints"]: self._load_group_from_current(joints),
        ).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(
            toolbar,
            text=f"Send {config['title']}",
            command=lambda key=group_key: self._send_group(key),
        ).pack(side=tk.LEFT)

        grid = ttk.Frame(parent)
        grid.pack(fill=tk.BOTH, expand=True)
        grid.columnconfigure(2, weight=1)

        ttk.Label(grid, text="Joint", width=12).grid(row=0, column=0, sticky="w")
        ttk.Label(grid, text="Current", width=12).grid(row=0, column=1, sticky="w")
        ttk.Label(grid, text="Target", width=12).grid(row=0, column=2, sticky="w")
        ttk.Label(grid, text="Value", width=10).grid(row=0, column=3, sticky="w")

        for row_index, joint_name in enumerate(config["joints"], start=1):
            lower, upper = self.joint_limits.get(joint_name, DEFAULT_LIMITS[joint_name])
            self.slider_vars[joint_name] = tk.DoubleVar(value=0.0)

            ttk.Label(grid, text=joint_name).grid(row=row_index, column=0, sticky="w", padx=(0, 8), pady=3)
            current_label = ttk.Label(grid, text="--", width=12)
            current_label.grid(row=row_index, column=1, sticky="w", padx=(0, 8), pady=3)
            self.current_labels[joint_name] = current_label

            scale = tk.Scale(
                grid,
                variable=self.slider_vars[joint_name],
                from_=lower,
                to=upper,
                orient=tk.HORIZONTAL,
                resolution=slider_resolution(joint_name),
                length=520,
                showvalue=False,
            )
            scale.grid(row=row_index, column=2, sticky="ew", padx=(0, 8), pady=3)
            ttk.Entry(grid, width=10, textvariable=self.slider_vars[joint_name]).grid(
                row=row_index, column=3, sticky="w", pady=3
            )

    def _get_duration(self, group_key: str) -> float:
        try:
            return max(0.1, float(self.duration_vars[group_key].get().strip()))
        except ValueError as exc:
            raise ValueError(f"{GROUPS[group_key]['title']} 的 duration 不是有效数字。") from exc

    def _load_group_from_current(self, joints: list[str]) -> None:
        for joint_name in joints:
            if joint_name in self.node.current_positions:
                self.slider_vars[joint_name].set(self.node.current_positions[joint_name])
        self.status_var.set(f"已把当前 joint 状态载入滑条：{', '.join(joints)}")

    def _load_all_from_current(self) -> None:
        self._load_group_from_current(CONTROLLED_JOINTS)

    def _load_head_forward_preset(self) -> None:
        self.slider_vars["h1_joint"].set(0.0)
        self.slider_vars["h2_joint"].set(-0.5236)
        self.status_var.set("已载入 head_forward 预设。")

    def _load_debug_home_preset(self) -> None:
        preset = {
            "base_x": 0.0,
            "base_y": 0.0,
            "base_theta": 0.0,
            "raise_joint": 0.0,
            "r1_joint": 0.0,
            "r2_joint": 0.0,
            "r3_joint": 0.0,
            "r4_joint": 0.0,
            "r5_joint": 0.0,
            "r6_joint": 0.0,
            "sucker_joint": 0.0,
            "h1_joint": 0.0,
            "h2_joint": -0.5236,
        }
        for joint_name, value in preset.items():
            self.slider_vars[joint_name].set(value)
        self.status_var.set("已载入 debug_home 预设。")

    def _send_group(self, group_key: str) -> None:
        config = GROUPS[group_key]
        try:
            duration = self._get_duration(group_key)
        except ValueError as exc:
            messagebox.showerror("Invalid Duration", str(exc))
            return

        joint_names = list(config["joints"])
        positions = [float(self.slider_vars[joint_name].get()) for joint_name in joint_names]
        sent = self.node.send_goal(config["controller"], joint_names, positions, duration, config["title"])
        if sent:
            self.status_var.set(f"{config['title']} 目标已发送。")
        else:
            self.status_var.set(f"{config['title']} 发送失败，请看状态栏。")

    def _send_head_forward(self) -> None:
        self._load_head_forward_preset()
        self._send_group("head")

    def _send_debug_home(self) -> None:
        self._load_debug_home_preset()
        self._send_group("head")
        self._send_group("base")
        self._send_group("arm")
        self._send_group("tool")

    def _spin_executor(self) -> None:
        while not self._shutdown_event.is_set() and rclpy.ok():
            try:
                self.executor.spin_once(timeout_sec=0.1)
            except Exception as exc:  # noqa: BLE001
                self.node._push_status(f"ROS executor 异常: {exc}")

    def _poll_ros(self) -> None:
        if self._closing:
            self._poll_after_id = None
            return

        current_positions = self.node.current_positions_snapshot()
        for joint_name, label in self.current_labels.items():
            value = current_positions.get(joint_name)
            if value is not None:
                label.configure(text=f"{value:.4f}")

        messages = self.node.pop_status_messages()
        if messages:
            self.status_var.set(messages[-1])

        self._poll_after_id = self.root.after(60, self._poll_ros)

    def _on_close(self) -> None:
        if self._closing:
            return

        self._closing = True
        if self._poll_after_id is not None:
            try:
                self.root.after_cancel(self._poll_after_id)
            except Exception:  # noqa: BLE001
                pass
            self._poll_after_id = None

        self.status_var.set("正在关闭控制器 GUI...")
        self.root.update_idletasks()
        self.root.withdraw()
        self._shutdown_event.set()
        try:
            self.executor.shutdown(timeout_sec=0.5)
        except Exception:  # noqa: BLE001
            pass
        if self._executor_thread.is_alive():
            self._executor_thread.join(timeout=1.0)
        try:
            self.executor.remove_node(self.node)
        except Exception:  # noqa: BLE001
            pass
        try:
            self.node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
        try:
            self.root.quit()
        except Exception:  # noqa: BLE001
            pass
        try:
            self.root.destroy()
        except Exception:  # noqa: BLE001
            pass


def main() -> None:
    root = tk.Tk()
    JointControlPanel(root)
    try:
        root.mainloop()
    finally:
        os._exit(0)


if __name__ == "__main__":
    main()
