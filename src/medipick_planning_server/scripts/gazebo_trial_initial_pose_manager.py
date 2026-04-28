#!/usr/bin/python3

from __future__ import annotations

import math
import re
import time
from pathlib import Path
from xml.etree import ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pick_task_shared import DEFAULT_STOW_STATE_POSITIONS


DEFAULT_CONTROLLER_JOINTS = (
    "base_x",
    "base_y",
    "base_theta",
)

TRIAL_BASE_JOINTS = (
    "base_x",
    "base_y",
    "base_theta",
)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def current_positions_for(joint_names: tuple[str, ...], current_joint_state: JointState, defaults: dict[str, float]) -> list[float]:
    current_positions = {name: pos for name, pos in zip(current_joint_state.name, current_joint_state.position)}
    return [current_positions.get(name, defaults.get(name, 0.0)) for name in joint_names]


class GazeboTrialInitialPoseManager(Node):
    def __init__(self) -> None:
        super().__init__("medipick_gazebo_trial_initial_pose_manager")

        self.declare_parameter("target_pose_topic", "/medipick/task/target_pose")
        self.declare_parameter(
            "world_file",
            str(
                Path(get_package_share_directory("medipick_simple3_description"))
                / "worlds"
                / "medipick_pharmacy_textured.world.sdf"
            ),
        )
        self.declare_parameter("target_seed", 0)
        self.declare_parameter("target_entity_name", "")
        self.declare_parameter("target_contact_inset", 0.002)
        self.declare_parameter("controller_action_name", "/base_controller/follow_joint_trajectory")
        self.declare_parameter("controller_name", "base_controller")
        self.declare_parameter("controller_joint_names", list(DEFAULT_CONTROLLER_JOINTS))
        self.declare_parameter("controller_manager_list_service", "/controller_manager/list_controllers")
        self.declare_parameter("task_start_service", "/medipick/task/start")
        self.declare_parameter("initialization_ready_topic", "/medipick/task/initialization_ready")
        self.declare_parameter("base_offset", 0.95)
        self.declare_parameter("base_lateral_offset", 0.0)
        self.declare_parameter("base_yaw_offset_deg", 0.0)
        self.declare_parameter("min_motion_duration", 14.0)
        self.declare_parameter("base_linear_speed", 0.20)
        self.declare_parameter("base_angular_speed", 0.35)
        self.declare_parameter("arm_joint_speed", 0.45)
        self.declare_parameter("motion_duration_scale", 2.5)
        self.declare_parameter("motion_duration_margin", 1.0)
        self.declare_parameter("post_arrival_settle_time", 1.5)
        self.declare_parameter("auto_start_task", True)
        self.declare_parameter("stow_arm_during_initialization", False)
        self.declare_parameter("navigation_route_mode", "central_corridor")
        self.declare_parameter("navigation_corridor_y", 0.0)
        self.declare_parameter("navigation_segment_translation_limit_m", 0.30)
        self.declare_parameter("navigation_segment_rotation_limit_deg", 20.0)
        self.declare_parameter("translate_then_rotate", True)

        self._base_offset = float(self.get_parameter("base_offset").value)
        self._base_lateral_offset = float(self.get_parameter("base_lateral_offset").value)
        self._base_yaw_offset = math.radians(float(self.get_parameter("base_yaw_offset_deg").value))
        self._min_motion_duration = float(self.get_parameter("min_motion_duration").value)
        self._base_linear_speed = max(0.05, float(self.get_parameter("base_linear_speed").value))
        self._base_angular_speed = max(0.05, float(self.get_parameter("base_angular_speed").value))
        self._arm_joint_speed = max(0.05, float(self.get_parameter("arm_joint_speed").value))
        self._motion_duration_scale = max(1.0, float(self.get_parameter("motion_duration_scale").value))
        self._motion_duration_margin = max(0.0, float(self.get_parameter("motion_duration_margin").value))
        self._post_arrival_settle_time = max(0.0, float(self.get_parameter("post_arrival_settle_time").value))
        self._auto_start_task = bool(self.get_parameter("auto_start_task").value)
        self._stow_arm_during_initialization = bool(self.get_parameter("stow_arm_during_initialization").value)
        self._navigation_route_mode = str(self.get_parameter("navigation_route_mode").value).strip()
        self._navigation_corridor_y = float(self.get_parameter("navigation_corridor_y").value)
        self._navigation_segment_translation_limit = max(
            0.10, float(self.get_parameter("navigation_segment_translation_limit_m").value)
        )
        self._navigation_segment_rotation_limit = math.radians(
            max(5.0, float(self.get_parameter("navigation_segment_rotation_limit_deg").value))
        )
        self._translate_then_rotate = bool(self.get_parameter("translate_then_rotate").value)
        self._world_file = Path(str(self.get_parameter("world_file").value))
        self._target_seed = int(self.get_parameter("target_seed").value)
        self._target_entity_name = str(self.get_parameter("target_entity_name").value).strip()
        self._target_contact_inset = float(self.get_parameter("target_contact_inset").value)
        self._controller_name = str(self.get_parameter("controller_name").value)
        self._controller_joint_names = tuple(
            str(joint_name)
            for joint_name in self.get_parameter("controller_joint_names").value
            if str(joint_name).strip()
        )
        if not self._controller_joint_names:
            self._controller_joint_names = DEFAULT_CONTROLLER_JOINTS

        self._current_joint_state = JointState()
        self._target_pose, self._target_name = self._select_target_pose(self._world_file)
        self._goal_completed = False
        self._goal_succeeded = False
        self._start_requested = False
        self._goal_completed_at = 0.0
        self._received_joint_state = False
        self._last_wait_log_time = 0.0
        self._controller_active = False
        self._controller_state_request_pending = False
        self._trajectory_in_flight = False
        self._stage_targets: list[dict[str, float]] = []
        self._stage_index = 0
        self._full_target_state: dict[str, float] | None = None

        target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        controller_action_name = str(self.get_parameter("controller_action_name").value)
        self._controller_action_name = controller_action_name
        controller_manager_list_service = str(self.get_parameter("controller_manager_list_service").value)
        task_start_service = str(self.get_parameter("task_start_service").value)
        initialization_ready_topic = str(self.get_parameter("initialization_ready_topic").value).strip()
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        self._trajectory_action = ActionClient(self, FollowJointTrajectory, controller_action_name)
        self._list_controllers_client = self.create_client(ListControllers, controller_manager_list_service)
        self._start_client = self.create_client(Trigger, task_start_service)
        self._initialization_ready_pub = self.create_publisher(Bool, initialization_ready_topic, latched_qos)
        self.create_timer(0.1, self._tick)
        self._publish_initialization_ready(False)

        self.get_logger().info("Gazebo 试验初始化器就绪，等待目标位姿后把机器人预放到目标货架附近。")
        yaw = quaternion_to_yaw(
            self._target_pose.pose.orientation.x,
            self._target_pose.pose.orientation.y,
            self._target_pose.pose.orientation.z,
            self._target_pose.pose.orientation.w,
        )
        self.get_logger().info(
            "已根据 world/seed 解析 Gazebo 抓取目标，准备计算初始化位姿："
            f" box={self._target_name}, "
            f"target=({self._target_pose.pose.position.x:.3f}, {self._target_pose.pose.position.y:.3f}, {self._target_pose.pose.position.z:.3f}), "
            f"yaw={math.degrees(yaw):.1f}deg, source_topic={target_pose_topic}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._current_joint_state = msg
        if not self._received_joint_state and len(msg.name) > 0:
            self._received_joint_state = True
            self.get_logger().info(
                f"初始化器已收到 joint_states，关节数={len(msg.name)}，准备等待控制器动作服务。"
            )

    def _parse_world(self, world_file: Path):
        root = ET.parse(world_file).getroot()
        includes = []
        for include in root.findall(".//include"):
            name = (include.findtext("name") or "").strip()
            uri = (include.findtext("uri") or "").strip()
            pose_text = (include.findtext("pose") or "0 0 0 0 0 0").strip()
            pose = tuple(map(float, pose_text.split()))
            includes.append({"name": name, "uri": uri, "pose": pose})
        return includes

    def _load_model_size(self, model_name: str) -> tuple[float, float, float]:
        sdf_path = (
            Path(get_package_share_directory("medipick_simple3_description"))
            / "models"
            / model_name
            / "model.sdf"
        )
        text = sdf_path.read_text(encoding="utf-8")
        match = re.search(r"<size>([0-9.eE+-]+)\s+([0-9.eE+-]+)\s+([0-9.eE+-]+)</size>", text)
        if not match:
            raise RuntimeError(f"无法从 {sdf_path} 读取药盒尺寸")
        return float(match.group(1)), float(match.group(2)), float(match.group(3))

    def _select_target_pose(self, world_file: Path) -> tuple[PoseStamped, str]:
        import random

        includes = self._parse_world(world_file)
        shelf_map: dict[str, dict] = {}
        drug_entities: list[dict] = []

        for item in includes:
            uri = item["uri"]
            if uri.startswith("model://textured_drug_"):
                drug_entities.append(item)
            elif uri.startswith("model://") and not uri.startswith("model://medipick"):
                shelf_map[item["name"]] = item

        if not drug_entities:
            raise RuntimeError("当前 world 中没有找到可作为目标的独立药盒实体")

        if self._target_entity_name:
            matches = [item for item in drug_entities if item["name"] == self._target_entity_name]
            if not matches:
                raise RuntimeError(f"指定目标药盒不存在：{self._target_entity_name}")
            selected = matches[0]
        else:
            rng = random.Random(self._target_seed)
            selected = rng.choice(drug_entities)

        entity_name = selected["name"]
        model_name = selected["uri"].replace("model://", "", 1)
        x, y, z, _roll, _pitch, _entity_yaw = selected["pose"]
        shelf_name = re.sub(r"_l\d+_s\d+$", "", entity_name)
        if shelf_name not in shelf_map:
            raise RuntimeError(f"找不到目标药盒对应的货架：{shelf_name}")
        _sx, _sy, _sz, _sroll, _spitch, shelf_yaw = shelf_map[shelf_name]["pose"]

        _width, depth, _height = self._load_model_size(model_name)
        inward_x = math.cos(shelf_yaw)
        inward_y = math.sin(shelf_yaw)
        target_x = x - inward_x * max(0.0, depth / 2.0 - self._target_contact_inset)
        target_y = y - inward_y * max(0.0, depth / 2.0 - self._target_contact_inset)

        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.pose.position.x = target_x
        target_pose.pose.position.y = target_y
        target_pose.pose.position.z = z
        yaw = shelf_yaw
        target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        return target_pose, entity_name

    def _tick(self) -> None:
        if self._goal_completed:
            if (
                self._goal_succeeded
                and
                self._auto_start_task
                and not self._start_requested
                and (time.monotonic() - self._goal_completed_at) >= self._post_arrival_settle_time
            ):
                self._request_task_start()
            return

        if self._trajectory_in_flight or self._target_pose is None:
            return

        now = time.monotonic()
        if len(self._current_joint_state.name) == 0:
            if now - self._last_wait_log_time >= 2.0:
                self._last_wait_log_time = now
                self.get_logger().info("初始化器仍在等待 /joint_states。")
            return

        if not self._trajectory_action.wait_for_server(timeout_sec=0.1):
            if now - self._last_wait_log_time >= 2.0:
                self._last_wait_log_time = now
                self.get_logger().info(f"初始化器仍在等待 {self._controller_action_name}。")
            return

        if not self._controller_active:
            self._request_controller_state()
            if now - self._last_wait_log_time >= 2.0:
                self._last_wait_log_time = now
                self.get_logger().info(
                    f"初始化器仍在等待控制器 {self._controller_name} 进入 active。"
                )
            return

        target_state = self._build_target_state(self._target_pose)
        self._full_target_state = dict(target_state)
        if not self._stage_targets:
            start_positions = current_positions_for(
                self._controller_joint_names,
                self._current_joint_state,
                target_state,
            )
            self._stage_targets = self._build_stage_targets(start_positions, target_state)
            self._stage_index = 0
            self.get_logger().info(
                f"Gazebo 试验前置导航将按 {self._navigation_route_mode} 路线分 {len(self._stage_targets)} 段执行。"
            )
        self._send_stage_goal()

    def _build_target_state(self, target_pose: PoseStamped) -> dict[str, float]:
        yaw = quaternion_to_yaw(
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z,
            target_pose.pose.orientation.w,
        )
        lateral_dx = -math.sin(yaw) * self._base_lateral_offset
        lateral_dy = math.cos(yaw) * self._base_lateral_offset
        state = {
            "base_x": target_pose.pose.position.x - math.cos(yaw) * self._base_offset + lateral_dx,
            "base_y": target_pose.pose.position.y - math.sin(yaw) * self._base_offset + lateral_dy,
            "base_theta": normalize_angle(yaw + self._base_yaw_offset),
        }
        arm_joint_names = self._controller_joint_names[3:]
        if self._stow_arm_during_initialization:
            for joint_name in arm_joint_names:
                state[joint_name] = float(DEFAULT_STOW_STATE_POSITIONS[joint_name])
        else:
            current_arm_positions = current_positions_for(arm_joint_names, self._current_joint_state, DEFAULT_STOW_STATE_POSITIONS)
            for joint_name, joint_value in zip(arm_joint_names, current_arm_positions):
                state[joint_name] = joint_value
        return state

    def _build_trajectory(self, target_state: dict[str, float]) -> JointTrajectory:
        active_joint_names = [name for name in self._controller_joint_names if name in target_state]
        if not active_joint_names:
            active_joint_names = list(self._controller_joint_names)

        if self._full_target_state is None:
            merged_target_state = dict(target_state)
        else:
            merged_target_state = dict(self._full_target_state)
            merged_target_state.update(target_state)

        start_positions = current_positions_for(
            tuple(active_joint_names),
            self._current_joint_state,
            merged_target_state,
        )

        active_joint_positions = dict(zip(active_joint_names, start_positions))
        current_base_x = active_joint_positions.get("base_x", merged_target_state.get("base_x", 0.0))
        current_base_y = active_joint_positions.get("base_y", merged_target_state.get("base_y", 0.0))
        current_base_theta = active_joint_positions.get("base_theta", merged_target_state.get("base_theta", 0.0))
        axis_distance = max(
            abs(merged_target_state["base_x"] - current_base_x),
            abs(merged_target_state["base_y"] - current_base_y),
        )
        yaw_distance = abs(normalize_angle(merged_target_state["base_theta"] - current_base_theta))
        arm_distance = 0.0
        for joint_name in active_joint_names:
            if joint_name in TRIAL_BASE_JOINTS:
                continue
            arm_distance = max(
                arm_distance,
                abs(merged_target_state[joint_name] - active_joint_positions.get(joint_name, merged_target_state[joint_name])),
            )
        duration_sec = max(
            self._min_motion_duration,
            axis_distance / self._base_linear_speed,
            yaw_distance / self._base_angular_speed,
            arm_distance / self._arm_joint_speed,
        )
        duration_sec = duration_sec * self._motion_duration_scale + self._motion_duration_margin

        trajectory = JointTrajectory()
        trajectory.joint_names = active_joint_names

        start_point = JointTrajectoryPoint()
        start_point.positions = start_positions
        start_point.time_from_start.sec = 0
        start_point.time_from_start.nanosec = 0

        end_point = JointTrajectoryPoint()
        end_point.positions = [merged_target_state[name] for name in active_joint_names]
        total_ns = int(duration_sec * 1e9)
        end_point.time_from_start.sec = total_ns // 1_000_000_000
        end_point.time_from_start.nanosec = total_ns % 1_000_000_000

        trajectory.points = [start_point, end_point]
        return trajectory

    def _build_stage_targets(
        self,
        start_positions: list[float],
        target_state: dict[str, float],
    ) -> list[dict[str, float]]:
        current_state = {name: value for name, value in zip(TRIAL_BASE_JOINTS, start_positions[:3])}
        staged_targets: list[dict[str, float]] = []
        route_points = self._build_navigation_route(current_state, target_state)
        route_cursor = dict(current_state)
        for route_point in route_points:
            staged_targets.extend(self._interpolate_route_segment(route_cursor, route_point))
            route_cursor = dict(route_point)
        return staged_targets

    def _build_navigation_route(
        self,
        current_state: dict[str, float],
        target_state: dict[str, float],
    ) -> list[dict[str, float]]:
        if self._translate_then_rotate:
            return self._build_axis_aligned_route(current_state, target_state)

        if self._navigation_route_mode != "central_corridor":
            return [target_state]

        route_points: list[dict[str, float]] = []
        corridor_y = self._navigation_corridor_y
        current_x = current_state["base_x"]
        current_y = current_state["base_y"]
        target_x = target_state["base_x"]
        target_y = target_state["base_y"]

        if abs(current_y - corridor_y) > 0.05:
            route_points.append(
                {
                    "base_x": current_x,
                    "base_y": corridor_y,
                    "base_theta": math.pi / 2.0 if corridor_y >= current_y else -math.pi / 2.0,
                }
            )

        if abs(target_x - current_x) > 0.05:
            route_points.append(
                {
                    "base_x": target_x,
                    "base_y": corridor_y,
                    "base_theta": 0.0 if target_x >= current_x else math.pi,
                }
            )
        final_corridor_heading = route_points[-1]["base_theta"] if route_points else current_state["base_theta"]
        if abs(normalize_angle(target_state["base_theta"] - final_corridor_heading)) > math.radians(2.0):
            route_points.append(
                {
                    "base_x": target_x,
                    "base_y": corridor_y,
                    "base_theta": target_state["base_theta"],
                }
            )

        route_points.append(dict(target_state))
        return route_points

    def _build_axis_aligned_route(
        self,
        current_state: dict[str, float],
        target_state: dict[str, float],
    ) -> list[dict[str, float]]:
        route_points: list[dict[str, float]] = []
        cursor = dict(current_state)

        def append_state(*, base_x: float | None = None, base_y: float | None = None, base_theta: float | None = None) -> None:
            next_state = dict(cursor)
            if base_x is not None:
                next_state["base_x"] = base_x
            if base_y is not None:
                next_state["base_y"] = base_y
            if base_theta is not None:
                next_state["base_theta"] = normalize_angle(base_theta)

            if (
                abs(next_state["base_x"] - cursor["base_x"]) <= 1e-4
                and abs(next_state["base_y"] - cursor["base_y"]) <= 1e-4
                and abs(normalize_angle(next_state["base_theta"] - cursor["base_theta"])) <= math.radians(0.1)
            ):
                return
            route_points.append(next_state)
            cursor.update(next_state)

        if self._navigation_route_mode == "central_corridor":
            corridor_y = self._navigation_corridor_y
            if abs(cursor["base_y"] - corridor_y) > 0.02:
                append_state(base_y=corridor_y)
            if abs(target_state["base_x"] - cursor["base_x"]) > 0.02:
                append_state(base_x=target_state["base_x"])
            if abs(target_state["base_y"] - cursor["base_y"]) > 0.02:
                append_state(base_y=target_state["base_y"])
        else:
            if abs(target_state["base_x"] - cursor["base_x"]) > 0.02:
                append_state(base_x=target_state["base_x"])
            if abs(target_state["base_y"] - cursor["base_y"]) > 0.02:
                append_state(base_y=target_state["base_y"])

        if abs(normalize_angle(target_state["base_theta"] - cursor["base_theta"])) > math.radians(1.0):
            append_state(base_theta=target_state["base_theta"])

        if not route_points:
            route_points.append(dict(target_state))
        return route_points

    def _interpolate_route_segment(
        self,
        start_state: dict[str, float],
        end_state: dict[str, float],
    ) -> list[dict[str, float]]:
        planar_distance = math.hypot(
            end_state["base_x"] - start_state["base_x"],
            end_state["base_y"] - start_state["base_y"],
        )
        yaw_distance = abs(normalize_angle(end_state["base_theta"] - start_state["base_theta"]))
        segment_count = max(
            1,
            math.ceil(planar_distance / self._navigation_segment_translation_limit),
            math.ceil(yaw_distance / self._navigation_segment_rotation_limit),
        )
        result: list[dict[str, float]] = []
        for index in range(1, segment_count + 1):
            ratio = float(index) / float(segment_count)
            result.append(
                {
                    "base_x": start_state["base_x"] + (end_state["base_x"] - start_state["base_x"]) * ratio,
                    "base_y": start_state["base_y"] + (end_state["base_y"] - start_state["base_y"]) * ratio,
                    "base_theta": normalize_angle(
                        start_state["base_theta"]
                        + normalize_angle(end_state["base_theta"] - start_state["base_theta"]) * ratio
                    ),
                }
            )
        return result

    def _send_stage_goal(self) -> None:
        if self._stage_index >= len(self._stage_targets):
            return
        target_state = self._stage_targets[self._stage_index]
        trajectory = self._build_trajectory(target_state)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        send_future = self._trajectory_action.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_sent)
        self._trajectory_in_flight = True
        self.get_logger().info(
            "已发送 Gazebo 试验初始化分段轨迹："
            f" stage={self._stage_index + 1}/{len(self._stage_targets)}, "
            f"base=({target_state['base_x']:.3f}, {target_state['base_y']:.3f}, {math.degrees(target_state['base_theta']):.1f}deg), "
            f"duration={trajectory.points[-1].time_from_start.sec + trajectory.points[-1].time_from_start.nanosec / 1e9:.1f}s"
        )

    def _on_goal_sent(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Gazebo 试验初始化轨迹被控制器拒绝。")
            self._trajectory_in_flight = False
            self._goal_completed = True
            self._goal_succeeded = False
            self._goal_completed_at = time.monotonic()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        result_wrapper = future.result()
        self._trajectory_in_flight = False

        if result_wrapper is None:
            self.get_logger().error("Gazebo 试验初始化轨迹没有返回结果。")
            self._goal_completed = True
            self._goal_completed_at = time.monotonic()
            return

        error_code = int(getattr(result_wrapper.result, "error_code", 0))
        if error_code == 0:
            if self._stage_index + 1 < len(self._stage_targets):
                self._stage_index += 1
                self.get_logger().info(
                    f"Gazebo 试验初始化分段轨迹执行成功，准备发送下一段：{self._stage_index + 1}/{len(self._stage_targets)}"
                )
                self._send_stage_goal()
                return
            self._goal_completed = True
            self._goal_completed_at = time.monotonic()
            self._goal_succeeded = True
            self._publish_initialization_ready(True)
            self.get_logger().info("Gazebo 试验初始化轨迹执行成功。")
        else:
            self._goal_completed = True
            self._goal_completed_at = time.monotonic()
            self._goal_succeeded = False
            self._publish_initialization_ready(False)
            self.get_logger().error(f"Gazebo 试验初始化轨迹执行失败，error_code={error_code}。")

    def _request_task_start(self) -> None:
        if not self._start_client.wait_for_service(timeout_sec=0.1):
            return
        future = self._start_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_start_response)
        self._start_requested = True

    def _on_start_response(self, future) -> None:
        response = future.result()
        if response is None:
            self.get_logger().error("Gazebo 试验初始化后调用 /medipick/task/start 失败。")
            return
        if response.success:
            self.get_logger().info("Gazebo 试验初始化完成，任务已启动。")
        else:
            self.get_logger().warning(f"Gazebo 试验初始化后任务启动被拒绝：{response.message}")
            self._start_requested = False

    def _request_controller_state(self) -> None:
        if self._controller_state_request_pending:
            return
        if not self._list_controllers_client.wait_for_service(timeout_sec=0.1):
            return
        future = self._list_controllers_client.call_async(ListControllers.Request())
        future.add_done_callback(self._on_list_controllers)
        self._controller_state_request_pending = True

    def _on_list_controllers(self, future) -> None:
        self._controller_state_request_pending = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"查询控制器状态失败：{exc}")
            return
        if response is None:
            return
        for controller in response.controller:
            if controller.name == self._controller_name:
                self._controller_active = controller.state == "active"
                if self._controller_active:
                    self.get_logger().info(f"控制器 {self._controller_name} 已进入 active，允许发送初始化轨迹。")
                return

    def _publish_initialization_ready(self, ready: bool) -> None:
        msg = Bool()
        msg.data = ready
        self._initialization_ready_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = GazeboTrialInitialPoseManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
