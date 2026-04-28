#!/usr/bin/python3

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, Quaternion
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pick_task_shared import DEFAULT_STOW_STATE_POSITIONS


DEFAULT_NAV_ARM_STOW_JOINT_NAMES = tuple(
    joint_name for joint_name in DEFAULT_STOW_STATE_POSITIONS.keys() if joint_name != "raise_joint"
)


def quaternion_to_yaw(quat: Quaternion) -> float:
    return math.atan2(
        2.0 * (quat.w * quat.z + quat.x * quat.y),
        1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z),
    )


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_from_yaw(yaw: float) -> Quaternion:
    quat = Quaternion()
    quat.z = math.sin(yaw / 2.0)
    quat.w = math.cos(yaw / 2.0)
    return quat


def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    quat = Quaternion()
    quat.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y
    quat.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x
    quat.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    quat.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    return quat


def quaternion_conjugate(q: Quaternion) -> Quaternion:
    quat = Quaternion()
    quat.x = -q.x
    quat.y = -q.y
    quat.z = -q.z
    quat.w = q.w
    return quat


def rotate_vector_by_quaternion(q: Quaternion, vector: tuple[float, float, float]) -> tuple[float, float, float]:
    vx, vy, vz = vector
    pure = Quaternion()
    pure.x = vx
    pure.y = vy
    pure.z = vz
    pure.w = 0.0
    rotated = quaternion_multiply(quaternion_multiply(q, pure), quaternion_conjugate(q))
    return rotated.x, rotated.y, rotated.z


def clone_pose_stamped(pose: PoseStamped) -> PoseStamped:
    result = PoseStamped()
    result.header = pose.header
    result.pose = pose.pose
    return result


class Nav2TargetNavigator(Node):
    def __init__(self) -> None:
        super().__init__("medipick_nav2_target_navigator")

        self.declare_parameter("target_pose_topic", "/medipick/task/target_pose")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("navigate_action_name", "/navigate_to_pose")
        self.declare_parameter("task_start_service", "/medipick/task/start")
        self.declare_parameter("bt_navigator_state_service", "/bt_navigator/get_state")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("base_offset", 0.95)
        self.declare_parameter("base_lateral_offset", 0.0)
        self.declare_parameter("base_yaw_offset_deg", 0.0)
        self.declare_parameter("navigation_route_mode", "central_corridor")
        self.declare_parameter("navigation_corridor_y", 0.0)
        self.declare_parameter("waypoint_position_tolerance", 0.15)
        self.declare_parameter("max_stage_travel_distance_m", 1.50)
        self.declare_parameter("min_known_map_cells", 10)
        self.declare_parameter("require_robot_inside_map", True)
        self.declare_parameter("map_ready_margin_m", 0.05)
        self.declare_parameter("task_start_delay_sec", 1.5)
        self.declare_parameter("retry_delay_sec", 5.0)
        self.declare_parameter("max_retry_count", 6)
        self.declare_parameter("goal_update_distance_threshold", 0.05)
        self.declare_parameter("goal_update_yaw_threshold_deg", 6.0)
        self.declare_parameter("lock_target_pose_during_navigation", True)
        self.declare_parameter("transform_timeout_sec", 0.20)
        self.declare_parameter("staging_margin_m", 0.45)
        self.declare_parameter("final_orientation_distance_threshold_m", 0.70)
        self.declare_parameter("final_goal_keep_current_yaw_when_close", False)
        self.declare_parameter("stage_goal_completion_distance_threshold_m", 0.06)
        self.declare_parameter("coarse_success_distance_threshold_m", 0.30)
        self.declare_parameter("enable_head_posture_control", True)
        self.declare_parameter("head_action_name", "/head_controller/follow_joint_trajectory")
        self.declare_parameter("head_joint_names", ["h1_joint", "h2_joint"])
        self.declare_parameter("nav_head_positions", [0.0, -0.5236])
        self.declare_parameter("observe_head_positions", [1.5708, 0.0])
        self.declare_parameter("head_motion_duration_sec", 1.0)
        self.declare_parameter("head_server_wait_sec", 0.05)
        self.declare_parameter("head_retry_delay_sec", 1.0)
        self.declare_parameter("enable_nav_arm_stow", True)
        self.declare_parameter("arm_action_name", "/mobile_arm_controller/follow_joint_trajectory")
        self.declare_parameter(
            "arm_joint_names",
            list(DEFAULT_NAV_ARM_STOW_JOINT_NAMES),
        )
        self.declare_parameter(
            "nav_arm_stow_positions",
            [DEFAULT_STOW_STATE_POSITIONS[joint_name] for joint_name in DEFAULT_NAV_ARM_STOW_JOINT_NAMES],
        )
        self.declare_parameter("arm_motion_duration_sec", 3.0)
        self.declare_parameter("arm_server_wait_sec", 0.10)
        self.declare_parameter("arm_retry_delay_sec", 1.0)
        self.declare_parameter("navigate_server_wait_sec", 1.0)

        self._target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self._map_topic = str(self.get_parameter("map_topic").value)
        self._navigate_action_name = str(self.get_parameter("navigate_action_name").value)
        self._task_start_service = str(self.get_parameter("task_start_service").value)
        self._bt_navigator_state_service = str(self.get_parameter("bt_navigator_state_service").value).strip()
        self._goal_frame = str(self.get_parameter("goal_frame").value).strip()
        self._robot_base_frame = str(self.get_parameter("robot_base_frame").value).strip()
        self._base_offset = float(self.get_parameter("base_offset").value)
        self._base_lateral_offset = float(self.get_parameter("base_lateral_offset").value)
        self._base_yaw_offset = math.radians(float(self.get_parameter("base_yaw_offset_deg").value))
        self._navigation_route_mode = str(self.get_parameter("navigation_route_mode").value).strip()
        self._navigation_corridor_y = float(self.get_parameter("navigation_corridor_y").value)
        self._waypoint_position_tolerance = max(0.05, float(self.get_parameter("waypoint_position_tolerance").value))
        self._max_stage_travel_distance = max(0.0, float(self.get_parameter("max_stage_travel_distance_m").value))
        self._min_known_map_cells = max(0, int(self.get_parameter("min_known_map_cells").value))
        self._require_robot_inside_map = bool(self.get_parameter("require_robot_inside_map").value)
        self._map_ready_margin = max(0.0, float(self.get_parameter("map_ready_margin_m").value))
        self._task_start_delay_sec = max(0.0, float(self.get_parameter("task_start_delay_sec").value))
        self._retry_delay_sec = max(0.5, float(self.get_parameter("retry_delay_sec").value))
        self._max_retry_count = max(0, int(self.get_parameter("max_retry_count").value))
        self._goal_update_distance_threshold = max(
            0.01, float(self.get_parameter("goal_update_distance_threshold").value)
        )
        self._goal_update_yaw_threshold = math.radians(
            max(1.0, float(self.get_parameter("goal_update_yaw_threshold_deg").value))
        )
        self._lock_target_pose_during_navigation = bool(
            self.get_parameter("lock_target_pose_during_navigation").value
        )
        self._transform_timeout = Duration(seconds=max(0.01, float(self.get_parameter("transform_timeout_sec").value)))
        self._staging_margin = max(0.10, float(self.get_parameter("staging_margin_m").value))
        self._final_orientation_distance_threshold = max(
            0.20, float(self.get_parameter("final_orientation_distance_threshold_m").value)
        )
        self._final_goal_keep_current_yaw_when_close = bool(
            self.get_parameter("final_goal_keep_current_yaw_when_close").value
        )
        self._stage_goal_completion_distance_threshold = max(
            0.0, float(self.get_parameter("stage_goal_completion_distance_threshold_m").value)
        )
        self._coarse_success_distance_threshold = max(
            0.0, float(self.get_parameter("coarse_success_distance_threshold_m").value)
        )
        self._enable_head_posture_control = bool(self.get_parameter("enable_head_posture_control").value)
        self._head_action_name = str(self.get_parameter("head_action_name").value).strip()
        self._head_joint_names = tuple(
            str(joint_name).strip()
            for joint_name in self.get_parameter("head_joint_names").value
            if str(joint_name).strip()
        )
        self._nav_head_positions = tuple(float(value) for value in self.get_parameter("nav_head_positions").value)
        self._observe_head_positions = tuple(
            float(value) for value in self.get_parameter("observe_head_positions").value
        )
        self._head_motion_duration_sec = max(0.1, float(self.get_parameter("head_motion_duration_sec").value))
        self._head_server_wait_sec = max(0.01, float(self.get_parameter("head_server_wait_sec").value))
        self._head_retry_delay_sec = max(0.2, float(self.get_parameter("head_retry_delay_sec").value))
        self._enable_nav_arm_stow = bool(self.get_parameter("enable_nav_arm_stow").value)
        self._arm_action_name = str(self.get_parameter("arm_action_name").value).strip()
        self._arm_joint_names = tuple(
            str(joint_name).strip()
            for joint_name in self.get_parameter("arm_joint_names").value
            if str(joint_name).strip()
        )
        self._nav_arm_stow_positions = tuple(
            float(value) for value in self.get_parameter("nav_arm_stow_positions").value
        )
        self._arm_motion_duration_sec = max(0.1, float(self.get_parameter("arm_motion_duration_sec").value))
        self._arm_server_wait_sec = max(0.01, float(self.get_parameter("arm_server_wait_sec").value))
        self._arm_retry_delay_sec = max(0.2, float(self.get_parameter("arm_retry_delay_sec").value))
        self._navigate_server_wait_sec = max(0.1, float(self.get_parameter("navigate_server_wait_sec").value))

        self._latest_target_pose: Optional[PoseStamped] = None
        self._locked_navigation_target_pose: Optional[PoseStamped] = None
        self._latest_joint_state = JointState()
        self._last_sent_goal_pose: Optional[PoseStamped] = None
        self._latest_map: Optional[OccupancyGrid] = None
        self._map_ready = False
        self._known_map_cells = 0
        self._goal_handle = None
        self._goal_in_flight = False
        self._active_goal_is_final = False
        self._last_completed_stage_goal: Optional[PoseStamped] = None
        self._goal_cancel_mode: Optional[str] = None
        self._navigation_succeeded = False
        self._navigation_retry_exhausted = False
        self._task_started = False
        self._nav_stack_active = False
        self._nav_state_request_in_flight = False
        self._nav_state_retry_after = 0.0
        self._retry_count = 0
        self._retry_after = 0.0
        self._navigation_completed_at = 0.0
        self._last_wait_log = 0.0
        self._last_tf_warning_time = 0.0
        self._head_goal_in_flight = False
        self._head_current_mode: Optional[str] = None
        self._head_requested_mode: Optional[str] = None
        self._head_current_positions: Optional[tuple[float, ...]] = None
        self._head_requested_positions: Optional[tuple[float, ...]] = None
        self._last_head_warning_time = 0.0
        self._head_retry_after = 0.0
        self._arm_goal_in_flight = False
        self._arm_current_positions: Optional[tuple[float, ...]] = None
        self._arm_requested_positions: Optional[tuple[float, ...]] = None
        self._arm_retry_after = 0.0
        self._arm_ready_for_navigation = False
        self._last_arm_warning_time = 0.0

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self.create_subscription(PoseStamped, self._target_pose_topic, self._on_target_pose, 10)
        self.create_subscription(OccupancyGrid, self._map_topic, self._on_map, map_qos)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        self._navigate_action = ActionClient(self, NavigateToPose, self._navigate_action_name)
        self._task_start_client = self.create_client(Trigger, self._task_start_service)
        self._bt_navigator_state_client = self.create_client(GetState, self._bt_navigator_state_service)
        self._head_action: Optional[ActionClient] = None
        self._arm_action: Optional[ActionClient] = None
        if self._enable_head_posture_control:
            if not self._head_action_name or len(self._head_joint_names) == 0:
                self.get_logger().warning("头部姿态控制参数不完整，已禁用导航锁头。")
                self._enable_head_posture_control = False
            elif (
                len(self._head_joint_names) != len(self._nav_head_positions)
                or len(self._head_joint_names) != len(self._observe_head_positions)
            ):
                self.get_logger().warning("头部关节名和姿态数组长度不一致，已禁用导航锁头。")
                self._enable_head_posture_control = False
            else:
                self._head_action = ActionClient(self, FollowJointTrajectory, self._head_action_name)
        if self._enable_nav_arm_stow:
            if not self._arm_action_name or len(self._arm_joint_names) == 0:
                self.get_logger().warning("导航收臂参数不完整，已禁用导航前收臂。")
                self._enable_nav_arm_stow = False
            elif len(self._arm_joint_names) != len(self._nav_arm_stow_positions):
                self.get_logger().warning("导航收臂关节名和目标姿态长度不一致，已禁用导航前收臂。")
                self._enable_nav_arm_stow = False
            else:
                self._arm_action = ActionClient(self, FollowJointTrajectory, self._arm_action_name)
        self.create_timer(0.2, self._on_timer)

        self.get_logger().info(
            "Nav2 目标导航器已启动："
            f" target={self._target_pose_topic}, map={self._map_topic}, action={self._navigate_action_name}, "
            f"task_start={self._task_start_service}, goal_frame={self._goal_frame or '<target>'}, "
            f"bt_state={self._bt_navigator_state_service}"
        )
        if self._enable_head_posture_control:
            self.get_logger().info(
                "Nav2 头部姿态控制已启用："
                f" joints={list(self._head_joint_names)}, nav={list(self._nav_head_positions)}, "
                f"observe={list(self._observe_head_positions)}, action={self._head_action_name}"
            )
        if self._enable_nav_arm_stow:
            self.get_logger().info(
                "Nav2 导航前收臂已启用："
                f" joints={list(self._arm_joint_names)}, positions={list(self._nav_arm_stow_positions)}, "
                f"action={self._arm_action_name}"
            )

    def _on_target_pose(self, msg: PoseStamped) -> None:
        self._latest_target_pose = msg
        if self._lock_target_pose_during_navigation and self._locked_navigation_target_pose is None:
            self._locked_navigation_target_pose = clone_pose_stamped(msg)
            self.get_logger().info(
                f"Nav2 已锁定导航目标位姿，frame={msg.header.frame_id or '<empty>'}。"
            )
        if self._navigation_succeeded or self._goal_in_flight:
            return
        self.get_logger().info(
            f"Nav2 导航器收到目标药盒位姿，frame={msg.header.frame_id or '<empty>'}，等待地图与导航栈就绪。"
        )

    def _target_pose_for_navigation(self) -> Optional[PoseStamped]:
        if self._lock_target_pose_during_navigation and self._locked_navigation_target_pose is not None:
            return self._locked_navigation_target_pose
        return self._latest_target_pose

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg
        known_cells = sum(1 for value in msg.data if value >= 0)
        self._known_map_cells = known_cells
        self._maybe_mark_map_ready()

    def _maybe_mark_map_ready(self) -> None:
        if self._map_ready or self._latest_map is None:
            return

        known_cells = self._known_map_cells
        robot_inside_map = True
        if self._require_robot_inside_map:
            robot_inside_map = self._robot_pose_inside_map(margin=self._map_ready_margin)

        if known_cells < self._min_known_map_cells or not robot_inside_map:
            return

        self._map_ready = True
        self._navigation_retry_exhausted = False
        self._retry_count = 0
        bounds = self._map_bounds()
        bounds_text = ""
        if bounds is not None:
            xmin, xmax, ymin, ymax = bounds
            bounds_text = (
                f"，bounds=[{xmin:.2f},{xmax:.2f}] x [{ymin:.2f},{ymax:.2f}]"
            )
        self.get_logger().info(
            f"Nav2 导航器已收到可用地图，known_cells={known_cells}{bounds_text}，允许发起导航。"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def _joint_positions_match(self, joint_names: tuple[str, ...], target_positions: tuple[float, ...], tolerance: float) -> bool:
        if len(self._latest_joint_state.name) == 0:
            return False
        current_positions = {
            joint_name: position for joint_name, position in zip(self._latest_joint_state.name, self._latest_joint_state.position)
        }
        for joint_name, target_position in zip(joint_names, target_positions):
            current_position = current_positions.get(joint_name)
            if current_position is None or abs(current_position - target_position) > tolerance:
                return False
        return True

    def _request_nav_arm_stow(self) -> bool:
        if not self._enable_nav_arm_stow:
            return True
        if self._arm_action is None:
            return True

        desired_positions = self._nav_arm_stow_positions
        if self._joint_positions_match(self._arm_joint_names, desired_positions, tolerance=0.08):
            self._arm_current_positions = desired_positions
            self._arm_ready_for_navigation = True
            return True

        now = time.monotonic()
        if now < self._arm_retry_after:
            return False

        if self._arm_goal_in_flight:
            return False

        if not self._arm_action.wait_for_server(timeout_sec=self._arm_server_wait_sec):
            if now - self._last_arm_warning_time >= 2.0:
                self._last_arm_warning_time = now
                self.get_logger().warning(f"导航收臂 action 未就绪：{self._arm_action_name}")
            self._arm_retry_after = now + self._arm_retry_delay_sec
            return False

        trajectory = JointTrajectory()
        trajectory.joint_names = list(self._arm_joint_names)
        point = JointTrajectoryPoint()
        point.positions = list(desired_positions)
        point.time_from_start = Duration(seconds=self._arm_motion_duration_sec).to_msg()
        trajectory.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self._arm_goal_in_flight = True
        self._arm_requested_positions = desired_positions
        self._arm_retry_after = 0.0
        self._arm_ready_for_navigation = False

        self.get_logger().info(
            "导航前收臂："
            + ", ".join(f"{joint}={position:.3f}" for joint, position in zip(self._arm_joint_names, desired_positions))
        )

        send_future = self._arm_action.send_goal_async(goal)
        send_future.add_done_callback(self._on_arm_goal_response)
        return False

    def _on_arm_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"发送导航收臂目标失败：{exc}")
            self._arm_goal_in_flight = False
            self._arm_retry_after = time.monotonic() + self._arm_retry_delay_sec
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning("导航收臂目标被拒绝，将稍后重试。")
            self._arm_goal_in_flight = False
            self._arm_retry_after = time.monotonic() + self._arm_retry_delay_sec
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_arm_goal_result)

    def _on_arm_goal_result(self, future) -> None:
        self._arm_goal_in_flight = False
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"获取导航收臂结果失败：{exc}")
            self._arm_retry_after = time.monotonic() + self._arm_retry_delay_sec
            return

        if result is None:
            self.get_logger().warning("导航收臂动作返回空结果，将稍后重试。")
            self._arm_retry_after = time.monotonic() + self._arm_retry_delay_sec
            return

        status = int(result.status)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._arm_current_positions = self._arm_requested_positions
            self._arm_ready_for_navigation = True
            return

        status_name = {
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }.get(status, f"status={status}")
        self.get_logger().warning(f"导航收臂动作未成功，结果={status_name}。")
        self._arm_retry_after = time.monotonic() + self._arm_retry_delay_sec

    def _head_positions_for_mode(self, mode: str) -> tuple[float, ...] | None:
        if mode == "nav":
            return self._nav_head_positions
        if mode == "observe":
            return self._observe_head_positions
        return None

    def _request_head_mode(self, mode: str) -> bool:
        if not self._enable_head_posture_control:
            return True
        if self._head_action is None:
            return True

        desired_positions = self._head_positions_for_mode(mode)
        if desired_positions is None:
            return True

        now = time.monotonic()
        if now < self._head_retry_after:
            return False

        if self._head_current_positions == desired_positions:
            self._head_current_mode = mode
            return True

        if self._head_goal_in_flight:
            return self._head_requested_positions == desired_positions

        if not self._head_action.wait_for_server(timeout_sec=self._head_server_wait_sec):
            if now - self._last_head_warning_time >= 2.0:
                self._last_head_warning_time = now
                self.get_logger().warning(f"头部控制 action 未就绪：{self._head_action_name}")
            self._head_retry_after = now + self._head_retry_delay_sec
            return False

        trajectory = JointTrajectory()
        trajectory.joint_names = list(self._head_joint_names)
        point = JointTrajectoryPoint()
        point.positions = list(desired_positions)
        point.time_from_start = Duration(seconds=self._head_motion_duration_sec).to_msg()
        trajectory.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self._head_goal_in_flight = True
        self._head_requested_mode = mode
        self._head_requested_positions = desired_positions
        self._head_retry_after = 0.0

        self.get_logger().info(
            f"发送头部{'导航锁定' if mode == 'nav' else '抓取观察'}姿态："
            + ", ".join(f"{joint}={position:.3f}" for joint, position in zip(self._head_joint_names, desired_positions))
        )

        send_future = self._head_action.send_goal_async(goal)
        send_future.add_done_callback(self._on_head_goal_response)
        return True

    def _on_head_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"发送头部姿态目标失败：{exc}")
            self._head_goal_in_flight = False
            self._head_retry_after = time.monotonic() + self._head_retry_delay_sec
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning("头部姿态目标被拒绝，将稍后重试。")
            self._head_goal_in_flight = False
            self._head_retry_after = time.monotonic() + self._head_retry_delay_sec
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_head_goal_result)

    def _on_head_goal_result(self, future) -> None:
        self._head_goal_in_flight = False
        requested_mode = self._head_requested_mode
        requested_positions = self._head_requested_positions
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"获取头部姿态结果失败：{exc}")
            self._head_retry_after = time.monotonic() + self._head_retry_delay_sec
            return

        if result is None:
            self.get_logger().warning("头部姿态动作返回空结果，将稍后重试。")
            self._head_retry_after = time.monotonic() + self._head_retry_delay_sec
            return

        status = int(result.status)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._head_current_mode = requested_mode
            self._head_current_positions = requested_positions
            return

        status_name = {
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }.get(status, f"status={status}")
        self.get_logger().warning(f"头部姿态动作未成功，结果={status_name}。")
        self._head_retry_after = time.monotonic() + self._head_retry_delay_sec

    def _transform_pose(self, pose: PoseStamped, target_frame: str) -> PoseStamped | None:
        if not target_frame or target_frame == pose.header.frame_id:
            transformed = clone_pose_stamped(pose)
            transformed.header.stamp = self.get_clock().now().to_msg()
            return transformed

        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                pose.header.frame_id,
                rclpy.time.Time(),
                timeout=self._transform_timeout,
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec - self._last_tf_warning_time >= 2.0:
                self._last_tf_warning_time = now_sec
                self.get_logger().warning(
                    f"Nav2 导航器等待目标位姿 TF：{pose.header.frame_id} -> {target_frame}，原因：{exc}"
                )
            return None

        transformed = PoseStamped()
        transformed.header.stamp = self.get_clock().now().to_msg()
        transformed.header.frame_id = target_frame
        rotated = rotate_vector_by_quaternion(
            transform.transform.rotation,
            (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ),
        )
        transformed.pose.position.x = transform.transform.translation.x + rotated[0]
        transformed.pose.position.y = transform.transform.translation.y + rotated[1]
        transformed.pose.position.z = transform.transform.translation.z + rotated[2]
        transformed.pose.orientation = quaternion_multiply(transform.transform.rotation, pose.pose.orientation)
        return transformed

    def _lookup_robot_pose(self) -> PoseStamped | None:
        if not self._goal_frame or not self._robot_base_frame:
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                self._goal_frame,
                self._robot_base_frame,
                rclpy.time.Time(),
                timeout=self._transform_timeout,
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._goal_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _map_bounds(self) -> tuple[float, float, float, float] | None:
        if self._latest_map is None:
            return None
        origin = self._latest_map.info.origin.position
        resolution = float(self._latest_map.info.resolution)
        width = float(self._latest_map.info.width) * resolution
        height = float(self._latest_map.info.height) * resolution
        return origin.x, origin.x + width, origin.y, origin.y + height

    def _point_inside_map(self, x: float, y: float, margin: float) -> bool:
        bounds = self._map_bounds()
        if bounds is None:
            return False
        xmin, xmax, ymin, ymax = bounds
        return (xmin + margin) <= x <= (xmax - margin) and (ymin + margin) <= y <= (ymax - margin)

    def _robot_pose_inside_map(self, margin: float) -> bool:
        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return False
        return self._point_inside_map(
            robot_pose.pose.position.x,
            robot_pose.pose.position.y,
            margin,
        )

    def _compute_staging_goal(
        self,
        robot_pose: PoseStamped,
        final_goal_pose: PoseStamped,
    ) -> PoseStamped | None:
        bounds = self._map_bounds()
        if bounds is None:
            return None

        xmin, xmax, ymin, ymax = bounds
        xmin += self._staging_margin
        xmax -= self._staging_margin
        ymin += self._staging_margin
        ymax -= self._staging_margin
        if xmin >= xmax or ymin >= ymax:
            return None

        rx = float(robot_pose.pose.position.x)
        ry = float(robot_pose.pose.position.y)
        gx = float(final_goal_pose.pose.position.x)
        gy = float(final_goal_pose.pose.position.y)
        dx = gx - rx
        dy = gy - ry
        distance = math.hypot(dx, dy)
        if distance < 1.0e-6:
            return None

        candidates: list[float] = []
        if dx > 0.0:
            candidates.append((xmax - rx) / dx)
        elif dx < 0.0:
            candidates.append((xmin - rx) / dx)
        if dy > 0.0:
            candidates.append((ymax - ry) / dy)
        elif dy < 0.0:
            candidates.append((ymin - ry) / dy)

        valid_t = []
        for candidate in candidates:
            if candidate <= 0.0:
                continue
            px = rx + dx * candidate
            py = ry + dy * candidate
            if xmin <= px <= xmax and ymin <= py <= ymax:
                valid_t.append(candidate)
        if not valid_t:
            return None

        t = min(valid_t)
        stage = PoseStamped()
        stage.header.stamp = self.get_clock().now().to_msg()
        stage.header.frame_id = self._goal_frame
        stage.pose.position.x = rx + dx * t
        stage.pose.position.y = ry + dy * t
        stage.pose.position.z = 0.0
        stage.pose.orientation = quaternion_from_yaw(math.atan2(dy, dx))
        return stage

    def _build_corridor_goal(self, x: float, y: float, yaw: float) -> PoseStamped:
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self._goal_frame
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation = quaternion_from_yaw(yaw)
        return goal

    def _limit_stage_goal_distance(self, robot_pose: PoseStamped, goal_pose: PoseStamped) -> PoseStamped:
        if self._max_stage_travel_distance <= 0.0:
            return goal_pose

        dx = goal_pose.pose.position.x - robot_pose.pose.position.x
        dy = goal_pose.pose.position.y - robot_pose.pose.position.y
        distance = math.hypot(dx, dy)
        if distance <= self._max_stage_travel_distance or distance < 1.0e-6:
            return goal_pose

        step_scale = self._max_stage_travel_distance / distance
        limited_goal = clone_pose_stamped(goal_pose)
        limited_goal.header.stamp = self.get_clock().now().to_msg()
        limited_goal.pose.position.x = robot_pose.pose.position.x + dx * step_scale
        limited_goal.pose.position.y = robot_pose.pose.position.y + dy * step_scale
        limited_goal.pose.position.z = 0.0
        limited_goal.pose.orientation = quaternion_from_yaw(math.atan2(dy, dx))
        return limited_goal

    def _build_goal_pose(self, target_pose: PoseStamped) -> PoseStamped | None:
        goal_target_pose = self._transform_pose(target_pose, self._goal_frame)
        if goal_target_pose is None:
            return None

        target_yaw = quaternion_to_yaw(goal_target_pose.pose.orientation)
        lateral_dx = -math.sin(target_yaw) * self._base_lateral_offset
        lateral_dy = math.cos(target_yaw) * self._base_lateral_offset

        goal_pose = PoseStamped()
        goal_pose.header = goal_target_pose.header
        goal_pose.pose.position.x = (
            goal_target_pose.pose.position.x - math.cos(target_yaw) * self._base_offset + lateral_dx
        )
        goal_pose.pose.position.y = (
            goal_target_pose.pose.position.y - math.sin(target_yaw) * self._base_offset + lateral_dy
        )
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation = quaternion_from_yaw(normalize_angle(target_yaw + self._base_yaw_offset))
        return goal_pose

    def _select_navigation_goal(self, target_pose: PoseStamped) -> tuple[PoseStamped | None, bool]:
        final_goal_pose = self._build_goal_pose(target_pose)
        if final_goal_pose is None:
            return None, False

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return final_goal_pose, True

        reference_pose = robot_pose
        if (
            self._last_completed_stage_goal is not None
            and self._last_completed_stage_goal.header.frame_id == self._goal_frame
        ):
            progress_anchor_distance = math.hypot(
                reference_pose.pose.position.x - self._last_completed_stage_goal.pose.position.x,
                reference_pose.pose.position.y - self._last_completed_stage_goal.pose.position.y,
            )
            if progress_anchor_distance <= max(0.35, self._max_stage_travel_distance):
                reference_pose = clone_pose_stamped(self._last_completed_stage_goal)

        route_progress_tolerance = max(
            self._waypoint_position_tolerance,
            self._stage_goal_completion_distance_threshold,
        )

        if self._navigation_route_mode == "central_corridor":
            current_x = float(reference_pose.pose.position.x)
            current_y = float(reference_pose.pose.position.y)
            target_x = float(final_goal_pose.pose.position.x)
            target_y = float(final_goal_pose.pose.position.y)
            corridor_y = self._navigation_corridor_y
            x_aligned_with_target = abs(target_x - current_x) <= route_progress_tolerance

            # Use the main corridor only until we have aligned with the target shelf
            # column. Once x is aligned, keep the robot on the shelf side instead of
            # bouncing back to the corridor between side-step updates.
            if abs(current_y - corridor_y) > route_progress_tolerance and not x_aligned_with_target:
                corridor_yaw = math.pi / 2.0 if corridor_y >= current_y else -math.pi / 2.0
                corridor_goal = self._build_corridor_goal(current_x, corridor_y, corridor_yaw)
                return self._limit_stage_goal_distance(reference_pose, corridor_goal), False

            if not x_aligned_with_target:
                corridor_yaw = 0.0 if target_x >= current_x else math.pi
                corridor_goal = self._build_corridor_goal(target_x, corridor_y, corridor_yaw)
                return self._limit_stage_goal_distance(reference_pose, corridor_goal), False

            # Once we have reached the main corridor and aligned with the shelf column,
            # approach the shelf in an axis-aligned side step first. This avoids
            # asking Nav2 to trace a shallow diagonal near the shelf entrance, which
            # tends to stall the controller in Gazebo.
            if abs(target_y - current_y) > route_progress_tolerance:
                side_yaw = math.pi / 2.0 if target_y >= current_y else -math.pi / 2.0
                side_goal = self._build_corridor_goal(current_x, target_y, side_yaw)
                return self._limit_stage_goal_distance(reference_pose, side_goal), False

            if abs(target_x - current_x) > route_progress_tolerance:
                side_yaw = 0.0 if target_x >= current_x else math.pi
                side_goal = self._build_corridor_goal(target_x, target_y, side_yaw)
                return self._limit_stage_goal_distance(reference_pose, side_goal), False

        dx = final_goal_pose.pose.position.x - reference_pose.pose.position.x
        dy = final_goal_pose.pose.position.y - reference_pose.pose.position.y
        distance_to_final = math.hypot(dx, dy)
        travel_yaw = math.atan2(dy, dx) if distance_to_final > 1.0e-6 else quaternion_to_yaw(final_goal_pose.pose.orientation)

        if not self._point_inside_map(final_goal_pose.pose.position.x, final_goal_pose.pose.position.y, self._staging_margin):
            staging_goal = self._compute_staging_goal(reference_pose, final_goal_pose)
            if staging_goal is not None:
                return self._limit_stage_goal_distance(reference_pose, staging_goal), False

        if distance_to_final > self._final_orientation_distance_threshold:
            travel_goal = clone_pose_stamped(final_goal_pose)
            travel_goal.header.stamp = self.get_clock().now().to_msg()
            travel_goal.pose.orientation = quaternion_from_yaw(travel_yaw)
            return self._limit_stage_goal_distance(reference_pose, travel_goal), False

        final_goal_pose.header.stamp = self.get_clock().now().to_msg()
        if self._final_goal_keep_current_yaw_when_close and reference_pose is not None and distance_to_final <= 1.0e-6:
            final_goal_pose.pose.orientation = reference_pose.pose.orientation
        return final_goal_pose, True

    def _goal_pose_changed(self, next_goal_pose: PoseStamped) -> bool:
        if self._last_sent_goal_pose is None:
            return True

        dx = next_goal_pose.pose.position.x - self._last_sent_goal_pose.pose.position.x
        dy = next_goal_pose.pose.position.y - self._last_sent_goal_pose.pose.position.y
        distance = math.hypot(dx, dy)
        yaw_delta = normalize_angle(
            quaternion_to_yaw(next_goal_pose.pose.orientation)
            - quaternion_to_yaw(self._last_sent_goal_pose.pose.orientation)
        )
        return distance >= self._goal_update_distance_threshold or abs(yaw_delta) >= self._goal_update_yaw_threshold

    def _send_navigation_goal(self, goal_pose: PoseStamped, is_final_goal: bool) -> None:
        self._request_head_mode("nav")

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            f"Nav2 开始导航到{'最终抓取工作位' if is_final_goal else '阶段导航点'}："
            f" x={goal_pose.pose.position.x:.3f}, y={goal_pose.pose.position.y:.3f}, "
            f"yaw={math.degrees(quaternion_to_yaw(goal_pose.pose.orientation)):.1f}deg, "
            f"frame={goal_pose.header.frame_id or '<empty>'}, retry={self._retry_count}"
        )

        self._goal_in_flight = True
        self._active_goal_is_final = is_final_goal
        self._goal_cancel_mode = None
        self._last_sent_goal_pose = goal_pose
        send_future = self._navigate_action.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"发送 Nav2 目标失败：{exc}")
            self._goal_in_flight = False
            self._schedule_retry()
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning("Nav2 拒绝了当前导航目标，稍后重试。")
            self._goal_in_flight = False
            self._schedule_retry()
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        self._goal_in_flight = False
        self._goal_handle = None
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"获取 Nav2 结果失败：{exc}")
            self._schedule_retry()
            return

        if result is None:
            self.get_logger().warning("Nav2 返回空结果，稍后重试。")
            self._schedule_retry()
            return

        status = int(result.status)
        if self._goal_cancel_mode == "stage_advance":
            self._goal_cancel_mode = None
            self._goal_in_flight = False
            self._goal_handle = None
            self._last_sent_goal_pose = None
            self._navigation_retry_exhausted = False
            self._retry_count = 0
            self._retry_after = time.monotonic() + 0.2
            self.get_logger().info("Nav2 已满足阶段点粗到位阈值，继续切换到下一段路径。")
            return

        if self._goal_cancel_mode == "final_handoff":
            self._goal_cancel_mode = None
            self._complete_navigation_success("Nav2 已完成粗到位交棒，准备触发抓取任务。")
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            if self._active_goal_is_final:
                self._last_completed_stage_goal = None
                self._complete_navigation_success("Nav2 已到达抓取工作位，准备触发抓取任务。")
            else:
                if self._last_sent_goal_pose is not None:
                    self._last_completed_stage_goal = clone_pose_stamped(self._last_sent_goal_pose)
                self._last_sent_goal_pose = None
                self._navigation_retry_exhausted = False
                self._retry_count = 0
                self._retry_after = time.monotonic() + 0.2
                self.get_logger().info("Nav2 已到达阶段导航点，继续逼近最终抓取工作位。")
            return

        status_name = {
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }.get(status, f"status={status}")
        self.get_logger().warning(f"Nav2 导航未成功，结果={status_name}，稍后重试。")
        self._schedule_retry()

    def _schedule_retry(self) -> None:
        if self._retry_count >= self._max_retry_count:
            self._navigation_retry_exhausted = True
            self.get_logger().error("Nav2 重试次数已耗尽，停止自动导航。")
            return
        self._retry_count += 1
        self._retry_after = time.monotonic() + self._retry_delay_sec

    def _request_task_start(self) -> None:
        if self._task_started:
            return
        if not self._task_start_client.wait_for_service(timeout_sec=0.1):
            return
        future = self._task_start_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_task_start_response)
        self._task_started = True

    def _on_task_start_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"调用抓取启动服务失败：{exc}")
            self._task_started = False
            return

        if response is None:
            self._task_started = False
            return

        if response.success:
            self.get_logger().info("抓取任务已由 Nav2 成功触发。")
        else:
            self.get_logger().warning(f"抓取任务启动被拒绝：{response.message}")
            self._task_started = False

    def _request_nav_stack_state(self) -> None:
        if self._nav_stack_active or self._nav_state_request_in_flight:
            return
        now = time.monotonic()
        if now < self._nav_state_retry_after:
            return
        if self._navigate_action.wait_for_server(timeout_sec=0.05):
            self._nav_stack_active = True
            self.get_logger().info("NavigateToPose action 已就绪，按 Nav2 ACTIVE 处理。")
            return
        if not self._bt_navigator_state_client.wait_for_service(timeout_sec=0.05):
            self._nav_state_retry_after = now + 0.5
            return
        future = self._bt_navigator_state_client.call_async(GetState.Request())
        self._nav_state_request_in_flight = True
        future.add_done_callback(self._on_nav_stack_state_response)

    def _on_nav_stack_state_response(self, future) -> None:
        self._nav_state_request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"查询 bt_navigator 生命周期状态失败：{exc}")
            self._nav_state_retry_after = time.monotonic() + 0.5
            return

        if response is None:
            self._nav_state_retry_after = time.monotonic() + 0.5
            return

        if int(response.current_state.id) == State.PRIMARY_STATE_ACTIVE:
            if not self._nav_stack_active:
                self.get_logger().info("Nav2 生命周期节点已进入 ACTIVE，允许发起导航。")
            self._nav_stack_active = True
            return

        self._nav_state_retry_after = time.monotonic() + 0.5

    def _on_timer(self) -> None:
        now = time.monotonic()
        if not self._nav_stack_active:
            self._request_nav_stack_state()

        if not self._map_ready:
            self._maybe_mark_map_ready()

        if self._navigation_succeeded:
            if not self._task_started:
                self._request_head_mode("observe")
                if self._head_goal_in_flight:
                    return
                if (now - self._navigation_completed_at) >= self._task_start_delay_sec:
                    self._request_task_start()
            return

        navigation_target_pose = self._target_pose_for_navigation()
        if navigation_target_pose is not None:
            self._request_head_mode("nav")
            if not self._request_nav_arm_stow():
                return

        if self._goal_in_flight:
            if self._maybe_advance_stage_goal():
                return
            if self._maybe_trigger_coarse_handoff():
                return
            return

        if navigation_target_pose is None or not self._map_ready:
            if now - self._last_wait_log >= 2.0:
                self._last_wait_log = now
                if navigation_target_pose is None:
                    self.get_logger().info("Nav2 导航器仍在等待目标药盒位姿。")
                else:
                    wait_parts = [f"known_cells={self._known_map_cells}"]
                    if self._require_robot_inside_map:
                        wait_parts.append(
                            f"robot_inside_map={self._robot_pose_inside_map(margin=self._map_ready_margin)}"
                        )
                    bounds = self._map_bounds()
                    if bounds is not None:
                        xmin, xmax, ymin, ymax = bounds
                        wait_parts.append(f"bounds=[{xmin:.2f},{xmax:.2f}] x [{ymin:.2f},{ymax:.2f}]")
                    self.get_logger().info(
                        "Nav2 导航器仍在等待地图准备完成，" + "，".join(wait_parts) + "。"
                    )
            return

        if now < self._retry_after:
            return

        if self._navigation_retry_exhausted:
            return

        if not self._nav_stack_active:
            if now - self._last_wait_log >= 2.0:
                self._last_wait_log = now
                self.get_logger().info("Nav2 生命周期节点尚未 ACTIVE，继续等待。")
            return

        if not self._navigate_action.wait_for_server(timeout_sec=self._navigate_server_wait_sec):
            if now - self._last_wait_log >= 2.0:
                self._last_wait_log = now
                self.get_logger().info("Nav2 action server 尚未就绪，继续等待。")
            return

        goal_pose, is_final_goal = self._select_navigation_goal(navigation_target_pose)
        if goal_pose is None:
            return
        if self._retry_count == 0 and not self._goal_pose_changed(goal_pose):
            return
        self._send_navigation_goal(goal_pose, is_final_goal)

    def _complete_navigation_success(self, reason: str) -> None:
        if self._navigation_succeeded:
            return
        self._goal_in_flight = False
        self._goal_handle = None
        self._goal_cancel_mode = None
        self._navigation_succeeded = True
        self._navigation_completed_at = time.monotonic()
        self._request_head_mode("observe")
        self.get_logger().info(reason)

    def _maybe_advance_stage_goal(self) -> bool:
        if (
            not self._goal_in_flight
            or self._active_goal_is_final
            or self._stage_goal_completion_distance_threshold <= 0.0
            or self._last_sent_goal_pose is None
        ):
            return False

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return False

        distance = math.hypot(
            self._last_sent_goal_pose.pose.position.x - robot_pose.pose.position.x,
            self._last_sent_goal_pose.pose.position.y - robot_pose.pose.position.y,
        )
        if distance > self._stage_goal_completion_distance_threshold:
            return False

        if self._goal_cancel_mode is not None:
            return True
        if self._goal_handle is None:
            return False

        self._goal_cancel_mode = "stage_advance"
        self.get_logger().info(
            f"Nav2 阶段点已进入粗到位阈值（distance={distance:.3f}m），取消当前目标并切到下一段。"
        )
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._on_goal_cancel_response)
        return True

    def _maybe_trigger_coarse_handoff(self) -> bool:
        navigation_target_pose = self._target_pose_for_navigation()
        if (
            not self._goal_in_flight
            or not self._active_goal_is_final
            or self._coarse_success_distance_threshold <= 0.0
            or navigation_target_pose is None
        ):
            return False

        robot_pose = self._lookup_robot_pose()
        final_goal_pose = self._build_goal_pose(navigation_target_pose)
        if robot_pose is None or final_goal_pose is None:
            return False

        distance = math.hypot(
            final_goal_pose.pose.position.x - robot_pose.pose.position.x,
            final_goal_pose.pose.position.y - robot_pose.pose.position.y,
        )
        if distance > self._coarse_success_distance_threshold:
            return False

        if self._goal_cancel_mode is not None:
            return True

        if self._goal_handle is None:
            return False

        self._goal_cancel_mode = "final_handoff"
        self.get_logger().info(
            f"Nav2 已进入抓取工作区附近（distance={distance:.3f}m），取消导航目标并交棒抓取流程。"
        )
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._on_goal_cancel_response)
        return True

    def _on_goal_cancel_response(self, future) -> None:
        try:
            future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"取消 Nav2 目标时失败：{exc}")


def main() -> None:
    rclpy.init()
    node = Nav2TargetNavigator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
