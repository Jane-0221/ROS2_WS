#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from threading import Event, Lock, Thread
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    DisplayTrajectory,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
    RobotState,
    RobotTrajectory,
)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

from medipick_planning_interfaces.srv import (
    PlanToPose,
    RemoveCollisionObject,
    UpsertCollisionBox,
)


ERROR_CODE_NAMES = {
    value: name
    for name, value in MoveItErrorCodes.__dict__.items()
    if name.isupper() and isinstance(value, int)
}


def rotate_vector_by_quaternion(vector, quaternion):
    vx, vy, vz = vector
    qx, qy, qz, qw = quaternion

    ix = qw * vx + qy * vz - qz * vy
    iy = qw * vy + qz * vx - qx * vz
    iz = qw * vz + qx * vy - qy * vx
    iw = -qx * vx - qy * vy - qz * vz

    return (
        ix * qw + iw * -qx + iy * -qz - iz * -qy,
        iy * qw + iw * -qy + iz * -qx - ix * -qz,
        iz * qw + iw * -qz + ix * -qy - iy * -qx,
    )


@dataclass
class PlanResult:
    success: bool
    message: str
    backend_name: str
    resolved_group_name: str
    resolved_pose_link: str
    planning_frame: str
    moveit_error_code: int
    planning_time: float
    joint_trajectory: JointTrajectory
    final_joint_state: JointState
    display_trajectory: Optional[DisplayTrajectory]


class MockPlanningBackend:
    def __init__(self, planning_frame: str) -> None:
        self._planning_frame = planning_frame
        self._objects: Dict[str, Dict[str, object]] = {}

    @property
    def name(self) -> str:
        return "mock"

    def plan_to_pose(self, request: PlanToPose.Request, group_name: str, pose_link: str) -> PlanResult:
        target_pose = request.target_pose
        frame_id = target_pose.header.frame_id or self._planning_frame
        message = (
            "Mock backend does not produce a deployable trajectory. "
            f"Captured request for group '{group_name}' toward frame '{frame_id}'."
        )
        return PlanResult(
            success=False,
            message=message,
            backend_name=self.name,
            resolved_group_name=group_name,
            resolved_pose_link=pose_link,
            planning_frame=frame_id,
            moveit_error_code=MoveItErrorCodes.FAILURE,
            planning_time=0.0,
            joint_trajectory=JointTrajectory(),
            final_joint_state=JointState(),
            display_trajectory=None,
        )

    def upsert_collision_box(
        self,
        object_id: str,
        pose: PoseStamped,
        size_x: float,
        size_y: float,
        size_z: float,
    ) -> Tuple[bool, str]:
        self._objects[object_id] = {
            "pose": pose,
            "size": (size_x, size_y, size_z),
        }
        return True, f"Stored box '{object_id}' in mock planning scene."

    def remove_collision_object(self, object_id: str) -> Tuple[bool, str]:
        existed = object_id in self._objects
        self._objects.pop(object_id, None)
        if existed:
            return True, f"Removed object '{object_id}' from mock planning scene."
        return True, f"Object '{object_id}' was not present in mock planning scene."

    def clear_scene(self) -> Tuple[bool, str]:
        count = len(self._objects)
        self._objects.clear()
        return True, f"Cleared {count} mock collision objects."

    def shutdown(self) -> None:
        return None


class MoveGroupPlanningBackend:
    def __init__(self, node: Node, planning_frame: str, callback_group: ReentrantCallbackGroup) -> None:
        self._node = node
        self._planning_frame = planning_frame
        self._callback_group = callback_group
        self._known_objects: Dict[str, CollisionObject] = {}
        self._client_node = rclpy.create_node(
            "medipick_move_group_backend_client",
            context=self._node.context,
        )
        self._client_executor = MultiThreadedExecutor(num_threads=2, context=self._node.context)
        self._client_executor.add_node(self._client_node)
        self._client_executor_thread = Thread(target=self._client_executor.spin, daemon=True)
        self._client_executor_thread.start()

        self._goal_position_tolerance = self._positive_or_default(
            float(self._node.get_parameter("goal_position_tolerance").value),
            default=0.005,
        )
        self._goal_orientation_tolerance = self._positive_or_default(
            float(self._node.get_parameter("goal_orientation_tolerance").value),
            default=0.02,
        )
        self._service_wait_timeout = self._positive_or_default(
            float(self._node.get_parameter("service_wait_timeout").value),
            default=3.0,
        )
        self._request_timeout = self._positive_or_default(
            float(self._node.get_parameter("request_timeout").value),
            default=20.0,
        )
        self._default_allowed_planning_time = self._positive_or_default(
            float(self._node.get_parameter("default_allowed_planning_time").value),
            default=5.0,
        )
        self._default_num_planning_attempts = max(
            1,
            int(self._node.get_parameter("default_num_planning_attempts").value),
        )
        self._default_pipeline_id = str(self._node.get_parameter("default_pipeline_id").value).strip()
        self._default_planner_id = str(self._node.get_parameter("default_planner_id").value).strip()
        self._ik_pose_link = str(self._node.get_parameter("ik_pose_link").value).strip()
        self._tool_reference_link = str(self._node.get_parameter("tool_reference_link").value).strip()
        self._tool_to_ik_offset = (
            float(self._node.get_parameter("tool_to_ik_offset_x").value),
            float(self._node.get_parameter("tool_to_ik_offset_y").value),
            float(self._node.get_parameter("tool_to_ik_offset_z").value),
        )

        motion_plan_service = str(self._node.get_parameter("motion_plan_service_name").value)
        apply_scene_service = str(self._node.get_parameter("apply_planning_scene_service_name").value)

        self._plan_client = self._client_node.create_client(
            GetMotionPlan,
            motion_plan_service,
        )
        self._apply_scene_client = self._client_node.create_client(
            ApplyPlanningScene,
            apply_scene_service,
        )

    @property
    def name(self) -> str:
        return "move_group"

    def plan_to_pose(self, request: PlanToPose.Request, group_name: str, pose_link: str) -> PlanResult:
        frame_id = request.target_pose.header.frame_id or self._planning_frame
        normalized_pose, pose_error = self._normalize_pose(request.target_pose, frame_id)
        if pose_error is not None:
            return PlanResult(
                success=False,
                message=pose_error,
                backend_name=self.name,
                resolved_group_name=group_name,
                resolved_pose_link=pose_link,
                planning_frame=frame_id,
                moveit_error_code=MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS,
                planning_time=0.0,
                joint_trajectory=JointTrajectory(),
                final_joint_state=JointState(),
                display_trajectory=None,
            )

        planning_pose, goal_pose_link = self._resolve_goal_pose(normalized_pose, pose_link)

        try:
            motion_plan_request = self._build_motion_plan_request(
                request=request,
                group_name=group_name,
                pose_link=goal_pose_link,
                normalized_pose=planning_pose,
            )
        except ValueError as exc:
            return PlanResult(
                success=False,
                message=str(exc),
                backend_name=self.name,
                resolved_group_name=group_name,
                resolved_pose_link=goal_pose_link,
                planning_frame=frame_id,
                moveit_error_code=MoveItErrorCodes.INVALID_ROBOT_STATE,
                planning_time=0.0,
                joint_trajectory=JointTrajectory(),
                final_joint_state=JointState(),
                display_trajectory=None,
            )

        motion_plan = GetMotionPlan.Request()
        motion_plan.motion_plan_request = motion_plan_request

        response, error = self._call_service(
            client=self._plan_client,
            request=motion_plan,
            service_name="motion plan",
        )
        if error is not None:
            return PlanResult(
                success=False,
                message=error,
                backend_name=self.name,
                resolved_group_name=group_name,
                resolved_pose_link=goal_pose_link,
                planning_frame=frame_id,
                moveit_error_code=MoveItErrorCodes.COMMUNICATION_FAILURE,
                planning_time=0.0,
                joint_trajectory=JointTrajectory(),
                final_joint_state=JointState(),
                display_trajectory=None,
            )

        motion_plan_response = response.motion_plan_response
        error_code = motion_plan_response.error_code.val
        error_name = ERROR_CODE_NAMES.get(error_code, "UNKNOWN")
        joint_trajectory = motion_plan_response.trajectory.joint_trajectory
        final_joint_state = self._build_final_joint_state(joint_trajectory)
        display_trajectory = self._build_display_trajectory(
            trajectory_start=motion_plan_response.trajectory_start,
            robot_trajectory=motion_plan_response.trajectory,
        )

        if error_code != MoveItErrorCodes.SUCCESS:
            return PlanResult(
                success=False,
                message=(
                    "MoveIt failed to produce a valid trajectory "
                    f"({error_name}, code={error_code})."
                ),
                backend_name=self.name,
                resolved_group_name=group_name,
                resolved_pose_link=goal_pose_link,
                planning_frame=frame_id,
                moveit_error_code=error_code,
                planning_time=motion_plan_response.planning_time,
                joint_trajectory=JointTrajectory(),
                final_joint_state=JointState(),
                display_trajectory=None,
            )

        if not joint_trajectory.points:
            return PlanResult(
                success=False,
                message="MoveIt returned success but produced an empty joint trajectory.",
                backend_name=self.name,
                resolved_group_name=group_name,
                resolved_pose_link=goal_pose_link,
                planning_frame=frame_id,
                moveit_error_code=error_code,
                planning_time=motion_plan_response.planning_time,
                joint_trajectory=joint_trajectory,
                final_joint_state=final_joint_state,
                display_trajectory=display_trajectory,
            )

        return PlanResult(
            success=True,
            message=(
                "MoveIt produced a collision-aware trajectory "
                f"in {motion_plan_response.planning_time:.3f}s "
                f"using goal link '{goal_pose_link}'."
            ),
            backend_name=self.name,
            resolved_group_name=group_name,
            resolved_pose_link=goal_pose_link,
            planning_frame=frame_id,
            moveit_error_code=error_code,
            planning_time=motion_plan_response.planning_time,
            joint_trajectory=joint_trajectory,
            final_joint_state=final_joint_state,
            display_trajectory=display_trajectory,
        )

    def upsert_collision_box(
        self,
        object_id: str,
        pose: PoseStamped,
        size_x: float,
        size_y: float,
        size_z: float,
    ) -> Tuple[bool, str]:
        if not object_id.strip():
            return False, "Collision object id must not be empty."

        if min(size_x, size_y, size_z) <= 0.0:
            return False, "Collision box dimensions must all be positive."

        frame_id = pose.header.frame_id or self._planning_frame
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.id = object_id
        collision_object.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [size_x, size_y, size_z]
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose.pose)

        success, message = self._apply_collision_objects([collision_object], "upsert collision box")
        if success:
            self._known_objects[object_id] = collision_object
        return success, message

    def remove_collision_object(self, object_id: str) -> Tuple[bool, str]:
        if not object_id.strip():
            return False, "Collision object id must not be empty."

        collision_object = CollisionObject()
        collision_object.header.frame_id = self._planning_frame
        collision_object.id = object_id
        collision_object.operation = CollisionObject.REMOVE

        success, message = self._apply_collision_objects([collision_object], "remove collision object")
        if success:
            self._known_objects.pop(object_id, None)
        return success, message

    def clear_scene(self) -> Tuple[bool, str]:
        if not self._known_objects:
            return True, "Planning scene is already empty from the server's perspective."

        removals = []
        for object_id, collision_object in self._known_objects.items():
            removal = CollisionObject()
            removal.header.frame_id = collision_object.header.frame_id or self._planning_frame
            removal.id = object_id
            removal.operation = CollisionObject.REMOVE
            removals.append(removal)

        success, message = self._apply_collision_objects(removals, "clear planning scene")
        if success:
            count = len(removals)
            self._known_objects.clear()
            return True, f"Cleared {count} collision objects from MoveIt planning scene."
        return False, message

    def _apply_collision_objects(
        self,
        collision_objects: list[CollisionObject],
        operation_name: str,
    ) -> Tuple[bool, str]:
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects = collision_objects

        request = ApplyPlanningScene.Request()
        request.scene = planning_scene

        response, error = self._call_service(
            client=self._apply_scene_client,
            request=request,
            service_name=operation_name,
        )
        if error is not None:
            return False, error
        if not response.success:
            return False, f"MoveIt rejected the request to {operation_name}."
        return True, f"MoveIt applied request to {operation_name}."

    def _build_motion_plan_request(
        self,
        request: PlanToPose.Request,
        group_name: str,
        pose_link: str,
        normalized_pose: PoseStamped,
    ):
        motion_plan_request = GetMotionPlan.Request().motion_plan_request
        motion_plan_request.group_name = group_name
        motion_plan_request.pipeline_id = self._default_pipeline_id
        requested_planner_id = request.planner_id.strip()
        motion_plan_request.planner_id = requested_planner_id or self._default_planner_id
        motion_plan_request.allowed_planning_time = self._positive_or_default(
            request.allowed_planning_time,
            default=self._default_allowed_planning_time,
        )
        motion_plan_request.num_planning_attempts = max(
            1,
            request.num_planning_attempts or self._default_num_planning_attempts,
        )
        motion_plan_request.max_velocity_scaling_factor = self._sanitize_scaling_factor(
            request.max_velocity_scaling,
        )
        motion_plan_request.max_acceleration_scaling_factor = self._sanitize_scaling_factor(
            request.max_acceleration_scaling,
        )
        if request.use_start_joint_state:
            start_state_error = self._populate_start_state(
                motion_plan_request.start_state,
                request.start_joint_state,
            )
            if start_state_error is not None:
                raise ValueError(start_state_error)
        else:
            motion_plan_request.start_state.is_diff = True
        motion_plan_request.goal_constraints.append(
            self._build_pose_goal_constraints(normalized_pose, pose_link)
        )
        return motion_plan_request

    def _build_pose_goal_constraints(self, target_pose: PoseStamped, pose_link: str) -> Constraints:
        constraints = Constraints()
        constraints.name = f"{pose_link}_pose_goal"

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = target_pose.header.frame_id or self._planning_frame
        position_constraint.link_name = pose_link
        position_constraint.weight = 1.0

        position_region = SolidPrimitive()
        position_region.type = SolidPrimitive.SPHERE
        position_region.dimensions = [self._goal_position_tolerance]
        position_constraint.constraint_region.primitives.append(position_region)
        position_constraint.constraint_region.primitive_poses.append(target_pose.pose)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = target_pose.header.frame_id or self._planning_frame
        orientation_constraint.link_name = pose_link
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = self._goal_orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = self._goal_orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = self._goal_orientation_tolerance
        orientation_constraint.weight = 1.0

        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        return constraints

    def _resolve_goal_pose(self, target_pose: PoseStamped, pose_link: str) -> Tuple[PoseStamped, str]:
        if (
            pose_link != self._tool_reference_link
            or not self._ik_pose_link
            or self._ik_pose_link == self._tool_reference_link
        ):
            return target_pose, pose_link

        goal_pose = PoseStamped()
        goal_pose.header = target_pose.header
        goal_pose.pose = target_pose.pose

        offset_world = rotate_vector_by_quaternion(
            self._tool_to_ik_offset,
            (
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.pose.orientation.w,
            ),
        )
        goal_pose.pose.position.x -= offset_world[0]
        goal_pose.pose.position.y -= offset_world[1]
        goal_pose.pose.position.z -= offset_world[2]
        return goal_pose, self._ik_pose_link

    def _call_service(self, client, request, service_name: str):
        self._node.get_logger().info(f"Calling MoveIt {service_name} service...")
        if not client.wait_for_service(timeout_sec=self._service_wait_timeout):
            return None, (
                f"MoveIt {service_name} interface is unavailable after "
                f"{self._service_wait_timeout:.1f}s."
            )

        future = client.call_async(request)
        done_event = Event()
        future.add_done_callback(lambda _: done_event.set())

        if not done_event.wait(timeout=self._request_timeout):
            future.cancel()
            return None, (
                f"MoveIt {service_name} request timed out after "
                f"{self._request_timeout:.1f}s."
            )

        try:
            response = future.result()
        except Exception as exc:
            return None, f"MoveIt {service_name} request failed: {exc}"

        if response is None:
            return None, f"MoveIt {service_name} returned an empty response."

        self._node.get_logger().info(f"MoveIt {service_name} service returned.")
        return response, None

    def _populate_start_state(self, robot_state: RobotState, start_joint_state: JointState) -> Optional[str]:
        joint_count = len(start_joint_state.name)
        if joint_count == 0:
            return "Requested custom start joint state is empty."

        if len(start_joint_state.position) != joint_count:
            return "Custom start joint state has mismatched joint names and positions."

        robot_state.joint_state = start_joint_state
        robot_state.is_diff = False
        return None

    @staticmethod
    def _build_final_joint_state(joint_trajectory: JointTrajectory) -> JointState:
        final_joint_state = JointState()
        final_joint_state.name = list(joint_trajectory.joint_names)
        if joint_trajectory.points:
            final_joint_state.position = list(joint_trajectory.points[-1].positions)
        return final_joint_state

    @staticmethod
    def _build_display_trajectory(
        trajectory_start: RobotState,
        robot_trajectory: RobotTrajectory,
    ) -> DisplayTrajectory:
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = trajectory_start
        display_trajectory.trajectory.append(robot_trajectory)
        return display_trajectory

    def _normalize_pose(self, pose: PoseStamped, frame_id: str) -> Tuple[PoseStamped, Optional[str]]:
        normalized_pose = PoseStamped()
        normalized_pose.header = pose.header
        normalized_pose.header.frame_id = frame_id
        normalized_pose.pose = pose.pose

        qx = normalized_pose.pose.orientation.x
        qy = normalized_pose.pose.orientation.y
        qz = normalized_pose.pose.orientation.z
        qw = normalized_pose.pose.orientation.w
        norm = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
        if norm < 1e-9:
            return normalized_pose, "Target pose quaternion is invalid because its norm is zero."

        normalized_pose.pose.orientation.x = qx / norm
        normalized_pose.pose.orientation.y = qy / norm
        normalized_pose.pose.orientation.z = qz / norm
        normalized_pose.pose.orientation.w = qw / norm
        return normalized_pose, None

    @staticmethod
    def _sanitize_scaling_factor(value: float) -> float:
        if 0.0 < value <= 1.0:
            return float(value)
        return 1.0

    @staticmethod
    def _positive_or_default(value: float, default: float) -> float:
        return float(value) if value > 0.0 else float(default)

    def shutdown(self) -> None:
        self._client_executor.shutdown()
        self._client_node.destroy_node()
        if self._client_executor_thread.is_alive():
            self._client_executor_thread.join(timeout=1.0)


class PlanningServer(Node):
    def __init__(self) -> None:
        super().__init__("medipick_planning_server")

        self.declare_parameter("backend", "auto")
        self.declare_parameter("default_group_name", "arm")
        self.declare_parameter("default_pose_link", "sucker_link")
        self.declare_parameter("default_planning_frame", "world")
        self.declare_parameter("goal_position_tolerance", 0.005)
        self.declare_parameter("goal_orientation_tolerance", 0.02)
        self.declare_parameter("default_allowed_planning_time", 5.0)
        self.declare_parameter("default_num_planning_attempts", 3)
        self.declare_parameter("default_pipeline_id", "")
        self.declare_parameter("default_planner_id", "")
        self.declare_parameter("ik_pose_link", "r6_link")
        self.declare_parameter("tool_reference_link", "sucker_link")
        self.declare_parameter("tool_to_ik_offset_x", 0.1112)
        self.declare_parameter("tool_to_ik_offset_y", -0.0235)
        self.declare_parameter("tool_to_ik_offset_z", 0.0)
        self.declare_parameter("motion_plan_service_name", "/plan_kinematic_path")
        self.declare_parameter("apply_planning_scene_service_name", "/apply_planning_scene")
        self.declare_parameter("service_wait_timeout", 3.0)
        self.declare_parameter("request_timeout", 20.0)
        self.declare_parameter("publish_display_trajectory", True)
        self.declare_parameter("display_trajectory_topic", "/display_planned_path")

        self._lock = Lock()
        self._service_callback_group = ReentrantCallbackGroup()
        self._client_callback_group = ReentrantCallbackGroup()

        self._default_group_name = str(self.get_parameter("default_group_name").value)
        self._default_pose_link = str(self.get_parameter("default_pose_link").value)
        self._default_planning_frame = str(self.get_parameter("default_planning_frame").value)
        self._publish_display_trajectory = bool(self.get_parameter("publish_display_trajectory").value)
        display_trajectory_topic = str(self.get_parameter("display_trajectory_topic").value)
        self._backend = self._build_backend(str(self.get_parameter("backend").value))
        self._display_trajectory_publisher = self.create_publisher(
            DisplayTrajectory,
            display_trajectory_topic,
            10,
        )

        self.create_service(
            PlanToPose,
            "~/plan_to_pose",
            self._handle_plan_to_pose,
            callback_group=self._service_callback_group,
        )
        self.create_service(
            UpsertCollisionBox,
            "~/upsert_collision_box",
            self._handle_upsert_collision_box,
            callback_group=self._service_callback_group,
        )
        self.create_service(
            RemoveCollisionObject,
            "~/remove_collision_object",
            self._handle_remove_collision_object,
            callback_group=self._service_callback_group,
        )
        self.create_service(
            Trigger,
            "~/clear_scene",
            self._handle_clear_scene,
            callback_group=self._service_callback_group,
        )

        self.get_logger().info(
            f"Planning server ready with backend '{self._backend.name}'. "
            "Services: ~/plan_to_pose, ~/upsert_collision_box, ~/remove_collision_object, ~/clear_scene"
        )

    def _build_backend(self, backend_name: str):
        requested_backend = backend_name.strip().lower()
        if requested_backend == "moveit_py":
            self.get_logger().warning(
                "Backend name 'moveit_py' is kept as an alias, but the server now uses move_group interfaces."
            )
            requested_backend = "move_group"

        if requested_backend == "auto":
            requested_backend = "move_group"

        if requested_backend == "mock":
            return MockPlanningBackend(self._default_planning_frame)

        if requested_backend != "move_group":
            self.get_logger().warning(
                f"Unsupported backend '{backend_name}', using move_group backend instead."
            )

        return MoveGroupPlanningBackend(
            node=self,
            planning_frame=self._default_planning_frame,
            callback_group=self._client_callback_group,
        )

    def _resolve_group_name(self, requested_group_name: str) -> str:
        group_name = requested_group_name.strip()
        return group_name or self._default_group_name

    def _resolve_pose_link(self, requested_pose_link: str) -> str:
        pose_link = requested_pose_link.strip()
        return pose_link or self._default_pose_link

    def _handle_plan_to_pose(self, request: PlanToPose.Request, response: PlanToPose.Response) -> PlanToPose.Response:
        group_name = self._resolve_group_name(request.group_name)
        pose_link = self._resolve_pose_link(request.pose_link)
        self.get_logger().info(
            f"Received plan_to_pose request: group='{group_name}', pose_link='{pose_link}', request_id='{request.request_id}'"
        )

        with self._lock:
            result = self._backend.plan_to_pose(request, group_name, pose_link)

        response.success = result.success
        response.message = result.message
        response.backend_name = result.backend_name
        response.resolved_group_name = result.resolved_group_name
        response.resolved_pose_link = result.resolved_pose_link
        response.planning_frame = result.planning_frame
        response.moveit_error_code = result.moveit_error_code
        response.planning_time = result.planning_time
        response.joint_trajectory = result.joint_trajectory
        response.final_joint_state = result.final_joint_state
        if self._publish_display_trajectory and result.display_trajectory is not None:
            self._display_trajectory_publisher.publish(result.display_trajectory)
        return response

    def _handle_upsert_collision_box(
        self,
        request: UpsertCollisionBox.Request,
        response: UpsertCollisionBox.Response,
    ) -> UpsertCollisionBox.Response:
        with self._lock:
            success, message = self._backend.upsert_collision_box(
                object_id=request.object_id,
                pose=request.pose,
                size_x=request.size_x,
                size_y=request.size_y,
                size_z=request.size_z,
            )
        response.success = success
        response.message = message
        return response

    def _handle_remove_collision_object(
        self,
        request: RemoveCollisionObject.Request,
        response: RemoveCollisionObject.Response,
    ) -> RemoveCollisionObject.Response:
        with self._lock:
            success, message = self._backend.remove_collision_object(request.object_id)
        response.success = success
        response.message = message
        return response

    def _handle_clear_scene(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        with self._lock:
            success, message = self._backend.clear_scene()
        response.success = success
        response.message = message
        return response

    def shutdown_backend(self) -> None:
        shutdown = getattr(self._backend, "shutdown", None)
        if callable(shutdown):
            shutdown()


def main() -> None:
    rclpy.init()
    node = PlanningServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.shutdown_backend()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
