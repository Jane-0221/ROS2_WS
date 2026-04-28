#!/usr/bin/python3

from __future__ import annotations

import math
from typing import Optional, Sequence

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pick_task_shared import DEFAULT_STOW_STATE_POSITIONS


Vector3 = tuple[float, float, float]


def clone_pose_stamped(msg: PoseStamped) -> PoseStamped:
    clone = PoseStamped()
    clone.header = msg.header
    clone.pose = clone_pose(msg.pose)
    return clone


def clone_pose(msg: Pose) -> Pose:
    clone = Pose()
    clone.position.x = msg.position.x
    clone.position.y = msg.position.y
    clone.position.z = msg.position.z
    clone.orientation.x = msg.orientation.x
    clone.orientation.y = msg.orientation.y
    clone.orientation.z = msg.orientation.z
    clone.orientation.w = msg.orientation.w
    return clone


def interpolate_pose_stamped(start: PoseStamped, end: PoseStamped, ratio: float) -> PoseStamped:
    result = PoseStamped()
    result.header = end.header if end.header.frame_id else start.header
    result.pose.position.x = start.pose.position.x + (end.pose.position.x - start.pose.position.x) * ratio
    result.pose.position.y = start.pose.position.y + (end.pose.position.y - start.pose.position.y) * ratio
    result.pose.position.z = start.pose.position.z + (end.pose.position.z - start.pose.position.z) * ratio
    result.pose.orientation = clone_pose(end.pose).orientation
    return result


def build_stow_joint_state() -> JointState:
    msg = JointState()
    msg.name = list(DEFAULT_STOW_STATE_POSITIONS.keys())
    msg.position = list(DEFAULT_STOW_STATE_POSITIONS.values())
    return msg


def arm_only_joint_state(joint_state: JointState) -> JointState:
    msg = JointState()
    for joint_name, joint_position_value in zip(joint_state.name, joint_state.position):
        if joint_name == "raise_joint":
            continue
        msg.name.append(joint_name)
        msg.position.append(joint_position_value)
    return msg


def joint_state_to_trajectory(
    joint_state: JointState,
    duration_sec: float,
    start_positions: Optional[dict[str, float]] = None,
) -> JointTrajectory:
    trajectory = JointTrajectory()
    trajectory.joint_names = list(joint_state.name)
    if start_positions:
        start_point = JointTrajectoryPoint()
        start_point.positions = [
            start_positions.get(name, position) for name, position in zip(joint_state.name, joint_state.position)
        ]
        start_point.time_from_start = duration_to_msg(0.0)
        trajectory.points.append(start_point)
    point = JointTrajectoryPoint()
    point.positions = list(joint_state.position)
    point.time_from_start = duration_to_msg(duration_sec)
    trajectory.points = [point]
    if start_positions:
        trajectory.points = [start_point, point]
    return trajectory


def filter_trajectory_joints(
    trajectory: JointTrajectory,
    allowed_joint_names: set[str],
) -> JointTrajectory:
    filtered = JointTrajectory()
    keep_indices = [
        index for index, joint_name in enumerate(trajectory.joint_names) if joint_name in allowed_joint_names
    ]
    filtered.joint_names = [trajectory.joint_names[index] for index in keep_indices]

    for point in trajectory.points:
        new_point = JointTrajectoryPoint()
        new_point.positions = [point.positions[index] for index in keep_indices]
        if point.velocities:
            new_point.velocities = [point.velocities[index] for index in keep_indices]
        if point.accelerations:
            new_point.accelerations = [point.accelerations[index] for index in keep_indices]
        if point.effort:
            new_point.effort = [point.effort[index] for index in keep_indices]
        new_point.time_from_start = point.time_from_start
        filtered.points.append(new_point)

    return filtered


def reverse_joint_trajectory(trajectory: JointTrajectory) -> JointTrajectory:
    reversed_trajectory = JointTrajectory()
    reversed_trajectory.header = trajectory.header
    reversed_trajectory.joint_names = list(trajectory.joint_names)
    if len(trajectory.points) == 0:
        return reversed_trajectory

    total_duration = trajectory_duration(trajectory)
    reversed_points: list[JointTrajectoryPoint] = []
    for point in reversed(trajectory.points):
        reversed_point = JointTrajectoryPoint()
        reversed_point.positions = list(point.positions)
        if point.velocities:
            reversed_point.velocities = [-value for value in point.velocities]
        if point.accelerations:
            reversed_point.accelerations = list(point.accelerations)
        if point.effort:
            reversed_point.effort = list(point.effort)
        point_time = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) / 1e9
        reversed_point.time_from_start = duration_to_msg(total_duration - point_time)
        reversed_points.append(reversed_point)

    reversed_points.sort(key=lambda point: float(point.time_from_start.sec) + float(point.time_from_start.nanosec) / 1e9)
    if reversed_points:
        reversed_points[0].time_from_start = duration_to_msg(0.0)
    reversed_trajectory.points = reversed_points
    return reversed_trajectory


def joint_state_positions(joint_state: JointState) -> dict[str, float]:
    return {name: pos for name, pos in zip(joint_state.name, joint_state.position)}


def joint_position(joint_state: JointState, joint_name: str, default: float = 0.0) -> float:
    positions = joint_state_positions(joint_state)
    return positions.get(joint_name, default)


def joint_state_from_trajectory_end(seed_joint_state: JointState, trajectory: JointTrajectory) -> JointState:
    result = JointState()
    result.name = list(seed_joint_state.name)
    result.position = list(seed_joint_state.position)
    if len(trajectory.joint_names) == 0 or len(trajectory.points) == 0:
        return result

    position_by_name = joint_state_positions(seed_joint_state)
    end_positions = trajectory.points[-1].positions
    for name, position in zip(trajectory.joint_names, end_positions):
        position_by_name[name] = position
    result.position = [position_by_name.get(name, 0.0) for name in result.name]
    return result


def merge_joint_states(base_joint_state: JointState, override_joint_state: JointState) -> JointState:
    result = JointState()
    position_by_name = joint_state_positions(base_joint_state)
    ordered_names = list(base_joint_state.name)

    for joint_name, joint_position_value in zip(override_joint_state.name, override_joint_state.position):
        if joint_name not in position_by_name:
            ordered_names.append(joint_name)
        position_by_name[joint_name] = joint_position_value

    if not ordered_names:
        ordered_names = list(override_joint_state.name)

    result.name = ordered_names
    result.position = [position_by_name.get(name, 0.0) for name in ordered_names]
    return result


def joint_states_to_interpolated_trajectory(
    joint_names: list[str],
    start_positions: list[float],
    end_positions: list[float],
    duration_sec: float,
    step_count: int,
) -> JointTrajectory:
    trajectory = JointTrajectory()
    trajectory.joint_names = list(joint_names)
    total_steps = max(2, step_count)
    for index in range(total_steps):
        ratio = float(index) / float(total_steps - 1)
        point = JointTrajectoryPoint()
        point.positions = [
            start + (end - start) * ratio
            for start, end in zip(start_positions, end_positions)
        ]
        point.time_from_start = duration_to_msg(duration_sec * ratio)
        trajectory.points.append(point)
    return trajectory


def canonicalize_revolute_joint_state(
    joint_state: JointState,
    reference_joint_state: JointState,
    revolute_joint_names: set[str],
) -> JointState:
    result = JointState()
    result.name = list(joint_state.name)
    reference_positions = joint_state_positions(reference_joint_state)
    result.position = []
    for joint_name, joint_position_value in zip(joint_state.name, joint_state.position):
        if joint_name in revolute_joint_names:
            joint_position_value = nearest_equivalent_angle(
                joint_position_value,
                reference_positions.get(joint_name, joint_position_value),
            )
        result.position.append(joint_position_value)
    return result


def predict_executed_final_joint_state(
    trajectory: JointTrajectory,
    seed_joint_state: JointState,
) -> JointState:
    return joint_state_from_trajectory_end(seed_joint_state, trajectory)


def trajectory_motion_metrics(
    trajectory: JointTrajectory,
    joint_names_of_interest: set[str],
) -> tuple[float, float]:
    if len(trajectory.points) < 2:
        return 0.0, 0.0

    total_motion = 0.0
    max_step = 0.0
    for previous_point, current_point in zip(trajectory.points[:-1], trajectory.points[1:]):
        for index, joint_name in enumerate(trajectory.joint_names):
            if joint_name not in joint_names_of_interest:
                continue
            delta = abs(current_point.positions[index] - previous_point.positions[index])
            total_motion += delta
            max_step = max(max_step, delta)
    return total_motion, max_step


def trajectory_joint_abs_max(
    trajectory: JointTrajectory,
    seed_joint_state: JointState,
    joint_name: str,
    revolute_joint_names: Optional[set[str]] = None,
) -> float:
    if joint_name not in trajectory.joint_names:
        return abs(joint_position(seed_joint_state, joint_name, 0.0))

    normalized = trajectory
    if revolute_joint_names:
        normalized = normalize_revolute_trajectory_continuity(
            trajectory,
            seed_joint_state=seed_joint_state,
            revolute_joint_names=revolute_joint_names,
        )

    joint_index = normalized.joint_names.index(joint_name)
    max_abs = abs(joint_position(seed_joint_state, joint_name, 0.0))
    for point in normalized.points:
        if joint_index < len(point.positions):
            max_abs = max(max_abs, abs(point.positions[joint_index]))
    return max_abs


def trajectory_joint_motion_extent(
    trajectory: JointTrajectory,
    seed_joint_state: JointState,
    joint_name: str,
    revolute_joint_names: Optional[set[str]] = None,
) -> float:
    if joint_name not in trajectory.joint_names:
        return 0.0

    start_position = joint_position(seed_joint_state, joint_name, 0.0)
    normalized = trajectory
    if revolute_joint_names:
        normalized = normalize_revolute_trajectory_continuity(
            trajectory,
            seed_joint_state=seed_joint_state,
            revolute_joint_names=revolute_joint_names,
        )

    joint_index = normalized.joint_names.index(joint_name)
    max_extent = 0.0
    for point in normalized.points:
        if joint_index < len(point.positions):
            position = point.positions[joint_index]
            if revolute_joint_names and joint_name in revolute_joint_names:
                position = nearest_equivalent_angle(position, start_position)
            max_extent = max(max_extent, abs(position - start_position))
    return max_extent


def joint_state_motion_extent(
    start_joint_state: JointState,
    end_joint_state: JointState,
    joint_name: str,
    revolute_joint_names: Optional[set[str]] = None,
) -> float:
    start_position = joint_position(start_joint_state, joint_name, 0.0)
    end_position = joint_position(end_joint_state, joint_name, start_position)
    if revolute_joint_names and joint_name in revolute_joint_names:
        end_position = nearest_equivalent_angle(end_position, start_position)
    return abs(end_position - start_position)


def joint_state_base_planar_motion_extent(
    start_joint_state: JointState,
    end_joint_state: JointState,
) -> float:
    start_x = joint_position(start_joint_state, "base_x", 0.0)
    start_y = joint_position(start_joint_state, "base_y", 0.0)
    end_x = joint_position(end_joint_state, "base_x", start_x)
    end_y = joint_position(end_joint_state, "base_y", start_y)
    return math.hypot(end_x - start_x, end_y - start_y)


def trajectory_base_planar_motion_extent(
    trajectory: JointTrajectory,
    seed_joint_state: JointState,
) -> float:
    if len(trajectory.points) == 0:
        return 0.0

    start_x = joint_position(seed_joint_state, "base_x", 0.0)
    start_y = joint_position(seed_joint_state, "base_y", 0.0)
    x_index = trajectory.joint_names.index("base_x") if "base_x" in trajectory.joint_names else -1
    y_index = trajectory.joint_names.index("base_y") if "base_y" in trajectory.joint_names else -1

    max_extent = 0.0
    for point in trajectory.points:
        current_x = start_x
        current_y = start_y
        if x_index >= 0 and x_index < len(point.positions):
            current_x = point.positions[x_index]
        if y_index >= 0 and y_index < len(point.positions):
            current_y = point.positions[y_index]
        max_extent = max(max_extent, math.hypot(current_x - start_x, current_y - start_y))
    return max_extent


def normalize_revolute_trajectory_continuity(
    trajectory: JointTrajectory,
    seed_joint_state: JointState,
    revolute_joint_names: set[str],
) -> JointTrajectory:
    if len(trajectory.points) == 0 or len(trajectory.joint_names) == 0:
        return trajectory

    normalized = JointTrajectory()
    normalized.joint_names = list(trajectory.joint_names)
    previous_positions = joint_state_positions(seed_joint_state)
    limit = 2.0 * math.pi

    for point in trajectory.points:
        new_point = JointTrajectoryPoint()
        new_point.positions = list(point.positions)
        if point.velocities:
            new_point.velocities = list(point.velocities)
        if point.accelerations:
            new_point.accelerations = list(point.accelerations)
        if point.effort:
            new_point.effort = list(point.effort)
        new_point.time_from_start = point.time_from_start

        for index, joint_name in enumerate(normalized.joint_names):
            if joint_name not in revolute_joint_names:
                previous_positions[joint_name] = new_point.positions[index]
                continue

            previous = previous_positions.get(joint_name, new_point.positions[index])
            best = new_point.positions[index]
            while best - previous > math.pi:
                best -= 2.0 * math.pi
            while best - previous < -math.pi:
                best += 2.0 * math.pi
            while best > limit:
                best -= 2.0 * math.pi
            while best < -limit:
                best += 2.0 * math.pi
            new_point.positions[index] = best
            previous_positions[joint_name] = best

        normalized.points.append(new_point)

    return normalized


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def nearest_equivalent_angle(target: float, reference: float) -> float:
    best = target
    while best - reference > math.pi:
        best -= 2.0 * math.pi
    while best - reference < -math.pi:
        best += 2.0 * math.pi
    return best


def rotate_xy(point: tuple[float, float], yaw: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    x, y = point
    return (cos_yaw * x - sin_yaw * y, sin_yaw * x + cos_yaw * y)


def pose_from_base_joint_state(joint_state: JointState, frame_id: str) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = joint_position(joint_state, "base_x")
    msg.pose.position.y = joint_position(joint_state, "base_y")
    msg.pose.position.z = 0.0
    msg.pose.orientation = yaw_to_quaternion(joint_position(joint_state, "base_theta"))
    return msg


def pose_error(target: Pose, achieved: Pose) -> tuple[float, float]:
    position_error = math.sqrt(
        (achieved.position.x - target.position.x) ** 2
        + (achieved.position.y - target.position.y) ** 2
        + (achieved.position.z - target.position.z) ** 2
    )

    dot = (
        target.orientation.x * achieved.orientation.x
        + target.orientation.y * achieved.orientation.y
        + target.orientation.z * achieved.orientation.z
        + target.orientation.w * achieved.orientation.w
    )
    dot = max(-1.0, min(1.0, abs(dot)))
    orientation_error = 2.0 * math.acos(dot)
    return position_error, orientation_error


def prepare_alignment_errors(target: Pose, achieved: Pose) -> tuple[float, float, float]:
    approach_axis = rotate_vector_by_quaternion(
        (1.0, 0.0, 0.0),
        (
            target.orientation.x,
            target.orientation.y,
            target.orientation.z,
            target.orientation.w,
        ),
    )
    axis_norm = math.sqrt(
        approach_axis[0] ** 2 + approach_axis[1] ** 2 + approach_axis[2] ** 2
    )
    position_delta = (
        achieved.position.x - target.position.x,
        achieved.position.y - target.position.y,
        achieved.position.z - target.position.z,
    )

    if axis_norm <= 1e-6:
        projection_error = math.sqrt(
            position_delta[0] ** 2 + position_delta[1] ** 2 + position_delta[2] ** 2
        )
        axial_error = projection_error
    else:
        approach_unit = (
            approach_axis[0] / axis_norm,
            approach_axis[1] / axis_norm,
            approach_axis[2] / axis_norm,
        )
        axial_delta = (
            position_delta[0] * approach_unit[0]
            + position_delta[1] * approach_unit[1]
            + position_delta[2] * approach_unit[2]
        )
        lateral_delta = (
            position_delta[0] - axial_delta * approach_unit[0],
            position_delta[1] - axial_delta * approach_unit[1],
            position_delta[2] - axial_delta * approach_unit[2],
        )
        projection_error = math.sqrt(
            lateral_delta[0] ** 2 + lateral_delta[1] ** 2 + lateral_delta[2] ** 2
        )
        axial_error = abs(axial_delta)

    _, orientation_error = pose_error(target, achieved)
    return projection_error, axial_error, orientation_error


def seconds_since(now, then) -> float:
    return (now - then).nanoseconds / 1e9


def trajectory_duration(trajectory: JointTrajectory) -> float:
    if len(trajectory.points) == 0:
        return 0.0
    duration = trajectory.points[-1].time_from_start
    return float(duration.sec) + float(duration.nanosec) / 1e9


def trajectory_effective_playback_duration(
    trajectory: JointTrajectory,
    fallback_step_sec: float = 0.15,
) -> float:
    if len(trajectory.points) == 0:
        return 0.0

    last_time = 0.0
    for index, point in enumerate(trajectory.points):
        time_from_start = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) / 1e9
        if time_from_start <= last_time:
            time_from_start = last_time + (fallback_step_sec if index > 0 else fallback_step_sec)
        last_time = time_from_start
    return last_time


def duration_to_msg(seconds: float) -> Duration:
    total_nanoseconds = int(max(0.0, seconds) * 1e9)
    message = Duration()
    message.sec = total_nanoseconds // 1_000_000_000
    message.nanosec = total_nanoseconds % 1_000_000_000
    return message


def normalize_xy(vector: Vector3, fallback: Vector3) -> Vector3:
    norm = math.hypot(vector[0], vector[1])
    if norm < 1e-6:
        return fallback
    return (vector[0] / norm, vector[1] / norm, 0.0)


def yaw_to_quaternion(yaw: float):
    quat = Pose().orientation
    quat.z = math.sin(yaw * 0.5)
    quat.w = math.cos(yaw * 0.5)
    return quat


def multiply_quaternions(
    lhs: Sequence[float],
    rhs: Sequence[float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def rotate_vector_by_quaternion(vector: Vector3, quaternion: Sequence[float]) -> Vector3:
    x, y, z, w = quaternion
    vx, vy, vz = vector

    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)

    rx = vx + w * tx + (y * tz - z * ty)
    ry = vy + w * ty + (z * tx - x * tz)
    rz = vz + w * tz + (x * ty - y * tx)
    return (rx, ry, rz)
