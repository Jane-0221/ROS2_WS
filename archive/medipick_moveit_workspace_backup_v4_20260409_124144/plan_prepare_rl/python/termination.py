from __future__ import annotations

import math
from typing import Dict

from .constraints import constraint_violations
from .geometry import cabinet_entry_x, position_error, quaternion_angular_error
from .types import CabinetSpec, PoseSpec, TaskConstraints


def is_success(
    start_q: Dict[str, float],
    current_q: Dict[str, float],
    current_tool_pose: PoseSpec,
    goal_pose: PoseSpec,
    cabinet: CabinetSpec,
    limits: TaskConstraints,
    tool_speed: float,
    position_tolerance_m: float = 0.02,
    orientation_tolerance_rad: float = 0.10,
    terminal_speed_threshold: float = 0.05,
) -> bool:
    violations = constraint_violations(start_q, current_q, limits)
    if (
        violations["base_planar_exceeded"]
        or violations["base_yaw_exceeded"]
        or violations["r1_exceeded"]
    ):
        return False
    if position_error(current_tool_pose, goal_pose) > position_tolerance_m:
        return False
    if quaternion_angular_error(current_tool_pose.quaternion_xyzw, goal_pose.quaternion_xyzw) > orientation_tolerance_rad:
        return False
    if current_tool_pose.position[0] > cabinet_entry_x(cabinet) - limits.outside_margin_m:
        return False
    if tool_speed > terminal_speed_threshold:
        return False
    return True


def terminal_reason(
    start_q: Dict[str, float],
    current_q: Dict[str, float],
    collision: bool,
    step_count: int,
    max_steps: int,
    limits: TaskConstraints,
) -> str | None:
    if collision:
        return "collision"
    violations = constraint_violations(start_q, current_q, limits)
    if violations["base_planar_exceeded"]:
        return "base_planar_limit"
    if violations["base_yaw_exceeded"]:
        return "base_yaw_limit"
    if violations["r1_exceeded"]:
        return "r1_limit"
    if step_count >= max_steps:
        return "timeout"
    return None
