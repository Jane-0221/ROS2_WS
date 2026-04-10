from __future__ import annotations

import math
from typing import Dict, List

from .constraints import constraint_violations
from .geometry import cabinet_entry_x, position_error, quaternion_angular_error
from .types import CabinetSpec, PoseSpec, RewardWeights, TaskConstraints


def _potential(
    start_q: Dict[str, float],
    current_q: Dict[str, float],
    current_tool_pose: PoseSpec,
    goal_pose: PoseSpec,
    cabinet: CabinetSpec,
    weights: RewardWeights,
) -> float:
    pos_err = position_error(current_tool_pose, goal_pose)
    ori_err = quaternion_angular_error(current_tool_pose.quaternion_xyzw, goal_pose.quaternion_xyzw)
    entry_violation = max(0.0, current_tool_pose.position[0] - cabinet_entry_x(cabinet))
    limits = TaskConstraints()
    violations = constraint_violations(start_q, current_q, limits)
    return (
        weights.progress_position * pos_err
        + weights.progress_orientation * ori_err
        + weights.outside_violation * entry_violation
        + weights.base_planar * violations["base_planar_motion_m"]
        + weights.base_yaw * math.radians(violations["base_yaw_motion_deg"])
        + weights.r1_motion * math.radians(violations["r1_motion_deg"])
    )


def dense_reward(
    start_q: Dict[str, float],
    prev_q: Dict[str, float],
    current_q: Dict[str, float],
    prev_tool_pose: PoseSpec,
    current_tool_pose: PoseSpec,
    goal_pose: PoseSpec,
    cabinet: CabinetSpec,
    action: List[float],
    prev_action: List[float],
    weights: RewardWeights,
) -> float:
    prev_phi = _potential(start_q, prev_q, prev_tool_pose, goal_pose, cabinet, weights)
    cur_phi = _potential(start_q, current_q, current_tool_pose, goal_pose, cabinet, weights)
    action_l2 = sum(value * value for value in action)
    action_delta_l2 = sum((a - b) * (a - b) for a, b in zip(action, prev_action))
    return (
        (prev_phi - cur_phi)
        - weights.action_l2 * action_l2
        - weights.action_delta_l2 * action_delta_l2
        - weights.time_penalty
    )
