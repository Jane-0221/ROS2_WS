from __future__ import annotations

from typing import Dict, List

from .constraints import base_planar_motion, base_yaw_motion, r1_motion
from .geometry import cabinet_entry_x, position_error, quaternion_angular_error
from .types import CabinetSpec, PoseSpec


def build_observation_dict(
    start_q: Dict[str, float],
    current_q: Dict[str, float],
    current_qd: Dict[str, float],
    current_tool_pose: PoseSpec,
    goal_pose: PoseSpec,
    cabinet: CabinetSpec,
    prev_action: List[float],
) -> dict:
    entry_x = cabinet_entry_x(cabinet)
    tool_x, tool_y, tool_z = current_tool_pose.position
    goal_x, goal_y, goal_z = goal_pose.position

    return {
        "joint_positions": dict(current_q),
        "joint_velocities": dict(current_qd),
        "tool_position_error": [
            goal_x - tool_x,
            goal_y - tool_y,
            goal_z - tool_z,
        ],
        "tool_position_error_norm": position_error(current_tool_pose, goal_pose),
        "tool_orientation_error": quaternion_angular_error(
            current_tool_pose.quaternion_xyzw,
            goal_pose.quaternion_xyzw,
        ),
        "cabinet_features": {
            "entry_plane_signed_distance": entry_x - tool_x,
            "left_wall_margin": (cabinet.shelf_center_y + cabinet.shelf_width / 2.0) - tool_y,
            "right_wall_margin": tool_y - (cabinet.shelf_center_y - cabinet.shelf_width / 2.0),
            "bottom_margin": tool_z - cabinet.shelf_bottom_z,
        },
        "stage_residuals": {
            "base_planar_motion_m": base_planar_motion(start_q, current_q),
            "base_yaw_motion_rad": base_yaw_motion(start_q, current_q),
            "r1_motion_rad": r1_motion(start_q, current_q),
        },
        "previous_action": list(prev_action),
    }
