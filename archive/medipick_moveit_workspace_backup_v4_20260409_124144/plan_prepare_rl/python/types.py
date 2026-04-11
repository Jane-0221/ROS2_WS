from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List


@dataclass
class PoseSpec:
    position: List[float]
    quaternion_xyzw: List[float]


@dataclass
class CabinetSpec:
    shelf_center_x: float
    shelf_center_y: float
    shelf_depth: float
    shelf_width: float
    shelf_bottom_z: float
    shelf_level_gap: float
    shelf_board_thickness: float
    shelf_side_thickness: float
    shelf_back_thickness: float


@dataclass
class StageStartState:
    joint_positions: Dict[str, float]
    joint_velocities: Dict[str, float] = field(default_factory=dict)


@dataclass
class TaskConstraints:
    outside_margin_m: float = 0.01
    max_base_planar_motion_m: float = 0.20
    max_base_yaw_motion_deg: float = 20.0
    max_r1_motion_deg: float = 100.0


@dataclass
class RewardWeights:
    progress_position: float = 8.0
    progress_orientation: float = 2.0
    outside_violation: float = 12.0
    base_planar: float = 4.0
    base_yaw: float = 2.0
    r1_motion: float = 3.0
    action_l2: float = 0.01
    action_delta_l2: float = 0.02
    time_penalty: float = 0.02
    success_bonus: float = 80.0
    collision_penalty: float = -80.0
    hard_constraint_penalty: float = -60.0
