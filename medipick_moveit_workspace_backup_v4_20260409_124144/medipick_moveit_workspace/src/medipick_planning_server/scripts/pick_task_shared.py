#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


TRANSIT_STOW_V0_STATE_POSITIONS = {
    "raise_joint": 0.302,
    "r1_joint": -0.681,
    "r2_joint": -1.309,
    "r3_joint": -1.309,
    "r4_joint": -1.571,
    "r5_joint": 0.017,
    "r6_joint": 0.087,
}

DEFAULT_STOW_STATE_POSITIONS = dict(TRANSIT_STOW_V0_STATE_POSITIONS)

DEFAULT_PREPARE_ARM_POSITIONS = {
    "r1_joint": -0.10,
    "r2_joint": 1.82,
    "r3_joint": -4.555309,
    "r4_joint": 1.00,
    "r5_joint": 0.30,
    "r6_joint": 0.20,
}

PREFERRED_PREPARE_ARM_POSITIONS = {
    "r1_joint": -0.10,
    "r2_joint": 1.82,
    "r3_joint": -4.555309,
    "r4_joint": 1.00,
    "r5_joint": 0.30,
    "r6_joint": 0.20,
}

DEFAULT_PREPARE_BRANCH_ARM_SEEDS = (
    (-0.10, 1.82, -4.555309, 1.00, 0.30, 0.20),
    (0.18, 1.75, -4.355309, 1.08, 0.38, 0.20),
    (-0.32, 1.75, -4.755309, 1.08, 0.38, 0.20),
    (0.05, 1.95, -4.555309, 0.92, 0.22, 0.15),
    (-0.20, 1.95, -4.555309, 0.92, 0.22, 0.15),
    (0.40, 1.65, -4.355309, 1.18, 0.48, 0.20),
    (-0.50, 1.65, -4.755309, 1.18, 0.48, 0.20),
)

DEFAULT_PREPARE_CANDIDATES = (
    (0.0, 0.0, 0.0, 0.0),
    (0.0, -0.04, 0.0, 0.0),
    (0.0, 0.04, 0.0, 0.0),
    (-0.08, 0.0, 0.0, -0.03),
    (-0.05, 0.0, 0.0, -0.03),
    (-0.10, -0.03, -4.0, -0.04),
    (-0.10, 0.03, 4.0, -0.04),
    (0.05, 0.0, 0.0, 0.03),
    (0.08, 0.0, 0.0, 0.03),
    (0.0, 0.0, 0.0, 0.08),
    (0.0, 0.0, 0.0, 0.12),
    (-0.05, 0.0, 0.0, 0.10),
    (0.05, 0.0, 0.0, 0.10),
    (0.0, 0.0, -8.0, -0.02),
    (0.0, 0.0, 8.0, 0.02),
    (0.0, 0.0, -6.0, 0.08),
    (0.0, 0.0, 6.0, 0.08),
    (-0.06, -0.04, -6.0, -0.03),
    (-0.06, 0.04, 6.0, -0.03),
)

DEFAULT_PREPARE_ARM_TEMPLATE_OFFSETS = (
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    (0.10, -0.12, 0.0, 0.08, 0.0, 0.0),
    (-0.10, 0.12, 0.0, -0.08, 0.0, 0.0),
    (0.0, -0.08, 0.20, 0.06, 0.0, 0.0),
    (0.0, 0.08, -0.20, -0.06, 0.0, 0.0),
    (0.0, -0.18, 0.0, 0.18, 0.0, 0.0),
    (0.0, 0.18, 0.0, -0.18, 0.0, 0.0),
    (0.08, -0.10, 0.12, 0.12, -0.08, 0.0),
    (-0.08, 0.10, -0.12, -0.12, 0.08, 0.0),
    (0.0, -0.10, 0.0, 0.10, -0.18, 0.12),
    (0.0, -0.10, 0.0, 0.10, 0.18, -0.12),
    (0.0, 0.10, 0.0, -0.10, -0.18, 0.12),
    (0.0, 0.10, 0.0, -0.10, 0.18, -0.12),
    (0.06, -0.06, 0.16, 0.10, -0.25, 0.18),
    (-0.06, 0.06, -0.16, -0.10, 0.25, -0.18),
    (0.04, -0.04, 0.08, 0.08, -0.30, 0.24),
    (-0.04, 0.04, -0.08, -0.08, 0.30, -0.24),
    (0.0, 0.0, 0.0, 0.0, -0.60, 0.18),
    (0.0, 0.0, 0.0, 0.0, -0.90, 0.24),
    (0.0, 0.0, 0.0, 0.0, -1.20, 0.32),
    (0.0, 0.0, 0.0, 0.0, 0.60, -0.18),
    (0.0, 0.0, 0.0, 0.0, 0.90, -0.24),
    (0.0, 0.0, 0.0, 0.0, 1.20, -0.32),
    (0.04, -0.10, 0.18, 0.14, -0.90, 0.28),
    (-0.04, 0.10, -0.18, -0.14, 0.90, -0.28),
    (0.08, -0.14, 0.22, 0.18, -1.10, 0.34),
    (-0.08, 0.14, -0.22, -0.18, 1.10, -0.34),
)


class PickStage(str, Enum):
    IDLE = "IDLE"
    ACQUIRE_TARGET = "ACQUIRE_TARGET"
    ARM_STOW_SAFE = "ARM_STOW_SAFE"
    BASE_ENTER_WORKSPACE = "BASE_ENTER_WORKSPACE"
    LIFT_TO_BAND = "LIFT_TO_BAND"
    SELECT_PRE_INSERT = "SELECT_PRE_INSERT"
    PLAN_TO_PRE_INSERT = "PLAN_TO_PRE_INSERT"
    INSERT_AND_SUCTION = "INSERT_AND_SUCTION"
    SAFE_RETREAT = "SAFE_RETREAT"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


@dataclass
class PlannedStage:
    stage: PickStage
    success: bool
    message: str
    planning_time: float
    trajectory: JointTrajectory
    final_joint_state: JointState


@dataclass
class PrepareCandidate:
    score: float
    prepare_pose: PoseStamped
    base_goal_pose: PoseStamped
    lift_height: float
    prepare_result: PlannedStage
    final_result: PlannedStage
    preferred_group_name: str = ""


@dataclass
class PrepareCandidatePreEval:
    cheap_score: float
    prepare_pose: PoseStamped
    base_goal_pose: PoseStamped
    lift_height: float
    prepare_result: PlannedStage


@dataclass
class CandidateDebugSample:
    base_goal_pose: PoseStamped
    lift_height: float
    score: float
    accepted: bool
    label: str
