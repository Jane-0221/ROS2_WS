#!/usr/bin/python3

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


TRANSIT_STOW_V0_STATE_POSITIONS = {
    # Transit tuck tuned from the requested MoveIt pose so the arm stays compact
    # while the base is moving in Gazebo/Nav2.
    "raise_joint": 0.302,
    "h1_joint": 0.0,
    "h2_joint": -0.5236,
    "r1_joint": -0.6806784082777885,
    "r2_joint": -1.3089969389957472,
    "r3_joint": -1.3089969389957472,
    "r4_joint": -1.5707963267948966,
    "r5_joint": -0.017453292519943295,
    "r6_joint": 0.08726646259971647,
}

DEFAULT_STOW_STATE_POSITIONS = dict(TRANSIT_STOW_V0_STATE_POSITIONS)

DEFAULT_PREPARE_ARM_POSITIONS = {
    "r1_joint": 0.0,
    "r2_joint": 0.0,
    "r3_joint": 0.0,
    "r4_joint": 0.0,
    "r5_joint": 0.0,
    "r6_joint": 0.0,
}

PREFERRED_PREPARE_ARM_POSITIONS = {
    "r1_joint": 0.0,
    "r2_joint": 0.0,
    "r3_joint": 0.0,
    "r4_joint": 0.0,
    "r5_joint": 0.0,
    "r6_joint": 0.0,
}

DEFAULT_PREPARE_BRANCH_ARM_SEEDS = (
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    (0.15, 0.0, 0.0, 0.0, 0.0, 0.0),
    (-0.15, 0.0, 0.0, 0.0, 0.0, 0.0),
    (0.0, 0.20, 0.0, 0.0, 0.0, 0.0),
    (0.0, -0.20, 0.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 0.20, 0.0, 0.0, 0.0),
    (0.0, 0.0, -0.20, 0.0, 0.0, 0.0),
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
