from __future__ import annotations

import math
from typing import Dict

from .types import TaskConstraints


def base_planar_motion(start_q: Dict[str, float], current_q: Dict[str, float]) -> float:
    dx = current_q.get("base_x", start_q.get("base_x", 0.0)) - start_q.get("base_x", 0.0)
    dy = current_q.get("base_y", start_q.get("base_y", 0.0)) - start_q.get("base_y", 0.0)
    return math.hypot(dx, dy)


def angle_wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def base_yaw_motion(start_q: Dict[str, float], current_q: Dict[str, float]) -> float:
    theta0 = start_q.get("base_theta", 0.0)
    theta1 = current_q.get("base_theta", theta0)
    return abs(angle_wrap(theta1 - theta0))


def r1_motion(start_q: Dict[str, float], current_q: Dict[str, float]) -> float:
    r0 = start_q.get("r1_joint", 0.0)
    r1 = current_q.get("r1_joint", r0)
    return abs(angle_wrap(r1 - r0))


def constraint_violations(
    start_q: Dict[str, float],
    current_q: Dict[str, float],
    limits: TaskConstraints,
) -> dict:
    base_xy = base_planar_motion(start_q, current_q)
    base_yaw = math.degrees(base_yaw_motion(start_q, current_q))
    r1_deg = math.degrees(r1_motion(start_q, current_q))
    return {
        "base_planar_motion_m": base_xy,
        "base_yaw_motion_deg": base_yaw,
        "r1_motion_deg": r1_deg,
        "base_planar_exceeded": base_xy > limits.max_base_planar_motion_m,
        "base_yaw_exceeded": base_yaw > limits.max_base_yaw_motion_deg,
        "r1_exceeded": r1_deg > limits.max_r1_motion_deg,
    }
