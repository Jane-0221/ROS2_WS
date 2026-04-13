#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


REQUIRED_ARM_JOINTS = (
    "r1_joint",
    "r2_joint",
    "r3_joint",
    "r4_joint",
    "r5_joint",
    "r6_joint",
)


@dataclass(frozen=True)
class ArmJointCalibration:
    joint_name: str
    command_index: int
    feedback_index: int
    direction: float
    command_scale: float
    feedback_scale: float
    joint_offset: float
    min_position: float
    max_position: float

    def clamp(self, joint_position: float) -> float:
        return max(self.min_position, min(self.max_position, joint_position))

    def joint_to_command(self, joint_position: float) -> float:
        clamped = self.clamp(joint_position)
        return self.direction * (clamped - self.joint_offset) / self.command_scale

    def feedback_to_joint(self, raw_feedback: float) -> float:
        return self.direction * raw_feedback * self.feedback_scale + self.joint_offset


@dataclass(frozen=True)
class LiftCalibration:
    direction: float
    mm_per_joint: float
    joint_offset: float
    min_mm: float
    max_mm: float

    def clamp_mm(self, height_mm: float) -> float:
        return max(self.min_mm, min(self.max_mm, height_mm))

    def mm_to_joint(self, height_mm: float) -> float:
        clamped = self.clamp_mm(height_mm)
        return self.direction * (clamped / self.mm_per_joint) + self.joint_offset

    def joint_to_mm(self, joint_position: float) -> float:
        raw_mm = (joint_position - self.joint_offset) * self.mm_per_joint
        return self.direction * raw_mm


@dataclass(frozen=True)
class HardwareCalibration:
    arm_joint_mappings: dict[str, ArmJointCalibration]
    fixed_joints: dict[str, float]
    base_joints: dict[str, float]
    lift: LiftCalibration


def _as_dict(value: Any, field_name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"Calibration field '{field_name}' must be a mapping.")
    return value


def _float_field(mapping: dict[str, Any], key: str) -> float:
    if key not in mapping:
        raise ValueError(f"Calibration field '{key}' is required.")
    return float(mapping[key])


def _int_field(mapping: dict[str, Any], key: str) -> int:
    if key not in mapping:
        raise ValueError(f"Calibration field '{key}' is required.")
    return int(mapping[key])


def load_hardware_calibration(calibration_file: str | Path) -> HardwareCalibration:
    path = Path(calibration_file).expanduser()
    if not path.is_file():
        raise ValueError(f"Calibration file does not exist: {path}")

    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    if not isinstance(data, dict):
        raise ValueError("Calibration root must be a mapping.")

    arm_mapping_key = "motor_arm_joint_mappings" if "motor_arm_joint_mappings" in data else "arm_joint_mappings"
    arm_data = _as_dict(data.get(arm_mapping_key), arm_mapping_key)
    arm_joint_mappings: dict[str, ArmJointCalibration] = {}
    for joint_name in REQUIRED_ARM_JOINTS:
        joint_data = _as_dict(arm_data.get(joint_name), f"{arm_mapping_key}.{joint_name}")
        arm_joint_mappings[joint_name] = ArmJointCalibration(
            joint_name=joint_name,
            command_index=_int_field(joint_data, "command_index"),
            feedback_index=_int_field(joint_data, "feedback_index"),
            direction=_float_field(joint_data, "direction"),
            command_scale=_float_field(joint_data, "command_scale"),
            feedback_scale=_float_field(joint_data, "feedback_scale"),
            joint_offset=float(joint_data.get("joint_offset", 0.0)),
            min_position=float(joint_data.get("min_position", -6.283185307179586)),
            max_position=float(joint_data.get("max_position", 6.283185307179586)),
        )

    fixed_joints = {
        str(joint_name): float(position)
        for joint_name, position in _as_dict(data.get("fixed_joints", {}), "fixed_joints").items()
    }
    base_joints = {
        str(joint_name): float(position)
        for joint_name, position in _as_dict(data.get("base_joints", {}), "base_joints").items()
    }
    for joint_name in ("base_x", "base_y", "base_theta"):
        base_joints.setdefault(joint_name, 0.0)

    lift_data = _as_dict(data.get("lift"), "lift")
    lift = LiftCalibration(
        direction=_float_field(lift_data, "direction"),
        mm_per_joint=_float_field(lift_data, "mm_per_joint"),
        joint_offset=float(lift_data.get("joint_offset", 0.0)),
        min_mm=float(lift_data.get("min_mm", 0.0)),
        max_mm=float(lift_data.get("max_mm", 1500.0)),
    )

    return HardwareCalibration(
        arm_joint_mappings=arm_joint_mappings,
        fixed_joints=fixed_joints,
        base_joints=base_joints,
        lift=lift,
    )
