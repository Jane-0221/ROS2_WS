from __future__ import annotations

import math
from typing import Iterable, Tuple

from .types import CabinetSpec, PoseSpec


Vector3 = Tuple[float, float, float]


def quat_mul(lhs: Iterable[float], rhs: Iterable[float]) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def rotate_vector_by_quaternion(vector: Vector3, quaternion_xyzw: Iterable[float]) -> Vector3:
    x, y, z, w = quaternion_xyzw
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    rx = vx + w * tx + (y * tz - z * ty)
    ry = vy + w * ty + (z * tx - x * tz)
    rz = vz + w * tz + (x * ty - y * tx)
    return (rx, ry, rz)


def cabinet_entry_x(cabinet: CabinetSpec) -> float:
    return cabinet.shelf_center_x - cabinet.shelf_depth / 2.0


def build_pre_insert_pose(
    target_pose: PoseSpec,
    retreat_distance: float,
    lateral_offset: float = 0.0,
    vertical_offset: float = 0.0,
    yaw_offset_deg: float = 0.0,
) -> PoseSpec:
    q = tuple(target_pose.quaternion_xyzw)
    approach_axis = rotate_vector_by_quaternion((1.0, 0.0, 0.0), q)
    lateral_axis = rotate_vector_by_quaternion((0.0, 1.0, 0.0), q)
    vertical_axis = rotate_vector_by_quaternion((0.0, 0.0, 1.0), q)

    px, py, pz = target_pose.position
    px -= retreat_distance * approach_axis[0]
    py -= retreat_distance * approach_axis[1]
    pz -= retreat_distance * approach_axis[2]

    px += lateral_offset * lateral_axis[0] + vertical_offset * vertical_axis[0]
    py += lateral_offset * lateral_axis[1] + vertical_offset * vertical_axis[1]
    pz += lateral_offset * lateral_axis[2] + vertical_offset * vertical_axis[2]

    q_out = q
    if abs(yaw_offset_deg) > 1e-9:
        yaw = math.radians(yaw_offset_deg)
        yaw_q = (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))
        q_out = quat_mul(yaw_q, q)

    return PoseSpec(position=[px, py, pz], quaternion_xyzw=list(q_out))


def is_outside_cabinet(pre_insert_pose: PoseSpec, cabinet: CabinetSpec, margin_m: float) -> bool:
    return pre_insert_pose.position[0] <= cabinet_entry_x(cabinet) - margin_m


def position_error(a: PoseSpec, b: PoseSpec) -> float:
    ax, ay, az = a.position
    bx, by, bz = b.position
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2)


def quaternion_angular_error(a_xyzw: Iterable[float], b_xyzw: Iterable[float]) -> float:
    ax, ay, az, aw = a_xyzw
    bx, by, bz, bw = b_xyzw
    dot = ax * bx + ay * by + az * bz + aw * bw
    dot = max(-1.0, min(1.0, abs(dot)))
    return 2.0 * math.acos(dot)
