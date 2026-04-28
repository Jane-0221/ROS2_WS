#!/usr/bin/env python3

from __future__ import annotations

import math
import re
import shutil
from dataclasses import dataclass
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parent.parent
MODELS_DIR = WORKSPACE / "src" / "medipick_simple3_description" / "models"
WORLD_PATH = WORKSPACE / "src" / "medipick_simple3_description" / "worlds" / "medipick_pharmacy_textured.world.sdf"
ASSET_MIRROR_ROOT = WORKSPACE / "assets" / "environment_models" / "pharmacy_gazebo_v5_textured_sources"

PRODUCT_MODELS = [f"textured_drug_{index:03d}" for index in range(1, 13)]

# 这次不再把所有药盒视为同一尺寸，而是读取每个 OBJ 的真实包围盒。
# 但为了保证 Gazebo 中的视觉厚度更自然，仍然允许对深度做统一放大。
BOX_DEPTH_SCALE = 1.35
BOX_VISUAL_Z_SCALE = 1.0
BOX_ROW_GAP = 0.010
BOX_EDGE_MARGIN = 0.045
BOX_VERTICAL_CLEARANCE = 0.015
BOX_SUPPORT_CLEARANCE = 0.005
BOX_MASS_DENSITY = 220.0
BOX_FRICTION = 1.3
BOX_CONTACT_KP = 120000.0
BOX_CONTACT_KD = 8.0

INTERACTIVE_CLUSTERS = [
    {
        "shelf_name": "aisle_b_right_04",
        "level_index": 3,
        "level_radius": 1,
        "slot_radius": 1,
    }
]

SHELF_SPECS = {
    "pharmacy_shelf": {
        "front_x": 0.070,
        "usable_y": 0.90,
        "levels": [
            (0.03, 0.06),
            (0.42, 0.03),
            (0.82, 0.03),
            (1.22, 0.03),
            (1.62, 0.03),
        ],
    },
    "pharmacy_shelf_dense": {
        "front_x": 0.075,
        "usable_y": 0.98,
        "levels": [
            (0.03, 0.06),
            (0.29, 0.025),
            (0.55, 0.025),
            (0.81, 0.025),
            (1.07, 0.025),
            (1.33, 0.025),
            (1.59, 0.025),
            (1.85, 0.025),
        ],
    },
    "pharmacy_shelf_compact": {
        "front_x": 0.060,
        "usable_y": 0.74,
        "levels": [
            (0.03, 0.06),
            (0.25, 0.025),
            (0.48, 0.025),
            (0.74, 0.025),
            (0.97, 0.025),
            (1.21, 0.025),
            (1.48, 0.025),
        ],
    },
    "pharmacy_endcap_display": {
        "front_x": 0.065,
        "usable_y": 0.68,
        "levels": [
            (0.05, 0.10),
            (0.42, 0.03),
            (0.78, 0.03),
            (1.14, 0.03),
        ],
    },
}


@dataclass(frozen=True)
class ProductSpec:
    name: str
    raw_width: float
    raw_depth: float
    raw_height: float
    width: float
    depth: float
    height: float
    visual_scale_y: float
    visual_scale_z: float


def box_inertia(mass: float, width: float, depth: float, height: float) -> tuple[float, float, float]:
    ixx = (mass / 12.0) * (depth * depth + height * height)
    iyy = (mass / 12.0) * (width * width + height * height)
    izz = (mass / 12.0) * (width * width + depth * depth)
    return ixx, iyy, izz


def compute_obj_bbox(obj_path: Path) -> tuple[float, float, float]:
    xs: list[float] = []
    ys: list[float] = []
    zs: list[float] = []

    for line in obj_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if not line.startswith("v "):
            continue
        _tag, x, y, z, *_rest = line.split()
        xs.append(float(x))
        ys.append(float(y))
        zs.append(float(z))

    if not xs:
        raise ValueError(f"OBJ 顶点为空，无法计算包围盒：{obj_path}")

    return max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)


def load_product_specs() -> dict[str, ProductSpec]:
    specs: dict[str, ProductSpec] = {}

    for model_name in PRODUCT_MODELS:
        obj_path = MODELS_DIR / model_name / "meshes" / "model.obj"
        raw_width, raw_depth, raw_height = compute_obj_bbox(obj_path)
        depth = raw_depth * BOX_DEPTH_SCALE
        height = raw_height * BOX_VISUAL_Z_SCALE
        specs[model_name] = ProductSpec(
            name=model_name,
            raw_width=raw_width,
            raw_depth=raw_depth,
            raw_height=raw_height,
            width=raw_width,
            depth=depth,
            height=height,
            visual_scale_y=BOX_DEPTH_SCALE,
            visual_scale_z=BOX_VISUAL_Z_SCALE,
        )

    return specs


PRODUCT_SPECS = load_product_specs()


def update_textured_drug_models() -> None:
    for model_name, product in PRODUCT_SPECS.items():
        sdf_path = MODELS_DIR / model_name / "model.sdf"
        mass = max(0.015, product.width * product.depth * product.height * BOX_MASS_DENSITY)
        ixx, iyy, izz = box_inertia(mass, product.width, product.depth, product.height)
        sdf_text = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{model_name}">
    <static>false</static>
    <self_collide>false</self_collide>
    <link name="body">
      <inertial>
        <pose>0 0 {product.height / 2.0:.6f} 0 0 0</pose>
        <mass>{mass:.6f}</mass>
        <inertia>
          <ixx>{ixx:.8f}</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>{iyy:.8f}</iyy>
          <iyz>0.0</iyz>
          <izz>{izz:.8f}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <pose>0 0 {product.height / 2.0:.6f} 0 0 0</pose>
        <geometry><box><size>{product.width:.6f} {product.depth:.6f} {product.height:.6f}</size></box></geometry>
        <surface>
          <friction>
            <ode>
              <mu>{BOX_FRICTION:.3f}</mu>
              <mu2>{BOX_FRICTION:.3f}</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>{BOX_CONTACT_KP:.1f}</kp>
              <kd>{BOX_CONTACT_KD:.1f}</kd>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name="visual">
        <pose>0 0 {product.height / 2.0:.6f} 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>meshes/model.obj</uri>
            <scale>1 {product.visual_scale_y:.6f} {product.visual_scale_z:.6f}</scale>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""
        sdf_path.write_text(sdf_text, encoding="utf-8")


def parse_shelf_includes(world_text: str):
    pattern = re.compile(
        r"<include><name>([^<]+)</name><uri>model://([^<]+)</uri><pose>([^<]+)</pose></include>"
    )
    includes = []
    for name, model_name, pose_text in pattern.findall(world_text):
        if model_name not in SHELF_SPECS:
            continue
        x, y, z, roll, pitch, yaw = map(float, pose_text.split())
        includes.append(
            {
                "name": name,
                "model": model_name,
                "pose": (x, y, z, roll, pitch, yaw),
            }
        )
    return includes


def level_clearance(levels, level_index: int) -> float:
    if level_index >= len(levels) - 1:
        return float("inf")
    current_center, current_thickness = levels[level_index]
    next_center, next_thickness = levels[level_index + 1]
    return (next_center - (next_thickness / 2.0)) - (current_center + (current_thickness / 2.0))


def local_to_world(shelf_pose, local_x: float, local_y: float):
    x, y, _z, _roll, _pitch, yaw = shelf_pose
    world_x = x + math.cos(yaw) * local_x - math.sin(yaw) * local_y
    world_y = y + math.sin(yaw) * local_x + math.cos(yaw) * local_y
    return world_x, world_y


def interactive_slots_for_shelf(shelf_name: str, level_count: int, slot_count: int) -> set[tuple[int, int]]:
    slots: set[tuple[int, int]] = set()
    center_slot = max(1, (slot_count + 1) // 2)

    for cluster in INTERACTIVE_CLUSTERS:
        if cluster["shelf_name"] != shelf_name:
            continue

        level_min = max(1, cluster["level_index"] - cluster["level_radius"])
        level_max = min(level_count, cluster["level_index"] + cluster["level_radius"])
        slot_min = max(1, center_slot - cluster["slot_radius"])
        slot_max = min(slot_count, center_slot + cluster["slot_radius"])

        for level_index in range(level_min, level_max + 1):
            for slot_index in range(slot_min, slot_max + 1):
                slots.add((level_index, slot_index))

    return slots


def rotate_candidates(seed: int, candidates: list[ProductSpec]) -> list[ProductSpec]:
    if not candidates:
        return []
    offset = seed % len(candidates)
    return candidates[offset:] + candidates[:offset]


def build_row_layout(usable_y: float, level_gap: float, seed: int) -> tuple[list[tuple[int, ProductSpec, float]], int]:
    available_y = usable_y - (2.0 * BOX_EDGE_MARGIN)
    if available_y <= 0.0:
        return [], seed

    fit_candidates = [
        product
        for product in PRODUCT_SPECS.values()
        if product.height + BOX_VERTICAL_CLEARANCE <= level_gap
    ]
    fit_candidates = rotate_candidates(seed, fit_candidates)
    if not fit_candidates:
        return [], seed

    chosen: list[ProductSpec] = []
    used_width = 0.0
    candidate_index = 0

    while True:
        product = fit_candidates[candidate_index % len(fit_candidates)]
        extra_gap = BOX_ROW_GAP if chosen else 0.0
        next_width = used_width + extra_gap + product.width
        if next_width > available_y + 1e-9:
            break
        chosen.append(product)
        used_width = next_width
        candidate_index += 1

    if not chosen:
        return [], seed

    layout: list[tuple[int, ProductSpec, float]] = []
    cursor = -used_width / 2.0
    for slot_index, product in enumerate(chosen, start=1):
        center_y = cursor + (product.width / 2.0)
        layout.append((slot_index, product, center_y))
        cursor = center_y + (product.width / 2.0) + BOX_ROW_GAP

    return layout, seed + len(chosen)


def build_product_block(shelf_includes) -> str:
    lines = [
        "    <!-- Auto-generated full-shelf products -->",
        "    <model name=\"pharmacy_stock_visuals\">",
        "      <static>true</static>",
        "      <link name=\"stock_link\">",
    ]
    entity_lines = ["    <!-- Interactive product entities -->"]
    row_seed = 0

    for shelf in shelf_includes:
        spec = SHELF_SPECS[shelf["model"]]
        shelf_pose = shelf["pose"]
        _x, _y, _z, _roll, _pitch, shelf_yaw = shelf_pose
        outward_yaw = shelf_yaw - (math.pi / 2.0)
        lines.append(f"        <!-- Products for {shelf['name']} ({shelf['model']}) -->")

        for zero_based_level_index, (board_center_z, board_thickness) in enumerate(spec["levels"]):
            level_index = zero_based_level_index + 1
            level_gap = level_clearance(spec["levels"], zero_based_level_index)
            row_layout, row_seed = build_row_layout(spec["usable_y"], level_gap, row_seed)
            if not row_layout:
                continue

            interactive_slots = interactive_slots_for_shelf(
                shelf["name"],
                len(spec["levels"]),
                len(row_layout),
            )

            board_top = board_center_z + (board_thickness / 2.0)
            for slot_index, product, local_y in row_layout:
                center_x = spec["front_x"] + (product.depth / 2.0)
                product_z = board_top + (product.height / 2.0) + BOX_SUPPORT_CLEARANCE
                world_x, world_y = local_to_world(shelf_pose, center_x, local_y)
                name = f"{shelf['name']}_l{level_index:02d}_s{slot_index:02d}"
                pose = f"{world_x:.3f} {world_y:.3f} {product_z:.3f} 0 0 {outward_yaw:.6f}"

                if (level_index, slot_index) in interactive_slots:
                    entity_lines.append(
                        f"    <include><name>{name}</name><uri>model://{product.name}</uri><pose>{pose}</pose></include>"
                    )
                else:
                    lines.append(
                        f"        <visual name=\"{name}\"><pose>{pose}</pose><cast_shadows>false</cast_shadows>"
                        f"<geometry><mesh><uri>model://{product.name}/meshes/model.obj</uri>"
                        f"<scale>1 {product.visual_scale_y:.6f} {product.visual_scale_z:.6f}</scale>"
                        f"</mesh></geometry></visual>"
                    )

    lines.extend(
        [
            "      </link>",
            "    </model>",
        ]
    )
    return "\n".join(lines + entity_lines) + "\n"


def regenerate_world() -> None:
    text = WORLD_PATH.read_text(encoding="utf-8")
    start_marker = None
    for candidate in [
        "    <!-- Back wall products -->",
        "    <!-- Auto-generated full-shelf products as individual entities -->",
        "    <!-- Auto-generated full-shelf products -->",
    ]:
        if candidate in text:
            start_marker = candidate
            break
    if start_marker is None:
        raise ValueError("Unable to find generated product block marker in pharmacy world.")
    end_marker = "  </world>"
    start_index = text.index(start_marker)
    end_index = text.index(end_marker)
    shelf_includes = parse_shelf_includes(text[:start_index])
    product_block = build_product_block(shelf_includes)
    updated = text[:start_index] + product_block + text[end_index:]
    WORLD_PATH.write_text(updated, encoding="utf-8")


def sync_assets_snapshot() -> None:
    mirror_models_root = ASSET_MIRROR_ROOT / "models"
    mirror_world_root = ASSET_MIRROR_ROOT / "worlds"
    mirror_world_root.mkdir(parents=True, exist_ok=True)

    for model_dir in sorted(MODELS_DIR.glob("textured_drug_*")):
        target_dir = mirror_models_root / model_dir.name
        target_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(model_dir / "model.sdf", target_dir / "model.sdf")
        shutil.copy2(model_dir / "model.config", target_dir / "model.config")
        (target_dir / "meshes").mkdir(parents=True, exist_ok=True)
        shutil.copy2(model_dir / "meshes" / "model.obj", target_dir / "meshes" / "model.obj")
        shutil.copy2(model_dir / "meshes" / "material.mtl", target_dir / "meshes" / "material.mtl")
        shutil.copy2(model_dir / "meshes" / "texture.png", target_dir / "meshes" / "texture.png")

    shutil.copy2(WORLD_PATH, mirror_world_root / WORLD_PATH.name)


def main() -> None:
    update_textured_drug_models()
    regenerate_world()
    sync_assets_snapshot()
    print("已按每种药盒真实尺寸重新生成贴图药店环境。")


if __name__ == "__main__":
    main()
