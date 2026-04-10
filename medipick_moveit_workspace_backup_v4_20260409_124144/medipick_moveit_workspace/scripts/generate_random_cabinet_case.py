#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import random
from dataclasses import asdict, dataclass


@dataclass
class BoxSpec:
    center: tuple[float, float, float]
    size: tuple[float, float, float]


def _sample(rng: random.Random, lower: float, upper: float) -> float:
    return round(rng.uniform(lower, upper), 3)


def _level_z(bottom_z: float, level_gap: float, level_index: int) -> float:
    return bottom_z + level_index * level_gap


def _back_panel_front_x(shelf_center_x: float, shelf_depth: float, shelf_back_thickness: float) -> float:
    return shelf_center_x + shelf_depth / 2.0 - shelf_back_thickness


def _build_target_box(params: dict) -> BoxSpec:
    box_size = (
        params["target_box_size_x"],
        params["target_box_size_y"],
        params["target_box_size_z"],
    )
    shelf_top_z = _level_z(params["shelf_bottom_z"], params["shelf_level_gap"], params["target_level"])
    shelf_top_z += params["shelf_board_thickness"] / 2.0
    back_panel_front_x = _back_panel_front_x(
        params["shelf_center_x"],
        params["shelf_depth"],
        params["shelf_back_thickness"],
    )
    center = (
        back_panel_front_x - box_size[0] / 2.0 - 0.01,
        params["shelf_center_y"] + min(0.08, 0.18 * params["shelf_width"]),
        shelf_top_z + box_size[2] / 2.0,
    )
    return BoxSpec(center=center, size=box_size)


def _build_clutter_boxes(params: dict, rng: random.Random, target_box: BoxSpec) -> list[BoxSpec]:
    clutter_boxes: list[BoxSpec] = []
    y_limit = params["shelf_width"] / 2.0 - params["shelf_side_thickness"] - 0.08
    back_panel_front_x = _back_panel_front_x(
        params["shelf_center_x"],
        params["shelf_depth"],
        params["shelf_back_thickness"],
    )

    for index in range(params["clutter_count"]):
        level_index = index % params["shelf_levels"]
        box_size = (
            rng.uniform(0.05, 0.10),
            rng.uniform(0.06, 0.12),
            rng.uniform(0.05, 0.09),
        )
        y = rng.uniform(-max(0.05, y_limit), max(0.05, y_limit))
        if level_index == params["target_level"] and abs(y - target_box.center[1]) < 0.16:
            y = -y if abs(-y) <= y_limit else (0.0 if y > 0 else 0.14)

        shelf_top_z = _level_z(params["shelf_bottom_z"], params["shelf_level_gap"], level_index)
        shelf_top_z += params["shelf_board_thickness"] / 2.0
        center = (
            back_panel_front_x - box_size[0] / 2.0 - rng.uniform(0.015, 0.04),
            params["shelf_center_y"] + y,
            shelf_top_z + box_size[2] / 2.0,
        )
        clutter_boxes.append(BoxSpec(center=center, size=box_size))
    return clutter_boxes


def _build_target_pose(params: dict, rng: random.Random) -> dict:
    inner_front_x = params["shelf_center_x"] - params["shelf_depth"] / 2.0
    inner_back_x = _back_panel_front_x(
        params["shelf_center_x"],
        params["shelf_depth"],
        params["shelf_back_thickness"],
    ) - 0.03
    lower_board_top_z = _level_z(
        params["shelf_bottom_z"],
        params["shelf_level_gap"],
        params["target_gap_index"],
    ) + params["shelf_board_thickness"] / 2.0
    upper_board_bottom_z = _level_z(
        params["shelf_bottom_z"],
        params["shelf_level_gap"],
        params["target_gap_index"] + 1,
    ) - params["shelf_board_thickness"] / 2.0
    lateral_limit = params["shelf_width"] / 2.0 - params["shelf_side_thickness"] - params["target_lateral_margin"]
    target_lateral_span = min(lateral_limit, params["target_lateral_span"])

    return {
        "position": {
            "x": round(inner_front_x + params["target_depth_ratio"] * max(0.02, inner_back_x - inner_front_x), 6),
            "y": round(params["shelf_center_y"] + rng.uniform(-target_lateral_span, target_lateral_span), 6),
            "z": round(0.5 * (lower_board_top_z + upper_board_bottom_z), 6),
        },
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }


def generate_case(seed: int, clutter_count: int) -> dict:
    rng = random.Random(seed)
    params = {
        "random_seed": seed,
        "shelf_levels": rng.randint(4, 7),
        "shelf_width": _sample(rng, 0.58, 0.92),
        "shelf_depth": _sample(rng, 0.18, 0.32),
        "shelf_center_x": _sample(rng, 0.80, 0.98),
        "shelf_center_y": _sample(rng, -0.12, 0.12),
        "shelf_bottom_z": _sample(rng, 0.40, 0.58),
        "shelf_level_gap": _sample(rng, 0.19, 0.30),
        "shelf_board_thickness": _sample(rng, 0.022, 0.045),
        "shelf_side_thickness": _sample(rng, 0.030, 0.060),
        "shelf_back_thickness": _sample(rng, 0.020, 0.050),
        "target_depth_ratio": _sample(rng, 0.04, 0.14),
        "target_lateral_margin": _sample(rng, 0.07, 0.14),
        "target_lateral_span": _sample(rng, 0.02, 0.10),
        "target_box_size_x": _sample(rng, 0.05, 0.11),
        "target_box_size_y": _sample(rng, 0.08, 0.16),
        "target_box_size_z": _sample(rng, 0.04, 0.10),
        "clutter_count": max(0, clutter_count),
    }
    params["target_gap_index"] = rng.randint(0, params["shelf_levels"] - 2)
    params["target_level"] = min(params["shelf_levels"] - 1, params["shelf_levels"] // 2)

    scene_rng = random.Random(seed)
    target_box = _build_target_box(params)
    clutter_boxes = _build_clutter_boxes(params, scene_rng, target_box)
    target_pose = _build_target_pose(params, scene_rng)

    derived = {
        "target_box": asdict(target_box),
        "clutter_boxes": [asdict(box) for box in clutter_boxes],
        "target_pose": target_pose,
        "inner_front_x": round(params["shelf_center_x"] - params["shelf_depth"] / 2.0, 6),
        "back_panel_front_x": round(
            _back_panel_front_x(params["shelf_center_x"], params["shelf_depth"], params["shelf_back_thickness"]),
            6,
        ),
    }
    return {"params": params, "derived": derived}


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate one deterministic random cabinet experiment case.")
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--clutter-count", type=int, default=0)
    parser.add_argument("--output", type=str, required=True)
    args = parser.parse_args()

    case = generate_case(seed=args.seed, clutter_count=args.clutter_count)
    with open(args.output, "w", encoding="utf-8") as handle:
        json.dump(case, handle, indent=2, ensure_ascii=False)
        handle.write("\n")


if __name__ == "__main__":
    main()
