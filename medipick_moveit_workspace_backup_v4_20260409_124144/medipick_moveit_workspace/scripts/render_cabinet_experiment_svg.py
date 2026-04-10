#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from pathlib import Path


def rect(x: float, y: float, width: float, height: float, fill: str, stroke: str = "#222", stroke_width: float = 2.0, opacity: float = 1.0) -> str:
    return (
        f'<rect x="{x:.2f}" y="{y:.2f}" width="{width:.2f}" height="{height:.2f}" '
        f'fill="{fill}" fill-opacity="{opacity:.2f}" stroke="{stroke}" stroke-width="{stroke_width:.2f}" rx="4" ry="4" />'
    )


def line(x1: float, y1: float, x2: float, y2: float, stroke: str, stroke_width: float = 3.0, dash: str | None = None) -> str:
    dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
    return (
        f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
        f'stroke="{stroke}" stroke-width="{stroke_width:.2f}"{dash_attr} />'
    )


def text(x: float, y: float, value: str, size: int = 18, fill: str = "#111", weight: str = "400") -> str:
    return f'<text x="{x:.2f}" y="{y:.2f}" font-size="{size}" font-weight="{weight}" fill="{fill}" font-family="DejaVu Sans Mono, monospace">{value}</text>'


def arrow(x: float, y: float, length: float, color: str) -> str:
    return (
        f'<g>'
        f'<line x1="{x:.2f}" y1="{y:.2f}" x2="{x + length:.2f}" y2="{y:.2f}" stroke="{color}" stroke-width="4"/>'
        f'<polygon points="{x + length:.2f},{y:.2f} {x + length - 14:.2f},{y - 8:.2f} {x + length - 14:.2f},{y + 8:.2f}" fill="{color}"/>'
        f'</g>'
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Render one cabinet experiment case to SVG.")
    parser.add_argument("--params", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    with open(args.params, "r", encoding="utf-8") as handle:
        payload = json.load(handle)

    params = payload["params"]
    derived = payload["derived"]
    target_box = derived["target_box"]
    target_pose = derived["target_pose"]["position"]
    clutter_boxes = derived.get("clutter_boxes", [])

    width = 1500
    height = 900
    margin = 50
    canvas: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f7f4ef" />',
        text(50, 45, f"Random Cabinet Experiment Case | seed={params['random_seed']}", 28, "#1f2a36", "700"),
    ]

    bottom_board_bottom = params["shelf_bottom_z"] - params["shelf_board_thickness"] / 2.0
    top_board_top = (
        params["shelf_bottom_z"]
        + (params["shelf_levels"] - 1) * params["shelf_level_gap"]
        + params["shelf_board_thickness"] / 2.0
    )
    cabinet_height = top_board_top - bottom_board_bottom

    front_origin_x = margin
    front_origin_y = 110
    front_width = 560
    front_height = 700
    front_scale_x = front_width / params["shelf_width"]
    front_scale_z = front_height / cabinet_height

    side_origin_x = 670
    side_origin_y = 110
    side_width = 430
    side_height = 700
    side_scale_x = side_width / params["shelf_depth"]
    side_scale_z = side_height / cabinet_height

    canvas.append(text(front_origin_x, 88, "Front View", 22, "#8a4f00", "700"))
    canvas.append(text(side_origin_x, 88, "Side View", 22, "#8a4f00", "700"))

    cabinet_front_x = front_origin_x
    cabinet_front_y = front_origin_y
    cabinet_front_w = params["shelf_width"] * front_scale_x
    cabinet_front_h = cabinet_height * front_scale_z
    canvas.append(rect(cabinet_front_x, cabinet_front_y, cabinet_front_w, cabinet_front_h, "#fff7e7", "#7b5c3d", 3.0))

    side_panel_w = params["shelf_side_thickness"] * front_scale_x
    canvas.append(rect(cabinet_front_x, cabinet_front_y, side_panel_w, cabinet_front_h, "#d9d1c7"))
    canvas.append(rect(cabinet_front_x + cabinet_front_w - side_panel_w, cabinet_front_y, side_panel_w, cabinet_front_h, "#d9d1c7"))

    for level_index in range(params["shelf_levels"]):
        level_z = params["shelf_bottom_z"] + level_index * params["shelf_level_gap"]
        board_bottom = level_z - params["shelf_board_thickness"] / 2.0
        board_y = front_origin_y + (top_board_top - (board_bottom + params["shelf_board_thickness"])) * front_scale_z
        canvas.append(rect(cabinet_front_x, board_y, cabinet_front_w, params["shelf_board_thickness"] * front_scale_z, "#d7b98e", "#9b7a54", 1.5))

    target_box_y = front_origin_y + (top_board_top - (target_box["center"][2] + target_box["size"][2] / 2.0)) * front_scale_z
    target_box_x = front_origin_x + ((target_box["center"][1] - target_box["size"][1] / 2.0) - (params["shelf_center_y"] - params["shelf_width"] / 2.0)) * front_scale_x
    canvas.append(rect(target_box_x, target_box_y, target_box["size"][1] * front_scale_x, target_box["size"][2] * front_scale_z, "#ffb347", "#d65a00", 2.5))

    pose_front_x = front_origin_x + (target_pose["y"] - (params["shelf_center_y"] - params["shelf_width"] / 2.0)) * front_scale_x
    pose_front_y = front_origin_y + (top_board_top - target_pose["z"]) * front_scale_z
    canvas.append(line(pose_front_x, pose_front_y - 18, pose_front_x, pose_front_y + 18, "#00a36c", 3.0))
    canvas.append(line(pose_front_x - 18, pose_front_y, pose_front_x + 18, pose_front_y, "#00a36c", 3.0))

    for clutter in clutter_boxes:
        clutter_y = front_origin_y + (top_board_top - (clutter["center"][2] + clutter["size"][2] / 2.0)) * front_scale_z
        clutter_x = front_origin_x + ((clutter["center"][1] - clutter["size"][1] / 2.0) - (params["shelf_center_y"] - params["shelf_width"] / 2.0)) * front_scale_x
        canvas.append(rect(clutter_x, clutter_y, clutter["size"][1] * front_scale_x, clutter["size"][2] * front_scale_z, "#c6d8ff", "#5a78b5", 1.5, 0.85))

    cabinet_side_x = side_origin_x
    cabinet_side_y = side_origin_y
    cabinet_side_w = params["shelf_depth"] * side_scale_x
    cabinet_side_h = cabinet_height * side_scale_z
    canvas.append(rect(cabinet_side_x, cabinet_side_y, cabinet_side_w, cabinet_side_h, "#fff7e7", "#7b5c3d", 3.0))
    canvas.append(rect(cabinet_side_x + cabinet_side_w - params["shelf_back_thickness"] * side_scale_x, cabinet_side_y, params["shelf_back_thickness"] * side_scale_x, cabinet_side_h, "#d9d1c7"))

    for level_index in range(params["shelf_levels"]):
        level_z = params["shelf_bottom_z"] + level_index * params["shelf_level_gap"]
        board_bottom = level_z - params["shelf_board_thickness"] / 2.0
        board_y = side_origin_y + (top_board_top - (board_bottom + params["shelf_board_thickness"])) * side_scale_z
        canvas.append(rect(cabinet_side_x, board_y, cabinet_side_w, params["shelf_board_thickness"] * side_scale_z, "#d7b98e", "#9b7a54", 1.5))

    shelf_front_x = params["shelf_center_x"] - params["shelf_depth"] / 2.0
    target_box_side_x = side_origin_x + ((target_box["center"][0] - target_box["size"][0] / 2.0) - shelf_front_x) * side_scale_x
    target_box_side_y = side_origin_y + (top_board_top - (target_box["center"][2] + target_box["size"][2] / 2.0)) * side_scale_z
    canvas.append(rect(target_box_side_x, target_box_side_y, target_box["size"][0] * side_scale_x, target_box["size"][2] * side_scale_z, "#ffb347", "#d65a00", 2.5))

    pose_side_x = side_origin_x + (target_pose["x"] - shelf_front_x) * side_scale_x
    pose_side_y = side_origin_y + (top_board_top - target_pose["z"]) * side_scale_z
    canvas.append(arrow(pose_side_x - 36, pose_side_y, 72, "#00a36c"))
    canvas.append(line(cabinet_side_x, pose_side_y, cabinet_side_x + cabinet_side_w, pose_side_y, "#00a36c", 1.5, "6 6"))

    info_x = 1140
    info_y = 120
    canvas.append(text(info_x, info_y, "Scenario Parameters", 22, "#1f2a36", "700"))
    info_lines = [
        f"levels={params['shelf_levels']} gap={params['target_gap_index']} target_level={params['target_level']}",
        f"center=({params['shelf_center_x']:.3f}, {params['shelf_center_y']:.3f})",
        f"size=({params['shelf_depth']:.3f}, {params['shelf_width']:.3f})",
        f"bottom_z={params['shelf_bottom_z']:.3f} level_gap={params['shelf_level_gap']:.3f}",
        f"board/side/back=({params['shelf_board_thickness']:.3f}, {params['shelf_side_thickness']:.3f}, {params['shelf_back_thickness']:.3f})",
        f"target_box=({params['target_box_size_x']:.3f}, {params['target_box_size_y']:.3f}, {params['target_box_size_z']:.3f})",
        f"target_depth_ratio={params['target_depth_ratio']:.3f}",
        f"target_lateral_margin={params['target_lateral_margin']:.3f}",
        f"target_lateral_span={params['target_lateral_span']:.3f}",
        f"target_pose=({target_pose['x']:.3f}, {target_pose['y']:.3f}, {target_pose['z']:.3f})",
        f"clutter_count={params['clutter_count']}",
    ]
    for index, line_text in enumerate(info_lines, start=1):
        canvas.append(text(info_x, info_y + 36 * index, line_text, 16, "#2f3d4a"))

    canvas.append(text(info_x, 620, "Legend", 22, "#1f2a36", "700"))
    canvas.append(rect(info_x, 644, 28, 18, "#ffb347", "#d65a00", 1.8))
    canvas.append(text(info_x + 42, 658, "Target medicine box", 16))
    canvas.append(line(info_x, 690, info_x + 28, 690, "#00a36c", 3.0))
    canvas.append(text(info_x + 42, 696, "Target pick pose / cabinet entry direction", 16))
    canvas.append(rect(info_x, 724, 28, 18, "#c6d8ff", "#5a78b5", 1.8, 0.85))
    canvas.append(text(info_x + 42, 738, "Clutter box (if enabled)", 16))

    canvas.append("</svg>")
    Path(args.output).write_text("\n".join(canvas), encoding="utf-8")


if __name__ == "__main__":
    main()
