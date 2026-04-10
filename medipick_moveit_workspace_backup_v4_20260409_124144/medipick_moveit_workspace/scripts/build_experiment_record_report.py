#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def _load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _load_optional_json(path: Path) -> dict | None:
    if not path.exists():
        return None
    return _load_json(path)


def _format_stage_lines(stage_timeline: list[dict]) -> list[str]:
    if not stage_timeline:
        return ["No stage timeline parsed."]
    return [
        f"- `{entry['stage']}`: {entry['duration_sec']:.3f}s"
        for entry in stage_timeline
    ]


def build_case_readme(case_dir: Path, params_payload: dict, result_payload: dict) -> None:
    params = params_payload["params"]
    derived = params_payload["derived"]
    video_payload = _load_optional_json(case_dir / "video.json")
    lines = [
        f"# {case_dir.name}",
        "",
        f"![scene](./scene.svg)",
        "",
        "## Result",
        "",
        f"- Success: `{result_payload['success']}`",
        f"- Final stage: `{result_payload['final_stage']}`",
    ]
    if result_payload.get("failure_message"):
        lines.append(f"- Failure message: `{result_payload['failure_message']}`")
    if video_payload is not None:
        lines.append(f"- Video recorded: `{video_payload.get('success', False)}`")
    lines.extend(
        [
            "",
            "## Parameters",
            "",
            f"- Seed: `{params['random_seed']}`",
            f"- Shelf levels: `{params['shelf_levels']}`",
            f"- Target gap index: `{params['target_gap_index']}`",
            f"- Target level: `{params['target_level']}`",
            f"- Shelf center: `({params['shelf_center_x']:.3f}, {params['shelf_center_y']:.3f})`",
            f"- Shelf size (depth,width): `({params['shelf_depth']:.3f}, {params['shelf_width']:.3f})`",
            f"- Shelf bottom / level gap: `({params['shelf_bottom_z']:.3f}, {params['shelf_level_gap']:.3f})`",
            f"- Shelf board / side / back thickness: `({params['shelf_board_thickness']:.3f}, {params['shelf_side_thickness']:.3f}, {params['shelf_back_thickness']:.3f})`",
            f"- Target box size: `({params['target_box_size_x']:.3f}, {params['target_box_size_y']:.3f}, {params['target_box_size_z']:.3f})`",
            f"- Target pose: `({derived['target_pose']['position']['x']:.3f}, {derived['target_pose']['position']['y']:.3f}, {derived['target_pose']['position']['z']:.3f})`",
            "",
            "## Stage Durations",
            "",
        ]
    )
    lines.extend(_format_stage_lines(result_payload.get("stage_timeline", [])))
    lines.extend(
        [
            "",
            "## Video",
            "",
        ]
    )
    if video_payload is None:
        lines.append("- No video metadata was generated for this case.")
    else:
        lines.append(f"- Record success: `{video_payload.get('success', False)}`")
        lines.append(f"- Reason: `{video_payload.get('reason', 'unknown')}`")
        if video_payload.get("frames") is not None:
            lines.append(f"- Frames: `{video_payload.get('frames')}`")
        if video_payload.get("duration_sec") is not None:
            lines.append(f"- Duration: `{float(video_payload.get('duration_sec')):.3f}s`")
        if video_payload.get("video_width") is not None and video_payload.get("video_height") is not None:
            lines.append(
                f"- Stored size: `{video_payload.get('video_width')}x{video_payload.get('video_height')}`"
            )
        if (case_dir / "rviz_capture.avi").exists():
            lines.append("- Video file: [rviz_capture.avi](./rviz_capture.avi)")
        if (case_dir / "video.log").exists():
            lines.append("- Recorder log: [video.log](./video.log)")
    lines.extend(
        [
            "",
            "## Files",
            "",
            "- `scene.svg`: cabinet image",
            "- `params.json`: generated cabinet parameters",
            "- `result.json`: parsed experiment result",
            "- `run.log`: raw ROS/MoveIt log",
            *(
                ["- `rviz_capture.avi`: recorded RViz video"]
                if (case_dir / "rviz_capture.avi").exists()
                else []
            ),
            *(
                ["- `video.json`: recorder metadata"]
                if (case_dir / "video.json").exists()
                else []
            ),
            *(
                ["- `video.log`: recorder stdout/stderr"]
                if (case_dir / "video.log").exists()
                else []
            ),
            "",
        ]
    )
    (case_dir / "README.md").write_text("\n".join(lines), encoding="utf-8")


def build_root_report(root_dir: Path) -> None:
    case_dirs = sorted(path for path in root_dir.iterdir() if path.is_dir() and path.name.startswith("case_"))
    rows: list[dict] = []
    readme_lines = [
        f"# Random Cabinet Experiment Record: {root_dir.name}",
        "",
        "## Cases",
        "",
    ]

    for case_dir in case_dirs:
        params_payload = _load_json(case_dir / "params.json")
        result_payload = _load_json(case_dir / "result.json")
        video_payload = _load_optional_json(case_dir / "video.json")
        build_case_readme(case_dir, params_payload, result_payload)
        params = params_payload["params"]
        stage_timeline = result_payload.get("stage_timeline", [])
        total_duration = sum(entry["duration_sec"] for entry in stage_timeline)
        rows.append(
            {
                "case": case_dir.name,
                "seed": params["random_seed"],
                "success": result_payload["success"],
                "final_stage": result_payload["final_stage"],
                "total_stage_time_sec": f"{total_duration:.3f}",
                "video_recorded": video_payload.get("success", False) if video_payload else False,
                "video_duration_sec": (
                    f"{float(video_payload['duration_sec']):.3f}"
                    if video_payload and video_payload.get("duration_sec") is not None
                    else ""
                ),
                "failure_message": result_payload.get("failure_message") or "",
            }
        )
        readme_lines.extend(
            [
                f"### {case_dir.name}",
                "",
                f"![{case_dir.name}](./{case_dir.name}/scene.svg)",
                "",
                f"- Seed: `{params['random_seed']}`",
                f"- Success: `{result_payload['success']}`",
                f"- Final stage: `{result_payload['final_stage']}`",
                f"- Shelf size (depth,width): `({params['shelf_depth']:.3f}, {params['shelf_width']:.3f})`",
                f"- Shelf center: `({params['shelf_center_x']:.3f}, {params['shelf_center_y']:.3f})`",
                f"- Shelf bottom / level gap: `({params['shelf_bottom_z']:.3f}, {params['shelf_level_gap']:.3f})`",
                f"- Target box size: `({params['target_box_size_x']:.3f}, {params['target_box_size_y']:.3f}, {params['target_box_size_z']:.3f})`",
                f"- Video recorded: `{video_payload.get('success', False) if video_payload else False}`",
                f"- Failure message: `{result_payload.get('failure_message') or 'N/A'}`",
                "- Stage durations:",
            ]
        )
        readme_lines.extend(_format_stage_lines(stage_timeline))
        if (case_dir / "rviz_capture.avi").exists():
            readme_lines.append(f"- Video: [rviz_capture.avi](./{case_dir.name}/rviz_capture.avi)")
        readme_lines.extend(
            [
                f"- Detailed record: [README.md](./{case_dir.name}/README.md)",
                "",
            ]
        )

    pass_count = sum(1 for row in rows if row["success"] in (True, "True"))
    summary_lines = [
        f"- Total cases: `{len(rows)}`",
        f"- Successful cases: `{pass_count}`",
        f"- Success ratio: `{(pass_count / len(rows) * 100.0):.1f}%`" if rows else "- Success ratio: `0.0%`",
        "- Failure analysis: [analysis.md](./analysis.md)" if (root_dir / "analysis.md").exists() else "",
        "",
    ]
    readme_lines[2:2] = summary_lines
    (root_dir / "README.md").write_text("\n".join(readme_lines), encoding="utf-8")

    if rows:
        with (root_dir / "summary.csv").open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description="Build markdown and CSV summaries for one experiment record directory.")
    parser.add_argument("--root", required=True)
    args = parser.parse_args()
    build_root_report(Path(args.root))


if __name__ == "__main__":
    main()
