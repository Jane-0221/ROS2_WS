#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path


def _load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _safe_load_json(path: Path) -> dict | None:
    if not path.exists():
        return None
    return _load_json(path)


def _stage_total(result_payload: dict) -> float:
    return float(sum(entry.get("duration_sec", 0.0) for entry in result_payload.get("stage_timeline", [])))


def _slow_stages(stage_timeline: list[dict], limit: float = 2.0) -> list[dict]:
    return [entry for entry in stage_timeline if float(entry.get("duration_sec", 0.0)) >= limit]


def _classify_failure(case_dir: Path, result_payload: dict) -> tuple[str, list[str]]:
    message = (result_payload.get("failure_message") or "").lower()
    error_messages = " | ".join(str(item.get("message", "")).lower() for item in result_payload.get("errors", []))
    warning_messages = " | ".join(str(item.get("message", "")).lower() for item in result_payload.get("warnings", []))
    combined = f"{message} | {error_messages} | {warning_messages}"
    final_stage = result_payload.get("final_stage", "UNKNOWN")

    suggestions: list[str] = []

    if final_stage == "ACQUIRE_TARGET":
        if "failed to send response to /compute_fk" in combined or "/compute_fk" in combined:
            suggestions.extend(
                [
                    "在 `ACQUIRE_TARGET` 前增加对 `/compute_fk`、`/check_state_validity` 等 MoveIt service 的 readiness 检查，不要只靠节点启动完成就立即进主流程。",
                    "把 mock 货架/点云发布后的短暂稳定窗口显式化，例如在首帧目标 pose 和点云都到齐后再允许 `ACQUIRE_TARGET` 调 FK。",
                    "优先处理反复出现的 `shape_mask: Missing transform for shape mesh`，这更像是场景物体 frame 或发布时间早于 TF 稳定的同步问题。"
                ]
            )
            return "startup_fk_or_scene_sync", suggestions
        suggestions.extend(
            [
                "给 `ACQUIRE_TARGET` 增加更明确的失败信息和重试机制，区分“目标未到达”“服务未就绪”“场景同步未完成”。",
            ]
        )
        return "acquire_target_other", suggestions

    if "pre-insert planning failed" in combined or final_stage == "PLAN_TO_PRE_INSERT":
        if "timed_out" in combined or "timed out" in combined:
            suggestions.extend(
                [
                    "让 `PLAN_TO_PRE_INSERT` 对深柜或窄柜自适应增大 `candidate_retry_planning_time` 和 `candidate_retry_num_planning_attempts`，而不是固定 1 次快速尝试。",
                    "按柜深动态增大 `pre_insert_offset` 和 `base_standoff`，减少手臂在柜口外就需要大折叠的情况。",
                ]
            )
            return "pre_insert_timeout", suggestions
        if "invalid_motion_plan" in combined or "collision" in combined:
            if "base_link" in combined and "r2_link" in combined:
                suggestions.append("继续优化 `ARM_STOW_SAFE` 和 `LIFT_TO_BAND`，让底盘靠近后手臂不需要通过 `r2` 反折到机体附近。")
            suggestions.extend(
                [
                    "给 `PLAN_TO_PRE_INSERT` 增加按目标高度和柜口几何自适应的站位/升降预调，而不是固定常数。",
                    "把 `pre-insert` 从单 pose 再扩成少量 2-3 个候选，但仍保持 seeded IK 优先，兼顾速度和兜底能力。",
                ]
            )
            return "pre_insert_invalid_motion", suggestions
        suggestions.extend(
            [
                "保留 seeded IK 优先，但给 pose fallback 增加更清晰的失败分流日志，区分 IK 不可达、碰撞、超时三类。",
                "对 `mobile_arm` 的 pre-insert 规划做深柜自适应站位调整。"
            ]
        )
        return "pre_insert_other", suggestions

    if "r1 stage motion" in combined or "r1_limit" in combined:
        suggestions.extend(
            [
                "把 `PLAN_TO_PRE_INSERT` 的 seeded IK 终点筛选也接入 `r1` 阶段位移代价，提前避开接近 100 度阈值的解，而不是规划完再拒绝。",
                "对这类柜型给 `pre-insert` 加一个更偏向当前 `r1` 半空间的小候选偏置，减少 `mobile_arm` 为了够到柜口而反向大摆。",
                "必要时把 `r1` 限制从硬阈值拆成两级：90 度以上重罚，100 度以上拒绝。"
            ]
        )
        return "pre_insert_r1_motion_limit", suggestions

    if final_stage == "SAFE_RETREAT" or "retreat" in combined:
        if "r1 stage motion" in combined or "r1_limit" in combined:
            suggestions.extend(
                [
                    "给 `SAFE_RETREAT` 增加 seeded IK / 短 joint-space 撤退，避免 OMPL 走出大绕圈。",
                    "把 `r1` 阶段运动限幅和撤退方向耦合：若直退可行，优先直退，不再尝试大角度绕行。 ",
                ]
            )
            return "retreat_r1_motion_limit", suggestions
        suggestions.extend(
            [
                "撤退阶段优先沿插入反方向做短路径退出，只有退出失败才回退到更宽松的全局规划。",
            ]
        )
        return "retreat_other", suggestions

    if final_stage == "LIFT_TO_BAND" or "lift" in combined:
        suggestions.extend(
            [
                "继续强化末端高度对齐逻辑，让 `raise_joint` 目标基于当前收臂 FK 高度和目标位姿动态反推。",
                "对不同层高把 `lift_band_half_width` 变成比例或分段参数，避免高层/低层误差感受不一致。",
            ]
        )
        return "lift_alignment", suggestions

    if "controller rejected trajectory" in combined or "doesn't match the controller's joints" in combined:
        suggestions.extend(
            [
                "确保所有执行前轨迹都按控制器白名单裁剪 joint，特别是 seeded IK 分支输出的全状态轨迹。",
            ]
        )
        return "controller_joint_mismatch", suggestions

    if final_stage == "FAILED":
        suggestions.extend(
            [
                "为失败 case 自动导出更细的分类字段，例如 `ik_failed`、`collision_blocked`、`stage_timeout`，避免后续只能人工读日志。",
                "在 case README 里追加失败时的最后 3 条 manager error，便于快速复盘。",
            ]
        )
        return "generic_failed", suggestions

    suggestions.append("需要补充更细的失败分类规则，当前日志还不足以自动定位根因。")
    return "unclassified", suggestions


def build_analysis(root_dir: Path) -> None:
    case_dirs = sorted(path for path in root_dir.iterdir() if path.is_dir() and path.name.startswith("case_"))
    failures: list[dict] = []
    successes: list[dict] = []
    slow_stage_counter: Counter[str] = Counter()
    stage_duration_samples: defaultdict[str, list[float]] = defaultdict(list)

    for case_dir in case_dirs:
        params_payload = _load_json(case_dir / "params.json")
        result_payload = _load_json(case_dir / "result.json")
        video_payload = _safe_load_json(case_dir / "video.json")
        stage_timeline = result_payload.get("stage_timeline", [])
        for entry in stage_timeline:
            stage_duration_samples[entry["stage"]].append(float(entry["duration_sec"]))
        for entry in _slow_stages(stage_timeline):
            slow_stage_counter[entry["stage"]] += 1

        record = {
            "case": case_dir.name,
            "seed": params_payload["params"]["random_seed"],
            "success": bool(result_payload.get("success")),
            "final_stage": result_payload.get("final_stage"),
            "failure_message": result_payload.get("failure_message"),
            "total_stage_time_sec": round(_stage_total(result_payload), 3),
            "params": params_payload["params"],
            "result": result_payload,
            "video": video_payload,
        }
        if record["success"]:
            successes.append(record)
        else:
            failure_type, suggestions = _classify_failure(case_dir, result_payload)
            record["failure_type"] = failure_type
            record["suggestions"] = suggestions
            failures.append(record)

    failure_counter = Counter(item["failure_type"] for item in failures)
    avg_stage_duration = {
        stage: round(sum(values) / len(values), 3)
        for stage, values in sorted(stage_duration_samples.items())
        if values
    }

    summary_payload = {
        "total_cases": len(case_dirs),
        "successful_cases": len(successes),
        "failed_cases": len(failures),
        "success_ratio": round((len(successes) / len(case_dirs) * 100.0), 1) if case_dirs else 0.0,
        "failure_type_counter": dict(failure_counter),
        "avg_stage_duration_sec": avg_stage_duration,
        "slow_stage_counter_over_2s": dict(slow_stage_counter),
    }

    (root_dir / "analysis.json").write_text(
        json.dumps(
            {
                "summary": summary_payload,
                "failures": failures,
                "successes": [
                    {
                        "case": item["case"],
                        "seed": item["seed"],
                        "final_stage": item["final_stage"],
                        "total_stage_time_sec": item["total_stage_time_sec"],
                    }
                    for item in successes
                ],
            },
            indent=2,
            ensure_ascii=False,
        )
        + "\n",
        encoding="utf-8",
    )

    lines = [
        f"# Failure Analysis: {root_dir.name}",
        "",
        "## Summary",
        "",
        f"- Total cases: `{summary_payload['total_cases']}`",
        f"- Successful cases: `{summary_payload['successful_cases']}`",
        f"- Failed cases: `{summary_payload['failed_cases']}`",
        f"- Success ratio: `{summary_payload['success_ratio']:.1f}%`",
        "",
        "## Average Stage Duration",
        "",
    ]
    if avg_stage_duration:
        lines.extend([f"- `{stage}`: `{duration:.3f}s`" for stage, duration in avg_stage_duration.items()])
    else:
        lines.append("- No stage durations parsed.")

    lines.extend(
        [
            "",
            "## Slow Stage Frequency",
            "",
        ]
    )
    if slow_stage_counter:
        lines.extend([f"- `{stage}`: `{count}` cases exceeded `2.0s`" for stage, count in slow_stage_counter.most_common()])
    else:
        lines.append("- No stage exceeded the slow-stage threshold.")

    lines.extend(
        [
            "",
            "## Failure Types",
            "",
        ]
    )
    if failure_counter:
        lines.extend([f"- `{failure_type}`: `{count}` cases" for failure_type, count in failure_counter.most_common()])
    else:
        lines.append("- No failures in this experiment batch.")

    lines.extend(
        [
            "",
            "## Failure Details",
            "",
        ]
    )
    if not failures:
        lines.append("- No failures to analyze.")
    else:
        for item in failures:
            params = item["params"]
            lines.extend(
                [
                    f"### {item['case']}",
                    "",
                    f"- Seed: `{item['seed']}`",
                    f"- Final stage: `{item['final_stage']}`",
                    f"- Failure type: `{item['failure_type']}`",
                    f"- Failure message: `{item['failure_message'] or 'N/A'}`",
                    f"- Shelf size (depth,width): `({params['shelf_depth']:.3f}, {params['shelf_width']:.3f})`",
                    f"- Shelf center: `({params['shelf_center_x']:.3f}, {params['shelf_center_y']:.3f})`",
                    f"- Shelf bottom / level gap: `({params['shelf_bottom_z']:.3f}, {params['shelf_level_gap']:.3f})`",
                    f"- Total parsed stage time: `{item['total_stage_time_sec']:.3f}s`",
                    f"- Detailed case: [README.md](./{item['case']}/README.md)",
                    "- Suggested improvements:",
                ]
            )
            lines.extend([f"- {suggestion}" for suggestion in item["suggestions"]])
            lines.append("")

    lines.extend(
        [
            "## Suggested Next Actions",
            "",
        ]
    )
    if failure_counter.get("pre_insert_invalid_motion", 0) or failure_counter.get("pre_insert_timeout", 0):
        lines.append("- 优先做柜深/层高驱动的 `base_standoff` 与 `pre_insert_offset` 自适应，而不是继续用全局常数。")
    if failure_counter.get("retreat_r1_motion_limit", 0):
        lines.append("- 给 `SAFE_RETREAT` 引入 seeded IK + 短 joint-space 退出，减少 `r1` 大幅回绕。")
    if slow_stage_counter.get("ARM_STOW_SAFE", 0) or slow_stage_counter.get("BASE_ENTER_WORKSPACE", 0):
        lines.append("- 检查收臂模板和底盘到位判定，必要时把 transit stow 再收紧，缩短前置阶段耗时。")
    if not failures:
        lines.append("- 当前这批未出现失败，可以继续扩大随机范围：增加 `clutter_count`、扩大 `shelf_depth` 和 `target_lateral_span`。")

    (root_dir / "analysis.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze one random cabinet experiment directory and produce failure insights.")
    parser.add_argument("--root", required=True)
    args = parser.parse_args()
    build_analysis(Path(args.root))


if __name__ == "__main__":
    main()
