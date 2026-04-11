#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


LOG_PATTERN = re.compile(
    r"^\[(?P<proc>[^\]]+)\]\s+\[(?P<level>[A-Z]+)\]\s+\[(?P<timestamp>\d+\.\d+)\]\s+\[(?P<logger>[^\]]+)\]:\s+(?P<message>.*)$"
)


def parse_log(log_path: Path) -> dict:
    transitions: list[dict] = []
    errors: list[dict] = []
    warnings: list[dict] = []
    last_manager_error: str | None = None

    with log_path.open("r", encoding="utf-8", errors="replace") as handle:
        for raw_line in handle:
            line = raw_line.rstrip("\n")
            match = LOG_PATTERN.match(line)
            if not match:
                continue
            level = match.group("level")
            timestamp = float(match.group("timestamp"))
            logger = match.group("logger")
            message = match.group("message")

            if logger == "medipick_pick_task_manager" and message.startswith("Stage -> "):
                transitions.append({"stage": message.replace("Stage -> ", "", 1), "timestamp": timestamp})
            if logger == "medipick_pick_task_manager" and level == "ERROR":
                last_manager_error = message
                errors.append({"timestamp": timestamp, "message": message})
            elif level == "ERROR":
                errors.append({"timestamp": timestamp, "logger": logger, "message": message})
            elif level == "WARN":
                warnings.append({"timestamp": timestamp, "logger": logger, "message": message})

    stage_timeline: list[dict] = []
    stage_totals: dict[str, dict] = {}
    for current, nxt in zip(transitions, transitions[1:]):
        duration = max(0.0, nxt["timestamp"] - current["timestamp"])
        entry = {
            "stage": current["stage"],
            "started_at": current["timestamp"],
            "ended_at": nxt["timestamp"],
            "duration_sec": duration,
        }
        stage_timeline.append(entry)
        total = stage_totals.setdefault(current["stage"], {"attempts": 0, "total_duration_sec": 0.0})
        total["attempts"] += 1
        total["total_duration_sec"] += duration

    final_stage = transitions[-1]["stage"] if transitions else "UNKNOWN"
    success = final_stage == "COMPLETED"
    return {
        "success": success,
        "final_stage": final_stage,
        "failure_message": None if success else last_manager_error,
        "stage_transitions": transitions,
        "stage_timeline": stage_timeline,
        "stage_totals": stage_totals,
        "errors": errors,
        "warnings": warnings,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Parse one Medipick experiment log into structured JSON.")
    parser.add_argument("--log", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    summary = parse_log(Path(args.log))
    with open(args.output, "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2, ensure_ascii=False)
        handle.write("\n")


if __name__ == "__main__":
    main()
