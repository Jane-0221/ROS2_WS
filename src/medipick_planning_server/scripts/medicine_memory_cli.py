#!/usr/bin/python3

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from medicine_memory_store import (
    MedicineObservation,
    default_db_path,
    format_records_table,
    get_best_record,
    list_records,
    upsert_observation,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Medipick 药品记忆库 CLI")
    parser.add_argument("--db-path", default=str(default_db_path()))

    subparsers = parser.add_subparsers(dest="command", required=True)

    list_parser = subparsers.add_parser("list", help="列出药品记忆库记录")
    list_parser.add_argument("--medicine-id", default="")

    get_parser = subparsers.add_parser("get", help="读取某个药品的最佳记录")
    get_parser.add_argument("--medicine-id", required=True)
    get_parser.add_argument("--location-key", default="")

    upsert_parser = subparsers.add_parser("upsert", help="直接写入一条药品观察记录")
    upsert_parser.add_argument("--medicine-id", required=True)
    upsert_parser.add_argument("--location-key", default="")
    upsert_parser.add_argument("--display-name", default="")
    upsert_parser.add_argument("--entity-name", default="")
    upsert_parser.add_argument("--frame-id", default="map")
    upsert_parser.add_argument("--x", type=float, required=True)
    upsert_parser.add_argument("--y", type=float, required=True)
    upsert_parser.add_argument("--z", type=float, required=True)
    upsert_parser.add_argument("--qx", type=float, default=0.0)
    upsert_parser.add_argument("--qy", type=float, default=0.0)
    upsert_parser.add_argument("--qz", type=float, default=0.0)
    upsert_parser.add_argument("--qw", type=float, default=1.0)
    upsert_parser.add_argument("--confidence", type=float, default=1.0)
    upsert_parser.add_argument("--source", default="manual_cli")
    upsert_parser.add_argument("--shelf-info", default="")
    upsert_parser.add_argument("--extra", default="")

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    db_path = Path(args.db_path).expanduser()

    if args.command == "list":
        records = list_records(db_path, medicine_id=args.medicine_id)
        if not records:
            print("药品记忆库为空。")
            return 0
        print(format_records_table(records))
        return 0

    if args.command == "get":
        record = get_best_record(db_path, args.medicine_id, args.location_key)
        if not record:
            print("未找到记录。", file=sys.stderr)
            return 1
        print(json.dumps(record, ensure_ascii=False, indent=2, sort_keys=True))
        return 0

    if args.command == "upsert":
        observation = MedicineObservation(
            medicine_id=args.medicine_id,
            location_key=args.location_key or args.entity_name or args.medicine_id,
            display_name=args.display_name,
            entity_name=args.entity_name,
            frame_id=args.frame_id,
            position_x=args.x,
            position_y=args.y,
            position_z=args.z,
            orientation_x=args.qx,
            orientation_y=args.qy,
            orientation_z=args.qz,
            orientation_w=args.qw,
            confidence=args.confidence,
            source=args.source,
            shelf_info_json=args.shelf_info,
            extra_json=args.extra,
        )
        record = upsert_observation(db_path, observation)
        print(json.dumps(record, ensure_ascii=False, indent=2, sort_keys=True))
        return 0

    parser.error("未知命令")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
