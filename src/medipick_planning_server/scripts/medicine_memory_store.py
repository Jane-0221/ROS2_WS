#!/usr/bin/python3

from __future__ import annotations

import json
import sqlite3
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable


def default_db_path() -> Path:
    return Path.home() / ".local" / "share" / "medipick" / "medicine_memory" / "medicine_memory.sqlite3"


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _json_string(value: Any) -> str:
    if value in (None, ""):
        return ""
    if isinstance(value, str):
        return value
    return json.dumps(value, ensure_ascii=False, sort_keys=True)


def _dict_from_json_text(text: str) -> dict[str, Any]:
    if not text:
        return {}
    try:
        payload = json.loads(text)
    except json.JSONDecodeError:
        return {"raw": text}
    if isinstance(payload, dict):
        return payload
    return {"value": payload}


def _connect(db_path: Path) -> sqlite3.Connection:
    db_path.parent.mkdir(parents=True, exist_ok=True)
    connection = sqlite3.connect(str(db_path))
    connection.row_factory = sqlite3.Row
    connection.execute("PRAGMA journal_mode=WAL;")
    connection.execute("PRAGMA synchronous=NORMAL;")
    return connection


def ensure_schema(db_path: Path) -> None:
    with _connect(db_path) as connection:
        connection.executescript(
            """
            CREATE TABLE IF NOT EXISTS medicine_memory (
                medicine_id TEXT NOT NULL,
                location_key TEXT NOT NULL,
                display_name TEXT NOT NULL DEFAULT '',
                entity_name TEXT NOT NULL DEFAULT '',
                frame_id TEXT NOT NULL,
                position_x REAL NOT NULL,
                position_y REAL NOT NULL,
                position_z REAL NOT NULL,
                orientation_x REAL NOT NULL,
                orientation_y REAL NOT NULL,
                orientation_z REAL NOT NULL,
                orientation_w REAL NOT NULL,
                confidence REAL NOT NULL DEFAULT 0.0,
                source TEXT NOT NULL DEFAULT '',
                shelf_info_json TEXT NOT NULL DEFAULT '',
                extra_json TEXT NOT NULL DEFAULT '',
                first_seen_at TEXT NOT NULL,
                last_seen_at TEXT NOT NULL,
                seen_count INTEGER NOT NULL DEFAULT 1,
                is_active INTEGER NOT NULL DEFAULT 1,
                PRIMARY KEY (medicine_id, location_key)
            );

            CREATE INDEX IF NOT EXISTS idx_medicine_memory_last_seen
            ON medicine_memory (medicine_id, last_seen_at DESC);
            """
        )


@dataclass
class MedicineObservation:
    medicine_id: str
    location_key: str
    frame_id: str
    position_x: float
    position_y: float
    position_z: float
    orientation_x: float
    orientation_y: float
    orientation_z: float
    orientation_w: float
    display_name: str = ""
    entity_name: str = ""
    confidence: float = 0.0
    source: str = ""
    shelf_info_json: str = ""
    extra_json: str = ""

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> "MedicineObservation":
        medicine_id = str(payload.get("medicine_id", "")).strip()
        if not medicine_id:
            raise ValueError("medicine_id 不能为空。")

        pose_block = payload.get("pose", {})
        position_block = payload.get("position", pose_block.get("position", {}))
        orientation_block = payload.get("orientation", pose_block.get("orientation", {}))
        frame_id = str(payload.get("frame_id", pose_block.get("frame_id", ""))).strip() or "map"

        entity_name = str(payload.get("entity_name", "")).strip()
        location_key = str(payload.get("location_key", "")).strip()
        if not location_key:
            location_key = entity_name or medicine_id

        return cls(
            medicine_id=medicine_id,
            location_key=location_key,
            frame_id=frame_id,
            position_x=float(position_block.get("x", 0.0)),
            position_y=float(position_block.get("y", 0.0)),
            position_z=float(position_block.get("z", 0.0)),
            orientation_x=float(orientation_block.get("x", 0.0)),
            orientation_y=float(orientation_block.get("y", 0.0)),
            orientation_z=float(orientation_block.get("z", 0.0)),
            orientation_w=float(orientation_block.get("w", 1.0)),
            display_name=str(payload.get("display_name", "")).strip(),
            entity_name=entity_name,
            confidence=float(payload.get("confidence", 0.0)),
            source=str(payload.get("source", "")).strip(),
            shelf_info_json=_json_string(payload.get("shelf_info", "")),
            extra_json=_json_string(payload.get("extra", "")),
        )

    def as_debug_dict(self) -> dict[str, Any]:
        return {
            "medicine_id": self.medicine_id,
            "location_key": self.location_key,
            "display_name": self.display_name,
            "entity_name": self.entity_name,
            "frame_id": self.frame_id,
            "position": {
                "x": self.position_x,
                "y": self.position_y,
                "z": self.position_z,
            },
            "orientation": {
                "x": self.orientation_x,
                "y": self.orientation_y,
                "z": self.orientation_z,
                "w": self.orientation_w,
            },
            "confidence": self.confidence,
            "source": self.source,
            "shelf_info": _dict_from_json_text(self.shelf_info_json),
            "extra": _dict_from_json_text(self.extra_json),
        }


def upsert_observation(db_path: Path, observation: MedicineObservation) -> dict[str, Any]:
    ensure_schema(db_path)
    now_iso = utc_now_iso()

    with _connect(db_path) as connection:
        connection.execute(
            """
            INSERT INTO medicine_memory (
                medicine_id,
                location_key,
                display_name,
                entity_name,
                frame_id,
                position_x,
                position_y,
                position_z,
                orientation_x,
                orientation_y,
                orientation_z,
                orientation_w,
                confidence,
                source,
                shelf_info_json,
                extra_json,
                first_seen_at,
                last_seen_at,
                seen_count,
                is_active
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, 1, 1)
            ON CONFLICT(medicine_id, location_key) DO UPDATE SET
                display_name=excluded.display_name,
                entity_name=CASE
                    WHEN excluded.entity_name != '' THEN excluded.entity_name
                    ELSE medicine_memory.entity_name
                END,
                frame_id=excluded.frame_id,
                position_x=excluded.position_x,
                position_y=excluded.position_y,
                position_z=excluded.position_z,
                orientation_x=excluded.orientation_x,
                orientation_y=excluded.orientation_y,
                orientation_z=excluded.orientation_z,
                orientation_w=excluded.orientation_w,
                confidence=excluded.confidence,
                source=excluded.source,
                shelf_info_json=CASE
                    WHEN excluded.shelf_info_json != '' THEN excluded.shelf_info_json
                    ELSE medicine_memory.shelf_info_json
                END,
                extra_json=CASE
                    WHEN excluded.extra_json != '' THEN excluded.extra_json
                    ELSE medicine_memory.extra_json
                END,
                last_seen_at=excluded.last_seen_at,
                seen_count=medicine_memory.seen_count + 1,
                is_active=1
            """,
            (
                observation.medicine_id,
                observation.location_key,
                observation.display_name,
                observation.entity_name,
                observation.frame_id,
                observation.position_x,
                observation.position_y,
                observation.position_z,
                observation.orientation_x,
                observation.orientation_y,
                observation.orientation_z,
                observation.orientation_w,
                observation.confidence,
                observation.source,
                observation.shelf_info_json,
                observation.extra_json,
                now_iso,
                now_iso,
            ),
        )
        row = connection.execute(
            """
            SELECT *
            FROM medicine_memory
            WHERE medicine_id = ? AND location_key = ?
            """,
            (observation.medicine_id, observation.location_key),
        ).fetchone()
    return row_to_dict(row)


def get_best_record(
    db_path: Path,
    medicine_id: str,
    location_key: str = "",
) -> dict[str, Any] | None:
    ensure_schema(db_path)
    query = """
        SELECT *
        FROM medicine_memory
        WHERE medicine_id = ? AND is_active = 1
    """
    params: list[Any] = [medicine_id]
    if location_key:
        query += " AND location_key = ?"
        params.append(location_key)
    query += " ORDER BY last_seen_at DESC, confidence DESC, seen_count DESC LIMIT 1"

    with _connect(db_path) as connection:
        row = connection.execute(query, tuple(params)).fetchone()
    if row is None:
        return None
    return row_to_dict(row)


def list_records(db_path: Path, medicine_id: str = "") -> list[dict[str, Any]]:
    ensure_schema(db_path)
    query = """
        SELECT *
        FROM medicine_memory
        WHERE is_active = 1
    """
    params: list[Any] = []
    if medicine_id:
        query += " AND medicine_id = ?"
        params.append(medicine_id)
    query += " ORDER BY medicine_id ASC, last_seen_at DESC"
    with _connect(db_path) as connection:
        rows = connection.execute(query, tuple(params)).fetchall()
    return [row_to_dict(row) for row in rows]


def row_to_dict(row: sqlite3.Row | None) -> dict[str, Any]:
    if row is None:
        return {}
    result = dict(row)
    result["shelf_info"] = _dict_from_json_text(result.pop("shelf_info_json", ""))
    result["extra"] = _dict_from_json_text(result.pop("extra_json", ""))
    return result


def format_records_table(records: Iterable[dict[str, Any]]) -> str:
    headers = [
        "medicine_id",
        "location_key",
        "entity_name",
        "frame_id",
        "x",
        "y",
        "z",
        "confidence",
        "seen_count",
        "last_seen_at",
    ]
    rows = []
    for record in records:
        rows.append(
            [
                str(record.get("medicine_id", "")),
                str(record.get("location_key", "")),
                str(record.get("entity_name", "")),
                str(record.get("frame_id", "")),
                f"{float(record.get('position_x', 0.0)):.3f}",
                f"{float(record.get('position_y', 0.0)):.3f}",
                f"{float(record.get('position_z', 0.0)):.3f}",
                f"{float(record.get('confidence', 0.0)):.3f}",
                str(record.get("seen_count", 0)),
                str(record.get("last_seen_at", "")),
            ]
        )

    widths = [len(header) for header in headers]
    for row in rows:
        for index, value in enumerate(row):
            widths[index] = max(widths[index], len(value))

    def render_row(values: list[str]) -> str:
        return " | ".join(value.ljust(widths[index]) for index, value in enumerate(values))

    lines = [render_row(headers), "-+-".join("-" * width for width in widths)]
    lines.extend(render_row(row) for row in rows)
    return "\n".join(lines)
