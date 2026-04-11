#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import os
import re
import struct
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np


XWD_HEADER_FORMAT = ">25I"
XWD_HEADER_SIZE = struct.calcsize(XWD_HEADER_FORMAT)
XWD_COLOR_SIZE = 12


@dataclass
class WindowMatch:
    window_id: str
    title: str
    width: int
    height: int


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record one X11 window to an uncompressed AVI file.")
    parser.add_argument("--output", required=True, help="Target .avi file.")
    parser.add_argument("--metadata-output", required=True, help="Target .json metadata file.")
    parser.add_argument("--window-pattern", default="RViz|rviz", help="Regex used to find the target window title.")
    parser.add_argument("--fps", type=float, default=2.0, help="Capture frames per second.")
    parser.add_argument("--max-width", type=int, default=640, help="Maximum stored video width.")
    parser.add_argument("--max-height", type=int, default=360, help="Maximum stored video height.")
    parser.add_argument("--wait-timeout", type=float, default=30.0, help="How long to wait for the window to appear.")
    parser.add_argument("--capture-timeout", type=float, default=120.0, help="Maximum recording duration after capture starts.")
    parser.add_argument("--stop-file", default="", help="Stop recording once this file exists.")
    return parser.parse_args()


def _list_windows() -> list[WindowMatch]:
    completed = subprocess.run(
        ["xwininfo", "-root", "-tree"],
        check=True,
        capture_output=True,
        text=True,
        errors="replace",
    )
    pattern = re.compile(r'^\s*(0x[0-9a-fA-F]+)\s+"([^"]*)":.*?\s(\d+)x(\d+)[+-]\d+[+-]\d+\s')
    matches: list[WindowMatch] = []
    for line in completed.stdout.splitlines():
        match = pattern.match(line)
        if match:
            matches.append(
                WindowMatch(
                    window_id=match.group(1),
                    title=match.group(2),
                    width=int(match.group(3)),
                    height=int(match.group(4)),
                )
            )
    return matches


def _find_window(window_pattern: re.Pattern[str], deadline: float) -> WindowMatch | None:
    blocked_title_pattern = re.compile(r"selection owner", re.IGNORECASE)
    while time.monotonic() < deadline:
        matches = [
            entry
            for entry in _list_windows()
            if window_pattern.search(entry.title) and not blocked_title_pattern.search(entry.title)
        ]
        matches = [entry for entry in matches if entry.width > 50 and entry.height > 50]
        if matches:
            def _sort_key(entry: WindowMatch) -> tuple[int, int]:
                title = entry.title.strip().lower()
                if title == "rviz":
                    title_rank = 0
                elif title.startswith("rviz"):
                    title_rank = 1
                else:
                    title_rank = 2
                return (title_rank, -(entry.width * entry.height))

            matches.sort(key=_sort_key)
            return matches[0]
        time.sleep(0.5)
    return None


def _extract_mask_shift(mask: int) -> tuple[int, int]:
    if mask == 0:
        return 0, 8
    shift = 0
    while ((mask >> shift) & 1) == 0:
        shift += 1
    width = 0
    while ((mask >> (shift + width)) & 1) == 1:
        width += 1
    return shift, width


def _scale_channel(channel: np.ndarray, width: int) -> np.ndarray:
    if width <= 0 or width >= 8:
        return channel.astype(np.uint8, copy=False)
    max_value = (1 << width) - 1
    return ((channel.astype(np.uint16) * 255) // max_value).astype(np.uint8)


def _decode_xwd_to_bgr(payload: bytes) -> np.ndarray:
    if len(payload) < XWD_HEADER_SIZE:
        raise ValueError("xwd payload too small")

    (
        header_size,
        _file_version,
        _pixmap_format,
        _pixmap_depth,
        pixmap_width,
        pixmap_height,
        _xoffset,
        byte_order,
        _bitmap_unit,
        _bitmap_bit_order,
        _bitmap_pad,
        bits_per_pixel,
        bytes_per_line,
        _visual_class,
        red_mask,
        green_mask,
        blue_mask,
        _bits_per_rgb,
        _colormap_entries,
        ncolors,
        _window_width,
        _window_height,
        _window_x,
        _window_y,
        _window_bdrwidth,
    ) = struct.unpack(XWD_HEADER_FORMAT, payload[:XWD_HEADER_SIZE])

    pixel_offset = header_size + ncolors * XWD_COLOR_SIZE
    pixel_size = bytes_per_line * pixmap_height
    pixel_bytes = payload[pixel_offset : pixel_offset + pixel_size]
    if len(pixel_bytes) < pixel_size:
        raise ValueError("xwd payload truncated")

    rows = np.frombuffer(pixel_bytes, dtype=np.uint8).reshape(pixmap_height, bytes_per_line)

    if bits_per_pixel == 32 and byte_order == 0 and red_mask == 0xFF0000 and green_mask == 0x00FF00 and blue_mask == 0x0000FF:
        return rows[:, : pixmap_width * 4].reshape(pixmap_height, pixmap_width, 4)[:, :, :3].copy()

    if bits_per_pixel == 24 and byte_order == 0 and red_mask == 0xFF0000 and green_mask == 0x00FF00 and blue_mask == 0x0000FF:
        return rows[:, : pixmap_width * 3].reshape(pixmap_height, pixmap_width, 3).copy()

    if bits_per_pixel not in (24, 32):
        raise ValueError(f"unsupported bits_per_pixel={bits_per_pixel}")

    if bits_per_pixel == 32:
        dtype = "<u4" if byte_order == 0 else ">u4"
        raw = np.frombuffer(pixel_bytes, dtype=np.dtype(dtype)).reshape(pixmap_height, bytes_per_line // 4)[:, :pixmap_width]
    else:
        padded = rows[:, : pixmap_width * 3]
        if byte_order == 0:
            raw = (
                padded[:, 0::3].astype(np.uint32)
                | (padded[:, 1::3].astype(np.uint32) << 8)
                | (padded[:, 2::3].astype(np.uint32) << 16)
            )
        else:
            raw = (
                (padded[:, 0::3].astype(np.uint32) << 16)
                | (padded[:, 1::3].astype(np.uint32) << 8)
                | padded[:, 2::3].astype(np.uint32)
            )

    r_shift, r_width = _extract_mask_shift(red_mask)
    g_shift, g_width = _extract_mask_shift(green_mask)
    b_shift, b_width = _extract_mask_shift(blue_mask)

    red = _scale_channel((raw & red_mask) >> r_shift, r_width)
    green = _scale_channel((raw & green_mask) >> g_shift, g_width)
    blue = _scale_channel((raw & blue_mask) >> b_shift, b_width)
    return np.stack([blue, green, red], axis=-1)


def _capture_window(window_id: str) -> np.ndarray:
    completed = subprocess.run(
        ["xwd", "-silent", "-id", window_id],
        check=True,
        capture_output=True,
    )
    return _decode_xwd_to_bgr(completed.stdout)


def _resize_nearest(frame: np.ndarray, max_width: int, max_height: int) -> np.ndarray:
    height, width = frame.shape[:2]
    scale = min(1.0, max_width / width if max_width > 0 else 1.0, max_height / height if max_height > 0 else 1.0)
    if scale >= 0.999:
        return frame
    target_width = max(1, int(round(width * scale)))
    target_height = max(1, int(round(height * scale)))
    return _resize_to_shape(frame, target_width, target_height)


def _resize_to_shape(frame: np.ndarray, target_width: int, target_height: int) -> np.ndarray:
    height, width = frame.shape[:2]
    y_indices = np.clip(np.round(np.linspace(0, height - 1, target_height)).astype(np.int32), 0, height - 1)
    x_indices = np.clip(np.round(np.linspace(0, width - 1, target_width)).astype(np.int32), 0, width - 1)
    return frame[y_indices][:, x_indices]


class AviWriter:
    def __init__(self, output_path: Path, width: int, height: int, fps: float) -> None:
        self.output_path = output_path
        self.width = width
        self.height = height
        self.fps = max(0.1, fps)
        self.frame_stride = ((self.width * 3 + 3) // 4) * 4
        self.frame_size = self.frame_stride * self.height
        self.frame_count = 0
        self.index_entries: list[tuple[int, int]] = []
        self.file = output_path.open("wb")

        self.file.write(b"RIFF")
        self.file.write(struct.pack("<I", 0))
        self.file.write(b"AVI ")

        self.hdrl_list_start = self.file.tell()
        self.file.write(b"LIST")
        self.file.write(struct.pack("<I", 0))
        self.file.write(b"hdrl")

        self.file.write(b"avih")
        self.file.write(struct.pack("<I", 56))
        self.avih_offset = self.file.tell()
        self.file.write(self._pack_avih(total_frames=0))

        self.strl_list_start = self.file.tell()
        self.file.write(b"LIST")
        self.file.write(struct.pack("<I", 0))
        self.file.write(b"strl")

        self.file.write(b"strh")
        self.file.write(struct.pack("<I", 56))
        self.strh_offset = self.file.tell()
        self.file.write(self._pack_strh(total_frames=0))

        self.file.write(b"strf")
        self.file.write(struct.pack("<I", 40))
        self.file.write(self._pack_strf())

        end_of_header = self.file.tell()
        self._patch_list_size(self.strl_list_start, end_of_header)
        self._patch_list_size(self.hdrl_list_start, end_of_header)

        self.movi_list_start = self.file.tell()
        self.file.write(b"LIST")
        self.file.write(struct.pack("<I", 0))
        self.file.write(b"movi")
        self.movi_data_start = self.file.tell()

    def _pack_avih(self, total_frames: int) -> bytes:
        return struct.pack(
            "<IIIIIIIIII4I",
            int(round(1_000_000.0 / self.fps)),
            int(round(self.frame_size * self.fps)),
            0,
            0x10,
            total_frames,
            0,
            1,
            self.frame_size,
            self.width,
            self.height,
            0,
            0,
            0,
            0,
        )

    def _pack_strh(self, total_frames: int) -> bytes:
        return struct.pack(
            "<4s4sIHHIIIIIIIIhhhh",
            b"vids",
            b"DIB ",
            0,
            0,
            0,
            0,
            1,
            int(round(self.fps)),
            0,
            total_frames,
            self.frame_size,
            0xFFFFFFFF,
            0,
            0,
            0,
            self.width,
            self.height,
        )

    def _pack_strf(self) -> bytes:
        return struct.pack(
            "<IIIHHIIIIII",
            40,
            self.width,
            self.height,
            1,
            24,
            0,
            self.frame_size,
            0,
            0,
            0,
            0,
        )

    def _patch_u32(self, offset: int, value: int) -> None:
        current = self.file.tell()
        self.file.seek(offset)
        self.file.write(struct.pack("<I", value))
        self.file.seek(current)

    def _patch_list_size(self, list_start: int, list_end: int) -> None:
        self._patch_u32(list_start + 4, list_end - list_start - 8)

    def write_frame(self, frame: np.ndarray) -> None:
        if frame.shape[:2] != (self.height, self.width):
            raise ValueError("AVI frame dimensions changed mid-recording")
        padded = np.zeros((self.height, self.frame_stride), dtype=np.uint8)
        flipped = frame[::-1]
        padded[:, : self.width * 3] = flipped.reshape(self.height, self.width * 3)
        payload = padded.tobytes()
        chunk_start = self.file.tell()
        self.file.write(b"00db")
        self.file.write(struct.pack("<I", len(payload)))
        self.file.write(payload)
        if len(payload) % 2 == 1:
            self.file.write(b"\0")
        self.index_entries.append((chunk_start, len(payload)))
        self.frame_count += 1

    def close(self) -> None:
        movi_end = self.file.tell()
        self._patch_list_size(self.movi_list_start, movi_end)

        self.file.write(b"idx1")
        self.file.write(struct.pack("<I", len(self.index_entries) * 16))
        for chunk_start, payload_size in self.index_entries:
            offset = chunk_start - (self.movi_list_start + 8)
            self.file.write(b"00db")
            self.file.write(struct.pack("<III", 0x10, offset, payload_size))

        file_end = self.file.tell()
        self._patch_u32(4, file_end - 8)
        self.file.seek(self.avih_offset)
        self.file.write(self._pack_avih(self.frame_count))
        self.file.seek(self.strh_offset)
        self.file.write(self._pack_strh(self.frame_count))
        self.file.close()


def _write_metadata(metadata_path: Path, payload: dict) -> None:
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def main() -> int:
    args = _parse_args()
    output_path = Path(args.output)
    metadata_path = Path(args.metadata_output)
    stop_path = Path(args.stop_file) if args.stop_file else None

    output_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.parent.mkdir(parents=True, exist_ok=True)

    metadata = {
        "success": False,
        "reason": "not_started",
        "output": str(output_path),
        "window_pattern": args.window_pattern,
        "fps": args.fps,
        "max_width": args.max_width,
        "max_height": args.max_height,
        "wait_timeout": args.wait_timeout,
        "capture_timeout": args.capture_timeout,
        "frames": 0,
    }

    pattern = re.compile(args.window_pattern, re.IGNORECASE)
    deadline = time.monotonic() + max(0.1, args.wait_timeout)
    match = _find_window(pattern, deadline)
    if match is None:
        metadata["reason"] = "window_not_found"
        _write_metadata(metadata_path, metadata)
        return 1

    writer: AviWriter | None = None
    capture_started_at = time.monotonic()
    next_frame_at = capture_started_at
    frame_interval = max(0.05, 1.0 / max(0.1, args.fps))
    consecutive_capture_failures = 0

    metadata["window_id"] = match.window_id
    metadata["window_title"] = match.title
    metadata["reason"] = "recording"

    try:
        while True:
            now = time.monotonic()
            if stop_path is not None and stop_path.exists():
                metadata["reason"] = "stop_file"
                break
            if now - capture_started_at > args.capture_timeout:
                metadata["reason"] = "capture_timeout"
                break
            if now < next_frame_at:
                time.sleep(min(0.05, next_frame_at - now))
                continue
            try:
                frame = _capture_window(match.window_id)
                consecutive_capture_failures = 0
            except Exception as exc:  # noqa: BLE001
                consecutive_capture_failures += 1
                metadata["last_capture_error"] = str(exc)
                replacement = _find_window(pattern, time.monotonic() + 1.0)
                if replacement is not None:
                    match = replacement
                    metadata["window_id"] = match.window_id
                    metadata["window_title"] = match.title
                    consecutive_capture_failures = 0
                    next_frame_at = time.monotonic() + 0.1
                    continue
                if consecutive_capture_failures >= 5:
                    metadata["reason"] = "capture_failed"
                    break
                next_frame_at = time.monotonic() + frame_interval
                continue

            frame = _resize_nearest(frame, args.max_width, args.max_height)
            if writer is None:
                writer = AviWriter(output_path=output_path, width=frame.shape[1], height=frame.shape[0], fps=args.fps)
                metadata["video_width"] = int(frame.shape[1])
                metadata["video_height"] = int(frame.shape[0])
                metadata["recording_started_at"] = time.time()
            elif frame.shape[1] != writer.width or frame.shape[0] != writer.height:
                frame = _resize_to_shape(frame, writer.width, writer.height)

            writer.write_frame(frame)
            metadata["frames"] = writer.frame_count
            next_frame_at = time.monotonic() + frame_interval
    finally:
        if writer is not None:
            writer.close()

    metadata["success"] = writer is not None and metadata["frames"] > 0
    if metadata["success"]:
        metadata["duration_sec"] = round(metadata["frames"] / max(0.1, args.fps), 3)
        metadata["file_size_bytes"] = output_path.stat().st_size if output_path.exists() else 0
    elif output_path.exists():
        output_path.unlink()
    _write_metadata(metadata_path, metadata)
    return 0 if metadata["success"] else 1


if __name__ == "__main__":
    sys.exit(main())
