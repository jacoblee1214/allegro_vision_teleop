#!/usr/bin/env python3
"""
detect_camera.py — pick the camera the teleop UI should open.

Prints `<name>:<index>` on stdout (`realsense:<index>` for an Intel RealSense RGB
stream) and a short report on stderr. Exits non-zero when nothing usable is found.

Why this is not just "take /dev/video0": this laptop exposes four V4L2 nodes for one
physical camera module —

    video0  HP True Vision FHD   MJPG, YUYV   <- the RGB stream
    video1  HP True Vision FHD   (none)       <- metadata node, cannot capture
    video2  HP True Vision IR    GREY         <- Windows Hello IR sensor
    video3  HP True Vision IR    (none)       <- metadata node

The IR sensor captures fine, so a plain "first device that returns a frame" probe
happily selects it whenever video0 is busy (a second UI still running, say) and the
operator gets a flickering greyscale image: it is 8-bit mono and its illuminator
pulses. Devices whose only pixel formats are greyscale are therefore rejected outright.

Usage:
  tools/detect_camera.py            # print the chosen device
  tools/detect_camera.py --list     # report every node and why it was kept or skipped
"""
from __future__ import annotations

import glob
import os
import re
import subprocess
import sys

# Probing the metadata nodes makes OpenCV print "can't open camera by index" and an
# obsensor error for every one of them. Those are expected here, so silence the backend
# chatter before cv2 is imported (the variable is read at import time).
os.environ.setdefault("OPENCV_LOG_LEVEL", "SILENT")
os.environ.setdefault("OPENCV_VIDEOIO_DEBUG", "0")

# Greyscale-only FourCCs. A device offering nothing else is an IR / mono sensor.
GREY_FOURCC = {"GREY", "Y8", "Y800", "Y10", "Y12", "Y16", "Y8I", "Y12I", "Y10B", "Y16 "}


def device_index(path: str) -> int:
    tail = path.replace("/dev/video", "")
    return int(tail) if tail.isdigit() else 999


def device_name(index: int) -> str:
    try:
        with open(f"/sys/class/video4linux/video{index}/name") as fh:
            return fh.read().strip()
    except Exception:
        return "Camera"


def pixel_formats(dev: str) -> list[str]:
    """FourCCs the node offers. Empty list means v4l2-ctl could not tell us."""
    try:
        out = subprocess.check_output(
            ["v4l2-ctl", "-d", dev, "--list-formats"],
            stderr=subprocess.DEVNULL, timeout=1.5,
        ).decode("utf-8", "ignore")
    except Exception:
        return []
    return [m.strip().upper() for m in re.findall(r"\[\d+\]:\s*'([^']+)'", out)]


def is_colour(dev: str) -> bool | None:
    """True/False, or None when the formats could not be read (then just probe it)."""
    fmts = pixel_formats(dev)
    if not fmts:
        return None
    return any(f not in GREY_FOURCC for f in fmts)


def is_realsense(dev: str) -> bool:
    try:
        out = subprocess.check_output(
            ["v4l2-ctl", "-d", dev, "--all"],
            stderr=subprocess.DEVNULL, timeout=1.5,
        ).decode("utf-8", "ignore")
    except Exception:
        return False
    return "RealSense" in out and ("YUYV" in out or "white_balance" in out)


def captures(index: int) -> bool:
    import cv2
    try:
        cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            return False
        ok, frame = cap.read()
        cap.release()
        return bool(ok and frame is not None and frame.size > 0)
    except Exception:
        return False


def main() -> int:
    report = "--list" in sys.argv
    devices = sorted(glob.glob("/dev/video*"), key=device_index)
    if not devices:
        print("no /dev/video* nodes found", file=sys.stderr)
        return 1

    usable: list[tuple[int, str, bool]] = []   # (index, name, realsense)
    for dev in devices:
        idx = device_index(dev)
        name = device_name(idx)
        colour = is_colour(dev)

        if colour is False:
            print(f"  video{idx:<2} {name:<38} skipped: greyscale only "
                  f"({', '.join(pixel_formats(dev))}) — IR sensor", file=sys.stderr)
            continue
        if not captures(idx):
            why = "cannot capture (metadata node, or already in use)"
            print(f"  video{idx:<2} {name:<38} skipped: {why}", file=sys.stderr)
            continue

        rs = is_realsense(dev)
        usable.append((idx, name, rs))
        print(f"  video{idx:<2} {name:<38} usable"
              f"{' (RealSense RGB)' if rs else ''}", file=sys.stderr)
        if not report and rs:
            break

    if not usable:
        print("no usable colour camera found", file=sys.stderr)
        return 1

    if report:
        return 0

    for idx, name, rs in usable:
        if rs:
            print(f"realsense:{idx}")
            return 0
    idx, name, _ = usable[0]
    print(f"{name}:{idx}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
