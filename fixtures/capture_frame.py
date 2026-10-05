#!/usr/bin/env python
"""Capture fixture stills from the overhead camera. Opens the camera and nothing else: no serial port, no motor bus.

    ~/lerobot-mps-venv/bin/python fixtures/capture_frame.py --name empty_table [--camera 0] [--n 3] [--note "..."]

Same mode as the null runner's camera (640x480, index 0 = the registration's C920). 15 warm-up frames are dropped,
then --n stills 0.4 s apart go to fixtures/frames/<name>_<k>.png (lossless), one line each in manifest.jsonl with the
sha256. An existing file is never overwritten.
"""
from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import sys
import time
from pathlib import Path

FRAMES = Path(__file__).resolve().parent / "frames"
IST = datetime.timezone(datetime.timedelta(hours=5, minutes=30))
WARMUP = 15


def main(argv: list | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", required=True, help="scene name, e.g. empty_table")
    ap.add_argument("--camera", default="0")
    ap.add_argument("--n", type=int, default=3)
    ap.add_argument("--note", default="")
    a = ap.parse_args(argv)
    import cv2
    FRAMES.mkdir(parents=True, exist_ok=True)
    taken = sorted(FRAMES.glob(f"{a.name}_*.png"))
    if taken:
        print(f"REFUSED: {a.name} already has {len(taken)} still(s) ({taken[0].name} ...) - use another name")
        return 2
    cap = cv2.VideoCapture(int(a.camera) if a.camera.lstrip("-").isdigit() else a.camera)
    if not cap.isOpened():
        print(f"camera {a.camera} could not be opened")
        return 1
    try:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        props = {"width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                 "fps": float(cap.get(cv2.CAP_PROP_FPS))}
        for _ in range(WARMUP):
            cap.read()
        for k in range(1, a.n + 1):
            ok, img = cap.read()
            if not ok:
                print("camera read failed")
                return 1
            p = FRAMES / f"{a.name}_{k}.png"
            cv2.imwrite(str(p), img)
            row = {"name": a.name, "file": p.name, "sha256": hashlib.sha256(p.read_bytes()).hexdigest(),
                   "t_iso": datetime.datetime.now(IST).isoformat(timespec="seconds"), "camera": a.camera,
                   "props": props, "shape": list(img.shape), "note": a.note}
            with open(FRAMES / "manifest.jsonl", "a", encoding="utf-8") as f:
                f.write(json.dumps(row) + "\n")
            print(f"{p.name}  {row['shape']}  sha256 {row['sha256'][:16]}")
            time.sleep(0.4)
    finally:
        cap.release()
    return 0


if __name__ == "__main__":
    sys.exit(main())
