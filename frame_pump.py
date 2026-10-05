"""The runner's camera thread: camera -> JPEG -> /tmp/khv/frame.jpg + frame.json (common/wire.py), >= 10 Hz.

It keeps what it wrote ({seq: sha256, t} and the bytes of the last frames) so that
  - HandLink.known_frame can tell a message computed on one of OUR frames from anything else (monitor 8), and
  - the frame saved at a freeze / checkpoint is byte-for-byte the frame the firing message names (its sha256).
A failed camera read stops the writing: the monitor goes silent and monitor 8 fires. Nothing here touches the bus.
Dry run: DryCamera draws the table through common/table_map.py (the two printed outlines, and a hand blob where the
simulated feed says a palm is) - rehearsal frames, labelled as such in the picture.
"""
from __future__ import annotations

import threading
import time
from collections import OrderedDict
from pathlib import Path

import numpy as np

from common import wire

PUMP_HZ = 15.0
KEEP = 120                       # frames kept in memory (8 s at 15 Hz)
JPEG_QUALITY = 90


class DryCamera:
    def __init__(self, tm, hands_fn=None, width: int = 640, height: int = 480):
        import cv2
        self.cv2, self.tm, self.hands_fn, self.n = cv2, tm, hands_fn, 0
        base = np.full((height, width, 3), (196, 200, 204), np.uint8)
        for quad in (tm.spawn_quad, tm.zone_quad):
            pts = np.array([tm.mm_to_px(p) for p in quad], np.float32)
            cv2.polylines(base, [np.round(pts).astype(np.int32)], True, (60, 60, 60), 1, cv2.LINE_AA)
        cv2.putText(base, "DRY RUN - synthetic frame (demonstration)", (8, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (90, 90, 90), 1, cv2.LINE_AA)
        self.base = base

    def read(self) -> np.ndarray:
        cv2 = self.cv2
        self.n += 1
        img = self.base.copy()
        for u, v in (self.hands_fn() if self.hands_fn else []):
            cv2.ellipse(img, (int(round(u)), int(round(v))), (38, 46), 0, 0, 360, (150, 175, 215), -1, cv2.LINE_AA)
            cv2.circle(img, (int(round(u)), int(round(v))), 3, (40, 40, 200), -1)
        cv2.putText(img, f"frame {self.n}", (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (90, 90, 90), 1, cv2.LINE_AA)
        return img

    def release(self) -> None:
        pass


class RealCamera:
    """The registration's camera, opened as run_anchor.Session.open_camera opens it (640x480, 30 fps)."""

    def __init__(self, index, width: int = 640, height: int = 480, fps: int = 30):
        import cv2
        idx = int(index) if str(index).lstrip("-").isdigit() else str(index)
        self.cap = cv2.VideoCapture(idx)
        if not self.cap.isOpened():
            raise RuntimeError(f"camera {index} could not be opened")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

    def read(self) -> np.ndarray:
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("camera read failed")
        return frame

    def release(self) -> None:
        self.cap.release()


class FramePump(threading.Thread):
    def __init__(self, camera, frame_dir: Path, hz: float = PUMP_HZ):
        super().__init__(daemon=True, name="frame-pump")
        import cv2
        self.cv2, self.camera, self.frame_dir, self.period = cv2, camera, Path(frame_dir), 1.0 / float(hz)
        self._lock = threading.Lock()
        self._frames: OrderedDict = OrderedDict()            # seq -> (meta, jpg)
        self._stop = threading.Event()
        self.seq = 0
        self.error: str | None = None

    def run(self) -> None:
        t_next = time.monotonic()
        while not self._stop.is_set():
            try:
                img = self.camera.read()
                ok, buf = self.cv2.imencode(".jpg", img, [self.cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if not ok:
                    raise RuntimeError("jpeg encode failed")
                jpg = buf.tobytes()
                self.seq += 1
                meta = {"t": time.time(), "seq": self.seq, "sha256": wire.sha256_bytes(jpg)}
                with self._lock:                             # known before it is on disk: the link may ask at once
                    self._frames[self.seq] = (meta, jpg)
                    while len(self._frames) > KEEP:
                        self._frames.popitem(last=False)
                wire.write_frame(self.frame_dir, jpg, self.seq, meta["t"])
            except Exception as e:  # noqa: BLE001 - a dead camera: stop writing, monitor 8 does the rest
                self.error = f"{type(e).__name__}: {e}"
                return
            t_next += self.period
            time.sleep(max(0.0, t_next - time.monotonic()))

    def stop(self) -> None:
        self._stop.set()
        if self.is_alive():
            self.join(timeout=2.0)
        try:
            self.camera.release()
        except Exception:  # noqa: BLE001
            pass

    def known_frame(self, seq: int) -> dict | None:
        with self._lock:
            got = self._frames.get(int(seq))
        return None if got is None else {"sha256": got[0]["sha256"], "t": got[0]["t"]}

    def latest(self) -> tuple | None:
        with self._lock:
            return next(reversed(self._frames.values())) if self._frames else None

    def save(self, photos: Path, name: str, seq: int | None = None) -> dict:
        """Write one kept frame (the one `seq` names if it is still kept, else the latest) into photos/ as written to
        the monitor. -> {file, sha256, frame_seq, frame_t, exact}."""
        with self._lock:
            got = self._frames.get(int(seq)) if seq is not None else None
            exact = got is not None
            if got is None and self._frames:
                got = next(reversed(self._frames.values()))
        if got is None:
            return {"file": None, "sha256": None, "frame_seq": None, "frame_t": None, "exact": False}
        meta, jpg = got
        photos.mkdir(parents=True, exist_ok=True)
        p = photos / f"{name}_f{meta['seq']:06d}.jpg"
        p.write_bytes(jpg)
        return {"file": f"photos/{p.name}", "sha256": meta["sha256"], "frame_seq": meta["seq"], "frame_t": meta["t"],
                "exact": exact if seq is not None else True}
