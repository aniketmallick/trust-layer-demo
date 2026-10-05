#!/usr/bin/env python
"""Hand monitor - a separate process in its own venv (PLAN.md sections 4 and 5). A demonstration component.

    .venv_monitor/bin/python monitor/hand_monitor.py                       # live: /tmp/khv -> UDP 127.0.0.1:47101
    .venv_monitor/bin/python monitor/hand_monitor.py --once STILL.png      # one still -> its message on stdout

It reads the frame pair the runner writes (frame.jpg + frame.json, by rename), and for every NEW frame publishes one
JSON line: per hand the palm centre (mean of landmarks 0, 1, 5, 9, 13, 17), its arm-frame mm through the
registration's own pixel map, the three table flags and the 21 landmarks in pixels (landmarks_px: drawn by the
replay, read by no rule); plus the 3-of-5 debounce over the raw per-frame result.
No new frame -> nothing is published: the runner's monitor 8 sees the silence. A frame that cannot be decoded is
not published either. It never opens a camera, a serial port or the judge's .env.
"""
from __future__ import annotations

import argparse
import collections
import hashlib
import json
import signal
import socket
import sys
import threading
import time
from pathlib import Path

sys.dont_write_bytecode = True
EXP = Path(__file__).resolve().parent.parent
if str(EXP) not in sys.path:
    sys.path.insert(0, str(EXP))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from common import paths, wire  # noqa: E402
from common.table_map import TableMap, load_table_map  # noqa: E402
from monitor.detector import MIN_CONFIDENCE, HandDetector, mediapipe_version  # noqa: E402
from monitor.model_lock import ModelLockError, locked_model  # noqa: E402
from monitor.palm import palm_centre_px  # noqa: E402

POLL_S = 0.002                # how often the frame pair is looked at while waiting for a new one
STATS_EVERY_S = 5.0


def model_name() -> str:
    return f"mediapipe-hands {mediapipe_version()}"


def hands_of(detector: HandDetector, tm: TableMap, bgr: np.ndarray) -> tuple[list, list]:
    """One frame -> (the message's hand entries, each hand's 21 landmarks in px - for a replay overlay)."""
    hands, landmarks = [], []
    for h in detector.detect(bgr):
        u, v = palm_centre_px(h.landmarks_px)
        lm = [[round(x, 1), round(y, 1)] for x, y in h.landmarks_px]
        hands.append({"handedness": h.handedness, "conf": round(h.conf, 3), "palm_px": [round(u, 1), round(v, 1)],
                      **tm.flags((u, v)), "landmarks_px": lm})       # landmarks: for the replay's overlay; no rule reads them
        landmarks.append(lm)
    return hands, landmarks


def message_for_still(path: Path, detector: HandDetector, tm: TableMap) -> tuple[dict, list]:
    """A saved still -> (its message, the landmarks). seq 0; the frame hash is the file's."""
    data = Path(path).read_bytes()
    bgr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError(f"{path} is not an image cv2 can read")
    t0 = time.perf_counter()
    hands, landmarks = hands_of(detector, tm, bgr)
    proc_ms = 1000.0 * (time.perf_counter() - t0)
    meta = {"seq": 0, "sha256": hashlib.sha256(data).hexdigest(), "t": time.time()}
    window = [any(h["in_workspace"] for h in hands)]
    return wire.build_hand_msg(time.time(), 0, meta, hands, window, model_name(), proc_ms), landmarks


class Stats:
    """Rate and processing time of what was published (reported on stderr and by the rate test)."""

    def __init__(self):
        self.t_sent: list = []
        self.proc_ms: list = []
        self.undecodable = 0
        self.send_errors = 0

    def rate_hz(self) -> float | None:
        if len(self.t_sent) < 2:
            return None
        return (len(self.t_sent) - 1) / (self.t_sent[-1] - self.t_sent[0])

    def line(self) -> str:
        p = sorted(self.proc_ms) or [0.0]
        hz = self.rate_hz()
        return (f"published {len(self.t_sent)}; {'-' if hz is None else f'{hz:.1f}'} Hz; proc_ms p50 "
                f"{p[len(p) // 2]:.1f} p95 {p[min(len(p) - 1, int(0.95 * len(p)))]:.1f} max {p[-1]:.1f}; undecodable "
                f"{self.undecodable}; send errors {self.send_errors}")


def run(frames_dir: Path, addr: tuple, detector: HandDetector, tm: TableMap, stop: threading.Event,
        stats: Stats | None = None, report=None) -> int:
    """The loop: one message per new frame seq, nothing otherwise. -> the number of messages published."""
    stats = stats if stats is not None else Stats()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    window: collections.deque = collections.deque(maxlen=wire.DEBOUNCE_N)
    model = model_name()
    last_seq, seq = None, 0
    t_report = time.monotonic()
    try:
        while not stop.is_set():
            got = wire.read_frame(frames_dir)
            if got is None or got[0].get("seq") == last_seq:
                stop.wait(POLL_S)
            else:
                meta, jpg = got
                last_seq = meta.get("seq")
                t0 = time.perf_counter()
                bgr = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if bgr is None or not all(isinstance(meta.get(k), (int, float)) for k in ("seq", "t")):
                    stats.undecodable += 1        # nothing is published for it: silence is what the runner sees
                    continue
                hands, _ = hands_of(detector, tm, bgr)
                window.append(any(h["in_workspace"] for h in hands))
                proc_ms = 1000.0 * (time.perf_counter() - t0)
                seq += 1
                msg = wire.build_hand_msg(time.time(), seq, meta, hands, list(window), model, proc_ms)
                try:
                    sock.sendto(wire.encode(msg), addr)
                    stats.t_sent.append(time.monotonic())
                    stats.proc_ms.append(proc_ms)
                except OSError:
                    stats.send_errors += 1
            if report is not None and time.monotonic() - t_report >= STATS_EVERY_S:
                t_report = time.monotonic()
                report(stats.line())
    finally:
        sock.close()
    return seq


def _addr(s: str) -> tuple:
    host, _, port = s.rpartition(":")
    return (host or "127.0.0.1", int(port))


def main(argv: list | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames-dir", default=str(paths.FRAME_DIR))
    ap.add_argument("--udp", default=f"{paths.UDP_ADDR[0]}:{paths.UDP_ADDR[1]}", help="host:port the runner listens on")
    ap.add_argument("--once", metavar="IMAGE", default=None, help="print the message for one still and exit")
    ap.add_argument("--landmarks", action="store_true",
                    help="with --once: print {'msg': ..., 'landmarks_px': ...} (the 21 points per hand)")
    ap.add_argument("--max-hands", type=int, default=2)
    ap.add_argument("--min-confidence", type=float, default=MIN_CONFIDENCE)
    ap.add_argument("--root", default=None, help="the rig tree (default: this checkout)")
    a = ap.parse_args(argv)
    try:
        model_path, model_sha = locked_model()
    except ModelLockError as e:
        print(f"HAND MONITOR REFUSED: {e}", file=sys.stderr)
        return 2
    try:
        tm = load_table_map(Path(a.root) if a.root else None)
    except (OSError, ValueError, KeyError) as e:
        print(f"HAND MONITOR REFUSED: the table map could not be loaded: {type(e).__name__}: {e}", file=sys.stderr)
        return 2
    detector = HandDetector(model_path, max_hands=a.max_hands, min_confidence=a.min_confidence)
    try:
        if a.once:
            msg, landmarks = message_for_still(Path(a.once), detector, tm)
            print(json.dumps({"msg": msg, "landmarks_px": landmarks} if a.landmarks else msg))
            return 0
        stop = threading.Event()
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: stop.set())
        say = lambda s: print(f"[hand_monitor] {s}", file=sys.stderr, flush=True)  # noqa: E731
        say(f"{model_name()}; model {model_path.name} sha256 {model_sha[:16]}; registration "
            f"{tm.registration_sha256[:12]}; frames {a.frames_dir}; udp {a.udp}; max hands {a.max_hands}")
        n = run(Path(a.frames_dir), _addr(a.udp), detector, tm, stop, report=say)
        say(f"stopped after {n} message(s)")
        return 0
    finally:
        detector.close()


if __name__ == "__main__":
    sys.exit(main())
