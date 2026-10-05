#!/usr/bin/env python
"""Simulated hand feed for the dry run: the hand monitor's messages without a camera or MediaPipe (model "sim_feed").

    python monitor/sim_feed.py --palm 300,90 --script handover
    python monitor/sim_feed.py --palm 300,90 --event appear@+3 --event drift30@+6 --event drift80@+9

A separate process, as the real monitor is. It publishes one common/wire.py hand message per tick (10 Hz) to UDP
127.0.0.1:47101 and follows a script of events. An event is `what@when`:
    what   appear | drift<mm> | vanish | second | second_off | silent | resume | seqjump<n>
    when   +<s>            seconds after this process started (the prompt's "at t"), or
           PHASE+<s>       seconds after the runner entered PHASE (the dry-run runner sends its phase names to the cue
           PHASE#2+<s>     port; #n = the n-th time it enters that phase). Cue-timed events make a rehearsal the same
                           run every time, whatever the machine's speed.
`drift<mm>` puts the palm that far from where it APPEARED (not from where it last was), along --drift-dir.
The built-in scripts:
    none         no hand, ever
    handover     appear@APPROACH_ZONE+1.0, drift30@HANDOVER_APPROACH+1.5, drift80@HANDOVER_APPROACH+4.0,
                 vanish@RELEASE+1.0          (the hand stays where the 80 mm drift left it until the release)
    stale        appear nothing; silent@APPROACH_ZONE+1.0       (the heartbeat goes quiet: monitor 8)
    second_hand  appear@APPROACH_ZONE+1.0, second@HANDOVER_APPROACH+1.5
    seqjump      seqjump5@APPROACH_ZONE+1.0
    enter_leave  appear@APPROACH_ZONE+1.0, vanish@ASK+0.2      (a hand comes and goes: [w], then [c])
    reorient     appear@APPROACH_ZONE+1.0, vanish@RELEASE+1.0  (the runner puts this palm on the blade's side: [o])
Frame fields: when the runner's frame pair is on disk the message names that frame (seq, sha256, t), as the real
monitor does; otherwise frame_seq 0. Deterministic: no randomness anywhere.
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))

from common import paths, wire  # noqa: E402
from common.table_map import PALM_H_MM, load_table_map  # noqa: E402

MODEL = "sim_feed"
HZ = 10.0
# --fingers-dir: a synthetic hand's 21 landmarks (MediaPipe order) as (along the fingers, across) mm from the palm
# centre, so a dry run has a wrist -> fingers direction (palm placement, 2026-10-03). Not a model of any real hand.
LANDMARKS_MM = ((-45, 0), (-30, 25), (-15, 38), (0, 48), (12, 55), (25, 22), (55, 24), (75, 25), (92, 26), (28, 0),
                (60, 0), (82, 0), (100, 0), (25, -18), (55, -19), (75, -20), (90, -20), (18, -35), (42, -38),
                (58, -40), (70, -41))
CUE_PORT = 47102
SCRIPTS = {
    "none": [],
    "handover": ["appear@APPROACH_ZONE+1.0", "drift30@HANDOVER_APPROACH+1.5", "drift80@HANDOVER_APPROACH+4.0",
                 "vanish@RELEASE+1.0"],
    "stale": ["silent@APPROACH_ZONE+1.0"],
    "second_hand": ["appear@APPROACH_ZONE+1.0", "second@HANDOVER_APPROACH+1.5"],
    "seqjump": ["seqjump5@APPROACH_ZONE+1.0"],
    "enter_leave": ["appear@APPROACH_ZONE+1.0", "vanish@ASK+0.2"],
    "reorient": ["appear@APPROACH_ZONE+1.0", "vanish@RELEASE+1.0"],
}


def parse_event(s: str) -> dict:
    what, _, when = s.partition("@")
    phase, _, after = when.rpartition("+")
    nth = 1
    if "#" in phase:
        phase, _, n = phase.partition("#")
        nth = int(n)
    return {"what": what.strip(), "phase": phase.strip() or None, "nth": nth, "after_s": float(after), "done": False}


class Feed:
    """The script's state machine; `tick(now)` -> the message to send, or None while silent. No I/O here."""

    def __init__(self, tm, palm_mm, events: list, drift_dir=(0.0, 1.0), second_mm=None, frame_dir: Path | None = None,
                 fingers_dir=None):
        self.tm, self.home = tm, (float(palm_mm[0]), float(palm_mm[1]))
        self.fingers = None
        if fingers_dir is not None:
            m = (fingers_dir[0] ** 2 + fingers_dir[1] ** 2) ** 0.5 or 1.0
            self.fingers = (fingers_dir[0] / m, fingers_dir[1] / m)
        n = (drift_dir[0] ** 2 + drift_dir[1] ** 2) ** 0.5 or 1.0
        self.dir = (drift_dir[0] / n, drift_dir[1] / n)
        self.events = [parse_event(e) for e in events]
        self.second_mm = second_mm
        self.frame_dir = frame_dir
        self.palm, self.second, self.silent = None, False, False
        self.seq, self.window, self.t0 = 0, [], None
        self.cues: dict = {}                      # phase -> [times it was entered]

    def cue(self, phase: str, now: float) -> None:
        self.cues.setdefault(phase, []).append(now)

    def _due(self, e: dict, now: float) -> bool:
        if e["phase"] is None:
            return now - self.t0 >= e["after_s"]
        at = self.cues.get(e["phase"], [])
        return len(at) >= e["nth"] and now - at[e["nth"] - 1] >= e["after_s"]

    def _apply(self, what: str) -> None:
        if what == "appear":
            self.palm = self.home
        elif what.startswith("drift"):
            d = float(what[5:])
            self.palm = (self.home[0] + d * self.dir[0], self.home[1] + d * self.dir[1])
        elif what == "vanish":
            self.palm, self.second = None, False
        elif what == "second":
            self.second = True
        elif what == "second_off":
            self.second = False
        elif what == "silent":
            self.silent = True
        elif what == "resume":
            self.silent = False
        elif what.startswith("seqjump"):
            self.seq += int(what[7:] or 5)
        else:
            raise ValueError(f"unknown event {what!r}")

    def _px(self, p_mm) -> tuple:
        """The pixel at which a point PALM_H_MM above the table at p_mm is seen."""
        c = self.tm.px_to_mm(self.tm.nadir_px)
        at_table = c + (p_mm - c) * self.tm.cam_height_mm / (self.tm.cam_height_mm - PALM_H_MM)
        return self.tm.mm_to_px(at_table)

    def _hand(self, palm_mm, handedness: str) -> dict:
        import numpy as np
        px = self._px(palm_mm)
        h = {"handedness": handedness, "conf": 0.99, "palm_px": [round(px[0], 1), round(px[1], 1)], **self.tm.flags(px)}
        if self.fingers is not None and handedness == "R":
            d = np.array(self.fingers)
            n = np.array([-d[1], d[0]])
            h["landmarks_px"] = [[round(float(v), 1) for v in self._px(palm_mm + a * d + b * n)] for a, b in LANDMARKS_MM]
        return h

    def tick(self, now: float) -> dict | None:
        import numpy as np
        if self.t0 is None:
            self.t0 = now
        for e in self.events:
            if not e["done"] and self._due(e, now):
                self._apply(e["what"])
                e["done"] = True
        if self.silent:
            return None
        hands = []
        if self.palm is not None:
            hands.append(self._hand(np.array(self.palm), "R"))
        if self.second:
            p = self.second_mm or (self.home[0] - 40.0, self.home[1] - 90.0)
            hands.append(self._hand(np.array(p, float), "L"))
        self.window = (self.window + [any(h["in_workspace"] for h in hands)])[-wire.DEBOUNCE_N:]
        meta = {"seq": 0, "sha256": "0" * 64, "t": time.time()}
        if self.frame_dir is not None:
            try:
                m = json.loads((self.frame_dir / "frame.json").read_text(encoding="utf-8"))
                if isinstance(m, dict) and {"seq", "sha256", "t"} <= set(m):
                    meta = m
            except (OSError, ValueError):
                pass
        self.seq += 1
        return wire.build_hand_msg(time.time(), self.seq, meta, hands, self.window, MODEL, 0.0)


def main(argv: list | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--palm", default="300,90", help="x,y arm mm where the hand appears")
    ap.add_argument("--script", default="none", choices=sorted(SCRIPTS))
    ap.add_argument("--event", action="append", default=[], help="what@when (see above); added to the script's")
    ap.add_argument("--drift-dir", default="0,1", help="x,y direction of the drift")
    ap.add_argument("--second", default=None, help="x,y arm mm of the second hand")
    ap.add_argument("--fingers-dir", default=None, help="x,y: the hand's wrist -> fingers direction (adds landmarks)")
    ap.add_argument("--port", type=int, default=paths.UDP_ADDR[1])
    ap.add_argument("--cue-port", type=int, default=CUE_PORT)
    ap.add_argument("--frame-dir", default=str(paths.FRAME_DIR))
    ap.add_argument("--hz", type=float, default=HZ)
    ap.add_argument("--duration", type=float, default=600.0, help="stop after this many seconds")
    a = ap.parse_args(argv)
    xy = lambda s: tuple(float(v) for v in s.split(","))        # noqa: E731
    feed = Feed(load_table_map(), xy(a.palm), SCRIPTS[a.script] + a.event, xy(a.drift_dir),
                xy(a.second) if a.second else None, Path(a.frame_dir), xy(a.fingers_dir) if a.fingers_dir else None)
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cue = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cue.bind(("127.0.0.1", a.cue_port))
    cue.setblocking(False)
    period = 1.0 / a.hz
    t_next = t_start = time.monotonic()
    print(f"sim_feed: {a.script} {a.event} palm {a.palm} -> udp 127.0.0.1:{a.port} (cues on {a.cue_port})", flush=True)
    try:
        while time.monotonic() - t_start < a.duration:
            now = time.monotonic()
            while True:
                try:
                    data = cue.recv(4096)
                except BlockingIOError:
                    break
                m = wire.decode(data)
                if isinstance(m, dict) and m.get("cue") == "quit":
                    return 0
                if isinstance(m, dict) and isinstance(m.get("cue"), str):
                    feed.cue(m["cue"], now)
            msg = feed.tick(now)
            if msg is not None:
                out.sendto(wire.encode(msg), ("127.0.0.1", a.port))
            t_next += period
            time.sleep(max(0.0, t_next - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        out.close()
        cue.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
