"""Rehearsal only (as null_session.DryWorld is): what a dry run needs in place of a rig.

  - the simulated hand feed, as a separate process (monitor/sim_feed.py), and the phase cues it times its script by;
  - the script for the judge's stub provider (judge/: provider "stub", named "stub" in every row) - what a judge
    would plausibly say about the synthetic scene, from the runner's own numbers, with --dry-judge overrides
    (CP1=unsafe, FREEZE=malformed, CP3=timeout, ...);
  - where the simulated hand appears: a palm inside the workspace box for which the planner has a plan, also 30 mm
    and 80 mm further along the drift direction, chosen by asking the planner (deterministic).
Nothing here runs on --arm.
"""
from __future__ import annotations

import json
import socket
import subprocess
import sys
from pathlib import Path

from common import paths, wire

SIM_FEED = paths.EXP / "monitor" / "sim_feed.py"
DRIFTS_MM = (0.0, 30.0, 80.0)
DRIFT_DIRS = ((0.0, 1.0), (0.0, -1.0), (1.0, 0.0), (-1.0, 0.0))


class Cues:
    """Phase names to the simulated feed (UDP, local). A lost cue only delays the script."""

    def __init__(self, port: int):
        self.addr = ("127.0.0.1", int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, phase: str) -> None:
        try:
            self.sock.sendto(wire.encode({"cue": phase}), self.addr)
        except OSError:
            pass

    def close(self) -> None:
        self.send("quit")
        self.sock.close()


def spawn_feed(script: str, palm, drift_dir, udp_port: int, cue_port: int, frame_dir: Path, events: list | None = None,
               fingers_dir=None):
    cmd = [sys.executable, "-B", str(SIM_FEED), "--script", script, "--palm", f"{palm[0]:.1f},{palm[1]:.1f}",
           "--drift-dir", f"{drift_dir[0]:g},{drift_dir[1]:g}", "--port", str(udp_port), "--cue-port", str(cue_port),
           "--frame-dir", str(frame_dir)]
    if fingers_dir is not None:
        cmd += ["--fingers-dir", f"{fingers_dir[0]:g},{fingers_dir[1]:g}"]
    for e in events or []:
        cmd += ["--event", e]
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def pick_palm(planner, tm, poses: list, want_blade_toward: bool = False) -> tuple | None:
    """(palm_mm, drift_dir) inside the box, the same answer from every pose in `poses` (the freeze lands on one of
    them, give or take a row): the planner plans for the palm and for the palm 30 and 80 mm along the drift; with
    want_blade_toward, a palm the blade heads toward and plan_reorient has a turn for (the [o] rehearsal)."""
    x0, x1, y0, y1 = tm.box_xy
    for x in range(int(x1) - 15, int(x0) + 10, -15):
        for y in range(int(y1) - 15, int(y0) + 10, -15):
            for d in DRIFT_DIRS:
                pts = [(x + k * d[0], y + k * d[1]) for k in DRIFTS_MM]
                if not all(tm.in_box(p) for p in pts):
                    continue
                toward = [planner.blade_toward_palm(q, pts[0]) for q in poses]
                if want_blade_toward:
                    if all(toward) and all(planner.plan_reorient(q, pts[0]).ok for q in poses):
                        return pts[0], d
                    break
                if not any(toward) and all(planner.plan_handover(q, p).ok for q in poses[:2] for p in pts):
                    return pts[0], d
    return None


class JudgeScript:
    """The stub provider's script: callable (phase, user_text, image) -> reply. `scene` is what the dry run knows."""

    def __init__(self, overrides: dict | None = None):
        self.scene = {"n_hands": 0, "blade_toward": None}
        self.overrides = dict(overrides or {})
        self.calls: list = []

    def __call__(self, phase: str, user_text: str, image):
        self.calls.append(phase)
        how = self.overrides.get(phase)
        if how == "malformed":
            return "I think the scene looks fine."
        if how == "timeout":
            return {"text": "{}", "delay_s": 9.0}
        n, blade = int(self.scene.get("n_hands") or 0), self.scene.get("blade_toward")
        v = {"hand_present": n > 0, "hand_open_waiting": n == 1, "blade_toward_hand": blade if n else None}
        if how == "unsafe":
            v.update(p_unsafe=0.9, recommend="abort", reason="dry-run stub: scripted UNSAFE verdict")
        elif phase in ("CP1", "CP2"):
            v.update(p_unsafe=0.8 if n else 0.03, recommend="wait" if n else "proceed",
                     reason="dry-run stub: " + ("a hand is in the synthetic frame" if n else "no hand in the synthetic frame"))
        elif phase == "CP3":
            v.update(p_unsafe=0.1, recommend="proceed", reason="dry-run stub: one hand at the handle" if n else
                     "dry-run stub: no hand at the handle")
        else:
            rec = "proceed" if n == 0 else ("wait" if n > 1 else ("reorient" if blade else "handover"))
            v.update(p_unsafe={0: 0.05, 1: 0.3}.get(n, 0.7), recommend=rec,
                     reason=f"dry-run stub: {n} hand(s) in the synthetic frame")
        return json.dumps(v)


def parse_overrides(s: str | None) -> dict:
    """'CP1=unsafe,FREEZE=malformed' -> {phase: how}."""
    out = {}
    for part in (s or "").split(","):
        if "=" in part:
            k, _, v = part.partition("=")
            out[k.strip().upper()] = v.strip().lower()
    return out
