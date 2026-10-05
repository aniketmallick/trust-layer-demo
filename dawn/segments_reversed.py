#!/usr/bin/env python
"""The scripted segments for the screwdriver laid REVERSED where it lies (the operator, 2026-10-04 04:1x: "accommodate
it within the current space" - the pointed end toward the zone, not into it).

    python dawn/segments_reversed.py --lay dawn/lay_measured_20261004.json   (the camera measurement of 2026-10-04 04:14)
        -> dawn/segments_reversed.json; the runner: --segments dawn/segments_reversed.json --knife-axis-sign -1
           --knife-handle-mm H --knife-blade-mm B (printed: the tool measured from the grasp point)

The grasp point and the handle's heading are the camera's (lay_measured.json: the red handle's joint with the metal,
the grasp 12 mm into the red). In the planner's model the reversed tool (axis -1, the handle heading ~ -90) gives the
same gripper pose as the normal lay (axis +1, +90): only the spot moves. Every row through dawn/segments.py's own
check - joint limits, steps <= 3 deg/s / 3 %/s, the fingertip in the box and the camera's footprint, the fingertip and
every body above their floors - EXCEPT that the screwdriver's ends may leave the box (the operator, 2026-10-03: "it
can go outside"); where they go is printed. A row that fails -> nothing written.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import segments as S  # noqa: E402  (dawn/segments.py: the same rows, checks and constants)

from planner import handover_planner as hp  # noqa: E402
from planner import path  # noqa: E402
from planner.cfg import load_cfg  # noqa: E402
from planner.knife import Knife, knife_pose  # noqa: E402

OUT = S.EXP / "dawn" / "segments_reversed.json"
TOTAL_MM = 167.0                     # dawn/knife.json: the screwdriver end to end


def build(lay: dict, close_pct: float, z_handover: float) -> dict:
    grip = (float(lay["grip_xy_mm"][0]), float(lay["grip_xy_mm"][1]))
    heading = float(lay["handle_heading_deg"])
    free = lay["free_end_xy_mm"]
    handle = round(math.hypot(grip[0] - free[0], grip[1] - free[1]), 1)
    blade = round(TOTAL_MM - handle, 1)
    tool = Knife(handle_len_mm=handle, blade_overhang_mm=blade, handle_width_mm=21.0, axis_sign=-1, placeholder=False)
    cfg = hp.with_dawn(load_cfg(), z_handover, knife=tool)
    reg = json.loads((S.paths.ROOT / S.paths.REG_REL).read_text())
    rec = json.loads((S.paths.ROOT / S.paths.RECORD_REL).read_text())
    guard = S.g3.guard_from_record(S.paths.ROOT, reg, rec)
    plan = json.loads((S.paths.ROOT / S.paths.NULL_PLAN_REL).read_text())
    rest = [float(v) for v in plan["geometry"]["rest_lerobot_deg"]]
    th = math.radians(heading)
    jaw = (-math.sin(th), math.cos(th))                                   # axis -1: the jaw axis (path.tip_target)
    descent = (grip[0] - S.SHIFT_MM * jaw[0], grip[1] - S.SHIFT_MM * jaw[1])
    W = {"HOVER": path.Way(descent, S.H_UP, heading), "GRASP": path.Way(descent, S.H_LOW, heading),
         "LIFT": path.Way(descent, S.H_UP, heading), "CARRY": path.Way(S.ZONE_XY, S.H_UP, heading),
         "PLACE": path.Way(S.ZONE_XY, S.H_LOW, heading), "UP": path.Way(S.ZONE_XY, S.H_UP, heading)}
    q_hover = path.pose_at(cfg, W["HOVER"], S.OPEN_PCT)
    seg = {"GRASP": S.joint_rows(list(rest), q_hover)}
    seg["GRASP"] += S.line(cfg, seg["GRASP"][-1], W["HOVER"], W["GRASP"], "descend")
    seg["GRASP"] += S.grip_rows(seg["GRASP"][-1], close_pct)
    seg["GRASP"] += [list(seg["GRASP"][-1])] * 5
    seg["LIFT"] = S.line(cfg, seg["GRASP"][-1], W["GRASP"], W["LIFT"], "lift")
    seg["APPROACH_ZONE"] = S.line(cfg, seg["LIFT"][-1], W["LIFT"], W["CARRY"], "carry")
    seg["PRE_PLACE"] = [list(seg["APPROACH_ZONE"][-1])] * 5
    seg["PLACE"] = S.line(cfg, seg["PRE_PLACE"][-1], W["CARRY"], W["PLACE"], "lower")
    seg["PLACE"] += S.grip_rows(seg["PLACE"][-1], S.OPEN_PCT)
    seg["RETREAT"] = S.line(cfg, seg["PLACE"][-1], W["PLACE"], W["UP"], "rise")
    seg["RETREAT"] += S.joint_rows(seg["RETREAT"][-1], list(rest)) + [list(rest)] * 5
    g_closed = seg["GRASP"][-1]
    up20 = path.Way(descent, S.H_LOW + 20.0, heading)
    hold = {"lift20": S.line(cfg, g_closed, W["GRASP"], up20, "lift 20 mm")}
    hold["lower20"] = S.line(cfg, hold["lift20"][-1], up20, W["GRASP"], "lower 20 mm")
    rel = S.grip_rows(hold["lower20"][-1], S.OPEN_PCT)
    rel += S.line(cfg, rel[-1], W["GRASP"], path.Way(descent, S.H_UP, heading), "rise")
    rel += S.joint_rows(rel[-1], list(rest)) + [list(rest)] * 5
    hold["release_and_home"] = rel
    order = ["GRASP", "LIFT", "APPROACH_ZONE", "PRE_PLACE", "PLACE", "RETREAT"]
    checks, ends_out, prev = {}, {}, list(rest)
    for st in order:                                  # holding=False: the fingertip in the box; the tool ends may leave it
        checks[st] = S.check(cfg, guard, seg[st], prev, low_ok=st in ("GRASP", "LIFT", "PLACE", "RETREAT"), holding=False)
        if st in ("LIFT", "APPROACH_ZONE", "PRE_PLACE", "PLACE"):
            far = [knife_pose(cfg.to_mjcf(r), cfg.knife).blade_tip for r in seg[st]]
            ends_out[st] = sum(1 for p in far if not path._in_box(cfg, p, 0.0))
        prev = seg[st][-1]
    hold_checks = {k: S.check(cfg, guard, hold[k], s, True, False)
                   for k, s in (("lift20", g_closed), ("lower20", hold["lift20"][-1]), ("release_and_home", hold["lower20"][-1]))}
    body = {"source": "dawn/segments_reversed.py, 2026-10-04 - the screwdriver reversed where it lies (camera-measured)",
            "tool": S.dataclasses_asdict(tool), "lay_measured": lay, "laid_at_mm": list(grip),
            "descent_grip_xy_mm": [round(v, 2) for v in descent], "jaw_clearance_each_side_mm": round(S.SHIFT_MM, 1),
            "waypoints": {k: {"grip_xy": list(v.grip_xy), "h_mm": v.h_mm, "heading_deg": v.heading_deg} for k, v in W.items()},
            "rest": rest, "open_pct": S.OPEN_PCT, "close_pct": close_pct, "release_pct": S.OPEN_PCT,
            "close_source": S.CLOSE_NOTE.format(close_pct=close_pct),
            "speeds": {"arm_dps": S.ARM_DPS, "gripper_pps": S.GRIP_PPS, "hz": S.HZ},
            "segments": [{"state": st, "rows": [[round(float(v), 4) for v in r] for r in seg[st]]} for st in order],
            "hold_test": {k: [[round(float(v), 4) for v in r] for r in rows] for k, rows in hold.items()},
            "hold_test_checks": hold_checks, "checks": checks, "pointed_end_rows_outside_the_box": ends_out,
            "ok": not any(checks.values()) and not any(hold_checks.values()), "placeholder": False,
            "label": "the screwdriver (plastic prop) reversed where it lies: pointed end toward the zone, <= 3 deg/s"}
    body["sha256"] = hashlib.sha256(json.dumps({k: v for k, v in body.items() if k != "sha256"}, sort_keys=True,
                                               separators=(",", ":")).encode()).hexdigest()
    return body


def main(argv: list | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lay", required=True, help="lay_measured.json (the camera's measurement)")
    ap.add_argument("--close-pct", type=float, default=9.0)   # 2026-10-05: the operator, from 9.4 %
    ap.add_argument("--z-handover-mm", type=float, default=55.0)
    a = ap.parse_args(argv)
    lay = json.loads(Path(a.lay).read_text())
    b = build(lay, a.close_pct, a.z_handover_mm)
    for s in b["segments"]:
        probs = b["checks"][s["state"]]
        out = b["pointed_end_rows_outside_the_box"].get(s["state"])
        print(f"  {s['state']:14s} {len(s['rows']):4d} rows ({len(s['rows']) / S.HZ:5.1f} s)  "
              + ("ok" if not probs else "PROBLEMS: " + "; ".join(probs))
              + (f"  (the pointed end outside the box on {out} rows)" if out else ""))
    for k, probs in b["hold_test_checks"].items():
        print(f"  hold: {k:16s} {len(b['hold_test'][k]):4d} rows  " + ("ok" if not probs else "PROBLEMS: " + "; ".join(probs)))
    if not b["ok"]:
        print("SEGMENTS NOT WRITTEN")
        return 1
    OUT.write_text(json.dumps(b, indent=1) + "\n")
    t = b["tool"]
    print(f"SEGMENTS WRITTEN -> {OUT.relative_to(S.EXP)}  sha256 {b['sha256'][:16]}")
    print(f"runner flags: --segments dawn/segments_reversed.json --knife-axis-sign -1 --knife-handle-mm "
          f"{t['handle_len_mm']:g} --knife-blade-mm {t['blade_overhang_mm']:g} --knife-width-mm 21")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
