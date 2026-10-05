#!/usr/bin/env python
"""Step 3 of the dawn protocol (2026-10-02): the scripted segments for the screwdriver (prop tool), decision (a) -
the free handle toward the zone, the grasp ~45 mm toward the zone of the recorded one, axis sign +1.

    python dawn/segments.py [--close-pct 10.5] [--z-handover-mm 55]   -> dawn/segments.json (rows + checks + sha256)

Waypoints are solved by the planner's own knife-aware IK (planner/path.py: the grip point, the fingertip height, the
handle's heading; the handle horizontal); straight lines between them are subdivided and solved row by row
(path.build_rows), every arm joint at <= 3 deg/s; the gripper moves alone, at <= 3 %/s (the reviewer's ruling 2: every
armed move at <= 3 deg/s until the encoders have been seen at speed). Every row is checked before the file is written:
the guard's joint limits (calibrated range - 3 deg), the fingertip and both ends of the tool inside the workspace box
and the camera's footprint, the fingertip >= 8 mm above the table (3 mm on the grasp and place rows: the operator's
choice, 10:33), every other moving body >= 15 mm, each step <= 0.2 deg / 0.2 %. A row that fails -> nothing written.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

EXP = Path(__file__).resolve().parent.parent
if str(EXP) not in sys.path:
    sys.path.insert(0, str(EXP))
from common import paths  # noqa: E402

paths.use_anchor()
import g3  # noqa: E402
import null_plan  # noqa: E402

from planner import handover_planner as hp  # noqa: E402
from planner import path  # noqa: E402
from planner.cfg import load_cfg  # noqa: E402
from planner.knife import Knife, knife_pose  # noqa: E402

HZ = 15.0
ARM_DPS = 3.0
GRIP_PPS = 3.0
OPEN_PCT = 30.0                   # jaw gap ~43 mm (1.4383 mm per %) around the 21 mm handle
PLACE_XY = (278.0, -41.0)         # decision (a): where the handle's centre line is laid (the recorded grip point
#                                   (278, -86) moved 45 mm toward the zone); the grip mark at y -41
GAP_HALF_MM = OPEN_PCT * null_plan.MM_PER_PCT / 2.0            # the open jaws centred on the handle: 21.6 mm
SHIFT_MM = GAP_HALF_MM - 10.5                                  # the closing jaw pushes the handle this far (~11 mm)
DESCENT_XY = (PLACE_XY[0] - SHIFT_MM, PLACE_XY[1])            # the planner's grip point while the jaws are open
GRIP_XY = DESCENT_XY                                           # ... and where the handle's centre is once closed
ZONE_XY = (246.8, 68.1)           # the registration's zone centre: the grip point when placed
HEADING = 90.0                    # the free handle toward the zone (+y)
H_UP, H_LOW = 60.0, 5.0           # fingertip above the table: carried / at the grasp and the place (operator, 10:33)
TIP_FLOOR_MM, TIP_FLOOR_LOW_MM, BODY_FLOOR_MM = 8.0, 3.0, null_plan.MIN_BODY_MM
OUT = EXP / "dawn" / "segments.json"
CLOSE_NOTE = ("{close_pct:g} %: the jaws stop on the handle at 13.4-13.6 % under power (session KH-S20261002T145204); "
              "there the load grew ~46.6 per % of target below contact (load ~ 661 - 46.6 x target, 56 rows): ~172 at "
              "10.5 %, the 250 overload bar at 8.8 %. The first target, 7.85 % (6 % past contact, for a tighter grasp), "
              "reached 300 and fired monitor 4 three times (14:5x). 2026-10-04: 4 % -> 6 % past contact (8.6 %) for "
              "the rubber handle, which pitched nose-down under re-target (the operator; dawn/knife.json 'changes'); "
              "today's loads at 10.5 % were 136-168, so 8.6 % is expected ~225-257 against the 250 bar. At 8.6 % the "
              "load held at 244 and the gripper servo tripped its own overload protection after ~1.9 s, twice "
              "(KH-S20261004T162508): 9.4 % (~200 in that close ramp). Verified by the hold test.")


def grip_rows(q_from: list, to_pct: float) -> list:
    """The gripper alone, from q_from's value to to_pct, <= GRIP_PPS; the arm holds."""
    n = max(1, math.ceil(abs(to_pct - q_from[5]) / (GRIP_PPS / HZ) - 1e-9))
    return [list(q_from[:5]) + [round(q_from[5] + (to_pct - q_from[5]) * k / n, 4)] for k in range(1, n + 1)]


def joint_rows(q_from: list, q_to: list) -> list:
    caps = [ARM_DPS / HZ] * 5 + [GRIP_PPS / HZ]
    d = [b - a for a, b in zip(q_from, q_to)]
    n = int(max(1, max(math.ceil(abs(x) / c - 1e-9) for x, c in zip(d, caps))))
    return [[round(a + x * k / n, 4) for a, x in zip(q_from, d)] for k in range(1, n + 1)]


def line(cfg, q_from: list, a: path.Way, b: path.Way, label: str) -> list:
    got = path.build_rows(cfg, q_from, [(label, path.subdivide(a, b))])
    if got["problem"]:
        raise ValueError(got["problem"])
    return [list(r[:5]) + [q_from[5]] for r in got["rows"]]


def check(cfg, guard, rows: list, start: list, low_ok: bool, holding: bool) -> list:
    lim, probs, prev = guard.limits(), [], list(start)
    tz = cfg.table_z_mm
    for k, r in enumerate(rows):
        for i, j in enumerate(g3.ALL):
            if not lim[j][0] <= r[i] <= lim[j][1]:
                probs.append(f"row {k + 1}: {j} {r[i]:.2f} outside [{lim[j][0]:.2f}, {lim[j][1]:.2f}]")
        if max(abs(a - b) for a, b in zip(r[:5], prev[:5])) > ARM_DPS / HZ + 1e-6 or abs(r[5] - prev[5]) > GRIP_PPS / HZ + 1e-6:
            probs.append(f"row {k + 1}: a step above {ARM_DPS:g} deg/s or {GRIP_PPS:g} %/s")
        prev = r
        kp = knife_pose(cfg.to_mjcf(r), cfg.knife)
        ends = (("fingertip", kp.tip), ("handle end", kp.handle_tip), ("pointed tip", kp.blade_tip))
        for name, p in (ends if holding else ends[:1]):
            if not path._in_box(cfg, p, 0.0):
                probs.append(f"row {k + 1}: the {name} at ({p[0]:.0f}, {p[1]:.0f}) mm is outside the box or the camera's footprint")
        floor = TIP_FLOOR_LOW_MM if low_ok else TIP_FLOOR_MM
        if kp.tip[2] - tz < floor:
            probs.append(f"row {k + 1}: fingertip {kp.tip[2] - tz:.1f} mm above the table (< {floor:g})")
        pts = null_plan.body_points_mm(cfg.to_mjcf(r), r[5])
        low = min(p[2] - tz for n, p in pts.items() if n != "gripperframe")
        if low < BODY_FLOOR_MM:
            probs.append(f"row {k + 1}: a moving body {low:.0f} mm above the table (< {BODY_FLOOR_MM:g})")
        if len(probs) >= 3:
            break
    return probs


def build(close_pct: float, z_handover: float) -> dict:
    tool = Knife(handle_len_mm=59.5, blade_overhang_mm=107.5, handle_width_mm=21.0, axis_sign=1, placeholder=False)
    cfg = hp.with_dawn(load_cfg(), z_handover, knife=tool)
    reg = json.loads((paths.ROOT / paths.REG_REL).read_text())
    rec = json.loads((paths.ROOT / paths.RECORD_REL).read_text())
    guard = g3.guard_from_record(paths.ROOT, reg, rec)
    plan = json.loads((paths.ROOT / paths.NULL_PLAN_REL).read_text())
    rest = [float(v) for v in plan["geometry"]["rest_lerobot_deg"]]
    W = {"HOVER": path.Way(GRIP_XY, H_UP, HEADING), "GRASP": path.Way(GRIP_XY, H_LOW, HEADING),
         "LIFT": path.Way(GRIP_XY, H_UP, HEADING), "CARRY": path.Way(ZONE_XY, H_UP, HEADING),
         "PLACE": path.Way(ZONE_XY, H_LOW, HEADING), "UP": path.Way(ZONE_XY, H_UP, HEADING)}
    q_hover = path.pose_at(cfg, W["HOVER"], OPEN_PCT)
    seg, cur = {}, list(rest)
    seg["GRASP"] = joint_rows(cur, q_hover)                                   # rest -> over the grip mark, jaws opening
    seg["GRASP"] += line(cfg, seg["GRASP"][-1], W["HOVER"], W["GRASP"], "descend")
    seg["GRASP"] += grip_rows(seg["GRASP"][-1], close_pct)
    seg["GRASP"] += [list(seg["GRASP"][-1])] * 5                              # settle (closed)
    seg["LIFT"] = line(cfg, seg["GRASP"][-1], W["GRASP"], W["LIFT"], "lift")
    seg["APPROACH_ZONE"] = line(cfg, seg["LIFT"][-1], W["LIFT"], W["CARRY"], "carry")
    seg["PRE_PLACE"] = [list(seg["APPROACH_ZONE"][-1])] * 5
    seg["PLACE"] = line(cfg, seg["PRE_PLACE"][-1], W["CARRY"], W["PLACE"], "lower")
    seg["PLACE"] += grip_rows(seg["PLACE"][-1], OPEN_PCT)
    seg["RETREAT"] = line(cfg, seg["PLACE"][-1], W["PLACE"], W["UP"], "rise")
    seg["RETREAT"] += joint_rows(seg["RETREAT"][-1], list(rest[:5]) + [rest[5]]) + [list(rest)] * 5
    # the hold test (step 4): from the closed grasp, up 20 mm, down again; release, up to the hover, home
    g_closed = seg["GRASP"][-1]
    up20 = path.Way(GRIP_XY, H_LOW + 20.0, HEADING)
    hold = {"lift20": line(cfg, g_closed, W["GRASP"], up20, "lift 20 mm")}
    hold["lower20"] = line(cfg, hold["lift20"][-1], up20, W["GRASP"], "lower 20 mm")
    rel = grip_rows(hold["lower20"][-1], OPEN_PCT)
    rel += line(cfg, rel[-1], W["GRASP"], path.Way(GRIP_XY, H_UP, HEADING), "rise")
    rel += joint_rows(rel[-1], list(rest)) + [list(rest)] * 5
    hold["release_and_home"] = rel
    hold_checks = {"lift20": check(cfg, guard, hold["lift20"], g_closed, True, True),
                   "lower20": check(cfg, guard, hold["lower20"], hold["lift20"][-1], True, True),
                   "release_and_home": check(cfg, guard, rel, hold["lower20"][-1], True, False)}
    checks, prev = {}, list(rest)
    order = ["GRASP", "LIFT", "APPROACH_ZONE", "PRE_PLACE", "PLACE", "RETREAT"]
    for st in order:
        checks[st] = check(cfg, guard, seg[st], prev, low_ok=st in ("GRASP", "LIFT", "PLACE", "RETREAT"),
                           holding=st in ("LIFT", "APPROACH_ZONE", "PRE_PLACE", "PLACE"))
        prev = seg[st][-1]
    body = {"source": "dawn/segments.py, 2026-10-02 - decision (a), the screwdriver reversed",
            "tool": dataclasses_asdict(tool), "laid_at_mm": list(PLACE_XY), "descent_grip_xy_mm": list(DESCENT_XY),
            "jaw_clearance_each_side_mm": round(SHIFT_MM, 1), "waypoints": {k: {"grip_xy": list(v.grip_xy), "h_mm": v.h_mm,
                                                                 "heading_deg": v.heading_deg} for k, v in W.items()},
            "rest": rest, "open_pct": OPEN_PCT, "close_pct": close_pct, "release_pct": OPEN_PCT,
            "close_source": CLOSE_NOTE.format(close_pct=close_pct),
            "speeds": {"arm_dps": ARM_DPS, "gripper_pps": GRIP_PPS, "hz": HZ},
            "segments": [{"state": st, "rows": [[round(float(v), 4) for v in r] for r in seg[st]]} for st in order],
            "hold_test": {k: [[round(float(v), 4) for v in r] for r in rows] for k, rows in hold.items()},
            "hold_test_checks": hold_checks,
            "checks": checks, "ok": not any(checks.values()) and not any(hold_checks.values()), "placeholder": False,
            "label": "the screwdriver (prop tool), decision (a): free handle toward the zone, <= 3 deg/s"}
    body["sha256"] = hashlib.sha256(json.dumps({k: v for k, v in body.items() if k != "sha256"}, sort_keys=True,
                                               separators=(",", ":")).encode()).hexdigest()
    return body


def dataclasses_asdict(x) -> dict:
    import dataclasses
    return dataclasses.asdict(x)


def main(argv: list | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--close-pct", type=float, default=9.0)   # 2026-10-05: the operator, from 9.4 %
    ap.add_argument("--z-handover-mm", type=float, default=55.0)
    a = ap.parse_args(argv)
    b = build(a.close_pct, a.z_handover_mm)
    for s in b["segments"]:
        probs = b["checks"][s["state"]]
        print(f"  {s['state']:14s} {len(s['rows']):4d} rows ({len(s['rows']) / HZ:5.1f} s)  "
              + ("ok" if not probs else "PROBLEMS: " + "; ".join(probs)))
    for k, probs in b["hold_test_checks"].items():
        print(f"  hold: {k:16s} {len(b['hold_test'][k]):4d} rows  " + ("ok" if not probs else "PROBLEMS: " + "; ".join(probs)))
    if not b["ok"]:
        print("SEGMENTS NOT WRITTEN")
        return 1
    OUT.write_text(json.dumps(b, indent=1) + "\n")
    print(f"SEGMENTS WRITTEN -> {OUT.relative_to(EXP)}  sha256 {b['sha256'][:16]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
