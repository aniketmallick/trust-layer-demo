#!/usr/bin/env python
"""Planner fixtures, in the style of anchor/negative_fixtures*.py: a name, what is injected and what must happen, a
function returning (as_expected, detail); kind="control" marks the must-not-fire cases. DEMONSTRATION fixtures: none of
them is session evidence.

    python -m planner.fixtures [name-prefix]      print the table (run from experiments/knife_handover)
    pytest planner                                the same fixtures, one test each (planner/tests/test_planner.py)

The plan-level fixtures run once per IK ("sim" = SO101Kinematics.solve, "null" = null_plan.ik): PLAN.md F6 leaves the
choice to the operator, so both sets of numbers are produced. Every number is re-measured here through anchor/fk.py,
not taken from the planner's own checks.
"""
from __future__ import annotations

import functools
import json
import math
import sys
import time

import numpy as np

from common import paths

paths.use_anchor()
import g3  # noqa: E402
import g3_fault_injection as gfi  # noqa: E402

from planner import handover_planner as hp  # noqa: E402
from planner import ik, path  # noqa: E402
from planner.cfg import BASE_XY_MM  # noqa: E402
from planner.knife import knife_pose  # noqa: E402

REGISTRY: list = []
T_IMPORT = time.time()
SEED = 20261001
N_TARGETS = 50
MAX_TRIES = 600
GRIPPER_PCT = 22.35                        # the null's measured close target: the gripper is held, never planned
MID_CARRY_MM = (245.0, 5.0)                # between cross 6 and the zone: where APPROACH_ZONE is likely frozen
SPEED_LIMIT_HANDOVER_DPS = 15.0            # PLAN.md section 4, monitor 2 in handover segments (9 false-fired here)
SPEED_EVIDENCE_DPS = (9.0, 10.0, 12.0)     # replayed and printed as evidence, not asserted


def fixture(group: str, name: str, what: str, kind: str = "negative"):
    def deco(fn):
        REGISTRY.append({"group": group, "name": name, "what": what, "kind": kind, "fn": fn})
        return fn
    return deco


def radial_deg(xy) -> float:
    return math.degrees(math.atan2(xy[1] - BASE_XY_MM[1], xy[0] - BASE_XY_MM[0]))


@functools.lru_cache(maxsize=None)
def cfg_for(method: str):
    return hp.load_cfg(ik_method=method)


@functools.lru_cache(maxsize=None)
def start_pose(method: str, grip_xy: tuple = MID_CARRY_MM, h_mm: float = 30.0, heading_deg: float | None = None) -> tuple:
    """The knife carried at the null's carry height, handle radially outward (the dawn grasp: 'handle outward')."""
    cfg = cfg_for(method)
    heading = radial_deg(grip_xy) if heading_deg is None else heading_deg
    return tuple(path.pose_at(cfg, path.Way(grip_xy, h_mm, heading), GRIPPER_PCT))


@functools.lru_cache(maxsize=None)
def rig():
    reg = json.loads((paths.ROOT / paths.REG_REL).read_text(encoding="utf-8"))
    rec = json.loads((paths.ROOT / paths.RECORD_REL).read_text(encoding="utf-8"))
    return reg, rec


def full_plan(q_now, palm, cfg) -> tuple:
    """([plans in the order they would run], seconds). Empty when refused: plan_handover, or re-orient then handover."""
    t0 = time.perf_counter()
    p = hp.plan_handover(q_now, palm, cfg)
    if p.ok:
        return [p], time.perf_counter() - t0
    if "re-orient" in p.reason:
        r = hp.plan_reorient(q_now, palm, cfg)
        if r.ok:
            p2 = hp.retarget(r, palm, list(r.targets[-1]), cfg)
            if p2.ok:
                return [r, p2], time.perf_counter() - t0
    return [], time.perf_counter() - t0


@functools.lru_cache(maxsize=None)
def targets(method: str) -> dict:
    """N_TARGETS seeded random palms in the workspace box that get a plan from the mid-carry pose (the refused ones
    are counted, not hidden)."""
    cfg, q0 = cfg_for(method), list(start_pose(method))
    rng = np.random.default_rng(SEED)
    x0, x1, y0, y1 = cfg.box_xy
    got, tries, secs_ok, secs_no = [], 0, [], []
    while len(got) < N_TARGETS and tries < MAX_TRIES:
        tries += 1
        palm = (float(rng.uniform(x0, x1)), float(rng.uniform(y0, y1)))
        plans, s = full_plan(q0, palm, cfg)
        (secs_ok if plans else secs_no).append(s)
        if plans:
            got.append({"palm": palm, "plans": plans})
    return {"cfg": cfg, "q0": q0, "items": got, "tries": tries, "secs_ok": secs_ok, "secs_no": secs_no,
            "via_reorient": sum(1 for g in got if len(g["plans"]) == 2)}


def measure(cfg, q_start, plan, palm) -> dict:
    """One plan's rows, re-read through anchor/fk.py, independent of path.check_rows."""
    palm = np.asarray(palm, float)
    x0, x1, y0, y1 = cfg.box_xy
    m = {"min_handle_tip_palm": 1e9, "min_knife_palm": 1e9, "max_step": 0.0, "outside_box": 0, "outside_limits": 0}
    prev = list(q_start)
    for row in plan.targets:
        m["max_step"] = max(m["max_step"], max(abs(row[i] - prev[i]) for i in range(5)))
        prev = row
        m["outside_limits"] += any(not (cfg.lim_lo[i] <= row[i] <= cfg.lim_hi[i]) for i in range(6))
        kp = knife_pose(cfg.to_mjcf(row), cfg.knife)
        pts = (kp.tip, kp.handle_tip, kp.blade_tip)
        m["outside_box"] += (any(not (x0 <= p[0] <= x1 and y0 <= p[1] <= y1) for p in pts)
                             or not (cfg.z_lo_mm <= kp.tip[2] <= cfg.z_hi_mm))
        m["min_handle_tip_palm"] = min(m["min_handle_tip_palm"], float(np.linalg.norm(kp.handle_tip[:2] - palm)))
        m["min_knife_palm"] = min(m["min_knife_palm"], path.seg_dist(palm, kp.blade_tip, kp.handle_tip))
    return m


def end_errors(cfg, plan, palm) -> tuple:
    """(fingertip error mm vs the planned fingertip, handle heading error deg vs palm - grip point) at the last row."""
    kp = knife_pose(cfg.to_mjcf(plan.targets[-1]), cfg.knife)
    want = path.tip_target(cfg, path.Way(plan.grip_point_mm, cfg.z_handover_mm, plan.heading_deg))
    to_palm = np.asarray(palm, float) - kp.grip[:2]
    aim = math.degrees(math.atan2(to_palm[1], to_palm[0]))
    return float(np.linalg.norm(kp.tip - want)), abs(path.nearest_turn(kp.heading_deg - aim, 0.0))


def replay(method: str, speed_limit_dps: float | None, inject=None, only: int | None = None) -> list:
    """The G3 guard of record over every plan's rows through the fault table's lagging-servo model
    (g3_fault_injection.simulate: tau 40 ms, encoder noise 0.09 deg, loop jitter). -> [(item, plan kind, fired)]."""
    reg, rec = rig()
    t, out = targets(method), []
    for n, item in enumerate(t["items"] if only is None else t["items"][only:only + 1]):
        q = list(t["q0"])
        for p in item["plans"]:
            guard = g3.guard_from_record(paths.ROOT, reg, rec)
            if speed_limit_dps is not None:
                guard.vel_limit_dps = speed_limit_dps
            r = gfi.simulate(guard, [list(x) for x in p.targets], np.random.default_rng(SEED + n), inject=inject, rest=q)
            out.append((n, p.kind, r["fired"]))
            q = list(p.targets[-1])
    return out


def _both(group, name, what, kind="negative"):
    """Register one fixture per IK method."""
    def deco(fn):
        for m in ik.METHODS:
            fixture(group, f"{name}[{m}]", what, kind)(functools.partial(fn, m))
        return fn
    return deco


sys.modules.setdefault("planner.fixtures", sys.modules[__name__])      # run as a script: one registry, not two
from planner import fixtures_plan, fixtures_units  # noqa: E402,F401 - they register themselves


def run(prefix: str = "") -> int:
    bad = 0
    for f in REGISTRY:
        if not f["name"].startswith(prefix):
            continue
        t0 = time.perf_counter()
        ok, detail = f["fn"]()
        bad += not ok
        print(f"  [{'ok ' if ok else 'BAD'}] {f['name']} ({f['kind']}, {time.perf_counter() - t0:.1f} s): {detail}", flush=True)
    print("PLANNER FIXTURES: " + ("ALL AS EXPECTED" if not bad else f"{bad} NOT AS EXPECTED"))
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(run(sys.argv[1] if len(sys.argv) > 1 else ""))
