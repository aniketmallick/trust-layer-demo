"""Inverse kinematics for the registered fingertip (anchor/fk.py site gripperframe) with a vertical approach.

    solve(tip_xyz_mm, roll_mjcf_deg, seeds, limits, method="sim" | "null") -> (q5 MJCF deg, position error mm, tilt deg)

Two solvers, one signature (PLAN.md F6; the choice is the operator's at M1):
  "sim"   so101_sim's SO101Kinematics.solve, as instructed: damped least squares on the sim model's site `tcp`, which
          is the gripper-body point (0, 0, -0.092) m, on the roll axis. The registration is fitted to gripperframe,
          13.5 mm from it, so the tcp target is the fingertip target minus the tool offset turned by the gripper's
          orientation - which depends on the answer: fixed-point iteration, as null_plan.ik_offset does for its jaw axis.
          The roll is pinned (it moves neither the tcp nor the approach axis).
  "null"  anchor/null_plan.py's ik(): numpy Gauss-Newton on fk.py, the solver the frozen null plan was computed with.
Either way the returned error and tilt are measured through anchor/fk.py, never through the solver's own model.
Angles are MJCF degrees (q_mjcf = q_lerobot - q_zero: the sign map is all +1); limits = (lo5, hi5) in the same units.
Nothing under anchor/ or so101_sim/ is written: both are imported by path.
"""
from __future__ import annotations

import functools
import math

import numpy as np

from common import paths

paths.use_anchor()
import fk  # noqa: E402
import null_plan  # noqa: E402

METHODS = ("sim", "null")
TCP_BODY_M = np.array([0.0, 0.0, -0.092])                       # so101_sim/model.py: site "tcp" in body "gripper"
TOOL_OFFSET_M = np.array(fk._GRIPPER_SITE_POS) - TCP_BODY_M     # gripperframe - tcp, gripper body frame (13.5 mm)
SIM_ROUNDS = 8               # fixed-point rounds on the tool offset
SIM_ITERS = 300              # SO101Kinematics.solve iterations per round
SIM_DAMPING = 0.02
SIM_AXIS_WEIGHT = 0.05
SIM_POS_TOL_M = 2e-5
DONE_MM = 0.02
# 4-joint seeds (MJCF deg): the null plan's own (A9 touches of 2026-09-24): near row, far row, over the zone
SEEDS = ((5.0, 12.0, 27.0, 63.0), (22.0, 59.0, -50.0, 67.0), (-18.0, 14.0, 13.0, 63.0))


def tip_and_tilt(q5) -> tuple:
    """anchor/fk.py: (fingertip mm, tilt of the approach axis from straight down, deg)."""
    r = fk.forward_kinematics([float(v) for v in q5[:5]] + [0.0])
    a = r.frames["gripperframe"][:3, 0]
    return r.ee_pos_m * 1000.0, math.degrees(math.acos(max(-1.0, min(1.0, -float(a[2])))))


@functools.lru_cache(maxsize=1)
def _sim():
    """The sim's kinematics, built once (reads so101_sim/so101_sim/assets; writes nothing)."""
    paths.use_sim()
    from so101_sim import model as sm
    from so101_sim.kinematics import SO101Kinematics
    m = sm.build_model()
    return SO101Kinematics(m[0] if isinstance(m, tuple) else m)


def _solve_sim(tip_mm, roll, seeds, lo, hi) -> tuple:
    kin = _sim()
    lo_r, hi_r = np.radians(np.asarray(lo, float)), np.radians(np.asarray(hi, float))
    lo_r[4] = hi_r[4] = math.radians(roll)                      # the roll is given: pinned by the solver's own clip
    kin.lo, kin.hi = lo_r, hi_r                                 # on our instance; the sim's files are untouched
    tip_m = np.asarray(tip_mm, float) / 1000.0
    best = None
    for sd in seeds:
        q = np.clip(np.radians(list(sd[:4]) + [roll]), lo_r, hi_r)
        for _ in range(SIM_ROUNDS):
            R = fk.forward_kinematics(list(np.degrees(q)) + [0.0]).frames["gripper"][:3, :3]
            q, _e = kin.solve(tip_m - R @ TOOL_OFFSET_M, q_init=q, iters=SIM_ITERS, damping=SIM_DAMPING,
                              pos_tol=SIM_POS_TOL_M, axis_weight=SIM_AXIS_WEIGHT)
            p, tilt = tip_and_tilt(np.degrees(q))
            if float(np.linalg.norm(p - tip_mm)) < DONE_MM and tilt < 0.05:
                break
        got = (np.degrees(q), float(np.linalg.norm(p - np.asarray(tip_mm, float))), tilt)
        if best is None or (round(got[1], 2), got[2]) < (round(best[1], 2), best[2]):
            best = got
        if best[1] < DONE_MM and best[2] < 0.05:
            break
    return best


def _solve_null(tip_mm, roll, seeds, lo, hi) -> tuple:
    """null_plan.ik runs every seed it is given to the end; the caller's own seeds go first, alone, and the null
    plan's seeds are only added when those do not reach the target."""
    own = max(1, len(seeds) - len(SEEDS))
    for sds in (seeds[:own], seeds):
        q4, _pe, _tilt = null_plan.ik(np.asarray(tip_mm, float), [np.array(s[:4], float) for s in sds], float(roll),
                                      np.array(lo[:4], float), np.array(hi[:4], float))
        q5 = np.array(list(q4) + [float(roll)])
        p, tilt = tip_and_tilt(q5)
        pe = float(np.linalg.norm(p - np.asarray(tip_mm, float)))
        if pe < DONE_MM and tilt < 0.05:
            break
    return q5, pe, tilt


def solve(tip_xyz_mm, roll_mjcf_deg: float, seeds=None, limits=None, method: str = "sim") -> tuple:
    """-> (q5 MJCF deg, position error mm, tilt deg), the error and tilt re-measured through anchor/fk.py.
    seeds: 4- or 5-joint MJCF guesses, tried in order (the caller's present pose first); limits: (lo5, hi5) MJCF deg.
    A roll outside its limits is returned as asked - the caller checks limits on the whole path."""
    if method not in METHODS:
        raise ValueError(f"method {method!r} is not one of {METHODS}")
    if limits is None:
        raise ValueError("limits are required: (lo5, hi5) in MJCF degrees")
    lo, hi = [float(v) for v in limits[0][:5]], [float(v) for v in limits[1][:5]]
    sds = [tuple(float(v) for v in s[:4]) for s in (seeds or [])] + list(SEEDS)
    fn = _solve_sim if method == "sim" else _solve_null
    return fn(np.asarray(tip_xyz_mm, float), float(roll_mjcf_deg), sds, lo, hi)
