#!/usr/bin/env python
"""Handover planner - a deterministic approach to the operator's palm, handle first (PLAN.md section 5). DEMONSTRATION.

    plan_handover(q_now_lerobot6, palm_mm, cfg) -> Plan | Refusal        (.ok, .reason, .targets on both)
    plan_reorient(q_now_lerobot6, palm_mm, cfg) -> Plan | Refusal        the [o] path: turn the handle toward the palm
    retarget(plan, palm_new_mm, q_now_lerobot6, cfg) -> Plan | Refusal   a fresh plan from the present pose
    blade_toward_palm(q_now_lerobot6, palm_mm, cfg) -> bool              the code's rule (the judge's is only logged)

It decides nothing about WHETHER to hand over: the runner asks the operator, then calls this. It moves nothing.
The plan: rise in place to the transit height (65 mm, the null's own height over the sheets) -> move above the
standoff point, the handle turned toward the palm (on the way, after the move or before it: the first that passes
every check) -> straight down to z_handover -> hold. The knife is
held top-down, so it is horizontal when the approach axis is vertical, and its heading is the pan plus the wrist roll.
The handle points along (palm - grip point). The standoff is measured from the HANDLE TIP:
    grip point = palm - (standoff + 2 mm + handle length) * d
d = the unit vector from the pan axis toward the palm first (the arm stays on its own side of the hand), then
+-15 .. +-60 deg about the palm. Every row is rounded to 3 decimals, hashed, and read back through anchor/fk.py
(path.check_rows) before the plan is returned; a plan that fails any check is a Refusal with the reason, and the
runner asks. Every arm joint moves at <= 3 deg/s (0.2 deg per 15 Hz row).
"""
from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass

import numpy as np

from planner import path
from planner.cfg import HandoverCfg, load_cfg, with_dawn  # noqa: F401 - the planner's public surface
from planner.knife import Knife, knife_pose  # noqa: F401

PLANNER_VERSION = "0.1.0"
CANDIDATES_DEG = (0.0, 15.0, -15.0, 30.0, -30.0, 45.0, -45.0, 60.0, -60.0)


@dataclass(frozen=True)
class Refusal:
    reason: str
    tried: tuple = ()                 # ((candidate deg, why), ...)
    kind: str = "handover"
    ok: bool = False
    targets: tuple = ()

    def to_row(self) -> dict:
        return {"ok": False, "plan_kind": self.kind, "reason": self.reason, "tried": [list(t) for t in self.tried],
                "planner_version": PLANNER_VERSION}


@dataclass(frozen=True)
class Plan:
    kind: str                         # "handover" | "reorient"
    targets: tuple                    # rows: lerobot deg x5 + gripper %, one per control step
    phases: tuple                     # ({"phase", "from_step", "n_steps"}, ...) as null_plan's
    palm_mm: tuple
    grip_point_mm: tuple              # where the knife's grip point ends
    handle_tip_mm: tuple
    heading_deg: float                # the handle's heading at the end
    candidate_deg: float
    checks: dict
    plan_sha256: str
    ik_method: str
    z_handover_mm: float
    knife_placeholder: bool
    tried: tuple = ()
    retarget_of: str | None = None
    drift_mm: float | None = None
    ok: bool = True
    reason: str = ""

    def to_row(self) -> dict:
        """The plan without its rows, for the session log (the rows are cited by plan_sha256)."""
        return {"ok": True, "plan_kind": self.kind, "plan_sha256": self.plan_sha256, "n_rows": len(self.targets),
                "phases": list(self.phases), "palm_mm": list(self.palm_mm), "grip_point_mm": list(self.grip_point_mm),
                "handle_tip_mm": list(self.handle_tip_mm), "heading_deg": self.heading_deg,
                "candidate_deg": self.candidate_deg, "checks": self.checks, "ik_method": self.ik_method,
                "z_handover_mm": self.z_handover_mm, "knife_placeholder": self.knife_placeholder,
                "tried": [list(t) for t in self.tried], "retarget_of": self.retarget_of, "drift_mm": self.drift_mm,
                "planner_version": PLANNER_VERSION}


def rows_sha256(rows) -> str:
    return hashlib.sha256(json.dumps([list(r) for r in rows], separators=(",", ":")).encode()).hexdigest()


def _rot(d, deg: float) -> np.ndarray:
    th = math.radians(deg)
    return np.array([d[0] * math.cos(th) - d[1] * math.sin(th), d[0] * math.sin(th) + d[1] * math.cos(th)])


def _in_box(cfg: HandoverCfg, p, margin: float = 0.0) -> bool:
    """The allowed motion region: the workspace box (less margin) AND the camera's footprint (path._in_box)."""
    return path._in_box(cfg, p, margin)


def _arm_refusal(cfg: HandoverCfg) -> Refusal | None:
    """On --arm: the values set at dawn must have been set - z_handover and the prop's measured dimensions."""
    if cfg.arm and not cfg.z_handover_set:
        return Refusal("z_handover_mm has not been set at dawn: on the arm the 30 mm default (inside a hand lying on the "
                       "table) is not used")
    if cfg.arm and cfg.knife.placeholder:
        return Refusal("the prop's dimensions have not been measured: on the arm the placeholder knife (handle 100, "
                       "blade 20, width 20 mm) is not used")
    return None


def _now(cfg: HandoverCfg, q_now) -> tuple:
    """The knife at the present pose, and that pose as the path's first waypoint."""
    kp = knife_pose(cfg.to_mjcf(q_now), cfg.knife)
    return kp, path.Way((float(kp.grip[0]), float(kp.grip[1])), float(kp.tip[2] - cfg.table_z_mm), kp.heading_deg)


def blade_toward_palm(q_now_lerobot6, palm_mm, cfg: HandoverCfg) -> bool:
    """True when the handle's heading is more than 90 deg from (palm - grip point): the blade end leads."""
    kp = knife_pose(cfg.to_mjcf(q_now_lerobot6), cfg.knife)
    return float(kp.handle_xy @ (np.asarray(palm_mm, float)[:2] - kp.grip[:2])) < 0.0


def _finish(cfg, kind, q_now, palm, segments, end: path.Way, cand, tried) -> Plan | str:
    """Solve, hold, check. -> a Plan, or the reason it is not one."""
    early = path.precheck(cfg, segments[1:], palm)             # the rise is where the arm already is
    if early:
        return early
    built = path.build_rows(cfg, q_now, segments)
    if built["problem"]:
        return built["problem"]
    rows, phases = built["rows"], built["phases"]
    path.hold_rows(rows, phases, q_now, cfg.hold_rows)
    checks = path.check_rows(cfg, rows, phases, palm, q_now)
    if checks["problem"]:
        return checks["problem"]
    kp = knife_pose(cfg.to_mjcf(rows[-1]), cfg.knife)
    checks = {**{k: v for k, v in checks.items() if k != "problem"}, "ik": built["ik"],
              "end_handle_tip_to_palm_mm": round(float(np.linalg.norm(kp.handle_tip[:2] - np.asarray(palm)[:2])), 2),
              "end_heading_err_deg": round(abs(path.nearest_turn(kp.heading_deg - end.heading_deg, 0.0)), 3),
              "end_tilt_deg": round(kp.tilt_deg, 3)}
    return Plan(kind=kind, targets=tuple(tuple(r) for r in rows), phases=tuple(phases),
                palm_mm=(round(float(palm[0]), 1), round(float(palm[1]), 1)),
                grip_point_mm=(round(end.grip_xy[0], 1), round(end.grip_xy[1], 1)),
                handle_tip_mm=(round(float(kp.handle_tip[0]), 1), round(float(kp.handle_tip[1]), 1)),
                heading_deg=round(path.nearest_turn(end.heading_deg, 0.0), 2), candidate_deg=cand, checks=checks,
                plan_sha256=rows_sha256(rows), ik_method=cfg.ik_method, z_handover_mm=cfg.z_handover_mm,
                knife_placeholder=cfg.knife.placeholder, tried=tuple(tried))


def _turns(start_heading: float, end_heading: float) -> tuple:
    """The end heading as the short turn from the start, then the long way round (the wrist roll may only allow one)."""
    short = path.nearest_turn(end_heading, start_heading)
    return short, short - 360.0 if short > start_heading else short + 360.0


def _routes(rise: path.Way, above: path.Way) -> list:
    """Three ways from the risen pose to above the standoff point, tried in this order: turning on the way; the move
    first with the heading kept, then the turn in place; the turn in place first, then the move."""
    moved, turned = path.Way(above.grip_xy, rise.h_mm, rise.heading_deg), path.Way(rise.grip_xy, rise.h_mm, above.heading_deg)
    return [[("move above the standoff point, handle turning toward the palm", path.subdivide(rise, above))],
            [("move above the standoff point, heading kept", path.subdivide(rise, moved)),
             ("turn in place, handle toward the palm", path.subdivide(moved, above))],
            [("turn in place, handle toward the palm", path.subdivide(rise, turned)),
             ("move above the standoff point", path.subdivide(turned, above))]]


def _approach(kind: str, q_now, palm, cfg: HandoverCfg, candidates, descend: bool) -> Plan | Refusal:
    """Rise in place -> above the standoff point of the first approach direction that passes every check
    (-> down to z_handover) -> hold."""
    kp, start = _now(cfg, q_now)
    out = [n for n, p in (("fingertip", kp.tip), ("handle tip", kp.handle_tip), ("blade tip", kp.blade_tip))
           if not _in_box(cfg, p)]
    if out:
        p = dict(fingertip=kp.tip, **{"handle tip": kp.handle_tip, "blade tip": kp.blade_tip})[out[0]]
        return Refusal(f"the {out[0]} is outside the workspace box where the arm is now, at ({p[0]:.0f}, {p[1]:.0f}) mm",
                       kind=kind)
    near = path.seg_dist(palm, kp.blade_tip, kp.handle_tip)
    if near < cfg.standoff_mm:
        return Refusal(f"the knife is {near:.0f} mm from the palm where the arm is now (standoff {cfg.standoff_mm:g}): "
                       "no move is planned from inside the standoff", kind=kind)
    rise = path.Way(start.grip_xy, max(start.h_mm, cfg.z_transit_mm), start.heading_deg)
    d0, reach, tried = path.unit(palm - np.asarray(cfg.base_xy)), cfg.standoff_mm + cfg.standoff_margin_mm, []
    for cand in candidates:
        d = _rot(d0, cand)
        grip = palm - (reach + cfg.knife.handle_len_mm) * d
        heading = math.degrees(math.atan2(d[1], d[0]))
        ends = {"fingertip": path.tip_target(cfg, path.Way(tuple(grip), cfg.z_handover_mm, heading)),
                "handle tip": palm - reach * d, "blade tip": grip - cfg.knife.blade_overhang_mm * d}
        out = [n for n, p in ends.items() if not _in_box(cfg, p, cfg.box_inner_mm)]
        if out:
            tried.append((cand, f"the {out[0]} would end outside the workspace box"))
            continue
        why = None
        for turn in _turns(start.heading_deg, heading):
            above = path.Way((float(grip[0]), float(grip[1])), cfg.z_transit_mm, turn)
            end = path.Way(above.grip_xy, cfg.z_handover_mm, turn) if descend else above
            down = [("descend to the handover height", path.subdivide(above, end))] if descend else []
            for route in _routes(rise, above):
                got = _finish(cfg, kind, q_now, palm, [("rise in place to the transit height", [rise])] + route + down,
                              end, cand, tried)
                if isinstance(got, Plan):
                    return got
                why = why or got
        tried.append((cand, why))
    what = "handover pose" if descend else "re-orientation"
    return Refusal(f"no {what} for the palm at ({palm[0]:.0f}, {palm[1]:.0f}) mm: {tried[0][1]} "
                   f"({len(tried)} approach directions tried)", tuple(tried), kind)


def plan_handover(q_now_lerobot6, palm_mm, cfg: HandoverCfg, candidates=CANDIDATES_DEG) -> Plan | Refusal:
    q_now, palm = [float(v) for v in q_now_lerobot6], np.asarray(palm_mm, float)[:2]
    refused = _arm_refusal(cfg)
    if refused is not None:
        return refused
    if not _in_box(cfg, palm):
        return Refusal(f"the palm at ({palm[0]:.0f}, {palm[1]:.0f}) mm is outside the allowed region (the workspace box "
                       "and the camera's footprint)")
    if blade_toward_palm(q_now, palm, cfg):
        return Refusal("the blade end is toward the palm (the handle heads more than 90 deg away from it): "
                       "re-orient first ([o])")
    return _approach("handover", q_now, palm, cfg, candidates, descend=True)


def plan_reorient(q_now_lerobot6, palm_mm, cfg: HandoverCfg, candidates=CANDIDATES_DEG) -> Plan | Refusal:
    """The [o] path: the same rise and the same move as the handover, WITHOUT the blade rule and without the descent -
    it ends at the transit height above the standoff point, the handle toward the palm, and stops. The runner then
    calls retarget(this plan, palm, q_now, cfg) (or plan_handover) for the descent, after asking nothing new: the
    knife's distance to the palm is checked on every row here exactly as there."""
    q_now, palm = [float(v) for v in q_now_lerobot6], np.asarray(palm_mm, float)[:2]
    refused = _arm_refusal(cfg)
    if refused is not None:
        return Refusal(refused.reason, kind="reorient")
    if not _in_box(cfg, palm):
        return Refusal(f"the palm at ({palm[0]:.0f}, {palm[1]:.0f}) mm is outside the allowed region (the workspace box "
                       "and the camera's footprint)", kind="reorient")
    return _approach("reorient", q_now, palm, cfg, candidates, descend=False)


def retarget(plan: Plan, palm_new_mm, q_now_lerobot6, cfg: HandoverCfg) -> Plan | Refusal:
    """The palm moved: a fresh plan from the present pose, the same checks, the approach direction of `plan` tried
    first. The 50 mm drift rule is the runner's (it knows the palm approved at [h]); the drift from `plan` is reported."""
    order = (plan.candidate_deg,) + tuple(c for c in CANDIDATES_DEG if c != plan.candidate_deg)
    got = plan_handover(q_now_lerobot6, palm_new_mm, cfg, candidates=order)
    drift = round(float(np.linalg.norm(np.asarray(palm_new_mm, float)[:2] - np.asarray(plan.palm_mm))), 1)
    if not got.ok:
        return Refusal(f"re-target refused ({drift:.0f} mm from the planned palm): {got.reason}", got.tried)
    return Plan(**{**{f: getattr(got, f) for f in got.__dataclass_fields__}, "retarget_of": plan.plan_sha256,
                   "drift_mm": drift})
