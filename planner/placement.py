"""Palm placement (the operator, 2026-10-03) - the screwdriver (plastic prop) laid in an open palm. DEMONSTRATION.

    plan_placement(q_now_lerobot6, palm_mm, fingers_dir, cfg, finger_reach_mm=None, grip_pct=None) -> Plan | Refusal

The target: the handle over the open palm (its middle over the palm centre), the pointed end past the fingertips,
away from the person (along `fingers_dir`, wrist -> fingers, on the table plane; +-15 .. +-45 deg tried when the box
needs it). The route: rise in place to the turn height -> the turn (on the way, after the move, before it, or at a
translate waypoint chosen inside the box) -> above the release point -> straight down to the release height -> hold.
No standoff: the handle goes over the palm. The screwdriver's ends may leave the workspace box (the operator,
2026-10-03: "it can go outside, it just needs to reorient"); the fingertip stays inside the box and the camera's
footprint (G3 monitor 3 reads it). On every row (beside the generic rules: joint limits, the fingertip in the box,
heights above the table, the screwdriver's ends above the table after the rise):
  - the pointed end >= POINT_CLEAR_MM from the palm, in 3D (the palm PALM_H_MM above the table);
  - over the hand (within HAND_R_MM of the palm on the table plane) nothing - fingertip, the screwdriver's underside -
    lower than the release height.
Release height: the lowest point of the gripper and the screwdriver RELEASE_MM above the palm, commanded SAG_MM higher
(the arm rides 12-20 mm under its commanded pose with the prop; STATUS, day-2 finding).
Speeds: a move with any part within NEAR_MM of the palm (3D) at either end, and the descent: <= 3 deg/s; the rest:
<= 10 deg/s.
"""
from __future__ import annotations

import dataclasses
import math
import time

import numpy as np

from common import paths

paths.use_anchor()
import fk  # noqa: E402
import null_plan  # noqa: E402

from common.table_map import PALM_H_MM  # noqa: E402
from planner import path  # noqa: E402
from planner.cfg import HandoverCfg  # noqa: E402
from planner.handover_planner import Plan, Refusal, _arm_refusal, _rot, rows_sha256  # noqa: E402
from planner.knife import knife_pose  # noqa: E402

POINT_CLEAR_MM = 30.0          # the pointed end <-> the palm, 3D, every row (the operator: a plastic prop)
RELEASE_MM = 20.0              # the lowest point above the palm at the release
SAG_MM = 16.0                  # commanded above that: the droop measured under the prop (CP1 z -15.6..-15.9 mm)
HAND_R_MM = 110.0              # the hand's footprint around the palm centre (fingertips ~100 mm)
TIP_PAST_MM = 10.0             # the pointed end at least this far past the fingertips
FINGER_REACH_MM = 100.0        # palm centre -> fingertips when the landmarks do not say
NEAR_MM = 150.0
NEAR_DPS, FAR_DPS = 3.0, 10.0
DESCENT_MIN_MM = 25.0          # the turn height is at least this far above the release height
DEVIATIONS_DEG = (0.0, 15.0, -15.0, 30.0, -30.0, 45.0, -45.0)
REACH_TOPDOWN_MM = 290.0       # grip point -> pan axis on the table plane: beyond it no top-down pose at release height
PLAN_BUDGET_S = 25.0           # the arm waits, frozen, while this plans: past it, a refusal with what was tried
N_WAYPOINTS = 4                # translate waypoints tried per direction
WAYPOINT_STEP_MM = 40.0        # the translate-waypoint grid
SAMPLES = 7                    # points along the screwdriver for the over-the-hand rule


def palm3(cfg: HandoverCfg, palm_mm) -> np.ndarray:
    return np.array([float(palm_mm[0]), float(palm_mm[1]), cfg.table_z_mm + PALM_H_MM])


def _points(cfg: HandoverCfg, kp) -> list:
    """(name, 3D point, how far below it the part reaches): the fingertip, then points along the screwdriver's axis."""
    r = cfg.knife.handle_width_mm / 2.0
    pts = [("fingertip", np.asarray(kp.tip, float), 0.0)]
    a, b = np.asarray(kp.blade_tip, float), np.asarray(kp.handle_tip, float)
    pts += [("screwdriver", a + (b - a) * k / (SAMPLES - 1), r) for k in range(SAMPLES)]
    return pts


def lowest_mm(cfg: HandoverCfg, kp) -> float:
    return min(float(p[2]) - below for _, p, below in _points(cfg, kp)) - cfg.table_z_mm


def _release_floor(cfg: HandoverCfg) -> float:
    return PALM_H_MM + RELEASE_MM + SAG_MM


def row_problem(cfg: HandoverCfg, row, palm_mm, after_rise: bool) -> str | None:
    """The rules for one row (lerobot 6). None = it passes. Before the rise is done (the arm going straight up from
    where it already is, away from the hand) the over-the-hand height rule is not read."""
    for i in range(5):
        if not (cfg.lim_lo[i] <= row[i] <= cfg.lim_hi[i]):
            return f"{fk.JOINT_NAMES[i]} at {row[i]:.1f} deg outside its limit [{cfg.lim_lo[i]:.1f}, {cfg.lim_hi[i]:.1f}]"
    kp = knife_pose(cfg.to_mjcf(row), cfg.knife)
    if not (path._in_box(cfg, kp.tip, 0.0) and cfg.z_lo_mm <= kp.tip[2] <= cfg.z_hi_mm):
        return f"the fingertip at ({kp.tip[0]:.0f}, {kp.tip[1]:.0f}, {kp.tip[2]:.0f}) mm leaves the workspace box"
    body = min(float(kp.frames[b][2, 3]) * 1000.0 - cfg.table_z_mm for b in null_plan.BODY_POINTS)
    if body < cfg.min_body_mm:
        return f"a moving body comes within {body:.0f} mm of the table (< {cfg.min_body_mm:g})"
    if after_rise:
        end = min(kp.handle_tip[2], kp.blade_tip[2]) - cfg.table_z_mm
        if end < cfg.min_knife_mm:
            return f"an end of the screwdriver comes within {end:.0f} mm of the table (< {cfg.min_knife_mm:g})"
    p3 = palm3(cfg, palm_mm)
    d_point = float(np.linalg.norm(np.asarray(kp.blade_tip, float) - p3))
    if d_point < POINT_CLEAR_MM:
        return f"the pointed end comes within {d_point:.0f} mm of the palm in 3D (< {POINT_CLEAR_MM:g})"
    floor = _release_floor(cfg) - 1.0
    for name, p, below in (_points(cfg, kp) if after_rise else ()):
        if float(np.linalg.norm(p[:2] - p3[:2])) <= HAND_R_MM and float(p[2]) - below - cfg.table_z_mm < floor:
            return (f"over the hand the {name} would be {float(p[2]) - below - cfg.table_z_mm:.0f} mm above the table "
                    f"(the release height is {_release_floor(cfg):g})")
    return None


def _near(cfg: HandoverCfg, kp, palm_mm) -> bool:
    p3 = palm3(cfg, palm_mm)
    return any(float(np.linalg.norm(p - p3)) < NEAR_MM for _, p, _ in _points(cfg, kp))


def _solve(cfg: HandoverCfg, way: path.Way, seed):
    q, pe, tilt = path.solve_way(cfg, way, seed)
    if pe > cfg.max_ik_err_mm or tilt > cfg.max_tilt_deg:
        return None, (f"no pose with the screwdriver horizontal at ({way.grip_xy[0]:.0f}, {way.grip_xy[1]:.0f}) mm, "
                      f"{way.h_mm:.0f} mm up (IK error {pe:.1f} mm, tilt {tilt:.1f} deg)")
    lo, hi = cfg.mjcf_limits()
    bad = [i for i in range(5) if not (lo[i] <= q[i] <= hi[i])]
    if bad:
        return None, f"{fk.JOINT_NAMES[bad[0]]} would leave its limits at ({way.grip_xy[0]:.0f}, {way.grip_xy[1]:.0f}) mm"
    return q, None


def release_h(cfg: HandoverCfg, grip_xy, heading_deg: float, seed) -> tuple:
    """The fingertip height that puts the lowest point at the commanded release floor.
    -> (h, the screwdriver's axis height above the fingertip, None) | (None, None, why)."""
    h = _release_floor(cfg)
    for _ in range(3):
        q, why = _solve(cfg, path.Way(tuple(grip_xy), h, heading_deg), seed)
        if q is None:
            return None, None, why
        kp = knife_pose(list(q), cfg.knife)
        err = _release_floor(cfg) - lowest_mm(cfg, kp)
        if abs(err) < 0.5:
            break
        h += err
    return h, float(kp.grip[2] - kp.tip[2]), None


def precheck(cfg: HandoverCfg, segments: list, palm_mm, axis_dz: float, ends_in_box: bool = False) -> str | None:
    """The waypoints' own geometry before any IK (the screwdriver horizontal, its axis axis_dz above the fingertip):
    the fingertip inside the box, the pointed end clear of the palm, nothing under the release height over the hand."""
    p3, r, floor = palm3(cfg, palm_mm), cfg.knife.handle_width_mm / 2.0, _release_floor(cfg) - 1.0
    for label, ways in segments[1:]:
        for w in ways:
            th = math.radians(w.heading_deg)
            d, g = np.array([math.cos(th), math.sin(th)]), np.asarray(w.grip_xy, float)
            z = cfg.table_z_mm + w.h_mm + axis_dz
            tip = path.tip_target(cfg, w)
            handle, blade = np.append(g + cfg.knife.handle_len_mm * d, z), np.append(g - cfg.knife.blade_overhang_mm * d, z)
            if not path._in_box(cfg, tip, 0.0):
                return f"{label}: the fingertip would leave the workspace box at ({tip[0]:.0f}, {tip[1]:.0f}) mm"
            if ends_in_box and not (path._in_box(cfg, handle, 0.0) and path._in_box(cfg, blade, 0.0)):
                return f"{label}: an end of the screwdriver would leave the box (the first, faster pass)"
            if float(np.linalg.norm(g - np.asarray(cfg.base_xy, float))) > REACH_TOPDOWN_MM + 15.0:
                return f"{label}: the grip point would be beyond the arm's top-down reach"
            if float(np.linalg.norm(blade - p3)) < POINT_CLEAR_MM:
                return f"{label}: the pointed end would come within {float(np.linalg.norm(blade - p3)):.0f} mm of the palm"
            pts = [(tip, 0.0)] + [(blade + (handle - blade) * k / (SAMPLES - 1), r) for k in range(SAMPLES)]
            for p, below in pts:
                if float(np.linalg.norm(p[:2] - p3[:2])) <= HAND_R_MM and float(p[2]) - below - cfg.table_z_mm < floor:
                    return f"{label}: over the hand a part would pass under the release height"
    return None


def _build(cfg: HandoverCfg, q_now, palm_mm, segments: list) -> dict:
    """Waypoints -> rows; each move at 3 deg/s if either end is within NEAR_MM of the palm or it is the descent,
    10 deg/s otherwise. -> {"rows", "phases", "problem"}."""
    cur, seed = [float(v) for v in q_now], cfg.to_mjcf(q_now)
    near_prev = _near(cfg, knife_pose(cfg.to_mjcf(cur), cfg.knife), palm_mm)
    rows, phases = [], []
    for label, ways in segments:
        n0 = len(rows)
        for w in ways:
            q, why = _solve(cfg, w, seed)
            if q is None:
                return {"rows": rows, "phases": phases, "problem": f"{label}: {why}"}
            near = _near(cfg, knife_pose(list(q), cfg.knife), palm_mm)
            dps = NEAR_DPS if (near or near_prev or label.startswith("descend")) else FAR_DPS
            cap = dps / cfg.control_hz - path.ROUND_MARGIN_DEG
            tgt = cfg.to_lerobot(q) + [cur[5]]
            rows.extend(null_plan.interpolate(cur, tgt, [cap] * 5 + [null_plan.GRIP_DELTA_PCT]))
            cur, seed, near_prev = tgt, list(q), near
        phases.append({"phase": label, "from_step": n0 + 1, "n_steps": len(rows) - n0})
    return {"rows": [[round(float(x), 3) for x in r] for r in rows], "phases": phases, "problem": None}


def check(cfg: HandoverCfg, rows: list, phases: list, palm_mm, q_now) -> dict:
    """Every row through the rules, and every step against its speed: a row near the palm moves <= 3 deg/s."""
    after_rise = phases[0]["n_steps"] if phases else 0
    p3 = palm3(cfg, palm_mm)
    st = {"min_point_palm_3d_mm": float("inf"), "max_step_deg_near": 0.0, "max_step_deg": 0.0, "n_rows": len(rows),
          "problem": None}
    prev = [float(v) for v in q_now]
    for k, row in enumerate(rows):
        why = row_problem(cfg, row, palm_mm, k >= after_rise)
        if why:
            st["problem"] = f"row {k + 1}: {why}"
            return st
        kp = knife_pose(cfg.to_mjcf(row), cfg.knife)
        st["min_point_palm_3d_mm"] = min(st["min_point_palm_3d_mm"], float(np.linalg.norm(np.asarray(kp.blade_tip) - p3)))
        step = max(abs(row[i] - prev[i]) for i in range(5))
        st["max_step_deg"] = max(st["max_step_deg"], step)
        if _near(cfg, kp, palm_mm):
            st["max_step_deg_near"] = max(st["max_step_deg_near"], step)
        prev = row
    if st["max_step_deg_near"] > NEAR_DPS / cfg.control_hz + 1e-6:
        st["problem"] = f"a step of {st['max_step_deg_near']:.3f} deg within {NEAR_MM:g} mm of the palm (> {NEAR_DPS:g} deg/s)"
    if st["max_step_deg"] > FAR_DPS / cfg.control_hz + 1e-6:
        st["problem"] = f"a step of {st['max_step_deg']:.3f} deg (> {FAR_DPS:g} deg/s)"
    return {k: (round(v, 3) if isinstance(v, float) and math.isfinite(v) else v) for k, v in st.items()}


def _waypoints(cfg: HandoverCfg, a, b) -> list:
    """Translate waypoints inside the box, nearest the straight route first."""
    x0, x1, y0, y1 = cfg.box_xy
    pts = [(float(x), float(y)) for x in np.arange(x0 + 20, x1 - 10, WAYPOINT_STEP_MM)
           for y in np.arange(y0 + 20, y1 - 10, WAYPOINT_STEP_MM) if path._in_box(cfg, (x, y), cfg.box_inner_mm)]
    a, b = np.asarray(a, float), np.asarray(b, float)
    return sorted(pts, key=lambda p: float(np.linalg.norm(np.asarray(p) - a) + np.linalg.norm(np.asarray(p) - b)))


def _routes(rise: path.Way, above: path.Way, cfg: HandoverCfg) -> list:
    """The turn on the way; the move then the turn; the turn then the move; then at translate waypoints."""
    moved = path.Way(above.grip_xy, rise.h_mm, rise.heading_deg)
    turned = path.Way(rise.grip_xy, rise.h_mm, above.heading_deg)
    out = [[("move above the release point, turning", path.subdivide(rise, above))],
           [("move above the release point", path.subdivide(rise, moved)), ("turn in place", path.subdivide(moved, above))],
           [("turn in place", path.subdivide(rise, turned)), ("move above the release point", path.subdivide(turned, above))]]
    for w in _waypoints(cfg, rise.grip_xy, above.grip_xy)[:N_WAYPOINTS]:
        at = path.Way(w, rise.h_mm, rise.heading_deg)
        at_t = path.Way(w, rise.h_mm, above.heading_deg)
        out.append([("move to a turning point", path.subdivide(rise, at)), ("turn at the turning point", path.subdivide(at, at_t)),
                    ("move above the release point", path.subdivide(at_t, above))])
    return out


def plan_placement(q_now_lerobot6, palm_mm, fingers_dir, cfg: HandoverCfg, finger_reach_mm: float | None = None,
                   deviations=DEVIATIONS_DEG, budget_s: float | None = None, grip_pct: float | None = None) -> Plan | Refusal:
    """grip_pct (holding the prop): the gripper column goes from its reading back to the close target at
    SQUEEZE_PCT_PER_ROW and stays there - a freeze's goals := present leaves the stalled jaws slack on the handle
    (KH-S20261004T170052: load 180 -> 0 at the first freeze, the heavy pointed end dipping; the operator, 2026-10-05)."""
    deadline = time.monotonic() + (PLAN_BUDGET_S if budget_s is None else float(budget_s))
    q_now, palm = [float(v) for v in q_now_lerobot6], np.asarray(palm_mm, float)[:2]
    refused = _arm_refusal(cfg)
    if refused is not None:
        return Refusal(refused.reason, kind="placement")
    if not path._in_box(cfg, palm, 0.0):
        return Refusal(f"the palm at ({palm[0]:.0f}, {palm[1]:.0f}) mm is outside the allowed region", kind="placement")
    df = np.asarray(fingers_dir, float)[:2] if fingers_dir is not None else None
    if df is None or not np.all(np.isfinite(df)) or float(np.linalg.norm(df)) < 1e-6:
        return Refusal("the hand's direction (wrist -> fingers) is not known: no placement", kind="placement")
    df = df / float(np.linalg.norm(df))
    kp0 = knife_pose(cfg.to_mjcf(q_now), cfg.knife)
    now = row_problem(dataclasses.replace(cfg), q_now, palm, False)
    if now and "pointed end" in now:
        return Refusal(f"where the arm is now, {now}: no move is planned from there", kind="placement")
    reach = float(finger_reach_mm) if finger_reach_mm else FINGER_REACH_MM
    s_min = max(0.0, reach + TIP_PAST_MM - cfg.knife.blade_overhang_mm)  # the pointed end still past the fingertips
    s0 = min(max(s_min, cfg.knife.handle_len_mm / 2.0), cfg.knife.handle_len_mm - 10.0)
    slides = [v for v in (s0, s0 - 12.5, s0 - 25.0) if v >= s_min]       # the grip toward the wrist when out of reach
    #                                                                     (04:07-04:22: six palms beyond it at s0)
    start = path.Way((float(kp0.grip[0]), float(kp0.grip[1])), float(kp0.tip[2] - cfg.table_z_mm), kp0.heading_deg)
    tried: list = []
    t0, total = time.monotonic(), deadline - time.monotonic()
    for ends_in_box, until in ((True, t0 + 0.6 * total), (False, deadline)):   # the box first (fast), then outside it
        for s in slides:
            got = _search(cfg, q_now, palm, df, s, reach, start, deviations, ends_in_box, until, tried, grip_pct)
            if got is not None:
                return got
    budget = " within %g s" % (PLAN_BUDGET_S if budget_s is None else float(budget_s)) if time.monotonic() > deadline else ""
    return Refusal(f"no placement for the palm at ({palm[0]:.0f}, {palm[1]:.0f}) mm{budget}: "
                   f"{tried[0][1] if tried else 'nothing tried'} ({len(tried)} directions tried)", tuple(tried), "placement")


def _search(cfg, q_now, palm, df, s, reach, start, deviations, ends_in_box: bool, until: float, tried: list,
            grip_pct: float | None = None):
    """One pass over the directions and routes. -> a Plan, or None (the reasons appended to `tried`)."""
    for dev in deviations:
        if time.monotonic() > until:
            return None
        d = _rot(df, dev)
        grip = palm + s * d
        heading = math.degrees(math.atan2(-d[1], -d[0]))                 # the handle toward the wrist
        far = float(np.linalg.norm(grip - np.asarray(cfg.base_xy, float)))
        if far > REACH_TOPDOWN_MM:
            tried.append((dev, f"the release pose: the grip point {far:.0f} mm from the pan axis (> {REACH_TOPDOWN_MM:g})"))
            continue
        h_rel, axis_dz, why = release_h(cfg, grip, heading, cfg.to_mjcf(q_now))
        if h_rel is None:
            tried.append((dev, why))
            continue
        z_turn = max(cfg.z_transit_mm, h_rel + DESCENT_MIN_MM)
        rise = path.Way(start.grip_xy, max(start.h_mm, z_turn), start.heading_deg)
        why = None
        for turn in _turn_options(start.heading_deg, heading):
            above = path.Way((float(grip[0]), float(grip[1])), z_turn, turn)
            end = path.Way(above.grip_xy, h_rel, turn)
            for route in _routes(rise, above, cfg):
                if time.monotonic() > until:
                    tried.append((dev, why or "the planning budget ran out"))
                    return None
                segs = [("rise in place", [rise])] + route + [("descend to the release height", path.subdivide(above, end))]
                early = precheck(cfg, segs, palm, axis_dz, ends_in_box)
                if early:
                    why = why or early
                    continue
                got = _finish(cfg, q_now, palm, segs, end, dev, tried, reach, s, grip_pct)
                if isinstance(got, Plan):
                    return got
                why = why or got
        tried.append((dev, why))
    return None


SQUEEZE_PCT_PER_ROW = 3.0 / 15.0  # the gripper at 3 %/s (15 Hz), as every scripted gripper move


def squeeze(rows: list, g0: float, target: float) -> list:
    """In place: the gripper column from g0 toward target (closing) at SQUEEZE_PCT_PER_ROW, then target. Never opens:
    a reading already at or past the target is kept."""
    for k, r in enumerate(rows):
        r[5] = round(max(target, g0 - SQUEEZE_PCT_PER_ROW * (k + 1)), 3) if g0 > target else round(g0, 3)
    return rows


def _turn_options(start_heading: float, end_heading: float) -> tuple:
    short = path.nearest_turn(end_heading, start_heading)
    return short, short - 360.0 if short > start_heading else short + 360.0


def _finish(cfg, q_now, palm, segs, end: path.Way, dev, tried, reach, s, grip_pct: float | None = None) -> Plan | str:
    built = _build(cfg, q_now, palm, segs)
    if built["problem"]:
        return built["problem"]
    rows, phases = built["rows"], built["phases"]
    path.hold_rows(rows, phases, q_now, cfg.hold_rows)
    if grip_pct is not None:
        squeeze(rows, float(q_now[5]), float(grip_pct))
    st = check(cfg, rows, phases, palm, q_now)
    if st["problem"]:
        return st["problem"]
    kp = knife_pose(cfg.to_mjcf(rows[-1]), cfg.knife)
    turned = abs(path.nearest_turn(kp.heading_deg - knife_pose(cfg.to_mjcf(q_now), cfg.knife).heading_deg, 0.0))
    checks = {**{k: v for k, v in st.items() if k != "problem"},
              "release_lowest_mm": round(lowest_mm(cfg, kp), 1), "release_floor_mm": _release_floor(cfg),
              "release_above_palm_mm": RELEASE_MM, "sag_allowance_mm": SAG_MM, "palm_h_mm": PALM_H_MM,
              "pointed_end_from_palm_mm": round(float(np.linalg.norm(kp.blade_tip[:2] - palm)), 1),
              "finger_reach_mm": round(reach, 1), "handle_middle_from_palm_mm": round(abs(s - cfg.knife.handle_len_mm / 2), 1),
              "turned_deg": round(turned, 1), "deviation_deg": dev, "point_clear_mm": POINT_CLEAR_MM,
              "near_mm": NEAR_MM, "near_dps": NEAR_DPS, "far_dps": FAR_DPS}
    return Plan(kind="placement", targets=tuple(tuple(r) for r in rows), phases=tuple(phases),
                palm_mm=(round(float(palm[0]), 1), round(float(palm[1]), 1)),
                grip_point_mm=(round(end.grip_xy[0], 1), round(end.grip_xy[1], 1)),
                handle_tip_mm=(round(float(kp.handle_tip[0]), 1), round(float(kp.handle_tip[1]), 1)),
                heading_deg=round(path.nearest_turn(end.heading_deg, 0.0), 2), candidate_deg=dev, checks=checks,
                plan_sha256=rows_sha256(rows), ik_method=cfg.ik_method, z_handover_mm=round(end.h_mm, 1),
                knife_placeholder=cfg.knife.placeholder, tried=tuple(tried))
