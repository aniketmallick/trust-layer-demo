"""Plan-level fixtures (registered into planner.fixtures.REGISTRY on import), one per IK method."""
from __future__ import annotations

import dataclasses

import numpy as np

from planner import fixtures as fx
from planner import handover_planner as hp
from planner.fixtures import _both
from planner import path
from planner.knife import Knife, knife_pose

BLADE_SIDE_PALM = (220.0, 120.0)          # behind the handle of the mid-carry pose: the blade end leads
MEASURED = Knife(blade_overhang_mm=40.0, placeholder=False)   # a stand-in marked measured: the arm gate, not the
#   geometry. The pointed end 40 mm (was the placeholder's 20): with 20 the control plan came 148.6 mm from the palm
#   and the 150 mm rule for measured tools (2026-10-03) refused it; with 40, 164.8 mm.
FAR_PALM = (340.0, 100.0)                 # a palm the mid-carry pose reaches without re-orienting
NO_POSE_PALM = (200.0, -100.0)            # inside the box; every standoff point for it lies outside the box


def _ms(xs) -> str:
    return "none" if not xs else f"median {1000 * float(np.median(xs)):.0f} ms, max {1000 * max(xs):.0f} ms"


@_both("PLAN", "PLAN_50_targets_ik_fk_error",
       "50 seeded random palms in the workspace box that get a plan from the mid-carry pose (handle radially outward): "
       "the last row, read through anchor/fk.py, puts the fingertip within 3 mm of the planned fingertip and the handle "
       "within 3 deg of (palm - grip point); refused palms are counted", kind="control")
def _targets(method):
    t = fx.targets(method)
    errs = [fx.end_errors(t["cfg"], it["plans"][-1], it["palm"]) for it in t["items"]]
    pos, head = [e[0] for e in errs], [e[1] for e in errs]
    ok = len(errs) == fx.N_TARGETS and max(pos) < 3.0 and max(head) < 3.0
    return ok, (f"{len(errs)} plans from {t['tries']} random palms ({t['via_reorient']} via re-orient); fingertip error "
                f"max {max(pos):.3f} / median {np.median(pos):.3f} mm; handle heading error max {max(head):.3f} / median "
                f"{np.median(head):.3f} deg; planning time: planned {_ms(t['secs_ok'])}; refused {_ms(t['secs_no'])}")


@_both("PLAN", "PLAN_handle_tip_never_inside_60mm",
       "every row of every plan for the 50 palms (re-orient plans included): the handle tip - and every other point of "
       "the knife - stays 60 mm or more from the palm on the table plane", kind="control")
def _standoff(method):
    t = fx.targets(method)
    ms = [fx.measure(t["cfg"], q, p, it["palm"]) for it in t["items"]
          for q, p in zip([t["q0"]] + [list(x.targets[-1]) for x in it["plans"][:-1]], it["plans"])]
    tip, knife = min(m["min_handle_tip_palm"] for m in ms), min(m["min_knife_palm"] for m in ms)
    rows = sum(len(p.targets) for it in t["items"] for p in it["plans"])
    return (tip >= 60.0 and knife >= 60.0), (f"{rows} rows in {len(ms)} plans: closest handle tip {tip:.2f} mm, closest "
                                              f"point of the knife {knife:.2f} mm")


@_both("PLAN", "PLAN_every_row_in_box_and_limits",
       "every row of every plan: fingertip (x, y, z), handle tip and blade tip inside the G3 workspace box, every joint "
       "inside the guard's limits (calibrated range - 3 deg), no arm joint above 3 deg/s at 15 Hz (0.2 deg per row)",
       kind="control")
def _rows(method):
    t = fx.targets(method)
    ms = [fx.measure(t["cfg"], q, p, it["palm"]) for it in t["items"]
          for q, p in zip([t["q0"]] + [list(x.targets[-1]) for x in it["plans"][:-1]], it["plans"])]
    out_box, out_lim, step = sum(m["outside_box"] for m in ms), sum(m["outside_limits"] for m in ms), max(m["max_step"] for m in ms)
    n = [len(p.targets) for it in t["items"] for p in it["plans"]]
    return (out_box == 0 and out_lim == 0 and step <= 0.2 + 1e-9), (
        f"rows outside the box {out_box}, outside limits {out_lim}; largest arm step {step:.4f} deg per row "
        f"({step * 15:.3f} deg/s); rows per plan median {int(np.median(n))}, max {max(n)} ({max(n) / 15:.0f} s)")


@_both("PLAN", "PLAN_g3_guard_replay_production_limits",
       "the G3 guard of record (production speed limit, box on) over every plan's rows through the fault table's "
       "lagging-servo model: nothing fires", kind="control")
def _g3_nominal(method):
    fired = [(n, k, f) for n, k, f in fx.replay(method, None) if f]
    return (not fired), f"{len(fired)} plan(s) raised a fire" + (f"; first: {fired[0][2][0][1]} - {fired[0][2][0][2]}" if fired else "")


@_both("PLAN", "PLAN_g3_guard_replay_speed_limit_15dps",
       "the same replay with monitor 2 at 15 deg/s (PLAN.md section 4, handover segments, commanded <= 3 deg/s): "
       "nothing fires over all plans; the counts at 9, 10 and 12 deg/s are printed as evidence, not asserted",
       kind="control")
def _g3_scaled(method):
    res = fx.replay(method, fx.SPEED_LIMIT_HANDOVER_DPS)
    fired = [(n, k, f) for n, k, f in res if f]
    codes = sorted({f[0][1] for _, _, f in fired})
    lower = {v: sum(1 for _, _, f in fx.replay(method, v) if f) for v in fx.SPEED_EVIDENCE_DPS}
    return (not fired), (f"{len(fired)} of {len(res)} plans raised a fire {codes} at {fx.SPEED_LIMIT_HANDOVER_DPS:g} deg/s"
                         + (f"; first: {fired[0][2][0][2]}" if fired else "")
                         + "; plans firing at lower limits (evidence): "
                         + ", ".join(f"{v:g} deg/s: {n}" for v, n in lower.items()))


@_both("PLAN", "PLAN_g3_guard_speed_step_fires_at_15dps",
       "one plan replayed with shoulder_lift's measured value stepped at row 31 so that it reads 20 deg/s for that one "
       "row: joint_speed fires at the 15 deg/s limit, at that row, and nothing fired before it")
def _g3_speed_fire(method):
    at, joint, reads, prev = 30, 1, 20.0, {}

    def step(k, m, dt, load):
        if k == at:
            m = m.copy()
            m[joint] = prev["m"][joint] + reads * dt
        prev["m"] = m.copy()
        return m, dt, load
    fired = fx.replay(method, fx.SPEED_LIMIT_HANDOVER_DPS, inject=step, only=0)[0][2]
    codes = [c for _, c, _ in fired]
    ok = bool(fired) and codes[0] == "joint_speed" and fired[0][0] == at and "shoulder_lift" in fired[0][2]
    return ok, (f"fired {codes} at row {fired[0][0] + 1}: {fired[0][2]}" if fired else "nothing fired")


@_both("PLAN", "PLAN_g3_guard_pan_pushed_out_fires",
       "one plan replayed with the measured pan swung away from row 20 (2.5 deg more each row, as the fault table's "
       "f_box): ee_workspace fires")
def _g3_box(method):
    def swing(k, m, dt, load):
        if k >= 20:
            m = m.copy()
            m[0] += min(90.0, 2.5 * (k - 19))
        return m, dt, load
    fired = fx.replay(method, None, inject=swing, only=0)[0][2]
    codes = [c for _, c, _ in fired]
    return ("ee_workspace" in codes), (f"fired {codes} at row {fired[0][0] + 1}: {fired[0][2]}" if fired else "nothing fired")


@_both("PLAN", "PLAN_unreachable_palm_refused",
       "a palm 40 mm outside the workspace box, and a palm inside it for which every standoff point lies outside the "
       "box: both are refused with the reason, and no rows are returned")
def _unreachable(method):
    cfg, q0 = fx.cfg_for(method), list(fx.start_pose(method))
    out = hp.plan_handover(q0, (cfg.box_xy[1] + 40.0, 0.0), cfg)
    a = hp.plan_handover(q0, NO_POSE_PALM, cfg)
    b = hp.plan_reorient(q0, NO_POSE_PALM, cfg)
    ok = (not out.ok and "outside the allowed region" in out.reason and not out.targets
          and not a.ok and not b.ok and not a.targets and not b.targets and bool(b.reason) and len(b.tried) == 9)
    return ok, f"outside: '{out.reason}'; inside: handover '{a.reason[:70]}', re-orient '{b.reason[:110]}'"


@_both("PLAN", "PLAN_blade_toward_palm_refused_then_reorient",
       "a palm behind the handle (the blade end leads): plan_handover refuses and names the re-orient path; "
       "plan_reorient returns rows that keep the knife 60 mm from the palm; after them the blade rule is clear and "
       "plan_handover and retarget both return a plan")
def _reorient(method):
    cfg, q0 = fx.cfg_for(method), list(fx.start_pose(method))
    first = hp.plan_handover(q0, BLADE_SIDE_PALM, cfg)
    r = hp.plan_reorient(q0, BLADE_SIDE_PALM, cfg)
    if not r.ok:
        return False, f"re-orient refused: {r.reason}"
    q1 = list(r.targets[-1])
    m = fx.measure(cfg, q0, r, BLADE_SIDE_PALM)
    after, again = hp.plan_handover(q1, BLADE_SIDE_PALM, cfg), hp.retarget(r, BLADE_SIDE_PALM, q1, cfg)
    ok = (hp.blade_toward_palm(q0, BLADE_SIDE_PALM, cfg) and not first.ok and "re-orient" in first.reason
          and not hp.blade_toward_palm(q1, BLADE_SIDE_PALM, cfg) and m["min_knife_palm"] >= 60.0 and m["outside_box"] == 0
          and after.ok and again.ok and again.retarget_of == r.plan_sha256)
    return ok, (f"handover: '{first.reason[:60]}'; re-orient {len(r.targets)} rows "
                f"{[(p['phase'][:14], p['n_steps']) for p in r.phases]}, knife >= {m['min_knife_palm']:.1f} mm from the palm; "
                f"then handover ok={after.ok} ({len(after.targets)} rows, approach {after.candidate_deg:+g} deg)")


@_both("PLAN", "PLAN_retarget_30mm",
       "the palm drifts 30 mm sideways while the knife is above the standoff point: retarget returns a fresh plan from "
       "the present pose that passes every check; the same drift straight at the held knife is refused (inside the "
       "standoff)")
def _retarget(method):
    cfg, q0 = fx.cfg_for(method), list(fx.start_pose(method))
    p = hp.plan_handover(q0, FAR_PALM, cfg)
    if not p.ok:
        return False, f"no first plan: {p.reason}"
    above = p.phases[-2]["from_step"] - 1                       # the row before the descent starts
    q_mid, q_hold = list(p.targets[above - 1]), list(p.targets[-1])
    kp = knife_pose(cfg.to_mjcf(q_hold), cfg.knife)
    side = np.array([-kp.handle_xy[1], kp.handle_xy[0]])
    new = tuple(np.asarray(FAR_PALM) + 30.0 * side)
    r = hp.retarget(p, new, q_mid, cfg)
    toward = hp.retarget(p, tuple(np.asarray(FAR_PALM) - 30.0 * kp.handle_xy), q_hold, cfg)
    if not r.ok:
        return False, f"sideways re-target refused: {r.reason}"
    m = fx.measure(cfg, q_mid, r, new)
    pos, head = fx.end_errors(cfg, r, new)
    ok = (r.retarget_of == p.plan_sha256 and abs(r.drift_mm - 30.0) < 0.2 and m["min_knife_palm"] >= 60.0
          and m["outside_box"] == 0 and m["outside_limits"] == 0 and m["max_step"] <= 0.2 + 1e-9 and pos < 3.0
          and head < 3.0 and not toward.ok and "standoff" in toward.reason)
    return ok, (f"sideways: {len(r.targets)} rows, drift {r.drift_mm} mm, knife >= {m['min_knife_palm']:.1f} mm, end error "
                f"{pos:.3f} mm / {head:.3f} deg; toward the knife: '{toward.reason[:90]}'")


@_both("PLAN", "PLAN_z_handover_unset_on_arm_refused",
       "cfg.arm=True with z_handover_mm never set: refused before any IK; with with_dawn(45 mm) the same palm gets a "
       "plan whose last row holds the fingertip 45 mm above the table")
def _z_unset(method):
    cfg, q0 = fx.cfg_for(method), list(fx.start_pose(method))
    no = hp.plan_handover(q0, FAR_PALM, dataclasses.replace(cfg, arm=True))
    dawn = hp.with_dawn(dataclasses.replace(cfg, arm=True), 45.0, knife=MEASURED)
    yes = hp.plan_handover(q0, FAR_PALM, dawn)
    h = knife_pose(dawn.to_mjcf(yes.targets[-1]), dawn.knife).tip[2] - dawn.table_z_mm if yes.ok else None
    return (not no.ok and "z_handover_mm" in no.reason and yes.ok and abs(h - 45.0) < 1.0), (
        f"unset: '{no.reason[:60]}'; set to 45: ok={yes.ok}, fingertip {None if h is None else round(float(h), 2)} mm up")


@_both("PLAN", "PLAN_knife_unset_on_arm_refused",
       "reviewer 2026-10-01 (question 5): cfg.arm=True with z_handover set at dawn but the placeholder knife: handover "
       "and re-orient refused before any IK, naming the dimensions; with measured dimensions the same palm gets a plan")
def _knife_unset(method):
    cfg, q0 = fx.cfg_for(method), list(fx.start_pose(method))
    dawn = hp.with_dawn(dataclasses.replace(cfg, arm=True), 45.0)
    no, no_o = hp.plan_handover(q0, FAR_PALM, dawn), hp.plan_reorient(q0, FAR_PALM, dawn)
    yes = hp.plan_handover(q0, FAR_PALM, hp.with_dawn(dataclasses.replace(cfg, arm=True), 45.0, knife=MEASURED))
    ok = (not no.ok and not no_o.ok and "dimensions" in no.reason and "dimensions" in no_o.reason and not no.targets
          and yes.ok)
    return ok, f"placeholder: '{no.reason[:70]}'; measured: ok={yes.ok}"


@_both("PLAN", "PLAN_palm_in_the_blind_strip_refused",
       "reviewer 2026-10-01 (question 11): a palm inside the G3 box but outside the camera's footprint (the near-edge "
       "strip the camera does not see) is refused by handover and re-orient; the footprint is the image less 10 px")
def _blind(method):
    cfg, q0 = fx.cfg_for(method), list(fx.start_pose(method))
    blind = (180.0, -150.0)                                   # x0 171.9 < 180 < the footprint edge at y -150 (196.2)
    x0, x1, y0, y1 = cfg.box_xy
    in_g3 = x0 <= blind[0] <= x1 and y0 <= blind[1] <= y1
    a, b = hp.plan_handover(q0, blind, cfg), hp.plan_reorient(q0, blind, cfg)
    seen = (200.0, 60.0)                                       # control: inside both
    ok = (in_g3 and len(cfg.visible_quad) == 4 and not a.ok and not b.ok and "camera's footprint" in a.reason
          and "camera's footprint" in b.reason and path._visible(cfg, seen) and not path._visible(cfg, blind))
    return ok, f"blind palm {blind}: '{a.reason[:90]}'"


@_both("PLAN", "PLAN_inside_standoff_at_start_refused",
       "a palm 40 mm to the side of the handle of the present pose: no move is planned from inside the standoff "
       "(handover and re-orient both refuse; the runner asks)")
def _inside(method):
    cfg, q0 = fx.cfg_for(method), list(fx.start_pose(method))
    kp = knife_pose(cfg.to_mjcf(q0), cfg.knife)
    side = np.array([-kp.handle_xy[1], kp.handle_xy[0]])
    palm = tuple(kp.grip[:2] + 50.0 * kp.handle_xy + 40.0 * side)
    a, b = hp.plan_handover(q0, palm, cfg), hp.plan_reorient(q0, palm, cfg)
    return (not a.ok and not b.ok and "standoff" in a.reason and "standoff" in b.reason), f"'{a.reason}'"


@_both("PLAN", "PLAN_same_inputs_same_rows",
       "the same pose, palm and cfg planned twice: the same plan_sha256, and it is the sha256 of the rows", kind="control")
def _same(method):
    cfg, q0 = fx.cfg_for(method), list(fx.start_pose(method))
    a, b = hp.plan_handover(q0, FAR_PALM, cfg), hp.plan_handover(q0, FAR_PALM, cfg)
    ok = a.ok and b.ok and a.plan_sha256 == b.plan_sha256 == hp.rows_sha256(a.targets) and a.targets == b.targets
    return ok, f"plan_sha256 {a.plan_sha256[:16] if a.ok else None} twice, {len(a.targets) if a.ok else 0} rows"
