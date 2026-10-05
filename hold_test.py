"""Dawn step 4 (2026-10-02): the hold test on the screwdriver (prop tool), inside the runner - every row through
Motion.run_rows, the eight monitors live, the kill switch, freeze-and-ask, the session's chain.

    handover_session.py --arm ... --segments dawn/segments.json --hold-test

The arm goes from rest over the grip mark (jaws opening), down around the handle, closes at the segments' close target,
lifts 20 mm and holds 10 s (the gripper's load in every step row, its mean and max in a 'hold_test' row, a still before
and after; the lift as commanded and as measured from the encoders - 20:38 on 2026-10-02 the load left it ~7 mm). The operator answers whether the screwdriver slipped. Slipped -> lowered, the close target one step (1 %)
tighter, lifted and held again - once. Then it is set down, released, and the arm goes home. Slipped twice -> the
outcome says so and nothing more is tried (the operator decides). The tighter step is not taken when it would pass
the overload bar or the gripper floor (the operator, 2026-10-04: "never past the gripper floor or the overload
limit"): the first hold's load max + LOAD_PER_PCT per % >= HOLD_LOAD_MAX (the servo's own trip, measured, is under
monitor 4's OVERLOAD_BAR), or the target under the gripper's floor ->
the outcome says which, with the numbers, and the arm sets the screwdriver down.
"""
from __future__ import annotations

from common import paths

paths.use_anchor()
import g3  # noqa: E402

from grasp_segment import interpolate  # noqa: E402
from handover_flow import Stop  # noqa: E402
from planner import ik  # noqa: E402

TIGHTER_STEP_PCT = 1.0          # 14:52: load ~ 661 - 46.6 x target; 9.5 % ~ 218, the 250 bar at 8.8 %
LOAD_PER_PCT = 46.6             # the gripper's load per % of target past contact (KH-S20261002T145204, 56 rows)
OVERLOAD_BAR = 250              # monitor 4: 50 % of the 500 torque limit for 0.5 s (anchor/g3.py)
HOLD_LOAD_MAX = 220             # the gripper servo tripped its own overload protection after ~1.9 s at 240-244, twice,
#                                 below monitor 4's bar (KH-S20261004T162508): a tighter step must stay under this
HOLD_S = 10.0
GRIP_PPS = 3.0
LIFT_MM = 20.0
LIFT_MEASURED_BY = ("jaw-tip height by the planner's FK (planner/ik.tip_and_tilt) of the encoder reads: "
                    "closed on the handle, before the lift, vs the end of the hold")


def _run(s, phase: str, rows: list) -> None:
    """Rows at the scripted speed with the monitors live. A freeze is asked like any other; [c] re-anchors at the
    present pose and goes on; [k] / [l] end the session (Stop)."""
    rows = [list(r) for r in rows]
    while True:
        if s.gate(phase):
            s.state = phase
            res = s.motion.run_rows(phase, rows, vel_limit=s.scripted_vel)
            if res["end"] == "done":
                return
            rows = res["rows"][res["k"]:] or [rows[-1]]
        if s.freeze_flow(phase, resume_ok=True) == "handed_over":
            raise Stop("the hold test ended in a handover the operator chose at the menu")
        rows = interpolate(s.rig.read_q(), rows[0], s.scripted_dps, GRIP_PPS) + rows


def _no_tighter(s, close: float, row: dict) -> str | None:
    """The tighter step's refusal (the overload bar, the gripper floor), or None."""
    nxt = round(close - TIGHTER_STEP_PCT, 2)
    peak = (row.get("gripper_load") or {}).get("max")
    if peak is None:
        return f"no gripper load was read in the first hold: {nxt:g} % is not tried blind"
    pred = round(float(peak) + LOAD_PER_PCT * TIGHTER_STEP_PCT)
    if pred >= HOLD_LOAD_MAX:
        return (f"slipped / pitched at {close:g} %; {nxt:g} % is predicted at load ~{pred} (first hold max {peak} + "
                f"{LOAD_PER_PCT:g} per %), past the {HOLD_LOAD_MAX} hold limit (the gripper servo tripped at 240-244): "
                f"stopped at {close:g} %")
    floor = float(s.guard.limits()["gripper"][0]) + 0.5
    if nxt < floor:
        return f"{nxt:g} % is under the gripper's floor ({floor:g} %): stopped at {close:g} %"
    return None


def _gripper_to(s, pct: float) -> list:
    goal = [float(s.rig.sbus.last_goal.get(j, s.rig.read_q()[i])) for i, j in enumerate(g3.ALL)]
    return interpolate(goal, goal[:5] + [float(pct)], s.scripted_dps, GRIP_PPS) + [goal[:5] + [float(pct)]] * 5


def _tip_z(q6) -> float:
    return float(ik.tip_and_tilt([float(v) for v in q6[:5]])[0][2])


def _hold(s, attempt: int, close_pct: float, z0: float) -> dict:
    before = s.pump.save(s.dir / "photos", f"hold{attempt}_start")
    n0 = len(s.chain.rows())
    res = s.motion.hold_rows("HOLD", HOLD_S)
    lift = round(_tip_z(s.rig.read_q()) - z0, 1)
    after = s.pump.save(s.dir / "photos", f"hold{attempt}_end")
    loads = [abs(int(r["load"])) for r in s.chain.rows()[n0:] if r.get("kind") == "step" and r.get("load") is not None]
    if res["end"] == "frozen":
        s.freeze_flow("HOLD", resume_ok=False)
    ans = s.keys.console.ask("  Did the screwdriver SLIP or PITCH (nose down) in the jaws during the hold? "
                             "[y = slipped or pitched / n = held]: ",
                             "n" if s.dry else "").strip().lower()
    row = s.chain.append({"kind": "hold_test", "t_iso": g3.now_iso(), "session_id": s.session_id, "trial_id": s.trial_id,
                          "attempt": attempt, "close_pct": close_pct, "held_s": HOLD_S, "lift_mm_commanded": LIFT_MM,
                          "lift_mm_measured": lift, "lift_measured_by": LIFT_MEASURED_BY,
                          "slipped_by_operator": ans == "y", "gripper_load": {
                              "n": len(loads), "mean": round(sum(loads) / len(loads), 1) if loads else None,
                              "max": max(loads) if loads else None, "overload_bar": 250},
                          "frozen_during_hold": res["end"] == "frozen",
                          "stills": {"start": before["file"], "start_sha256": before["sha256"],
                                     "end": after["file"], "end_sha256": after["sha256"]}})
    s.say(f"  HOLD {attempt}: close {close_pct:g} %, load mean {row['gripper_load']['mean']} / max "
          f"{row['gripper_load']['max']} (bar 250 for 0.5 s), lift {lift:g} mm measured ({LIFT_MM:g} commanded), "
          f"slipped: {ans == 'y'}")
    return row


def run(s) -> dict:
    ht = s.seg.get("hold_test")
    if not ht:
        raise Stop("the segments file has no hold-test rows (dawn/segments.py)")
    s.trial_id = s.motion.trial_id = f"{s.session_id}/HOLD"
    grasp = next(x["rows"] for x in s.seg["segments"] if x["state"] == "GRASP")
    close = float(s.seg["close_pct"])
    s.keys.console.ask("\n=== HOLD TEST: the screwdriver laid at its mark (free handle toward the zone). Hands OUT. "
                       "Press Enter: ", "")
    _run(s, "GRASP", grasp)                                   # rest -> over the mark -> down -> close -> settle
    outcome, why, row = "HOLD_OK", "", {}
    for attempt in (1, 2):
        if attempt == 2:
            why = _no_tighter(s, close, row)
            if why:
                outcome = "HOLD_SLIPPED"
                s.say(f"  NOT TIGHTENED: {why}")
                break
            close = round(close - TIGHTER_STEP_PCT, 2)
            s.say(f"  Tighter by one step: the close target is now {close:g} %.")
            _run(s, "GRASP", _gripper_to(s, close))
        z0 = _tip_z(s.rig.read_q())
        _run(s, "LIFT", ht["lift20"])
        row = _hold(s, attempt, close, z0)
        _run(s, "PLACE", ht["lower20"])
        if not row["slipped_by_operator"]:
            why = f"held 10 s at {close:g} % (attempt {attempt})"
            break
        if attempt == 2:
            outcome, why = "HOLD_SLIPPED", f"slipped at {close + TIGHTER_STEP_PCT:g} % and at {close:g} %: stopped here"
    _run(s, "RETREAT", ht["release_and_home"])                # open, up to the hover, home
    return s.trial_end(outcome, why)
