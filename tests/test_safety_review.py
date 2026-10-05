"""Fixtures for the safety review's findings (SR01-SR14). Each was run against the code as reviewed and seen to
fail, then against the fix. Dry run only: the fake bus, synthetic frames, the simulated hand feed, the judge's stub.
"""
from __future__ import annotations

import dataclasses
import datetime
import math
import os
import signal
import threading
import time

import pytest

from common import paths

paths.use_anchor()
import g3  # noqa: E402
import safety  # noqa: E402

import ask_menu  # noqa: E402
import handover_flow  # noqa: E402
import handover_session  # noqa: E402
import motion as motion_mod  # noqa: E402
import ports  # noqa: E402
import rig as rig_mod  # noqa: E402
import start_checks  # noqa: E402
from steplog import StepLog  # noqa: E402
from tests.helpers import PlannerTap, first_index, froze_once, in_order, run_obj, tap_factory, trace  # noqa: E402

APPEAR = ("--sim-event", "appear@APPROACH_ZONE+1.0")


def kinds(rows, kind):
    return [r for r in rows if r["kind"] == kind]


def key_index(rows, key):
    return first_index(rows, lambda r: r["kind"] == "key" and r["key"] == key and r["accepted"])


def targets_after(rows, i):
    return [r for r in rows[i:] if r["kind"] == "step" and r["target6"]]


# ====================================================================== SR01: a stop pressed after the key is not erased
def test_SR01_esc_during_planning_freezes_before_any_write(tmp_path):
    tap, h = PlannerTap(), {}

    def esc(q6, palm):
        r = h["r"]
        if froze_once(r) and "writes" not in h:
            h["writes"] = r.rig.sbus.goal_writes
            r.rig.kill.trigger("ESC")
    tap.before["plan_handover"] = esc
    s = run_obj(tmp_path, *APPEAR, keys="h,l", planner=tap, setup=lambda r: h.update(r=r))
    assert s["error"] is None and s["chain_ok"][0]
    i = key_index(s["rows"], "h")
    assert targets_after(s["rows"], i) == []                               # no target after the key
    assert s["runner"].rig.sbus.goal_writes == h["writes"]                 # and none reached the send boundary
    fz = [r for r in s["rows"][i:] if r["kind"] == "freeze"]
    assert fz and fz[0]["monitor"] == 6 and fz[0]["held"] is True
    assert in_order(trace(s["rows"]), "key:h", "plan:handover:ok", "FREEZE:6", "judge:FREEZE", "key:l", "end:ABORTED")


def test_SR01_stale_link_during_planning_is_latched(tmp_path):
    """The hand feed stops for 450 ms while the planner runs and is back before the first control step."""
    tap, h = PlannerTap(), {}

    def stall(q6, palm):
        r = h["r"]
        if froze_once(r) and "done" not in h:
            h["done"] = True
            os.kill(r.feed.pid, signal.SIGSTOP)
            time.sleep(0.45)
            os.kill(r.feed.pid, signal.SIGCONT)
            time.sleep(0.30)
    tap.before["plan_handover"] = stall
    s = run_obj(tmp_path, *APPEAR, keys="h,l", planner=tap, setup=lambda r: h.update(r=r))
    assert s["error"] is None
    i = key_index(s["rows"], "h")
    assert targets_after(s["rows"], i) == []
    fz = [r for r in s["rows"][i:] if r["kind"] == "freeze"]
    assert fz and fz[0]["monitor"] == 8 and "monitor8" in str(fz[0]["source"])


def test_SR01_esc_during_a_refused_plan_is_recorded_and_the_arm_is_latched(tmp_path):
    """The planner refuses (the end of the carry) and ESC arrives while it runs: a freeze row naming monitor 6."""
    tap, h = PlannerTap(), {}

    def esc(q6, palm):
        if froze_once(h["r"]):
            h["r"].rig.kill.trigger("ESC")
    tap.before["plan_handover"] = esc
    s = run_obj(tmp_path, "--sim-event", "appear@APPROACH_ZONE+3.0", keys="h,l", planner=tap, setup=lambda r: h.update(r=r))
    assert s["error"] is None
    i = key_index(s["rows"], "h")
    assert targets_after(s["rows"], i) == []
    assert any(r["kind"] == "plan" and r["plan"]["ok"] is False for r in s["rows"][i:])
    fz = [r for r in s["rows"][i:] if r["kind"] == "freeze"]
    assert fz and fz[0]["monitor"] == 6 and fz[0]["held"] is True


def test_SR01_a_refused_h_leaves_the_arm_latched_with_a_row(tmp_path):
    s = run_obj(tmp_path, *APPEAR, "--sim-event", "second@APPROACH_ZONE+1.0", keys="h,l")
    i = key_index(s["rows"], "h")
    rl = [r for r in s["rows"][i:] if r["kind"] == "relatch"]
    assert rl and rl[0]["held"] is True and rl[0]["stop_flag_set"] is True
    assert targets_after(s["rows"], i) == []


# ====================================================================== SR02: session end needs OFF then YES
def _end_answers(monkeypatch, yes: str):
    orig = safety.Console.ask

    def ask(self, prompt, default=""):
        if "Type OFF" in prompt:
            ans = "OFF"
        elif "type YES" in prompt:
            ans = yes
        else:
            return orig(self, prompt, default)
        self.say(f"{prompt}[fixture] {ans}")
        return ans
    monkeypatch.setattr(safety.Console, "ask", ask)


def test_SR02_off_alone_at_session_end_keeps_torque_on(tmp_path, monkeypatch):
    _end_answers(monkeypatch, "")
    s = run_obj(tmp_path, script="none")
    assert s["rc"] == 0 and not any(e["event"] == "torque_off" for e in s["events"])
    assert [e for e in s["events"] if e["event"] == "bus_closed"][-1]["torque_disabled"] is False
    assert "Not confirmed" in s["out"]


def test_SR02_off_then_yes_at_session_end_disables_torque_control(tmp_path, monkeypatch):
    _end_answers(monkeypatch, "YES")
    s = run_obj(tmp_path, script="none")
    assert s["rc"] == 0 and any(e["event"] == "torque_off" for e in s["events"])
    end_keys = [r for r in kinds(s["rows"], "key") if r["state"] == "END"]
    assert end_keys and end_keys[-1]["key"] == "OFF/YES" and end_keys[-1]["accepted"] is True


# ====================================================================== SR03: the exact key, nothing else
@pytest.mark.parametrize("typed,want_key,want_accepted", [
    (["help", "ok", "hh", "", "continue?", "h "], "h", [False, False, False, False, False, True]),
    (["h"], "h", [True]), ([" H "], "h", [True]), (["cc", "c"], "c", [False, True]), (["okay", "o"], "o", [False, True]),
])
def test_SR03_menu_takes_only_the_exact_key(tmp_path, typed, want_key, want_accepted):
    con = safety.Console(auto=True, out=open(os.devnull, "w"))
    chain = StepLog(tmp_path / "c.jsonl")
    out = ask_menu.ask_freeze(ask_menu.Keys(con, list(typed)), chain, "S", "APPROACH_ZONE", "fixture", None,
                              offer_continue=True, holding=True, dry=False)
    key = out[0] if isinstance(out, tuple) else out
    rows = [r for r in chain.rows() if r["kind"] == "key"]
    assert key == want_key and [r["accepted"] for r in rows] == want_accepted


def test_SR03_release_menu_takes_only_the_exact_key(tmp_path):
    con = safety.Console(auto=True, out=open(os.devnull, "w"))
    chain = StepLog(tmp_path / "c.jsonl")
    out = ask_menu.ask_release(ask_menu.Keys(con, ["release", "ready", "r"]), chain, "S", None, False)
    key = out[0] if isinstance(out, tuple) else out
    assert key == "r" and [r["accepted"] for r in chain.rows()] == [False, False, True]


# ====================================================================== SR04: an exception in the control path
def test_SR04_on_step_that_raises_freezes_held(tmp_path, monkeypatch):
    def bad_retargeter(self, plan, approved_palm, rows):
        def on_step(k, q6, snap):
            if k >= 3:
                raise RuntimeError("fixture: the retargeter raised")
            return None
        return on_step
    monkeypatch.setattr(handover_flow.Flow, "retargeter", bad_retargeter)
    s = run_obj(tmp_path, *APPEAR, keys="h,l")
    assert s["error"] is None and s["rc"] == 1
    fz = kinds(s["rows"], "freeze")[-1]
    assert fz["code"] == "internal" and fz["held"] is True and fz["phase"] == "HANDOVER_APPROACH"
    assert "the retargeter raised" in fz["detail"]
    assert in_order(trace(s["rows"]), "key:h", "phase:HANDOVER_APPROACH", "FREEZE:0", "key:l", "end:ABORTED")
    assert kinds(s["rows"], "session_end")[0]["why"].startswith("stopped")


def test_SR04_planner_returning_none_at_h_is_a_refusal(tmp_path):
    tap, h = PlannerTap(), {}
    tap.replace["plan_handover"] = lambda real, q6, palm: None if froze_once(h["r"]) else real(q6, palm)
    s = run_obj(tmp_path, *APPEAR, keys="h,l", planner=tap, setup=lambda r: h.update(r=r))
    assert s["error"] is None and s["rc"] == 1
    i = key_index(s["rows"], "h")
    assert targets_after(s["rows"], i) == []
    plans = [r for r in s["rows"][i:] if r["kind"] == "plan"]
    assert plans and plans[0]["plan"]["ok"] is False and "nothing" in plans[0]["plan"]["reason"]


def test_SR04_planner_returning_none_at_a_retarget_freezes(tmp_path):
    tap = PlannerTap()
    tap.replace["retarget"] = lambda real, plan, palm, q6: None
    s = run_obj(tmp_path, keys="h,l", script="handover", planner=tap)
    assert s["error"] is None and s["rc"] == 1
    rt = [r for r in kinds(s["rows"], "retarget") if r["status"] == "refused"]
    assert rt and rt[0]["plan"]["ok"] is False and rt[0]["swapped"] is False and rt[0]["worker"] == "thread"
    fz = kinds(s["rows"], "freeze")[-1]
    assert fz["phase"] == "HANDOVER_APPROACH" and "re-target was refused" in fz["detail"] and fz["held"] is True
    assert targets_after(s["rows"], s["rows"].index(fz)) == []


def test_SR04_bus_read_failure_inside_hold_rows_freezes(tmp_path, monkeypatch):
    h = {"arm": False}
    orig_hold, orig_read = motion_mod.Motion.hold_rows, rig_mod.Rig.read_q

    def hold_rows(self, phase, seconds, until=None):
        if phase == "CP1" and "used" not in h:
            h["arm"] = True
        return orig_hold(self, phase, seconds, until)

    def read_q(self):
        if h["arm"]:
            h["arm"], h["used"] = False, True
            raise OSError("fixture: no status packet")
        return orig_read(self)
    monkeypatch.setattr(motion_mod.Motion, "hold_rows", hold_rows)
    monkeypatch.setattr(rig_mod.Rig, "read_q", read_q)
    s = run_obj(tmp_path, script="none", keys="l")
    assert s["error"] is None and s["rc"] == 1
    fz = kinds(s["rows"], "freeze")
    assert len(fz) == 1 and fz[0]["code"] == "hardware" and fz[0]["phase"] == "CP1" and fz[0]["held"] is True
    assert in_order(trace(s["rows"]), "phase:LIFT", "FREEZE:0", "judge:FREEZE", "key:l", "end:ABORTED")   # the first CP1 read failed


def test_SR04_internal_error_holds_stops_and_never_asks_for_torque_off(tmp_path, monkeypatch):
    orig = handover_flow.Flow.judge_at

    def judge_at(self, phase, quiet=False):
        if phase == "CP1":
            raise RuntimeError("fixture: an internal error at CP1")
        return orig(self, phase, quiet)
    monkeypatch.setattr(handover_flow.Flow, "judge_at", judge_at)
    s = run_obj(tmp_path, script="none")
    assert s["error"] is None and s["rc"] not in (0, None)
    end = kinds(s["rows"], "session_end")[0]
    assert end["why"].startswith("stopped: internal error: RuntimeError")
    fz = kinds(s["rows"], "freeze")
    assert fz and fz[-1]["code"] == "internal" and fz[-1]["held"] is True
    assert any(e["event"] == "freeze_hold_present" for e in s["events"])
    assert not any(e["event"] == "torque_off" for e in s["events"]) and "Type OFF" not in s["out"]
    assert [e for e in s["events"] if e["event"] == "bus_closed"][-1]["torque_disabled"] is False
    assert kinds(s["rows"], "trial_end")[0]["outcome"] == "ABORTED"


# ====================================================================== SR06: drift is from the palm approved at [o]
def test_SR06_drift_after_reorient_is_measured_from_the_palm_approved_at_the_key(tmp_path):
    s = run_obj(tmp_path, "--sim-palm", "353,-52", "--sim-drift-dir", "0,-1", "--sim-event", "drift30@REORIENT+2.0",
                "--sim-event", "drift60@HANDOVER_APPROACH+2.0", keys="o,l", script="reorient")
    assert s["error"] is None
    plans = [r for r in kinds(s["rows"], "plan") if r["plan"].get("ok")]
    assert [p["requested"] for p in plans] == ["reorient", "handover"]
    p0 = plans[0]["approved_palm_mm"]
    assert plans[1]["approved_palm_mm"] == p0                              # P0 stays the approved palm
    assert abs(math.hypot(plans[1]["plan"]["palm_mm"][0] - p0[0], plans[1]["plan"]["palm_mm"][1] - p0[1]) - 30.0) < 1.0
    fz = [r for r in kinds(s["rows"], "freeze") if r["phase"] == "HANDOVER_APPROACH"]
    assert fz and fz[0]["monitor"] == 7 and "60 mm from the approved palm" in fz[0]["detail"]


# ====================================================================== SR08: a re-target plan is checked at the swap
def _sr08_never_sent(s, runner) -> None:
    """No row of a plan that was not swapped in is ever sent - except rows the swapped-in plan has too (plans asked from
    the same start share their first rows: 55 identical 'rise in place' rows measured, 2026-10-01 20:20)."""
    R = lambda row: tuple(round(float(v), 3) for v in row)                 # noqa: E731
    sent = {tuple(r["target6"]) for r in s["rows"] if r["kind"] == "step" and r["target6"]}
    got = [(jid, res) for rtr in runner.retargeters for jid, res in rtr.received if getattr(res, "ok", False)]
    swapped_ids = {r["job_id"] for r in kinds(s["rows"], "retarget") if r["status"] == "swapped"}
    used = set().union(*[{R(x) for x in res.targets} for jid, res in got if jid in swapped_ids]) if swapped_ids else set()
    for jid, res in got:
        if jid not in swapped_ids:
            only = {R(x) for x in res.targets} - used
            assert not (only & sent), f"job {jid}: {len(only & sent)} rows of a plan never swapped in were sent"


def test_SR08_retarget_in_the_worker_process_stale_plans_never_sent(tmp_path):
    """Reviewer 2026-10-01 (SR08), one run, no retry: the re-target plans come from the worker PROCESS while the current
    plan keeps running; every swap is checked (palm within 5 mm, first row within 1 deg of the goal sent); no row of a
    plan that was not swapped in is sent; monitor 5 does not fire. The step-by-step decisions (discard, pending,
    late) are tests/test_retarget_unit.py."""
    s = run_obj(tmp_path, *APPEAR, "--sim-event", "drift15@HANDOVER_APPROACH+1.5", "--sim-event",
                "drift35@HANDOVER_APPROACH+1.8", "--sim-event", "second@HANDOVER_APPROACH+6.0", "--dry-retarget-delay-s", "0.2",
                keys="h,l")
    assert s["error"] is None and s["chain_ok"] == (True, "ok")
    chk = [r for r in kinds(s["rows"], "check") if r.get("name") == "retarget_worker"][0]
    assert chk["ok"] and chk["worker"] == "process"
    rt = kinds(s["rows"], "retarget")
    assert [r for r in rt if r["status"] == "asked"]
    for r in rt:
        if r["status"] == "swapped":
            assert r["palm_vs_plan_mm"] <= 5.0 and r["first_row_vs_goal_deg"] <= 1.0
    _sr08_never_sent(s, s["runner"])
    assert not [r for r in kinds(s["rows"], "freeze") if r["monitor"] == 5]
    a = s["rows"].index([r for r in rt if r["status"] == "asked"][0])
    moving = [r["target6"] for r in s["rows"][a:a + 12] if r["kind"] == "step" and r["target6"]]
    assert len({tuple(x) for x in moving}) > 1                             # the current plan went on: not paused


def test_SR08_a_plan_not_ready_at_its_swap_row_is_pending_and_never_used(tmp_path):
    """The worker takes 1.0 s (dry-run knob), longer than the 8-row lookahead (0.53 s): every job reaches its swap row
    unfinished -> 'retarget pending', the current plan goes on, the late plans are never sent."""
    s = run_obj(tmp_path, *APPEAR, "--sim-event", "drift30@HANDOVER_APPROACH+1.5", "--dry-retarget-delay-s", "1.0",
                keys="h,l")
    assert s["error"] is None and s["chain_ok"] == (True, "ok")
    st = [r["status"] for r in kinds(s["rows"], "retarget")]
    assert "pending" in st and "swapped" not in st
    pend = [r for r in kinds(s["rows"], "retarget") if r["status"] == "pending"][0]
    assert "retarget pending" in pend["reason"]
    _sr08_never_sent(s, s["runner"])
    assert not [r for r in kinds(s["rows"], "freeze") if r["monitor"] == 5]


def test_SR08_a_plan_that_does_not_start_at_the_held_goal_freezes(tmp_path):
    tap = PlannerTap()

    def shifted(real, plan, palm, q6):
        res = real(plan, palm, q6)
        return dataclasses.replace(res, targets=tuple([r[0] + 3.0] + list(r[1:]) for r in res.targets))
    tap.replace["retarget"] = shifted
    s = run_obj(tmp_path, *APPEAR, "--sim-event", "drift30@HANDOVER_APPROACH+1.5", keys="h,l", planner=tap)
    assert s["error"] is None
    rt = [r for r in kinds(s["rows"], "retarget") if r["status"] == "not_swapped"]
    assert rt and rt[0].get("swapped") is False and 2.5 < rt[0]["first_row_vs_goal_deg"] < 3.5
    prev = None
    for r in s["rows"]:
        if r["kind"] == "step" and r["phase"] == "HANDOVER_APPROACH" and r["target6"]:
            if prev is not None:
                assert abs(r["target6"][0] - prev[0]) <= 1.0               # the 3 deg jump was never commanded
            prev = r["target6"]
    fz = kinds(s["rows"], "freeze")[-1]
    assert fz["phase"] == "HANDOVER_APPROACH" and "deg from the held goal" in fz["detail"]


# ====================================================================== SR09: gates before RETREAT and at [c]
HANDOVER_SIMPLE = ("--sim-event", "appear@APPROACH_ZONE+1.0", "--sim-event", "vanish@RELEASE+1.0")


def _not_clear(h):
    """One hand frame in the last five, none in the latest: monitor 7 does not fire and is not clear."""
    def override(snap):
        if time.monotonic() < h.get("dirty_until", 0.0):
            return {**snap, "window": [True, False, False, False, False], "msg": {**(snap.get("msg") or {}), "hands": []}}
        return snap
    return override


def test_SR09_retreat_after_the_release_wait_is_gated(tmp_path, monkeypatch):
    h = {}
    orig = motion_mod.Motion.hold_rows

    def hold_rows(self, phase, seconds, until=None):
        res = orig(self, phase, seconds, until)
        if res["end"] == "until":
            h["dirty_until"] = time.monotonic() + 1.0
            h["link"].override = _not_clear(h)
        return res
    monkeypatch.setattr(motion_mod.Motion, "hold_rows", hold_rows)
    s = run_obj(tmp_path, *HANDOVER_SIMPLE, keys="h,r,l", link_factory=tap_factory(h))
    assert s["error"] is None
    i = key_index(s["rows"], "r")
    after = s["rows"][i:]
    j = first_index(after, lambda r: r["kind"] == "step" and r["phase"] == "HOLD")
    assert not [r for r in after[j:] if r["kind"] == "step" and r["phase"] == "RETREAT" and r["target6"]]
    fz = [r for r in after if r["kind"] == "freeze"]
    assert fz and fz[0]["phase"] == "RETREAT" and "gate" in str(fz[0]["source"]) and fz[0]["monitor"] == 7


def test_SR09_c_is_refused_unless_all_clear_at_the_key(tmp_path, monkeypatch):
    h = {}
    orig_run, orig_ask = motion_mod.Motion.run_rows, ask_menu.Keys.ask

    def run_rows(self, phase, rows, **kw):
        if phase == "RETREAT" and "esc" not in h:
            h["esc"] = threading.Timer(0.7, lambda: self.rig.kill.trigger("dry:ESC"))
            h["esc"].start()
        return orig_run(self, phase, rows, **kw)

    def ask(self, prompt, default=""):
        ans = orig_ask(self, prompt, default)
        if ans == "c" and "link" in h:
            h["dirty_until"] = time.monotonic() + 1.0
            h["link"].override = _not_clear(h)
        return ans
    monkeypatch.setattr(motion_mod.Motion, "run_rows", run_rows)
    monkeypatch.setattr(ask_menu.Keys, "ask", ask)
    s = run_obj(tmp_path, *HANDOVER_SIMPLE, keys="h,r,c,l", link_factory=tap_factory(h))
    assert s["error"] is None
    i = key_index(s["rows"], "c")
    assert "c" in s["rows"][i]["menu"] and s["rows"][i].get("refused")
    assert targets_after(s["rows"], i) == []                               # nothing moved on that [c]
    assert in_order(trace(s["rows"]), "phase:RETREAT", "FREEZE:6", "key:c", "key:l", "end:ABORTED")


# ====================================================================== SR10: a failed hold is retried and said
def _failing_hold(monkeypatch, fails: int):
    h = {"left": fails}
    orig = safety.SafeBus.hold_present

    def hold_present(self):
        if h["left"] > 0:
            h["left"] -= 1
            raise OSError("fixture: the bus did not answer")
        return orig(self)
    monkeypatch.setattr(safety.SafeBus, "hold_present", hold_present)


def test_SR10_hold_that_fails_three_times_is_logged_and_said_in_capitals(tmp_path, monkeypatch):
    _failing_hold(monkeypatch, 3)
    s = run_obj(tmp_path, *APPEAR, keys="l")
    fz = kinds(s["rows"], "freeze")[0]
    assert fz["held"] is False and fz["hold_attempts"] == 3
    assert "THE HOLD FAILED" in s["out"] and "MAY NOT BE THE PRESENT POSITION" in s["out"]


def test_SR10_hold_that_succeeds_on_the_third_try_control(tmp_path, monkeypatch):
    _failing_hold(monkeypatch, 2)
    s = run_obj(tmp_path, *APPEAR, keys="l")
    fz = kinds(s["rows"], "freeze")[0]
    assert fz["held"] is True and fz["hold_attempts"] == 3 and "THE HOLD FAILED" not in s["out"]


# ====================================================================== SR11: the hours at every key and trial; outputs
def at(hh, mm=0):
    return datetime.datetime(2026, 10, 2, hh, mm, tzinfo=g3.IST)


def test_SR11_a_motion_key_outside_the_hours_is_refused_with_a_row(tmp_path, monkeypatch):
    monkeypatch.setattr(start_checks, "MOTION_HOURS", (6, 22))          # the mechanism, with a window set
    h = {"now": at(12)}
    orig = motion_mod.Motion.freeze

    def freeze(self, *a, **k):
        h["now"] = at(22, 5)
        return orig(self, *a, **k)
    monkeypatch.setattr(motion_mod.Motion, "freeze", freeze)
    s = run_obj(tmp_path, *APPEAR, keys="h,l", now_fn=lambda: h["now"], setup=lambda r: setattr(r, "enforce_hours", True))
    assert s["error"] is None
    i = key_index(s["rows"], "h")
    assert "22:05" in str(s["rows"][i].get("refused")) and targets_after(s["rows"], i) == []
    assert not kinds(s["rows"], "plan")


def test_SR11_a_trial_outside_the_hours_does_not_start(tmp_path, monkeypatch):
    monkeypatch.setattr(start_checks, "MOTION_HOURS", (6, 22))          # the mechanism, with a window set
    h = {"now": at(21, 50)}
    orig = handover_session.HandoverSession.trial_end

    def trial_end(self, outcome, why=""):
        h["now"] = at(22, 1)
        return orig(self, outcome, why)
    monkeypatch.setattr(handover_session.HandoverSession, "trial_end", trial_end)
    s = run_obj(tmp_path, "--trials", "2", script="none", now_fn=lambda: h["now"],
                setup=lambda r: setattr(r, "enforce_hours", True))
    assert s["error"] is None and s["rc"] == 1
    assert [r["outcome"] for r in kinds(s["rows"], "trial_end")] == ["PLACED"]
    ref = kinds(s["rows"], "refused")
    assert ref and ref[0]["reason"] == "motion_hours" and "22:01" in ref[0]["detail"]


def test_SR11_output_folders_under_the_frozen_tree_are_refused(tmp_path, capsys):
    for sub in ("anchor", "phase0", "so101_sim"):
        assert handover_session.frozen_path_refusal(paths.ROOT / sub / "kh_never" / "x")   # the rule first
    assert handover_session.frozen_path_refusal(tmp_path / "ok") is None
    link = tmp_path / "link"
    link.symlink_to(paths.ROOT / "phase0")
    assert handover_session.frozen_path_refusal(link / "kh_never")                         # a symlink into it
    for flag in ("--sessions-dir", "--frame-dir"):
        target = paths.ROOT / "phase0" / "kh_never_written"
        argv = ["--dry-run", "--yes", "--no-esc", "--sessions-dir", str(tmp_path / "s"), "--frame-dir", str(tmp_path / "f")]
        argv[argv.index(flag) + 1] = str(target)
        assert handover_session.main(argv) == 2 and not target.exists()
    assert "REFUSED" in capsys.readouterr().out and not (tmp_path / "s").exists()


# ====================================================================== SR12: a hard abort holds first
def test_SR12_hard_abort_mid_segment_leaves_goals_at_the_present_position(tmp_path, monkeypatch):
    h = {}
    orig = motion_mod.Motion.run_rows

    def run_rows(self, phase, rows, **kw):
        if phase == "GRASP" and "t" not in h:
            h["t"] = threading.Timer(1.5, self.rig.kill.hard_abort.set)
            h["t"].start()
        return orig(self, phase, rows, **kw)
    monkeypatch.setattr(motion_mod.Motion, "run_rows", run_rows)
    s = run_obj(tmp_path, script="none")
    assert s["error"] is None and s["rc"] == 3
    holds = [e for e in s["events"] if e["event"] == "freeze_hold_present"]
    assert holds, "no hold was written before the hard abort"
    last = [r for r in s["rows"] if r["kind"] == "step"][-1]
    assert last["phase"] == "GRASP"
    goals = [holds[-1]["goals"][j] for j in g3.ALL]
    assert max(abs(a - b) for a, b in zip(goals, last["q6"])) < 1.0           # goals = where the arm was read
    assert not any(e["event"] == "torque_off" for e in s["events"])


# ====================================================================== SR13: one planner call at a time
def test_SR13_planner_calls_never_overlap(monkeypatch):
    port = ports.PlannerPort(paths.ROOT, False, None, None)
    live = {"n": 0, "max": 0}

    def slow(*_a, **_k):
        live["n"] += 1
        live["max"] = max(live["max"], live["n"])
        time.sleep(0.05)
        live["n"] -= 1
        return None
    monkeypatch.setattr(port.hp, "plan_handover", slow)
    monkeypatch.setattr(port.hp, "retarget", slow)
    monkeypatch.setattr(port.hp, "plan_reorient", slow)
    ts = [threading.Thread(target=port.plan_handover, args=([0] * 6, (300, 90))),
          threading.Thread(target=port.retarget, args=(None, (300, 90), [0] * 6)),
          threading.Thread(target=port.plan_reorient, args=([0] * 6, (300, 90)))]
    for t in ts:
        t.start()
    for t in ts:
        t.join()
    assert live["max"] == 1


def test_SR13_a_freeze_during_a_retarget_abandons_the_job(tmp_path):
    """A second hand while a re-target job is out (the worker process takes 1.2 s): the arm freezes; the job is
    abandoned with a row; nothing it computed is ever sent; nothing at all is sent after the freeze."""
    s = run_obj(tmp_path, *APPEAR, "--sim-event", "drift30@HANDOVER_APPROACH+1.5", "--sim-event",
                "second@HANDOVER_APPROACH+2.0", "--dry-retarget-delay-s", "1.2", keys="h,l")
    assert s["error"] is None and s["chain_ok"] == (True, "ok")
    fz = kinds(s["rows"], "freeze")[-1]
    assert "second hand" in fz["detail"]
    i = s["rows"].index(fz)
    assert targets_after(s["rows"], i) == []
    rt = kinds(s["rows"], "retarget")
    assert rt and rt[-1]["status"] in ("abandoned", "pending") and not [r for r in rt if r["status"] == "swapped"]
    _sr08_never_sent(s, s["runner"])
