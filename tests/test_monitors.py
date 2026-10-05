"""monitors.py and state_machine.py: each rule seen to fire and seen not to fire (no bus, no camera)."""
from __future__ import annotations

import json

import pytest

from common import paths

paths.use_anchor()
import g3  # noqa: E402

import grasp_segment  # noqa: E402
import monitors as m  # noqa: E402
import state_machine as sm  # noqa: E402


def hand(x=300.0, y=90.0, inside=True):
    return {"handedness": "R", "conf": 0.9, "palm_px": [1.0, 2.0], "palm_mm": [x, y], "in_workspace": inside,
            "in_spawn": False, "in_zone": False}


def msg(*hands):
    return {"seq": 5, "frame_seq": 9, "hands": list(hands)}


@pytest.fixture(scope="module")
def guard():
    reg = json.loads((paths.ROOT / paths.REG_REL).read_text())
    rec = json.loads((paths.ROOT / paths.RECORD_REL).read_text())
    return g3.guard_from_record(paths.ROOT, reg, rec)


# ------------------------------------------------------------------------------------------ monitor 7
def test_M7_no_hand_does_not_fire_control():
    r = m.monitor7(msg(), [False] * 5, "APPROACH_ZONE")
    assert not r["fired"] and r["clear"]


def test_M7_hand_outside_the_box_does_not_fire_control():
    r = m.monitor7(msg(hand(inside=False)), [False] * 5, "APPROACH_ZONE")
    assert not r["fired"] and r["clear"] and r["n_hands"] == 1


def test_M7_one_raw_frame_fires_while_moving():
    r = m.monitor7(msg(hand()), [False, False, False, False, True], "APPROACH_ZONE")
    assert r["fired"] and "raw frame" in r["detail"]


def test_M7_three_of_five_fires_with_the_latest_frame_empty():
    r = m.monitor7(msg(), [True, True, True, False, False], "LIFT")
    assert r["fired"] and "3 of the last 5" in r["detail"]


def test_M7_one_old_frame_does_not_fire_but_is_not_clear():
    r = m.monitor7(msg(), [True, False, False, False, False], "LIFT")
    assert not r["fired"] and not r["clear"]
    assert sm.may_start_moving(r["clear"], False) is not None
    assert sm.may_start_moving(True, False) is None and sm.may_start_moving(True, True) is not None


def test_M7_stationary_states_do_not_fire_control():
    for phase in ("HOLD", "ASK", "CP1", "IDLE"):
        r = m.monitor7(msg(hand()), [True] * 5, phase)
        assert not r["fired"] and r["present"] and not r["clear"]


def test_M7_approved_palm_within_50mm_does_not_fire_control():
    for phase in ("HANDOVER_APPROACH", "REORIENT"):
        r = m.monitor7(msg(hand(300, 120)), [True] * 5, phase, approved_palm=[300, 90])
        assert not r["fired"] and r["drift_mm"] == 30.0


def test_M7_drift_is_from_the_approved_palm_and_fires_at_50mm():
    assert not m.monitor7(msg(hand(300, 139.9)), [True] * 5, "HANDOVER_APPROACH", [300, 90])["fired"]
    r = m.monitor7(msg(hand(300, 140.0)), [True] * 5, "HANDOVER_APPROACH", [300, 90])
    assert r["fired"] and "50" in r["detail"]


def test_M7_second_hand_during_the_approach_fires():
    r = m.monitor7(msg(hand(300, 90), hand(250, -40)), [True] * 5, "HANDOVER_APPROACH", [300, 90])
    assert r["fired"] and "second hand" in r["detail"]


def test_M7_approved_hand_gone_fires_but_one_dropped_frame_does_not():
    assert not m.monitor7(msg(), [True, True, True, True, False], "HANDOVER_APPROACH", [300, 90])["fired"]
    assert m.monitor7(msg(), [True, False, False, False, False], "HANDOVER_APPROACH", [300, 90])["fired"]


def test_M7_release_one_hand_does_not_fire_control_two_hands_fire():
    """SR05: RELEASE is exempt for the one hand taking the handle; a second hand in the message fires."""
    assert not m.monitor7(msg(hand(300, 150)), [True] * 5, "RELEASE")["fired"]
    assert not m.monitor7(msg(), [False] * 5, "RELEASE")["fired"]
    r = m.monitor7(msg(hand(300, 90), hand(250, -40)), [True] * 5, "RELEASE")
    assert r["fired"] and "second hand" in r["detail"]


def test_M7_non_finite_palm_in_an_approved_phase_fires():
    """SR07: a hand whose palm_mm is not two finite numbers cannot be measured against the approved palm."""
    for bad in ([float("nan"), 90.0], [300.0, float("inf")], [300.0], None, ["a", "b"]):
        for phase in ("HANDOVER_APPROACH", "REORIENT"):
            h = {**hand(), "palm_mm": bad}
            r = m.monitor7(msg(h), [True] * 5, phase, approved_palm=[300, 90])
            assert r["fired"] and "finite" in r["detail"], (bad, phase)
    assert not m.monitor7(msg(hand(300, 100)), [True] * 5, "HANDOVER_APPROACH", approved_palm=[300, 90])["fired"]


def test_M7_without_an_approval_a_hand_fires_in_the_approach_phase():
    assert m.monitor7(msg(hand()), [True], "HANDOVER_APPROACH", None)["fired"]


# ------------------------------------------------------------------------------------------ monitors 1-6, 8
def rest():
    return grasp_segment.load()["rest"]


def test_EVAL_nominal_step_fires_nothing_control(guard):
    q = rest()
    mon = m.evaluate(guard, 1.0, q, q, 1 / 15, 28, "GRASP", False, None, {"msg": msg(), "window": [False] * 5},
                     (False, "ok"))
    assert set(mon) == {str(i) for i in range(1, 9)} and m.fired_list(mon) == [] and m.all_clear(mon)


def test_EVAL_each_g3_term_fires_under_its_number(guard):
    q = rest()
    snap, ok8 = {"msg": msg(), "window": []}, (False, "ok")
    hi = guard.limits()["wrist_flex"][1]
    bad = list(q)
    bad[3] = hi + 1.0
    assert m.fired_list(m.evaluate(guard, 1, bad, None, None, 28, "GRASP", False, None, snap, ok8))[0][:2] == (1, "joint_margin")
    moved = list(q)
    moved[1] += 3.0                                            # 45 deg/s against the 30 deg/s scripted limit
    assert (2, "joint_speed") in [f[:2] for f in m.fired_list(m.evaluate(guard, 1, moved, q, 1 / 15, 28, "GRASP", False, None, snap, ok8))]
    out = list(q)
    out[0] += 70.0
    assert (3, "ee_workspace") in [f[:2] for f in m.fired_list(m.evaluate(guard, 1, out, None, None, 28, "GRASP", False, None, snap, ok8))]
    guard._grip_over_since = None
    m.evaluate(guard, 1.0, q, None, None, 400, "GRASP", False, None, snap, ok8)
    assert (4, "gripper_overload") in [f[:2] for f in m.fired_list(m.evaluate(guard, 1.6, q, None, None, 400, "GRASP", False, None, snap, ok8))]
    guard._grip_over_since = None
    assert (5, "loop_dt") in [f[:2] for f in m.fired_list(m.evaluate(guard, 1, q, q, 0.120, 28, "GRASP", False, None, snap, ok8))]


def test_EVAL_speed_limit_is_scaled_per_phase(guard):
    q = rest()
    moved = list(q)
    moved[1] += 1.2                                            # 18 deg/s: inside 30 (scripted), outside 15 (handover)
    snap, ok8 = {"msg": msg(), "window": []}, (False, "ok")
    assert not m.fired_list(m.evaluate(guard, 1, moved, q, 1 / 15, 28, "APPROACH_ZONE", False, None, snap, ok8))
    fired = m.fired_list(m.evaluate(guard, 1, moved, q, 1 / 15, 28, "HANDOVER_APPROACH", False, None, snap, ok8, approved_palm=None))
    assert (2, "joint_speed") in [f[:2] for f in fired]
    slow = list(q)
    slow[1] += 0.2                                             # the commanded 3 deg/s
    assert (2, "joint_speed") not in [f[:2] for f in m.fired_list(
        m.evaluate(guard, 1, slow, q, 1 / 15, 28, "REORIENT", False, None, snap, ok8))]


def test_EVAL_kill_switch_and_link_fire_under_6_and_8(guard):
    q = rest()
    snap = {"msg": msg(), "window": []}
    assert m.fired_list(m.evaluate(guard, 1, q, q, 1 / 15, 28, "LIFT", True, "ESC", snap, (False, "ok")))[0][0] == 6
    assert m.fired_list(m.evaluate(guard, 1, q, q, 1 / 15, 28, "LIFT", False, None, snap, (False, "ok"), heartbeat_due=True))[0][0] == 6
    assert m.fired_list(m.evaluate(guard, 1, q, q, 1 / 15, 28, "LIFT", False, None, snap, (True, "last hand message 320 ms old")))[0][0] == 8
    latched = m.evaluate(guard, 1, q, q, 1 / 15, 28, "LIFT", True, "monitor8:last hand message 305 ms old", snap, (False, "ok"))
    assert [f[0] for f in m.fired_list(latched)] == [8]        # the watchdog's stop stays named after a late message


# ------------------------------------------------------------------------------------------ state machine
def test_SM_task_order_and_freeze_from_every_moving_state():
    s = "IDLE"
    for ev, want in (("start", "GRASP"), ("done", "LIFT"), ("done", "CP1"), ("safe", "APPROACH_ZONE"), ("done", "PRE_PLACE"),
                     ("done", "CP2"), ("safe", "PLACE"), ("done", "RETREAT"), ("done", "IDLE")):
        s, why = sm.next_state(s, ev)
        assert (s, why) == (want, None)
    for st in sm.MOVING:
        assert sm.next_state(st, "fire") == ("FREEZE", None)
    assert sm.next_state("CP1", "unsafe") == ("FREEZE", None) and sm.next_state("CP2", "unsafe") == ("FREEZE", None)


def test_SM_handover_path_and_keys():
    path = [("FREEZE", "judged", "ASK"), ("ASK", "key:h", "HANDOVER_PLAN"), ("HANDOVER_PLAN", "plan_ok", "HANDOVER_APPROACH"),
            ("HANDOVER_APPROACH", "done", "HOLD"), ("HOLD", "settled", "CP3"), ("CP3", "safe", "ASK_RELEASE"),
            ("ASK_RELEASE", "key:r", "RELEASE"), ("RELEASE", "done", "RETREAT"), ("ASK", "key:o", "REORIENT"),
            ("REORIENT", "done", "HANDOVER_PLAN"), ("HANDOVER_PLAN", "plan_none", "ASK"), ("ASK", "key:w", "WAIT"),
            ("WAIT", "judged", "ASK"), ("ASK", "key:k", "KILL"), ("ASK", "key:l", "LEAVE")]
    for s, ev, want in path:
        assert sm.next_state(s, ev) == (want, None)


def test_SM_unknown_events_never_move_the_state():
    for s, ev in (("ASK", "key:x"), ("HOLD", "key:r"), ("IDLE", "done"), ("ASK", "safe"), ("FREEZE", "key:h")):
        nxt, why = sm.next_state(s, ev)
        assert nxt == s and why


def test_SM_continue_only_when_all_clear_and_there_is_a_segment():
    assert sm.next_state("ASK", "key:c", resume="APPROACH_ZONE", all_clear=True) == ("APPROACH_ZONE", None)
    assert sm.next_state("ASK", "key:c", resume="APPROACH_ZONE", all_clear=False)[0] == "ASK"
    assert sm.next_state("ASK", "key:c", resume=None, all_clear=True)[0] == "ASK"


def test_SM_handover_refusals():
    assert sm.handover_refusal(1, False, False, True) is None
    assert "no hand" in sm.handover_refusal(0, False, False, None)
    assert "2 hands" in sm.handover_refusal(2, False, False, None)
    assert "monitor 8" in sm.handover_refusal(1, True, False, None)
    assert "[o]" in sm.handover_refusal(1, False, True, None)
    assert "no plan" in sm.handover_refusal(1, False, False, False, "the palm is outside the workspace box")


# ------------------------------------------------------------------------------------------ the placeholder segment
def test_SEG_placeholder_speeds_and_frozen_source(guard):
    from motion import segment_problems
    d = grasp_segment.load()
    assert d["placeholder"] and d["source"]["sha256"] == grasp_segment.NULL_PLAN_SHA and d["source"]["cross"] == 6
    assert [s["state"] for s in d["segments"]] == ["GRASP", "LIFT", "APPROACH_ZONE", "PRE_PLACE", "PLACE", "RETREAT"]
    prev = d["rest"]
    for s in d["segments"]:
        arm, grip = grasp_segment.max_step(s["rows"], prev)
        assert arm <= grasp_segment.SCRIPTED_DPS / 15 + 1e-3 and grip <= grasp_segment.GRIP_PPS / 15 + 1e-3, s["state"]
        assert segment_problems(guard, s["rows"]) == [], s["state"]
        prev = s["rows"][-1]
    assert max(abs(a - b) for a, b in zip(prev[:5], d["rest"][:5])) < 1e-3


def test_SEG_fk_check_refuses_a_low_or_outside_row(guard):
    from motion import segment_problems
    d = grasp_segment.load()
    low = list(d["segments"][0]["rows"][-1])
    low[1] += 12.0                                             # shoulder lift pushed down at the grasp pose
    assert segment_problems(guard, [low])
    out = list(d["rest"])
    out[0] += 70.0
    assert any("workspace box" in p for p in segment_problems(guard, [out]))
    assert segment_problems(guard, [out], box_on=False) == []
