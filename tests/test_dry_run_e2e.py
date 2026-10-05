"""The runner end to end on the fake bus: each path read back from the session's chain (a DEMONSTRATION rehearsal).

Every scenario is one dry-run session in a temporary tree - FakeFeetechBus with the rig's calibration, synthetic
frames, monitor/sim_feed.py as a separate process, the judge's stub provider, the real planner, the real HandLink -
in real time (a fast clock multiplies the simulated arm's compute into the step time; timing is part of what is read).
"""
from __future__ import annotations

import json

import pytest

from tests.helpers import in_order, run_dry, trace

LIMIT_MS = 89.9


def kinds(rows, kind):
    return [r for r in rows if r["kind"] == kind]


def common_checks(s):
    assert s["chain_ok"] == (True, "ok")
    start = kinds(s["rows"], "session_start")[0]
    assert "DEMONSTRATION" in start["label"] and start["dry_run"] is True and start["judge"]["provider"] == "stub"
    assert start["grasp_segment"]["placeholder"] is True and start["base_guard"] == "not run for this demonstration"
    assert "cert" + "ified" not in json.dumps(s["rows"]).lower()          # never claimed
    steps = kinds(s["rows"], "step")
    for r in steps[:: max(1, len(steps) // 50)]:
        assert set(r["monitors"]) == {str(i) for i in range(1, 9)}
        assert {"k", "t_s", "dt_ms", "phase", "q6", "target6", "tcp_mm", "load", "hb_age_ms", "hand_seq", "frame_seq"} <= set(r)
    end = kinds(s["rows"], "session_end")[0]
    assert end["loop_dt_ms"]["max"] < LIMIT_MS and end["pump_error"] is None
    return end


# ------------------------------------------------------------------------------------------ the M2 path
@pytest.fixture(scope="module")
def handover(tmp_path_factory):
    return run_dry(tmp_path_factory.mktemp("handover"), keys="h,h,r", script="handover")


def test_E2E_handover_path_in_order(handover):
    s = handover
    end = common_checks(s)
    tr = trace(s["rows"])
    assert in_order(tr, "phase:GRASP", "phase:LIFT", "judge:CP1", "phase:APPROACH_ZONE", "FREEZE:7", "judge:FREEZE",
                    "key:h", "plan:handover:ok", "phase:HANDOVER_APPROACH", "retarget", "FREEZE:7", "judge:FREEZE",
                    "key:h", "plan:handover:ok", "phase:HANDOVER_APPROACH", "phase:HOLD", "judge:CP3", "key:r",
                    "phase:RELEASE", "phase:RETREAT", "end:HANDED_OVER"), tr
    assert s["rc"] == 0
    print(f"loop dt {end['loop_dt_ms']}; frame -> hold {end['frame_to_hold_ms_monitor7']} ms")


def test_E2E_freeze_rows_carry_the_monitor_and_the_frame(handover):
    import hashlib
    fz = kinds(handover["rows"], "freeze")
    assert len(fz) == 2 and all(r["monitor"] == 7 and r["held"] for r in fz)
    assert "raw frame" in fz[0]["detail"] and "from the approved palm" in fz[1]["detail"]
    for r in fz:
        jpg = (handover["dir"] / r["photo"]).read_bytes()
        assert hashlib.sha256(jpg).hexdigest() == r["photo_sha256"] == r["frame_sha256"]   # the message's own frame
        assert r["photo_is_the_message_frame"] and 0 < r["t_hold_minus_frame_t_ms"] < 400
        assert r["hands"] and r["hands"][0]["in_workspace"]


def test_E2E_nothing_is_sent_between_a_freeze_and_its_key(handover):
    rows = handover["rows"]
    for i, r in enumerate(rows):
        if r["kind"] != "freeze":
            continue
        j = next(k for k in range(i, len(rows)) if rows[k]["kind"] == "key" and rows[k]["accepted"])
        between = [x for x in rows[i:j] if x["kind"] == "step"]
        assert all(x["target6"] is None for x in between)
        assert [x["kind"] for x in rows[i:j]].count("judge") == 1          # the judge read the frozen scene first


def test_E2E_retarget_is_silent_and_inside_50mm(handover):
    rt = kinds(handover["rows"], "retarget")
    sw = [r for r in rt if r["status"] == "swapped"]
    assert [r["status"] for r in rt][:2] == ["asked", "swapped"] and rt[0]["worker"] == "process"
    assert len(sw) == 1 and sw[0]["plan"]["ok"] and sw[0]["drift_from_approved_mm"] == 30.0
    rows = handover["rows"]
    i = rows.index(sw[0])
    j = max(k for k in range(i) if rows[k]["kind"] == "plan")              # the plan the [h] approved
    assert not any(r["kind"] in ("key", "freeze", "judge") for r in rows[j + 1:i])   # nothing asked on the way


def test_E2E_handover_speed_and_standoff(handover):
    rows = handover["rows"]
    prev = None
    for r in rows:
        if r["kind"] == "step" and r["phase"] == "HANDOVER_APPROACH" and r["target6"]:
            if prev is not None:
                assert max(abs(a - b) for a, b in zip(r["target6"][:5], prev[:5])) <= 3.0 / 15 + 1e-3
            prev = r["target6"]
        elif r["kind"] != "step":
            prev = None
    plans = [r for r in kinds(rows, "plan") if r["plan"].get("ok")]
    for p in plans:
        tip, palm = p["plan"]["handle_tip_mm"], p["plan"]["palm_mm"]
        assert ((tip[0] - palm[0]) ** 2 + (tip[1] - palm[1]) ** 2) ** 0.5 >= 60.0
    keys = [r for r in kinds(rows, "key") if r["accepted"]]
    assert [k["key"] for k in keys] == ["h", "h", "r"] and all(k["verdict_on_screen"]["model"] == "stub" for k in keys)


# ------------------------------------------------------------------------------------------ the other paths
def test_E2E_no_hand_pick_lift_cp1_place_completes(tmp_path):
    s = run_dry(tmp_path, script="none")
    common_checks(s)
    tr = trace(s["rows"])
    assert tr == ["phase:IDLE", "phase:SETUP", "phase:IDLE", "phase:GRASP", "phase:LIFT", "phase:CP1", "judge:CP1",
                  "phase:APPROACH_ZONE", "phase:PRE_PLACE", "phase:CP2", "judge:CP2", "phase:PLACE", "phase:RETREAT",
                  "end:PLACED"], tr
    assert s["rc"] == 0 and not kinds(s["rows"], "freeze")
    keys = kinds(s["rows"], "key")                  # the one key: the end prompt (finding 2), answered with Enter
    assert [(k["state"], k["key"], k["accepted"]) for k in keys] == [("END", "(Enter)", False)]
    assert kinds(s["rows"], "session_end")[0]["torque_off_at_end"] is False


def test_E2E_stale_heartbeat_freezes_monitor_8(tmp_path):
    s = run_dry(tmp_path, keys="l", script="stale")
    common_checks(s)
    fz = kinds(s["rows"], "freeze")
    assert len(fz) == 1 and fz[0]["monitor"] == 8 and fz[0]["phase"] == "APPROACH_ZONE" and "ms old" in fz[0]["detail"]
    assert 300.0 < fz[0]["hb_age_ms"] < 300.0 + 67.0 + 30.0, fz[0]["hb_age_ms"]   # threshold + one control step + slack
    assert in_order(trace(s["rows"]), "FREEZE:8", "judge:FREEZE", "key:l", "end:ABORTED") and s["rc"] == 1
    print(f"last message -> hold: {fz[0]['hb_age_ms']:.0f} ms (monitor 8 fires above 300)")


def test_E2E_second_hand_during_the_approach_freezes(tmp_path):
    s = run_dry(tmp_path, keys="h,l", script="second_hand")
    common_checks(s)
    fz = kinds(s["rows"], "freeze")
    assert len(fz) == 2 and fz[1]["phase"] == "HANDOVER_APPROACH" and "second hand" in fz[1]["detail"]
    assert in_order(trace(s["rows"]), "FREEZE:7", "key:h", "plan:handover:ok", "phase:HANDOVER_APPROACH", "FREEZE:7",
                    "judge:FREEZE", "key:l", "end:ABORTED")
    menu = [r for r in kinds(s["rows"], "key") if r["accepted"]][-1]["menu"]
    assert "c" not in menu                                                  # off the task's path: no [c]


def test_E2E_h_with_two_hands_is_refused_by_code(tmp_path):
    s = run_dry(tmp_path, "--sim-event", "appear@APPROACH_ZONE+1.0", "--sim-event", "second@APPROACH_ZONE+1.0",
                keys="h,l", script="none")
    common_checks(s)
    plans = kinds(s["rows"], "plan")
    assert len(plans) == 1 and plans[0]["plan"]["ok"] is False and plans[0]["refused_by"] == "code"
    assert "2 hands" in plans[0]["plan"]["reason"]
    assert not any(r["kind"] == "step" and r["phase"] == "HANDOVER_APPROACH" for r in s["rows"])
    assert kinds(s["rows"], "judge")[-1]["unsafe"] is True                 # the stub's 0.7 for two hands: UNSAFE


def test_E2E_kill_needs_off_and_yes_then_torque_off_aborted(tmp_path):
    s = run_dry(tmp_path, keys="k", script="handover")
    common_checks(s)
    assert in_order(trace(s["rows"]), "FREEZE:7", "judge:FREEZE", "key:k", "kill", "end:ABORTED") and s["rc"] == 1
    confirm = [r for r in kinds(s["rows"], "key") if r["state"] == "KILL"]
    assert confirm and confirm[0]["key"] == "OFF/YES" and confirm[0]["accepted"]
    ev = (s["dir"] / "safety_events.jsonl").read_text()
    assert '"torque_off"' in ev and kinds(s["rows"], "session_end")[0]["torque_off_by_kill"] is True
    i = next(k for k, r in enumerate(s["rows"]) if r["kind"] == "kill")
    assert not any(r["kind"] == "step" and r["target6"] for r in s["rows"][i:])


def test_E2E_unsafe_judge_at_cp1_freezes_and_asks(tmp_path):
    s = run_dry(tmp_path, "--dry-judge", "CP1=unsafe", keys="l", script="none")
    common_checks(s)
    fz = kinds(s["rows"], "freeze")
    assert len(fz) == 1 and fz[0]["phase"] == "CP1" and fz[0]["code"] == "judge_unsafe" and fz[0]["source"] == "judge:CP1"
    assert in_order(trace(s["rows"]), "phase:LIFT", "judge:CP1", "FREEZE:0", "key:l", "end:ABORTED")
    assert not any(r["kind"] == "step" and r["phase"] == "APPROACH_ZONE" for r in s["rows"])


def test_E2E_malformed_judge_reply_at_cp2_is_unsafe(tmp_path):
    s = run_dry(tmp_path, "--dry-judge", "CP2=malformed", keys="c", script="none")
    common_checks(s)
    j = [r for r in kinds(s["rows"], "judge") if r["phase"] == "CP2"][0]
    assert j["unsafe"] is True and j["verdict"]["reason"] == "malformed reply" and j["verdict"]["recommend"] == "abort"
    # finding 14: [c] past an UNSAFE verdict asks the judge again first; a fresh verdict that is not UNSAFE lets it go on
    assert in_order(trace(s["rows"]), "judge:CP2", "FREEZE:0", "judge:FREEZE", "key:c", "phase:PLACE", "end:PLACED")
    key = [r for r in kinds(s["rows"], "key") if r["key"] == "c" and r["accepted"]][0]
    assert key["verdict_on_screen"]["unsafe"] is True and key["rejudged_after_unsafe"] is True
    assert key["fresh_verdict"]["unsafe"] is False and not key.get("refused")
    assert "the fresh verdict is not UNSAFE" in s["out"]


def test_SR14_c_after_unsafe_is_refused_when_the_fresh_verdict_is_unsafe(tmp_path):
    """Finding 14 (reviewer 2026-10-01): UNSAFE at CP2, [c]: the fresh judge call is UNSAFE too -> [c] refused, the arm
    stays latched (the stop flag was never cleared), nothing is sent; [l] ends it."""
    s = run_dry(tmp_path, "--dry-judge", "CP2=malformed,FREEZE=unsafe", keys="c,l", script="none")
    common_checks(s)
    key = [r for r in kinds(s["rows"], "key") if r["key"] == "c" and r["accepted"]][0]
    assert "fresh judge call is UNSAFE" in key["refused"] and key["fresh_verdict"]["unsafe"] is True
    assert not key.get("stop_flag_cleared_at_key")
    i = s["rows"].index(key)
    assert not [r for r in s["rows"][i:] if r["kind"] == "step" and r["target6"]]
    assert in_order(trace(s["rows"]), "judge:CP2", "FREEZE:0", "judge:FREEZE", "key:c", "key:l", "end:ABORTED")
    assert not any(r["kind"] == "step" and r["phase"] == "PLACE" for r in s["rows"])


def test_E2E_hand_enters_and_leaves_wait_then_continue(tmp_path):
    s = run_dry(tmp_path, keys="w,c", script="enter_leave")
    common_checks(s)
    acc = [r for r in kinds(s["rows"], "key") if r["accepted"]]
    assert [k["key"] for k in acc] == ["w", "c"]
    assert "c" not in acc[0]["menu"] and "c" in acc[1]["menu"]            # [c] only once all eight are clear
    assert in_order(trace(s["rows"]), "FREEZE:7", "judge:FREEZE", "key:w", "judge:FREEZE", "key:c",
                    "phase:APPROACH_ZONE", "judge:CP2", "phase:PLACE", "end:PLACED")


def test_E2E_blade_toward_the_hand_h_refused_then_reorient(tmp_path):
    s = run_dry(tmp_path, keys="h,o,r", script="reorient")
    common_checks(s)
    plans = kinds(s["rows"], "plan")
    assert plans[0]["plan"]["ok"] is False and "[o]" in plans[0]["plan"]["reason"]
    assert in_order(trace(s["rows"]), "FREEZE:7", "key:h", "plan:handover:none", "key:o", "plan:reorient:ok",
                    "phase:REORIENT", "plan:handover:ok", "phase:HANDOVER_APPROACH", "phase:HOLD", "judge:CP3", "key:r",
                    "phase:RELEASE", "phase:RETREAT", "end:HANDED_OVER")
    assert len(kinds(s["rows"], "freeze")) == 1


def test_E2E_h_at_the_end_of_the_carry_is_refused_by_the_planner(tmp_path):
    """The placeholder grasp puts the handle tip outside the workspace box on the last carry rows: the planner has
    no plan from there. [h] -> the reason, a row, back to ASK, no motion."""
    s = run_dry(tmp_path, "--sim-event", "appear@APPROACH_ZONE+3.0", keys="h,l", script="none")
    common_checks(s)
    plans = kinds(s["rows"], "plan")
    assert len(plans) == 1 and plans[0]["plan"]["ok"] is False and plans[0]["plan"]["reason"]
    assert not any(r["kind"] == "step" and r["phase"] in ("HANDOVER_APPROACH", "REORIENT") for r in s["rows"])
    i = s["rows"].index(plans[0])
    assert not any(r["kind"] == "step" and r["target6"] for r in s["rows"][i:])
    assert in_order(trace(s["rows"]), "key:h", "plan:handover:none", "key:l", "end:ABORTED")
    print("planner refusal:", plans[0]["plan"]["reason"][:160])
