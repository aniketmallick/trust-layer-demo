"""The operator's three changes of 2026-10-03 (after step 6, KH-S20261003T130937). Dry run only.

  1. --handover-trial pre-authorises the handover at trial start: on a hand event the arm still freezes and the judge
     still runs; if the verdict is not UNSAFE and a handle-first plan exists, the approach starts without a key. Any
     refusal or an UNSAFE verdict holds and asks as before. Once per trial; [r] still releases.
  2. After the release, the retreat waits for 5 fresh hand-monitor frames with no hand in the workspace, counted from
     the end of the release (13:17: frames counted during the release let it start with the hand still there).
  3. The pointed end stays >= 150 mm from the palm on every row of every plan.
"""
from __future__ import annotations

import numpy as np

from common import paths

paths.use_anchor()

import monitors as mon_mod  # noqa: E402
from tests.helpers import in_order, run_dry, run_obj, trace  # noqa: E402

PREAUTH = "pre-authorised at trial start (--handover-trial)"
HAND = ["--sim-event", "appear@APPROACH_ZONE+1.0", "--sim-event", "vanish@RELEASE+1.0"]


def kinds(rows, kind):
    return [r for r in rows if r["kind"] == kind]


def accepted_keys(rows):
    return [r for r in kinds(rows, "key") if r["accepted"]]


# ------------------------------------------------------------------------------------------ 1. --handover-trial
def test_PREAUTH_safe_verdict_and_a_plan_proceed_without_a_key(tmp_path):
    s = run_dry(tmp_path, "--handover-trial", *HAND, keys="r", script="none")
    assert s["chain_ok"] == (True, "ok") and s["rc"] == 0
    tr = trace(s["rows"])
    assert in_order(tr, "phase:APPROACH_ZONE", "FREEZE:7", "judge:FREEZE", "key:h", "plan:handover:ok",
                    "phase:HANDOVER_APPROACH", "phase:HOLD", "judge:CP3", "key:r", "phase:RELEASE", "phase:RETREAT",
                    "end:HANDED_OVER"), tr
    keys = accepted_keys(s["rows"])
    assert [k["key"] for k in keys] == ["h", "r"]
    assert keys[0]["by"] == PREAUTH and keys[0]["stop_flag_cleared_at_key"] and "by" not in keys[1]   # [r] is typed
    assert keys[0]["verdict_on_screen"]["unsafe"] is False
    pre = kinds(s["rows"], "preauth")
    assert len(pre) == 1 and pre[0]["trial_id"].endswith("/T001")
    assert kinds(s["rows"], "session_start")[0]["handover_trial"] is True
    rows = s["rows"]                                                       # the freeze still froze: nothing sent
    i = rows.index(kinds(rows, "freeze")[0])
    j = rows.index(keys[0])
    assert all(r["target6"] is None for r in rows[i:j] if r["kind"] == "step")
    assert [r["kind"] for r in rows[i:j]].count("judge") == 1


def test_PREAUTH_without_the_flag_the_same_hand_asks(tmp_path):
    s = run_dry(tmp_path, *HAND, keys="l", script="none")
    keys = accepted_keys(s["rows"])
    assert [k["key"] for k in keys] == ["l"] and "h" in keys[0]["menu"] and not kinds(s["rows"], "preauth")
    assert not kinds(s["rows"], "plan")


def test_PREAUTH_an_unsafe_verdict_holds_and_asks(tmp_path):
    s = run_dry(tmp_path, "--handover-trial", "--dry-judge", "FREEZE=unsafe", *HAND, keys="l", script="none")
    keys = accepted_keys(s["rows"])
    assert [k["key"] for k in keys] == ["l"] and "by" not in keys[0] and "h" in keys[0]["menu"]
    assert not kinds(s["rows"], "plan") and "phase:HANDOVER_APPROACH" not in trace(s["rows"])
    held = kinds(s["rows"], "preauth_held")
    assert len(held) == 1 and "UNSAFE" in held[0]["why"]


def test_PREAUTH_a_planner_refusal_holds_and_asks(tmp_path):
    """The placeholder grasp at the end of the carry: no plan (test_E2E_h_at_the_end_of_the_carry_...)."""
    s = run_dry(tmp_path, "--handover-trial", "--sim-event", "appear@APPROACH_ZONE+3.0", keys="l", script="none")
    keys = accepted_keys(s["rows"])
    assert [k["key"] for k in keys] == ["h", "l"] and keys[0]["by"] == PREAUTH and "by" not in keys[1]
    plans = kinds(s["rows"], "plan")
    assert len(plans) == 1 and plans[0]["plan"]["ok"] is False
    assert "phase:HANDOVER_APPROACH" not in trace(s["rows"])
    assert "was refused" in str(keys[1].get("cause", "")) + str(keys[1].get("note", "")) or kinds(s["rows"], "relatch")


def test_PREAUTH_only_the_first_hand_event_of_a_trial(tmp_path):
    """The 'handover' script: the palm drifts 80 mm during the approved approach - a second freeze. That one asks."""
    s = run_dry(tmp_path, "--handover-trial", keys="h,r", script="handover")
    keys = accepted_keys(s["rows"])
    assert [k["key"] for k in keys] == ["h", "h", "r"]
    assert keys[0]["by"] == PREAUTH and "by" not in keys[1] and "by" not in keys[2]
    assert trace(s["rows"])[-1] == "end:HANDED_OVER"


def test_PREAUTH_the_arm_statement_says_so(tmp_path):
    on = run_obj(tmp_path / "on", "--handover-trial", script="none")["out"]
    off = run_obj(tmp_path / "off", script="none")["out"]
    assert "PRE-AUTHORISED" in on and "WITHOUT a key" in on and "[r] still releases" in on
    assert "PRE-AUTHORISED" not in off


# ------------------------------------------------------------------------------------------ 2. the release wait
def _msg(seq, hand):
    return {"seq": seq, "hands": [{"in_workspace": True, "palm_mm": [250.0, 100.0]}] if hand else []}


def test_WAIT_clear_watch_counts_fresh_frames_only():
    w = mon_mod.ClearWatch()
    assert w.need == 5
    assert not any(w.see(_msg(s, False)) for s in (1, 2, 3, 4))
    assert not w.see(_msg(4, False))                                        # the same frame again: not counted
    assert not w.see(_msg(5, True)) and w.n == 0 and w.resets == 1          # a hand: the count starts again
    assert not any(w.see(_msg(s, False)) for s in (6, 7, 8, 9))
    assert w.see(_msg(10, False)) and w.n == 5
    assert not mon_mod.ClearWatch().see(None)


def test_WAIT_a_hand_that_flickers_after_the_release_holds_the_retreat(tmp_path):
    """13:17 again: the hand drops out of view during the release and comes back just after it. The retreat waits;
    no freeze; it starts after 5 fresh clear frames."""
    s = run_dry(tmp_path, "--handover-trial", "--sim-event", "appear@APPROACH_ZONE+1.0", "--sim-event",
                "vanish@RELEASE+0.0", "--sim-event", "appear@WAIT_CLEAR+0.25", "--sim-event", "vanish@WAIT_CLEAR+1.5",
                keys="r", script="none")
    assert s["chain_ok"] == (True, "ok") and s["rc"] == 0
    rows = s["rows"]
    r_key = next(r for r in accepted_keys(rows) if r["key"] == "r")
    after = rows[rows.index(r_key):]
    assert not kinds(after, "freeze")
    wc = kinds(after, "wait_clear")
    assert len(wc) == 1 and wc[0]["frames"] == 5 and wc[0]["resets"] >= 1 and wc[0]["waited_s"] >= 1.0
    i = after.index(wc[0])
    assert not [r for r in after[:i] if r["kind"] == "step" and r["phase"] == "RETREAT"]
    assert trace(rows)[-1] == "end:HANDED_OVER"


# ------------------------------------------------------------------------------------------ 3. the pointed end
def _dawn_cfg():
    from planner import handover_planner as hp
    from planner.cfg import load_cfg
    from planner.knife import Knife
    tool = Knife(handle_len_mm=59.5, blade_overhang_mm=107.5, handle_width_mm=21.0, axis_sign=1, placeholder=False)
    return hp.with_dawn(load_cfg(), 55.0, standoff_mm=30.0, knife=tool)


def test_POINT_a_row_with_the_pointed_end_under_150_mm_is_refused():
    import json
    from planner import path
    from planner.knife import knife_pose
    cfg = _dawn_cfg()
    assert cfg.point_min_mm == 150.0
    seg = json.loads((paths.EXP / "dawn" / "segments.json").read_text())
    row = next(s["rows"] for s in seg["segments"] if s["state"] == "APPROACH_ZONE")[-1]
    kp = knife_pose(cfg.to_mjcf(row), cfg.knife)
    out = (kp.blade_tip[:2] - kp.grip[:2]) / np.linalg.norm(kp.blade_tip[:2] - kp.grip[:2])
    near = kp.blade_tip[:2] + 100.0 * out                    # 100 mm past the point: the knife segment is 100 away
    got = path.check_rows(cfg, [row], [], near, row)
    assert got["problem"] and "pointed end" in got["problem"] and "150" in got["problem"]
    far = kp.blade_tip[:2] + 160.0 * out
    assert "pointed end" not in str(path.check_rows(cfg, [row], [], far, row)["problem"])        # control


def test_POINT_every_plan_reports_its_closest_pointed_end():
    import json
    from planner import handover_planner as hp
    fz = json.loads((paths.EXP / "tests" / "data" / "rig_poses.json").read_text())["freezes"]["13:22:26"]
    a = hp.plan_handover(fz["q6"], fz["palm_mm"], _dawn_cfg())         # KH-S20261003T130937, extracted with row hashes
    assert a.ok and a.checks["min_point_palm_mm"] >= 150.0


def test_POINT_the_rule_binds_every_measured_tool_and_not_the_rehearsal_placeholder():
    import dataclasses
    import json
    from planner import path
    from planner.knife import knife_pose
    cfg = _dawn_cfg()
    seg = json.loads((paths.EXP / "dawn" / "segments.json").read_text())
    row = next(s["rows"] for s in seg["segments"] if s["state"] == "APPROACH_ZONE")[-1]
    kp = knife_pose(cfg.to_mjcf(row), cfg.knife)
    out = (kp.blade_tip[:2] - kp.grip[:2]) / np.linalg.norm(kp.blade_tip[:2] - kp.grip[:2])
    near = kp.blade_tip[:2] + 100.0 * out
    ph = dataclasses.replace(cfg, knife=dataclasses.replace(cfg.knife, placeholder=True))
    assert "pointed end" not in str(path.check_rows(ph, [row], [], near, row)["problem"])


def test_PREAUTH_rehearsal_with_the_real_tool_segments_and_30_mm(tmp_path):
    """The configuration of the session that follows, as a dry run: the measured screwdriver, the dawn segments,
    --standoff-mm 30, --handover-trial. One hand, no key but [r]; every plan's pointed end >= 150 mm."""
    s = run_dry(tmp_path, "--handover-trial", "--segments", str(paths.EXP / "dawn" / "segments.json"),
                "--z-handover-mm", "55", "--standoff-mm", "30", "--knife-handle-mm", "59.5", "--knife-blade-mm", "107.5",
                "--knife-width-mm", "21", "--knife-axis-sign", "1", *HAND, keys="r", script="none")
    assert s["chain_ok"] == (True, "ok") and s["rc"] == 0, s["out"][-600:]
    assert trace(s["rows"])[-1] == "end:HANDED_OVER"
    keys = accepted_keys(s["rows"])
    assert [k["key"] for k in keys] == ["h", "r"] and keys[0]["by"] == PREAUTH
    plans = [p for p in kinds(s["rows"], "plan") if p["plan"].get("ok")]
    assert plans and all(p["plan"]["checks"]["min_point_palm_mm"] >= 150.0 for p in plans)
    assert all(p["plan"]["checks"]["end_handle_tip_to_palm_mm"] >= 30.0 for p in plans)


def test_PREAUTH_the_replay_says_the_key_was_not_typed(tmp_path):
    import sys
    sys.path.insert(0, str(paths.EXP / "replay"))
    import build_replay
    s = run_dry(tmp_path, "--handover-trial", *HAND, keys="r", script="none")
    lines = [x for x in (build_replay._log_line(r) for r in s["rows"]) if x]
    assert any("NOT TYPED: " + PREAUTH in x for x in lines)
    assert any(x.startswith("handover PRE-AUTHORISED") for x in lines)
    assert any("fresh frames with no hand before the retreat" in x for x in lines)
    assert build_replay.build(s["dir"]).is_file()
