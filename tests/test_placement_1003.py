"""Palm placement (the operator, 2026-10-03): the planner (planner/placement.py) and the runner (placement_flow.py).

  1. the handle over the open palm, the pointed end past the fingertips, down to 20 mm above the palm (+16 for the sag),
     released on a CP3 verdict that is not UNSAFE - no standoff, no [r];
  2. turned automatically when the pointed end faces the hand; the pointed end >= 30 mm from the palm in 3D, every row;
  3. 1 s settle before every judge call on the pre-authorised path;
  4. <= 10 deg/s beyond 150 mm of the palm, <= 3 within and on the descent;
  5. "did you receive it?" logged beside the judge's verdict.
Dry runs use the measured screwdriver and the dawn segments, the palm and the finger direction given.
"""
from __future__ import annotations

import json

import numpy as np
import pytest

from common import paths

paths.use_anchor()

import placement_flow as pf  # noqa: E402
from planner import handover_planner as hp  # noqa: E402
from planner import placement as pl  # noqa: E402
from planner.cfg import load_cfg  # noqa: E402
from planner.knife import Knife, knife_pose  # noqa: E402
from tests.helpers import in_order, run_dry, trace  # noqa: E402

TOOL = Knife(handle_len_mm=59.5, blade_overhang_mm=107.5, handle_width_mm=21.0, axis_sign=1, placeholder=False)
RIG = paths.EXP / "tests" / "data" / "rig_poses.json"      # CP1 / CP2 of the 16:25 handover trials (KH-S20261003T162529)
ZONE_PALM, ZONE_FINGERS = (240.0, 60.0), (1.0, 0.0)          # the near half of the zone, fingers away from the robot
TURN_PALM, TURN_FINGERS = (220.0, -30.0), (0.0, 1.0)         # the spawn box's middle, fingers toward the zone
SQUARES_4_8 = ((241.1, -118.8), (271.1, -118.8), (301.1, -118.8))
PREAUTH = "pre-authorised at trial start (--palm-placement)"
REAL = ["--segments", str(paths.EXP / "dawn" / "segments.json"), "--z-handover-mm", "55", "--knife-handle-mm", "59.5",
        "--knife-blade-mm", "107.5", "--knife-width-mm", "21", "--knife-axis-sign", "1", "--palm-placement"]


def cfg():
    return hp.with_dawn(load_cfg(arm=True), 55.0, standoff_mm=30.0, knife=TOOL)


@pytest.fixture(scope="module")
def poses():
    return {p: v["q6"] for p, v in json.loads(RIG.read_text())["poses"].items()}


def kinds(rows, kind):
    return [r for r in rows if r["kind"] == kind]


# ------------------------------------------------------------------------------------------------ the planner
def test_PLACE_zone_side_handle_over_the_palm_tip_past_the_fingertips(poses):
    c = cfg()
    a = pl.plan_placement(poses["CP2"], ZONE_PALM, ZONE_FINGERS, c)
    assert a.ok, a.reason
    ch = a.checks
    assert abs(ch["release_lowest_mm"] - (pl.PALM_H_MM + pl.RELEASE_MM + pl.SAG_MM)) <= 0.6
    assert ch["handle_middle_from_palm_mm"] <= 1.0                                   # the handle's middle over the palm
    assert ch["pointed_end_from_palm_mm"] >= pl.FINGER_REACH_MM + pl.TIP_PAST_MM      # past the fingertips
    kp = knife_pose(c.to_mjcf(a.targets[-1]), c.knife)
    tip_dir = (kp.blade_tip[:2] - np.asarray(ZONE_PALM)) / np.linalg.norm(kp.blade_tip[:2] - np.asarray(ZONE_PALM))
    assert float(tip_dir @ np.asarray(ZONE_FINGERS)) > np.cos(np.radians(46))       # along the fingers, not the wrist
    assert ch["min_point_palm_3d_mm"] >= pl.POINT_CLEAR_MM and ch["max_step_deg_near"] <= 0.2
    after = a.phases[0]["n_steps"]
    assert all(pl.row_problem(c, r, ZONE_PALM, k >= after) is None for k, r in enumerate(a.targets))
    assert a.phases[-2]["phase"].startswith("descend") and a.kind == "placement"


def test_PLACE_reorient_turns_the_pointed_end_away_from_the_hand(poses):
    c = cfg()
    kp = knife_pose(c.to_mjcf(poses["CP1"]), c.knife)                                # the handle must end toward the wrist:
    assert float(kp.handle_xy @ -np.asarray(TURN_FINGERS)) < 0.0                      # now it heads more than 90 deg away
    a = pl.plan_placement(poses["CP1"], TURN_PALM, TURN_FINGERS, c)
    assert a.ok, a.reason
    assert a.checks["turned_deg"] >= 135.0 and a.checks["min_point_palm_3d_mm"] >= pl.POINT_CLEAR_MM


def test_PLACE_square_8_turns_and_places_square_4_is_out_of_reach(poses):
    """With the screwdriver allowed past the box (20:30), square 8 is served with the fingers toward the robot (a 90 deg
    turn); square 4 stays beyond the arm's top-down reach - refused with the reason."""
    c = cfg()
    a = pl.plan_placement(poses["CP2"], SQUARES_4_8[0], (-1.0, 0.0), c)
    assert a.ok and a.checks["turned_deg"] >= 80.0, a.reason
    for d in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        a = pl.plan_placement(poses["CP2"], SQUARES_4_8[2], d, c)
        assert not a.ok and a.kind == "placement" and a.reason.startswith("no placement for the palm"), d


def test_PLACE_the_20_29_palm_on_the_spawn_side_reorients_180(poses):
    """20:29: the palm at (192, -76), fingers toward the zone: refused then ('the tip would go outside'); now a turn."""
    a = pl.plan_placement(poses["CP2"], (192.0, -76.0), (0.0, 1.0), cfg())
    assert a.ok and a.checks["turned_deg"] >= 150.0, a.reason


def test_PLACE_the_screwdriver_may_leave_the_box_the_fingertip_may_not(poses):
    """The operator, 20:30: "it can go outside, it just needs to reorient". The pointed end past the box edge is allowed;
    the fingertip (G3 monitor 3) stays inside the box and the camera's footprint on every row."""
    c = cfg()
    a = pl.plan_placement(poses["CP2"], SQUARES_4_8[0], (-1.0, 0.0), c)               # square 8, fingers to the robot
    assert a.ok, a.reason
    kp = knife_pose(c.to_mjcf(a.targets[-1]), c.knife)
    assert kp.blade_tip[0] < c.box_xy[0]                                             # the pointed end outside the box
    assert all(pl.path._in_box(c, knife_pose(c.to_mjcf(r), c.knife).tip, 0.0) for r in a.targets)


def test_PLACE_a_plan_that_cannot_be_found_is_refused_within_its_budget(poses):
    import time
    t0 = time.monotonic()
    a = pl.plan_placement(poses["CP2"], (330.0, 100.0), (1.0, 0.0), cfg())          # beyond the top-down reach
    assert not a.ok and "no placement" in a.reason and time.monotonic() - t0 <= pl.PLAN_BUDGET_S + 2.0


def test_PLACE_the_pointed_end_rule_and_the_over_the_hand_rule(poses, monkeypatch):
    c = cfg()
    a = pl.plan_placement(poses["CP2"], ZONE_PALM, ZONE_FINGERS, c)
    last = a.targets[-1]
    assert pl.row_problem(c, last, ZONE_PALM, True) is None                          # control
    monkeypatch.setattr(pl, "POINT_CLEAR_MM", 400.0)
    assert "pointed end" in pl.row_problem(c, last, ZONE_PALM, True)
    monkeypatch.setattr(pl, "POINT_CLEAR_MM", 30.0)
    monkeypatch.setattr(pl, "RELEASE_MM", 30.0)                                      # the same row is now 10 mm too low
    assert "over the hand" in pl.row_problem(c, last, ZONE_PALM, True)
    assert pl.row_problem(c, last, ZONE_PALM, False) is None                        # the rise is not read for it


def test_PLACE_no_hand_direction_no_placement(poses):
    for d in (None, (0.0, 0.0), (float("nan"), 1.0)):
        a = pl.plan_placement(poses["CP2"], ZONE_PALM, d, cfg())
        assert not a.ok and "direction" in a.reason


def test_PLACE_speeds_10_beyond_150_mm_3_within(poses):
    c = cfg()
    q = poses["CP2"]
    kp = knife_pose(c.to_mjcf(q), c.knife)
    way = pl.path.Way((float(kp.grip[0]) - 30.0, float(kp.grip[1])), float(kp.tip[2] - c.table_z_mm) + 10.0, kp.heading_deg)
    seg = [("move", pl.path.subdivide(pl.path.Way((float(kp.grip[0]), float(kp.grip[1])),
                                                  float(kp.tip[2] - c.table_z_mm), kp.heading_deg), way))]
    steps = lambda rows: max(max(abs(r[i] - p[i]) for i in range(5)) for p, r in zip([q] + rows[:-1], rows))  # noqa: E731
    far = pl._build(c, q, (900.0, 900.0), seg)["rows"]
    near = pl._build(c, q, ZONE_PALM, seg)["rows"]
    assert 0.2 < steps(far) <= pl.FAR_DPS / 15 + 1e-6 and steps(near) <= pl.NEAR_DPS / 15 + 1e-6
    assert len(far) < len(near)


# ------------------------------------------------------------------------------------------------ the runner (dry)
def _run(tmp, palm, fingers, *extra, keys="l"):
    return run_dry(tmp, *REAL, "--sim-palm", f"{palm[0]},{palm[1]}", "--sim-fingers-dir", f"{fingers[0]},{fingers[1]}",
                   *extra, keys=keys, script="none")


@pytest.fixture(scope="module")
def zone(tmp_path_factory):
    return _run(tmp_path_factory.mktemp("zone"), ZONE_PALM, ZONE_FINGERS, "--sim-event", "appear@APPROACH_ZONE+1.0",
                "--sim-event", "vanish@RELEASE+1.0")


def test_RUN_zone_side_placement_with_no_key(zone):
    s = zone
    assert s["chain_ok"] == (True, "ok") and s["rc"] == 0
    tr = trace(s["rows"])
    assert in_order(tr, "phase:APPROACH_ZONE", "FREEZE:7", "judge:FREEZE", "key:h", "plan:placement:ok",
                    "phase:PLACE_APPROACH", "phase:PLACE_DESCENT", "phase:HOLD", "judge:CP3", "phase:RELEASE",
                    "phase:RETREAT", "end:HANDED_OVER"), tr
    keys = [r for r in kinds(s["rows"], "key") if r["accepted"]]
    assert keys[0]["key"] == "h" and keys[0]["by"] == PREAUTH and not [k for k in keys[1:] if k["state"] != "END"]
    assert len(kinds(s["rows"], "release_auto")) == 1
    judged = {r["phase"]: r["verdict"]["rubric"] for r in kinds(s["rows"], "judge")}
    assert judged["FREEZE"] == judged["CP3"] == "placement" and judged["CP1"] == "checkpoint"
    start = kinds(s["rows"], "session_start")[0]
    assert start["mode"] == "palm_placement" and start["placement"]["prop"] == "plastic prop"
    assert start["placement"]["point_clear_mm_3d"] == 30.0


def test_RUN_a_settle_of_1_s_before_each_pre_authorised_judge_call(zone):
    rows = zone["rows"]
    for phase in ("FREEZE", "CP3"):
        j = next(i for i, r in enumerate(rows) if r["kind"] == "judge" and r["phase"] == phase)
        st = [r for r in rows[:j] if r["kind"] == "settle"]
        assert st and st[-1]["s"] == 1.0
        k = rows.index(st[-1])
        assert not [r for r in rows[k:j] if r["kind"] in ("key", "plan", "judge")]
    i, j = (rows.index(kinds(rows, "settle")[0]), next(i for i, r in enumerate(rows) if r["kind"] == "judge"
                                                          and r["phase"] == "FREEZE"))
    from datetime import datetime
    assert (datetime.fromisoformat(rows[j]["t_iso"]) - datetime.fromisoformat(rows[i]["t_iso"])).total_seconds() >= 0.9


def test_RUN_the_outcome_label_beside_the_release_verdict(zone):
    lab = kinds(zone["rows"], "outcome_label")
    assert len(lab) == 1 and lab[0]["received"] is True and lab[0]["judge_at_release"]["unsafe"] is False
    assert "CP3 verdict" in lab[0]["released_by"]


def test_RUN_speeds_and_monitor_2(zone):
    rows = zone["rows"]
    assert not [r for r in kinds(rows, "freeze") if r["monitor"] == 2]
    prev = None
    for r in rows:
        if r["kind"] == "step" and r["phase"] in ("PLACE_DESCENT",) and r["target6"]:
            if prev is not None:
                assert max(abs(a - b) for a, b in zip(r["target6"][:5], prev[:5])) <= 3.0 / 15 + 1e-3
            prev = r["target6"]


def test_RUN_the_replay_prints_the_30_mm_and_the_plastic_prop(zone):
    import sys
    sys.path.insert(0, str(paths.EXP / "replay"))
    import build_replay
    page = build_replay.build(zone["dir"]).read_text()
    assert "pointed-end clearance 30.0 mm (3D)" in page and "plastic prop" in page
    lines = [x for x in (build_replay._log_line(r) for r in zone["rows"]) if x]
    assert any(x.startswith("RELEASED WITHOUT A KEY") for x in lines) and any(x.startswith("OUTCOME LABEL") for x in lines)
    assert "PALM PLACEMENT" in zone["out"] and "plastic prop" in zone["out"] and ">= 30 mm" in zone["out"]


def test_RUN_reorient_on_the_spawn_box_turns_and_places(tmp_path):
    s = _run(tmp_path, TURN_PALM, TURN_FINGERS, "--sim-event", "appear@APPROACH_ZONE+0.3", "--sim-event",
             "vanish@RELEASE+1.0")
    assert s["chain_ok"] == (True, "ok") and s["rc"] == 0, s["out"][-500:]
    plan = [p for p in kinds(s["rows"], "plan") if p["plan"].get("ok")][0]
    assert plan["plan"]["checks"]["turned_deg"] >= 135.0 and trace(s["rows"])[-1] == "end:HANDED_OVER"


def test_RUN_a_hand_that_moves_during_the_descent_freezes(tmp_path):
    s = _run(tmp_path, ZONE_PALM, ZONE_FINGERS, "--sim-event", "appear@APPROACH_ZONE+1.0", "--sim-event",
             "drift25@PLACE_DESCENT+1.0", "--sim-drift-dir", "0,1", keys="l")
    fz = [r for r in kinds(s["rows"], "freeze") if r["phase"] == "PLACE_DESCENT"]
    assert fz and fz[0]["monitor"] == 7 and "from the approved palm" in fz[0]["detail"]
    i = s["rows"].index(fz[0])
    assert not [r for r in s["rows"][i:] if r["kind"] == "step" and r["target6"]]   # nothing sent after it
    assert not kinds(s["rows"], "release_auto") and kinds(s["rows"], "trial_end")[0]["outcome"] == "ABORTED"


def test_RUN_unsafe_after_the_settle_asks(tmp_path):
    s = _run(tmp_path, ZONE_PALM, ZONE_FINGERS, "--dry-judge", "FREEZE=unsafe", "--sim-event", "appear@APPROACH_ZONE+1.0")
    held = kinds(s["rows"], "preauth_held")                     # every automatic try (2026-10-04: AUTO_TRIES) held
    assert len(held) == pf.AUTO_TRIES and all("UNSAFE" in h["why"] for h in held) and not kinds(s["rows"], "plan")
    typed = [r for r in kinds(s["rows"], "key") if r["accepted"] and "by" not in r]
    assert typed[0]["key"] == "l"


def test_RUN_unsafe_at_the_release_asks_for_r(tmp_path):
    s = _run(tmp_path, ZONE_PALM, ZONE_FINGERS, "--dry-judge", "CP3=unsafe", "--sim-event", "appear@APPROACH_ZONE+1.0",
             "--sim-event", "vanish@RELEASE+1.0", keys="r")
    assert not kinds(s["rows"], "release_auto") and trace(s["rows"])[-1] == "end:HANDED_OVER"
    assert kinds(s["rows"], "outcome_label")[0]["released_by"] == "[r]"


def test_RUN_no_plan_asks(tmp_path):
    s = _run(tmp_path, (330.0, 100.0), (1.0, 0.0), "--sim-event", "appear@APPROACH_ZONE+1.0")
    plans = kinds(s["rows"], "plan")                             # every automatic try refused, then the menu
    assert len(plans) == pf.AUTO_TRIES and all(p["plan"]["ok"] is False and "no placement" in p["plan"]["reason"]
                                               for p in plans)
    keys = [r for r in kinds(s["rows"], "key") if r["accepted"]]
    assert [k["key"] for k in keys] == ["h"] * pf.AUTO_TRIES + ["l"] and all(k["by"] == PREAUTH for k in keys[:-1])
    assert "phase:PLACE_APPROACH" not in trace(s["rows"])


def test_RUN_a_drift_under_the_arm_is_caught_by_the_descent_rule(tmp_path):
    """20:28 built a re-plan above the palm for a hand that moved in the approach. Since 2026-10-04 the readings under
    the arm are not used (12-73 mm off with the hand still): with the zone palm under the arm for the whole approach
    the still check has only the read before the plan, so a 20 mm move there is not seen above the palm - the descent's
    15 mm rule freezes it, before anything is released (STATUS: a finding, kept as freeze-and-ask)."""
    s = _run(tmp_path, ZONE_PALM, (0.0, -1.0), "--sim-event", "appear@APPROACH_ZONE+1.0", "--sim-event",
             "drift20@PLACE_APPROACH+2.0", "--sim-drift-dir", "0,1", "--sim-event", "vanish@RELEASE+1.0")
    assert s["chain_ok"] == (True, "ok")
    fz = [r for r in kinds(s["rows"], "freeze") if r["phase"] == "PLACE_DESCENT"]
    assert fz and "from the approved palm (>= 15)" in fz[0]["detail"]
    assert not kinds(s["rows"], "release_auto") and "phase:RELEASE" not in trace(s["rows"])


def test_RUN_a_big_drift_freezes_and_h_retries_the_placement(tmp_path):
    """60 mm in the approach (>= 50) freezes; the menu in this mode has no [o] and its [h] retries the placement - a
    settle and a fresh judge first - never the standoff handover."""
    s = _run(tmp_path, ZONE_PALM, (0.0, -1.0), "--sim-event", "appear@APPROACH_ZONE+1.0", "--sim-event",
             "drift60@PLACE_APPROACH+1.0", "--sim-drift-dir", "0,1", "--sim-event", "vanish@RELEASE+1.0", keys="h")
    fz = [r for r in kinds(s["rows"], "freeze") if r["phase"] == "PLACE_APPROACH"]
    assert fz and "from the approved palm" in fz[0]["detail"]
    menu = [r for r in kinds(s["rows"], "key") if r["accepted"] and r["key"] == "h" and "by" not in r]
    assert menu and "o" not in menu[0]["menu"]
    assert all(p["requested"] == "placement" for p in kinds(s["rows"], "plan"))
    i = s["rows"].index(menu[0])
    assert [r["kind"] for r in s["rows"][i:]].index("settle") < [r["kind"] for r in s["rows"][i:]].index("judge")
    assert trace(s["rows"])[-1] == "end:HANDED_OVER"


def test_RUN_placement_with_a_hold_test_is_refused(tmp_path):
    s = run_dry(tmp_path, *REAL, "--hold-test", keys="l", script="none")
    assert s["rc"] == 2 and kinds(s["rows"], "refused")[0]["reason"] == "bad_flags"


# ====================================================================== 2026-10-04: [r] at the freeze menu, the depth
def test_RUN_r_at_the_freeze_menu_releases_then_retreats(tmp_path):
    """The operator's own release where the arm stands (2026-10-04): offered in placement mode once a placement was
    planned; the gripper opens, no lift-off off the descent, the wait for clear frames, the retreat."""
    s = _run(tmp_path, ZONE_PALM, (0.0, -1.0), "--sim-event", "appear@APPROACH_ZONE+1.0", "--sim-event",
             "drift60@PLACE_APPROACH+1.0", "--sim-drift-dir", "0,1", "--sim-event", "vanish@RELEASE+1.0", keys="r")
    assert s["chain_ok"] == (True, "ok") and s["rc"] == 0, s["out"][-600:]
    key = next(r for r in kinds(s["rows"], "key") if r["accepted"] and r["key"] == "r")
    assert key["state"] == "ASK" and "r" in key["menu"] and key.get("stop_flag_cleared_at_key")
    rel = kinds(s["rows"], "release_typed")
    assert len(rel) == 1 and rel[0]["on_descent_row"] is None                       # froze in the approach
    assert in_order(trace(s["rows"]), "FREEZE:7", "key:r", "phase:RELEASE", "phase:RETREAT", "end:HANDED_OVER")
    assert "phase:PLACE_LIFTOFF" not in trace(s["rows"])
    assert "released by [r] at the freeze menu" in kinds(s["rows"], "trial_end")[0]["why"]
    assert kinds(s["rows"], "wait_clear") and kinds(s["rows"], "outcome_label")


def test_RUN_the_judge_is_told_height_as_well_as_horizontal_distance(zone):
    ctx = [r["context"] for r in kinds(zone["rows"], "judge") if r["context"].get("mode") == "placement"]
    assert ctx and all("arm_to_palm_mm" not in c for c in ctx)
    cp3 = [r["context"] for r in kinds(zone["rows"], "judge") if r["phase"] == "CP3"][0]
    assert cp3["horizontal_arm_to_palm_mm"] is not None and cp3["height_above_palm_mm"] >= 15.0
    assert cp3["pointed_end_to_palm_3d_mm"] >= 30.0


def test_MENU_r_and_the_h_text_over_the_palm():
    import types

    import ask_menu
    said = []
    con = types.SimpleNamespace(auto=True, say=said.append, ask=lambda p, d="": "l")
    chain = types.SimpleNamespace(append=lambda row: row)
    keys = ask_menu.Keys(con, ["l"])
    _, row = ask_menu.ask_freeze(keys, chain, "S", "PLACE_APPROACH", "cause", None, offer_continue=False, holding=True,
                                 dry=True, placement=True, offer_release=True, h_text="continue the placement to your palm")
    assert row["menu"] == ["w", "h", "k", "l", "r"] and any("continue the placement to your palm" in x for x in said)
    _, row = ask_menu.ask_freeze(ask_menu.Keys(con, ["l"]), chain, "S", "LIFT", "cause", None, offer_continue=False,
                                 holding=True, dry=True, placement=True, offer_release=False)
    assert "r" not in row["menu"]                                                     # no placement planned yet
    _, row = ask_menu.ask_freeze(ask_menu.Keys(con, ["l"]), chain, "S", "LIFT", "cause", None, offer_continue=False,
                                 holding=False, dry=True, placement=True, offer_release=True)
    assert "r" not in row["menu"]                                                     # nothing held


def test_PLACE_the_plan_squeezes_the_grip_back_to_its_close_target(poses):
    """KH-S20261004T170052: after the first freeze (goals := present) the jaws sat slack on the handle at 12.6 %, load
    0, and the placement carried that reading to the release - the heavy pointed end dipped. With grip_pct the plan
    closes back to the target at 3 %/s in its first rows and holds it; without, the reading is kept as before."""
    c = cfg()
    slack = list(poses["CP2"][:5]) + [12.6]
    a = pl.plan_placement(slack, ZONE_PALM, ZONE_FINGERS, c, grip_pct=9.0)
    assert a.ok, a.reason
    g = [float(r[5]) for r in a.targets]
    assert g[0] < 12.6 and all(x >= y for x, y in zip(g, g[1:])) and min(g) == 9.0
    assert all(abs(x - y) <= pl.SQUEEZE_PCT_PER_ROW + 1e-6 for x, y in zip([12.6] + g, g))
    k = g.index(9.0)
    assert k <= 20 and all(x == 9.0 for x in g[k:]) and k < a.phases[0]["n_steps"]       # closed before it moves on
    b = pl.plan_placement(slack, ZONE_PALM, ZONE_FINGERS, c)
    assert all(float(r[5]) == 12.6 for r in b.targets)
    assert pl.squeeze([[0, 0, 0, 0, 0, 0]], 8.0, 9.0)[0][5] == 8.0                     # never opens
