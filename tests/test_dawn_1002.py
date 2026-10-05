"""The dawn of 2026-10-02: the recorded segments for the screwdriver (prop tool), their loader, the --arm gate on them,
and the hold test on the fake bus. Dry run only."""
from __future__ import annotations

import json

import pytest

from common import paths

paths.use_anchor()

import grasp_segment  # noqa: E402
from tests.helpers import run_obj  # noqa: E402
from tests.test_arm_refusals import PORT, at, run_arm  # noqa: E402

SEGMENTS = paths.EXP / "dawn" / "segments.json"
CLOSE = json.loads(SEGMENTS.read_text())["close_pct"]          # the operator sets it (8.6 % since 2026-10-04)


def test_DAWN_segments_file_loads_and_an_edited_one_is_refused(tmp_path):
    seg = grasp_segment.load_file(SEGMENTS)
    assert seg["placeholder"] is False and seg["speeds"]["arm_dps"] == 3.0 and seg["hold_test"]
    d = json.loads(SEGMENTS.read_text())
    d["segments"][0]["rows"][3][0] += 0.5                                 # one joint of one row moved half a degree
    bad = tmp_path / "segments.json"
    bad.write_text(json.dumps(d))
    with pytest.raises(ValueError, match="sha256"):
        grasp_segment.load_file(bad)


def test_DAWN_every_row_at_3dps_and_the_close_target(tmp_path):
    seg = grasp_segment.load_file(SEGMENTS)
    prev = seg["rest"]
    for s in seg["segments"]:
        arm, grip = grasp_segment.max_step(s["rows"], prev)
        assert arm <= 3.0 / 15 + 1e-6 and grip <= 3.0 / 15 + 1e-6, (s["state"], arm, grip)
        prev = s["rows"][-1]
    grasp = next(s["rows"] for s in seg["segments"] if s["state"] == "GRASP")
    assert abs(grasp[-1][5] - seg["close_pct"]) < 1e-6 and seg["close_pct"] == 10.5
    assert "KH-S20261002T145204" in seg["close_source"]                 # the target says where it came from


def test_DAWN_arm_without_the_segments_file_is_refused(tmp_path):
    rc, rows, _ = run_arm(tmp_path, "--port", PORT, "--z-handover-mm", "55", now=at(10), segments=False)
    assert rc == 2 and rows[-1]["reason"] == "segments_not_recorded"


def test_DAWN_hold_test_rehearsal_closes_lifts_holds_and_asks(tmp_path):
    s = run_obj(tmp_path, "--segments", str(SEGMENTS), "--hold-test", script="none")
    assert s["error"] is None and s["chain_ok"] == (True, "ok")
    start = [r for r in s["rows"] if r["kind"] == "session_start"][0]
    assert start["scripted_dps"] == 3.0 and start["monitors"]["vel_limit_scripted_dps"] == 15.0
    ht = [r for r in s["rows"] if r["kind"] == "hold_test"]
    assert ht and ht[0]["attempt"] == 1 and ht[0]["close_pct"] == CLOSE and ht[0]["gripper_load"]["n"] >= 140
    phases = [r["phase"] for r in s["rows"] if r["kind"] == "step" and r.get("target6")]
    assert "GRASP" in phases and "LIFT" in phases
    # 20:38: the row said 'lift_mm 20' while the encoders showed ~6.8 mm. Both are logged now; the measured one is the
    # planner FK of the encoder reads (the fake bus's deadband stops it short each way: ~13 mm, not the 20 sent)
    from planner import ik
    z = lambda q: float(ik.tip_and_tilt(q[:5])[0][2])                    # noqa: E731
    steps = [r for r in s["rows"] if r["kind"] == "step"]
    grasp_end = [r for r in steps if r["phase"] == "GRASP"][-1]["q6"]
    hold_end = [r for r in steps if r["phase"] == "HOLD"][-1]["q6"]
    assert ht[0]["lift_mm_commanded"] == 20.0 and "lift_mm" not in ht[0] and ht[0]["lift_measured_by"]
    assert abs(ht[0]["lift_mm_measured"] - (z(hold_end) - z(grasp_end))) < 0.15
    assert 5.0 < ht[0]["lift_mm_measured"] < 20.0                          # measured, not the commanded number copied


def test_DAWN_hold_test_statement_says_what_will_move(tmp_path):
    """14:52: the statement said '<= 10 deg/s' and 'a placeholder segment' while the file ran at 3 deg/s. It is now
    built from the session's own numbers."""
    s = run_obj(tmp_path, "--segments", str(SEGMENTS), "--hold-test", script="none")
    out = s["out"]
    assert "HOLD TEST from the dawn-recorded segments" in out and f"close to {CLOSE:g} %" in out
    assert "<= 3 deg/s" in out and "gripper <= 3 %/s" in out and "No handover in a hold test" in out
    assert "10 deg/s" not in out and "PLACEHOLDER" not in out and "placeholder segment" not in out


def test_DAWN_hold_test_freeze_menu_offers_no_handover(tmp_path):
    """14:52: [h] and [o] were offered at a hold-test freeze. A hold test has no handover: not offered."""
    s = run_obj(tmp_path, "--segments", str(SEGMENTS), "--hold-test", "--sim-event", "appear@LIFT+0.5", keys="l",
                script="none")
    menus = [r["menu"] for r in s["rows"] if r["kind"] == "key"]
    assert menus and all("h" not in m and "o" not in m for m in menus)
    assert all(k in menus[0] for k in ("w", "k", "l"))                    # control: the rest of the menu is there


def test_DAWN_standoff_floor_is_30_by_the_operator_and_the_default_stays_60():
    """2026-10-03, after step 6 (KH-S20261003T130937): at 80 mm 1 of 29 handover requests planned; the operator lowered
    the standoff to 30 mm. The floor is one named constant; below it is refused at the planner and at its port."""
    import ports
    from planner import cfg as pcfg
    assert pcfg.STANDOFF_MIN_MM == 30.0 and pcfg.load_cfg().standoff_mm == 60.0
    assert pcfg.with_dawn(pcfg.load_cfg(), 55.0, standoff_mm=30.0).standoff_mm == 30.0
    for bad in (29.9, 20.0):
        with pytest.raises(ValueError, match="30 mm minimum"):
            pcfg.with_dawn(pcfg.load_cfg(), 55.0, standoff_mm=bad)
        with pytest.raises(ValueError, match="30 mm minimum"):
            ports.PlannerPort(paths.ROOT, False, None, bad)
    assert ports.PlannerPort(paths.ROOT, False, None, 30.0).cfg.standoff_mm == 30.0      # control


def test_DAWN_a_palm_from_step_6_plans_at_30_not_at_80():
    """The first refused [h] of 13:22 (palm (271.6, 104.8), carry frozen): refused at 80, planned at 30; the handle tip
    lands no nearer the palm than the standoff it was planned for."""
    import dataclasses
    import json as _json
    from planner import handover_planner as hp
    from planner.cfg import load_cfg
    from planner.knife import Knife, knife_pose
    fz = _json.loads((paths.EXP / "tests" / "data" / "rig_poses.json").read_text())["freezes"]["13:22:26"]
    q, palm = fz["q6"], fz["palm_mm"]                                  # KH-S20261003T130937, extracted with row hashes
    tool = Knife(handle_len_mm=59.5, blade_overhang_mm=107.5, handle_width_mm=21.0, axis_sign=1, placeholder=False)
    cfg = hp.with_dawn(load_cfg(), 55.0, standoff_mm=30.0, knife=tool)
    assert not hp.plan_handover(q, palm, dataclasses.replace(cfg, standoff_mm=80.0)).ok
    a = hp.plan_handover(q, palm, cfg)
    assert a.ok
    tip = knife_pose(cfg.to_mjcf(a.targets[-1]), tool).handle_tip
    assert ((tip[0] - palm[0]) ** 2 + (tip[1] - palm[1]) ** 2) ** 0.5 >= 30.0
