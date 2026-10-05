"""The reviewer's rulings of 2026-10-01 that change behaviour: the dawn speed (2), the measured prop on the arm (5),
the hours before every moving state (11). Dry run only; the --arm cases return before a port or a camera is touched."""
from __future__ import annotations

import pytest

from common import paths

paths.use_anchor()
import g3  # noqa: E402

import grasp_segment  # noqa: E402
import handover_flow  # noqa: E402
import handover_session  # noqa: E402
import monitors as mon_mod  # noqa: E402
import start_checks  # noqa: E402
from tests.helpers import in_order, run_obj, trace  # noqa: E402
from tests.test_arm_refusals import PORT, at, run_arm  # noqa: E402


def _args(*argv):
    return handover_session.build_parser().parse_args(list(argv))


# ====================================================================== ruling 2: 3 deg/s on the arm until seen at speed
def test_R2_scripted_speed_is_3_on_the_arm_10_in_a_dry_run_never_above_10():
    assert handover_session.scripted_dps(_args("--arm")) == 3.0
    assert handover_session.scripted_dps(_args()) == 10.0                                  # control: the rehearsal
    assert handover_session.scripted_dps(_args("--arm", "--scripted-dps", "10")) == 10.0   # the operator's explicit step
    for bad in ("12", "0", "-3"):
        with pytest.raises(ValueError):
            handover_session.scripted_dps(_args("--arm", "--scripted-dps", bad))


def test_R2_segments_at_3dps_never_step_above_it_and_monitor2_follows():
    seg = grasp_segment.load(paths.ROOT, 1.31, 3.0)
    start = seg["rest"]
    for s in seg["segments"]:
        arm, _ = grasp_segment.max_step(s["rows"], start)
        assert arm <= 3.0 / grasp_segment.CONTROL_HZ + 1e-6, (s["state"], arm)
        start = s["rows"][-1]
    assert seg["speeds"]["arm_dps"] == 3.0 and "3 deg/s" in seg["label"]
    assert mon_mod.scripted_vel_limit(3.0) == 15.0 and mon_mod.scripted_vel_limit(10.0) == 30.0


# ====================================================================== ruling 5: the prop measured before --arm
def test_R5_arm_without_the_prop_measured_is_refused_with_a_row(tmp_path):
    rc, rows, out = run_arm(tmp_path, "--port", PORT, "--z-handover-mm", "45", now=at(9), knife=False)
    assert rc == 2 and rows[-1]["kind"] == "refused" and rows[-1]["reason"] == "knife_not_measured"
    assert "--knife-handle-mm" in rows[-1]["detail"] and "--knife-axis-sign" in rows[-1]["detail"] and len(rows) == 1


def test_R5_arm_with_the_dimensions_but_no_axis_sign_is_refused(tmp_path):
    """2026-10-02, the screwdriver: which way the free handle points is set at the grasp; without it, refused."""
    rc, rows, out = run_arm(tmp_path, "--port", PORT, "--z-handover-mm", "45", "--knife-handle-mm", "59.5",
                            "--knife-blade-mm", "107.5", "--knife-width-mm", "21", now=at(9), knife=False)
    assert rc == 2 and rows[-1]["reason"] == "knife_not_measured" and rows[-1]["detail"].count("--knife-") == 1


def test_R5_part_of_the_dimensions_is_refused(tmp_path):
    with pytest.raises(ValueError):
        handover_session.knife_dims(_args("--knife-handle-mm", "95"))
    assert handover_session.knife_dims(_args()) is None                                    # control: the dry run's placeholder
    assert handover_session.knife_dims(_args("--knife-handle-mm", "95", "--knife-blade-mm", "85", "--knife-width-mm",
                                             "18")) == {"handle_len_mm": 95.0, "blade_overhang_mm": 85.0, "handle_width_mm": 18.0}


# ====================================================================== ruling on 11: the hours before every moving state
def test_SR11_the_clock_passing_2200_between_two_segments_freezes_before_any_target(tmp_path, monkeypatch):
    """The trial starts at 21:58; the clock passes 22:00 during the CP1 judge call. The next moving state
    (APPROACH_ZONE) is not started: a freeze row names the hours, nothing is sent; [c] is refused for the hours; [l]."""
    monkeypatch.setattr(start_checks, "MOTION_HOURS", (6, 22))          # the mechanism, with a window set
    h = {"now": at(21, 58)}
    orig = handover_flow.Flow.judge_at

    def judge_at(self, phase, quiet=False):
        v = orig(self, phase, quiet)
        if phase == "CP1":
            h["now"] = at(22, 1)
        return v
    monkeypatch.setattr(handover_flow.Flow, "judge_at", judge_at)
    s = run_obj(tmp_path, keys="c,l", script="none", now_fn=lambda: h["now"],
                setup=lambda r: setattr(r, "enforce_hours", True))
    assert s["error"] is None and s["chain_ok"] == (True, "ok")
    fz = [r for r in s["rows"] if r["kind"] == "freeze"]
    assert fz and fz[0]["code"] == "motion_hours" and fz[0]["phase"] == "APPROACH_ZONE" and "22:01" in fz[0]["detail"]
    i = s["rows"].index(fz[0])
    assert not [r for r in s["rows"][i:] if r["kind"] == "step" and r["target6"]]
    key_c = [r for r in s["rows"] if r["kind"] == "key" and r["key"] == "c"][0]
    assert "22:01" in str(key_c.get("refused"))
    assert in_order(trace(s["rows"]), "judge:CP1", "FREEZE:0", "key:c", "key:l", "end:ABORTED")


# ====================================================================== question 11: the camera's footprint, runner side
def test_Q11_scripted_rows_outside_the_footprint_are_reported():
    import json

    import motion
    from planner.cfg import visible_quad
    reg = json.loads((paths.ROOT / paths.REG_REL).read_text())
    rec = json.loads((paths.ROOT / paths.RECORD_REL).read_text())
    guard = g3.guard_from_record(paths.ROOT, reg, rec)
    q = visible_quad(reg, guard.box)
    seg = grasp_segment.load(paths.ROOT, guard.limits()["gripper"][0])
    rows = [r for s in seg["segments"] for r in s["rows"]]
    assert motion.segment_problems(guard, rows, visible=q) == []                          # control: the placeholder
    moved = tuple((x + 200.0, y) for x, y in q)                                           # the same quad 200 mm away
    probs = motion.segment_problems(guard, rows, visible=moved, limit=1)
    assert probs and "camera's footprint" in probs[0]
