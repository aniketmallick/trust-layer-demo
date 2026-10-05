"""VIDEO PREP (the operator, 2026-10-04): --layer-off (the contrast shot) and the presentation page (present.html)."""
from __future__ import annotations

import json
import re
import threading

import pytest

import monitors as mon_mod
import motion as motion_mod
from handover_session import LAYER_OFF
from replay import build_replay, present
from tests.helpers import in_order, run_dry, run_obj, trace

PICK_PLACE = ["phase:SETUP", "phase:GRASP", "phase:LIFT", "phase:APPROACH_ZONE", "phase:PRE_PLACE", "phase:PLACE",
              "phase:RETREAT", "end:PLACED"]


def page_data(html: str) -> dict:
    return json.loads(re.search(r'<script id="data" type="application/json">(.*?)</script>', html, re.S).group(1)
                      .replace("<\\/", "</"))


# ====================================================================== --layer-off
@pytest.fixture(scope="module")
def layer_off(tmp_path_factory):
    return run_dry(tmp_path_factory.mktemp("layer_off"), "--layer-off", keys="")


def test_LAYER_OFF_pick_place_only_every_row_stamped(layer_off):
    s = layer_off
    assert s["rc"] == 0 and s["chain_ok"][0]
    assert all(r.get("trust_layer") == LAYER_OFF for r in s["rows"])                 # every row, the first included
    assert in_order(trace(s["rows"]), *PICK_PLACE)
    kinds = {r["kind"] for r in s["rows"]}
    assert not kinds & {"judge", "plan", "preauth", "retarget"}                       # no judge, no planner, no CP
    assert not {r["phase"] for r in s["rows"] if r["kind"] == "step"} & {"CP1", "CP2", "CP3", "HOLD"}


def test_LAYER_OFF_monitors_7_8_off_1_6_read(layer_off):
    steps = [r for r in layer_off["rows"] if r["kind"] == "step"]
    for r in steps:
        for n in ("7", "8"):
            assert r["monitors"][n]["off"] is True and r["monitors"][n]["fired"] is False
        assert all("off" not in r["monitors"][str(n)] for n in range(1, 7))
    start = next(r for r in layer_off["rows"] if r["kind"] == "session_start")
    assert start["layer_off"] == LAYER_OFF and start["monitors_off"] == [7, 8] and start["mode"] == "layer_off"
    assert start["judge"]["provider"] == "off" and start["planner"]["provider"] == "off"
    assert start["hand_feed"].startswith("OFF") and "pick and place with the trust layer OFF" in start["label"]


def test_LAYER_OFF_arm_statement_says_what_is_off_and_the_prop_hand(layer_off):
    out = layer_off["out"]
    stmt = out[out.index("OPERATOR-PRESENT INTERLOCK"):out.index("Type ARM")]
    assert f"*** {LAYER_OFF} ***" in stmt and "PROP HAND - NOT A REAL HAND" in stmt
    assert "hand monitor (monitors 7 and 8), the judge and the handover planner are NOT" in stmt
    assert "Monitors 1-6 live" in stmt and "PICK -> PLACE ONLY" in stmt
    assert re.search(r"lowers the fingertip to \d+ mm above the table", stmt)
    assert "real hands OUT of the workspace" in out                                  # the trial's Enter prompt


@pytest.mark.parametrize("flag", ["--palm-placement", "--handover-trial", "--hold-test"])
def test_LAYER_OFF_refused_with_any_handover_flag(tmp_path, flag):
    s = run_dry(tmp_path, "--layer-off", flag, keys="")
    ref = [r for r in s["rows"] if r["kind"] == "refused"]
    assert s["rc"] == 2 and ref and ref[0]["reason"] == "bad_flags"
    assert not [r for r in s["rows"] if r["kind"] == "step"]                          # nothing moved


def test_LAYER_OFF_a_monitor_6_stop_freezes_and_asks_without_judge_or_handover(tmp_path, monkeypatch):
    h = {}
    orig = motion_mod.Motion.run_rows

    def run_rows(self, phase, rows, **kw):
        if phase == "APPROACH_ZONE" and "esc" not in h:
            h["esc"] = threading.Timer(0.5, lambda: self.rig.kill.trigger("dry:ESC"))
            h["esc"].start()
        return orig(self, phase, rows, **kw)
    monkeypatch.setattr(motion_mod.Motion, "run_rows", run_rows)
    s = run_obj(tmp_path, "--layer-off", keys="c")
    assert s["error"] is None and s["rc"] == 0
    tr = trace(s["rows"])
    assert in_order(tr, "phase:APPROACH_ZONE", "FREEZE:6", "key:c", "phase:PLACE", "end:PLACED")
    key = next(r for r in s["rows"] if r["kind"] == "key" and r["key"] == "c")
    assert key["menu"] == ["w", "k", "l", "c"] and key["verdict_on_screen"] is None    # no [h] / [o], no verdict
    assert not [r for r in s["rows"] if r["kind"] == "judge"]
    assert "monitors 1-6 clear" in s["out"] and "TRUST LAYER OFF: no judge, no handover" in s["out"]


def test_LAYER_OFF_only_7_and_8_can_be_off():
    with pytest.raises(ValueError):
        mon_mod.evaluate(None, 0.0, [0.0] * 6, None, None, 0, "GRASP", False, None, {}, (False, ""), off=(2,))
    st = mon_mod.off_state(7)
    assert st["off"] and not st["fired"] and st["clear"] and not st["present"]


def test_LAYER_OFF_off_pieces_refuse_use():
    import ports
    j = ports.Off("judge", LAYER_OFF)
    assert j.describe()["provider"] == "off"
    with pytest.raises(ports.LayerOff):
        j.ask("frame.jpg", "FREEZE", {})
    link = ports.NullLink()
    assert link.snapshot()["msg"] is None and link.monitor8()[0] is False


def test_LAYER_OFF_replay_and_present_carry_the_stamp(layer_off):
    build_replay.build(layer_off["dir"])
    rep = (layer_off["dir"] / "replay.html").read_text(encoding="utf-8")
    d = page_data(rep)
    assert d["layer_off"] == LAYER_OFF and any(f[0] == "TRUST LAYER" for f in d["facts"])
    p = page_data((layer_off["dir"] / "present.html").read_text(encoding="utf-8"))
    assert p["layer_off"] == LAYER_OFF and LAYER_OFF in p["stamps"] and p["judge"] == "OFF"
    assert all(m[6] == "3" and m[7] == "3" for m in p["steps"]["m"])                   # 7 and 8 drawn OFF


# ====================================================================== present.html
@pytest.fixture(scope="module")
def handover(tmp_path_factory):
    s = run_dry(tmp_path_factory.mktemp("present"), keys="k", script="handover")
    build_replay.build(s["dir"])
    return s, (s["dir"] / "present.html").read_text(encoding="utf-8")


def test_PRESENT_beside_replay_self_contained_and_honest(handover):
    s, html = handover
    assert (s["dir"] / "replay.html").is_file()
    assert not re.search(r"https?://", html) and "<link" not in html and "fetch(" not in html
    assert "DEMONSTRATION" in html and "cert" + "ified" not in html.lower()
    assert "width:720px;height:1080px" in html
    assert 'location.replace("present.html"' in (s["dir"] / "replay.html").read_text(encoding="utf-8")


def test_PRESENT_data_from_the_chain(handover):
    s, html = handover
    d = page_data(html)
    steps = [r for r in s["rows"] if r["kind"] == "step"]
    assert len(d["steps"]["t"]) == len(steps) and all(len(m) == 8 for m in d["steps"]["m"])
    first = next(r for r in steps if r.get("target6") is not None)
    assert d["steps"]["t"][steps.index(first)] == 0.0                                 # t = 0 at the first motion
    assert [x[0] for x in d["log"]] == sorted(x[0] for x in d["log"])                 # the log plays in chain order
    fz = [r for r in s["rows"] if r["kind"] == "freeze"]
    assert len([e for e in d["events"] if e["kind"] == "freeze"]) == len(fz) == 1
    labels = [b[1] for b in d["track"]]
    assert "MOVING" in labels and any(lb == f"FREEZE — monitor {fz[0]['monitor']}" for lb in labels)
    assert any(o[2].startswith("JUDGE: ") and o[2] != "JUDGE: asking…" for o in d["over"])
    fz_t = next(b[0] for b in d["track"] if b[1].startswith("FREEZE —"))
    assert all(not (fz_t <= o[0] < fz_t + present.FREEZE_SHOW_S) for o in d["over"])   # the freeze holds the badge


def test_PRESENT_row_times_inside_their_steps_and_in_chain_order():
    rows = [{"seq": 1, "kind": "check", "t_iso": "2026-10-04T10:00:00+05:30"},
            {"seq": 2, "kind": "step", "t_s": 10.0},
            {"seq": 3, "kind": "freeze", "t_iso": "2026-10-04T10:00:05+05:30"},
            {"seq": 4, "kind": "judge", "t_iso": "2026-10-04T10:00:07+05:30"},
            {"seq": 5, "kind": "step", "t_s": 18.0},
            {"seq": 6, "kind": "key", "t_iso": "2026-10-04T10:00:59+05:30"},
            {"seq": 7, "kind": "step", "t_s": 70.0}]
    tt, off = present.row_times(rows)
    assert 10.0 <= tt[3] <= tt[4] <= 18.0 and 18.0 <= tt[6] <= 70.0
    assert [tt[r["seq"]] for r in rows] == sorted(tt[r["seq"]] for r in rows)
    assert present.cuts([0.0, 4.0, 40.0, 41.0], 41.0) == [[4.0, 40.0]]                # a gap over 30 s is cut
    assert present.cuts([0.0, 20.0, 41.0], 41.0) == []


def test_PRESENT_reorient_from_a_turning_plan():
    plan = {"checks": {"turned_deg": 82.7}, "phases": [{"from_step": 1, "n_steps": 81, "phase": "rise in place"},
                                                       {"from_step": 82, "n_steps": 708,
                                                        "phase": "move above the release point, turning"}]}
    assert present._turning(plan) == (82, 789, 82.7)
    assert present._turning({**plan, "checks": {"turned_deg": 18.3}}) is None


# ====================================================================== the hold test's tighter step (2026-10-04)
def test_HOLD_no_tighter_step_past_the_overload_bar_or_the_floor():
    import types

    import hold_test
    s = types.SimpleNamespace(guard=types.SimpleNamespace(limits=lambda: {"gripper": [1.31, 97.72]}))
    hold = lambda peak: {"gripper_load": {"max": peak}}                                    # noqa: E731
    assert hold_test._no_tighter(s, 9.4, hold(160)) is None                                # 160 + 46.6 < 220: tried
    why = hold_test._no_tighter(s, 9.4, hold(200))                                         # 200 + 46.6 >= 220
    assert why and "past the 220 hold limit" in why and "stopped at 9.4 %" in why
    assert "not tried blind" in hold_test._no_tighter(s, 8.6, hold(None))
    assert "floor" in hold_test._no_tighter(s, 2.0, hold(50))


def test_PRESENT_page_script_parses():
    """The presentation's script, through node's parser (a stray // once commented out the play loop's tail and the
    page drew nothing). node is on this Mac; without it the test fails rather than skips (a skip is not green)."""
    import shutil
    import subprocess
    node = shutil.which("node") or "/opt/homebrew/bin/node"
    js = present.TEMPLATE.read_text(encoding="utf-8").split("<script>")[1].split("</script>")[0]
    r = subprocess.run([node, "-e", "new Function(require('fs').readFileSync(0, 'utf8'))"], input=js, text=True,
                       capture_output=True, timeout=30)
    assert r.returncode == 0, r.stderr[-400:]


def test_PRESENT_landscape_by_default_and_the_mp4_renderer_parses():
    """2026-10-05: the 3D view on top, the timeline and the call log under it, 1920x1080 (?layout=portrait keeps the
    720x1080 panel); replay/render_mp4.mjs turns a session into an MP4 at 1x (node, Chrome, PyAV)."""
    import shutil
    import subprocess
    html = present.TEMPLATE.read_text(encoding="utf-8")
    assert 'Q.get("layout") !== "portrait"' in html and "top: [0, 0, 1920, 540]" in html and "?record=1" in html
    node = shutil.which("node") or "/opt/homebrew/bin/node"
    mjs = present.HERE / "render_mp4.mjs"
    r = subprocess.run([node, "--check", str(mjs)], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr[-400:]
    assert "real=1&record=1" in mjs.read_text(encoding="utf-8")                      # 1x, real time, no cuts


def test_VERIFY_the_standalone_verifier_agrees_with_g3(layer_off, tmp_path):
    """verify_chain.py (no anchor/: for readers of the public repository) gives g3.verify_chain's answer, and finds
    an edited row and a changed photo."""
    import shutil

    import verify_chain as vc
    from common import paths as _p
    _p.use_anchor()
    import g3
    chain = layer_off["dir"] / "session.jsonl"
    assert vc.verify(chain)[0] is True and g3.verify_chain(chain)[0] is True
    d = tmp_path / "copy"
    shutil.copytree(layer_off["dir"], d)
    lines = (d / "session.jsonl").read_text().splitlines()
    lines[30] = lines[30].replace('"phase": "', '"phase": "X', 1)
    (d / "session.jsonl").write_text("\n".join(lines) + "\n")
    assert vc.verify(d / "session.jsonl")[0] is False and g3.verify_chain(d / "session.jsonl")[0] is False
    assert vc.main([str(layer_off["dir"])]) == 0
