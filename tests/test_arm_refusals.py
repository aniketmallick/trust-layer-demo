"""--arm is refused in code, each refusal a row and a non-zero exit. Nothing here opens a port or a camera: every
case returns before either is touched, and the camera hook raises if it is ever called."""
from __future__ import annotations

import contextlib
import datetime
import io
import json

from common import paths

paths.use_anchor()
import g3  # noqa: E402

import handover_session  # noqa: E402
import ports  # noqa: E402
import start_checks  # noqa: E402
from tests.helpers import boom, run_dry  # noqa: E402

PORT = "/dev/never-opened-by-a-test"


def at(h, m=0):
    return datetime.datetime(2026, 10, 2, h, m, tzinfo=g3.IST)


KNIFE = ["--knife-handle-mm", "95", "--knife-blade-mm", "85", "--knife-width-mm", "18", "--knife-axis-sign", "1"]   # measured (for the gates after it)


SEGMENTS = ["--segments", str(paths.EXP / "dawn" / "segments.json")]   # the dawn-recorded segments (2026-10-02)


def run_arm(tmp, *extra, now, knife: bool = True, segments: bool = True):
    sessions = tmp / "sessions"
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = handover_session.main(["--arm", "--sessions-dir", str(sessions), *extra, *(KNIFE if knife else []),
                                    *(SEGMENTS if segments else [])],
                                   now_fn=lambda: now,
                                   camera_factory=boom, link_factory=boom, judge=object(), planner=object())
    d = sorted(sessions.glob("KH-S*"))[-1]
    rows = [json.loads(x) for x in (d / "session.jsonl").read_text().splitlines() if x.strip()]
    assert g3.verify_chain(d / "session.jsonl") == (True, "ok")
    assert not (d / "safety_events.jsonl").exists() or '"torque_on"' not in (d / "safety_events.jsonl").read_text()
    return rc, rows, buf.getvalue()


def test_ARM_default_mode_is_dry_run_and_both_flags_refuse():
    assert handover_session.build_parser().parse_args([]).arm is False
    with contextlib.redirect_stdout(io.StringIO()):
        assert handover_session.main(["--arm", "--dry-run"]) == 2


def test_ARM_no_motion_window_by_default(tmp_path):
    """The operator, 2026-10-03: 'don't keep any time bounded restrictions like 22:00 or anything'. --arm at any hour
    is not refused for the hour (it goes on to the start checks, which this tree does not pass)."""
    assert start_checks.MOTION_HOURS is None
    for now in (at(23, 10), at(22, 0), at(5, 59), at(2, 0)):
        rc, rows, _ = run_arm(tmp_path, "--port", PORT, "--z-handover-mm", "60", now=now)
        assert rc == 2 and rows[-1]["reason"] == "start_checks"
        hours = [r for r in rows if r["kind"] == "check" and r["name"] == "motion_hours"]
        assert hours and hours[0]["ok"] is True and "no motion window" in hours[0]["detail"]


def test_ARM_with_a_window_set_outside_it_is_refused_and_the_refusal_is_a_row(tmp_path, monkeypatch):
    monkeypatch.setattr(start_checks, "MOTION_HOURS", (6, 22))          # the rule as it was until 2026-10-03
    for now in (at(23, 10), at(22, 0), at(5, 59), at(2, 0)):
        rc, rows, out = run_arm(tmp_path, "--port", PORT, "--z-handover-mm", "60", now=now)
        assert rc == 2 and rows[-1]["kind"] == "refused" and rows[-1]["reason"] == "motion_hours" and rows[-1]["arm"] is True
        assert f"{now:%H:%M}" in rows[-1]["detail"] and "REFUSED" in out and len(rows) == 1


def test_ARM_hours_rule_boundaries_control(monkeypatch):
    assert all(start_checks.motion_hours_ok(at(h)) for h in range(24))               # no window: every hour
    monkeypatch.setattr(start_checks, "MOTION_HOURS", (6, 22))
    assert start_checks.motion_hours_ok(at(6, 0)) and start_checks.motion_hours_ok(at(21, 59))
    assert not start_checks.motion_hours_ok(at(22, 0)) and not start_checks.motion_hours_ok(at(5, 59))
    utc = datetime.datetime(2026, 10, 2, 1, 0, tzinfo=datetime.timezone.utc)       # 06:30 IST
    assert start_checks.motion_hours_ok(utc)


def test_ARM_refused_without_port(tmp_path):
    rc, rows, _ = run_arm(tmp_path, "--z-handover-mm", "60", now=at(12))
    assert rc == 2 and rows[-1]["reason"] == "no_port"


def test_ARM_refused_without_z_handover(tmp_path):
    rc, rows, _ = run_arm(tmp_path, "--port", PORT, now=at(12))
    assert rc == 2 and rows[-1]["reason"] == "z_handover_not_set"


def test_ARM_refused_when_this_demos_fixtures_or_fault_table_are_not_green(tmp_path):
    """In the hours, with a port and z set: the start checks that open nothing are read first; this tree has no
    fixture status and no handover fault table -> refused before the camera or the port is opened."""
    rc, rows, _ = run_arm(tmp_path, "--port", PORT, "--z-handover-mm", "60", now=at(12))
    assert rc == 2 and rows[-1]["reason"] == "start_checks"
    assert "fixtures_green" in rows[-1]["detail"] and "handover_fault_table" in rows[-1]["detail"]
    checks = {r["name"]: r["ok"] for r in rows if r["kind"] == "check"}
    assert checks["kill_switch_verified"] is True and checks["calibration_file"] is True and checks["motion_hours"] is True
    assert "framing_recheck" not in checks                                  # the camera was never reached


def test_ARM_kill_switch_must_be_verified_on_the_rig():
    rec = {"A10": {"kill_switch_verified": True, "dry_run": True}}
    assert next(c for c in start_checks.from_record(paths.ROOT, rec, dry=False) if c["name"] == "kill_switch_verified")["ok"] is False
    rec = {"A10": {"kill_switch_verified": False, "dry_run": False}}
    assert next(c for c in start_checks.from_record(paths.ROOT, rec, dry=False) if c["name"] == "kill_switch_verified")["ok"] is False
    rec = {"A10": {"kill_switch_verified": True, "dry_run": False}}
    assert next(c for c in start_checks.from_record(paths.ROOT, rec, dry=False) if c["name"] == "kill_switch_verified")["ok"] is True


def test_ARM_fixture_status_and_fault_table_gate_control(tmp_path):
    names = lambda dry: {c["name"]: c["ok"] for c in start_checks.this_demo(tmp_path, dry)}   # noqa: E731
    assert names(False) == {"handover_fault_table": False, "fixtures_green": False}
    assert names(True) == {"handover_fault_table": True, "fixtures_green": True}             # a dry run needs neither
    (tmp_path / "_dryrun").mkdir()
    (tmp_path / "_dryrun" / start_checks.FAULT_TABLE).write_text(json.dumps({"all_as_expected": True, "t_iso": "x"}))
    (tmp_path / start_checks.FIXTURE_STATUS).write_text(json.dumps({"all_green": False, "t_iso": "x"}))
    assert names(False) == {"handover_fault_table": True, "fixtures_green": False}
    (tmp_path / start_checks.FIXTURE_STATUS).write_text(json.dumps({"all_green": True, "t_iso": "x"}))
    assert names(False) == {"handover_fault_table": True, "fixtures_green": False}            # green, but for no named code
    from fixtures.status import tree_hash
    (tmp_path / start_checks.FIXTURE_STATUS).write_text(json.dumps({"all_green": True, "t_iso": "x",
                                                                    "tree_sha256": tree_hash()}))
    assert names(False) == {"handover_fault_table": True, "fixtures_green": True}


def test_RUN_a_missing_piece_is_a_refusal_never_a_stand_in(tmp_path):
    def no_link(*_a, **_k):
        raise ports.MissingPiece("common/hand_link.py is not importable (fixture)")
    s = run_dry(tmp_path, link_factory=no_link)
    assert s["rc"] == 2 and s["rows"][-1]["kind"] == "refused" and s["rows"][-1]["reason"] == "missing_piece"
    assert not any(r["kind"] == "step" for r in s["rows"])


def test_RUN_dry_run_writes_nothing_under_the_frozen_folders(tmp_path):
    """A full no-hand rehearsal: every file under anchor/, phase0/ and so101_sim/ has the mtime it had before."""
    def snap():
        return {str(p): p.stat().st_mtime_ns for d in ("anchor", "phase0", "so101_sim")
                for p in (paths.ROOT / d).rglob("*") if p.is_file()}
    before = snap()
    s = run_dry(tmp_path, script="none")
    after = snap()
    assert s["rc"] == 0 and before == after
