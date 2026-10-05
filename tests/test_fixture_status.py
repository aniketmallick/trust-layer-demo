"""The --arm gate on the fixtures: a status counts only when all green AND written for the code that is on disk."""
from __future__ import annotations

import json

import start_checks
from fixtures import status


def _gate(sessions, dry: bool = False) -> dict:
    return next(c for c in start_checks.this_demo(sessions, dry) if c["name"] == "fixtures_green")


def test_GATE_missing_status_refuses_arm_and_not_a_dry_run(tmp_path):
    assert _gate(tmp_path)["ok"] is False
    assert _gate(tmp_path, dry=True)["ok"] is True                       # control: a dry run does not need it


def test_GATE_green_for_other_code_refuses(tmp_path):
    (tmp_path / "fixture_status.json").write_text(json.dumps({"all_green": True, "tree_sha256": "0" * 64, "t_iso": "x"}))
    c = _gate(tmp_path)
    assert c["ok"] is False and "OTHER code" in c["detail"]


def test_GATE_not_green_for_this_code_refuses(tmp_path):
    (tmp_path / "fixture_status.json").write_text(json.dumps({"all_green": False, "tree_sha256": status.tree_hash()}))
    assert _gate(tmp_path)["ok"] is False


def test_GATE_green_for_this_code_passes(tmp_path):
    (tmp_path / "fixture_status.json").write_text(json.dumps({"all_green": True, "tree_sha256": status.tree_hash()}))
    assert _gate(tmp_path)["ok"] is True                                 # control


def test_TREE_hash_moves_with_code_and_frames_not_with_sessions_or_env(tmp_path):
    (tmp_path / "a.py").write_text("x = 1\n")
    (tmp_path / "fixtures" / "frames").mkdir(parents=True)
    (tmp_path / "fixtures" / "frames" / "empty_table_1.png").write_bytes(b"png-1")
    h0 = status.tree_hash(tmp_path)
    (tmp_path / "sessions").mkdir()
    (tmp_path / "sessions" / "fixture_status.json").write_text("{}")
    (tmp_path / ".env").write_text("KEY=never-hashed\n")
    (tmp_path / "STATUS.md").write_text("status\n")
    assert status.tree_hash(tmp_path) == h0                              # control
    (tmp_path / "fixtures" / "frames" / "empty_table_1.png").write_bytes(b"png-2")
    h1 = status.tree_hash(tmp_path)
    (tmp_path / "a.py").write_text("x = 2\n")
    assert len({h0, h1, status.tree_hash(tmp_path)}) == 3


def test_STATUS_a_skipped_fixture_is_not_green(tmp_path, monkeypatch):
    fake = {"ran": True, "cmd": "x", "exit": 0, "n": 2, "passed": 1, "failed": [],
            "skipped": [{"test": "t::FRAME_empty_table", "why": "frame not captured"}]}
    monkeypatch.setattr(status, "run_suite", lambda *a: dict(fake))
    monkeypatch.setattr(status, "EXP", tmp_path)
    assert status.main() == 1
    out = json.loads((tmp_path / "sessions" / "fixture_status.json").read_text())
    assert out["all_green"] is False and out["suites"]["monitor"]["skipped"]


JUNIT = """<?xml version="1.0"?><testsuites><testsuite name="pytest">
<testcase classname="judge.tests.test_judge" name="test_fixture[JUDGE_timeout_returns_at_budget_abort_unsafe]"/>
<testcase classname="judge.tests.test_judge" name="test_saved_frame_live[hand_zone-1]">
  <failure message="hand_zone_1.png: hand_open_waiting = False, expected True">x</failure>
  <system-out>hand_zone_1.png: model m latency 2.2 s fenced True stripped True</system-out></testcase>
<testcase classname="judge.tests.test_judge" name="test_saved_frame_live[empty_table-1]"/>
%s
</testsuite></testsuites>"""


def test_STATUS_live_judge_accuracy_is_reported_not_gating(tmp_path):
    """Ruling 2 (reviewer 2026-10-01): a live-accuracy miss is reported with its line, and does not block green."""
    x = tmp_path / "j.xml"
    x.write_text(JUNIT % "")
    s = status.summarise(x, ["pytest"], 1)
    assert s["failed"] == [] and s["skipped"] == [] and s["n_gating"] == 1
    miss = [r for r in s["reported"] if r["result"] == "failed"]
    assert len(miss) == 1 and "hand_open_waiting" in miss[0]["message"] and "latency 2.2 s" in miss[0]["line"]


def test_STATUS_a_gating_judge_fixture_failing_is_not_green(tmp_path):
    x = tmp_path / "j.xml"
    x.write_text(JUNIT % '<testcase classname="judge.tests.test_judge" name="test_fixture[JUDGE_fake_key_never_leaves]">'
                          '<failure message="key leaked">x</failure></testcase>')
    s = status.summarise(x, ["pytest"], 1)
    assert s["failed"] == ["judge.tests.test_judge::test_fixture[JUDGE_fake_key_never_leaves]"]


def test_STATUS_known_misses_carry_the_frame_hash():
    import hashlib
    for m in status.KNOWN_MISSES:
        f = status.EXP / "fixtures" / "frames" / m["frame"]
        assert hashlib.sha256(f.read_bytes()).hexdigest() == m["sha256"]
    assert any(m["frame"] == "hand_cross6_3.png" for m in status.KNOWN_MISSES)


def test_STATUS_records_commands_relative_to_this_folder():
    """The public fixture_status.json carries no home path (2026-10-05): the interpreter is recorded relative to this
    folder; the command run is unchanged."""
    from pathlib import Path

    from fixtures import status as st
    s = st.shown([str(st.RUNNER_PY), "-m", "pytest", "tests"])
    assert not s.startswith("/") and str(Path.home()) not in s and s.endswith("lerobot-mps-venv/bin/python -m pytest tests")
    assert st.shown([str(st.MONITOR_PY), "-m", "pytest", "monitor"]) == ".venv_monitor/bin/python -m pytest monitor"
