"""The handover fault table (without its rehearsal sessions) and the replay page built from a rehearsal's chain."""
from __future__ import annotations

import json
import re

import pytest

import fault_table_handover as fth
from common import paths
from replay import build_replay
from tests.helpers import run_dry


def test_FT_every_row_as_expected_fire_and_control(tmp_path):
    res = fth.build(paths.ROOT, tmp_path / "sessions", with_sessions=False)
    bad = [r for r in res["table"] if not r["ok"]]
    assert res["all_as_expected"] and not bad, bad
    assert "DRY RUN" in res["label"] and "demonstration" in res["label"] and "nothing seen live" in res["label"]
    assert all(r["live"] == "no" for r in res["table"])
    for mon in ("7 hand_in_workspace", "8 hand_link", "judge gate"):
        kinds = {r["kind"] for r in res["table"] if r["monitor"] == mon}
        assert kinds == {"fire", "control"}, mon                     # each seen to fire and seen not to fire
    cited = res["g3_fault_table_cited"]
    assert cited["all_as_expected"] is True and cited["sha256_matches_its_log_line"] is True
    out = tmp_path / "sessions" / "_dryrun"
    assert json.loads((out / "fault_table_handover.json").read_text())["all_as_expected"] is True
    assert "cert" + "ified" not in (out / "fault_table_handover.md").read_text().lower()
    print(f"{res['n_rows']} rows: {res['n_fire']} fire, {res['n_control']} control")


@pytest.fixture(scope="module")
def replay(tmp_path_factory):
    s = run_dry(tmp_path_factory.mktemp("replay"), keys="k", script="handover")
    out = build_replay.build(s["dir"])
    return s, out, out.read_text(encoding="utf-8")


def test_REPLAY_is_one_self_contained_file(replay):
    s, out, html = replay
    assert out.name == "replay.html" and out.parent == s["dir"]
    assert not re.search(r"https?://", html) and not re.search(r'\b(?:src|href)\s*=\s*"(?!data:)', html)
    assert "<link" not in html and "@import" not in html and "fetch(" not in html and "XMLHttpRequest" not in html
    assert "data:image/jpeg;base64," in html


def test_REPLAY_says_demonstration_and_never_the_other_word(replay):
    _, _, html = replay
    assert "demonstration" in html[html.index("<title>"):html.index("</title>")].lower()
    assert "DEMONSTRATION" in html[html.index("<header>"):html.index("</header>")]
    assert "cert" + "ified" not in html.lower()


def test_REPLAY_data_comes_from_the_chain(replay):
    s, _, html = replay
    d = json.loads(re.search(r'<script id="data" type="application/json">(.*?)</script>', html, re.S).group(1).replace("<\\/", "</"))
    steps = [r for r in s["rows"] if r["kind"] == "step"]
    assert len(d["steps"]["t"]) == len(steps) and len(d["steps"]["mon"][0]) == 8
    fz = [r for r in s["rows"] if r["kind"] == "freeze"]
    ev = [e for e in d["events"] if e["kind"] == "freeze"]
    assert len(ev) == len(fz) == 1 and ev[0]["monitor"] == 7 and ev[0]["frame_sha256"] == fz[0]["frame_sha256"]
    assert d["frames"][0]["sha256"] == fz[0]["photo_sha256"] and d["frames"][0]["hands"][0]["palm_mm"] == fz[0]["hands"][0]["palm_mm"]
    assert [e["key"] for e in d["events"] if e["kind"] == "key" and e["accepted"]][0] == "k"
    assert any(e["kind"] == "judge" and e["phase"] == "FREEZE" and e["model"] == "stub" for e in d["events"])
    facts = {f[0]: f[1] for f in d["facts"]}
    assert facts["Chain"].startswith("verifies") and "PLACEHOLDER" in facts["Screwdriver (prop tool)"] and "DRY RUN" in facts["Mode"]
    kinds = [r[2] for r in d["log"]]
    assert kinds.index("freeze") < kinds.index("judge", kinds.index("freeze")) < kinds.index("key")
    assert len(d["box"]) == 6 and len(d["overlay"]["zone"]) == 4


def test_REPLAY_takeover_marker_where_the_plan_took_over(tmp_path):
    s = run_dry(tmp_path, keys="h,l", script="second_hand")
    html = build_replay.build(s["dir"]).read_text(encoding="utf-8")
    d = json.loads(re.search(r'<script id="data" type="application/json">(.*?)</script>', html, re.S).group(1).replace("<\\/", "</"))
    kinds = [e["kind"] for e in d["events"]]
    assert kinds.count("takeover") == 1 and kinds.count("freeze") == 2
    assert kinds.index("freeze") < kinds.index("takeover") < len(kinds) - 1 - kinds[::-1].index("freeze")


def test_REPLAY_a_broken_chain_is_shown_as_broken(replay, tmp_path):
    import shutil
    s, _, _ = replay
    d = tmp_path / "copy"
    shutil.copytree(s["dir"], d)
    lines = (d / "session.jsonl").read_text().splitlines()
    (d / "session.jsonl").write_text("\n".join(lines[:40] + lines[41:]) + "\n")
    html = build_replay.build(d).read_text(encoding="utf-8")
    assert "BROKEN" in html and '"verifies' not in html
