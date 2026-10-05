#!/usr/bin/env python
"""Run every fixture suite and write sessions/fixture_status.json - the file `handover_session.py --arm` reads.

    ~/lerobot-mps-venv/bin/python fixtures/status.py            (about 8 minutes; no motor, no camera, no port)

all_green is true only when every suite ran, nothing failed and NOTHING WAS SKIPPED: a fixture skipped for a missing
frame or a missing .env is unknown, and an unknown fixture does not count (the operator's rule 6). One class of test
is REPORTED, not gating (reviewer 2026-10-01, ruling 2): the judge's live-accuracy cases on the saved stills
(test_saved_frame_live) - their results, the model's own lines (model, latency, fence) and the known misses with
their frame hashes are written beside the gating result. The judge's schema / timeout / key fixtures stay gating.
The file carries the hash of the code and fixture frames it was run on; the runner recomputes that hash and refuses a
status written for other code. Suites run one after another: the link's timing fixtures are read on a quiet machine.
"""
from __future__ import annotations

import datetime
import hashlib
import json
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

EXP = Path(__file__).resolve().parent.parent
IST = datetime.timezone(datetime.timedelta(hours=5, minutes=30))
RUNNER_PY = Path.home() / "lerobot-mps-venv" / "bin" / "python"
MONITOR_PY = EXP / ".venv_monitor" / "bin" / "python"
SUITES = (("runner", RUNNER_PY, ["tests"]), ("judge", RUNNER_PY, ["judge"]), ("planner", RUNNER_PY, ["planner"]),
          ("monitor", MONITOR_PY, ["monitor"]))
SKIP_DIRS = {".venv_monitor", "sessions", "__pycache__", ".pytest_cache", ".git", "_to_delete"}
REPORTED = ("test_saved_frame_live", "test_handover_rubric_live", "test_placement_rubric_live")   # reported, not gating
KNOWN_MISSES = (                                 # reported with the frame hash; no rubric edit follows from them
    {"frame": "hand_cross6_3.png", "sha256": "a319416fa82ea789b593a42e5663c1b115b1f4312acbd5af7590808c2d1c5c1b",
     "model": "claude-opus-5-5", "seen": "2026-10-01T20:45", "field": "hand_open_waiting", "got": False, "want": True},
    {"frame": "hand_zone_1.png", "sha256": "00471940cea9e4928101141f95e33a57c45b2216d2a1d64bff1ada71376d3422",
     "model": "claude-haiku-4-5-20251001", "seen": "2026-10-01T22:09", "field": "hand_open_waiting", "got": False, "want": True},
    {"frame": "hand_zone_3.png", "sha256": "e2781e164563ddf8a584ce84b390834d3514d675a4270b723f5a4b0d4cc99250",
     "model": "claude-haiku-4-5-20251001", "seen": "2026-10-01T22:09", "field": "hand_open_waiting", "got": False, "want": True},
)
HASHED_SUFFIXES = {".py", ".md", ".json", ".jsonl", ".png", ".txt", ".lock", ".task", ".html"}
NOT_HASHED = {"STATUS.md", "PLAN.md"}


def tree_hash(exp: Path = EXP) -> str:
    """sha256 over (relative path, file sha256) of the code, the rubric, the model lock and the fixture frames."""
    h = hashlib.sha256()
    for p in sorted(exp.rglob("*")):
        rel = p.relative_to(exp)
        if not p.is_file() or SKIP_DIRS & set(rel.parts) or rel.name.startswith(".env"):
            continue
        if p.suffix not in HASHED_SUFFIXES or rel.name in NOT_HASHED:
            continue
        h.update(f"{rel.as_posix()}\0{hashlib.sha256(p.read_bytes()).hexdigest()}\n".encode())
    return h.hexdigest()


def shown(cmd: list) -> str:
    """The command as recorded: the interpreter's path relative to this folder (no home path in the public file)."""
    return " ".join([os.path.relpath(cmd[0], EXP)] + [str(c) for c in cmd[1:]])


def run_suite(name: str, py: Path, targets: list) -> dict:
    cmd = [str(py), "-m", "pytest", *targets, "-q", "-p", "no:cacheprovider", "-o", "junit_logging=system-out"]
    if not py.is_file():
        return {"ran": False, "cmd": shown(cmd), "why": f"{os.path.relpath(py, EXP)} not found"}
    with tempfile.TemporaryDirectory() as td:
        xml = Path(td) / "junit.xml"
        r = subprocess.run(cmd + [f"--junitxml={xml}"], cwd=EXP, capture_output=True, text=True,
                           env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})
        if not xml.is_file():
            return {"ran": False, "cmd": shown(cmd), "why": f"pytest exit {r.returncode}: {r.stdout[-300:]}"}
        return summarise(xml, cmd, r.returncode)


def _outcome(c) -> tuple:
    """(failed | skipped | passed, its message) of one JUnit testcase. (An element with no children is falsy: compared
    with None, never by truth.)"""
    for tag, word in (("failure", "failed"), ("error", "failed"), ("skipped", "skipped")):
        e = c.find(tag)
        if e is not None:
            return word, (e.get("message") or "")[:300]
    return "passed", ""


def summarise(xml: Path, cmd: list, returncode: int) -> dict:
    """One suite's JUnit file -> gating failures and skips, and the reported cases (REPORTED) apart."""
    cases = list(ET.parse(xml).getroot().iter("testcase"))
    cid = lambda c: f"{c.get('classname')}::{c.get('name')}"                      # noqa: E731
    rep = lambda c: any(x in str(c.get("name")) for x in REPORTED)                 # noqa: E731
    gating = [c for c in cases if not rep(c)]
    failed = [cid(c) for c in gating if _outcome(c)[0] == "failed"]
    skipped = [{"test": cid(c), "why": _outcome(c)[1][:160]} for c in gating if _outcome(c)[0] == "skipped"]
    reported = [{"test": cid(c), "result": _outcome(c)[0], "message": _outcome(c)[1],
                 "line": next((x for x in (c.findtext("system-out") or "").splitlines() if ".png:" in x), "")[:400]}
                for c in cases if rep(c)]
    return {"ran": True, "cmd": shown(cmd), "exit": returncode, "n": len(cases), "n_gating": len(gating),
            "passed": len(gating) - len(failed) - len(skipped), "failed": failed, "skipped": skipped,
            "reported": reported}


def main() -> int:
    before = tree_hash()
    suites = {name: run_suite(name, py, targets) for name, py, targets in SUITES}
    for name, s in suites.items():
        print(f"{name:8} " + (f"{s['passed']} passed, {len(s['failed'])} failed, {len(s['skipped'])} skipped"
                              if s["ran"] else f"DID NOT RUN: {s['why']}"))
        for f in s.get("failed", []):
            print(f"         FAILED  {f}")
        for k in s.get("skipped", []):
            print(f"         UNKNOWN {k['test']} - {k['why']}")
        rep = s.get("reported") or []
        if rep:
            print(f"         reported (not gating): {sum(r['result'] == 'passed' for r in rep)} of {len(rep)} passed")
            for r in rep:
                if r["result"] != "passed":
                    print(f"         REPORTED {r['result']}: {r['test']} - {r['message'][:120]}")
    green = all(s["ran"] and s["exit"] in (0, 1) and not s["failed"] and not s["skipped"]   # 1: a reported case failed
                for s in suites.values())
    after = tree_hash()
    out = {"all_green": bool(green and before == after), "t_iso": datetime.datetime.now(IST).isoformat(timespec="seconds"),
           "tree_sha256": after, "tree_changed_during_run": before != after, "suites": suites,
           "rule": ("all_green = every suite ran, 0 gating failed, 0 gating skipped; a skipped fixture is unknown, not "
                    "passed; the judge's live-accuracy cases are reported, not gating (ruling 2)"),
           "known_misses": list(KNOWN_MISSES),
           "label": "DEMONSTRATION - fixture status of a rehearsed screwdriver (prop tool) handover; not certificate evidence"}
    p = EXP / "sessions" / "fixture_status.json"
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8")
    print(f"all_green = {out['all_green']} -> {p.relative_to(EXP)} (tree {after[:12]})")
    return 0 if out["all_green"] else 1


if __name__ == "__main__":
    sys.exit(main())
