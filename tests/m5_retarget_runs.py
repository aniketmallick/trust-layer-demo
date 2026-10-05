#!/usr/bin/env python
"""Monitor 5 during re-targets, over N consecutive dry runs (reviewer 2026-10-01, SR08). Not a pytest file: it takes
about a minute a run. Nothing else should run on the Mac meanwhile.

    ~/lerobot-mps-venv/bin/python tests/m5_retarget_runs.py [--runs 20]

Each run: the hand appears during the carry, [h], then during the approach the palm moves 10, 20, 30, 40 mm from where
it was approved (four re-targets, each computed in the worker process while the arm moves), [r] at the hold. Written:
sessions/_dryrun/m5_retarget_<n>runs.json - per run every re-target row's status, every freeze, the control steps'
times over the whole run and over the steps while a job was out, and whether monitor 5 fired.
"""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import os
import sys
import time
from pathlib import Path

EXP = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(EXP))
sys.dont_write_bytecode = True

from common import paths  # noqa: E402

paths.use_anchor()
import g3  # noqa: E402

import handover_session  # noqa: E402
from tests.helpers import free_port  # noqa: E402

EVENTS = ["appear@APPROACH_ZONE+1.0", "drift10@HANDOVER_APPROACH+1.0", "drift20@HANDOVER_APPROACH+2.5",
          "drift30@HANDOVER_APPROACH+4.0", "drift40@HANDOVER_APPROACH+5.5", "vanish@RELEASE+1.0"]


def one(out: Path, n: int) -> dict:
    sessions = out / f"run{n:02d}"
    argv = ["--dry-run", "--yes", "--no-esc", "--sessions-dir", str(sessions), "--frame-dir", str(out / f"khv{n:02d}"),
            "--udp-port", "0", "--cue-port", str(free_port()), "--dry-keys", "h,r,l", "--sim-script", "none"]
    for e in EVENTS:
        argv += ["--sim-event", e]
    t0, load0 = time.monotonic(), os.getloadavg()[0]
    with contextlib.redirect_stdout(io.StringIO()):
        rc = handover_session.main(argv)
    d = sorted((sessions / "_dryrun").glob("KH-DRY-*"))[-1]
    rows = [json.loads(x) for x in (d / "session.jsonl").read_text().splitlines() if x.strip()]
    steps = [r for r in rows if r["kind"] == "step" and r.get("dt_ms") is not None]
    out_window, open_job, m5_in, m5_out = [], False, 0, 0
    for r in rows:                                         # the steps taken while a re-target job was out
        if r["kind"] == "retarget":
            open_job = r["status"] == "asked"
        elif r["kind"] == "step" and open_job and r.get("dt_ms") is not None:
            out_window.append(r["dt_ms"])
        elif r["kind"] == "freeze" and r["monitor"] == 5:
            m5_in, m5_out = m5_in + (1 if open_job else 0), m5_out + (0 if open_job else 1)
    rt = [r["status"] for r in rows if r["kind"] == "retarget"]
    fz = [(r["monitor"], r["code"], r["phase"]) for r in rows if r["kind"] == "freeze"]
    m5 = [r for r in steps if r["monitors"]["5"]["fired"]]
    end = [r for r in rows if r["kind"] == "trial_end"]
    return {"run": n, "rc": rc, "session": d.name, "chain_ok": g3.verify_chain(d / "session.jsonl")[0],
            "outcome": end[-1]["outcome"] if end else None, "seconds": round(time.monotonic() - t0, 1),
            "retarget_status": {s: rt.count(s) for s in sorted(set(rt))}, "freezes": fz,
            "monitor5_fired_steps": len(m5), "monitor5_while_a_job_was_out": m5_in, "monitor5_otherwise": m5_out,
            "retargets_asked": rt.count("asked"), "load_avg_1min": [round(load0, 1), round(os.getloadavg()[0], 1)],
            "steps": len(steps),
            "dt_ms_max": round(max(r["dt_ms"] for r in steps), 2),
            "dt_ms_while_a_job_was_out": {"n": len(out_window), "max": round(max(out_window), 2) if out_window else None}}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=20)
    a = ap.parse_args()
    out = EXP / "_to_delete" / f"m5runs_{time.strftime('%Y%m%dT%H%M%S')}"
    res = []
    for n in range(1, a.runs + 1):
        r = one(out, n)
        res.append(r)
        print(json.dumps(r), flush=True)
    limit = 89.9
    summary = {"t_iso": g3.now_iso(), "runs": a.runs, "events": EVENTS, "worker": "process",
               "runs_with_monitor5": sum(1 for r in res if r["monitor5_fired_steps"] or any(f[0] == 5 for f in r["freezes"])),
               "runs_with_monitor5_while_a_job_was_out": sum(1 for r in res if r["monitor5_while_a_job_was_out"]),
               "runs_that_reached_a_retarget": sum(1 for r in res if r["retargets_asked"]),
               "load_avg_1min_range": [min(min(r["load_avg_1min"]) for r in res), max(max(r["load_avg_1min"]) for r in res)],
               "dt_ms_max_over_all_runs": max(r["dt_ms_max"] for r in res), "loop_dt_limit_ms": limit,
               "swapped_total": sum(r["retarget_status"].get("swapped", 0) for r in res),
               "pending_total": sum(r["retarget_status"].get("pending", 0) for r in res),
               "label": "DRY RUN - demonstration - fake bus, simulated hand feed", "per_run": res}
    p = EXP / "sessions" / "_dryrun" / f"m5_retarget_{a.runs}runs.json"
    p.write_text(json.dumps(summary, indent=1) + "\n", encoding="utf-8")
    print(f"re-targets reached in {summary['runs_that_reached_a_retarget']} of {a.runs} runs; monitor 5 while a job was out: "
          f"{summary['runs_with_monitor5_while_a_job_was_out']} runs; monitor 5 anywhere: "
          f"{summary['runs_with_monitor5']} runs; load {summary['load_avg_1min_range']}")
    print(f"monitor 5 fired in {summary['runs_with_monitor5']} of {a.runs} runs; worst step "
          f"{summary['dt_ms_max_over_all_runs']} ms (limit {limit}); swapped {summary['swapped_total']}, pending "
          f"{summary['pending_total']} -> {p.relative_to(EXP)}")
    return 0 if summary["runs_with_monitor5_while_a_job_was_out"] == 0 and summary["runs_that_reached_a_retarget"] == a.runs else 1


if __name__ == "__main__":
    sys.exit(main())
