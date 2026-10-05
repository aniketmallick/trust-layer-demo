"""retarget_worker.ProcessWorker on its own: a job send never blocks the caller, even with the worker busy; the newest
job is answered; a dead worker is reported. (A whole plan per job once filled the pipe both ways: 2026-10-01 20:55.)"""
from __future__ import annotations

import time

from common import paths

paths.use_anchor()

import retarget_worker as rw  # noqa: E402
from planner import fixtures as fx  # noqa: E402
from planner import handover_planner as hp  # noqa: E402


def test_WORKER_sends_never_block_and_the_newest_job_is_answered():
    w = rw.ProcessWorker((paths.ROOT, False, None, None, None), delay_s=0.5)
    try:
        assert w.wait_ready() is None
        cfg = fx.cfg_for("sim")
        plan = hp.plan_handover(list(fx.start_pose("sim")), (340.0, 100.0), cfg)
        sends = []
        for jid in range(1, 21):                                       # 20 jobs at a worker busy for 0.5 s each
            t0 = time.perf_counter()
            w.submit(jid, plan, (340.0, 100.0 + jid), plan.targets[10])
            sends.append(1000 * (time.perf_counter() - t0))
        assert max(sends) < 20.0, f"a send blocked: max {max(sends):.1f} ms"
        got, t_end = [], time.monotonic() + 10.0
        while time.monotonic() < t_end and not any(g[0] == 20 for g in got):
            got += w.poll()
            time.sleep(0.02)
        ids = [g[0] for g in got]
        assert 20 in ids and len(ids) <= 3, ids                          # the newest answered; the queued ones skipped
        res = [g for g in got if g[0] == 20][0][1]
        assert res.ok and tuple(res.palm_mm) == (340.0, 120.0)
    finally:
        w.close()


def test_WORKER_killed_is_reported_dead():
    w = rw.ProcessWorker((paths.ROOT, False, None, None, None))
    try:
        assert w.wait_ready() is None
        w.proc.kill()
        w.proc.join(timeout=5)
        out = w.poll()
        assert out and out[-1][0] == "dead"
    finally:
        w.close()
