"""Where re-target plans are computed: never on the control thread's clock (reviewer 2026-10-01, SR08).

ProcessWorker (the session's): a separate process that holds its own planner (ports.PlannerPort, built from the same
arguments as the session's) and answers one job at a time over a pipe. The control loop only ever calls poll(), which
does not block. Its own interpreter, so the planner's 50-215 ms of numpy / mujoco work cannot hold the control loop's
interpreter lock - a planner THREAD did, once: a 94 ms control step (monitor 5) during a re-target, 2026-10-01 20:05.

ThreadWorker: the same interface around a planner object handed to the session (tests that need a planner the process
cannot be given). Not used by a session built from its own arguments.

Every job carries an id; a result is (id, plan | None, error | None, ms). Results for a job the caller has given up on
are the caller's to ignore by id.
"""
from __future__ import annotations

import multiprocessing as mp
import threading
import time
import types

READY_TIMEOUT_S = 60.0


def _serve(conn, planner_args: tuple, delay_s: float) -> None:
    """The worker process: build the planner once, then answer jobs until the pipe closes or 'stop' arrives."""
    import sys
    sys.dont_write_bytecode = True
    try:
        from ports import PlannerPort
        port = PlannerPort(*planner_args)
        conn.send(("ready", None, None, 0.0))
    except Exception as e:  # noqa: BLE001 - the session reads this and refuses to start
        conn.send(("failed", None, f"{type(e).__name__}: {e}", 0.0))
        return
    while True:
        try:
            msg = conn.recv()
        except (EOFError, OSError):
            return
        if msg == "stop":
            return
        while conn.poll(0):                             # a newer job queued behind it: only the newest is planned
            newer = conn.recv()
            if newer == "stop":
                return
            msg = newer
        job_id, plan_ref, palm, start = msg
        plan = types.SimpleNamespace(**plan_ref)
        t0 = time.perf_counter()
        if delay_s:
            time.sleep(delay_s)                         # dry-run test knob: a slow planner (never on --arm)
        try:
            res, err = port.retarget(plan, palm, start), None
        except Exception as e:  # noqa: BLE001
            res, err = None, f"the planner raised {type(e).__name__}: {e}"
        try:
            conn.send((job_id, res, err, 1000.0 * (time.perf_counter() - t0)))
        except (EOFError, OSError, BrokenPipeError):
            return


class ProcessWorker:
    def __init__(self, planner_args: tuple, delay_s: float = 0.0):
        ctx = mp.get_context("spawn")
        self._conn, child = ctx.Pipe()
        self.proc = ctx.Process(target=_serve, args=(child, tuple(planner_args), float(delay_s)), daemon=True,
                                name="retarget-worker")
        self.proc.start()
        child.close()
        self.kind, self.delay_s, self.dead = "process", float(delay_s), None

    def wait_ready(self, timeout_s: float = READY_TIMEOUT_S) -> str | None:
        """None when the worker's planner is built; else why not."""
        if not self._conn.poll(timeout_s):
            return f"the re-target worker did not report ready in {timeout_s:g} s"
        tag, _, err, _ = self._conn.recv()
        return None if tag == "ready" else f"the re-target worker could not build its planner: {err}"

    def submit(self, job_id: int, plan, palm, start_q6) -> None:
        """A few hundred bytes: what the planner's retarget() reads of the current plan, the palm, the start pose.
        A whole plan (hundreds of rows) here filled the pipe while the worker was sending its own result back: both
        sides blocked (seen 2026-10-01 20:55). The control loop never waits on this pipe."""
        ref = {"candidate_deg": float(plan.candidate_deg), "palm_mm": [float(v) for v in plan.palm_mm],
               "plan_sha256": str(plan.plan_sha256)}
        try:
            self._conn.send((int(job_id), ref, [float(v) for v in palm], [float(v) for v in start_q6]))
        except (OSError, BrokenPipeError) as e:
            self.dead = f"the re-target worker is gone ({type(e).__name__})"

    def poll(self) -> list:
        """Every result that has arrived, without waiting. A worker that died -> [('dead', None, why, 0)]."""
        out = []
        try:
            while self._conn.poll(0):
                out.append(self._conn.recv())
        except (EOFError, OSError) as e:
            self.dead = f"the re-target worker is gone ({type(e).__name__})"
        if self.dead is None and not self.proc.is_alive():
            self.dead = f"the re-target worker exited (code {self.proc.exitcode})"
        if self.dead is not None:
            out.append(("dead", None, self.dead, 0.0))
        return out

    def close(self) -> None:
        try:
            self._conn.send("stop")
        except (OSError, BrokenPipeError):
            pass
        self.proc.join(timeout=3)
        if self.proc.is_alive():
            self.proc.terminate()
            self.proc.join(timeout=3)


class ThreadWorker:
    def __init__(self, planner):
        self.planner, self.kind, self.delay_s, self.dead = planner, "thread", 0.0, None
        self._out: list = []
        self._lock = threading.Lock()

    def wait_ready(self, timeout_s: float = 0.0) -> str | None:
        return None

    def submit(self, job_id: int, plan, palm, start_q6) -> None:
        def work() -> None:
            t0 = time.perf_counter()
            try:
                res, err = self.planner.retarget(plan, list(palm), list(start_q6)), None
            except Exception as e:  # noqa: BLE001
                res, err = None, f"the planner raised {type(e).__name__}: {e}"
            with self._lock:
                self._out.append((job_id, res, err, 1000.0 * (time.perf_counter() - t0)))
        threading.Thread(target=work, daemon=True, name=f"retarget-{job_id}").start()

    def poll(self) -> list:
        with self._lock:
            out, self._out = self._out, []
        return out

    def close(self) -> None:
        pass
