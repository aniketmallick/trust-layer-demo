"""Shared by the runner's tests: one dry-run session in a temporary tree, read back from its chain."""
from __future__ import annotations

import contextlib
import io
import json
import socket
from pathlib import Path

from common import paths

paths.use_anchor()
import g3  # noqa: E402

import handover_session  # noqa: E402


def free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def boom(*_a, **_k):
    raise AssertionError("the real camera must never be opened by a test")


def run_dry(tmp: Path, *extra, keys: str = "", script: str = "none", **inject) -> dict:
    """handover_session.main as a dry run on the fake bus -> {rc, rows, dir, out, chain_ok}."""
    sessions = Path(tmp) / "sessions"
    argv = ["--dry-run", "--yes", "--no-esc", "--sessions-dir", str(sessions), "--frame-dir", str(Path(tmp) / "khv"),
            "--udp-port", "0", "--cue-port", str(free_port()), "--dry-keys", keys, "--sim-script", script, *extra]
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = handover_session.main(argv, camera_factory=boom, **inject)
    d = sorted((sessions / "_dryrun").glob("KH-DRY-*"))[-1]
    rows = [json.loads(x) for x in (d / "session.jsonl").read_text(encoding="utf-8").splitlines() if x.strip()]
    return {"rc": rc, "rows": rows, "dir": d, "out": buf.getvalue(), "chain_ok": g3.verify_chain(d / "session.jsonl")}


def trace(rows: list) -> list:
    """The session as a short ordered list: phase changes, freezes, judge calls, keys, plans, ends."""
    out, last = [], None
    for r in rows:
        k = r["kind"]
        if k == "step":
            if r["phase"] != last:
                out.append(f"phase:{r['phase']}")
                last = r["phase"]
            continue
        if k == "freeze":
            out.append(f"FREEZE:{r['monitor']}")
        elif k == "judge":
            out.append(f"judge:{r['phase']}")
        elif k == "key" and r["accepted"]:
            out.append(f"key:{r['key']}")
        elif k == "plan":
            out.append(f"plan:{r['requested']}:{'ok' if r['plan'].get('ok') else 'none'}")
        elif k in ("retarget", "kill", "refused"):
            out.append(k)
        elif k == "trial_end":
            out.append(f"end:{r['outcome']}")
        if k in ("freeze", "key", "plan"):
            last = None
    return out


def in_order(tr: list, *want) -> bool:
    """Every item of `want` appears in `tr`, in that order (other items may lie between)."""
    i = 0
    for t in tr:
        if i < len(want) and t == want[i]:
            i += 1
    return i == len(want)


# ------------------------------------------------------------------------------------------ taps for the safety fixtures
def run_obj(tmp: Path, *extra, keys: str = "", script: str = "none", setup=None, **inject) -> dict:
    """As run_dry, but the session object is built here so a fixture can reach into it: `setup(runner)` runs before
    run(); an exception out of run() is returned ('error'), not raised."""
    import sys

    from common import paths as _paths
    sessions = Path(tmp) / "sessions"
    argv = ["--dry-run", "--yes", "--no-esc", "--sessions-dir", str(sessions), "--frame-dir", str(Path(tmp) / "khv"),
            "--udp-port", "0", "--cue-port", str(free_port()), "--dry-keys", keys, "--sim-script", script, *extra]
    a = handover_session.build_parser().parse_args(argv)
    sys.setswitchinterval(0.001)
    buf, err, rc = io.StringIO(), None, None
    with contextlib.redirect_stdout(buf):
        runner = handover_session.HandoverSession(a, _paths.ROOT, camera_factory=boom, **inject)
        if setup is not None:
            setup(runner)
        try:
            rc = runner.run()
        except Exception as e:  # noqa: BLE001 - the fixture reads it
            err = e
            for piece in (runner.feed,):                       # never leave the simulated feed running
                if piece is not None and piece.poll() is None:
                    piece.kill()
            for piece in (runner.link, runner.pump):
                try:
                    piece.stop()
                except Exception:  # noqa: BLE001
                    pass
    d = sorted((sessions / "_dryrun").glob("KH-DRY-*"))[-1]
    rows = [json.loads(x) for x in (d / "session.jsonl").read_text(encoding="utf-8").splitlines() if x.strip()]
    ev = d / "safety_events.jsonl"
    events = [json.loads(x) for x in ev.read_text(encoding="utf-8").splitlines() if x.strip()] if ev.is_file() else []
    return {"rc": rc, "rows": rows, "dir": d, "out": buf.getvalue(), "chain_ok": g3.verify_chain(d / "session.jsonl"),
            "runner": runner, "error": err, "events": events}


class PlannerTap:
    """The real planner behind hooks: before[name](*args) runs first; replace[name](real_call, *args) answers instead."""

    def __init__(self):
        import ports
        from common import paths as _paths
        self.real = ports.PlannerPort(_paths.ROOT, False, None, None)
        self.cfg, self.before, self.replace, self.calls = self.real.cfg, {}, {}, []

    def _call(self, name: str, *args):
        self.calls.append(name)
        if name in self.before:
            self.before[name](*args)
        if name in self.replace:
            return self.replace[name](getattr(self.real, name), *args)
        return getattr(self.real, name)(*args)

    def plan_handover(self, q6, palm):
        return self._call("plan_handover", q6, palm)

    def plan_reorient(self, q6, palm):
        return self._call("plan_reorient", q6, palm)

    def retarget(self, plan, palm, q6):
        return self._call("retarget", plan, palm, q6)

    def blade_toward_palm(self, q6, palm) -> bool:
        return self.real.blade_toward_palm(q6, palm)

    def describe(self) -> dict:
        return self.real.describe()


class LinkTap:
    """The real HandLink; `override(snapshot) -> snapshot` lets a fixture change what the runner reads."""

    def __init__(self, real):
        self._real, self.override = real, None

    def __getattr__(self, name):
        return getattr(self._real, name)

    def snapshot(self) -> dict:
        s = self._real.snapshot()
        return self.override(s) if self.override is not None else s


def tap_factory(holder: dict):
    def factory(addr, on_stale, known_frame):
        import ports
        holder["link"] = LinkTap(ports.real_link(addr, on_stale, known_frame))
        return holder["link"]
    return factory


def froze_once(runner) -> bool:
    return runner.motion is not None and runner.motion.n_freeze >= 1


def first_index(rows: list, pred) -> int:
    return next(i for i, r in enumerate(rows) if pred(r))
