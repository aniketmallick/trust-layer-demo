#!/usr/bin/env python
"""Monitor 8 (common/hand_link.py): seen to fire and seen not to fire, and the heartbeat-kill test.

    ~/lerobot-mps-venv/bin/python -m pytest tests/test_hand_link.py -q -p no:cacheprovider -s
    ~/lerobot-mps-venv/bin/python tests/test_hand_link.py          # the same fixtures as a table

  LINK  the rule on a fake clock (no sockets, no sleeps): silence, seq steps, frame age, frames the runner did not
        write, malformed datagrams, the watchdog's arming; then the real socket on loopback
  KILL  a real publisher process is killed (SIGKILL) while a 15 Hz loop stands in for the runner's control loop.
        Three times are measured: last message -> stop flag, kill -> stop flag, stop flag -> the step that would
        write goals := present. "Within 300 ms of the kill" cannot be the bar (PLAN.md F5): silence is only known
        300 ms after the LAST MESSAGE, which left up to one publishing period before the kill.
"""
from __future__ import annotations

import os
import random
import socket
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

sys.dont_write_bytecode = True
EXP = Path(__file__).resolve().parent.parent
if str(EXP) not in sys.path:
    sys.path.insert(0, str(EXP))

from common import wire  # noqa: E402
from common.hand_link import FRAME_STALE_MS, STALE_MS, TICK_S, HandLink  # noqa: E402
from monitor.fixture_registry import Unknown, registry, run_all  # noqa: E402

REGISTRY, fixture = registry()
SHA = "ab" * 32
CONTROL_HZ = 15.0
SCHED_SLACK_MS = 15.0          # thread scheduling on a desktop OS, allowed on top of each bar and stated in the detail
MONITOR_PY = EXP / ".venv_monitor" / "bin" / "python"


class FakeTime:
    """One hand on both clocks: the link's monotonic clock and the wall clock the frame's t is compared with."""

    def __init__(self):
        self.ms = 0                    # whole milliseconds, so "exactly 300 ms" is exact

    def mono(self) -> float:
        return self.ms / 1000.0

    def wall(self) -> float:
        return 1_790_000_000.0 + self.ms / 1000.0

    def advance(self, ms: int) -> None:
        self.ms += int(ms)


def _msg(seq: int, ft: FakeTime | None = None, frame_seq: int | None = None, frame_age_ms: float = 60.0,
         in_workspace: bool = False, sha: str = SHA) -> bytes:
    wall = ft.wall() if ft is not None else time.time()
    hands = [{"handedness": "R", "conf": 0.9, "palm_px": [247.6, 109.5], "palm_mm": [241.1, -58.8],
              "in_workspace": True, "in_spawn": True, "in_zone": False}] if in_workspace else []
    meta = {"seq": seq if frame_seq is None else frame_seq, "sha256": sha, "t": wall - frame_age_ms / 1000.0}
    return wire.encode(wire.build_hand_msg(wall, seq, meta, hands, [in_workspace], "fixture", 20.0))


def _link(ft: FakeTime, **kw) -> tuple:
    calls: list = []
    link = HandLink(addr=("127.0.0.1", 0), on_stale=calls.append, clock=ft.mono, wall=ft.wall, **kw)
    return link, calls


# ======================================================================================== LINK: the rule, fake clock
@fixture("LINK", "LINK_no_message_yet_fires",
         "a link that has received nothing: monitor 8 is firing (a runner may not enter a moving state on it), and the "
         "armed watchdog calls on_stale")
def _none_yet():
    ft = FakeTime()
    link, calls = _link(ft)
    fired, detail = link.monitor8()
    link.arm_watchdog(True)
    link.tick()
    return fired and len(calls) == 1 and link.snapshot()["msg"] is None, f"fired {fired}: {detail}; on_stale calls {len(calls)}"


@fixture("LINK", "LINK_nominal_10hz_with_jitter_never_fires",
         "300 messages 100 ms apart with +/-30 ms of jitter, frames 60 ms old, the watchdog armed and looking every "
         "10 ms: monitor 8 must never fire and on_stale must never be called", kind="control")
def _nominal():
    ft = FakeTime()
    link, calls = _link(ft)
    rng = random.Random(8)
    link.feed(_msg(1, ft))
    link.arm_watchdog(True)
    worst, fires = 0.0, 0
    for seq in range(2, 302):
        for _ in range(int(round((100 + rng.uniform(-30, 30)) / 10))):
            ft.advance(10)
            link.tick()
            fires += link.monitor8()[0]
            worst = max(worst, link.snapshot()["age_ms"])
        link.feed(_msg(seq, ft))
    return fires == 0 and not calls, f"{fires} fire(s) in 300 messages; oldest message seen {worst:.0f} ms; on_stale calls {len(calls)}"


@fixture("LINK", "LINK_silence_fires_past_300ms",
         "one message, then silence: at 300 ms monitor 8 has not fired, at 310 ms (the watchdog's next look) it has, and "
         "on_stale is called exactly once however long the silence lasts")
def _silence():
    ft = FakeTime()
    link, calls = _link(ft)
    link.feed(_msg(1, ft))
    link.arm_watchdog(True)
    at = {}
    for ms in range(10, 1001, 10):
        ft.advance(10)
        link.tick()
        at[ms] = link.monitor8()[0]
    first = min(ms for ms, f in at.items() if f)
    ok = (not at[300]) and at[310] and first == 310 and len(calls) == 1 and "ms old" in calls[0]
    return ok, f"fired at 300 ms: {at[300]}; first fire at {first} ms; on_stale calls in 1 s of silence: {len(calls)} ({calls[0] if calls else ''})"


@fixture("LINK", "LINK_seq_step_2_tolerated_3_fires",
         "seq 1, 2, then 4 (one message lost): no fire; then 7 (two lost, a step of 3): fires; then 8: clear again; then "
         "8 again (a repeat) and 3 (backward): each fires")
def _seq():
    ft = FakeTime()
    link, _ = _link(ft)
    seen = []
    for seq in (1, 2, 4, 7, 8, 8, 3):
        ft.advance(100)
        link.feed(_msg(seq, ft))
        seen.append((seq, link.snapshot()["seq_gap"], link.monitor8()[0]))
    fired = [f for _, _, f in seen]
    return fired == [False, False, False, True, False, True, True], f"(seq, step, fired): {seen}"


@fixture("LINK", "LINK_stale_frame_fires",
         "fresh messages whose frame is 400 ms old: no fire; whose frame is 600 ms old (a live monitor reading a frame "
         "that stopped changing): fires although the message itself is fresh")
def _stale_frame():
    ft = FakeTime()
    link, calls = _link(ft)
    link.feed(_msg(1, ft, frame_age_ms=400.0))
    ok_400 = not link.monitor8()[0]
    ft.advance(100)
    link.feed(_msg(2, ft, frame_age_ms=600.0))
    fired, detail = link.monitor8()
    link.arm_watchdog(True)
    link.tick()
    return ok_400 and fired and link.snapshot()["age_ms"] < 1.0 and len(calls) == 1, f"400 ms frame: no fire {ok_400}; 600 ms frame: {detail}"


@fixture("LINK", "LINK_frame_the_runner_did_not_write_fires",
         "the runner's record of written frames holds seq 5 with one hash: a message for seq 5 with that hash does not "
         "fire; one for seq 6 (never written) fires; one for seq 5 with another hash fires; with no record given "
         "(the dry run's sim feed) the same messages do not fire")
def _unknown_frame():
    ft = FakeTime()
    written = {5: {"sha256": SHA, "t": ft.wall() - 0.05}}
    link, _ = _link(ft, known_frame=written.get)
    link.feed(_msg(1, ft, frame_seq=5))
    good = link.monitor8()
    ft.advance(100)
    link.feed(_msg(2, ft, frame_seq=6))
    unknown = link.monitor8()
    ft.advance(100)
    link.feed(_msg(3, ft, frame_seq=5, sha="cd" * 32))
    other_hash = link.monitor8()
    free, _ = _link(ft)
    free.feed(_msg(1, ft, frame_seq=6))
    skipped = free.monitor8()
    ok = (not good[0]) and unknown[0] and other_hash[0] and not skipped[0]
    return ok, f"written frame: fired {good[0]}; unwritten seq: {unknown[1]}; other hash: {other_hash[1]}; no record: fired {skipped[0]}"


@fixture("LINK", "LINK_malformed_datagram_is_silence",
         "one good message, then every 100 ms a datagram that is not a well-formed message (not JSON; JSON with a field "
         "missing; a hand without palm_mm): none becomes the latest message, the age keeps growing and monitor 8 fires "
         "past 300 ms")
def _malformed():
    ft = FakeTime()
    link, calls = _link(ft)
    link.feed(_msg(1, ft))
    link.arm_watchdog(True)
    no_field = wire.decode(_msg(2, ft))
    del no_field["frame_sha256"]
    bad_hand = wire.decode(_msg(2, ft, in_workspace=True))
    del bad_hand["hands"][0]["palm_mm"]
    accepted = []
    for bad in (b"{not json", wire.encode(no_field), wire.encode(bad_hand), b""):
        ft.advance(100)
        accepted.append(link.feed(bad))
        link.tick()
    snap = link.snapshot()
    fired, detail = link.monitor8()
    ok = not any(accepted) and snap["msg"]["seq"] == 1 and snap["age_ms"] >= 400 and fired and len(calls) == 1 \
        and any("malformed" in p for p in snap["problems"])
    return ok, f"accepted {accepted}; latest seq {snap['msg']['seq']}, {snap['age_ms']:.0f} ms old; {detail}; problems {snap['problems']}"


@fixture("LINK", "LINK_watchdog_calls_once_per_arming_only_when_armed",
         "silence past 300 ms with the watchdog NOT armed (a stationary state): on_stale is not called, monitor8() still "
         "reports the fire; armed: called once; 50 more looks: not again; armed again: called again")
def _arming():
    ft = FakeTime()
    link, calls = _link(ft)
    link.feed(_msg(1, ft))
    ft.advance(400)
    link.tick()
    unarmed = (len(calls), link.monitor8()[0])
    link.arm_watchdog(True)
    link.tick()
    once = len(calls)
    for _ in range(50):
        ft.advance(10)
        link.tick()
    still = len(calls)
    link.arm_watchdog(True)
    link.tick()
    again = len(calls)
    ok = unarmed == (0, True) and once == 1 and still == 1 and again == 2 and link.last_stale is not None
    return ok, f"unarmed: {unarmed[0]} call(s), fired {unarmed[1]}; armed: {once}; after 50 looks: {still}; re-armed: {again}"


@fixture("LINK", "LINK_window_holds_the_last_five_raw_results",
         "seven messages, a hand in the workspace in the 3rd, 5th and 7th: the link's window is the last five raw "
         "results, oldest first, and 3-of-5 over it is true", kind="control")
def _window():
    ft = FakeTime()
    link, _ = _link(ft)
    for seq in range(1, 8):
        ft.advance(100)
        link.feed(_msg(seq, ft, in_workspace=seq in (3, 5, 7)))
    w = link.snapshot()["window"]
    return w == [True, False, True, False, True] and wire.debounce(w)["hand_in_workspace"], f"window {w} -> {wire.debounce(w)}"


# ======================================================================================== LINK: the real socket
@fixture("LINK", "LINK_loopback_10hz_real_clock_never_fires",
         "the started link on a real UDP port, 20 messages 100 ms apart on the real clock, the watchdog armed after the "
         "first: every message arrives, monitor 8 never fires, on_stale is never called", kind="control")
def _loopback():
    calls: list = []
    link = HandLink(addr=("127.0.0.1", 0), on_stale=calls.append)
    link.start()
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fires, worst = 0, 0.0
    try:
        for seq in range(1, 21):
            tx.sendto(_msg(seq), link.addr)
            time.sleep(0.02)
            if seq == 1:
                link.arm_watchdog(True)
            for _ in range(8):
                fires += link.monitor8()[0]
                worst = max(worst, link.snapshot()["age_ms"] or 0.0)
                time.sleep(0.01)
        n = link.n_valid
    finally:
        tx.close()
        link.stop()
    return n == 20 and fires == 0 and not calls, f"{n} of 20 received; {fires} fire(s); oldest message seen {worst:.0f} ms; on_stale calls {len(calls)}"


# ======================================================================================== KILL: the publisher dies
_PUBLISHER = r"""
import sys, time
sys.dont_write_bytecode = True
sys.path.insert(0, sys.argv[1])
import socket
from common import wire
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
seq = 0
while True:
    seq += 1
    t = time.time()
    m = wire.build_hand_msg(t, seq, {"seq": seq, "sha256": "ab" * 32, "t": t - 0.05}, [], [False], "minimal publisher", 1.0)
    s.sendto(wire.encode(m), ("127.0.0.1", int(sys.argv[2])))
    time.sleep(0.1)
"""


class FrameWriter:
    """Stands in for the runner's frame pump: a jpg + json pair at 15 Hz, and the record of what was written."""

    def __init__(self, frames_dir: Path):
        import cv2
        import numpy as np
        self.dir = frames_dir
        self.jpg = cv2.imencode(".jpg", np.full((480, 640, 3), (196, 200, 204), np.uint8))[1].tobytes()
        self.written: dict = {}
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        seq = 0
        while not self.stop.wait(1.0 / CONTROL_HZ):
            seq += 1
            meta = wire.write_frame(self.dir, self.jpg, seq, time.time())
            with self.lock:
                self.written[seq] = meta

    def known_frame(self, seq: int) -> dict | None:
        with self.lock:
            return self.written.get(seq)


def _kill_case(cmd_for_port, known_frame=None, warm_s: float = 30.0) -> dict:
    """Start the publisher, wait for 10 messages, arm the watchdog, run a 15 Hz loop that notes the step at which it
    would write goals := present, SIGKILL the publisher. -> the three times (ms) and what fired."""
    t = {}
    flag = threading.Event()

    def on_stale(reason: str) -> None:
        t["flag"] = time.monotonic()
        t["age_at_flag_ms"] = link.snapshot()["age_ms"]
        t["reason"] = reason
        flag.set()

    link = HandLink(addr=("127.0.0.1", 0), on_stale=on_stale, known_frame=known_frame)
    link.start()
    proc = subprocess.Popen(cmd_for_port(link.addr[1]), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"}, cwd=str(EXP))

    def control_loop() -> None:
        t_next = time.monotonic()
        while "hold" not in t and not done.is_set():
            if flag.is_set():
                t["hold"] = time.monotonic()         # the step that writes goals := present
                return
            t_next += 1.0 / CONTROL_HZ
            time.sleep(max(0.0, t_next - time.monotonic()))

    done = threading.Event()
    loop = threading.Thread(target=control_loop, daemon=True)
    try:
        t0 = time.monotonic()
        while link.n_valid < 10:
            if proc.poll() is not None:
                raise Unknown(f"the publisher exited with code {proc.returncode} before publishing")
            if time.monotonic() - t0 > warm_s:
                raise Unknown(f"the publisher sent {link.n_valid} message(s) in {warm_s:g} s")
            time.sleep(0.02)
        before = link.monitor8()
        link.arm_watchdog(True)
        loop.start()
        time.sleep(0.3 + random.random() * 0.1)       # the kill lands anywhere in a publishing period
        calls_before_kill = flag.is_set()
        t["kill"] = time.monotonic()
        proc.kill()
        flag.wait(2.0)
        loop.join(1.0)
        n_valid = link.n_valid
    finally:
        done.set()
        if proc.poll() is None:
            proc.kill()
        proc.wait(5)
        link.stop()
    if "flag" not in t or "hold" not in t:
        return {"froze": False, "detail": f"no stop flag within 2 s of the kill (fired before the kill: {calls_before_kill})"}
    return {"froze": True, "fired_before_kill": calls_before_kill or before[0], "messages": n_valid,
            "last_msg_to_flag_ms": t["age_at_flag_ms"], "kill_to_flag_ms": 1000.0 * (t["flag"] - t["kill"]),
            "flag_to_hold_ms": 1000.0 * (t["hold"] - t["flag"]), "reason": t["reason"]}


def _kill_verdict(r: dict) -> tuple:
    if not r["froze"]:
        return False, r["detail"]
    bar_flag = STALE_MS + 1000.0 * TICK_S + SCHED_SLACK_MS
    bar_hold = 1000.0 / CONTROL_HZ + SCHED_SLACK_MS
    ok = (not r["fired_before_kill"] and STALE_MS < r["last_msg_to_flag_ms"] <= bar_flag
          and r["kill_to_flag_ms"] <= bar_flag and r["flag_to_hold_ms"] <= bar_hold)
    return ok, (f"last message -> stop flag {r['last_msg_to_flag_ms']:.0f} ms (bar {STALE_MS:.0f} + {1000 * TICK_S:.0f} "
                f"tick + {SCHED_SLACK_MS:.0f} scheduling); kill -> stop flag {r['kill_to_flag_ms']:.0f} ms; stop flag -> "
                f"hold step {r['flag_to_hold_ms']:.0f} ms (bar one {CONTROL_HZ:g} Hz period {1000 / CONTROL_HZ:.0f} + "
                f"{SCHED_SLACK_MS:.0f}); kill -> hold step {r['kill_to_flag_ms'] + r['flag_to_hold_ms']:.0f} ms; "
                f"{r['messages']} messages before the kill; fired: {r['reason']}")


@fixture("KILL", "KILL_minimal_publisher_killed_runner_freezes",
         "a minimal publisher process (10 Hz, well-formed messages) is SIGKILLed while the watchdog is armed: the stop "
         "flag is set between 300 and 310 ms after the last message (+ scheduling), and the 15 Hz loop reaches its "
         "hold step within one control period of the flag")
def _kill_minimal():
    r = _kill_case(lambda port: [sys.executable, "-c", _PUBLISHER, str(EXP), str(port)])
    return _kill_verdict(r)


@fixture("KILL", "KILL_hand_monitor_process_killed_runner_freezes",
         "the real hand monitor (its own venv, MediaPipe) reading a temp frames dir a writer fills at 15 Hz is SIGKILLed "
         "while the watchdog is armed and the link checks every message's frame against the writer's record: same bars "
         "as the minimal publisher")
def _kill_monitor():
    if not MONITOR_PY.is_file():
        raise Unknown(".venv_monitor is not built (uv venv + uv pip install -r monitor/requirements.txt)")
    frames = Path(tempfile.mkdtemp(prefix="khv_kill_"))
    writer = FrameWriter(frames)
    writer.thread.start()
    try:
        r = _kill_case(lambda port: [str(MONITOR_PY), str(EXP / "monitor" / "hand_monitor.py"), "--frames-dir", str(frames),
                                     "--udp", f"127.0.0.1:{port}"], known_frame=writer.known_frame)
    finally:
        writer.stop.set()
        writer.thread.join(1.0)
    return _kill_verdict(r)


# ======================================================================================== pytest
def pytest_generate_tests(metafunc):
    if "f" in metafunc.fixturenames:
        metafunc.parametrize("f", REGISTRY, ids=[f.name for f in REGISTRY])


def test_fixture(f):
    import pytest
    try:
        ok, detail = f.fn()
    except Unknown as e:
        pytest.skip(str(e))
    print(f"{f.name}: {detail}")
    assert ok, f"{f.name} ({f.what}): {detail}"


def test_constants_are_the_plans():
    assert (STALE_MS, FRAME_STALE_MS, TICK_S) == (300.0, 500.0, 0.010)


if __name__ == "__main__":
    sys.exit(1 if run_all(REGISTRY) else 0)
