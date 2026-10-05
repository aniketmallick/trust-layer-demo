"""The receiving half of the hand monitor's protocol, inside the runner (PLAN.md section 5; monitor 8 of section 4).

    link = HandLink(on_stale=lambda why: kill.trigger("monitor8:" + why), known_frame=pump.known_frame)
    link.start(); link.arm_watchdog(True)      # armed in moving states only
    fired, detail = link.monitor8()            # every control step, for the step row
    snap = link.snapshot()                     # msg, age_ms, seq_gap, frame_age_ms, window, problems

Monitor 8 fires when any of these holds:
  - no well-formed message yet, or the last one was received more than stale_ms (300) ago;
  - the last two messages' seq differ by more than max_seq_gap (2: one lost message is tolerated, two are not) or
    by less than 1 (a repeated or backward seq: the monitor restarted or something else is sending);
  - the frame the last message was computed from is older than frame_stale_ms (500), or - when the runner gives its
    record of written frames - is not a frame the runner wrote (unknown seq or another hash).
A datagram that is not a well-formed message counts as no message: the age keeps growing.

Two threads: a listener (UDP) and a watchdog that looks every 10 ms and, while armed, calls on_stale(reason) once
per arming. The link never touches a bus: on_stale sets the runner's stop flag, SafeBus drops every target from
then on, and the control loop writes goals := present at its next step. stdlib only (it runs in the runner's venv).
"""
from __future__ import annotations

import collections
import socket
import threading
import time
from typing import Callable

from common import wire
from common.paths import UDP_ADDR

STALE_MS = 300.0
MAX_SEQ_GAP = 2
FRAME_STALE_MS = 500.0
TICK_S = 0.010
RECV_TIMEOUT_S = 0.05
MAX_DATAGRAM = 65535


class HandLink:
    def __init__(self, addr: tuple = UDP_ADDR, on_stale: Callable[[str], None] | None = None,
                 known_frame: Callable[[int], dict | None] | None = None, stale_ms: float = STALE_MS,
                 max_seq_gap: int = MAX_SEQ_GAP, frame_stale_ms: float = FRAME_STALE_MS, *,
                 clock: Callable[[], float] = time.monotonic, wall: Callable[[], float] = time.time,
                 tick_s: float = TICK_S):
        """known_frame(seq) -> {"sha256", "t"} of a frame the runner wrote, or None for a seq it did not write; it is
        called from the listener thread. known_frame=None skips that check (the dry run's sim feed has no frames).
        clock / wall are injectable for tests: clock measures message age, wall is compared with the frame's t."""
        self.addr = tuple(addr)
        self.on_stale, self.known_frame = on_stale, known_frame
        self.stale_ms, self.max_seq_gap, self.frame_stale_ms = float(stale_ms), int(max_seq_gap), float(frame_stale_ms)
        self._clock, self._wall, self._tick_s = clock, wall, float(tick_s)
        self._lock = threading.Lock()
        self._msg: dict | None = None
        self._t_recv: float | None = None
        self._seq_gap = 0
        self._frame_t: float | None = None
        self._frame_problem: str | None = None
        self._window: collections.deque = collections.deque(maxlen=wire.DEBOUNCE_N)
        self._malformed = 0
        self._last_malformed = ""
        self.n_valid = 0
        self._armed = False
        self._called = False
        self.last_stale: dict | None = None        # {"reason", "t_clock"} of the watchdog's latest on_stale call
        self._stop = threading.Event()
        self._sock: socket.socket | None = None
        self._threads: list = []

    # -- lifecycle ----------------------------------------------------------------------------------------
    def start(self) -> None:
        """Bind the UDP port and start the listener and the watchdog. Port 0 binds a free port (see self.addr)."""
        if self._sock is not None:
            raise RuntimeError("HandLink already started")
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(self.addr)
        s.settimeout(RECV_TIMEOUT_S)
        self._sock, self.addr = s, s.getsockname()
        self._stop.clear()
        self._threads = [threading.Thread(target=self._listen, name="hand-link-listen", daemon=True),
                         threading.Thread(target=self._watch, name="hand-link-watchdog", daemon=True)]
        for t in self._threads:
            t.start()

    def stop(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        self._threads = []
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def arm_watchdog(self, armed: bool) -> None:
        """While armed, the watchdog calls on_stale(reason) the first time monitor 8 fires; arming again re-arms it."""
        with self._lock:
            self._armed = bool(armed)
            self._called = False

    # -- the two threads' work (also called directly by the fixtures) ---------------------------------------
    def feed(self, datagram: bytes) -> bool:
        """One received datagram. -> True if it was a well-formed message and became the latest."""
        msg = wire.decode(datagram)
        probs = wire.validate_hand_msg(msg)
        if probs:
            with self._lock:
                self._malformed += 1
                self._last_malformed = probs[0]
            return False
        frame_t, frame_problem = float(msg["frame_t"]), None
        if self.known_frame is not None:
            try:
                rec = self.known_frame(int(msg["frame_seq"]))
            except Exception as e:  # noqa: BLE001 - the runner's record failing is a problem with the frame, not a crash
                rec, frame_problem = None, f"the runner's frame record raised {type(e).__name__}"
            if rec is None:
                frame_problem = frame_problem or f"frame_seq {msg['frame_seq']} is not a frame the runner wrote"
            elif rec.get("sha256") != msg["frame_sha256"]:
                frame_problem = f"frame_seq {msg['frame_seq']}: its hash is not the hash of the frame the runner wrote"
            else:
                frame_t = float(rec.get("t", frame_t))      # the runner's own time for that frame
        now = self._clock()
        with self._lock:
            self._seq_gap = 1 if self._msg is None else int(msg["seq"]) - int(self._msg["seq"])
            self._msg, self._t_recv = msg, now
            self._frame_t, self._frame_problem = frame_t, frame_problem
            self._window.append(any(h["in_workspace"] for h in msg["hands"]))
            self._malformed = 0
            self.n_valid += 1
        return True

    def tick(self) -> bool:
        """One look by the watchdog. -> True if it called on_stale now."""
        fired, detail = self.monitor8()
        with self._lock:
            call = fired and self._armed and not self._called
            if call:
                self._called = True
                self.last_stale = {"reason": detail, "t_clock": self._clock()}
        if call and self.on_stale is not None:
            self.on_stale(detail)
        return call

    def _listen(self) -> None:
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(MAX_DATAGRAM)
            except socket.timeout:
                continue
            except OSError:
                return                                       # the socket was closed by stop()
            self.feed(data)

    def _watch(self) -> None:
        while not self._stop.wait(self._tick_s):
            self.tick()

    # -- what the runner reads -----------------------------------------------------------------------------
    def snapshot(self) -> dict:
        with self._lock:
            age = None if self._t_recv is None else 1000.0 * (self._clock() - self._t_recv)
            frame_age = None if self._frame_t is None else 1000.0 * (self._wall() - self._frame_t)
            problems = [self._frame_problem] if self._frame_problem else []
            if self._malformed:
                problems.append(f"{self._malformed} malformed datagram(s) since the last message: {self._last_malformed}")
            return {"msg": self._msg, "age_ms": age, "seq_gap": self._seq_gap, "frame_age_ms": frame_age,
                    "window": list(self._window), "problems": problems}

    def monitor8(self) -> tuple[bool, str]:
        """(fired, detail) - the rule in the module docstring, on the state as it is now."""
        s = self.snapshot()
        if s["msg"] is None:
            why = "no hand message received yet"
            return True, why + (f" ({s['problems'][-1]})" if s["problems"] else "")
        fired = []
        if s["age_ms"] > self.stale_ms:
            fired.append(f"last hand message {s['age_ms']:.0f} ms old (> {self.stale_ms:.0f})")
        if s["seq_gap"] > self.max_seq_gap:
            fired.append(f"seq jumped by {s['seq_gap']} (> {self.max_seq_gap})")
        elif s["seq_gap"] < 1:
            fired.append(f"seq went back or repeated ({s['seq_gap']:+d})")
        if s["frame_age_ms"] > self.frame_stale_ms:
            fired.append(f"the message's frame is {s['frame_age_ms']:.0f} ms old (> {self.frame_stale_ms:.0f})")
        fired += [p for p in s["problems"] if "malformed" not in p]
        if fired:
            return True, "; ".join(fired)
        return False, (f"ok: message {s['age_ms']:.0f} ms old, seq step {s['seq_gap']}, frame "
                       f"{s['frame_age_ms']:.0f} ms old")
