"""The control loop: table rows replayed at 15 Hz with the eight monitors read on every step.

Mirrors anchor/null_session.py's replay / move_to / hold_now / fire / log_kill: observation k follows action k; a
fire or a stop freezes the arm at that step (every goal := the present position) and the freeze row is written
before anything is shown. After a freeze this file sends nothing: what happens next is a key (ask_menu.py).
Every step is one row of the session's chain: joints, target, TCP (FK), phase, the eight monitor states, the hand
link's age and the seq of the hand message and frame that step used.
"""
from __future__ import annotations

import time

from common import paths

paths.use_anchor()
import g3  # noqa: E402
import null_plan  # noqa: E402
from safety import HardAbort, StopRequested  # noqa: E402

import numpy as np

import monitors as mon_mod
from common.table_map import _inside
from grasp_segment import CONTROL_HZ

SPIN_S = 0.025                   # the last 25 ms of every period are waited out awake (see wait_until)
MIN_BODY_MM = 15.0               # every moving body but the fingertip, above the table (the null planner's rule)
MIN_TIP_MM = 8.0                 # the fingertip itself (the null grasps at 12 mm)
MIN_TIP_LOW_MM = 3.0             # the dawn grasp / place rows: the jaws around a 21 mm round handle, fingertip 5 mm up
#                                  (the operator's choice, 2026-10-02 10:33)


def segment_problems(guard: g3.EnvelopeGuard, rows: list, box_on: bool = True, limit: int = 3,
                     visible: tuple = (), tip_floor_mm: float = MIN_TIP_MM) -> list:
    """FK of every row of a segment this runner built itself, before it runs: joint margins, the fingertip inside the
    workspace box, every moving body above the table. Empty = clear."""
    probs, lim, q0, tz = [], guard.limits(), guard.q_zero, guard.table_z
    x0, x1, y0, y1 = guard.box
    for k, r in enumerate(rows):
        for i, j in enumerate(g3.ALL):
            if not (lim[j][0] <= float(r[i]) <= lim[j][1]):
                probs.append(f"row {k + 1}: {j} {float(r[i]):.1f} outside [{lim[j][0]:.1f}, {lim[j][1]:.1f}]")
        pts = null_plan.body_points_mm([float(r[i]) - q0[i] for i in range(5)], float(r[5]))
        tip = pts["gripperframe"]
        if box_on and not (x0 <= tip[0] <= x1 and y0 <= tip[1] <= y1 and guard.z_lo <= tip[2] <= guard.z_hi):
            probs.append(f"row {k + 1}: fingertip ({tip[0]:.0f}, {tip[1]:.0f}, {tip[2]:.0f}) mm outside the workspace box")
        elif box_on and visible and not _inside(np.asarray(visible, float), tip[:2]):
            probs.append(f"row {k + 1}: fingertip ({tip[0]:.0f}, {tip[1]:.0f}) mm outside the camera's footprint "
                         "(reviewer 2026-10-01, question 11)")
        low = min((p[2] - tz, n) for n, p in pts.items() if n != "gripperframe")
        if low[0] < MIN_BODY_MM or tip[2] - tz < tip_floor_mm:
            probs.append(f"row {k + 1}: {low[1]} {low[0]:.0f} mm / fingertip {tip[2] - tz:.0f} mm above the table")
        if len(probs) >= limit:
            break
    return probs


def wait_until(clock, t_next: float) -> None:
    """Sleep to SPIN_S before the next step, then wait awake. time.sleep on this Mac wakes about 9 ms late (measured:
    p50 8.8, max 14.7 ms on a 66.7 ms sleep); that lateness would be read by monitor 5 as loop time."""
    clock.sleep(max(0.0, t_next - clock.now() - SPIN_S))
    while clock.now() < t_next:
        pass


class BusFault(RuntimeError):
    """A bus read failed inside a control step: a hardware fault, frozen like any other fire."""


class Motion:
    def __init__(self, rig, guard: g3.EnvelopeGuard, chain, link, pump, session_id: str, say, cue=None):
        self.rig, self.guard, self.chain, self.link, self.pump = rig, guard, chain, link, pump
        self.session_id, self.say, self.cue = session_id, say, cue
        self.t0 = rig.clock.now()
        self.k_total = 0
        self.last_obs: dict | None = None
        self.last_mon: dict | None = None
        self.last_freeze: dict | None = None
        self.n_freeze = 0
        self.dts: list = []
        self.approved_palm = None
        self.hand_seen_t = None                        # when a hand was last in a message (monitor 7's occlusion rule)
        self.stamp = None                              # --operator-override: written on every step row
        self.monitors_off: tuple = ()                  # --layer-off: (7, 8), reported OFF on every step row
        self.covering = False                          # palm placement: the arm or the prop over the approved palm
        self.drift_unreliable = False                  # palm placement: the hand as measured is under the arm
        self.trial_id: str | None = None
        self.force_fire: tuple | None = None           # (monitor number, detail): named at the next observation
        self.last_hold: tuple = (True, 0)              # (held, attempts) of the latest goals := present
        self.pending_row: dict | None = None
        self.pre_move = None                           # () -> refusal or None: read before every motion call
        #                                                (start_checks.MOTION_HOURS on the arm, if set; reviewer 2026-10-01)

    # -- one observation --------------------------------------------------------------------------------
    def observe(self, phase: str, t_prev, q_prev, target, box_on: bool, vel_limit, live_only: bool = False,
                defer_row: bool = False) -> tuple:
        """Read the arm and the link, evaluate the eight, write the step row. -> (t, q6, monitors, snapshot).
        live_only (the frozen arm's re-reads): the stop flag that froze the arm is not itself counted - the question
        there is whether the condition is still present. defer_row (run_rows): the row is kept in self.pending_row and
        written by step_row() with the target that is about to be sent - or with none, when nothing is sent."""
        rig = self.rig
        t = rig.clock.now()
        try:
            q6, load = rig.read_q(), rig.read_load()
        except (HardAbort, KeyboardInterrupt):
            raise
        except Exception as e:  # noqa: BLE001 - comms loss
            raise BusFault(f"bus read failed: {type(e).__name__}: {e}") from e
        dt = None if t_prev is None else t - t_prev
        snap = self.link.snapshot()
        if (snap.get("msg") or {}).get("hands"):
            self.hand_seen_t = t
        unseen = (t - self.hand_seen_t) / max(1e-9, rig.clock.scale) if self.hand_seen_t is not None else float("inf")
        self.guard.ee_box_on = box_on
        stopped = rig.kill.stop_requested.is_set() and not live_only
        mon = mon_mod.evaluate(self.guard, t - self.t0, q6, q_prev, dt, load, phase, stopped,
                               rig.kill.source, snap, self.link.monitor8(), self.approved_palm,
                               heartbeat_due=rig.interlock.heartbeat_due() and not live_only, vel_limit=vel_limit,
                               unseen_s=unseen, covering=self.covering, drift_unreliable=self.drift_unreliable,
                               off=self.monitors_off)
        if self.force_fire is not None and not live_only:
            n, why = self.force_fire
            mon[str(n)] = {**mon[str(n)], "fired": True, "detail": why}
            self.force_fire = None
        if stopped and not mon_mod.fired_list(mon):            # a stop nobody has named: monitor 6
            mon["6"] = {"code": "kill_switch", "fired": True, "detail": f"stop flag set ({rig.kill.source})"}
        msg = snap.get("msg") or {}
        tcp = self.guard.ee_mm(q6)
        self.k_total += 1
        if dt is not None:
            self.dts.append(1000.0 * dt)
        self.pending_row = {"kind": "step", "k": self.k_total, "t_s": round(t - self.t0, 4),
                            "dt_ms": None if dt is None else round(1000.0 * dt, 2), "phase": phase,
                            "q6": [round(v, 3) for v in q6], "target6": None,
                            "tcp_mm": [round(float(v), 1) for v in tcp], "load": load, "monitors": mon,
                            "hb_age_ms": None if snap.get("age_ms") is None else round(float(snap["age_ms"]), 1),
                            "hand_seq": msg.get("seq"), "frame_seq": msg.get("frame_seq")}
        if msg.get("hands"):                                   # where the monitor put each hand (the replay's marker)
            self.pending_row["hands_mm"] = [h.get("palm_mm") for h in msg["hands"]]
        if self.stamp:
            self.pending_row["override"] = self.stamp
        self.last_obs = {"q6": q6, "tcp": tcp, "load": load, "snap": snap, "phase": phase}
        self.last_mon = mon
        if not defer_row:
            self.step_row(None)
        return t, q6, mon, snap

    def step_row(self, target) -> None:
        """The step's row, written BEFORE its target is sent: target6 is what this step sends, null when it sends
        nothing (a fire, a stationary state, the end of a segment)."""
        row, self.pending_row = self.pending_row, None
        if row is not None:
            row["target6"] = None if target is None else [round(float(v), 3) for v in target]
            self.chain.append(row)

    # -- the freeze -------------------------------------------------------------------------------------
    def freeze(self, phase: str, fired: list, q6, snap: dict, source: str | None = None) -> dict:
        """kill.trigger -> goals := present -> the freeze row. Before any menu, before the judge."""
        rig = self.rig
        num, code, detail = fired[0]
        src = source or (f"envelope:{code}" if num <= 5 else f"monitor{num}:{code}")
        if not rig.kill.stop_requested.is_set():
            rig.kill.trigger(src)
        held, attempts = rig.hold_now()                        # retried up to 3 times, 50 ms apart
        self.last_hold = (held, attempts)
        t_hold = time.time()
        self.link.arm_watchdog(False)
        msg = (snap or {}).get("msg") or {}
        self.n_freeze += 1
        photo = self.pump.save(self.chain.path.parent / "photos", f"freeze{self.n_freeze:02d}", msg.get("frame_seq"))
        lat = None if msg.get("frame_t") is None else round(1000.0 * (t_hold - float(msg["frame_t"])), 1)
        row = self.chain.append({
            "kind": "freeze", "t_iso": g3.now_iso(), "session_id": self.session_id, "trial_id": self.trial_id,
            "phase": phase, "monitor": num, "code": code, "detail": detail, "source": rig.kill.source,
            "fired": [{"monitor": n, "code": c, "detail": d} for n, c, d in fired], "held": held,
            "hold_attempts": attempts, "q6": [round(float(v), 3) for v in q6],
            "tcp_mm": [round(float(v), 1) for v in self.guard.ee_mm(q6)] if len(q6) >= 5 else None,
            "hand_seq": msg.get("seq"), "frame_seq": msg.get("frame_seq"), "frame_sha256": msg.get("frame_sha256"),
            "hands": msg.get("hands"), "hb_age_ms": (snap or {}).get("age_ms"), "t_hold_minus_frame_t_ms": lat,
            "photo": photo["file"], "photo_sha256": photo["sha256"], "photo_is_the_message_frame": photo["exact"]})
        if num == 6 or code in ("hardware",):                 # G3.4: the kill row for an operator / hardware stop
            g3.kill_row(self.chain, "operator" if num == 6 else "hardware", q6, self.trial_id, None,
                        None, self.session_id, f"{rig.kill.source} during {phase}: {detail}")
        self.say(f"  >>> FROZEN in {phase}: monitor {num} ({code}) - {detail}")
        if not held:
            self.say(f"  >>> THE HOLD FAILED ({attempts} attempts): THE GOALS MAY NOT BE THE PRESENT POSITION.")
        self.last_freeze = row
        return row

    # -- a segment --------------------------------------------------------------------------------------
    def run_rows(self, phase: str, rows: list, box_on: bool = True, vel_limit: float | None = None,
                 on_step=None, stationary: bool = False) -> dict:
        """Replay `rows` (lerobot deg / %, one per control step).
        -> {'end': 'done' | 'frozen', 'k', 'rows', 'freeze', 'sent': the targets written, in order}.
        on_step(k, q6, snap) may return a replacement for the rows not yet sent (a silent re-target). vel_limit: monitor
        2's deg/s, or a callable k -> deg/s (per row: the placement's 10 and 3 deg/s rows)."""
        rig, clock, kill = self.rig, self.rig.clock, self.rig.kill
        period = 1.0 / CONTROL_HZ
        if self.cue:
            self.cue(phase)
        rows = [list(r) for r in rows]
        self.guard._grip_over_since = None
        self.link.arm_watchdog(not stationary)
        t_prev = q_prev = None
        t_next = clock.now()
        k, sent, q6, snap = 0, [], [], {}

        def frozen(code: str, detail: str, source: str) -> dict:
            self.step_row(None)
            q_last = q6 or (self.last_obs or {}).get("q6") or []
            fr = self.freeze(phase, [(0, code, detail)], q_last, snap or (self.last_obs or {}).get("snap") or {}, source)
            return {"end": "frozen", "k": k, "rows": rows, "freeze": fr, "sent": sent}

        why = self.pre_move() if self.pre_move is not None else None
        if why:                                            # nothing is sent: a freeze row names the reason
            self.link.arm_watchdog(False)
            q6, snap = rig.read_q(), self.link.snapshot()
            return frozen("motion_hours", f"{phase} not started: {why}", "hours")
        try:
            while True:
                if kill.hard_abort.is_set():
                    raise HardAbort("second Ctrl-C: hard abort, arm left where it is")
                try:
                    lim = vel_limit(k) if callable(vel_limit) else vel_limit
                    t, q6, mon, snap = self.observe(phase, t_prev, q_prev, None, box_on, lim, defer_row=True)
                except BusFault as e:                           # comms loss: a hardware fault, frozen like any other
                    return frozen("hardware", str(e), "hardware:comms")
                fired = mon_mod.fired_list(mon)
                if fired:
                    self.step_row(None)                        # this step sends nothing
                    return {"end": "frozen", "k": k, "rows": rows, "freeze": self.freeze(phase, fired, q6, snap),
                            "sent": sent}
                if on_step is not None:
                    try:
                        new = on_step(k, q6, snap)
                    except (HardAbort, KeyboardInterrupt):
                        raise
                    except Exception as e:  # noqa: BLE001 - the re-target code failed: the arm does not go on
                        return frozen("internal", f"on_step raised {type(e).__name__}: {e}", "internal:on_step")
                    if self.force_fire is not None:            # on_step's own finding: nothing is sent this step
                        n, why = self.force_fire
                        self.force_fire = None
                        self.step_row(None)
                        return {"end": "frozen", "k": k, "rows": rows, "sent": sent,
                                "freeze": self.freeze(phase, [(n, mon_mod.CODES.get(n, "internal"), why)], q6, snap,
                                                      f"{mon_mod.CODES.get(n, 'internal')}:on_step")}
                    if new is not None:
                        rows, k = [list(r) for r in new], 0
                if k >= len(rows):
                    self.step_row(None)
                    return {"end": "done", "k": k, "rows": rows, "freeze": None, "sent": sent}
                self.step_row(rows[k])                         # logged, then sent
                try:
                    rig.write_goal(rows[k])
                    sent.append(list(rows[k]))
                except StopRequested:
                    continue                                   # named at the loop top
                except (HardAbort, KeyboardInterrupt):
                    raise
                except Exception as e:  # noqa: BLE001
                    return frozen("hardware", f"bus write failed: {type(e).__name__}: {e}", "hardware:comms")
                k += 1
                q_prev, t_prev = q6, t
                t_next += period
                wait_until(clock, t_next)
        except (HardAbort, KeyboardInterrupt):
            rig.hold_now(tries=1)                              # as SafetyContext.checkpoint: hold first, then leave
            self.step_row(None)
            raise
        except Exception as e:  # noqa: BLE001 - anything else in the control path: held, with a row
            return frozen("internal", f"the control step raised {type(e).__name__}: {e}", "internal:step")
        finally:
            self.link.arm_watchdog(False)

    def hold_rows(self, phase: str, seconds: float, until=None) -> dict:
        """A stationary state with the monitors read at 15 Hz: nothing is commanded (the goal stays where it is).
        until(mon, snapshot) -> True ends it early. Monitor 7 does not fire here; 1-6 and 8 do."""
        rig, clock = self.rig, self.rig.clock
        period = 1.0 / CONTROL_HZ
        if self.cue:
            self.cue(phase)
        t_prev = q_prev = None
        t_end = clock.now() + seconds
        t_next = clock.now()
        q6, snap, mon = [], {}, None
        try:
            while True:
                if rig.kill.hard_abort.is_set():
                    raise HardAbort("second Ctrl-C: hard abort, arm left where it is")
                t, q6, mon, snap = self.observe(phase, t_prev, q_prev, None, True, None)
                fired = mon_mod.fired_list(mon)
                if fired:
                    return {"end": "frozen", "freeze": self.freeze(phase, fired, q6, snap), "mon": mon}
                if until is not None and until(mon, snap):
                    return {"end": "until", "mon": mon}
                if clock.now() >= t_end:
                    return {"end": "done", "mon": mon}
                q_prev, t_prev = q6, t
                t_next += period
                wait_until(clock, t_next)
        except (HardAbort, KeyboardInterrupt):
            rig.hold_now(tries=1)
            raise
        except Exception as e:  # noqa: BLE001 - a failed read or anything else while holding: frozen, with a row
            code = "hardware" if isinstance(e, BusFault) else "internal"
            q_last = q6 or (self.last_obs or {}).get("q6") or []
            fr = self.freeze(phase, [(0, code, str(e) if code == "hardware" else f"{type(e).__name__}: {e}")], q_last,
                             snap or (self.last_obs or {}).get("snap") or {}, f"{code}:hold")
            return {"end": "frozen", "freeze": fr, "mon": mon}

    def read_monitors(self, phase: str) -> tuple:
        """One read with a row and no command: WAIT's re-check, and the gate before a moving state."""
        _, q6, mon, snap = self.observe(phase, None, None, None, True, None, live_only=True)
        return q6, mon, snap
