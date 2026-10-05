"""What happens after a freeze: judge -> ASK -> the operator's key -> (wait | handover | re-orient | kill | leave).

The half of the runner that is not the scripted task (handover_session.py holds the session and the task). Rules
kept here, all from PLAN.md section 2:
  - nothing is commanded between a freeze and a key; no key is pressed for the operator; the judge only recommends.
    One exception, the operator's own (2026-10-03): with --handover-trial the [h] is given at trial start, and the
    first hand event of a trial (a monitor-7 freeze, the prop held) takes it - after the judge, if the verdict is not
    UNSAFE; any refusal or an UNSAFE verdict holds and asks as below. [r] is always typed;
  - the stop flag is cleared AT the accepted [h] / [o] / [c] (ask_menu's on_key), never later: a stop that arrives
    after the key - ESC, Ctrl-C, the hand link's watchdog (armed from the key on) - sets it afresh, and the first
    observation of the next segment freezes before any target is written. Every way back to ASK without motion leaves
    the arm latched again (flag set, goals := present) with a row;
  - [c] is accepted only if all eight monitors are clear, monitor 7 at 0 of 5, in a read taken AFTER the key;
  - on the arm, [h] / [o] / [c] outside start_checks.MOTION_HOURS (if set) are refused with a row;
  - [h] is refused by code - back to ASK with the reason - on no hand / two hands / monitor 8 firing / the blade
    heading toward the palm / no plan;
  - the palm approved at the key stays the approved palm for everything that key allows ([o]: the re-orientation
    and the approach after it); drift is measured from it. The re-target is retarget.py;
  - RELEASE opens the gripper only; RETREAT starts only after 5 fresh hand-monitor frames with no hand in the
    workspace, counted from the end of the release (monitors.ClearWatch), behind the same gate as every other moving
    state that is not an approved phase. Monitor 7 stays live in RETREAT.
"""
from __future__ import annotations

import time

from common import paths

paths.use_anchor()
import g3  # noqa: E402

import ask_menu
import monitors as mon_mod
import start_checks
import state_machine as sm
from grasp_segment import GRIP_PPS, HANDOVER_DPS, interpolate
from motion import segment_problems
from retarget import RETARGET_MAX_S, RETARGET_MIN_MM, Retargeter, plan_view  # noqa: F401 - the flow's public numbers

HOLD_SETTLE_S = 1.0
WAIT_S = 1.0
HAND_GONE_S = 20.0               # one wait for the hand to leave before the reminder is printed again
MOTION_KEYS = ("h", "o", "c", "r")
PREAUTH = "pre-authorised at trial start (--handover-trial)"


def code_unsafe(v: dict) -> bool:
    """The code's reading of a verdict (PLAN.md section 2): the judge's own `unsafe` flag (timeout, malformed, not
    configured, ...) or p_unsafe >= 0.5 - and anything that cannot be read as a number is UNSAFE. `recommend` is not
    read here: a recommendation commands nothing."""
    try:
        return bool(v.get("unsafe")) or float(v.get("p_unsafe", 1.0)) >= 0.5
    except (TypeError, ValueError, AttributeError):
        return True


class Stop(Exception):
    """The trial and the session end here (kill, leave as is, or a refusal the session cannot go past)."""

    def __init__(self, why: str, outcome: str = "ABORTED", torque_off: bool = False):
        super().__init__(why)
        self.outcome, self.torque_off = outcome, torque_off


class Flow:
    """Mixed into HandoverSession: uses its rig, motion, chain, keys, judge, planner, link, say()."""

    # -- small pieces -----------------------------------------------------------------------------------
    def go(self, event: str, all_clear: bool = False) -> str:
        nxt, refusal = sm.next_state(self.state, event, self.resume_state, all_clear)
        if refusal:
            raise Stop(f"state machine refused: {refusal}")
        self.state = nxt
        if self.cues is not None and nxt in sm.STATIONARY:
            self.cues.send(nxt)
        return nxt

    def hands_now(self) -> tuple:
        snap = self.link.snapshot()
        return list((snap.get("msg") or {}).get("hands") or []), snap

    def ids(self) -> dict:
        return {"session_id": self.session_id, "trial_id": self.trial_id}

    def hours_refusal(self) -> str | None:
        """On the arm: start_checks.MOTION_HOURS (None = no window), read at every trial and every key that allows motion."""
        if not self.enforce_hours:
            return None
        t = self.now().astimezone(g3.IST)
        return None if start_checks.motion_hours_ok(t) else f"{t:%H:%M} IST: {start_checks.hours_text()}"

    def on_motion_key(self, key: str, verdict: dict | None) -> dict:
        """At the accepted key, before its row is written. [h] / [o] / [c]: the hours; then the stop flag is cleared
        HERE and the link's watchdog armed, so anything that stops the arm from now on is seen; [c]: the monitors are
        read again and the key is refused unless all eight are clear."""
        if key not in MOTION_KEYS:
            return {}
        why = self.hours_refusal()
        if why:
            return {"refused": why}
        out: dict = {}
        if key == "c" and verdict is not None and code_unsafe(verdict):
            # finding 14 (reviewer 2026-10-01): [c] past an UNSAFE verdict needs a fresh judge call that is not
            # UNSAFE - asked here, the arm still latched (the stop flag is cleared only after it)
            self.say("  [c] with an UNSAFE verdict on screen: the judge reads the scene again before anything moves.")
            fresh = self.judge_at("FREEZE")
            out["fresh_verdict"] = ask_menu.on_screen(fresh)
            if fresh["unsafe"]:
                return {**out, "refused": "the fresh judge call is UNSAFE as well: [c] needs a verdict that is not "
                                         f"UNSAFE ({fresh.get('reason')})"}
            out["rejudged_after_unsafe"] = True
        self.rig.kill.clear()
        self.rig.kill.resume_pending = False
        self.rig.interlock.t_last_ok = time.monotonic()        # the key is the operator, present (G3.6)
        self.link.arm_watchdog(True)
        out["stop_flag_cleared_at_key"] = True
        if key == "c":
            _, _, mon, _ = self.motion.observe("ASK", None, None, None, True, None)
            if not mon_mod.all_clear(mon):
                fired = mon_mod.fired_list(mon)
                out["refused"] = "not all clear when read after the key: " + (
                    "; ".join(f"monitor {n} ({c})" for n, c, _ in fired) or f"monitor 7 at {mon['7'].get('n_of_5')} of 5")
        return out

    def relatch(self, phase: str, why: str, as_freeze: bool = False) -> None:
        """A key cleared the stop flag and the arm is not going to move (or has stopped moving and the next step is
        refused): latched again - flag set, goals := present - with a row. A stop that arrived since the key gets its
        own freeze row, under its own monitor."""
        self.link.arm_watchdog(False)
        if self.rig.kill.stop_requested.is_set():
            _, q6, mon, snap = self.motion.observe(phase, None, None, None, True, None)
            fired = mon_mod.fired_list(mon) or [(6, "kill_switch", f"stop flag set ({self.rig.kill.source})")]
            self.motion.freeze(phase, fired, q6, snap)
            return
        if as_freeze:
            self.motion.freeze(phase, [(0, "refused", why)], self.rig.read_q(), self.link.snapshot(),
                               source=f"refused:{phase}")
            return
        self.rig.kill.trigger(f"relatch:{phase}")
        held, attempts = self.rig.hold_now()
        self.motion.last_hold = (held, attempts)
        self.chain.append({"kind": "relatch", "t_iso": g3.now_iso(), **self.ids(), "phase": phase, "why": why, "held": held,
                           "hold_attempts": attempts, "stop_flag_set": self.rig.kill.stop_requested.is_set(),
                           "source": self.rig.kill.source})

    def judge_at(self, phase: str, quiet: bool = False) -> dict:
        """The judge reads one saved frame, the arm stationary. Every call is a row; the verdict commands nothing."""
        hands, _ = self.hands_now()
        q6 = self.rig.read_q()
        palm = hands[0]["palm_mm"] if len(hands) == 1 and mon_mod.finite_palm(hands[0].get("palm_mm")) else None
        blade = self.planner.blade_toward_palm(q6, palm) if (palm is not None and self.holding) else None
        self.n_judge += 1
        photo = self.pump.save(self.dir / "photos", f"judge{self.n_judge:02d}_{phase.lower()}")
        ctx = {"state": self.state, "trial": self.trial_id, "holding_prop": self.holding, "hands_seen_by_monitor": len(hands),
               "mode": "placement" if self.a.palm_placement else "standoff",
               "blade_toward_palm_by_code": blade,
               "monitors_fired": [c for _, c, _ in mon_mod.fired_list(self.motion.last_mon or {})]}
        if self.a.palm_placement:                                 # what the code knows of the hand (2026-10-04): the
            ctx.update(self.placement_context(q6, hands))         #   judge decides, better informed
        if self.judge_script is not None:
            self.judge_script.scene.update(n_hands=len(hands), blade_toward=blade)
        v = self.judge.ask(str(self.dir / photo["file"]) if photo["file"] else "", phase, ctx)
        unsafe = code_unsafe(v)
        v = {**(v if isinstance(v, dict) else {"raw": str(v)[:200]}), "unsafe": unsafe}
        self.latest_verdict = v
        disagree = (blade is not None and v.get("blade_toward_hand") is not None and bool(v["blade_toward_hand"]) != blade)
        self.chain.append({"kind": "judge", "t_iso": g3.now_iso(), **self.ids(), "phase": phase, "state": self.state,
                           "verdict": v, "unsafe": unsafe, "frame": photo["file"], "frame_sha256": photo["sha256"],
                           "frame_seq": photo["frame_seq"], "context": ctx, "blade_toward_palm_by_code": blade,
                           "blade_disagreement_flagged": bool(disagree)})
        if disagree:
            self.say("  [FLAG: the judge and the code disagree on which end of the knife is toward the hand]")
        if not quiet:
            self.say(ask_menu.verdict_line(v))
        return v

    def kill_or_leave(self, key: str) -> bool:
        """[l] and a confirmed [k] end the session (Stop). An unconfirmed [k] -> False: the menu is shown again."""
        if key == "l":
            self.state = "LEAVE"
            raise Stop("the operator chose LEAVE AS IS (torque on, no further commands)")
        if not ask_menu.confirm_kill(self.keys, self.chain, self.session_id, self.dry):
            return False
        self.state = "KILL"
        self.rig.sbus.disable_torque()
        g3.kill_row(self.chain, "operator", self.rig.read_q(), self.trial_id, None, None, self.session_id,
                    "[k] at the menu: torque OFF after OFF + YES")
        self.say("Torque disabled on all motors.")
        raise Stop("the operator chose KILL (torque off)", torque_off=True)

    # -- ASK ----------------------------------------------------------------------------------------------
    def freeze_flow(self, where: str, resume_ok: bool, verdict: dict | None = None) -> str:
        """The arm is frozen. -> 'continue' (the operator's [c], all monitors clear) | 'handed_over'. [k] / [l] raise."""
        self.state = "FREEZE"
        note = ""
        while True:
            fz = self.motion.last_freeze or {}
            cause = (f"monitor {fz.get('monitor')} ({fz.get('code')}): {fz.get('detail')}" if fz.get("monitor")
                     else f"{fz.get('code')}: {fz.get('detail')}")
            if verdict is None and not self.layer_off:                         # --layer-off: no judge; the menu asks
                if self.preauth_applies(fz) and not self.settle("FREEZE"):     # 1 s before a pre-authorised call
                    continue                                                   # another monitor fired: ask as usual
                verdict = self.judge_at("FREEZE", quiet=True)
            self.state = "ASK"
            if self.cues is not None:
                self.cues.send("ASK")
            q6, mon, _ = self.motion.read_monitors("ASK")
            clear = mon_mod.all_clear(mon)
            self.holding = self.gripper_closed(q6)
            if self.preauth_applies(fz):
                got, note = (self.place(where, cause, verdict) if self.a.palm_placement else
                             self.preauth(where, cause, verdict))
                if got == "done":
                    return "handed_over"
                if got is not None:                                        # as after a typed [h]
                    self.state, verdict = "ASK", (self.latest_verdict if got == "refused" else None)
                    if got != "refused":
                        resume_ok, where = False, (self.motion.last_freeze or {}).get("phase") or where
                    continue
            if self.a.palm_placement and self.latest_verdict is not None:   # the newest verdict on the menu
                verdict = self.latest_verdict
            over = bool(self.a.palm_placement and self.under_arm(q6))
            key, row = ask_menu.ask_freeze(self.keys, self.chain, self.session_id, where, cause, verdict,
                                           offer_continue=bool(clear and resume_ok),
                                           holding=self.holding and not self.a.hold_test and not self.layer_off, dry=self.dry,
                                           note=note or ("TRUST LAYER OFF: no judge, no handover; [c] is yours "
                                                         "alone (monitors 1-6 clear)" if self.layer_off else ""),
                                           held=self.motion.last_hold[0],
                                           on_key=lambda k, v=verdict: self.on_motion_key(k, v),
                                           placement=bool(self.a.palm_placement),
                                           monitors="monitors 1-6" if self.layer_off else "all 8 monitors",
                                           offer_release=bool(self.a.palm_placement and self.place_palm is not None),
                                           h_text=("continue the placement to your palm: the arm is over it, so no "
                                                   "fresh judge call; CP3 judges before the release") if over else None)
            note = ""
            if row.get("refused"):                                         # the hours, or [c] not clear at the key
                self.say(f"  [{key}] REFUSED: {row['refused']}")
                if row.get("stop_flag_cleared_at_key"):
                    self.relatch("ASK", f"[{key}] refused: {row['refused']}")
                note = f"[{key}] was refused: {row['refused']}"
            elif key == "w":
                self.go("key:w")
                self.rig.clock.sleep(WAIT_S * self.rig.clock.scale)        # real seconds: the hand feed is real time
                verdict = None
                self.state = "FREEZE"
            elif key == "c":
                if row.get("rejudged_after_unsafe"):
                    self.say("  [c]: the fresh verdict is not UNSAFE; continuing (both verdicts are in the chain).")
                self.resume_state = where if where in sm.MOVING else None
                if self.resume_state:
                    self.go("key:c", all_clear=True)
                return "continue"
            elif key in ("k", "l"):
                self.kill_or_leave(key)
            elif key == "r":                                               # palm placement: the operator's release
                out, note = self.release_typed(verdict)
                if out == "done":
                    return "handed_over"
                self.state, verdict, resume_ok = "ASK", None, False
                where = (self.motion.last_freeze or {}).get("phase") or where
            elif key in ("h", "o"):
                out, note = (self.place_typed(verdict) if self.a.palm_placement and key == "h" else
                             self.handover(key, verdict))
                if out == "done":
                    return "handed_over"
                self.state, verdict = "ASK", (verdict if out == "refused" else None)
                if out != "refused":                                       # the arm left the task's path: no [c]
                    resume_ok, where = False, (self.motion.last_freeze or {}).get("phase") or where

    # -- the pre-authorised handover (--handover-trial) ----------------------------------------------------
    def preauth_applies(self, fz: dict) -> bool:
        return bool((self.a.handover_trial or self.a.palm_placement) and not self.a.hold_test and not self.preauth_used
                    and self.holding and fz.get("monitor") == 7 and self.trial_id is not None)

    def preauth(self, where: str, cause: str, verdict: dict) -> tuple:
        """The first hand event of the trial, after the judge. -> (None, note): held, the menu decides | the handover's
        (out, note), exactly as after a typed [h]. Either way the key row says who gave it."""
        self.preauth_used = True
        if verdict.get("unsafe"):
            why = f"the judge's verdict is UNSAFE ({verdict.get('reason')})"
            self.chain.append({"kind": "preauth_held", "t_iso": g3.now_iso(), **self.ids(), "where": where, "why": why})
            self.say(f"  Pre-authorised handover HELD: {why}. The menu decides.")
            return None, f"pre-authorised handover held: {why}"
        row = {"kind": "key", "t_iso": g3.now_iso(), "session_id": self.session_id, "state": "ASK", "menu": ["h"],
               "key": "h", "accepted": True, "by": PREAUTH, "verdict_on_screen": ask_menu.on_screen(verdict),
               "where": where, "cause": cause, "held": bool(self.motion.last_hold[0])}
        row.update(self.on_motion_key("h", verdict) or {})
        self.chain.append(row)
        if row.get("refused"):
            self.say(f"  Pre-authorised [h] REFUSED: {row['refused']}")
            if row.get("stop_flag_cleared_at_key"):
                self.relatch("ASK", f"[h] refused: {row['refused']}")
            return None, f"pre-authorised [h] was refused: {row['refused']}"
        self.say(f"  Hand seen, the judge's verdict is not UNSAFE: [h] {PREAUTH}.")
        return self.handover("h", verdict)

    # -- [h] / [o] ----------------------------------------------------------------------------------------
    def plan_row(self, kind: str, view: dict, approved, blade: bool | None, verdict: dict | None, ms: float,
                 extra: dict | None = None) -> None:
        jb = (verdict or {}).get("blade_toward_hand")
        self.chain.append({"kind": "plan", "t_iso": g3.now_iso(), **self.ids(), "plan": view["row"], "requested": kind,
                           "approved_palm_mm": None if approved is None else [float(approved[0]), float(approved[1])],
                           "blade_toward_palm_by_code": blade, "blade_toward_hand_by_judge": jb,
                           "blade_disagreement_flagged": bool(blade is not None and jb is not None and bool(jb) != blade),
                           "plan_ms": round(ms, 1), **(extra or {})})

    def ask_planner(self, call, *args) -> tuple:
        """One planner call with the arm stationary -> (the raw result, its view, ms). A planner that raises or
        returns nothing is a refusal with that reason."""
        t0 = time.perf_counter()
        try:
            res = call(*args)
            view = plan_view(res)
        except Exception as e:  # noqa: BLE001
            res, why = None, f"the planner raised {type(e).__name__}: {e}"
            view = {"ok": False, "reason": why, "targets": [], "palm_mm": None, "row": {"ok": False, "reason": why}}
        return res, view, 1000 * (time.perf_counter() - t0)

    def refused(self, key: str, why: str, palm=None, row: bool = True) -> tuple:
        """Back to ASK with the reason, the arm latched again. row=False: the planner's refusal is already a row."""
        if row:
            self.chain.append({"kind": "plan", "t_iso": g3.now_iso(), **self.ids(), "plan": {"ok": False, "reason": why},
                               "requested": ("placement" if self.a.palm_placement and key == "h" else
                                             {"h": "handover", "o": "reorient"}[key]), "approved_palm_mm": palm,
                               "refused_by": "code"})
        self.say(f"  [{key}] REFUSED by code: {why}")
        self.relatch("HANDOVER_PLAN", f"[{key}] refused: {why}")
        return "refused", f"[{key}] was refused: {why}"

    def refreeze(self, phase: str, why: str) -> tuple:
        """The arm has been moved by a key and the next step is refused: frozen again, with its own freeze row."""
        self.motion.approved_palm = None
        self.relatch(phase, why, as_freeze=True)
        return "frozen", why

    def retargeter(self, plan, approved_palm, rows):
        """The approach's on_step (retarget.py): plans come from the worker, never from the control thread. The palm
        approved at the key is what drift is measured from."""
        self._rt = Retargeter(self.rt_worker, plan, rows, approved_palm, self.planner.cfg, self.chain, self.motion,
                              self.rig, self.ids(), self.next_job_id)
        self.retargeters.append(self._rt)
        return self._rt.on_step

    def handover(self, key: str, verdict: dict | None) -> tuple:
        """The key was accepted and the stop flag cleared at it. -> ('done' | 'refused' | 'frozen', a note)."""
        hands, _ = self.hands_now()
        why = sm.handover_refusal(len(hands), self.link.monitor8()[0], None, None)
        if not why and not mon_mod.finite_palm(hands[0].get("palm_mm")):
            why = "the hand's palm position is not two finite numbers"
        if why:
            return self.refused(key, why)
        palm0 = [float(v) for v in hands[0]["palm_mm"]]                    # P0: the palm approved at this key
        palm, q6 = palm0, self.rig.read_q()
        reoriented, turned = False, None
        if key == "o":
            self.go("key:o")
            plan, view, ms = self.ask_planner(self.planner.plan_reorient, q6, palm0)
            self.plan_row("reorient", view, palm0, self.blade(q6, palm0), verdict, ms)
            if not view["ok"]:
                self.state = "ASK"
                return self.refused(key, f"the planner returned no re-orientation: {view['reason']}", palm0, row=False)
            self.motion.approved_palm = palm0
            res = self.motion.run_rows("REORIENT", view["targets"], vel_limit=mon_mod.VEL_LIMIT_HANDOVER_DPS)
            self.handover_path += res["sent"]
            if res["end"] == "frozen":
                self.motion.approved_palm = None
                return "frozen", ""
            self.go("done")                                                # HANDOVER_PLAN: stationary, not latched
            self.link.arm_watchdog(True)
            hands, _ = self.hands_now()
            if len(hands) != 1 or not mon_mod.finite_palm(hands[0].get("palm_mm")):
                return self.refreeze("HANDOVER_PLAN", f"{len(hands)} hand(s) seen after the re-orientation: one is needed")
            palm = [float(v) for v in hands[0]["palm_mm"]]
            d = mon_mod.drift_mm(palm, palm0)
            if d >= mon_mod.DRIFT_MAX_MM:
                return self.refreeze("HANDOVER_PLAN", f"the palm is {d:.0f} mm from the palm approved at [o] "
                                                      f"(>= {mon_mod.DRIFT_MAX_MM:g})")
            q6, reoriented, turned = self.rig.read_q(), True, plan
        else:
            self.go("key:h")
        blade = self.blade(q6, palm)
        why = ("the blade heading could not be computed" if blade is None else
               sm.handover_refusal(1, self.link.monitor8()[0], blade, None))
        if why:
            self.state = "ASK"
            return self.refreeze("HANDOVER_PLAN", why) if reoriented else self.refused(key, why, palm0)
        if turned is not None:                                             # the arm is stationary: planning blocks nothing
            plan, view, ms = self.ask_planner(self.planner.retarget, turned, palm, q6)
        else:
            plan, view, ms = self.ask_planner(self.planner.plan_handover, q6, palm)
        self.plan_row("handover", view, palm0, blade, verdict, ms,
                      {"planned_for_palm_mm": palm, "palm_now_from_approved_mm": round(mon_mod.drift_mm(palm, palm0), 1)})
        if not view["ok"]:
            self.go("plan_none")
            why = f"the planner returned no plan: {view['reason']}"
            return self.refreeze("HANDOVER_PLAN", why) if reoriented else self.refused(key, why, palm0, row=False)
        self.go("plan_ok")
        self.motion.approved_palm = palm0                                  # drift stays measured from P0
        self._rt = None
        on_step = self.retargeter(plan, palm0, view["targets"])
        res = self.motion.run_rows("HANDOVER_APPROACH", view["targets"], vel_limit=mon_mod.VEL_LIMIT_HANDOVER_DPS,
                                   on_step=on_step)
        if self._rt is not None:
            self._rt.end()                                                 # a plan still being computed is never used
        self.handover_path += res["sent"]
        self.motion.approved_palm = None
        if res["end"] == "frozen":
            return "frozen", ""
        self.go("done")                                                    # HOLD
        if self.motion.hold_rows("HOLD", HOLD_SETTLE_S)["end"] == "frozen":
            return "frozen", ""
        return self.release_and_retreat()

    def blade(self, q6, palm) -> bool | None:
        try:
            return bool(self.planner.blade_toward_palm(q6, palm))
        except Exception:  # noqa: BLE001 - no heading, no handover
            return None

    # -- HOLD -> CP3 -> ASK_RELEASE -> RELEASE -> RETREAT -----------------------------------------------
    def release_and_retreat(self, auto_if_safe: bool = False) -> tuple:
        """auto_if_safe (palm placement): a CP3 verdict that is not UNSAFE releases without a key; UNSAFE asks."""
        while True:
            self.go("settled")                                             # CP3
            v = self.judge_at("CP3", quiet=True)
            self.release_verdict = v
            from placement_flow import recent_fist
            fist = auto_if_safe and recent_fist(list(self.read_sights) + list(self.sights))
            if fist:
                self.say("  The hand read as a FIST before the arm covered it: no release without your [r].")
            if auto_if_safe and not v["unsafe"] and not fist:
                self.released_by = "the CP3 verdict (not UNSAFE), pre-authorised"
                self.chain.append({"kind": "release_auto", "t_iso": g3.now_iso(), **self.ids(),
                                   "verdict": ask_menu.on_screen(v)})
                self.go("release_ok")                                      # RELEASE
                break
            self.go("unsafe" if v["unsafe"] else "safe")                   # ASK_RELEASE either way: the arm is holding
            key, _ = ask_menu.ask_release(self.keys, self.chain, self.session_id, v, self.dry)
            if key == "r":
                self.released_by = "[r]"
                self.go("key:r")                                           # RELEASE: the gripper, nothing else
                break
            if key == "w":
                self.state = "HOLD"
                if self.motion.hold_rows("HOLD", HOLD_SETTLE_S)["end"] == "frozen":
                    return "frozen", ""
                continue
            self.kill_or_leave(key)
            self.state = "HOLD"
        return self.open_and_retreat()

    def open_and_retreat(self, down: list | None = None) -> tuple:
        """RELEASE (the gripper only), the placement's lift-off (along `down`, the descent rows the arm is on; None = all
        of them; [] = none), the wait for 5 clear frames, the retreat."""
        q6 = self.rig.read_q()
        goal = [float(self.rig.sbus.last_goal.get(j, q6[i])) for i, j in enumerate(g3.ALL)]
        opened = goal[:5] + [self.seg["release_pct"]]
        res = self.motion.run_rows("RELEASE", interpolate(goal, opened, HANDOVER_DPS, GRIP_PPS) + [opened] * 5,
                                   vel_limit=mon_mod.VEL_LIMIT_HANDOVER_DPS)
        if res["end"] == "frozen":
            return "frozen", ""
        self.holding = False
        if self.a.palm_placement and self.place_palm is not None and down != [] and self.liftoff(down) == "frozen":
            return "frozen", ""
        self.say(f"  Released. Take the prop and your hand OUT of the workspace; the arm retreats after "
                 f"{mon_mod.CLEAR_FRAMES} fresh frames with no hand.")
        self.state = "HOLD"
        watch, t0, waits = mon_mod.ClearWatch(), self.rig.clock.now(), 0
        if self.cues is not None:
            self.cues.send("WAIT_CLEAR")
        while True:
            got = self.motion.hold_rows("HOLD", HAND_GONE_S,
                                        until=lambda m, snap: watch.see(snap.get("msg")) and not m["8"]["fired"])
            if got["end"] == "frozen":
                return "frozen", ""
            if got["end"] == "until":
                break
            waits += 1
            self.say("  A hand is still in the workspace: the arm holds. (ESC / Ctrl-C = the menu.)")
            if self.dry and waits >= 2:
                return self.refreeze("HOLD", "dry run: the simulated hand never left the workspace")
        self.chain.append({"kind": "wait_clear", "t_iso": g3.now_iso(), **self.ids(), "frames": watch.need,
                           "frames_seen": watch.frames, "resets": watch.resets,
                           "waited_s": round((self.rig.clock.now() - t0) / self.rig.clock.scale, 2)})
        self.state = "RELEASE"
        self.go("done")                                                    # RETREAT
        return self.retreat_after_handover()

    def retreat_after_handover(self) -> tuple:
        """Back up the way the arm came down (the rows it was sent, reversed, to the transit height), then to rest at
        the scripted speed - FK-checked before it runs, and behind the gate (monitors 7 and 8 clear at that moment)
        every time it starts or starts again. A freeze on the way is asked like any other."""
        q6 = self.rig.read_q()
        grip = float(self.rig.sbus.last_goal.get("gripper", q6[5]))
        z_transit = self.guard.table_z + float(self.planner.describe().get("z_transit_mm", 65.0)) - 2.0
        up = []
        for r in reversed(self.handover_path):
            up.append(list(r[:5]) + [grip])
            if float(self.guard.ee_mm(r)[2]) >= z_transit:
                break
        rest6 = self.seg["rest"][:5] + [self.seg["open_pct"]]
        if self.a.palm_placement and self.place_palm is not None:          # 3 deg/s near where the palm was, 10 beyond
            rows = up + self.placement_retreat_rows(up[-1] if up else q6, rest6)
        else:
            rows = up + interpolate(up[-1] if up else q6, rest6, self.scripted_dps, GRIP_PPS)
        rows += [rows[-1]] * 5
        while True:
            self.state = "RETREAT"
            probs = segment_problems(self.guard, rows, visible=self.visible)
            if probs:
                self.refreeze("RETREAT", "the retreat path is refused by its FK check: " + "; ".join(probs[:2]))
                self.freeze_flow("RETREAT", resume_ok=False)               # [w] / [k] / [l]: recovery is by hand
                continue
            if self.gate("RETREAT"):
                limits = mon_mod.row_vel_limits(rows, self.rig.read_q()) if self.a.palm_placement else None
                res = self.motion.run_rows("RETREAT", rows, vel_limit=(self.scripted_vel if limits is None else
                                                                       (lambda k, lim=limits: lim[min(k, len(lim) - 1)])))
                if res["end"] == "done":
                    self.go("done")                                        # IDLE
                    return "done", ""
                rows = res["rows"][res["k"]:] or [rows[-1]]
            self.freeze_flow("RETREAT", resume_ok=True)                    # 'continue' or Stop ([h] is not offered)
            rows = interpolate(self.rig.read_q(), rows[0], HANDOVER_DPS, GRIP_PPS) + rows
