"""Palm placement (the operator, 2026-10-03): --palm-placement. Pre-authorised at trial start; no key during the trial
unless the planner finds no plan, a code rule refuses, a monitor fires, or the judge reads UNSAFE after its settle.

  a hand event (monitor 7, the prop held) -> 1 s settle -> the judge (rubric_placement.md) -> UNSAFE: the menu
  -> the plan (planner/placement.py: the handle over the open palm, the pointed end past the fingertips, turned
     automatically when it faces the hand; the pointed end >= 30 mm from the palm in 3D on every row) -> none: the menu
  -> PLACE_APPROACH (<= 10 deg/s beyond 150 mm of the palm, <= 3 within) -> PLACE_DESCENT (<= 3 deg/s) to 20 mm above
     the palm (+16 mm commanded for the sag) -> HOLD, 1 s settle -> CP3 (one hand, open, still, palm under the handle,
     the pointed end clear) -> not UNSAFE: RELEASE, no key | UNSAFE: the release menu
  -> 5 fresh frames with no hand -> RETREAT (3 deg/s near the palm, 10 beyond) -> "did you receive it?" (y/n), logged
     as the outcome label beside the CP3 verdict.
In PLACE_APPROACH / PLACE_DESCENT monitor 7 takes only the planned hand: a second hand or the hand lost freezes the
arm; the palm 50 mm from where the plan was made freezes it in the approach, 15 mm in the descent
(monitors.DRIFT_MAX_BY_PHASE). Above the release point, before the descent, the hand is read again: moved >= 5 mm or
turned >= 20 deg -> a fresh plan from there (at most MAX_REPLANS). Every step also checks the row about to be sent: the
pointed end >= 30 mm from the palm as it is now (3D), else monitor 7 freezes the arm. In this mode the menu's [h]
retries the placement (a settle and a fresh judge first); there is no [o] - the placement turns the screwdriver itself.
"""
from __future__ import annotations

import numpy as np

from common import paths

paths.use_anchor()
import g3  # noqa: E402

import ask_menu  # noqa: E402
import monitors as mon_mod  # noqa: E402
from grasp_segment import GRIP_PPS, interpolate  # noqa: E402

SETTLE_S = 1.0
REPLAN_MM = 5.0              # the palm moved this far by the time the arm is above it: plan again from there
REPLAN_DEG = 20.0            # or the fingers turned this much
MAX_REPLANS = 3
STILL_S = 1.0                # above the palm: 1 s more of the hand there (it may be under the arm by then)
STILL_LOOKBACK_S = 20.0      # the sightings that count: the last ones within this, before the arm covered the hand
STILL_MIN_FRAMES = 5         # at least this many of them (the last 10 at most)
STILL_SPREAD_MM = 10.0       # and every one within this of their median
HAND_READ_S = 0.6            # the hand for a plan: the median of the frames over this, not one frame (04:21: one frame
HAND_READ_MIN = 3            #   read "no hand" five times while the judge described the hand); at least this many
AUTO_TRIES = 4               # the pre-authorised placement: the first try and up to 3 more, RETRY_WAIT_S apart, each
RETRY_WAIT_S = 2.0           #   with a settle and a fresh judge call - then the menu
# after the release: straight up first (the descent's own rows, reversed: >= 25 mm, to the turn height), so the camera
# sees whether the hand is still there before the arm goes home
COVER_MM = 45.0              # the fingertip or the screwdriver this near the palm (table plane): it covers the hand -
#                              00:59 the palm read steady (1-7 mm) until 29 mm, then jumped 12-44 mm, then was lost
PREAUTH = "pre-authorised at trial start (--palm-placement)"
PROP = "plastic prop"


def _placement():
    from planner import placement
    return placement


class PlacementFlow:
    """Mixed into HandoverSession with Flow."""

    def settle(self, phase: str) -> bool:
        """SETTLE_S of stillness before a pre-authorised judge call. At a freeze the arm is latched (the stop flag set,
        goals = present): it waits as [w] does, nothing read or sent. At the hold the monitors are read throughout
        (1-6 and 8 freeze it). -> False: frozen."""
        how = "latched, waiting as [w] does" if phase == "FREEZE" else "holding, monitors 1-6 and 8 read"
        self.chain.append({"kind": "settle", "t_iso": g3.now_iso(), **self.ids(), "phase": phase, "s": SETTLE_S, "how": how})
        if phase == "FREEZE":
            self.rig.clock.sleep(SETTLE_S * self.rig.clock.scale)        # real seconds: the hand feed is real time
            return True
        return self.motion.hold_rows(phase, SETTLE_S)["end"] != "frozen"

    def place(self, where: str, cause: str, verdict: dict) -> tuple:
        """The first hand event of the trial, after the settle and the judge: up to AUTO_TRIES attempts (a refusal or
        an UNSAFE verdict waits RETRY_WAIT_S, settles and asks the judge again; a freeze in motion is never retried).
        -> (None, note): held, the menu decides | ('done' | 'refused' | 'frozen', note) as after a typed [h]."""
        self.preauth_used = True
        got, note = None, ""
        for attempt in range(AUTO_TRIES):
            if attempt:
                self.say(f"  Placement: trying again ({attempt}/{AUTO_TRIES - 1}) in {RETRY_WAIT_S:g} s - {_hint(note)}")
                self.chain.append({"kind": "auto_retry", "t_iso": g3.now_iso(), **self.ids(), "attempt": attempt,
                                   "after": note, "hint": _hint(note)})
                self.rig.clock.sleep(RETRY_WAIT_S * self.rig.clock.scale)
                self.settle("FREEZE")
                verdict = self.judge_at("FREEZE", quiet=True)
            got, note = self._place_once(where, cause, verdict, attempt)
            if got in ("done", "frozen"):
                return got, note
        return got, note

    def _place_once(self, where: str, cause: str, verdict: dict, attempt: int) -> tuple:
        if verdict.get("unsafe"):
            why = f"the judge's verdict is UNSAFE after the settle ({verdict.get('reason')})"
            self.chain.append({"kind": "preauth_held", "t_iso": g3.now_iso(), **self.ids(), "where": where, "why": why})
            self.say(f"  Placement HELD: {why}. The menu decides.")
            return None, f"placement held: {why}"
        row = {"kind": "key", "t_iso": g3.now_iso(), "session_id": self.session_id, "state": "ASK", "menu": ["h"],
               "key": "h", "accepted": True, "by": PREAUTH, "verdict_on_screen": ask_menu.on_screen(verdict),
               "where": where, "cause": cause, "held": bool(self.motion.last_hold[0]), "attempt": attempt}
        row.update(self.on_motion_key("h", verdict) or {})
        self.chain.append(row)
        if row.get("refused"):
            if row.get("stop_flag_cleared_at_key"):
                self.relatch("ASK", f"[h] refused: {row['refused']}")
            return None, f"the placement was refused: {row['refused']}"
        self.say(f"  Hand seen, the judge's verdict is not UNSAFE: placing ({PREAUTH}).")
        return self.place_go(verdict)

    def place_typed(self, verdict: dict) -> tuple:
        """[h] at the menu in placement mode (its row is the menu's; the stop flag was cleared at the key): a settle and
        a fresh judge call, then the same placement. UNSAFE -> back to the menu."""
        if self.under_arm(self.rig.read_q()):
            # the arm is over the hand: the camera and the judge cannot see it; the operator's [h] says it is there.
            self.chain.append({"kind": "resume_under_arm", "t_iso": g3.now_iso(), **self.ids(), "palm_mm": self.place_palm,
                               "why": "[h] with the arm over the last-seen palm: the operator confirms the hand; the "
                                      "placement goes on to it; CP3 is judged before the release"})
            self.say("  [h]: the arm is over your hand - continuing the placement to where your palm was last seen.")
            return self.place_go(None, given=(self.place_palm, self.place_dir, self.place_reach))
        self.settle("FREEZE")
        v = self.judge_at("FREEZE", quiet=True)
        if v["unsafe"]:
            self.relatch("ASK", f"[h] placement held: the fresh verdict is UNSAFE ({v.get('reason')})")
            return "refused", f"[h] placement held: the fresh verdict after the settle is UNSAFE ({v.get('reason')})"
        return self.place_go(v)

    def under_arm(self, q6) -> bool:
        """[h] goes on to the last-seen palm only when the arm covers it AND the monitor sees no hand now - the hand
        hidden under the arm. A hand in view (moved: the 60 mm drift freeze) is read again, judged and planned for."""
        if self.place_palm is None or not self.covers(q6, self.place_palm):
            return False
        hands, _ = self.hands_now()
        return not hands

    def covers(self, q6, palm) -> bool:
        """The fingertip or the screwdriver within COVER_MM of the palm on the table plane: it hides the hand."""
        from planner.knife import knife_pose
        from planner.path import seg_dist
        cfg = self.planner.cfg
        kp = knife_pose(cfg.to_mjcf(q6), cfg.knife)
        near = min(seg_dist(palm, kp.blade_tip, kp.handle_tip), float(np.hypot(kp.tip[0] - palm[0], kp.tip[1] - palm[1])))
        return near <= COVER_MM

    def _hand_now(self) -> tuple:
        """The hand for a plan, read over HAND_READ_S with monitors 1-6 and 8 live (the stop flag was cleared at the key):
        the median palm of the fresh frames with one hand. -> (palm, (dx, dy), reach, None) | (None, None, None, why)."""
        seen, many = [], [0]

        def look(mon, snap):
            msg = snap.get("msg") or {}
            hands = list(msg.get("hands") or [])
            many[0] += len(hands) >= 2
            if len(hands) == 1 and mon_mod.finite_palm(hands[0].get("palm_mm")) and \
                    (not seen or seen[-1][0] != msg.get("seq")):
                seen.append((msg.get("seq"), hands[0]))
            return False
        if self.motion.hold_rows("HOLD", HAND_READ_S, until=look)["end"] == "frozen":
            return None, None, None, "a monitor fired while the hand was read"
        why = ("monitor 8 (hand link) is firing: no trusted palm position" if self.link.monitor8()[0] else
               "two hands are seen: one hand only" if many[0] >= 2 else
               f"the hand was seen in {len(seen)} frames in {HAND_READ_S:g} s (< {HAND_READ_MIN}): no hand is seen "
               "steadily" if len(seen) < HAND_READ_MIN else None)
        fists = sum(1 for _, h in seen if mon_mod.hand_open(h) is False)
        if not why and fists * 2 > len(seen):
            why = f"a fist ({fists} of {len(seen)} frames): open your hand, palm up"
        if why:
            return None, None, None, why
        pts = np.array([[float(v) for v in h["palm_mm"]] for _, h in seen])
        med = np.median(pts, axis=0)
        axis = next((a for a in (mon_mod.hand_axis(self.tm, h) for _, h in reversed(seen)) if a is not None), None)
        if axis is None:
            return None, None, None, "the hand's direction (wrist -> fingers) cannot be read from its landmarks"
        now = self.rig.clock.now()
        self.read_sights = [(now, seq, [float(v) for v in h["palm_mm"]], axis, mon_mod.hand_open(h))
                            for seq, h in seen]                                                   # the reference
        return [float(med[0]), float(med[1])], axis[0], axis[1], None

    def place_go(self, verdict: dict | None, given: tuple | None = None) -> tuple:
        """Plan, approach, re-read the hand above it (re-plan if it moved), descend, settle, release. given: (palm,
        direction, reach) the operator confirmed under the arm - no still check above it."""
        palm, d, reach, why = (given + (None,)) if given else self._hand_now()
        if why:
            return self.refused("h", why)
        self.go("key:h")                                                   # HANDOVER_PLAN
        for n in range(MAX_REPLANS + 1):
            plan, view, ms = self.ask_planner(self.planner.plan_placement, self.rig.read_q(), palm, d, reach,
                                              float(self.seg["close_pct"]) if self.holding else None)   # re-squeeze
            self.plan_row("placement", view, palm, None, verdict, ms,
                          {"planned_for_palm_mm": palm, "fingers_dir": [round(d[0], 3), round(d[1], 3)],
                           "finger_reach_mm": reach, "replan": n})
            if not view["ok"]:
                why = f"the planner returned no placement: {view['reason']}"
                if n:
                    return self.refreeze("HANDOVER_PLAN", why)
                self.go("plan_none")
                return self.refused("h", why, palm, row=False)
            cut = next((p["from_step"] - 1 for p in plan.phases if str(p["phase"]).startswith("descend")),
                       len(view["targets"]))
            rows = [list(r) for r in view["targets"]]
            self.place_palm, self.place_dir, self.place_reach = palm, d, reach
            if n == 0:
                self.sights = []                                           # kept across re-plans (04:08: wiped)
            self.go("place_ok")                                            # PLACE_APPROACH
            res = self.run_place("PLACE_APPROACH", rows[:cut], palm)
            if res["end"] == "frozen":
                return "frozen", ""
            if given:                                                      # confirmed by the operator's [h]
                self.chain.append({"kind": "still_check", "t_iso": g3.now_iso(), **self.ids(), "ok": True,
                                   "why": "skipped: the operator's [h] with the arm over the hand", "palm_mm": palm})
                break
            now, d_now, reach_now, why = self.look_still()                 # above the palm: seen still? moved?
            self.chain.append({"kind": "still_check", "t_iso": g3.now_iso(), **self.ids(), "ok": why is None,
                               "why": why, "palm_mm": now, "n_sightings": len(self.sights)})
            if why:
                return self.refreeze("PLACE_APPROACH", f"above the palm, before the descent: {why}")
            moved = float(np.hypot(now[0] - palm[0], now[1] - palm[1]))
            turned = float(np.degrees(np.arccos(np.clip(d[0] * d_now[0] + d[1] * d_now[1], -1.0, 1.0))))
            if moved < REPLAN_MM and turned < REPLAN_DEG:
                break
            if n == MAX_REPLANS:
                return self.refreeze("PLACE_APPROACH", f"the hand kept moving: {moved:.0f} mm / {turned:.0f} deg after "
                                                       f"{MAX_REPLANS} re-plans")
            self.chain.append({"kind": "replan", "t_iso": g3.now_iso(), **self.ids(), "moved_mm": round(moved, 1),
                               "turned_deg": round(turned, 1), "from_palm_mm": palm, "to_palm_mm": now, "n": n + 1})
            self.say(f"  The hand moved {moved:.0f} mm / turned {turned:.0f} deg: planning again from above it.")
            palm, d, reach = now, d_now, reach_now
            self.go("replan")                                              # HANDOVER_PLAN
        self.go("done")                                                    # PLACE_DESCENT
        self.place_descent_rows = [list(r) for r in rows[cut:]]
        res = self.run_place("PLACE_DESCENT", rows[cut:], palm)
        if res["end"] == "frozen":
            return "frozen", ""
        self.go("done")                                                    # HOLD
        if not self.settle("HOLD"):
            return "frozen", ""
        return self.release_and_retreat(auto_if_safe=True)

    def run_place(self, phase: str, rows: list, palm) -> dict:
        if not rows:
            return {"end": "done", "sent": []}
        q_now = self.rig.read_q()
        limits = mon_mod.row_vel_limits(rows, q_now)
        self.motion.approved_palm = palm
        self.motion.covering = self.covers(q_now, palm)                   # for the very first step too (00:59)
        try:
            res = self.motion.run_rows(phase, rows, vel_limit=lambda k: limits[min(k, len(limits) - 1)],
                                       on_step=self.point_guard(rows))
        finally:
            self.motion.approved_palm = None
            self.motion.covering = self.motion.drift_unreliable = False
        self.handover_path += res["sent"]
        return res

    def point_guard(self, rows: list):
        """on_step: the row about to be sent, its pointed end against the palm as it is now (3D)."""
        pl, cfg = _placement(), self.planner.cfg
        from planner.knife import knife_pose

        def on_step(k, q6, snap):
            self.motion.covering = self.covers(q6, self.place_palm)
            hands = list((snap.get("msg") or {}).get("hands") or [])
            one = len(hands) == 1 and mon_mod.finite_palm(hands[0].get("palm_mm"))
            self.motion.drift_unreliable = bool(one and self.covers(q6, hands[0]["palm_mm"]))
            if k >= len(rows) or not one:
                return None                                                # monitor 7 rules on these
            if not self.motion.covering:                                   # a partly hidden hand reads off by tens of mm
                self._sight((snap.get("msg") or {}).get("seq"), hands[0])
            kp = knife_pose(cfg.to_mjcf(rows[k]), cfg.knife)
            palm_now = self.place_palm if self.motion.drift_unreliable else hands[0]["palm_mm"]
            d = float(np.linalg.norm(np.asarray(kp.blade_tip, float) - pl.palm3(cfg, palm_now)))
            if d < pl.POINT_CLEAR_MM:
                self.motion.force_fire = (7, f"the next row would bring the pointed end {d:.0f} mm from the palm as it "
                                             f"is now (< {pl.POINT_CLEAR_MM:g}, 3D)")
            return None
        return on_step

    def _sight(self, seq, hand) -> None:
        """One frame's sighting of the hand (fresh frames only), kept for the still check above the palm."""
        sights = getattr(self, "sights", None)
        if sights is None or (sights and sights[-1][1] == seq):
            return
        axis = mon_mod.hand_axis(self.tm, hand)
        sights.append((self.rig.clock.now(), seq, [float(v) for v in hand["palm_mm"]], axis, mon_mod.hand_open(hand)))

    def look_still(self) -> tuple:
        """Above the palm: STILL_S more of the hand (monitors 1-6 and 8 read), then the sightings of the last
        2 * STILL_S. -> (palm median, (dx, dy), reach, None) | (None, None, None, why)."""
        covered = self.covers(self.rig.read_q(), self.place_palm)

        def look(mon, snap):
            hands = list((snap.get("msg") or {}).get("hands") or [])
            if not covered and len(hands) == 1 and mon_mod.finite_palm(hands[0].get("palm_mm")):
                self._sight((snap.get("msg") or {}).get("seq"), hands[0])
            return False
        if self.motion.hold_rows("HOLD", STILL_S, until=look)["end"] == "frozen":
            return None, None, None, "a monitor fired while the hand was read above the palm"
        recent = [s for s in self.sights if s[0] >= self.rig.clock.now() - STILL_LOOKBACK_S]
        if len(recent) < STILL_MIN_FRAMES:                                 # the arm covered the hand early: the hand as
            recent = list(getattr(self, "read_sights", [])) + recent       #   read for the plan, still, counts
        recent = recent[-10:]
        if len(recent) < STILL_MIN_FRAMES:
            return None, None, None, (f"the hand was seen in {len(recent)} frames in the last {STILL_LOOKBACK_S:g} s "
                                      f"(< {STILL_MIN_FRAMES}): it must be seen still before the descent")
        pts = np.array([s[2] for s in recent])
        med = np.median(pts, axis=0)
        spread = float(np.max(np.linalg.norm(pts - med, axis=1)))
        if spread > STILL_SPREAD_MM:
            return None, None, None, f"the hand was moving ({spread:.0f} mm over its last {len(recent)} sightings)"
        axis = next((s[3] for s in reversed(recent) if s[3] is not None), None)
        if axis is None:
            return None, None, None, "the hand's direction cannot be read from its landmarks above the palm"
        return [float(med[0]), float(med[1])], axis[0], axis[1], None

    def release_typed(self, verdict: dict | None) -> tuple:
        """[r] at the freeze menu, palm placement (the operator, 2026-10-04): the gripper opens where the arm stands - the
        operator's own key, the verdict on screen beside it, as at ASK_RELEASE - then the lift-off if the arm is on the
        descent (from the row it is on, never down first), the wait for 5 clear frames and the retreat."""
        self.released_by = "[r] at the freeze menu"
        self.release_verdict = verdict
        self.go("key:r")                                                   # ASK -> RELEASE
        k = self.descent_index(self.rig.read_q())
        self.chain.append({"kind": "release_typed", "t_iso": g3.now_iso(), **self.ids(), "where": self.state,
                           "on_descent_row": k, "palm_mm": self.place_palm, "verdict": ask_menu.on_screen(verdict)})
        self.say("  [r]: the gripper opens here" + (", then the arm lifts back up the descent" if k is not None else "")
                 + "; it retreats once your hand is out.")
        return self.open_and_retreat(down=[] if k is None else self.place_descent_rows[:k + 1])

    def descent_index(self, q6, tol_deg: float = 2.0) -> int | None:
        """The descent row the arm stands on (every arm joint within tol_deg), or None: not on the descent."""
        rows = getattr(self, "place_descent_rows", None) or []
        best = min(((max(abs(float(r[i]) - float(q6[i])) for i in range(5)), k) for k, r in enumerate(rows)), default=None)
        return best[1] if best is not None and best[0] <= tol_deg else None

    def liftoff(self, down: list | None = None) -> str:
        """After the release: straight up the descent's own rows, reversed (>= 25 mm, to the turn height; <= 3 deg/s;
        the hand may be under it, taking the prop - a second hand or any of 1-6, 8 freezes), so the camera sees whether
        the hand is still there before the arm goes home. -> 'done' | 'frozen'."""
        q6 = self.rig.read_q()
        down = down if down is not None else (getattr(self, "place_descent_rows", None) or [])
        if not down:
            self.refreeze("PLACE_LIFTOFF", "no descent path to lift back up along")
            return "frozen"
        up = [list(r[:5]) + [float(q6[5])] for r in reversed(down)]
        rows = interpolate(q6, up[0], 3.0, GRIP_PPS) + up + [up[-1]] * 5
        self.state = "PLACE_LIFTOFF"
        self.motion.approved_palm = self.place_palm
        try:
            res = self.motion.run_rows("PLACE_LIFTOFF", rows, vel_limit=mon_mod.VEL_LIMIT_HANDOVER_DPS)
        finally:
            self.motion.approved_palm = None
        self.handover_path = []                                            # the retreat starts from up here
        return "frozen" if res["end"] == "frozen" else "done"

    def placement_context(self, q6, hands: list) -> dict:
        """For the judge, in placement mode: the code's reading of the hand (open or a fist, from its landmarks), how near
        the arm is to it - across the table AND above it (the operator, 2026-10-04: a 2 mm horizontal distance was read
        as touching; the camera sees no depth, the encoders do) - and what the hand was before the arm covered it.
        Context only: the judge still decides."""
        now_open = mon_mod.hand_open(hands[0]) if len(hands) == 1 else None
        seen = list(self.read_sights) + list(self.sights)
        known = [s for s in seen if len(s) > 4 and s[4] is not None]
        palm = (hands[0].get("palm_mm") if len(hands) == 1 and mon_mod.finite_palm(hands[0].get("palm_mm")) else
                seen[-1][2] if seen else self.place_palm)   # where the hand IS: now, else last seen, else planned
        near = above = point3d = None
        if palm is not None and mon_mod.finite_palm(palm):
            from planner.knife import knife_pose
            from planner.path import seg_dist
            pl, cfg = _placement(), self.planner.cfg
            kp = knife_pose(cfg.to_mjcf(q6), cfg.knife)
            near = round(min(seg_dist(palm, kp.blade_tip, kp.handle_tip),
                             float(np.hypot(kp.tip[0] - palm[0], kp.tip[1] - palm[1]))), 0)
            above = round(pl.lowest_mm(cfg, kp) - pl.PALM_H_MM, 0)
            point3d = round(float(np.linalg.norm(np.asarray(kp.blade_tip) - pl.palm3(cfg, palm))), 0)
        return {"hand_open_by_code": now_open,
                "hand_open_before_the_arm_covered_it": (known[-1][4] if known else None),
                "horizontal_arm_to_palm_mm": near,
                "height_above_palm_mm": above,
                "pointed_end_to_palm_3d_mm": point3d,
                "arm_over_the_hand": bool(near is not None and near <= COVER_MM)}

    def placement_retreat_rows(self, start, rest6) -> list:
        """From above the palm to rest: <= 3 deg/s until every part is NEAR_MM from where the palm was, then 10."""
        pl, cfg = _placement(), self.planner.cfg
        from planner.knife import knife_pose
        slow = interpolate(list(start), list(rest6), pl.NEAR_DPS, GRIP_PPS)
        near = [i for i in range(0, len(slow), 3) if pl._near(cfg, knife_pose(cfg.to_mjcf(slow[i]), cfg.knife), self.place_palm)]
        i = (near[-1] + 3) if near else 0                                  # one joint-space line: past it, all far
        if i >= len(slow):
            return slow
        return slow[:i] + interpolate(slow[i - 1] if i else list(start), list(rest6), pl.FAR_DPS, GRIP_PPS)

    def outcome_label(self) -> dict:
        """The end-of-trial question; the answer is logged beside the judge's verdict at the release."""
        while True:
            ans = str(self.keys.console.ask("  Did you receive it? [y/n]: ", "y" if self.dry else "")).strip().lower()
            if ans in ("y", "n"):
                break
            self.say("  Type y or n, then Enter.")
        return self.chain.append({"kind": "outcome_label", "t_iso": g3.now_iso(), **self.ids(), "received": ans == "y",
                                  "asked": "did you receive it?", "judge_at_release": ask_menu.on_screen(
                                      getattr(self, "release_verdict", None)), "released_by": getattr(
                                      self, "released_by", None)})


def _hint(note: str) -> str:
    """What the person can do about the last refusal - printed before an automatic retry."""
    n = str(note or "")
    if "no placement" in n or "no pose" in n or "reach" in n:
        return "move your palm nearer the robot (the near half of the zone)"
    if "two hands" in n:
        return "one hand only"
    if "no hand" in n or "seen in" in n or "direction" in n:
        return "hold one open hand still, palm up, in view"
    if "UNSAFE" in n:
        return "the judge: " + n.split("(", 1)[-1].rstrip(")")[:120]
    return n[:120]


def recent_fist(sights: list) -> bool:
    """The last five sightings with landmarks: a majority read as a fist."""
    known = [s[4] for s in sights if len(s) > 4 and s[4] is not None][-5:]
    return bool(known) and sum(1 for o in known if o is False) * 2 > len(known)
