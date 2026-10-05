"""The silent re-target during the approach (PLAN.md section 2), as the control loop's on_step.

Reviewer 2026-10-01 (SR08): the planner never runs on the control thread's clock. The palm moved more than 5 mm from
the palm the current plan was made for, and is still within 50 mm of the palm approved at the key (monitor 7 freezes
beyond that): a job goes to the worker (retarget_worker.py, its own process) asking for a plan that starts at the row
the CURRENT plan will have reached LOOKAHEAD_ROWS steps from now - the swap row. The current plan keeps running. At
the swap row:
  - the finished plan is there: checked, then swapped in; it continues exactly from the pose the arm was sent to;
  - it is not there: the current plan goes on, a row says "retarget pending"; the late result is never used, and a
    new job is asked for while the palm is still off.
Checked at the swap, both numbers in the row: the palm in the latest message within 5 mm of the palm the plan was made
for (else discarded, asked again); the new plan's first row within 1.0 deg of the goal last sent (else the arm
freezes). A refused plan, a planner error or a dead worker freezes the arm. Every result and every decision is a row.

Because the arm keeps moving while a re-target is computed, one cheap check runs on every step of the approach (FK of
the row about to be sent, no planning): the knife may not come within STANDOFF_GUARD_MM of the palm as it is NOW
(the 60 mm standoff less the 5 mm the re-target tolerates). If it would, monitor 7 freezes the arm.
"""
from __future__ import annotations

import math
import time

from common import paths

paths.use_anchor()
import g3  # noqa: E402

import monitors as mon_mod  # noqa: E402

RETARGET_MIN_MM = 5.0            # a palm that moved less than this from the planned palm is not re-planned
RETARGET_MAX_S = 3.0             # kept for the flow's public numbers; a job's only deadline is its swap row now
SWAP_MAX_DEG = 1.0               # the new plan's first row from the goal last sent, any arm joint
LOOKAHEAD_ROWS = 8               # the swap row: 8 steps (0.53 s at 15 Hz) after the job is asked for; the planner
#                                  took 50-215 ms (C's fixtures), the worker's round trip is measured in every row


def plan_view(res) -> dict:
    """Whatever the planner returned, as {ok, reason, targets, palm_mm, row}. Nothing here can raise."""
    if res is None:
        return {"ok": False, "reason": "the planner returned nothing", "targets": [], "palm_mm": None,
                "row": {"ok": False, "reason": "the planner returned nothing"}}
    try:
        ok = bool(getattr(res, "ok", False))
        reason = str(getattr(res, "reason", "") or "")
        targets = [[float(v) for v in r] for r in (getattr(res, "targets", None) or [])]
        palm = getattr(res, "palm_mm", None)
        palm = [float(palm[0]), float(palm[1])] if palm is not None and mon_mod.finite_palm(list(palm)[:2]) else None
        row = res.to_row() if hasattr(res, "to_row") else {"ok": ok, "reason": reason}
        if ok and (not targets or any(len(r) != 6 for r in targets)):
            ok, reason = False, "the planner returned a plan without usable rows"
        return {"ok": ok, "reason": reason or ("" if ok else "the planner gave no reason"), "targets": targets,
                "palm_mm": palm, "row": row if ok else {**row, "ok": False, "reason": reason or "no reason given"}}
    except Exception as e:  # noqa: BLE001 - a result that cannot be read is a refusal
        why = f"the planner's result could not be read ({type(e).__name__}: {e})"
        return {"ok": False, "reason": why, "targets": [], "palm_mm": None, "row": {"ok": False, "reason": why}}


class Retargeter:
    def __init__(self, worker, plan, rows: list, approved_palm, cfg, chain, motion, rig, ids: dict, next_id):
        self.worker, self.plan, self.rows = worker, plan, [list(r) for r in rows]
        self.approved = [float(v) for v in approved_palm]
        self.plan_palm = plan_view(plan)["palm_mm"] or self.approved
        self.cfg, self.chain, self.motion, self.rig, self.ids, self.next_id = cfg, chain, motion, rig, ids, next_id
        self.job: dict | None = None             # {id, swap_k, palm, start, t0, ready: (view, raw) | None, pending}
        self.received: list = []                 # every result this approach got (the tests read it)
        self.guard_ms: list = []                 # the per-step knife check's cost
        self.ended = False

    # -- rows -----------------------------------------------------------------------------------------
    def _row(self, status: str, **extra) -> dict:
        return self.chain.append({"kind": "retarget", "t_iso": g3.now_iso(), **self.ids, "status": status,
                                  "approved_palm_mm": self.approved, "worker": self.worker.kind,
                                  "swapped": status == "swapped", **extra})

    def _job_fields(self) -> dict:
        j = self.job or {}
        return {"job_id": j.get("id"), "swap_k": j.get("swap_k"), "asked_for_palm_mm": j.get("palm"),
                "since_asked_ms": None if not j else round(1000 * (time.perf_counter() - j["t0"]), 1)}

    # -- the knife and the palm as it is now --------------------------------------------------------
    def _knife_too_close(self, row, palm) -> str | None:
        from planner.knife import knife_pose
        from planner.path import seg_dist
        t0 = time.perf_counter()
        kp = knife_pose(self.cfg.to_mjcf(row), self.cfg.knife)
        d = float(seg_dist(palm, kp.blade_tip, kp.handle_tip))
        self.guard_ms.append(1000 * (time.perf_counter() - t0))
        limit = float(self.cfg.standoff_mm) - RETARGET_MIN_MM
        return None if d >= limit else (f"the next row would bring the knife {d:.0f} mm from the palm as it is now "
                                        f"(< {limit:g}: the standoff less the re-target's {RETARGET_MIN_MM:g} mm)")

    # -- the step -----------------------------------------------------------------------------------
    def end(self) -> None:
        """The approach ended (done or frozen): a job still out is dropped - its result is never used."""
        if self.job is not None and not self.ended:
            self._row("abandoned", reason="the approach ended while this plan was out", **self._job_fields())
        self.job, self.ended = None, True

    abandon = end

    def _ask(self, k: int, palm) -> None:
        swap_k = min(k + LOOKAHEAD_ROWS, len(self.rows))
        start = self.rows[swap_k - 1] if swap_k >= 1 else self.rows[0]
        jid = self.next_id()
        self.job = {"id": jid, "swap_k": swap_k, "palm": list(palm), "start": list(start), "t0": time.perf_counter(),
                    "ready": None, "pending": False}
        self.worker.submit(jid, self.plan, palm, start)
        self._row("asked", **self._job_fields(), drift_from_plan_mm=round(mon_mod.drift_mm(palm, self.plan_palm), 1))

    def _take_results(self, k: int) -> None:
        for jid, res, err, ms in self.worker.poll():
            if jid == "dead":
                self.motion.force_fire = (7, f"the re-target worker failed: {err}")
                self._row("refused", reason=err, **self._job_fields())
                self.job = None
                continue
            if self.job is None or jid != self.job["id"]:
                continue                                   # a job already given up on (its row is written)
            self.received.append((jid, res))
            view = plan_view(res) if err is None else {**plan_view(None), "reason": err, "row": {"ok": False, "reason": err}}
            if not view["ok"]:
                self._row("refused", plan=view["row"], plan_ms=round(ms, 1), **self._job_fields())
                self.motion.force_fire = (7, f"the re-target was refused: {view['reason']}")
                self.job = None
            elif k > self.job["swap_k"]:
                self._row("late", plan=view["row"], plan_ms=round(ms, 1), arrived_at_k=k, **self._job_fields(),
                          reason="arrived after its swap row: never used")
                self.job = None
            else:
                self.job["ready"] = (view, res, ms)

    def on_step(self, k: int, q6, snap: dict):
        if self.ended:
            return None
        hands = (snap.get("msg") or {}).get("hands") or []
        palm = ([float(v) for v in hands[0]["palm_mm"]]
                if len(hands) == 1 and mon_mod.finite_palm(hands[0].get("palm_mm")) else None)
        self._take_results(k)
        if self.motion.force_fire is not None:
            return None
        out = None
        if self.job is not None and k == self.job["swap_k"]:
            out = self._at_swap(k, palm)
            if self.motion.force_fire is not None:
                return None
        if self.job is not None and k > self.job["swap_k"] and not self.job["pending"]:
            self.job["pending"] = True                     # the swap row passed with no plan: the current one goes on
            self._row("pending", reason="retarget pending: the plan for this swap row was not ready; the current plan "
                                        "goes on", **self._job_fields())
            self.job = None
        rows_now = out if out is not None else self.rows
        k_now = 0 if out is not None else k
        if palm is not None and k_now < len(rows_now):
            why = self._knife_too_close(rows_now[k_now], palm)
            if why:
                self.motion.force_fire = (7, why)
                return out
        if (self.job is None and palm is not None and k_now < len(rows_now)
                and mon_mod.drift_mm(palm, self.plan_palm) > RETARGET_MIN_MM
                and mon_mod.drift_mm(palm, self.approved) < mon_mod.DRIFT_MAX_MM):
            self._ask(k_now, palm)
        return out

    def _at_swap(self, k: int, palm):
        """k == the swap row: the goal last sent is rows[k - 1], the pose the new plan starts from."""
        if self.job["ready"] is None:
            return None                                    # 'pending' is written just after
        view, res, ms = self.job["ready"]
        goal = [float(self.rig.sbus.last_goal.get(j, self.rows[k - 1][i])) for i, j in enumerate(g3.ALL)]
        d_palm = None if palm is None or view["palm_mm"] is None else round(mon_mod.drift_mm(palm, view["palm_mm"]), 1)
        d_row = round(max(abs(a - b) for a, b in zip(view["targets"][0][:5], goal[:5])), 3)
        nums = {"palm_vs_plan_mm": d_palm, "first_row_vs_goal_deg": d_row, "plan_ms": round(ms, 1)}
        if d_palm is None or d_palm > RETARGET_MIN_MM:
            self._row("discarded", plan=view["row"], **nums, **self._job_fields(),
                      reason="the palm in the latest message is not the palm this plan was made for")
            self.job = None
            return None
        if d_row > SWAP_MAX_DEG or not all(math.isfinite(v) for v in view["targets"][0]):
            self._row("not_swapped", plan=view["row"], **nums, **self._job_fields(),
                      reason=f"its first row is {d_row:.1f} deg from the goal last sent (> {SWAP_MAX_DEG:g})")
            self.motion.force_fire = (7, f"the re-target plan starts {d_row:.1f} deg from the held goal "
                                         f"(> {SWAP_MAX_DEG:g}): not sent")
            self.job = None
            return None
        self._row("swapped", plan=view["row"], **nums, **self._job_fields(),
                  drift_from_approved_mm=round(mon_mod.drift_mm(view["palm_mm"], self.approved), 1))
        self.plan, self.plan_palm, self.rows, self.job = res, view["palm_mm"], [list(r) for r in view["targets"]], None
        return [list(r) for r in self.rows]
