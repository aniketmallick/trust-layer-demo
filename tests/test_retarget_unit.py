"""retarget.py's decisions, step by step, with no clock: a fake worker whose results arrive at the steps a test chooses,
a fake loop that sends what on_step lets it send, real plans from the planner. Reviewer 2026-10-01 (SR08): the planner
never runs on the control thread's clock; a plan not ready at its swap row is not used and the current plan goes on."""
from __future__ import annotations

import types

import pytest

from common import paths

paths.use_anchor()
import g3  # noqa: E402

import retarget as rt  # noqa: E402
from planner import fixtures as fx  # noqa: E402
from planner import handover_planner as hp  # noqa: E402

PALM0 = (340.0, 100.0)


class FakeWorker:
    kind = "fake"

    def __init__(self):
        self.asked, self.out = [], []

    def submit(self, jid, plan, palm, start):
        self.asked.append({"id": jid, "plan": plan, "palm": list(palm), "start": list(start)})

    def poll(self):
        out, self.out = self.out, []
        return out


class Loop:
    """run_rows's part that matters here: on_step before each send; a replacement restarts at its row 0; a fire stops."""

    def __init__(self, plan, cfg):
        self.cfg, self.worker, self.rows_log = cfg, FakeWorker(), []
        self.motion = types.SimpleNamespace(force_fire=None)
        self.rig = types.SimpleNamespace(sbus=types.SimpleNamespace(last_goal={}))
        self.n = 0
        self.r = rt.Retargeter(self.worker, plan, list(plan.targets), PALM0, cfg, self, self.motion, self.rig,
                               {"session_id": "unit", "trial_id": "unit/T001"}, self.next_id)
        self.rows, self.k, self.sent, self.fired = [list(x) for x in plan.targets], 0, [], None

    def next_id(self):
        self.n += 1
        return self.n

    def append(self, row):                                   # the chain
        self.rows_log.append(row)
        return row

    def step(self, palm=PALM0, hands=None):
        hs = hands if hands is not None else ([] if palm is None else [{"palm_mm": list(palm)}])
        new = self.r.on_step(self.k, None, {"msg": {"hands": hs}})
        if self.motion.force_fire is not None:
            self.fired, self.motion.force_fire = self.motion.force_fire, None
            return "frozen"
        if new is not None:
            self.rows, self.k = [list(x) for x in new], 0
        if self.k >= len(self.rows):
            return "done"
        self.sent.append(list(self.rows[self.k]))
        self.rig.sbus.last_goal = dict(zip(g3.ALL, self.rows[self.k]))
        self.k += 1
        return "sent"

    def statuses(self):
        return [r["status"] for r in self.rows_log]


@pytest.fixture(scope="module")
def setup():
    cfg = fx.cfg_for("sim")
    plan = hp.plan_handover(list(fx.start_pose("sim")), PALM0, cfg)
    assert plan.ok
    return cfg, plan


def answer(loop, palm, job=-1):
    """The worker finishes the job with a real re-target plan for `palm` from the job's start pose."""
    a = loop.worker.asked[job]
    res = hp.retarget(a["plan"], list(palm), a["start"], loop.cfg)
    loop.worker.out.append((a["id"], res, None, 50.0))
    return res


R = lambda row: tuple(round(float(v), 3) for v in row)       # noqa: E731


def test_RT_swap_at_the_swap_row_continues_from_the_pose_sent(setup):
    cfg, plan = setup
    L = Loop(plan, cfg)
    p1 = (340.0, 112.0)
    for _ in range(3):
        L.step()
    L.step(p1)                                               # k=3: the palm moved 12 mm -> a job for row 3 + 8
    assert L.statuses() == ["asked"] and L.r.job["swap_k"] == 3 + rt.LOOKAHEAD_ROWS
    assert L.worker.asked[0]["start"] == list(plan.targets[3 + rt.LOOKAHEAD_ROWS - 1])
    res = answer(L, p1)
    while L.r.job is not None:
        assert L.step(p1) == "sent"
    sw = [r for r in L.rows_log if r["status"] == "swapped"][0]
    assert sw["palm_vs_plan_mm"] == 0.0 and sw["first_row_vs_goal_deg"] <= rt.SWAP_MAX_DEG
    i = 3 + rt.LOOKAHEAD_ROWS
    assert L.sent[:i] == [list(x) for x in plan.targets[:i]]         # the current plan ran on to the swap row
    assert L.sent[i] == list(res.targets[0])                          # then the new plan, from its first row


def test_RT_palm_moved_while_planning_discarded_then_asked_again(setup):
    cfg, plan = setup
    L = Loop(plan, cfg)
    p1, p2 = (340.0, 112.0), (340.0, 132.0)
    L.step(p1)                                               # k=0: asked for p1
    stale = answer(L, p1)
    for _ in range(rt.LOOKAHEAD_ROWS - 1):
        L.step(p2)                                           # the palm moves on while the p1 plan is computed
    assert L.step(p2) == "sent"                              # the swap row: p2 is 20 mm from p1 -> discarded, asked again
    assert L.statuses()[:3] == ["asked", "discarded", "asked"]
    d = L.rows_log[1]
    assert 19.0 < d["palm_vs_plan_mm"] < 21.0 and d["swapped"] is False
    fresh = answer(L, p2)
    while L.r.job is not None:
        L.step(p2)
    assert L.statuses()[-1] == "swapped"
    only_stale = {R(x) for x in stale.targets} - {R(x) for x in fresh.targets}
    assert not (only_stale & {R(x) for x in L.sent})                  # no row of the discarded plan was sent


def test_RT_not_ready_at_the_swap_row_pending_current_plan_goes_on_late_never_used(setup):
    cfg, plan = setup
    L = Loop(plan, cfg)
    p1 = (340.0, 112.0)
    L.step(p1)
    for _ in range(rt.LOOKAHEAD_ROWS + 1):
        L.step(p1)                                           # no result: the swap row passes
    assert "pending" in L.statuses()
    late = answer(L, p1, job=0)                              # job 1's result arrives after its swap row
    L.step(p1)
    assert "swapped" not in L.statuses()
    assert L.sent == [list(x) for x in plan.targets[:len(L.sent)]]    # the current plan, row after row
    assert not ({R(x) for x in late.targets} - {R(x) for x in plan.targets}) & {R(x) for x in L.sent}
    assert len(L.worker.asked) == 2                                    # asked again: the palm is still off


def test_RT_knife_too_close_to_the_palm_as_it_is_now_fires_before_the_send(setup):
    cfg, plan = setup
    L = Loop(plan, cfg)
    for _ in range(len(plan.targets) - 3):
        L.step()
    n = len(L.sent)
    from planner.knife import knife_pose
    tip = knife_pose(cfg.to_mjcf(L.rows[L.k]), cfg.knife).handle_tip
    assert L.step((float(tip[0]) + 30.0, float(tip[1]))) == "frozen"  # a palm 30 mm past the handle tip, still < 50 drift?
    assert L.fired[0] == 7 and "knife" in L.fired[1] and len(L.sent) == n


def test_RT_knife_guard_quiet_on_the_plan_as_made_control(setup):
    cfg, plan = setup
    L = Loop(plan, cfg)
    while (s := L.step()) == "sent":
        pass
    assert s == "done" and L.fired is None and len(L.sent) == len(plan.targets)
    assert max(L.r.guard_ms) < 5.0                                     # the per-step check costs well under a step


def test_RT_refused_or_dead_worker_freezes(setup):
    cfg, plan = setup
    L = Loop(plan, cfg)
    L.step((340.0, 112.0))
    L.worker.out.append((1, hp.Refusal("no pose (unit)"), None, 40.0))
    assert L.step((340.0, 112.0)) == "frozen" and "re-target was refused" in L.fired[1]
    L2 = Loop(plan, cfg)
    L2.step()
    L2.worker.out.append(("dead", None, "the re-target worker exited (code -9)", 0.0))
    assert L2.step() == "frozen" and "worker failed" in L2.fired[1]


def test_RT_a_plan_not_starting_at_the_goal_sent_is_never_sent(setup):
    cfg, plan = setup
    L = Loop(plan, cfg)
    p1 = (340.0, 112.0)
    L.step(p1)
    a = L.worker.asked[0]
    res = hp.retarget(a["plan"], p1, a["start"], cfg)
    import dataclasses
    shifted = dataclasses.replace(res, targets=tuple([r[0] + 3.0] + list(r[1:]) for r in res.targets))
    L.worker.out.append((a["id"], shifted, None, 50.0))
    out = None
    while out != "frozen":
        out = L.step(p1)
    assert "deg from the held goal" in L.fired[1] and "not_swapped" in L.statuses()
    assert not ({R(x) for x in shifted.targets} & {R(x) for x in L.sent})
