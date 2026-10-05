"""The demonstration's states and transitions as data, and one pure transition function (PLAN.md section 2).

    IDLE -> GRASP -> LIFT -> CP1(judge) -> APPROACH_ZONE -> PRE_PLACE -> CP2(judge) -> PLACE -> RETREAT -> IDLE
    any moving state, on any monitor: FREEZE -> ASK -> {WAIT | HANDOVER_PLAN -> HANDOVER_APPROACH -> HOLD -> CP3(judge)
        -> ASK_RELEASE -> RELEASE -> RETREAT | REORIENT -> HANDOVER_PLAN | KILL | LEAVE}

The runner asks this file what comes next; nothing here reads a sensor or writes a goal. An event that has no row in
the table is refused (the state does not change) - an unknown event never moves the arm.
"""
from __future__ import annotations

MOVING = ("GRASP", "LIFT", "APPROACH_ZONE", "PRE_PLACE", "PLACE", "RETREAT", "REORIENT", "HANDOVER_APPROACH", "RELEASE",
          "PLACE_APPROACH", "PLACE_DESCENT", "PLACE_LIFTOFF")
STATIONARY = ("IDLE", "CP1", "CP2", "CP3", "FREEZE", "ASK", "WAIT", "HANDOVER_PLAN", "HOLD", "ASK_RELEASE",
              "KILL", "LEAVE")
STATES = MOVING + STATIONARY
CHECKPOINTS = ("CP1", "CP2", "CP3")
TASK_ORDER = ("GRASP", "LIFT", "CP1", "APPROACH_ZONE", "PRE_PLACE", "CP2", "PLACE", "RETREAT")

# (state, event) -> next state. Events: done (a segment ran to its end), fire (any monitor 1-8), safe / unsafe (a
# judge verdict as the code reads it), key:<k>, plan_ok / plan_none, gate (a moving state refused at its start).
T: dict = {
    ("IDLE", "start"): "GRASP",
    ("GRASP", "done"): "LIFT",
    ("LIFT", "done"): "CP1",
    ("CP1", "safe"): "APPROACH_ZONE",
    ("APPROACH_ZONE", "done"): "PRE_PLACE",
    ("PRE_PLACE", "done"): "CP2",
    ("CP2", "safe"): "PLACE",
    ("PLACE", "done"): "RETREAT",
    ("RETREAT", "done"): "IDLE",
    ("FREEZE", "judged"): "ASK",
    ("ASK", "key:w"): "WAIT",
    ("WAIT", "judged"): "ASK",
    ("ASK", "key:h"): "HANDOVER_PLAN",
    ("ASK", "key:o"): "REORIENT",
    ("ASK", "key:k"): "KILL",
    ("ASK", "key:l"): "LEAVE",
    ("REORIENT", "done"): "HANDOVER_PLAN",
    ("HANDOVER_PLAN", "plan_ok"): "HANDOVER_APPROACH",
    ("HANDOVER_PLAN", "plan_none"): "ASK",
    ("HANDOVER_APPROACH", "done"): "HOLD",
    ("HOLD", "settled"): "CP3",
    ("CP3", "safe"): "ASK_RELEASE",
    ("CP3", "unsafe"): "ASK_RELEASE",          # the arm is holding; the verdict is shown beside the release menu
    ("ASK_RELEASE", "key:r"): "RELEASE",
    ("ASK_RELEASE", "key:w"): "CP3",
    ("ASK_RELEASE", "key:k"): "KILL",
    ("ASK_RELEASE", "key:l"): "LEAVE",
    ("RELEASE", "done"): "RETREAT",
    # palm placement (--palm-placement, 2026-10-03): the pre-authorised [h] plans a placement; CP3 not UNSAFE releases
    ("HANDOVER_PLAN", "place_ok"): "PLACE_APPROACH",
    ("PLACE_APPROACH", "done"): "PLACE_DESCENT",
    ("PLACE_APPROACH", "replan"): "HANDOVER_PLAN",            # above the palm, the hand moved: a fresh plan from there
    ("PLACE_DESCENT", "done"): "HOLD",
    ("CP3", "release_ok"): "RELEASE",
    ("ASK", "key:r"): "RELEASE",              # palm placement: the operator's [r] at the freeze menu (2026-10-04)
}
for _s in MOVING + ("HOLD",):
    T[(_s, "fire")] = "FREEZE"                 # any monitor 1-8 while moving; monitors 1-6 and 8 while holding
for _s in ("IDLE", "CP1", "CP2", "CP3", "HOLD"):
    T[(_s, "gate")] = "FREEZE"                 # a moving state was refused at its start (monitor 7 or 8 not clear)
for _s in ("CP1", "CP2"):
    T[(_s, "unsafe")] = "FREEZE"               # an UNSAFE verdict at a checkpoint: the arm stays frozen, ASK is shown


def next_state(state: str, event: str, resume: str | None = None, all_clear: bool = False) -> tuple:
    """-> (next state, refusal or None). `resume` is the moving state the freeze interrupted; [c] returns to it and
    is offered only when every monitor is clear."""
    if state not in STATES:
        return state, f"unknown state {state}"
    if (state, event) == ("ASK", "key:c"):
        if not all_clear:
            return state, "[c] refused: a monitor is not clear"
        if resume not in MOVING:
            return state, "[c] refused: nothing to continue"
        return resume, None
    nxt = T.get((state, event))
    if nxt is None:
        return state, f"no transition from {state} on {event}"
    return nxt, None


def may_start_moving(m7_clear: bool, m8_fired: bool) -> str | None:
    """A stationary state enters a moving one only with monitors 7 and 8 clear. -> the refusal, or None."""
    if m8_fired:
        return "monitor 8 (hand link) is firing"
    if not m7_clear:
        return "monitor 7: a hand in the workspace in the last 5 frames"
    return None


def handover_refusal(n_hands: int, m8_fired: bool, blade_toward_palm: bool | None, plan_ok: bool | None,
                     plan_reason: str = "", reoriented: bool = False) -> str | None:
    """[h] is refused by code (back to ASK, the reason shown) when: no hand or more than one hand, monitor 8 firing,
    the blade heading toward the palm ([o] is the path), or the planner returned no plan."""
    if m8_fired:
        return "monitor 8 (hand link) is firing: no trusted palm position"
    if n_hands == 0:
        return "no hand is seen: nothing to hand over to"
    if n_hands > 1:
        return f"{n_hands} hands are seen: one hand only"
    if blade_toward_palm and not reoriented:
        return "the blade heading is toward the palm: [o] re-orient first"
    if plan_ok is False:
        return f"the planner returned no plan: {plan_reason}"
    return None
