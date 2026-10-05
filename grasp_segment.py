"""Tonight's scripted pick-and-place: a PLACEHOLDER (PLAN.md section 5; the dawn recording replaces this table).

The frozen null plan's cell for cross 6 (phase0/null/NULL-001_plan_9fb34cea_e3.json, read-only, hash-checked) with
its waypoints kept and its speed cut from the null's 2.86 deg per step (43 deg/s) to <= 10 deg/s (PLAN.md F3). The
null's phases map onto the demonstration's states:

    GRASP          rest -> over the grasp point -> descend -> close -> settle
    LIFT           lift                                                   then CP1 (judge)
    APPROACH_ZONE  carry over the zone centre
    PRE_PLACE      settle above the zone                                  then CP2 (judge)
    PLACE          lower -> open to release
    RETREAT        rise -> return to rest -> hold

The close target is the null's measured 22.35 % unless planner/knife.py offers a caliper close target for the prop's
handle; either way it is a placeholder until the hold check at dawn (dawn protocol step 2).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from common import paths

paths.use_anchor()
import g3  # noqa: E402

NULL_PLAN_SHA = "8ecf9a00d87f125206d1f2096e116204e665bf04137cb561142ac9511e226d39"   # PR-001 erratum 3 (canonical)
CROSS = 6
CONTROL_HZ = 15.0
SCRIPTED_DPS = 10.0              # every arm joint, scripted segments (the anchor's REST_DPS)
HANDOVER_DPS = 3.0               # every arm joint, handover segments
GRIP_PPS = 30.0                  # the gripper, % per second
SETTLE_STEPS = 5
STATE_OF_PHASE = {"transit": "GRASP", "descent": "GRASP", "grasp": "GRASP", "lift": "LIFT", "carry": "APPROACH_ZONE",
                  "lower": "PLACE", "release": "PLACE", "retreat": "RETREAT", "return": "RETREAT"}


def interpolate(q_from, q_to, arm_dps: float = SCRIPTED_DPS, grip_pps: float = GRIP_PPS, hz: float = CONTROL_HZ) -> list:
    """15 Hz rows from q_from to q_to in joint space, every joint arriving together, no arm joint above arm_dps and
    the gripper not above grip_pps. The start row is not included; the last row is q_to."""
    caps = [arm_dps / hz] * 5 + [grip_pps / hz]
    d = [float(b) - float(a) for a, b in zip(q_from, q_to)]
    n = int(max(1, max(math.ceil(abs(x) / c - 1e-9) for x, c in zip(d, caps))))
    return [[round(float(a) + x * (k / n), 4) for a, x in zip(q_from, d)] for k in range(1, n + 1)]


def knife_close_pct(gripper_floor_pct: float) -> tuple:
    """(close target % or None, where it came from). planner/knife.py's caliper helper when it is there (the A8
    coefficient on the prop's placeholder handle width); else the null's measured target stands."""
    try:
        from planner import knife
        got = knife.close_target_pct(knife.Knife().handle_width_mm, gripper_floor_pct)
        return float(got["target_pct"]), (f"planner/knife.py close_target_pct: handle {got['width_mm']:g} mm -> contact "
                                          f"{got['contact_pct']} % - {got['squeeze_pct']:g} % squeeze (A8 caliper "
                                          f"{got['mm_per_pct']} mm per %) - PLACEHOLDER, not verified with a hold")
    except Exception as e:  # noqa: BLE001 - the planner is a sibling's; without it the null's target stands
        return None, f"the null's measured close target (erratum 3) - PLACEHOLDER for the prop ({type(e).__name__})"


def load(root: Path | None = None, gripper_floor_pct: float = 1.31, arm_dps: float = SCRIPTED_DPS) -> dict:
    """-> {segments: [{state, rows, from, to}], rest, open_pct, close_pct, release_pct, source, ...}."""
    root = Path(root) if root is not None else paths.ROOT
    p = root / paths.NULL_PLAN_REL
    plan = json.loads(p.read_text(encoding="utf-8"))
    if g3.canonical_hash(plan) != plan.get("sha256") or plan["sha256"] != NULL_PLAN_SHA:
        raise ValueError(f"{paths.NULL_PLAN_REL} is not the frozen erratum-3 plan ({NULL_PLAN_SHA[:12]})")
    cell = next(c for c in plan["cells"] if c["cross"] == CROSS)
    rest = [float(v) for v in plan["geometry"]["rest_lerobot_deg"]]
    grip = plan["geometry"]["gripper"]
    null_close = float(grip["close_pct"])
    close, close_src = knife_close_pct(gripper_floor_pct)
    close = null_close if close is None else close
    release = max(float(plan["geometry"].get("release_pct", 31.81)), close + 10.0)
    segs: dict = {}
    order = ["GRASP", "LIFT", "APPROACH_ZONE", "PRE_PLACE", "PLACE", "RETREAT"]
    cur = list(rest)
    waypoints = []
    for ph in cell["phases"]:
        if ph["check"] == "hold":
            continue                                           # the null's settle / success hold: ours are below
        end = [float(v) for v in cell["targets_lerobot"][ph["from_step"] - 1 + ph["n_steps"] - 1]]
        if abs(end[5] - null_close) < 0.01:
            end[5] = close                                     # the prop's close target in place of the cube's
        elif ph["check"] == "release":
            end[5] = release
        state = STATE_OF_PHASE[ph["check"]]
        segs.setdefault(state, []).extend(interpolate(cur, end, arm_dps))
        waypoints.append({"null_phase": ph["phase"], "state": state, "end_lerobot": [round(v, 3) for v in end]})
        if ph["check"] == "grasp":
            segs[state].extend([list(end)] * SETTLE_STEPS)     # settle (closed)
        if ph["check"] == "carry":
            segs["PRE_PLACE"] = [list(end)] * SETTLE_STEPS     # settle above the zone, before CP2
        cur = end
    segs["RETREAT"].extend([list(cur)] * SETTLE_STEPS)
    return {"segments": [{"state": s, "rows": segs[s]} for s in order], "rest": rest, "close_pct": close,
            "open_pct": float(grip["open_pct"]), "release_pct": release,
            "close_source": close_src, "waypoints": waypoints, "placeholder": True,
            "source": {"file": paths.NULL_PLAN_REL, "sha256": plan["sha256"], "cross": CROSS,
                       "sequence_sha256": cell["sequence_sha256"]},
            "speeds": {"arm_dps": float(arm_dps), "gripper_pps": GRIP_PPS, "hz": CONTROL_HZ},
            "label": f"PLACEHOLDER: the null's cross-6 grasp geometry at <= {float(arm_dps):g} deg/s; the dawn recording "
                     "replaces it"}


def max_step(rows: list, start: list) -> tuple:
    """(largest arm-joint step in deg, largest gripper step in %) over a segment, from `start`."""
    prev, arm, grip = list(start), 0.0, 0.0
    for r in rows:
        arm = max(arm, max(abs(a - b) for a, b in zip(r[:5], prev[:5])))
        grip = max(grip, abs(r[5] - prev[5]))
        prev = r
    return arm, grip


def load_file(p: Path) -> dict:
    """The dawn-recorded segments (dawn/segments.py, 2026-10-02), in the shape load() returns. The file's own sha256
    is checked, and its row checks must have passed. A file that fails either is refused (ValueError)."""
    import hashlib
    d = json.loads(Path(p).read_text(encoding="utf-8"))
    body = {k: v for k, v in d.items() if k != "sha256"}
    got = hashlib.sha256(json.dumps(body, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    if got != d.get("sha256"):
        raise ValueError(f"{Path(p).name}: its rows do not match its sha256 (edited after it was checked)")
    if d.get("ok") is not True:
        raise ValueError(f"{Path(p).name}: its own row checks did not pass")
    return {"segments": d["segments"], "rest": [float(v) for v in d["rest"]], "close_pct": float(d["close_pct"]),
            "open_pct": float(d["open_pct"]), "release_pct": float(d["release_pct"]), "close_source": d["close_source"],
            "waypoints": d["waypoints"], "placeholder": False, "hold_test": d.get("hold_test"),
            "source": {"file": str(Path(p).name), "sha256": d["sha256"], "built_by": d.get("source")},
            "speeds": d["speeds"], "label": d["label"]}
