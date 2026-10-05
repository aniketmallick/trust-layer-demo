"""The eight monitors, read once per control step (PLAN.md section 4). No motion here - this file only decides.

  1 joint_margin  2 joint_speed  3 ee_workspace  4 gripper_overload  5 loop_dt   anchor/g3.py EnvelopeGuard.check
  6 kill_switch        the operator's ESC / Ctrl-C (safety.KillSwitch) or the 300 s operator heartbeat (Interlock)
  7 hand_in_workspace  fires on ONE raw frame with a hand in the workspace, or 3 of the last 5; clears only at 0 of 5.
                       HANDOVER_APPROACH and REORIENT ([o] is pressed with the hand in view): one hand within 50 mm
                       of the palm approved at the key does not fire (drift is measured from the approved palm, never
                       from the last re-target); a palm that is not two finite numbers fires. RELEASE: the gripper
                       alone opens under the operator's own key with ONE hand on the handle - does not fire; a second
                       hand in the message fires.
  8 hand_link          common/hand_link.py monitor8(): the last message older than 300 ms, a seq gap > 2, the
                       message's frame stale or unknown; latched when the link's watchdog set the stop flag.
"""
from __future__ import annotations

import math

CODES = {1: "joint_margin", 2: "joint_speed", 3: "ee_workspace", 4: "gripper_overload", 5: "loop_dt",
         6: "kill_switch", 7: "hand_in_workspace", 8: "hand_link"}
NUM_OF_CODE = {v: k for k, v in CODES.items()}
MOVING = {"GRASP", "LIFT", "APPROACH_ZONE", "PRE_PLACE", "PLACE", "RETREAT", "REORIENT", "HANDOVER_APPROACH", "RELEASE",
          "SETUP", "PLACE_APPROACH", "PLACE_DESCENT", "PLACE_LIFTOFF"}
HANDOVER_PHASES = {"HANDOVER_APPROACH", "REORIENT", "HOLD", "RELEASE", "PLACE_APPROACH", "PLACE_DESCENT", "PLACE_LIFTOFF"}
VEL_LIMIT_SCRIPTED_DPS = 30.0      # scripted segments are commanded at <= 10 deg/s
VEL_LIMIT_HANDOVER_DPS = 15.0      # handover segments are commanded at <= 3 deg/s (PLAN.md section 4: 15, not 9 -
#                                    at 9 the encoder-noise model fired on plans commanded at 3 deg/s)
DRIFT_MAX_MM = 50.0
APPROVED_PHASES = ("HANDOVER_APPROACH", "REORIENT", "PLACE_APPROACH", "PLACE_DESCENT", "PLACE_LIFTOFF")
DRIFT_MAX_BY_PHASE = {"PLACE_APPROACH": 50.0, "PLACE_DESCENT": 15.0, "PLACE_LIFTOFF": float("inf")}   # palm placement (2026-10-03): the approach as
#                          the standoff approach (re-planned above the palm if it moved); the descent freezes at 15 mm
DRIFT_MAX_COVERED_MM = 60.0        # under the arm the palm reads 12-44 mm off with the hand still (00:59): a move counts
#                                    beyond this while the arm covers the palm
OCCLUSION_S = {"PLACE_APPROACH": 1.0, "PLACE_DESCENT": float("inf"), "PLACE_LIFTOFF": float("inf")}   # palm placement (the operator, 2026-10-03):
#   the approved hand may be unseen this long - flicker in the approach, under the arm through the descent - before
#   monitor 7 fires; it reappearing too far off or a second hand still fires at once
FAST_STEP_DEG = 3.0 / 15.0 + 1e-6  # a row commanded faster than 3 deg/s at 15 Hz
VEL_LAG_ROWS = 8                   # monitor 2 keeps a fast row's limit this many rows on: the arm slows after the rows do
OPERATOR_SOURCES = ("ESC", "SIGINT", "dry:", "operator")


def row_vel_limits(rows: list, q_start) -> list:
    """Monitor 2 per row for rows of mixed speed: 30 deg/s while a row in the last VEL_LAG_ROWS moved faster than
    3 deg/s, else 15."""
    prev, fast = [float(v) for v in q_start], []
    for r in rows:
        fast.append(max(abs(float(r[i]) - prev[i]) for i in range(5)) > FAST_STEP_DEG)
        prev = [float(v) for v in r]
    return [VEL_LIMIT_SCRIPTED_DPS if any(fast[max(0, k - VEL_LAG_ROWS):k + 1]) else VEL_LIMIT_HANDOVER_DPS
            for k in range(len(rows))]


def hand_axis(tm, hand: dict) -> tuple | None:
    """(unit wrist -> fingers on the table plane, palm centre -> middle fingertip mm) from a hand's 21 landmarks
    (MediaPipe: 0 wrist, 9 middle-finger knuckle, 12 its tip), each read at the palm's height. None without them."""
    from common.table_map import PALM_H_MM
    lm = hand.get("landmarks_px")
    if not isinstance(lm, list) or len(lm) != 21 or not finite_palm(hand.get("palm_mm")):
        return None
    try:
        w, k9, t12 = (tm.px_to_mm(lm[i], PALM_H_MM)[:2] for i in (0, 9, 12))
    except Exception:  # noqa: BLE001 - a landmark that cannot be mapped: no direction
        return None
    d = [float(k9[0] - w[0]), float(k9[1] - w[1])]
    n = math.hypot(*d)
    if not (n > 10.0 and math.isfinite(n)):
        return None
    reach = math.hypot(float(t12[0]) - float(hand["palm_mm"][0]), float(t12[1]) - float(hand["palm_mm"][1]))
    return (d[0] / n, d[1] / n), round(reach, 1)


FIST_RATIO = 0.55                 # fingertip -> its knuckle over knuckle -> wrist: an open hand ~0.8-1.0, a fist ~0.3


def hand_open(hand: dict) -> bool | None:
    """From the 21 landmarks (pixels; the ratio does not care about scale): False for a fist (the fingertips folded back
    to the knuckles), True for an open hand, None without landmarks."""
    lm = hand.get("landmarks_px")
    if not isinstance(lm, list) or len(lm) != 21:
        return None
    d = lambda a, b: math.hypot(float(lm[a][0]) - float(lm[b][0]), float(lm[a][1]) - float(lm[b][1]))   # noqa: E731
    tips = sum(d(t, m) for t, m in ((8, 5), (12, 9), (16, 13), (20, 17))) / 4.0
    palm = sum(d(m, 0) for m in (5, 9, 13, 17)) / 4.0
    return None if palm < 1e-6 else bool(tips / palm >= FIST_RATIO)


def vel_limit_for(phase: str) -> float:
    return VEL_LIMIT_HANDOVER_DPS if phase in HANDOVER_PHASES else VEL_LIMIT_SCRIPTED_DPS


def scripted_vel_limit(arm_dps: float) -> float:
    """Monitor 2 for scripted segments: 15 deg/s when they are commanded at <= 3 deg/s (the dawn rule), else 30."""
    return VEL_LIMIT_HANDOVER_DPS if float(arm_dps) <= 3.0 + 1e-9 else VEL_LIMIT_SCRIPTED_DPS


def finite_palm(palm_mm) -> bool:
    """palm_mm is two finite numbers (a message with anything else cannot be measured against an approved palm)."""
    try:
        return len(palm_mm) == 2 and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                                         for v in palm_mm)
    except TypeError:
        return False


def drift_mm(palm_mm, approved_mm) -> float:
    return math.hypot(float(palm_mm[0]) - float(approved_mm[0]), float(palm_mm[1]) - float(approved_mm[1]))


CLEAR_FRAMES = 5      # the operator, 2026-10-03: fresh frames with no hand in the workspace before the retreat


class ClearWatch:
    """Counts fresh hand-monitor frames with no hand in the workspace, from the moment it is made: a frame seen before
    (the same seq) is not counted again, a hand in the workspace starts the count again. see() -> True at `need`."""

    def __init__(self, need: int = CLEAR_FRAMES):
        self.need, self.n, self.resets, self.frames, self._seq = int(need), 0, 0, 0, None

    def see(self, msg: dict | None) -> bool:
        seq = (msg or {}).get("seq")
        if msg is None or seq is None or seq == self._seq:
            return self.n >= self.need
        self._seq, self.frames = seq, self.frames + 1
        if any(bool(h.get("in_workspace")) for h in msg.get("hands") or []):
            self.resets += 1 if self.n else 0                    # a count under way, broken by a hand
            self.n = 0
        else:
            self.n += 1
        return self.n >= self.need


def monitor7(msg: dict | None, window: list, phase: str, approved_palm=None, unseen_s: float = 0.0,
             covering: bool = False, drift_unreliable: bool = False) -> dict:
    """-> {fired, detail, present, clear, n_hands, drift_mm}. `window`: the last <= 5 raw per-frame booleans (any hand
    in_workspace), oldest first, the latest message included. `clear` (0 of 5) is what lets a stationary state move."""
    hands = list((msg or {}).get("hands") or [])
    raw = any(bool(h.get("in_workspace")) for h in hands)
    w = [bool(x) for x in (window or [])][-5:]
    n = sum(w)
    present = raw or n >= 3
    out = {"present": present, "clear": (n == 0 and not raw), "n_hands": len(hands), "n_of_5": n, "drift_mm": None}
    one = hands[0] if len(hands) == 1 else None
    measurable = one is not None and finite_palm(one.get("palm_mm"))
    if measurable and approved_palm is not None:
        out["drift_mm"] = round(drift_mm(one["palm_mm"], approved_palm), 1)
    if phase not in MOVING:
        return {**out, "fired": False, "detail": f"stationary ({phase}): hand present={present} ({n} of 5)"}
    if phase == "RELEASE":
        if len(hands) >= 2:
            return {**out, "fired": True, "detail": f"a second hand during the release ({len(hands)} hands)"}
        return {**out, "fired": False, "detail": "release: the gripper alone opens, the operator's own key, one hand"}
    if phase in APPROVED_PHASES and approved_palm is not None:
        if len(hands) >= 2:
            return {**out, "fired": True, "detail": f"a second hand during the approach ({len(hands)} hands)"}
        if one is not None and not measurable:
            return {**out, "fired": True, "detail": "the hand's palm position is not two finite numbers: it cannot be "
                                                    "measured against the approved palm"}
        if one is not None:
            lim = DRIFT_MAX_BY_PHASE.get(phase, DRIFT_MAX_MM)
            if drift_unreliable and phase in OCCLUSION_S:              # the hand as measured is under the arm: its
                return {**out, "fired": False, "detail": "the approved hand under the arm: its position reading "
                                                         "(12-73 mm off with the hand still) is not used"}
            if out["drift_mm"] >= lim:
                return {**out, "fired": True, "detail": f"the palm is {out['drift_mm']:.0f} mm from the approved palm "
                                                        f"(>= {lim:g})"}
            return {**out, "fired": False, "detail": f"the approved hand, drift {out['drift_mm']:.0f} mm (< {lim:g})"}
        if n >= 3:
            return {**out, "fired": False, "detail": f"the approved hand not in this frame ({n} of 5: a dropout)"}
        lim = OCCLUSION_S.get(phase)
        if lim is not None and covering:
            lim = float("inf")                         # the arm or the prop over the approved palm: it hides the hand
        if lim is not None and unseen_s < lim:
            return {**out, "fired": False, "detail": f"the approved hand unseen for {unseen_s:.1f} s (< {lim:g}: "
                                                     "flicker, or under the arm)"}
        return {**out, "fired": True, "detail": f"the approved hand is not seen ({n} of 5)"}
    if raw:
        h = next(h for h in hands if h.get("in_workspace"))
        return {**out, "fired": True, "detail": f"a hand in the workspace at palm {h.get('palm_mm')} mm (raw frame; {n} of 5)"}
    if n >= 3:
        return {**out, "fired": True, "detail": f"a hand in the workspace in {n} of the last 5 frames"}
    return {**out, "fired": False, "detail": f"no hand in the workspace ({n} of 5)"}


def monitor6(stop_requested: bool, source: str | None, heartbeat_due: bool = False) -> tuple:
    src = str(source or "")
    if stop_requested and src.startswith(OPERATOR_SOURCES):
        return True, f"operator stop ({src})"
    if heartbeat_due:
        return True, "no operator confirmation for 300 s (G3.6)"
    return False, "not pressed"


OFF_DETAIL = "OFF: the trust layer is off (--layer-off) - not read"


def off_state(n: int) -> dict:
    """A monitor that is not read (--layer-off, the operator 2026-10-04: 7 and 8 with the hand monitor). Never fires;
    the row says OFF. Monitor 7 reads clear: nothing that needs a hand reading is offered in that mode."""
    out = {"code": CODES[n], "fired": False, "off": True, "detail": OFF_DETAIL}
    if n == 7:
        out.update(present=False, clear=True, n_hands=0, n_of_5=0, drift_mm=None)
    return out


def evaluate(guard, t_s: float, q6, q_prev, dt_s, load, phase: str, stop_requested: bool, stop_source: str | None,
             snapshot: dict | None, link8: tuple, approved_palm=None, heartbeat_due: bool = False,
             vel_limit: float | None = None, unseen_s: float = 0.0, covering: bool = False,
             drift_unreliable: bool = False, off: tuple = ()) -> dict:
    """One control step -> {"1".."8": {code, fired, detail}} (+ "7" carries present / clear / n_hands / drift_mm).
    off: monitors not read this session (only 7 and 8 may be: --layer-off); each reported OFF, never fired."""
    if any(n not in (7, 8) for n in off):
        raise ValueError(f"only monitors 7 and 8 can be off, not {off}")
    guard.vel_limit_dps = float(vel_limit) if vel_limit is not None else vel_limit_for(phase)
    by_code: dict = {}
    for code, detail in guard.check(t_s, q6, q_prev, dt_s, load):
        by_code.setdefault(code, []).append(detail)
    out = {}
    for n in range(1, 6):
        d = by_code.get(CODES[n])
        out[str(n)] = {"code": CODES[n], "fired": bool(d), "detail": "; ".join(d) if d else "inside"}
    f6, d6 = monitor6(stop_requested, stop_source, heartbeat_due)
    out["6"] = {"code": CODES[6], "fired": f6, "detail": d6}
    snap = snapshot or {}
    out["7"] = {"code": CODES[7], **monitor7(snap.get("msg"), snap.get("window") or [], phase, approved_palm, unseen_s,
                                             covering, drift_unreliable)}
    f8, d8 = bool(link8[0]), str(link8[1])
    if stop_requested and str(stop_source or "").startswith("monitor8"):      # the watchdog's stop is latched
        f8, d8 = True, str(stop_source)
    out["8"] = {"code": CODES[8], "fired": f8, "detail": d8}
    for n in off:
        out[str(n)] = off_state(n)
    return out


def fired_list(mon: dict) -> list:
    """[(number, code, detail)] of the monitors that fired, in number order."""
    return [(int(k), v["code"], v["detail"]) for k, v in sorted(mon.items(), key=lambda kv: int(kv[0])) if v["fired"]]


def all_clear(mon: dict) -> bool:
    """Every monitor quiet AND monitor 7 at 0 of 5: the condition for [c] and for leaving a stationary state."""
    return not fired_list(mon) and bool(mon["7"].get("clear"))
