"""The presentation page (the operator, 2026-10-04: VIDEO PREP): present.html beside replay.html, a fixed 720x1080
right-hand panel for the video - the fingertip path in 3D with a status badge over it (top, 420 px), the timeline with
a now-line (middle, 420 px), the call log scrolling as it plays (bottom, 240 px). Played at real time from the
session's first motion; space pauses; 1x / 2x / 4x.

Everything comes from the session's chain (build_replay.build verifies it first). Step rows carry their own clock
(t_s); every other row carries only t_iso (1 s resolution): it is placed on the steps' clock by one offset fitted to
all rows' neighbouring steps, then clamped between them, so its order is the chain's. A stretch with no row at all
longer than CUT_MIN_S (the runner waiting at a prompt: nothing is commanded without a step row) is played in
CUT_SHOW_S and marked on the page as cut; ?real=1 plays it at real time.
"""
from __future__ import annotations

import datetime
import json
import math
from pathlib import Path

from monitors import MOVING

HERE = Path(__file__).resolve().parent
TEMPLATE = HERE / "present_template.html"
CUT_MIN_S = 30.0          # a gap with no row longer than this is cut
CUT_SHOW_S = 4.0          # ... to this many seconds on screen
JUDGE_SHOW_S = 3.0        # a verdict stays on the badge this long, unless the arm moves first
FREEZE_SHOW_S = 1.5       # a freeze holds the badge at least this long: a judge call begun inside it is shown from
#                           then on (its true start is on the timeline and in the call log)
TURN_MIN_DEG = 30.0       # a placement plan turning the screwdriver at least this much: the badge says RE-ORIENT
MON_OFF, MON_QUIET, MON_FIRED, MON_PRESENT = "3", "0", "1", "2"
MOTION_CLS = ("moving", "reorient", "release")
PRETTY_CODE = {"joint_margin": "joint margin", "joint_speed": "joint speed", "ee_workspace": "workspace box",
               "gripper_overload": "gripper load", "loop_dt": "loop time", "kill_switch": "kill switch",
               "hand_in_workspace": "hand in workspace", "hand_link": "hand link", "judge_unsafe": "judge UNSAFE",
               "internal": "internal error", "hardware": "hardware", "refused": "refused", "motion_hours": "hours"}


def _epoch(iso) -> float | None:
    try:
        return datetime.datetime.fromisoformat(str(iso)).timestamp()
    except (TypeError, ValueError):
        return None


def row_times(rows: list) -> tuple:
    """-> ({seq: seconds on the steps' clock}, offset: epoch - steps' clock). Steps exact; others by t_iso (floored to
    the second by g3.now_iso), on one fitted offset, clamped between the neighbouring steps, never before the row
    above them."""
    n = len(rows)
    prev_t, next_t, last = [None] * n, [None] * n, None
    for i, r in enumerate(rows):
        if r["kind"] == "step":
            last = float(r["t_s"])
        prev_t[i] = last
    last = None
    for i in range(n - 1, -1, -1):
        if rows[i]["kind"] == "step":
            last = float(rows[i]["t_s"])
        next_t[i] = last
    lo, hi, mids = -math.inf, math.inf, []
    for i, r in enumerate(rows):
        e = _epoch(r.get("t_iso")) if r["kind"] != "step" else None
        if e is None:
            continue
        if next_t[i] is not None:
            lo = max(lo, e - next_t[i])
        if prev_t[i] is not None:
            hi = min(hi, e + 1.0 - prev_t[i])
        if prev_t[i] is not None and next_t[i] is not None:
            mids.append(e + 0.5 - (prev_t[i] + next_t[i]) / 2.0)
    if math.isfinite(lo) and math.isfinite(hi) and lo <= hi:
        off = (lo + hi) / 2.0
    elif mids:
        off = sorted(mids)[len(mids) // 2]
    else:
        off = lo if math.isfinite(lo) else (hi if math.isfinite(hi) else 0.0)
    out, t_prev = {}, -math.inf
    for i, r in enumerate(rows):
        if r["kind"] == "step":
            t = float(r["t_s"])
        else:
            e = _epoch(r.get("t_iso"))
            t = (e + 0.5 - off) if e is not None else (prev_t[i] if prev_t[i] is not None else t_prev)
            if prev_t[i] is not None:
                t = max(t, prev_t[i])
            if next_t[i] is not None:
                t = min(t, next_t[i])
        t = max(t, t_prev) if math.isfinite(t_prev) else t
        out[r["seq"]] = t
        t_prev = t
    return out, off


def first_motion(steps: list) -> float:
    """The session's first commanded step (a target sent), else its first step."""
    s = next((r for r in steps if r.get("target6") is not None), steps[0])
    return float(s["t_s"])


def cuts(times: list, t_end: float) -> list:
    """[[a, b]] relative seconds: the gaps longer than CUT_MIN_S between consecutive rows (and to the end)."""
    pts = sorted(t for t in times if t >= 0.0) + [t_end]
    return [[round(a, 3), round(b, 3)] for a, b in zip(pts, pts[1:]) if b - a > CUT_MIN_S]


def _mon_char(m: dict) -> str:
    if m.get("off"):
        return MON_OFF
    if m.get("fired"):
        return MON_FIRED
    return MON_PRESENT if m.get("present") else MON_QUIET


def _pretty(phase: str) -> str:
    return str(phase).replace("_", " ")


def _turning(plan: dict) -> tuple | None:
    """A placement / handover plan's turning rows (1-based, inclusive) and degrees, when it turns >= TURN_MIN_DEG."""
    turned = float(((plan.get("checks") or {}).get("turned_deg")) or 0.0)
    ph = next((p for p in plan.get("phases") or [] if "turn" in str(p.get("phase"))), None)
    if ph is None or abs(turned) < TURN_MIN_DEG:
        return None
    a = int(ph["from_step"])
    return a, a + int(ph["n_steps"]) - 1, turned


def _freeze_badge(r: dict) -> tuple:
    code = PRETTY_CODE.get(r.get("code"), str(r.get("code")))
    label = f"FREEZE — monitor {r['monitor']}" if r.get("monitor") else f"FREEZE — {code}"
    ms = r.get("t_hold_minus_frame_t_ms")
    what = f"{code}: {str(r.get('detail') or '').split(' (')[0][:60]}" if r.get("monitor") else code
    if not r.get("held", True):
        return label, f"{what} · THE HOLD FAILED"
    return label, (f"{what} · held in {ms:.0f} ms" if ms is not None else f"{what} · held (goals := present)")


def _key_text(r: dict) -> str:
    k = r.get("key")
    who = f"auto — {r['by']}" if r.get("by") else "typed by the operator"
    out = f"[{k}] {who}"
    if r.get("refused"):
        out += " — REFUSED: " + str(r["refused"])[:90]
    return out


def badge_tracks(rows: list, tt: dict, t0: float) -> tuple:
    """-> (base [[t, label, sub, cls]], overlays [[t0, t1, label, sub, cls]], chips [[t, text, cls]]), relative t."""
    base, over, chips = [], [], []
    last, freeze_on, turn, k_app, hand_now = None, False, None, 0, False

    def put(t, label, sub, cls):
        nonlocal last
        if last != (label, sub, cls):
            base.append([round(t, 3), label, sub, cls])
            last = (label, sub, cls)

    for r in rows:
        t, k = tt[r["seq"]] - t0, r["kind"]
        if k == "step":
            ph = r["phase"]
            m7 = (r.get("monitors") or {}).get("7") or {}
            hand_now = bool(m7.get("present")) and not m7.get("off")
            if ph in MOVING:
                freeze_on = False
                if ph == "PLACE_APPROACH":
                    k_app += 1
                if ph == "RELEASE":
                    put(t, "RELEASE", "the gripper opens", "release")
                elif ph == "REORIENT":
                    put(t, "RE-ORIENT", "turning the screwdriver: the pointed end away from the hand", "reorient")
                elif ph == "PLACE_APPROACH" and turn and turn[0] <= k_app <= turn[1]:
                    put(t, "RE-ORIENT", f"turning the screwdriver {abs(turn[2]):.0f}° on the way over the palm",
                        "reorient")
                else:
                    put(t, "MOVING", _pretty(ph), "moving")
            elif freeze_on:
                continue
            elif ph == "HOLD":
                put(t, "HOLD", "the arm still, the monitors read every step", "hold")
            elif ph in ("CP1", "CP2", "CP3"):
                put(t, "CHECKPOINT", f"{ph}: the arm still before the judge", "hold")
            elif ph == "IDLE":
                put(t, "IDLE", "torque on, holding", "idle")
        elif k == "freeze":
            freeze_on = True
            put(t, *_freeze_badge(r), "freeze")
        elif k == "relatch":
            freeze_on = True
            put(t, "FREEZE", "latched again: " + str(r.get("why"))[:70], "freeze")
        elif k == "judge":
            v = r.get("verdict") or {}
            lat = float(v.get("latency_s") or 0.0)
            over.append([round(t - lat, 3), round(t, 3), "JUDGE: asking…",
                         f"{v.get('model')} · {v.get('rubric', 'checkpoint')} rubric · the arm still", "judge"])
            over.append([round(t, 3), round(t + JUDGE_SHOW_S, 3), f"JUDGE: {str(v.get('recommend')).upper()}",
                         f"p_unsafe {v.get('p_unsafe')} · {'UNSAFE' if r.get('unsafe') else 'not UNSAFE'} by the "
                         f"code's reading · {lat:.1f} s", "unsafe" if r.get("unsafe") else "judge"])
        elif k == "key":
            if r.get("key") == "w" and r.get("accepted"):
                over.append([round(t, 3), round(t + 1.0, 3), "WAIT — hand in workspace" if hand_now else
                             "WAIT — re-checking", "1 s, then the judge reads the scene again", "wait"])
            chips.append([round(t, 3), _key_text(r), "bad" if r.get("refused") or not r.get("accepted") else "key"])
        elif k == "wait_clear":
            w = float(r.get("waited_s") or 0.0)
            over.append([round(t - w, 3), round(t, 3), "WAIT — hand in workspace",
                         f"{r.get('frames')} fresh frames with no hand before the retreat", "wait"])
            chips.append([round(t, 3), f"workspace clear: {r.get('frames')} fresh frames in {w:.1f} s", "ok"])
        elif k == "settle":
            chips.append([round(t, 3), f"settle {r.get('s')} s, the arm still, before the judge", "ok"])
        elif k == "plan":
            p = r.get("plan") or {}
            if p.get("ok"):
                turn, k_app = _turning(p), 0
                tx = f", turning {abs(turn[2]):.0f}°" if turn else ""
                chips.append([round(t, 3), f"PLAN ({r.get('requested')}): {p.get('n_rows')} rows{tx}", "ok"])
            else:
                chips.append([round(t, 3), f"PLAN REFUSED: {str(p.get('reason'))[:90]}", "bad"])
        elif k == "kill":
            freeze_on = True
            put(t, "KILL", "torque off (OFF, then YES)", "freeze")
        elif k == "session_end":
            put(t, "END", str(r.get("why"))[:80], "idle")
        elif k in ("trial_end", "release_auto", "still_check", "grasp_check", "preauth", "outcome_label", "preauth_held"):
            chips.append([round(t, 3), _chip(r), "bad" if _chip_bad(r) else "ok"])
    motion = [b[0] for b in base if b[3] in MOTION_CLS]
    freezes = [b[0] for b in base if b[1].startswith("FREEZE \u2014")]
    for o in over:
        nxt = next((m for m in motion if m > o[0]), None)       # a verdict leaves the badge when the arm moves
        if nxt is not None and nxt < o[1]:
            o[1] = nxt
        fz = next((f for f in freezes if f <= o[0] < f + FREEZE_SHOW_S), None)
        if fz is not None:                                       # the freeze holds the badge FREEZE_SHOW_S first
            o[0] = round(fz + FREEZE_SHOW_S, 3)
    return base, [o for o in over if o[1] > o[0]], chips


def _chip_bad(r: dict) -> bool:
    k = r["kind"]
    return ((k == "trial_end" and r.get("outcome") not in ("HANDED_OVER", "PLACED")) or (k == "grasp_check" and not r.get("held"))
            or (k == "still_check" and not r.get("ok")) or k == "preauth_held"
            or (k == "outcome_label" and not r.get("received")))


def _chip(r: dict) -> str:
    k = r["kind"]
    if k == "trial_end":
        return f"TRIAL: {r.get('outcome')} — {r.get('why')}"
    if k == "release_auto":
        v = r.get("verdict") or {}
        return f"released without a key: the judge not UNSAFE (p_unsafe {v.get('p_unsafe')})"
    if k == "still_check":
        return f"hand still: {'ok' if r.get('ok') else 'MOVED'} ({r.get('n_sightings')} sightings)"
    if k == "grasp_check":
        return (f"grasp {'held' if r.get('held') else 'DID NOT HOLD'}: jaws {r.get('gripper_pct')} %, load "
                f"{r.get('load')}")
    if k == "preauth":
        return "pre-authorised: " + str(r.get("by"))
    if k == "preauth_held":
        return "pre-authorisation HELD: " + str(r.get("why"))[:80]
    return f"operator: the prop was {'RECEIVED' if r.get('received') else 'NOT received'}"


def log_line(r: dict) -> tuple | None:
    """One short line per row for the presentation's call log -> (cls, text), or None."""
    k = r["kind"]
    if k == "check":
        return ("check" if r.get("ok") else "bad"), f"{'ok ' if r.get('ok') else 'BAD'} {r.get('name')}: {r.get('detail')}"
    if k == "session_start":
        j = r.get("judge") or {}
        return "head", f"SESSION {r['session_id']} · operator {r.get('operator')} · judge {j.get('model') or j.get('provider')}"
    if k == "override":
        return "bad", f"OVERRIDE: {r.get('what')} (bypassed: {', '.join(r.get('bypassed') or [])})"
    if k == "freeze":
        label, sub = _freeze_badge(r)
        return "freeze", f"{label} in {r.get('phase')}: {r.get('detail')} · {sub.split(' · ')[-1]}"
    if k == "judge":
        v = r.get("verdict") or {}
        return ("unsafe" if r.get("unsafe") else "judge"), (
            f"JUDGE at {r.get('phase')} ({v.get('rubric', 'checkpoint')}, {v.get('latency_s')} s): "
            f"{str(v.get('recommend')).upper()}, p_unsafe {v.get('p_unsafe')} — {v.get('reason')}")
    if k == "key":
        return ("bad" if r.get("refused") or not r.get("accepted") else "key"), f"KEY {_key_text(r)}"
    if k == "plan":
        p = r.get("plan") or {}
        if not p.get("ok"):
            return "bad", f"PLAN ({r.get('requested')}) REFUSED: {p.get('reason')}"
        c = p.get("checks") or {}
        extra = (f", pointed end >= {c.get('point_clear_mm'):g} mm (closest {c.get('min_point_palm_3d_mm')} mm)"
                 if c.get("point_clear_mm") is not None else "")
        return "plan", (f"PLAN ({r.get('requested')}): {p.get('n_rows')} rows to the palm {r.get('approved_palm_mm')}"
                        f"{extra} · {r.get('plan_ms')} ms, the arm still")
    if k == "settle":
        return "dim", f"settle {r.get('s')} s before the judge at {r.get('phase')}"
    if k == "relatch":
        return "freeze", f"latched again in {r.get('phase')}: {r.get('why')}"
    if k == "wait_clear":
        return "plan", f"workspace clear: {r.get('frames')} fresh frames with no hand, {r.get('waited_s')} s"
    if k == "kill":
        return "freeze", f"KILL: {r.get('detail')}"
    if k == "session_end":
        d = r.get("loop_dt_ms") or {}
        return "head", f"SESSION END: {r.get('why')} · loop p95 {d.get('p95')} ms (limit {d.get('limit')})"
    if k == "refused":
        return "bad", f"REFUSED ({r.get('reason')}): {r.get('detail')}"
    if k == "retarget":
        return "dim", f"re-target {r.get('status')}"
    if k in ("trial_end", "release_auto", "still_check", "grasp_check", "preauth", "outcome_label", "preauth_held"):
        return ("bad" if _chip_bad(r) else "plan"), _chip(r)
    return None


def _mode(start: dict) -> str:
    if start.get("layer_off"):
        out = "pick \u2192 place only \u00b7 trust layer OFF"
    elif start.get("mode") == "palm_placement":
        pl = start.get("placement") or {}
        out = f"palm placement · pre-authorised · a {pl.get('prop', 'plastic prop')}"
    else:
        out = "standoff handover" + (" · pre-authorised" if start.get("handover_trial") else "")
    return ("DRY RUN, no motors · " if start.get("dry_run") else "") + out


def present_data(rows: list, start: dict, box: list, quads: dict, chain_ok: bool) -> dict:
    tt, off = row_times(rows)
    steps = [r for r in rows if r["kind"] == "step"]
    t0 = first_motion(steps)
    rel = lambda r: round(tt[r["seq"]] - t0, 3)                                     # noqa: E731
    phases = list(dict.fromkeys(r["phase"] for r in steps))
    S = {"t": [], "ph": [], "m": [], "tcp": [], "hand": [], "hp": []}
    for r in steps:
        mon = r.get("monitors") or {}
        S["t"].append(rel(r))
        S["ph"].append(phases.index(r["phase"]))
        S["m"].append("".join(_mon_char(mon.get(str(i)) or {}) for i in range(1, 9)))
        S["tcp"].append(r["tcp_mm"])
        m7 = mon.get("7") or {}
        S["hand"].append(0 if m7.get("off") else int(m7.get("n_hands") or (1 if m7.get("present") else 0)))
        hp = r.get("hands_mm") or []
        S["hp"].append(hp[0] if hp else None)
    events, palms, log = [], [], []
    pending_takeover, n_on, block = False, 8, None
    for r in rows:
        k, t = r["kind"], rel(r)
        if k == "step":
            if block != r["phase"] and r["phase"] not in ("ASK", "IDLE"):
                block = r["phase"]
                n_on = sum(1 for i in range(1, 9) if not (r["monitors"].get(str(i)) or {}).get("off"))
                log.append([t, "step", f"▶ {_pretty(block)} — " + (
                    "8 monitors read every step (15 Hz)" if n_on == 8 else
                    f"monitors 1–{n_on} read every step (15 Hz); 7 and 8 OFF")])
            if pending_takeover and r["phase"] in ("HANDOVER_APPROACH", "REORIENT", "PLACE_APPROACH"):
                events.append({"kind": "takeover", "t": t, "tcp": r["tcp_mm"]})
                pending_takeover = False
            continue
        line = log_line(r)
        if line:
            log.append([t, *line])
        if k == "freeze":
            hands = r.get("hands") or []
            palm = hands[0].get("palm_mm") if hands else None
            events.append({"kind": "freeze", "t": t, "monitor": r.get("monitor"), "code": r.get("code"),
                           "tcp": r.get("tcp_mm"), "palm": palm, "held_ms": r.get("t_hold_minus_frame_t_ms")})
            if palm:
                palms.append([t, palm[0], palm[1]])
        elif k == "judge":
            v = r.get("verdict") or {}
            events.append({"kind": "judge", "t": t, "lat": v.get("latency_s"), "rec": v.get("recommend"),
                           "p": v.get("p_unsafe"), "unsafe": bool(r.get("unsafe")), "phase": r.get("phase"),
                           "rubric": v.get("rubric", "checkpoint")})
        elif k == "key" and r.get("menu") and len(str(r["menu"][0])) == 1:
            events.append({"kind": "key", "t": t, "key": str(r.get("key"))[:1], "auto": bool(r.get("by")),
                           "accepted": bool(r.get("accepted")), "refused": bool(r.get("refused"))})
        elif k == "plan":
            p = r.get("plan") or {}
            if p.get("ok"):
                pending_takeover = True
            if r.get("approved_palm_mm"):
                palms.append([t, *r["approved_palm_mm"]])
        elif k in ("check", "still_check") and r.get("palm_mm"):
            palms.append([t, *r["palm_mm"][:2]])
    base, over, chips = badge_tracks(rows, tt, t0)
    t_end = max([S["t"][-1]] + [rel(r) for r in rows if r["kind"] != "step"])
    stamps = [s for s in (start.get("operator_override"), start.get("layer_off")) if s]
    tz = (datetime.datetime.fromisoformat(start["t_iso"]).utcoffset() or datetime.timedelta()).total_seconds() / 60
    return {"session": start["session_id"], "mode": _mode(start), "stamps": stamps, "layer_off": start.get("layer_off"),
            "dry_run": bool(start.get("dry_run")), "chain_ok": bool(chain_ok), "t_end": round(t_end, 3),
            "wall0": round(t0 + off, 3), "tz_min": tz, "cuts": cuts([rel(r) for r in rows], t_end), "cut_show_s": CUT_SHOW_S,
            "phases": phases, "steps": S, "events": events, "palms": palms, "track": base, "over": over, "chips": chips,
            "log": log, "box": box, "quads": quads,
            "judge": "OFF" if start.get("layer_off") else ((start.get("judge") or {}).get("model")
                                                          or (start.get("judge") or {}).get("provider"))}


def write(session_dir: Path, data: dict) -> Path:
    html = TEMPLATE.read_text(encoding="utf-8").replace("__SESSION__", data["session"]).replace(
        "__DATA__", json.dumps(data, separators=(",", ":")).replace("</", "<\\/"))
    out = Path(session_dir) / "present.html"
    out.write_text(html, encoding="utf-8")
    return out
