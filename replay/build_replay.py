#!/usr/bin/env python
"""One session's chain + photos -> one self-contained replay.html (a DEMONSTRATION's replay).

    python replay/build_replay.py sessions/_dryrun/KH-DRY-<stamp>          -> <that directory>/replay.html
    python replay/build_replay.py --latest                                   (the newest session with a freeze)

Everything on the page comes from session.jsonl and the photos it names: the chain is verified first with
anchor/g3.py's verify_chain and the result is printed on the page; every photo's sha256 is checked against its row.
No network: CSS, JS, data and images are inline. Read-only on the session (it writes replay.html beside it).
The page: a timeline with lanes (phase, monitors 1-8, hand-link age, judge calls, keys), the frame at each freeze
with the place zone, the spawn box, the workspace box and the palm (and landmarks when the monitor sent them), the
fingertip path in 3D from the step rows' FK with the workspace box, a red marker at each freeze and a green one where
a handover plan took over, and the call log in chain order. present.html beside it: the same session as a 720x1080
panel for the video, played at real time (replay/present.py); replay.html?present=1 opens it.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
from common import paths  # noqa: E402
from common.table_map import TableMap  # noqa: E402
from replay import present  # noqa: E402

paths.use_anchor()
import g3  # noqa: E402

TEMPLATE = HERE / "template.html"
BUILD_VERSION = "0.1.0"


def _tm(start: dict) -> TableMap:
    t = start["table_map"]
    return TableMap(**{k: (tuple(tuple(p) if isinstance(p, list) else p for p in v) if isinstance(v, list) else v)
                       for k, v in t.items()})


def _px_at_height(tm: TableMap, p_mm, h_mm: float) -> list:
    """The pixel where a point h_mm above the table is seen (the map's own parallax rule, inverted)."""
    c = tm.px_to_mm(tm.nadir_px)
    seen = [c[i] + (float(p_mm[i]) - c[i]) * tm.cam_height_mm / max(1.0, tm.cam_height_mm - h_mm) for i in range(2)]
    return [round(v, 1) for v in tm.mm_to_px(seen)]


def _log_line(r: dict) -> str | None:
    k = r["kind"]
    if k == "check":
        return f"[{'ok' if r['ok'] else 'BAD'}] {r['name']}: {r['detail']}"
    if k == "session_start":
        return (f"{r['label']} | operator {r['operator']} | judge {r['judge'].get('provider')} / {r['judge'].get('model')} | "
                f"hand feed {r['hand_feed']} | {r['grasp_segment']['label']}")
    if k == "freeze":
        return (f"FREEZE in {r['phase']}: monitor {r['monitor']} ({r['code']}) - {r['detail']} | goals := present "
                f"{'done' if r['held'] else 'FAILED'} ({r.get('hold_attempts', 1)} attempt(s)) | frame "
                f"{str(r['frame_sha256'])[:16]} (seq {r['frame_seq']}) | "
                f"hand message seq {r['hand_seq']}, link age {r['hb_age_ms'] and round(r['hb_age_ms'])} ms | frame -> hold "
                f"{r['t_hold_minus_frame_t_ms']} ms")
    if k == "judge":
        v = r["verdict"]
        return (f"judge at {r['phase']} ({v.get('model')}, {v.get('latency_s')} s): recommends {v.get('recommend')}, p_unsafe "
                f"{v.get('p_unsafe')} -> {'UNSAFE' if r['unsafe'] else 'not unsafe'} by the code's reading - {v.get('reason')} | "
                f"rubric {v.get('rubric', 'checkpoint')} {str(v.get('rubric_sha256'))[:12]} frame "
                f"{str(v.get('frame_sha256'))[:12]}"
                + (" | FLAG: judge and code disagree on the pointed end" if r.get("blade_disagreement_flagged") else ""))
    if k == "key":
        on = r.get("verdict_on_screen") or {}
        return (f"key [{r['key']}] in {r['state']} ({'accepted' if r['accepted'] else 'not a menu key: asked again'}); menu "
                f"{' '.join(r['menu'])}; on screen: " + (f"judge recommends {on.get('recommend')}, p_unsafe {on.get('p_unsafe')}"
                                                         if on else "no verdict")
                + (f" | REFUSED: {r['refused']}" if r.get("refused") else "")
                + (" | stop flag cleared at this key" if r.get("stop_flag_cleared_at_key") and not r.get("refused") else "")
                + (f" | UNSAFE on screen: judged again -> p_unsafe {(r.get('fresh_verdict') or {}).get('p_unsafe')}"
                   if r.get("fresh_verdict") else "")
                + (f" | NOT TYPED: {r['by']}" if r.get("by") else ""))
    if k == "preauth":
        return f"handover PRE-AUTHORISED for {r['trial_id']} by {r['by']}: {r['detail']}"
    if k == "preauth_held":
        return f"pre-authorised handover HELD at {r['where']}: {r['why']} - the menu decides"
    if k == "settle":
        return f"settle {r['s']} s before the judge at {r['phase']} ({r.get('how')})"
    if k == "release_auto":
        on = r.get("verdict") or {}
        return (f"RELEASED WITHOUT A KEY (palm placement, pre-authorised): the CP3 verdict p_unsafe {on.get('p_unsafe')}, "
                f"recommends {on.get('recommend')} - {on.get('reason')}")
    if k == "outcome_label":
        on = r.get("judge_at_release") or {}
        return (f"OUTCOME LABEL: the operator says the prop was {'RECEIVED' if r['received'] else 'NOT received'}; "
                f"beside the judge at the release: p_unsafe {on.get('p_unsafe')}, recommends {on.get('recommend')}; "
                f"released by {r.get('released_by')}")
    if k == "wait_clear":
        return (f"after the release: {r['frames']} fresh frames with no hand before the retreat - {r['frames_seen']} frames "
                f"read, the count broken {r['resets']} time(s), {r['waited_s']} s")
    if k == "relatch":
        return (f"latched again in {r['phase']} without motion: {r['why']} | stop flag set {r['stop_flag_set']} | goals := "
                f"present {'done' if r['held'] else 'FAILED'} ({r['hold_attempts']} attempt(s))")
    if k == "plan":
        p = r["plan"]
        if p.get("ok"):
            return (f"plan ({r['requested']}): {p.get('n_rows')} rows at <= 3 deg/s to the palm {r.get('approved_palm_mm')} mm; "
                    f"handle tip {p.get('handle_tip_mm')} mm, heading {p.get('heading_deg')} deg; plan {str(p.get('plan_sha256'))[:12]}"
                    f"; screwdriver (prop tool) dimensions {'PLACEHOLDER' if p.get('knife_placeholder') else 'measured'}; computed in "
                    f"{r.get('plan_ms')} ms, arm stationary")
        return f"plan ({r['requested']}) REFUSED{' by code' if r.get('refused_by') else ' by the planner'}: {p.get('reason')}"
    if k == "retarget":
        p, st = r.get("plan") or {}, r.get("status")
        nums = (f"; palm now vs the plan's palm {r.get('palm_vs_plan_mm')} mm (<= 5), first row vs the goal sent "
                f"{r.get('first_row_vs_goal_deg')} deg (<= 1)")
        job = f"job {r.get('job_id')} for swap row {r.get('swap_k')} ({r.get('worker')} worker)"
        if st == "asked":
            return (f"re-target asked: the palm is {r.get('drift_from_plan_mm')} mm from the plan's palm; {job}; the "
                    f"current plan goes on meanwhile")
        if st == "swapped":
            return (f"re-target swapped in at its swap row: palm {r.get('drift_from_approved_mm')} mm from the approved "
                    f"palm (< 50), plan {str(p.get('plan_sha256'))[:12]} of {p.get('n_rows')} rows, {r.get('plan_ms')} ms "
                    f"in the worker; {job}" + nums)
        if st == "pending":
            return f"RETARGET PENDING: {job} was not ready at its swap row; the current plan went on; it is never used"
        if st == "late":
            return f"re-target LATE: {job} arrived at row {r.get('arrived_at_k')}, after its swap row; never used"
        if st == "abandoned":
            return f"re-target ABANDONED: the approach ended while {job} was out; its result is never used"
        if st == "discarded":
            return f"re-target DISCARDED, not sent: {r.get('reason')}" + nums
        if st == "not_swapped":
            return f"re-target NOT SENT, the arm froze: {r.get('reason')}" + nums
        return f"re-target REFUSED, the arm froze: {r.get('reason') or p.get('reason')}"
    if k == "kill":
        return f"kill row (G3.4 format): cause {r['cause']} - {r['detail']}"
    if k == "trial_end":
        return f"TRIAL {r['trial_id']}: {r['outcome']} - {r['why']} ({r['freezes']} freeze(s), {r['judge_calls']} judge call(s))"
    if k == "session_end":
        d = r["loop_dt_ms"]
        return (f"session end: {r['why']} | loop time p50 {d['p50']} p95 {d['p95']} max {d['max']} ms (limit {d['limit']}) | "
                f"frame -> hold {r['frame_to_hold_ms_monitor7']} ms")
    if k == "refused":
        return f"REFUSED ({r['reason']}): {r['detail']}"
    return None


def _mode_text(start: dict) -> str:
    """The handover mode, and for palm placement its numbers - printed on every page (the operator, 2026-10-03)."""
    if start.get("layer_off"):
        return f"none: {start['layer_off']} - pick -> place only, a prop hand in the place zone"
    pl = start.get("placement") or {}
    if start.get("mode") != "palm_placement":
        return ("standoff handover" + (", pre-authorised (--handover-trial)" if start.get("handover_trial") else
                                       ", every [h] typed"))
    return (f"palm placement, pre-authorised (--palm-placement); the prop: a {pl.get('prop')}; pointed-end clearance "
            f"{pl.get('point_clear_mm_3d')} mm (3D) on every row; release {pl.get('release_above_palm_mm')} mm above the "
            f"palm (+{pl.get('sag_allowance_mm')} mm commanded for the sag); <= {pl.get('far_dps')} deg/s beyond "
            f"{pl.get('near_mm')} mm of the palm, <= {pl.get('near_dps')} within; the hand moving "
            f"{pl.get('drift_max_mm')} mm freezes; {pl.get('settle_s')} s settle before each judge call")


def build(session_dir: Path) -> Path:
    d = Path(session_dir)
    for root in (paths.ANCHOR, paths.ROOT / "phase0", paths.SIM):          # replay.html is written beside the chain
        if root.resolve() == d.resolve() or root.resolve() in d.resolve().parents:
            raise SystemExit(f"{d} is under {root.name}/, which is frozen: no replay is written there")
    chain_p = d / "session.jsonl"
    ok, why = g3.verify_chain(chain_p)
    rows = [json.loads(x) for x in chain_p.read_text(encoding="utf-8").splitlines() if x.strip()]
    start = next((r for r in rows if r["kind"] == "session_start"), None)
    if start is None:
        raise SystemExit(f"{d.name}: no session_start row (a refused session has nothing to replay)")
    tm = _tm(start)
    table_z = float(start["guard"]["z_mm"][0]) + g3.Z_BELOW_TABLE_MM
    steps = {"t": [], "phase": [], "mon": [], "hb": [], "tcp": []}
    events, log, frames, photo_bad = [], [], [], []
    t_last, pending_takeover, block = None, False, None

    def close_block():
        if block:
            fired = sorted({n for n in block["fired"]})
            read = ("the eight monitors read on each" if not block["off"] else
                    f"monitors {', '.join(str(n) for n in range(1, 9) if n not in block['off'])} read on each "
                    f"({', '.join(str(n) for n in block['off'])} OFF: trust layer off)")
            log.append([block["seq"], block["t"], "steps", f"{block['phase']}: {block['n']} control steps, {read}; "
                        f"fired: {', '.join('monitor ' + str(n) for n in fired) or 'none'}"])

    for r in rows:
        k = r["kind"]
        if k == "step":
            t_last = r["t_s"]
            m = r["monitors"]
            steps["t"].append(r["t_s"])
            steps["phase"].append(r["phase"])
            steps["mon"].append([3 if m[str(i)].get("off") else 1 if m[str(i)]["fired"] else
                                 (2 if i == 7 and m["7"].get("present") else 0) for i in range(1, 9)])
            steps["hb"].append(r["hb_age_ms"])
            steps["tcp"].append(r["tcp_mm"])
            if pending_takeover and r["phase"] in ("HANDOVER_APPROACH", "REORIENT"):
                events.append({"kind": "takeover", "t": r["t_s"], "tcp": r["tcp_mm"]})
                pending_takeover = False
            if block is None or block["phase"] != r["phase"]:
                close_block()
                block = {"phase": r["phase"], "seq": r["seq"], "t": r["t_s"], "n": 0, "fired": [],
                         "off": [i for i in range(1, 9) if m[str(i)].get("off")]}
            block["n"] += 1
            block["fired"] += [i for i in range(1, 9) if m[str(i)]["fired"]]
            continue
        close_block()
        block = None
        line = _log_line(r)
        if line:
            log.append([r["seq"], t_last, k, line])
        if k == "freeze":
            palm = (r.get("hands") or [{}])[0].get("palm_mm") if r.get("hands") else None
            events.append({"kind": "freeze", "t": t_last, "monitor": r["monitor"], "code": r["code"], "detail": r["detail"],
                           "frame_sha256": r["frame_sha256"], "tcp": r["tcp_mm"], "palm": palm, "phase": r["phase"]})
            if r.get("photo") and (d / r["photo"]).is_file():
                jpg = (d / r["photo"]).read_bytes()
                sha = hashlib.sha256(jpg).hexdigest()
                if sha != r["photo_sha256"]:
                    photo_bad.append(r["photo"])
                n = len(frames) + 1
                frames.append({"label": f"freeze {n} · monitor {r['monitor']}", "sha256": sha, "hands": r.get("hands") or [],
                               "jpg": "data:image/jpeg;base64," + base64.b64encode(jpg).decode("ascii"),
                               "tcp_px": _px_at_height(tm, r["tcp_mm"], float(r["tcp_mm"][2]) - table_z),
                               "caption": f"Freeze {n} at {t_last:.2f} s in {r['phase']}: monitor {r['monitor']} ({r['code']}) - "
                                          f"{r['detail']}",
                               "note": ("This is the frame the firing hand message was computed on (same sha256)."
                                        if r.get("photo_is_the_message_frame") and sha == r.get("frame_sha256") else
                                        "The latest frame at the freeze (the firing message named no kept frame).")
                                       + (" A synthetic dry-run frame: the simulated feed says where the hand is and "
                                          "the synthetic camera draws it one frame later, so the frame the firing "
                                          "message names shows no hand yet; the red ring is the palm the message reported."
                                          if start["dry_run"] else "")})
        elif k == "judge":
            v = r["verdict"]
            events.append({"kind": "judge", "t": t_last, "phase": r["phase"], "unsafe": r["unsafe"],
                           **{x: v.get(x) for x in ("model", "latency_s", "recommend", "p_unsafe", "reason", "rubric_sha256",
                                                    "frame_sha256")}})
        elif k == "key":
            on = r.get("verdict_on_screen") or {}
            events.append({"kind": "key", "t": t_last, "key": r["key"][:1] if len(r["menu"][0]) == 1 else "!",
                           "state": r["state"], "menu": r["menu"], "accepted": r["accepted"], "refused": r.get("refused"),
                           "by": r.get("by"),
                           "on_screen": f"judge recommends {on.get('recommend')}, p_unsafe {on.get('p_unsafe')}" if on else None})
        elif k == "plan" and r["plan"].get("ok"):
            pending_takeover = True
    close_block()
    if not steps["t"]:
        raise SystemExit(f"{d.name}: no control steps in the chain")
    end = next((r for r in rows if r["kind"] == "session_end"), {})
    box = start["guard"]["box_xy_mm"]
    knife = (start.get("planner") or {}).get("knife") or {}
    dt = end.get("loop_dt_ms") or {}
    outcomes = ", ".join(o["outcome"] for o in end.get("outcomes") or []) or "none"
    facts = [
        ["Session", start["session_id"]], ["What this is", start["label"]],
        ["Mode", "DRY RUN: fake bus, synthetic frames, simulated hand feed - no motor was powered" if start["dry_run"]
         else "the rig"],
        ["Chain", f"{'verifies' if ok else 'BROKEN: ' + why} - {len(rows)} rows, anchor/g3.py verify_chain; file sha256 "
                  f"{g3.sha_file(chain_p)[:16]}", "ok" if ok else "bad"],
        ["Photos", "every photo matches the sha256 in its row" if not photo_bad else f"MISMATCH: {photo_bad}",
         "ok" if not photo_bad else "bad"],
        ["Outcome", outcomes], ["Operator", str(start["operator"])],
        ["Judge", "OFF (--layer-off): no model was asked" if start.get("layer_off") else
                  f"{start['judge'].get('provider')} / {start['judge'].get('model')}"
                  + (" - the stub provider: no model was asked" if start["judge"].get("provider") == "stub" else "")
                  + f"; rubric {str(start['judge'].get('rubric_sha256'))[:12]}; recommends only"],
        ["Hand feed", start["hand_feed"]],
        ["Planner", "OFF (--layer-off): pick -> place only" if start.get("layer_off") else
                    f"{(start.get('planner') or {}).get('planner_version')}; standoff {(start.get('planner') or {}).get('standoff_mm')} "
                    f"mm from the handle tip; z_handover {(start.get('planner') or {}).get('z_handover_mm')} mm "
                    f"({'set' if (start.get('planner') or {}).get('z_handover_set') else 'default, dry runs only'})"],
        ["Screwdriver (prop tool)", ("PLACEHOLDER dimensions" if knife.get("placeholder") else "measured") + f": free handle "
                  f"{knife.get('handle_len_mm')} mm, pointed end {knife.get('blade_overhang_mm')} mm, grip "
                  f"{knife.get('handle_width_mm')} mm", "bad" if knife.get("placeholder") else ""],
        *([["OVERRIDE", f"{start['operator_override']}: this session ran on code whose fixture gate was not green "
                         "for it - every step row carries the stamp", "bad"]] if start.get("operator_override") else []),
        *([["TRUST LAYER", f"{start['layer_off']}: the hand monitor (monitors 7 and 8), the judge and the handover "
                           "planner were off; monitors 1-6 live; pick -> place only; a prop hand in the place zone - "
                           "every row carries the stamp", "bad"]] if start.get("layer_off") else []),
        ["Handover mode", _mode_text(start)],
        ["Scripted segment", start["grasp_segment"]["label"]],
        ["Freezes", str(sum(1 for e in events if e["kind"] == "freeze"))],
        ["Loop time", f"p50 {dt.get('p50')} / p95 {dt.get('p95')} / max {dt.get('max')} ms (limit {dt.get('limit')})"],
        ["Frame to hold (monitor 7)", f"{end.get('frame_to_hold_ms_monitor7')} ms"],
        ["Registration", str(start["frozen"].get(paths.REG_REL))[:16]],
        ["Runner", f"{start['runner']['version']}; replay builder {BUILD_VERSION}"],
    ]
    limits = ["One overhead camera cannot see a hand's height: palm positions assume a hand resting on the table (palm 20 mm up).",
              "The workspace box's near edge (x below 165-201 mm, depending on y) is outside the camera frame: a hand there is not "
              "seen. For this demonstration the allowed motion region is the box cut to the camera's footprint (the image less 10 px); "
              "the planner refuses targets outside it and the runner checks every scripted row against it. The blind strip itself "
              "is not watched.",
              "No base guard was run for this demonstration."]
    if start["dry_run"]:
        limits.insert(0, "A rehearsal: nothing on this page was seen on the rig. Frames are synthetic, the hand feed is a script, "
                         "the judge is the stub provider.")
    if knife.get("placeholder") or start["grasp_segment"].get("placeholder"):
        limits.append("The prop tool's dimensions and the grasp segment are placeholders until they are measured and recorded.")
    q = lambda quad: [[round(v, 1) for v in tm.mm_to_px(p)] for p in quad]          # noqa: E731
    data = {"session": start["session_id"], "dry_run": start["dry_run"], "layer_off": start.get("layer_off"), "steps": steps, "events": events, "frames": frames,
            "log": log, "facts": facts, "limits": limits,
            "box": [box[0], box[1], box[2], box[3], round(table_z, 1), start["guard"]["z_mm"][1]],
            "quads": {"zone": [list(p) for p in tm.zone_quad], "spawn": [list(p) for p in tm.spawn_quad]},
            "overlay": {"zone": q(tm.zone_quad), "spawn": q(tm.spawn_quad),
                        "box": q([(box[0], box[2]), (box[1], box[2]), (box[1], box[3]), (box[0], box[3])])}}
    html = TEMPLATE.read_text(encoding="utf-8").replace("__SESSION__", start["session_id"]).replace(
        "__DATA__", json.dumps(data, separators=(",", ":")).replace("</", "<\\/"))
    out = d / "replay.html"
    out.write_text(html, encoding="utf-8")
    present.write(d, present.present_data(rows, start, data["box"], data["quads"], ok))    # present.html beside it
    return out


def latest(sessions: Path) -> Path | None:
    """The newest session (dry runs included) whose chain has a freeze row."""
    for d in sorted(list(sessions.glob("KH-S*")) + list((sessions / "_dryrun").glob("KH-DRY-*")),
                    key=lambda p: p.stat().st_mtime, reverse=True):
        p = d / "session.jsonl"
        if p.is_file() and '"kind": "freeze"' in p.read_text(encoding="utf-8"):
            return d
    return None


def main(argv: list | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", nargs="?", help="a session directory")
    ap.add_argument("--latest", action="store_true")
    ap.add_argument("--sessions-dir", default=str(paths.SESSIONS))
    a = ap.parse_args(argv)
    d = Path(a.session) if a.session else (latest(Path(a.sessions_dir)) if a.latest else None)
    if d is None:
        print("no session given (a directory, or --latest)")
        return 2
    out = build(d)
    print(f"replay (demonstration) -> {out}  ({out.stat().st_size // 1024} kB, sha256 {g3.sha_file(out)[:16]})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
