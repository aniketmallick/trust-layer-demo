#!/usr/bin/env python
"""Handover fault table - every new monitor seen to fire and seen not to fire, before the arm is powered.
DRY RUN - a demonstration - nothing here has been seen live.

    python fault_table_handover.py                 # the table + four rehearsal sessions -> sessions/_dryrun/
    python fault_table_handover.py --no-sessions   # the table only (about 10 s)

In the form of anchor/g3_fault_injection.py's table: one row per injected fault or control, with what was expected,
what was observed, and whether that was as expected.
  monitor 7 (hand in the workspace)   monitors.monitor7 on built messages: fire and not-fire
  monitor 8 (hand link)               the real common/hand_link.py HandLink on a hand-set clock: fire and not-fire
  judge gate                          judge/fixtures.py's own fixtures, and the runner's reading of a verdict
  runner                              --arm at 23:10 not refused for the hour (no motion window since 2026-10-03), 
                                      refused next with a row; the chain fails on a deleted
                                      row; a token-shaped string never reaches the chain
  monitors 1-6                        NOT re-tested here: the latest phase0/g3 fault table is cited by file and
                                      sha256 (read-only); tonight's segments and one handover plan are replayed
                                      through g3_fault_injection.simulate (lagging servo, encoder noise) at the
                                      scaled speed limits with nothing firing, and one overspeed is seen to fire
  rehearsal sessions                  four dry-run sessions on the fake bus (handover_session.py), each read back
                                      from its own chain: the handover path, the stale heartbeat, the second hand,
                                      no hand
Output: sessions/_dryrun/fault_table_handover.json + .md. handover_session.py --arm reads the json's all_as_expected.
"""
from __future__ import annotations

import argparse
import contextlib
import datetime
import io
import json
import os
import socket
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
from common import paths, wire  # noqa: E402

paths.use_anchor()
import g3  # noqa: E402

import grasp_segment  # noqa: E402
import handover_flow  # noqa: E402
import monitors as m  # noqa: E402
import state_machine as sm  # noqa: E402
from steplog import StepLog  # noqa: E402

TABLE_VERSION = "0.1.0"
LABEL = "DRY RUN - demonstration - nothing seen live"
OWN = ("monitors.py", "motion.py", "state_machine.py", "handover_flow.py", "handover_session.py", "steplog.py",
       "fault_table_handover.py", "common/hand_link.py", "common/wire.py", "monitor/sim_feed.py")


def _hand(x=300.0, y=90.0, inside=True) -> dict:
    return {"handedness": "R", "conf": 0.9, "palm_px": [1.0, 2.0], "palm_mm": [x, y], "in_workspace": inside,
            "in_spawn": False, "in_zone": False}


def _row(rows, monitor, fault, expected, observed, ok, kind="fire") -> None:
    rows.append({"monitor": monitor, "fault": fault, "expected": expected, "observed": observed, "ok": bool(ok),
                 "kind": kind, "live": "no"})


# ------------------------------------------------------------------------------------------ monitor 7
def monitor7_rows(rows: list) -> None:
    M = "7 hand_in_workspace"
    cases = [
        ("none - no hand in any of the last 5 frames, arm moving", {"hands": []}, [False] * 5, "APPROACH_ZONE", None, False),
        ("none - a hand in frame but outside the workspace box, arm moving", {"hands": [_hand(inside=False)]},
         [False] * 5, "APPROACH_ZONE", None, False),
        ("a hand in the workspace in ONE raw frame, arm moving", {"hands": [_hand()]}, [False] * 4 + [True],
         "APPROACH_ZONE", None, True),
        ("a hand in 3 of the last 5 frames, the latest frame empty (a dropout)", {"hands": []},
         [True, True, True, False, False], "LIFT", None, True),
        ("none - the approved palm 30 mm from where it was approved, during the approach", {"hands": [_hand(300, 120)]},
         [True] * 5, "HANDOVER_APPROACH", [300, 90], False),
        ("the approved palm 50 mm from where it was approved, during the approach", {"hands": [_hand(300, 140)]},
         [True] * 5, "HANDOVER_APPROACH", [300, 90], True),
        ("a second hand during the approach", {"hands": [_hand(300, 90), _hand(250, -40)]}, [True] * 5,
         "HANDOVER_APPROACH", [300, 90], True),
        ("the approved hand gone during the approach (1 of 5)", {"hands": []}, [True, False, False, False, False],
         "HANDOVER_APPROACH", [300, 90], True),
        ("none - a hand at the handle while the arm HOLDS (stationary)", {"hands": [_hand()]}, [True] * 5, "HOLD", None, False),
        ("none - ONE hand at the handle while the gripper opens (RELEASE)", {"hands": [_hand(300, 150)]}, [True] * 5,
         "RELEASE", None, False),
        ("a second hand while the gripper opens (RELEASE)", {"hands": [_hand(300, 150), _hand(250, -40)]}, [True] * 5,
         "RELEASE", None, True),
        ("a palm position that is not a finite number, during the approach",
         {"hands": [{**_hand(), "palm_mm": [float("nan"), 90.0]}]}, [True] * 5, "HANDOVER_APPROACH", [300, 90], True),
    ]
    for fault, msg, window, phase, approved, want in cases:
        r = m.monitor7(msg, window, phase, approved)
        _row(rows, M, fault, "fires" if want else "no fire", ("fired: " if r["fired"] else "no fire: ") + r["detail"],
             r["fired"] == want, "fire" if want else "control")
    r = m.monitor7({"hands": []}, [True, False, False, False, False], "CP1")
    why = sm.may_start_moving(r["clear"], False)
    _row(rows, M, "a hand seen 4 frames ago, none since: may a stationary state start moving?",
         "refused until 0 of 5", f"clear={r['clear']}; start refused: {why}", (not r["clear"]) and why is not None)
    r = m.monitor7({"hands": []}, [False] * 5, "CP1")
    _row(rows, M, "none - 0 of 5: a stationary state may start moving", "allowed",
         f"clear={r['clear']}; refusal: {sm.may_start_moving(r['clear'], False)}",
         r["clear"] and sm.may_start_moving(r["clear"], False) is None, "control")


# ------------------------------------------------------------------------------------------ monitor 8
class _Clock:
    def __init__(self):
        self.t = 1000.0

    def now(self) -> float:
        return self.t


def _link(known_frame=None, on_stale=None):
    from common.hand_link import HandLink
    mono, wall = _Clock(), _Clock()
    link = HandLink(on_stale=on_stale, known_frame=known_frame, clock=mono.now, wall=wall.now)
    return link, mono, wall


def _msg(seq: int, frame_t: float, frame_seq: int = 1, sha: str = "ab" * 32) -> bytes:
    return wire.encode(wire.build_hand_msg(frame_t, seq, {"seq": frame_seq, "sha256": sha, "t": frame_t}, [], [False],
                                           "sim_feed", 1.0))


def _advance(mono, wall, dt: float) -> None:
    mono.t += dt
    wall.t += dt


def monitor8_rows(rows: list) -> None:
    M = "8 hand_link"
    link, mono, wall = _link()
    rng, fired, seq = np.random.default_rng(8), 0, 0
    for _ in range(50):                                        # 5 s at 10 Hz, +/-20 ms of jitter
        _advance(mono, wall, 0.1 + float(rng.uniform(-0.02, 0.02)))
        seq += 1
        link.feed(_msg(seq, wall.t - 0.03))
        fired += int(link.monitor8()[0])
    _row(rows, M, "none - 50 messages at 10 Hz with +/-20 ms jitter", "no fire", f"{fired} fire(s) in 50", fired == 0, "control")
    for age, want in ((0.299, False), (0.301, True)):
        link, mono, wall = _link()
        link.feed(_msg(1, wall.t))
        link.feed(_msg(2, wall.t))
        _advance(mono, wall, age)
        f, d = link.monitor8()
        _row(rows, M, f"{'none - ' if not want else ''}the last message is {1000 * age:.0f} ms old",
             "fires" if want else "no fire", ("fired: " if f else "no fire: ") + d, f == want, "fire" if want else "control")
    for step, want in ((2, False), (3, True), (0, True)):
        link, mono, wall = _link()
        link.feed(_msg(10, wall.t))
        link.feed(_msg(10 + step, wall.t))
        f, d = link.monitor8()
        what = {2: "none - seq steps by 2 (one message lost)", 3: "seq steps by 3 (two messages lost)",
                0: "seq repeats (the monitor restarted, or another sender)"}[step]
        _row(rows, M, what, "fires" if want else "no fire", ("fired: " if f else "no fire: ") + d, f == want,
             "fire" if want else "control")
    link, mono, wall = _link()
    link.feed(_msg(1, wall.t))
    link.feed(_msg(2, wall.t - 0.501))
    f, d = link.monitor8()
    _row(rows, M, "a fresh message computed on a frame 501 ms old", "fires", ("fired: " if f else "no fire: ") + d, f)
    frames = {7: {"sha256": "cd" * 32, "t": 1000.0}}
    for name, fseq, sha, want in (("none - the message names a frame the runner wrote", 7, "cd" * 32, False),
                                  ("the message names a frame seq the runner never wrote", 8, "cd" * 32, True),
                                  ("the message's frame hash is not the hash of the frame the runner wrote", 7, "ee" * 32, True)):
        link, mono, wall = _link(known_frame=frames.get)
        link.feed(_msg(1, wall.t, fseq, sha))
        link.feed(_msg(2, wall.t, fseq, sha))
        f, d = link.monitor8()
        _row(rows, M, name, "fires" if want else "no fire", ("fired: " if f else "no fire: ") + d, f == want,
             "fire" if want else "control")
    link, mono, wall = _link()
    link.feed(_msg(1, wall.t))
    link.feed(_msg(2, wall.t))
    for _ in range(4):
        _advance(mono, wall, 0.1)
        link.feed(b'{"not": "a hand message"}')
    f, d = link.monitor8()
    _row(rows, M, "only malformed datagrams for 400 ms (they count as no message)", "fires",
         ("fired: " if f else "no fire: ") + d, f)
    calls: list = []
    link, mono, wall = _link(on_stale=calls.append)
    link.feed(_msg(1, wall.t))
    link.feed(_msg(2, wall.t))
    _advance(mono, wall, 0.35)
    link.tick()
    quiet = len(calls)
    link.arm_watchdog(True)
    link.tick()
    link.tick()
    _row(rows, M, "the watchdog: stale while NOT armed (stationary), then armed (moving), two looks",
         "no call while disarmed; one call while armed", f"{quiet} call(s) disarmed; {len(calls) - quiet} call(s) armed: "
         f"{calls[-1] if calls else None}", quiet == 0 and len(calls) == 1)


# ------------------------------------------------------------------------------------------ judge gate
def judge_rows(rows: list) -> None:
    M = "judge gate"
    try:
        from judge import fixtures as jf
        for r in jf.run_all():
            _row(rows, M, f"{r['name']}: {r['what']}"[:300], "as its fixture states", str(r["detail"])[:300], r["ok"],
                 "control" if r.get("kind") == "control" else "fire")
    except Exception as e:  # noqa: BLE001 - the judge's fixtures not running is a row that is not ok
        _row(rows, M, "judge/fixtures.py run_all()", "runs", f"{type(e).__name__}: {e}", False)
    for v, want, what in (({"p_unsafe": 0.49, "recommend": "proceed"}, False, "none - p_unsafe 0.49"),
                          ({"p_unsafe": 0.5, "recommend": "proceed"}, True, "p_unsafe 0.50"),
                          ({"p_unsafe": 0.2, "recommend": "abort"}, False, "none - recommend abort with p_unsafe 0.2 "
                                                                           "(a recommendation commands nothing)"),
                          ({"p_unsafe": 0.1, "unsafe": True, "reason": "timeout"}, True, "the judge's own unsafe flag (timeout)"),
                          ({"p_unsafe": "high"}, True, "p_unsafe is not a number"), ({}, True, "an empty verdict")):
        got = handover_flow.code_unsafe(v)
        _row(rows, M, f"the runner's reading: {what}", "UNSAFE -> freeze / stay frozen, ASK" if want else "not unsafe",
             f"unsafe={got}", got == want, "fire" if want else "control")


# ------------------------------------------------------------------------------------------ runner
def runner_rows(rows: list) -> None:
    import handover_session
    with tempfile.TemporaryDirectory() as td:
        def boom(*_a, **_k):
            raise AssertionError("nothing may be opened by the fault table")
        res = {}
        for label, now, extra in (("late", datetime.datetime(2026, 10, 1, 23, 10, tzinfo=g3.IST), ["--port", "/dev/none", "--z-handover-mm", "60"]),
                                  ("day", datetime.datetime(2026, 10, 1, 12, 0, tzinfo=g3.IST), ["--z-handover-mm", "60"])):
            sessions = Path(td) / label
            with contextlib.redirect_stdout(io.StringIO()):
                rc = handover_session.main(["--arm", "--sessions-dir", str(sessions), *extra], now_fn=lambda n=now: n,
                                           camera_factory=boom, link_factory=boom, judge=object(), planner=object())
            d = sorted(sessions.glob("KH-S*"))[-1]
            last = json.loads((d / "session.jsonl").read_text().splitlines()[-1])
            res[label] = (rc, last.get("kind"), last.get("reason"), g3.verify_chain(d / "session.jsonl")[0])
        _row(rows, "runner: no motion window", "--arm at 23:10 IST (no --knife-* given, so nothing can open)",
             "not refused for the hour (no motion window: the operator, 2026-10-03); refused next, exit 2, a 'refused' "
             "row (knife_not_measured)",
             f"exit {res['late'][0]}; last row {res['late'][1]} / {res['late'][2]}; chain ok {res['late'][3]}",
             res["late"] == (2, "refused", "knife_not_measured", True))
        _row(rows, "runner: motion hours", "none - --arm at 12:00 IST (no --port given, so nothing can open)",
             "not refused for the hour (refused next, for the missing port)",
             f"exit {res['day'][0]}; last row {res['day'][1]} / {res['day'][2]}", res["day"][2] == "no_port", "control")
        log = StepLog(Path(td) / "c.jsonl")
        for k in range(6):
            log.append({"kind": "step", "k": k})
        fake = "sk-" + "z" * 40
        log.append({"kind": "key", "note": f"token {fake} pasted by mistake"})
        log.close()
        ok_before = g3.verify_chain(log.path)
        lines = log.path.read_text().splitlines()
        (Path(td) / "cut.jsonl").write_text("\n".join(lines[:2] + lines[3:]) + "\n")
        ok_after = g3.verify_chain(Path(td) / "cut.jsonl")
        _row(rows, "runner: chain (steplog.py, verified by anchor/g3.py)", "row 3 of 7 deleted", "verification fails",
             f"intact: {ok_before[0]}; after the deletion: {ok_after[0]} ({ok_after[1]})", ok_before[0] and not ok_after[0])
        leaked = fake in log.path.read_text()
        _row(rows, "runner: secrets scrubber (g3.scrub on every row)", "a key-shaped string in a row",
             "absent from the file", "LEAKED" if leaked else "redacted", not leaked)
        import ask_menu
        from safety import Console
        for typed, want in ((["help", "ok", "hh", "continue?", "l"], "l"), (["h"], "h")):
            chain = StepLog(Path(td) / f"menu_{len(typed)}.jsonl")
            with open(os.devnull, "w") as null:
                key, _ = ask_menu.ask_freeze(ask_menu.Keys(Console(auto=True, out=null), list(typed)), chain, "FT", "CARRY",
                                             "fixture", None, offer_continue=True, holding=True, dry=False)
            acc = [r["accepted"] for r in chain.rows()]
            chain.close()
            _row(rows, "runner: menu key", f"typed, in order: {typed}", f"only the exact letter selects: [{want}]",
                 f"selected [{key}]; rows accepted: {acc}", key == want and acc == [False] * (len(typed) - 1) + [True],
                 "fire" if len(typed) > 1 else "control")
        never = paths.ROOT / "phase0" / "kh_never_written"
        why_frozen, why_ok = handover_session.frozen_path_refusal(never), handover_session.frozen_path_refusal(Path(td) / "fine")
        _row(rows, "runner: output folders", "--sessions-dir under phase0/", "refused before anything is written",
             f"refusal: {why_frozen}; folder exists: {never.exists()}", bool(why_frozen) and not never.exists())
        _row(rows, "runner: output folders", "none - a folder outside the frozen tree", "not refused",
             f"refusal: {why_ok}", why_ok is None, "control")


# ------------------------------------------------------------------------------------------ monitors 1-6
def g3_rows(rows: list, root: Path) -> dict:
    import g3_fault_injection as gfi
    ft = g3._latest_jsonl(root / "phase0" / "g3" / "fault_injection.jsonl") or {}
    p = root / "phase0" / "g3" / str(ft.get("file"))
    sha = g3.sha_file(p) if p.is_file() else None
    cited = {"file": f"phase0/g3/{ft.get('file')}", "sha256": sha, "t_iso": ft.get("t_iso"),
             "all_as_expected": ft.get("all_as_expected"), "sha256_matches_its_log_line": sha == ft.get("sha256")}
    _row(rows, "1-6 (G3: joint margin, speed, workspace box, gripper load, loop time, kill switch)",
         "not re-tested here: the anchor's own fault table is cited (read-only)", "the latest table all as expected",
         f"{cited['file']} sha256 {str(sha)[:16]} t {ft.get('t_iso')} all_as_expected={ft.get('all_as_expected')}",
         ft.get("all_as_expected") is True and sha is not None and sha == ft.get("sha256"), "control")
    reg = json.loads((root / paths.REG_REL).read_text(encoding="utf-8"))
    rec = json.loads((root / paths.RECORD_REL).read_text(encoding="utf-8"))
    guard = g3.guard_from_record(root, reg, rec)
    seg = grasp_segment.load(root, guard.limits()["gripper"][0])
    table = [r for s in seg["segments"] for r in s["rows"]]
    guard.vel_limit_dps = m.VEL_LIMIT_SCRIPTED_DPS
    fired = []
    for i in range(10):
        fired += gfi.simulate(guard, table, np.random.default_rng(100 + i), rest=seg["rest"])["fired"]
    _row(rows, "1-5 on tonight's scripted segments", f"none - 10 nominal replays of the placeholder segment ({len(table)} rows, "
         f"<= 10 deg/s) through the lagging-servo model, speed limit {m.VEL_LIMIT_SCRIPTED_DPS:g} deg/s", "no fire",
         f"{len(fired)} fire(s)" + (f": {fired[0][1]} at row {fired[0][0] + 1}" if fired else ""), not fired, "control")
    plan_ok, n_plan, pfired = False, 0, []
    try:
        import ports
        from common.table_map import table_map_from
        import dry_world
        planner = ports.PlannerPort(root, False, None, None)
        carry = next(s["rows"] for s in seg["segments"] if s["state"] == "APPROACH_ZONE")
        got = dry_world.pick_palm(planner, table_map_from(reg, guard.box), [carry[16], carry[20]])
        plan = planner.plan_handover(carry[16], got[0])
        plan_ok, n_plan = bool(plan.ok), len(plan.targets)
        guard.vel_limit_dps = m.VEL_LIMIT_HANDOVER_DPS
        for i in range(10):
            pfired += gfi.simulate(guard, [list(r) for r in plan.targets], np.random.default_rng(200 + i), rest=carry[16])["fired"]
        observed = f"plan of {n_plan} rows for the palm {list(got[0])}: {len(pfired)} fire(s) in 10 replays"
    except Exception as e:  # noqa: BLE001
        observed = f"could not plan: {type(e).__name__}: {e}"
    _row(rows, "1-5 on a handover plan", f"none - 10 nominal replays of one planned approach (<= 3 deg/s), speed limit "
         f"{m.VEL_LIMIT_HANDOVER_DPS:g} deg/s", "no fire", observed, plan_ok and not pfired, "control")

    def overspeed(k, meas, dt, load):
        if k == 40:
            meas = meas.copy()
            meas[1] += 3.0                                     # 45 deg/s in one step
        return meas, dt, load
    guard.vel_limit_dps = m.VEL_LIMIT_SCRIPTED_DPS
    r = gfi.simulate(guard, table, np.random.default_rng(300), inject=overspeed, rest=seg["rest"])
    codes = [c for _, c, _ in r["fired"]]
    _row(rows, "2 joint_speed at the scaled limit", "shoulder_lift jumps 3 deg in one step (45 deg/s) in a scripted segment",
         f"fires joint_speed (limit {m.VEL_LIMIT_SCRIPTED_DPS:g} deg/s)", f"fired {codes[0]} at row {r['fired'][0][0] + 1}: "
         f"{r['fired'][0][2]}" if codes else "nothing fired", "joint_speed" in codes)
    return cited


# ------------------------------------------------------------------------------------------ rehearsal sessions
SCENARIOS = (
    ("handover", "h,h,r", ["phase:APPROACH_ZONE", "FREEZE:7", "judge:FREEZE", "key:h", "plan:handover:ok",
                           "phase:HANDOVER_APPROACH", "retarget", "FREEZE:7", "judge:FREEZE", "key:h", "plan:handover:ok",
                           "phase:HANDOVER_APPROACH", "phase:HOLD", "judge:CP3", "key:r", "phase:RELEASE", "phase:RETREAT",
                           "end:HANDED_OVER"],
     "a hand enters during the carry; [h]; the palm drifts 30 mm (silent re-target), then 80 mm (freeze, ask); [h]; hold; [r]"),
    ("stale", "l", ["phase:APPROACH_ZONE", "FREEZE:8", "judge:FREEZE", "key:l", "end:ABORTED"],
     "the hand feed goes silent during the carry: monitor 8 freezes the arm; [l]"),
    ("second_hand", "h,l", ["FREEZE:7", "key:h", "plan:handover:ok", "phase:HANDOVER_APPROACH", "FREEZE:7", "judge:FREEZE",
                            "key:l", "end:ABORTED"],
     "a second hand appears during the approved approach: freeze, ask again; [l]"),
    ("none", "", ["phase:GRASP", "phase:LIFT", "judge:CP1", "phase:APPROACH_ZONE", "phase:PRE_PLACE", "judge:CP2",
                  "phase:PLACE", "phase:RETREAT", "end:PLACED"],
     "none - no hand: pick, lift, CP1, carry, CP2, place, retreat; nothing fires"),
)


def _free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def trace(rows: list) -> list:
    out, last = [], None
    for r in rows:
        k = r["kind"]
        if k == "step":
            if r["phase"] != last:
                out.append(f"phase:{r['phase']}")
                last = r["phase"]
            continue
        item = {"freeze": lambda: f"FREEZE:{r['monitor']}", "judge": lambda: f"judge:{r['phase']}",
                "key": lambda: f"key:{r['key']}" if r["accepted"] else None,
                "plan": lambda: f"plan:{r['requested']}:{'ok' if r['plan'].get('ok') else 'none'}",
                "retarget": lambda: "retarget", "kill": lambda: "kill",
                "trial_end": lambda: f"end:{r['outcome']}"}.get(k, lambda: None)()
        if item:
            out.append(item)
        if k in ("freeze", "key", "plan"):
            last = None
    return out


def in_order(tr: list, want: list) -> bool:
    i = 0
    for t in tr:
        if i < len(want) and t == want[i]:
            i += 1
    return i == len(want)


def session_rows(rows: list, sessions: Path) -> list:
    import handover_session
    out = []
    for script, keys, want, what in SCENARIOS:
        argv = ["--dry-run", "--yes", "--no-esc", "--sessions-dir", str(sessions), "--udp-port", "0", "--cue-port",
                str(_free_port()), "--dry-keys", keys, "--sim-script", script]
        with contextlib.redirect_stdout(io.StringIO()):
            rc = handover_session.main(argv)
        d = sorted((sessions / "_dryrun").glob("KH-DRY-*"))[-1]
        chain = [json.loads(x) for x in (d / "session.jsonl").read_text(encoding="utf-8").splitlines() if x.strip()]
        ok_chain, why = g3.verify_chain(d / "session.jsonl")
        tr = trace(chain)
        end = next(r for r in chain if r["kind"] == "session_end")
        fz = [f"monitor {r['monitor']} ({r['code']}) frame {str(r['frame_sha256'])[:12]}" for r in chain if r["kind"] == "freeze"]
        ok = in_order(tr, want) and ok_chain and end["loop_dt_ms"]["max"] < end["loop_dt_ms"]["limit"]
        _row(rows, f"rehearsal session ({script})", what, " -> ".join(want),
             f"{d.name}: exit {rc}; chain {'ok' if ok_chain else 'BROKEN ' + why} ({len(chain)} rows); freezes: {fz or 'none'}; "
             f"loop dt max {end['loop_dt_ms']['max']} ms (limit {end['loop_dt_ms']['limit']}); path "
             f"{'as expected' if in_order(tr, want) else 'NOT as expected: ' + ' -> '.join(tr)}", ok,
             "control" if script == "none" else "fire")
        out.append({"script": script, "session_id": d.name, "dir": str(d.relative_to(sessions.parent)) if sessions.parent in d.parents else str(d),
                    "chain_sha256": g3.sha_file(d / "session.jsonl"), "rows": len(chain), "chain_ok": ok_chain, "exit": rc,
                    "loop_dt_ms": end["loop_dt_ms"], "frame_to_hold_ms_monitor7": end["frame_to_hold_ms_monitor7"],
                    "hb_age_at_freeze_ms": [r["hb_age_ms"] for r in chain if r["kind"] == "freeze" and r["monitor"] == 8]})
    return out


def build(root: Path, sessions: Path, with_sessions: bool = True) -> dict:
    rows: list = []
    monitor7_rows(rows)
    monitor8_rows(rows)
    judge_rows(rows)
    runner_rows(rows)
    cited = g3_rows(rows, root)
    ran = session_rows(rows, sessions) if with_sessions else []
    now = g3.now_iso()
    res = {"what": "handover fault table (screwdriver (prop tool) handover demonstration)", "label": LABEL, "table_version": TABLE_VERSION,
           "t_iso": now, "mode": "dry run (no motors, no camera, simulated hand feed, the judge's stub provider)",
           "seen_live": "nothing - every row is a dry-run row", "g3_fault_table_cited": cited, "sessions": ran,
           "with_sessions": with_sessions, "table": rows, "all_as_expected": all(r["ok"] for r in rows),
           "n_rows": len(rows), "n_fire": sum(1 for r in rows if r["kind"] == "fire"),
           "n_control": sum(1 for r in rows if r["kind"] == "control"),
           "code_sha256": {f: g3.sha_file(HERE / f) for f in OWN if (HERE / f).is_file()}}
    out = sessions / "_dryrun"
    out.mkdir(parents=True, exist_ok=True)
    (out / "fault_table_handover.json").write_text(json.dumps(res, indent=1) + "\n", encoding="utf-8")
    md = [f"# Handover fault table - {LABEL}", "",
          f"{now} · table {TABLE_VERSION} · {res['n_rows']} rows ({res['n_fire']} fire, {res['n_control']} control) · "
          f"G3 table cited: `{cited['file']}` `{str(cited['sha256'])[:12]}`", "",
          "A demonstration. Nothing in this table has been seen on the rig; it is not evidence for any certificate.", "",
          "| Monitor | Fault injected | Expected | Observed | As expected |", "|---|---|---|---|---|"]
    for r in rows:
        md.append("| " + " | ".join(str(r[k]).replace("|", "/").replace("\n", " ") for k in ("monitor", "fault", "expected", "observed"))
                  + f" | {'yes' if r['ok'] else 'NO'} |")
    md += ["", f"All as expected: **{'yes' if res['all_as_expected'] else 'NO'}**.", ""]
    (out / "fault_table_handover.md").write_text("\n".join(md), encoding="utf-8")
    return res


def main(argv: list | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default=str(paths.ROOT))
    ap.add_argument("--sessions-dir", default=str(paths.SESSIONS))
    ap.add_argument("--no-sessions", action="store_true", help="skip the four rehearsal sessions (about 2.5 min)")
    a = ap.parse_args(argv)
    sys.setswitchinterval(0.001)
    import handover_session
    why = handover_session.frozen_path_refusal(a.sessions_dir)
    if why:
        print(f"HANDOVER FAULT TABLE REFUSED: {why}")
        return 2
    res = build(Path(a.root).resolve(), Path(a.sessions_dir), not a.no_sessions)
    for r in res["table"]:
        print(f"  [{'ok ' if r['ok'] else 'BAD'}] {r['monitor'][:28]:28} {r['fault'][:60]:60} -> {r['observed'][:100]}")
    print(f"HANDOVER FAULT TABLE ({LABEL}): " + ("ALL AS EXPECTED" if res["all_as_expected"] else "NOT AS EXPECTED")
          + f" - {res['n_rows']} rows -> {Path(a.sessions_dir) / '_dryrun' / 'fault_table_handover.json'}")
    return 0 if res["all_as_expected"] else 1


if __name__ == "__main__":
    sys.exit(main())
