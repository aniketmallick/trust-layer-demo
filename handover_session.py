#!/usr/bin/env python
"""Screwdriver (prop tool) handover on the SO-101 - session runner. A DEMONSTRATION: nothing it writes is session
evidence for any certificate. Prop changed 2026-10-02: a screwdriver gripped on the handle just behind the shaft.

    python handover_session.py --dry-run --yes [--dry-keys h,h,r] [--sim-script handover]   (rehearsal: FakeFeetechBus,
                                                 synthetic frames, the simulated hand feed, the judge's stub provider)
    python handover_session.py --arm --port PORT --z-handover-mm Z      (the rig; the operator present)

The default is a dry run. --arm is refused in code, each refusal a row and a non-zero exit: outside start_checks.MOTION_HOURS (if set),
without --port, without --z-handover-mm (set at dawn), with a start check not ok (frozen files, framing re-check,
ruler, clamps, kill switch verified on the rig, the G3 fault table, this demo's fault table and fixtures), with a
piece missing, or without the hand monitor's heartbeat. The hours are read again at every trial and at every key that
allows motion. An output folder under anchor/, phase0/ or so101_sim/ is refused before anything is written.

One trial: IDLE -> GRASP -> LIFT -> CP1(judge) -> APPROACH_ZONE -> PRE_PLACE -> CP2(judge) -> PLACE -> RETREAT.
Every control step (15 Hz) reads the eight monitors (monitors.py); any fire freezes the arm at that step - every
goal becomes the present position - writes the freeze row, has the judge read the frozen frame, and asks
(handover_flow.py, ask_menu.py). The runner never drives the arm anywhere by itself after a freeze.
Every step, monitor state, judge call, key and plan is a row of sessions/<id>/session.jsonl, an append-only
SHA-256 chain that anchor/g3.py's verify_chain verifies (steplog.py).
"""
from __future__ import annotations

import argparse
import dataclasses
import datetime
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
from common import paths  # noqa: E402

paths.use_anchor()
import g3  # noqa: E402
from safety import HardAbort, InterlockError  # noqa: E402

import ask_menu  # noqa: E402
import dry_world  # noqa: E402
import grasp_segment  # noqa: E402
import monitors as mon_mod  # noqa: E402
import ports  # noqa: E402
import start_checks  # noqa: E402
import state_machine as sm  # noqa: E402
from common.table_map import table_map_from  # noqa: E402
from frame_pump import DryCamera, FramePump, RealCamera  # noqa: E402
from handover_flow import Flow, Stop  # noqa: E402
from placement_flow import PROP, SETTLE_S, PlacementFlow  # noqa: E402

OVERRIDE = "UNTESTED CODE \u2014 OPERATOR OVERRIDE"   # --operator-override (the operator, 2026-10-03)
LAYER_OFF = "TRUST LAYER OFF \u2014 DEMONSTRATION"      # --layer-off (the operator, 2026-10-04): on every row
LAYER_OFF_LABEL = ("DEMONSTRATION - pick and place with the trust layer OFF (hand monitor, judge and handover planner "
                   "off; a prop hand in the place zone); not session evidence for any certificate")
import motion as mot  # noqa: E402
from motion import Motion, segment_problems  # noqa: E402
from rig import Rig  # noqa: E402
from steplog import StepLog  # noqa: E402

RUNNER_VERSION = "0.1.0"
LABEL = "DEMONSTRATION - a rehearsed screwdriver (prop tool) handover; not session evidence for any certificate"
OWN_FILES = ("handover_session.py", "handover_flow.py", "motion.py", "monitors.py", "state_machine.py", "ask_menu.py",
             "steplog.py", "rig.py", "frame_pump.py", "grasp_segment.py", "start_checks.py", "ports.py")
GRASP_STALL_PCT = 1.5              # after GRASP the jaws on the 21 mm handle stop short of the close target (10.5 % ->
GRASP_LOAD_MIN = 80                #   13.1 %, load 172; 03:18 empty: 10.5 %, load 0 - and the carry went on, unnoticed)
NEAR_REST_MAX_DEG = 15.0           # the arm must be this close to rest (by hand) before torque comes on
CP_SETTLE_S = 0.5
LINK_WAIT_S = 10.0
CUE_PORT = 47102
LOW_STATES = ("GRASP", "LIFT", "PLACE", "RETREAT")   # rows that start or end at the grasp height


END_PROMPT = ("Session done. Torque OFF lets the arm drop under gravity: support it. Type OFF to disable torque (then "
              "YES), or Enter to leave torque ON and close the port: ")


def frozen_path_refusal(path) -> str | None:
    """An output folder that resolves under anchor/, phase0/ or so101_sim/ (symlinks followed) is refused: nothing
    frozen is written. -> the reason, or None."""
    p = Path(path).expanduser().resolve()
    for root in (paths.ANCHOR, paths.ROOT / "phase0", paths.SIM):
        r = root.resolve()
        if p == r or r in p.parents:
            return f"{path} resolves under {r.name}/, which is frozen: nothing is written there"
    return None


class HandoverSession(Flow, PlacementFlow):
    def __init__(self, a, root: Path, judge=None, planner=None, link_factory=None, now_fn=None, camera_factory=None):
        self.a, self.root, self.dry = a, Path(root), not a.arm
        self.sessions = Path(a.sessions_dir) if a.sessions_dir else paths.SESSIONS
        self.now = now_fn or (lambda: datetime.datetime.now(g3.IST))
        self._judge, self._planner, self._link_factory = judge, planner, link_factory
        self._camera_factory = camera_factory or RealCamera
        self.enforce_hours = not self.dry and not a.operator_override   # MOTION_HOURS (if set), every trial and key
        self.layer_off = bool(a.layer_off)
        self.label = LAYER_OFF_LABEL if self.layer_off else LABEL
        self._rt = None
        self.rt_worker, self.retargeters, self._job_n = None, [], 0
        self.reg = json.loads((self.root / paths.REG_REL).read_text(encoding="utf-8"))
        self.record = json.loads((self.root / paths.RECORD_REL).read_text(encoding="utf-8"))
        self.guard = g3.guard_from_record(self.root, self.reg, self.record)
        self.tm = table_map_from(self.reg, self.guard.box)
        from planner.cfg import visible_quad
        self.visible = visible_quad(self.reg, self.guard.box)  # the camera's footprint: motion stays inside it
        self.scripted_dps = scripted_dps(a)                   # ruling 2: 3 deg/s on the arm until the encoders were seen
        self.scripted_vel = mon_mod.scripted_vel_limit(self.scripted_dps)
        self.seg = (grasp_segment.load_file(Path(a.segments)) if a.segments else
                    grasp_segment.load(self.root, self.guard.limits()["gripper"][0], self.scripted_dps))
        if a.segments:                                        # the file's rows set the speed (3 deg/s at dawn)
            self.scripted_dps = float(self.seg["speeds"]["arm_dps"])
            self.scripted_vel = mon_mod.scripted_vel_limit(self.scripted_dps)
        self.state, self.resume_state, self.holding = "IDLE", None, False
        self.trial_id, self.n_judge, self.handover_path = None, 0, []
        self.judge_script, self.cues, self.feed = None, None, None
        self.preauth_used = False                              # --handover-trial: set again at every trial
        self.place_palm = self.release_verdict = self.released_by = None
        self.place_dir = self.place_reach = None
        self.latest_verdict, self.sights, self.read_sights, self.place_descent_rows = None, [], [], []
        self.rig = self.chain = self.motion = self.link = self.pump = self.keys = None
        self.outcomes: list = []

    # -- small helpers ---------------------------------------------------------------------------------
    def say(self, msg: str = "") -> None:
        (self.rig.console.say if self.rig is not None else print)(msg)

    def grasp_held(self, q6) -> tuple:
        """After GRASP, on the arm: the jaws stopped >= GRASP_STALL_PCT short of the close target AND the gripper is
        loaded >= GRASP_LOAD_MIN - the handle is between them. -> (held, why). Dry runs: not read (the fake bus does
        not stall on a handle)."""
        if self.dry:
            return True, "dry run: not read"
        load = abs(int(self.rig.read_load()))
        stall = float(q6[5]) - float(self.seg["close_pct"])
        held = stall >= GRASP_STALL_PCT and load >= GRASP_LOAD_MIN
        why = (f"the jaws closed to {float(q6[5]):.1f} % (target {float(self.seg['close_pct']):g} %; on the handle they "
               f"stop near 13 %) with a gripper load of {load} (holding the handle: ~170)")
        self.chain.append({"kind": "grasp_check", "t_iso": g3.now_iso(), "session_id": self.session_id,
                           "trial_id": self.trial_id, "held": held, "gripper_pct": round(float(q6[5]), 2),
                           "close_pct": float(self.seg["close_pct"]), "stall_pct": round(stall, 2), "load": load,
                           "rule": f"stall >= {GRASP_STALL_PCT:g} % and load >= {GRASP_LOAD_MIN}"})
        return held, why

    def gripper_closed(self, q6) -> bool:
        """The jaws nearer the close target than the release opening: the prop is taken as held ([h] / [o] offered)."""
        return float(q6[5]) < (float(self.seg["close_pct"]) + float(self.seg["release_pct"])) / 2.0

    def log_checks(self, res: dict) -> None:
        for c in res["checks"]:
            self.chain.append({"kind": "check", "t_iso": g3.now_iso(), "session_id": self.session_id, **c})
            self.say(f"  [{'ok ' if c['ok'] else 'BAD'}] {c['name']}: {c['detail']}")

    def next_job_id(self) -> int:
        self._job_n += 1
        return self._job_n

    def start_worker(self) -> None:
        """The re-target worker (retarget_worker.py): its own process with its own planner, built from this session's
        arguments; a planner handed to the session (a test's) is run in a thread instead. Ready before anything moves."""
        import retarget_worker as rw
        a, t0 = self.a, time.monotonic()
        if self._planner is not None:
            self.rt_worker = rw.ThreadWorker(self.planner)
        else:
            delay = float(a.dry_retarget_delay_s or 0.0) if self.dry else 0.0
            self.rt_worker = rw.ProcessWorker((self.root, not self.dry, a.z_handover_mm, a.standoff_mm, knife_dims(a)),
                                              delay_s=delay)
        why = self.rt_worker.wait_ready()
        row = {"kind": "check", "t_iso": g3.now_iso(), "session_id": self.session_id, "name": "retarget_worker",
               "ok": why is None, "worker": self.rt_worker.kind, "test_delay_s": self.rt_worker.delay_s,
               "detail": why or f"{self.rt_worker.kind} worker ready in {time.monotonic() - t0:.1f} s"}
        self.chain.append(row)
        self.say(f"  [{'ok ' if why is None else 'BAD'}] retarget_worker: {row['detail']}")
        if why is not None:
            raise Stop(why)

    def arm_statement(self) -> str:
        """What will move, how far and how fast - from this session's own numbers (the 14:52 statement said <= 10 deg/s
        and 'a placeholder segment' while the dawn file ran at 3 deg/s: never again a fixed text)."""
        s, dps, gps = self.seg, self.scripted_dps, float(self.seg["speeds"].get("gripper_pps", 30.0))
        src = (f"the dawn-recorded segments ({Path(self.a.segments).name if self.a.segments else 'dawn/segments.json'})"
               if not s.get("placeholder") else "a PLACEHOLDER segment")
        head = ((f"*** {LAYER_OFF} ***\n" if self.layer_off else "") +
                (f"*** {OVERRIDE} ***\n" if self.a.operator_override else "") +
                f"Screwdriver (prop tool) {'REHEARSAL' if self.dry else 'DEMONSTRATION'} - the marked prop tool only.\n"
                f"  Torque ON on all 6 joints, holding the pose. Then ALL joints move to the rest pose at <= {dps:g} deg/s.\n")
        if self.layer_off:
            body = self.layer_off_body(src, dps, gps)
        elif self.a.hold_test:
            body = (f"  HOLD TEST from {src}: over the grip mark (jaws opening to {s['open_pct']:g} %), straight down around the\n"
                    f"  handle, the jaws close to {s['close_pct']:g} %, lift 20 mm, hold 10 s; slipped -> one step tighter, once;\n"
                    f"  then set down, open, home. Arm joints <= {dps:g} deg/s, gripper <= {gps:g} %/s. No handover in a hold test.\n")
        elif self.a.palm_placement:
            from planner import placement as pl
            body = (f"  Pick / lift / carry from {src} at <= {dps:g} deg/s (gripper <= {gps:g} %/s, closing to "
                    f"{s['close_pct']:g} %).\n"
                    f"  PALM PLACEMENT, PRE-AUTHORISED for every trial (--palm-placement), the screwdriver a {PROP}: when\n"
                    f"  a hand comes in, the arm freezes, waits {SETTLE_S:g} s, and the judge reads the scene; if not UNSAFE\n"
                    f"  and a plan exists, the arm moves WITHOUT a key: turns the screwdriver if its pointed end faces the\n"
                    f"  hand, lays the handle over the open palm with the pointed end past the fingertips, descends to\n"
                    f"  {pl.RELEASE_MM:g} mm above the palm (+{pl.SAG_MM:g} mm commanded for the sag), waits {SETTLE_S:g} s, and if\n"
                    f"  the judge is not UNSAFE OPENS THE GRIPPER WITHOUT A KEY. Pointed end >= {pl.POINT_CLEAR_MM:g} mm from\n"
                    f"  the palm (3D) on every row. <= {pl.FAR_DPS:g} deg/s beyond {pl.NEAR_MM:g} mm of the palm, <= "
                    f"{pl.NEAR_DPS:g} deg/s within it and\n"
                    f"  on the descent. The hand moving {mon_mod.DRIFT_MAX_BY_PHASE['PLACE_DESCENT']:g} mm, a second hand, "
                    f"no plan or UNSAFE: freeze and ask.\n")
        elif self.a.handover_trial:
            so = float(self.a.standoff_mm or 60.0)
            pt = float(getattr(getattr(self.planner, "cfg", None), "point_min_mm", 150.0))
            body = (f"  Pick / lift / carry / place from {src} at <= {dps:g} deg/s (gripper <= {gps:g} %/s, closing to "
                    f"{s['close_pct']:g} %).\n"
                    f"  HANDOVER PRE-AUTHORISED for every trial (--handover-trial): when a hand comes in, the arm freezes and\n"
                    f"  the judge reads the scene; if its verdict is not UNSAFE and a handle-first plan exists, the approach\n"
                    f"  starts WITHOUT a key, at <= 3 deg/s, stopping with the handle tip {so:g} mm from the palm, the pointed\n"
                    f"  end >= {pt:g} mm from it. Once per trial; an UNSAFE verdict or any refusal holds and asks.\n"
                    f"  [r] still releases; the retreat waits for {mon_mod.CLEAR_FRAMES} fresh frames with no hand.\n")
        else:
            body = (f"  Pick / lift / carry / place from {src} at <= {dps:g} deg/s (gripper <= {gps:g} %/s, closing to "
                    f"{s['close_pct']:g} %).\n  A handover approach runs only after your [h], at <= 3 deg/s.\n")
        if self.layer_off:
            return head + body + ("  Monitors 1-6 live (joint margin, joint speed, workspace box, gripper load, loop time, "
                                  "kill switch):\n  any fire freezes the arm and asks. ESC / Ctrl-C freezes it at any time.")
        tail = ("  Eight monitors live: any fire freezes the arm and asks" +
                (" (the pre-authorised handover above excepted)" if (self.a.handover_trial or self.a.palm_placement)
                 and not self.a.hold_test else "") +
                ". Hands out of the workspace until the menu says otherwise.")
        return head + body + tail

    def layer_off_body(self, src: str, dps: float, gps: float) -> str:
        """--layer-off: what is off, where the place goes down to (FK of the PLACE rows), the prop hand."""
        rows = next(x["rows"] for x in self.seg["segments"] if x["state"] == "PLACE")
        low = min(([float(v) for v in self.guard.ee_mm(r)] for r in rows), key=lambda p: p[2])
        s = self.seg
        return (f"  PICK -> PLACE ONLY from {src} at <= {dps:g} deg/s (gripper <= {gps:g} %/s, closing to "
                f"{s['close_pct']:g} %):\n"
                f"  grasp, lift, carry, lower into the place zone, open, retreat to rest. No checkpoint, no handover.\n"
                f"  TRUST LAYER OFF: the hand monitor (monitors 7 and 8), the judge and the handover planner are NOT\n"
                f"  running. Nothing looks for a hand: the PLACE lowers the fingertip to {low[2] - self.guard.table_z:.0f} mm "
                f"above the table at\n  ({low[0]:.0f}, {low[1]:.0f}) mm in the place zone, whatever lies there.\n"
                f"  A PROP HAND - NOT A REAL HAND - lies in the place zone for this shot: anything there standing higher\n"
                f"  than that is pressed by the screwdriver. Real hands stay OUT of the workspace for the whole run.\n")

    def placement_params(self) -> dict:
        from planner import placement as pl
        return {"prop": PROP, "point_clear_mm_3d": pl.POINT_CLEAR_MM, "release_above_palm_mm": pl.RELEASE_MM,
                "sag_allowance_mm": pl.SAG_MM, "palm_h_mm": pl.PALM_H_MM, "near_mm": pl.NEAR_MM, "near_dps": pl.NEAR_DPS,
                "far_dps": pl.FAR_DPS, "drift_max_mm": mon_mod.DRIFT_MAX_BY_PHASE["PLACE_DESCENT"], "settle_s": SETTLE_S,
                "clear_frames": mon_mod.CLEAR_FRAMES}

    def scripted_in_view(self) -> dict:
        """Every scripted segment through FK before anything opens: joint margins, the box, the camera's footprint,
        heights above the table."""
        low = mot.MIN_TIP_LOW_MM if self.a.segments else mot.MIN_TIP_MM     # the dawn file: grasp / place at 5 mm
        probs = [f"{s['state']}: {p}" for s in self.seg["segments"]
                 for p in segment_problems(self.guard, s["rows"], visible=self.visible, limit=1,
                                           tip_floor_mm=low if s["state"] in LOW_STATES else mot.MIN_TIP_MM)]
        return {"name": "scripted_segments", "ok": not probs, "visible_quad_mm": [list(v) for v in self.visible],
                "detail": ("; ".join(probs[:3]) if probs else f"{len(self.seg['segments'])} segments, every row inside "
                           "the joint margins, the workspace box and the camera's footprint")}

    def refuse(self, reason: str, detail: str) -> int:
        self.chain.append({"kind": "refused", "t_iso": g3.now_iso(), "session_id": self.session_id, "reason": reason,
                           "detail": detail, "arm": not self.dry})
        self.say(f"HANDOVER SESSION REFUSED ({reason}): {detail}\nNothing moved.")
        self.chain.close()
        return 2

    def open_session(self) -> None:
        base = self.sessions / "_dryrun" if self.dry else self.sessions
        stamp = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
        sid, n = (f"KH-DRY-{stamp}" if self.dry else f"KH-S{stamp}"), 1
        while (base / sid).exists():
            n += 1
            sid = sid.split("_")[0] + f"_{n}"
        self.session_id, self.dir = sid, base / sid
        self.dir.mkdir(parents=True)
        self.chain = StepLog(self.dir / "session.jsonl")
        if self.layer_off:
            self.chain.stamps = {"trust_layer": LAYER_OFF}      # every row, the first included

    # -- the session -----------------------------------------------------------------------------------
    def run(self) -> int:
        a = self.a
        self.open_session()
        print(f"Screwdriver (prop tool) handover runner {RUNNER_VERSION} - {self.label}{' - DRY RUN' if self.dry else ''}")
        if self.layer_off:
            print(f"*** {LAYER_OFF}: monitors 7 and 8, the judge and the handover planner are off ***")
        if a.palm_placement and a.hold_test:
            return self.refuse("bad_flags", "--palm-placement with --hold-test: a hold test has no handover")
        if self.layer_off and (a.palm_placement or a.handover_trial or a.hold_test):
            return self.refuse("bad_flags", "--layer-off is pick -> place only: not with --palm-placement, "
                                            "--handover-trial or --hold-test")
        if not self.dry:                                       # the gates that need nothing else, before anything opens
            t = self.now().astimezone(g3.IST)
            if not start_checks.motion_hours_ok(t) and not a.operator_override:
                return self.refuse("motion_hours", f"--arm at {t:%H:%M} IST: {start_checks.hours_text()}")
            if not a.port:
                return self.refuse("no_port", "--arm needs --port")
            if a.z_handover_mm is None:
                return self.refuse("z_handover_not_set", "--arm needs --z-handover-mm (dawn protocol step 3); the 30 mm "
                                                         "default is inside a hand lying on the table")
            missing = [f for f, v in (("--knife-handle-mm", a.knife_handle_mm), ("--knife-blade-mm", a.knife_blade_mm),
                                      ("--knife-width-mm", a.knife_width_mm), ("--knife-axis-sign", a.knife_axis_sign))
                       if v is None]
            if missing:
                return self.refuse("knife_not_measured", "--arm needs the prop's measured dimensions: " + ", ".join(missing)
                                   + " (the planner's knife is a placeholder until then)")
            if not a.segments:
                return self.refuse("segments_not_recorded", "--arm needs --segments dawn/segments.json: the null's "
                                                             "placeholder segment was for the cube and the knife")
        try:
            if self.layer_off:                                 # --layer-off: neither is built
                self.planner, self.judge = ports.Off("handover planner", LAYER_OFF), ports.Off("judge", LAYER_OFF)
            else:
                self.planner = self._planner or ports.PlannerPort(self.root, not self.dry, a.z_handover_mm,
                                                                  a.standoff_mm, knife_dims(a))
                if self._judge is None and self.dry:
                    self.judge_script = dry_world.JudgeScript(dry_world.parse_overrides(a.dry_judge))
                self.judge = self._judge or ports.real_judge(self.dry, self.judge_script)
        except (ports.MissingPiece, ValueError) as e:
            return self.refuse("missing_piece", str(e))
        start = start_checks.pre(self.root, self.sessions, self.reg, self.record, self.dry, self.now())
        start["checks"].append(self.scripted_in_view())
        if a.operator_override and not self.dry:               # the fixture status and the hours: not gating
            for c in start["checks"]:
                if c["name"] in ("fixtures_green", "motion_hours") and not c["ok"]:
                    c.update(ok=True, gating=False, overridden=OVERRIDE, detail=f"{c['detail']} - {OVERRIDE}")
            self.chain.append({"kind": "override", "t_iso": g3.now_iso(), "session_id": self.session_id,
                               "what": OVERRIDE, "bypassed": ["fixtures_green", "motion_hours"]})
        start["ok"] = all(c["ok"] for c in start["checks"])
        self.log_checks(start)
        if not start["ok"]:                                    # before the camera or the port is opened
            return self.refuse("start_checks", "not ok: " + ", ".join(c["name"] for c in start["checks"] if not c["ok"]))
        self.rig = Rig(self.dir, self.root, self.dry, a.port, a.fast, a.yes, a.no_esc)
        self.keys = ask_menu.Keys(self.rig.console, [k for k in (a.dry_keys or "").split(",") if k] if self.dry else None)
        ask = self.keys.console.ask
        camera = DryCamera(self.tm, self.palms_px) if self.dry else self._camera_factory(a.camera)
        grab = None if self.dry else (lambda n: [camera.read() for _ in range(n + 10)][10:])
        seen = start_checks.live(self.root, self.dir, self.dry, ask, self.say, grab)
        self.log_checks(seen)
        start = {"ok": seen["ok"], "checks": start["checks"] + seen["checks"]}
        if not start["ok"]:
            camera.release()
            return self.refuse("start_checks", "not ok: " + ", ".join(c["name"] for c in start["checks"] if not c["ok"]))
        self.pump = FramePump(camera, Path(a.frame_dir))
        try:
            factory = (lambda *_a: ports.NullLink()) if self.layer_off else (self._link_factory or ports.real_link)
            self.link = factory(("127.0.0.1", int(a.udp_port)), lambda why="": self.rig.kill.trigger(f"monitor8:{why}"),
                                self.pump.known_frame)
        except ports.MissingPiece as e:
            camera.release()
            return self.refuse("missing_piece", str(e))
        self.operator = (a.operator or ("dry" if self.dry else "")).strip()
        while not self.operator:
            self.operator = ask("Operator id (initials): ").strip()
        self.motion = Motion(self.rig, self.guard, self.chain, self.link, self.pump, self.session_id, self.say)
        self.motion.stamp = OVERRIDE if a.operator_override and not self.dry else None
        self.motion.monitors_off = (7, 8) if self.layer_off else ()
        self.motion.pre_move = self.hours_refusal                # the hours before every motion call (ruling on 11)
        self.chain.append(self.start_row(start))
        rc, why, torque_off = 0, "complete", False
        try:
            self.power_up()
            if a.hold_test:
                import hold_test
                self.outcomes.append(hold_test.run(self))
            else:
                for n in range(1, int(a.trials) + 1):
                    self.outcomes.append(self.trial(n))
        except Stop as e:
            rc, why, torque_off = 1, f"stopped: {e}", e.torque_off
            if self.trial_id and not any(o["trial_id"] == self.trial_id for o in self.outcomes):
                self.outcomes.append(self.trial_end("ABORTED", str(e)))
            self.say(f"\nSESSION STOPPED: {e}")
        except InterlockError as e:
            rc, why = 2, f"refused: {e}"
            self.chain.append({"kind": "refused", "t_iso": g3.now_iso(), "session_id": self.session_id,
                               "reason": "interlock", "detail": str(e), "arm": not self.dry})
            self.say(f"HANDOVER SESSION REFUSED: {e}")
        except (HardAbort, KeyboardInterrupt) as e:
            rc, why, torque_off = 3, f"hard abort: {e}", None
            self.say(f"\nHARD ABORT: {e} - the port is closed, torque untouched.")
        except Exception as e:  # noqa: BLE001 - anything else: the arm is held, the session ends, torque stays on
            rc, why = 4, f"stopped: internal error: {type(e).__name__}: {e}"
            self.internal_stop(e)
            if self.trial_id and not any(o["trial_id"] == self.trial_id for o in self.outcomes):
                self.outcomes.append(self.trial_end("ABORTED", why))
        finally:
            self.shut_down(why, torque_off)
        return rc

    def internal_stop(self, e: Exception) -> None:
        """An exception nobody expected, somewhere in the control path: the stop flag, goals := present (retried), a
        freeze row. Torque is left on and nothing asks to turn it off: the operator decides with the arm in view."""
        try:
            self.rig.kill.trigger(f"internal:{type(e).__name__}")
        except Exception:  # noqa: BLE001
            pass
        held, attempts = self.rig.hold_now()
        detail = f"{type(e).__name__}: {e}"
        try:
            q6 = (self.motion.last_obs or {}).get("q6") or []
            self.chain.append({"kind": "freeze", "t_iso": g3.now_iso(), "session_id": self.session_id,
                               "trial_id": self.trial_id, "phase": self.state, "monitor": 0, "code": "internal",
                               "detail": detail, "source": self.rig.kill.source,
                               "fired": [{"monitor": 0, "code": "internal", "detail": detail}], "held": held,
                               "hold_attempts": attempts, "q6": [round(float(v), 3) for v in q6],
                               "tcp_mm": [round(float(v), 1) for v in self.guard.ee_mm(q6)] if len(q6) >= 5 else None,
                               "hand_seq": None, "frame_seq": None, "frame_sha256": None, "hands": None, "hb_age_ms": None,
                               "t_hold_minus_frame_t_ms": None, "photo": None, "photo_sha256": None,
                               "photo_is_the_message_frame": False})
        except Exception:  # noqa: BLE001 - the hold matters more than its row
            pass
        self.say(f"\nINTERNAL ERROR: {detail}\n  The arm is "
                 + ("held where it is (goals = present position)." if held else
                    f"NOT CONFIRMED HELD ({attempts} attempts): THE GOALS MAY NOT BE THE PRESENT POSITION.")
                 + " Torque stays ON. The session ends here.")

    def start_row(self, start: dict) -> dict:
        return {"kind": "session_start", "t_iso": g3.now_iso(), "session_id": self.session_id, "label": self.label,
                "operator": self.operator, "dry_run": self.dry,
                "runner": {"version": RUNNER_VERSION, "sha256": {f: g3.sha_file(HERE / f) for f in OWN_FILES}},
                "g3": {"version": g3.G3_VERSION, "sha256": g3.sha_file(paths.ANCHOR / "g3.py")},
                "frozen": {**start_checks.FROZEN, "calibration": self.reg["arm_calibration"]["sha256"]},
                "checks_ok": start["ok"], "guard": self.guard.describe(),
                "scripted_dps": self.scripted_dps,
                "monitors": {"table": mon_mod.CODES, "vel_limit_scripted_dps": self.scripted_vel,
                             "vel_limit_handover_dps": mon_mod.VEL_LIMIT_HANDOVER_DPS, "drift_max_mm": mon_mod.DRIFT_MAX_MM,
                             "rule_7": "fires on one raw in_workspace frame or 3 of 5; clears at 0 of 5",
                             "rule_8": "message older than 300 ms, seq gap > 2, frame stale or unknown"},
                "judge": self.judge.describe() if hasattr(self.judge, "describe") else {"provider": type(self.judge).__name__},
                "planner": self.planner.describe() if hasattr(self.planner, "describe") else {},
                "grasp_segment": {k: self.seg[k] for k in ("label", "source", "close_pct", "close_source", "release_pct",
                                                            "open_pct", "speeds", "placeholder")},
                "hand_feed": ("OFF (--layer-off): no hand monitor" if self.layer_off else
                              f"monitor/sim_feed.py --script {self.a.sim_script}" if self.dry else "monitor/hand_monitor.py"),
                "control_hz": grasp_segment.CONTROL_HZ, "trials": int(self.a.trials),
                "handover_trial": bool(self.a.handover_trial and not self.a.hold_test),
                "mode": "layer_off" if self.layer_off else "palm_placement" if self.a.palm_placement else "standoff",
                "layer_off": LAYER_OFF if self.layer_off else None,
                "monitors_off": list(self.motion.monitors_off),
                "operator_override": OVERRIDE if self.a.operator_override else None,
                "placement": self.placement_params() if self.a.palm_placement else None,
                "base_guard": "not run for this demonstration", "state_machine": {"moving": sm.MOVING, "task": sm.TASK_ORDER},
                "table_map": dataclasses.asdict(self.tm)}       # the replay page draws from the chain alone

    def power_up(self) -> None:
        a, rig, rest = self.a, self.rig, self.seg["rest"]
        rig.open_bus(rest)
        if self.dry:
            for i, j in enumerate(g3.ALL):
                rig.bus.hand_move(j, rest[i] + (2.0 if i < 5 else 0.0))
        stmt = self.arm_statement()
        while True:                                            # torque is off: near rest by hand first (as the null's)
            err = max(abs(v - r) for v, r in zip(rig.read_q()[:5], rest[:5]))
            if err <= NEAR_REST_MAX_DEG:
                break
            self.keys.console.ask(f"  The arm is {err:.0f} deg from rest (limit {NEAR_REST_MAX_DEG:g}). Guide it closer to "
                                  "the rest pose by hand, then press Enter: ")
        rig.kill.clear()
        rig.kill.arm()
        if not rig.interlock.require_arm(stmt):
            raise Stop("not armed")
        rig.safe_torque_on()
        self.pump.start()
        self.link.start()
        if self.layer_off:                                     # no planner, no hand monitor: nothing to wait for
            self.chain.append({"kind": "check", "t_iso": g3.now_iso(), "session_id": self.session_id, "ok": True,
                               "name": "hand_mapping", "gating": False,
                               "detail": f"NOT RUN: {LAYER_OFF} - no hand monitor, no re-target worker"})
            self.say(f"  [ok ] hand_mapping: NOT RUN ({LAYER_OFF})")
            self.setup_to_rest()
            return
        self.start_worker()
        if self.dry:
            self.cues = dry_world.Cues(a.cue_port)
            self.motion.cue = self.cues.send
            palm, ddir = self.sim_palm()
            fdir = tuple(float(v) for v in a.sim_fingers_dir.split(",")) if a.sim_fingers_dir else None
            self.feed = dry_world.spawn_feed(a.sim_script, palm, ddir, self.link.addr[1], a.cue_port, Path(a.frame_dir),
                                              a.sim_event, fdir)
            self.say(f"[dry run] simulated hand feed: script {a.sim_script}, palm {palm} mm, drift direction {ddir}")
        t0 = time.monotonic()
        while (self.link.snapshot().get("msg") is None or self.link.monitor8()[0]) and time.monotonic() - t0 < LINK_WAIT_S:
            time.sleep(0.1)
        if self.link.monitor8()[0]:
            raise Stop(f"the hand monitor is not heard ({self.link.monitor8()[1]}) - start monitor/hand_monitor.py first")
        hm = start_checks.hand_mapping(self.dry, self.keys.console.ask, self.link.snapshot)
        self.chain.append({"kind": "check", "t_iso": g3.now_iso(), "session_id": self.session_id, **hm})
        self.say(f"  [{'ok ' if hm['ok'] else 'BAD'}] {hm['name']}: {hm['detail']}")
        if not hm["ok"]:
            raise Stop("the hand mapping is off: fix the mapping before anything else (dawn protocol step 1)")
        self.setup_to_rest()

    def setup_to_rest(self) -> None:
        rig, rest = self.rig, self.seg["rest"]
        while True:                                            # set-up: to the exact rest pose (box off, as the null's)
            q6 = rig.read_q()
            rows = grasp_segment.interpolate(q6, rest, self.scripted_dps) + [list(rest)] * 5
            probs = segment_problems(self.guard, rows, box_on=False)
            if probs:
                raise Stop("the set-up move to rest is refused by its FK check (" + "; ".join(probs[:2]) + ") - guide "
                           "the arm nearer its rest pose by hand and start again")
            if self.gate("SETUP") and self.motion.run_rows("SETUP", rows, box_on=False,
                                                          vel_limit=self.scripted_vel)["end"] == "done":
                break
            self.freeze_flow("SETUP", resume_ok=True)

    def sim_palm(self) -> tuple:
        a = self.a
        if a.sim_palm != "auto":
            return tuple(float(v) for v in a.sim_palm.split(",")), tuple(float(v) for v in a.sim_drift_dir.split(","))
        rows = next(s["rows"] for s in self.seg["segments"] if s["state"] == "APPROACH_ZONE")
        poses = [rows[min(k, len(rows) - 1)] for k in (16, 20, 12, 24)]      # the freeze lands near row 16-20
        got = dry_world.pick_palm(self.planner, self.tm, poses, a.sim_script == "reorient")
        if got is None:
            raise Stop("dry run: no palm in the workspace box the planner has a plan for")
        return got

    def palms_px(self) -> list:
        """The synthetic camera draws a hand where the simulated feed last said a palm was."""
        hands = ((self.link.snapshot() if self.link is not None else {}).get("msg") or {}).get("hands") or []
        return [h["palm_px"] for h in hands]

    def shut_down(self, why: str, torque_off) -> None:
        if self.feed is not None:
            self.cues.close()
            self.feed.terminate()
            self.feed.wait(timeout=5)
        for piece in (self.link, self.pump):
            try:
                piece.stop()
            except Exception:  # noqa: BLE001
                pass
        if self.rt_worker is not None:
            try:
                self.rt_worker.close()
            except Exception:  # noqa: BLE001
                pass
        off_at_end = False
        if self.rig.bus is not None and why == "complete":       # only a session that ran to its end asks; a stop,
            off_at_end = ask_menu.confirm_torque_off(self.keys, self.chain, self.session_id, "END", END_PROMPT)   # an
            if off_at_end:                                        # internal error or a hard abort leave torque as it is
                self.rig.sbus.disable_torque()
        dts = self.motion.dts
        pct = lambda p: round(float(sorted(dts)[min(len(dts) - 1, int(p / 100 * len(dts)))]), 2) if dts else None  # noqa: E731
        lat = [r["t_hold_minus_frame_t_ms"] for r in self.chain.rows() if r.get("kind") == "freeze"
               and r.get("monitor") == 7 and r.get("t_hold_minus_frame_t_ms") is not None]
        self.chain.append({"kind": "session_end", "t_iso": g3.now_iso(), "session_id": self.session_id, "why": why,
                           "label": self.label, "outcomes": self.outcomes, "torque_off_by_kill": bool(torque_off),
                           "torque_off_at_end": off_at_end,
                           "loop_dt_ms": {"n": len(dts), "p50": pct(50), "p95": pct(95), "max": round(max(dts), 2) if dts else None,
                                          "limit": round(1000 * self.guard.loop_dt_max_s, 1)},
                           "frame_to_hold_ms_monitor7": lat, "pump_error": self.pump.error})
        ok, detail = self.chain.verify()
        self.say(f"  chain -> {self.chain.path} ({'ok' if ok else 'BROKEN: ' + detail}; {self.chain.seq} rows)")
        self.chain.close()
        if self.rig.bus is not None:
            self.rig.close(disable_torque=off_at_end)             # never by itself: only after OFF and YES

    # -- one trial -------------------------------------------------------------------------------------
    def gate(self, phase: str) -> bool:
        """A stationary state enters a moving one only with monitors 7 and 8 clear; else it is a freeze with a row."""
        q6, mon, snap = self.motion.read_monitors(self.state)
        why = sm.may_start_moving(bool(mon["7"]["clear"]), bool(mon["8"]["fired"]))
        if why is None:
            return True
        n = 8 if mon["8"]["fired"] else 7
        self.motion.freeze(phase, [(n, mon_mod.CODES[n], f"refused at the start of {phase}: {why}")], q6, snap,
                           source=f"gate:{phase}")
        return False

    def trial_end(self, outcome: str, why: str = "") -> dict:
        rows = [r for r in self.chain.rows() if r.get("trial_id") == self.trial_id]
        out = {"trial_id": self.trial_id, "outcome": outcome, "why": why,
               "freezes": sum(1 for r in rows if r.get("kind") == "freeze"),
               "judge_calls": sum(1 for r in rows if r.get("kind") == "judge")}
        self.chain.append({"kind": "trial_end", "t_iso": g3.now_iso(), "session_id": self.session_id, **out,
                           "label": self.label})
        self.say(f"  TRIAL {self.trial_id}: {outcome}{' - ' + why if why else ''}")
        return out

    def trial(self, n: int) -> dict:
        why = self.hours_refusal()
        if why:                                                # on the arm: the hours are read at every trial
            self.chain.append({"kind": "refused", "t_iso": g3.now_iso(), "session_id": self.session_id,
                               "reason": "motion_hours", "detail": f"trial {n} not started: {why}", "arm": not self.dry})
            raise Stop(f"trial {n} not started: {why}")
        self.trial_id = self.motion.trial_id = f"{self.session_id}/T{n:03d}"
        self.state, self.holding, self.handover_path, self.preauth_used = "IDLE", False, [], False
        self.place_palm = self.release_verdict = self.released_by = None
        self.place_dir = self.place_reach = None
        if self.a.handover_trial or self.a.palm_placement:
            flag = "--palm-placement" if self.a.palm_placement else "--handover-trial"
            detail = ("palm placement for this trial's first hand event: settle 1 s, the judge, the plan, the move, "
                      "settle 1 s, the judge, the release - no key unless a refusal, a monitor or UNSAFE"
                      if self.a.palm_placement else
                      "[h] given at trial start for this trial's first hand event, after the judge; an UNSAFE verdict "
                      "or any refusal holds and asks; [r] is typed")
            self.chain.append({"kind": "preauth", "t_iso": g3.now_iso(), "session_id": self.session_id,
                               "trial_id": self.trial_id, "by": f"the operator, {flag}", "detail": detail})
            self.say(f"  {'Palm placement' if self.a.palm_placement else 'Handover'} pre-authorised for trial {n} ({flag}).")
        hands = (f"{LAYER_OFF}: the PROP HAND in the place zone, real hands OUT of the workspace" if self.layer_off
                 else "Hands OUT")
        self.keys.console.ask(f"\n=== Trial {n}: the screwdriver at its spawn pose, laid across the radial line, grip mark under the jaws. "
                              f"{hands}. Press Enter: ", "")
        segs = {s["state"]: s["rows"] for s in self.seg["segments"]}
        order = [st for st in sm.TASK_ORDER if not (self.layer_off and st in sm.CHECKPOINTS)]   # no judge: no CP1 / CP2
        for st in order:
            out = self.checkpoint(st) if st in sm.CHECKPOINTS else self.segment(st, segs[st])
            if out == "handed_over":
                if self.a.palm_placement:
                    self.outcome_label()
                    return self.trial_end("HANDED_OVER", f"placed in the palm, released by {self.released_by}")
                return self.trial_end("HANDED_OVER", "the operator approved a handover; released at the standoff")
        self.state = "IDLE"
        return self.trial_end("PLACED", "the scripted task ran to its end" + (f" ({LAYER_OFF})" if self.layer_off else ""))

    def segment(self, st: str, rows: list) -> str:
        while True:
            if self.gate(st):
                self.state = st
                res = self.motion.run_rows(st, rows, vel_limit=self.scripted_vel)
                if res["end"] == "done":
                    q6 = self.rig.read_q()
                    self.holding = self.gripper_closed(q6)
                    if st == "GRASP":
                        held, why = self.grasp_held(q6)
                        if not held:
                            self.holding = False
                            self.say(f"  THE GRASP DID NOT HOLD THE HANDLE: {why}. Lay the screwdriver with its grip "
                                     "mark under the jaws.")
                            self.refreeze("GRASP", f"the grasp did not hold the handle: {why}")
                            if self.freeze_flow("GRASP", resume_ok=False) == "handed_over":
                                return "handed_over"
                            raise Stop("the grasp did not hold the handle")
                    return "done"
                rows = res["rows"][res["k"]:] or [rows[-1]]
            if self.freeze_flow(st, resume_ok=True) == "handed_over":
                return "handed_over"
            rows = grasp_segment.interpolate(self.rig.read_q(), rows[0], self.scripted_dps) + rows   # [c]: re-anchored

    def checkpoint(self, cp: str) -> str:
        """The arm stationary, the monitors read for 0.5 s, then the judge. UNSAFE -> frozen, ASK."""
        while True:
            self.state = cp
            if self.motion.hold_rows(cp, CP_SETTLE_S)["end"] == "frozen":
                if self.freeze_flow(cp, resume_ok=True) == "handed_over":
                    return "handed_over"
                continue
            v = self.judge_at(cp)
            if not v["unsafe"]:
                return "done"
            self.motion.freeze(cp, [(0, "judge_unsafe", f"the judge's verdict at {cp} is UNSAFE by the code's reading: "
                                                        f"{v.get('reason')}")], self.rig.read_q(), self.link.snapshot(),
                               source=f"judge:{cp}")
            return "handed_over" if self.freeze_flow(cp, resume_ok=True, verdict=v) == "handed_over" else "done"



def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", action="store_true", help="the rig (the operator present)")
    ap.add_argument("--dry-run", action="store_true", help="the default: fake bus, synthetic frames, simulated hand feed")
    ap.add_argument("--port", default=None)
    ap.add_argument("--camera", default="0")
    ap.add_argument("--root", default=str(paths.ROOT))
    ap.add_argument("--sessions-dir", default=None)
    ap.add_argument("--operator", default=None)
    ap.add_argument("--trials", type=int, default=1)
    ap.add_argument("--z-handover-mm", type=float, default=None, help="fingertip height at the hold (dawn step 3)")
    ap.add_argument("--standoff-mm", type=float, default=None, help="handle tip to palm (mm); default 60, never below planner.cfg.STANDOFF_MIN_MM (30)")
    ap.add_argument("--segments", default=None, help="the dawn-recorded segments (dawn/segments.json); required on --arm")
    ap.add_argument("--operator-override", action="store_true",
                    help="with --arm: run without a green fixture status for this code (and outside any hours); "
                         "'UNTESTED CODE - OPERATOR OVERRIDE' is printed and stamped on every step row")
    ap.add_argument("--palm-placement", action="store_true",
                    help="pre-authorised palm placement (the operator, 2026-10-03): see placement_flow.py")
    ap.add_argument("--handover-trial", action="store_true",
                    help="pre-authorise the handover at trial start (the operator, 2026-10-03): see handover_flow.py")
    ap.add_argument("--hold-test", action="store_true", help="dawn step 4: close, lift 20 mm, hold 10 s, log the load")
    ap.add_argument("--layer-off", action="store_true",
                    help="the video's contrast shot (the operator, 2026-10-04): hand monitor, judge and handover planner "
                         "off; monitors 1-6 live; pick -> place only; a prop hand in the place zone; every row stamped "
                         "'TRUST LAYER OFF - DEMONSTRATION'")
    ap.add_argument("--scripted-dps", type=float, default=None,
                    help="scripted segments, deg/s: default 3 on --arm (dawn rule), 10 in a dry run; at most 10")
    ap.add_argument("--knife-handle-mm", type=float, default=None, help="grip point to handle tip (measured)")
    ap.add_argument("--knife-blade-mm", type=float, default=None, help="grip point to blade tip (measured)")
    ap.add_argument("--knife-width-mm", type=float, default=None, help="handle across the jaws at the grip (measured)")
    ap.add_argument("--knife-axis-sign", type=int, choices=(1, -1), default=None,
                    help="which way along the gripper's y axis the free handle points (set at the grasp, dawn step 2)")
    ap.add_argument("--yes", action="store_true", help="dry run only: prompts answered by script")
    ap.add_argument("--fast", type=float, default=None, help="dry run only: clock scale")
    ap.add_argument("--no-esc", action="store_true")
    ap.add_argument("--udp-port", type=int, default=paths.UDP_ADDR[1])
    ap.add_argument("--frame-dir", default=str(paths.FRAME_DIR))
    for flag, default in (("--dry-keys", "h,h,r"), ("--sim-script", "handover"), ("--sim-palm", "auto"),
                          ("--sim-drift-dir", "0,1"), ("--dry-judge", None), ("--sim-fingers-dir", None)):
        ap.add_argument(flag, default=default, help=argparse.SUPPRESS)
    ap.add_argument("--sim-event", action="append", default=[], help=argparse.SUPPRESS)
    ap.add_argument("--dry-retarget-delay-s", type=float, default=0.0, help=argparse.SUPPRESS)   # dry run only
    ap.add_argument("--cue-port", type=int, default=CUE_PORT, help=argparse.SUPPRESS)
    return ap


def scripted_dps(a) -> float:
    """Ruling 2 (reviewer, 2026-10-01): on the arm every move runs at <= 3 deg/s until the encoders have been seen at
    speed; then 10 by an explicit --scripted-dps 10. A dry run rehearses at 10 unless told otherwise."""
    v = float(a.scripted_dps) if a.scripted_dps is not None else (3.0 if a.arm else grasp_segment.SCRIPTED_DPS)
    if not 0.0 < v <= grasp_segment.SCRIPTED_DPS:
        raise ValueError(f"--scripted-dps {v:g}: must be above 0 and at most {grasp_segment.SCRIPTED_DPS:g}")
    return v


def knife_dims(a) -> dict | None:
    """The prop's measured dimensions, all three or none (None = the planner's placeholder knife)."""
    got = {"handle_len_mm": a.knife_handle_mm, "blade_overhang_mm": a.knife_blade_mm, "handle_width_mm": a.knife_width_mm}
    if all(v is None for v in got.values()):
        return None
    if any(v is None or not v > 0 for v in got.values()):
        raise ValueError("--knife-handle-mm, --knife-blade-mm and --knife-width-mm go together, each above 0")
    out = {k: float(v) for k, v in got.items()}
    if getattr(a, "knife_axis_sign", None) is not None:
        out["axis_sign"] = int(a.knife_axis_sign)
    return out


def main(argv: list | None = None, **inject) -> int:
    a = build_parser().parse_args(argv)
    sys.setswitchinterval(0.001)       # the camera and link threads hand the interpreter back within 1 ms (default 5)
    if a.arm and a.dry_run:
        print("--arm and --dry-run together: refused")
        return 2
    for out_dir in (a.sessions_dir, a.frame_dir):               # before anything is written anywhere
        why = frozen_path_refusal(out_dir) if out_dir else None
        if why:
            print(f"HANDOVER SESSION REFUSED: {why}")
            return 2
    try:
        runner = HandoverSession(a, Path(a.root).resolve(), **inject)
    except (FileNotFoundError, KeyError, ValueError) as e:
        print(f"HANDOVER SESSION REFUSED: {type(e).__name__}: {e}")
        return 2
    return runner.run()


if __name__ == "__main__":
    sys.exit(main())
