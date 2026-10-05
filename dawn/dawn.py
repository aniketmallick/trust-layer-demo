#!/usr/bin/env python
"""Dawn protocol tools, 2026-10-02 (steps 1-2). One subcommand per check. Nothing here sends a goal position.

    framing                                camera only: the anchor's framing re-check against the A2b of record
    ruler --zone Z --far F --clamps y|n    the session pre-check's rule (erratum 8): each side within 2 mm, clamps
    mapping [--n 3]                        camera + the hand monitor (--once): the palm on cross 6, <= 15 mm
    bus --port P                           READ ONLY: torque state per joint and the joint positions
    pose --port P --name NAME              READ ONLY, torque must be off: the pose, its fingertip (FK), a still
    live --port P                          READ ONLY, the operator's terminal: a live readout; Enter records a pose
    torque-off --port P --holding-the-arm  WRITES Torque_Enable = 0 on all six: THE ARM DROPS unless held

The bus is closed with disable_torque=False (lerobot's default close turns torque off, which drops a held arm).
Every result is a row of sessions/KH-DAWN-<date>/dawn.jsonl, the same hash chain as a session (steplog.py).
"""
from __future__ import annotations

import argparse
import datetime
import functools
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

EXP = Path(__file__).resolve().parent.parent
if str(EXP) not in sys.path:
    sys.path.insert(0, str(EXP))
from common import paths  # noqa: E402

paths.use_anchor()
import g3  # noqa: E402

import start_checks  # noqa: E402
from steplog import StepLog  # noqa: E402

DAWN = EXP / "dawn"
POSES = DAWN / "poses.json"
MONITOR_PY = EXP / ".venv_monitor" / "bin" / "python"
WARMUP = 15
REG_Q_ZERO = json.loads((paths.ROOT / paths.REG_REL).read_text())["arm_frame"]["q_zero_lerobot_deg"]


def chain() -> tuple:
    d = paths.SESSIONS / f"KH-DAWN-{datetime.datetime.now(g3.IST):%Y%m%d}"
    d.mkdir(parents=True, exist_ok=True)
    return StepLog(d / "dawn.jsonl"), d


def row(kind: str, **kw) -> dict:
    log, _ = chain()
    r = log.append({"kind": kind, "t_iso": g3.now_iso(), "label": "DEMONSTRATION - dawn protocol, screwdriver (prop tool)",
                    **kw})
    log.close()
    return r


def frames(n: int, camera: str = "0", spacing_s: float = 0.0, retry: bool = True) -> list:
    """n frames after a warm-up. A failed read releases the camera, waits 1 s, opens it again and starts over - once
    (null_session.ReopeningCamera's fix of 2026-09-29: the first real session stopped on 'camera read failed')."""
    import cv2
    cap = cv2.VideoCapture(int(camera) if camera.lstrip("-").isdigit() else camera)
    try:
        if not cap.isOpened():
            raise RuntimeError(f"camera {camera} could not be opened")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        for _ in range(WARMUP):
            cap.read()
        out = []
        for _ in range(n):
            ok, img = cap.read()
            if not ok:
                raise RuntimeError("camera read failed")
            out.append(img)
            time.sleep(spacing_s)
        return out
    except RuntimeError as e:
        if not retry:
            raise
        print(f"  [camera] {e} - re-opening once")
        cap.release()
        time.sleep(1.0)
        return frames(n, camera, spacing_s, retry=False)
    finally:
        cap.release()


def save_png(img, d: Path, name: str) -> tuple:
    import cv2
    p = d / "photos" / name
    p.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(p), img)
    return f"photos/{name}", hashlib.sha256(p.read_bytes()).hexdigest()


# ------------------------------------------------------------------------------------------ step 1
def cmd_framing(a) -> int:
    _, d = chain()
    res = start_checks.framing(paths.ROOT, d, False, lambda n: frames(n, a.camera), print)
    row("check", **res)
    print(f"FRAMING RE-CHECK: {'PASS' if res['ok'] else 'FAIL'} - {res['detail']}")
    return 0 if res["ok"] else 1


def cmd_ruler(a) -> int:
    answers = {"ZONE": str(a.zone), "FAR": str(a.far), "C-clamps": a.clamps}
    ask = lambda prompt, default="": next(v for k, v in answers.items() if k in prompt)      # noqa: E731
    checks = start_checks.ruler_and_clamps(paths.ROOT, False, ask)
    for c in checks:
        row("check", **c)
        print(f"[{'ok ' if c['ok'] else 'BAD'}] {c['name']}: {c['detail']}")
    return 0 if all(c["ok"] for c in checks) else 1


def monitor_once(still: Path) -> dict:
    r = subprocess.run([str(MONITOR_PY), str(EXP / "monitor" / "hand_monitor.py"), "--once", str(still)],
                       capture_output=True, text=True, env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})
    line = next((x for x in r.stdout.splitlines() if x.startswith("{")), None)
    if r.returncode != 0 or line is None:
        raise RuntimeError(f"hand monitor --once failed (exit {r.returncode}): {r.stderr[-300:]}")
    return json.loads(line)


def cmd_mapping(a) -> int:
    _, d = chain()
    stamp = datetime.datetime.now(g3.IST).strftime("%H%M%S")
    reads, ok = [], True
    for k, img in enumerate(frames(a.n, a.camera, spacing_s=0.4), 1):
        rel, sha = save_png(img, d, f"mapping_{stamp}_{k}.png")
        msg = monitor_once(d / rel)
        hands = msg.get("hands") or []
        r = {"still": rel, "sha256": sha, "n_hands": len(hands), "model": msg.get("model"), "proc_ms": msg.get("proc_ms")}
        if len(hands) == 1:
            p = hands[0]["palm_mm"]
            off = ((p[0] - start_checks.CROSS6_MM[0]) ** 2 + (p[1] - start_checks.CROSS6_MM[1]) ** 2) ** 0.5
            r.update(palm_mm=p, palm_px=hands[0]["palm_px"], conf=hands[0]["conf"], off_mm=round(off, 1),
                     in_spawn=hands[0]["in_spawn"], in_zone=hands[0]["in_zone"])
            ok = ok and off <= start_checks.MAP_TOL_MM
        else:
            ok = False
        reads.append(r)
        print(f"  reading {k}: {r['n_hands']} hand(s)" + (f", palm {r['palm_mm']} mm, {r['off_mm']} mm from cross 6 "
                                                          f"{start_checks.CROSS6_MM}" if "off_mm" in r else ""))
    row("check", name="hand_mapping_live", ok=ok, readings=reads, cross6_mm=list(start_checks.CROSS6_MM),
        tol_mm=start_checks.MAP_TOL_MM, detail=f"{sum(1 for r in reads if r.get('off_mm') is not None and r['off_mm'] <= start_checks.MAP_TOL_MM)} of {len(reads)} readings within {start_checks.MAP_TOL_MM:g} mm")
    print(f"HAND MAPPING: {'PASS' if ok else 'FAIL - stop and fix the mapping first'}")
    return 0 if ok else 1


# ------------------------------------------------------------------------------------------ step 2 (the bus)
def open_bus(port: str):
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus
    from safety import ALL_JOINTS, load_calibration_file
    cal = load_calibration_file(paths.ROOT / paths.CALIBRATION_REL)
    motors = {j: Motor(i + 1, "sts3215", MotorNormMode.RANGE_0_100 if j == "gripper" else MotorNormMode.DEGREES)
              for i, j in enumerate(ALL_JOINTS)}
    bus = FeetechMotorsBus(port=port, motors=motors, calibration={
        j: MotorCalibration(id=int(c["id"]), drive_mode=int(c.get("drive_mode", 0)), homing_offset=int(c["homing_offset"]),
                            range_min=int(c["range_min"]), range_max=int(c["range_max"])) for j, c in cal.items()})
    bus.connect()                                          # pings all six ids; writes nothing
    return bus


def read_state(bus) -> dict:
    q = bus.sync_read("Present_Position")
    q6 = [float(q[j]) for j in g3.ALL]
    torque = {j: int(bus.read("Torque_Enable", j, normalize=False)) for j in g3.ALL}
    return {"q6": [round(v, 3) for v in q6], "torque": torque, "calibrated": bool(bus.is_calibrated)}


@functools.lru_cache(maxsize=1)
def _guard_and_view() -> tuple:
    """The G3 guard of record and the camera's footprint, read once (the anchor record is 17 MB)."""
    import numpy as np
    from planner.cfg import visible_quad
    reg = json.loads((paths.ROOT / paths.REG_REL).read_text())
    rec = json.loads((paths.ROOT / paths.RECORD_REL).read_text())
    guard = g3.guard_from_record(paths.ROOT, reg, rec)
    return guard, np.array(visible_quad(reg, guard.box))


def fingertip(q6) -> dict:
    from common.table_map import _inside
    guard, quad = _guard_and_view()
    tip = guard.ee_mm(q6)
    vis = _inside(quad, tip[:2])
    x0, x1, y0, y1 = guard.box
    return {"fingertip_mm": [round(float(v), 1) for v in tip], "h_above_table_mm": round(float(tip[2]) - guard.table_z, 1),
            "in_box": bool(x0 <= tip[0] <= x1 and y0 <= tip[1] <= y1), "in_camera_footprint": bool(vis),
            "joint_margin_fires": [d for c, d in guard.check(0.0, q6) if c == "joint_margin"]}


def cmd_bus(a) -> int:
    bus = open_bus(a.port)
    try:
        st = read_state(bus)
    finally:
        bus.disconnect(disable_torque=False)               # never a torque change on the way out
    ft = fingertip(st["q6"])
    row("bus_read", port=a.port, **st, **ft)
    print(json.dumps({**st, **ft}))
    return 0 if st["calibrated"] else 2


def cmd_pose(a) -> int:
    bus = open_bus(a.port)
    try:
        st = read_state(bus)
    finally:
        bus.disconnect(disable_torque=False)
    on = [j for j, v in st["torque"].items() if v]
    if on:
        print(f"REFUSED: torque is ON on {on}. Poses are read with the arm moved by hand, torque off.")
        row("refused", what=f"pose {a.name}", reason="torque on", torque=st["torque"])
        return 2
    record_pose(a.name, st, fingertip(st["q6"]), a.camera, a.note)
    return 0


def record_pose(name: str, st: dict, ft: dict, camera: str, note: str) -> None:
    """One pose into dawn/poses.json and the chain, with a still named by the time (never overwritten)."""
    _, d = chain()
    img = frames(1, camera)[0]
    rel, sha = save_png(img, d, f"pose_{name}_{datetime.datetime.now(g3.IST):%H%M%S}.png")   # never overwritten (the first GRASP still was, 10:29)
    poses = json.loads(POSES.read_text()) if POSES.is_file() else {}
    if name in poses:
        poses[f"{name}_replaced_{datetime.datetime.now(g3.IST):%H%M%S}"] = poses[name]
    poses[name] = {"q6_lerobot": st["q6"], **ft, "still": f"sessions/{d.name}/{rel}", "still_sha256": sha,
                   "t_iso": g3.now_iso(), "note": note}
    POSES.write_text(json.dumps(poses, indent=1) + "\n")
    row("pose", name=name, q6=st["q6"], torque=st["torque"], still=rel, still_sha256=sha, note=note, **ft)
    print(f"\nPOSE {name}: q6 {st['q6']}\n  fingertip {ft['fingertip_mm']} mm, {ft['h_above_table_mm']} mm above the "
          f"table; in box {ft['in_box']}, in the camera's footprint {ft['in_camera_footprint']}; joint margin "
          f"{ft['joint_margin_fires'] or 'inside'}; still {rel} {sha[:12]}")
    if name == "HANDOVER_SAMPLE":
        print(f"  z_handover_mm = {ft['h_above_table_mm']} (fingertip above the table) - for the operator to confirm")


def cmd_live(a) -> int:
    """READ ONLY, for the operator's own terminal: a live line (fingertip, height, jaws, roll, the checks) while the arm
    is moved by hand; Enter, then a name, records that pose with a still. q quits. The port is closed without a torque
    change. Nothing is written to the bus."""
    import threading
    names = {"h": "SPAWN_HOVER", "g": "GRASP", "l": "LIFT", "s": "HANDOVER_SAMPLE"}
    bus = open_bus(a.port)
    state, stop = {}, threading.Event()

    def watch() -> None:
        while not stop.is_set():
            try:
                st = read_state(bus)
                ft = fingertip(st["q6"])
                state.update(st=st, ft=ft)
                on = [j for j, v in st["torque"].items() if v]
                roll = st["q6"][4] - REG_Q_ZERO[4]
                sys.stdout.write(f"\r  tip {ft['fingertip_mm']} mm | {ft['h_above_table_mm']:6.1f} mm above the table | "
                                 f"jaws {st['q6'][5]:5.1f} % | roll {roll:+6.1f} deg | box {'ok' if ft['in_box'] else 'OUT'}"
                                 f" | view {'ok' if ft['in_camera_footprint'] else 'OUT'} | margins "
                                 f"{'ok' if not ft['joint_margin_fires'] else 'OUT'} | torque {'ON ' + str(on) if on else 'off'}   ")
                sys.stdout.flush()
            except Exception as e:  # noqa: BLE001 - a bus hiccup is shown, the watch goes on
                sys.stdout.write(f"\r  (read failed: {type(e).__name__}: {e})   ")
            time.sleep(0.3)

    th = threading.Thread(target=watch, daemon=True)
    th.start()
    print("Move the arm by hand. Enter, then h = SPAWN_HOVER, g = GRASP, l = LIFT, s = HANDOVER_SAMPLE; q = quit.")
    try:
        while True:
            input()
            ans = input("\n  record which? [h/g/l/s, q = quit, Enter = keep watching]: ").strip().lower()
            if ans == "q":
                break
            if ans not in names:
                continue
            st, ft = state.get("st"), state.get("ft")
            if not st:
                print("  no reading yet")
                continue
            on = [j for j, v in st["torque"].items() if v]
            if on:
                print(f"  NOT RECORDED: torque is ON on {on} (poses are taken with the arm moved by hand)")
                continue
            note = input("  a note for this pose (Enter for none): ").strip()
            record_pose(names[ans], st, ft, a.camera, note or "recorded live by the operator")
    finally:
        stop.set()
        th.join(timeout=2)
        bus.disconnect(disable_torque=False)
    return 0


def cmd_torque_off(a) -> int:
    if not a.holding_the_arm:
        print("REFUSED: --holding-the-arm is required: torque off lets the arm drop under gravity.")
        return 2
    bus = open_bus(a.port)
    try:
        before = read_state(bus)["torque"]
        bus.disable_torque()                               # the only write in this file
        after = read_state(bus)["torque"]
    finally:
        bus.disconnect(disable_torque=False)
    row("torque_off", port=a.port, before=before, after=after, ok=not any(after.values()))
    print(f"torque before {before}\ntorque after  {after}")
    return 0 if not any(after.values()) else 1


def main(argv: list | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("framing", "mapping", "pose", "bus", "torque-off", "ruler", "live"):
        s = sub.add_parser(name)
        s.add_argument("--camera", default="0")
        if name in ("pose", "bus", "torque-off", "live"):
            s.add_argument("--port", required=True)
    sub.choices["mapping"].add_argument("--n", type=int, default=3)
    sub.choices["pose"].add_argument("--name", required=True,
                                     choices=("SPAWN_HOVER", "GRASP", "LIFT", "HANDOVER_SAMPLE"))
    sub.choices["pose"].add_argument("--note", default="")
    sub.choices["torque-off"].add_argument("--holding-the-arm", action="store_true")
    r = sub.choices["ruler"]
    r.add_argument("--zone", type=float, required=True)
    r.add_argument("--far", type=float, required=True)
    r.add_argument("--clamps", choices=("y", "n"), required=True)
    a = ap.parse_args(argv)
    return {"framing": cmd_framing, "ruler": cmd_ruler, "mapping": cmd_mapping, "bus": cmd_bus, "pose": cmd_pose,
            "torque-off": cmd_torque_off, "live": cmd_live}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
