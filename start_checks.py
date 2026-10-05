"""Session-start checks for the demonstration (PLAN.md F7): what the null runner checks, read from the frozen record
and re-done here, with every row written under THIS session's directory. Nothing under phase0/ is written.

  frozen files   the registration, the layout and the null plan by canonical hash; the calibration file = the
                 registration's; the anchor modules this runner imports, by sha256 (logged)
  framing        the printed outlines against the A2b of record and the A3 clear view (framing_recheck.recheck +
                 erratum 4b's rule, references read from phase0/anchor); a second capture before a FAIL stands
  ruler, clamps  the two prompts of session_precheck.py (erratum 8), each reading within 2 mm of the registration's
  kill switch    A10 in the record: verified on the rig
  G3             the latest dry fault table all as expected; the real excursion froze
  hours          MOTION_HOURS (None since 2026-10-03: no window, the operator's decision)
  hand mapping   dawn step 1: the operator's palm on spawn cross 6 reads within 15 mm of the cross
  this demo      the handover fault table all as expected; the fixture status all green
NOT run: the base guard (a motion step of PR-001; this is a demonstration and says so in its start row).
A dry run does the same list on the synthetic bench, as null_session.py does; its PASS satisfies nothing on the rig.
"""
from __future__ import annotations

import datetime
import json
from pathlib import Path

import numpy as np

from common import paths

paths.use_anchor()
import g3  # noqa: E402

CROSS6_MM = (241.1, -58.8)
MAP_TOL_MM = 15.0
ANCHOR_MODULES = ("safety.py", "g3.py", "fk.py", "fake_bus.py", "null_plan.py", "framing_recheck.py", "check_framing.py",
                  "session_precheck.py")
NULL_PLAN_SHA = "8ecf9a00d87f125206d1f2096e116204e665bf04137cb561142ac9511e226d39"
FROZEN = {paths.REG_REL: paths.REG_SHA256, paths.LAYOUT_REL: paths.LAYOUT_SHA256, paths.NULL_PLAN_REL: NULL_PLAN_SHA}
FAULT_TABLE = "fault_table_handover.json"         # under sessions/_dryrun/
FIXTURE_STATUS = "fixture_status.json"            # under sessions/: {"all_green": bool, "t_iso": ..., "suites": {...}}


# (start, end) hour of the day, IST, for powered motion; None = no window. The operator, 2026-10-03: "don't keep any
# time bounded restrictions like 22:00 or anything". Until then (6, 22): run_anchor.motion_hours_ok's rule (architect
# ruling 2026-09-22 addendum E). Setting it again re-enables every hours check (start, every trial, every motion key).
MOTION_HOURS: tuple[int, int] | None = None


def hours_text() -> str:
    if MOTION_HOURS is None:
        return "no motion window (the operator, 2026-10-03)"
    a, b = MOTION_HOURS
    return f"powered motion {a:02d}:00-{b:02d}:00 IST only"


def motion_hours_ok(now: datetime.datetime | None = None) -> bool:
    """Inside MOTION_HOURS (IST, start inclusive); always True when there is no window."""
    if MOTION_HOURS is None:
        return True
    t = (now or datetime.datetime.now(g3.IST)).astimezone(g3.IST)
    a, b = MOTION_HOURS
    return (a, 0) <= (t.hour, t.minute) < (b, 0)


def frozen_files(root: Path, reg: dict) -> list:
    out = []
    for rel, want in FROZEN.items():
        p = root / rel
        got = g3.canonical_hash(json.loads(p.read_text(encoding="utf-8"))) if p.is_file() else None
        out.append({"name": f"frozen:{rel}", "ok": got == want, "detail": f"canonical {str(got)[:12]} (frozen {want[:12]})"})
    cal = root / paths.CALIBRATION_REL
    got = g3.sha_file(cal) if cal.is_file() else None
    out.append({"name": "calibration_file", "ok": got is not None and got == reg["arm_calibration"]["sha256"],
                "detail": f"sha256 {str(got)[:12]} (registration {reg['arm_calibration']['sha256'][:12]})"})
    mods = {m: g3.sha_file(paths.ANCHOR / m) for m in ANCHOR_MODULES if (paths.ANCHOR / m).is_file()}
    out.append({"name": "anchor_modules", "ok": len(mods) == len(ANCHOR_MODULES), "sha256": mods,
                "detail": "imported read-only, by path: " + ", ".join(f"{m} {s[:12]}" for m, s in mods.items())})
    return out


def framing(root: Path, session_dir: Path, dry: bool, grab=None, say=print) -> dict:
    """grab(n) -> n BGR frames of the clear table (the rig); a dry run renders the anchor's synthetic bench."""
    import cv2
    import check_framing as cf
    import framing_recheck as fr
    if dry:
        bench = dict(cf.DRY_RUN_BENCH)
        clean = cf._render_scene(seed=0, **bench)[0]
        o = cf.find_printed_outlines(clean)
        a2b = {"spawn_quad_native": o["spawn"]["quad"].tolist(), "zone_quad_native": o["zone"]["quad"].tolist(),
               "t_end_iso": "dry-run synthetic bench"}
        photos = [clean]
        shots = lambda k0: [cf._render_scene(seed=k0 + k, **bench)[0] for k in range(1, 6)]   # noqa: E731
    else:
        a2b, photos, _ = fr._refs(root / "phase0" / "anchor")                 # read-only
        shots = lambda k0: grab(8)                                             # noqa: E731
    frames = shots(0)
    res = fr.apply_ruling_0928(fr.recheck(frames, a2b, photos))
    first = None
    if not res["pass"]:
        first = {"fails": res["fails"], "vs_a2b_px": res["vs_a2b"].get("max_shift_px"),
                 "vs_a3_px": res["vs_a3"].get("max_shift_px")}
        say("  FRAMING RE-CHECK: the first capture FAILED - a second capture (erratum 4b). Keep the view clear.")
        frames = shots(10)
        res = fr.apply_ruling_0928(fr.recheck(frames, a2b, photos))
    img = np.median(np.stack(frames), 0).astype(np.uint8)
    png = session_dir / "photos" / "framing_recheck.png"
    png.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(png), img)
    return {"name": "framing_recheck", "ok": bool(res["pass"]), "fails": res["fails"], "capture": f"photos/{png.name}",
            "capture_sha256": g3.sha_file(png), "vs_a2b_shift_px": res["vs_a2b"].get("max_shift_px"),
            "vs_a3_shift_px": res["vs_a3"].get("max_shift_px"), "second_capture": first is not None, "first_capture": first,
            "a3_at_boundary": bool((res.get("decision_rule") or {}).get("a3_at_boundary")), "dry_run": dry,
            "detail": ("PASS" if res["pass"] else "FAIL: " + "; ".join(res["fails"][:2]))
                      + f" (A2b {res['vs_a2b'].get('max_shift_px')} px, A3 {res['vs_a3'].get('max_shift_px')} px; "
                        f"reference {(a2b or {}).get('t_end_iso')})"}


def ruler_and_clamps(root: Path, dry: bool, ask) -> list:
    """The two prompts of session_precheck.py (its make_entry decides); the answers go in this session's rows."""
    import session_precheck as sp
    ref = g3.ruler_reference(root)
    if ref.get("zone_side_mm") is None or ref.get("far_side_mm") is None:
        return [{"name": "ruler", "ok": False, "detail": "the registration has no ruler reference (base_square)"}]
    readings = {}
    for key, side in sp.SIDES:
        s = ask(f"  Ruler at the {side} side, base plate front edge to the spawn sheet's NEAR line (mm) "
                f"[registration {ref[key]:g}]: ", str(ref[key]) if dry else "")
        try:
            readings[key] = float(str(s).strip())
        except ValueError:
            readings[key] = None
    clamps = ask("  Are BOTH C-clamps hand-tight and NOT moved? [y/n]: ", "y" if dry else "").strip().lower() in ("y", "yes")
    e = sp.make_entry(readings, ref, clamps, dry)
    return [{"name": "ruler", "ok": bool(e["ruler_ok"]), "entry": e,
             "detail": f"ruler {readings} vs the registration's {ref} (off {e['deviation_mm']}; each within "
                       f"{g3.RULER_TOL_MM:g} mm)"},
            {"name": "clamps", "ok": bool(e["clamps_ok"]), "detail": f"clamps hand-tight and not moved = {e['clamps_ok']}"}]


def from_record(root: Path, record: dict, dry: bool) -> list:
    a10 = record.get("A10") or {}
    out = [{"name": "kill_switch_verified", "ok": a10.get("kill_switch_verified") is True
            and (dry or a10.get("dry_run") is False),
            "detail": f"A10 kill_switch_verified={a10.get('kill_switch_verified')} dry_run={a10.get('dry_run')}"}]
    ft = g3._latest_jsonl(root / "phase0" / "g3" / "fault_injection.jsonl")
    out.append({"name": "g3_fault_table", "ok": bool(ft and ft.get("all_as_expected") is True),
                "detail": f"latest G3 dry fault table {(ft or {}).get('t_iso')} file {(ft or {}).get('file')} "
                          f"all_as_expected={(ft or {}).get('all_as_expected')} (read-only)"})
    if not dry:
        rx = g3._latest_jsonl(root / "phase0" / "g3" / "real_excursion.jsonl")
        out.append({"name": "g3_real_excursion", "ok": bool(rx and rx.get("froze") is True),
                    "detail": f"latest real excursion {(rx or {}).get('t_iso')} froze={(rx or {}).get('froze')}"})
    return out


def this_demo(sessions: Path, dry: bool) -> list:
    out = []
    p = sessions / "_dryrun" / FAULT_TABLE
    ft = json.loads(p.read_text(encoding="utf-8")) if p.is_file() else None
    ok = bool(ft and ft.get("all_as_expected") is True)
    out.append({"name": "handover_fault_table", "ok": ok or dry, "gating": not dry,
                "detail": f"{p.name}: " + (f"{ft.get('t_iso')} all_as_expected={ft.get('all_as_expected')}" if ft else
                                           "not built") + (" (a dry run does not need it)" if dry and not ok else "")})
    p = sessions / FIXTURE_STATUS
    fs = json.loads(p.read_text(encoding="utf-8")) if p.is_file() else None
    from fixtures.status import tree_hash                 # the status counts only for the code it was run on
    tree = tree_hash()
    same = bool(fs and fs.get("tree_sha256") == tree)
    ok = bool(fs and fs.get("all_green") is True and same)
    out.append({"name": "fixtures_green", "ok": ok or dry, "gating": not dry, "tree_sha256": tree,
                "detail": f"{p.name}: " + (f"{fs.get('t_iso')} all_green={fs.get('all_green')}, written for "
                                           f"{'this code' if same else 'OTHER code'} (tree {tree[:12]})" if fs else "missing")
                          + (" (a dry run does not need it)" if dry and not ok else "")})
    return out


def hand_mapping(dry: bool, ask, snapshot_fn) -> dict:
    """Dawn protocol step 1: the palm on spawn cross 6 must read within 15 mm of the cross (the live monitor)."""
    if dry:
        return {"name": "hand_mapping", "ok": True, "detail": "dry run: the simulated feed has no camera mapping to check"}
    ask("  HAND MAPPING: lay one hand flat, palm up, palm centred on spawn cross 6. Hold still, then press Enter: ", "")
    msg = (snapshot_fn() or {}).get("msg") or {}
    hands = msg.get("hands") or []
    if len(hands) != 1:
        return {"name": "hand_mapping", "ok": False, "detail": f"the monitor sees {len(hands)} hand(s); one is needed"}
    d = float(np.hypot(hands[0]["palm_mm"][0] - CROSS6_MM[0], hands[0]["palm_mm"][1] - CROSS6_MM[1]))
    ask("  Take the hand OUT of the workspace, then press Enter: ", "")
    return {"name": "hand_mapping", "ok": d <= MAP_TOL_MM, "palm_mm": hands[0]["palm_mm"], "off_mm": round(d, 1),
            "frame_sha256": msg.get("frame_sha256"),
            "detail": f"palm reads {hands[0]['palm_mm']} mm, {d:.1f} mm from cross 6 {CROSS6_MM} (<= {MAP_TOL_MM:g})"}


def pre(root: Path, sessions: Path, reg: dict, record: dict, dry: bool, now: datetime.datetime | None = None) -> dict:
    """The checks that open nothing and ask nothing: read before the camera or the port is touched."""
    checks = frozen_files(root, reg) + from_record(root, record, dry) + this_demo(sessions, dry)
    t = (now or datetime.datetime.now(g3.IST)).astimezone(g3.IST)
    checks.append({"name": "motion_hours", "ok": dry or motion_hours_ok(now),
                   "detail": f"{t:%H:%M} IST ({hours_text()})"})
    checks.append({"name": "base_guard", "ok": True, "gating": False,
                   "detail": "NOT RUN: no base guard is run for this demonstration (PR-001's guard is a motion step of "
                             "the certificate's sessions; stated here, not claimed)"})
    return {"ok": all(c["ok"] for c in checks), "checks": checks, "t_iso": g3.now_iso(), "dry_run": dry}


def live(root: Path, session_dir: Path, dry: bool, ask, say=print, grab=None) -> dict:
    """The framing re-check (the camera) and the ruler + clamp prompts. Nothing moves."""
    try:
        checks = [framing(root, session_dir, dry, grab, say)]
    except Exception as e:  # noqa: BLE001 - a re-check that cannot run is a failed re-check
        checks = [{"name": "framing_recheck", "ok": False, "detail": f"could not run: {type(e).__name__}: {e}"}]
    checks += ruler_and_clamps(root, dry, ask)
    return {"ok": all(c["ok"] for c in checks), "checks": checks, "t_iso": g3.now_iso(), "dry_run": dry}
