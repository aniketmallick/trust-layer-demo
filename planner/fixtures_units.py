"""Fixtures for the pieces under the plan: the two IKs, the knife, the cfg loader, and the read-only rule."""
from __future__ import annotations

import os
import shutil
import tempfile
import time
from pathlib import Path

import numpy as np

from common import paths
from planner import fixtures as fx
from planner import handover_planner as hp
from planner import ik
from planner.fixtures import _both, fixture
from planner.knife import Knife, close_target_pct, knife_axis_from_grasp

N_IK = 40


@_both("IK", "IK_40_fingertip_targets_through_anchor_fk",
       "40 seeded fingertip targets over the sheets (x 180-280, y -150..130 mm, 30-65 mm up, roll -150..150 deg), "
       "vertical approach: anchor/fk.py puts the fingertip within 1 mm and the approach within 1 deg of vertical for "
       "every one", kind="control")
def _ik_targets(method):
    cfg = fx.cfg_for(method)
    rng = np.random.default_rng(fx.SEED)
    pos, tilt, secs = [], [], []
    for _ in range(N_IK):
        tgt = [rng.uniform(180, 280), rng.uniform(-150, 130), cfg.table_z_mm + rng.uniform(30, 65)]
        t0 = time.perf_counter()
        _q, pe, tl = ik.solve(tgt, float(rng.uniform(-150, 150)), None, cfg.mjcf_limits(), method)
        secs.append(time.perf_counter() - t0)
        pos.append(pe)
        tilt.append(tl)
    return (max(pos) < 1.0 and max(tilt) < 1.0), (f"position error max {max(pos):.4f} / median {np.median(pos):.4f} mm; "
                                                    f"tilt max {max(tilt):.3f} deg; {1000 * np.median(secs):.0f} ms median, "
                                                    f"{1000 * max(secs):.0f} ms max per solve (cold seeds)")


@_both("IK", "IK_beyond_reach_reports_its_error",
       "a fingertip target 420 mm out at 65 mm up (beyond the arm with the gripper vertical): the solver returns its "
       "best pose with the error it has, measured through anchor/fk.py - it never claims a pose it does not reach")
def _ik_far(method):
    cfg = fx.cfg_for(method)
    _q, pe, tl = ik.solve([420.0, 0.0, cfg.table_z_mm + 65.0], 0.0, None, cfg.mjcf_limits(), method)
    return (pe > cfg.max_ik_err_mm or tl > cfg.max_tilt_deg), f"position error {pe:.1f} mm, tilt {tl:.1f} deg"


@fixture("KNIFE", "KNIFE_axis_sign_from_the_grasp",
         "a top-down pose with the gripper's y axis along the radial line: a handle declared radially outward gives "
         "axis sign +1, declared toward the base -1, and a heading 45 deg off the y axis is refused (the jaws would "
         "not be closing across the knife)")
def _axis():
    cfg, q = fx.cfg_for("sim"), list(fx.start_pose("sim"))
    out_deg = fx.radial_deg(fx.MID_CARRY_MM)
    a, b = knife_axis_from_grasp(q, out_deg, cfg), knife_axis_from_grasp(q, out_deg + 180.0, cfg)
    try:
        knife_axis_from_grasp(q, out_deg + 45.0, cfg)
        refused = False
    except ValueError:
        refused = True
    return (a["axis_sign"] == 1 and b["axis_sign"] == -1 and a["off_axis_deg"] < 1.0 and refused), (
        f"outward {a}, toward the base {b['axis_body']}, 45 deg off refused: {refused}")


@fixture("KNIFE", "KNIFE_close_target_from_the_caliper",
         "handle 20 mm wide: contact 20 / 1.4383 = 13.91 %, target 9.91 % (contact - 4 %); a 3 mm blade: the target is "
         "floored at the guard's gripper floor + 0.5 %; both are labelled placeholder", kind="control")
def _close():
    floor = fx.cfg_for("sim").lim_lo[5]
    a, b = close_target_pct(20.0, floor), close_target_pct(3.0, floor)
    ok = (abs(a["contact_pct"] - 13.91) < 0.01 and abs(a["target_pct"] - 9.91) < 0.01 and not a["floored"]
          and b["floored"] and abs(b["target_pct"] - (floor + 0.5)) < 0.01 and a["placeholder"] and b["placeholder"])
    return ok, f"20 mm -> {a['target_pct']} % (contact {a['contact_pct']} %); 3 mm -> {b['target_pct']} % (floor {floor:.2f} %)"


@fixture("KNIFE", "KNIFE_longer_handle_keeps_the_standoff",
         "the handle measured 140 mm at dawn instead of the 100 mm placeholder: the grip point ends 40 mm farther from "
         "the palm and the handle tip still 62 mm from it", kind="control")
def _longer():
    cfg, q0 = fx.cfg_for("sim"), list(fx.start_pose("sim", (215.0, 5.0)))     # room for the longer handle in the box
    long_cfg = hp.with_dawn(cfg, cfg.z_handover_mm, knife=Knife(handle_len_mm=140.0, placeholder=False))
    palm = (350.0, 110.0)
    a, b = hp.plan_handover(q0, palm, cfg), hp.plan_handover(q0, palm, long_cfg)
    if not (a.ok and b.ok):
        return False, f"100 mm: {a.reason or 'ok'}; 140 mm: {b.reason or 'ok'}"
    da, db = (float(np.linalg.norm(np.subtract(p.grip_point_mm, palm))) for p in (a, b))
    tip = b.checks["end_handle_tip_to_palm_mm"]
    return (abs(db - da - 40.0) < 0.5 and 60.0 <= tip < 63.0 and not b.knife_placeholder and a.knife_placeholder), (
        f"grip point {da:.1f} -> {db:.1f} mm from the palm; handle tip {tip} mm; placeholder flag {a.knife_placeholder} -> "
        f"{b.knife_placeholder}")


@fixture("CFG", "CFG_limits_are_the_guards_and_an_edited_calibration_is_refused",
         "load_cfg's limits and box are g3.guard_from_record's; a tree whose calibration file differs by one tick from "
         "the one the registration names is refused")
def _cfg():
    import g3
    reg, rec = fx.rig()
    guard, cfg = g3.guard_from_record(paths.ROOT, reg, rec), fx.cfg_for("sim")
    lim = guard.limits()
    same = (all(abs(cfg.lim_lo[i] - lim[j][0]) < 1e-9 and abs(cfg.lim_hi[i] - lim[j][1]) < 1e-9
                for i, j in enumerate(g3.ALL)) and tuple(guard.box) == cfg.box_xy)
    with tempfile.TemporaryDirectory() as td:
        t = Path(td)
        for rel in (paths.REG_REL, paths.CALIBRATION_REL):
            (t / rel).parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(paths.ROOT / rel, t / rel)
        os.symlink(paths.ROOT / paths.RECORD_REL, t / paths.RECORD_REL)
        cal = t / paths.CALIBRATION_REL
        cal.write_text(cal.read_text().replace('"range_max": 3350', '"range_max": 3351'), encoding="utf-8")
        try:
            hp.load_cfg(t)
            refused, why = False, "loaded"
        except ValueError as e:
            refused, why = True, str(e)
    return (same and refused), f"limits and box equal the guard's: {same}; edited calibration: {why}"


@fixture("RUN", "RUN_frozen_folders_not_written",
         "after everything above (both IKs, the sim model built, anchor modules imported): no file under anchor/, "
         "phase0/ or so101_sim/ has a modification time later than this fixture module's import")
def _frozen():
    new = [str(p.relative_to(paths.ROOT)) for d in ("anchor", "phase0", "so101_sim")
           for p in (paths.ROOT / d).rglob("*") if p.is_file() and p.stat().st_mtime > fx.T_IMPORT]
    return (not new), f"{len(new)} file(s) written" + (f": {new[:3]}" if new else "")
