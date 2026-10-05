"""From knife waypoints to joint rows, and the checks every row must pass.

A waypoint says where the knife's grip point is on the table plane, how high the fingertip is, and which way the
handle points. Each is solved with a vertical approach (the knife horizontal): the fingertip target follows from the
grip point and the heading, the wrist roll in closed form from the FK heading at roll 0 - iterated, because the
fingertip sits 12 mm off the roll axis and the other four joints move with it. Rows between waypoints are joint-space
steps (null_plan.interpolate), every arm joint at or under the speed cap. Every row is then read back through
anchor/fk.py: limits, the workspace box, heights above the table, and the knife's and the arm's distance to the palm.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from common import paths

paths.use_anchor()
import fk  # noqa: E402
import null_plan  # noqa: E402

from planner import ik  # noqa: E402
from planner.cfg import HandoverCfg  # noqa: E402
from planner.knife import knife_pose  # noqa: E402

STEP_MM = 15.0               # waypoint spacing along a move
STEP_HEADING_DEG = 8.0       # and along a turn
ROLL_ROUNDS = 7
ROLL_DONE_DEG = 0.02
ROUND_MARGIN_DEG = 0.001     # rows are rounded to 3 decimals: the interpolation cap leaves room for it
ARM_CHAIN = ("shoulder", "upper_arm", "lower_arm", "wrist", "gripper", "gripperframe")


@dataclass(frozen=True)
class Way:
    grip_xy: tuple           # the knife's grip point, arm mm
    h_mm: float              # fingertip above the table
    heading_deg: float       # the handle's heading, continuous along a path (not wrapped)


def unit(v) -> np.ndarray:
    v = np.asarray(v, float)[:2]
    return v / max(1e-9, float(np.linalg.norm(v)))


def nearest_turn(a_deg: float, ref_deg: float) -> float:
    """a + k * 360 closest to ref."""
    return a_deg + 360.0 * round((ref_deg - a_deg) / 360.0)


def seg_dist(p, a, b) -> float:
    """Distance on the table plane from point p to the segment a-b."""
    p, a, b = (np.asarray(v, float)[:2] for v in (p, a, b))
    ab = b - a
    t = 0.0 if float(ab @ ab) < 1e-12 else max(0.0, min(1.0, float((p - a) @ ab) / float(ab @ ab)))
    return float(np.linalg.norm(p - (a + t * ab)))


def tip_target(cfg: HandoverCfg, way: Way) -> np.ndarray:
    """The fingertip for this waypoint: the grip point moved grip_offset back along the jaw axis. With the gripper's
    z up and the handle along s * y, the jaw axis (body x = y cross z) is s * (d_y, -d_x) on the table plane."""
    th = math.radians(way.heading_deg)
    jaw = cfg.knife.axis_sign * np.array([math.sin(th), -math.cos(th)])
    g = np.asarray(way.grip_xy, float) - cfg.knife.grip_offset_mm * jaw
    return np.array([g[0], g[1], cfg.table_z_mm + way.h_mm])


def solve_way(cfg: HandoverCfg, way: Way, seed5) -> tuple:
    """-> (q5 MJCF deg, position error mm, tilt deg). The roll is the turn nearest the seed's that gives the heading;
    it may come back outside its limits - the row check decides."""
    tgt = tip_target(cfg, way)
    y_heading = way.heading_deg + (0.0 if cfg.knife.axis_sign > 0 else 180.0)
    roll, seed = float(seed5[4]), list(seed5)
    for _ in range(ROLL_ROUNDS):
        q, pe, tilt = ik.solve(tgt, roll, [seed], cfg.mjcf_limits(), cfg.ik_method)
        at0 = knife_pose(list(q[:4]) + [0.0], cfg.knife)             # the same four joints at roll 0
        y0 = at0.heading_deg + (0.0 if cfg.knife.axis_sign > 0 else 180.0)
        new = nearest_turn(y_heading - y0, roll)
        if abs(new - roll) < ROLL_DONE_DEG:
            break
        roll, seed = new, list(q)
    return q, pe, tilt


def pose_at(cfg: HandoverCfg, way: Way, gripper_pct: float, seed5=None) -> list:
    """A lerobot pose (6) holding the knife at this waypoint - for fixtures and the dry run's start poses.
    Raises if there is none."""
    q, pe, tilt = solve_way(cfg, way, list(seed5) if seed5 is not None else list(ik.SEEDS[0]) + [0.0])
    if pe > cfg.max_ik_err_mm or tilt > cfg.max_tilt_deg:
        raise ValueError(f"no pose at {way} (IK error {pe:.1f} mm, tilt {tilt:.1f} deg)")
    return [round(v, 3) for v in cfg.to_lerobot(q)] + [float(gripper_pct)]


def subdivide(a: Way, b: Way) -> list:
    """a -> b in equal parts no longer than STEP_MM / STEP_HEADING_DEG (b included, a not)."""
    d = float(np.linalg.norm(np.subtract(b.grip_xy, a.grip_xy)))
    n = int(max(1, math.ceil(max(d / STEP_MM, abs(b.h_mm - a.h_mm) / STEP_MM,
                                 abs(b.heading_deg - a.heading_deg) / STEP_HEADING_DEG) - 1e-9)))
    out = []
    for k in range(1, n + 1):
        f = k / n
        g = np.asarray(a.grip_xy, float) * (1 - f) + np.asarray(b.grip_xy, float) * f
        out.append(Way((float(g[0]), float(g[1])), a.h_mm + (b.h_mm - a.h_mm) * f,
                       a.heading_deg + (b.heading_deg - a.heading_deg) * f))
    return out


def precheck(cfg: HandoverCfg, segments: list, palm_mm) -> str | None:
    """The waypoints' own geometry, before any IK: the fingertip and both ends of the knife inside the box, the knife
    no closer to the palm than the standoff. Cheap, and it rejects most routes that the row check would reject."""
    palm = np.asarray(palm_mm, float)[:2]
    for label, ways in segments:
        for w in ways:
            th = math.radians(w.heading_deg)
            d, g = np.array([math.cos(th), math.sin(th)]), np.asarray(w.grip_xy, float)
            ends = {"fingertip": tip_target(cfg, w), "handle tip": g + cfg.knife.handle_len_mm * d,
                    "blade tip": g - cfg.knife.blade_overhang_mm * d}
            out = [n for n, p in ends.items() if not _in_box(cfg, p, 0.0)]
            if out:
                return f"{label}: the {out[0]} would leave the workspace box at ({ends[out[0]][0]:.0f}, {ends[out[0]][1]:.0f}) mm"
            near = seg_dist(palm, ends["blade tip"], ends["handle tip"])
            if near < cfg.standoff_mm:
                return f"{label}: the knife would come within {near:.0f} mm of the palm (standoff {cfg.standoff_mm:g})"
    return None


def build_rows(cfg: HandoverCfg, q_now_lerobot6, segments: list) -> dict:
    """segments: [(label, [Way, ...])], solved in order from the present pose. -> {"rows", "phases", "ik", "problem"}:
    rows are lerobot degrees + the gripper held at its present value, one per control step."""
    q_now = [float(v) for v in q_now_lerobot6]
    cap = cfg.speed_dps / cfg.control_hz - ROUND_MARGIN_DEG
    caps = [cap] * 5 + [null_plan.GRIP_DELTA_PCT]
    rows, phases, cur, seed = [], [], list(q_now), cfg.to_mjcf(q_now)
    worst = {"pos_err_mm": 0.0, "tilt_deg": 0.0, "waypoints": 0}
    for label, ways in segments:
        n0 = len(rows)
        for w in ways:
            q, pe, tilt = solve_way(cfg, w, seed)
            worst = {"pos_err_mm": max(worst["pos_err_mm"], pe), "tilt_deg": max(worst["tilt_deg"], tilt),
                     "waypoints": worst["waypoints"] + 1}
            if pe > cfg.max_ik_err_mm or tilt > cfg.max_tilt_deg:
                return {"rows": rows, "phases": phases, "ik": worst,
                        "problem": (f"{label}: no pose with the knife horizontal at grip point ({w.grip_xy[0]:.0f}, "
                                    f"{w.grip_xy[1]:.0f}) mm, {w.h_mm:.0f} mm up (IK error {pe:.1f} mm, tilt {tilt:.1f} "
                                    f"deg; limits {cfg.max_ik_err_mm:g} mm, {cfg.max_tilt_deg:g} deg)")}
            lo, hi = cfg.mjcf_limits()
            bad = [i for i in range(5) if not (lo[i] <= q[i] <= hi[i])]
            if bad:
                return {"rows": rows, "phases": phases, "ik": worst,
                        "problem": (f"{label}: {fk.JOINT_NAMES[bad[0]]} would need {q[bad[0]] + cfg.q_zero[bad[0]]:.0f} deg, "
                                    f"outside its limit [{cfg.lim_lo[bad[0]]:.0f}, {cfg.lim_hi[bad[0]]:.0f}] less "
                                    f"{cfg.joint_inner_deg:g} deg")}
            tgt = cfg.to_lerobot(q) + [q_now[5]]
            rows.extend(null_plan.interpolate(cur, tgt, caps))
            cur, seed = tgt, list(q)
        phases.append({"phase": label, "from_step": n0 + 1, "n_steps": len(rows) - n0})
    return {"rows": [[round(float(x), 3) for x in r] for r in rows], "phases": phases,
            "ik": {k: round(v, 3) if isinstance(v, float) else v for k, v in worst.items()}, "problem": None}


def hold_rows(rows: list, phases: list, q_now_lerobot6, n: int) -> None:
    """The plan ends holding its last row (or the present pose, if nothing moved) for n control steps."""
    last = list(rows[-1]) if rows else [round(float(v), 3) for v in q_now_lerobot6]
    phases.append({"phase": "hold", "from_step": len(rows) + 1, "n_steps": n})
    rows.extend([list(last) for _ in range(n)])


def _in_box(cfg: HandoverCfg, p, margin: float) -> bool:
    """Inside the workspace box (less margin) AND inside the camera's footprint: the allowed motion region."""
    x0, x1, y0, y1 = cfg.box_xy
    return bool(x0 + margin <= p[0] <= x1 - margin and y0 + margin <= p[1] <= y1 - margin) and _visible(cfg, p)


def _visible(cfg: HandoverCfg, p) -> bool:
    """Inside the camera's footprint on the table (cfg.visible_quad; empty = no cut). Convex, either winding."""
    q = cfg.visible_quad
    if not q:
        return True
    s = [(q[(i + 1) % 4][0] - q[i][0]) * (p[1] - q[i][1]) - (q[(i + 1) % 4][1] - q[i][1]) * (p[0] - q[i][0])
         for i in range(4)]
    return all(v >= 0 for v in s) or all(v <= 0 for v in s)


def check_rows(cfg: HandoverCfg, rows: list, phases: list, palm_mm, q_start_lerobot6) -> dict:
    """Every row through anchor/fk.py. -> {"problem": first failure or None, + the worst value of each term}.
    The knife's end heights are judged after the first phase (the rise): a pose frozen mid-carry may be tilted."""
    palm = np.asarray(palm_mm, float)[:2]
    after_rise = phases[0]["n_steps"] if phases else 0
    st = {"min_knife_palm_mm": float("inf"), "min_point_palm_mm": float("inf"), "min_arm_palm_mm": float("inf"),
          "min_body_mm": float("inf"),
          "min_knife_end_mm": float("inf"), "max_step_deg": 0.0, "n_rows": len(rows), "problem": None}
    prev = [float(v) for v in q_start_lerobot6]

    def fail(k, why):
        ph = next((p["phase"] for p in phases if p["from_step"] <= k + 1 < p["from_step"] + p["n_steps"]), "?")
        st["problem"] = f"row {k + 1} ({ph}): {why}"
        return {key: (round(v, 2) if isinstance(v, float) and math.isfinite(v) else v) for key, v in st.items()}

    for k, row in enumerate(rows):
        st["max_step_deg"] = max(st["max_step_deg"], max(abs(row[i] - prev[i]) for i in range(5)))
        prev = row
        for i in range(5):
            if not (cfg.lim_lo[i] <= row[i] <= cfg.lim_hi[i]):
                return fail(k, f"{fk.JOINT_NAMES[i]} at {row[i]:.1f} deg outside its limit "
                               f"[{cfg.lim_lo[i]:.1f}, {cfg.lim_hi[i]:.1f}]")
        kp = knife_pose(cfg.to_mjcf(row), cfg.knife)
        if not (_in_box(cfg, kp.tip, 0.0) and cfg.z_lo_mm <= kp.tip[2] <= cfg.z_hi_mm):
            return fail(k, f"the fingertip at ({kp.tip[0]:.0f}, {kp.tip[1]:.0f}, {kp.tip[2]:.0f}) mm leaves the workspace box")
        for name, p in (("handle tip", kp.handle_tip), ("blade tip", kp.blade_tip)):
            if not _in_box(cfg, p, 0.0):
                return fail(k, f"the {name} at ({p[0]:.0f}, {p[1]:.0f}) mm leaves the workspace box")
        body = min(float(kp.frames[b][2, 3]) * 1000.0 - cfg.table_z_mm for b in null_plan.BODY_POINTS)
        st["min_body_mm"] = min(st["min_body_mm"], body)
        if body < cfg.min_body_mm:
            return fail(k, f"a moving body comes within {body:.0f} mm of the table (< {cfg.min_body_mm:g})")
        if k >= after_rise:
            end = min(kp.handle_tip[2], kp.blade_tip[2]) - cfg.table_z_mm
            st["min_knife_end_mm"] = min(st["min_knife_end_mm"], float(end))
            if end < cfg.min_knife_mm:
                return fail(k, f"an end of the knife comes within {end:.0f} mm of the table (< {cfg.min_knife_mm:g})")
        d_knife = seg_dist(palm, kp.blade_tip, kp.handle_tip)
        st["min_knife_palm_mm"] = min(st["min_knife_palm_mm"], d_knife)
        if d_knife < cfg.standoff_mm:
            return fail(k, f"the knife comes within {d_knife:.0f} mm of the palm (standoff {cfg.standoff_mm:g})")
        d_point = float(np.linalg.norm(np.asarray(kp.blade_tip, float)[:2] - palm))
        st["min_point_palm_mm"] = min(st["min_point_palm_mm"], d_point)
        if d_point < cfg.point_min_mm and not cfg.knife.placeholder:
            return fail(k, f"the pointed end comes within {d_point:.0f} mm of the palm (< {cfg.point_min_mm:g})")
        pts = [kp.frames[b][:3, 3] * 1000.0 for b in ARM_CHAIN]
        d_arm = min(seg_dist(palm, a, b) for a, b in zip(pts[:-1], pts[1:]))
        st["min_arm_palm_mm"] = min(st["min_arm_palm_mm"], d_arm)
        if d_arm < cfg.standoff_mm:
            return fail(k, f"the arm passes within {d_arm:.0f} mm of the palm on the table plane (standoff "
                           f"{cfg.standoff_mm:g})")
    if st["max_step_deg"] > cfg.speed_dps / cfg.control_hz + 1e-9:
        st["problem"] = f"a step of {st['max_step_deg']:.4f} deg exceeds {cfg.speed_dps:g} deg/s at {cfg.control_hz:g} Hz"
    return {key: (round(v, 4) if isinstance(v, float) and math.isfinite(v) else v) for key, v in st.items()}
