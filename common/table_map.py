"""Camera pixel <-> arm-frame table mm, from the registration record (read-only), and the three table flags.

The r2 record stores no homography. It stores a similarity map, pixel -> arm-frame mm (PLAN.md F1):
    p_arm = R(yaw) * (v * s, u * s) + t        s = a2b.mm_per_native_px, yaw / t = arm_frame.registration
the same map as null_session.CamMap (tested equal). No lens undistortion, as there. A point h mm above the table is
moved toward the camera nadir by (H - h) / H, H = fx_phys * s (347 mm), as CamMap.cube_centre does.
numpy only: imported by the runner (lerobot venv) and by the hand monitor (its own venv).
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

PALM_H_MM = 20.0          # a hand lying on the table, palm up: the palm centre above the table
HAND_H_MAX_MM = 150.0     # in_workspace is true if the palm is in the box at ANY height 0..this (one camera: no height)
QUAD_ORDER = ("near-left", "near-right", "far-right", "far-left")


def _inside(quad: np.ndarray, p) -> bool:
    """p inside the convex quad (either winding)."""
    n = len(quad)
    s = [(quad[(i + 1) % n][0] - quad[i][0]) * (p[1] - quad[i][1])
         - (quad[(i + 1) % n][1] - quad[i][1]) * (p[0] - quad[i][0]) for i in range(n)]
    return all(v >= 0 for v in s) or all(v <= 0 for v in s)


@dataclass(frozen=True)
class TableMap:
    yaw_deg: float
    t_mm: tuple
    mm_per_px: float
    nadir_px: tuple            # (cx, cy)
    cam_height_mm: float
    box_xy: tuple              # G3 workspace box (x_min, x_max, y_min, y_max), arm mm, margin included
    spawn_quad: tuple          # arm mm, QUAD_ORDER
    zone_quad: tuple
    registration_sha256: str

    @property
    def _R(self) -> np.ndarray:
        th = math.radians(self.yaw_deg)
        return np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])

    def px_to_mm(self, px, h_mm: float = 0.0) -> np.ndarray:
        """Pixel (u, v) -> arm-frame (x, y) mm of a point h_mm above the table."""
        p = self._R @ np.array([float(px[1]) * self.mm_per_px, float(px[0]) * self.mm_per_px]) + np.array(self.t_mm)
        if h_mm:
            c = self.px_to_mm(self.nadir_px)
            p = c + (p - c) * (self.cam_height_mm - float(h_mm)) / self.cam_height_mm
        return p

    def mm_to_px(self, p_mm) -> tuple:
        """Arm-frame table point -> pixel (u, v)."""
        v = np.linalg.solve(self._R, np.asarray(p_mm, float)[:2] - np.array(self.t_mm))
        return float(v[1] / self.mm_per_px), float(v[0] / self.mm_per_px)

    def in_box(self, p_mm) -> bool:
        x0, x1, y0, y1 = self.box_xy
        return bool(x0 <= p_mm[0] <= x1 and y0 <= p_mm[1] <= y1)

    def flags(self, palm_px) -> dict:
        """palm_mm (hand on the table) and the three flags for one palm pixel."""
        lo, hi = self.px_to_mm(palm_px, 0.0), self.px_to_mm(palm_px, HAND_H_MAX_MM)
        on_segment = any(self.in_box(lo + (hi - lo) * k / 10.0) for k in range(11))
        p = self.px_to_mm(palm_px, PALM_H_MM)
        return {"palm_mm": [round(float(p[0]), 1), round(float(p[1]), 1)], "in_workspace": bool(on_segment),
                "in_spawn": _inside(np.array(self.spawn_quad), p), "in_zone": _inside(np.array(self.zone_quad), p)}


def table_map_from(reg: dict, box_xy: tuple) -> TableMap:
    arm, cam = reg["arm_frame"], reg["camera"]["a2_of_record"]
    s = float(reg["a2b"]["mm_per_native_px"])
    corners = {c["name"]: tuple(c["arm_mm"][:2]) for c in arm["targets_arm_mm"]["corners"]}
    return TableMap(yaw_deg=float(arm["registration"]["yaw_deg"]), t_mm=tuple(float(v) for v in arm["registration"]["t_mm"]),
                    mm_per_px=s, nadir_px=(float(cam["intrinsics"]["cx"]), float(cam["intrinsics"]["cy"])),
                    cam_height_mm=float(cam["fx_phys"]) * s, box_xy=tuple(float(v) for v in box_xy),
                    spawn_quad=tuple(corners[f"spawn {k}"] for k in QUAD_ORDER),
                    zone_quad=tuple(corners[f"zone {k}"] for k in QUAD_ORDER), registration_sha256=str(reg["sha256"]))


def load_table_map(root: Path | None = None) -> TableMap:
    """The map for this rig: r2 (hash checked) and the G3 guard's own workspace box (g3.guard_from_record)."""
    from common import paths
    paths.use_anchor()
    import g3
    root = Path(root) if root is not None else paths.ROOT
    reg = json.loads((root / paths.REG_REL).read_text(encoding="utf-8"))
    if g3.canonical_hash(reg) != reg.get("sha256") or reg["sha256"] != paths.REG_SHA256:
        raise ValueError(f"{paths.REG_REL} is not the registration of record ({paths.REG_SHA256[:12]})")
    record = json.loads((root / paths.RECORD_REL).read_text(encoding="utf-8"))
    return table_map_from(reg, g3.guard_from_record(root, reg, record).box)
