"""The prop tool as the planner sees it: a straight bar held top-down in the jaws. Since 2026-10-02 the prop is a
screwdriver gripped on the handle just behind the shaft: handle_len = grip -> free handle end (offered to the palm),
blade_overhang = grip -> pointed tip (the shaft); the field names are kept.

Tonight every number is a PLACEHOLDER (placeholder=True rides into every plan); the dawn protocol measures them.
Frame facts (anchor/fk.py, anchor/null_plan.py): the fingertip (site gripperframe) is the FIXED jaw tip; the moving
jaw opens along the gripper body's +x; the approach axis is the body's -z (down in a top-down grasp). A bar gripped
across its width therefore lies along the body's y axis, its centre line handle_width / 2 from the fixed jaw tip
toward the moving jaw. Which way along y the handle points is fixed at the grasp (knife_axis_from_grasp).
A prop tool only: the marked screwdriver.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from common import paths

paths.use_anchor()
import fk  # noqa: E402
import null_plan  # noqa: E402

MM_PER_PCT = null_plan.MM_PER_PCT        # 1.4383 mm of tip gap per gripper %: A8.caliper_erratum (ruling C9)
SQUEEZE_PCT = 4.0                        # the null's measured squeeze: contact - 4 % (PR-001 erratum 3)
FLOOR_CLEAR_PCT = 0.5                    # the close target stays this far above the guard's gripper floor
MAX_AXIS_OFF_DEG = 20.0                  # the declared handle heading must be this close to the gripper's y axis


@dataclass(frozen=True)
class Knife:
    handle_len_mm: float = 100.0         # grip point -> handle tip, along the knife axis        (placeholder)
    blade_overhang_mm: float = 20.0      # grip point -> blade tip, the other way                 (placeholder)
    handle_width_mm: float = 20.0        # across the jaws at the grip point                      (placeholder)
    axis_sign: int = 1                   # handle along +y (1) or -y (-1) of the gripper body: set at the grasp
    placeholder: bool = True

    @property
    def grip_offset_mm(self) -> float:
        """Fixed jaw tip -> the knife's centre line, along the jaw axis."""
        return self.handle_width_mm / 2.0

    @property
    def sweep_mm(self) -> float:
        """The radius the knife sweeps when it turns about its grip point."""
        return max(self.handle_len_mm, self.blade_overhang_mm)


@dataclass(frozen=True)
class KnifePose:
    tip: np.ndarray            # fingertip (gripperframe), arm mm
    grip: np.ndarray           # the knife's centre line between the jaws
    handle_tip: np.ndarray
    blade_tip: np.ndarray
    handle_xy: np.ndarray      # unit vector, the handle's heading on the table plane
    heading_deg: float
    tilt_deg: float            # approach axis from straight down = the knife from horizontal, at most
    frames: dict               # anchor/fk.py body frames (m)


def knife_pose(q5_mjcf, knife: Knife) -> KnifePose:
    """Where the knife is at this arm pose (anchor/fk.py; MJCF degrees)."""
    r = fk.forward_kinematics([float(v) for v in q5_mjcf[:5]] + [0.0])
    R = r.frames["gripper"][:3, :3]
    tip = r.ee_pos_m * 1000.0
    y = float(knife.axis_sign) * R[:, 1]
    grip = tip + R[:, 0] * knife.grip_offset_mm
    n = max(1e-9, float(np.linalg.norm(y[:2])))
    return KnifePose(tip=tip, grip=grip, handle_tip=grip + y * knife.handle_len_mm,
                     blade_tip=grip - y * knife.blade_overhang_mm, handle_xy=y[:2] / n,
                     heading_deg=math.degrees(math.atan2(y[1], y[0])),
                     tilt_deg=math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2]))))), frames=r.frames)


def knife_axis_from_grasp(q_grasp_lerobot, handle_heading_world_deg: float, cfg) -> dict:
    """The knife axis in the gripper frame, from the grasp pose and the handle's declared heading at the spawn pose
    (arm frame, deg; 0 = +x, away from the robot). -> {"axis_body": (0, s, 0), "axis_sign": s, "off_axis_deg": e}.
    Raises if the declared heading is more than MAX_AXIS_OFF_DEG off the gripper's y axis: the jaws would not be
    closing across the knife."""
    q = cfg.to_mjcf(q_grasp_lerobot)
    y = fk.forward_kinematics(list(q[:5]) + [0.0]).frames["gripper"][:3, 1]
    th = math.radians(float(handle_heading_world_deg))
    c = float(y[0] * math.cos(th) + y[1] * math.sin(th)) / max(1e-9, float(np.linalg.norm(y[:2])))
    s = 1 if c >= 0 else -1
    off = math.degrees(math.acos(max(-1.0, min(1.0, abs(c)))))
    if off > MAX_AXIS_OFF_DEG:
        raise ValueError(f"the declared handle heading is {off:.0f} deg off the gripper's y axis (limit "
                         f"{MAX_AXIS_OFF_DEG:g}): at this grasp the jaws do not close across the knife")
    return {"axis_body": (0, s, 0), "axis_sign": s, "off_axis_deg": round(off, 2)}


def close_target_pct(width_mm: float, gripper_floor_pct: float) -> dict:
    """The gripper close target for a handle this wide, from the A8 caliper coefficient: contact = width / 1.4383,
    target = contact - 4 % (the null's measured squeeze), never below the guard's gripper floor + 0.5 %.
    A PLACEHOLDER until a hold at dawn verifies it (the null's lesson, erratum 3: a close target is measured)."""
    contact = float(width_mm) / MM_PER_PCT
    target = max(contact - SQUEEZE_PCT, float(gripper_floor_pct) + FLOOR_CLEAR_PCT)
    return {"width_mm": float(width_mm), "contact_pct": round(contact, 2), "target_pct": round(target, 2),
            "floored": target > contact - SQUEEZE_PCT, "mm_per_pct": MM_PER_PCT, "squeeze_pct": SQUEEZE_PCT,
            "placeholder": True, "source": "A8.caliper_erratum (1.4383 mm per %) and the null's 4 % squeeze; "
                                           "not verified with a hold"}
