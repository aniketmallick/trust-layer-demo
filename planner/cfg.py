"""What the handover planner plans against, loaded once from the frozen files (read-only): the registration r2
(arm frame, zero pose, sign map), the calibration file (checked against the registration's hash and against the
guard's ranges) and the G3 guard itself (g3.guard_from_record: joint limits = calibrated range - 3 deg, the
workspace box). The dawn protocol sets z_handover_mm, the standoff and the knife: with_dawn()."""
from __future__ import annotations

import dataclasses
import json
from dataclasses import dataclass
from pathlib import Path

from common import paths

paths.use_anchor()
import g3  # noqa: E402
import null_plan  # noqa: E402
from safety import DEG_PER_TICK, load_calibration_file  # noqa: E402

from planner.knife import Knife  # noqa: E402

VISIBLE_INSET_PX = 10.0            # the footprint is the image less this border (a hand cut by the frame edge is
#                                   seen late or not at all)
IMAGE_WH = (640, 480)
BASE_XY_MM = (38.8, 0.0)            # the pan axis in the arm frame (fk.py: the shoulder body; null_plan.ik_offset)


@dataclass(frozen=True)
class HandoverCfg:
    q_zero: tuple                    # lerobot deg at the MJCF zero pose (6)
    table_z_mm: float
    lim_lo: tuple                    # lerobot deg / %, 6 joints: the guard's limits (calibrated range - 3 deg)
    lim_hi: tuple
    box_xy: tuple                    # (x0, x1, y0, y1) arm mm: the guard's workspace box
    z_lo_mm: float
    z_hi_mm: float
    knife: Knife = Knife()
    standoff_mm: float = 60.0                             # handle tip <-> palm, on the table plane
    standoff_margin_mm: float = 2.0                       # aimed beyond the standoff: IK error and rounding
    point_min_mm: float = 150.0                           # the pointed end <-> palm, every row of every plan with a
    #                                                       MEASURED tool (operator, 2026-10-03). The placeholder (a 20 mm
    #                                                       'blade' for rehearsals; --arm refuses it) cannot meet it anywhere.
    z_handover_mm: float = null_plan.TIP_CARRY_MM         # 30: fingertip above the table at the hold (set at dawn)
    z_handover_set: bool = False
    z_transit_mm: float = null_plan.TIP_RETREAT_MM        # 65: the null's own height over the sheets
    speed_dps: float = 3.0                                # every arm joint, every handover segment (PLAN.md F3)
    control_hz: float = 15.0
    arm: bool = False                                     # True on --arm: z_handover_mm must have been set
    ik_method: str = "sim"                                # "sim" (SO101Kinematics.solve) or "null" (null_plan.ik)
    box_inner_mm: float = 5.0                             # planned points stay this far inside the box (lag, noise)
    joint_inner_deg: float = 2.0                          # waypoints stay this far inside the joint limits (the
    #                                                       measured joint sags and jitters: A5, the elbow ~1.7 deg)
    min_body_mm: float = null_plan.MIN_BODY_MM            # 15: every moving body above the table
    min_knife_mm: float = 10.0                            # the knife's two ends above the table, after the rise
    max_ik_err_mm: float = null_plan.MAX_IK_POS_ERR_MM    # 1.0
    max_tilt_deg: float = 5.0                             # the knife from horizontal at every waypoint
    hold_rows: int = null_plan.HOLD_STEPS                 # 5
    base_xy: tuple = BASE_XY_MM
    registration_sha256: str = ""
    visible_quad: tuple = ()                              # the camera's footprint on the table, arm mm, inset by
    #                                                       VISIBLE_INSET_PX: planned points stay inside it as well as
    #                                                       inside the box (reviewer 2026-10-01, question 11). () = no cut

    def to_mjcf(self, q_lerobot) -> list:
        """lerobot deg -> MJCF deg, the arm's 5 joints (sign map all +1: checked at load)."""
        return [float(q_lerobot[i]) - self.q_zero[i] for i in range(5)]

    def to_lerobot(self, q5_mjcf) -> list:
        return [float(q5_mjcf[i]) + self.q_zero[i] for i in range(5)]

    def mjcf_limits(self) -> tuple:
        """What the IK and every waypoint must stay inside: the guard's limits less joint_inner_deg, MJCF deg."""
        m = self.joint_inner_deg
        return ([self.lim_lo[i] + m - self.q_zero[i] for i in range(5)], [self.lim_hi[i] - m - self.q_zero[i] for i in range(5)])

    def describe(self) -> dict:
        d = dataclasses.asdict(self)
        return {k: (list(v) if isinstance(v, tuple) else v) for k, v in d.items()}


def load_cfg(root: Path | None = None, **overrides) -> HandoverCfg:
    """The planner's inputs for this rig. Refuses (ValueError) on a registration or calibration file that is not
    the one of record, a sign map that is not all +1, or guard ranges that are not the calibration file's."""
    root = Path(root) if root is not None else paths.ROOT
    reg = json.loads((root / paths.REG_REL).read_text(encoding="utf-8"))
    if g3.canonical_hash(reg) != reg.get("sha256") or reg["sha256"] != paths.REG_SHA256:
        raise ValueError(f"{paths.REG_REL} is not the registration of record ({paths.REG_SHA256[:12]})")
    signs = (reg.get("joint_sign_map") or {}).get("map") or {}
    if any(int(signs.get(j, 0)) != 1 for j in g3.ALL):
        raise ValueError(f"joint_sign_map is not all +1 ({signs}): fk, the guard and this planner assume it")
    cal_p = root / reg["arm_calibration"]["file"]
    if g3.sha_file(cal_p) != reg["arm_calibration"]["sha256"]:
        raise ValueError(f"{cal_p.name} is not the calibration the registration names")
    cal = load_calibration_file(cal_p)
    record = json.loads((root / paths.RECORD_REL).read_text(encoding="utf-8"))
    guard = g3.guard_from_record(root, reg, record)
    for i, j in enumerate(g3.ARM):                         # the guard's ranges (A1) are the calibration file's
        half = (int(cal[j]["range_max"]) - int(cal[j]["range_min"])) / 2.0 * DEG_PER_TICK
        if abs(guard.ranges[i][1] - half) > 0.05 or abs(guard.ranges[i][0] + half) > 0.05:
            raise ValueError(f"{j}: the guard's range {guard.ranges[i]} is not the calibration file's (+/-{half:.2f})")
    lim = guard.limits()
    arm = reg["arm_frame"]
    cfg = HandoverCfg(q_zero=tuple(float(v) for v in arm["q_zero_lerobot_deg"]), table_z_mm=float(arm["table_z_mm"]),
                      lim_lo=tuple(float(lim[j][0]) for j in g3.ALL), lim_hi=tuple(float(lim[j][1]) for j in g3.ALL),
                      box_xy=tuple(float(v) for v in guard.box), z_lo_mm=float(guard.z_lo), z_hi_mm=float(guard.z_hi),
                      registration_sha256=str(reg["sha256"]), visible_quad=visible_quad(reg, guard.box))
    return dataclasses.replace(cfg, **overrides) if overrides else cfg


def visible_quad(reg: dict, box) -> tuple:
    """The image, less VISIBLE_INSET_PX at each edge, through the registration's map onto the table plane (arm mm)."""
    from common.table_map import table_map_from
    tm = table_map_from(reg, box)
    w, h, m = IMAGE_WH[0] - 1, IMAGE_WH[1] - 1, VISIBLE_INSET_PX
    return tuple(tuple(round(float(v), 2) for v in tm.px_to_mm(px)) for px in ((m, m), (w - m, m), (w - m, h - m), (m, h - m)))


# The lowest standoff --standoff-mm may set. The specification's 60 mm, lowered by the operator on 2026-10-03 after
# step 6 (KH-S20261003T130937): with the 167 mm screwdriver in this workspace box, 1 of 29 handover requests planned at
# 80 mm, 7 at 60, 21 at 30. The default stays 60. The arm rides 12-20 mm off its commanded pose with the tool
# (STATUS, day-2 finding): at 30 commanded the real handle-tip gap can be ~10 mm.
STANDOFF_MIN_MM = 30.0


def with_dawn(cfg: HandoverCfg, z_handover_mm: float, standoff_mm: float | None = None,
              knife: Knife | None = None) -> HandoverCfg:
    """The values set with the operator's hand on the table (dawn protocol, step 3). The standoff never goes below
    STANDOFF_MIN_MM."""
    if standoff_mm is not None and float(standoff_mm) < STANDOFF_MIN_MM:
        raise ValueError(f"standoff {standoff_mm} mm is below the {STANDOFF_MIN_MM:g} mm minimum")
    return dataclasses.replace(cfg, z_handover_mm=float(z_handover_mm), z_handover_set=True,
                               standoff_mm=cfg.standoff_mm if standoff_mm is None else float(standoff_mm),
                               knife=cfg.knife if knife is None else knife)
