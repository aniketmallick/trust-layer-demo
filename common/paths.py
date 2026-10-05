"""Where things are. Importing this module makes the frozen folders importable without writing into them."""
from __future__ import annotations

import sys
from pathlib import Path

sys.dont_write_bytecode = True      # anchor/ and so101_sim/ are read-only: no __pycache__ writes on import

EXP = Path(__file__).resolve().parent.parent            # experiments/knife_handover
ROOT = EXP.parent.parent                                # cert_deploy_robotics
ANCHOR = ROOT / "anchor"
SIM = ROOT / "so101_sim"

REG_REL = "phase0/registration_2026-09-24_r2.json"
LAYOUT_REL = "phase0/layout/layout_v3.json"
RECORD_REL = "phase0/anchor/anchor_measurements.json"
CALIBRATION_REL = "phase0/anchor/calibration/so101_follower_anchor.json"
NULL_PLAN_REL = "phase0/null/NULL-001_plan_9fb34cea_e3.json"
REG_SHA256 = "9fb34cea93f5dddd379fad43c9f0a371b9a1575a7dc1556f0fe8113158605c88"      # canonical, as null_session.FROZEN
LAYOUT_SHA256 = "e6e60f79d1c68753e16265b355bca546f010efd1f84277cec8105ee4df9f45c5"   # canonical

FRAME_DIR = Path("/tmp/khv")
FRAME_JPG = FRAME_DIR / "frame.jpg"
FRAME_JSON = FRAME_DIR / "frame.json"
UDP_ADDR = ("127.0.0.1", 47101)

SESSIONS = EXP / "sessions"
FIXTURE_FRAMES = EXP / "fixtures" / "frames"


def use_anchor() -> None:
    """Put anchor/ on sys.path (import by path; nothing in it is edited)."""
    for p in (str(EXP), str(ANCHOR)):
        if p not in sys.path:
            sys.path.insert(0, p)


def use_sim() -> None:
    if str(SIM) not in sys.path:
        sys.path.insert(0, str(SIM))
