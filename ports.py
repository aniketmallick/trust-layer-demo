"""The runner's three sockets for the other pieces: planner (C), judge (B), hand link (A).

The runner takes each through its constructor; with nothing passed it builds the real one here. There is no stand-in
in this file: a piece that is not on disk is a refusal with its name, never a silent substitute.
"""
from __future__ import annotations

import dataclasses
import threading
from pathlib import Path


class MissingPiece(RuntimeError):
    """A sibling module is not importable: the session refuses before anything moves."""


class PlannerPort:
    """planner/handover_planner.py behind the four calls the runner makes. One call at a time: the planner shares one
    simulation model between calls, and a re-target worker left behind by a freeze must not be inside it while the
    main thread plans (every call takes the lock)."""

    def __init__(self, root: Path, arm: bool, z_handover_mm: float | None, standoff_mm: float | None,
                 knife: dict | None = None):
        try:
            from planner import handover_planner as hp
        except Exception as e:  # noqa: BLE001
            raise MissingPiece(f"planner/handover_planner.py is not importable ({type(e).__name__}: {e})")
        self.hp = hp
        cfg = hp.load_cfg(root, arm=bool(arm))
        if z_handover_mm is not None:
            cfg = hp.with_dawn(cfg, float(z_handover_mm), standoff_mm)
        elif standoff_mm is not None:
            from planner.cfg import STANDOFF_MIN_MM
            if float(standoff_mm) < STANDOFF_MIN_MM:
                raise ValueError(f"standoff {standoff_mm} mm is below the {STANDOFF_MIN_MM:g} mm minimum")
            cfg = dataclasses.replace(cfg, standoff_mm=float(standoff_mm))
        if knife is not None:                                # measured: the placeholder flag goes
            cfg = dataclasses.replace(cfg, knife=dataclasses.replace(cfg.knife, placeholder=False, **knife))
        self.cfg = cfg
        self._lock = threading.RLock()

    def plan_handover(self, q6, palm_mm):
        with self._lock:
            return self.hp.plan_handover(q6, palm_mm, self.cfg)

    def plan_reorient(self, q6, palm_mm):
        with self._lock:
            return self.hp.plan_reorient(q6, palm_mm, self.cfg)

    def retarget(self, plan, palm_mm, q6):
        with self._lock:
            return self.hp.retarget(plan, palm_mm, q6, self.cfg)

    def plan_placement(self, q6, palm_mm, fingers_dir, finger_reach_mm=None, grip_pct=None):
        from planner import placement
        with self._lock:
            return placement.plan_placement(q6, palm_mm, fingers_dir, self.cfg, finger_reach_mm, grip_pct=grip_pct)

    def blade_toward_palm(self, q6, palm_mm) -> bool:
        with self._lock:
            return bool(self.hp.blade_toward_palm(q6, palm_mm, self.cfg))

    def describe(self) -> dict:
        d = self.cfg.describe()
        return {"planner_version": self.hp.PLANNER_VERSION, "standoff_mm": d["standoff_mm"],
                "z_handover_mm": d["z_handover_mm"], "z_handover_set": d["z_handover_set"],
                "z_transit_mm": d["z_transit_mm"], "speed_dps": d["speed_dps"], "ik_method": d["ik_method"],
                "knife": d["knife"], "arm": d["arm"]}


def real_judge(dry: bool, stub_script=None):
    """B's judge: provider 'stub' with the dry run's script, asked for explicitly and named in every row; on the arm
    the .env decides (read inside judge/, never here)."""
    try:
        from judge.judge import load
    except Exception as e:  # noqa: BLE001
        raise MissingPiece(f"judge/judge.py is not importable ({type(e).__name__}: {e})")
    return load(provider="stub", stub_script=stub_script) if dry else load()


def real_link(addr: tuple, on_stale, known_frame):
    """A's receiving half of the hand message (common/hand_link.py)."""
    try:
        from common.hand_link import HandLink
    except Exception as e:  # noqa: BLE001
        raise MissingPiece(f"common/hand_link.py is not importable ({type(e).__name__}: {e})")
    return HandLink(addr=addr, on_stale=on_stale, known_frame=known_frame)


class LayerOff(AttributeError):
    """--layer-off: a piece of the trust layer was asked for while it is off - the session stops, the arm held. An
    AttributeError, so hasattr() / getattr(default) on an Off piece answer "not there" instead of raising."""


class Off:
    """--layer-off (the operator, 2026-10-04): the judge or the handover planner, not built. describe() says so; any
    other use raises LayerOff (nothing in that mode should reach it)."""

    def __init__(self, what: str, stamp: str):
        self._what, self._stamp = what, stamp

    def describe(self) -> dict:
        return {"provider": "off", "model": None, "off": self._stamp, "detail": f"the {self._what} is off (--layer-off)"}

    def __getattr__(self, name):
        raise LayerOff(f"the {self._what} is off (--layer-off): '{name}' was asked for")


class NullLink:
    """--layer-off: no hand monitor. No message ever, monitor 8 never fires (step rows say OFF), no watchdog."""

    addr = ("127.0.0.1", 0)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def arm_watchdog(self, on: bool) -> None:
        pass

    def snapshot(self) -> dict:
        return {"msg": None, "window": [], "age_ms": None}

    def monitor8(self) -> tuple:
        return False, "OFF: no hand monitor (--layer-off)"
