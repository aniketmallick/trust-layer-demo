"""The demonstration's own session rig, built from anchor/safety.py pieces (PLAN.md section 5, Runner).

It is put together the way anchor/tests/test_anchor.py::_rig and run_anchor.Session.open_bus / safe_torque_on do it,
but it is NOT run_anchor.Session: that class writes its event log and its dry-run tree under phase0/anchor, which is
frozen. Everything this rig writes goes under the session directory in experiments/knife_handover/sessions/.
The calibration is the registration's file, read from phase0 (never written). A dry run drives FakeFeetechBus with
that same calibration, so the clamps and the G3 limits are the rig's own.
"""
from __future__ import annotations

import time
from pathlib import Path

from common import paths

paths.use_anchor()
import g3  # noqa: E402
from fake_bus import Clock, FakeFeetechBus  # noqa: E402
from fake_bus import MotorCalibration as FakeMotorCalibration  # noqa: E402
from safety import (ALL_JOINTS, Clamps, Console, EventLog, Interlock, InterlockError, KillSwitch,  # noqa: E402
                    SafeBus, SafeMover, SafetyContext, load_calibration_file, set_low_acceleration)

HEARTBEAT_S = 300.0
HOLD_TRIES = 3                   # a failed hold (goals := present) is retried
HOLD_GAP_S = 0.05


class Rig:
    def __init__(self, out: Path, root: Path, dry: bool, port: str | None = None, fast: float | None = None,
                 yes: bool = False, no_esc: bool = False):
        self.out, self.root, self.dry, self.port = Path(out), Path(root), bool(dry), port
        self.out.mkdir(parents=True, exist_ok=True)
        self.clock = Clock(scale=float(fast) if (self.dry and fast) else 1.0)
        self.console = Console(auto=bool(yes) and self.dry)       # scripted answers are ONLY allowed in a dry run
        self.events = EventLog(self.out / "safety_events.jsonl")
        self.kill = KillSwitch(self.console, self.events, clock=self.clock, esc=not no_esc)
        self.interlock = Interlock(self.console, self.events, heartbeat_s=HEARTBEAT_S)
        self.calib_path = self.root / paths.CALIBRATION_REL
        self.calibration = load_calibration_file(self.calib_path)
        self.clamps = Clamps(self.calibration, self.events)
        self.bus = None
        self.bus_is_fake = False
        self.sbus: SafeBus | None = None
        self.ctx: SafetyContext | None = None
        self.mover: SafeMover | None = None
        self.acc_snapshot = None

    # -- bus (mirrors run_anchor.Session.open_bus) ------------------------------------------------------
    def open_bus(self, rest: list) -> SafeBus:
        if self.dry:
            cal = {j: FakeMotorCalibration(int(c["id"]), int(c.get("drive_mode", 0)), int(c["homing_offset"]),
                                           int(c["range_min"]), int(c["range_max"])) for j, c in self.calibration.items()}
            self.bus = FakeFeetechBus(port="fake://so101", calibration=cal, clock=self.clock,
                                      initial_deg={j: float(rest[i]) for i, j in enumerate(ALL_JOINTS)})
            self.bus_is_fake = True
            self.bus.connect()
        else:
            if not self.port:
                raise InterlockError("--port is required with --arm")
            try:
                from lerobot.motors import Motor as LrMotor
                from lerobot.motors import MotorCalibration as LrMotorCalibration
                from lerobot.motors import MotorNormMode as LrNormMode
                from lerobot.motors.feetech import FeetechMotorsBus
            except Exception as e:  # pragma: no cover - the rig's interpreter only
                raise InterlockError(f"lerobot is not importable in this interpreter ({type(e).__name__}: {e}); "
                                     "use ~/lerobot-mps-venv/bin/python")
            motors = {j: LrMotor(i + 1, "sts3215", LrNormMode.RANGE_0_100 if j == "gripper" else LrNormMode.DEGREES)
                      for i, j in enumerate(ALL_JOINTS)}
            cal = {j: LrMotorCalibration(id=int(c["id"]), drive_mode=int(c.get("drive_mode", 0)),
                                         homing_offset=int(c["homing_offset"]), range_min=int(c["range_min"]),
                                         range_max=int(c["range_max"])) for j, c in self.calibration.items()}
            self.bus = FeetechMotorsBus(port=self.port, motors=motors, calibration=cal)
            self.bus.connect()                               # handshake: pings all 6 ids
            if not self.bus.is_calibrated:
                self.bus.disconnect(disable_torque=False)
                raise InterlockError("the calibration file does not match the limits / homing stored in the motors - "
                                     "refusing motion")
        self.sbus = SafeBus(self.bus, self.interlock, self.kill, self.clamps, self.events)
        self.ctx = SafetyContext(self.console, self.events, self.interlock, self.kill, self.sbus, self.clock, "KH")
        self.mover = SafeMover(self.sbus, self.clamps, self.ctx, self.clock, rate_hz=30.0)
        self.console.say(f"[bus] connected {'FAKE' if self.bus_is_fake else 'REAL'} bus on {self.bus.port}")
        return self.sbus

    def safe_torque_on(self) -> dict:
        """Goal := present first, so enabling torque cannot jump to a stale goal (run_anchor.Session.safe_torque_on)."""
        present = {j: float(v) for j, v in self.sbus.sync_read("Present_Position", list(ALL_JOINTS)).items()}
        self.sbus.sync_write("Goal_Position", present)
        self.sbus.enable_torque(list(ALL_JOINTS))
        self.clock.sleep(0.3)
        after = {j: float(v) for j, v in self.sbus.sync_read("Present_Position", list(ALL_JOINTS)).items()}
        jump = {j: after[j] - present[j] for j in present if abs(after[j] - present[j]) > 2.0}
        if jump:
            self.events.write(event="torque_on_jump", jump=jump)
        self.acc_snapshot = set_low_acceleration(self.sbus, list(g3.ARM))
        return after

    def read_q(self) -> list:
        q = self.sbus.sync_read("Present_Position")
        return [float(q[j]) for j in g3.ALL]

    def read_load(self) -> int:
        return int(self.sbus.read("Present_Load", "gripper", normalize=False))

    def hold_now(self, tries: int = HOLD_TRIES, gap_s: float = HOLD_GAP_S) -> tuple:
        """The freeze: every goal - gripper and arm - becomes the present position. Never raises. A hold that fails
        (the bus did not answer) is tried again, up to `tries` times `gap_s` apart. -> (held, attempts)."""
        if self.ctx is None:
            return False, 0
        for n in range(1, int(tries) + 1):
            try:
                if self.ctx._hold():
                    return True, n
            except Exception:  # noqa: BLE001 - _hold catches bus errors itself; nothing may come out of a freeze
                pass
            if n < tries:
                time.sleep(gap_s)
        return False, int(tries)

    def write_goal(self, target6) -> None:
        self.sbus.sync_write("Goal_Position", {j: self.clamps.clamp_target(j, float(target6[i]))
                                               for i, j in enumerate(g3.ALL)})

    def close(self, disable_torque: bool) -> None:
        try:
            if self.acc_snapshot is not None:
                self.acc_snapshot.restore()
        except Exception:  # noqa: BLE001 - the port is closed either way
            pass
        try:
            self.kill.disarm()
        finally:
            if self.bus is not None and self.bus.is_connected:
                self.bus.disconnect(disable_torque=disable_torque)
                self.events.write(event="bus_closed", torque_disabled=disable_torque)
            self.bus = None
