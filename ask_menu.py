"""The two menus (PLAN.md section 2). The arm is frozen when either is shown; nothing moves until a key is pressed.

  ASK          [w] wait and re-check   [h] handover   [o] re-orient then handover   [k] kill   [l] leave as is
               [c] continue - shown only when all eight monitors are clear and there is a segment to continue
  ASK_RELEASE  [r] release   [w] wait and re-check   [k] kill   [l] leave as is

A key is the exact single letter (spaces around it and its case aside). "help", "ok", "continue?" are not keys: each
is logged as a key row with accepted false and the menu is shown again.
[k] is torque off, and torque off lets the arm drop: it keeps safety.py's two confirmations (OFF, then YES), worded
as there; the session's end uses the same two (confirm_torque_off). [l] is safety.py's "leave as is": torque on, no
further commands, the session ends.
The judge's recommendation is printed beside the menu. It presses nothing: every key is the operator's, and every
key is a row carrying the menu shown, the key and the verdict that was on screen. `on_key(key)` runs at the accepted
key, before its row is written, and its result goes into that row (the runner clears the stop flag there, re-reads
the monitors for [c], and may refuse the key - 'refused' in the row).
A dry run answers from --dry-keys (its own queue: safety.Console's scripted answers also feed the ARM prompt).
"""
from __future__ import annotations

from common import paths

paths.use_anchor()
import g3  # noqa: E402

ASK_KEYS = {"w": "wait and re-check", "h": "handover", "o": "re-orient, then handover",
            "k": "kill (torque OFF: type OFF, then YES)", "l": "leave as is (torque ON, session ends)"}
RELEASE_KEYS = {"r": "release (the gripper opens)", "w": "wait and re-check",
                "k": "kill (torque OFF: type OFF, then YES)", "l": "leave as is (torque ON, session ends)"}
HOLD_FAILED = ("# THE HOLD FAILED: THE GOALS MAY NOT BE THE PRESENT POSITION. THE ARM MAY STILL MOVE TOWARD ITS LAST GOAL.\n"
               "# KEEP CLEAR. [k] (torque off) or the power switch stops it.")
KILL_PROMPT = "Torque OFF will let the arm drop under gravity. Support the arm. Type OFF to confirm: "
YES_PROMPT = "Second confirmation - type YES to disable torque now: "


class Keys:
    """Prompts through safety.Console; in a dry run with --yes the keys come from the scripted queue, echoed."""

    def __init__(self, console, script: list | None = None):
        self.console, self.script = console, list(script or [])

    def ask(self, prompt: str, default: str = "") -> str:
        if self.console.auto and self.script:
            ans = self.script.pop(0)
            self.console.say(f"{prompt}[auto] {ans}")
            return ans
        return self.console.ask(prompt, default)


def verdict_line(v: dict | None) -> str:
    if not v:
        return "  judge: no verdict on screen"
    tag = "UNSAFE by the code's reading" if v.get("unsafe") else "not unsafe by the code's reading"
    return (f"  judge ({v.get('model')}, {float(v.get('latency_s') or 0):.1f} s): recommends {str(v.get('recommend')).upper()}, "
            f"p_unsafe {v.get('p_unsafe')} - {v.get('reason')}  [{tag}; a recommendation, not a command]")


def on_screen(v: dict | None) -> dict | None:
    return None if not v else {k: v.get(k) for k in ("recommend", "p_unsafe", "unsafe", "reason", "model", "latency_s",
                                                     "hand_present", "hand_open_waiting", "blade_toward_hand",
                                                     "frame_sha256", "rubric", "rubric_sha256")}


def _menu(keys: Keys, chain, session_id: str, state: str, header: str, options: dict, verdict: dict | None,
          default: str, extra: dict | None = None, on_key=None) -> tuple:
    """-> (key, its row). Only an exact menu key returns; anything else is a row with accepted false and a re-ask."""
    con = keys.console
    con.say("")
    con.say("#" * 78)
    con.say(header)
    con.say(verdict_line(verdict))
    con.say("  " + "   ".join(f"[{k}] {txt}" for k, txt in options.items()))
    con.say("#" * 78)
    while True:
        typed = str(keys.ask("  key: ", default))
        ans = typed.strip().lower()
        ok = ans in options
        row = {"kind": "key", "t_iso": g3.now_iso(), "session_id": session_id, "state": state, "menu": list(options),
               "key": ans if ok else typed.strip()[:40], "accepted": ok, "verdict_on_screen": on_screen(verdict),
               **(extra or {})}
        if ok and on_key is not None:
            row.update(on_key(ans) or {})
        row = chain.append(row)
        if ok:
            return ans, row
        con.say(f"  Not a key. Type exactly one of: {', '.join(options)} (one letter, then Enter).")


def ask_freeze(keys: Keys, chain, session_id: str, where: str, cause: str, verdict: dict | None, offer_continue: bool,
               holding: bool, dry: bool, note: str = "", held: bool = True, on_key=None, placement: bool = False,
               monitors: str = "all 8 monitors", offer_release: bool = False, h_text: str | None = None) -> tuple:
    options = dict(ASK_KEYS)
    if not holding:
        options.pop("h"), options.pop("o")
    elif placement:                                  # palm placement: [h] retries it; it turns the screwdriver itself
        options.pop("o")
        options["h"] = h_text or "place it in the open palm (a settle and a fresh judge call first)"
    if holding and placement and offer_release:     # the operator's own release, where the arm stands (2026-10-04)
        options["r"] = "release: open the gripper here (your key), then the arm lifts and retreats once your hand is out"
    if offer_continue:
        options["c"] = f"continue the interrupted segment ({monitors} clear)"
    header = (f"# FROZEN during {where}: {cause}\n# The arm holds its pose (goals = present position). The runner will "
              f"NOT move it by itself." if held else f"# FROZEN during {where}: {cause}\n{HOLD_FAILED}")
    header += f"\n# {note}" if note else ""
    return _menu(keys, chain, session_id, "ASK", header, options, verdict, "l" if dry else "",
                 {"where": where, "cause": cause, "held": bool(held)}, on_key)


def ask_release(keys: Keys, chain, session_id: str, verdict: dict | None, dry: bool) -> tuple:
    header = ("# HOLD: the arm is at the standoff, stationary. Take the handle; when you have it, press [r].\n"
              "# [r] opens the gripper and nothing else. The arm retreats only after your hand has left the workspace.")
    return _menu(keys, chain, session_id, "ASK_RELEASE", header, dict(RELEASE_KEYS), verdict, "l" if dry else "")


def confirm_torque_off(keys: Keys, chain, session_id: str, state: str, first_prompt: str = KILL_PROMPT,
                       defaults: tuple = ("", "")) -> bool:
    """safety.py's double confirmation: OFF, then YES, each typed exactly. -> True only after both. One row."""
    c1 = str(keys.ask(first_prompt, defaults[0])).strip()
    c2 = str(keys.ask(YES_PROMPT, defaults[1])).strip() if c1 == "OFF" else ""
    ok = c1 == "OFF" and c2 == "YES"
    chain.append({"kind": "key", "t_iso": g3.now_iso(), "session_id": session_id, "state": state,
                  "menu": ["OFF", "YES"], "key": f"{c1[:10]}/{c2[:10]}" if c1 else "(Enter)", "accepted": ok,
                  "verdict_on_screen": None})
    if c1 and not ok:
        keys.console.say("Not confirmed. Torque stays ON; the arm holds.")
    return ok


def confirm_kill(keys: Keys, chain, session_id: str, dry: bool) -> bool:
    """[k]: OFF and YES, word for word as safety.py's menu. A dry run answers both (its [k] is scripted)."""
    return confirm_torque_off(keys, chain, session_id, "KILL", KILL_PROMPT, ("OFF", "YES") if dry else ("", ""))
