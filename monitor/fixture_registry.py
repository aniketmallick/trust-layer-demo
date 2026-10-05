"""Fixtures in the style of anchor/negative_fixtures*.py: a name, one sentence of what is injected and what must
happen, a function returning (as_expected, detail). kind="control" marks the must-not-fire cases. A fixture whose
input does not exist yet raises Unknown: it is reported as skipped with its reason, never as passed. stdlib only."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

FRAME_NOT_CAPTURED = "frame not captured"


class Unknown(Exception):
    """The fixture could not be run (its reason is the message): unknown, not passed."""


@dataclass(frozen=True)
class Fixture:
    group: str
    name: str
    what: str
    kind: str                  # "negative": the fault must be caught; "control": nothing may fire
    fn: Callable[[], tuple]


def registry() -> tuple[list, Callable]:
    """A fresh (REGISTRY, fixture decorator) pair for one fixtures module."""
    reg: list = []

    def fixture(group: str, name: str, what: str, kind: str = "negative"):
        def deco(fn):
            reg.append(Fixture(group, name, what, kind, fn))
            return fn
        return deco
    return reg, fixture


def run_all(reg: list) -> int:
    """Run every fixture and print one line each (the anchor's table). -> the number not as expected."""
    bad = unknown = 0
    for f in reg:
        try:
            ok, detail = f.fn()
            tag = "ok " if ok else "BAD"
            bad += 0 if ok else 1
        except Unknown as e:
            tag, detail = "?? ", f"UNKNOWN - {e}"
            unknown += 1
        print(f"  [{tag}] {f.group:5} {f.name:52} {'(control) ' if f.kind == 'control' else ''}-> {detail}")
    print(f"{len(reg) - bad - unknown} as expected, {bad} NOT as expected, {unknown} unknown (not run)")
    return bad
