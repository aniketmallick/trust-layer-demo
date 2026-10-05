"""The judge's .env: parsed here, held privately, never shown.

    python -m judge.env          # which variable NAMES are set (true / false) - never a value, never a length

experiments/knife_handover/.env is the operator's file (git-ignored). Lines are KEY=VALUE; blank lines and # comments
are skipped; a leading `export ` is dropped; a value may sit in single or double quotes; an unquoted value ends at
` #`. Names the judge reads: JUDGE_PROVIDER (anthropic | openai), JUDGE_MODEL (optional), ANTHROPIC_API_KEY,
OPENAI_API_KEY. The process environment is not consulted: the provider is chosen by what is in the file.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ENV_PATH = Path(__file__).resolve().parent.parent / ".env"
NAMES = ("JUDGE_PROVIDER", "JUDGE_MODEL", "ANTHROPIC_API_KEY", "OPENAI_API_KEY")
KEY_OF = {"anthropic": "ANTHROPIC_API_KEY", "openai": "OPENAI_API_KEY"}
_NAME_RE = re.compile(r"[A-Za-z_][A-Za-z0-9_]*\Z")
_SECRET_SUFFIX = ("_KEY", "_TOKEN", "_SECRET", "_PASSWORD")


def parse(text: str) -> dict:
    """KEY=VALUE lines -> {name: value}. A line that is not one is skipped (never echoed: it may hold a secret)."""
    out = {}
    for line in text.splitlines():
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        if s.startswith("export ") or s.startswith("export\t"):
            s = s[7:].lstrip()
        name, sep, val = s.partition("=")
        name = name.strip()
        if not sep or not _NAME_RE.match(name):
            continue
        val = val.strip()
        if val[:1] in ("'", '"') and val.find(val[0], 1) > 0:      # quoted: up to the closing quote; the rest is dropped
            val = val[1:val.find(val[0], 1)]
        else:
            val = re.split(r"\s+#", val, maxsplit=1)[0].strip()
        out[name] = val
    return out


class Env:
    """The parsed values. repr / str / format show which NAMES are set, nothing else; it cannot be pickled."""

    __slots__ = ("_values",)

    def __init__(self, values: dict | None = None):
        self._values = dict(values or {})

    def value(self, name: str) -> str | None:
        """The one accessor of a value. Callers pass it to a request header and nowhere else."""
        v = self._values.get(name)
        return v if v else None

    def is_set(self, name: str) -> bool:
        return bool(self._values.get(name))

    def names_set(self) -> dict:
        return {n: self.is_set(n) for n in NAMES}

    def secrets(self) -> tuple:
        """Every secret-named value, for redaction."""
        return tuple(v for k, v in self._values.items() if v and k.upper().endswith(_SECRET_SUFFIX))

    def __repr__(self) -> str:
        return "Env(set: " + ", ".join(n for n, ok in self.names_set().items() if ok) + ")"

    __str__ = __repr__

    def __format__(self, spec: str) -> str:
        return repr(self)

    def __reduce__(self):
        raise TypeError("Env is not picklable (it holds secrets)")


def load_env(path: Path | None = None) -> Env:
    """The .env at `path` (default: experiments/knife_handover/.env). Missing or unreadable -> an empty Env."""
    p = Path(path) if path is not None else ENV_PATH
    try:
        return Env(parse(p.read_text(encoding="utf-8")))
    except (OSError, UnicodeDecodeError):
        return Env()


def main() -> int:
    env = load_env()
    print(f".env present: {ENV_PATH.is_file()}")
    for name, ok in env.names_set().items():
        print(f"  {name}: {'set' if ok else 'not set'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
