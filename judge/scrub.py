"""Secrets hygiene for the judge: token-shaped strings and the .env's own values never leave this package.

The patterns are copied from anchor/g3.py's scrub (G3.8) - copied, not imported: the judge imports nothing from the
control side. redact() also replaces the exact values the .env holds, whatever their shape.
"""
from __future__ import annotations

import re

_SECRET_RES = [re.compile(p) for p in (r"hf_[A-Za-z0-9]{16,}", r"sk-[A-Za-z0-9_\-]{16,}", r"AKIA[0-9A-Z]{16}",
                                       r"gh[pousr]_[A-Za-z0-9]{20,}",
                                       r"-----BEGIN [A-Z ]*PRIVATE KEY-----[\s\S]*?-----END [A-Z ]*PRIVATE KEY-----")]
REDACTED = "[REDACTED]"


def redact(obj, secrets: tuple = ()):
    """A copy with every token-shaped substring and every given secret value replaced (strings in dicts/lists too)."""
    if isinstance(obj, str):
        s = obj
        for v in secrets:
            if v:
                s = s.replace(v, REDACTED)
        for rx in _SECRET_RES:
            s = rx.sub(REDACTED, s)
        return s
    if isinstance(obj, dict):
        return {k: redact(v, secrets) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [redact(v, secrets) for v in obj]
    return obj
