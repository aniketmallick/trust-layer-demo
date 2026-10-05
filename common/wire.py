"""The two hand-offs every piece agrees on (PLAN.md section 5): the frame pair on disk and the hand message on UDP.
stdlib only."""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path

DEBOUNCE_N, DEBOUNCE_K = 5, 3            # presence = 3 of the last 5 frames
HAND_KEYS = {"handedness": str, "conf": (int, float), "palm_px": list, "palm_mm": list,
             "in_workspace": bool, "in_spawn": bool, "in_zone": bool}
MSG_KEYS = {"t": (int, float), "seq": int, "frame_seq": int, "frame_sha256": str, "frame_t": (int, float),
            "hands": list, "debounced": dict, "model": str, "proc_ms": (int, float)}


def sha256_bytes(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


# ------------------------------------------------------------------------------------------ frame pair
def write_frame(frame_dir: Path, jpg: bytes, seq: int, t: float) -> dict:
    """jpg first, then json, each to a temp name and renamed: a reader never sees half a file, and a json always
    names a jpg that is already complete."""
    d = Path(frame_dir)
    d.mkdir(parents=True, exist_ok=True)
    meta = {"t": float(t), "seq": int(seq), "sha256": sha256_bytes(jpg)}
    for name, data in (("frame.jpg", jpg), ("frame.json", json.dumps(meta).encode("utf-8"))):
        tmp = d / f".{name}.tmp"
        tmp.write_bytes(data)
        os.replace(tmp, d / name)
    return meta


def read_frame(frame_dir: Path) -> tuple | None:
    """(meta, jpg bytes), or None when there is no pair or the jpg is not the one the json names (a newer jpg landed
    between the two reads: the caller reads again)."""
    d = Path(frame_dir)
    try:
        meta = json.loads((d / "frame.json").read_text(encoding="utf-8"))
        jpg = (d / "frame.jpg").read_bytes()
    except (OSError, ValueError):
        return None
    if not isinstance(meta, dict) or sha256_bytes(jpg) != meta.get("sha256"):
        return None
    return meta, jpg


# ------------------------------------------------------------------------------------------ hand message
def debounce(window: list) -> dict:
    """window: the last <= 5 raw per-frame booleans (any hand in_workspace), oldest first."""
    w = [bool(x) for x in window][-DEBOUNCE_N:]
    return {"hand_in_workspace": sum(w) >= DEBOUNCE_K, "n_of_5": sum(w)}


def build_hand_msg(t: float, seq: int, frame_meta: dict, hands: list, window: list, model: str, proc_ms: float) -> dict:
    return {"t": float(t), "seq": int(seq), "frame_seq": int(frame_meta["seq"]), "frame_sha256": str(frame_meta["sha256"]),
            "frame_t": float(frame_meta["t"]), "hands": hands, "debounced": debounce(window), "model": model,
            "proc_ms": round(float(proc_ms), 2)}


def _finite(x) -> bool:
    """A real number: json.loads reads NaN and Infinity, and NaN compares false against every limit."""
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def validate_hand_msg(m) -> list:
    """Problems with a decoded message; empty = well formed. The runner treats a malformed message as no message."""
    if not isinstance(m, dict):
        return ["not an object"]
    probs = [f"{k}: missing or not {getattr(t, '__name__', t)}" for k, t in MSG_KEYS.items()
             if not isinstance(m.get(k), t) or isinstance(m.get(k), bool)]
    probs += [f"{k}: not finite" for k in ("t", "frame_t", "proc_ms") if isinstance(m.get(k), float) and not math.isfinite(m[k])]
    for i, h in enumerate(m.get("hands") if isinstance(m.get("hands"), list) else []):
        if not isinstance(h, dict):
            probs.append(f"hands[{i}]: not an object")
            continue
        probs += [f"hands[{i}].{k}: missing or wrong type" for k, t in HAND_KEYS.items() if not isinstance(h.get(k), t)]
        for k in ("palm_px", "palm_mm"):
            v = h.get(k)
            if isinstance(v, list) and not (len(v) == 2 and all(_finite(x) for x in v)):
                probs.append(f"hands[{i}].{k}: not two finite numbers")
        if not _finite(h.get("conf")):
            probs.append(f"hands[{i}].conf: not a finite number")
        if h.get("handedness") not in ("L", "R"):
            probs.append(f"hands[{i}].handedness: not L or R")
        lm = h.get("landmarks_px")                # optional (the replay's overlay); no rule reads it
        if lm is not None and not (isinstance(lm, list) and len(lm) == 21 and all(
                isinstance(p, list) and len(p) == 2 and all(isinstance(x, (int, float)) for x in p) for p in lm)):
            probs.append(f"hands[{i}].landmarks_px: not 21 pixel pairs")
    d = m.get("debounced")
    if isinstance(d, dict) and not (isinstance(d.get("hand_in_workspace"), bool) and isinstance(d.get("n_of_5"), int)):
        probs.append("debounced: needs hand_in_workspace (bool) and n_of_5 (int)")
    return probs


def encode(m: dict) -> bytes:
    return (json.dumps(m, separators=(",", ":")) + "\n").encode("utf-8")


def decode(b: bytes):
    try:
        return json.loads(b.decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        return None
