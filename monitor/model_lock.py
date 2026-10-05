"""The hand-landmarker model file, pinned by sha256 (models/MODEL.lock). A file that is not the pinned one is refused.

    python monitor/model_lock.py            # verify the file on disk against the lock
    python monitor/model_lock.py --fetch    # download the locked URL if the file is missing, then verify
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import urllib.request
from pathlib import Path

MODELS = Path(__file__).resolve().parent / "models"
LOCK = MODELS / "MODEL.lock"


class ModelLockError(RuntimeError):
    """The model file is missing or is not the one the lock names."""


def read_lock() -> dict:
    lock = json.loads(LOCK.read_text(encoding="utf-8"))
    for k in ("file", "sha256", "url"):
        if not isinstance(lock.get(k), str):
            raise ModelLockError(f"{LOCK.name} lacks '{k}'")
    return lock


def locked_model() -> tuple[Path, str]:
    """(path, sha256) of the model, after checking its bytes against the lock."""
    lock = read_lock()
    p = MODELS / lock["file"]
    if not p.is_file():
        raise ModelLockError(f"{p.name} is missing - run: python monitor/model_lock.py --fetch")
    got = hashlib.sha256(p.read_bytes()).hexdigest()
    if got != lock["sha256"]:
        raise ModelLockError(f"{p.name} is {got[:12]}, the lock names {lock['sha256'][:12]} - refusing to start")
    return p, got


def fetch() -> Path:
    """Download the locked URL once. The bytes are checked before the file is put in place."""
    lock = read_lock()
    p = MODELS / lock["file"]
    if p.is_file():
        return p
    with urllib.request.urlopen(lock["url"], timeout=60) as r:      # noqa: S310 - the URL is the lock's, https
        data = r.read()
    got = hashlib.sha256(data).hexdigest()
    if got != lock["sha256"]:
        raise ModelLockError(f"the download is {got[:12]}, the lock names {lock['sha256'][:12]} - nothing written")
    p.write_bytes(data)
    return p


def main(argv: list | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fetch", action="store_true")
    a = ap.parse_args(argv)
    try:
        if a.fetch:
            fetch()
        p, sha = locked_model()
    except ModelLockError as e:
        print(f"MODEL REFUSED: {e}")
        return 2
    print(f"{p.name}  sha256 {sha}  (matches {LOCK.name})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
