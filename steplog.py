"""The session's hash chain, one row per control step (PLAN.md F8).

Row format = anchor/g3.py's ProvenanceLog: seq, prev_sha256, row_sha256 (sha256 of the row's canonical JSON without
row_sha256), every row through g3.scrub first; the verifier is the anchor's g3.verify_chain, unchanged. The difference
is the append: g3.ProvenanceLog re-reads and re-verifies the whole file on every append (right for 40 trial rows, too
slow at 15 Hz); here the file is verified once at open and the tail (seq, last hash) is kept in memory.
"""
from __future__ import annotations

import hashlib
import json
import os
import time
from pathlib import Path

from common import paths

paths.use_anchor()
import g3  # noqa: E402

SYNC_KINDS = {"freeze", "key", "judge", "kill", "session_start", "session_end", "trial_end", "refused"}


def plain(obj):
    """numpy scalars / arrays / tuples -> JSON types (the row that is hashed is the row that is written)."""
    if isinstance(obj, dict):
        return {str(k): plain(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [plain(v) for v in obj]
    if hasattr(obj, "tolist"):
        return plain(obj.tolist())
    if isinstance(obj, (bool, int, str)) or obj is None:
        return obj
    if isinstance(obj, float):
        return obj if obj == obj and abs(obj) != float("inf") else None
    return str(obj)


def row_hash(row: dict) -> str:
    body = {k: v for k, v in row.items() if k != "row_sha256"}
    return hashlib.sha256(json.dumps(body, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()


class StepLog:
    def __init__(self, path: Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        ok, why = g3.verify_chain(self.path)
        if not ok:
            raise RuntimeError(f"provenance chain broken at open ({why}) - refusing to write")
        self.seq, self.tail = 0, None
        if self.path.is_file():
            lines = [x for x in self.path.read_text(encoding="utf-8").splitlines() if x.strip()]
            if lines:
                last = json.loads(lines[-1])
                self.seq, self.tail = int(last["seq"]), last["row_sha256"]
        self._f = open(self.path, "a", encoding="utf-8")
        self.append_ms: list = []
        self.stamps: dict = {}                         # written into every row from here on (--layer-off's stamp)

    def append(self, row: dict, sync: bool | None = None) -> dict:
        t0 = time.perf_counter()
        row = g3.scrub(plain({**dict(row), **self.stamps}))
        row.pop("row_sha256", None)
        row["seq"] = self.seq + 1
        row["prev_sha256"] = self.tail
        row["row_sha256"] = row_hash(row)
        self._f.write(json.dumps(row, sort_keys=True) + "\n")
        self._f.flush()
        if sync if sync is not None else row.get("kind") in SYNC_KINDS:
            os.fsync(self._f.fileno())
        self.seq, self.tail = row["seq"], row["row_sha256"]
        self.append_ms.append(1000.0 * (time.perf_counter() - t0))
        return row

    def rows(self) -> list:
        self._f.flush()
        return [json.loads(x) for x in self.path.read_text(encoding="utf-8").splitlines() if x.strip()]

    def verify(self) -> tuple:
        self._f.flush()
        return g3.verify_chain(self.path)

    def close(self) -> None:
        if not self._f.closed:
            self._f.flush()
            os.fsync(self._f.fileno())
            self._f.close()
