#!/usr/bin/env python3
"""Verify a session's hash chain and its photos with the standard library only (no anchor/ needed).

    python3 verify_chain.py sessions/KH-S20261005T015041        (a session folder, or its session.jsonl)

The same three rules as anchor/g3.py's verify_chain: row n has seq n; its prev_sha256 is the previous row's
row_sha256 (null for the first); its row_sha256 is the sha256 of the row without that field, as JSON with sorted keys
and no spaces. Then every file a row names (photo, frame) is hashed against the sha256 that row carries.
Exit 0 when everything matches.
"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path


def row_hash(row: dict) -> str:
    body = {k: v for k, v in row.items() if k != "row_sha256"}
    return hashlib.sha256(json.dumps(body, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()


def verify(chain: Path) -> tuple:
    """-> (ok, reason, rows)."""
    prev, rows = None, []
    for i, line in enumerate(x for x in chain.read_text(encoding="utf-8").splitlines() if x.strip()):
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            return False, f"row {i + 1} is not JSON", rows
        if row.get("seq") != i + 1:
            return False, f"row {i + 1}: sequence {row.get('seq')} (a row was deleted or reordered)", rows
        if row.get("prev_sha256") != prev:
            return False, f"row {i + 1}: its link to the previous row does not match", rows
        if row_hash(row) != row.get("row_sha256"):
            return False, f"row {i + 1}: its content does not match its hash (edited)", rows
        prev = row["row_sha256"]
        rows.append(row)
    return True, "ok", rows


def files(folder: Path, rows: list) -> list:
    """[(file, ok or why)] for every file a row names with its sha256."""
    out = []
    for r in rows:
        for name, sha in (("photo", "photo_sha256"), ("frame", "frame_sha256")):
            f = r.get(name)
            if isinstance(f, str) and f and r.get(sha):
                p = folder / f
                got = hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None
                out.append((f, "ok" if got == r[sha] else ("missing" if got is None else "MISMATCH")))
    return out


def main(argv: list) -> int:
    if not argv:
        print(__doc__)
        return 2
    p = Path(argv[0])
    chain = p / "session.jsonl" if p.is_dir() else p
    ok, why, rows = verify(chain)
    print(f"{chain}: {'verifies' if ok else 'BROKEN'} ({why}), {len(rows)} rows")
    checked = files(chain.parent, rows) if ok else []
    bad = [(f, s) for f, s in checked if s != "ok"]
    print(f"files named by rows: {len(checked)} checked, {len(bad)} not matching" + "".join(f"\n  {s}: {f}" for f, s in bad))
    return 0 if ok and not bad else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
