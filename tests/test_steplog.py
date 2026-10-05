"""steplog.py: the anchor's chain format with an O(1) append, verified by the anchor's own g3.verify_chain."""
from __future__ import annotations

import json

from common import paths

paths.use_anchor()
import g3  # noqa: E402

from steplog import StepLog  # noqa: E402


def _fill(path, n):
    log = StepLog(path)
    for k in range(n):
        log.append({"kind": "step", "k": k, "q6": [0.1 * k, 1.0, 2.0, 3.0, 4.0, 50.0], "monitors": {"1": {"fired": False}}})
    log.close()
    return log


def test_STEPLOG_5000_rows_verify_with_the_anchors_verifier(tmp_path):
    log = _fill(tmp_path / "s.jsonl", 5000)
    assert g3.verify_chain(tmp_path / "s.jsonl") == (True, "ok")
    ms = sorted(log.append_ms)
    p95 = ms[int(0.95 * len(ms))]
    print(f"append p50 {ms[len(ms) // 2]:.3f} ms p95 {p95:.3f} ms max {ms[-1]:.3f} ms over {len(ms)} rows")
    assert p95 < 5.0                                           # the control step's budget is 89.9 ms


def test_STEPLOG_same_bytes_as_the_anchors_provenance_log(tmp_path):
    """Control: for the same rows, the file is byte-identical to anchor/g3.py ProvenanceLog's."""
    rows = [{"kind": "trial", "n": i, "note": "x", "v": [1.5, 2]} for i in range(5)]
    a, b = g3.ProvenanceLog(tmp_path / "a.jsonl"), StepLog(tmp_path / "b.jsonl")
    for r in rows:
        a.append(r)
        b.append(r)
    b.close()
    assert (tmp_path / "a.jsonl").read_bytes() == (tmp_path / "b.jsonl").read_bytes()


def test_STEPLOG_delete_edit_reorder_each_fail(tmp_path):
    _fill(tmp_path / "s.jsonl", 20)
    lines = (tmp_path / "s.jsonl").read_text().splitlines()
    cut = tmp_path / "cut.jsonl"
    cut.write_text("\n".join(lines[:7] + lines[8:]) + "\n")
    assert g3.verify_chain(cut)[0] is False
    row = json.loads(lines[7])
    row["q6"][0] += 0.001
    edited = tmp_path / "edited.jsonl"
    edited.write_text("\n".join(lines[:7] + [json.dumps(row, sort_keys=True)] + lines[8:]) + "\n")
    assert g3.verify_chain(edited)[0] is False
    swapped = tmp_path / "swapped.jsonl"
    swapped.write_text("\n".join(lines[:7] + [lines[8], lines[7]] + lines[9:]) + "\n")
    assert g3.verify_chain(swapped)[0] is False


def test_STEPLOG_reopen_continues_the_chain_and_refuses_a_broken_one(tmp_path):
    _fill(tmp_path / "s.jsonl", 10)
    log = StepLog(tmp_path / "s.jsonl")
    assert log.append({"kind": "step", "k": 99})["seq"] == 11
    log.close()
    assert g3.verify_chain(tmp_path / "s.jsonl")[0] is True
    lines = (tmp_path / "s.jsonl").read_text().splitlines()
    (tmp_path / "s.jsonl").write_text("\n".join(lines[:3] + lines[4:]) + "\n")
    try:
        StepLog(tmp_path / "s.jsonl")
        raise AssertionError("a broken chain was opened for writing")
    except RuntimeError as e:
        assert "broken" in str(e)


def test_STEPLOG_token_shaped_strings_never_reach_the_file(tmp_path):
    log = StepLog(tmp_path / "s.jsonl")
    fakes = ["hf_" + "A" * 30, "sk-" + "b" * 40]
    log.append({"kind": "key", "note": f"pasted {fakes[0]} by mistake", "verdict": {"raw": f"key {fakes[1]}"}})
    log.close()
    text = (tmp_path / "s.jsonl").read_text()
    assert not any(f in text for f in fakes) and "[REDACTED]" in text
    assert g3.verify_chain(tmp_path / "s.jsonl")[0] is True


def test_STEPLOG_numpy_values_are_written_as_plain_json(tmp_path):
    import numpy as np
    log = StepLog(tmp_path / "s.jsonl")
    log.append({"kind": "step", "tcp": np.array([1.0, 2.0]), "ok": np.bool_(True), "n": np.int64(3), "x": float("nan")})
    log.close()
    row = json.loads((tmp_path / "s.jsonl").read_text())
    assert row["tcp"] == [1.0, 2.0] and row["ok"] is True and row["n"] == 3 and row["x"] is None
    assert g3.verify_chain(tmp_path / "s.jsonl")[0] is True
