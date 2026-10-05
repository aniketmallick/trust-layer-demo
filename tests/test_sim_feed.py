"""monitor/sim_feed.py: the script's state machine (no sockets): well-formed messages, deterministic events."""
from __future__ import annotations

import pytest

from common import wire
from common.table_map import load_table_map
from monitor.sim_feed import SCRIPTS, Feed, parse_event


@pytest.fixture(scope="module")
def tm():
    return load_table_map()


def run(feed, t0=100.0, n=120, cues=()):
    out = []
    for k in range(n):
        now = t0 + 0.1 * k
        for at, phase in cues:
            if abs(now - (t0 + at)) < 1e-9:
                feed.cue(phase, now)
        out.append(feed.tick(now))
    return out


def test_SIM_parse_event():
    assert parse_event("drift30@HANDOVER_APPROACH#2+1.5") == {"what": "drift30", "phase": "HANDOVER_APPROACH", "nth": 2,
                                                              "after_s": 1.5, "done": False}
    assert parse_event("appear@+3")["phase"] is None and parse_event("appear@+3")["after_s"] == 3.0


def test_SIM_time_script_no_hand_then_appear_then_drift_30_then_80(tm):
    feed = Feed(tm, (300.0, 90.0), ["appear@+3", "drift30@+6", "drift80@+9"], drift_dir=(0, -1))
    msgs = run(feed)
    assert all(wire.validate_hand_msg(m) == [] for m in msgs) and [m["seq"] for m in msgs] == list(range(1, 121))
    assert all(m["model"] == "sim_feed" for m in msgs)
    assert msgs[29]["hands"] == [] and msgs[30]["hands"][0]["palm_mm"] == [300.0, 90.0]
    assert msgs[60]["hands"][0]["palm_mm"] == [300.0, 60.0] and msgs[90]["hands"][0]["palm_mm"] == [300.0, 10.0]
    assert msgs[30]["hands"][0]["in_workspace"] and msgs[30]["debounced"] == {"hand_in_workspace": False, "n_of_5": 1}
    assert msgs[32]["debounced"] == {"hand_in_workspace": True, "n_of_5": 3}


def test_SIM_same_script_same_messages(tm):
    strip = lambda ms: [{k: v for k, v in m.items() if k not in ("t", "frame_t")} for m in ms]   # noqa: E731
    a = run(Feed(tm, (300.0, 90.0), SCRIPTS["handover"]), cues=((2.0, "APPROACH_ZONE"), (5.0, "HANDOVER_APPROACH")))
    b = run(Feed(tm, (300.0, 90.0), SCRIPTS["handover"]), cues=((2.0, "APPROACH_ZONE"), (5.0, "HANDOVER_APPROACH")))
    assert strip(a) == strip(b)


def test_SIM_cue_timed_events_wait_for_their_phase(tm):
    feed = Feed(tm, (300.0, 90.0), SCRIPTS["handover"])
    msgs = run(feed, cues=((2.0, "APPROACH_ZONE"), (5.0, "HANDOVER_APPROACH"), (9.0, "RELEASE")))
    first = next(i for i, m in enumerate(msgs) if m["hands"])
    assert first == 30                                                    # APPROACH_ZONE at 2.0 s + 1.0 s
    assert msgs[66]["hands"][0]["palm_mm"] == [300.0, 120.0]              # 30 mm at HANDOVER_APPROACH + 1.5 s
    assert msgs[91]["hands"][0]["palm_mm"] == [300.0, 170.0] and msgs[100]["hands"] == []
    never = run(Feed(tm, (300.0, 90.0), SCRIPTS["handover"]))             # control: no cue, no hand
    assert all(m["hands"] == [] for m in never)


def test_SIM_silent_second_hand_and_seq_jump(tm):
    msgs = run(Feed(tm, (300.0, 90.0), ["silent@+1", "resume@+2"]), n=40)
    assert all(m is None for m in msgs[10:20]) and msgs[9] is not None and msgs[20] is not None
    assert msgs[20]["seq"] == msgs[9]["seq"] + 1
    msgs = run(Feed(tm, (300.0, 90.0), ["appear@+1", "second@+2"]), n=40)
    assert len(msgs[15]["hands"]) == 1 and len(msgs[25]["hands"]) == 2
    assert {h["handedness"] for h in msgs[25]["hands"]} == {"L", "R"}
    msgs = run(Feed(tm, (300.0, 90.0), ["seqjump5@+1"]), n=20)
    assert msgs[10]["seq"] - msgs[9]["seq"] == 6


def test_SIM_hand_outside_the_box_is_not_in_workspace(tm):
    x1 = tm.box_xy[1]
    msgs = run(Feed(tm, (x1 + 70.0, 0.0), ["appear@+0"]), n=3)
    assert msgs[0]["hands"][0]["in_workspace"] is False
