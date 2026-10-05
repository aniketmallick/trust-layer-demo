"""common/: the pixel map against the frozen runner's own, the flags, the frame pair, the message schema."""
from __future__ import annotations

import json

import numpy as np
import pytest

from common import paths, wire
from common.table_map import load_table_map

CROSS6_MM, CROSS6_PX = (241.1, -58.8), (247.6, 109.5)       # layout v3 cross 6 through null_plan.cross_xy
ZONE_C_MM = (246.8, 68.1)                                     # registration: zone_centre_mm


@pytest.fixture(scope="module")
def tm():
    return load_table_map()


def test_map_equals_the_null_runners_cammap(tm):
    paths.use_anchor()
    import null_plan
    import null_session
    inp = null_plan.load_inputs(paths.ROOT / paths.REG_REL, paths.ROOT / paths.LAYOUT_REL, paths.ROOT / paths.RECORD_REL)
    cm = null_session.CamMap(inp)
    rng = np.random.default_rng(1)
    for px in rng.uniform([0, 0], [640, 480], (50, 2)):
        assert np.allclose(tm.px_to_mm(px), cm.to_arm(px), atol=1e-6)
        assert np.allclose(tm.px_to_mm(px, 30.0), cm.cube_centre(px, 30.0), atol=1e-6)
        assert np.allclose(tm.mm_to_px(tm.px_to_mm(px)), px, atol=1e-6)


def test_registered_points_round_trip(tm):
    assert np.allclose(tm.mm_to_px(CROSS6_MM), CROSS6_PX, atol=0.5)
    reg = json.loads((paths.ROOT / paths.REG_REL).read_text())
    want = {tuple(c["arm_mm"]) for c in reg["arm_frame"]["targets_arm_mm"]["corners"]}
    got = [tm.px_to_mm(px) for px in reg["a2b"]["spawn_quad_native_px"] + reg["a2b"]["zone_quad_native_px"]]
    for g in got:                                             # every A2b outline corner is a registered corner
        assert min(np.linalg.norm(g - np.array(w)) for w in want) < 0.5


def test_flags_fire_and_do_not_fire(tm):
    on_cross = tm.flags(tm.mm_to_px(CROSS6_MM))
    assert on_cross["in_spawn"] and not on_cross["in_zone"] and on_cross["in_workspace"]
    on_zone = tm.flags(tm.mm_to_px(ZONE_C_MM))
    assert on_zone["in_zone"] and not on_zone["in_spawn"] and on_zone["in_workspace"]
    gap = tm.flags(tm.mm_to_px((255.0, 4.0)))                 # the 40 mm gap between the sheets: in neither
    assert not gap["in_spawn"] and not gap["in_zone"] and gap["in_workspace"]
    x0, x1, y0, y1 = tm.box_xy
    far = tm.flags(tm.mm_to_px((x1 + 60.0, 0.0)))             # control: well past the box's far edge
    assert not far["in_workspace"]


def test_a_raised_hand_just_outside_the_table_reading_still_counts(tm):
    """A palm 150 mm up over a point inside the box reads outward of it at table height; the flag still fires."""
    x0, x1, y0, y1 = tm.box_xy
    inside = np.array([x1 - 2.0, 100.0])
    c = tm.px_to_mm(tm.nadir_px)
    seen_at_table = c + (inside - c) * tm.cam_height_mm / (tm.cam_height_mm - 150.0)
    assert not tm.in_box(seen_at_table)
    assert tm.flags(tm.mm_to_px(seen_at_table))["in_workspace"]


def test_frame_pair_is_atomic_and_checked(tmp_path):
    meta = wire.write_frame(tmp_path, b"jpg-bytes-1", 7, 12.5)
    got = wire.read_frame(tmp_path)
    assert got is not None and got[0] == meta and got[1] == b"jpg-bytes-1"
    (tmp_path / "frame.jpg").write_bytes(b"a newer jpg, its json not yet renamed")
    assert wire.read_frame(tmp_path) is None
    assert not list(tmp_path.glob(".*.tmp"))


def test_debounce_three_of_five():
    assert wire.debounce([True, False, True, False, True]) == {"hand_in_workspace": True, "n_of_5": 3}
    assert wire.debounce([True, True, False, False, False])["hand_in_workspace"] is False
    assert wire.debounce([True] * 9)["n_of_5"] == 5


def test_hand_message_schema():
    hand = {"handedness": "R", "conf": 0.93, "palm_px": [247.6, 109.5], "palm_mm": [241.1, -58.8],
            "in_workspace": True, "in_spawn": True, "in_zone": False}
    m = wire.build_hand_msg(1.0, 3, {"seq": 9, "sha256": "ab" * 32, "t": 0.9}, [hand], [True], "sim_feed", 4.2)
    assert wire.validate_hand_msg(m) == [] and wire.decode(wire.encode(m)) == m
    assert wire.validate_hand_msg({**m, "hands": [{**hand, "handedness": "left"}]})
    assert wire.validate_hand_msg({k: v for k, v in m.items() if k != "frame_sha256"})
    assert wire.validate_hand_msg({**m, "seq": True})
    assert wire.validate_hand_msg({**m, "hands": [{**hand, "landmarks_px": [[1.0, 2.0]] * 21}]}) == []
    assert wire.validate_hand_msg({**m, "hands": [{**hand, "landmarks_px": [[1.0, 2.0]] * 20}]})
    assert wire.decode(b"{not json") is None and wire.validate_hand_msg(None)
    for bad in (float("nan"), float("inf")):                  # json.loads reads NaN; NaN >= 50 is False
        assert wire.validate_hand_msg({**m, "hands": [{**hand, "palm_mm": [bad, -58.8]}]})
        assert wire.validate_hand_msg({**m, "hands": [{**hand, "conf": bad}]})
        assert wire.validate_hand_msg({**m, "frame_t": bad})
    assert wire.validate_hand_msg(wire.decode(b'{"t": NaN}'))
