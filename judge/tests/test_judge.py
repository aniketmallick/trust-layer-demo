"""The judge's fixtures through pytest: the offline registry (judge/fixtures.py), then the saved frames against the
live provider. A live case with no frame or no configured judge is SKIPPED with the reason - unknown, never passed."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

from judge import fixtures as fx
from judge import judge as jd

FRAMES = Path(__file__).resolve().parents[2] / "fixtures" / "frames"
STILLS = (1, 2, 3)
# PLAN.md section 9: scene -> the fields the judge must report (None = not asserted)
EXPECTED = {
    "empty_table": {"hand_present": False},
    "hand_cross6": {"hand_present": True, "hand_open_waiting": True},
    "hand_zone": {"hand_present": True, "hand_open_waiting": True},
    "fist_zone": {"hand_present": True, "hand_open_waiting": False},
    "two_hands": {"hand_present": True},
}


# the handover rubric (the operator, 2026-10-03) at the handover freeze: the stills show no arm, so which end faces the
# hand comes from the runner's own geometry in the context, as the rubric says
HANDOVER_EXPECTED = {
    "hand_zone": {"hand_present": True, "hand_open_waiting": True, "unsafe": False},
    "hand_cross6": {"hand_present": True, "hand_open_waiting": True, "unsafe": False},
    "fist_zone": {"hand_present": True, "hand_open_waiting": False, "unsafe": True},
    "two_hands": {"hand_present": True, "unsafe": True},
}


class _Capture:
    """A provider that keeps the system text it was sent and answers one fixed, well-formed handover verdict."""
    name = model = "capture"

    def __init__(self):
        self.systems = []

    def complete(self, system, user_text, image, media_type, timeout_s):
        self.systems.append(system)
        return json.dumps({"hand_present": True, "hand_open_waiting": True, "blade_toward_hand": False, "p_unsafe": 0.2,
                           "recommend": "handover", "reason": "capture provider"})


def test_RUBRIC_kind_by_phase_and_context():
    k = jd.rubric_kind
    assert k("CP3", {}) == k("CP3", None) == "handover"
    assert k("FREEZE", {"holding_prop": True, "hands_seen_by_monitor": 1}) == "handover"
    assert k("FREEZE", {"holding_prop": True, "hands_seen_by_monitor": 2}) == "handover"
    for phase, ctx in (("CP1", {"holding_prop": True, "hands_seen_by_monitor": 1}), ("CP2", {}),
                       ("FREEZE", {"holding_prop": False, "hands_seen_by_monitor": 1}),
                       ("FREEZE", {"holding_prop": True, "hands_seen_by_monitor": 0}), ("FREEZE", None),
                       ("FREEZE", {"holding_prop": "yes", "hands_seen_by_monitor": 1}),
                       ("FREEZE", {"holding_prop": True, "hands_seen_by_monitor": "x"})):
        assert k(phase, ctx) == "checkpoint", (phase, ctx)


def test_RUBRIC_placement_mode_selects_the_placement_rubric():
    k = jd.rubric_kind
    pl = {"holding_prop": True, "hands_seen_by_monitor": 1, "mode": "placement"}
    assert k("FREEZE", pl) == k("CP3", {"mode": "placement"}) == "placement"
    assert k("CP1", pl) == k("CP2", pl) == "checkpoint"                     # CP1 / CP2 keep rubric.md
    assert k("FREEZE", {**pl, "holding_prop": False}) == "checkpoint"
    assert k("FREEZE", {**pl, "mode": "standoff"}) == "handover"
    cap = _Capture()
    v = jd.Judge(cap).ask(FRAMES / "empty_table_1.png", "CP3", {"mode": "placement"})
    body = jd.PLACEMENT_RUBRIC_PATH.read_bytes()
    assert v["rubric"] == "placement" and v["rubric_sha256"] == hashlib.sha256(body).hexdigest()
    assert cap.systems[-1] == body.decode() and "plastic prop" in body.decode()
    assert jd.Judge(cap).describe()["rubric_placement_sha256"] == hashlib.sha256(body).hexdigest()


def test_RUBRIC_each_call_sends_its_rubric_and_logs_its_hash():
    cap = _Capture()
    j = jd.Judge(cap)
    frame = FRAMES / "empty_table_1.png"
    hand, cp = jd.HANDOVER_RUBRIC_PATH.read_bytes(), jd.RUBRIC_PATH.read_bytes()
    v = j.ask(frame, "FREEZE", {"holding_prop": True, "hands_seen_by_monitor": 1})
    assert v["rubric"] == "handover" and v["rubric_sha256"] == hashlib.sha256(hand).hexdigest()
    assert cap.systems[-1] == hand.decode() and v["unsafe"] is False                  # 0.2 < the 0.5 bar
    v = j.ask(frame, "CP1", {"holding_prop": True, "hands_seen_by_monitor": 0})
    assert v["rubric"] == "checkpoint" and v["rubric_sha256"] == hashlib.sha256(cp).hexdigest()
    assert cap.systems[-1] == cp.decode()
    assert hashlib.sha256(cp).hexdigest().startswith("2825992d")                    # rubric.md itself unchanged
    d = j.describe()
    assert d["rubric_handover_sha256"] == hashlib.sha256(hand).hexdigest() and d["p_unsafe_bar"] == 0.5
    v = jd.Judge(cap, handover_rubric_path=FRAMES / "no_such_rubric.md").ask(frame, "CP3", {})
    assert v["unsafe"] is True and v["reason"] == "rubric not readable" and v["rubric"] == "handover"


@pytest.mark.parametrize("fixture", fx.REGISTRY, ids=[f["name"] for f in fx.REGISTRY])
def test_fixture(fixture):
    ok, detail = fixture["fn"]()
    assert ok, f"{fixture['name']} ({fixture['kind']}): {detail}\n  {fixture['what']}"


@pytest.fixture(scope="module")
def live_judge():
    """The judge the .env configures (models listed once, here). Loaded only when a live case has its frame."""
    return jd.load()


@pytest.mark.live
@pytest.mark.parametrize("scene,k", [(s, k) for s in EXPECTED for k in STILLS], ids=lambda v: str(v))
def test_saved_frame_live(scene, k, request):
    frame = FRAMES / f"{scene}_{k}.png"
    if not frame.is_file():
        pytest.skip(f"UNKNOWN: no frame {frame.name} (fixtures/capture_frame.py --name {scene})")
    judge = request.getfixturevalue("live_judge")
    if not judge.configured:
        pytest.skip(f"UNKNOWN: judge not configured ({judge.problem})")
    v = judge.ask(frame, "FREEZE", {"fixture": scene, "still": k})
    fenced = f"{str(v.get('raw') or '').lstrip().startswith('```')} stripped {v.get('fence_stripped')}"
    print(f"{frame.name}: model {v['model']} latency {v['latency_s']} s fenced {fenced} reason {v['reason']!r} "
          f"hand_present {v['hand_present']} hand_open_waiting {v['hand_open_waiting']} recommend {v['recommend']} "
          f"p_unsafe {v['p_unsafe']}")
    assert v["reason"] not in ("timeout", "malformed reply", "provider error"), (v["reason"], v["raw"][:200])
    for field, want in EXPECTED[scene].items():
        assert v[field] is want, f"{frame.name}: {field} = {v[field]!r}, expected {want!r} ({v['reason']})"
    assert v["latency_s"] <= jd.BUDGET_S


@pytest.mark.live
@pytest.mark.parametrize("scene,k", [(s, k) for s in HANDOVER_EXPECTED for k in STILLS], ids=lambda v: str(v))
def test_handover_rubric_live(scene, k, request):
    frame = FRAMES / f"{scene}_{k}.png"
    if not frame.is_file():
        pytest.skip(f"UNKNOWN: no frame {frame.name} (fixtures/capture_frame.py --name {scene})")
    judge = request.getfixturevalue("live_judge")
    if not judge.configured:
        pytest.skip(f"UNKNOWN: judge not configured ({judge.problem})")
    ctx = {"holding_prop": True, "hands_seen_by_monitor": 2 if scene == "two_hands" else 1,
           "blade_toward_palm_by_code": False, "state": "FREEZE", "fixture": scene, "still": k}
    v = judge.ask(frame, "FREEZE", ctx)
    print(f"{frame.name}: rubric {v['rubric']} {str(v['rubric_sha256'])[:12]} model {v['model']} latency "
          f"{v['latency_s']} s reason {v['reason']!r} hand_present {v['hand_present']} hand_open_waiting "
          f"{v['hand_open_waiting']} recommend {v['recommend']} p_unsafe {v['p_unsafe']} unsafe {v['unsafe']}")
    assert v["rubric"] == "handover"
    assert v["reason"] not in ("timeout", "malformed reply", "provider error"), (v["reason"], v["raw"][:200])
    for field, want in HANDOVER_EXPECTED[scene].items():
        assert v[field] is want, f"{frame.name}: {field} = {v[field]!r}, expected {want!r} ({v['reason']})"
    assert v["latency_s"] <= jd.BUDGET_S


@pytest.mark.live
@pytest.mark.parametrize("scene,k", [(s, k) for s in HANDOVER_EXPECTED for k in STILLS], ids=lambda v: str(v))
def test_placement_rubric_live(scene, k, request):
    """The placement rubric (2026-10-03) at the placement freeze: the still as the frozen scene, the prop held."""
    frame = FRAMES / f"{scene}_{k}.png"
    if not frame.is_file():
        pytest.skip(f"UNKNOWN: no frame {frame.name} (fixtures/capture_frame.py --name {scene})")
    judge = request.getfixturevalue("live_judge")
    if not judge.configured:
        pytest.skip(f"UNKNOWN: judge not configured ({judge.problem})")
    ctx = {"holding_prop": True, "hands_seen_by_monitor": 2 if scene == "two_hands" else 1, "mode": "placement",
           "state": "FREEZE", "fixture": scene, "still": k}
    v = judge.ask(frame, "FREEZE", ctx)
    print(f"{frame.name}: rubric {v['rubric']} {str(v['rubric_sha256'])[:12]} model {v['model']} latency "
          f"{v['latency_s']} s reason {v['reason']!r} hand_present {v['hand_present']} hand_open_waiting "
          f"{v['hand_open_waiting']} recommend {v['recommend']} p_unsafe {v['p_unsafe']} unsafe {v['unsafe']}")
    assert v["rubric"] == "placement"
    assert v["reason"] not in ("timeout", "malformed reply", "provider error"), (v["reason"], v["raw"][:200])
    for field, want in HANDOVER_EXPECTED[scene].items():
        assert v[field] is want, f"{frame.name}: {field} = {v[field]!r}, expected {want!r} ({v['reason']})"
    assert v["latency_s"] <= jd.BUDGET_S
