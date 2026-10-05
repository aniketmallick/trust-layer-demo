#!/usr/bin/env python
"""The hand monitor's fixtures (monitor venv): each rule seen to fire and seen not to fire.

    .venv_monitor/bin/python monitor/fixtures_monitor.py          # the table
    .venv_monitor/bin/python -m pytest monitor -q                 # the same, through pytest

  MON   the palm centre, the detector on a blank table and on a rig still with two hands, the model lock, the
        publishing loop (one message per new frame, silence otherwise), the publishing rate
  FRAME the saved stills of fixtures/frames against fixtures/expected.json (UNKNOWN until they are captured)
The code-path checks use fixtures/frames/two_hands_1.png (a still of this rig, two hands in the workspace; pinned
by sha256 as in fixtures/frames/manifest.jsonl): the detector seen to report hands at all, and a frame with a hand
for the publishing loop. (Until 2026-10-05 they used MediaPipe's own demo picture; it is not in the public tree.)
"""
from __future__ import annotations

import hashlib
import json
import socket
import sys
import tempfile
import threading
import time
from pathlib import Path

sys.dont_write_bytecode = True
EXP = Path(__file__).resolve().parent.parent
if str(EXP) not in sys.path:
    sys.path.insert(0, str(EXP))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from common import paths, wire  # noqa: E402
from common.table_map import load_table_map  # noqa: E402
from monitor import hand_monitor as hm  # noqa: E402
from monitor import model_lock  # noqa: E402
from monitor.detector import HandDetector  # noqa: E402
from monitor.fixture_registry import FRAME_NOT_CAPTURED, Unknown, registry, run_all  # noqa: E402
from monitor.palm import PALM_LANDMARKS, palm_centre_px  # noqa: E402

REGISTRY, fixture = registry()
SAMPLE = EXP / "fixtures" / "frames" / "two_hands_1.png"                      # 640 x 480, two hands (manifest.jsonl)
SAMPLE_SHA256 = "cd2bc399a24ac4d564c3f82083186b71454a9cba46950039f2b2df873c92b453"
EXPECTED = EXP / "fixtures" / "expected.json"
MIN_HZ, TARGET_HZ, FEED_HZ = 5.0, 10.0, 15.0
MAX_UDP_PAYLOAD = 9216          # macOS net.inet.udp.maxdgram default: the largest datagram sendto() takes on loopback
_shared: dict = {}


def detector() -> HandDetector:
    if "det" not in _shared:
        _shared["det"] = HandDetector(model_lock.locked_model()[0], max_hands=2)
    return _shared["det"]


def table_map():
    if "tm" not in _shared:
        _shared["tm"] = load_table_map()
    return _shared["tm"]


def blank_table(seed: int = 0) -> np.ndarray:
    """A synthetic clear table as the null runner's DryWorld draws it: grey paper, sensor noise, the two outlines."""
    tm = table_map()
    rng = np.random.default_rng(seed)
    img = np.full((480, 640, 3), (196, 200, 204), np.float32) + rng.normal(0, 3, (480, 640, 1))
    img = np.clip(img, 0, 255).astype(np.uint8)
    for quad in (tm.spawn_quad, tm.zone_quad):
        pts = np.array([tm.mm_to_px(p) for p in quad], np.float32)
        cv2.polylines(img, [np.round(pts).astype(np.int32)], True, (60, 60, 60), 1, cv2.LINE_AA)
    return img


def sample_frame() -> np.ndarray:
    """The 640x480 rig still with hands in it (so the landmark model runs, as with a hand on the rig)."""
    data = SAMPLE.read_bytes()
    if hashlib.sha256(data).hexdigest() != SAMPLE_SHA256:
        raise Unknown("the two-hands still is not the pinned one")
    return cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)


def jpg(img: np.ndarray) -> bytes:
    ok, buf = cv2.imencode(".jpg", img)
    return buf.tobytes()


class Loop:
    """The monitor's loop on a temp frames dir, publishing to a loopback socket this object listens on."""

    def __init__(self):
        self.dir = Path(tempfile.mkdtemp(prefix="khv_mon_"))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.stop, self.stats = threading.Event(), hm.Stats()
        self.thread = threading.Thread(target=hm.run, daemon=True, args=(self.dir, self.sock.getsockname(), detector(),
                                                                         table_map(), self.stop, self.stats))
        self.thread.start()

    def recv(self, timeout_s: float) -> dict | None:
        self.sock.settimeout(timeout_s)
        try:
            return wire.decode(self.sock.recvfrom(65535)[0])
        except (socket.timeout, BlockingIOError):
            return None

    def close(self) -> None:
        self.stop.set()
        self.thread.join(timeout=2.0)
        self.sock.close()


# ======================================================================================== MON: the pieces
@fixture("MON", "MON_palm_centre_is_the_mean_of_six",
         "21 synthetic landmarks with landmarks 0, 1, 5, 9, 13, 17 at known pixels and the 15 others far away: the palm "
         "centre is the mean of the six; a hand with 20 landmarks is refused", kind="control")
def _palm():
    six = {0: (100.0, 200.0), 1: (130.0, 190.0), 5: (140.0, 150.0), 9: (120.0, 140.0), 13: (100.0, 142.0), 17: (82.0, 150.0)}
    lm = [six.get(i, (9000.0 + i, -9000.0)) for i in range(21)]
    got = palm_centre_px(lm)
    want = (sum(p[0] for p in six.values()) / 6, sum(p[1] for p in six.values()) / 6)
    try:
        palm_centre_px(lm[:20])
        refused = False
    except ValueError:
        refused = True
    ok = tuple(sorted(six)) == PALM_LANDMARKS and abs(got[0] - want[0]) < 1e-9 and abs(got[1] - want[1]) < 1e-9 and refused
    return ok, f"palm ({got[0]:.2f}, {got[1]:.2f}) px, mean of the six ({want[0]:.2f}, {want[1]:.2f}); 20 landmarks refused: {refused}"


@fixture("MON", "MON_blank_table_no_hands",
         "three synthetic clear-table images (grey paper, noise, the spawn and zone outlines): the detector must report "
         "no hand in any", kind="control")
def _blank():
    counts = [len(hm.hands_of(detector(), table_map(), blank_table(seed))[0]) for seed in range(3)]
    return counts == [0, 0, 0], f"hands per image {counts}"


@fixture("MON", "MON_two_hands_still_hands_are_reported",
         "fixtures/frames/two_hands_1.png, a still of this rig with two hands in the workspace (a code-path check): the "
         "monitor must report two hands, each L or R, its palm centre inside the frame, landmarks_px = 21 pixel pairs on the wire "
         "(the mean of pairs 0, 1, 5, 9, 13, 17 is its palm_px), in a well-formed message that fits one UDP datagram")
def _sample():
    if not SAMPLE.is_file() or hashlib.sha256(SAMPLE.read_bytes()).hexdigest() != SAMPLE_SHA256:
        raise Unknown("the two-hands still is missing or not the pinned one")
    msg, lms = hm.message_for_still(SAMPLE, detector(), table_map())
    probs = wire.validate_hand_msg(msg)
    hands = msg["hands"]
    pairs = [h.get("landmarks_px") for h in hands]
    pairs_ok = all(isinstance(lm, list) and len(lm) == 21 and all(isinstance(p, list) and len(p) == 2 for p in lm)
                   for lm in pairs) and pairs == lms
    palm_ok = pairs_ok and all(max(abs(a - b) for a, b in zip(palm_centre_px(h["landmarks_px"]), h["palm_px"])) <= 0.15
                               for h in hands)
    size = len(wire.encode(msg))
    ok = (len(hands) == 2 and probs == [] and pairs_ok and palm_ok and size <= MAX_UDP_PAYLOAD
          and wire.decode(wire.encode(msg)) == msg
          and all(0 <= h["palm_px"][0] < 640 and 0 <= h["palm_px"][1] < 480 for h in hands)
          and msg["frame_sha256"] == SAMPLE_SHA256 and msg["model"] == hm.model_name())
    return ok, (f"{len(hands)} hands {[(h['handedness'], h['conf'], h['palm_px']) for h in hands]}; landmarks_px "
                f"{[None if lm is None else len(lm) for lm in pairs]} pairs; schema problems {probs or 'none'}; datagram "
                f"{size} bytes (limit {MAX_UDP_PAYLOAD}); proc {msg['proc_ms']} ms")


@fixture("MON", "MON_model_file_changed_refused",
         "the model file with one byte changed against MODEL.lock: the lock check raises and the monitor's main returns "
         "2 before it reads a frame; the untouched file passes the same check")
def _model_lock():
    good_path, good_sha = model_lock.locked_model()
    d = Path(tempfile.mkdtemp(prefix="khv_lock_"))
    lock = model_lock.read_lock()
    data = bytearray(good_path.read_bytes())
    data[1000] ^= 0x01
    (d / lock["file"]).write_bytes(bytes(data))
    (d / "MODEL.lock").write_text(json.dumps(lock), encoding="utf-8")
    keep = (model_lock.MODELS, model_lock.LOCK)
    model_lock.MODELS, model_lock.LOCK = d, d / "MODEL.lock"
    try:
        try:
            model_lock.locked_model()
            raised = False
        except model_lock.ModelLockError:
            raised = True
        rc = hm.main(["--once", str(SAMPLE)])
    finally:
        model_lock.MODELS, model_lock.LOCK = keep
    return raised and rc == 2 and good_sha == lock["sha256"], f"changed file refused: {raised}; main rc {rc}; pinned file ok"


# ======================================================================================== MON: the publishing loop
@fixture("MON", "MON_one_message_per_new_frame_none_for_a_repeat",
         "the loop on a temp frames dir and a loopback socket: frame seq 1 -> one message; the same seq written again -> "
         "nothing for 0.4 s; seq 2 -> the second message. Each is well formed, names its frame's seq and sha256, and "
         "the message seq counts 1, 2")
def _one_per_frame():
    lp = Loop()
    try:
        a, b = jpg(blank_table(1)), jpg(blank_table(2))
        m1_meta = wire.write_frame(lp.dir, a, 1, time.time())
        m1 = lp.recv(3.0)
        wire.write_frame(lp.dir, a, 1, time.time())
        repeat = lp.recv(0.4)
        m2_meta = wire.write_frame(lp.dir, b, 2, time.time())
        m2 = lp.recv(3.0)
    finally:
        lp.close()
    ok = (m1 is not None and m2 is not None and repeat is None
          and not wire.validate_hand_msg(m1) and not wire.validate_hand_msg(m2)
          and (m1["seq"], m1["frame_seq"], m1["frame_sha256"]) == (1, 1, m1_meta["sha256"])
          and (m2["seq"], m2["frame_seq"], m2["frame_sha256"]) == (2, 2, m2_meta["sha256"])
          and m1["frame_sha256"] == wire.sha256_bytes(a) and m1["frame_t"] == m1_meta["t"])
    return ok, (f"message 1 {None if m1 is None else (m1['seq'], m1['frame_seq'])}, repeat -> "
                f"{'nothing' if repeat is None else 'A MESSAGE'}, message 2 {None if m2 is None else (m2['seq'], m2['frame_seq'])}")


@fixture("MON", "MON_undecodable_frame_is_silence",
         "a frame pair whose jpg is not an image (its json names its hash correctly): nothing is published for it - the "
         "runner sees silence, not an empty table; the next good frame is published")
def _undecodable():
    lp = Loop()
    try:
        wire.write_frame(lp.dir, b"not a jpeg at all", 1, time.time())
        bad = lp.recv(0.5)
        wire.write_frame(lp.dir, jpg(blank_table(3)), 2, time.time())
        good = lp.recv(3.0)
        n_bad = lp.stats.undecodable
    finally:
        lp.close()
    ok = bad is None and good is not None and good["frame_seq"] == 2 and good["seq"] == 1 and n_bad == 1
    return ok, f"undecodable frame -> {'nothing' if bad is None else 'A MESSAGE'} (counted {n_bad}); next frame -> seq {None if good is None else good['seq']}"


@fixture("MON", "MON_rate_with_frames_at_15hz",
         "frames written at 15 Hz for 4 s, alternating a clear table and a picture with a hand: the loop must publish at "
         "10 Hz or more, with no gap between messages longer than 200 ms (never below 5 Hz)", kind="control")
def _rate():
    lp = Loop()
    frames = [jpg(blank_table(4)), jpg(sample_frame())]
    try:
        wire.write_frame(lp.dir, frames[0], 0, time.time())
        lp.recv(3.0)                                   # the first detection warms the model up; not timed
        lp.stats.t_sent.clear()
        lp.stats.proc_ms.clear()
        t0 = time.monotonic()
        for k in range(1, int(4 * FEED_HZ) + 1):
            wire.write_frame(lp.dir, frames[k % 2], k, time.time())
            time.sleep(max(0.0, t0 + k / FEED_HZ - time.monotonic()))
        time.sleep(0.2)
        sent, proc = list(lp.stats.t_sent), sorted(lp.stats.proc_ms)
        sizes = []
        while (m := lp.recv(0.0)) is not None:
            sizes.append((len(wire.encode(m)), len(m["hands"])))
    finally:
        lp.close()
    if len(sent) < 2:
        return False, f"{len(sent)} message(s) in 4 s"
    hz = (len(sent) - 1) / (sent[-1] - sent[0])
    gap = 1000.0 * max(b - a for a, b in zip(sent, sent[1:]))
    ok = hz >= TARGET_HZ and gap <= 1000.0 / MIN_HZ and len(sizes) == len(sent) and max(s for s, _ in sizes) <= MAX_UDP_PAYLOAD
    return ok, (f"{len(sent)} messages for {int(4 * FEED_HZ)} frames ({len(sizes)} received): {hz:.1f} Hz, longest gap "
                f"{gap:.0f} ms; proc_ms p50 {proc[len(proc) // 2]:.1f} p95 {proc[int(0.95 * len(proc))]:.1f} max "
                f"{proc[-1]:.1f}; largest datagram {max(sizes)[0]} bytes ({max(sizes)[1]} hand(s))")


# ======================================================================================== FRAME: the saved stills
def _stills(scene: str) -> list:
    got = sorted((paths.FIXTURE_FRAMES).glob(f"{scene}_*.png"))
    if not got:
        raise Unknown(f"{FRAME_NOT_CAPTURED}: fixtures/frames/{scene}_<k>.png")
    return got


def _read_still(p: Path) -> list:
    """The still's hands as PNG and as the JPEG the live path sees: [(label, hands)]."""
    img = cv2.imread(str(p), cv2.IMREAD_COLOR)
    if img is None:
        raise Unknown(f"{p.name} cannot be read as an image")
    as_jpg = cv2.imdecode(np.frombuffer(jpg(img), np.uint8), cv2.IMREAD_COLOR)
    return [(f"{p.name}", hm.hands_of(detector(), table_map(), img)[0]),
            (f"{p.name} as jpg", hm.hands_of(detector(), table_map(), as_jpg)[0])]


def _check(hands: list, want: dict, tol: float) -> tuple:
    if len(hands) != want["hands"]:
        return False, f"{len(hands)} hand(s), expected {want['hands']}"
    if want["hands"] != 1 or "palm_mm" not in want:
        return True, f"{len(hands)} hand(s)" + (f" {[(h['palm_mm'], h['in_spawn'], h['in_zone']) for h in hands]}" if hands else "")
    h = hands[0]
    off = float(np.linalg.norm(np.subtract(h["palm_mm"], want["palm_mm"])))
    flags = all(h[k] == want[k] for k in ("in_spawn", "in_zone", "in_workspace") if k in want)
    return (off <= tol and flags), (f"palm_mm {h['palm_mm']} is {off:.1f} mm from {want['palm_mm']} (tolerance {tol:g}); "
                                    f"in_spawn {h['in_spawn']} in_zone {h['in_zone']} in_workspace {h['in_workspace']}")


def _frame_fixture(scene: str, want: dict, tol: float):
    def fn():
        results = [(label, *_check(hands, want, tol)) for p in _stills(scene) for label, hands in _read_still(p)]
        return all(ok for _, ok, _ in results), "; ".join(f"{label}: {'ok' if ok else 'NO'} - {d}" for label, ok, d in results)
    return fn


@fixture("MON", "MON_saved_still_check_fires_and_passes",
         "the saved-still check itself, on made-up readings and one temp still: a palm 16 mm from its cross, a wrong "
         "flag and a wrong hand count are each failed; 14 mm with the right flags passes; a still is read twice (PNG "
         "and the JPEG the live path sees)")
def _still_check():
    want = {"hands": 1, "palm_mm": [241.1, -58.8], "in_spawn": True, "in_zone": False, "in_workspace": True}
    hand = lambda dx, **kw: {"palm_mm": [241.1 + dx, -58.8], "in_spawn": True, "in_zone": False,  # noqa: E731
                             "in_workspace": True, **kw}
    far, near = _check([hand(16.0)], want, 15.0)[0], _check([hand(14.0)], want, 15.0)[0]
    flag = _check([hand(0.0, in_zone=True)], want, 15.0)[0]
    count = _check([], want, 15.0)[0], _check([hand(0.0), hand(0.0)], want, 15.0)[0], _check([hand(0.0)], {"hands": 0}, 15.0)[0]
    p = Path(tempfile.mkdtemp(prefix="khv_still_")) / "sample_1.png"
    cv2.imwrite(str(p), sample_frame())
    read = _read_still(p)
    ok = (not far) and near and (not flag) and not any(count) and [lbl for lbl, _ in read] == ["sample_1.png", "sample_1.png as jpg"] \
        and all(len(h) >= 1 for _, h in read) and _check([], {"hands": 0}, 15.0)[0]
    return ok, (f"16 mm passes: {far}; 14 mm passes: {near}; wrong flag passes: {flag}; wrong counts pass: {list(count)}; "
                f"a still read as {[lbl for lbl, _ in read]} -> {[len(h) for _, h in read]} hand(s)")


def _register_frames() -> None:
    exp = json.loads(EXPECTED.read_text(encoding="utf-8"))
    tol = float(exp["tolerance_mm"])
    for scene, line in exp["scenes"].items():
        want = line.get("monitor")
        if want is None:
            continue
        if want["hands"] == 0:
            what, kind = f"the saved stills of '{scene}': no hand may be reported in any", "control"
        elif "palm_mm" in want:
            what, kind = (f"the saved stills of '{scene}': one hand, palm_mm within {tol:g} mm of {want['palm_mm']}, "
                          f"in_spawn {want['in_spawn']}, in_zone {want['in_zone']}"), "negative"
        else:
            what, kind = f"the saved stills of '{scene}': {want['hands']} hand entries in every still", "negative"
        fixture("FRAME", f"FRAME_{scene}", what, kind)(_frame_fixture(scene, want, tol))


_register_frames()


if __name__ == "__main__":
    sys.exit(1 if run_all(REGISTRY) else 0)
