# Trust-layer demonstration — SO-101

**A demonstration** on an SO-101 arm with a **plastic prop tool** (a screwdriver); the layer-off clip uses a **prop
hand**. Nothing here is part of any pre-registered evaluation.

[![license: Apache-2.0](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)
![python: 3.12](https://img.shields.io/badge/python-3.12-3776AB.svg?logo=python&logoColor=white)
![arm: SO-101](https://img.shields.io/badge/arm-SO--101-orange.svg)
![judge: Claude Haiku 4.5](https://img.shields.io/badge/judge-Claude%20Haiku%204.5-8A63D2.svg)
![log: hash-chained](https://img.shields.io/badge/log-hash--chained-2ea44f.svg)

[Video](#video) · [The run, step by step](#the-layer-on-run-step-by-step) · [Architecture](#architecture) ·
[Findings](#findings) · [Lines that do not move](#lines-that-do-not-move) · [Verify a chain](#verify-a-chain) ·
[Running it](#running-it)

An SO-101 (five arm joints and a gripper) hands a screwdriver to a person: it picks the tool up, and when a hand comes
into the workspace it freezes, has a vision-language judge read the frame, plans a placement that turns the pointed
end away from the hand, lays the handle in the open palm and lets go. Around that motion sits a *trust layer*: a
separate hand-monitor process, eight per-step monitors, the judge, a planner with hard geometric rules, and
freeze-and-ask. Every control step, check, verdict and key press is a row of a hash-chained log, and the replay and
presentation pages are built from that log alone.

## Video

[![The trust-layer demonstration on the SO-101: layer off on the left, layer on on the right](https://img.youtube.com/vi/eBQKAU_5WWM/maxresdefault.jpg)](https://youtu.be/eBQKAU_5WWM)

**[Watch on YouTube](https://youtu.be/eBQKAU_5WWM)** — layer off on the left, layer on on the right: each take's
camera footage under the panel built from its session's log.

- The panels used in it, from the logs: `sessions/KH-S20261005T015041/present.html` (layer on) and
  `sessions/KH-S20261004T174200/present.html` (layer off). Open either in a browser; space plays at 1×.
- To render a panel as an MP4 (1×, from the first motion to the end, real time):
  `node replay/render_mp4.mjs sessions/<session>` → `present_1x.mp4` beside it. Videos are not committed
  (`*.mp4` is ignored).

## The layer-on run, step by step

Session `KH-S20261005T015041` (2026-10-05, 01:51 IST), palm placement, pre-authorised at trial start. Times are from
the first motion.

1. **Start checks**, each a row: the frozen registration, layout and null plan by canonical hash; the calibration
   file; the anchor modules by sha256; the kill switch verified; the G3 and handover fault tables; the fixture gate
   (overridden here, see *Findings*); the camera framing against its reference; the ruler and the clamps; the hand
   mapping (a palm on spawn cross 6 read 7.7 mm from it, limit 15 mm); every scripted row checked by FK against the
   joint margins, the workspace box and the camera's footprint.
2. **The ARM statement**, built from this session's own numbers (speeds, segments, mode, the override stamp), is
   printed; nothing has torque until the operator types `ARM`.
3. **Set-up to rest** was refused at its start: a hand was in the workspace (monitor 7). The arm held, the judge read
   the frame, the operator typed `[c]` once the workspace was clear.
4. **Grasp** at ≤ 3 deg/s; the jaws stop on the rubber handle at 12.44 % (target 9.0 %) with a gripper load of 220:
   the grasp check (≥ 1.5 % short of the target and load ≥ 80) says held.
5. **Lift**, then **CP1**: the arm still, the judge reads the scene (1.81 s): proceed, p_unsafe 0.05.
6. **Carry** toward the zone. The operator's hand comes in: monitor 7 fires on one raw frame and the arm is held
   **81 ms** after that frame (every goal := the present position).
7. **Settle 1 s, then the judge** with the placement rubric (2.13 s): wait, p_unsafe 0.40. Not UNSAFE (the code's
   bar is 0.5), so the placement the operator pre-authorised at trial start goes ahead without a key.
8. **The planner**: no placement for the first reading of the palm within its 25 s budget. An automatic retry (a
   settle, the judge again: 0.40, a fresh reading of the hand) finds one — turn the screwdriver 76° so the pointed
   end faces away from the hand, 764 rows, the pointed end never nearer than **41.8 mm** to the palm in 3D (the rule:
   ≥ 30 mm on every row). While frozen the gripper had gone slack (load 44); the plan's first rows squeeze it back to
   its target (load 220).
9. **Approach** at ≤ 10 deg/s, ≤ 3 deg/s within 150 mm of the palm. Above the palm the hand is read again (208
   sightings): it had moved to (267, 59) mm, and no placement exists for that spot within 25 s, so the arm froze and
   asked. Judge: p_unsafe 0.35. The operator's `[h]`: a settle, a fresh judge call (0.30), a new plan — turn 13°,
   161 rows, the pointed end ≥ 140.8 mm from the palm.
10. **Approach and descent**: the hand read still above the palm; the descent at 3 deg/s to 20 mm above the palm
    (commanded 16 mm higher for the arm's measured sag).
11. **Settle 1 s, then CP3**: the judge (2.05 s): proceed, p_unsafe 0.25, the lowest point 25 mm above the palm. The
    gripper opens without a key (pre-authorised); a fist, a second hand or an UNSAFE verdict would have asked for
    `[r]` instead.
12. **Lift-off** straight back up the descent, **wait** for 5 fresh frames with no hand (0.27 s), **retreat** to rest.
13. **"Did you receive it?"** — yes: `HANDED_OVER`. In the trial: two freezes (the hand coming in, and the refused
    re-plan above the palm), six judge calls at 1.8–2.3 s each; control step p95 66.67 ms against a limit of 89.9 ms;
    the chain verifies; every step row carries the override stamp.

**The layer-off run** (`KH-S20261004T174200`, `--layer-off`) is the same pick-and-place with the hand monitor
(monitors 7 and 8), the judge and the planner off and a prop hand in the place zone: grasp, lift, carry, lower into
the zone. Monitor 2 froze the arm once in the place (`shoulder_pan` read at 19.8 deg/s against 15); the operator's
`[c]` continued; `PLACED`. Every row of that chain carries `"trust_layer": "TRUST LAYER OFF — DEMONSTRATION"`.

## Architecture

```
camera ──► frame pump (in the runner, 15 Hz) ──► /tmp/khv/frame.jpg + frame.json
                                                     │
                                                     ▼
              hand monitor — its own process and venv (MediaPipe hand landmarks)
                                                     │  UDP 127.0.0.1:47101, one message per new frame:
                                                     ▼  seq, frame hash, per hand: palm (mm), 21 landmarks, flags
runner (handover_session.py), 15 Hz control loop
  every step: joints + gripper load → the eight monitors → step row (hash-chained) → the goal is sent
  ├── judge (judge/): at CP1, CP2, CP3 and every freeze — one saved frame + context + rubric → a verdict row
  ├── planner (planner/): placement and standoff plans, FK-checked row by row before anything moves
  └── freeze-and-ask (motion.py, handover_flow.py, placement_flow.py, ask_menu.py)
```

- **Monitor process → UDP heartbeat → runner.** The runner owns the camera and writes frames; the hand monitor
  (`monitor/hand_monitor.py`, MediaPipe, `.venv_monitor`) reads them and publishes one message per new frame. In the
  runner, monitor 8 fires when the last message is over 300 ms old, the sequence skips more than 2, or the message's
  frame is stale or unknown; its watchdog sets the stop flag. Monitor 7 fires on one raw frame with a hand in the
  workspace, or 3 of the last 5, and clears only at 0 of 5. Monitors 1–5 (joint margin, joint speed, workspace box,
  gripper overload, loop time) are the anchor's envelope guard; monitor 6 is the operator's ESC / Ctrl-C.
- **Judge at checkpoints, with logged latency.** `judge/judge.py` sends one saved frame, the code's own reading of
  the scene (hands seen, open or a fist from the landmarks, distances across the table and the height above the
  palm from the joint encoders) and a rubric (`judge/rubric*.md`). The model is chosen by a written rule from the
  provider's list (here `claude-haiku-4-5-20251001`); the budget is 8 s. A timeout, a malformed reply or
  p_unsafe ≥ 0.5 is UNSAFE by the code's reading. The judge recommends; UNSAFE holds every automatic step. Each call
  is a row with the model, latency, rubric sha256, frame sha256 and context.
- **Planner with the 3D pointed-end clearance.** `planner/placement.py` puts the middle of the handle over the palm
  and the pointed end past the fingertips, turning the tool first when the pointed end faces the hand; on every row
  the pointed end stays ≥ 30 mm from the palm in 3D, the release is 20 mm above the palm, moves near the hand are
  ≤ 3 deg/s and ≤ 10 deg/s elsewhere. A plan made while holding squeezes the gripper back to its target.
  `planner/handover_planner.py` is the standoff variant (handle tip ≥ 30 mm from the palm, pointed end ≥ 150 mm).
- **Freeze-and-ask.** Any monitor firing: the stop flag, every goal := the present position (retried), the freeze
  row with the frame hash and the hold latency, a photo, the judge on the frozen frame, then a menu — `[w]` wait,
  `[h]` place, `[r]` release, `[c]` continue (only with every monitor clear), `[k]` torque off (`OFF`, then `YES`),
  `[l]` leave as is. Every key is a row with the verdict that was on screen. The runner never moves the arm by itself
  after a freeze; the one exception is the operator's own pre-authorisation at trial start, and it still passes the
  judge and every monitor.
- **Operator override stamps.** `--operator-override` runs code whose fixture gate (`fixtures/status.py`) is not
  green for this exact tree: the ARM statement says `UNTESTED CODE — OPERATOR OVERRIDE` and every step row carries
  it. `--layer-off` stamps every row of the chain `TRUST LAYER OFF — DEMONSTRATION`.

## Replay and presentation pages

```
python replay/build_replay.py sessions/<session>
```

verifies the chain first (the result is printed on the page), checks every photo against the sha256 in its row,
and writes two self-contained pages beside the chain (no network):

- `replay.html` — the session's facts and limits, a timeline with lanes (phase, monitors 1–8, hand-link age, judge
  calls, keys), the frame at each freeze with the zone, the boxes and the reported palm drawn on it, the fingertip
  path in 3D from the step rows' FK, and the call log in chain order.
- `present.html` — the same session for video, 1920×1080: the 3D path with a state badge (MOVING, FREEZE — monitor
  N with its hold time, JUDGE with the verdict, RE-ORIENT, RELEASE, WAIT), the timeline with a now-line, and the
  call log scrolling as it plays; 1× / 2× / 4×, from the first motion. Step rows carry their own clock; every other
  row carries a wall time to the second and is placed between its neighbouring steps. Stretches with no row longer
  than 30 s play in 4 s unless `?real=1`; `?layout=portrait` gives a 720×1080 panel.
  `node replay/render_mp4.mjs sessions/<session>` renders it to MP4.

`replay/live.py` rebuilds the newest session's `replay.html` every few seconds while it runs.

## Findings

| Finding | What the logs show | What was done |
|---|---|---|
| The arm sags 12–20 mm with the tool | Measured fingertip against commanded: 19.5–20.4 mm (z −15.6 to −15.9) lifted at the grasp, 12.3–13.0 mm over the zone; a 20 mm lift test measured 3.8 mm. | The release is commanded 16 mm higher. The pointed-end check reads the commanded rows, so the real tool can be up to ~20 mm nearer a palm than computed — stated, not corrected. |
| Occlusion under the arm | The overhead camera loses the hand under the screwdriver; readings under the arm are 12–73 mm off with the hand still; three freezes in one trial on "the approved hand is not seen" with the arm 52–64 mm away. | Within 45 mm of the planned palm the hand may stay unseen; elsewhere 1 s, then freeze-and-ask. A hand that moves while covered is caught by the descent's 15 mm rule, not by a re-plan. |
| Judge misreads | The red handle held over an open palm read as the person gripping something (13:16–13:18, 2026-10-04); a 2 mm horizontal distance read as touching; open hands on saved stills read as not waiting (`hand_zone_1`, `hand_zone_3`, `hand_cross6_3`). | The judge still gates. Its context now carries the code's hand reading and the height above the palm; re-asked with it, 6 of the 7 13:16 frames were not UNSAFE (the 7th had no hand). Misses on the stills are reported by the fixture gate, not gating. |
| 97 ms from frame to hold | A hand came in at the handle during a grasp descent (2026-10-03): monitor 7 fired on one raw frame and the arm held 97 ms after that frame. In the final layer-on take: 81 and 85 ms. | Kept: one raw frame is enough to freeze. |
| Speed-limit change | Monitor 2 at 9 deg/s fired on plans commanded at 3 deg/s (encoder noise). | 15 deg/s for rows commanded ≤ 3 deg/s; 30 deg/s while a 10 deg/s row ran in the last 8 rows. In the layer-off take it fired at 19.8 deg/s in the place. |
| Override rows | 58 of the 70 arm sessions, from 2026-10-04 00:10 on, ran under `--operator-override`: the fixture gate was last all green on 2026-10-03 22:13, on earlier code. | All 59,912 step rows of those sessions carry `"override": "UNTESTED CODE — OPERATOR OVERRIDE"`; the final takes are among them. |
| Also | The gripper servo's own overload protection tripped at a load of 240–244 held ~2 s, under monitor 4's 250 bar; after a freeze the stalled gripper went slack (load 180 → 0). | Close target 9.0 %, held 10 s at 9.4 % in a hold test; plans re-squeeze the grip. |

## Lines that do not move

- Nothing under `anchor/`, `phase0/` or `so101_sim/` is written; an output folder that resolves there is refused.
- A safety event freezes and asks. The runner never drives the arm after a freeze on its own.
- The judge recommends; the code decides by fixed rules. A timeout or p_unsafe ≥ 0.5 is UNSAFE and holds every
  automatic step.
- Every command is a step row, written before it is sent; the log is hash-chained and verified before any replay.
- No torque without `ARM` typed to a statement built from the session's own numbers; torque off only after `OFF`,
  then `YES`.
- ≤ 3 deg/s near the hand and on the descent; the pointed end ≥ 30 mm from the palm in 3D on every planned row.
- Monitors 1–6 are never off. Monitors 7 and 8 are off only with `--layer-off`, stamped on every row.
- Untested code runs only under `--operator-override`, stamped on every step row.
- A plastic prop tool only; a prop hand in the layer-off clip.
- No secret in the repository, a log or a transcript: the API key lives in `.env` (ignored) and is never read into
  a log.
- It is called a demonstration, and nothing here is presented as a safety certificate.

## Verify a chain

```
python3 verify_chain.py sessions/KH-S20261005T015041
```

Standard library only. Row *n* must have `seq` *n*; its `prev_sha256` must be the previous row's `row_sha256` (null
for the first); its `row_sha256` must be the sha256 of the row without that field, serialised as JSON with sorted
keys and no spaces. Then every photo or frame a row names is hashed against the sha256 that row carries. The same
three chain rules as `anchor/g3.py`'s `verify_chain`, which the runner and the replay builder use.

## Running it

Runs against the ARM-ANI anchor tools; see [robotics_research_arm_ani](https://github.com/aniketmallick/robotics_research_arm_ani).
This folder expects to sit at
`experiments/knife_handover` inside that tree and reads, never writes:

- `../../anchor/` — imported by path: `g3.py`, `safety.py`, `fk.py`, `fake_bus.py`, `null_plan.py`,
  `framing_recheck.py`, `check_framing.py`, `session_precheck.py`;
- `../../phase0/` — the frozen registration, layout, null plan, anchor measurements and calibration, and the G3
  fault logs;
- `../../so101_sim/` — the kinematics model the planner's IK uses.

None of these is in this repository. Also needed: Python 3.12 with the LeRobot stack (the runner's venv,
`~/lerobot-mps-venv`, with NumPy, OpenCV, PyAV); a separate venv for the hand monitor (`monitor/requirements.txt`);
an Anthropic API key in `.env` for the judge (a dry run uses a stub); node ≥ 22 and Chrome for MP4 renders.

```
python handover_session.py --dry-run --yes                 # a rehearsal: fake bus, synthetic frames, stub judge
python fault_table_handover.py                             # the handover fault table (the arm refuses without it)
python fixtures/status.py                                  # every suite; --arm refuses unless all green for this tree
python handover_session.py --arm --port <port> --segments dawn/segments_reversed.json --palm-placement \
    --z-handover-mm 55 --knife-handle-mm 43.2 --knife-blade-mm 123.8 --knife-width-mm 21 --knife-axis-sign -1
```

## Repository map

| Path | What it is |
|---|---|
| `handover_session.py`, `handover_flow.py`, `placement_flow.py`, `motion.py`, `monitors.py`, `ask_menu.py`, `state_machine.py` | The runner: session, trial, freeze-and-ask, the control loop, the eight monitors, the menus, the states |
| `monitor/` | The hand monitor (own venv) and its fixtures |
| `judge/` | The judge, its rubrics and fixtures |
| `planner/` | IK, the standoff and placement planners and their fixtures |
| `dawn/` | The recorded scripted segments, the measured tool and lay |
| `replay/` | `replay.html`, `present.html`, the MP4 renderer, the live view |
| `sessions/` | Three arm sessions: `KH-S20261004T152718` (a handover), `KH-S20261004T174200` (layer off), `KH-S20261005T015041` (layer on) — chain, photos, pages |
| `tests/`, `fixtures/` | The runner's fixtures and the gate (`fixtures/status.py`) |
| `PLAN.md`, `STATUS.md` | The plan with its rulings, and the running record of what passed, failed and was found |

## License

Apache License 2.0 — see `LICENSE`.

Third-party file: `monitor/models/hand_landmarker.task` is Google's MediaPipe Hand Landmarker model (float16,
version 1), distributed by Google under the Apache License 2.0 and pinned by sha256 in `monitor/models/MODEL.lock`.
