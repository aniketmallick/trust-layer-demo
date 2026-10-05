# Prop-knife handover on the SO-101 — PLAN (demonstration)

Written 2026-10-01 12:25 IST, before any code. This is a **demonstration**. Nothing here is session evidence for
PR-001 / CERT-001, and nothing it produces is presented as a safety certificate. A prop knife only (plastic or wood).

Everything lives in `experiments/knife_handover/`. `anchor/`, `phase0/`, `so101_sim/` are read and imported, never
written: every entry point sets `sys.dont_write_bytecode = True` before importing them, and nothing here calls
`anchor/run_anchor_session.sh` (it installs missing packages into the lerobot venv and writes rulings into the record).

## 1. What the frozen code says where the prompt says something else

The code wins on each of these; each is also an open question in section 8.

| # | Prompt | Found | What the plan does |
|---|---|---|---|
| F1 | "the registration's camera-to-table homography … if stored sheet-to-pixel, invert it" | r2 stores **no homography**. It stores a similarity map, **pixel → arm-frame mm**: `p_arm = R(6.16°)·(v·s, u·s) + (196.1, −199.8)`, `s = 0.54669 mm/px` (`a2b.mm_per_native_px`, `arm_frame.registration`). It is the map `null_session.CamMap` uses. No lens undistortion in either. | `common/table_map.py` re-implements that map (no inversion needed) and is tested equal to `CamMap`. |
| F2 | "my hand … centred on a named cross of the place zone" | `layout_v3.place_zone` is "empty inside". The named crosses 1–8 are in the **spawn box**. | Accuracy fixture on spawn **cross 6** (arm 241.1, −58.8 mm; px 247.6, 109.5), which is also the spawn-box frame; the zone frame is checked against the registered **zone centre** (246.8, 68.1 mm; px 477.3, 144.8). |
| F3 | "the existing 3°/s scaled limit" | 3 deg/s exists only in `g3_live_faults.py`, as the **speed monitor's limit scaled down for one test**. No motion limit of 3 deg/s exists. The null replays at up to 2.86 deg/step × 15 Hz = 43 deg/s; set-up moves run at 10 and 30 deg/s; the production speed guard is 292 deg/s. | Handover segments are interpolated at ≤ 3 deg/s per arm joint. Scripted pick/place segments at ≤ 10 deg/s (the anchor's `REST_DPS`), not the null's 43. Monitor 2 is scaled per phase (section 4). |
| F4 | "`[k]` kill (torque off …)" | `safety.py`: torque off needs **two confirmations** (`OFF`, then `YES`) because the arm drops; the menu also has **leave as is** (torque on, no further commands). | `[k]` keeps the double confirmation. `[l]` leave-as-is is added to the menu. |
| F5 | "runner freezing within 300 ms" of a monitor kill, and "heartbeat older than 300 ms" | Both cannot hold: staleness is only known 300 ms after the last message. | Watchdog thread sets the stop flag when age > 300 ms (checked every 10 ms); `SafeBus` refuses every target from that instant; goals := present at the next control step (≤ 67 ms). The test reports both times. |
| F6 | "reuse `SO101Kinematics` if it has an IK" | It has `solve()` (damped least squares, position + approach axis, roll free), and `mujoco 3.8.1` is in the lerobot venv. Its chain equals `anchor/fk.py` (0.000 mm over 200 poses), but its `tcp` site is **13.5 mm** from `gripperframe`, the fingertip the registration is fitted to. `anchor/null_plan.py` also carries a numpy IK (`ik()`) on `fk.py`: the one the frozen null plan was computed with. | C reuses `SO101Kinematics.solve` as instructed, with the fixed tool offset, and checks every solution through `anchor/fk.py`. Fallback if not green by M1: `null_plan.ik`. |
| F7 | "session-start checks as the null runner does them" | `g3.session_start` reads and the re-check / pre-check tools **write** `phase0/anchor/*.jsonl`, and it requires a base-guard row (a motion step). | The demo runs the same checks by importing `framing_recheck` functions and writes its own rows under `sessions/`. No base guard: stated in the session-start row. |
| F8 | "every … joint target goes into an append-only hash-chained log" | `g3.ProvenanceLog.append` re-verifies the whole file on every append (fine for 40 trial rows; too slow per control step, limit 89.9 ms). | `steplog.py`: same row format, tail hash cached, O(1) append; verified by the anchor's `g3.verify_chain`, unchanged. |
| F9 | commit at M2 | `cert_deploy_robotics/` is not a git repository; no `ARM-ANI` checkout found under `~/Documents`. | Blocked on the operator (Q7). |
| F10 | MediaPipe "current 0.10.x line" | PyPI latest is 1.0.1; the 0.10.x line ends at 0.10.35. | Pin `mediapipe==0.10.35`. |

Facts used below (computed from the record, read-only): workspace box x [171.9, 368.6] y [−166.2, 143.0] z [−27, 283] mm
(arm frame); loop-time limit 89.9 ms; camera nadir (315.4, −16.3) mm, height 347 mm; camera footprint corners
(196.1, −199.8) (158.6, 147.5) (419.0, 175.6) (456.5, −171.7) mm — **the box's near edge (x < ~196 mm, a strip up to
37 px) is outside the frame**; runner interpreter `~/lerobot-mps-venv/bin/python` (3.12.13: numpy 2.2.6, cv2, mujoco
3.8.1, httpx 0.28.1, pytest 9.1.1; no anthropic/openai SDK, no python-dotenv).

## 2. State machine

```
IDLE → GRASP → LIFT → CP1(judge) → APPROACH_ZONE → PRE_PLACE → CP2(judge) → PLACE → RETREAT → IDLE
any moving state, on any monitor 1–8:
  FREEZE (goals := present, within the control step) → judge reads the frozen scene → ASK
  ASK: [w] WAIT       re-read monitors + judge → ASK again; [c] continue is offered only when all 8 are clear
       [h] HANDOVER_PLAN → HANDOVER_APPROACH → HOLD → CP3(judge) → ASK_RELEASE → [r] RELEASE → (hand gone) → RETREAT
       [o] REORIENT → HANDOVER_PLAN → …
       [k] KILL       torque off after OFF + YES; trial ABORTED
       [l] LEAVE      torque on, no further commands; trial ABORTED
```

- Moving states: GRASP, LIFT, APPROACH_ZONE, PRE_PLACE, PLACE, RETREAT, REORIENT, HANDOVER_APPROACH, RELEASE.
  Stationary: IDLE, CP1–3, FREEZE, ASK, WAIT, HANDOVER_PLAN, HOLD, ASK_RELEASE.
- FREEZE is `kill.trigger(source)` then `SafeBus.hold_present()`; the freeze row is written before the menu. Nothing is
  commanded after a freeze until a key is pressed. No state drives to rest by itself (G3.4).
- A stationary state may enter a moving state only if monitors 7 and 8 are clear at that moment; otherwise FREEZE.
- `[h]` is refused by code (back to ASK, reason shown) when the planner returns `None`, when more than one hand is
  present, when monitor 8 is firing, or when the blade heading is toward the palm (then `[o]` is the path).
- HANDOVER_APPROACH: one hand whose palm is within 50 mm of the palm **approved at `[h]`** does not fire monitor 7; the
  plan re-targets silently. Drift is measured from the approved palm (cumulative), not from the last re-target. Drift
  ≥ 50 mm, a second hand, or any monitor 1–6 or 8 → FREEZE → ASK.
- HOLD: the arm is stationary; the hand may move to the handle. RELEASE opens the gripper only. RETREAT starts only
  when monitor 7 is clear (0 of the last 5 frames); otherwise the arm holds and asks.
- The judge is called in CP1, CP2, CP3 and FREEZE only: arm stationary, 8 s budget, never inside the control loop.
  Timeout, malformed reply or `p_unsafe ≥ 0.5` = UNSAFE → at a checkpoint the arm stays frozen and ASK is shown. The
  judge's `recommend` is printed beside the menu and logged; no key is pressed for the operator.
- ESC / Ctrl-C = FREEZE at any time; second Ctrl-C = hard abort (as `safety.py`). The 300 s operator heartbeat stays.

## 3. Files (one job each)

```
experiments/knife_handover/
  PLAN.md  STATUS.md  .gitignore  .env (operator's, git-ignored, never read into a transcript)
  common/     paths.py (roots, frozen file paths, /tmp/khv, UDP port)   table_map.py (px <-> arm mm, flags)
              wire.py (hand message + frame.json: build, validate, atomic frame write, 3-of-5)
  monitor/    [A] hand_monitor.py  palm.py  requirements.txt  models/MODEL.lock  tests/
              [D] sim_feed.py
  judge/      [B] judge.py  providers.py  env.py  rubric.md  tests/
  planner/    [C] handover_planner.py  ik.py  knife.py  tests/
  [D] handover_session.py  state_machine.py  monitors.py  hand_link.py  frame_pump.py  ask_menu.py
      steplog.py  grasp_segment.py  start_checks.py  fault_table_handover.py
  replay/     [D] build_replay.py  (output: sessions/<id>/replay.html, self-contained)
  fixtures/   capture_frame.py  frames/*.png  frames/manifest.jsonl  expected.json
  sessions/   KH-S<stamp>/ (rig)   _dryrun/KH-DRY-<stamp>/ (rehearsal)
  tests/      integration (D)
```
Fixtures follow `anchor/negative_fixtures*.py`: a name, one sentence of what is injected and what must happen, a
function returning `(as_expected, detail)`, `kind="control"` for the must-not-fire cases. Runner-side tests run under
`~/lerobot-mps-venv/bin/python -m pytest`; monitor tests under `.venv_monitor/bin/python -m pytest`.

## 4. The eight monitors

| # | Code | Rule | Source |
|---|---|---|---|
| 1 | `joint_margin` | every joint inside its calibrated range − 3 deg | `g3.EnvelopeGuard` |
| 2 | `joint_speed` | per phase: 30 deg/s in scripted segments (commanded ≤ 10), **15 deg/s** in handover segments (commanded ≤ 3) | same check, `guard.vel_limit_dps` set on our instance (the hook `g3_live_faults.py` uses) |
| 3 | `ee_workspace` | fingertip inside the box above | `g3.EnvelopeGuard` |
| 4 | `gripper_overload` | load > 50 % of the torque limit for > 0.5 s | `g3.EnvelopeGuard` |
| 5 | `loop_dt` | control step > 89.9 ms | `g3.EnvelopeGuard` |
| 6 | `kill_switch` | ESC / Ctrl-C; 300 s operator heartbeat | `safety.KillSwitch`, `Interlock` |
| 7 | `hand_in_workspace` | **fires** if the latest raw frame has any hand `in_workspace`, or 3 of the last 5 do; **clears** only at 0 of 5. Exception: the approved target, HANDOVER_APPROACH/HOLD, < 50 mm. | new |
| 8 | `hand_link` | last message older than 300 ms, or `seq` gap > 2, or the message's frame older than 500 ms / not a frame the runner wrote | new |

Monitor 2 in handover segments was first set at 9 deg/s. C's control fixture (12:56 IST) replayed 56 plans through
the fault table's servo model (0.09 deg encoder noise, differenced at 15 Hz): `joint_speed` fired on 23 of 56 at
9 deg/s, 3 at 10, 0 at 12, 0 at 15, with the rows commanded at 2.985 deg/s. The limit is 15 deg/s from here on (five
times the commanded speed). What the rig's encoders do at this limit is unknown until dawn.

Choices made toward stopping the arm, written down as asked:
- Monitor 7 fires on one raw frame, not only on 3-of-5 (the prompt's debounce). 3-of-5 holds presence through dropouts.
- `in_workspace` is true if the palm lies in the box for **any** hand height 0–150 mm (one camera cannot see height;
  a hand 100 mm up reads up to ~40 mm outward of where it is). `palm_mm` itself assumes a hand resting on the table
  (palm 20 mm up). The handover therefore needs the hand **on the table**; a raised hand gives a wrong target.
- Monitor 8 also checks the frame's age and hash, so a live monitor reading a stale frame cannot pass as fresh.
- Stopping travel = latency × speed. Budget from frame to hold ≈ 200 ms (raw hit); at 10 deg/s that is 2 deg ≈ 10 mm
  at 300 mm reach, at 3 deg/s ≈ 3 mm. Each freeze row logs `t_hold − frame.t`.

As built by D (13:39 IST), recorded here because each widens or narrows a rule above:
- **Monitor 7's exemption covers four states, not two.** REORIENT is exempt like HANDOVER_APPROACH (one hand, the palm
  approved at `[o]`, < 50 mm cumulative): `[o]` is pressed with the hand in view and could not run otherwise. RELEASE
  is exempt outright: the gripper alone opens, on the operator's `[r]`, with the hand at the handle. HOLD is
  stationary. RETREAT is not exempt and starts only at 0 of 5.
- **The approved hand going missing freezes the approach** when fewer than 3 of the last 5 frames had it; one dropped
  frame does not.
- **`[c]` is not offered once a handover approach or a re-orient has moved the arm** off the scripted path. The exits
  from there are `[h]`, `[o]`, `[w]`, `[k]`, `[l]`.
- **"Holding the prop" is read from the gripper's position** (below the midpoint of the close and release targets). A
  missed grasp reads as holding: `[h]` would then hand over an empty gripper. No sensor on this rig tells the two apart.
- **Silent re-target (reviewer 2026-10-01, SR08 — replaces the paused setpoint):** plans are computed in a worker
  process (`retarget_worker.py`), never on the control thread's clock. A job asks for a plan from the row the
  current plan reaches 8 steps later; the current plan keeps running. Ready at that row: checked (palm within 5 mm,
  first row within 1 deg of the goal sent) and swapped in. Not ready: `retarget pending`, the current plan goes on,
  the late plan is never used. A refused plan or a dead worker freezes. Because the arm keeps moving meanwhile,
  every approach step checks that the knife on the row about to be sent stays >= 55 mm from the palm as it is now
  (60 standoff - 5); else monitor 7 freezes before the send (integrator's addition; kept by the reviewer at M2).
- **Retreat after a release:** the rows that were sent, reversed up to the transit height, then joint-space to rest
  at ≤ 10 deg/s, FK-checked before it runs.
- **`--arm` gate on fixtures (integrator):** `fixtures/status.py` runs every suite and writes
  `sessions/fixture_status.json`; `all_green` needs 0 failed and **0 skipped**, and the file carries a hash of the code
  and fixture frames it ran on; the runner refuses a status written for other code.
- `set_low_acceleration` is applied to the arm joints at torque-on and restored at close (`g3_live_faults.py` does
  this; `null_session.py` does not). Kill rows carry `policy_sha256: null`: this demonstration has no policy.

## 5. Interfaces (restated; changes from the prompt marked ⊕)

**Frame hand-off (runner → monitor).** `frame_pump.py` (a thread in the runner) reads the camera and writes
`/tmp/khv/frame.jpg` and `/tmp/khv/frame.json` = `{"t": epoch_s, "seq": int, "sha256": hex of the jpg}` at ≥ 10 Hz,
each to a temp name then `os.replace`, json after jpg. The monitor reads json, then jpg, and drops the pair if the
jpg's hash is not the json's. (`/tmp` on this Mac is disk, not a ramdisk; named as the prompt names it.)

**Hand message (monitor → runner).** One JSON line per processed frame, UDP `127.0.0.1:47101`, target 10 Hz, never
below 5 Hz. The monitor publishes only when it has processed a **new** frame (no new frame → silence → monitor 8).
```
{"t": epoch_s, "seq": int, "frame_seq": int, "frame_sha256": hex, "frame_t": epoch_s ⊕,
 "hands": [{"handedness": "L|R", "conf": 0-1, "palm_px": [u, v], "palm_mm": [x, y],
            "in_workspace": bool, "in_spawn": bool, "in_zone": bool,
            "landmarks_px": [[u, v] x 21] ⊕ (optional; drawn by the replay, read by no rule)}],
 "debounced": {"hand_in_workspace": bool, "n_of_5": int} ⊕,
 "model": "mediapipe-hands 0.10.35" | "sim_feed", "proc_ms": float}
```
`palm_px` = mean of landmarks 0, 1, 5, 9, 13, 17 (wrist and the five MCPs, as the prompt lists them: thumb CMC stands
for the thumb's base). `palm_mm` = `TableMap.px_to_mm(palm_px, h=20)`, arm frame. `in_spawn` / `in_zone`: inside the
registered quads (arm frame). `handedness` is the model's label; no rule uses it. `build`/`validate` live in
`common/wire.py`; both sides call them.

**Hand link (the receiving half, `common/hand_link.py`, built by A, used by D).**
```
link = HandLink(addr=UDP_ADDR, on_stale=None, known_frame=None, stale_ms=300, max_seq_gap=2, frame_stale_ms=500)
link.start(); link.stop()
link.arm_watchdog(True|False)   # on_stale(reason) is called (once per arming) only while armed: the runner arms it
                                # in moving states and passes kill.trigger; the watchdog thread ticks every 10 ms
link.snapshot() -> {"msg": dict|None, "age_ms": float|None, "seq_gap": int, "frame_age_ms": float|None,
                    "window": [bool x <=5], "problems": [str]}
link.monitor8() -> (fired: bool, detail: str)        # the rule of section 4, row 8
```
`known_frame(seq) -> {"sha256", "t"} | None` is the runner's record of the frames it wrote (the parameter left `None`
= check skipped, as in the dry run's sim feed; a callable returning `None` or another hash fires). It is called from
the listener thread: the runner's frame record must be thread-safe. As built (A, 12:49 IST): a `seq` step of 0 or
less fires too; `conf` is the handedness score (the only per-hand score MediaPipe's Tasks API returns); detection is
per frame, no tracking; extra members `feed()`, `tick()`, `n_valid`, `last_stale`, `addr`, and test-only keyword
arguments `clock`, `wall`, `tick_s`. A malformed datagram counts as no message. The link never touches the bus: it sets a flag;
`SafeBus` drops targets from then on and the control loop writes goals := present at its next step.

**Judge.** `judge.ask(frame_path, phase, context) -> verdict`; verdict = the rubric's JSON
`{hand_present, hand_open_waiting, blade_toward_hand, p_unsafe, recommend, reason}` plus `latency_s, model,
rubric_sha256, frame_sha256, raw` (≤ 2 kB) and ⊕ `unsafe: bool` (the code's reading: timeout, malformed or
`p_unsafe ≥ 0.5`). Timeout 8 s. Parse failure → `p_unsafe 1.0, recommend "abort", reason "malformed reply"`; timeout →
the same with reason "timeout". Provider from `.env` (`JUDGE_PROVIDER`, `JUDGE_MODEL`, the key) over `httpx`, which
is already in the runner's env: no install. `JUDGE_MODEL` unset → list models once, choose, log the choice. `.env` is
parsed by `judge/env.py`; values never reach a log, a row, an exception text or stdout; rows pass `g3.scrub`.
The judge imports nothing from the runner. With no `.env`, the dry run uses provider `stub` and every row says so.
The judged frame goes to a third-party API (the provider in `.env`).

**Planner.** `plan_handover(q_now_lerobot6, palm_mm, cfg) -> Plan | None-with-reason`.
`cfg`: registration (q_zero, sign map all +1, table z −17), calibration limits − 3 deg, workspace box, `standoff_mm`
60, `z_handover_mm`, `z_transit_mm` 65 (the null's retreat height), `speed_dps` 3, the knife (`knife.py`).
`Plan.targets`: lerobot degrees, 15 Hz rows: rise to transit height → above the standoff point → descend → hold.
Geometry: top-down grasp, so the knife is horizontal when the approach axis is vertical (wrist pitch), and its heading
is set by pan + wrist roll (roll solved in closed form from the FK heading at roll 0, iterated because the fingertip
is 12 mm off the roll axis). Handle axis points along (palm − TCP). The standoff is measured from the **handle tip**:
TCP = palm − (standoff + handle_len)·d, d = the unit vector from the base toward the palm (the arm stays on its own
side of the hand); candidates ±15…±60 deg around it if d is unreachable. `None` when: palm or TCP or handle tip or
blade tip outside the box, IK position error > 1 mm, knife tilt > 5 deg, a joint outside its limit, any moving body
< 15 mm above the table, or blade heading toward the palm (reason says re-orient). `blade_toward_palm(q, palm)` is
the code's rule; the judge's `blade_toward_hand` is logged beside it and a disagreement is flagged, not acted on.
On `--arm`, `z_handover_mm` must have been set at dawn; the default (30 mm, the null's lift height) is inside a hand
lying on the table and is used in dry runs only.

**Runner.** `handover_session.py --dry-run [--yes] [--fast N]` (the default is a dry run) or `--arm --port P`.
`--arm` outside 06:00–22:00 IST: refused, and the refusal is a row. Builds its own session from `safety.py`
(`Console`, `EventLog`, `KillSwitch`, `Interlock`, `Clamps`, `SafeBus`, `SafetyContext`, `SafeMover`,
`set_low_acceleration`, `load_calibration_file`), `g3` (`guard_from_record`, `verify_chain`, `kill_row`, `scrub`),
`fk`; mirrors `null_session.py`'s `move_to` / `replay` / `hold_now` / `fire` / `log_kill`. Step row:
`{kind: "step", k, t_s, dt_ms, phase, q6, target6, tcp_mm, load, monitors: {1..8: {fired, detail}}, hb_age_ms,
hand_seq, frame_seq}`; other kinds: `session_start, check, freeze, judge, key, plan, retarget, kill, trial_end,
session_end, refused`. Key rows carry the menu shown, the key, and the verdict on screen when it was pressed.

## 6. Work split

Four subagents in parallel (A monitor, B judge, C planner, D runner + replay), each inside its own folder; `common/`,
`fixtures/capture_frame.py`, integration and `STATUS.md` are mine. An interface change goes into section 5 first.

## 7. Milestones

- M1 (02:00 IST, 2026-10-02): venv; fixtures A, B, C; heartbeat-kill test; stop and ask (IK decision is the operator's).
- M2: dry run end to end (FREEZE → ASK → `[h]` → plan → HOLD → `[r]`; stale heartbeat; second hand);
  `fault_table_handover.json`; chain verifies; secrets scan; commit.
- M3 (after 06:00, operator present, after "arm on"): dawn protocol, results written as they happen.
- M4: the take; `verify_chain`; evidence manifest; `replay.html`; final STATUS.

## 8. Open questions for the operator

Each has the default the build uses until answered; every default is the one that stops the arm sooner.

1. **Zone fixture (F2).** Cross 6 for the 15 mm check and the zone centre for `in_zone`. OK, or another cross?
2. **Speeds (F3).** ≤ 10 deg/s scripted, ≤ 3 deg/s handover, monitor 2 at 30 / 15 deg/s. Confirm or give numbers.
3. **Monitor 7 on one raw frame**, not only 3-of-5. If false freezes are a nuisance at dawn it is one constant.
4. **`[k]` keeps OFF + YES; `[l]` leave-as-is added; `[c]` continue after `[w]` only when all monitors are clear.**
   Without `[c]`, a wait can only end in a handover or an abort.
5. **Standoff from the handle tip** (TCP = 60 mm + handle length from the palm). Needs the prop's dimensions: total
   length, handle length past the fingertips, handle width and thickness at the grip point.
6. **IK (F6).** `SO101Kinematics.solve` + tool offset as instructed, `null_plan.ik` as the fallback. Your call at M1.
7. **Commit (F9).** `git init` inside `experiments/knife_handover/`, or a path to the ARM-ANI checkout? Are the
   fixture frames (your hand) committed?
8. **`.env`.** It does not exist yet. Until it does, B's live-provider fixtures are "unknown" and the dry run uses
   the stub. Create it yourself; I never open it.
9. **Grasp roll.** The null's jaws close along the radial line (roll 0). A knife lying "blade toward the arm, handle
   outward" is radial, so its grasp needs wrist roll ≈ ±90 deg (or the knife laid across). Tonight's placeholder keeps
   the null's geometry; the dawn recording settles it.
10. **Clock.** This session started 12:18 IST on 2026-10-01. I read "tonight" as this evening, M1 as 02:00 on
    2026-10-02, dawn as 06:00 on 2026-10-02. No powered motion before you say "arm on", whatever the hour.
11. **Blind strip.** The box's near edge is outside the camera frame and a hand is seen only once most of it is in
    frame. Not fixable in code tonight; it goes on the replay page as a stated limit unless you want the box cut to
    the visible area for this demo.

### Rulings (reviewer, 2026-10-01 ~20:00 IST) — these replace the defaults above

1. Cross 6 and the zone centre: yes.
2. Speeds as stated, with one change: **on `--arm` every move runs at ≤ 3 deg/s, the scripted segments included,
   until the encoders have been seen at speed; then 10.** Built: `--scripted-dps` defaults to 3 on `--arm` and 10 in a
   dry run, at most 10; monitor 2 for scripted segments is 15 deg/s at ≤ 3, 30 at 10. Going to 10 is the operator's
   explicit `--scripted-dps 10`, recorded in the session-start row.
3. Monitor 7 fires on one raw frame; 3-of-5 is for clearing only. (As built.)
4. `[k]` OFF + YES, `[l]`, `[c]` only when all clear: yes, with **finding 14: `[c]` after an UNSAFE verdict requires a
   fresh judge call that is not UNSAFE.** Built: the fresh call is made at the key with the arm still latched; an
   UNSAFE fresh verdict refuses `[c]` and nothing moves; both verdicts are in the chain.
5. The prop's numbers come from the operator; **until then the planner refuses `--arm`.** Built: `cfg.arm` with a
   placeholder knife is refused before any IK (same pattern as `z_handover` unset); the runner refuses `--arm` without
   `--knife-handle-mm`, `--knife-blade-mm`, `--knife-width-mm`.
6. `[sim]` solve + tool offset, every solution checked through `anchor/fk.py`; `null_plan.ik` stays wired as the
   fallback. (As built.)
7. `git init` inside `experiments/knife_handover/`; the fixture frames are committed; `.env` and `.venv_monitor`
   ignored; a secrets scan before each commit.
8. The operator creates `.env`; then the 15 live-judge cases run on the stills, with latency against the 8 s budget
   and whether the live model fences its JSON. If it fences: a fenced JSON object is accepted only when its content
   parses to the exact schema, and the stripped fence is logged.
9. **Dawn grasp: the prop laid across the radial line first (roll 0, the null's grasp); ±90 deg roll only if across
   is unreachable; a torque-off dry pass before either.**
10. Dawn = 2026-10-02 06:00 IST; nothing powered before "arm on".
11. **For this demonstration the allowed motion region is the G3 box cut to the camera's visible footprint** (the image
    less 10 px, mapped to the table plane). Built: the planner refuses palms and planned points outside it
    (`PLAN_palm_in_the_blind_strip_refused`); the runner checks every scripted row against it before anything opens
    (start check `scripted_segments`). The blind strip is stated on the replay page and in STATUS as a limit. Measured:
    on the 20 mm grid (150 palms, mid-carry start) the cut takes direct handovers 50 → 50, re-orient 12 → 10.

Also from the reviewer: at dawn the Mac runs nothing but this (the 331 ms heartbeat run was under load).

### Rulings at M2 acceptance (reviewer, 2026-10-01 ~22:05)

1. **Judge model:** `JUDGE_MODEL` cleared by the operator; the rule in `judge/providers.py` chose
   `claude-haiku-4-5-20251001` (13 models listed). Budget stays 8 s. That model fenced 15 of 15 replies, so ruling 8's
   fence handling is now built: a reply that is nothing but one ```json (or ```) fence around the exact object is
   unwrapped, still checked against the exact schema, and the verdict carries `fence_stripped: true`; anything outside
   the fence is malformed (`JUDGE_prose_wrapped_json_is_malformed`, `JUDGE_fenced_exact_object_accepted_and_logged`).
2. **Judge accuracy is reported, not gating.** `fixtures/status.py` gates on the judge's schema / timeout / key
   fixtures and reports the live-accuracy cases (`test_saved_frame_live`) with each call's line; known misses are kept
   with their frame hashes (`KNOWN_MISSES`). No rubric edits.
3. **The 55 mm per-step knife check is kept** (SR08 redesign). The 20-run monitor-5 result is stated as measured: 0
   fires while a re-target job was out, over 19 runs that reached a re-target out of 20 consecutive runs.

Next: the dawn protocol, after "arm on".

### Operator decisions on day 2 (2026-10-02 / 10-03)

1. **No motion window** (the operator, 2026-10-03): "don't keep any time bounded restrictions like 22:00 or anything".
   `start_checks.MOTION_HOURS = None`; the 06:00–22:00 IST rule (architect ruling 2026-09-22 addendum E, reviewer
   ruling on 11) is off. The hours checks stay in code — at `--arm`, at every trial, before every motion call, at
   every motion key — and come back by setting `MOTION_HOURS = (6, 22)`; their tests set it and still pass. The
   start-check row says `no motion window (the operator, 2026-10-03)`. Everything else that gates `--arm` is unchanged.
2. **Close target 10.5 %, slip step 1 %** (after the 14:52 hold test: 7.85 % reached load 300, monitor 4). Hold test
   20:38: HOLD_OK at 10.5 %, load 172. The hold-test row now logs the lift as commanded and as measured (the encoders
   showed ~6.8 mm of the 20 mm sent: shoulder 3.1°, elbow 2.0° short under the load).
3. **The ARM statement** is built from the session's own segments, speeds and mode (the 14:52 text said ≤ 10 deg/s
   and "placeholder" while the file ran at 3 deg/s); no [h] / [o] at a hold-test freeze.
4. **Standoff floor 30 mm** (the operator, 2026-10-03, after step 6, KH-S20261003T130937): "reduce this standoff to
   30 mm or something like that because it's a very small space". At 80 mm 1 of that session's 29 handover requests
   planned (replayed offline: 7 at 60, 11 at 40, 21 at 30) - the 167 mm screwdriver does not fit the workspace box at
   a larger standoff. `planner.cfg.STANDOFF_MIN_MM = 30` (was the specification's 60); the default stays 60, the
   sessions pass `--standoff-mm 30`. The per-step knife check follows (30 - 5 = 25 mm). With the arm 12-20 mm off its
   commanded pose under the tool (STATUS, day-2 finding), the real handle-tip gap at the hold can be ~10 mm. The
   pointed end stays >= 199 mm from the palm (30 + 2 + 59.5 handle + 107.5 shaft); blade-first is still refused.
5. **`--handover-trial`** (the operator, 2026-10-03): the [h] is given at trial start (a `preauth` row per trial; the
   ARM statement says so). The first hand event of a trial - a monitor-7 freeze with the prop held - still freezes
   and still calls the judge; if the verdict is not UNSAFE the code takes [h] (its key row carries `by: pre-authorised
   at trial start`, the replay prints NOT TYPED) and the handover runs exactly as after a typed [h] - every code
   refusal, the planner, the per-step check, re-targets. An UNSAFE verdict (`preauth_held` row) or any refusal holds
   and asks as before. Once per trial; [r] is always typed. Speed unchanged: <= 3 deg/s.
6. **The release wait** (the operator, 2026-10-03): after RELEASE, 5 fresh hand-monitor frames with no hand in the
   workspace, counted from the end of the release (`monitors.ClearWatch`; a `wait_clear` row), then RETREAT. At 13:17
   the hand holding the screwdriver dropped out of detection for 9 frames during the release; the old 0-of-5 window
   was already clear when the release ended and the retreat froze 2 frames later. Monitor 7 stays live in RETREAT: a
   hand that comes back still freezes it.
7. **The pointed end >= 150 mm from the palm on every row of every plan made with a measured tool** (the operator,
   2026-10-03; `HandoverCfg.point_min_mm`, `path.check_rows`, `min_point_palm_mm` in every plan's checks). With the
   screwdriver it costs nothing on step 6's requests (21 of 29 plan at 30 mm with or without it). The rehearsal-only
   placeholder (a 20 mm "blade"; `--arm` refuses it) cannot meet it anywhere in the box and is exempt.
   The planner fixtures' measured stand-in (`fixtures_plan.MEASURED`) now has a 40 mm pointed end (was 20: its
   control plan came 148.6 mm from the palm; with 40, 164.8 mm).
8. **The handover rubric** (the operator, 2026-10-03): `judge/rubric_handover.md` (sha256 768da31e0e12...) for CP3
   and the handover freeze (FREEZE with the prop held and a hand seen by the monitor; `judge.rubric_kind`); every
   other call keeps `rubric.md` unchanged (2825992d...). The same six-key JSON, the 0.5 bar; every verdict carries
   `rubric` and that file's `rubric_sha256`. Live on the stills (`test_handover_rubric_live`, reported, not gating),
   2026-10-03 ~15:00: 12 of 12 as expected - hand_zone 1-3 and hand_cross6 1-3 `handover` p_unsafe 0.3; fist_zone 1-3
   and two_hands 1-3 `wait` 0.6, UNSAFE. The stills hold no arm and no screwdriver: the reasons nevertheless describe
   "the pointed end away" - the model restates `blade_toward_palm_by_code` from the context as if seen.
9. **Palm placement** (the operator, 2026-10-03, `--palm-placement`; `planner/placement.py`, `placement_flow.py`,
   `judge/rubric_placement.md`). Pre-authorised, no key during the trial: a hand event -> 1 s settle -> the judge
   (placement rubric) -> the plan -> PLACE_APPROACH -> PLACE_DESCENT -> HOLD, 1 s settle -> CP3 -> RELEASE when not
   UNSAFE -> 5 fresh clear frames -> RETREAT -> "did you receive it? y/n" (`outcome_label`, beside the CP3 verdict).
   The menu only on no plan, a code refusal, a monitor, or UNSAFE after a settle.
   - Target: the handle's middle over the palm centre, the pointed end along wrist -> fingers (the hand monitor's
     landmarks 0 -> 9), >= 10 mm past the fingertips (landmark 12; 100 mm if unknown); +-15..45 deg tried.
   - Release: the lowest point of the gripper and the screwdriver 20 mm above the palm (palm 20 mm above the table,
     the monitor's assumption), commanded +16 mm for the measured sag: 56 mm above the table.
   - **The pointed end >= 30 mm from the palm in 3D on every row; the screwdriver is a plastic prop.** (In this mode
     the standoff rules - 30 mm handle tip, 150 mm pointed end on the table plane - do not apply: the handle goes over
     the palm.) Over the hand (110 mm of the palm) nothing lower than the release height, except the rise in place.
   - The turn at the turn height (>= the 65 mm transit, >= 25 mm above the release) - on the way, before or after the
     move, or at a translate waypoint inside the box.
   - Speeds: <= 3 deg/s for a move with any part within 150 mm of the palm (3D) and the descent; <= 10 deg/s otherwise.
     Monitor 2 per row: 30 deg/s while a row in the last 8 was faster than 3 deg/s, else 15.
   - Monitor 7 in PLACE_APPROACH / PLACE_DESCENT: no re-target; the planned hand 15 mm from where the plan was made, a
     second hand, or the hand lost freezes; every step the row about to be sent keeps the pointed end >= 30 mm (3D)
     from the palm as it is now.
   - Feasible region (scanned 2026-10-03 from the measured CP2 pose, `sessions/KH-DAWN-20261003/placement_map.png`):
     palms x ~200-280 mm, y -60..+120 mm with the fingers away from the robot or toward its right; with the fingers
     toward the zone (a 135-180 deg turn) around (220, -60..+30) and (240, 0). Squares 4 and 8 (y -119): no placement
     (the box or the top-down reach) - refused with the reason. A top-down hold at 56 mm reaches <= ~290 mm from the
     pan axis.
10. **Palm placement, after the first placement trial** (20:25, KH-S20261003T202508; the operator: "it can go
   outside, it just needs to reorient"; "it should hover on top and place it by itself"). At 20:28 the placement
   started by itself, then the 15 mm drift limit froze the approach (the palm 17 mm off after 15 s), and the menu's
   [h] / [o] ran the standoff handover ("standoff 60", "the tip would go outside"). Now:
   - the screwdriver's ends may leave the workspace box; the fingertip stays in it (G3 monitor 3) and in the
     camera's footprint; the ends stay above the table; the pointed end >= 30 mm from the palm (3D). The search tries
     routes with the ends inside the box first (fast), then outside, within an 8 s planning budget (a refusal after);
   - the approach tolerates 50 mm of drift (as the standoff approach); above the palm, before the descent, the hand
     is read again and the placement planned again from there if it moved >= 5 mm or turned >= 20 deg (at most 3);
     the descent still freezes at 15 mm;
   - in this mode the menu's [h] retries the placement (a settle and a fresh judge call first), and [o] is not
     offered.
   From the measured carry poses: the 20:28 palm (237, 92) - fingers away from / toward the robot / toward the spawn
   box; the 20:29 palm (192, -76) with the fingers toward the zone - a 180 deg turn; square 8 with the fingers toward
   the robot - a 90 deg turn; the zone centre - three directions. Square 4 and the zone's far edge (290, 120): out of
   the arm's top-down reach.

## 9. Fixture frames needed (arm torque off, camera only)

`~/lerobot-mps-venv/bin/python experiments/knife_handover/fixtures/capture_frame.py --name <name>` opens the camera
and nothing else (no serial port). Three stills per scene.

| name | scene | expected |
|---|---|---|
| `empty_table` | sheets only, arm out of frame | no hands |
| `hand_cross6` | one hand flat, palm up, palm centred on spawn cross 6 | `palm_mm` within 15 mm of (241.1, −58.8); `in_spawn` true, `in_zone` false |
| `hand_zone` | one hand flat, palm up, palm centred on the zone square | `in_zone` true, `in_spawn` false; within 15 mm of (246.8, 68.1) |
| `two_hands` | one hand in the spawn box, one in the zone | two entries |
| `arm_no_hand` | arm posed by hand over the spawn box, no hand in view | no hands (the gripper must not read as a hand) |
| `knife_no_hand` | prop knife at its spawn pose, no hand | no hands |
| `fist_zone` | closed fist, knuckles up, on the zone | judge: `hand_present` true, `hand_open_waiting` false |
