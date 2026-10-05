Published 2026-10-05 as a single squashed commit; commit hashes cited below refer to the private pre-publication history, kept as a git bundle.

# STATUS — prop-knife handover demonstration

Times are IST on 2026-10-01, read from the clock or from file modification times. A demonstration: nothing here is
session evidence for any certificate, and nothing is presented as a safety certificate. No motor has been powered. No file under
`anchor/`, `phase0/` or `so101_sim/` has been written (`find ../../anchor ../../phase0 ../../so101_sim -newer PLAN.md
-type f` prints nothing, 21:52).

**2026-10-05 · Erratum: the tool and the hand.** The labels written into the logs during the runs say a prop was
used; it was not. Every session's `session_start` row (`label`: "a rehearsed screwdriver (prop tool) handover";
`grasp_segment.label`: "the screwdriver (plastic prop) reversed where it lies"; `placement.prop`: "plastic prop") and
ARM statement in `safety_events.jsonl` ("the marked prop tool only"; "the screwdriver a plastic prop") name a prop
tool: **a real screwdriver was used — red rubber handle, metal shaft — the same one in every session.** The layer-off
session KH-S20261004T174200 says "a prop hand in the place zone" in its `label` and "A PROP HAND - NOT A REAL HAND -
lies in the place zone for this shot" in its ARM statement: **the operator's own hand was in the place zone, in the
layer-off take as in the layer-on take (KH-S20261005T015041).** The frame at the layer-off take's one freeze,
`sessions/KH-S20261004T174200/photos/freeze01_f001103.jpg` (sha256 `f050b177…`, named in that freeze row), shows the
operator's hand in the place zone under the screwdriver. The chains are left unedited: every row is hash-chained, and
changing a label would break the chain from that row on; the pages built from them repeat the labels. This entry is
the correction (README: Errata).

**2026-10-04 · video prep (built 14:15–14:40, dry run only; nothing new has moved the arm).**
- **Presentation page.** `replay/build_replay.py <session>` now writes `present.html` beside `replay.html`
  (`replay.html?present=1` opens it): a fixed 720×1080 right-hand panel on a dark page — the fingertip path in 3D
  with the box, the zone, the hand where the monitor reported it, a red dot per freeze, a green dot where a plan took
  over and the fingertip as the time cursor (top, 420 px); the timeline with the phase bar, monitor lanes 1–8, judge
  diamonds with their latency bars, typed (white) and automatic (amber) key ticks and a now-line (middle, 420 px);
  the call log, one line per check, scrolling as it plays (bottom, 240 px); a badge over the top panel driven by the
  chain (MOVING / FREEZE — monitor N, held in N ms / JUDGE: verdict / RE-ORIENT / RELEASE / WAIT — hand in workspace,
  and HOLD / SETTLE chips / KILL / END). Play from the first motion at real time; space pauses; 1× / 2× / 4×;
  ← → 5 s; N next event; H hides the control bar (outside the panel). Built for `KH-S20261004T134743` and checked
  in headless Chrome at 1920×1080 (screens at 01:56 freeze, 02:20 re-orient, 04:14 CP3, 10:00 cut).
  **What the page estimates:** only step rows carry a clock; every other row has `t_iso` to the second, so it is placed
  on the steps' clock by one fitted offset and clamped between its neighbouring steps (order is the chain's; ±0.5 s).
  A stretch with no row longer than 30 s (the runner at a prompt: nothing is commanded without a step row) plays in
  4 s and is marked ✂ on the timeline and over the 3D view (`KH-S20261004T134743`: 1 min 7 s at the trial prompt,
  19 min 17 s at the last menu before [k]); `?real=1` or C plays them at real time. A freeze holds the badge 1.5 s
  before a judge call begun inside that is shown. New step rows carry `hands_mm` (the palm per hand) so the hand
  marker follows the hand; older sessions show it at the palm last reported by a freeze, plan or still check.
- **`--layer-off` (the contrast shot).** The hand monitor (monitors 7 and 8 read as OFF on every step row, never
  fired), the judge and the handover planner are not built (`ports.Off`: any use raises and stops the session, arm
  held); monitors 1–6 live; pick → place only (GRASP, LIFT, APPROACH_ZONE, PRE_PLACE, PLACE, RETREAT — no CP1 / CP2,
  no handover, [h] / [o] never offered); every row of the chain, the first included, carries
  `trust_layer: "TRUST LAYER OFF — DEMONSTRATION"`; the replay shows it as a red banner and the presentation badge as
  its top line. A monitor 1–6 freeze asks as always (no verdict on screen; [c] is the operator's alone, offered only
  when monitors 1–6 read clear after the key). Refused with `--palm-placement`, `--handover-trial`, `--hold-test`.
  The ARM statement says what is off, what lies in the place zone (its words, and what was there: the 2026-10-05
  erratum above), and where the PLACE goes down to by FK of its rows: with `dawn/segments.json` the fingertip goes to ~5 mm above the table at the
  zone centre, so **anything standing higher in the zone is pressed by the screwdriver** — only monitor 4 (gripper
  load) and the operator's ESC watch that. Fixtures: `tests/test_video_prep_1004.py`, 16 passed (15:5x; the
  hold-test guard and a parse check of the page's script added).
- **The full gate has NOT been run on this tree.** Started 14:47, stopped 15:19 by the operator's choice to trial
  first (the preflight with it running: load 4.03–4.84 against 4, judge 2.1 s, gate p95 66.67 ms - not met). Every
  `--arm` run on this code is under `--operator-override`, stamped.
- **Trials by the operator, 15:20 and 15:27** (palm placement, reversed lay, override): KH-S20261004T152015 ABORTED,
  KH-S20261004T152718 HANDED_OVER (re-orient 75°, then 30° after a re-plan; released without a key).
- **Grip, 15:4x (the operator):** the squeeze on the rubber handle 4 % → 6 % past contact - close target 8.6 %
  (contact 14.6 %), was 10.5 % - reason "compliant handle, tool pitched nose-down under re-target"; logged in
  `dawn/knife.json` (`changes`, target `verified: false`); `dawn/segments.json` and `dawn/segments_reversed.json`
  rebuilt at 8.6 % (every row checked). **Expected load is at the overload bar:** today's grasp checks at 10.5 % read
  136–168, and at 46.6 per % that is ~225–257 at 8.6 % (monitor 4: 250 for 0.5 s) and ~271–303 at 7.6 %. The hold
  test now refuses its one tighter step when the first hold's load max + 46.6 reaches 250, or the target is under the
  gripper floor, and says so (`hold_test._no_tighter`); its question asks about a slip OR a nose-down pitch. **Not
  yet verified on the arm.**
- **Hold test at 8.6 % (KH-S20261004T162508, the operator, ~16:25): the gripper servo tripped its own overload
  protection, twice, below monitor 4's bar.** Load held at 240–244 (bar 250); after ~1.9 s the servo answered the
  Present_Load read with "Overload error" (LIFT 85.7 s, HOLD 110.2 s); each became a hardware freeze, the arm held;
  [c] once, then [k] (torque off). The close ramp: 10.6 % 148, 10.0 % 176, 9.4 % 200, 9.0 % 220, 8.8 % 228, 8.6 % 244;
  the jaws stop at ~13.4–12.4 % under power. **Finding:** monitor 4 (250 for 0.5 s) sits above what the gripper
  servo holds for 2 s; the servo's trip is caught only as a bus fault. **Set to 9.4 %** (~200, ~18 % under the trip;
  `dawn/knife.json` change 2, segments rebuilt); the hold test's tighter step now stops at a predicted 220
  (`hold_test.HOLD_LOAD_MAX`). **Verified, KH-S20261004T163650 (~16:37): HOLD_OK at 9.4 %** - load 208 steady for
  the 10 s hold (151 rows, mean = max), no monitor fired, no servo trip, no slip or pitch (the operator). The lift
  measured 3.8 mm of the 20 mm commanded (encoder FK): the arm's sag under the screwdriver, larger than the 6.8 mm of
  2026-10-02 - the sag finding above stands.
- **Palm placement at 9.4 %, KH-S20261004T170052 (~17:00): HANDED_OVER, after 4 freezes, 17 judge calls and the
  operator moving the hand.** Three causes, each a check doing its job: (1) the hand in at ~(222, 95) mm, the zone
  side, needs an 82° turn whose release point (244, 95–100) mm, 92 mm up, is at the IK's reach edge: two plans failed
  after the full 25 s budget each (IK error 3.7–4.5 mm); a palm 6 mm nearer planned in 0.2 s. (2) Three approaches
  froze on monitor 7, "the approved hand is not seen", with the arm 52–64 mm from the palm: the screwdriver hid the
  hand from the overhead camera before it was within the 45 mm cover distance (the 13:16 case, kept as freeze-and-ask).
  (3) Four [h] were refused, "no hand seen steadily" (0–2 of 3 frames in 0.6 s), the arm still over it. It placed
  when the operator moved the palm to (200, −1) mm, fingers toward the zone: a 4.6° turn, 97 rows. From the LIFT
  pose the planner turns least (20°) for a palm near (190–210, −20..0) mm with the fingers pointing away from the
  robot and toward the zone; with x >= 230 and y > 0 it finds none in 4 s.
- **The freeze menu, 2026-10-04 evening (the operator's two asks).** (1) "The closeness is a depth issue - the
  action should keep going": **not made automatic** - a check that stops the arm stays gating (the judge-advisory /
  hide-zone change was denied by the permission system this morning; this is the same outcome). What was wrong is
  fixed instead: the judge was given `arm_to_palm_mm`, a top-down distance (2 mm = over the palm), and read it as
  touching; it now gets `horizontal_arm_to_palm_mm`, `height_above_palm_mm` (the lowest point of the gripper and the
  screwdriver above the palm, from the encoders) and `pointed_end_to_palm_3d_mm`, and the placement rubric says to
  judge nearness by height and 3D distance. And [h] with the arm over a hidden palm already goes on in one key (no
  fresh judge; CP3 judges before the release) - the menu now says so instead of "a settle and a fresh judge call".
  (2) **[r] at the freeze menu** (palm placement, holding, a placement planned): the gripper opens where the arm
  stands, the operator's key with the verdict on screen beside it (as at ASK_RELEASE); then a lift-off only if the arm
  is on the descent (from the row it is on, never down first), the wait for 5 clear frames, the retreat; a
  `release_typed` row; state ASK -> RELEASE.
- **Bug found by the fixtures and fixed:** since this morning [h] went on to the last-seen palm whenever the arm was
  within 45 mm of it - also after a freeze because the hand MOVED 60 mm (monitor 7 froze it again at once). Now only
  when the monitor sees no hand (hidden under the arm); a hand in view is settled on, judged and planned for again.
- **Finding:** with the palm under the arm for the whole approach (a palm near where the hand event happened), the
  still check above the palm has only the reading before the plan (readings under the arm are not used), so a move
  there is caught by the descent's 15 mm rule (dry run: frozen at 20 mm, nothing released), not by a re-plan.
- **Stale fixtures updated, not the code:** `test_RUN_unsafe_after_the_settle_asks` and `test_RUN_no_plan_asks` still
  expected one try; the automatic retries (4) are the operator's decision of this morning.
- **The grip goes slack at the first freeze - found 2026-10-05 ~00:3x in KH-S20261004T170052.** Load 184 at the
  grasp, 180 through LIFT, then at the first freeze 36 and 0 for the rest of the trial; the jaws 12.17 -> 12.64 %.
  The freeze sets every goal to the present position, the gripper's too: stalled on the handle at ~12.2 %, its goal
  became 12.2 % and the squeeze was gone; the placement plan then carried the gripper at that reading (`cur[5]`) to
  the release. That is the operator's "the grasp loosens after the first pause and the hand, the pointed end dips".
  **Fixed (the operator's choice of the two offered):** a placement plan made while holding closes the gripper back
  to the segments' close target at 3 %/s in its first rows ("rise in place", ~1 s) and holds it to the release
  (`placement.squeeze`, inside the plan's hash); it never opens. The freeze rule is unchanged: while frozen the grip
  stays slack. **Close target 9.0 %** (the operator, from 9.4 %; expected load ~220-228, the servo's own trip at
  240-244). Not yet run on the arm.
- **For the record (2026-10-04 ~14:1x, option 1):** the 13:16-13:18 freezes' photos (KH-S20261004T131330), re-asked
  with the code's hand reading in the judge's context: 6 of the 7 UNSAFE verdicts came back not UNSAFE (p_unsafe
  0.35-0.4); the 7th frame had no hand in it.
- **Publication prep (2026-10-05, the operator: github.com/aniketmallick/trust-layer-demo, Apache-2.0).** Kept in
  `sessions/`: KH-S20261004T152718 (a handover), KH-S20261004T174200 (the layer-off take rendered for the video),
  KH-S20261005T015041 (the final layer-on take); every other session folder (91 arm sessions and the three
  KH-DAWN-* folders, 154 MB) moved to `_to_delete/sessions/` (ignored, not deleted). `sessions/_dryrun/` and
  `*.mp4` ignored and the dry runs untracked (they rebuild locally). The three tests that read chains of moved
  sessions now read `tests/data/rig_poses.json` (the same values, each with its row's seq and sha256); the 2026-10-04
  lay measurement is `dawn/lay_measured_20261004.json`. `verify_chain.py` verifies a chain with the standard
  library only (the public tree has no `anchor/`). README, LICENSE (Apache-2.0, the canonical text). The word for a
  certificate's claim appears nowhere in the tree (three doc lines reworded; tests spell it in two parts).
- **present.html in landscape, and MP4s (2026-10-05, the operator).** The page is now 1920x1080 by default: the 3D
  path with the badge across the top (1920x540), the timeline bottom-left (1180x540), the call log bottom-right
  (740x540); `?layout=portrait` keeps the 720x1080 panel. `node replay/render_mp4.mjs sessions/<id>` writes
  `present_1x.mp4` beside it: headless Chrome draws each frame through the page's own seek/frame at 25 fps of session
  time and screenshots it, `replay/encode_mp4.py` (PyAV, libx264, crf 18) encodes it - **1x from the first motion to
  the end, real time, no cuts** (stretches with no row play at their real length), plus 1 s on the last frame.
  Rendered for KH-S20261004T174200 (layer off, 4.0 min) and KH-S20261005T015041 (palm placement, 6.0 min).
- **present.html checked in a real (headless) Chrome over CDP, the page's own play loop driven at 60 fps:** for
  `KH-S20261004T134743` and `KH-S20261004T152718`, t = 0 at the first motion; at 1× the clock advances 1.0 s per
  second outside the cut stretches (3.3 s of wall clock → +3.1 s under the browser's own frames, the first frame
  late); the badge goes from MOVING (green) to "FREEZE — monitor 7" (#ff4d4f) at the first hand freeze (+01:55.6 /
  +00:59.4) and holds it. The cut stretches (no row > 30 s: 134743 at the trial prompt and the last menu; 152718 at
  the end) play in 4 s; `?real=1` plays them at real time. A hidden tab now pauses the playback instead of jumping.
- **Earlier today, for the record:** the placement sessions of 2026-10-04 ran under `--operator-override` (untested
  code, stamped on every step row). A change that would have made the judge advisory in palm placement (release
  without the judge, [h] skipping it, a 90 mm hide zone) was **denied by the permission system and not made**; the
  operator chose option 1 instead: the judge still gates, with the code's reading of the hand in its context.

**M2 · accepted by the reviewer (~22:05) · status run 22:12–22:30 · commit 2.** The run that counts, with every change
in it, is `fixtures/status.py` (all four suites, one after another): **`all_green: true`**, written to
`sessions/fixture_status.json` for code hash `5990435b58dc…`, the code unchanged during the run. Runner 143/143 (finding
4's test included), judge gating fixtures 19/19, planner 39/39, monitor 14/14. Judge live accuracy, reported and not
gating (ruling 2): 13 of 15. The 55 mm per-step knife check is kept (ruling 3). Monitor 5 during re-targets: 0 fires
while a job was out, over **19 runs that reached a re-target out of 20 consecutive runs** (run 3 froze on monitor 5
during the scripted segment, 5 s in, before any re-target). Next: the dawn protocol, after "arm on".

**M2 · 21:55 · written.** Dry run only, on `FakeFeetechBus`, synthetic frames, the simulated hand feed and the judge's
stub. Findings 1–14 of the safety review each have named tests; 13 of 14 are green in the full run of 21:31–21:48 and
the 14th (finding 4) has one test that failed in that run on a stale test stand-in and passed alone after the fix.
Live judge: 12 of 15 pass. Monitor 5 during re-targets: 0 fires over 20 consecutive runs, in 19 runs that reached a
re-target. Nothing is armed: `--arm` stays refused until all of this is green and the operator says "arm on".

## Passed (fixture name beside each)

### Status run 22:12–22:30 — judge model and live accuracy (rulings 1 and 2)

- Model: `JUDGE_MODEL` cleared by the operator; the rule in `judge/providers.py` (vision-capable; tier haiku before
  sonnet before opus; newest within the tier) chose **`claude-haiku-4-5-20251001`** from 13 listed. Budget 8 s.
- It fenced every reply in ```json. Run at 22:07 under the strict parser: 15 of 15 malformed (UNSAFE), latency
  1.86–2.79 s (`fixtures/judge_live_20261001T2208_haiku.txt`). Ruling 8's fence handling built (PLAN.md, rulings at M2
  acceptance) — `JUDGE_prose_wrapped_json_is_malformed`, `JUDGE_fenced_exact_object_accepted_and_logged`.
- Run at 22:09 with the fence rule: 13 of 15, latency 1.90–2.66 s, 15 of 15 fenced and stripped; misses `hand_zone_1`,
  `hand_zone_3` (`hand_open_waiting` false: "not in a receiving posture"; it also calls the zone "the spawn box")
  (`fixtures/judge_live_20261001T2214_haiku_fence_rule.txt`).
- In the status run (22:12–22:30): 13 of 15, answered calls 2.19–4.03 s; `hand_cross6_1` **timed out at 8.004 s**
  (UNSAFE, abort); `hand_zone_1` missed again (`hand_open_waiting` false). `hand_cross6_3` passed in both haiku runs.
- Known misses kept with their frame hashes (`fixtures/status.py` `KNOWN_MISSES`): `hand_cross6_3.png` `a319416f…`
  (claude-opus-5-5, 20:45), `hand_zone_1.png` `00471940…`, `hand_zone_3.png` `e2781e16…` (haiku, 22:09) — reported,
  no rubric edit. `test_STATUS_live_judge_accuracy_is_reported_not_gating`, `test_STATUS_a_gating_judge_fixture_failing_is_not_green`,
  `test_STATUS_known_misses_carry_the_frame_hash`.
- Seen in the answers, reported: `hand_zone_2` at 22:09 answered `p_unsafe` 0.3 with a hand in view (not UNSAFE by the
  code's reading). The gate there is monitor 7, which needs 0 of 5 frames with a hand before anything moves.

### M2 — the safety review's findings 1–14

Full run 21:31–21:48: `tests`, `judge`, `planner` in the lerobot venv, 197 tests, 196 passed, 1 failed (the 15
live-judge cases were run separately, below). JUnit: `sessions/_dryrun/m2_suite_20261001T2131_junit.xml`; the
finding-4 re-run: `sessions/_dryrun/m2_sr04_rerun_20261001T2149_junit.xml`. Tests are in `tests/test_safety_review.py`
(SR*), `tests/test_monitors.py` (M7*), `tests/test_retarget_unit.py` (RT*), `tests/test_retarget_worker.py` (WORKER*),
`tests/test_dry_run_e2e.py` (E2E*), `tests/test_common.py`.

| # | finding | tests and their result | |
|---|---|---|---|
| 1 | HIGH: a stop pressed between the [h]/[o] key and motion was erased; the arm moved | `test_SR01_esc_during_planning_freezes_before_any_write` pass, `test_SR01_stale_link_during_planning_is_latched` pass, `test_SR01_esc_during_a_refused_plan_is_recorded_and_the_arm_is_latched` pass, `test_SR01_a_refused_h_leaves_the_arm_latched_with_a_row` pass | **pass** |
| 2 | HIGH: torque off at session end after one OFF | `test_SR02_off_alone_at_session_end_keeps_torque_on` pass, `test_SR02_off_then_yes_at_session_end_disables_torque_control` pass | **pass** |
| 3 | HIGH: the menu took the first letter of anything typed | `test_SR03_menu_takes_only_the_exact_key` pass, `test_SR03_release_menu_takes_only_the_exact_key` pass | **pass** |
| 4 | MEDIUM: an exception in the control path left the arm running, logged complete (all five green in the status run 22:12–22:30) | `test_SR04_on_step_that_raises_freezes_held` FAIL in the full run, pass re-run alone 21:49 (see Failed or unknown), `test_SR04_planner_returning_none_at_h_is_a_refusal` pass, `test_SR04_planner_returning_none_at_a_retarget_freezes` pass, `test_SR04_bus_read_failure_inside_hold_rows_freezes` pass, `test_SR04_internal_error_holds_stops_and_never_asks_for_torque_off` pass | **not green in the full run** |
| 5 | MEDIUM: RELEASE exempt from monitor 7 whatever was in view | `test_M7_release_one_hand_does_not_fire_control_two_hands_fire` pass | **pass** |
| 6 | MEDIUM: after [o] the approved palm was re-read (drift up to ~100 mm) | `test_SR06_drift_after_reorient_is_measured_from_the_palm_approved_at_the_key` pass | **pass** |
| 7 | MEDIUM: a NaN palm read as the approved hand in range | `test_M7_non_finite_palm_in_an_approved_phase_fires` pass, `test_hand_message_schema` pass | **pass** |
| 8 | MEDIUM: a re-target plan swapped in without checking the palm (SR08: now planned in a worker process) | `test_SR08_retarget_in_the_worker_process_stale_plans_never_sent` pass, `test_SR08_a_plan_not_ready_at_its_swap_row_is_pending_and_never_used` pass, `test_SR08_a_plan_that_does_not_start_at_the_held_goal_freezes` pass, `test_RT_swap_at_the_swap_row_continues_from_the_pose_sent` pass, `test_RT_palm_moved_while_planning_discarded_then_asked_again` pass, `test_RT_not_ready_at_the_swap_row_pending_current_plan_goes_on_late_never_used` pass, `test_RT_knife_too_close_to_the_palm_as_it_is_now_fires_before_the_send` pass, `test_RT_knife_guard_quiet_on_the_plan_as_made_control` pass, `test_RT_refused_or_dead_worker_freezes` pass, `test_RT_a_plan_not_starting_at_the_goal_sent_is_never_sent` pass, `test_WORKER_sends_never_block_and_the_newest_job_is_answered` pass, `test_WORKER_killed_is_reported_dead` pass, `test_E2E_retarget_is_silent_and_inside_50mm` pass | **pass** |
| 9 | MEDIUM: RETREAT re-entered without the 0-of-5 gate; [c] used a reading from before the prompt | `test_SR09_retreat_after_the_release_wait_is_gated` pass, `test_SR09_c_is_refused_unless_all_clear_at_the_key` pass | **pass** |
| 10 | LOW: a failed hold was logged, not retried | `test_SR10_hold_that_fails_three_times_is_logged_and_said_in_capitals` pass, `test_SR10_hold_that_succeeds_on_the_third_try_control` pass | **pass** |
| 11 | LOW: hours checked once; output folders could point into a frozen folder | `test_SR11_a_motion_key_outside_the_hours_is_refused_with_a_row` pass, `test_SR11_a_trial_outside_the_hours_does_not_start` pass, `test_SR11_the_clock_passing_2200_between_two_segments_freezes_before_any_target` pass, `test_SR11_output_folders_under_the_frozen_tree_are_refused` pass | **pass** |
| 12 | LOW: a hard abort raised without goals := present | `test_SR12_hard_abort_mid_segment_leaves_goals_at_the_present_position` pass | **pass** |
| 13 | LOW: the planner could be entered from two threads | `test_SR13_planner_calls_never_overlap` pass, `test_SR13_a_freeze_during_a_retarget_abandons_the_job` pass | **pass** |
| 14 | [c] after an UNSAFE verdict needs a fresh verdict that is not UNSAFE | `test_E2E_malformed_judge_reply_at_cp2_is_unsafe` pass, `test_SR14_c_after_unsafe_is_refused_when_the_fresh_verdict_is_unsafe` pass | **pass** |

How the findings were closed (code): 1 the stop flag is cleared AT the accepted key, before any planning, and every
path back to ASK re-latches with a row; 2 the session-end prompt is OFF then YES, as `[k]`; 3 a key is accepted only
as exactly one listed letter; 4 `on_step`, `hold_rows` and the session guard freeze with a row on any exception, rc 4,
torque left as it is; 5 RELEASE fires on a second hand; 6 drift stays measured from the palm approved at `[o]`; 7
non-finite numbers are a malformed message; 8 see SR08 below; 9 `gate()` (0 of 5, monitor 8 clear) before every
moving state, `[c]` re-reads the monitors after the key; 10 the hold is retried 3 times, a failed hold is said in
capitals; 11 the 06:00–22:00 rule is read before every motion call (`Motion.pre_move`) and at every motion key,
output folders under a frozen folder are refused; 12 goals := present before a hard abort leaves; 13 one lock around
the planner; 14 `[c]` with an UNSAFE verdict on screen asks the judge again with the arm latched, and is refused if
the fresh verdict is UNSAFE.

### M2 — SR08: re-target plans computed in a worker process (reviewer, 2026-10-01)

- `retarget_worker.py`: the session's planner for re-targets runs in its own process; the control loop only polls
  the pipe. A job asks for a plan from the row the current plan reaches 8 steps later (the swap row); the current plan
  keeps running. Ready at the swap row: checked (palm within 5 mm, first row within 1 deg of the goal sent) and
  swapped in. Not ready: a `retarget pending` row, the current plan goes on, the late plan is never used.
- Added because the arm now keeps moving while a plan is computed: every step of the approach, the knife on the row
  about to be sent must stay ≥ 55 mm (60 standoff − 5) from the palm as it is now, else monitor 7 freezes the arm
  before the send. Cost 0.07 ms a step. A reviewer's call to keep or change (PLAN.md section 4, as built).
- Found while building it: a whole plan sent down the pipe per job filled it while the worker was sending its result
  back; both sides blocked (20:55). Jobs now carry three fields (a few hundred bytes); the worker plans the newest
  queued job only. `test_WORKER_sends_never_block_and_the_newest_job_is_answered`: 20 sends at a busy worker, max
  under 20 ms each.
- The retry-up-to-3 in the SR08 test is gone; the decisions are tested step by step without a clock
  (`tests/test_retarget_unit.py`, 7 tests).
- **Monitor 5 over 20 consecutive dry runs** (`tests/m5_retarget_runs.py`, 21:09–21:31, file
  `sessions/_dryrun/m5_retarget_20runs.json`): four re-targets a run (palm 10, 20, 30, 40 mm from the approved palm);
  76 asked, 76 swapped, 0 pending; 608 control steps taken while a job was out, longest 67.9 ms (limit 89.9);
  **monitor 5 fired while a job was out: 0 runs.** Outside a job window it fired in 4 runs (Failed or unknown).

### M2 — the reviewer's other rulings, built

- Ruling 2, `--scripted-dps` 3 on `--arm`, 10 in a dry run, never above 10; monitor 2 at 15 deg/s when scripted at 3
  — `test_R2_scripted_speed_is_3_on_the_arm_10_in_a_dry_run_never_above_10`, `test_R2_segments_at_3dps_never_step_above_it_and_monitor2_follows`
- Ruling 5, the prop measured before `--arm` — `test_R5_arm_without_the_prop_measured_is_refused_with_a_row`,
  `test_R5_part_of_the_dimensions_is_refused`, `PLAN_knife_unset_on_arm_refused`
- Ruling 11, motion region = G3 box ∩ the camera's footprint (image less 10 px) — `PLAN_palm_in_the_blind_strip_refused`,
  `test_Q11_scripted_rows_outside_the_footprint_are_reported`; start check `scripted_segments` (the placeholder: every
  row inside). On the 20 mm grid (150 palms, mid-carry start) the cut changes direct handovers 50 → 50, via re-orient
  12 → 10; 7 grid palms lie in the blind strip. Stated as a limit on the replay page.
- Planner suite with these: 39 passed (`planner/tests/test_planner.py`, in the full run above).
- Ruling 7: `git init` inside `experiments/knife_handover/`; `.env`, `.venv_monitor/`, `_to_delete/` ignored;
  `secrets_scan.py` (a pattern scan of the staged diff - no scanner such as gitleaks is installed on this Mac).

### M2 — rehearsal sessions, fault table, replay (rebuilt 21:49–21:52)

| session | script | rows | `verify_chain` | loop dt p50 / max ms | frame → hold (monitor 7) | path |
|---|---|---|---|---|---|---|
| `KH-DRY-20261001T214959` | handover, keys h,h,r | 934 | ok | 66.67 / 66.77 | 92.6, 75.3 ms | FREEZE(7) → judge → `[h]` → plan → approach → re-target asked → swapped (30 mm) → FREEZE(7) at 80 mm → `[h]` → HOLD → CP3 → `[r]` → RELEASE → RETREAT → HANDED_OVER |
| `KH-DRY-20261001T215101` | stale heartbeat, key l | 210 | ok | 66.67 / 66.69 | – | FREEZE(8) → judge → `[l]` → ABORTED |
| `KH-DRY-20261001T215115` | second hand, keys h,l | 237 | ok | 66.67 / 66.69 | 52.3, 90.7 ms | FREEZE(7) → `[h]` → approach → FREEZE(7) second hand → `[l]` → ABORTED |
| `KH-DRY-20261001T215131` | no hand | 396 | ok | 66.67 / 66.74 | – | pick, lift, CP1, carry, CP2, place, retreat → PLACED |

- `sessions/_dryrun/fault_table_handover.json` / `.md`: 66 rows, all as expected (fire and not-fire rows for
  monitors 7 and 8, the judge gate, the hours, the chain, the scrubber, the menu keys, the output folders; monitors
  1–6 cite `phase0/g3/fault_injection_20260928T141801.json` read-only). Label: DRY RUN, nothing seen live.
- `sessions/_dryrun/KH-DRY-20261001T214959/replay.html`: 116 kB, self-contained, "demonstration" in title and header,
  no certificate claim in its text, 0 network references, the blind strip stated.
- The 13:24 sessions built by the pre-fix code are moved to `_to_delete/pre_fix_sessions/` (not committed).

### Earlier evidence (M0–M1), unchanged

- Pixel map equals the frozen runner's `CamMap` on 50 random pixels, with and without height — `test_map_equals_the_null_runners_cammap`
- Cross 6 and the eight A2b outline corners map to their registered arm-frame points within 0.5 mm / 0.5 px — `test_registered_points_round_trip`
- `in_spawn` / `in_zone` / `in_workspace` fire on cross 6, the zone centre and the gap, and not 60 mm past the box — `test_flags_fire_and_do_not_fire`
- A palm 150 mm above a point inside the box still sets `in_workspace` — `test_a_raised_hand_just_outside_the_table_reading_still_counts`
- Frame pair: written by rename, a jpg without its json is rejected — `test_frame_pair_is_atomic_and_checked`
- 3-of-5 debounce — `test_debounce_three_of_five`
- Hand message schema accepts a well-formed message and rejects four malformed ones — `test_hand_message_schema`

**B · judge (built 12:40 IST; re-run by the integrator: 18 passed, 15 skipped).** No request has reached a real
provider. Each name is `judge/tests/test_judge.py::test_fixture[<name>]`.

- Non-JSON reply → `p_unsafe` 1.0, abort, "malformed reply" — `JUDGE_malformed_text_is_abort_unsafe`
- A sentence or a code fence around the JSON → malformed — `JUDGE_prose_wrapped_json_is_malformed`
- 9 of 9 schema violations → malformed — `JUDGE_schema_violations_are_malformed`
- Stub answers after 9 s: `ask()` returned at 8.01 s, abort, "timeout" — `JUDGE_timeout_returns_at_budget_abort_unsafe`
- `p_unsafe` 0.5 → unsafe — `JUDGE_p_unsafe_at_bar_is_unsafe`; 0.49 → not unsafe (control) — `JUDGE_p_unsafe_below_bar_is_not_unsafe`
- Fields passed through, rubric and frame hashes match the bytes, `raw` cut at 2048 bytes (control) — `JUDGE_well_formed_reply_passes_through`
- One byte changed in the rubric → another `rubric_sha256` — `JUDGE_rubric_edit_changes_hash`
- No `.env`, no key, unknown provider → UNSAFE "judge not configured" — `JUDGE_not_configured_is_unsafe`
- The stub is named "stub" and is never chosen silently (control) — `JUDGE_explicit_stub_says_stub`
- Missing frame, non-image, unknown phase → UNSAFE, 0 provider calls — `JUDGE_bad_frame_or_phase_is_unsafe`
- Request shape and model choice against a mock transport (controls) — `JUDGE_anthropic_request_and_model_choice`, `JUDGE_anthropic_named_model_low_effort`, `JUDGE_openai_request_and_model_choice`
- HTTP 500, empty content, failed listing → UNSAFE — `JUDGE_provider_failure_is_unsafe`
- A planted fake key is absent from verdicts, exceptions, logs, stdout, stderr (4877 characters inspected) — `JUDGE_fake_key_never_leaves`
- `.env` parser; repr shows names only (control) — `JUDGE_env_parser`
- `judge/` imports stdlib, httpx and itself only — `JUDGE_import_graph`

**A · hand monitor and hand link (built 12:49 IST; re-run by the integrator: monitor venv 8 passed, 6 skipped; runner
venv `tests/test_hand_link.py` 13 passed).** No fixture has run on a rig frame. `mediapipe==0.10.35` in
`.venv_monitor/` only; the lerobot venv has no mediapipe. Model file sha256 `fbc2a30080c3…cde1`, pinned in
`monitor/models/MODEL.lock`.

- Palm centre = mean of landmarks 0, 1, 5, 9, 13, 17 (control) — `MON_palm_centre_is_the_mean_of_six`
- Three synthetic clear-table images → 0 hands (control) — `MON_blank_table_no_hands`
- A public sample picture (not a rig frame) → 2 hands, 21 landmarks each, 39 ms — `MON_public_sample_hands_are_reported`
- Model file changed by one byte → the monitor refuses to start — `MON_model_file_changed_refused`
- One message per new frame, none for a repeated frame, hash = the frame's — `MON_one_message_per_new_frame_none_for_a_repeat`
- An undecodable frame publishes nothing — `MON_undecodable_frame_is_silence`
- Frames at 15 Hz for 4 s → 60 of 60 published, longest gap 81 ms, `proc_ms` p95 28.0 (synthetic images, control) — `MON_rate_with_frames_at_15hz`
- With `landmarks_px` on the wire (12:52 IST): two-hand datagram 1143 bytes (loopback limit 9216), 60 of 60 received at 15 Hz in 6 runs, `proc_ms` p95 28.3–32.5 — same two fixtures
- The saved-still check fails at 16 mm, a wrong flag, a wrong count; passes at 14 mm — `MON_saved_still_check_fires_and_passes`
- Monitor 8 fires with no message yet — `LINK_no_message_yet_fires`
- 300 messages at 100 ± 30 ms → 0 fires (control) — `LINK_nominal_10hz_with_jitter_never_fires`
- Silence → no fire at 300 ms, fire at 310 ms, one `on_stale` call — `LINK_silence_fires_past_300ms`
- `seq` step 2 does not fire; step 3, a repeat or a step back fires — `LINK_seq_step_2_tolerated_3_fires`
- Frame 400 ms old does not fire, 600 ms fires — `LINK_stale_frame_fires`
- A frame the runner did not write (seq or hash) fires — `LINK_frame_the_runner_did_not_write_fires`
- Malformed datagrams count as silence — `LINK_malformed_datagram_is_silence`
- The watchdog calls once per arming, only while armed — `LINK_watchdog_calls_once_per_arming_only_when_armed`
- Window = last five raw results (control) — `LINK_window_holds_the_last_five_raw_results`
- Real UDP loopback, 20 of 20, 0 fires (control) — `LINK_loopback_10hz_real_clock_never_fires`
- **Heartbeat-kill** (process killed with -9, 8 runs each) — `KILL_minimal_publisher_killed_runner_freezes`, `KILL_hand_monitor_process_killed_runner_freezes`:

  | | last message → stop flag | kill → stop flag | stop flag → hold step | kill → hold step |
  |---|---|---|---|---|
  | minimal publisher | 300–311 ms | 204–298 ms | 0–66 ms | 229–343 ms |
  | real hand monitor | 302–312 ms | 251–294 ms | 1–63 ms | 275–324 ms |

  The hold step here is a 15 Hz loop that records when it would write goals := present; no bus is involved (A's test).
  The same through the runner on the fake bus is D's.

**C · planner + IK (built 12:56 IST: 31 passed, 2 failed; after the speed-limit change, re-run by the integrator at
13:00: 35 passed, 0 failed — the history is under "Failed or unknown").**
Each name is `planner/tests/test_planner.py::test_fixture[<name>]`, run for both IKs: `[sim]` =
`SO101Kinematics.solve` + tool offset, `[null]` = `anchor/null_plan.ik`. Errors are re-measured through
`anchor/fk.py`. The knife's dimensions are placeholders (handle 100 mm, blade overhang 20 mm, width 20 mm).

- 50 plans from 124 seeded random palms (6 via re-orient): fingertip error max 0.066 mm `[sim]` / 0.061 mm `[null]`; handle heading error max 0.020 / 0.019 deg — `PLAN_50_targets_ik_fk_error`
- 25125 rows in 56 plans: the knife's closest point to the palm 60.43 mm — `PLAN_handle_tip_never_inside_60mm`
- 0 rows outside the box (fingertip, handle tip, blade tip), 0 outside joint limits; largest step 2.985 deg/s — `PLAN_every_row_in_box_and_limits`
- The G3 guard at production limits over every plan through the fault table's servo model: 0 fires (control) — `PLAN_g3_guard_replay_production_limits`
- One row's pan pushed out of the box → `ee_workspace` fires — `PLAN_g3_guard_pan_pushed_out_fires`
- A palm outside the box and an unreachable palm inside it → refused with a reason, no rows — `PLAN_unreachable_palm_refused`
- Blade toward the palm → handover refused; re-orient plan keeps the knife ≥ 60.1 mm away; handover then succeeds — `PLAN_blade_toward_palm_refused_then_reorient`
- Re-target by 30 mm keeps every check; the same drift toward the knife is refused — `PLAN_retarget_30mm`
- `z_handover` unset on an armed config → refused — `PLAN_z_handover_unset_on_arm_refused`
- Knife inside the standoff at the start → refused — `PLAN_inside_standoff_at_start_refused`
- Same inputs → same `plan_sha256` — `PLAN_same_inputs_same_rows`
- 40 fingertip targets: IK error max 0.0196 mm `[sim]`, 0.0001 mm `[null]` — `IK_40_fingertip_targets_through_anchor_fk`
- `IK_beyond_reach_reports_its_error`, `KNIFE_axis_sign_from_the_grasp`, `KNIFE_close_target_from_the_caliper` (20 mm → 9.91 %), `KNIFE_longer_handle_keeps_the_standoff`, `CFG_limits_are_the_guards_and_an_edited_calibration_is_refused`, `RUN_frozen_folders_not_written`

The two IKs: both pass 15 of 16; `plan_handover` median 74 ms `[sim]` / 79 ms `[null]`; reachability identical. The
default wired in is `[sim]`, as the operator's prompt instructs; the choice is the operator's at M1.

Reachable palms on a 20 mm grid over the workspace box (160 palms), placeholder knife:

| start pose | direct | via re-orient | no plan |
|---|---|---|---|
| null lift pose over cross 6 | 55 | 26 | 79 |
| mid-carry, handle outward | 57 | 16 | 87 |
| over the zone centre, handle outward | 31 | 23 | 106 |

The limit is the box's near edge, not arm reach: the grip point sits 162 mm from the palm and must itself be inside
the box. A palm within 60 mm of the knife at the freeze is always refused.

**A on the operator's rig stills (captured 18:12–18:35, checked 18:40; monitor venv 14 passed, 0 skipped).** Each
scene has three stills in `fixtures/frames/`; the latest manifest row of each matches its file's sha256.

- 3 of 3 → 0 hands — `FRAME_empty_table`
- 3 of 3 → 1 hand, conf 0.94–0.97, palm 8.6 / 9.0 / 9.3 mm from cross 6, `in_spawn` true, `in_zone` false — `FRAME_hand_cross6`
- 3 of 3 → 1 hand, conf 0.95–0.96, palm 11.4 / 11.6 / 11.7 mm from the zone centre, `in_zone` true — `FRAME_hand_zone`
- 3 of 3 → 2 hands (one in the spawn box, one in the zone) — `FRAME_two_hands`
- 3 of 3 → 0 hands with the gripper in view (posed over the zone, not the spawn box) — `FRAME_arm_no_hand`
- 3 of 3 → 0 hands with the knife on the spawn box — `FRAME_knife_no_hand`
- `fist_zone`: 1 hand, conf 0.88–0.91, `in_zone` true (no monitor expectation; it is the judge's scene)
- `proc_ms` on rig stills: 15–16 with no hand, 26–27 with one, 38 with two

## Failed or unknown

- **Placement trial 20:25 (KH-S20261003T202508), ended by [k]:** the placement planned and started by itself at
  20:28 (judge 0.3 'handover'); 15 s into the approach the palm was 17 mm off and the 15 mm limit froze it; the menu's
  [h] / [o] then ran the standoff handover (refused: 'standoff 60', 'the tip would leave the box'). Fixed (PLAN
  decision 10): 50 mm in the approach with a re-plan above the palm, the ends of the screwdriver allowed past the box,
  [h] retries the placement, no [o].
- **Palm placement (2026-10-03, built and dry-run only): the pointed end >= 30 mm from the palm in 3D on every
  row.** Release 20 mm above the palm (+16 mm commanded for the sag). Dry runs: zone
  side, a 135-180 deg re-orient on the spawn box, a hand moving mid-descent (freezes), UNSAFE after a settle (asks),
  UNSAFE at the release (asks for [r]), no plan (asks). **Not yet run on the arm.** Known limits: placement fits only
  palms x ~200-280, y -60..+120 mm with the fingers pointed away from the robot or to its right (or toward the zone
  around (220, -60..+30) for the turn); squares 4 and 8 cannot be served (`sessions/KH-DAWN-20261003/placement_map.png`).
  The palm height is the monitor's assumption (20 mm); a thicker hand is that much nearer the jaws. The placement
  rubric on the stills: 12 of 12 as expected (open hands p_unsafe 0.3-0.4 - four of six recommend `wait` at 0.4, below
  the 0.5 bar, so the code proceeds; fists and two hands 0.6-0.7, UNSAFE).
- **Finding (day 2): the arm rides below its commanded pose with the screwdriver, and the knife-to-palm check reads
  the commanded pose.** Measured tip (planner FK of the encoder reads) against the commanded tip, the same direction
  in every pass of step 5 (KH-S20261003T124318, T001–T003): at CP1 (lifted at the grasp) 19.5–20.4 mm, z −15.6 to
  −15.9; at CP2 (over the zone) 12.3–13.0 mm, z −8.9 to −9.3, x −7 to −8 (toward the base). The hold test of
  2026-10-02 20:38 lifted ~6.8 mm of the 20 mm sent (shoulder_lift 3.1°, elbow_flex 2.0° short). The per-step
  check (`retarget._knife_too_close`) and the planner use the commanded rows, so the real knife can be up to ~20 mm
  nearer a palm than they compute. **Mitigation tried, then dropped:** `--standoff-mm 80` (60 + the 20 mm
  measured) in step 6 (KH-S20261003T130937): 1 of 29 handover requests planned. The operator then lowered the floor
  to 30 mm (PLAN, day-2 decision 4); **at 30 commanded, the real handle-tip gap can be ~10 mm** - this finding is not
  mitigated in the sessions that follow. Reach: `sessions/KH-DAWN-20261003/reach_80.json` / `.png`, `reach_30.json` /
  `.png`. **The proper fix, for later:** run the
  knife check on the measured pose as well as the row about to be sent (FK of the latest encoder read, every step),
  and plan from the measured pose with the load's error fed forward (or a gravity term) so the hold lands where it is
  planned; then return the standoff to 60.
- **Step 6, first session (KH-S20261003T130937, `--standoff-mm 80`, ended by [k]):** T001 PLACED (the third clean
  no-hand pass). T002 HANDED_OVER: the hand came in during the carry at (207.7, 96.1); [h] refused 10 times (the
  knife 27–72 mm from the palm with the standoff at 80; twice no hand seen), planned at 13:16:34 for the palm at
  (247.4, 132.6), CP3 not UNSAFE, [r]; the retreat froze on monitor 7 (the operator's hand holding the screwdriver,
  (218.1, −5.3)), [w], [c] after 3.5 min. T003 ABORTED by [k]: 25 refusals - the knife inside the 80 mm standoff
  where the arm was; no handover pose with the blade tip or the fingertip inside the workspace box for palms at
  (305–332, 4–39); for the palm at (317.4, −146.2) [h] refused (the blade heading toward the palm) and [o] refused
  (no re-orientation: the blade tip would leave the box) - the tip-side refusal, logged with its numbers (13:23:58–
  13:24:08). Monitor 7 held 84.7 / 46.6 / 65.0 ms after its frame; control step max 68.96 ms; chain ok, 3684 rows.
- **Step 5, trial 2 (KH-S20261003T124318/T002), a real test of monitor 7 on the rig:** the operator's hand came in
  early, at the handle, during the GRASP descent (tool 22 mm up). Monitor 7 fired on one raw frame; the arm was held
  97 ms after that frame; judge UNSAFE; [w]; judge not UNSAFE; [c] (offered only once all eight were clear);
  resumed; PLACED. No [h] / [o] in that menu (not holding).
- Finding 4, `test_SR04_on_step_that_raises_freezes_held`: failed in the full run of 21:31 (its stand-in had the old
  two-argument signature); fixed; passed alone at 21:49 and in the status run of 22:12–22:30 (runner 143/143).
- **Live judge, reported (ruling 2):** with `claude-haiku-4-5-20251001`, 13 of 15 in the status run; one call timed
  out at 8.004 s (`hand_cross6_1`); misses on `hand_zone_1` (twice) and `hand_zone_3` (once). Earlier, with
  `claude-opus-5-5` named by `JUDGE_MODEL` (20:45, one call each): 12 passed, 3 failed. Answered calls 3.4–6.6 s; the first two calls
  (`empty_table_1`, `empty_table_2`) timed out at 8.0 s (budget 8 s) -> UNSAFE, abort; `hand_cross6_3` answered
  `hand_open_waiting: false` (expected true: "in the workspace rather than waiting at a handover"). 0 of 13 answered
  replies were fenced: ruling 8's fence handling is not built (nothing to strip). `empty_table_3` answered
  `p_unsafe` 0.5 (UNSAFE by the code's reading) for "an unidentified dark shape at the bottom-left edge"; every hand
  scene answered `p_unsafe` 0.6–0.8 (UNSAFE). Output: `fixtures/judge_live_20261001T2045.txt`.
- **Monitor 5 outside a re-target: 4 of the 20 runs.** Steps of 175.6, 97.5, 131.8, 93.2 ms (limit 89.9) with load
  averages 8.1–26.5; Chrome renderers and WindowServer held the Mac at a load average of 6–27 throughout (13.8 at 21:08
  with none of these tests running). Run 3 froze 5 s in, during the scripted segment, and never reached a re-target:
  **19 of the 20 consecutive runs exercised a re-target, not 20.** Earlier the same evening: a 109 ms step in the
  scripted carry (21:05) and the 94 ms step that started SR08 (20:05). Steps of 80–86 ms appear with load 6–10.
- `sessions/fixture_status.json` is written (22:30, `all_green: true`) for code hash `5990435b58dc…`; any change to
  the code or the fixture frames makes `--arm` refuse it until `fixtures/status.py` is run again.
- Encoders at 15 deg/s (monitor 2, handover) and at the 3 deg/s dawn speed: unknown until the arm is powered.
- The placeholder grasp and knife: the handle tip is outside the workspace box on the last two carry rows of the
  null's cell 6 (y 152 and 162 against 143); the G3 box check reads the fingertip only. The close target (9.91 % for a
  20 mm handle) is not verified with a hold. Both are replaced at dawn.
- "Runner freezing within 300 ms" of the hand monitor's death: the stop flag was within 300 ms in all 16 runs; the
  hold step was up to 343 ms after the kill (PLAN F5).
- The OpenAI request shape and the Anthropic listing's pagination are from memory; only the Anthropic path has been
  called live.
- Framing: the re-check FAILED three times at 18:17–18:55 (2.76, 2.76, 3.54 px against 2.5) while the camera was off
  its place; after the operator put it back, `recheck2_1..3.png` (19:01) PASSED at 1.58 / 1.46 px. The hand stills of
  18:17–18:35 were taken with the camera about 1.5 mm off; dawn step 1 measures the mapping again, live.

## Blocked on the operator

- The prop's measurements for `--knife-handle-mm`, `--knife-blade-mm`, `--knife-width-mm`; `--z-handover-mm` at dawn.
- A quiet Mac for every timing run: the load from other applications fired monitor 5 in 4 of 20 dry runs.
- "arm on" for the dawn protocol (M3).

## Exact commands to reproduce

```
cd experiments/knife_handover
PYTHONDONTWRITEBYTECODE=1 ~/lerobot-mps-venv/bin/python -m pytest tests judge planner -q -p no:cacheprovider \
    --deselect judge/tests/test_judge.py::test_saved_frame_live          # about 17 min under load
PYTHONDONTWRITEBYTECODE=1 ~/lerobot-mps-venv/bin/python -m pytest judge/tests/test_judge.py -k saved_frame_live -s   # 15 live calls
PYTHONDONTWRITEBYTECODE=1 ~/lerobot-mps-venv/bin/python tests/m5_retarget_runs.py --runs 20                  # about 22 min
PYTHONDONTWRITEBYTECODE=1 ~/lerobot-mps-venv/bin/python fault_table_handover.py                             # table + 4 sessions
PYTHONDONTWRITEBYTECODE=1 ~/lerobot-mps-venv/bin/python replay/build_replay.py sessions/_dryrun/KH-DRY-20261001T214959
PYTHONDONTWRITEBYTECODE=1 ~/lerobot-mps-venv/bin/python handover_session.py --dry-run --yes --no-esc
PYTHONDONTWRITEBYTECODE=1 ~/lerobot-mps-venv/bin/python handover_session.py --dry-run --yes --no-esc --layer-off --dry-keys ""
PYTHONDONTWRITEBYTECODE=1 ~/lerobot-mps-venv/bin/python replay/build_replay.py sessions/KH-S20261004T134743   # + present.html
PYTHONDONTWRITEBYTECODE=1 ~/lerobot-mps-venv/bin/python replay/live.py        # the live view: rebuilds the newest session
PYTHONDONTWRITEBYTECODE=1 ~/lerobot-mps-venv/bin/python -m judge.env          # which variable NAMES are set; never a value
PYTHONDONTWRITEBYTECODE=1 .venv_monitor/bin/python -m pytest monitor -q -p no:cacheprovider -rs
PYTHONDONTWRITEBYTECODE=1 ~/lerobot-mps-venv/bin/python fixtures/status.py   # writes sessions/fixture_status.json (--arm reads it)
python3 secrets_scan.py                                                      # before every commit (staged diff)
find ../../anchor ../../phase0 ../../so101_sim -newer PLAN.md -type f         # must print nothing
uv venv --python /opt/homebrew/bin/python3.12 .venv_monitor && uv pip install --python .venv_monitor/bin/python -r monitor/requirements.txt
```
