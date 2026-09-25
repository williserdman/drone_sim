# Human handoff / current status

[Start here](../README.md) · [Architecture](architecture.md) · [Runbook](runbook.md) ·
[Contribution rules](../AGENTS.md)

Audited 2026-09-25. Branch `design/moving-pad-landing` adds the moving-pad
world, configured precision-landing operation, concurrent camera observation,
SITL overlay, physical scoring, and independent artifact checks. The core-runner
PR and imported `companion/comp2026` source remain unchanged.

The stationary A/B with `PLND_OPTIONS=4` landed on the deck and scored 100/100.
Peak roll fell from 19.04 to 1.17 degrees. Artifact finalization rejected the
bundle because retrospective touchdown score events have earlier timestamps
than the final score event published before them. The moving-target profile
with option 5 still lacks a stable landing. A subsequent actual 0.5 m/s moving
flight reached LAND at about 49.08 s, lost tracking at 51.55 s, and scored 0/100.
Neither run is an accepted moving-pad demo.

The diagnostic passed 250 companion/SITL tests. All seven runtime images were
built from clean `7ec09e1a7f02849b264690aa763da40d4fdb4c38`. Earlier full checks
at `c523af3` passed 2,071 tests with 27 ROS/environment cases skipped; the
unchanged imported Comp2026 suite passed 1,126 tests and native Gazebo checks
passed 4/4. Those broader checks were not repeated for the parameter-only A/B.
The full repository still has pre-existing annotation errors. Source checks do
not prove physical landing or artifact acceptance.

Historical audit, 2026-09-11: fast precision-landing recovery was implemented and
verified on the isolated `fix/precision-landing-reacquire` parent and nested
branches. Fresh run `b3dfad75-4630-4233-84e3-836943459903` completed the
600-second window with accepted terminal artifacts and a 150/150 score.

## Maintainer start

Read the architecture map, then use the runbook for current setup, build, launch,
and acceptance commands. The tracked `companion/comp2026` source is included in
a fresh clone. Verify monorepo HEAD before a Phase 3 build; a mission log or
score alone is not a pass.

## Moving-pad verification

Completed diagnostic: one stationary flight with `PLND_OPTIONS=4`, retaining
`PLND_EST_TYPE=1`, `PLND_LAG=0.08`, all gains, geometry, and tracking-loss policy.
The recorded launch overlay and companion parameter gate selected this profile. It was
an experiment, not a promoted moving-pad fix. Recorded control parameters
confirm that only `PLND_OPTIONS` changed. Frozen mission inputs match the fifth
attempt except for run identity and its checksum.

The recorded Kalman moving candidate restored `PLND_OPTIONS=5` and tested `PLND_LAG=0.04`, with
camera exposure timestamps preserved through MAVLink and chronological score
events. The target run is `config/configured-moving-pad-run.json`: the pad moves
at 0.5 m/s from mission start. Run `2a5f3972-c851-4d13-b92e-145ef34cedb6`
failed above final clearance without touchdown or disarm. Current affected-module
checks passed 1,335 tests with
12 ROS/environment cases skipped. These cover companion, SITL, scorekeeper, and
artifacts; unchanged native Gazebo and imported Comp2026 suites were not rerun.

The next experiment changes only `PLND_EST_TYPE` from 1 to 0. It retains option 5,
40 ms lag, all gains, geometry, and loss policy. Raw mode supplies zero target
velocity and follows camera positions; whether this can track the 0.5 m/s pad
through the shrinking field of view remains unproven. A fresh build, prelaunch
provenance, and actual moving run are required. The parameter-only follow-up
passed 251 companion/SITL tests; other suites were not repeated after the prior
1,335-test check.

Six stationary attempts were preserved. The first three exposed startup RPC,
contact-watermark, and shutdown defects, now covered by focused regressions.
The fourth exposed acquisition initialization between camera frames; `c523af3`
fixes that and increases the relative precision timeout to the full public run
budget without changing the absolute settle/acquisition deadlines.

| Run | Physical outcome | Score | Artifacts / acceptance | Runtime source |
| --- | --- | --- | --- | --- |
| `0f608531-736f-495e-a6b9-b55afb0452f9` | Takeoff at 7.05 s; waypoint at 14.45 s; no LAND; aborted at 54 s | 0/100, incomplete | `ABORTED`; rosbag incomplete at abort boundary; not accepted | `1b4efbf42ef55f75af8c887de77af1870995eb7b` |
| `24a1f1c5-7846-40d4-9c4d-ddcd6da15506` | Takeoff at 7.05 s; waypoint at 14.55 s; LAND at 17.35 s; tracking lost at 20.40 s; no touchdown/disarm | 0/100, incomplete | `FAILED`; rosbag incomplete at failure boundary; not accepted | `c523af3adce8230624cf8c7953920ca51c3f0891` |
| `f7c75af5-12a3-4bda-b4c2-41c519ac3943` | Takeoff at 7.05 s; waypoint at 14.45 s; touchdown at 27.45 s; observed disarm/mission success at 30.05 s | 100/100, complete | Full 90 s recorded; `FAILED` for nonmonotonic score-event timestamps; not accepted | `7ec09e1a7f02849b264690aa763da40d4fdb4c38` |
| `2a5f3972-c851-4d13-b92e-145ef34cedb6` | Moving pad; takeoff 7.05 s; waypoint 14.55 s; LAND about 49.08 s; tracking lost 51.55 s; no touchdown/disarm | 0/100, incomplete | `FAILED`; abort-tail pad/vehicle grid mismatch; not accepted | `921e09b9eca6eaab9756d54a43994959eae5a867` |

Each bundle lives at `runs/RUN_ID/` in the moving-pad worktree. Videos are
`video/onboard.mp4` and `video/observer.mp4`; manifests contain both clean source
revisions, all seven image digests, frozen configuration, and artifact hashes.
Manifest SHA-256 values, in table order:

- `2e0d4998b47ad454e6b5329f84e9e29d0900f474ccb02cd768f1f6e5f6b756b7`
- `ccfd1ae4b74a5c2709454ea2bc5e3b609918016db735be0e9d6d8e36633aeb93`
- `124157bdd19718dfd2fd21b1011f0c9b531a80950803ffd517f6066082918fa1`
- `d3dd03c03195e9fe7a292e53ae0488f78b04d09655d824ca6ffedb6db2911abf`

The failed option-5 onboard recording contains 408 frames at 640x480/20 Hz. Recorded
camera/range timestamps match at every 50 ms tick. Production camera replay
finds marker 7 through 19.85 s; it clips the image edge at 19.90 s and remains
undetected through failure. Ground-truth projection agrees within 0.91 pixels;
body-right vector error averages 8.9 mm, with 31 mm maximum. Roll reaches
about -17.7 degrees. This establishes real field-of-view loss, not a
camera/range timestamp or mounting-sign defect. The 0.55-second loss correctly crosses the 0.50-second
tracking threshold.

Both stationary recordings used `LANDING_TARGET.time_usec=0`. Pinned ArduPilot
jitter correction then clamps measurement time to its maximum lag after an
initial constant-timestamp period. The current candidate carries camera exposure
time through the policy and operation owner, converting nanoseconds to MAVLink
microseconds. Focused tests cover delayed observations, duplicate suppression,
and wire conversion. The subsequent moving flight still failed. The old zero field alone
does not establish why the estimator inferred false motion.

The failed baseline questioned whether Kalman moving-target tracking provides
stable descent with this airframe's existing gains. The base
`descent.parm` explicitly uses the raw estimator to avoid earlier Kalman
oscillation; the option-5 overlay reintroduced Kalman estimation and enabled
moving-target velocity feedforward. Baseline DataFlash analysis finds that the stationary
pad's inferred east velocity grows to roughly 0.5-0.8 m/s and enters the navigation
command. The aircraft follows the growing roll command with 0.19 degrees mean
absolute error; motor outputs remain clear of their limits. This locates the
failure upstream of attitude control without proving the estimator is its sole
cause. Raw `PL.mY` and predicted `PL.pY` describe different time horizons, so their
earlier wrong-sign comparison alone was not proof of filter lag.
The approved option-4 A/B then disabled velocity feedforward while retaining
Kalman position estimation. Over the same first 3.038 s after LAND, peak actual
roll fell from 19.04 to 1.17 degrees and mean roll tracking error from 0.189 to
0.016 degrees. Actual roll stayed within 1.17 degrees throughout descent and
disarm. ArduPilot target loss occurred after physical touchdown. This supports
false target-velocity feedforward as the immediate destabilizing path, but does
not establish why the estimator inferred motion. Raw estimator type 0 changes
both position filtering and target-velocity behavior, so remains a broader
fallback experiment.

The subsequent moving flight used clean `921e09b`, matching all seven captured
image IDs and both source identities. Recorded pad position advances from
10.0255 to 35.7755 m over 51.5 s, with east velocity 0.5 m/s throughout. The
marker was found at 47.10 s; ArduPilot reported initialization complete at
49.10 s. During 2.44 s of LAND, peak roll reached 14.54 degrees and inferred
pad east velocity ranged from -0.359 to 1.207 m/s against the actual 0.5 m/s.
The aircraft followed the roll command with 0.177 degrees mean absolute error.
GUIDED recovery appears in DataFlash at about 51.52 s; the companion failed
the operation at 51.55 s. These estimates align two matched ArduPilot status
messages with 0.34 ms maximum residual. The timing patch and shorter lag did
not resolve the moving-target instability. No new gain or guard changes were
made after this failure.

Both moving-run videos contain 1,031 frames at 640x480/20 Hz, lasting 51.55 s,
with hashes matching the manifest. Strict acceptance rejects the failed
manifest. Its bag has 1,031 pad samples and 1,030 vehicle samples at the abort
boundary, so the physical grid is incomplete. No touchdown events were produced,
so this flight does not verify the successful-landing score-ordering fix end
to end; that fix has focused test coverage only.

The option-4 scorer records touchdown offset 0.00570 m and relative speed
0.0607 m/s. Both H.264 videos contain 1,800 frames at 640x480/20 Hz and last 90 s.
Bag validation rejects score-event times `90.00, 27.45, 27.45, 90.00` s: the final
physical result precedes two retrospective touchdown diagnostics in event-ID
order. Independent full acceptance exits 1 on the failed manifest. A separate,
partial semantic check recomputes 100/100 from all 1,800 vehicle/pad samples;
it does not make the bundle accepted. Preserve this
bundle unchanged; reconcile the event ordering contract before another accepted
flight. The diagnosis is not permission to remove timestamp checks.

Machine-local diagnostics and prelaunch expectations are under
`.superpowers/sdd/moving-pad/`, including `stationary-5-diagnostics/REPORT.md`,
the `stationary-5-diagnostics/flight-log-analysis/` reports and timeline plot,
`options4-ab-f7c75af5-12a3-4bda-b4c2-41c519ac3943.json` and its plot,
`options4-artifact-diagnosis.md`, and `provenance-options4.json`. Preserve them
with the run bundles. The superseded
implementation checklist remains in Git at `63f9674`; the architecture now
contains the implemented contract and this handoff owns remaining validation.

## Verified behavior and limits

| Evidence | Physical mission | Score | Terminal artifacts | Limits |
| --- | --- | --- | --- | --- |
| [Accepted competition baseline](verification/comp2026-mvp.md), run `3dc895c3-a5aa-4651-a893-d21884fa43f5` | Returned Home and disarmed in the accepted 600-second run | 150/150 | `COMPLETED`; accepted bundle | Historical external evidence; does not prove current source or images |
| [Payload timestamp verification](payload-timestamp-fix.md), run `259863d9-c558-4102-ae34-fe6c31f5cf94` | Completed all three payloads and Home | 150/150 | `FAILED`; terminal artifacts invalid after a later downward-range timestamp/grid fault | Proves the recorded physical flight, not a full-window pass |
| Fast precision-landing recovery, run `b3dfad75-4630-4233-84e3-836943459903` | Payload 2 released; payloads 3 and 4 attached, lifted, and released; Home disarmed and completed at 261.15 s | 150/150 | `COMPLETED`; canonical semantic acceptance passed; manifest SHA-256 `9ed6496414746e76d2e146128d5055e95a2a2c219c6ece83d724add9040e85a7` | Machine-local run directory; preserve the complete bundle when transferring |

Run directories and absolute paths in verification notes are ignored, machine-local
evidence. Transfer a needed bundle with its manifest, checksums, configuration, and provenance intact.

## Historical source verification

Focused verification for the precision-landing branch passed 171 host companion
and ArduPilot tests and 81 nested mission tests. The profile now preserves
`LAND_SPD_MS=0.50`, fast-final-descent precision landing, and the promoted
AutoTune gains. The nested mission now uses atomic camera observations, validates
the live profile, fixes a five-frame median anchor, holds and reacquires in
GUIDED, and retries acquisition once before failing closed. Below
`PLND_ALT_MIN=0.75`, it keeps LAND active without requiring marker visibility so
the normal near-ground loss of the marker cannot interrupt touchdown.

The first fresh attempt, `3db34962-84ad-44cd-bc51-bfbdfc585fba`, exposed that
near-ground edge case: target 3 left the camera view at about 0.095 m AGL, the
controller entered GUIDED hold, retried, and failed FM3_3. The solution mirrors
ArduPilot's precision floor in the companion and is covered by a regression
test. In the accepted rerun, target 3 and target 4 both logged the handoff below
0.75 m and physically attached without a recovery retry.

Both accepted videos are H.264, 640x480 at 20 fps, with exactly 12,000 frames
and 600 seconds duration. DataFlash recorded the exact guarded profile. Across
471 acquired-target attitude samples, desired roll/pitch stayed within
0.67/0.31 degrees and actual roll/pitch within 0.77/0.25 degrees.

The earlier cleanup-branch verification below remains historical context:

The checkout-independent command was:

```bash
uv run --locked pytest orchestration/tests artifacts/tests companion/tests \
  ardupilot_sitl/tests gazebo/tests electromagnet/tests scorekeeper/tests tests/contracts \
  tests/integration/test_phase2_runtime_contract.py tests/integration/test_phase3_runtime_contract.py -q
```

Controller verification reported `1846 passed, 26 skipped, 48 failed in 112.67s`.
Independent Task 14 review reproduced those counts and classification in `77.41s`;
wall duration is host-dependent. Every failure came from the absent `companion/comp2026`
dependency or a consequence, so this is not a full pass. The prior baseline was `1841 passed, 26 skipped, 49 failed`; its unrelated ABA timing flake did not recur.

`uv lock --check`, full-source `compileall`, the corrected deletion audit, and
`git diff --check` passed. No container, image, or physical integration check ran.

| Task | Production Python net lines |
| --- | ---: |
| 1, dead code | -66 |
| 2, ArduPilot workspace | 0 |
| 3, Make target | 0 |
| 4, Compose test isolation | 0 |
| 5, SITL resource cleanup | +31 |
| 6, video test synchronization | 0 |
| 7, typed Gazebo readiness | -57 |
| 8, redundant test subscriptions | 0 |
| 9, timestamp selector | -21 |
| 10, score finalization | -55 |
| 11, payload command ledger | -46 |
| 12, status codec metadata | -49 |
| 13, adapter consolidation | -1 |
| **Total** | **+498/-762, net -264** |

Independent task reviews 1-14 and final Standards/Spec reviews passed after
scoped review fixes; no Critical, Important, or Minor findings remain.

## Image and runtime boundary

Current Phase 3 tags point to the clean `921e09b` build used by the moving
attempt. Prelaunch expectations were captured in `provenance-moving-timing.json`
before that run. The stationary images remain preserved separately.
Subsequent documentation commits do not change those recorded source identities.
Earlier images remain under preservation tags; never retag them as new evidence.
Rebuild after runtime edits and capture new expectations before the next flight.

Physical stationary landing: achieved. Score: 100/100, complete. Artifact acceptance:
not passed. Moving flight: attempted, failed tracking, no landing. Images: rebuilt. No current competition flight was run in this task;
the accepted competition evidence above remains historical.

## Active priorities

1. Run the controlled raw-estimator experiment on the same moving course.
   Preserve `2a5f3972` evidence, all gains, and the tracking-loss guard. Do not
   present the stationary run or the moving option bit as moving-flight success.
2. Require physical landing, 100/100, complete recordings, and independent artifact
   acceptance before claiming the moving mission works. The new lag remains a
   hypothesis until flight evidence supports it.
3. Separately diagnose historical competition range-stream loss before claiming
   a fresh full-window competition baseline.

Lower-priority work remains deferred: structured precision-phase diagnostics; improve zero-budget Compose timeout
diagnostics; freeze final flight-exchange counters; investigate rare process and
session races; expand malformed-input, network, and multi-vehicle stress tests;
make rendering hosts reproducible; and add later-command simulation-time
rendezvous where required.

## Preservation warning

Preserve `runs/`, ArduPilot parameters, Gazebo resources and textures,
provenance, licenses, Compose/image configuration, and imported mission history.
A source edit does not rebuild an image. Before cleanup or integration, check
Git status and keep evidence tied to its source revisions and image digests.
