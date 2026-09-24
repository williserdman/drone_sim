# Human handoff / current status

[Start here](../README.md) · [Architecture](architecture.md) · [Runbook](runbook.md) ·
[Contribution rules](../AGENTS.md)

Audited 2026-09-24. Branch `design/moving-pad-landing` adds the moving-pad
world, configured precision-landing operation, concurrent camera observation,
SITL overlay, physical scoring, and independent artifact checks. The core-runner
PR and imported `companion/comp2026` source remain unchanged.

The implementation is committed, but flight acceptance is not complete. The
stationary control takes off, reaches its waypoint, acquires marker 7, and enters
LAND. It then oscillates until the marker leaves the camera view. Tracking loss
fails the mission and requests GUIDED hold. No stationary landing or moving-pad
landing has passed; this is not a demo-ready mission.

Current source checks passed 2,071 tests, with 27 ROS/environment cases skipped.
The unchanged imported Comp2026 suite passed 1,126 tests. All seven runtime images
were built from clean `c523af3adce8230624cf8c7953920ca51c3f0891`; native Gazebo
integration checks passed 4/4. Focused new-module type checks passed; the full
repository still has pre-existing annotation errors. Source checks do not prove
physical landing or artifact acceptance.

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

Five stationary attempts were preserved. The first three exposed startup RPC,
contact-watermark, and shutdown defects, now covered by focused regressions.
The fourth exposed acquisition initialization between camera frames; `c523af3`
fixes that and increases the relative precision timeout to the full public run
budget without changing the absolute settle/acquisition deadlines.

| Run | Physical outcome | Score | Artifacts / acceptance | Runtime source |
| --- | --- | --- | --- | --- |
| `0f608531-736f-495e-a6b9-b55afb0452f9` | Takeoff at 7.05 s; waypoint at 14.45 s; no LAND; aborted at 54 s | 0/100, incomplete | `ABORTED`; rosbag incomplete at abort boundary; not accepted | `1b4efbf42ef55f75af8c887de77af1870995eb7b` |
| `24a1f1c5-7846-40d4-9c4d-ddcd6da15506` | Takeoff at 7.05 s; waypoint at 14.55 s; LAND at 17.35 s; tracking lost at 20.40 s; no touchdown/disarm | 0/100, incomplete | `FAILED`; rosbag incomplete at failure boundary; not accepted | `c523af3adce8230624cf8c7953920ca51c3f0891` |

Each bundle lives at `runs/RUN_ID/` in the moving-pad worktree. Videos are
`video/onboard.mp4` and `video/observer.mp4`; manifests contain both clean source
revisions, all seven image digests, frozen configuration, and artifact hashes.
Manifest SHA-256 values, in table order:

- `2e0d4998b47ad454e6b5329f84e9e29d0900f474ccb02cd768f1f6e5f6b756b7`
- `ccfd1ae4b74a5c2709454ea2bc5e3b609918016db735be0e9d6d8e36633aeb93`

The latest onboard recording contains 408 frames at 640x480/20 Hz. Recorded
camera/range timestamps match at every 50 ms tick. Production camera replay
finds marker 7 through 19.85 s; it clips the image edge at 19.90 s and remains
undetected through failure. Ground-truth projection agrees within 0.91 pixels;
body-right vector error averages 8.9 mm, with 31 mm maximum. Roll reaches
about -17.7 degrees. This establishes real field-of-view loss, not a timestamp
or mounting-sign defect. The 0.55-second loss correctly crosses the 0.50-second
tracking threshold.

The doubtful assumption is that re-enabling the Kalman precision estimator
provides stable descent with this airframe's existing gains. The base
`descent.parm` explicitly uses the raw estimator to avoid earlier Kalman
oscillation; the moving overlay reintroduces the Kalman estimator. Recorded
Kalman lateral position lags and briefly opposes the raw target offset. This
supports an estimator/control interaction, but does not establish a validated fix.
The raw estimator is a candidate comparison; it supplies zero target-velocity
feedforward even when moving-target support is enabled.
Further full-flight retries stopped after repeated fixes. The 0.5 m/s moving
mission has not been launched, pending a successful stationary control.

Machine-local diagnostics and prelaunch expectations are under
`.superpowers/sdd/moving-pad/`, including `stationary-5-diagnostics/REPORT.md`
and `provenance.json`. Preserve them with the run bundles. The superseded
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

Current Phase 3 tags point to the clean `c523af3` build used by the latest
stationary attempt. Prelaunch expectations were captured before that run.
Subsequent documentation commits do not change those recorded source identities.
Earlier images remain under preservation tags; never retag them as new evidence.
Rebuild after runtime edits and capture new expectations before the next flight.

Physical landing: not achieved. Score: 0/100, incomplete. Artifact acceptance:
not passed. Images: rebuilt. No current competition flight was run in this task;
the accepted competition evidence above remains historical.

## Active priorities

1. Compare `PLND_EST_TYPE=0` against the current value of 1 in a bounded
   stationary diagnostic, keeping `PLND_OPTIONS=5` and all other settings fixed.
   Keep the live parameter guard aligned with the diagnostic profile; verify
   continuous target visibility and absence of growing lateral/roll oscillation.
2. Obtain an independently accepted stationary control, then the 0.5 m/s moving
   mission. Require physical landing, 100/100, complete recordings, and provenance.
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
