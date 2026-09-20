# Human handoff / current status

[Start here](../README.md) · [Architecture](architecture.md) · [Runbook](runbook.md) ·
[Contribution rules](../AGENTS.md)

Audited 2026-09-11. The fast precision-landing recovery is implemented and
verified on the isolated `fix/precision-landing-reacquire` parent and nested
branches. Fresh run `b3dfad75-4630-4233-84e3-836943459903` completed the
600-second window with accepted terminal artifacts and a 150/150 score.

## 2026-09-20 search observer framing

Raised the search-delivery observer from 30 m to 50 m, keeping its downward
orientation, ENU center (9, 10), field of view, and 1280×960 at 20 Hz.
The previous view clipped the route, especially near pickup at flight altitude.
Projection of all 4,800 positions from the successful September 19 flight now
fits a 1.5 m cube around the drone with at least 99 pixels to every frame edge.
The seven search asset tests and twelve competition asset tests passed.

Verification evidence is in `runs/builds/observer-height-20260920-68qlwskd/`.
The Gazebo image rebuild exited 0 and its installed world hash matches the
generated source. The other six runtime image identities are unchanged.
This is a geometry check against the recorded trajectory; a new flight and
visual recording check remain unrun. Existing recordings and their scores
retain the previous camera position. See [Gazebo](../gazebo/README.md) for the
current framing and asset regeneration command. Operator procedures and shared
interfaces are unchanged.

## 2026-09-19 search-and-deliver mission

Added [a 24-step search-and-deliver plan](../config/configured-search-delivery-run.json)
with its own registered course, scenario, world, and initially empty vehicle.
Only payload 3 is present. The route exercises turns, altitude changes, an
approach 2 m west of the marker, camera search/landing, physical pickup, delivery,
and home landing. Observer recording is 1280×960; onboard calibration remains
640×480. Both streams use 20 Hz and a 240-second public window after 90 seconds
of private warmup. See the [mission contract](../companion/README.md#search-and-deliver-contract)
and [run procedure](runbook.md#search-and-deliver-mission).

Initial source checks passed for companion (130), orchestration (472),
Gazebo/electromagnet (393, with 14 ROS-dependent skips), and scorekeeper (79).
An additional container run passed 35 Gazebo adapter/payload/asset tests with
ROS dependencies available. Artifact checks passed 535 tests with 12 skips;
the final combined scenario/runtime/acceptance subset passed 101 tests. A
generator check caught the delivery pad extending beyond the initial ground
plane; the generated 60 m plane now contains every pad.

First flight `752b97e0-79ce-42d0-ba6e-9078a07bcf20` used parent `658282a`
and unchanged nested `a89aede`. All seven images were rebuilt; 175 installed
source/asset hashes matched across the six changed images. Evidence is retained
under `runs/builds/search-delivery-20260919-8i8xhceh/`.

- Physical outcome: navigation and camera search reached marker 3. Precision
  landing failed at public 35.6 s with `Camera acquisition worker failed`.
  Local LAND recovery completed at 41.05 s; raw ground truth confirms stationary
  contact at the pickup marker. No payload attached or delivery occurred.
- Score: 0/100. Only SEARCH STARTED was published; the mission did not complete.
- Artifacts: terminal FAILED with an incomplete rosbag. Both MP4s decode at
  20 fps: onboard 640×480, 822 frames; observer 1280×960, 821 frames. The partial
  bag retains 821 vehicle samples and 822 payload samples. This is useful failure
  evidence, not a valid complete mission bundle.

The log preserved only the outer camera error. A real-component reproduction
failed 20/20 times when a frame aged beyond the 500 ms simulation freshness
limit during acquisition. That timing failure is the leading explanation;
the first flight's discarded exception cause prevents exact confirmation.

The follow-up replaces configured background acquisition with capture from a
ready, fresh frame under the runtime owner's I/O lock. Stale frames become
missing targets; the 500 ms limit is unchanged. Direct camera failures still
stop the mission, and operation results retain explicit exception causes.
All 571 companion tests passed; a final 81-test I/O, camera, operation and live-loop
subset also passed. This includes a real-camera test that excludes a
600 ms clock advance during frame handoff.

Retry `527fa45a-1565-40b1-b6da-6db57ae65f73` used parent `179fcfe` with nested
`a89aede` unchanged. The confirmed seven-image rebuild exited 0; 175 installed
source/asset checks matched. It completed in 3,521.23 wall seconds. Its audit is
`runs/builds/search-delivery-retry-20260919-tpz1_zf4/`.

- Physical outcome: all 24 operations succeeded. Raw evidence confirms search
  staging at 22 s, 0.772 m eastward search progress at 24.05 s, precision landing
  at 40.05 s, physical attachment at 40.1 s and attached lift at 44.35 s. Payload
  3 detached at 62.5 s and settled inside F2 during 63.95–64.95 s. HOME completed
  at 87.1 s after landing/disarm; final ground truth shows stationary contact
  inside the home pad.
- Score: the live scorer awarded 100/100. Raw evidence independently passes
  search, pickup, delivery and home conditions, including both stable release
  windows. The first semantic replay rejected the score because its new oracle
  incorrectly required a physical sample at time zero. An in-memory correction
  admitting the actual first sample at 50 ms makes the result and all score
  events match exactly. The correction passed 36 focused artifact/replay tests,
  including rejection of a missing first tick and an interior gap. Independent
  semantic acceptance then passed at 100/100 with the rebuilt checker from
  parent `0c4c3d4`, using the original flight's launch revisions and seven image
  identities. All 175 installed source/asset hashes matched that checker build.
- Artifacts: terminal COMPLETED / `mission_complete`, all 25 manifest artifacts
  valid and no incomplete paths. Each H.264 video has 4,800 decoded frames at
  20 fps for 240 s: observer 1280×960, onboard 640×480. The bag has 4,800 samples
  each for vehicle and payload 3, exactly 50 ms apart, seven mission events and
  two payload events. Normal teardown left no run resources.

The observer recording is `runs/527fa45a-1565-40b1-b6da-6db57ae65f73/video/observer.mp4`.
The retry audit contains `vehicle_trajectory.csv`,
`search_delivery_physical_audit.json` and `video-verification.json`. The rejected
initial replay report remains unchanged beside them. The checker correction and
revalidation audit is `runs/builds/search-replay-20260919-2xm7nofo/`; flight inputs,
recordings and published scores are preserved.
`revalidation.json` records both flight and checker revisions, the checker image
identity, matching original launch expectations, and empty run-resource inventories.
Only the replay module changed among runtime sources between the flight and
that recheck. The flight ran at `179fcfe`; the corrected checker at `0c4c3d4`
revalidated that preserved flight. Later documentation-only commits do not
change those evidence identities.

## 2026-09-19 configured competition conversion

The [competition config](../config/configured-competition-run.json) now expresses
the full FM1, FM2, FM3_3, FM3_4 and HOME attempt as 42 sequential tool calls.
The configured host adds calibrated camera precision landing, asynchronous
physically confirmed payload commands and ordered competition events while
retaining one MAVLink command owner. The guarded QGC route and independent
Comp2026 checkout remain unchanged. See the
[contract](../companion/README.md#configured-competition-contract) and
[launch command](runbook.md#configured-competition-plan).

Focused source verification passed **825 tests** across companion, orchestration
configuration, artifacts configuration, electromagnet runtime and Phase 3
contracts. This includes real PyMAVLink messages with fake ROS, range/attitude
timestamp integration, asynchronous payload confirmation and precision target
reacquisition. All 122 checked local links in the changed guides resolve.
Two subsequent missing/stale range cases passed in the 10-test range/release
subset. Real ROS, camera acquisition and range projection also passed a
container dependency check. All seven Phase 3 images were rebuilt from parent
`8a0dda398270d8590cb3eaffce2615a3f1b996cc` on `feat/configured-missions` with nested
`a89aede424036410dc98312b58d197230f0265d4` unchanged. Installed source verification
matched 23 companion files and orchestration configuration to the launch source.

Run `89fffa9f-6b10-403a-a8b0-0c29684d30d3` used the competition template unchanged
and completed in 3,762.64 wall seconds. Evidence is local under `runs/<run_id>/`:

- **Physical outcome:** all 42 operations succeeded. Camera precision landings
  on markers 3 and 4 completed at public times 108.1 and 174.1 seconds; both
  payloads then attached and lifted. Payloads 2, 3 and 4 settled inside F2 at
  74.9, 140.6 and 206.65 seconds. HOME completed at 250.15 seconds with observed
  landing/disarm; independent ground truth confirms home contact and zero speed.
- **Score:** `competition_v1` awarded 150/150. Independent replay through the
  installed artifact acceptance module also accepted 150/150 with matching
  launch revisions, dirty flags and all seven image identities.
- **Artifacts:** `collect-results` reports `COMPLETED` / `mission_complete`;
  all 25 manifest entries are valid with no incomplete paths. Both H.264 MP4s
  have 8,400 frames, 640×480, 20 fps and 420 seconds. The rosbag contains 8,400
  vehicle samples, 25,200 payload-state samples and the exact 11 mission events.
  Normal teardown left no run containers, networks or volumes.

Watch `video/observer.mp4` or `video/onboard.mp4`; tool arguments and outcomes
are in `logs/companion.jsonl`. Build, installed-source, physical and semantic
verification are retained in `runs/builds/configured-competition-20260919-_qwxb7up/`.
That directory also contains `trajectory.csv` and `physical-audit.json` derived
from the finalized bag. Acceptance was recorded before the documentation-only
follow-up commit; rerunning an inspector against a different HEAD can reject
the same evidence on provenance alone. The parent dirty flag reflects the
independent untracked `companion/comp2026/` directory; tracked launch source was
committed and its hashes matched exactly.

This proves one complete automatic simulation on the fixed course. Operator
waiting, failure recovery and precision target-loss branches were not exercised
in this flight. Standalone sensor tools, an agent endpoint, named waypoints,
configurable branching and exclusive compute scheduling remain deferred.

## 2026-09-19 configured mission runner

Added `mission: configured` with a checksum-bound inline plan, explicit mode and
arm operations, passive operator waiting, takeoff, numeric GPS waypoint flight,
hold, and land. The sequential caller uses reusable operation IDs/status and
stops on failure. A distinct execution-ready fact releases paused simulation
without inventing a GUIDED command. A bounded local LAND recovery preserves the
failed mission result. See the [tool contract](../companion/README.md#configured-diagnostic-missions)
and [launch procedure](runbook.md#configured-mission-runner).

Verification covers the working tree based on parent `2cece17`. The independent
Comp2026 checkout remained clean at `a89aede`. Focused source checks cover all
companion tests, orchestration configuration, runtime status validation, Gazebo
startup, and Phase 3 Compose contracts. The configured live-loop test uses real
PyMAVLink messages with fake ROS and transport; it proves wiring and cleanup,
not vehicle dynamics. The focused suite passed **823 tests**, with no failures or
skips. Both templates and all seven exported tool schemas validate,
and 176 local links across the affected guides resolve.

All seven Phase 3 images were rebuilt for the automatic template, including
pinned ArduCopter 4.5.7. The companion build initially failed because Ubuntu no
longer offered `python3.12-venv=3.12.3-1ubuntu0.15`; advancing that pin to the
available `3.12.3-1ubuntu0.17` produced a successful build. Other images reused
cached package layers; clean-cache availability of their package pins was not
checked. Build logs, launch source hashes, image identities, and verification
outputs are retained locally in `runs/builds/configured-20260919-03v6x7jv/`.
Seven installed companion source files matched their launch hashes exactly.

Run `36293c5d-2cc6-4d13-bb4a-5713746a1dff` used
[configured-descent-run.json](../config/configured-descent-run.json) unchanged
and completed in 1,806 wall seconds. Evidence is local under `runs/<run_id>/`:

- **Physical outcome:** GUIDED, arm, takeoff to 1.5 m, hold for two simulation
  seconds, and LAND all succeeded. Landing/disarm was observed at public time
  13.05 s. Independent reading of all 1,200 ground-truth samples found a 1.544 m
  rise above the initial pose and final ground contact with zero linear speed.
- **Score:** `descent_v1` awarded 100/100; all four rules passed.
- **Artifacts:** `collect-results` reports `COMPLETED` / `mission_complete`, with
  no incomplete paths. Both MP4s independently probe as H.264, 320×240, 20 fps,
  1,200 frames, 60 seconds. The rosbag is readable; `logs/companion.jsonl` records
  operation arguments and results. Run containers were removed by normal teardown.

This verifies one automatic descent in the Gazebo `vertical_descent` world.
Operator-wait plans, waypoint flight, and failure recovery have source-test
coverage but were not flown in this run. QGC integration and physical aircraft
remain outside this evidence. Precision landing is covered by the separate
competition run above. Standalone camera/LiDAR tools, agent transport, named
waypoints, branches/retries and exclusive compute scheduling remain deferred.
Operator-wait mode does not provision a new QGC connection.

## 2026-09-07 QGC flight-safety software checkpoint

The guarded parent QGC host and nested FM1/FM2 safety implementation are
accepted on offline source evidence. The final frozen-tree runs passed all
1,118 nested tests and 1,933 parent tests across artifacts, companion,
electromagnet, Gazebo, orchestration, scorekeeper, and root contracts; 27 parent
tests were explicitly skipped. The parent companion subset passed 476 tests.
Maximum-intelligence standards and spec reviews, plus a separate hostile-error
fidelity audit, report no actionable findings. Both repository diff checks pass,
and the operator waypoint store remains byte-identical to its initial snapshot.

On interruption, the companion retains simulation/navigation time for one
original-H cruise-altitude transit home followed by LAND, with one separately
guarded local-LAND fallback. A TAKEOFF interruption selects local LAND. It never
assumes the independent pilot has taken control: only a fresh healthy RC switch
edge followed by a new accepted `LOITER` or `STABILIZE` observation establishes
sticky `PILOT`, after which the companion is permanently command-silent. Fatal
infrastructure failures may stop the shared clock; overlapping error paths keep
the first error authoritative and cannot downgrade a required hard stop.

This is not deployment or flight approval. ArduCopter is pinned to 4.5.7 commit
`2a3dc4b7bf2507120f7378a7b2fde73185e0c325`; Ubuntu 22.04 is selected, but the
exact companion model/CPU architecture and hardware dependency lock are not
verified. No image matching this frozen source was built, and no live QGC, ROS,
integrated SITL, bench, device, actuator, or aircraft run was performed.
Hardware FM3 remains disabled. The known pre-existing roll-gain mismatch also
remains unresolved:
the dirty overlay has `ATC_RAT_RLL_P=0.0503722`, while its test expects
`0.0675`; do not change either without aircraft tuning evidence.

Keep every physical gate closed until the exact QGC build/profile/action file,
aircraft FC/RC configuration, FC watchdog/link-loss behavior, operating-site
corridor/reserve, rangefinder/precision-landing/landing-reposition/yaw policy,
matching image provenance, integrated fault matrix, propellers-removed bench,
and staged supervised-flight evidence are recorded.

## 2026-09-06 paused-startup correction

The guarded parent host now opts into the nested listener's narrow
`staged_simulation` telemetry mode. Source-only fake tests prove that listener
readiness no longer waits for telemetry cadence while Gazebo is paused, the
first admitted guarded GUIDED delivery precedes post-gate verification, and no
ARM output occurs before valid advancing telemetry. The default physical path
still completes telemetry configuration and collection before listener
installation. Cancellation, clock stop, missing installation, zero-time data,
wrong-source data, and failed verification remain closed and clean the startup
collector.

This is software evidence only. The remaining gate is a fresh pinned
ArduCopter 4.5.7, QGC application, parent companion, and Gazebo run that observes
the real pause-to-GUIDED release, post-release telemetry cadence, normal FM1/FM2
flight, failure cases, complete cleanup, score, and valid artifacts. No image
build, Docker run, device use, or physical flight was performed for this fix.

## 2026-09-06 firmware source target

Parent build source and provenance now target official ArduCopter 4.5.7 commit
`2a3dc4b7bf2507120f7378a7b2fde73185e0c325`. The native serial argument is
`tcp:5760`; the overlay translates the retained 0.50 m/s landing target to
`LAND_SPEED=50` cm/s and channel 0 passive readiness to `SR0_EXT_STAT=1` Hz.
It also translates the prior 2547.76 deg/s² roll acceleration request to
`ATC_ACCEL_R_MAX=254776` centidegrees/s². That request exceeds both releases'
published parameter ranges and is not a flight-approved tune. Source tests
stubbed the external process. No parent image was built, no parent integration
or flight ran, and existing image tags and historical 4.7 evidence remain
unchanged. The `comp2026_auto` quarantine still applies. The selected Ubuntu
22.04 LTS onboard target was not provisioned by this simulator-source change.

## Maintainer start

Read the architecture map, then use the runbook for current setup, build, launch,
and acceptance commands. The tracked `companion/comp2026` source is included in
a fresh clone. Verify monorepo HEAD before a Phase 3 build; a mission log or
score alone is not a pass.

## Verified behavior and limits

| Evidence | Physical mission | Score | Terminal artifacts | Limits |
| --- | --- | --- | --- | --- |
| [Accepted competition baseline](verification/comp2026-mvp.md), run `3dc895c3-a5aa-4651-a893-d21884fa43f5` | Returned Home and disarmed in the accepted 600-second run | 150/150 | `COMPLETED`; accepted bundle | Historical external evidence; does not prove current source or images |
| [Payload timestamp verification](payload-timestamp-fix.md), run `259863d9-c558-4102-ae34-fe6c31f5cf94` | Completed all three payloads and Home | 150/150 | `FAILED`; terminal artifacts invalid after a later downward-range timestamp/grid fault | Proves the recorded physical flight, not a full-window pass |
| Fast precision-landing recovery, run `b3dfad75-4630-4233-84e3-836943459903` | Payload 2 released; payloads 3 and 4 attached, lifted, and released; Home disarmed and completed at 261.15 s | 150/150 | `COMPLETED`; canonical semantic acceptance passed; manifest SHA-256 `9ed6496414746e76d2e146128d5055e95a2a2c219c6ece83d724add9040e85a7` | Machine-local run directory; preserve the complete bundle when transferring |

Run directories and absolute paths in verification notes are ignored, machine-local
evidence. Transfer a needed bundle with its manifest, checksums, configuration, and provenance intact.

## Current source verification

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

No image was rebuilt or retagged. Existing tags predate these changes and are
stale and unrun for this revision. That cleanup worktree had no
`companion/comp2026`; no symlink was created and no Phase 3 build was claimed.

Campaign outcomes are separate: physical mission `not run`; score `not produced`;
artifact validity `not evaluated`; images `not rebuilt`. Source tests do not change them.

## Active priorities

1. Diagnose the historical range-stream sample loss, then obtain a fresh pinned,
   full-window competition baseline. Check physical behavior, score, and artifact
   validity separately while keeping the timestamp and physical checks strict.

Lower-priority work remains deferred: improve zero-budget Compose timeout
diagnostics; freeze final flight-exchange counters; investigate rare process and
session races; expand malformed-input, network, and multi-vehicle stress tests;
make rendering hosts reproducible; and add later-command simulation-time
rendezvous where required.

## Preservation warning

Preserve `runs/`, ArduPilot parameters, Gazebo resources and textures,
provenance, licenses, Compose/image configuration, and imported mission history.
A source edit does not rebuild an image. Before cleanup or integration, check
Git status and keep evidence tied to its source revisions and image digests.
