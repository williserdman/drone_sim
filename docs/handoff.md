# Human handoff / current status

[Start here](../README.md) · [Architecture](architecture.md) · [Runbook](runbook.md) ·
[Contribution rules](../AGENTS.md)

Audited 2026-10-06. VFS 2027 implementation adds configurable A/B/C targets
(default C), version-2 eligibility groups, camera/payload tools, and automatic
phase judging to the existing configured runner. Its world starts unloaded
with three rigid marked payloads in S. The ground mission, liquid/breakage, and
physical removal of failed drops remain deferred. Operator commands and current
contracts live in the [runbook](runbook.md) and module guides.

Current VFS flight evidence is run `ed9d6023-aa59-4d26-8d77-61dd34c7dc86`
at runtime revision `34d85c0`. All three payloads were dropped at C, retrieved,
and released grounded in L; the aircraft returned to H and disarmed. FM3 was
approved at 588.5 public seconds and 579.7 active seconds, with 365/380 points.
The 780-second recording finalized, but the terminal bundle failed because
`/simulation/scenario_events` had zero messages. That legacy descent event is
intentionally inert for both competition profiles; VFS uses payload events,
judge decisions, and attempt status. Its validator now applies the existing
competition empty-diagnostic exception to VFS as well, retaining every required
VFS stream check. The regression and missing-judge/attempt controls passed within
72 focused checks (four ROS skips). Native full decoding also validated both
15,600-frame videos. Replaying the preserved bag then exposed a separate timing
bug: independent replay treated the FM2 start at 85.60 seconds as an inspection
pause before FM2 takeoff at 88.85. The judge and replay now require FM2 airborne
evidence before a home/disarm pause; inherited FM1 home contact cannot pause the
clock. Both new regressions failed before the correction and passed afterward,
with 42 focused checks. Native source-bound replay now agrees with all four judge
decisions and independently validates 579.7 active seconds and 365/380 points.
These checks are diagnostic only: the original failed manifest remains unchanged.
All seven images rebuilt at `6637136`; installed native replay validated the
preserved bag and exact judge clocks. Flight `6e004e62-1728-4d53-a5ca-2d04b7355192`
then completed all three C drops and received FM2 approval at 293.50 public
seconds and 290.15 active seconds, freezing IDs 2, 3, and 4. FM3 failed on its
first retrieval at 346.20 public seconds with `precision target reacquisition
failed`. The aircraft touched down on payload 2 but remained armed in GUIDED;
no L delivery or FM3 approval occurred. Its score was 215/380; terminal artifact
finalization is pending. The original run remains failed evidence.

Focused regressions reproduced two configured landing gaps: a future range scan
hid the last clock-current scan, and fresh near-ground range did not refresh
target health. A subsequent scan gap could therefore immediately switch LAND
to GUIDED. The adapter now uses the existing mission's committed/pending scan
pattern, and near-ground tracking refreshes health within the unchanged 500 ms
timeout. Three new regressions failed before the fix; 53 focused companion
checks passed afterward. The exact triggering callback order of that flight remains unrecorded;
these deterministic regressions do not establish a successful physical flight.
Matching rebuilt images and a fresh accepted full-window run remain pending.

The preceding integration failures below are historical evidence. All seven VFS
runtime images first built successfully at `26e0d07`. First launch
`fe0831d5-c8d8-4532-84b6-db3509cc5705` failed before takeoff: artifact validation
looked for the mission-plan version at the run document's top level. The check
now reads the nested mission plan, with a real host-snapshot regression.
Second launch `94992697-9946-4d04-85ff-c1d9948e7da9` passed artifact startup,
released the public epoch, and issued takeoff. Gazebo then stopped on payload
support lookahead exhaustion. Its join waited for a future leg contact event,
but those events stop while airborne. The fix closes absent-leg intervals with
later odometry and holds vehicle truth until matching payload support arrives,
retaining strict payload-grid checks. Neither failed launch
proves a completed physical mission or a valid full-window bundle.
Support-join verification passed 19 host tests with 10 ROS skips and two native
ROS regressions. A delayed attached, grounded payload still produces grounded
vehicle truth; 25 sparse-leg epochs drain, and missing payload truth faults at
the existing 20-epoch bound. The pure aggregator type check passed.
All seven images rebuilt at `eccad33`; installed Gazebo adapter hashes matched
the committed source. Third launch `c3d7d3a6-e5c5-4afb-8fea-973a7a13f362`
took off to 10 m and landed/disarmed at D. Its immediate 47.95 s check still
saw public status 47.9 s as armed, so the automatic judge rejected it correctly.
Checks now wait one published disarmed interval before emission. The actual
failure timeline, freshness, armed resets, and correlation passed 53 focused
tests; relevant type checks passed. Full mission and full-window acceptance
remain pending.
Fourth launch `b6f40106-ae81-4012-91aa-22939b82c92a`, with all seven images at
`b3c4ab3`, received the D approval at 48.1 s, proving the published-disarm fix
in flight. After rearming at D, the H command used ArduPilot's updated home
position and landed at D again; the H judge correctly rejected it. Course
navigation now freezes the first complete observed home pair. H and payload
target regressions retain their original ENU coordinates after later home
updates; 65 focused checks and the operations type check passed. That run shut
down normally. Full three-payload flight and artifact acceptance remain pending.
Fifth launch `034f5639-8e00-4e48-bd89-6578e664e830` at `37104be` passed both
FM1 checks (D 48.1 s, H 96.2 s). It reached payload 2's 4.572 m acquisition
hover, then failed at 117.5 s because the Gazebo calibration lacked verified
image dimensions. The existing calibration now declares the simulator's
640×480 dimensions; its focal length matches the configured 0.6 rad field of
view. Precision-readiness checks remain enforced. Full payload flight and
artifact acceptance remain pending.
Sixth launch `503793a8-f88c-497a-91d1-efbbca910de6` at `a1c4988` passed FM1
and acquired marker 2 at 130.35 s, then failed precision reacquisition at
159.35 s. A read-only live capture proved the actual 640×480 RGB8 image; the
installed camera backend detected ID 2 with a 4.668 m down vector. A slow-frame
reproduction showed the 0.1 s wall wait repeatedly cancelling the camera's
acquisition worker before 0.31 s frames could commit. The worker now waits up to
1 s in wall time while retaining the 500 ms simulation-time freshness limit.
Precision hold counts fresh frames across intervening telemetry/clock ticks;
fresh invalid observations still reset the count. The real-manager slow-frame
handoff and interleaved-tick regressions passed within 58 focused checks;
relevant type checks passed. Full payload flight and artifact acceptance remain
pending.
Seventh launch `f22afe60-3d48-40c6-94e4-eb8431642d42` at `5a0d2aa` passed
two precision pickups and releases at C (payload 2 at 154.25 s, payload 3 at
212.8 s). Its third acquisition failed immediately at 233.95 s: the camera
returned a cached observation newer than the previous marker's sequence, but
older than the exposure limit. An actual-manager reproduction returned that
same error before a fresh frame arrived. New marker requests now start after
the camera's current sequence; stale exposures remain rejected. The exact
cached-stale/fresh-frame handoff passed within 34 focused checks, and the worker
type check passed. FM2 eligibility
and full payload/artifact acceptance remain unverified.
Eighth launch `4f5fdb39-c230-4aa8-bf2d-924782915ed1` at `6245893` passed
the first two C drops and acquired marker 4, then failed precision landing at 244.65 s
with a generic camera-worker error. DataFlash confirms LAND; the final MCAP
chunk is unreadable, so neither fresh ROS range nor the underlying camera error
can be recovered. The configured landing policy now checks its fresh near-ground
range guard before polling the camera. New requests isolate their result queues,
and terminal camera errors retain their cause chain. These changes passed 35
focused tests and relevant type checks; they do not establish this run's root
camera cause or a completed mission. Full three-payload flight, score, and
artifact acceptance remain pending.
Ninth launch `a5234350-97c8-4285-af84-39b9293c891a` at `fa98aae` passed FM1 D
at 48.2 s, then its H check at 96.3 s failed with stale evidence. Native MCAP
showed valid H contact and disarm: the request reached the judge after ground
truth 96.2 s but before already-produced 96.25/96.3 s samples, making the latest
consumed truth 100 ms old against the unchanged 50 ms limit. The scorekeeper
runtime now joins exact physical epochs in canonical truth/payload/status order
and defers a check to its original epoch, with a 500 ms simulation-time
fail-closed bound. Focused source verification is recorded with the implementing
change: 60 scorekeeper tests and relevant type checks passed. All seven images
rebuilt at `706ed48`; the installed scorekeeper replayed the ninth run's native
MCAP receive order and approved D/H with their exact original physical epochs.
Tenth launch `ca5cc52e-1503-4a92-a88b-edcb8b9f74c6` passed FM1 D/H at
48.3/96.4 s, all three precision pickups and airborne C drops, and FM2 approval
at 310.0 s with eligible IDs 2, 3, 4 and 306.65 s active elapsed time. All three
payloads physically settled at C. Its first FM3 precision retrieval failed at
344.15 s: a stale frame permanently ended the camera acquisition worker. No L
delivery or full-window artifact acceptance was established. An installed-image
reproduction queued four images at 343.55–343.70 s while the clock was 344.15 s;
the adapter selected the oldest, 600 ms-old image even though a newer image was
within the unchanged 500 ms limit. Configured autonomy now requires newest fresh
image consumption at this adapter boundary, with an age recheck after copying
pixels. The configured adapter and unchanged default path passed 67 focused
tests; four imported freshness/future/replay guards passed separately. The
changed frame-source class type check passed; whole-file checks retain existing
legacy coordinate-narrowing errors and unavailable host ROS imports. All seven
temporary images built at `1348f68`; the installed adapter selected the 450
ms-old newest image in the same queued-frame reproduction. Before launch, a
native ROS callback-order probe found another boundary requirement: Gazebo
published clock before image, but the subscriber handled image first. Selecting
that newest image while
the accepted clock was still 50 ms earlier correctly triggered the camera's
future-frame guard. Configured acquisition must retain such images until their
clock is accepted, then wake without requiring another image. The camera's own
future-frame guard remains unchanged. The source change passed 69 focused tests
and the changed frame-source class type check. All seven images were rebuilt at
`b84f722`; the installed configured companion selected the newest fresh queued
frame, and a native ROS image-first callback probe confirmed that capture waited
for the accepted clock, then completed without another image. The failed run
finalized normally; its incomplete bag and absent finalized videos cannot
establish artifact validity.

Eleventh launch `5d503177-1e0a-4b76-ad44-d78531a9d359` at `b84f722` passed
FM1, placed all three payloads at C, received FM2 approval with IDs 2, 3, and 4,
and physically delivered all three payloads on the ground in L. The active
attempt reached 600 seconds during the final H LAND, and the mission failed at
606.75 s public time with `active attempt deadline expired`. Its failed score was
215/380: A 65, BP 90 with the 1.5 weight multiplier, and BA 15; without FM3
approval, the three physical L deliveries earned no CP, CA, or time bonus. The
configured plan now descends at the same waypoint from its stationary 10 m
transit hold to 4.572 m before each of the seven ordinary landings at D, H, FM2
H, L2, L3, L4, and final H. Transit and airborne release remain at 10 m. A fresh
flight and full-window bundle are still required for complete acceptance.
The first image build exposed an unavailable Ubuntu Python venv
pin; three affected runtime Dockerfiles now use the available `.17` revision.
The FFmpeg lock now uses available `libpciaccess0` `.3`; both exact artifact
apt sets simulated successfully. Gazebo provenance checks
passed seven tests. Gazebo's existing pinned dependency layer builds from local
cache; reconstructing that older lock from current repositories remains deferred.
Its fresh runtime source compiled the new messages, and a native probe passed
three CDR round trips, three MCAP reads, and three artifact evidence decodes.
This is build/serialization evidence, not a flight pass.
Full host gate on `06ff611`: 2,093 passed, 27 skipped in 57.79 s. Bundled mission
gate: 1,126 passed in 24.46 s. All eight templates resolve and schema-check;
mixed A/B/C targets survive source snapshots and change resolved aimpoints.
Phase 3 Compose config, lock check, compile, and local documentation link checks
passed. These checks do not prove live ROS/Gazebo/SITL behavior; host skips include
ROS-dependent recording cases.
Final review fixes passed 59 focused tests: longer recording windows, terminal
approval deadlines, all eligible L placements, and physical judge evidence links.
Relevant type checks passed. The superseded implementation plan is preserved in
Git history at `dd27044`.

Historical audit, 2026-09-21: this branch added the fixed-sequence configured runner with
seven basic tools and two diagnostic templates. The existing competition path
is unchanged from `main`. Competition conversion, search-and-deliver, camera,
and QGC work remain on `archive/configured-missions-full-20260921` for separate
follow-ups.

Source verification passed 904 tests across companion, orchestration, shared
runtime statuses/protocol, Gazebo runtime, and contracts. All seven run templates
validate, and Phase 3 Compose configuration resolves. No images were rebuilt or
flights run for this reduced branch; physical outcome, score, and artifact
validity have not been verified on this source. Rebuild before launching the
[configured example](runbook.md#local-developer-workflow).

Historical audit, 2026-09-11: fast precision-landing recovery was implemented and
verified on the isolated `fix/precision-landing-reacquire` parent and nested
branches. Fresh run `b3dfad75-4630-4233-84e3-836943459903` completed the
600-second window with accepted terminal artifacts and a 150/150 score.

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
