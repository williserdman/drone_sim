# Human handoff / current status

[Start here](../README.md) · [Architecture](architecture.md) · [Runbook](runbook.md) ·
[Contribution rules](../AGENTS.md)

Audited 2026-10-06. VFS 2027 implementation adds configurable A/B/C targets
(default C), version-2 eligibility groups, camera/payload tools, and automatic
phase judging to the existing configured runner. Its world starts unloaded
with three rigid marked payloads in S. The ground mission, liquid/breakage, and
physical removal of failed drops remain deferred. Operator commands and current
contracts live in the [runbook](runbook.md) and module guides.

All seven VFS runtime images built successfully at `26e0d07`; physical mission,
score, and full-window artifact verification remain pending. First launch
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
