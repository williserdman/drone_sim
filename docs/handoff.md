# Human handoff / current status

[Start here](../README.md) · [Architecture](architecture.md) · [Runbook](runbook.md) ·
[Contribution rules](../AGENTS.md)

Audited 2026-09-28. Branch `design/moving-pad-landing` adds the moving-pad
world, configured precision-landing operation, concurrent camera observation,
SITL overlay, physical scoring, and independent artifact checks. The core-runner
PR and imported `companion/comp2026` source remain unchanged.

The stock EKF3 moving run `a3f79da9-037d-4734-ac4d-e8381149f407` landed on the
0.5 m/s deck, disarmed, and passed independent acceptance with 100/100 and full
90-second recordings. Native ArduPilot LAND, camera, pad speed, and tracking
guards were unchanged. This is the first accepted moving-pad recording.
The stationary control also landed, but a later missing contact sample stopped
its recording at 49.5 s. That bundle failed acceptance; repeatability remains
unproven. Earlier failed experiments are preserved below.

The subsequent offline diagnosis identifies a frame error in the pinned
ArduPilot SIM attitude path used by `AHRS_EKF_TYPE=10`. It returns body-frame
delta velocity to the precision estimator's NED interface. Source execution
and recorded prediction-only intervals agree on the error. The approved
moving-only stock EKF3 profile is implemented and has fresh moving-flight evidence.
See the native-estimator findings below.

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

The raw follow-up changed only `PLND_EST_TYPE` from 1 to 0, retaining option 5,
40 ms lag, all gains, geometry, and loss policy. Run `08113757` reached LAND and
tracked longer, but failed at 63.65 s without touchdown or disarm. Raw mode
supplies zero target velocity; this result does not validate position-only
moving landing. The parameter-only follow-up passed 251 companion/SITL tests; other suites were not repeated after the prior
1,335-test check.

The approved 2026-09-28 experiment changed only `PSC_NE_POS_P` from the recorded
default of 1 to 4 in the moving profile. The companion requires that value before
flight. Raw estimation, pad motion, camera, other gains, and guards stay fixed.
The proposed reduction in following error was a hypothesis. Success requires
actual touchdown/disarm, 100/100, and independent artifact acceptance from a
fresh recorded run. Focused companion/SITL checks
passed 252 tests; other modules were unchanged and their suites were not rerun.
Run `2b5c9332-757a-4f75-a477-aef315427354` did not land within 90 public seconds.
The host was still waiting for mission completion after the public clock capped;
explicit abort preserved the recording. Physical outcome: no touchdown/disarm.
Score: incomplete 0/100. Terminal state: `ABORTED`, not accepted.

The approved native-LAND follow-up sets `AHRS_EKF_TYPE=3`, restores
`PLND_EST_TYPE=1` and `PSC_NE_POS_P=1`, and keeps option 5, 40 ms lag, geometry,
and guards unchanged. Relative to the earlier Kalman moving run `2a5f3972`,
only the aircraft estimator changes. The companion now requires its readback
before flight. Focused checks passed 254 companion/SITL tests; unchanged modules
were not rerun. All seven runtime images were built from clean `faef600` for
both flights. The stationary control touched down at 27.40 s and reported disarm
at 30.05 s; peak LAND roll was 0.940 degrees. Its adapter then failed on a missing
landing-pad contact sample at 49.50 s. Pad truth has 989 ticks through 49.45 s,
while vehicle truth and both videos have 990 through 49.50 s. Score: incomplete
0/100. Bundle: `FAILED`, not accepted. Private source timestamps were not recorded,
so a dropped contact sample cannot yet be distinguished from delayed delivery
or timestamp misalignment. Do not synthesize missing contact evidence.

The actual moving flight touched down at 59.10 s, 8.7 mm from the pad center,
and reported disarm at 61.05 s. It completed with 100/100 and independent
artifact acceptance. Pad and vehicle truth have matching 1,800-sample grids;
both 640x480/20 fps videos contain 1,800 frames and last 90 s. Pad speed stayed
between 0.499999989 and 0.500008820 m/s through the full window, including
touchdown and disarm. Its 44.975 m displacement confirms continued motion.
All 25 artifacts, video hashes, both clean source identities, and seven image
digests match the captured expectations. This also verifies chronological
touchdown score events in an accepted full-window run.
DataFlash confirms the guarded profile. Peak LAND roll fell from 14.541 degrees
in the earlier Kalman moving flight to 4.110 degrees. Inferred pad east velocity
stayed between 0.487 and 0.552 m/s, median 0.515 m/s; no GUIDED recovery followed
LAND. These estimates use precision-relative velocity plus recorded navigation
velocity, independently of the scorer's diagnostic below.

The recorded touchdown-relative-speed diagnostic needs a frame-semantics fix:
the adapter copies child-frame odometry twists while the scorer subtracts them
as if they share a frame. Do not treat its 0.722 m/s value as physical relative
speed. The 100-point rule checks deck contact, disarm, and remaining aboard;
it does not gate on this velocity diagnostic. Preserve the accepted bundle and
fix the diagnostic separately rather than rewriting recorded evidence.

The supplemental motion-proof helper initially rejected contact-scale speed
variation under a 1e-9 m/s tolerance. Its preserved second report uses 1e-5 m/s
for the moving pad; canonical artifact acceptance passed unchanged in both
checks. A passive private contact/odometry subscriber ran from native time
about 68 s through teardown, outside the bundle, without sending flight commands.
The stationary contact fault did not recur in the moving run.

EKF3 manifest SHA-256 values: stationary
`0ed43de8850e78d14f1c200625e6d16842bd287d5380371ffe46b6e969306e6b`;
moving `645576c3759c7c6bdd1f5c9b5f8fb7f39a9ed2ec04594b615c456d9a2f638a51`.

Earlier stationary attempts were preserved. The first three exposed startup RPC,
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
| `08113757-a7d3-4f5d-aa21-ca26ea0fa1be` | Moving pad, raw estimator; takeoff 7.05 s; waypoint 14.45 s; tracking lost 63.65 s; no touchdown/disarm | 0/100, incomplete | `FAILED`; not accepted | `2371e61cf497168e0e5c1cf9316c2194eff0dbf9` |
| `2b5c9332-757a-4f75-a477-aef315427354` | Moving pad, raw estimator, gain 4; takeoff 7.05 s; waypoint 14.45 s; no touchdown/disarm in 90 s | 0/100, incomplete | `ABORTED` after recording window; pad/vehicle grid mismatch; not accepted | `a14347c5bcec4d134c725b909ff6cc9a8f15dc60` |
| `8446a4e6-7b48-411b-89e5-794ea8d510ca` | Stationary pad, EKF3; touchdown 27.40 s; disarm 30.05 s | 0/100, incomplete | `FAILED`; missing contact sample at 49.50 s; not accepted | `faef600a4bd711c00619450c9eff1aa7f279f0a9` |
| `a3f79da9-037d-4734-ac4d-e8381149f407` | Moving pad, EKF3; touchdown 59.10 s; disarm 61.05 s | 100/100, complete | `COMPLETED`; full 90 s; independent acceptance passed | `faef600a4bd711c00619450c9eff1aa7f279f0a9` |

Each bundle lives at `runs/RUN_ID/` in the moving-pad worktree. Videos are
`video/onboard.mp4` and `video/observer.mp4`; manifests contain both clean source
revisions, all seven image digests, frozen configuration, and artifact hashes.
Earlier manifest SHA-256 values, in the first six table rows' order:

- `2e0d4998b47ad454e6b5329f84e9e29d0900f474ccb02cd768f1f6e5f6b756b7`
- `ccfd1ae4b74a5c2709454ea2bc5e3b609918016db735be0e9d6d8e36633aeb93`
- `124157bdd19718dfd2fd21b1011f0c9b531a80950803ffd517f6066082918fa1`
- `d3dd03c03195e9fe7a292e53ae0488f78b04d09655d824ca6ffedb6db2911abf`
- `f5b05cbac3a2148d0a4fff894d5da9eb63391dc89461d30c2161c737d7c330a4`
- `2e4587ace597352612a78b13c4d390f563698746ae3a2211ddc10ecb8df84fe4`

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

The raw follow-up lasted 14.486 s from LAND to GUIDED. After the initial
transient, vehicle east speed had median 0.499 m/s and target-right offset
stayed around 0.52-0.55 m. The initial explanation attributed this to
`speed / PSC_NE_POS_P`; the gain-4 result below disproves that model. Peak roll was
5.335 degrees during initial capture, with 0.048 degrees mean tracking error.
The last fresh target was 0.542 m right at 1.914 m down. With the pinned 0.6 rad
horizontal camera FOV and 0.1 m marker, the visible center limit is also 0.542 m.
Thus raw mode removed the large oscillation but lost the marker at the shrinking
image edge. GUIDED followed 0.523 s after the final measurement. Exact relative
times come from DataFlash; absolute public alignment has only one matched status
message. The 0.5 m horizontal descent gate also explains intermediate plateaus.
Further tuning is stopped pending a control choice that addresses following
error without recreating the velocity instability.

The gain-4 DataFlash log confirms `PSC_NE_POS_P=4`. Median fresh target offset
remained 0.534 m, essentially unchanged from the prior raw flight. Peak actual
roll reached 8.01 degrees during capture; tracking error averaged 0.035 degrees
through the recorded window. The final bounded measurement was 0.535 m east at
2.118 m down, with about 0.070 m of horizontal marker margin. No measurement
reached the 0.75 m handoff. ArduPilot retained target acquisition throughout the
window; its later target-loss message occurred after the public sensor stream
ended and must not be treated as an in-window tracking failure.
At the end, the commanded east position and aircraft east position differed by
only about 0.0003 m while the pad remained roughly 0.54 m ahead. This contradicts
the proposed assumption that the observed following offset would shrink as
`speed / PSC_NE_POS_P`; the aircraft already tracked the intermediate command.
The pinned [precision estimator](https://github.com/ArduPilot/ardupilot/blob/1511f27194f1dcc3728270883047bdf022b3fd53/libraries/AC_PrecLand/AC_PrecLand.cpp)
supplies zero target velocity in raw mode. The [position controller](https://github.com/ArduPilot/ardupilot/blob/1511f27194f1dcc3728270883047bdf022b3fd53/libraries/AC_AttitudeControl/AC_PosControl.cpp)
shapes the incoming position before applying `PSC_NE_POS_P` between the shaped
target and aircraft. The [input shaper](https://github.com/ArduPilot/ardupilot/blob/1511f27194f1dcc3728270883047bdf022b3fd53/libraries/AP_Math/control.cpp)
derives its correction gain from jerk and acceleration limits. Median logged
shaped-target error fell from 0.66 mm to 0.09 mm with gain 4, while pad offset
remained unchanged. This supports an upstream lag problem; the logs do not
separate input shaping from measurement delay. Further parameter changes stop
here; the next design must address target motion in that upstream command path.
The analysis bounds DataFlash using the single pre-window-end target-found
message, so absolute public timing has event-quantization uncertainty. It excludes
the post-window target-loss message from alignment.

The gain-4 pad moved 44.975 m between public 0.05 and 90 s, confirming 0.5 m/s.
Both videos contain 1,800 frames at 640x480/20 Hz and last 90 s. Video hashes,
both source revisions, and all seven image IDs match prelaunch expectations.
Strict acceptance rejects the aborted manifest. Its bag has 1,800 pad samples
versus 1,799 vehicle samples, so the physical grid is incomplete.

### Native-estimator findings

Read-only diagnosis on 2026-09-28 found a specific frame error in pinned
[AP_AHRS::_getCorrectedDeltaVelocityNED](https://github.com/ArduPilot/ardupilot/blob/1511f27194f1dcc3728270883047bdf022b3fd53/libraries/AP_AHRS/AP_AHRS.cpp#L2660).
The SIM branch leaves `imu_idx=-1`; its early return passes integrated body
specific force through without the later body-to-NED rotation and gravity
correction. `AC_PrecLand` consumes that result as NED delta velocity. Both
Kalman runs recorded `AHRS_EKF_TYPE=10` and `PLND_OPTIONS=5`.

An isolated C++ probe executes that exact upstream function with stubbed sensor
inputs. At 0.14 rad roll and a 10 ms interval, the SIM branch returns zero east
delta velocity; the EKF3 branch returns the expected 0.0136845 m/s. This proves
the branch behavior, not a successful flight. Stationary DataFlash independently
matches it: during rejected camera fusions, relative east velocity changes at
about 0.000-0.003 m/s² while vehicle east acceleration is 1.1-1.4 m/s². The
prediction follows body-Y acceleration instead of subtracting earth-east motion.
Healthy raw IMU measurements do not rule out this interface conversion error.

The moving recording independently rules out a large camera-motion error.
Production camera vectors reconstructed with recorded exposure-time attitude
give 0.49676 m/s east pad speed over 49.10-51.00 s against true 0.5 m/s, with
23.9 mm maximum absolute east position error. Truth is used only for this offline
comparison. Exposure timestamp correction handles the public/native clock
offset; `PLND_LAG` still independently selects a fixed inertial-history depth.

The approved bounded experiment uses stock EKF3 (`AHRS_EKF_TYPE=3`) in the
moving profile, restoring Kalman precision estimation and the pre-gain-trial
position gain. It preserves native LAND, camera, 0.5 m/s pad motion, and guards.
This changes the aircraft's attitude/navigation estimate as well as the
delta-velocity path. The fresh stationary check and accepted moving flight above
support this profile; they do not prove the old frame error was the only possible
source of instability. No firmware patch or companion descent controller was needed.

The raw-run pad moved 31.825 m over 63.65 s at 0.5 m/s. Both videos contain
1,273 frames at 640x480/20 Hz and last 63.65 s. Both source revisions, all seven
image IDs, and video hashes match captured expectations. Strict acceptance
rejects the failed manifest; its abort tail has 1,274 pad samples versus 1,273
vehicle samples. Physical outcome: no touchdown/disarm. Score: incomplete 0/100.


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
`options4-artifact-diagnosis.md`, `provenance-options4.json`, the moving
`provenance-moving-timing.json`, `provenance-moving-raw.json`,
`provenance-moving-gain4.json`, `gain4-bounded-report.md`, and each
`moving-diagnostic-RUN_ID.json` / `moving-acceptance-RUN_PREFIX.log`. Preserve them
with the run bundles. `native-estimator-diagnosis/` contains the exact-source
`probe_ahrs_frame.py`, its C++ output, inertial checks, and moving-camera replay.
The superseded
implementation checklist remains in Git at `63f9674`; the architecture now
contains the implemented contract and this handoff owns remaining validation.
The EKF3 snapshots, acceptance logs, and flight reports use `stationary-ekf3-*`
and `moving-ekf3-*` names under the same ignored diagnostics directory. Use
`moving-ekf3-acceptance-a3f79da9-v2.log` for the completed supplemental motion
proof; preserve its first failed helper report too. The stationary contact
diagnosis is under `native-estimator-diagnosis/stationary-ekf3-contact/`.

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

Current runtime tags point to the clean `faef600` build used by both EKF3 flights.
Prelaunch expectations were captured separately in `provenance-stationary-ekf3.json`
and `provenance-moving-ekf3.json`. All seven images are also preserved under
`moving-pad-ekf3-faef600` tags. The earlier experiment images remain preserved separately.
Subsequent documentation commits do not change those recorded source identities.
Earlier images remain under preservation tags; never retag them as new evidence.
Rebuild after runtime edits and capture new expectations before the next flight.

Latest stationary landing: achieved. Score: incomplete 0/100 after the contact-stream
failure; artifact acceptance failed. Latest moving landing: achieved, 100/100,
independently accepted. Images: rebuilt. No current competition flight was run in this task;
the accepted competition evidence above remains historical.

## Active priorities

1. Diagnose the intermittent private pad-contact gap exposed by the stationary
   EKF3 control. Capture native and bridged timestamps plus tracker fault state
   before changing transport or join behavior. Keep missing evidence fail-closed.
2. Establish repeatability of the accepted EKF3 moving profile. Preserve the
   failed controls, accepted bundle, source identities, and image digests.
3. Resolve the configured-operation deadline beyond the capped public window.
   At 90 s the source finishes, but the host waits for mission completion while
   the precision timeout at 104.45 s is unreachable. Keep artifact-grid diagnosis
   separate; do not weaken acceptance.
4. Separately diagnose historical competition range-stream loss before claiming
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
