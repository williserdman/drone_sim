# Human handoff / current status

[Start here](../README.md) · [Architecture](architecture.md) · [Runbook](runbook.md) ·
[Contribution rules](../AGENTS.md)

Audited 2026-10-06. Branch `design/moving-pad-landing` adds the moving-pad
world, configured precision-landing operation, concurrent camera observation,
SITL overlay, physical scoring, and independent artifact checks. The core-runner
PR and imported `companion/comp2026` source remain unchanged.

Calibration return update 2026-10-06: the companion now captures the disarmed
launch position, returns there in GUIDED at 5 m after tuning/gain activation,
and requires two stable seconds within 0.5 m before native LAND. Checks passed
2,285 host/module tests with 27 skips and 1,126 imported Comp2026 tests. Fresh
flight/recording validation is pending;
the older accepted and rejected landings below had no return command. Landing
speed, physical scoring and saved-gain verification are unchanged.

Implementation update 2026-10-05: branch `feat/manual-ci-suite` implements the
approved [11-setup design](superpowers/specs/2026-10-05-full-ci-suite-design.md)
and [plan](superpowers/plans/2026-10-05-full-ci-suite.md). The shared local `suite`
command builds once, gates consumers on accepted calibration/fresh reload and
reports mission-specific independent results. After the final review fixes,
host/module checks passed 2,257 tests with 27 environment skips; the imported
Comp2026 suite passed 1,126 tests. Focused suite/CLI checks passed 48 tests.
No static typechecker is configured; Python compilation passed. The fresh
common-build flight sweep remains pending.

Suite `2079a7e2-cd19-4bd0-ba31-0bc45e0338ed` from clean `76dd2b9` was gracefully
interrupted at 02:15 UTC on 2026-10-06 to test the startup fixes. Calibration,
fresh reload, configured descent, and controlled descent each established
`LANDED`, scored 100/100, and passed independent acceptance. Operator wait
failed at public 60 seconds: GUIDED executed, but its ACK was not observed and
ARM never ran. It scored 0/100 with rejected acceptance and no physical outcome.
The operator now drains MAVLink during private warmup without producing public
telemetry or flight commands, avoiding the previous interval without reads.

The interrupted hover-roll run
`0396e6e9-c5a0-414d-a584-691515da078f` reached the fixed 15-second private epoch
without publishing the calibration snapshot, execution-ready status, or first
mission command. The calibration guard remained unmet; the current evidence does
not expose which cached parameter names were absent. Gazebo therefore retained
the release barrier and no flight began. The roll diagnostic templates now use
the existing 90-second calibrated-consumer warmup while retaining their 45- and
120-second public windows, target RTF, seed, full 46-parameter gate, scoring, and
wall deadlines. Hover finalized `ABORTED` at 0/100 with rejected acceptance and
a ground-truth sample-count diagnostic; five later cases remained unrun. Both
suite processes and its Compose projects exited, and all evidence remains under
`runs/local-ci/20261006-76dd2b9/`. Both startup fixes are integrated on
`feat/manual-ci-suite`; neither has fresh native-flight validation yet. This
interrupted attempt is not an all-11 demonstration.

Post-integration checks on 2026-10-06 passed 2,265 host/module tests with 27
skips and 1,126 imported Comp2026 tests. Run the imported tests with
`--import-mode=prepend`; the root import mode cannot resolve their sibling-test
imports. The executable changes passed their focused checks before integration.

Fresh suite `72cd9035-94f5-42ad-9ee3-c5a2ec798731` from clean `04aa25a`
finished `FAILED` on 2026-10-06. Calibration run
`b75e44b6-a378-4503-bfa7-2d23a32dc964` completed its full 900-second recording.
Native all-axis AutoTune succeeded at public 725.299 seconds; the companion
verified all 34 parameters after activation at 725.55 seconds. Native LAND
began at 730.242 seconds, followed by disarm and saved Roll/Pitch/Yaw gains at
742.759 seconds. Post-disarm readback verified all 34 parameters, and the
companion recorded `LANDED` at 743.25 seconds. All 15 exported gains match
that readback within export precision.

The lifecycle completed without diagnostics, but physical scoring awarded
60/100 and independent acceptance rejected the required 100/100 gate. The
first contact sample at 740.850 seconds reported 0.438518 m/s smoothed speed,
above the stable-contact limit; the following ten samples were at most
0.001304 m/s. Contact and tilt passed throughout the evaluated half-second
interval. Calibration still uses the first-contact interval, unlike descent's
versioned bounded-settling policy. Native LAND targets 0.50 m/s in the current
profile; its [pinned parameter metadata](https://github.com/ArduPilot/ardupilot/blob/1511f27194f1dcc3728270883047bdf022b3fd53/ArduCopter/mode_land.cpp#L5-L12)
starts at 0.3 m/s. A parameter change alone does not establish the
first-contact stability requirement.

The other ten setups were blocked before allocation. The report's physical
outcome remains unknown because independent acceptance failed; the native and
companion landing observations above are separate evidence. Both suite processes
exited and the owned Compose project was removed. Reports, recordings and the
canonical 60/100 result remain under `runs/local-ci/20261006-04aa25a/`.
The landing-scoring contract decision is pending; no scoring rule or historical
evidence was changed, and this attempt does not validate the consumer startup
fixes or establish an all-11 pass.

Read-only comparison of the preserved native Gazebo recordings on 2026-10-06
found contact at native 611.487 seconds in accepted calibration `684758ab` and
830.850 seconds in rejected calibration `b75e44b6`. Their first public contact
samples correspond to native 611.500 and 830.850 seconds: respectively 13 ms
after contact and the same simulation epoch as contact. Both approach at about
0.50 m/s and their recorded poses stop descending within milliseconds. The
retained failed image uses Gazebo OdometryPublisher 8.11.0, whose
[velocity calculation](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.11.0/src/systems/odometry_publisher/OdometryPublisher.cc#L364-L451)
smooths per-step pose differences before publishing at this world's 20 Hz.
The first-contact stability check therefore depends on whether that sample's
velocity window still contains preimpact motion. Native pose reconstruction
supports this explanation; it is approximate because serialized poses are
rounded. The old accepted core images are no longer retained, so executable
equality across the two runs cannot be established. No scoring change follows
from this observation. The exact failing-image diagnostic repeat `509424dc`
observed physical contact at public 376.822 seconds and its first public contact
sample 28 ms later at 376.850 seconds: the preceding speed was 0.500456 m/s and
the contact sample 0.000796 m/s. The companion reported LANDED at 379.25 seconds.
The full recording then exceeded its 7,200-second wall deadline; lifecycle FAILED,
raw 0/100 with `ground_truth_sample_count_mismatch`, and rejected recording
acceptance are separate from that physical observation. The raw probe is retained
under `runs/diagnostics/20261006-calibration-repeat-04aa25a/`; this is diagnostic
evidence, not an accepted CI calibration.
Video inspection also found that both observer recordings miss the aircraft at
touchdown; the onboard view shows featureless ground. These recordings cannot
support visual landing acceptance. The rigid collision model and velocity/contact
score contain no landing-gear damage or payload-shock criterion, so passing this
contract does not establish a hardware-safe touchdown speed.

First suite attempt `0d11a7cb-e3f3-4379-86e0-95d76b274ef1` from clean `8f628be`
built all seven images and passed Comp2026 import smoke, then failed preflight
before allocating a flight. Compose returned identical image identities in
different orders; the suite compared ordered tuples. The suite now compares
name/digest mappings and still rejects changed or missing images. The failed
setup report is retained under `runs/local-ci/20261005-8f628be/suites/`.

Retry suite `793556e0-f809-48f8-b172-986816a301c1` from clean `f5a4411` passed
preflight, then calibration run `ee4bb972-5d94-4575-be42-90e7236e0864` failed
during startup before arming. Malformed TCP server arguments bound both native
UART channels to 5760; SITL exited and DroneKit's connection failed. Teardown
completed with no diagnostics, and the gate blocked all ten remaining cases.
The native arguments now use `tcp:5760` and `tcp:5762`; 49 SITL module tests
passed, and a pinned-binary smoke observed both distinct listeners and connected
both sockets. Failed evidence remains under `runs/local-ci/20261005-f5a4411/`.

Third suite `664c1546-c36c-4ccd-a746-015e00676652` from clean `2f2ba2e`
started calibration run `eba17a27-01bf-4e05-8b8c-efdb8773a8fa`. Native AutoTune
failed roll rate-D gain determination at public 43.95 seconds. The driver missed
that terminal text and kept waiting; the operator aborted the failed attempt,
and all owned containers were removed. The suite failed and blocked the ten
consumers. The interrupted recording has a ground-truth sample-count diagnostic;
it is not accepted calibration evidence. Starting parameters and aircraft inputs
match the earlier accepted calibration, but the first roll response differs;
the cause remains under investigation. The driver now recognizes all three
native gain-determination failure messages and enters existing bounded LAND
recovery. Focused companion/runtime checks passed 91 tests; fresh flight proof
of this change remains pending. Evidence is retained under
`runs/local-ci/20261005-2f2ba2e/`.

Fourth suite `8504b189-776f-442f-8053-62635ce0b601`, built from clean `91d8cae`,
finished `FAILED` at 21:26 UTC on 2026-10-05 and attempted all 11 setups.
Calibration, reload validation, configured descent, controlled descent, moving
pad, and stationary pad each completed, established physical `LANDED`, scored
100/100, and passed independent acceptance. Operator wait, hover-roll, and
AutoTune-roll failed before flight: their physical outcomes are unknown,
their raw scores are 0/100, and their artifacts are rejected. The operator
production writer rejected its registered wait status. Both roll hosts invoked
DroneKit's blocking complete-parameter wait when reading the cache.

Both competition cases failed before flight with
`DroneKit connection failed: source_identity must be explicit`. Their physical
outcomes and scores are unknown; artifact acceptance rejected both. Each retained
finalization/terminal-notification/runtime-failure-observation deadline
diagnostics. All 11 owned Compose projects were removed. The report, failed
bundles, and six accepted recordings remain in the shared
`/home/willis/projects/drone_sim/runs/local-ci/20261005-91d8cae/` directory.
The report is `suites/8504b189-776f-442f-8053-62635ce0b601/report.json` beneath it.

Two focused regressions reproduced the status-writer and parameter-cache bugs.
The fixes admit the companion-owned wait status and inspect DroneKit's current
cache without blocking. Their focused checks passed 84 tests. A full host sweep
passed 2,263 tests with 27 environment skips; one lock check failed while the live
suite held the workstation lock, then passed after teardown. Imported Comp2026
checks passed 1,126 tests. Compilation and edited local documentation links
passed. Physical reruns of the fixes remain pending.

Fresh suite `15ff4914-3356-42d9-aff9-0a241a3faa7c`, built from clean
`e113217`, failed its calibration gate on 2026-10-06 with
`AutoTune reserved landing window reached`. At public 540 seconds, native
AutoTune was still tuning yaw angle-P-up at step 5. A complete DataFlash MSG
scan contains neither native AutoTune failure nor success. Native LAND began
at native 629.942 seconds, disarm was recorded at native 642.367 seconds, and
the companion published `mission_failed` at public 553.05 seconds.

The final report records calibration lifecycle `FAILED`, raw score 0/100 and
rejected artifact acceptance. Its physical outcome remains unknown: native LAND
and disarm do not establish the independently accepted mission contract. The
failed calibration gate blocked the other ten cases before allocation. This
run provides no consumer flight evidence or suite pass. Its retained report is
`runs/local-ci/20261005-e113217/suites/15ff4914-3356-42d9-aff9-0a241a3faa7c/report.json`.
The owned suite process exited and its Compose project was removed.

The calibration template now allows 900 public seconds, with the same 60-second
landing reserve and 7,200-second wall limit. The reserve therefore starts at
public 840 seconds. Native tuning parameters, flight control, scoring and
acceptance are unchanged. A fresh frozen-build calibration and full suite
remain required to verify this timing change and the earlier runtime fixes.
Timing, recorder, configuration, suite and workflow checks passed 179 tests;
the full host sweep passed 2,264 tests with 27 environment skips. The recorder
contract now requires 18,000 calibration camera frames at 20 Hz. All 79 local
links in the edited guides resolve; external URLs and anchors were not checked.

The competition adapter needs a larger compatibility change. Identities alone
satisfy only the constructor: native missions also require pinned home,
FlightState/supervisor/output transactions, release/clearance configuration, and
an explicit precision policy. The hardware/QGC initializer requires deployment
artifacts absent from simulator configuration. Sensor geometry and physical
attachment confirmation exist, but required safety and precision policy values
are not defined. Do not bypass those guards, use ground truth as onboard
evidence, or treat test-only policies as deployment configuration. No common-build
full-suite pass is claimed.

The manual Actions workflow is implemented. Official runner v2.337.0 is
registered as `drone-sim-workstation` with label `drone-sim`. A temporary user
listener connected at 16:33 UTC and GitHub reports online. The persistent service
remains pending: noninteractive sudo is unavailable, so installation/start requires
the operator steps in the [runbook](runbook.md#manual-workstation-ci).
Actions dispatch remains unrun until the workflow exists on default branch
`main` and the runner service is online. No current end-to-end pass is claimed.

The new CI calibration implementation shares aircraft dynamics across the three
vehicle variants, tunes all axes and saves gains after native LAND. It adds
independent calibration acceptance and a fresh-SITL validation consumer.
Calibration and fresh-SITL validation both passed independent acceptance at
100/100. This completes the first calibration/reload milestone; the new suite wires the dependency,
but its current all-setup flight proof remains pending. The sweep below used the older
`33da957` profile; its passes do not establish compatibility with the new body or
gains. See the [calibration workflow](runbook.md#calibrate-and-validate-saved-gains).

Calibration attempt `840fc850-63b0-44f5-b124-af892e2d9b76`, built from clean
`e9272ca`, reached AUTOTUNE at public 9.35 seconds but disarmed at 14.45 seconds.
DataFlash shows neutral throttle expiring from 1500 to 1000 exactly three seconds
after its one-time override. The driver also failed to terminate on that disarm.
The run was stopped normally and preserved as ABORTED; validation never started.
It is not a calibration pass. Commit `304f966` refreshes neutral input and rejects
unexpected disarm; the following calibration verified that fix in flight.
An earlier startup-only failure,
`21d8ffe8-b2ac-409e-99c9-f4731386487a`, exposed recorder-geometry and diagnostic
world-speed mismatches; both contracts now have focused tests and fixes.

The next calibration, `51ed33d0-7747-42f7-82a2-3f2653d2585e` from clean
`4a092d3`, completed all three axes, native LAND and saved-gain export. Physical
scoring and independent artifact acceptance both passed at 100/100. Its first
validation consumer, `21f989f6-eb18-4257-9d42-f0c1a089c72e`, never armed: a burst
of 34 parameter-read requests exceeded ArduPilot's 20-entry request queue, and
missing replies blocked execution readiness. That consumer was preserved as
ABORTED. Commit `bfec619` replaces the burst with one complete-list request,
retaining the same strict required-value checks.

Validation retry `308969fb-c2ca-4225-959b-70c1a51ffa6b`, built from clean
`045e3f5`, reused the accepted source with byte-identical gain and manifest files.
SITL/Gazebo image identities, aircraft and base parameters stayed unchanged; only
the companion image changed. All 34 effective parameters matched before arming.
The five configured operations succeeded, including the 5 m takeoff, 10-second
hold and native LAND; observed landing/disarm completed at public 30.05 seconds.
Independent provenance, recordings, readback and five-second stable-hover checks
passed before acceptance rejected the final 80/100 score. Airborne/contact,
touchdown precision and safe preimpact speed passed; stable contact failed.
That retry failed acceptance despite CLI `COMPLETED`.

Bag replay isolates the failure to the first contact sample at public 27.450 s:
speed 0.1994064 m/s exceeds the stable-contact limit of 0.1 m/s. The remaining ten
samples through 27.950 s stay in contact at at most 0.001 m/s; maximum tilt across
all eleven samples is 0.004544 degrees. Preimpact descent at 27.400 s is
0.4997697 m/s, within the 1 m/s safety limit. The original
[descent policy](../scorekeeper/rules/descent_v1_legacy.json) starts its half-second
stability window at first contact, including that impact sample.
The vehicle settles immediately afterward; this evidence does not establish a
sustained oscillation or loss of contact.

Both bundles retain observer/onboard videos under `runs/RUN_ID/video/`, source
flight logs and immutable manifests. Their recorded acceptance results remain
unchanged. After reviewing the touchdown recording, the operator approved descent
settling-policy version 2: continuous contact and acceptable tilt must lead to a
qualifying stable interval before the configured deadline. Original version 1
rules remain available for historical replay. Full recorded-truth replay gives
80/100 under the original policy and 100/100 under version 2 in both runtime and
independent scorers. Its qualifying interval is 27.500–28.000 seconds. This is
derivative evidence; the historical bundle remains 80/100 and rejected.

Fresh validation `474b7b49-a814-4c7b-a7a5-aaa12a9e47d4`, built from clean
`9fb7c56`, passed independent acceptance at 100/100. It reused the accepted source
gains and manifest byte-for-byte, with identical SITL/Gazebo images and all 34
effective parameters verified before arm. All five operations succeeded; landing
and disarm completed at public 30.05 seconds. Physical touchdown was at 27.450
seconds, with preimpact downward speed 0.4990884 m/s, contact speed 0.0990265 m/s
and tilt 0.008252 degrees. Continuous settled contact qualified from 27.450 to
27.950 seconds. Independent acceptance also passed the required stable-hover
check during the 5 m, 10-second hold.

The new run records 1,200 public samples and both 60-second videos after its
90-second native warmup. CLI outcome is `COMPLETED`, physical score is 100/100,
and independent artifact acceptance is true. Its manifest SHA-256 is
`5ec36f73b6cd7e14c4e2af2879a03779bf581517ccba3cb501d0c93fbe9aebbe`.
All run resources were removed after finalization.

Focused companion verification after `bfec619`: 233 passed. Earlier integration
at `4414103`: 2,123 passed, 27 skipped; later startup-contract checks also passed.
Settling-policy verification at `9fb7c56`: 223 focused checks passed, covering
deadline boundaries, bounce, tilt, speed oscillation, hard impact, incomplete
settlement and legacy replay. No broad suite was repeated for this bounded change.
Other missions remain unrun with the new common airframe and gains. Full-suite
dependencies, Comp2026 consumption/packaging repair, CI-provider setup and run-time
optimization remain deferred; this accepted pair does not clear them.

The latest [scenario sweep](#scenario-regression-sweep) attempted all eight
unattended templates and controls from one frozen build. Controlled descent,
stationary-pad landing, and moving-pad landing passed independent acceptance.
The sweep is not fully accepted: three diagnostic runs expose an existing log
validation mismatch, AutoTune also scores 40/100, and both competition templates
fail before takeoff because required Comp2026 files are absent from their image.
Native ArduPilot LAND, camera, pad speed, and tracking guards remain unchanged.
Earlier failed experiments and the first accepted moving flight are preserved below.

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

### Lessons for future missions

- Native ArduPilot precision LAND can land on the 0.5 m/s pad with the current
  camera. The accepted EKF3 run did not need a companion descent controller.
- Check estimator input frames and timing before tuning gains. The pinned SIM
  attitude path returned body-frame delta velocity through a NED interface;
  the camera's measured 0.49676 m/s versus true 0.5 m/s was not a large error.
- An enabled moving-target option is insufficient when raw precision estimation
  supplies zero target velocity. Increasing position gain from 1 to 4 did not
  remove the observed following offset or produce touchdown.
- Require live parameter readback and preserve source, images, configuration,
  and flight logs together. Scope estimator changes to the moving scenario
  until the existing scenarios have fresh regression evidence.
- Physical landing, scoring, and artifact acceptance are separate results.
  Keep missing contact evidence fail-closed, and fix velocity-frame diagnostics
  separately without rewriting accepted recordings.

The full scenario sweep below records the subsequent results and coverage gaps.
Detailed evidence from the earlier landing experiments follows it.

### Scenario regression sweep

The 2026-09-28 UTC sweep rebuilt all seven images from clean
`33da9578ea696047ce55845884d870452a91111c` and attempted the seven automatic
templates plus the stationary control sequentially. Source, image digests,
configurations, and scoring rules stayed frozen through launch and acceptance.
All eight attempts have terminal bundles; no run-scoped Compose resources remain.

| Template / control | Run UUID | Physical outcome | Score | Artifact acceptance |
| --- | --- | --- | --- | --- |
| Configured descent | `5a569645-c0dd-4948-9639-b0b9b4e5d886` | Landing/disarm confirmed at 13.05 s | 100/100 | Failed: legacy log predicate |
| Controlled descent | `85e5a4bb-1154-436a-b8e5-5b744b78aff0` | Landing/disarm confirmed at 11.05 s | 100/100 | Passed |
| Hover/roll | `af6c2f6f-88a7-4bc4-becd-0491c013f85b` | Ten-second hover; landing/disarm confirmed at 38.00 s | 100/100 | Failed: legacy log predicate |
| Roll AutoTune | `08f47ef9-2095-4c95-8752-5735faf54d0e` | Tuned, saved gains, landed/disarmed; complete at 56.00 s | 40/100 | Failed: legacy log predicate; also below maximum score |
| Stationary pad | `f8524a02-4d46-43c1-a870-40b823321904` | Touchdown 27.35 s, 8.1 mm offset; disarm observed 30.05 s | 100/100 | Passed; full 90 s |
| Moving pad | `03853b04-7b91-4b9c-ad82-89da47ac8689` | Touchdown 59.10 s, 8.0 mm offset; disarm observed 61.10 s | 100/100 | Passed; full 90 s |
| Competition default | `42cd23b8-e06f-4a5b-8c7e-758a8c92659a` | Startup failed; no flight | 0/150, incomplete | Failed; incomplete bundle |
| Competition realtime | `da90025d-d83a-4205-a40e-33655d41bb9d` | Startup failed; no flight | 0/150, incomplete | Failed; incomplete bundle |

Both pad runs have 1,800 matching vehicle/pad truth samples and 1,800 frames in
each video. The moving pad stayed at 0.5 m/s within 1e-5 m/s through touchdown,
disarm, and the remaining recording window. The earlier stationary contact gap
did not recur; its cause remains unresolved. DataFlash confirms EKF3 only in the
pad scenarios. The four descent/hover/AutoTune runs retained the base SIM/raw
precision profile.

The three diagnostic acceptance failures are preexisting. The `descent_v1`
validator requires controlled-descent command and event logs, while configured,
hover, and AutoTune missions emit different events. Supplemental bag/video checks
and independent score recomputation passed without changing canonical acceptance.
AutoTune's 40/100 is real: touchdown was 16.88 m from the target with 1.052 m/s
preimpact speed, exceeding the 0.5 m and 1.0 m/s limits. It finished tuning and
saved gains, but failed landing quality and independent acceptance. No comparable
historical flight establishes a score regression.

Offline diagnosis on 2026-09-29 traced the AutoTune result to two mission choices:

- The [mission](../companion/src/drone_sim_companion/autotune.py) enters
  `AUTOTUNE` from `ALT_HOLD`. The
  [pinned ArduPilot implementation](https://github.com/ArduPilot/ardupilot/blob/1511f27194f1dcc3728270883047bdf022b3fd53/ArduCopter/mode_autotune.cpp#L24)
  enables tuning position hold only when entered from `LOITER` or `POSHOLD`.
  Recorded mode changes therefore select no position hold. Ground truth shows
  displacement growing from 0.003 m at 15.00 s to 12.20 m at tune success,
  48.30 s.
- Success triggers throttle override 1300 while retaining `AUTOTUNE` through
  landing/disarm to save gains. There is no centering step or native `LAND`
  approach. Touchdown follows at 53.95 s, 16.88 m away. Mission completion checks
  saved gains, disarm, and low altitude; it does not check touchdown position or
  descent speed.

The throttle override requests a nominal 0.9375 m/s descent with the recorded
RC calibration. DataFlash shows the corrected vertical target and actual descent
near 1.052 m/s; the native LAND speed setting of 0.5 m/s does not apply in
`AUTOTUNE`. The hover control enters `LAND` and descends near 0.5 m/s. Detailed
control timing and pinned-source references are in the diagnosis `REPORT.md`.

`COMPLETED` and the start command's zero exit code describe lifecycle completion,
not physical acceptance. The batch command checks that exit code. Calling this
AutoTune run a regression pass would be incorrect; the saved sweep result has
`accepted: false`. The separate log-validator mismatch must also be fixed before
this mission can pass canonical acceptance, even after improving its landing.

The ground-truth adapter also copies body-frame odometry velocity without rotating
it into world coordinates. That existing frame defect does not explain this
score: replaying all 2,400 samples with world-rotated velocity still gives 40/100.
Preimpact downward speed changes from 1.05162 to 1.05183 m/s; world-position
differences independently give 1.05169 m/s. Diagnosis scripts and outputs are
preserved in `.superpowers/sdd/autotune-diagnosis-2026-09-29/`. This was read-only
flight analysis; no runtime fix, gain promotion, or new flight was performed.

Both competition failures originate in [.dockerignore](../.dockerignore): it
excludes tracked control and LiDAR modules imported by `DroneControl`, first
failing on `drone.control.flight_state`. The mismatch already exists in the
monorepo import `23111e9c`; the moving branch did not change the affected files.
The fix needs the complete import closure, a companion rebuild, and an in-image
import check before repeating these flights. Finalization exhausted its budget;
the first failure also left a stopped Gazebo container and network. Their state
was backed up before scoped cleanup and resuming the last case.

Coverage gaps: `configured-operator-run.json` was not flown. Companion owns the
only single-client MAVLink TCP endpoint; the shipped topology has no independent
operator endpoint or router. Focused configured-runner/config checks reported
159 passed in 5.24 s, including waiting without sending arm/mode commands. That
terminal-only test observation has no saved log and is not flight evidence.
Historical search-and-deliver is absent from the current supported templates.
These gaps prevent a blanket claim of no regressions.

Evidence remains under `runs/RUN_ID/`. The ignored
`.superpowers/sdd/regression-2026-09-28/` directory contains `snapshot.json`,
`final-summary.json` with all manifest hashes, per-case diagnoses, startup-log
and container backups, and `session-20260928T190116Z-753316/` with launch and
canonical acceptance logs. Preserve these with the bundles. Runtime code and
acceptance policy were not changed to obtain the results.

### Earlier landing experiments

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

Current runtime tags point to the clean `33da957` regression build. All seven
images are preserved under `regression-33da957` tags; prelaunch expectations
are in `.superpowers/sdd/regression-2026-09-28/snapshot.json`.
The earlier `faef600` EKF3 images remain under `moving-pad-ekf3-faef600` tags,
with their original `provenance-stationary-ekf3.json` and
`provenance-moving-ekf3.json` snapshots. Earlier experiment images are preserved too.
Subsequent documentation commits do not change those recorded source identities.
Earlier images remain under preservation tags; never retag them as new evidence.
Rebuild after runtime edits and capture new expectations before the next flight.

Latest stationary and moving landings: achieved, each 100/100 and independently
accepted. Both competition attempts failed at startup; earlier accepted
competition flights remain historical evidence.

## Active priorities

1. Repair the Comp2026 Docker-context allowlist and verify imports in the built
   image, then rerun both competition templates. Preserve the failed bundles.
2. Reconcile diagnostic mission logs with independent acceptance, and define
   appropriate AutoTune scoring expectations without relabeling its 40/100.
3. Diagnose the intermittent private pad-contact gap exposed by the earlier stationary
   EKF3 control. Capture native and bridged timestamps plus tracker fault state
   before changing transport or join behavior. Keep missing evidence fail-closed.
4. Resolve the configured-operation deadline beyond the capped public window.
   At 90 s the source finishes, but the host waits for mission completion while
   the precision timeout at 104.45 s is unreachable. Keep artifact-grid diagnosis
   separate; do not weaken acceptance.
5. Correct the touchdown-velocity diagnostic's frame semantics separately from
   the physical landing rule; retain recorded evidence unchanged.

Deferred work includes an independent operator MAVLink endpoint, startup-failure
finalization/cleanup, and historical competition range-stream diagnosis.
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
