# Human handoff / current status

[Start here](../README.md) · [Architecture](architecture.md) · [Runbook](runbook.md)

Reviewed 2026-09-08 against the dated evidence below and code through `dcd43cc`,
plus the explicit companion revision guard described here. This is not a claim
that mutable checkouts or local image tags will remain unchanged.

## What a new maintainer should do first

1. Read the [architecture map](architecture.md), especially the distinction
   between mission intent, physical truth, scoring, and artifact validation.
2. Run the cheap checks in the [runbook](runbook.md). Confirm the separate mission
   checkout before spending time building images or launching a flight.
3. Read the [payload timestamp issue note](payload-timestamp-fix.md) and review
   its video/evidence links. They show both the completed flight and the later
   failed run, without conflating those results.
4. Choose one open item below. Keep its source/image revisions and evidence
   together; avoid tuning, scorer changes, and recording changes in one experiment.

The human entry point is now the root README. You do not need to read every
historical plan, explore every worktree, or run an hour-long simulation first.

## Verified behavior and limits

| Evidence | What it establishes | What it does not establish |
| --- | --- | --- |
| [Historical competition MVP](verification/comp2026-mvp.md), run `3dc895c3-a5aa-4651-a893-d21884fa43f5` | A documented earlier 600-second, 150/150 `COMPLETED` baseline with pinned revisions | That today's checkout/images reproduce it; the bundle lives outside this repository |
| [Payload fix verification](payload-timestamp-fix.md), run `259863d9-c558-4102-ae34-fe6c31f5cf94` | Third payload physically lifted about 10.11 m, released and settled inside the drop zone; home contact at zero speed at 261.2 s | A passing full-window run: final state was `FAILED` despite 150/150, with a later downward-range timestamp fault and invalid bag grid |
| Earlier handoff host suite (before this cleanup) | 1,376 passed, 26 skipped, 1 failed across the seven modules and ROS schema contracts | Live ROS/Gazebo flight correctness; skipped ROS cases did not run |

The successful physical flight used parent `fd1a5d4` plus the queue/landing
patches, nested mission `7e45d51`, and pinned images listed in the issue note.
Subsequent source changes include sensor-readiness, native-clock handling, and
the typed runtime status consolidation below. They have **not** been
flight-validated by this documentation task. No new flight was launched.

Recordings under `runs/` and the absolute external paths in old verification
notes are machine-local, ignored data. A recipient of a Git clone will not have
them. Transfer the desired evidence bundle separately, preserving its manifest,
checksums, configuration, and provenance.

### Documentation cleanup verification

The cleanup changes documentation and a stale recording-contract test, not
runtime behavior. That test now reads the deployed artifacts QoS file and checks
the metadata-only bag inventory while retaining reliable camera-transport checks.
The superseded root config copy was unused at runtime.

Final focused verification in the main working directory: all Phase 2 runtime
contracts, bag adapter, and schema contracts: **81 passed, 4 skipped**.
All **234 local links across 19 documents** and the timing values in **five
resolved run templates** checked successfully. The GPU Compose overlay resolves
to seven services; the contract tests also validate base/Phase 2 Compose.
All 105 deletion targets are absent, all 271 protected file hashes match the
pre-cleanup snapshot, and nested mission revision/status are unchanged.
Module documentation checks also exercised
orchestration's lightweight tests (**126 passed**), artifacts (**168 passed,
4 skipped**), and electromagnet/scorekeeper (**114 passed**). Broader orchestration
tests in the isolated worktree encountered **44 environment failures** due to
unavailable nested source-revision provenance; this is not a clean full-suite pass.
The earlier broader host suite recorded the roll-gain mismatch below.

No image builds, host provisioning, or flight were performed for that cleanup.
Skipped ROS cases and source-only tests do not establish runtime correctness.

### Explicit companion revision guard, 2026-09-08

Task 8 removed the historical Comp2026 revision fallback. With
`SIM_COMP2026_REVISION` absent, base, Phase 2, Phase 3, and GPU Phase 3 Compose
configuration all resolved. Phase 3 exposed an empty companion build argument;
a 40-character explicit value passed through unchanged. The companion
Dockerfile's first build step rejected an empty value with exit 2 and accepted a
nonempty value. That step precedes package installation and every source copy.

The focused RED run failed the two new behaviors and passed the two existing
profile checks. After the change, all four selected tests passed. The requested
three-file integration run reported **26 passed, 4 failed** because this
worktree still lacks `companion/comp2026`: Phase 2 launch failed with the known
`source_revision_unavailable` result and three later bundle checks depended on
that launch. No checkout was inserted to hide the limitation. No image was
built or retagged, and no flight, score, or artifact-validity claim was made.

### Typed runtime status verification, 2026-09-08

Deletion and stale-API audits found no duplicate runtime validators, legacy
Gazebo/scorekeeper writers, or string-selected status calls. The sole remaining
ArduPilot `atomic_document` call writes its private `work/failure.json`
child-process diagnostic. Manual review found typed status use in multiline
calls. Tests exercise all 14 registered types through both adapters, exact-type
selection, first-wins failure, false source completion, malformed runtime
freeze, and all four artifact-final corruptions on production controller paths.

At `1fba1c5`, the seven module suites reported: artifacts **896 passed, 12 skipped**;
orchestration **270 passed**; companion **110 passed**; Gazebo **330 passed, 14
skipped**; electromagnet **43 passed**; scorekeeper **69 passed**; and ArduPilot
**33 passed, 1 failed**. That ArduPilot failure is the byte-unchanged pre-existing
disagreement between `descent.parm`'s `ATC_RAT_RLL_P=0.0503722` and the test's
`0.0675` expectation. The combined synthetic, contract, and Phase 2/3 runtime
suite passed **50 tests**. Compilation and the locked dependency check passed.
Scorekeeper's standalone source self-test, when pointed at the checkout's rules,
completed its real typed status write and then stopped at the separate ROS check
because host `rclpy` is unavailable.

Final provenance review resolved that disagreement as a stale test. Commit
`fe44a96` promoted the complete five-value roll AutoTune set from run
`881fe09d-08ef-4ca5-8304-fe79c5220e61` but did not update the older expectation.
The manifest validates `ardupilot_sitl/autotune-roll.parm` at SHA-256
`9126cb1b656dcc5055e992d32b78d8df23f4cea01a737816ad947546257784a5`,
and the artifact's five values match the deployed overlay. Direct decoding of
the independent DataFlash BIN found the same five values together in the latest
coherent save epoch at timestamp `103968189`. Applying the artifact writer's
formatting to those decoded values produces the checked artifact values. The BIN
itself is not manifest-hashed. This validates the parameter evidence only: the
run ended `FAILED` after an unrelated clock stall, its parent source was dirty,
and it produced no valid score or current flight baseline. The old test
reproduced RED with **1 failed**. The corrected five-value contract passed **1
test**, all ArduPilot tests passed **34 tests**, and the AutoTune promotion check
passed **31 tests**. With the independent mission checkout exposed through a
temporary exact symlink and excluded from collection, the full parent suite
passed **1,819 tests** with **26 skipped**. No runtime parameters, production
source, images, or run evidence changed.

Task 7a then changed only `runtime_status.py` at `5a0d820`. Its final-head
focused status/protocol suite passed **406 tests**, its contract and integration
suite passed **34 tests**, and `artifacts/src` compiled. These focused checks
cover the behavior-preserving wire-field refactor; they are not a claim that the
earlier seven full module suites ran again at `5a0d820`.

Final review found and fixed a missed typed-status conversion in the Phase 2
synthetic Gazebo durable RUNNING fallback at `f92bb13`. The regression test
failed RED with **1 failed, 15 passed**, then passed GREEN with **16 passed**.
The combined Phase 2 and integration check passed **28 tests**. A probe inside
the rebuilt synthetic Gazebo image used the real status class and printed
`ready=True running=True`.

This worktree has no `companion/comp2026`. Initial source runs therefore
reported artifacts **895 passed, 12 skipped, 1 failed**, orchestration **224
passed, 46 failed**, and companion **109 passed, 1 failed** from unavailable
mission provenance/imports. Those suites were rerun using an exact temporary
symlink to external checkout `54cdeff`, which had **33 tracked changes and 35
untracked entries**. The link was removed afterward. The external dirty checkout
was neither modified nor staged and cannot prove a current companion image.

Base, Phase 2, and Phase 3 GPU Compose resolution passed. At shared-contract
revision `5a0d820`, the Phase 2 build named all seven images and the selective
Phase 3 build named all six available services. The known-doomed aggregate
Phase 3 build was not repeated. Its earlier `1fba1c5` attempt failed because the
companion Dockerfile could not find `companion/comp2026/src`.

| Tag | Before | Final-head ID | Created |
| --- | --- | --- | --- |
| `drone-sim-orchestration-runtime:phase2` | `1a6f2b6b` | `a8d21238d8956a160cd241f5133382989f71ba35292e0f6b58728fcb147c72fb` | 2026-09-08 08:26:53 +02:00 |
| `drone-sim-artifacts-runtime:phase2` | `6f75ac49` | `c00b9574d35109b0efa6fa726ff858bf4407ed6f77db3745f19e57fed1cfff48` | 2026-09-08 08:26:52 +02:00 |
| `drone-sim-synthetic-companion:phase2` | `45759617` | `ad11de5e5da52d83e35e7196fad3bc24db1d6aedeff582d60142d8e674f5a2d2` | 2026-09-08 08:26:39 +02:00 |
| `drone-sim-synthetic-ardupilot-sitl:phase2` | `687ff399` | `362a4604af62b54d7dcd212b50d89cd3a28ee2f8833e7ad33c2da6da2e57842d` | 2026-09-08 08:26:39 +02:00 |
| `drone-sim-synthetic-gazebo:phase2` | `d1ac7040` | `b95035896c525a031573a2b7227fd5087604e49b93ac0c74d498072550db5f4c` | 2026-09-08T08:59:32.589974385+02:00 |
| `drone-sim-synthetic-electromagnet:phase2` | `c4ebef28` | `8567b2cebdcbb582bf5edfed8481d82d0c32cef8271d22ec6a36e2d513008c83` | 2026-09-08 08:26:39 +02:00 |
| `drone-sim-synthetic-scorekeeper:phase2` | `69e0bba5` | `ab57293cfc0e409639d7c694cbefbd7ea386f4554cb30f9004d5c0759d38093d` | 2026-09-08 08:26:39 +02:00 |
| `drone-sim-companion-runtime:phase3` | `dbe2737a` | `dbe2737a708d75809b4bb628317544c22a57d16f6e2eb63bc74ab6961144cc7d` (stale) | 2026-09-03 23:07:23 +02:00 |
| `drone-sim-ardupilot-runtime:phase3` | `0df18663` | `e7fda4bf156fba551d6134519058ed91f743c216b54eb0b8d3ef9c23c962fe75` | 2026-09-08 08:29:01 +02:00 |
| `drone-sim-gazebo-runtime:phase3` | `a4d8b04f` | `7db4e3c88c99420460f75158d7cd755cafa7bcf57d3861d4135c88ed624674a3` | 2026-09-08 08:28:34 +02:00 |
| `drone-sim-electromagnet-runtime:phase3` | `d12cc27f` | `f12180b5f2f569b01815926e1633040956d9e75888ccf72be4061db36e88e556` | 2026-09-08 08:27:38 +02:00 |
| `drone-sim-scorekeeper-runtime:phase3` | `d35d9743` | `36fd9a5538a0b2ed24432709b791d5a18c92a13f92843b723116c2a3a2a474dd` | 2026-09-08 08:27:23 +02:00 |

The shared-contract rebuild at `5a0d820` rebuilt **11 of 12** required tags and
passed contract imports in seven image families: orchestration, artifacts,
synthetic companion, ArduPilot, Gazebo, electromagnet, and scorekeeper. The later
`f92bb13` rebuild replaced only the Phase 2 synthetic Gazebo image shown above;
the other table entries remain evidence from the shared-contract rebuild.
Because this worktree lacks `companion/comp2026`, the Phase 3 companion tag is
**stale, unbuilt, and unrun**. The required image gate remains incomplete.

Typed `GazeboReadyStatus` requires a real `FlightExchange`. The passive
`phase3_foundation` world has none and is rejected before Gazebo server startup.
Restoring that operator workflow requires a separate contract decision. The
runbook was reviewed and remains unchanged because no supported procedure
changed.

Against baseline `9731801`, final-head production Python is **+1000/-1014, net
-14**. The approved negative-net gate passes. These checks are source,
configuration, build, and import evidence only; no live mission, score, or
artifact result was produced.

## Open items, in recommended order

### 1. Make a fresh clone reproducible

The parent does not track `companion/comp2026` and has no submodule declaration.
The companion image requires its source and startup checks its Git revision.
Publishing an accessible pinned mission revision and choosing a submodule or
another explicit acquisition mechanism is still needed. The README's existing
packaging TODO is preserved. Do not invent a remote or bypass the label check.

Before that packaging work, the runbook's explicit revision build argument is
the supported manual path. Plain Phase 3 Compose build no longer substitutes a
stale revision; the companion Dockerfile rejects the empty build argument before
package installation or source copies. Compose configuration and selective
noncompanion workflows can still resolve without the variable.
Relevant files: [Dockerfile](../companion/Dockerfile), [.dockerignore](../.dockerignore),
[Compose binding](../orchestration/src/orchestration/_adapters/compose.py).

### 2. Obtain a clean full-window competition baseline

The payload-joint timestamp queue fault has a deterministic regression test and
a successful physical mission rerun. The later **range** fault is a different
stream. Its historical lost-sample path has not been proven; do not assume that
enlarging one more queue is sufficient or that new clock code has resolved it.

Start with the [issue note](payload-timestamp-fix.md), the
[adapter](../gazebo/src/drone_sim_gazebo/ros_adapter/node.py),
[competition bridge](../gazebo/config/bridge-competition.yaml), and bag validator.
Reproduce with diagnostic evidence before changing behavior. Then verify the
entire public window, normal video finalization, bag validity, and scoring on
one pinned source/image set. Keep strict timestamp/physical checks intact.

### 3. Keep this documentation useful as code changes

[Contribution rules](../AGENTS.md) require updating every affected module README
and shared guide in the same change, with an explicit documentation-impact note.
Exact fields, settings, and QoS remain linked executable definitions, not duplicated
configuration tables. Documentation generators and broad refactoring are deferred.

## Existing work preserved

At the start of this cleanup, these were already outside the committed parent
state:

- `companion/comp2026/`: independent repository at `54cdeff`, with untracked
  documentation inside it. Parent Git status showing this directory as untracked
  is not permission to add or delete the entire nested repository.
- Root `README.md` and `SYSTEM_DIAGRAM.md`: existing untracked documentation.
  The README was expanded in place; the diagram was backed up before deletion.

Use `git status --short`, `git worktree list`, and the nested repository's own
status before merging or cleaning. The detached
`.worktrees/joint-backlog-verification` checkout contains the pinned mission,
uncommitted patch copies, and a local run config. It and unrelated worktrees
were deliberately left in place. Their presence does not identify the current
authoritative implementation; the reviewed parent checkout and explicit run
provenance do.

## Documentation and recovery

[README](../README.md) is the human entry point; [AGENTS](../AGENTS.md) defines
contribution rules. The [architecture module map](architecture.md#where-to-read-or-change-code)
links all seven local guides. This handoff and the runbook cover status and
operations; [verification notes](verification) and the
[payload issue/solution](payload-timestamp-fix.md) preserve dated evidence.
The [earlier hardening ledger](technical-debt/vertical-slice-hardening.md) is
historical, not an active task list.

Superseded interface pairs, per-directory plans, the old roadmap, diagram dump,
and agent reports were removed from the active tree. Tracked originals remain
in Git history. Dirty, untracked, and ignored originals were additionally saved
and SHA-256 verified in the machine-local recovery archive
`/home/willis/drone-sim-doc-backup.QU0qLT/originals.tar.gz`, with paths/hashes in
`inventory.json`. Recover selected files into a temporary directory first;
do not overwrite current work by extracting the whole archive into the project.
The approved cleanup design/plan is recorded in commit `474b512` on
`docs/documentation-cleanup`, rather than remaining another live documentation layer.

Run evidence, licenses, the nested mission repository, and other worktrees were
preserved. A clone will not contain the local recovery archive or run bundles.
