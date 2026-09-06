# Human handoff / current status

[Start here](../README.md) · [Architecture](architecture.md) · [Runbook](runbook.md)

Reviewed 2026-09-05 against parent `f4c86a8` and the working tree, with nested
mission HEAD `54cdeff`. This is a dated handoff, not a claim that those mutable
checkouts or local image tags will remain unchanged.

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
Subsequent source changes include sensor-readiness and native-clock handling;
they have **not** been flight-validated by this documentation task. No new flight
was launched and no runtime images were rebuilt for this handoff.

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

No image builds, host provisioning, or flight were performed. Skipped ROS cases
and source-only tests do not establish runtime correctness.

### Protocol-file consolidation verification, 2026-09-06

The corrected protocol helper suite passed **46 tests**. Runtime protocol and
host status-store tests passed **92 tests**, and shared contracts plus the Phase
3 runtime contract passed **21 tests**. `git diff --check` passed, and the
required removed-publication-symbol search returned no matches.

From original slice base `5900d0d`, the three production files add 539 lines and
remove 352, a net production increase of **187 lines**. This source-only
verification did not build runtime images or launch a flight.

## Open items, in recommended order

### 1. Make a fresh clone reproducible

The parent does not track `companion/comp2026` and has no submodule declaration.
The companion image requires its source and startup checks its Git revision.
Publishing an accessible pinned mission revision and choosing a submodule or
another explicit acquisition mechanism is still needed. The README's existing
packaging TODO is preserved. Do not invent a remote or bypass the label check.

Before that packaging work, the runbook's explicit revision build argument is
the supported manual path; plain Compose build uses a stale fallback label.
Relevant files: [Dockerfile](../companion/Dockerfile), [.dockerignore](../.dockerignore),
[Compose binding](../orchestration/src/orchestration/_adapters/compose.py).

### 2. Resolve the roll-parameter/test disagreement

The current overlay records promoted roll AutoTune gains from run
`881fe09d-08ef-4ca5-8304-fe79c5220e61`. It sets `ATC_RAT_RLL_P/I=0.0503722` and
`ATC_RAT_RLL_D=0.000375`; the test still expects the older `0.0675/0.0675/0.0018`.

Reproduce just that discrepancy:

```bash
uv run --locked pytest ardupilot_sitl/tests/test_config.py::test_descent_parameters_use_verified_competition_roll_rate_gains -q
```

Inspect the original tuning evidence and decide which baseline is intended
before changing either the [overlay](../ardupilot_sitl/params/descent.parm) or
[test](../ardupilot_sitl/tests/test_config.py). This cleanup changed neither.

### 3. Obtain a clean full-window competition baseline

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

### 4. Keep this documentation useful as code changes

[Contribution rules](../AGENTS.md) require updating every affected module README
and shared guide in the same change, with an explicit documentation-impact note.
Exact fields, settings, and QoS remain linked executable definitions, not duplicated
configuration tables. Documentation generators and broad refactoring are deferred.

## Existing work preserved

At the start of this cleanup, these were already outside the committed parent
state:

- `ardupilot_sitl/params/descent.parm`, and
  `ardupilot_sitl/tests/test_config.py`: final landing speed / precision-landing
  edits. The working overlay requests `LAND_SPD_MS=0.50`, with `PLND_OPTIONS=4`;
  do not confuse this with the parent's previously committed 0.10 m/s value.
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
