# Manual full-mission CI suite

Written 2026-10-05. The conversational design is approved; this written spec
awaits review. The suite command and GitHub workflow are not implemented.

## Goal and agreed scope

Catch regressions in every checked-in flight setup after a developer changes
the simulator or `companion/comp2026`. One command runs locally and from a
manually dispatched GitHub Actions job on this workstation. Each invocation
builds once, calibrates the unloaded aircraft, validates the saved gains in a
fresh process, then attempts every remaining setup with those same source gains.

Calibration and reload validation are gates. After both pass, a failed mission
does not prevent later independent missions from running. Any required failed,
blocked, or unrun setup prevents an overall pass. Report physical behavior,
score, and artifact acceptance separately. Process exit zero is insufficient.

This delivery includes the operator-wait setup, stationary control, diagnostics,
and both competition timing variants. Automatic triggers, parallel flights,
calibration caching, parameter promotion, hosted recording storage, and agentic
mission control are deferred.

## Shared command and catalog

The proposed commands are:

```bash
uv run --locked drone-sim suite --config config/ci-suite.json
uv run --locked drone-sim suite --config config/ci-suite.json \
  --output-root /absolute/persistent/run-directory
```

Both forms run the complete catalog. The default output root is the invoking
checkout's `runs/`. Actions uses an absolute directory outside its checkout.
These are proposed commands, not current operator instructions.

Keep the runner in [orchestration](../../../orchestration/README.md), beside the
existing [CLI](../../../orchestration/src/orchestration/cli.py) and
[RunController](../../../orchestration/src/orchestration/controller.py). Reuse
the existing run lifecycle, configuration resolution, independent acceptance,
and Compose teardown. The workflow only prepares the checkout/environment,
invokes the same command, and publishes its report.

The JSON catalog has `schema_version: 1` and an ordered `cases` array. Each case
has a unique `name`, repository-relative `template` path, and `acceptance` from
the fixed types below. This is not a new expression language. The first two
entries are the calibration gates. The `operator_wait` contract additionally
enables the bounded simulated operator.

| Order | Template | Acceptance contract |
| --- | --- | --- |
| 1 | `config/autotune-run.json` | `calibration` |
| 2 | `config/calibration-validation-run.json` | `reload_validation` |
| 3 | `config/configured-descent-run.json` | `configured_descent` |
| 4 | `config/configured-operator-run.json` | `operator_wait` |
| 5 | `config/vertical-descent-run.json` | `controlled_descent` |
| 6 | `config/hover-roll-run.json` | `hover_roll` |
| 7 | `config/autotune-roll-run.json` | `autotune_roll` |
| 8 | `config/configured-moving-pad-run.json` | `moving_pad` |
| 9 | `tests/fixtures/configured-stationary-pad-run.json` | `moving_pad` |
| 10 | `config/default-run.json` | `competition` |
| 11 | `config/realtime-run.json` | `competition` |

A focused coverage check compares this catalog with top-level `config/*.json`
objects declaring a flight's `world`, `vehicle`, and `mission`, plus the
stationary flight fixture. Schema files, the suite catalog, course data,
synthetic faults, and unit-test inputs are not additional flight setups. A new
flight template must receive a catalog entry and acceptance contract.

Preserve each template's mission plan, seed, recording contract, timing, and
scenario. Resolve temporary copies under the suite directory to substitute the
output root and calibration source. Do not edit tracked templates or defaults
during execution. Resolve template-relative inputs before relocating a copy.

## Execution and provenance

Require a clean committed source checkout. Acquire one workstation lock before
building, then use the existing documented seven-image Phase 3 build with the
checkout's exact monorepo revision. Record source identity, catalog/template
checksums, and immutable image IDs once. Run a focused in-image Comp2026 import
check before spending time on calibration.

Every flight uses a new run directory, isolated Compose project, fresh SITL
storage, and `--no-build`. Source and image identity must continue to match the
suite's frozen identity. Reuse per-run provenance records rather than inventing
a second provenance format. A changed checkout or runtime image blocks remaining
flights and fails the suite instead of producing evidence from mixed builds.

The all-axis gate requires independent 100/100 acceptance and the existing
verified saved-gain artifact. Freeze its parameter bytes and source-manifest
identity. The next gate loads that artifact into fresh SITL, verifies effective
parameters before flight, takes off to 5 m, holds 10 simulated seconds, and
lands. It requires 100/100 and the existing recorded stable-hover check.

If either gate fails, report the dependent setups as blocked. Otherwise attempt
the remaining nine sequentially. An individual mission or acceptance failure
is retained while the next independent case runs. Teardown must finish before
the next launch. A teardown failure that leaves shared resources active blocks
further scheduling and fails the suite.

## Calibration consumption across aircraft variants

Extend the existing checksum-bound import path to `controlled_descent`,
`hover_roll`, `autotune_roll`, and `comp2026_auto`, as well as `configured`.
Every consumer copies the same accepted source artifact and manifest, loads
base parameters then scenario overlays then gains, and verifies all required
effective values before execution readiness or an arming command.

Use the shared airframe definition from the
[asset generator](../../../gazebo/scripts/prepare_competition_assets.py) to
establish compatibility among `iris_flight`, `iris_moving_pad`, and
`iris_competition`. Bind the shared physical definition, sensor/controller
hardware, base parameters, firmware revision, and SITL/Gazebo image IDs. Also
bind each consumer's exact generated model and ordered overlay checksums to its
frozen configuration and recorded resources. Model names alone do not establish
compatibility. Declared payload attachments and scenario overlays are allowed;
an arbitrary aircraft or changed dynamics are not.

Consumer readback uses its effective overlay values for settings deliberately
changed by that scenario, not the calibration flight's base values. Accepted
gains remain the final parameter layer. Preserve the complete required-value
gate and float-aware comparison tolerances. Missing values or mismatches cannot
release execution readiness. The MAVLink adapter requests one complete parameter
list, retaining the existing protection against overflowing individual-request
queues.

The roll-only diagnostic deliberately seeds and retunes roll gains. Verify its
imported baseline before these declared diagnostic writes, record the writes,
and keep its saved result within that run. Every subsequent flight starts from
the original all-axis artifact. The diagnostic never replaces the suite's
calibration source.

## Acceptance contracts

All required cases need a completed lifecycle, independently valid provenance,
configuration and score replay, required native logs, both finalized videos,
valid bag evidence, and observed landing/disarm. A suite report cannot override
a failed manifest or mutate an old bundle. Frozen scoring rules remain the
authority for each run's raw score.

Use the existing [independent acceptance entry point](../../../artifacts/src/artifacts/acceptance.py),
[calibration artifact validator](../../../artifacts/src/artifacts/calibration.py),
and [versioned scoring rules](../../../scorekeeper/rules) for exact current
contracts. Extend the mission-specific checks there; the suite selects a
contract rather than reimplementing scoring.

Separate universal calibration-input checks from the fresh reload flight's
specific 5 m/10 s hover test. Merely consuming gains must not impose that flight
sequence or force maximum score on a diagnostic. Check parameter evidence before
the first internal or external arming event, using the applicable mission's
evidence rather than assuming configured `arm` operations exist everywhere.

| Cases | Additional required behavior |
| --- | --- |
| All-axis calibration | 100/100 under `calibration_v1`; all axes complete; activation, native landing, native save, artifact and live/DataFlash readback agree. |
| Fresh reload validation | 100/100 under `descent_v1`; source copies and pre-arm effective readback match; the recorded validation hold contains the required stable interval. |
| Configured, operator-wait and controlled descent | 100/100 under `descent_v1`; ordered mission-specific commands/operations and observed landing/disarm. |
| Hover diagnostic | Existing 10 s ALT_HOLD behavior, a recorded stable hover interval, native LAND, terminal hover phase, and the required safe physical landing predicates. |
| Roll-only tuning diagnostic | Ordered tuning success and roll saved-gain evidence, terminal tuning phase, valid saved roll artifact matched to DataFlash, and the required safe physical landing predicates. |
| Moving and stationary deck | 100/100 under `moving_pad_v1`; recorded touchdown, disarm and continued deck support. |
| Both competition timings | 150/150 under `competition_v1`; independently verified payload pickup/delivery outcomes and home landing. |

The reload stable-hover predicate already requires five continuous seconds of
50 ms ground-truth samples during its actual hold. It checks no contact,
altitude within 0.5 m of 5 m above initial height, world horizontal and vertical
speeds no greater than 0.2 m/s, and absolute roll/pitch no greater than 5 degrees.
Use the same physical predicate within the hover diagnostic's actual ALT_HOLD
interval. An elapsed timer alone does not prove stable hover.

For both diagnostics, independently replay their frozen `descent_v1` score and
require the `airborne_then_contact`, `safe_preimpact_speed`, and `stable_contact`
rules to pass. Do not require the origin-radius points because neither
diagnostic commands a return to that XY target. Raw scores remain visible.
The existing settling-policy version and thresholds remain unchanged, including
the impact-speed limit. The historical roll-only 40/100 run remains a failure
because it missed safe landing criteria, not just the XY target.

Validate diagnostic logs using their actual `hover_phase` or `autotune_phase`
and mode/save events. Preserve the stricter controlled-descent grammar and
configured-plan matching. Validate the roll artifact against the existing
`ROLL_GAIN_PARAMETERS` allowlist and parser, including one coherent native save.
Do not require five-second stable hover during AutoTune oscillations.

If a fresh diagnostic exposes an unsafe landing, its result stays failed.
Any required landing fix must preserve tuning success, gain activation/save,
and real touchdown/disarm evidence; lowering safety thresholds is not a fix.

## Exercise external operator waiting

Keep the operator plan's leading `wait_for_state(armed=true, mode=GUIDED)`.
Do not replace it with internal mode/arm operations or exempt this setup.

The companion retains its primary MAVLink TCP endpoint on 5760. Add a separate
native SITL SERIAL1 MAVLink TCP endpoint on 5762 inside the private Compose
network. Use the pinned firmware's serial support, preserving GPS endpoints.
No MAVLink router or host-public port is needed.

A small operator actor reuses the existing companion image and runs only for
this case. It waits for run-scoped RUNNING, verified mission execution readiness,
and the started wait operation. It then observes fresh telemetry on its own
connection, requests GUIDED, confirms acceptance and observed mode, requests
normal arming, and confirms acceptance and observed armed state. It becomes
passive afterward. The companion independently observes the state change and
owns takeoff, hold and landing through its existing connection.

Retain run-correlated operator command, acknowledgement, and observed-state
evidence in finalized checksummed logs before the run manifest commits. Use
existing run deadlines, normal prearm checks and teardown. A missing operator,
failed command, or premature command fails the case. Runtime verification must
demonstrate both connections together; source inspection alone is insufficient.

## Reports, retention and interruption

Under the selected output root, keep each existing `RUN_ID/` bundle intact.
Store the suite index at `suites/SUITE_ID/report.json` and render its human view
as `report.md`. Thus the default paths are
`runs/suites/SUITE_ID/report.json` and `runs/RUN_ID/`. Link bundles rather than
copying videos or bags into the suite directory.

The index records the suite identity and frozen provenance, calibration and
reload identities, start/finish state, and each catalog entry's run ID, lifecycle
result/reason, physical outcome, raw score, independent acceptance, teardown
diagnostics, and evidence paths. Use explicit passed, failed, blocked and unrun
case statuses. Unknown physical facts remain unknown. Update the index after
each case so a partial result survives interruption.

Retain complete and partial local bundles, including videos, bag, DataFlash and
logs. Upload reports, suite console output, and compact manifest/acceptance/log
diagnostics to GitHub. Full recordings stay on this workstation. The report
includes their local location; a workstation path is not a downloadable GitHub
video link. Automatic pruning is deferred.

Local suites and the Actions runner use the same workstation account and common
advisory lock, independent of checkout and output root. A second suite fails
promptly while that lock is held. Ordinary single-run commands remain available;
the runbook instructs operators to avoid concurrent flights or image rebuilds
during a suite. Preserve current run-scoped lifecycle isolation.

On SIGINT/SIGTERM, stop scheduling, request abort for the active run through the
existing controller, allow bounded finalization/teardown, persist an interrupted
report, and return a nonzero interrupted exit. Do not delete run evidence or
stop unrelated Compose projects. Normal overall exit is zero only when all 11
required cases pass. Mission/gate failure, setup failure and interruption remain
distinguishable in the report and CLI result.

## Workstation GitHub Actions flow

Add one `workflow_dispatch` workflow using the workstation's self-hosted runner.
No push, pull-request or scheduled triggers are enabled. Use fixed concurrency
with cancellation of a running suite disabled, plus the CLI's workstation lock.
The job checks out the selected revision, uses the locked Python environment,
invokes the complete suite, and publishes summary/diagnostics even when it fails.
Set the job timeout above the sum of catalog run deadlines plus bounded build
and report-upload allowances.

Register and configure the runner during implementation. Record installation,
service/account details, persistent evidence location, audit date and verification
in `/home/willis/SETUP_REPLICATION.md`, without secrets. The local command can
be demonstrated before the workflow is merged. GitHub's manual workflow becomes
available once its definition is on the default branch.

Official provider references are
[self-hosted runners](https://docs.github.com/en/actions/concepts/runners/self-hosted-runners),
[manual workflow dispatch](https://docs.github.com/en/actions/how-tos/manage-workflow-runs/manually-run-a-workflow),
and [workflow concurrency](https://docs.github.com/en/actions/reference/workflows-and-actions/workflow-syntax).

## Required implementation repairs and verification

Current reusable calibration and reload flight evidence is recorded in
[handoff](../../handoff.md). It does not prove the new full suite. The critical
path includes fixing Comp2026's Docker import closure, extending calibrated
consumers/profile compatibility, separating import acceptance from reload-only
hover acceptance, correcting diagnostic evidence grammar, and providing the
operator endpoint/actor. Keep these changes scoped to running this catalog.

Use focused verification for catalog coverage and runner sequencing, calibrated
consumer mismatch gates, diagnostic acceptance, operator readiness/order, report
results and interruption. Include failure cases proving a failed gate blocks
consumers and an individual case failure does not skip later cases. The image
import smoke check must precede an expensive flight.

Then build from clean committed runtime source and execute one complete fresh
11-setup sweep. Freeze source/images throughout it and independently accept
each bundle. Retain any failed sweep and its report. A failed case cannot be
described as regression-free; any necessary fix needs a new matching build and
fresh evidence for affected behavior. An Actions dispatch must exercise the same
command and publish its report.

Update affected orchestration, artifacts, companion and SITL READMEs when their
interfaces change; update architecture, runbook and dated handoff in that same
implementation. Add Gazebo/scorekeeper documentation only if their contracts
actually change. Keep executable schemas/configuration authoritative, and
remove a superseded implementation plan from active docs after delivery.
