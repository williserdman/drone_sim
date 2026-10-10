# Orchestration

[Project README](../README.md) · [Architecture](../docs/architecture.md) ·
[Runbook](../docs/runbook.md)

Orchestration owns the host-side run lifecycle. It resolves an operator template,
assigns the run ID, records immutable configuration and provenance, starts the
selected Docker Compose topology, observes readiness and completion, coordinates
bounded finalization, commits the terminal manifest, and tears the topology down.

It does **not** own simulation time, physics, flight control, mission policy,
payload behavior, scoring calculations, or media encoding. Those modules expose
readiness, runtime, completion, failure, and quiescence facts that orchestration
validates; it does not infer physical success from a command or log message.

## Entry points and code map

- The installed `drone-sim` command maps to [`cli.main`](src/orchestration/cli.py)
  through [`pyproject.toml`](pyproject.toml). Its operator commands are `start`,
  `status`, `abort`, `collect-results`, and `suite`. `console` hosts the local
  browser interface; `prepare`, `start-mission`, `abort-flight`, and `stop` call
  that same persistent operator boundary.
- [`RunController`](src/orchestration/controller.py) owns run allocation, Compose
  supervision, deadlines, terminal-cause selection, log capture, and manifest
  commit.
- [`resolve_run_config`](src/orchestration/config.py) is the template-to-resolved-
  configuration boundary; [`ComposeRuntime`](src/orchestration/_adapters/compose.py)
  is the Docker boundary.
- [`qgc_actions.py`](src/orchestration/qgc_actions.py) allocates durable
  run-bound QGC attempt tokens, renders the two native Fly View actions, and
  publishes a run-named file without replacing a different existing file.
- [`RunLifecycle`](src/orchestration/lifecycle.py) defines valid state transitions.
  [`runtime_node.py`](src/orchestration/runtime_node.py) publishes runtime lifecycle
  state and aggregates the quiescence barrier.
- [`StatusStore`](src/orchestration/status_store.py) owns host allocation and is
  the host-side adapter for the durable `.control/` and `.status/` protocol.
  The shared [`runtime_status.py`](../artifacts/src/artifacts/runtime_status.py)
  contract owns runtime status types, JSON conversion, and write policy;
  [`artifacts.protocol_files`](../artifacts/src/artifacts/protocol_files.py) owns
  low-level safe persistence.
  `StatusStore.validated_manifest_result` accepts paths defined by
  [`artifacts.manifest.is_manifest_relative_path`](../artifacts/src/artifacts/manifest.py);
  the [`manifest.json` schema](../artifacts/schemas/manifest.schema.json) is the wire authority.

[`suite.py`](src/orchestration/suite.py) owns the complete catalog, temporary
input snapshots, sequential cases and reports. [`_adapters/suite.py`](src/orchestration/_adapters/suite.py)
owns the account-wide workstation lock, bounded one-time build, frozen
provenance and independent acceptance. Calibration and reload are prerequisites;
remaining failures continue unless source/image identity or teardown is uncertain.
Template preparation errors fail the affected case before run allocation. A failed
calibration or reload preparation still blocks dependents; other preparation
failures leave later independent cases eligible to run.
The report records each allocated run immediately with transient `running`
status, then its terminal result after finalization. Unknown physical evidence
stays null when the independent inspector cannot establish acceptance.
The executable catalog is [`ci-suite.json`](../config/ci-suite.json). Reports and
recordings follow the [full-suite workflow](../docs/runbook.md#run-the-full-mission-suite).

## Consumer and producer seams

For local operator sessions, [`operator_session.py`](src/orchestration/operator_session.py)
keeps the controller alive independently of browser connections. The approved
[`catalog.py`](src/orchestration/catalog.py) reads complete CI preset tuples;
[`console.py`](src/orchestration/console.py) exposes loopback-only JSON actions,
static UI, and bounded camera proxy connections. Mutations require a local host,
local origin, and per-server token. The CLI uses
[`console_client.py`](src/orchestration/console_client.py), not a second controller.

Host lifecycle adds `PREPARED` and `STARTING_MISSION`. Preparation requires fresh,
advancing Gazebo observations and fresh disarmed companion preflight. The
controller waits for an immutable admitted native epoch before starting the
mission wall budget. Start retries with the same identity return the existing
request; competing identities fail. Run configuration stays immutable and the
admission checksum becomes a separate manifest configuration record.

[`workstation.py`](src/orchestration/workstation.py) shares the workstation lease
across direct launches, suites, and console workers; controller calls nested
inside a suite reuse its lease. Prerequisites and image provenance are checked by
[`local_runtime.py`](src/orchestration/local_runtime.py). Failed cleanup or an
orphaned owner blocks another preparation. An interrupted owner can be cleaned
up while preserving evidence, without inventing a completed manifest.

QGC token allocation uses a separate monotonic ledger under
`~/Library/Application Support/drone-sim/qgc/`; callers inject that directory
for tests. Prepare owns the workstation lease while it calls the allocator. The
ledger persists the next generation before action rendering, so failed Prepare
cannot reuse the token. A durable initialization marker distinguishes first use
from lost state. Missing, corrupt, symlinked, or exhausted state fails Prepare
instead of resetting the counter. Publication takes an explicit QGC
`MavlinkActions` directory, accepts an existing byte-identical run file, and
never overwrites a conflicting file or another user action file.

[`gcs_bridge.py`](src/orchestration/gcs_bridge.py) forwards the dedicated SITL GCS
MAVLink connection to native QGC, preserves wire frames, and inventories manual
control commands under `logs/docker/`. It originates no mission commands and
stops forwarding commands once finalization is requested. The companion still
owns automatic flight and guarded recovery. Recovery facts do not imply runtime
shutdown; the separate stop action invokes existing finalization.

The [local console workflow](../docs/runbook.md#local-mac-console) defines the
operator commands, port and spool requirements, and view semantics.

The CLI consumes templates such as [`config/default-run.json`](../config/default-run.json).
The resolved configuration selects one immutable service ownership topology; the
actual service definitions and container commands live in [`compose.yaml`](../compose.yaml).
GPU operation is an optional deployment workflow documented in the
[runbook](../docs/runbook.md#optional-nvidia-path).

The Phase 3 `configured` selector requires an inline `mission_plan`. The
[template schema](../config/run-template.schema.json) defines its envelope;
resolution normalizes step timeouts and binds the entire plan to `config_sha256`.
The resolved configuration stores the plan as immutable canonical JSON and
persists it in `configuration/run.json`. Exact tool names and arguments are
validated by the companion. Examples are
[automatic descent](../config/configured-descent-run.json) and
[operator waiting](../config/configured-operator-run.json).

Every resolved run also freezes `launch_mode` and
`preparation_wall_seconds`; the defaults are `unattended` and 1800 wall
seconds. Operator mode keeps simulation live and disarmed during preparation,
then waits for mission/prearm readiness, an immutable start request, and Gazebo's
request-scoped admission within one preparation wall-time budget. Process and
transport readiness retain the separate startup budget. The admission digest is
published in `RunState` from `RUNNING` onward so each runtime producer can reject
a different or missing decision. Unattended mode retains the fixed-epoch startup
path and publishes an empty admission digest.

When operator preparation supplies an action factory, configuration resolution
allocates the run ID first, then allocates and hashes the QGC action bytes before
calculating `config_sha256`. `operator_control` binds commands 31020 and 31021,
the token, request identities, MAVLink identities, relative bundle path, and
digest. The nonserialized `operator_actions` field retains the complete host
publication material, including the run ID, derived run-named filename, and
exact bytes. `write_resolved_config` checks that material against the descriptor
and writes the unchanged bytes to `configuration/qgc-actions.json`. Templates
cannot provide either runtime value. Descriptor-absent operator and unattended
configurations retain their historical serialization.

An `autotune` run receives a host-generated `calibration_profile` in its resolved
configuration before Compose starts. The version-2 profile binds the shared physics of `iris_flight`,
`iris_moving_pad`, and `iris_competition`, each exact model digest, the base
parameter file, ArduPilot revision and immutable Gazebo/SITL image IDs. The
configured, descent, hover, roll and competition consumers may declare
`calibration.source_run_directory`; relative paths resolve from the template.
Before allocating the new run, orchestration independently accepts the source at
maximum score, checks that profile against the current runtime, and freezes the
15 gains plus preserved baseline readback. It then copies the exact source bytes
to `configuration/calibration.parm` and
`configuration/calibration-manifest.json`. See the
[calibration validation template](../config/calibration-validation-run.json);
replace `SOURCE_RUN_ID` with an accepted `autotune` run ID before launch.
The explicit `calibration_validation: true` role selects reload hover checks;
ordinary calibrated missions retain their own contracts. Imported profiles bind
the consumer model, ordered scenario overlays and effective baseline. The
competition consumer includes the native RC mode-channel and SITL rangefinder
overlay from
[`competition.parm`](../ardupilot_sitl/params/competition.parm) in that identity
and preflight readback. The common Gazebo range subscription remains identical
across all three vehicle variants, preserving the shared airframe fingerprint.
The original unloaded version-1 profile remains valid only for `iris_flight`.

The [moving-pad template](../config/configured-moving-pad-run.json) binds
`configured`, `iris_moving_pad`, and `moving_pad_v1` to the moving-pad world,
640x480 recording, and a 90-second native warmup. Resolution rejects mismatched
scene bindings before launch. The stationary control uses the same contract
with `moving_pad_stationary`; its template lives in
[`tests/fixtures`](../tests/fixtures/configured-stationary-pad-run.json).
Completed moving-pad runs validate the moving-pad scoring rules, independently
of the companion's mission completion.

An explicit `auxiliary_services=("operator-wait-runtime",)` launch adds the
external operator service to health supervision. It remains outside the seven
module owners and quiescence barrier. After runtime-frozen, orchestration stops
it before artifact/manifest hashing. The suite uses it only for the operator
case; direct operator templates still support manual arming.

At runtime, orchestration publishes `/simulation/run_state` using the actual
[`RunState` schema](../ros_ws/src/simulation_interfaces/msg/RunState.msg), consumes
aggregate [`ArtifactStatus`](../ros_ws/src/simulation_interfaces/msg/ArtifactStatus.msg),
and exchanges typed durable facts through the run directory.
Publisher/subscriber setup and discovery requirements remain authoritative in
[`runtime_node.py`](src/orchestration/runtime_node.py). The shared runtime status
contract above owns typed wire and schema validation, with
[`status_store.py`](src/orchestration/status_store.py) as its host-side adapter.
[`controller.py`](src/orchestration/controller.py) owns orchestration's host
lifecycle and semantic checks, including deadlines, artifact consistency, and
run-state/result logic.

The controller produces `configuration/run.json`, operator status, finalization
requests, captured logs, and `manifest.json`. It consumes module readiness and
failure facts, recorder completeness, source/image provenance, simulation timing,
and scoring provenance. `collect-results` reads the committed result; it does not
copy or rebuild the bundle.

While waiting for a durable runtime status, the controller checks finalization
requests, runtime failures, and Compose child health before reading the target
status. The default delay between completed probes is one wall second. Each
probe also pays its command and status-read cost: `docker compose ps` retains its
`min(remaining deadline, 5 seconds)` attempt timeout, and the following sleep is
clamped to the remaining deadline. Detection latency therefore includes probe
work plus the inter-probe delay; this is not a one-second abort-latency guarantee.
The cadence changes host supervision overhead without changing APIs or lifecycle,
finalization, and teardown deadlines.

## Constraints worth preserving

- `source-finished` only says the public simulation source ended. A completed
  Phase 3 run separately requires a landed `mission-finished` fact and a
  `score-finished` fact at the source-completion timestamp. A good score alone is
  not a successful run.
- Runtime status reads select an exact registered type. Schema failures remain
  protocol failures; orchestration does not reinterpret a malformed value as a
  missing or successful status.
- Every terminal path enters `FINALIZING`. Publishers must become quiescent before
  orchestration writes aggregate `runtime-frozen`; artifacts then drains and
  closes its recorders before the host validates the bundle.
- The bag intentionally ends at `FINALIZING`. Terminal success depends on closure
  and validation, so the atomically committed `manifest.json`—not the final ROS
  sample or mutable operator-status cache—is authoritative.
- Finalization uses one monotonic wall-clock deadline with bounded manifest and
  teardown reserves. Individual waits and adapters must not restart that budget.
- `start` uses an isolated run-scoped Compose project and `--no-build`. Source
  edits, image identity, and recorded provenance must agree; operational build and
  launch steps belong in the [runbook](../docs/runbook.md).

## Focused tests

Run from the project root:

```bash
uv run pytest orchestration/tests/test_cli.py orchestration/tests/test_config.py \
  orchestration/tests/test_calibration.py orchestration/tests/test_lifecycle.py -q
uv run pytest orchestration/tests/test_controller.py \
  orchestration/tests/test_runtime_node.py orchestration/tests/test_status_store.py -q
```
