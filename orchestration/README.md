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
  `status`, `abort`, and `collect-results`.
- [`RunController`](src/orchestration/controller.py) owns run allocation, Compose
  supervision, deadlines, terminal-cause selection, log capture, and manifest
  commit.
- [`resolve_run_config`](src/orchestration/config.py) is the template-to-resolved-
  configuration boundary; [`ComposeRuntime`](src/orchestration/_adapters/compose.py)
  is the Docker boundary.
- [`RunLifecycle`](src/orchestration/lifecycle.py) defines valid state transitions.
  [`runtime_node.py`](src/orchestration/runtime_node.py) publishes runtime lifecycle
  state and aggregates the quiescence barrier.
- [`StatusStore`](src/orchestration/status_store.py) implements the durable
  `.control/` and `.status/` protocol used by the host and containers.

## Consumer and producer seams

The CLI consumes templates such as
[`config/vertical-descent-run.json`](../config/vertical-descent-run.json).
The resolved configuration selects one immutable service ownership topology; the
actual service definitions and container commands live in [`compose.yaml`](../compose.yaml).
GPU operation is an optional deployment workflow documented in the
[runbook](../docs/runbook.md#optional-nvidia-path).

An operator template may name a complete `qgc` input set: deployment profile,
listener session, QGC actions, and runtime policy. Orchestration reads each
regular non-symlink JSON file once, keeps those bytes in one `QGCSources` value,
and writes canonical copies and SHA-256 digests into `configuration/`. The
deployment-profile digest also derives `attempt_state_id`. This is byte-identity
proof for the four snapshots. Immediately before a QGC run launches with
Compose `up`, orchestration requires the corresponding canonical state directory under
`/var/lib/drone-sim/comp2026-attempt-state`. It checks that directory and its
path components without following symlinks. The directory may contain only the
required regular `attempt-ledger.json` and required regular
`attempt-ledger.json.lock`; orchestration neither reads nor changes either file.
After launch, state consumption or a retained temporary file cannot block
status inspection, log capture, image lookup, service stop, or teardown.
QGC Docker commands explicitly use the local
`unix:///var/run/docker.sock` endpoint and discard ambient daemon, context, and
TLS selectors. This keeps host-path validation and bind resolution on the same
machine even if Docker has a saved remote context. Non-QGC runs retain their
existing daemon-selection behavior.

QGC runs add `compose.qgc.yaml`. That overlay gives only `companion-runtime` a
read-write bind of the single digest directory at the same absolute container
path, with automatic host-path creation disabled. It gives only
`ardupilot-sitl` the launch-origin environment. For QGC runs, orchestration
parses the canonical value without defaults from `simulator_launch_origin` in
the immutable runtime-policy bytes. Diagnostic Phase 3 runs use the fixed
adapter-owned origin `37.4003371,-122.0800351,0,0`; ambient process environment
cannot select either origin. The companion and ArduPilot still perform their
full domain validation. Runs without `qgc` keep the existing Compose files.
The checked-in default and realtime competition templates omit `qgc` and are
quarantined historical inputs. Resolution rejects them until an operator adds
the complete QGC input set.

At runtime, orchestration publishes `/simulation/run_state` using the actual
[`RunState` schema](../ros_ws/src/simulation_interfaces/msg/RunState.msg), consumes
aggregate [`ArtifactStatus`](../ros_ws/src/simulation_interfaces/msg/ArtifactStatus.msg),
and exchanges durable facts through the run directory. Publisher/subscriber setup
and discovery requirements remain authoritative in
[`runtime_node.py`](src/orchestration/runtime_node.py); protocol validation remains
authoritative in [`controller.py`](src/orchestration/controller.py) and
[`status_store.py`](src/orchestration/status_store.py).

The controller produces `configuration/run.json`, operator status, finalization
requests, captured logs, and `manifest.json`. It consumes module readiness and
failure facts, recorder completeness, source/image provenance, simulation timing,
and scoring provenance. `collect-results` reads the committed result; it does not
copy or rebuild the bundle.

## Constraints worth preserving

- `source-finished` only says the public simulation source ended. A completed
  Phase 3 run separately requires a landed `mission-finished` fact and a
  `score-finished` fact at the source-completion timestamp. A good score alone is
  not a successful run.
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
  orchestration/tests/test_lifecycle.py -q
uv run pytest orchestration/tests/test_controller.py \
  orchestration/tests/test_runtime_node.py orchestration/tests/test_status_store.py -q
```
