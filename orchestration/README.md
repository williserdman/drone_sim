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
- [`StatusStore`](src/orchestration/status_store.py) owns host allocation and is
  the host-side adapter for the durable `.control/` and `.status/` protocol.
  The shared [`runtime_status.py`](../artifacts/src/artifacts/runtime_status.py)
  contract owns runtime status types, JSON conversion, and write policy;
  [`artifacts.protocol_files`](../artifacts/src/artifacts/protocol_files.py) owns
  low-level safe persistence.
  `StatusStore.validated_manifest_result` accepts paths defined by
  [`artifacts.manifest.is_manifest_relative_path`](../artifacts/src/artifacts/manifest.py);
  the [`manifest.json` schema](../artifacts/schemas/manifest.schema.json) is the wire authority.

## Consumer and producer seams

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
  orchestration/tests/test_lifecycle.py -q
uv run pytest orchestration/tests/test_controller.py \
  orchestration/tests/test_runtime_node.py orchestration/tests/test_status_store.py -q
```
