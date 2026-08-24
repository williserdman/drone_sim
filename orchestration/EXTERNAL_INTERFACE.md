# Orchestration External Interface

## Operator operations

```text
uv run drone-sim start --config PATH
uv run drone-sim status RUN_ID [--output-root PATH]
uv run drone-sim abort RUN_ID [--output-root PATH]
uv run drone-sim collect-results RUN_ID [--output-root PATH]
```

`start` resolves the operator template, generates the run ID, and owns the
foreground Compose run. It exits `0` for `COMPLETED`, `1` for `FAILED`, and
`130` for `ABORTED`. The other commands communicate only through the run
directory and may run concurrently. `collect-results` is read-only.

## Module lifecycle

Modules receive `run_id`, configuration, output paths, and lifecycle state. Required endpoints and artifact recorders report readiness before `RUNNING`. Every terminal cause enters `FINALIZING`, after which the artifacts module reports completeness.

Lifecycle states use the fixed order `CREATED`, `STARTING`, `READY`, `RUNNING`,
`FINALIZING`, `COMPLETED`, `FAILED`, and `ABORTED`. The orchestrator publishes
`simulation_interfaces/msg/RunState` on `/simulation/run_state` with reliable,
transient-local QoS depth 1. Phase 1 proves this publisher with the synthetic
foundation service; production orchestration is implemented in a later phase.

Artifact readiness and completeness use
`simulation_interfaces/msg/ArtifactStatus` on
`/simulation/artifact_status` with reliable, transient-local QoS depth 1. The
aggregate pre-clock message uses simulation time zero. The final message uses
portable `manifest_path` value `manifest.json` and reports sorted missing or
invalid relative paths.

The run directory also carries the durable wall-time control/status protocol:
`.control/finalize-request.json`, `.control/terminal-committed.json`, and
`.status/{operator-state,artifacts-ready,runtime-running,source-finished,runtime-failure,runtime-frozen,artifacts-final,terminal-notified}.json`.
Each file is atomically replaced only after file and directory `fsync`.

`runtime-running.json` contains the current `run_id`, fixed state `RUNNING`, and
the nonnegative first-clock simulation timestamp in integer nanoseconds. The
runtime writes it immediately after the corresponding ROS publication. It is
the host controller's authoritative durable evidence for reporting `RUNNING`.

## Timing

Simulation state uses `/clock`. Wall-clock deadlines are restricted to startup,
stalled-host detection, finalization, and forced shutdown and are measured with
a monotonic clock. Finalization uses one bounded deadline shared across
quiescence, recorder closure, validation, manifest commit, notification, and
teardown, with separate bounded manifest and teardown reserves.

## Failure behavior

Startup fails closed. Partial outputs are preserved. A manifest is produced for completed, failed, and aborted runs.
Once `ArtifactSession` returns its typed committed result, that manifest's
terminal status and reason are authoritative; a subsequent manifest read or
deadline failure is diagnostic only.
control acknowledgment, terminal-notification, observability, and teardown
failures are retained as diagnostics and cannot rewrite the terminal result.

All terminal paths stop publishers and cross the `runtime-frozen.json`
quiescence barrier before recorders drain and close. The bag ends at
`FINALIZING`; `manifest.json` is authoritative for terminal status because the
terminal state depends on successful close and validation.
