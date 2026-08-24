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

## Configuration

`runtime_profile` is an optional repository-owned selector restricted to
`phase2` and `phase3`. Omission resolves to `phase2` only for backward-compatible
infrastructure regression templates; `config/default-run.json` explicitly
selects `phase3`. Phase 2 rejects a `simulation` object. Phase 3 requires:

- `seed`: integer `0` through `4294967295`.
- `duration_sim_seconds`: positive finite duration whose nanoseconds divide
  exactly by `50,000,000`.
- `target_real_time_factor`: exactly `0.1`.

The resolved snapshot normalizes the selector and simulation values, includes
them in `config_sha256`, and derives the expected per-stream frame count before
Compose construction. Recording remains fixed at `320x240`, `rgb8`, and 20 FPS.

## Module lifecycle

Modules receive `run_id`, configuration, output paths, and lifecycle state.
Required endpoints and artifact recorders report readiness before `RUNNING`.
Every terminal cause enters `FINALIZING`, after which the artifacts module
reports completeness.

Under `phase3`, startup also requires current-run `gazebo-ready`,
`ardupilot-ready`, and `companion-ready` facts. The latter two prove live
Gazebo JSON exchange, the fixed internal MAVLink endpoint, and a consumed
heartbeat before the host accepts the production stack as ready.
The production server remains paused through endpoint discovery, bridge and
adapter startup, native-recorder startup, and artifact readiness. Orchestration
publishes `READY`, allows exactly one first step, persists `RUNNING` from the
first valid clock, and only then permits unpause. Production Gazebo does not
consume `/simulation/camera_pair_ack`.

Lifecycle states use the fixed order `CREATED`, `STARTING`, `READY`, `RUNNING`,
`FINALIZING`, `COMPLETED`, `FAILED`, and `ABORTED`. The orchestrator publishes
`simulation_interfaces/msg/RunState` on `/simulation/run_state` with reliable,
transient-local QoS depth 1. Before its first `STARTING` publication, the Phase
2 runtime requires the exact six runtime consumers plus rosbag recorder
subscription, including exact type and compatible QoS; this bounded
infrastructure barrier makes the full four-sample lifecycle archival record
deterministic without advancing simulation time.

Artifact readiness and completeness use
`simulation_interfaces/msg/ArtifactStatus` on
`/simulation/artifact_status` with reliable, transient-local QoS depth 1. The
aggregate pre-clock message uses simulation time zero. The final message uses
portable `manifest_path` value `manifest.json` and reports sorted missing or
invalid relative paths.

The run directory also carries the durable wall-time control/status protocol:
`.control/finalize-request.json`, `.control/terminal-committed.json`, and
`.status/{operator-state,artifacts-ready,gazebo-ready,ardupilot-ready,companion-ready,runtime-running,source-finished,mission-finished,score-finished,runtime-failure,runtime-frozen,artifacts-final,terminal-notified}.json`.
Each file is atomically replaced only after file and directory `fsync`.

For `phase3`, `source-finished` is not mission success. A completed run also
requires `mission-finished` with `outcome="LANDED"` and `score-finished` at the
same simulation timestamp as `source-finished`. The mission timestamp may be
earlier than source completion but cannot be later.

The six non-artifact publishers own exact
`.status/quiescence/<module>.json={run_id,module,quiescent:true}` markers for
orchestration, companion, `ardupilot_sitl`, Gazebo, electromagnet, and
scorekeeper. Each stops publishers and stdout before its marker. Orchestration
alone waits for all six markers and atomically publishes aggregate
`runtime-frozen.json`; artifacts trusts only that aggregate.

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

All terminal paths stop publishers and cross the aggregate `runtime-frozen.json`
quiescence barrier before recorders drain and close. The bag ends at
`FINALIZING`; `manifest.json` is authoritative for terminal status because the
terminal state depends on successful close and validation. Terminal durable
acknowledgement is intentionally silent on ROS/stdout after the marker boundary.

Each `start` uses a fresh run-scoped Compose project. Phase 3 reset means
destroying that server/container and starting a new server with a distinct run
ID and Gazebo partition; orchestration exposes no in-process reset operation.

Phase 3 makes no claim about ArduPilot SITL, MAVLink, motor dynamics,
ArduPilot-Gazebo lockstep, companion mission behavior, electromagnet forces, or
competition scoring.
