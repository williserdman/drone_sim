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

The repository default selects the `vertical_descent` world, `iris_flight`
vehicle, controlled-descent mission, inactive `descent_v1` scenario, and 60.0
simulated seconds, yielding exactly 1,200 frames per camera. Production evidence
showed that the earlier 30-second window could end before cold ArduCopter
application initialization completed.

The resolved snapshot normalizes the selector and simulation values, includes
them in `config_sha256`, and derives the expected per-stream frame count before
Compose construction. Recording remains fixed at 20 FPS `rgb8`: `320x240` for
descent and `640x480` for the competition mission.

For every start, source provenance is captured from the actual parent and
`companion/comp2026` worktrees, in that order, including untracked files in the
dirty flag. Phase 3 binds the nested revision into the companion image label
before `up --no-build`; a missing or mismatched label fails startup without
altering either worktree.

## Module lifecycle

Modules receive `run_id`, configuration, output paths, and lifecycle state.
Required endpoints and artifact recorders report readiness before `RUNNING`.
Every terminal cause enters `FINALIZING`, after which the artifacts module
reports completeness.

Under `phase3`, startup also requires current-run `gazebo-ready`,
`ardupilot-ready`, and `companion-ready` facts. They prove live Gazebo JSON
exchange, the fixed internal MAVLink endpoint, and the companion's successful
TCP transport connection before the host accepts the production stack as
infrastructure-ready. `companion-ready` deliberately does not claim a
heartbeat.
The production server remains paused through endpoint discovery, bridge and
adapter startup, native-recorder startup, and artifact readiness. Orchestration
then publishes `READY` and Gazebo unpauses into a private lockstep warmup. Public
clock, camera, ground-truth, scenario, score, and mission-command output remain
inactive. The companion passively latches heartbeat and healthy prearm status
and writes exact `mission-ready`; only then does orchestration publish and
persist `RUNNING`. Gazebo arms the configured fixed native public epoch (Phase
3 default `90.0` seconds) before its exact target at that boundary. Production
Gazebo does not consume `/simulation/camera_pair_ack`.

Lifecycle states use the fixed order `CREATED`, `STARTING`, `READY`, `RUNNING`,
`FINALIZING`, `COMPLETED`, `FAILED`, and `ABORTED`. The orchestrator publishes
`simulation_interfaces/msg/RunState` on `/simulation/run_state` with reliable,
transient-local QoS depth 4. Before its first `STARTING` publication, the Phase
2 runtime requires the exact six runtime consumers plus rosbag recorder
subscription, including exact type and compatible QoS; this bounded
infrastructure barrier makes the full four-sample lifecycle archival record
deterministic without advancing simulation time.

The Phase 3 barrier instead requires the real `artifacts_runtime`,
`drone_sim_companion`, `drone_sim_gazebo_lifecycle`, `drone_sim_electromagnet`, and
`drone_sim_scorekeeper` subscriptions plus the rosbag recorder. ArduPilot SITL
does not subscribe to RunState and is gated only by its truthful durable
readiness fact.

Artifact readiness and completeness use
`simulation_interfaces/msg/ArtifactStatus` on
`/simulation/artifact_status` with reliable, transient-local QoS depth 1. The
aggregate pre-clock message uses simulation time zero. The final message uses
portable `manifest_path` value `manifest.json` and reports sorted missing or
invalid relative paths.

The run directory also carries the durable wall-time control/status protocol:
`.control/finalize-request.json`, `.control/terminal-committed.json`, and
`.status/{operator-state,artifacts-ready,gazebo-ready,ardupilot-ready,companion-ready,mission-ready,runtime-running,source-finished,mission-finished,score-finished,runtime-failure,runtime-frozen,artifacts-final,terminal-notified}.json`.
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

`mission-ready.json` is exactly
`{run_id,ready:true,heartbeat_observed:true,prearm_checks_healthy:true}`.
`runtime-running.json` contains the current `run_id`, fixed state `RUNNING`, and
zero as the public simulation epoch in integer nanoseconds. The runtime writes
it immediately after the corresponding ROS publication. It is the host
controller's authoritative durable evidence for reporting `RUNNING`.

## Timing

Simulation state uses `/clock`. Wall-clock deadlines are restricted to startup,
stalled-host detection, finalization, and forced shutdown and are measured with
a monotonic clock. Finalization uses one bounded deadline shared across
quiescence, recorder closure, validation, manifest commit, notification, and
teardown, with separate bounded manifest and teardown reserves.
Each Docker Compose process-inventory attempt is capped at five wall seconds;
a hung CLI probe is retried within the unchanged lifecycle deadline.

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

The Phase 3 competition selection uses the same seven services and durable
failure/finalization protocol. No eighth inspector or retry service is added.
