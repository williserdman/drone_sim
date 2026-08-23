# Root External Interface

## Operator surface

The fixed operator commands are:

```text
uv run drone-sim start --config PATH
uv run drone-sim status RUN_ID [--output-root PATH]
uv run drone-sim abort RUN_ID [--output-root PATH]
uv run drone-sim collect-results RUN_ID [--output-root PATH]
```

`start` owns the foreground Compose run and exits `0` for `COMPLETED`, `1` for
`FAILED`, or `130` for `ABORTED`. The other commands communicate only through
the run directory and are safe to invoke concurrently. Their output root
defaults to resolved `runs`; a caller using a custom template output root passes
the same absolute path explicitly.

## Configuration inputs

- An operator template without `run_id`; orchestration generates the unique ID
- World, vehicle, mission, and scenario configuration
- ROS 2 discovery and network configuration
- Result and log destinations

Secret values must be supplied at runtime and must not be committed.

## Lifecycle operations

- `start`: validate and resolve configuration, start modules, and wait for a
  terminal manifest.
- `status`: report the durable operator state for one run.
- `abort`: request one idempotent `ABORTED` finalization.
- `collect-results`: validate and print an existing manifest path without
  changing run data.

The fixed lifecycle state order is `CREATED`, `STARTING`, `READY`, `RUNNING`,
`FINALIZING`, then one of `COMPLETED`, `FAILED`, or `ABORTED`. All terminal
causes pass through `FINALIZING`. The lifecycle ROS contract is
`simulation_interfaces/msg/RunState` on `/simulation/run_state` with reliable,
transient-local QoS depth 1.

## Fixed ROS 2 contracts

Phase 1 fixes these topic and QoS contracts for later module implementations:

| Topic | Message | QoS |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | Best effort, depth 1 |
| `/simulation/run_state` | `simulation_interfaces/msg/RunState` | Reliable, transient local, depth 1 |
| `/simulation/artifact_status` | `simulation_interfaces/msg/ArtifactStatus` | Reliable, transient local, depth 1 |
| `/simulation/ground_truth` | `simulation_interfaces/msg/GroundTruth` | Best effort, depth 10 |
| `/simulation/scenario_events` | `simulation_interfaces/msg/ScenarioEvent` | Reliable, depth 100 |
| `/simulation/score_events` | `simulation_interfaces/msg/ScoreEvent` | Reliable, depth 100 |
| `/camera/onboard/image_raw` | ROS 2 image transport | Best effort, depth 5 |
| `/camera/onboard/frame_metadata` | `simulation_interfaces/msg/FrameMetadata` | Best effort, depth 5 |
| `/camera/observer/image_raw` | ROS 2 image transport | Best effort, depth 5 |
| `/camera/observer/frame_metadata` | `simulation_interfaces/msg/FrameMetadata` | Best effort, depth 5 |

`ArtifactStatus` aggregates the whole artifact subsystem. Before the first
clock it uses simulation time zero. Its final notification reports aggregate
completeness, sorted missing or invalid paths, and the portable manifest path
`manifest.json`. Every custom message carries `run_id` and `sim_timestamp`;
event and frame messages carry their stable identifiers as declared in the
`.msg` files.

## Observable outputs

Lifecycle status, module diagnostics, score results, logs, recordings, ROS bags, and run metadata are correlated by `run_id`. A terminal run preserves a manifest describing completeness and checksums even when the run failed or was aborted.

Every owned process emits JSON Lines with the common fields `run_id`, `module`,
`severity`, `event`, `sim_timestamp`, and `wall_timestamp`. Event-specific data
is stored under `fields` by the Phase 1 serializer.

The fixed required artifact categories are the configuration snapshot, Gazebo
server log and native state, onboard and observer MP4 files, ROS bag, one JSONL
log for each of the seven modules, score events, and the final score result.
`COMPLETED` requires all categories to validate; `FAILED` and `ABORTED` retain
explicit missing or invalid records.

The ROS bag contains the ten fixed topics above and deliberately ends with the
`FINALIZING` lifecycle event. After all publishers are quiescent, recorders
drain, close, and validate within one shared bounded deadline measured by a
monotonic wall clock. `manifest.json` is authoritative for the terminal status
because `COMPLETED` depends on successful bag closure and artifact validation.

## Health semantics

A container is ready only when its required process and communication endpoints are ready. Bounded deadlines use a monotonic wall clock to identify a stalled host but never advance simulation state.

## Clock semantics

Gazebo's ROS 2 `/clock` is authoritative. Simulation-aware nodes enable `use_sim_time`.

## Failure behavior

Startup fails closed when required endpoints are unavailable. Loss of clock or lockstep progress pauses simulated decisions. Shutdown preserves available diagnostics and partial results.

## Deferred decisions

- Reset and readiness wire protocols
- Production Docker Compose topology and health-check intervals
