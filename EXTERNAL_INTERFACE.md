# Root External Interface

## Operator surface

The orchestration surface conceptually provides `start`, `reset`, `status`, `collect-results`, and `stop`. Exact CLI or service syntax is deferred.

## Configuration inputs

- Unique `run_id`
- World, vehicle, mission, and scenario configuration
- ROS 2 discovery and network configuration
- Result and log destinations

Secret values must be supplied at runtime and must not be committed.

## Lifecycle operations

- `start`: validate configuration, start modules, and wait for readiness.
- `reset`: issue a new `run_id` and reset authoritative simulation state.
- `status`: report endpoint readiness and infrastructure health.
- `collect-results`: collect scores, logs, and run metadata.
- `stop`: stop the run cleanly without discarding results.

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
| `/simulation/ground_truth` | `simulation_interfaces/msg/GroundTruth` | Best effort, depth 10 |
| `/simulation/scenario_events` | `simulation_interfaces/msg/ScenarioEvent` | Reliable, depth 100 |
| `/simulation/score_events` | `simulation_interfaces/msg/ScoreEvent` | Reliable, depth 100 |
| `/camera/onboard/image_raw` | ROS 2 image transport plus `simulation_interfaces/msg/FrameMetadata` correlation | Best effort, depth 5 |
| `/camera/observer/image_raw` | ROS 2 image transport plus `simulation_interfaces/msg/FrameMetadata` correlation | Best effort, depth 5 |

The shared package also defines `ArtifactStatus` for recorder readiness and
completeness. Its final topic or service binding remains a Phase 2 decision.
Every custom message carries `run_id` and `sim_timestamp`; event and frame
messages carry their stable identifiers as declared in the `.msg` files.

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

## Health semantics

A container is ready only when its required process and communication endpoints are ready. Wall-clock health timeouts may identify a stalled host but never advance simulation state.

## Clock semantics

Gazebo's ROS 2 `/clock` is authoritative. Simulation-aware nodes enable `use_sim_time`.

## Failure behavior

Startup fails closed when required endpoints are unavailable. Loss of clock or lockstep progress pauses simulated decisions. Shutdown preserves available diagnostics and partial results.

## Deferred decisions

- Command-line or orchestration-service syntax
- Reset and readiness wire protocols
- Production Docker Compose topology and health-check intervals
- Artifact-status topic or service binding
