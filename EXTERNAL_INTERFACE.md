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
- Optional `runtime_profile`, restricted to `phase2` or `phase3`. Omission is
  reserved for the Phase 2 infrastructure regression template; the repository
  default selects `phase3`.
- Phase 3 `simulation` fields: unsigned 32-bit `seed`, positive finite
  `duration_sim_seconds` on the fixed 50,000,000 ns camera grid, and frozen
  `target_real_time_factor=0.1`.
- ROS 2 discovery and network configuration
- Result and log destinations

Recording geometry is exactly `320x240` at 20 FPS with `rgb8` encoding.
Template and resolved-config validation reject incompatible profile/simulation
pairs, invalid timing, or other geometry before Compose construction. Phase 3
derives the expected per-stream frame count from the exact integer-nanosecond
duration; the default 2.0 simulated seconds yields 40 frames.

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

The topic and type inventory remains fixed. Phase 2 test doubles and the Phase
3 production Gazebo adapter use these public delivery contracts:

| Topic | Message | QoS |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | Best effort, volatile, depth 1 |
| `/simulation/run_state` | `simulation_interfaces/msg/RunState` | Reliable, transient local, depth 1 |
| `/simulation/artifact_status` | `simulation_interfaces/msg/ArtifactStatus` | Reliable, transient local, depth 1 |
| `/simulation/ground_truth` | `simulation_interfaces/msg/GroundTruth` | Best effort, volatile, depth 10 |
| `/simulation/scenario_events` | `simulation_interfaces/msg/ScenarioEvent` | Reliable, volatile, depth 100 |
| `/simulation/score_events` | `simulation_interfaces/msg/ScoreEvent` | Reliable, volatile, depth 100 |
| `/camera/onboard/image_raw` | `sensor_msgs/msg/Image` | Reliable, volatile, depth 5 |
| `/camera/onboard/frame_metadata` | `simulation_interfaces/msg/FrameMetadata` | Reliable, volatile, depth 5 |
| `/camera/observer/image_raw` | `sensor_msgs/msg/Image` | Reliable, volatile, depth 5 |
| `/camera/observer/frame_metadata` | `simulation_interfaces/msg/FrameMetadata` | Reliable, volatile, depth 5 |

The Phase 2 synthetic camera publisher, video subscriptions, and rosbag
overrides use the reliable depth-5 archival contract so exact recording does
not depend on lossy delivery. A later mission consumer may request compatible
best-effort delivery from the same reliable publisher.

`ArtifactStatus` aggregates the whole artifact subsystem. Before the first
clock it uses simulation time zero and the bag records first
`ready=false, complete=false, missing=[onboard, observer, rosbag]`, then
`ready=true, complete=false, missing=[]`. After the bag closes and the host
commits `manifest.json`, artifacts publishes the live transient-local final
notification with aggregate completeness, sorted missing or invalid paths,
and portable manifest path `manifest.json`; that final notification is
intentionally outside the immutable bag. Every custom message carries `run_id`
and `sim_timestamp`; event and frame messages carry their stable identifiers as
declared in the `.msg` files.

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

Phase 3 starts a fresh headless Gazebo server paused. Readiness requires the
server, private transport endpoints, bridges, public publishers, native
recorder, and artifact recorders before `READY`. The first controlled world
step establishes `/clock`; only after the current run reaches `RUNNING` may the
server unpause. The production runtime never waits for or consumes
`/simulation/camera_pair_ack`.

## Clock semantics

Gazebo's ROS 2 `/clock` is authoritative. Simulation-aware nodes enable `use_sim_time`.

## Failure behavior

Startup fails closed when required endpoints are unavailable. Loss of clock or
required physical progress pauses simulated decisions. Shutdown preserves
available diagnostics and partial results.

Reset is run-scoped replacement: destroy the container/server and start a fresh
Compose project and Gazebo partition from immutable SDF. There is no public
in-process reset endpoint.

## Phase 3 exclusions

Phase 3 does not implement ArduPilot SITL integration, MAVLink, actuator or
motor dynamics, NED conversion, ArduPilot-Gazebo lockstep, companion mission or
vision behavior, electromagnet forces, course policy, or competition scoring.
