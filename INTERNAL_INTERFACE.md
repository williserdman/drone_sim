# Root Internal Interface

## Purpose

Define relationships among the repository's immediate child modules. Top-level children are initially separate processes, so they communicate through their external interfaces rather than language imports.

## Sibling modules

| Producer | Consumer | Mechanism | Data |
| --- | --- | --- | --- |
| Orchestration | All modules | Compose configuration and lifecycle | Run identity, configuration, startup, and finalization |
| Artifacts | Orchestration | ROS 2 `/simulation/artifact_status` and run-directory status files | Recorder readiness and artifact completeness |
| Companion | ArduPilot SITL | MAVLink | Mission and flight commands |
| ArduPilot SITL | Companion | MAVLink | Telemetry, modes, acknowledgements |
| ArduPilot SITL | Gazebo | Reserved Phase 4 ArduPilot-Gazebo adapter | Actuator outputs; inactive in Phase 3 |
| Gazebo | ArduPilot SITL | Reserved Phase 4 ArduPilot-Gazebo adapter | Simulated sensors and dynamics; inactive in Phase 3 |
| Gazebo | Companion | ROS 2 image transport | Camera frames |
| Gazebo | Scorekeeper | ROS 2 | Ground truth |
| Electromagnet | Gazebo | ROS 2 | Physical-effect requests |
| Electromagnet | Scorekeeper | ROS 2 | Scenario events |
| Gazebo | Simulation-aware modules | ROS 2 `/clock` | Simulation time |
| All modules | Artifacts | Structured stdout | Run-correlated JSON Lines logs |
| Gazebo | Artifacts | ROS 2 and filesystem | Both camera streams, ground truth, server log, and state |
| Scorekeeper | Artifacts | ROS 2 and filesystem | Score events and final result |

## Allowed dependencies

Nested same-process siblings may import one another only through the interface documented by the module they consume.

Gazebo Transport is private to the Gazebo module. Repository siblings consume
only Gazebo's documented ROS 2 and durable-file interfaces. Within
`drone_sim_gazebo`, the `worlds`, `models`, `server`, `ros_adapter`, and
`runtime` sibling packages communicate through their documented directory
APIs; callers do not import private implementation files across those seams.

## Forbidden dependencies

- Companion-to-Gazebo control
- Electromagnet-to-ArduPilot or direct aircraft-state mutation
- Scorekeeper-to-control or physics mutation
- Imports of another module's private implementation files

## Run identity

Every run has a unique `run_id`. Run-scoped messages preserve it, and receivers ignore stale run IDs while emitting diagnostics.

## Timing rules

Simulation time schedules simulated behavior. Wall time is limited to infrastructure health checks, profiling, and host-performance diagnostics.

## Finalization protocol

The run directory is the wall-time-safe control plane. Orchestration writes
`.control/finalize-request.json`; the runtime then writes
`.status/runtime-frozen.json` only after all synthetic publishers have stopped
permanently. Artifacts treats that file as the quiescence barrier, drains
callbacks, closes videos and the bag, and writes `.status/artifacts-final.json`.
The host captures logs, validates the bundle, commits `manifest.json`, and
writes `.control/terminal-committed.json`. Terminal ROS notifications happen
after that commit and write no required artifact data.

The host establishes one absolute finalization deadline from the resolved
`finalization_wall_seconds` using a monotonic wall clock. Publisher quiescence,
recorder drain and close, validation, manifest commit, terminal notification,
and teardown share its remaining bounded budget; no stage restarts the clock.
Equal bounded manifest and teardown reserves remain inside that deadline.
Cooperative callbacks stop host parsing, traversal, and hashing at the earlier
work boundary; unfinished records become explicit timeout-invalid evidence so
a diagnostic `FAILED`/`ABORTED` manifest can still be attempted in its reserve.

All control and status JSON is committed through a collision-safe temporary
sibling, file flush and `fsync`, atomic replacement, and directory `fsync`.
The complete status set is `operator-state.json`, `artifacts-ready.json`, the
Phase 3 `gazebo-ready.json`, `runtime-running.json`, `source-finished.json`,
`runtime-failure.json`, `runtime-frozen.json`, `artifacts-final.json`, and
`terminal-notified.json` under `.status/`. The ROS orchestration runtime writes `runtime-running.json`
immediately after publishing `RUNNING` for the first valid clock; the host uses
that durable signal to update operator state without treating wall time as
simulation progress.

For Phase 3, the Gazebo runtime first pauses the world, stops and drains its
public adapter, stops bridges and the server, freezes native state and the raw
server log, and writes only its module quiescence marker. Aggregate freeze and
manifest authority remain with orchestration and artifacts respectively.
