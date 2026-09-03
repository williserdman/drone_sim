# Runnable Drone Simulation Design

## Purpose

Deliver a Dockerized drone simulation whose behavior is governed by ROS 2 simulation time and whose successful output is a complete, reproducible run bundle. This design extends `2026-08-22-module-interfaces-design.md` from a documentation scaffold into an implementation architecture.

The system targets ROS 2 Jazzy on Ubuntu 24.04 LTS. Gazebo owns physical truth, ArduPilot SITL owns flight control, the companion owns mission decisions, the electromagnet module owns scenario policy, and the scorekeeper remains a read-only evaluator.

## Delivery Strategy

Use a vertical-slice rebuild with selective reuse from `/home/willis/projects/comp2026-transfer/drone_sim-comp2026`.

Reusable candidates include:

- source Gazebo worlds, models, meshes, and focused plugins;
- `config/competition.parm` and other validated ArduPilot parameters;
- course and scenario configuration;
- build knowledge and narrowly scoped artifact validators;
- existing recording manifests and timelines as behavioral references.

The following are not migrated wholesale:

- legacy Docker Compose orchestration;
- generated `.build`, `.logs`, and `.recordings` state;
- ticket-specific diagnostic scripts;
- coupled recording wrappers;
- wall-time scheduling and direct cross-module coupling.

Every reused item must identify its new owning module, comply with simulation-time rules, avoid undocumented seams, and have a focused verification test.

The nested `companion/comp2026` repository is integrated with the smallest practical patch. Its changes remain local and are not pushed. LiDAR assumptions may be disabled or removed only where required to run the camera/MAVLink mission path.

## Top-Level Modules

```text
drone_sim/
├── orchestration/
├── artifacts/
├── companion/
├── ardupilot_sitl/
├── gazebo/
├── electromagnet/
└── scorekeeper/
```

Every module maintains `PLAN.md`, `INTERNAL_INTERFACE.md`, and `EXTERNAL_INTERFACE.md`. Nested modules use the same recursive shape when decomposition improves ownership or testability.

### Orchestration

Owns Docker Compose lifecycle, run identity, configuration validation, endpoint readiness, terminal-state decisions, bounded shutdown, and manifest metadata. It never schedules simulated events from wall time.

### Artifacts

Owns ROS 2 bag recording, onboard and observer MP4 encoding, compressed Gazebo
state, structured module-log collection, checksums, completeness validation,
and final bundle assembly. Gazebo produces camera streams and native state;
the artifacts module records, validates, and organizes them without duplicating
raw image payloads in MCAP.

### Existing simulation modules

- Companion: mission, vision, autonomy, and MAVLink commands.
- ArduPilot SITL: estimation, navigation, control, parameters, MAVLink, and Gazebo lockstep participation.
- Gazebo: physics, sensors, cameras, ground truth, `/clock`, collisions, and external physical effects.
- Electromagnet: deterministic scenario decisions and Gazebo effect requests.
- Scorekeeper: deterministic evaluation from ground truth and scenario events.

## Run Lifecycle

```text
CREATED
  -> STARTING
  -> READY
  -> RUNNING
  -> FINALIZING
  -> COMPLETED | FAILED | ABORTED
```

- The orchestrator creates `run_id` before starting containers.
- Required artifact recorders become ready before Gazebo advances simulation time.
- `RUNNING` begins at the first valid simulation-clock epoch after readiness.
- Mission completion, infrastructure failure, operator cancellation, and timeout all pass through `FINALIZING`.
- Finalization uses bounded wall-clock deadlines because simulation time may have stopped.
- All terminal states preserve every recoverable artifact.
- `COMPLETED` requires the mission completion condition and successful validation of every required artifact.
- `FAILED` identifies a module, clock, lockstep, mission, scoring, or recording failure.
- `ABORTED` identifies operator cancellation or a configured external timeout.
- Missing or corrupt artifacts are recorded explicitly and never silently omitted.

## Required Run Bundle

```text
runs/<run_id>/
├── manifest.json
├── configuration/
├── gazebo/
│   ├── server.log
│   └── state/
│       └── state.tlog.zst
├── video/
│   ├── onboard.mp4
│   └── observer.mp4
├── rosbag/
├── logs/
│   ├── orchestration.jsonl
│   ├── artifacts.jsonl
│   ├── companion.jsonl
│   ├── ardupilot_sitl.jsonl
│   ├── gazebo.jsonl
│   ├── electromagnet.jsonl
│   └── scorekeeper.jsonl
└── scoring/
    ├── events.jsonl
    └── result.json
```

The manifest records:

- `run_id`, terminal status, and reason;
- simulation start, end, and duration;
- wall-clock start, end, and duration;
- source revisions and container image digests;
- configuration paths and checksums;
- artifact paths, sizes, checksums, and validation results;
- missing or incomplete artifacts;
- the scoring result summary.

## Video and Replay

Gazebo publishes two simulation-time camera streams:

- onboard: exactly the imagery delivered to companion vision;
- observer: an external view containing the aircraft and landing area.

Both streams use simulation timestamps and are encoded once into H.264 MP4
files by the artifacts module. Camera publication represents 20 frames per
simulated second regardless of real-time factor. These videos are the canonical
pixel evidence.

Every run also records an MCAP ROS 2 bag containing `/clock`, frame metadata for
both cameras, ground truth, scenario and score events, run lifecycle events,
and profile-specific physical evidence. Raw camera images are intentionally
excluded from MCAP because the MP4 files already preserve their pixels. Frame
IDs and simulation timestamps in MCAP provide the correlation needed to align
the videos with physical truth.

Gazebo native replay state is recorded as `gazebo/state/state.tlog`, compressed
with zstd during finalization, integrity-tested, and published only as
`state.tlog.zst`. The uncompressed source is removed only after successful
compression and validation. Viewers may stream-decompress this member while
leaving the bundle itself compressed; they must not require a second expanded
copy beside the bundle.

## Observability

Every owned process emits one JSON object per line to stdout. The common fields are:

- `run_id`;
- `module`;
- `severity`;
- `event`;
- `sim_timestamp` when a simulation clock is available;
- `wall_timestamp` for host diagnostics;
- structured event-specific fields.

Docker Compose logs remain available live. The artifacts module stores each module's stream in its corresponding run-bundle JSONL file. Centralized logging systems such as Loki, Grafana, and OpenTelemetry are deferred until the runnable stack demonstrates a need for them.

## Timing and Lockstep

- Gazebo publishes authoritative ROS 2 `/clock`.
- ROS 2 nodes affecting simulation enable `use_sim_time=true`.
- ArduPilot SITL and Gazebo advance through their supported lockstep exchange.
- A slow host changes wall duration, not simulated event timing or camera spacing.
- Modeled processing latency is represented in simulation time and is not inferred from host execution time.
- Loss of `/clock` or lockstep progress prevents new simulated decisions and leads to a diagnosed terminal state if recovery does not occur within an infrastructure wall-clock deadline.

## Data Flow

```text
Orchestration -> all modules: run configuration and lifecycle
Gazebo -> simulation-aware modules: /clock
Gazebo -> Companion: onboard camera
Gazebo -> Artifacts: onboard and observer cameras, ground truth, logs
Companion <-> ArduPilot SITL: MAVLink commands, telemetry, acknowledgements
ArduPilot SITL <-> Gazebo: actuator/sensor lockstep exchange
Electromagnet -> Gazebo: physical-effect requests
Electromagnet -> Scorekeeper: scenario events
Gazebo -> Scorekeeper: ground truth
Scorekeeper -> Artifacts: score events and result
All modules -> Artifacts: structured logs
Artifacts -> Orchestration: readiness and final completeness report
```

Run-scoped data carries `run_id` and a simulation timestamp. Causally derived data carries a source identifier where applicable. Receivers ignore stale runs and handle duplicate event identities idempotently.

## Failure Handling

- Startup fails closed when required endpoints or recorders are not ready.
- Camera, bag, and log recorders report readiness before physics begins.
- Required recorder failure makes the run `FAILED`, then triggers finalization.
- Optional diagnostics may be incomplete without masking the primary terminal reason.
- Finalization writes the manifest atomically after artifact validation.
- Partial runs remain inspectable and never overwrite another `run_id`.
- The scorekeeper reports incomplete input and never attempts control correction.
- The companion never directly changes Gazebo.
- The electromagnet module never directly changes ArduPilot or aircraft state.

## Implementation Phases

### Phase 1: Foundation and contracts

Create the ROS 2 Jazzy workspace, shared message package, run configuration schema, structured logging module, Compose skeleton, and contract tests. Prove lifecycle and logging with synthetic nodes before using Gazebo.

### Phase 2: Run lifecycle and artifacts

Implement the lifecycle state machine, manifests, JSONL capture, ROS bag recording, both MP4 encoders, finalization, and artifact validation against synthetic clock and image publishers.

### Phase 3: Gazebo physical foundation

Selectively import reviewed legacy assets. Establish `/clock`, ground truth, reset behavior, onboard and observer cameras, and successful recording at a slow real-time factor.

### Phase 4: ArduPilot lockstep

Pin or build the SITL image, inject parameter files owned by `ardupilot_sitl`, establish MAVLink and the Gazebo adapter, and prove lockstep under host slowdown.

### Phase 5: Companion integration

Containerize the nested companion repository with minimal local edits, disable incompatible LiDAR assumptions, connect onboard imagery and simulation timestamps, connect MAVLink, and trace commands to source frames.

### Phase 6: Scenario, scoring, and complete run

Implement electromagnet physics requests, ground-truth scoring, deterministic completion criteria, failure and abort cases, and a complete recorded descent that achieves the maximum possible score.

Each phase has its own implementation plan and integration gate. Work inside a phase may run concurrently only when file ownership and interfaces do not overlap.

## Verification Strategy

### Contract tests

Validate message schemas, run identity, timestamp rules, lifecycle transitions, structured-log fields, manifest schema, and forbidden communication paths.

### Module tests

Test each module through its documented interface. Use synthetic ROS 2 publishers/subscribers and test adapters before requiring the full stack.

### Integration gates

1. Synthetic lifecycle produces a valid finalized bundle.
2. Slow Gazebo execution records two 20-Hz simulation-time streams and a ROS bag.
3. ArduPilot and Gazebo remain in lockstep when real-time factor is below one.
4. Companion consumes images and commands ArduPilot without direct Gazebo access.
5. Electromagnet effects appear through Gazebo physics and scoring observes ground truth.
6. Completed, failed, and aborted runs all preserve diagnosable bundles.

### Final acceptance

A real end-to-end descent must finish with status `COMPLETED` and validate:

- Gazebo server log and integrity-checked `gazebo/state/state.tlog.zst`;
- playable onboard and observer MP4 files;
- readable MCAP with required metadata, lifecycle, physical-truth, and scoring
  topics, without duplicate raw image topics;
- valid per-module JSONL logs;
- score events and final result;
- manifest checksums, source revisions, image digests, timing, and artifact completeness;
- no dependence of simulated behavior on host real-time factor.

At least one preserved acceptance run must achieve the maximum score defined by the versioned scoring configuration. Its manifest records the maximum available score, achieved score, scoring-configuration checksum, and evidence paths for every awarded event.

## Agent Orchestration

Use one root coordinator to own cross-module contracts, phase gates, integration, and final verification. Assign subagents non-overlapping module directories or focused test surfaces. Subagents may coordinate through messages, but all cross-module decisions are recorded in the relevant interface document and approved through the root coordinator.

With an eight-thread limit, prefer one coordinator and up to seven workers only when the work is genuinely independent. Nested coordinators consume the same thread pool and are introduced only for a module with multiple independent internal workstreams. Concurrent workers do not commit overlapping paths or modify the nested companion repository without explicit task ownership.

## Default Technical Decisions

- ROS 2 Jazzy and Ubuntu 24.04 LTS.
- Gazebo Harmonic with `ros_gz` integration.
- Fast DDS, the ROS 2 Jazzy default middleware, unless container discovery verification demonstrates a concrete defect.
- Docker Compose for local lifecycle.
- ROS 2 image transport for cameras.
- MCAP recording for synchronized metadata, lifecycle, physical-truth, and
  scoring replay; MP4 files retain camera pixels.
- FFmpeg with H.264/libx264 and `yuv420p` pixel format for portable MP4 output at 20 frames per simulated second.
- Zstandard level 3 for native Gazebo state, with a full integrity test before
  atomic publication and deletion of the uncompressed source.
- JSON Lines for structured logs and score events.
- JSON for manifests, configuration snapshots where applicable, and final score results.
- SHA-256 artifact and configuration checksums.
- Source assets are copied selectively into their owning module; generated legacy outputs are not copied.
- ROS topic names start with `/simulation/run_state`, `/simulation/ground_truth`, `/simulation/scenario_events`, `/simulation/score_events`, `/camera/onboard/image_raw`, `/camera/observer/image_raw`, and standard `/clock`.
- `/clock` uses best-effort depth 1; physical video inputs use reliable,
  volatile depth 100; frame metadata uses reliable, volatile depth 100 for the
  private recorder; ground truth uses best-effort depth 10; lifecycle state
  uses reliable, transient-local depth 1; scenario and scoring events use
  reliable depth 100. Synthetic Phase 2 retains its smaller test-profile
  depths.
