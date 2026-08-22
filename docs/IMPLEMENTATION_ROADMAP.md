# Runnable Simulation Implementation Roadmap

The authoritative design is `docs/superpowers/specs/2026-08-22-runnable-simulation-design.md`. Each phase receives its own executable Superpowers plan. A later phase starts only after the prior phase's integration gate passes and its interface changes are recorded.

## Phase 1: Foundation and contracts

Deliver a Dockerized ROS 2 Jazzy workspace, shared message schemas, immutable run configuration, lifecycle state machine, structured logging, bundle/manifest schemas, synthetic ROS nodes, and contract tests.

Gate: a synthetic run reaches all valid lifecycle states, rejects invalid transitions, emits correlated JSONL, and produces a schema-valid partial bundle without Gazebo.

## Phase 2: Run lifecycle and artifacts

Deliver operator commands, Compose lifecycle, recorder readiness, full ROS 2 bag capture, two H.264 MP4 pipelines, Docker-log capture, checksum validation, atomic manifest finalization, and completed/failed/aborted artifact tests.

Gate: synthetic `/clock` and two image streams produce a validated bundle containing playable videos, readable bag, logs, and terminal manifest.

## Phase 3: Gazebo physical foundation

Review and selectively import legacy worlds, models, meshes, and focused plugins. Deliver Gazebo Harmonic, `ros_gz`, authoritative `/clock`, reset, ground truth, onboard and observer cameras, native state, and server logs.

Gate: Gazebo running below real time produces correctly spaced 20-Hz simulation-time streams and a complete recorded physical-simulation bundle.

## Phase 4: ArduPilot lockstep

Select and pin the ArduPilot SITL image or reproducible image build, place all parameters under `ardupilot_sitl/`, establish MAVLink and the Gazebo adapter, and verify lockstep under host slowdown.

Gate: a commanded vehicle arms, changes actuator output, advances with Gazebo in lockstep, and preserves telemetry, acknowledgements, parameters, and logs.

## Phase 5: Companion integration

Containerize `companion/comp2026` with the smallest local patch, disable incompatible LiDAR dependencies, consume onboard imagery and simulation timestamps, connect MAVLink, and preserve frame-to-command causality.

Gate: the companion consumes the exact recorded onboard stream and commands ArduPilot without direct Gazebo access. No nested-repository changes are pushed.

## Phase 6: Scenario, scoring, and acceptance

Deliver electromagnet scenario events and Gazebo physical effects, ground-truth score calculation, versioned maximum score, deterministic terminal conditions, and full-stack failure/abort handling.

Gate: preserve at least one complete descent run whose achieved score equals the maximum available score and whose manifest links evidence for every awarded event. Completed, failed, and aborted test runs all preserve diagnosable bundles.

## Parallel ownership

The root coordinator owns cross-module contracts, integration gates, and final verification. Workers receive exclusive path ownership. Within a phase, independent work may use up to seven workers under the eight-thread limit; dependent integration work remains sequential. Any cross-module contract change is recorded before consumers implement against it.

## Planned executable documents

- `docs/superpowers/plans/2026-08-22-phase-1-foundation.md`
- `docs/superpowers/plans/phase-2-run-artifacts.md`
- `docs/superpowers/plans/phase-3-gazebo-foundation.md`
- `docs/superpowers/plans/phase-4-ardupilot-lockstep.md`
- `docs/superpowers/plans/phase-5-companion-integration.md`
- `docs/superpowers/plans/phase-6-scoring-acceptance.md`

Only Phase 1 is detailed before implementation begins. Subsequent plans use evidence and fixed interfaces from the preceding gate rather than guessing around unproven dependencies.
