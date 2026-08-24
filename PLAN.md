# Drone Simulation Plan

## Responsibility

Coordinate the lifecycle and configuration of deterministic simulation runs containing the companion, ArduPilot SITL, Gazebo, electromagnet, and scorekeeper modules.

## Non-responsibilities

- Flight-control, mission, physics, sensor, scenario-effect, or scoring decisions
- Using orchestration timing to schedule simulated events

## Child modules

| Module | Responsibility |
| --- | --- |
| `orchestration` | Run lifecycle, readiness, terminal state, and manifest metadata |
| `artifacts` | Videos, ROS bags, logs, checksums, and bundle finalization |
| `companion` | Mission, vision, and autonomy decisions |
| `ardupilot_sitl` | Estimation, navigation, and flight control |
| `gazebo` | Physics, sensors, camera, external forces, and ground truth |
| `electromagnet` | Deterministic electromagnet scenario logic |
| `scorekeeper` | Read-only mission evaluation |

## Orchestration stages

1. Validate configuration and assign a unique run ID.
2. Start ROS 2 discovery and modules through Docker Compose.
3. Wait for endpoint readiness, not merely container startup.
4. Start or reset Gazebo and its authoritative simulation clock.
5. Permit execution after all required modules report readiness.
6. Collect logs, diagnostics, scores, and run metadata.
7. Stop modules cleanly and preserve results.

## Active execution plan

The active implementation path is `docs/superpowers/plans/runnable-vertical-descent.md`. It prioritizes, in order: a Compose-launched Gazebo smoke run, ArduPilot lockstep flight, a complete artifact-producing run, a valid scored run, and a verified maximum-score run. Independent module work proceeds in parallel at frozen interfaces; integration reviews gate only runtime correctness, deterministic timing, artifact integrity, and truthful scoring.

## Acceptance criteria

- All seven child modules have a plan and separate internal and external interfaces.
- Every communication path identifies its producer, consumer, mechanism, and clock.
- Responsibilities do not overlap.
- The scorekeeper cannot change control or physics.
- The companion cannot directly change Gazebo.
- Simulation and wall-clock time have distinct uses.
- Unresolved implementation choices are explicitly deferred.
- A preserved completed run achieves the versioned scoring configuration's maximum score.

## Deferred decisions

- Broader competition rules beyond committed `descent_v1`
- Active electromagnet/payload behavior
- Operator UX beyond the existing start and result-collection workflow
- Non-blocking adversarial hardening recorded in `docs/technical-debt/vertical-slice-hardening.md`
