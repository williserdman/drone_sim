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

- Exact reset and readiness protocols
- Production Docker Compose topology and health-check intervals
- Operator command-line or service syntax
