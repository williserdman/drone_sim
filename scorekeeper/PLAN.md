# Scorekeeper Plan

## Responsibility

Observe authoritative ground truth and scenario events and calculate deterministic mission results.

## Non-responsibilities

- Aircraft commands or mode changes
- Actuator output, external forces, or pose changes
- Mission, flight-control, physics, or scenario decisions

## Inputs and outputs

The scorekeeper consumes run configuration, ROS 2 `/clock`, Gazebo ground truth, electromagnet events, and optionally diagnostic-only ArduPilot telemetry. It produces score events, final results, and incomplete-run diagnostics.

## Implementation stages

1. Consume run configuration and simulation time.
2. Consume ground truth and scenario events.
3. Correlate inputs by run and simulation time.
4. Calculate deterministic score changes.
5. Publish or persist results.
6. Diagnose incomplete runs.
7. Prove a completed acceptance run can achieve the versioned maximum score.

## Acceptance criteria

- Scores derive from Gazebo ground truth rather than mission estimates.
- Duplicate inputs cannot score twice.
- Results are reproducible for the same ordered run inputs.
- The module remains read-only with respect to flight control and physics.
- The final result records achieved score, maximum available score, scoring-configuration checksum, and evidence for every awarded event.

## Deferred decisions

- Scoring rules and result schema
- Topic names, message schemas, and QoS
- Result persistence format
