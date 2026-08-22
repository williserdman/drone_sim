# Electromagnet Plan

## Responsibility

Evaluate deterministic electromagnet scenario rules, request their physical effects from Gazebo, and publish matching events for scoring.

## Non-responsibilities

- Direct aircraft pose, velocity, actuator, or sensor mutation
- ArduPilot commands
- Physics implementation or score calculation

## Inputs and outputs

The module consumes run configuration and ROS 2 `/clock`. It produces physical-effect requests for Gazebo and scenario events for the scorekeeper.

## Implementation stages

1. Consume configuration and simulation time.
2. Evaluate deterministic scenario conditions.
3. Publish idempotent activation or deactivation requests to Gazebo.
4. Publish corresponding scorekeeper events.
5. Expose health and event diagnostics.

## Acceptance criteria

- Identical inputs produce identical simulation-timestamped events.
- Physical changes occur only through Gazebo.
- Events are correlated by `run_id`, event identity, and magnet identity.
- Loss of simulation time prevents new scenario decisions.

## Deferred decisions

- Scenario rule representation
- Physical-effect request topic, schema, and acknowledgement semantics
- Force, field, or constraint model parameters
