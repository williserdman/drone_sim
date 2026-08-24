# Electromagnet Plan

## Responsibility

Evaluate deterministic electromagnet scenario rules, request their physical effects from Gazebo, and publish matching events for scoring.

## Non-responsibilities

- Direct aircraft pose, velocity, actuator, or sensor mutation
- ArduPilot commands
- Physics implementation or score calculation

## Inputs and outputs

The vertical-slice runtime consumes run configuration and ROS 2 `/clock` and
produces the truthful inactive scenario event for the scorekeeper. It exposes
no Gazebo effect request in `descent_v1`; that seam remains reserved for a
future active-magnet ruleset.

## Implementation stages

1. Consume the authoritative ROS simulation clock.
2. Publish one deterministic `descent_v1` inactive event.
3. Expose readiness, publication, failure, finalization, and quiescence logs.

Active-effect requests and force parameters remain deferred because the
vertical descent rules require a permanently inactive magnet.

## Acceptance criteria

- Identical inputs produce identical simulation-timestamped events.
- Physical changes occur only through Gazebo.
- Events are correlated by `run_id`, event identity, and magnet identity.
- Loss of simulation time prevents new scenario decisions.

## Deferred decisions

- Scenario rule representation
- Physical-effect request topic, schema, and acknowledgement semantics
- Force, field, or constraint model parameters
