# Electromagnet External Interface

## ROS 2 inputs

- Authoritative `/clock` using best-effort QoS depth 1 with `use_sim_time=true`
- Run and scenario configuration supplied through the orchestration lifecycle

## ROS 2 outputs

- Idempotent physical-effect requests consumed by Gazebo
- Scenario events on `/simulation/scenario_events` using
  `simulation_interfaces/msg/ScenarioEvent` and reliable QoS depth 100

Each output identifies `run_id`, event identity, `magnet_id`, desired state, and simulation timestamp. Repeated delivery of the same event identity must not apply an effect or score twice.

## Timing and failure behavior

Scenario rules use simulation time exclusively. Stale-run inputs are ignored with diagnostics. Loss of `/clock` prevents new transitions. Delivery or peer failures are reported without directly compensating through aircraft state.

For Phase 2 synthetic finalization, the fixture publisher stops output before
writing `.status/quiescence/electromagnet.json` with exact current-run
quiescence schema, then remains silent while orchestration aggregates the
freeze.

## Prohibited paths

The module must never directly command ArduPilot or directly mutate aircraft pose, velocity, actuator, or sensor state. Physical effects pass through Gazebo physics.

## Deferred decisions

- Physical-effect request topic, schema, and acknowledgement semantics
- Physical effect parameters
