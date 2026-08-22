# Scorekeeper External Interface

## ROS 2 inputs

- Authoritative `/clock` with `use_sim_time=true`
- Gazebo ground truth
- Electromagnet scenario events
- Optional ArduPilot telemetry for diagnostics only

Inputs carry `run_id`, simulation timestamps, and stable state or event identities as applicable.

## Outputs

The module emits run-scoped score events, final results, and incomplete-run diagnostics. The final result contains achieved score, maximum available score, scoring-configuration checksum, and evidence references. It is read-only with respect to the simulated aircraft.

## Ordering and failure behavior

Inputs are correlated by run and simulation time. Duplicate event identities are idempotently ignored. Late or out-of-order data follows a documented buffering policy before results are finalized. Missing required inputs mark a run incomplete rather than causing corrective control.

## Prohibited outputs

The scorekeeper exposes no command, mode, actuator, force, constraint, pose, velocity, or physics-mutation output.

## Deferred decisions

- Exact topics, message and result schemas, and QoS
- Event buffering and result-finalization window
- Persistence format
