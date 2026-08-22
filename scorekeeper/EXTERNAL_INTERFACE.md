# Scorekeeper External Interface

## ROS 2 inputs

- Authoritative `/clock` using best-effort QoS depth 1 with `use_sim_time=true`
- Gazebo ground truth on `/simulation/ground_truth` using
  `simulation_interfaces/msg/GroundTruth` and best-effort QoS depth 10
- Electromagnet events on `/simulation/scenario_events` using
  `simulation_interfaces/msg/ScenarioEvent` and reliable QoS depth 100
- Optional ArduPilot telemetry for diagnostics only

Inputs carry `run_id`, simulation timestamps, and stable state or event identities as applicable.

## Outputs

The module emits run-scoped `simulation_interfaces/msg/ScoreEvent` messages on
`/simulation/score_events` using reliable QoS depth 100, plus final results and
incomplete-run diagnostics. The final result contains achieved score, maximum
available score, scoring-configuration checksum, and evidence references. It
is persisted as `scoring/result.json` and is read-only with respect to the
simulated aircraft.

## Ordering and failure behavior

Inputs are correlated by run and simulation time. Duplicate event identities are idempotently ignored. Late or out-of-order data follows a documented buffering policy before results are finalized. Missing required inputs mark a run incomplete rather than causing corrective control.

## Prohibited outputs

The scorekeeper exposes no command, mode, actuator, force, constraint, pose, velocity, or physics-mutation output.

## Deferred decisions

- Event buffering and result-finalization window
- Result fields beyond the fixed scoring summary
