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
available score, `ruleset_id=descent_v1`, scoring-configuration checksum, and
safe evidence references. It
is persisted as `scoring/result.json` and is read-only with respect to the
simulated aircraft.

## Ordering and failure behavior

Ground-truth samples are accepted in exact 50,000,000 ns order for one canonical
run ID. The first duplicate, regression, or gap permanently marks the result
incomplete; a later suffix cannot repair it. Missing required input likewise
marks a run incomplete rather than causing corrective control.

The committed `rules/descent_v1.json` has maximum 100 and freezes a safe
pre-impact downward-speed threshold of 1.0 m/s. The scorekeeper emits exactly
four ordered rule events (`descent.airborne_then_contact`,
`descent.touchdown_precision`, `descent.safe_preimpact_speed`, and
`descent.stable_contact`) followed by `score.finalized`. It persists the same
five events as JSONL and exposes a narrow immutable finished-status document
for the production ROS/lifecycle adapter.

For Phase 2 synthetic finalization, the fixture publisher persists its scoring
files, stops output, then writes `.status/quiescence/scorekeeper.json` with
exact current-run quiescence schema. A test-only bounded wall delay may postpone
that already-decided marker to exercise aggregate freeze ordering; it never
changes simulated facts.

## Prohibited outputs

The scorekeeper exposes no command, mode, actuator, force, constraint, pose, velocity, or physics-mutation output.

## Deferred decisions

- Broader competition or active-electromagnet rulesets
- Optional diagnostic telemetry beyond authoritative ground truth
