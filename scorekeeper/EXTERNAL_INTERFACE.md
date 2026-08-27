# Scorekeeper External Interface

## ROS 2 inputs

- Authoritative `/clock` using best-effort QoS depth 1 with `use_sim_time=true`
- Gazebo ground truth on `/simulation/ground_truth` using
  `simulation_interfaces/msg/GroundTruth`. `descent_v1` retains best-effort QoS
  depth 10; `competition_v1` uses reliable, volatile QoS depth 10.
- For `descent_v1`, electromagnet events on `/simulation/scenario_events` use
  `simulation_interfaces/msg/ScenarioEvent` and reliable, transient-local QoS
  depth 100.
- For `competition_v1`, three 20 Hz physical streams on
  `/simulation/payload_state` use `simulation_interfaces/msg/PayloadState` and
  reliable, volatile QoS depth 100. Confirmed `/simulation/payload_events` and
  ordered `/simulation/mission_events` use their shared message contracts and
  reliable, transient-local QoS depth 100.
- Optional ArduPilot telemetry for diagnostics only

Inputs carry `run_id`, simulation timestamps, and stable state or event identities as applicable.

## Outputs

The module emits run-scoped `simulation_interfaces/msg/ScoreEvent` messages on
`/simulation/score_events` using reliable QoS depth 100, plus final results and
incomplete-run diagnostics. The final result contains achieved score, maximum
available score, the selected `descent_v1` or `competition_v1` ruleset,
scoring-configuration checksum, and safe evidence references. It
is persisted as `scoring/result.json` and is read-only with respect to the
simulated aircraft.

`competition_v1` emits seven ordered point-component events followed by
`score.finalized`: FM1 landing 20, FM1 autonomy 30, payload 2 delivery 10, FM2
autonomy 20, payload 3 delivery 15, FM3 autonomy 50, and payload 4 delivery 5.
Their cumulative checkpoints are 80 after FM2, 145 after marker 3, and 150
after marker 4. A confirmed release is evidence only and awards no points by
itself.

## Ordering and failure behavior

Ground-truth and all three payload-state streams are accepted in exact
50,000,000 ns order after mission start for one canonical run ID. The first
duplicate, regression, or gap permanently marks the result incomplete; a later
suffix cannot repair it or hide a pose jump or concurrent attachment. Missing
required input likewise marks a run incomplete rather than causing corrective
control.

For `competition_v1`, the first current-run `FM1/STARTED` mission event stores
the authoritative `start_sim_time`. Release stability, settling, freshness,
and the 600-second deadline use differences between existing simulation
timestamps. Pre-start events earn no score. No epoch, timestamp rebase,
activation barrier, or cross-service clock synchronization is introduced.

Each release requires 2 simulated seconds of contiguous 20 Hz vehicle truth at
or above 10 m AGL, within 0.15 m of F2, and no faster than 0.10 m/s
horizontally. A payload component passes only after confirmed physical
detachment and 1 simulated second of contiguous grounded, low-speed truth with
the full rotated 0.1524 m square footprint inside the 0.9144 m F2 rectangle.
FM3 releases additionally require a confirmed physical attachment, a
non-teleporting pickup within 0.075 m, marker 3's center inside WA or marker 4's
center inside WM, capacity at most one, and marker order 3 then 4. A recognized
payload 2 settlement cannot occur after marker 3's pickup, payload 3 settlement
after marker 4's pickup, or payload 4 settlement after physical Home landing.
Completion requires physical Home XY/contact/low-speed truth to remain valid
through distinct ordered `HOME/DISARMED` and `HOME/COMPLETE` mission events at
or before 600 elapsed simulated seconds; only then can `score-finished` be
written. `HOME/DISARMED` is the downstream controller's current-run evidence
that its original DroneKit vehicle reported `armed is False` after its disarm
wait; `HOME/COMPLETE` alone is insufficient. A physically missed point
component produces an honest finalized partial score when stream continuity,
event grammar, physical ordering, and terminal evidence remain valid.

The committed `rules/descent_v1.json` has maximum 100 and freezes a safe
pre-impact downward-speed threshold of 1.0 m/s. The scorekeeper emits exactly
four ordered rule events (`descent.airborne_then_contact`,
`descent.touchdown_precision`, `descent.safe_preimpact_speed`, and
`descent.stable_contact`) followed by `score.finalized`. It persists the same
five events as JSONL. Only after both files are durably created and all five
reliable publications are acknowledged does it create
`.status/score-finished.json` with exactly `run_id`, `finished=true`, and the
final simulation timestamp. Existing evidence is never overwritten.

On incomplete, discontinuous, conflicting, or truncated input, the runtime
fails closed, persists an incomplete result, and writes the shared runtime
failure instead of `score-finished`. During finalization it stops output and
writes `.status/quiescence/scorekeeper.json`, then remains alive and silent
until orchestration commits a terminal state. Structured JSONL events provide
readiness, scenario, final-score, failure, and finalization diagnostics; wall
time appears only as observability metadata and deadlines.

## Prohibited outputs

The scorekeeper exposes no command, mode, actuator, force, constraint, pose,
velocity, electromagnet request, or physics-mutation output.

## Deferred decisions

- Optional diagnostic telemetry beyond authoritative ground truth
