# Scorekeeper module

[Project README](../README.md) · [Architecture](../docs/architecture.md) · [Runbook](../docs/runbook.md)

This module owns deterministic, run-scoped evaluation of authoritative physical
evidence for the frozen `descent_v1`, `competition_v1`, and
`competition_2027_v1` policies,
then persists and publishes their score results.

It is read-only with respect to the simulated system: it does **not** command the
aircraft, electromagnet, Gazebo, mission phases, or retry behavior. A mission
reporting success is not physical proof, and a complete or maximum score is not
the terminal run result; orchestration and the validated `manifest.json` decide
whether the evidence bundle is complete and valid.

## Code map

- [runtime_node.py](src/drone_sim_scorekeeper/runtime_node.py) is the process
  entry point, resolved-scenario selector, ROS translation layer, and lifecycle
  driver.
- [competition.py](src/drone_sim_scorekeeper/competition.py) is the pure competition
  evidence model and scorer.
- [vfs.py](src/drone_sim_scorekeeper/vfs.py) is the pure 2027 automatic judge,
  active-attempt clock, footprint evaluator, and scorer;
  [vfs_runtime.py](src/drone_sim_scorekeeper/vfs_runtime.py) routes its public
  evidence and decisions.
- [descent.py](src/drone_sim_scorekeeper/descent.py) is the pure descent scorer.
- [competition_runtime.py](src/drone_sim_scorekeeper/competition_runtime.py) and
  [runtime.py](src/drone_sim_scorekeeper/runtime.py) bind scorers to persistence,
  publication, failure, and quiescence through the private shared
  [_finalization.py](src/drone_sim_scorekeeper/_finalization.py) lifecycle.
- [models.py](src/drone_sim_scorekeeper/models.py) defines result contracts;
  [output.py](src/drone_sim_scorekeeper/output.py) creates no-clobber evidence.
- The shared [runtime status contract](../artifacts/src/artifacts/runtime_status.py)
  defines score completion and failure values. Runtime code writes them through
  the [container protocol adapter](../artifacts/src/artifacts/runtime_protocol.py).
- The installed command is defined in [pyproject.toml](pyproject.toml); Compose
  starts it in the [`scorekeeper-runtime` service](../compose.yaml).

## Interfaces and rules

The runtime consumes `/clock`, [GroundTruth](../ros_ws/src/simulation_interfaces/msg/GroundTruth.msg),
[RunState](../ros_ws/src/simulation_interfaces/msg/RunState.msg), and scenario-specific
[ScenarioEvent](../ros_ws/src/simulation_interfaces/msg/ScenarioEvent.msg),
[PayloadState](../ros_ws/src/simulation_interfaces/msg/PayloadState.msg),
[PayloadEvent](../ros_ws/src/simulation_interfaces/msg/PayloadEvent.msg), and
[MissionEvent](../ros_ws/src/simulation_interfaces/msg/MissionEvent.msg). The 2027
profile also consumes observed
[VehicleStatus](../ros_ws/src/simulation_interfaces/msg/VehicleStatus.msg) and
publishes correlated
[JudgeDecision](../ros_ws/src/simulation_interfaces/msg/JudgeDecision.msg),
continuous [AttemptStatus](../ros_ws/src/simulation_interfaces/msg/AttemptStatus.msg),
and [ScoreEvent](../ros_ws/src/simulation_interfaces/msg/ScoreEvent.msg). Judge
and attempt messages contain timing and evidence references, never exact payload
positions.
Topic selection and QoS live in
[runtime_node.py](src/drone_sim_scorekeeper/runtime_node.py), not this guide.
The VFS runtime joins each exact 20 Hz ground-truth and three-payload epoch with
the matching 10 Hz vehicle status when due. It applies truth, payload, then
status before publishing that epoch's attempt state. A phase check waits for its
exact physical epoch, retains its original timestamp and correlation, and fails
closed if the epoch remains incomplete for 500 ms of observed simulation time.

The exact scoring data authorities are
[competition_v1.json](rules/competition_v1.json) and
[competition_2027_v1.json](rules/competition_2027_v1.json), plus
[descent_v1.json](rules/descent_v1.json), enforced by their loaders and scorers.
Do not duplicate point allocations, timing windows, or physical thresholds in
documentation. The persisted schema is defined by
[ScoreResult](src/drone_sim_scorekeeper/models.py), while creation of
`scoring/events.jsonl` and `scoring/result.json` is implemented in
[output.py](src/drone_sim_scorekeeper/output.py). The runtime writes the typed
`score-finished` status only after it persists score evidence.

## Constraints worth preserving

- Score derives from ordered Gazebo truth, observed arming/mode/failsafe status,
  and confirmed payload and mission events. Estimated positions and success text
  do not establish physical placement.
- Competition vehicle and payload streams must remain contiguous on the ruleset's
  simulation-time grid after mission start. A gap, duplicate, regression,
  conflicting physical order, or missing terminal evidence makes scoring
  incomplete; a later suffix cannot repair it.
- Judge freshness remains an as-of rule: deferred checks use only evidence at or
  before the original request timestamp. Callback arrival order cannot substitute
  later physical facts or change the request's active-clock value.
- Payload release is evidence, not points by itself. Delivery requires physical
  detachment and settled geometry. Comp2026 Home completion requires physical
  landing truth through ordered `HOME/DISARMED` and `HOME/COMPLETE` events.
  VFS completion uses correlated phase checks against contact, complete landing
  footprint, altitude transitions, and observed disarm. Eligibility uses actual
  container ground contact, including tilted corner/edge support.
- Missing point components may yield an honest finalized partial score when the
  evidence grammar and terminal conditions remain valid.
- Evidence is persisted before reliable score-event publication is flushed and
  before `score-finished` is created. Existing score evidence is never replaced.
  Finalization then writes quiescence and produces no further output.
- Rejected same-run ROS evidence latches incomplete scoring before shutdown.
  Emergency finalization writes runtime failure and quiescence, never
  `score-finished`.

## Focused checks

Run from the repository root; these do not launch a mission:

```bash
uv run pytest scorekeeper/tests -v
uv run pytest tests/contracts -v
```

For interpreting a completed run, use the independent acceptance command and
prerequisites in the [runbook](../docs/runbook.md#independent-competition-acceptance).
