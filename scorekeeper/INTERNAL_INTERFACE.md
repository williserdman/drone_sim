# Scorekeeper Internal Interface

`drone_sim_scorekeeper.models` owns immutable scorer-neutral `RuleResult`,
`ScoreEvent`, and `ScoreResult` contracts. `drone_sim_scorekeeper.descent`
retains the byte-compatible pure `descent_v1` policy. `GroundTruthSample` and
`DescentRules` remain immutable; `DescentScorer.accept()` consumes one ordered
sample and `finalize()` returns an idempotent result without any ROS, control,
or physics dependency. Construction requires the config-derived expected
ground-truth sample count; a contiguous but truncated sequence is incomplete
just like a gap or overrun.

`drone_sim_scorekeeper.competition` is the separate pure physical authority.
`CompetitionScorer` accepts immutable vehicle truth, three marker-specific
`PayloadStateSample` streams, confirmed `PayloadEventSample` rows, and ordered
`MissionEventSample` rows. It stores the first `FM1/STARTED` timestamp as
`start_sim_time_ns`, then compares input timestamps directly for release,
settling, freshness, and deadline windows. It never changes or rebases source
timestamps.

The evaluator independently joins confirmed events to physical attachment and
detachment state, exact-grid release windows, non-jumping pickups, capacity one,
and grounded low-speed settling. `_inside_f2` projects the payload's complete
0.1524 m square XY footprint through final quaternion yaw before checking the
0.9144 m F2 rectangle. Seven fixed rule results produce events 0 through 6;
event 7 is `score.finalized`.

`load_descent_rules(path)` checksums the exact rules bytes.
`persist_score_outputs(run_directory, result)` creates, never overwrites,
`scoring/events.jsonl` and `scoring/result.json`. The result's
`finished_status()` returns only the exact current-run completion fact.

`drone_sim_scorekeeper.runtime.ScorekeeperRuntime` remains the descent lifecycle
seam. `drone_sim_scorekeeper.competition_runtime.CompetitionScorekeeperRuntime`
provides the matching competition seam and waits until vehicle and all three
payload state streams have reached `source-finished`. Both persist evidence
before publication, flush reliable score-event acknowledgements before
completion, and own failure/quiescence transitions. `runtime_node` selects the
policy only from the resolved `scenario`; all score decisions remain in pure
simulation-time policy.

The ROS boundary publishes only `/simulation/score_events`. It has no vehicle,
electromagnet, Gazebo command, service client, or physics-mutation path.
