# Scorekeeper Internal Interface

`drone_sim_scorekeeper.descent` is the pure same-process policy interface.
`GroundTruthSample`, `DescentRules`, rule results, score events, and the final
result are immutable. `DescentScorer.accept()` consumes one ordered sample and
`finalize()` returns an idempotent result without any ROS, control, or physics
dependency. Construction requires the config-derived expected ground-truth
sample count; a contiguous but truncated sequence is incomplete just like a
gap or overrun.

`load_descent_rules(path)` checksums the exact rules bytes.
`persist_score_outputs(run_directory, result)` creates, never overwrites,
`scoring/events.jsonl` and `scoring/result.json`. The result's
`finished_status()` returns only the exact current-run completion fact.

`drone_sim_scorekeeper.runtime.ScorekeeperRuntime` is the lifecycle seam. It
orders scenario and ground-truth input, persists evidence before publication,
flushes reliable score-event acknowledgements before completion, and owns
failure/quiescence transitions. `runtime_node` is the thin ROS 2 and durable
control-file adapter; all score decisions remain in simulation-time policy.
