# Artifacts Internal Interface

## Internal seams

The bundle builder consumes immutable artifact records and produces a manifest. Recorder adapters own ROS bag, image encoding, Docker-log capture, and filesystem validation. Tests replace adapters without changing manifest or lifecycle logic.

## Idempotency

Finalizing the same `run_id` repeatedly produces the same artifact inventory and never overwrites another run.

## Finalization barrier

Recorder adapters start before the first `/clock` and publish aggregate
readiness. On `FINALIZING`, the session waits for
`.status/runtime-frozen.json`, drains callbacks, closes FFmpeg inputs, stops
rosbag2 with escalation bounded by the remaining shared finalization budget,
validates recorder-local output, and atomically writes
`.status/artifacts-final.json`. The budget comes from one absolute deadline
measured by a monotonic wall clock; an adapter cannot restart it. This
quiescence barrier prevents required artifacts from changing during host
validation and manifest commit.

After `.control/terminal-committed.json`, artifacts publishes the final
`/simulation/artifact_status` with `manifest_path` set to `manifest.json`, then
exits without writing another required log or recording event.
