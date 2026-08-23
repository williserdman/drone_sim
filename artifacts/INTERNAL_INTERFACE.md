# Artifacts Internal Interface

## Internal seams

The bundle builder consumes immutable artifact records and produces a manifest. Recorder adapters own ROS bag, image encoding, Docker-log capture, and filesystem validation. Tests replace adapters without changing manifest or lifecycle logic.

## Idempotency

Finalizing the same `run_id` repeatedly produces the same artifact inventory and never overwrites another run.

## Ownership and trust boundary

Orchestration exclusively allocates and owns each run root. The artifacts
container is trusted: no untrusted same-UID code or hostile co-tenant can open,
rename, relink, or mutate entries inside that root. The host does not move the
run root or its `video` directory while recorder finalization is in progress.

Mode `0444` and read-only retained descriptors enforce the recorder lifecycle;
they are not Linux immutability guarantees against a hostile same-UID process
or a privileged co-tenant. Owned writers must quiesce before the manifest is
committed. If an encoder cannot be confirmed stopped within its primary
budget, the adapter publishes an independent, read-only recovery snapshot and
asynchronously kills and reaps the child that still owns only the original
anonymous inode. That child cannot mutate the named snapshot.

## Finalization barrier

Recorder adapters start before the first `/clock` and publish aggregate
readiness. On `FINALIZING`, the session waits for
`.status/runtime-frozen.json`, drains callbacks, closes FFmpeg inputs, stops
rosbag2 with escalation bounded by the remaining shared finalization budget,
validates recorder-local output, and atomically writes
`.status/artifacts-final.json`. The budget comes from one absolute deadline
measured by a monotonic wall clock; an adapter cannot restart it. This
quiescence barrier prevents required artifacts from changing during host
validation and manifest commit. Recorder adapters reserve recovery time inside
that same deadline, so a timeout or invalid/empty output has a named diagnostic
partial before the adapter returns to the barrier.

After `.control/terminal-committed.json`, artifacts publishes the final
`/simulation/artifact_status` with `manifest_path` set to `manifest.json`, then
exits without writing another required log or recording event.
