# Artifacts Internal Interface

## Internal seams

The bundle builder consumes immutable artifact records and produces a manifest. Recorder adapters own ROS bag, image encoding, Docker-log capture, and filesystem validation. Tests replace adapters without changing manifest or lifecycle logic.

## Idempotency

Finalizing the same `run_id` repeatedly produces the same artifact inventory and never overwrites another run.

## Ownership and trust boundary

Orchestration exclusively allocates and owns each run root. The artifacts
container is trusted: no untrusted same-UID code or hostile co-tenant can open,
rename, relink, or mutate entries inside that root. The host does not move the
run root or its owned `video`, `logs`, or `logs/docker` directories while
recorder finalization is in progress.

Mode `0444` and read-only retained descriptors enforce the recorder lifecycle;
they are not Linux immutability guarantees against a hostile same-UID process
or a privileged co-tenant. Owned writers must quiesce before the manifest is
committed. If an encoder cannot be confirmed stopped within its primary
budget, the adapter publishes independent, read-only video and FFmpeg-log
recovery snapshots under the same absolute deadline and asynchronously kills
and reaps the child that still owns only the original anonymous inodes. That
child cannot mutate either named snapshot.

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
video partial and FFmpeg diagnostic log before the adapter returns to the
barrier.

The recorder-local report has exactly three path-keyed records for the two
videos and bag. A valid record carries the descriptor-stable byte count and
SHA-256/tree SHA-256 returned by its semantic validator plus a nonempty
`semantic` summary (video codec/pixel format/dimensions/rate/frame count, or
bag storage/topic/message summary). The host controller recomputes safe
size/checksum values and accepts semantic validity only when the report agrees;
it never replaces semantic validation with presence-only checks.

`DockerLogCapture(deadline_check=...)` checks before and after each command,
line, payload chunk, candidate write, link, and durability boundary.
`ArtifactSession(deadline_check=..., commit_deadline_check=...)` uses the first
callback for required/optional validation and the second for canonical manifest
validation and no-clobber publication. Existing callers may omit both. A work
timeout marks the current and remaining required records invalid without
continuing discovery or hashing; requested `ABORTED` is never upgraded.

After `.control/terminal-committed.json`, artifacts publishes the final
`/simulation/artifact_status` with `manifest_path` set to `manifest.json`, then
exits without writing another required log or recording event.
