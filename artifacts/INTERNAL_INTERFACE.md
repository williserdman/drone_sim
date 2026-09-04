# Artifacts Internal Interface

## Internal seams

The bundle builder consumes immutable artifact records and produces a manifest. Recorder adapters own ROS bag, image encoding, Docker-log capture, and filesystem validation. Tests replace adapters without changing manifest or lifecycle logic.

`runtime_configuration` selects the unchanged descent base inventory or the
competition extension from immutable run configuration. The separate
`competition_score_validation` module imports no production competition
scorer; it recomputes continuous release and settlement windows, rotated
containment, ordered Home landing/disarm/completion, and the mission-relative
deadline from decoded bag facts.

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
readiness. The aggregate runtime polls the durable finalize request rather
than gating shutdown on its ROS `FINALIZING` callback. On first observation it
latches one absolute deadline, then waits for
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

Aggregate finalization monitors FFmpeg and rosbag health after readiness.
Diagnostic/callback failure writes durable first-wins runtime failure before
structured output. An unconfirmed live rosbag is a fail-closed barrier: no
`artifacts-final.json` is written, no recorder-local completeness is claimed,
and no validator hashes the mutable named bag. The silent process remains alive
so the controller reaches its work deadline, reserves timeout-invalid/null-hash
records, and then terminates the container during bounded teardown.

The synthetic aggregate runtime places separate reliable raw-image and metadata
arrivals in a fixed 40-pair buffer keyed by exact simulation stamp and frame ID.
It drains only contiguous exact pairs through the fail-closed video recorder;
the MCAP recorder archives only the metadata half. Once both inputs have drained
frame `N`, it publishes the internal camera-pair acknowledgement. Duplicate,
malformed, missing, or excess buffered inputs fail closed; finalization freezes
and clears the buffer without waiting for another acknowledgement.

Physical profiles use a buffer bounded by the exact configured camera-frame
count only to reorder the paired video inputs. They publish no pair
acknowledgement, make no ACK discovery check, and exert no physics backpressure.
The same configured count is passed to both video recorders and both final
video validators.

`VideoStreamRecorder` selects CPU `libx264` unless the deployment explicitly
sets `SIM_VIDEO_ENCODER=h264_nvenc`. The Vast GPU overlay grants the artifact
container the normal GPU reservation plus `/dev/nvidia-caps/nvidia-cap2`, which
is required by nested Docker for NVENC. Startup fails closed if FFmpeg does not
advertise the selected encoder. Both paths produce the same validated H.264
artifact contract.

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
For physical profiles, orchestration also passes `physical_gazebo=True`; this
selects nonempty server-log and valid native `state/state.tlog.zst` validation. The
default remains the Phase 2-compatible generic Gazebo evidence validator.
`finalize_with_result(...)` returns a frozen `FinalizationResult` containing
the published path and exact committed run ID, terminal status, and reason.
That return is the authority boundary; `finalize(...)` remains the compatible
path-only wrapper.

After `.control/terminal-committed.json`, artifacts exits without another ROS
publication, log event, or recording write.
