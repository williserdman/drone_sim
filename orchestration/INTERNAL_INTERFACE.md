# Orchestration Internal Interface

## Lifecycle model

The internal lifecycle module accepts validated events and returns a new immutable run state or a typed invalid-transition error. Docker, filesystem, and clock adapters remain behind internal seams so state-machine tests require no containers.

## Owned values

`run_id`, lifecycle state, terminal status and reason, configuration checksum, source revisions, image digests, simulation timing summary, and wall-clock infrastructure timing.

## Configuration seam

`resolve_run_config` is the only template-to-run boundary. It rejects a caller
supplied `run_id`, obtains exactly one UUID, resolves a relative `output_root`
against the invoking process, and writes the immutable resolved snapshot with
exclusive creation plus file and directory `fsync`.

The Phase 2 template and resolved runtime boundary accept only exact recording
geometry `320x240`, 20 FPS, `rgb8`. Nonmatching dimensions fail before Compose
construction rather than generalizing the synthetic source/video contract.

Run-directory allocation remains an operator-controller responsibility: it
rejects any pre-existing run directory before calling `write_resolved_config`.
The configuration writer itself rejects an existing `configuration/run.json`.

## Durable control seam

The host controller owns `.control/finalize-request.json`,
`.status/operator-state.json`, and `.control/terminal-committed.json`. Runtime
adapters own the remaining `.status` files. The controller consumes
`runtime-running.json` to persist the observable `RUNNING` state. For normal
completion it waits for source completion before requesting `FINALIZING`, then
waits for the `runtime-frozen.json` quiescence barrier and
`artifacts-final.json`. It commits the authoritative terminal manifest before
allowing terminal ROS notifications; those notifications cannot append to
required artifacts.

The runtime publishes `FINALIZING`, permanently stops its own output, and
writes `.status/quiescence/orchestration.json`. It then validates exact markers
from companion, `ardupilot_sitl`, Gazebo, electromagnet, and scorekeeper before
it alone writes aggregate `runtime-frozen.json`. Wrong/stale/duplicate/schema or
unsafe path evidence does not satisfy the barrier. The orchestration runtime
observes committed terminal facts and exits silently. The artifacts runtime
alone publishes the post-manifest live `ArtifactStatus`, then writes the
durable `terminal-notified` acknowledgement; neither action appends required
artifact data or stdout.

The controller converts resolved `finalization_wall_seconds` to one absolute
deadline using a monotonic wall clock. Every finalization wait and adapter call
receives the remaining time from that shared deadline; no step receives a fresh
timeout. It reserves two equal bounded slices of
`min(5 seconds, finalization_wall_seconds / 5)` from that same deadline: one
for a failure-manifest commit and one for `compose down`. Earlier validation
and capture use the deadline minus both reserves. A work timeout marks unfinished
artifacts invalid and attempts a `FAILED`/`ABORTED` manifest inside the manifest
reserve; teardown receives the actual remaining total budget.

Host-side log parsing, protocol reads, file/tree hashing, optional discovery,
manifest serialization, and publication consume cooperative deadline callbacks.
`ArtifactSession` accepts separate backward-compatible work and commit checks;
work exhaustion stops validation and emits explicit timeout-invalid records,
while manifest publication uses only the reserved commit slice.
Runtime-status waits pass the applicable cooperative check into every read and
check both before and after it. Quiescence/report waits use the work slice;
post-commit terminal notification uses the pre-teardown manifest slice and
cannot consume teardown reserve.

`ComposeRuntime` pins the absolute repository `compose.yaml`, disables implicit
`.env` loading, removes ambient Compose file/env-file/profile/project
selectors, and then activates only the `phase2` profile with
`COMPOSE_PROFILES=phase2`. Docker host, TLS, certificate, and context variables
remain available for daemon connectivity. Health observation runs exact
`ps --all --format json` and requires the frozen seven unique service names to
be present and running/restarting; missing, extra, duplicate, malformed,
exited, or unhealthy rows fail closed.

All seven Phase 2 services have explicit stable `:phase2` tags, so the unique
per-run project name and frozen `up --no-build` command resolve identical
prebuilt images. `foundation` is isolated in its own profile with a stable
`:phase1` tag and remains explicitly runnable for Phase 1 verification.

Returning a typed `FinalizationResult` from `ArtifactSession` marks the manifest
publication boundary; its status and reason are immediately immutable.
Post-publication path/read/deadline verification can add diagnostics but cannot
replace those terminal facts.
`terminal-committed`, terminal-notification, host-event output/close, and
teardown failures append diagnostics only. Host-event output is fail-once so a
broken stream cannot recursively prevent file evidence or finalization.

Once a manifest hard link is published, cooperative deadline expiry cannot
turn that commit into an uncommitted result. Publication resolves the named
bytes, completes parent-directory durability without another cooperative
check, and returns typed `FinalizationResult` authority. `status`, `abort`, and
`collect-results` prefer the descriptor-validated typed manifest result over
the mutable operator-status cache.
