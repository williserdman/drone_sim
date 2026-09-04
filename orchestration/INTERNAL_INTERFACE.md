# Orchestration Internal Interface

## Lifecycle model

The internal lifecycle module accepts validated events and returns a new immutable run state or a typed invalid-transition error. Docker, filesystem, and clock adapters remain behind internal seams so state-machine tests require no containers.

## Owned values

`run_id`, lifecycle state, terminal status and reason, configuration checksum, source revisions, image digests, simulation timing summary, and wall-clock infrastructure timing.

`_source_revisions` spends one shared startup deadline on fixed parent then
nested `rev-parse HEAD` and porcelain status calls. `ComposeRuntime` passes only
the nested revision as `SIM_COMP2026_REVISION`, inspects the already-built OCI
label, and refuses launch on disagreement. Provenance records observed dirty
booleans; it never cleans, ignores, stages, or rewrites user-owned files.

## Configuration seam

`resolve_run_config` is the only template-to-run boundary. It rejects a caller
supplied `run_id`, obtains exactly one UUID, resolves a relative `output_root`
against the invoking process, and writes the immutable resolved snapshot with
exclusive creation plus file and directory `fsync`.

The Phase 2 template and resolved runtime boundary accept only exact recording
geometry `320x240`, 20 FPS, `rgb8`. An omitted `runtime_profile` normalizes to
`phase2` and rejects `simulation`, preserving the synthetic regression seam.
The repository default explicitly selects `phase3`, which requires an unsigned
32-bit seed, a duration on the exact 50,000,000 ns camera grid, and
`target_real_time_factor=0.1`. Duration conversion uses `Decimal(str(value))`
before integer nanoseconds; no binary-float multiplication participates in
frame-count or divisibility decisions. All invalid values fail before Compose
construction.

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

`RunConfig.topology` returns one immutable `RuntimeTopology` containing the
validated profile and exact service-to-module ownership tuple. Phase 2 contains
the original seven synthetic services. Phase 3 selects the production seven:
orchestration, artifacts, companion, ArduPilot SITL, Gazebo, electromagnet, and
scorekeeper. The controller passes that value to `ComposeRuntime` and uses the
same ownership for health checks, log capture, image-digest discovery, service
stopping, and exact-service validation.

The production RunState discovery barrier requires only actual ROS consumers:
artifacts, companion, Gazebo, electromagnet, scorekeeper, and the rosbag
recorder. ArduPilot SITL has no RunState subscription and is deliberately not
invented as a ROS node; `.status/ardupilot-ready.json` remains its durable
startup gate. Phase 3 publishes `READY` after infrastructure readiness, which
lets Gazebo and ArduPilot advance in private lockstep warmup. The runtime stays
`READY` until current-run `ardupilot-ready.json`, `companion-ready.json`, and
exact `mission-ready.json` all validate. It then publishes `RUNNING` and writes
`runtime-running.json` at public time zero. Gazebo arms the configured Phase 3
native epoch before its exact target at that boundary, so host-sensitive controller boot
cannot consume simulated mission time. Phase 2 retains its clock-triggered
transition because it has no durable flight-readiness tuple.

`ComposeRuntime` pins the absolute repository `compose.yaml`, disables implicit
`.env` loading, removes ambient Compose file/env-file/profile/project
selectors, and sets `COMPOSE_PROFILES` only from the immutable topology. Docker
host, TLS, certificate, and context variables remain available for daemon
connectivity. The explicit `SIM_COMPOSE_OVERLAY=gpu` selector adds only the
repository-owned `compose.gpu.yaml`; any other overlay value fails closed and
the selector is not forwarded to services. Health observation runs exact
`ps --all --format json` and
requires the selected seven unique service names to be present and
running/restarting; missing, extra, duplicate, malformed, exited, or unhealthy
rows fail closed. No operation reads an ambient topology selector.

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
