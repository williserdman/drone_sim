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

The controller converts resolved `finalization_wall_seconds` to one absolute
deadline using a monotonic wall clock. Every finalization wait and adapter call
receives the remaining time from that shared deadline; no step receives a fresh
timeout. It reserves `min(5 seconds, finalization_wall_seconds / 5)` from that
same deadline for `compose down`; earlier finalization work uses the deadline
minus that reserve, and teardown receives the actual remaining total budget.
