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

## Durable control seam

The host controller owns `.control/finalize-request.json`,
`.status/operator-state.json`, and `.control/terminal-committed.json`. Runtime
adapters own the remaining `.status` files. After requesting finalization, the
controller waits for source completion, requests `FINALIZING`, waits for the
`runtime-frozen.json` quiescence barrier, then waits for
`artifacts-final.json`. It commits the authoritative terminal manifest before
allowing terminal ROS notifications; those notifications cannot append to
required artifacts.
