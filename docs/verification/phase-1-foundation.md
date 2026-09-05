# Phase 1 Foundation Verification

Historical evidence for the revisions below, not today's runtime contract.
The referenced plans and interface documents are retained in Git history;
inspect them with `git show 7064e86:<repository-relative-path>`.
Current ownership and contracts start at [architecture](../architecture.md).

## Current post-fix verification (authoritative)

- Tested implementation commit: `7064e860b395a1cb21c3d8c40064b0bbc07e912f`
- Verification completed: `2026-08-22T16:56:48Z`
- Built foundation image digest:
  `sha256:7431edf66d1fb49a52898b3d918bbbcac2c3fefed9d803380be27a2b13296f12`
- Authorities compared: `docs/superpowers/specs/2026-08-22-runnable-simulation-design.md`,
  `docs/superpowers/plans/2026-08-22-phase-1-foundation.md`, the root
  interface documents, and each affected module interface document.

The tested implementation commit contains the DDS observer, stdout/file sink
comparison, finite-score validation, bounded integration timeout, and exact ROS
field checks described below. This evidence-only document commit follows that
tested implementation commit, avoiding a self-referential commit identity.

### Post-fix command evidence

| Command | Exit | Evidence |
| --- | ---: | --- |
| `make test` | 0 | 58 unit/contract tests passed and 2 Docker Compose integration tests passed; 60 total, 0 failed |
| `docker compose config --quiet` | 0 | Compose configuration accepted with no output |
| `git diff --check` | 0 | No whitespace errors |
| `docker image inspect phase-1-foundation-foundation:latest --format '{{.Id}}'` | 0 | Returned the image digest recorded above |
| `git status --short --untracked-files=all` | 0 | No tracked implementation changes after the fix commit |
| `test ! -e companion/comp2026` | 0 | Confirmed the nested repository was not materialized by Phase 1 |
| `git diff --name-only dbd7d63..7064e86 -- companion/comp2026` | 0 | Empty output; the final-review fix did not add or modify the nested repository path |

The `make test` result consists of:

- 9 configuration tests;
- 10 lifecycle tests;
- 16 manifest tests;
- 16 structured-log tests;
- 7 ROS source-contract tests;
- 2 Docker Compose foundation integration tests.

## Superseded original gate (historical only)

The original Phase 1 gate below predates the final-review fixes. It is retained
only as historical evidence and must not be used to support the post-fix DDS,
stdout/file equality, finite-score, timeout, or exact-field claims in this
document.

- Tested source commit: `9a4fb51b4b649ca05bdc9c35fbfae99a3ef567c5`
- Verification window: `2026-08-22T16:21:34Z` through `2026-08-22T16:23:46Z`
- Built foundation image digest:
  `sha256:4715ba350468c424bbc211c44f4c72ba6a196e8d6ec21209f13fd62cba2a1be4`
- Test result: 50 unit/contract tests and 1 Docker Compose integration
  test passed; 51 total, 0 failed.

## Contract alignment

The generated package name is `simulation_interfaces`. Its six interface names
and fixed fields are:

| Interface | Fields after any constants |
| --- | --- |
| `RunState` | `run_id`, `sim_timestamp`, `state`, `reason`, `config_sha256` |
| `FrameMetadata` | `run_id`, `sim_timestamp`, `frame_id`, `stream` |
| `GroundTruth` | `run_id`, `sim_timestamp`, `vehicle_id`, `pose`, `twist`, `in_contact` |
| `ScenarioEvent` | `run_id`, `sim_timestamp`, `event_id`, `magnet_id`, `state` |
| `ScoreEvent` | `run_id`, `sim_timestamp`, `event_id`, `event_type`, `value`, `evidence_ref` |
| `ArtifactStatus` | `run_id`, `sim_timestamp`, `ready`, `complete`, `missing`, `manifest_path` |

`RunState` declares the exact numeric order `CREATED=0`, `STARTING=1`,
`READY=2`, `RUNNING=3`, `FINALIZING=4`, `COMPLETED=5`, `FAILED=6`, and
`ABORTED=7`. The domain lifecycle tests cover the success path, failure and
abort from `STARTING`, `READY`, and `RUNNING`, finalization failure, invalid
transition diagnostics, and rejection of every event from terminal states.
The synthetic integration observes DDS-delivered states `STARTING`, `READY`,
`RUNNING`, `FINALIZING`, and `COMPLETED` at deterministic simulation times.
Its concurrent ROS node waits for bidirectional discovery, subscribes to both
foundation topics, and records every received message before publisher exit.

The fixed topic/QoS contract is:

| Topic | QoS |
| --- | --- |
| `/clock` | Best effort, depth 1 |
| `/simulation/run_state` | Reliable, transient local, depth 1 |
| `/simulation/ground_truth` | Best effort, depth 10 |
| `/simulation/scenario_events` | Reliable, depth 100 |
| `/simulation/score_events` | Reliable, depth 100 |
| `/camera/onboard/image_raw` | Best effort, depth 5 |
| `/camera/observer/image_raw` | Best effort, depth 5 |

Phase 1 executes the synthetic `/clock` and `/simulation/run_state` publishers
and a concurrent observer with compatible subscriptions. The integration test
asserts the exact three received clock values, the exact five received run-state
messages in order, and the discovered publisher reliability and durability
against the observer's requested QoS. The remaining topic/QoS entries are fixed
contracts for later phases, not evidence that their producers or consumers
exist yet.

The structured-log common fields are exactly `run_id`, `module`, `severity`,
`event`, `sim_timestamp`, and `wall_timestamp`; the Phase 1 serializer nests
event-specific values under `fields`. Tests cover compact valid JSON Lines,
UTC timestamps, nullable simulation time, field-collision rejection,
non-finite-number rejection, flush behavior, and equality between the
foundation process's stdout and file event streams.

The required artifact categories are configuration, Gazebo server log and
native state, onboard and observer MP4, ROS bag, JSONL logs for orchestration,
artifacts, companion, ArduPilot SITL, Gazebo, electromagnet, and scorekeeper,
score events, and score result. Phase 1 tests their manifest inventory and
validation rules with synthetic filesystem data. Score summaries reject NaN
and positive or negative infinity at the domain boundary, and manifest JSON is
encoded with non-standard numeric tokens disabled. Phase 1 does not produce a
full run bundle.

## Explicit non-claims

This gate does not claim runnable or validated Gazebo behavior, ArduPilot SITL
behavior, nested companion behavior, electromagnet behavior, scorekeeper or
scoring behavior, camera/video recording, ROS bag recording, or complete
artifact finalization. It does not claim a real mission, a maximum score, a
production orchestration service, Gazebo-authoritative `/clock`, or
ArduPilot-Gazebo lockstep. Those remain later-phase integration gates.
