# Artifacts External Interface

## Inputs

- Orchestration start and finalize lifecycle events
- `/clock`, both full image and frame-metadata streams, artifact status, ground
  truth, scenario events, score events, and run state
- Structured stdout from every module
- Gazebo server log and native state
- Scorekeeper result files

The fixed ROS subscriptions are `/clock` at best-effort depth 1,
`/simulation/run_state` at reliable transient-local depth 1,
`/simulation/artifact_status` at reliable transient-local depth 1,
`/simulation/ground_truth` at best-effort depth 10,
`/simulation/scenario_events` and `/simulation/score_events` at reliable depth
100, and `/camera/{onboard,observer}/{image_raw,frame_metadata}` at best-effort
depth 5. The metadata topics are `/camera/onboard/frame_metadata` and
`/camera/observer/frame_metadata`, both using
`simulation_interfaces/msg/FrameMetadata`. The exact subscriber overrides are
stored in `config/recording-qos.yaml`.

## Outputs

- Aggregate recorder readiness and final completeness on
  `/simulation/artifact_status`
- Durable `.status/artifacts-ready.json` and `.status/artifacts-final.json`
- `runs/<run_id>/manifest.json`
- Onboard and observer MP4 files, ROS 2 bag, logs, Gazebo state, configuration snapshots, and scoring files

The required bundle inventory is fixed as `configuration/`,
`gazebo/server.log`, `gazebo/state/`, `video/onboard.mp4`,
`video/observer.mp4`, `rosbag/`, one JSONL log for each of orchestration,
artifacts, companion, ArduPilot SITL, Gazebo, electromagnet, and scorekeeper,
plus `scoring/events.jsonl` and `scoring/result.json`.

Every owned process log line has `run_id`, `module`, `severity`, `event`,
`sim_timestamp`, and `wall_timestamp`; event-specific values are nested under
`fields` by the Phase 1 serializer.

## Failure behavior

Recorder failure is reported immediately. After simulation stops, finalization
uses the remaining budget of one shared bounded deadline measured by a
monotonic wall clock, writes the manifest atomically, and explicitly records
missing or invalid artifacts.

Host capture and `ArtifactSession` expose optional cooperative deadline checks
without changing existing callers. Parsing, per-line routing, file chunks,
tree entries, optional inventory, manifest encoding, and publication boundaries
check the supplied budget. Work-time exhaustion stops further validation and
records unfinished required paths as timeout-invalid before the reserved
manifest commit is attempted.

Artifacts does not begin draining until `.status/runtime-frozen.json` proves
all publishers are permanently quiescent. It closes both video pipelines and
the bag before writing `artifacts-final.json`. That report has exact top-level
keys `run_id`, `complete`, and `records`; the records list contains exactly one
record for each of `video/onboard.mp4`,
`video/observer.mp4`, and `rosbag`. Each record contains `relative_path`,
`status` (`valid`, `missing`, or `invalid`), `detail`, `size_bytes`, `sha256`,
and a nonempty `semantic` object describing the recorder-local validation.
Missing records, malformed facts, or host-recomputed size/checksum mismatches
fail closed. The bag deliberately ends with
`FINALIZING`; the host-written `manifest.json` is authoritative for terminal
status. After `.control/terminal-committed.json`, the final artifact-status
notification writes no required artifact data.
