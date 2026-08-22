# Artifacts External Interface

## Inputs

- Orchestration start and finalize lifecycle events
- `/clock`, both full image streams, ground truth, scenario events, score events, and run state
- Structured stdout from every module
- Gazebo server log and native state
- Scorekeeper result files

The fixed ROS subscriptions are `/clock` at best-effort depth 1,
`/simulation/run_state` at reliable transient-local depth 1,
`/simulation/ground_truth` at best-effort depth 10,
`/simulation/scenario_events` and `/simulation/score_events` at reliable depth
100, and both `/camera/onboard/image_raw` and
`/camera/observer/image_raw` at best-effort depth 5. Camera frames correlate
with `simulation_interfaces/msg/FrameMetadata`; the other simulation topics use
the corresponding shared `simulation_interfaces` message.

## Outputs

- Recorder readiness
- Final artifact completeness report
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

Recorder failure is reported immediately. Finalization uses bounded wall time after simulation stops, writes the manifest atomically, and explicitly records missing or invalid artifacts.
