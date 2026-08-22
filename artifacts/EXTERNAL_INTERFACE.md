# Artifacts External Interface

## Inputs

- Orchestration start and finalize lifecycle events
- `/clock`, both full image streams, ground truth, scenario events, score events, and run state
- Structured stdout from every module
- Gazebo server log and native state
- Scorekeeper result files

## Outputs

- Recorder readiness
- Final artifact completeness report
- `runs/<run_id>/manifest.json`
- Onboard and observer MP4 files, ROS 2 bag, logs, Gazebo state, configuration snapshots, and scoring files

## Failure behavior

Recorder failure is reported immediately. Finalization uses bounded wall time after simulation stops, writes the manifest atomically, and explicitly records missing or invalid artifacts.
