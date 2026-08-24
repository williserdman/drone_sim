# Artifacts Plan

## Responsibility

Own per-run ROS 2 bag recording, onboard and observer MP4 encoding, Gazebo logs and state, structured module logs, scoring outputs, checksums, validation, and final bundle assembly.

## Non-responsibilities

- Producing camera images or ground truth
- Selecting run terminal status
- Calculating scores

## Implementation stages

1. Define bundle and manifest schemas.
2. Capture structured stdout into per-module JSONL files.
3. Record the ten fixed ROS 2 topics, including both full image and matching
   frame-metadata streams, using the private overrides in
   `artifacts/recording-qos.yaml`.
4. Encode both streams as 20-FPS H.264 MP4 files.
5. Preserve Gazebo native state, logs, configuration, and scoring results.
6. Publish aggregate readiness and completeness on
   `/simulation/artifact_status`, and persist the matching status files.
7. Cross the runtime-frozen quiescence barrier, drain and close recorders, then
   validate artifacts and return a completeness report within the remaining
   budget of one shared bounded deadline measured by a monotonic wall clock.

## Acceptance criteria

- Recorders are ready before simulation time advances.
- Finalization is idempotent and safe after clock stoppage.
- Every artifact has size, SHA-256 checksum, and validation status.
- Completed runs contain every required artifact.
- Failed and aborted runs explicitly enumerate incomplete artifacts.
- The bag ends at `FINALIZING`; `manifest.json` authoritatively records the
  terminal outcome after validation.

## Phase subplans

- Phase 1: schemas and synthetic log/bundle tests
- Phase 2: real recorder processes, encoding, and finalization
- Phases 3-6: Gazebo, companion, scenario, and scoring artifact integration
