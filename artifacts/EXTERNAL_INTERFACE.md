# Artifacts External Interface

## Inputs

- Orchestration start and finalize lifecycle events
- `/clock`, both full image streams for MP4 encoding, both frame-metadata
  streams for MCAP correlation, artifact status, ground truth, scenario events,
  score events, and run state
- Structured stdout from every module
- Gazebo server log and native state
- Scorekeeper result files

The fixed MCAP subscriptions are `/clock` at best-effort depth 1,
`/simulation/run_state` at reliable transient-local depth 4,
`/simulation/artifact_status` at reliable transient-local depth 2,
`/simulation/ground_truth` at best-effort depth 10,
`/simulation/scenario_events` and `/simulation/score_events` at reliable depth
100. The live video pipelines request raw camera topics at reliable, volatile depth
100 for physical production profiles and retain reliable, volatile depth 5 for
synthetic Phase 2. Raw images are encoded once into the two required H.264 MP4
files and are intentionally omitted from MCAP. The private rosbag recorder
requests the frame-metadata topics at reliable depth 100 so a bounded writer
stall cannot lose the timing and correlation evidence. The metadata topics are
`/camera/onboard/frame_metadata` and
`/camera/observer/frame_metadata`, both using
`simulation_interfaces/msg/FrameMetadata`. The exact private rosbag subscriber
overrides are stored in `artifacts/recording-qos.yaml`. The public
`ArtifactStatus` publisher remains reliable transient-local depth 1; the
private recorder requests depth 2 so its cache retains both startup samples
until rosbag takes them.

The metadata reliability request is scoped to the recorder path. A reliable
camera publisher remains compatible with later mission consumers that request
best effort; exact metadata and video acceptance does not rely on a best-effort
delivery promise. Pixel evidence lives in the videos, while MCAP retains the
contiguous frame IDs and simulation timestamps needed to align each video with
ground truth.

`BASE_TOPICS` is the eight-topic lean-video descent inventory. `competition_v1` adds
`/simulation/payload_state` (`PayloadState`, reliable volatile depth 100),
`/simulation/payload_events` (`PayloadEvent`, reliable transient-local depth
100), `/simulation/mission_events` (`MissionEvent`, reliable transient-local
depth 100), and `/competition/range/downward` (`LaserScan`, reliable volatile
depth 100). The competition bag requires IDs 2, 3, and 4 at every exact 50 ms
grid point. Video validation separately enforces the resolved 640x480 shape.
Payload-state timestamps must be monotonic independently for each marker ID;
delivery order may cross between IDs only when the exact three-marker grid
still proves every required sample is present.

Competition acceptance dispatches by `ruleset_id` to a physically independent
oracle. It decodes vehicle, payload, mission, event, and downward-range facts
and requires score events 0 through 7 plus `result.json` to match the seven
recomputed components exactly. It never accepts the production scorekeeper's
`complete` or point booleans as authority.

Phase 2 synthetic infrastructure also publishes a transport-only acknowledgement
on `/simulation/camera_pair_ack` after both exact camera pairs have drained into
their recorders. It reuses `simulation_interfaces/msg/FrameMetadata` with the
current canonical `run_id`, `stream="aggregate"`, contiguous `frame_id=N`, and
`sim_timestamp=(N+1)*50_000_000` nanoseconds. QoS is reliable, transient-local, depth
1. This acknowledgement is intentionally absent from the fixed eight-topic bag
inventory and models neither camera latency nor simulation time.

For every non-Phase-2 physical profile, the artifact runtime derives the exact
camera count from `simulation.duration_sim_seconds * 20` on the 50 ms grid.
It neither creates nor discovers `/simulation/camera_pair_ack`; recorder
readiness and physics advancement are independent of that synthetic transport.
Completed Phase 3 acceptance binds this configured cadence to the recorded
evidence: `/clock` spans exactly zero through the configured duration, both
frame-metadata streams and matching ground truth occupy exactly 50 ms through
that duration, both videos contain the configured frame count, `RUNNING` is
stamped at zero, and `FINALIZING` is stamped at the duration. Manifest
`start_ns`, `end_ns`, and `duration_ns` must describe that same public epoch
exactly.

## Outputs

- Aggregate recorder readiness and final completeness on
  `/simulation/artifact_status`
- Durable `.status/artifacts-ready.json` and `.status/artifacts-final.json`;
  the shared status protocol also validates the production readiness and
  completion facts owned by Gazebo, ArduPilot, companion, and scorekeeper.
  This includes the exact one-shot companion rendezvous fact
  `.status/mission-command-delivered.json` with
  `{run_id,command:"SET_GUIDED",sim_timestamp_ns:0,delivered:true}`
- `runs/<run_id>/manifest.json`
- Onboard and observer MP4 files, ROS 2 bag, logs, Gazebo state, configuration snapshots, and scoring files

Successful startup publishes two simulation-time-zero aggregate statuses before
the first clock. The first is exactly not-ready with missing recorder names
`onboard`, `observer`, and `rosbag`; the second is ready with an empty missing
list. The runtime waits for a matched-reader acknowledgment before the second
publish, and the private recorder cache has depth 2 so both are archived even
if rosbag takes them afterward. After the host commits the manifest and the
bag is closed, the artifacts runtime publishes one live reliable
transient-local final status derived through descriptor-safe manifest reading,
then writes `terminal-notified`. The final live sample does not mutate the bag,
required artifacts, or stdout.

The required bundle inventory is fixed as `configuration/`,
`gazebo/server.log`, `gazebo/state/`, `video/onboard.mp4`,
`video/observer.mp4`, `rosbag/`, one JSONL log for each of orchestration,
artifacts, companion, ArduPilot SITL, Gazebo, electromagnet, and scorekeeper,
plus `scoring/events.jsonl` and `scoring/result.json`.
For a physical profile, the coordinator constructs `ArtifactSession` with
`physical_gazebo=true`: `gazebo/server.log` must then be nonempty, and
`gazebo/state/` must be a nonempty safe tree containing an integrity-checked
regular `state.tlog.zst`. The default preserves the Phase 2 synthetic-state
contract; placeholders cannot satisfy completed physical-run validation.

Completed Phase 3 provenance contains exactly one source revision named
`drone_sim` and exactly one uniquely digested image record for each of the
seven image names configured by the production Compose file. Substituted,
missing, extra, or digest-aliased image records are not accepted.
Semantic acceptance compares those manifest values with provenance captured
independently by the caller before inspection. The host CLI requires
`--expected-source-revision`, `--expected-source-dirty=true|false`, and one
`--expected-image-digest=IMAGE=SHA256` option for each exact Phase 3 image.
SHA256 values are the lowercase 64-hex Docker image IDs without the
`sha256:` prefix. The host forwards the values as arguments to the isolated,
read-only semantic container; the manifest and module logs are never used as
the authority for their own expected provenance.

Completed Phase 3 log acceptance requires the Gazebo runtime's public-epoch
`ActivateOutput` action together with its readiness, pause, source-finished,
finalization, stop, and quiescence actions. The legacy private `RequestSteps`
action is not evidence that public camera and ground-truth output was enabled.

Every owned process log line has `run_id`, `module`, `severity`, `event`,
`sim_timestamp`, and `wall_timestamp`; event-specific values are nested under
`fields` by the Phase 1 serializer.

## Failure behavior

Recorder failures, including premature post-readiness FFmpeg or rosbag exits,
are first-wins durably reported in `.status/runtime-failure.json` before any
structured diagnostic is attempted. After simulation stops, finalization
uses the remaining budget of one shared bounded deadline measured by a
monotonic wall clock, writes the manifest atomically, and explicitly records
missing or invalid artifacts.

The artifacts runtime polls the authoritative
`.control/finalize-request.json` directly and latches its deadline on first
observation. Delivery of the ROS `FINALIZING` sample is retained for lifecycle
observation but is not the recorder-shutdown trigger, so callback delivery
cannot leave live recorders waiting until the host deadline.

Host capture and `ArtifactSession` expose optional cooperative deadline checks
without changing existing callers. Parsing, per-line routing, file chunks,
tree entries, optional inventory, manifest encoding, and publication boundaries
check the supplied budget. Work-time exhaustion stops further validation and
records unfinished required paths as timeout-invalid before the reserved
manifest commit is attempted.

`read_regular_file_bytes(run_directory, relative_path)` returns a
`ValidationResult` plus bounded bytes only after a retained `O_NOFOLLOW`
descriptor walk proves a regular single-link current-run file with stable
pre/post identity. Scoring provenance uses this interface rather than following
path-based substitutions.

`ArtifactSession.finalize_with_result(FinalizationInput)` returns the immutable
published path, run ID, terminal status, and reason in a `FinalizationResult`.
Consumers that make terminal decisions use that typed result so a later read
failure cannot contradict an already committed manifest. The existing
`finalize(FinalizationInput) -> Path` call remains supported.

Artifacts trusts only the orchestration-owned aggregate
`.status/runtime-frozen.json`; individual module markers cannot begin capture
or draining. It closes both video pipelines and the bag before writing
`artifacts-final.json`. That report has exact top-level
keys `run_id`, `complete`, and `records`; the records list contains exactly one
record for each of `video/onboard.mp4`,
`video/observer.mp4`, and `rosbag`. Each record contains `relative_path`,
`status` (`valid`, `missing`, or `invalid`), `detail`, `size_bytes`, `sha256`,
and a nonempty `semantic` object describing the recorder-local validation.
Missing records, malformed facts, or host-recomputed size/checksum mismatches
fail closed. If rosbag remains alive after bounded escalation, artifacts writes
no final report/completeness claim and stays silent/alive until controller
teardown; the controller must not hash that mutable named bag. The bag
deliberately ends with
`FINALIZING`; the host-written `manifest.json` is authoritative for terminal
status. After writing `artifacts-final.json`, the artifacts runtime remains
silent while awaiting `.control/terminal-committed.json`; after that commit it
publishes only the final live aggregate status, writes `terminal-notified`, and
then exits.
