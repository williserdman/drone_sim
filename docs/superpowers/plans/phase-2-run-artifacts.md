# Phase 2 Run Lifecycle and Artifacts Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce completed, failed, and aborted synthetic runs through an operator CLI, with recorder readiness before the first simulation clock, two playable 20-FPS H.264 MP4s, a readable ROS 2 bag, preserved structured logs, validated checksums, and an atomic terminal manifest.

**Architecture:** A host-side synchronous operator owns Docker Compose, run-directory allocation, log capture, offline validation, and the atomic manifest commit; it never schedules simulated events. ROS 2 containers own lifecycle publication, aggregate artifact readiness, synthetic simulation-time publishers, rosbag2, and two FFmpeg pipelines. A filesystem control/status protocol survives a stopped simulation clock, while `/simulation/run_state` and `/simulation/artifact_status` remain the live ROS contracts.

**Tech Stack:** Ubuntu 24.04 LTS, ROS 2 Jazzy, Python 3.12, Fast DDS, rosbag2 with MCAP storage, FFmpeg/libx264, H.264/yuv420p MP4, Docker Compose v2, pytest, JSON Schema Draft 2020-12

**Spec:** `docs/superpowers/specs/2026-08-22-runnable-simulation-design.md`

## Global Constraints

- Phase 2 is synthetic run lifecycle and artifact recording only; Gazebo is Phase 3 and ArduPilot lockstep is Phase 4.
- Do not modify or push `companion/comp2026` in this phase.
- Generate `run_id` and the resolved immutable configuration before any Compose service starts.
- Reject an existing run directory; no command may overwrite a previous run.
- Recorder readiness is required before the first `/clock` message.
- Synthetic image timestamps are exact integer nanoseconds at 20 frames per simulated second; wall time never determines their values.
- `/clock` remains best effort depth 1, lifecycle and aggregate artifact status are reliable transient-local depth 1, and both image and metadata streams are best effort depth 5.
- The ROS bag records complete image payloads, not thumbnails or filenames.
- The bag deliberately records lifecycle through `FINALIZING`; `manifest.json` is the authoritative terminal commit because successful terminal status depends on closing and validating the bag.
- Finalization uses bounded monotonic wall time after publishers become quiescent.
- A requested `COMPLETED` outcome is downgraded to `FAILED` on any required-artifact validation failure; `FAILED` and `ABORTED` never upgrade.
- Required logs and recordings become immutable before `manifest.json` is atomically published. Post-commit terminal ROS notifications do not write required artifacts.
- Completed, failed, and aborted runs preserve all recoverable output and explicitly inventory missing or invalid output.
- Every owned process emits structured JSON Lines with `run_id`, `module`, `severity`, `event`, `sim_timestamp`, and `wall_timestamp`; third-party stdout is preserved separately as raw Docker logs.
- Use behavioral lessons from the legacy recorder only: integer simulation timestamps, fail-closed validation, no-overwrite allocation, durable JSONL, and file-plus-directory `fsync`. Do not copy its 5-Hz JPEG archive, ticket-specific acceptance logic, coupled shell wrapper, or wall-time scheduling.
- Pin new Phase 2 images to `ros:jazzy-ros-base@sha256:2589a8fba5257307857890173c069852c2abf913a0be7970f172478baecb09e4`; record the resulting application-image digests in every run manifest.

## Fixed Phase 2 Protocol

### Operator commands

```text
uv run drone-sim start --config PATH
uv run drone-sim status RUN_ID [--output-root PATH]
uv run drone-sim abort RUN_ID [--output-root PATH]
uv run drone-sim collect-results RUN_ID [--output-root PATH]
```

`start` owns the foreground Compose run. It exits `0` for `COMPLETED`, `1` for `FAILED`, and `130` for `ABORTED`. `status`, `abort`, and `collect-results` are safe concurrent commands that communicate only through the run directory. Their output root defaults to resolved `runs`; callers using a custom template output root pass the same absolute path explicitly.

### ROS topics fixed in this phase

| Topic | Message | QoS |
| --- | --- | --- |
| `/simulation/artifact_status` | `simulation_interfaces/msg/ArtifactStatus` | Reliable, transient local, depth 1 |
| `/camera/onboard/frame_metadata` | `simulation_interfaces/msg/FrameMetadata` | Best effort, depth 5 |
| `/camera/observer/frame_metadata` | `simulation_interfaces/msg/FrameMetadata` | Best effort, depth 5 |

`ArtifactStatus` is aggregate for the whole artifact subsystem. Before the first clock its simulation timestamp is zero. Startup publishes `ready=false, complete=false` with missing recorder names, then `ready=true, complete=false, missing=[]`. The post-manifest notification sets `complete` to whether every required artifact validated, lists sorted missing/invalid relative paths in `missing`, and sets portable run-relative `manifest_path` to `manifest.json`.

The explicit bag topic set is:

```text
/clock
/simulation/run_state
/simulation/artifact_status
/simulation/ground_truth
/simulation/scenario_events
/simulation/score_events
/camera/onboard/image_raw
/camera/onboard/frame_metadata
/camera/observer/image_raw
/camera/observer/frame_metadata
```

The bag recorder does not use `--use-sim-time`: ROS 2 Jazzy suppresses pre-clock writes in that mode, which would discard readiness evidence. Simulation time remains explicit in `/clock`, image headers, and custom-message timestamps.

### Filesystem control and status

Every JSON file below is written to a collision-safe temporary sibling, flushed, file-`fsync`ed, replaced, and followed by directory `fsync`:

```text
runs/<run_id>/.control/finalize-request.json
runs/<run_id>/.control/terminal-committed.json
runs/<run_id>/.status/operator-state.json
runs/<run_id>/.status/artifacts-ready.json
runs/<run_id>/.status/runtime-running.json
runs/<run_id>/.status/source-finished.json
runs/<run_id>/.status/runtime-failure.json
runs/<run_id>/.status/runtime-frozen.json
runs/<run_id>/.status/artifacts-final.json
runs/<run_id>/.status/terminal-notified.json
```

Finalization order is fixed:

1. Host selects the requested terminal outcome and writes `finalize-request.json`.
2. The ROS orchestrator publishes `FINALIZING`; synthetic publishers stop permanently and write `runtime-frozen.json`.
3. Artifacts drain callbacks, close both FFmpeg inputs, stop rosbag2 with `SIGINT` and bounded `TERM`/`KILL` escalation, rename valid `.partial` videos, validate recorder-local output, and write `artifacts-final.json`. The report has exact path-keyed records for both videos and the bag with status/detail, stable byte count and checksum, and nonempty semantic facts; the host rejects absent, stale, or mismatched records.
4. Host captures raw Docker logs and partitions structured events, validates the complete bundle, downgrades invalid completion to `FAILED`, closes the orchestration log, and atomically commits `manifest.json`.
5. Host writes `terminal-committed.json`; the ROS orchestrator publishes the terminal `RunState`, artifacts publishes final `ArtifactStatus`, both write no further required artifact data, and services exit.
6. Host captures no new required data, removes Compose containers/network, and leaves the run directory intact.

## File Structure

```text
config/
├── default-run.json
├── run-template.schema.json
├── run.schema.json
└── recording-qos.yaml
pyproject.toml
uv.lock
orchestration/
├── Dockerfile
├── pyproject.toml
├── src/orchestration/
│   ├── __main__.py
│   ├── cli.py
│   ├── controller.py
│   ├── runtime_node.py
│   ├── status_store.py
│   └── _adapters/compose.py
└── tests/
    ├── test_cli.py
    ├── test_controller.py
    └── test_status_store.py
artifacts/
├── Dockerfile
├── schemas/manifest.schema.json
├── src/artifacts/
│   ├── recorder_node.py
│   ├── session.py
│   ├── validation.py
│   └── _adapters/
│       ├── docker_logs.py
│       ├── rosbag.py
│       └── video.py
└── tests/
    ├── test_docker_logs.py
    ├── test_rosbag_adapter.py
    ├── test_session.py
    ├── test_validation.py
    └── test_video_adapter.py
tests/phase2/
├── Dockerfile
├── entrypoint.sh
├── module_stub.py
├── scoring.json
├── synthetic_electromagnet.py
├── synthetic_gazebo.py
└── synthetic_scorekeeper.py
tests/integration/test_phase2_runtime_contract.py
tests/integration/test_phase2_compose.py
docs/verification/phase-2-run-artifacts.md
```

---

### Task 1: Freeze Phase 2 Contracts and Resolve Operator Configuration

**Files:**
- Create: `config/run-template.schema.json`
- Create: `config/recording-qos.yaml`
- Modify: `config/default-run.json`
- Modify: `config/run.schema.json`
- Modify: `orchestration/src/orchestration/config.py`
- Modify: `orchestration/src/orchestration/__init__.py`
- Modify: `orchestration/tests/test_config.py`
- Modify: `EXTERNAL_INTERFACE.md`
- Modify: `INTERNAL_INTERFACE.md`
- Modify: `orchestration/PLAN.md`
- Modify: `orchestration/EXTERNAL_INTERFACE.md`
- Modify: `orchestration/INTERNAL_INTERFACE.md`
- Modify: `artifacts/PLAN.md`
- Modify: `artifacts/EXTERNAL_INTERFACE.md`
- Modify: `artifacts/INTERNAL_INTERFACE.md`

**Interfaces:**
- Consumes: Phase 1 `RunConfig`, shared ROS messages, and the approved Phase 2 defaults
- Produces: frozen `RecordingConfig`, `RunTemplate`, `resolve_run_config(path, run_id_factory)`, resolved snapshot JSON, operator command names, artifact-status binding, metadata-topic bindings, and finalization protocol

- [ ] **Step 1: Write failing template-resolution and contract tests**

Add tests that load `config/default-run.json` as a template without `run_id`, inject `lambda: UUID("00000000-0000-4000-8000-000000000222")`, and expect:

```python
resolved = resolve_run_config(DEFAULT_TEMPLATE, run_id_factory=fixed_uuid)
assert resolved.run_id == "00000000-0000-4000-8000-000000000222"
assert resolved.recording == RecordingConfig(
    width_px=320,
    height_px=240,
    fps=20,
    encoding="rgb8",
)
assert resolved.startup_wall_seconds == 120
assert resolved.finalization_wall_seconds == 120
```

Assert template and resolved schemas reject odd dimensions, `fps != 20`, encodings other than `rgb8`, booleans in integer fields, nonpositive deadlines, unknown keys, and a caller-supplied template `run_id`. Assert `write_resolved_config(run_dir, resolved)` creates exactly `configuration/run.json`, validates against `run.schema.json`, and refuses an existing file.

- [ ] **Step 2: Run the focused tests and verify RED**

Run:

```bash
uv run pytest orchestration/tests/test_config.py -v
```

Expected: failures because `RecordingConfig`, `RunTemplate`, `resolve_run_config`, `run-template.schema.json`, and the fixed Phase 2 bindings do not exist.

- [ ] **Step 3: Implement immutable template resolution**

Use frozen dataclasses with these fields:

```python
@dataclass(frozen=True)
class RecordingConfig:
    width_px: int
    height_px: int
    fps: int
    encoding: str

@dataclass(frozen=True)
class RunTemplate:
    world: str
    vehicle: str
    mission: str
    scenario: str
    output_root: Path
    max_wall_seconds: int
    startup_wall_seconds: int
    finalization_wall_seconds: int
    recording: RecordingConfig
```

Extend `RunConfig` with the same deadline and recording fields. `resolve_run_config` validates the template, obtains one UUID from `run_id_factory`, creates `RunConfig`, and computes `config_sha256` from canonical resolved JSON excluding the derived checksum. `write_resolved_config` uses exclusive creation, flush, file `fsync`, and directory `fsync`.

Set the default template to competition/iris/descent/maximum_score, output root `runs`, overall wall limit `3600`, startup/finalization limits `120`, and recording `{width_px: 320, height_px: 240, fps: 20, encoding: "rgb8"}`.

- [ ] **Step 4: Record the fixed external contracts**

Document the four CLI commands, `/simulation/artifact_status`, both metadata topics, status-file protocol, quiescence barrier, and the fact that the bag ends at `FINALIZING` while `manifest.json` is authoritative for terminal status. Remove the matching deferred-decision bullets and preserve the existing forbidden communication paths and simulation-time rules.

Create `config/recording-qos.yaml` with these exact subscriber overrides:

```yaml
/clock: &clock_qos
  history: keep_last
  depth: 1
  reliability: best_effort
  durability: volatile
/simulation/run_state: &latched_state_qos
  history: keep_last
  depth: 1
  reliability: reliable
  durability: transient_local
/simulation/artifact_status: *latched_state_qos
/simulation/ground_truth:
  history: keep_last
  depth: 10
  reliability: best_effort
  durability: volatile
/simulation/scenario_events: &event_qos
  history: keep_last
  depth: 100
  reliability: reliable
  durability: volatile
/simulation/score_events: *event_qos
/camera/onboard/image_raw: &image_qos
  history: keep_last
  depth: 5
  reliability: best_effort
  durability: volatile
/camera/onboard/frame_metadata: *image_qos
/camera/observer/image_raw: *image_qos
/camera/observer/frame_metadata: *image_qos
```

- [ ] **Step 5: Verify Task 1**

Run:

```bash
uv run pytest orchestration/tests/test_config.py tests/contracts/test_ros_interfaces.py -v
git diff --check
```

Expected: all focused tests pass and no interface binding remains ambiguous.

- [ ] **Step 6: Commit Task 1**

```bash
git add config orchestration/src/orchestration/config.py orchestration/src/orchestration/__init__.py orchestration/tests/test_config.py EXTERNAL_INTERFACE.md INTERNAL_INTERFACE.md orchestration artifacts/PLAN.md artifacts/EXTERNAL_INTERFACE.md artifacts/INTERNAL_INTERFACE.md
git commit -m "feat: freeze phase 2 runtime contracts"
```

### Task 2: Deepen Bundle Validation and Atomic Manifest Finalization

**Files:**
- Create: `artifacts/src/artifacts/validation.py`
- Create: `artifacts/src/artifacts/session.py`
- Create: `artifacts/tests/test_validation.py`
- Create: `artifacts/tests/test_session.py`
- Modify: `artifacts/src/artifacts/manifest.py`
- Modify: `artifacts/src/artifacts/__init__.py`
- Modify: `artifacts/schemas/manifest.schema.json`
- Modify: `artifacts/tests/test_manifest.py`

**Interfaces:**
- Consumes: immutable resolved configuration, required bundle paths, terminal request, validator results, source revisions, image digests, and timing summaries
- Produces: `ValidationStatus`, `ValidationResult`, `validate_regular_file`, `validate_tree`, `FinalizationInput`, `ArtifactSession.finalize`, expanded `RunManifest`, and durable idempotent manifest commit

- [ ] **Step 1: Write failing filesystem validation tests**

Test regular files and directory trees with hand-derived SHA-256 values. A directory digest is SHA-256 over sorted UTF-8 rows of:

```text
<relative-posix-path>\0<size-bytes>\0<file-sha256>\n
```

and its size is the sum of contained regular-file sizes. Assert empty required trees, symlinks, FIFOs, path escapes, unreadable files, and a file where a directory is required return `INVALID` with a stable diagnostic. Missing paths return `MISSING`; valid nonempty paths return `VALID`.

Use these exact domain types:

```python
class ValidationStatus(str, Enum):
    VALID = "valid"
    MISSING = "missing"
    INVALID = "invalid"

@dataclass(frozen=True)
class ValidationResult:
    status: ValidationStatus
    size_bytes: int | None
    sha256: str | None
    detail: str
```

- [ ] **Step 2: Write failing manifest/session tests**

Construct a `FinalizationInput` with exact fields:

```python
@dataclass(frozen=True)
class FinalizationInput:
    run_id: str
    requested_terminal: str
    reason: str
    sim_start_ns: int | None
    sim_end_ns: int | None
    wall_started_at: datetime
    wall_ended_at: datetime
    source_revisions: tuple[SourceRevision, ...]
    image_digests: tuple[ImageDigest, ...]
    configuration_records: tuple[ConfigurationRecord, ...]
    achieved_score: float | None
    maximum_available_score: float | None
    scoring_checksum: str | None
    evidence_paths: tuple[str, ...]
```

Assert the expanded manifest includes schema version `1`, simulation and wall timing, source revision plus dirty flag, image name/digest, configuration path/checksum, artifact `detail`, explicit `incomplete_paths`, and scoring summary. Assert requested completion becomes `FAILED` when any result is not valid, while requested failure/abort never upgrades. Assert repeated finalization with identical inputs returns identical bytes; a conflicting second finalization raises `FinalizationConflict`; no temporary file remains; and parent-directory `fsync` is invoked after replacement.

Assert the inventory also includes recoverable diagnostics under `logs/docker/` and failed `.partial` recorder outputs, while excluding `manifest.json` itself and mutable `.control/`/`.status/` protocol files. Optional diagnostics never satisfy a missing required path.

- [ ] **Step 3: Run the focused tests and verify RED**

Run:

```bash
uv run pytest artifacts/tests/test_validation.py artifacts/tests/test_manifest.py artifacts/tests/test_session.py -v
```

Expected: collection or assertion failures because the new validation/session types and expanded schema do not exist.

- [ ] **Step 4: Implement fail-closed filesystem validation**

Use descriptor-based reads with `O_NOFOLLOW` where supported, require regular files with one hard link, compare `fstat` identity before and after reads, sort directory entries by POSIX relative path, and reject symlinks anywhere in a required tree. Stream file hashes in 1 MiB chunks. Keep validation pure: it returns immutable results and does not delete or repair artifacts.

- [ ] **Step 5: Implement expanded manifest and idempotent commit**

Replace Phase 1 `present` with `valid`; retain `missing` and `invalid`. Add `detail`, timing, sources, images, configurations, and `incomplete_paths` to the schema and dataclasses. Validate nonnegative ordered simulation times, timezone-aware ordered wall times, finite nonnegative wall duration, exact 64-lowercase-hex digests, unique names/paths, and evidence paths contained inside the run directory.

Write a collision-safe temporary file in the run directory, flush and `fsync`, replace `manifest.json`, open and `fsync` the directory, and clean the temporary file on every exception. If `manifest.json` exists, return it only when its bytes exactly match the newly canonicalized payload.

- [ ] **Step 6: Verify Task 2**

Run:

```bash
uv run pytest artifacts/tests -v
git diff --check
```

Expected: all artifact-domain tests pass, including migrated Phase 1 tests using `valid`.

- [ ] **Step 7: Commit Task 2**

```bash
git add artifacts/src/artifacts artifacts/schemas/manifest.schema.json artifacts/tests
git commit -m "feat: validate and finalize run bundles"
```

### Task 3: Implement the Explicit ROS Bag Recorder Adapter

**Files:**
- Create: `artifacts/src/artifacts/_adapters/__init__.py`
- Create: `artifacts/src/artifacts/_adapters/rosbag.py`
- Create: `artifacts/Dockerfile`
- Create: `artifacts/tests/test_rosbag_adapter.py`
- Modify: `artifacts/src/artifacts/__init__.py`

**Interfaces:**
- Consumes: run directory, fixed ten-topic inventory, ROS graph endpoint information, subprocess adapter, and bounded finalization deadline
- Produces: `RosbagRecorder.start`, `RosbagRecorder.is_ready`, `RosbagRecorder.finalize`, `RosbagValidator.validate`, and MCAP output under `rosbag/`

- [ ] **Step 1: Write failing command and readiness tests**

Assert `RosbagRecorder.command()` returns this semantic command with an absolute output path and QoS file:

```text
ros2 bag record --storage mcap --output <run>/rosbag
--disable-keyboard-controls --include-unpublished-topics
--qos-profile-overrides-path /etc/drone_sim/recording-qos.yaml
--node-name rosbag2_recorder_<uuidhex>
--topics <the ten fixed topics in plan order>
```

Assert readiness is false when the process exited, when any topic lacks the recorder subscription, or when offered/requested QoS is incompatible; it becomes true only with all ten subscriptions. Assert start refuses an existing nonempty `rosbag/`.

- [ ] **Step 2: Write failing shutdown and validation tests**

Using a real short synthetic bag fixture in the ROS container test surface, assert `finalize(deadline)` sends `SIGINT`, waits for exit, escalates to `SIGTERM` then `SIGKILL` only after bounds expire, and records escalation in the returned result. `RosbagValidator` must reject a missing metadata file, wrong storage, missing/extra required topic, wrong type, zero required count, unreadable serialized message, wrong run ID, nonmonotonic custom timestamps, missing image payload, or mismatched image/metadata counts.

- [ ] **Step 3: Run focused tests and verify RED**

Run:

```bash
uv run pytest artifacts/tests/test_rosbag_adapter.py -v
```

Expected: import failure because the rosbag adapter does not exist.

- [ ] **Step 4: Implement the recorder process boundary**

Inject process creation, monotonic time, and signal sending into the adapter for deterministic unit tests. The production path uses `subprocess.Popen` without a shell, captures stdout/stderr into `logs/docker/rosbag2.log.partial`, and never passes `--use-sim-time`. `is_ready` accepts a ROS-node graph adapter and checks the recorder node name plus exact topic subscription coverage.

- [ ] **Step 5: Implement bag validation**

Use `rosbag2_py.Info.read_metadata`, `rosbag2_py.SequentialReader`, `rosidl_runtime_py.utilities.get_message`, and `rclpy.serialization.deserialize_message`. Return `ValidationResult` plus topic/type/count/timestamp diagnostics; do not mutate the bag. Validate the explicit inventory and correlation rules without relying on human-formatted `ros2 bag info` output.

- [ ] **Step 6: Verify Task 3 in the artifact image**

Create `artifacts/Dockerfile` from the exact pinned ROS base digest in Global Constraints. Build `simulation_interfaces`, copy `config/recording-qos.yaml` to `/etc/drone_sim/recording-qos.yaml`, install the artifacts package, and provide a `test` target containing uv, pytest, and the artifact tests. Do not install FFmpeg until Task 4 needs it.

Run:

```bash
docker build -f artifacts/Dockerfile --target test -t drone-sim-artifacts:test .
docker run --rm drone-sim-artifacts:test uv run pytest artifacts/tests/test_rosbag_adapter.py -v
```

Expected: adapter tests pass and the image has ROS 2 Jazzy rosbag2 MCAP support.

- [ ] **Step 7: Commit Task 3**

```bash
git add artifacts/Dockerfile artifacts/src/artifacts/_adapters artifacts/tests/test_rosbag_adapter.py artifacts/src/artifacts/__init__.py
git commit -m "feat: add explicit rosbag recorder"
```

### Task 4: Implement Two Simulation-Time Video Pipelines

**Files:**
- Create: `artifacts/src/artifacts/_adapters/video.py`
- Create: `artifacts/src/artifacts/recorder_node.py`
- Create: `artifacts/tests/test_video_adapter.py`
- Modify: `artifacts/Dockerfile`
- Modify: `artifacts/src/artifacts/__init__.py`

**Interfaces:**
- Consumes: onboard/observer `sensor_msgs/msg/Image`, paired `FrameMetadata`, immutable recording configuration, FFmpeg subprocesses, and finalization deadline
- Produces: `VideoStreamRecorder`, `VideoRecorderNode`, `VideoValidator`, `video/onboard.mp4`, and `video/observer.mp4`

- [ ] **Step 1: Write failing frame-pair and command tests**

For each stream, feed metadata and image in both arrival orders and assert one raw RGB frame is written only after exact stamp pairing. Require frame IDs `0, 1, ...`, timestamps separated by exactly `50_000_000` ns, matching run ID/stream, `rgb8`, fixed even `320x240`, `step == 960`, and `len(data) == 230400`.

Reject wrong run ID, wrong stream, unmatched stale pairs, duplicate/gapped IDs, duplicate/nonmonotonic timestamps, a delta other than 50 ms, odd or changed dimensions, wrong encoding/step, and short data. Bound unmatched pending pairs to one per input per stream so memory cannot grow without limit.

Assert the generated command is exactly equivalent to:

```text
ffmpeg -nostdin -hide_banner -loglevel error
-f rawvideo -pixel_format rgb24 -video_size 320x240 -framerate 20
-i pipe:0 -an -c:v libx264 -pix_fmt yuv420p
-movflags +faststart -f mp4 <run>/video/onboard.mp4.partial
```

- [ ] **Step 2: Write failing finalization and ffprobe tests**

Assert `finalize` closes stdin, honors a shared monotonic deadline, waits for zero, retains `.partial` plus diagnostics on failure, and atomically renames only a successful nonempty output. Generate a real 4-frame fixture and require `VideoValidator` to report `codec_name=h264`, `pix_fmt=yuv420p`, `avg_frame_rate=20/1`, expected dimensions, and decoded frame count. A completed run requires the configured frame count; failed/aborted runs accept a readable shorter video but never label an empty/corrupt file valid.

- [ ] **Step 3: Run focused tests and verify RED**

Run:

```bash
uv run pytest artifacts/tests/test_video_adapter.py -v
```

Expected: import failure because the video adapter does not exist.

- [ ] **Step 4: Implement FFmpeg and pairing adapters**

Use `Popen` argument arrays and `stdin=PIPE`; never invoke a shell. Preflight `ffmpeg -encoders` for `libx264` and `ffprobe -version` before reporting readiness. Convert ROS time to integer nanoseconds with `sec * 1_000_000_000 + nanosec`. Write `bytes(image.data)` unchanged for valid `rgb8` frames and flush only at finalization; pipe backpressure is allowed to slow wall execution without altering simulation stamps.

`VideoRecorderNode` owns two `VideoStreamRecorder` instances and four best-effort depth-5 subscriptions. It exposes discovered subscription counts to aggregate readiness and emits structured recorder errors without selecting terminal status.

- [ ] **Step 5: Verify Task 4 in the artifact image**

Extend `artifacts/Dockerfile` to install Ubuntu 24.04 `ffmpeg`, then fail the build unless `ffmpeg -encoders` lists `libx264` and `ffprobe` is executable.

Run:

```bash
docker build -f artifacts/Dockerfile --target test -t drone-sim-artifacts:test .
docker run --rm drone-sim-artifacts:test uv run pytest artifacts/tests/test_video_adapter.py -v
```

Expected: unit tests and real FFmpeg/ffprobe fixture validation pass; image build fails if libx264 is unavailable.

- [ ] **Step 6: Commit Task 4**

```bash
git add artifacts/Dockerfile artifacts/src/artifacts/_adapters/video.py artifacts/src/artifacts/recorder_node.py artifacts/tests/test_video_adapter.py artifacts/src/artifacts/__init__.py
git commit -m "feat: record simulation-time camera videos"
```

### Task 5: Capture Raw Docker Logs and Partition Structured Module Logs

**Files:**
- Create: `artifacts/src/artifacts/_adapters/docker_logs.py`
- Create: `artifacts/tests/test_docker_logs.py`
- Modify: `artifacts/src/artifacts/__init__.py`

**Interfaces:**
- Consumes: Compose project name, explicit service-to-module ownership map, run ID, and command runner
- Produces: `DockerLogCapture.capture`, raw `logs/docker/<service>.log`, and the seven required `logs/<module>.jsonl` files

- [ ] **Step 1: Write failing capture tests**

Use a fake command runner that returns mixed third-party text and structured lines. Assert one argument-array invocation per service:

```text
docker compose -p <project> logs --no-color --no-log-prefix <service>
```

Assert raw bytes are preserved in `logs/docker/<service>.log`, valid structured events are routed by `module`, and the seven module files contain only compact JSON objects in capture order. Reject structured objects with a wrong run ID, unknown module, missing/extra common fields, nonfinite simulation timestamp, naive/invalid wall timestamp, or non-object `fields`. Require at least one correctly attributed event for every module.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
uv run pytest artifacts/tests/test_docker_logs.py -v
```

Expected: import failure because `DockerLogCapture` does not exist.

- [ ] **Step 3: Implement host-side log capture**

Capture services individually to avoid Compose prefix ambiguity. Keep non-JSON and JSON that does not claim the six common fields only in the raw log. A JSON object containing any common field is an attempted structured event and must either validate completely or fail capture. Write required JSONL files to `.partial`, flush and `fsync`, then replace and directory-`fsync` after all services validate. Do not mount or access `/var/run/docker.sock` from a container.

- [ ] **Step 4: Verify Task 5**

Run:

```bash
uv run pytest artifacts/tests/test_docker_logs.py artifacts/tests/test_structured_log.py -v
git diff --check
```

Expected: all log tests pass and no malformed structured line can be silently dropped.

- [ ] **Step 5: Commit Task 5**

```bash
git add artifacts/src/artifacts/_adapters/docker_logs.py artifacts/tests/test_docker_logs.py artifacts/src/artifacts/__init__.py
git commit -m "feat: capture structured compose logs"
```

### Task 6: Implement Status Store, Compose Adapter, Controller, and CLI

**Files:**
- Create: `orchestration/src/orchestration/status_store.py`
- Create: `orchestration/src/orchestration/_adapters/__init__.py`
- Create: `orchestration/src/orchestration/_adapters/compose.py`
- Create: `orchestration/src/orchestration/controller.py`
- Create: `orchestration/src/orchestration/cli.py`
- Create: `orchestration/src/orchestration/__main__.py`
- Create: `orchestration/tests/test_status_store.py`
- Create: `orchestration/tests/test_controller.py`
- Create: `orchestration/tests/test_cli.py`
- Modify: `pyproject.toml`
- Modify: `uv.lock`
- Modify: `orchestration/pyproject.toml`
- Modify: `orchestration/src/orchestration/__init__.py`

**Interfaces:**
- Consumes: template resolution, immutable lifecycle, Compose command adapter, status/control files, log capture, artifact validation/session, monotonic wall deadlines, and OS signal adapter
- Produces: `RunController.start/status/abort/collect_results`, unique Compose project lifecycle, atomic status, terminal manifest, and `drone-sim` console script

- [ ] **Step 1: Write failing status-store and CLI tests**

Assert exclusive allocation creates `runs/<uuid>/{.control,.status,configuration}` and refuses an existing run. Assert every status write round-trips valid JSON with no temp sibling. Test CLI parsing for the exact four commands and JSON stdout, including default `runs` and explicit absolute `--output-root` on status/abort/collect. Expected exits are `0` completed, `1` failed, `130` aborted, and `2` for controlled usage/configuration errors without traceback.

`status RUN_ID` returns at least:

```json
{"run_id":"...","state":"RUNNING","reason":"","manifest_path":null}
```

`abort RUN_ID` atomically creates one requested `ABORTED` finalization and is idempotent. It never invokes Compose stop/down itself. `collect-results` validates and prints the existing manifest path without changing any file.

- [ ] **Step 2: Write failing controller sequence tests**

With fake adapters and a hand-controlled monotonic clock, assert this exact call order for completion:

```text
allocate -> snapshot config -> compose up -> wait artifacts-ready
-> wait source-finished -> request FINALIZING -> wait runtime-frozen
-> wait artifacts-final -> capture logs -> validate -> commit manifest
-> notify terminal -> wait terminal-notified -> compose down
```

Add failures for startup deadline, recorder failure, clock stall, log-capture failure, validation failure, Ctrl-C, and finalization deadline. Every path that started Compose enters finalization and calls down in `finally`. Requested completion with invalid output commits a `FAILED` manifest; requested abort stays `ABORTED` even when all artifacts happen to validate. Repeating cleanup never removes the run directory.

During startup and running, race the expected success status against `.status/runtime-failure.json`, operator abort, child-process death, and the shared wall deadline. The first observed cause wins and is persisted; later failures remain diagnostics and cannot replace the primary reason.

The runtime writes `.status/runtime-running.json` immediately after publishing `RUNNING` for the first valid clock. The controller consumes that durable signal to update `operator-state.json`; test an abort racing the signal so Task 8 can reliably wait for `RUNNING` without inferring simulation progress from wall time.

- [ ] **Step 3: Run focused tests and verify RED**

Run:

```bash
uv run pytest orchestration/tests/test_status_store.py orchestration/tests/test_controller.py orchestration/tests/test_cli.py -v
```

Expected: import failures because the controller surfaces do not exist.

- [ ] **Step 4: Implement atomic status and Compose adapters**

`StatusStore` owns only JSON control/status files and exclusive directory allocation. `ComposeRuntime` uses argument arrays with a unique project `drone-sim-<run_id-without-hyphens>`, explicit `--project-directory`, and environment variables `SIM_RUN_ID`, `SIM_RUN_DIRECTORY`, `SIM_CONFIG_PATH`, and `SIM_PHASE2_PROFILE=1`. It implements `up`, `logs`, `stop_services`, `down`, `ps`, and `image_digests`; it has no lifecycle policy. `up` is exactly bounded detached startup (`up --detach --no-build`). Its Docker-log runner validates Task 5's frozen per-service argv, augments the actual subprocess invocation with the project directory/environment, and receives a newly computed remaining timeout for each of the seven calls.

- [ ] **Step 5: Implement controller and CLI**

Use one shared monotonic deadline for each startup/finalization phase so retries cannot reset the budget. Reserve `min(5 seconds, finalization_wall_seconds / 5)` inside the finalization deadline for `compose down`; earlier work uses the shortened work deadline and teardown uses the remaining total budget. Polling wall time only observes status files and process health. Append host operator structured events to stdout and `logs/orchestration-host.jsonl.partial`; include them in the final orchestration module log during Task 5 capture.

Parse the runtime-owned `artifacts-final.json` as a strict report with exactly three records: `video/onboard.mp4`, `video/observer.mp4`, and `rosbag`. Each record contains `relative_path`, `status`, `detail`, `size_bytes`, `sha256`, and nonempty `semantic` facts. Inject `ArtifactSession` validators that recompute host-safe size/tree checksum and require exact agreement with the report; absent/malformed records, corrupt media/bag evidence, stale digests, or status mismatches downgrade requested completion. The controller does not need host FFmpeg or ROS dependencies and must not fall back to presence-only validation.

Before manifest commit, record source revision/dirty state with read-only Git commands and image digests with `docker image inspect`. After commit, write `terminal-committed.json`; never rewrite required artifacts. Always run Compose teardown in a bounded `finally` block.

Add `artifacts` and `orchestration` as uv workspace members in the root `pyproject.toml`, make the root development project depend on both workspace packages, make orchestration depend on the artifacts package, and register `drone-sim = "orchestration.cli:main"` under `[project.scripts]` in `orchestration/pyproject.toml`. Regenerate `uv.lock`; do not rely on pytest-only `pythonpath` for the executable.

- [ ] **Step 6: Verify Task 6**

Run:

```bash
uv run pytest orchestration/tests -v
uv run drone-sim --help
git diff --check
```

Expected: controller/CLI tests pass and help lists exactly `start`, `status`, `abort`, and `collect-results`.

- [ ] **Step 7: Commit Task 6**

```bash
git add pyproject.toml uv.lock orchestration/pyproject.toml orchestration/src/orchestration orchestration/tests
git commit -m "feat: add simulation operator lifecycle"
```

### Task 7: Build the Dockerized Phase 2 Runtime and Synthetic Publishers

**Files:**
- Create: `orchestration/Dockerfile`
- Create: `orchestration/src/orchestration/runtime_node.py`
- Modify: `artifacts/Dockerfile`
- Create: `tests/phase2/Dockerfile`
- Create: `tests/phase2/module_stub.py`
- Create: `tests/phase2/synthetic_gazebo.py`
- Create: `tests/phase2/synthetic_electromagnet.py`
- Create: `tests/phase2/synthetic_scorekeeper.py`
- Create: `tests/phase2/scoring.json`
- Create: `tests/phase2/entrypoint.sh`
- Create: `tests/integration/test_phase2_runtime_contract.py`
- Modify: `compose.yaml`

**Interfaces:**
- Consumes: control/status protocol, shared ROS interfaces, fixed QoS, recorder adapters, resolved configuration, and run-directory mount
- Produces: profile-scoped Phase 2 services, real recorder readiness, deterministic two-second synthetic streams, fixture Gazebo/scoring files, and all seven module log sources

- [ ] **Step 1: Write the failing Compose/runtime contract test**

Render Compose with the `phase2` profile and assert services exist for `orchestration-runtime`, `artifacts-runtime`, `synthetic-gazebo`, `synthetic-electromagnet`, `synthetic-scorekeeper`, `synthetic-companion`, and `synthetic-ardupilot-sitl`. Assert every service uses `init: true`, `restart: "no"`, the default Compose network, a read-only resolved-config mount, and the same run-directory mount. Assert no service uses host networking, privileged mode, or a Docker-socket mount.

Run the source in a ROS integration harness and assert it publishes no clock before aggregate readiness. After `READY`, assert 41 clock values `0, 0.05, ..., 2.00` seconds and exactly 40 frames per stream at `0.05, ..., 2.00` with IDs `0..39`, `rgb8` 320x240 payloads, exact matching metadata, and ground truth at each frame.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
uv run pytest tests/integration/test_phase2_runtime_contract.py -v
```

Expected: failure because the Phase 2 services and runtime nodes do not exist.

- [ ] **Step 3: Implement the ROS orchestration runtime**

Publish `STARTING` immediately. Subscribe to aggregate artifact status and publish `READY` only for the current run with `ready=true`. On the first valid clock after readiness publish `RUNNING`, then atomically write `.status/runtime-running.json` with the run ID, fixed state, and first-clock simulation nanoseconds. Watch `finalize-request.json`, publish `FINALIZING` with its reason, then wait for `terminal-committed.json`, apply `ARTIFACTS_FINALIZED` or `FINALIZATION_FAILED`, publish the terminal state, write `terminal-notified.json`, and exit. Use zero ROS time before the first clock and the final observed simulation timestamp afterward.

- [ ] **Step 4: Implement aggregate artifacts runtime**

Start the bag and both video pipelines, verify all required graph subscriptions and writable paths, publish/write ready status, and watch lifecycle. On `FINALIZING`, require `runtime-frozen.json`, drain callbacks, finalize videos and bag under one wall deadline, validate recorder-local output, and write the strict three-record `artifacts-final.json` contract frozen in Task 6. After host commit, publish final `ArtifactStatus` with `manifest_path`, write no required log event, and exit.

If a recorder fails before finalization, atomically write `.status/runtime-failure.json` with module `artifacts`, a stable reason, and diagnostic paths, but keep the runtime alive to preserve and finalize surviving output.

- [ ] **Step 5: Implement deterministic synthetic services**

`synthetic_gazebo` waits for `READY`, publishes the initial zero clock, waits for `RUNNING`, then publishes the 40 deterministic RGB frame pairs and ground truth using integer nanosecond stamps. It waits for `FINALIZING`, stops permanently, writes a clearly labeled fixture `gazebo/server.log` and `gazebo/state/synthetic-state.json`, then writes `runtime-frozen.json`. A test-only `SIM_SYNTHETIC_WALL_DELAY_MS` may slow each already-determined step for abort/slow-host tests but never changes a timestamp, frame ID, payload, or event order.

After the fortieth frame, it writes `.status/source-finished.json`; this is an infrastructure completion signal to the host, not a wall-time-derived simulated event. Test-only `SIM_PHASE2_FAULT=clock_stall_after_5` stops clock/frame progress without claiming completion; `SIM_PHASE2_FAULT=observer_encoder_after_5` makes the observer recorder report failure after frame 4. No other fault strings are accepted.

`synthetic_electromagnet` publishes one deterministic scenario event at 1.0 simulated second. `synthetic_scorekeeper` publishes one score event, writes `scoring/events.jsonl`, and writes a fixture `scoring/result.json` with achieved/max score `0.0` and the SHA-256 of `tests/phase2/scoring.json`; Phase 2 makes no scoring-validity claim. The companion and ArduPilot stub services emit correctly attributed readiness/finalization logs and never publish control or physics data.

- [ ] **Step 6: Implement container images and Compose profile**

Build all new images from the exact pinned ROS base digest in Global Constraints, build `simulation_interfaces`, and install only owned Python sources. `artifacts/Dockerfile` installs `ffmpeg`, asserts `libx264` appears in `ffmpeg -encoders`, and includes rosbag2 MCAP plus `ffprobe`. Provide a `test` target containing pytest/uv and a minimal runtime target. Preserve the existing `foundation` service outside the Phase 2 profile.

- [ ] **Step 7: Verify Task 7**

Run:

```bash
docker compose --profile phase2 build
uv run pytest tests/integration/test_phase2_runtime_contract.py -v
docker compose --profile phase2 config --quiet
```

Expected: images build, the runtime contract passes, and Compose validates without forbidden mounts/networking.

- [ ] **Step 8: Commit Task 7**

```bash
git add orchestration/Dockerfile orchestration/src/orchestration/runtime_node.py artifacts/Dockerfile tests/phase2 tests/integration/test_phase2_runtime_contract.py compose.yaml
git commit -m "feat: add synthetic artifact runtime stack"
```

### Task 8: Prove Completed, Failed, and Aborted Run Bundles

**Files:**
- Create: `tests/integration/test_phase2_compose.py`
- Create: `docs/verification/phase-2-run-artifacts.md`
- Modify: `Makefile`

**Interfaces:**
- Consumes: operator CLI, Phase 2 Compose profile, recorder/validator stack, bundle schema, and all fixed Phase 2 contracts
- Produces: executable Phase 2 gate and evidence that Phase 3 can replace only the synthetic physical source

- [ ] **Step 1: Write the completed-run acceptance test**

Start a run with a temporary output root and require exit zero. Validate:

```python
assert manifest["terminal_status"] == "COMPLETED"
assert manifest["incomplete_paths"] == []
assert all(item["validation"] == "valid" for item in manifest["artifacts"])
```

Use `ffprobe` JSON and full decode to require both MP4s have one H.264/yuv420p stream, 320x240, `20/1`, and 40 frames. Use the bag validator to require the ten exact topics, 41 clocks, 40 images and metadata per stream, 40 ground-truth messages, paired IDs/stamps, one scenario event, one score event, and lifecycle order `STARTING, READY, RUNNING, FINALIZING`. Require no clock before artifact-ready evidence.

Run the completed case once with zero synthetic wall delay and once with a nonzero delay. Compare extracted simulation timestamps, IDs, image payload hashes, event payloads, and decoded video-frame hashes; they must be identical even though wall timing and bag container bytes may differ.

Reparse all seven nonempty module JSONL logs; recompute every file/tree SHA-256 and size; validate configuration/scoring JSON; require no `.partial`/manifest temp; require source/image/config provenance; and require no Compose container/network for the run project after exit.

- [ ] **Step 2: Write failed and aborted acceptance tests**

Inject observer-encoder failure after five frames. Require exit `1`, terminal `FAILED`, explicit invalid/missing observer video record, a readable bag, playable onboard video, preserved raw logs, and no deletion of partial observer diagnostics.

Start a normal run as a subprocess, wait until `RUNNING`, invoke `drone-sim abort RUN_ID`, and require start exit `130`, terminal `ABORTED`, readable bag, playable shorter videos when at least one frame was accepted, explicit incomplete records where needed, and preserved diagnostics. A second abort and second `collect-results` must be idempotent.

- [ ] **Step 3: Run the new tests and verify RED before integration completion**

Run:

```bash
uv run pytest tests/integration/test_phase2_compose.py -v
```

Expected before the final integration wiring: failures identifying the incomplete completed/failed/aborted paths. Preserve this RED output in the implementation report.

- [ ] **Step 4: Complete only the integration wiring required by the failures**

Connect the CLI, status files, Compose profile, recorder runtime, log capture, validators, and manifest session. Do not add Gazebo Harmonic, ArduPilot, companion mission code, electromagnet physics, or real scoring in this task.

- [ ] **Step 5: Run the complete Phase 2 gate**

Update `Makefile` with `test-phase2` and include it in `test`. Run:

```bash
make test
docker compose config --quiet
docker compose --profile phase2 config --quiet
git diff --check
git -C companion/comp2026 status --short
```

Expected: every command exits zero; Phase 1 remains green; all three Phase 2 terminal modes preserve validated bundles; the nested companion repository has no Phase 2 change.

- [ ] **Step 6: Record verification evidence and exact non-claims**

Create `docs/verification/phase-2-run-artifacts.md` with tested commit, UTC window, commands/exits/test counts, image digests, completed/failed/aborted run IDs, bundle paths, manifest checksums, MP4 ffprobe facts, bag topic/count facts, log counts, finalization timings, and Compose cleanup evidence.

State explicitly that Phase 2 uses synthetic Gazebo/scoring fixtures and does not claim Gazebo Harmonic physics, Gazebo-authoritative clock, native Gazebo state, ArduPilot lockstep, companion behavior, electromagnet physics, mission scoring, or maximum-score acceptance.

- [ ] **Step 7: Commit the Phase 2 gate**

```bash
git add tests/integration/test_phase2_compose.py Makefile docs/verification/phase-2-run-artifacts.md
git diff --cached --quiet || git commit -m "test: prove phase 2 run artifacts"
```

## Phase 2 Definition of Done

- The operator commands are executable and documented.
- A unique immutable run is allocated before Compose starts and is never overwritten.
- Aggregate recorder readiness is observed before the first simulation clock.
- Two complete synthetic image streams are encoded as playable H.264/yuv420p MP4 at 20 frames per simulated second.
- A readable MCAP bag contains the explicit required topic/type/count inventory through `FINALIZING`.
- Raw Docker output and valid structured JSONL for all seven modules are preserved.
- Configuration, synthetic Gazebo/scoring fixtures, videos, bag, logs, and provenance have reproducible sizes/checksums and semantic validation.
- Completed, failed, and aborted runs all commit a schema-valid atomic manifest and preserve recoverable diagnostics.
- No required artifact changes after manifest commit, no temporary output is mistaken for valid output, and Compose resources are removed.
- Phase 1 tests remain green and `companion/comp2026` remains unchanged and unpushed.
