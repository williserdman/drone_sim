# Local Simulation Console Implementation Plan

**Goal:** Run one selected cloud mission locally with live disarmed preparation, an explicit mission-start action, native QGC, interactive Gazebo, and an independent camera view.

**Architecture:** Extend the existing run controller and shared durable runtime protocol. A local HTTP console and CLI share one persistent session owner; Gazebo admits a dynamic mission epoch without changing the frozen configuration. Existing mission hosts retain flight authority and artifact finalization remains authoritative.

**Tech stack:** Python 3.12, standard-library HTTP server and browser JavaScript, ROS 2 Jazzy, Gazebo Harmonic, ArduPilot SITL, Docker/Colima arm64, noVNC, native macOS QGroundControl.

**Spec:** [Local simulation console](../specs/2026-10-08-local-simulation-console-spec.md).

## Global constraints

- Baseline PR #3 at `646892f`; implement and commit in the user-approved sibling worktree.
- One mission per run and one active workstation session; preserve all earlier evidence.
- Preparation advances native physics, telemetry and real camera images while disarmed; mission time starts only after admission.
- Keep immutable configuration and separately immutable admission identity. Preserve 50 ms public sensor cadence, native timestamps, independent scoring, and existing unattended behavior.
- Automatic mission commands belong to the selected companion host; QGC intervention must be observed and recorded.
- Flight recovery and simulator teardown are separate operations. No force arming, truth-based perception, or invented physical success.
- Control/view endpoints bind to loopback. Browser refresh reconnects to the existing owner.
- Update affected READMEs, architecture, runbook, handoff, and the implementation ledger with verification scope.

## Review focus

1. Duplicate/reordered clicks and browser reconnect must never create a second mission or release an unready aircraft (tasks 1, 4).
2. Long preparation and samples queued across start must not exhaust mission budgets or contaminate public recordings (tasks 1, 2).
3. Native QGC mode takeover, telemetry loss, or rejected LAND must remain visible instead of being reported as successful recovery (task 2).
4. Stale images, failed desktop reconnects, and a slow browser must not block simulation or silently show a live indicator (tasks 1, 3, 4).
5. Owner crashes, stale images/source, occupied ports, and incomplete cleanup must block a conflicting run with actionable diagnostics (tasks 3, 4).

## Task 1: Live preparation and immutable start admission

**Files:** `artifacts/src/artifacts/operator_protocol.py`, artifact manifest/configuration/acceptance code and schemas; `orchestration/src/orchestration/config.py`, `runtime_node.py`; `config/run-template.schema.json`; Gazebo runtime/epoch/preview modules and their existing tests; related module guides.

**Interface:** resolved `RunConfig.launch_mode` is `unattended` (default) or `operator`; `preparation_wall_seconds` defaults to 1800. A central operator protocol, built on `protocol_files`, owns run/request/config-bound controls, fresh preflight observations, request-scoped companion readiness, and immutable `configuration/start-admission.json`. Gazebo alone chooses the native epoch; the admission binds the request/config digests and public zero. The private camera preview is served inside Gazebo at port 8081, `/camera.mjpg` and `/status`, independently of public mission outputs.

- [x] Add and run failing focused tests for equivalent/conflicting start requests, foreign run/config IDs, immutable admissions, a start after the original warmup epoch, and no public frames during live preparation.
- [x] Implement the shared protocol and explicit launch configuration, preserving legacy serialized defaults and unattended behavior.
- [x] Implement live private preparation, request-scoped epoch latching, common admission binding, bounded latest-frame preview, and runtime admission gating.
- [x] Verify first public frame remains 50 ms/frame 0, recorder counts stay unchanged, and cancellation can quiesce before any admission; run affected focused tests and typechecking.
- [x] Bind admission evidence through validation and manifest publication; update artifact/Gazebo/orchestration guides and commit the coherent runtime change.

## Task 2: Companion preflight, interventions, and bounded flight recovery

**Files:** `companion/src/drone_sim_companion/operator_control.py`, configured and other runtime hosts, controlled descent recovery, companion tests and README.

**Interface:** consume task 1's central operator protocol and launch settings. Publish fresh observed preflight/vehicle facts and request-scoped readiness. Every host holds its initial flight commands until RUNNING plus matching admission. A durable flight-abort request cancels mission intent and invokes mission-owned recovery; observed recovery states are separate from runtime-finalization controls.

- [x] Add and run failing tests at mission-host seams: no commands during preparation; request readiness rejects armed/stale peers; deadlines start at admission; equivalent abort retries initiate one recovery.
- [x] Integrate the common operator helper with configured, competition, tuning, hover, and descent hosts without changing unattended mission behavior.
- [x] Add observed intervention and recovery progress, including landed/disarmed, rejected authority, missing telemetry, timeout, and Stop Simulation interrupting recovery.
- [x] Run each affected test file and typechecking, update the companion guide, and commit.

## Task 3: Native local runtime and interactive operator views

**Files:** `compose.local.yaml`, `scripts/local-build.sh`, `gazebo/local-viewer/`, relevant Dockerfiles and architecture-specific package locks, focused local deployment tests; deployment documentation.

**Interface:** local overlay adds `gazebo-viewer` auxiliary service on loopback 6080, publishes task 1 camera on loopback 8081, and dedicated SITL GCS TCP on loopback 5762. Viewer uses the same run-scoped Gazebo partition/assets/network. Linux GUI permits navigation/inspection but excludes ungoverned world controls. Build script produces the existing seven runtime tags plus viewer and records source/image inputs for reuse checks.

- [x] Add failing checks for loopback-only bindings, same partition, no GUI mutation plugins, and deterministic native architecture selection.
- [x] Build arm64 runtime dependencies without silently changing pinned simulator/firmware behavior; preserve existing cloud deployment path.
- [x] Add Xvfb/desktop/noVNC with a matching Gazebo GUI and reconnect support; verify actual GUI launch and a real scene.
- [x] Run Compose validation and focused checks; record build/runtime evidence and operational limitations in the runbook/ledger; commit.

## Task 4: Persistent run-control interface, console, and native QGC

**Files:** `orchestration/src/orchestration/operator_session.py`, `console.py`, `console_static/`, `catalog.py`, `local_runtime.py`; existing controller, lifecycle, status store, CLI and Compose/suite adapters; CLI/session/browser tests; root developer typecheck configuration and docs.

**Interface:** `drone-sim console` owns the loopback server (default 8765); CLI `prepare`, `start-mission`, `abort-flight`, and `stop` call the same persistent service. Existing `start`, `suite`, `status`, `abort`, and `collect-results` remain supported. HTTP reads `/api/catalog`, `/api/session`, `/api/checks`; mutations use JSON `/api/prepare`, `/api/start-mission`, `/api/abort-flight`, `/api/stop`, `/api/open-qgc`, `/api/build`. Mutations require local origin and a session token; run/request IDs bind mission actions. `RunController.start` accepts explicit operator launch arguments and stays alive through finalization.

- [x] First run a failing host-import check and fix Linux-only imports without moving Linux recording into the Mac host; add scoped static typechecking.
- [x] Add failing catalog/controller tests for approved tuples, separate preparation/mission deadlines, atomic repeated starts, cancellation, terminal requests, and preserved manifests.
- [x] Implement controller integration and a workstation lock shared with direct launches/suites, avoiding nested acquisition. Hold ownership through cleanup, persist session identity, and reconcile orphaned owners before another launch.
- [x] Add failing HTTP/CLI tests for duplicate Prepare, foreign/invalid requests, refresh, local-origin enforcement, bounded input, and disconnected runtime.
- [x] Implement the local service, catalog from approved complete templates, prerequisites/provenance checks, native QGC launch and dedicated telemetry bridge, and UI actions/status/views/results.
- [ ] Verify browser selection/Prepare/Start/recovery/Stop/Run Again using the public interface and focused test files; update README/runbook and commit.

## Task 5: Real acceptance and whole-change review

- [x] Build matching committed sources and exercise a real configured descent: native preflight advances while disarmed, QGC connects, actual Gazebo scene is interactive, onboard preview updates, Start admits once, and mission/evidence are independently assessed.
- [ ] Exercise prepared cancellation, stale view indication, flight recovery, Stop Simulation, refresh/reconnect, and a fresh repeat identity. Save logs/results and record what was actually verified.
- [x] Run scoped typechecking and the full test suite once; diagnose any failures against baseline and implementation.
- [ ] Use the code-review skill with independent Standards and Spec reviews against fixed base `646892f`; resolve material findings, rerun affected checks, and commit final changes without authorship trailers.
- [ ] Give concise run instructions and distinguish implemented behavior, passing checks, and any unverified runtime acceptance.
