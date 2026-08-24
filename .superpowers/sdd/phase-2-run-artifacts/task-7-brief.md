# Task 7: Build the Dockerized Phase 2 Runtime and Synthetic Publishers

## Context and base

- Worktree: `/home/willis/projects/drone_sim/.worktrees/phase-2-run-artifacts`
- Branch: `phase-2-run-artifacts`
- Base commit: `d69d76bbb57a4dca2e6f0bb8a6cdeec13674010a`
- Authoritative plan: `docs/superpowers/plans/phase-2-run-artifacts.md`, Task 7
- Tasks 1–6 are complete and independently approved. Consume their exact ROS, status, recorder, log, manifest, controller, Compose, deadline, and filesystem seams.
- This is a synthetic Phase 2 runtime proving infrastructure only. Do not claim real Gazebo physics, ArduPilot lockstep, mission behavior, or scoring validity; those remain later phases.
- Do not modify machine configuration or `companion/comp2026`.

## Scope and required files

Create/modify the Task 7 files listed in the authoritative plan, including the corrected omissions:

- `orchestration/Dockerfile`
- `orchestration/src/orchestration/runtime_node.py`
- `orchestration/tests/test_runtime_node.py`
- `artifacts/src/artifacts/runtime_protocol.py`
- `artifacts/src/artifacts/runtime_node.py`
- `artifacts/tests/test_runtime_protocol.py`
- `artifacts/tests/test_runtime_node.py`
- `artifacts/Dockerfile`
- `tests/phase2/{Dockerfile,entrypoint.sh,module_stub.py,synthetic_gazebo.py,synthetic_electromagnet.py,synthetic_scorekeeper.py,scoring.json}`
- `tests/integration/test_phase2_runtime_contract.py`
- `compose.yaml`
- package exports/metadata only as needed to install/run these modules
- frozen interface/plan/report/ledger documents when implementation evidence sharpens them

Do not implement Task 8’s completed/failed/aborted end-to-end CLI gate yet. Task 7 must nevertheless build, render, and prove the actual ROS runtime contract needed by Task 8.

## Frozen seven-service topology

The Phase 2 profile contains exactly these controller-owned service names and structured-log modules, in order:

```text
orchestration-runtime       orchestration
artifacts-runtime           artifacts
synthetic-companion         companion
synthetic-ardupilot-sitl    ardupilot_sitl
synthetic-gazebo            gazebo
synthetic-electromagnet     electromagnet
synthetic-scorekeeper       scorekeeper
```

Every service must:

- declare `profiles: [phase2]`, `init: true`, and `restart: "no"`;
- use only the default Compose network;
- avoid host networking, privileged mode, added capabilities, and Docker-socket mounts;
- receive the same canonical UUID and exact absolute run/config paths;
- bind the run directory at the identical absolute host/container path using long bind syntax;
- overlay `configuration/run.json` read-only at the identical `SIM_CONFIG_PATH`;
- use interpolation defaults that leave ordinary `docker compose config --quiet` valid without Phase 2 variables;
- remain alive through normal source completion/fault reporting until terminal coordination.

Task 6 already sets `COMPOSE_PROFILES=phase2`; preserve its exact detached `up --detach --no-build` argv. The foundation service stays outside the profile and remains behaviorally unchanged.

## Runtime protocol helper

Implement a lower-level `artifacts.runtime_protocol` helper usable by both runtime containers without reversing package dependencies. It owns no lifecycle policy. It must descriptor-relatively read host controls and write runtime statuses with exact-key validation, canonical run ID checks, containment, no-follow, regular-file/single-link enforcement, bounded reads, duplicate/nonfinite JSON rejection, first-wins/conflicting-rewrite rejection, same-directory temp publication, file `fsync`, atomic replace, and directory `fsync`.

Exact runtime-owned schemas:

```text
artifacts-ready.json  {"run_id": UUID, "ready": true}
runtime-running.json  {"run_id": UUID, "state": "RUNNING", "sim_timestamp_ns": int>=0}
source-finished.json  {"run_id": UUID, "finished": true, "sim_timestamp_ns": int>=0}
runtime-failure.json   {"run_id": UUID, "module": nonempty, "reason": nonempty,
                        "diagnostic_paths": [safe unique relative paths]}
runtime-frozen.json    {"run_id": UUID, "frozen": true}
terminal-notified.json {"run_id": UUID, "notified": true}
```

Exact host-owned inputs:

```text
finalize-request.json   {"run_id": UUID, "requested_terminal": COMPLETED|FAILED|ABORTED,
                         "reason": nonempty}
terminal-committed.json {"run_id": UUID, "terminal_status": COMPLETED|FAILED|ABORTED,
                         "reason": string, "manifest_path": "manifest.json"}
```

`artifacts-final.json` remains exact `{run_id, complete, records}` with exactly three unique records for `video/onboard.mp4`, `video/observer.mp4`, and `rosbag`, and exact record keys `{relative_path,status,detail,size_bytes,sha256,semantic}`.

## ROS orchestration runtime

Implement a small injected/testable domain core plus an `rclpy` adapter:

- publish `STARTING` immediately on `/simulation/run_state`, reliable transient-local depth 1, zero simulation timestamp;
- accept only current-run aggregate artifact readiness, then publish `READY`;
- accept no clock transition before readiness;
- on first valid `/clock` after readiness, publish `RUNNING`, then durably write `runtime-running.json` with that exact stamp (the synthetic first valid clock is zero);
- monitor `finalize-request.json` from STARTING, READY, or RUNNING; publish `FINALIZING` with last known simulation stamp and request reason;
- wait for exact `terminal-committed.json`, apply finalized/failure lifecycle event consistently with its terminal state, publish terminal `RunState`, atomically write `terminal-notified.json`, and exit;
- after the host capture barrier, terminal publication/status acknowledgement must be silent on stdout and write no required artifact/log event;
- ignore stale ROS run IDs with diagnostics before the quiescence/log-capture boundary.

Use wall monotonic time only for bounded infrastructure polling. Simulation timestamps come only from messages/clock.

## Aggregate artifacts runtime

Implement one process that owns the explicit ten-topic MCAP `RosbagRecorder`, both `VideoStreamRecorder` pipelines through `VideoRecorderNode`, recorder readiness, failure containment, and strict final report.

Startup:

- preflight FFmpeg/libx264, bag storage, paths, and exact ROS graph subscriptions;
- start recorder processes before allowing clock;
- publish `/simulation/artifact_status` ready only for the current run and atomically write `artifacts-ready.json` after all required endpoints/output handles are usable;
- never claim ready on partial startup; write `runtime-failure.json` and remain alive long enough to preserve diagnostics.

Running/faults:

- feed both image+metadata streams using Task 4’s exact pairing rules;
- let rosbag record the fixed Task 3 inventory;
- accept only `SIM_PHASE2_FAULT` unset, `clock_stall_after_5`, or `observer_encoder_after_5`;
- implement `observer_encoder_after_5` in artifacts runtime immediately after observer frame ID 4, not in the publisher; report failure, preserve bag/onboard, stop accepting observer output, and keep the process alive for finalization.

Finalization:

- wait for `FINALIZING` and require `runtime-frozen.json` before draining;
- stop callback acceptance, drain, close both FFmpeg inputs, finalize rosbag, and validate under one supplied absolute monotonic deadline with the existing adapters;
- write exactly three report records even when missing/invalid; nonvalid records still have a nonempty semantic object identifying validator/kind/failure;
- video semantic keys are exactly `codec_name`, `pix_fmt`, `avg_frame_rate`, `width`, `height`, `frame_count`, `diagnostics`;
- bag semantic contains `storage_id` and ordered `topics`, each with `name`, `message_type`, `message_count`, `first_sim_timestamp_ns`, `last_sim_timestamp_ns`;
- record stable descriptor-based size/checksum/tree-checksum facts from the validation results;
- atomically write `artifacts-final.json`, then wait silently for `terminal-committed.json`;
- publish final `ArtifactStatus` with portable `manifest_path="manifest.json"`, write no required event, and exit.

## Deterministic synthetic services

`synthetic-gazebo` is the sole `/clock`, camera, and ground-truth publisher:

- offer the four archival camera image/metadata publishers as reliable depth 5;
  artifact video and rosbag subscribers request the same QoS, without adding a
  topic to the fixed ten-topic bag inventory;
- before initial clock/frame output, require both archival subscribers on all
  four camera publishers, then drain a bounded six-item frame publication queue
  one item per executor turn in fixed clock/onboard image/onboard metadata/
  observer image/observer metadata/ground-truth order; finalization preempts it;

- advertise endpoints immediately but publish no clock/frame before aggregate READY;
- after READY, publish initial clock 0; wait for RUNNING; publish clocks at integer nanoseconds `0, 50_000_000, ..., 2_000_000_000` (41 total);
- publish exactly 40 onboard and 40 observer `rgb8` 320x240 frames at `0.05..2.00` seconds with `frame_id=0..39`, exact paired `FrameMetadata`, and one matching ground-truth message per frame;
- deterministic payloads are functions only of stream/frame ID; no wall-derived values;
- after frame 39, atomically write `source-finished.json` with stamp `2_000_000_000` and remain alive;
- on FINALIZING, stop permanently, write clearly labeled fixture `gazebo/server.log` and `gazebo/state/synthetic-state.json`, atomically write `runtime-frozen.json`, then remain silent/alive for terminal handshake.

`SIM_SYNTHETIC_WALL_DELAY_MS` may delay already-decided steps only. `clock_stall_after_5` publishes through frame ID 4 and then stalls without `source-finished`; it stays alive for controller finalization. Reject all other source fault strings.

`synthetic-electromagnet` publishes exactly one current-run scenario event at 1.0 simulated second. `synthetic-scorekeeper` publishes exactly one score event, writes deterministic fixture `scoring/events.jsonl`, and writes `scoring/result.json` with achieved/max score 0.0 plus the SHA-256 of `tests/phase2/scoring.json`; label all Phase 2 scoring as fixture/non-validity evidence. The companion and ArduPilot SITL stubs publish no control/physics data and emit correctly owned structured readiness/finalization events before the capture barrier.

All seven services must produce at least one valid six-field `StructuredEvent` before log capture. No service writes stdout after its quiescence/final report boundary.

## Images and entrypoints

- Use the exact pinned `ros:jazzy-ros-base@sha256:2589...b09e4` base from the plan.
- Build `simulation_interfaces` in every ROS runtime image that needs custom messages.
- Extend the existing exact-lock artifacts image; keep FFmpeg package lock/libx264 assertion, rosbag2 MCAP, and ffprobe.
- Install only owned runtime Python packages/sources in runtime stages; keep uv/pytest in test stages.
- Source ROS and the interface workspace in entrypoints; use exec/traps so PID 1 and Compose shutdown are deterministic.
- Do not broaden Linux capabilities or weaken Task 4’s anonymous-inode trust model.

## TDD and verification

Follow Superpowers TDD/systematic debugging. Persist genuine RED evidence before production code. The Sol implementer may spawn at most one Luna child only for a fully frozen, bounded Compose/Dockerfile contract subtask with objective tests; central protocol/ROS/runtime integration remains Sol-owned.

Test at minimum:

- all protocol schema, idempotency/conflict, durability, symlink/hardlink/path/JSON cases;
- orchestration runtime transition/order/stamps/stale IDs/finalize from every preterminal state;
- artifact aggregate startup/readiness, fault containment, freeze barrier, deadline sharing, exact semantic report, and silence boundary;
- exact Compose services/profile/mounts/env/no forbidden privileges/socket/network;
- no pre-ready clock and exact 41 clocks/40 paired frame sets/ground truth;
- deterministic slow-wall behavior with identical simulation stamps/payload/order;
- both accepted fault modes and rejection of unknown fault strings;
- all seven structured log owners emit before quiescence and remain alive;
- plain Phase 1 Compose config remains valid.

Run at least:

```bash
uv run pytest artifacts/tests/test_runtime_protocol.py artifacts/tests/test_runtime_node.py orchestration/tests/test_runtime_node.py -v
uv run pytest tests/integration/test_phase2_runtime_contract.py -v
uv run pytest artifacts/tests orchestration/tests tests/contracts tests/integration/test_phase2_runtime_contract.py -v
docker compose config --quiet
docker compose --profile phase2 config --quiet
docker compose --profile phase2 build
uv run python -m compileall -q artifacts/src orchestration/src tests/phase2 tests/integration/test_phase2_runtime_contract.py
git diff --check
```

Also run the relevant container test targets after image changes and record exact image IDs/digests. Do not claim Task 8 terminal runs yet.

Write `.superpowers/sdd/phase-2-run-artifacts/task-7-report.md` with RED/GREEN evidence, exact tests/builds/image IDs, runtime schemas, service topology, self-review, and Task 8 concerns. Commit scoped work as `feat: add synthetic artifact runtime stack`, leave the worktree clean, and report the hash.

## Judgment constraints

- Keep runtime protocol durability below lifecycle policy; do not import orchestration into artifacts.
- Prefer small stateful cores with injected ROS/filesystem/process adapters over untestable monolithic node loops.
- Reuse Tasks 3–6 rather than creating second bag/video/log/status/manifest implementations.
- Do not move the observer encoder fault into the source publisher.
- Do not use wall time to generate simulation stamps or event ordering.
- Do not let a synthetic fixture masquerade as real Gazebo, ArduPilot, mission, or scoring evidence.
- Follow the global no-AI-attribution rule.
