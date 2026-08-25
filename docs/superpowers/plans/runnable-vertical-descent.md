# Runnable Vertical Descent Implementation Plan

> **Execution:** Use subagent-driven development with parallel agents for exclusive-path workstreams. Review at milestone integration boundaries. Do not serialize unrelated work or expand into deferred hardening.

**Goal:** Produce and preserve a Docker Compose run in which ArduPilot SITL controls a Gazebo vehicle through takeoff and landing, all required artifacts are valid, and the committed `descent_v1` score is 100/100.

**Spec:** `docs/superpowers/specs/2026-08-24-runnable-vertical-descent-design.md`

**Foundation:** Phase 3 Tasks 1–5 through commit `416a180` are retained. The remaining Phase 3 Tasks 6–8 are incorporated below rather than completed as an isolated passive-physics phase.

## Execution rules

- Keep the persistent goal active through a verified 100/100 bundle.
- Use the current feature worktree until the first integration checkpoint is green; preserve the untracked nested `companion/comp2026` repository.
- One writer owns a path at a time. The coordinator alone edits Compose, orchestration, root configuration, and integration acceptance.
- Start independent Gazebo, SITL, artifacts/scoring, and mission work as interfaces permit.
- Tests are proportional to runtime risk: focused units first, then actual container/Compose evidence.
- Record deferred hardening explicitly; do not fix it unless it blocks launch, correctness, evidence integrity, or deterministic timing.

## Wave 1 — parallel foundations

### Workstream A: Gazebo ROS runtime and first Compose smoke

**Owns:** `gazebo/**`

- Add the one-way private Gazebo-to-ROS bridge configuration.
- Implement the ROS adapter node around the existing pure adapter model.
- Implement the runtime entry point that resolves config/world, supervises Gazebo and bridge children, applies pure runtime actions, and uses `RuntimeProtocol`.
- Install the local Python package and custom ROS messages in the pinned image without weakening the exact package lock.
- Publish only the existing public `/clock`, camera/metadata, and ground-truth contracts; never consume production camera ACK.
- Add bounded private pose/twist/contact aggregation at the camera-pair timestamp.
- Add a `gazebo-runtime` Compose service through a coordinator-owned integration patch.

**Milestone gate — first Gazebo runtime smoke:**

```bash
docker compose --profile phase3 build gazebo-runtime
uv run pytest tests/integration/test_phase3_gazebo.py -k smoke -v
```

Require Compose startup, paused-first behavior, endpoint readiness, exact 50,000,000 ns frame deltas, exact configured frame count, and nonempty `gazebo/state/state.tlog` plus `gazebo/server.log` after bounded finalization.

### Workstream B: pinned ArduPilot SITL runtime

**Owns:** `ardupilot_sitl/**`

- Add a reproducible image for ArduPilot `Copter-4.7.0` at `1511f27194f1dcc3728270883047bdf022b3fd53`.
- Build only the required SITL target and preserve source/license/provenance.
- Run ArduCopter with the JSON Gazebo frame, `--sim-address=gazebo-runtime`, and no wall-time synchronization.
- Expose MAVLink TCP `5760` only on the Compose network and target the plugin at UDP `9002`.
- Add parameters, structured stdout/stderr events, DataFlash/SITL logs, readiness, first-failure evidence, and quiescence.
- Prove the runtime against a bounded fake JSON peer before Gazebo integration.

**Independent gate:** image builds offline from pinned inputs after supply acquisition; heartbeat is available; JSON frame counters are contiguous; lost peer fails closed with diagnostics.

### Workstream C: config-driven artifacts and `descent_v1` scoring

**Owns:** `artifacts/**`, `scorekeeper/**`, versioned scoring rules.

- Replace every production 40-frame assumption with resolved `expected_camera_frames` while preserving Phase 2 regression behavior.
- Make production Gazebo recording independent of `/simulation/camera_pair_ack`; retain ACK only in the Phase 2 synthetic profile.
- Add descriptor-safe Gazebo state/log validation needed for a completed physical run.
- Implement a pure score model over ordered 50 ms ground truth using the frozen four-rule, 100-point ruleset.
- Publish four rule events plus `score.finalized`; write score result/evidence and rules checksum accepted by the existing manifest authority.
- Unit-test 0, partial, and 100 scores; missing/gapped data cannot claim completion.

**Independent gate:** arbitrary exact-grid duration yields correct video/bag expectations, a score result round-trips into manifest validation, and test ground truth produces exactly 100/100.

## Wave 2 — flight and mission convergence

### Workstream D: flight-capable Gazebo model and official plugin

**Owns:** `gazebo/**` after Workstream A merges.

- Build the provenance-pinned official ArduPilot Gazebo plugin into the runtime image.
- Add a separate flight model/world with IMU, rotors, lift/drag, and four controls; retain project cameras and marker.
- Configure UDP 9002, lockstep on, no SITL wall-time sync, frame transforms, motor channels, and required plugin search path.
- Add observable connection and actuator counters without creating a public control bypass.

**Gate:** actual plugin loads, exchanges contiguous JSON frames with Workstream B, motor outputs change physics, and loss of SITL prevents uncontrolled simulation advance.

### Workstream E: real companion and inactive scenario

**Owns:** `companion/**` excluding `companion/comp2026`, then `electromagnet/**`.

- Implement a pure mission state machine plus thin PyMAVLink/ROS adapter.
- Execute heartbeat → GUIDED → arm → takeoff 1.5 m → LAND → landed/disarmed.
- Advance mission decisions on telemetry and simulation timestamps; use wall deadlines only for absent infrastructure.
- Emit command, ACK, mode, altitude, landing, failure, and quiescence evidence as structured events.
- Implement a production-shaped electromagnet runtime that publishes one truthful `INACTIVE` initialization event and never applies force in `descent_v1`.

**Independent gate:** fake MAVLink tests cover every transition, stale telemetry, negative ACK, timeout, and no wall-time mission scheduling. Scenario event is nonempty and deterministic.

## Coordinator integration

**Owns:** `compose.yaml`, `orchestration/**`, root configuration/interfaces, acceptance tests.

- Freeze exact production service ownership:

  ```text
  orchestration-runtime
  artifacts-runtime
  companion-runtime
  ardupilot-sitl
  gazebo-runtime
  electromagnet-runtime
  scorekeeper-runtime
  ```

- Add durable current-run readiness for Gazebo, ArduPilot exchange, companion heartbeat, and artifacts.
- Make `READY` require all four; preserve one controlled first step and Gazebo-owned `/clock`.
- Add `mission-finished` and `score-finished`; require them with `source-finished` before `COMPLETED`.
- Carry one absolute monotonic finalization deadline without restarting it.
- Use a 60.0 simulated-second default (`1,200` frames per camera); the first
  physical 30-second run proved too short for cold ArduCopter initialization.
- Preserve the fixed ten-topic bag and exactly seven structured module logs.
- Ensure failed/aborted runs still preserve partial evidence and teardown Compose resources.

## Milestone 2 — first ArduPilot-controlled flight

Run the production Gazebo, SITL, and companion together. Require evidence of:

- plugin/SITL connection and contiguous lockstep frames;
- MAVLink heartbeat, mode/arm/takeoff/LAND commands, and ACKs;
- changing actuator outputs;
- ground truth above 1 m, then descending after LAND, then contact;
- no simulation-time advancement when SITL is stopped beyond an already-issued step.

Do not proceed on a passive drop or direct Gazebo control.

## Milestone 3 — first complete artifact-producing run

Start all seven production services through the operator workflow and collect results. The acceptance inspector must prove:

- terminal `COMPLETED` with source, mission, and score terminal facts;
- two playable H.264/yuv420p `320x240` MP4s at 20 FPS and exact configured frame count;
- readable fixed ten-topic MCAP with contiguous IDs/stamps and aligned ground truth;
- nonempty valid `state.tlog` and server log;
- seven parseable module JSONL logs including MAVLink/plugin evidence;
- valid scoring events/result/rules checksum/evidence paths;
- canonical manifest checksums and copied resolved configuration;
- no leftover run containers or networks.

## Milestone 4 — first valid scored run

Run the actual read-only scorekeeper over live ground truth. Require `complete=true`, `0 <= achieved <= maximum == 100`, five score events, and independently verified evidence references. Synthetic scoring is forbidden.

## Milestone 5 — verified maximum-score run

- Freeze seed, world, 30-second duration, rules file, images, and supply pins.
- Inspect the lowest failed rule from score events, ground truth, ArduPilot/companion logs, and both videos.
- Change only companion mission parameters/timing or recorded ArduPilot parameters.
- Rebuild only the affected image and rerun; preserve every completed attempt.
- Stop when a preserved bundle independently verifies `achieved_score == maximum_available_score == 100.0`.
- Repeat the same configuration under injected host wall slowdown; compare simulation timestamp sequence, counts, normalized ground truth/events, and score.
- Record the winning run ID, manifest checksum, rules checksum, video/bag/state checksums, and verification command output in `docs/verification/`.

## Deferred-hardening ledger

Maintain `docs/technical-debt/vertical-slice-hardening.md`. Seed it with:

- extra-camera enumeration;
- adversarial filesystem substitutions beyond Task 5;
- unreproduced narrow process identity races;
- exhaustive protocol/fault matrices;
- rendering-host byte determinism;
- active electromagnet physics and broader competition scoring;
- legacy vision/FMs, precision landing, LiDAR/dropper, multi-vehicle, GUI, and distributed telemetry.

None may silently become a prerequisite unless runtime evidence shows it blocks launch, correctness, deterministic timing, artifact integrity, or truthful scoring.
