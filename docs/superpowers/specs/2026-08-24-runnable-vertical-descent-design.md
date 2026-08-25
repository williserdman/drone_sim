# Runnable Vertical Descent Design

**Date:** 2026-08-24

## Objective

Prioritize a verified end-to-end result over completing infrastructure phases in isolation. The preserved success artifact is one deterministic Docker Compose run in which a real ArduPilot SITL controls a Gazebo vehicle through a basic takeoff and descent, the seven production modules quiesce, all required evidence is committed, and the versioned `descent_v1` score is `100/100`.

Completed Phase 3 Tasks 1–5 remain the foundation. Remaining Phase 3, ArduPilot, mission, scenario, scoring, and artifact work is reorganized into one vertical slice.

## Non-negotiable boundaries

- Gazebo owns physics, sensors, both cameras, ground truth, and `/clock`.
- ArduPilot SITL owns estimation, navigation, and actuator commands.
- The companion uses MAVLink; it never manipulates Gazebo directly.
- The scorekeeper is read-only and scores Gazebo ground truth.
- The electromagnet publishes a truthful, permanently inactive initial scenario for `descent_v1`; it applies no force in this slice.
- Simulation decisions use simulation time. Wall time may only bound startup, shutdown, discovery, and unavailable infrastructure.
- Both cameras publish `320x240 rgb8` at exactly 20 frames per simulated second with contiguous IDs and 50,000,000 ns timestamp deltas.
- A completed bundle contains both MP4s, the ten-topic MCAP bag, Gazebo native state and server log, seven structured logs, scoring evidence, configuration provenance, and the manifest.
- `companion/comp2026` remains unmodified and uncommitted.

## Production topology

The production profile has exactly seven services on the default Compose network and publishes no host ports:

```text
orchestration-runtime
artifacts-runtime
companion-runtime
ardupilot-sitl
gazebo-runtime
electromagnet-runtime
scorekeeper-runtime
```

The first Gazebo smoke milestone may retain the four Phase 2 test doubles outside Gazebo. No `synthetic-*` service is permitted in the preserved scored run.

## Flight seam

Freeze the first compatibility target as:

- ArduPilot tag `Copter-4.7.0`, commit `1511f27194f1dcc3728270883047bdf022b3fd53`.
- Official `ArduPilot/ardupilot_gazebo` source pinned by committed provenance.
- Gazebo plugin UDP bind `0.0.0.0:9002`.
- SITL JSON model targets `gazebo-runtime:9002`.
- Plugin configuration includes `<lock_step>1</lock_step>` and `<no_time_sync>1</no_time_sync>`.
- Companion connects by MAVLink TCP to `ardupilot-sitl:5760`.
- Gazebo's authoritative environment adds a pinned `GZ_SIM_SYSTEM_PLUGIN_PATH` only after the built plugin path is proven in the runtime image.

The flight-capable model is separate from the passive Phase 3 model so the existing physical-foundation evidence remains reproducible. Selectively import only the required upstream IMU, rotor joints, lift/drag configuration, and four motor controls. Preserve the two project-owned cameras and landing marker.

Lockstep acceptance requires runtime evidence that actuator outputs advance one-for-one with the JSON exchange and that stopping SITL stops further simulation-time advancement. Host slowdown may increase wall duration but must not change normalized simulation facts.

## Mission

The first real companion is a small event-driven PyMAVLink module outside the nested legacy repository. Before the public mission epoch, it passively observes heartbeat and `MAV_SYS_STATUS_PREARM_CHECK`; it issues no command until `RUNNING` and the first public clock have both been observed:

```text
wait heartbeat
  -> set GUIDED and observe ACK/mode
  -> arm and observe armed state
  -> take off to 1.5 m
  -> issue LAND
  -> observe descent, contact, landed state, and disarm
```

Mission transitions are driven by MAVLink telemetry stamped/correlated with ROS simulation time. Wall deadlines only fail unavailable infrastructure. The initial full-run configuration used 30 simulated seconds and produced exactly 600 frames per camera. Physical cold-start evidence showed ArduCopter application initialization could outlast that horizon, so the production default is 60 simulated seconds, producing exactly 1,200 frames per camera on the same 50 ms grid.

## Lifecycle

Keep the existing durable status and quiescence protocol and add current-run facts:

- `gazebo-ready`
- `ardupilot-ready`
- `companion-ready`
- `mission-ready`
- `mission-finished`
- `score-finished`

`READY` requires artifact recorders, Gazebo endpoints/native recording, active ArduPilot–Gazebo exchange, and the companion's successful MAVLink TCP connection. `READY` starts a private unscored warmup: Gazebo and ArduPilot advance in lockstep, while public `/clock`, cameras, ground truth, scenario, score, and mission commands remain inactive. The companion writes `mission-ready={run_id,ready:true,heartbeat_observed:true,prearm_checks_healthy:true}` exactly once after it has passively observed both facts. Orchestration publishes `RUNNING` only after `ardupilot-ready`, `companion-ready`, and `mission-ready` are durable.

At `RUNNING`, the Gazebo adapter floors the latest native Gazebo time to the preceding 50 ms camera epoch, publishes public `/clock=0`, and rebases later public timestamps against that epoch without resetting Gazebo or ArduPilot. Queued native samples at or before the epoch are discarded. The first public camera pair and ground-truth sample is frame 0 at 50 ms; frame 1199 is at 60.000 seconds. Native Gazebo state/log timestamps remain unchanged and include warmup. This makes host-sensitive flight-controller initialization visible diagnostically but unable to consume the fixed scored interval.

`COMPLETED` requires all of:

```text
source-finished + mission-finished(LANDED) + score-finished
```

`source-finished` alone is not mission completion. Finalization carries one absolute monotonic infrastructure deadline without restarting it across durable/runtime boundaries. Every module stops output before publishing quiescence.

## Recording contract

For duration `D` seconds, `N = 20 * D`:

- `/clock`: zero-based, monotonic public simulation time rebased from native Gazebo time, with `0` at mission activation and every frame stamp represented;
- `/simulation/run_state`: lifecycle samples through terminal transition;
- `/simulation/artifact_status`: recorder readiness/finalization samples;
- `/simulation/ground_truth`: exactly `N` aligned samples;
- each image and metadata topic: exactly `N` samples;
- `/simulation/scenario_events`: at least one truthful `INACTIVE` initialization event;
- `/simulation/score_events`: four rule results plus `score.finalized`.

The bag remains the fixed ten-topic public contract. MAVLink commands, ACKs, telemetry, and ArduPilot JSON frame counters are preserved initially in companion/ArduPilot structured logs rather than expanding the bag.

Camera publishers and the private rosbag subscriptions use reliable, volatile,
100-sample histories. At 20 simulated Hz this retains five simulated seconds
through a bounded host-side writer stall; exact frame acceptance still fails
closed on any loss.

The production artifact runtime derives `N` from resolved configuration. It does not use the Phase 2 hard-coded 40-frame assumption and does not require `/simulation/camera_pair_ack` to advance production physics.

## Scoring

Freeze a repository-owned `descent_v1` rules file with maximum 100:

| Points | Rule |
| ---: | --- |
| 20 | Ground truth rises above 0.5 m and later reaches first contact. |
| 40 | First touchdown is within 0.5 m horizontal radius of the landing marker center. |
| 20 | Pre-impact downward speed from the last contiguous non-contact sample is at or below the frozen safe threshold. |
| 20 | Contact remains continuous for 0.5 simulated seconds with speed at or below 0.1 m/s and tilt at or below 10 degrees. |

The implementation must freeze the safe touchdown threshold in the rules file before mission tuning. All decisions use ordered 50 ms ground-truth samples. A gap makes scoring incomplete; no later sample repairs it. The scorekeeper emits one event per rule and one final event, then writes a result containing the ruleset ID, finite achieved/maximum values, rules checksum, and safe evidence paths.

“Maximum score” means `100/100` under the committed `descent_v1` configuration. Broader competition, active-magnet, payload, or precision-vision scoring remains outside this slice until a separate authoritative ruleset is supplied.

## Milestones and evidence

### 1. First Gazebo runtime smoke run

Compose launches `gazebo-runtime`; its server starts paused; required private endpoints and six public ROS outputs exist; a controlled short run yields exact 50 ms camera deltas plus nonempty native state and server log.

### 2. First ArduPilot-controlled flight

The plugin and SITL connect; heartbeat and command ACKs are observed; motor outputs change; ground truth rises above 1 m; LAND precedes decreasing altitude and contact; stopping SITL stops simulation progress.

### 3. First complete artifact-producing run

The seven production services complete and preserve two playable MP4s, readable ten-topic MCAP, Gazebo state/log, seven JSONL logs, scoring files, configuration, checksums, and a `COMPLETED` manifest.

### 4. First valid scored run

The score is complete, bounded, checksum-backed, and traceable to ground-truth evidence.

### 5. Verified maximum-score run

With seed, world, duration, and ruleset frozen, change only mission parameters or recorded ArduPilot parameters until `achieved_score == maximum_available_score == 100`. Preserve the first verified bundle and its run ID/checksums. Repeat it under injected wall slowdown and compare normalized timestamps, ground truth, events, counts, and score.

## Parallel ownership

At stable interface boundaries, use independent writers with exclusive paths:

- Gazebo/ROS/flight model: `gazebo/**`.
- SITL supply/runtime: `ardupilot_sitl/**`.
- Artifact duration/scoring: `artifacts/**`, `scorekeeper/**`, scoring rules.
- Mission/scenario: `companion/**` excluding `companion/comp2026`, and `electromagnet/**`.
- Coordinator integration only: `compose.yaml`, `orchestration/**`, root configuration/interfaces, end-to-end acceptance.

Review at milestone integration boundaries. Fix Critical/Important findings that prevent launch, corrupt evidence, violate deterministic timing, or permit a false maximum-score claim.

## Deferred technical debt

Do not gate the vertical slice on:

- extra-camera enumeration beyond the fixed public pair;
- adversarial path/inode substitutions beyond completed Task 5 fixes;
- unreproduced extremely narrow process races;
- exhaustive malformed-protocol/fault matrices;
- byte-identical encoded MP4 output across rendering hosts;
- legacy FM2/FM3, precision vision, LiDAR, payload/dropper, or active magnet physics;
- multi-vehicle support, GUI, distributed observability backends, or network-partition stress.
- a reusable Phase 3 slowdown-injection flag and normalized two-run comparator;
- launch-time provenance snapshots and external pinning of every canonical
  configuration field and the acceptance-validator image.

Do not defer simulation-time continuity, lockstep loss behavior, exact 20 sim FPS, artifact closure, scoring provenance, or safe preservation of the workspace and completed run.
