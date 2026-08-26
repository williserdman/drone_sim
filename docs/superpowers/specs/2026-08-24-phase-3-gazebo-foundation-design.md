# Phase 3 Gazebo Physical Foundation Design

**Status:** Approved default design

**Date:** 2026-08-24

**Parent design:** `docs/superpowers/specs/2026-08-22-runnable-simulation-design.md`

## Purpose

Phase 3 replaces the Phase 2 synthetic physical source with a real Gazebo
Harmonic server while preserving the lifecycle, ROS 2, recording, manifest,
and observability contracts already proven in Phase 2. It establishes Gazebo
as the sole owner of physical truth and simulation time. It does not yet fly
the aircraft or run the competition mission.

The acceptance question is narrow and physical: can the existing seven-module
run lifecycle record a deterministic Gazebo world, two 20-sim-Hz camera
streams, ground truth, native state, and server logs when the host runs slower
than real time?

## Scope

### In scope

- A headless Gazebo Harmonic server started paused in one run-scoped Gazebo
  container.
- A minimal deterministic world containing a ground plane, landing marker,
  and an Iris visual carrier.
- A downward-looking onboard camera and a fixed observer camera.
- Gazebo-owned `/clock`, camera capture timestamps, pose, twist, contact state,
  native state recording, and server logging.
- A thin ROS 2 adapter that exposes the existing public contracts with the
  required QoS and run metadata.
- Deterministic run isolation, seed handling, startup readiness, completion,
  finalization, and quiescence.
- A `phase3` Compose profile containing exactly the same seven logical module
  owners as Phase 2, with only the synthetic Gazebo service replaced by the
  production Gazebo runtime.
- Successful completed, failed, and aborted bundles at slow real-time factors.
- Selective import of reviewed Iris visual assets with exact provenance.

### Out of scope

- ArduPilot SITL, MAVLink, actuator exchange, motor dynamics, NED conversion,
  and ArduPilot-Gazebo lockstep. Those are Phase 4.
- Companion mission and vision integration. That is Phase 5.
- Electromagnet forces, payload attachment, course policy, competition scoring,
  and the maximum-score run. Those are Phase 6.
- LiDAR, gimbals, GUI operation, playback tooling, variable camera geometry,
  or byte-identical rendered pixels across GPU and Mesa versions.
- In-process reuse of a world across run IDs.

## Approaches considered

### 1. One Gazebo module container with stock systems and a thin adapter

This is the selected approach. The container owns the server process, private
Gazebo Transport endpoints, ROS bridges, public ROS adapter, lifecycle
coordination, native state, and raw server log. One owner can prove readiness
and quiescence without cross-container Gazebo discovery races.

### 2. Separate Gazebo server and ROS gateway containers

This gives process isolation but splits one logical module across two Docker
failure domains. Gazebo discovery, native-log ownership, shutdown ordering, and
camera-loss attribution all become harder without improving the public module
boundary.

### 3. A custom Gazebo C++ system plugin that publishes all ROS contracts

This provides direct access to the entity-component manager but couples
Gazebo, ROS 2, and compiler ABIs. Phase 3 does not need that complexity. A
custom system plugin remains an option only if Phase 4 lockstep proves that the
stock transport seam cannot provide deterministic stepping.

## Authoritative boundaries

Gazebo is the only producer of physical time and truth. Gazebo Transport stays
private to the Gazebo module. Other modules consume the stable ROS 2 and
durable-file interfaces documented at the repository and module boundaries.

The production Gazebo runtime does not consume
`/simulation/camera_pair_ack`. That topic is a Phase 2 test-transport device;
physics must not advance because an archival recorder acknowledged a frame.
Instead, startup prevents the first step until required subscribers are ready,
and reliable ROS delivery plus bounded recorder queues handle the running
stream.

The Phase 3 container may contain multiple supervised processes, but Compose
still sees one `gazebo-runtime` service and the rest of the system sees one
Gazebo module.

## Directory and module structure

```text
gazebo/
├── Dockerfile
├── harmonic-packages.lock
├── pyproject.toml
├── EXTERNAL_INTERFACE.md
├── INTERNAL_INTERFACE.md
├── PLAN.md
├── provenance/
│   ├── EXTERNAL_INTERFACE.md
│   ├── INTERNAL_INTERFACE.md
│   ├── LICENSE.ardupilot_gazebo.md
│   └── ardupilot_gazebo-assets.json
├── resources/
│   ├── models/iris_phase3/
│   └── worlds/phase3_foundation.sdf
├── src/drone_sim_gazebo/
│   ├── models/
│   │   ├── EXTERNAL_INTERFACE.md
│   │   └── INTERNAL_INTERFACE.md
│   ├── ros_adapter/
│   │   ├── EXTERNAL_INTERFACE.md
│   │   └── INTERNAL_INTERFACE.md
│   ├── runtime/
│   │   ├── EXTERNAL_INTERFACE.md
│   │   └── INTERNAL_INTERFACE.md
│   ├── server/
│   │   ├── EXTERNAL_INTERFACE.md
│   │   └── INTERNAL_INTERFACE.md
│   └── worlds/
│       ├── EXTERNAL_INTERFACE.md
│       └── INTERNAL_INTERFACE.md
├── tests/
└── ...
```

`worlds` and `models` resolve and validate immutable image resources. `server`
owns the Gazebo subprocess and its private transport. `ros_adapter` validates
and maps native messages to public ROS contracts. `runtime` owns the module
lifecycle and supervises the other three units. These are sibling subpackages
under one unambiguous `drone_sim_gazebo` import root. Each directory exposes
one internal and one external interface document even when the external
interface is explicitly empty.

## Versions and reproducibility

The image inherits the repository's immutable ROS base:

```text
ros:jazzy-ros-base@sha256:2589a8fba5257307857890173c069852c2abf913a0be7970f172478baecb09e4
```

The build installs Gazebo Harmonic / Gazebo Sim 8 and ROS Jazzy `ros_gz` 1.0.22
packages. Direct package versions are pinned, and the complete newly installed
package delta is committed in `gazebo/harmonic-packages.lock` and verified at
image build time. Mutable legacy images and locally built shared libraries are
not inputs.

Runtime uses a unique `GZ_PARTITION` derived from the canonical run ID. It does
not inherit an ambient Gazebo partition, ROS domain selection, Compose profile,
or resource path. Image-owned resource paths are explicit.

## Selective legacy asset reuse

The only Phase 3 legacy import is the Iris visual/collision asset set from
`ArduPilot/ardupilot_gazebo` commit
`082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`:

- `models/iris_with_standoffs/model.config`
- `models/iris_with_standoffs/model.sdf`
- `meshes/iris.dae`
- `meshes/iris_collision.stl`
- `meshes/iris_prop_ccw.dae`
- `meshes/iris_prop_cw.dae`
- upstream `LICENSE.md`

The repository records the upstream URL, commit, original paths, imported
paths, license, and SHA-256 for every file in a machine-readable provenance
manifest. Phase 3 adapts the model locally only to remove ArduPilot coupling
and add the camera sensor. It does not import a compiled plugin.

The legacy competition world, runway textures, generated course, payload and
gimbal plugins, attempt/referee clocks, recorder scripts, Compose file,
diagnostic dumps, and dirty generated artifacts are rejected. Useful SDF and
bridge configuration patterns may inform new code but are not copied from the
dirty transfer repository.

## Run configuration

The resolved run configuration gains:

```json
{
  "runtime_profile": "phase3",
  "simulation": {
    "seed": 1,
    "duration_sim_seconds": 2.0,
    "public_epoch_native_sim_seconds": 90.0,
    "target_real_time_factor": 0.1
  }
}
```

- `runtime_profile` is a validated repository-owned topology selector. Phase 3
  accepts only `phase3`; the Phase 2 synthetic test fixture resolves an omitted
  value to `phase2` for backward-compatible infrastructure regression tests.
- `seed` is an integer from `0` through `4294967295`.
- `duration_sim_seconds` is a positive finite number whose nanosecond value is
  an exact multiple of the fixed 50,000,000 ns camera interval.
- `public_epoch_native_sim_seconds` is required in Phase 3 and defaults to
  `90.0`; its exact positive finite integer-nanosecond value is on the 50 ms
  grid and is the fixed native activation target.
- `target_real_time_factor` is frozen at `0.1` in Phase 3. It controls the
  immutable fixture's physics pacing target, not application event timestamps.
- The existing `320x240`, `rgb8`, 20 FPS recording contract remains frozen.
- Expected frames per stream equal
  `duration_sim_seconds * recording.fps`; Phase 3 defaults yield 40.

The normalized resolved configuration, including seed, participates in
`config_sha256` and is copied into the bundle.

## World and sensors

`phase3_foundation.sdf` is a local, server-only SDF world with deterministic
physics settings, fixed model names, an explicit random seed supplied at
startup, and no remote model lookup.

The Iris carrier begins at a fixed pose above the landing marker. It is a
passive rigid body in Phase 3; gravity produces observable physical evolution.
Phase 4 replaces or extends its control seam without changing public camera or
ground-truth topics.

Both native cameras use:

```text
width:       320
height:      240
format:      R8G8B8
update rate: 20 simulated Hz
```

The onboard camera is rigidly attached and points downward. The observer camera
is fixed in the world and frames the carrier and landing marker. Rendering uses
headless software-compatible settings so the gate does not require a display
or GPU. Pixel and H.264 checksums are not determinism claims.

Ground-truth source data contains world-frame ENU pose, world-frame linear and
angular velocity, and contact state for the Iris carrier. Phase 3 performs no
ENU/NED conversion.

## ROS 2 transport

Gazebo publishes its native world clock, camera, pose/odometry, and contact
data on private Gazebo Transport names. Bridges are unidirectional
Gazebo-to-ROS. No ROS-to-Gazebo `/clock` route exists.

The public adapter publishes:

| Topic | Type | QoS |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | best effort, volatile, depth 1 |
| `/camera/onboard/image_raw` | `sensor_msgs/msg/Image` | reliable, volatile, depth 5 |
| `/camera/onboard/frame_metadata` | `simulation_interfaces/msg/FrameMetadata` | reliable, volatile, depth 5 |
| `/camera/observer/image_raw` | `sensor_msgs/msg/Image` | reliable, volatile, depth 5 |
| `/camera/observer/frame_metadata` | `simulation_interfaces/msg/FrameMetadata` | reliable, volatile, depth 5 |
| `/simulation/ground_truth` | `simulation_interfaces/msg/GroundTruth` | best effort, volatile, depth 10 |

The public adapter receives images on private ROS topics, validates `320x240`
`rgb8` geometry and native simulation timestamps, assigns per-stream contiguous
frame IDs starting at zero, and publishes each image with matching metadata.
The image header stamp and metadata `sim_timestamp` are identical. Native
capture times must be exactly 50,000,000 ns apart after the first capture.
Duplicate, missing, regressing, or off-grid samples fail the run rather than
being silently repaired.

Pose/twist publication is configured at the same native 20-sim-Hz cadence.
After validating an onboard/observer camera pair, the adapter requires a
pose/twist/contact sample at that exact native timestamp and publishes exactly
one `GroundTruth` sample for the pair. A missing or mismatched physical sample
fails closed. A 40-frame Phase 3 run therefore contains 40 aligned ground-truth
samples rather than an unrelated high-rate pose stream.

The onboard public image is the exact image later consumed by companion vision
and by artifacts; there is no separately rendered or transformed mission
stream.

`GroundTruth` maps as follows:

- `run_id`: resolved canonical run ID.
- `sim_timestamp`: native Gazebo sample time.
- `vehicle_id`: `iris`.
- `pose`: Gazebo world ENU pose.
- `twist`: world-frame linear and angular velocity.
- `in_contact`: true when the vehicle has a current-epoch collision with a
  non-vehicle entity.

## Lifecycle and readiness

The server starts without `-r`, so physics is paused. Startup proceeds in this
order:

1. Validate configuration, run directory ownership, run ID, partition, world,
   resource checksums, and output paths.
2. Start `gz sim -s --headless-rendering --seed <seed> --record-path
   <run>/gazebo/state <world>` with stdout and stderr captured to
   `gazebo/server.log.partial`.
3. While paused, discover world-control, clock, both cameras, pose/odometry,
   contact, and native recording endpoints.
4. Start private bridges and the public adapter. Public publishers exist but
   no simulation sample has been released.
5. The existing artifacts runtime starts its rosbag and two video recorders and
   writes `artifacts-ready` after discovering the four reliable camera
   publishers.
6. Gazebo writes `.status/gazebo-ready.json` only after the server, bridges,
   native recorder, and public publishers are ready.
7. Orchestration publishes `READY` only after both readiness records.
8. Gazebo receives `READY` and requests exactly one world step. The resulting
   first `/clock` sample allows orchestration to publish `RUNNING` and persist
   `runtime-running.json`.
9. After observing the current run's `RUNNING`, Gazebo unpauses.

All startup waits use the configured infrastructure wall-clock deadline.
Simulation timestamps never come from those waits.

## Completion, pause, reset, and failures

Completion is simulation-driven. When both streams have published the expected
final camera pair and ground truth has reached the configured duration, the
runtime pauses Gazebo and writes the existing current-run `source-finished`
status with the final simulation timestamp. Orchestration then performs its
normal terminal transition.

Phase 3 reset means destroying the run-scoped server/container and starting a
new server from immutable SDF with a new Compose project and Gazebo partition.
There is no public in-process reset endpoint. This prevents old entity,
transport, sensor, or plugin state from leaking across run IDs.

Stale run IDs, duplicate readiness, foreign partition data, malformed native
messages, child-process exit, missing endpoints, clock stalls, camera gaps,
and native-recorder failures produce a durable `runtime-failure` fact. The
orchestrator remains the sole authority for choosing the terminal status.

On `FINALIZING`, the Gazebo runtime:

1. Pauses the world if possible.
2. Stops accepting and publishing new public samples.
3. Stops bridges and drains bounded adapter callbacks.
4. Gracefully stops Gazebo within the remaining finalization deadline.
5. Fsyncs native state files and the partial server log.
6. Atomically publishes `gazebo/server.log` from the partial log.
7. Writes only `.status/quiescence/gazebo.json` with the current run ID.

If graceful shutdown fails, the runtime escalates within the same bounded
deadline and preserves partial native evidence. It never writes the aggregate
runtime freeze or manifest; those authorities remain with orchestration and
artifacts.

## Native artifacts and observability

Gazebo Sim records state directly to `gazebo/state/` using `--record-path`.
The directory must contain a nonempty native `state.tlog`; console output is
captured separately as `gazebo/server.log`. Artifacts validates the directory,
file types, containment, link counts, nonempty state log, and final server log
before committing manifest records.

The Gazebo module emits the same structured JSONL envelope used by every other
module. Required events include configuration resolved, server starting,
server ready, bridges ready, adapter ready, Gazebo ready, first step requested,
running, frame accepted, ground truth accepted, completion reached, finalizing,
server stopped, native artifacts frozen, and quiescent. Fields distinguish
simulation timestamps from wall-clock durations and include seed, world
checksum, child exit status, frame counts, and terminal diagnostics where
applicable.

Raw child stdout/stderr belongs in the raw server log, not in the structured
JSONL stream.

## Compose and orchestration integration

Compose gains an explicit `phase3` profile with exactly seven services:

```text
orchestration-runtime
artifacts-runtime
synthetic-companion
synthetic-ardupilot-sitl
gazebo-runtime
synthetic-electromagnet
synthetic-scorekeeper
```

The five non-Gazebo services remain controlled Phase 3 test doubles. Their
names and logs continue to represent their logical module owners; no Phase 3
claim is made about mission, flight-control, scenario, or scoring behavior.

The run controller selects the profile from a validated internal phase plan,
not from ambient `COMPOSE_PROFILES`. Phase 2 remains an explicit synthetic test
profile. Health checking, image-digest collection, log collection, shutdown,
and exact-service validation operate on the selected immutable service map.

Phase 3 adds `gazebo-ready` to the startup barrier. It does not weaken the
existing artifacts-ready, first-clock, runtime-frozen, terminal-notified, or
manifest authority contracts.

## Acceptance and verification

Phase 3 is complete only when all existing unit, Phase 1, and Phase 2 gates
still pass and a new `make test-phase3` gate proves all of the following:

1. A normal 2.0-sim-second run produces 40 frames per stream, contiguous IDs,
   exact 50 ms simulation spacing, 40 timestamp-aligned ground-truth samples,
   timestamp, a readable fixed ten-topic bag, two playable 320x240 20 FPS MP4s,
   seven structured module logs, nonempty `gazebo/server.log`, native
   `gazebo/state/state.tlog`, synthetic scoring artifacts, and a valid completed
   manifest.
2. A target real-time factor of 0.1 changes wall duration but not simulation
   frame counts, IDs, or timestamps.
3. Holding the world paused for several wall seconds after recorder readiness
   produces no `/clock`, image, metadata, or ground-truth advancement.
4. Two fresh runs with identical resolved configuration and seed have equal
   world/resource checksums, model identities, simulation timestamps, frame
   IDs, and ground-truth evolution within exact serialized numeric values.
   Rendered pixels, MP4 bytes, wall timestamps, and native-log container bytes
   are excluded from equality.
5. A different seed changes the resolved configuration checksum and is visible
   in structured startup evidence and the raw server invocation.
6. Clock stall, camera discontinuity, Gazebo child exit, and finalization while
   paused each produce bounded terminal handling and preserve a diagnosable
   failed or aborted bundle with native evidence where it was available.
7. Two consecutive runs use distinct Compose projects and Gazebo partitions;
   stale facts cannot satisfy readiness, completion, or quiescence.
8. The Phase 3 image uses the pinned base, exact package lock, local resources,
   and verified asset provenance, and has no ArduPilot plugin or legacy runtime
   scripts.

Passing this gate proves a Gazebo physical foundation. It does not prove
ArduPilot lockstep, mission success, competition scoring, or the required final
maximum-score run.

## Source basis

The design follows the official ROS Jazzy `ros_gz` model: Gazebo Transport is
bridged into ROS 2, `/clock` is unidirectional from Gazebo, and image bridging
supports an explicit ROS QoS. Gazebo Harmonic's native recorder produces a
`state.tlog`, while its world-control service provides pause and step behavior.
The implementation plan must verify the exact installed Jazzy/Harmonic command
and message syntax in the pinned image rather than relying on legacy Classic
Gazebo conventions.

Primary references:

- ROS Jazzy `ros_gz_image`: <https://docs.ros.org/en/jazzy/p/ros_gz_image/>
- ROS Jazzy `ros_gz_bridge`: <https://docs.ros.org/en/ros2_packages/jazzy/api/ros_gz_bridge/>
- Gazebo Sim 8 native logging: <https://gazebosim.org/api/sim/8/log.html>
- Gazebo Sim server clock and lifecycle API: <https://github.com/gazebosim/gz-sim/blob/gz-sim8/include/gz/sim/Server.hh>
