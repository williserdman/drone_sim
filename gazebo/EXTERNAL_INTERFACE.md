# Gazebo External Interface

## Phase 3 scope

One run-scoped `gazebo-runtime` service owns the headless Gazebo Harmonic
server, private Gazebo Transport endpoints, bridges, public ROS adapter, native
state, and raw server log. Gazebo is the sole producer of physical truth and
simulation time. Gazebo Transport is not a repository-wide interface.

## ROS 2 outputs

| Topic | Type | QoS |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | Best effort, volatile, depth 1 |
| `/camera/onboard/image_raw` | `sensor_msgs/msg/Image` | Reliable, volatile, depth 5 |
| `/camera/onboard/frame_metadata` | `simulation_interfaces/msg/FrameMetadata` | Reliable, volatile, depth 5 |
| `/camera/observer/image_raw` | `sensor_msgs/msg/Image` | Reliable, volatile, depth 5 |
| `/camera/observer/frame_metadata` | `simulation_interfaces/msg/FrameMetadata` | Reliable, volatile, depth 5 |
| `/simulation/ground_truth` | `simulation_interfaces/msg/GroundTruth` | Best effort, volatile, depth 10 |

All run-scoped outputs carry `run_id`. Each image has matching metadata with a
stream-local contiguous frame ID and an identical native simulation timestamp.
Both streams are fixed at `320x240`, `rgb8`, and 20 simulated Hz. Each accepted
camera pair has one ground-truth sample at the same native timestamp. The
onboard public image is the exact stream later consumed by companion vision and
artifacts.

`GroundTruth.pose` and its linear and angular velocity are expressed in the
Gazebo world frame using ENU axes. `GroundTruth.in_contact` is true when the
Iris carrier has current-epoch contact with a non-vehicle entity.

## Lifecycle inputs

The runtime consumes the current run's resolved configuration, lifecycle state,
artifact readiness, and finalization request. It starts the server paused and
releases no simulation sample before its private endpoints, native recorder,
bridges, public publishers, and artifact recorders are ready. After `READY`,
one controlled step establishes the first `/clock`; the server unpauses only
after observing the current run's `RUNNING` state.

The Phase 2 synthetic source consumes the transport-only
`/simulation/camera_pair_ack` contract documented by artifacts. The production
Phase 3 Gazebo runtime never subscribes to or waits for that topic. Recorder
acknowledgement cannot govern production physics advancement.

## Completion and finalization

At the configured integer-nanosecond duration, Gazebo pauses after both final
camera/metadata streams and aligned ground truth are complete, then writes the
current run's source-finished fact. On `FINALIZING`, it stops public samples,
drains bounded adapter work, stops bridges and the server, freezes native state
and `gazebo/server.log`, and writes only `.status/quiescence/gazebo.json`.
Orchestration retains aggregate freeze authority; artifacts retains manifest
authority.

## Reset, timing, and failure behavior

Reset destroys the run-scoped container/server and starts a fresh server from
immutable `phase3_foundation.sdf` under a new Compose project and Gazebo
partition. There is no public in-process reset endpoint. Stale-run or foreign
partition data cannot satisfy readiness or completion. When paused, `/clock`,
images, metadata, pose, and ground truth do not advance.

Malformed native data, missing endpoints, child exit, clock stall, camera
discontinuity, or recorder failure emits durable run-failure evidence and
preserves available native diagnostics.

The production executable is `drone-sim-gazebo-runtime`. It requires
`SIM_RUN_ID`, `SIM_RUN_DIRECTORY`, and the resolved configuration at
`SIM_CONFIG_PATH` (defaulting to `configuration/run.json` in that run). The
image owns `/etc/drone_sim/gazebo-bridge.yaml` and immutable resources at
`/opt/drone_sim/gazebo/resources`. It discovers the actual Harmonic clock,
camera, odometry, contact, and world-control endpoints before publishing
`.status/gazebo-ready.json`.

On the first valid finalization request the runtime converts the durable intent
and resolved finalization allowance into one absolute monotonic deadline.
Repeated reads reuse that deadline; bridge, server, and native-log stages do
not receive restarted budgets.

## Excluded and reserved interfaces

Phase 3 has no ArduPilot actuator/sensor seam and does not claim lockstep.
ArduPilot SITL, MAVLink, motor dynamics, NED conversion, and ArduPilot-Gazebo
lockstep are reserved for Phase 4. Electromagnet physical-effect requests,
payload behavior, course policy, and competition scoring remain Phase 6 work.
