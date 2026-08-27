# Gazebo External Interface

## Runtime scope

One run-scoped `gazebo-runtime` service owns the headless Gazebo Harmonic
server, private Gazebo Transport endpoints, bridges, public ROS adapter, native
state, and raw server log. Gazebo is the sole producer of physical truth and
simulation time. Gazebo Transport is not a repository-wide interface.

Three immutable selections are accepted. `phase3_foundation/iris` is passive,
`vertical_descent/iris_flight` is the descent world, and
`comp2026_course/iris_flight` is the competition world. The physical vehicle
entity and private topic child remain named `iris`.

## ROS 2 outputs

| Topic | Type | QoS |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | Reliable, volatile, depth 1000 |
| `/camera/onboard/image_raw` | `sensor_msgs/msg/Image` | Reliable, volatile, depth 100 |
| `/camera/onboard/frame_metadata` | `simulation_interfaces/msg/FrameMetadata` | Reliable, volatile, depth 100 |
| `/camera/observer/image_raw` | `sensor_msgs/msg/Image` | Reliable, volatile, depth 100 |
| `/camera/observer/frame_metadata` | `simulation_interfaces/msg/FrameMetadata` | Reliable, volatile, depth 100 |
| `/simulation/ground_truth` | `simulation_interfaces/msg/GroundTruth` | Reliable, volatile, depth 10 |
| `/simulation/payload_state` | `simulation_interfaces/msg/PayloadState` | Reliable, volatile, depth 100 |
| `/competition/range/downward` | `sensor_msgs/msg/LaserScan` | Reliable, volatile, depth 100 |

All run-scoped outputs carry `run_id`. Each image has matching metadata with a
stream-local contiguous frame ID and an identical public simulation timestamp.
Both streams are `320x240` for descent or `640x480` for competition, `rgb8`,
and 20 simulated Hz. Each accepted
camera pair has one ground-truth sample at the same public timestamp. The
onboard public image is the exact stream later consumed by companion vision and
artifacts. Camera history retains five simulated seconds so a bounded host-side
MCAP writer stall cannot evict unacknowledged archival evidence. Before
`RUNNING`, none of these topics publishes warmup evidence.
`simulation.public_epoch_native_sim_seconds` is required in Phase 3 and
defaults to `90.0`; its exact configured positive integer-nanosecond value is
the native activation target. `RUNNING` arms output before that target. If
activation is late or reliable native `/clock` skips the exact target, the
adapter fails closed. It drops native samples at or before the target, then
publishes `/clock=0` before draining a bounded pre-zero queue of at most two
public epochs (six already-validated frame/truth outputs); overflow faults.
The first public camera/truth sample at target + 50 ms maps to public 50 ms.
Frame 0 is therefore at public 50 ms; for a 60 second run, frame 1199 is at
public 60.000 seconds, and no later private clock is public.

`GroundTruth.pose` and its linear and angular velocity are expressed in the
Gazebo world frame using ENU axes. `GroundTruth.in_contact` is true when the
Iris carrier has current-epoch contact with a non-vehicle entity.

## Lifecycle inputs

The runtime consumes the current run's resolved configuration, lifecycle state,
artifact readiness, and finalization request. It starts the server paused and
releases no simulation sample before its private endpoints, native recorder,
bridges, public publishers, and artifact recorders are ready. After `READY`,
the server unpauses so Gazebo and ArduPilot can advance in private lockstep,
while the adapter waits for its configured native target. `RUNNING` explicitly
arms public output and requests a pause. After Transport statistics confirm the
world is paused, the runtime advances to the exact configured native target,
holds there while the companion queues `SET_GUIDED` at public zero and writes
`mission-command-delivered`, then unpauses. It fails closed on an overshot
target or malformed rendezvous fact and never resets Gazebo or ArduPilot.

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
the selected immutable `phase3_foundation.sdf` or `vertical_descent.sdf` under
a new Compose project and Gazebo partition. There is no public in-process reset endpoint. Stale-run or foreign
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

For `vertical_descent` only, Gazebo loads the pinned, downstream-patched
ArduPilotPlugin from `/opt/drone_sim/gazebo/plugins`. The plugin binds UDP 9002,
accepts Copter servo frames, sends JSON sensor state, and holds physics in
lockstep while awaiting the next frame. It emits the initial state at simulation
time zero while the world remains paused; this performs no physics step or
motor-force update. Its private `/model/iris/ardupilot/status` service is
advertised only after UDP bind and reports exchange, motor-update, gap, send
error, last-frame, and last-sim-time counters.

Each lockstep receive consumes exactly one UDP servo datagram. A queued burst
of sequential frames is therefore applied one motor update and one controlled
physics step at a time; the plugin never drains the queue to its newest frame.
The existing receive loop consumes exact duplicates but sends the prior JSON
state at most once for a contiguous duplicate burst, then stops at the first
sequential packet. An empty receive (queue timeout) or an accepted sequential
packet re-arms that recovery, so a lost recovery can be retried without turning
queued duplicates into a feedback burst. A genuinely lone forward jump remains
accepted with its missing-frame count reported.

Flight-local readiness fails closed until the bounded paused bootstrap has
accepted the initial servo frame, sent one simulation-time-zero JSON state, and
accepted the resulting servo frame. The paused exchange then stops consuming
frames. Its stable online counters must show at least two motor-command updates,
one JSON state, no frame gaps, and no send errors; those counters
are preserved in `.status/gazebo-ready.json`. Orchestration alone aggregates this
fact with durable ArduPilot, companion, artifact, and scorekeeper readiness;
the Gazebo fact does not claim those peer processes are lifecycle-ready. Once
the aggregate lifecycle reaches `READY`, the unpaused exchange is private
warmup and remains visible only in native Gazebo state/log time until the
public epoch is activated at `RUNNING`.
Cold-render status-command timeouts or temporary service unavailability remain
not-ready observations and are retried within the original startup wall-time
allowance. Malformed or invalid status replies fail immediately.

On the first valid finalization request the runtime converts the durable intent
and resolved finalization allowance into one absolute monotonic deadline.
Repeated reads reuse that deadline; bridge, server, and native-log stages do
not receive restarted budgets.

## Excluded and reserved interfaces

The passive foundation selection has no ArduPilot actuator/sensor seam.
MAVLink is owned by ArduPilot SITL and companion, not Gazebo. Gazebo owns only
physical detachable-joint mutation and recurrent payload/range truth; request
authorization belongs to electromagnet and point decisions belong to
scorekeeper.
