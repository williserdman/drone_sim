# Companion External Interface

## Runtime selection

The current run's immutable resolved `mission` selects one of two adapters.
`controlled_descent` retains the existing PyMAVLink controller path.
`comp2026_auto` hosts the original mission source copied from the exact nested
repository commit through a thin ROS/DroneKit adapter; the nested
`run_auto_attempt` remains the phase and flight-decision authority.

## ROS 2 inputs

- Gazebo camera frames on `/camera/onboard/image_raw` through ROS 2 image
  transport at 20 frames per simulated second. Competition uses reliable,
  volatile QoS depth 100 and exact resolved 640x480 `rgb8` bytes.
- Authoritative frame-aligned `/clock` using reliable QoS depth 1000

Each frame correlates with `simulation_interfaces/msg/FrameMetadata`, which
carries `run_id`, `frame_id`, stream identity, and a simulation capture
timestamp.

For `comp2026_auto`, the companion also consumes reliable, volatile
`/competition/range/downward` `LaserScan` samples and the existing
`/simulation/payload_command` `PayloadCommand` service. It publishes ordered
reliable, transient-local `MissionEvent` rows on
`/simulation/mission_events`. Payload command IDs are
`run:<aruco_id>:<attach|release>:<per-action-sequence>`; a call returns to the
original mission only after an accepted response with the identical command ID
and a positive response sequence. Rejections, missing responses, and
correlation mismatches raise into the mission failure path. Attachment returns
literal `True` on success, as required by the original FM3 branch.

## MAVLink interface

- Output: flight and mission commands to ArduPilot SITL
- Input: vehicle telemetry, modes, command acknowledgements, and `STATUSTEXT`
- Production endpoint: `tcp:ardupilot-sitl:5760` on the Compose network

The `descent_v1` command sequence is `GUIDED`, arm, take off to 1.5 m,
and `LAND`. Every transition requires the ordered positive command ACK and
observed vehicle state; a command send is logged only after PyMAVLink accepts
it. During private warmup, the companion passively and independently latches
the first heartbeat and the first `SYS_STATUS` in which
`MAV_SYS_STATUS_PREARM_CHECK` is both enabled and healthy. Neither fact alone
establishes mission readiness. After GUIDED is observed, ARM still waits for a
healthy prearm observation in the ordered mission telemetry. Negative ACKs,
unexpected ACKs, mode inconsistency, timestamp regression, and contact before
descent fail the mission without repair.
MAVLink `STATUSTEXT` is retained as structured `mavlink_status_text` evidence
with its severity and the latest authoritative simulation timestamp, including
the exact reason for a normal pre-arm rejection.

Commands derived from imagery retain `source_frame_id` where the adapter permits.

The `comp2026_auto` branch uses DroneKit 2.9.2 at the same fixed
`tcp:ardupilot-sitl:5760` endpoint. At public zero it queues the established
GUIDED transport command and records `mission-command-delivered`, allowing the
existing Gazebo epoch rendezvous to release. The original worker does not emit
`FM1/STARTED` or arm until the current run is `RUNNING` and one atomic refresh
finds a public clock, an undelivered exact image/metadata pair, a downward range
that passes `get_distance()`, a currently available payload service, current
DroneKit heartbeat health, and `is_armable is True`. These dynamic predicates
do not latch: loss or reversion invalidates readiness before worker release. The
frame/service/vehicle facts are sampled first; range freshness is evaluated
last against the then-current simulation clock while holding the start-gate
lock, which is the worker-release linearization point. Thus simulation time
cannot advance during an earlier predicate read and leave a stale cached range
eligible for release. The heartbeat check reads DroneKit's current
`last_heartbeat` against
`DroneControl`'s existing 60-second `heartbeat_timeout`; it does not retain a
wall observation or add a new timer. No QGC or outer retry/state machine is
introduced.

## Timing and ordering

Mission logic is frame- or event-triggered in simulation time. Duplicate frames are idempotently ignored, missing frames are diagnosed, stale `run_id` data is ignored, and loss of `/clock` prevents new simulated decisions. Wall time measures computation and infrastructure health only.

The original mission's clock seam reads the accepted public `/clock` value.
Each sleep captures its own start and waits until
`current_timestamp - saved_start >= duration`; shutdown stops the wait. Frames
are joined only when current-run metadata and the image header have the exact
same simulation timestamp. Capture blocks for a strictly newer pair, exposes
that genuine timestamp as `last_timestamp_ns`, converts RGB to BGR without
`cv_bridge`, and never duplicates or rescales a delivered frame. A downward
range older than 0.5 elapsed simulated seconds fails closed.

The MAVLink TCP connection deadline uses the current run's resolved
`startup_wall_seconds` from `SIM_CONFIG_PATH`. After that transport connects,
the first heartbeat is not subject to the startup wall deadline: ArduPilot boot
continues in lockstep simulation time, so a slow host cannot reduce its
simulated boot allowance. The configured simulation duration and mission
evidence provide the deterministic bound; `max_wall_seconds` remains the
run-level infrastructure failsafe. The snapshot path must be the absolute
`SIM_RUN_DIRECTORY/configuration/run.json`, its `run_id` must match
`SIM_RUN_ID`, and both wall bounds must be positive integers. Tests may override
the connection bound explicitly with
`SIM_COMPANION_STARTUP_TIMEOUT_SECONDS`; the override does not alter any
simulation-time mission deadline.

MAVLink messages are correlated with the latest authoritative `/clock` value.
Ground truth retains its native message timestamp. Equal timestamps are ordered
by receipt; a lower timestamp than the preceding mission input is rejected.
Before the first public `/clock`, passive MAVLink observations use timestamp zero
and cannot advance mission policy. The companion sends no flight command or
telemetry-stream request until both the current run is `RUNNING` and at least one
public clock sample has been received.
At public zero it queues `SET_GUIDED` to the connected MAVLink transport before
polling later telemetry, then durably records that delivery. This transport
delivery fact is distinct from, and does not replace, the later positive
vehicle command acknowledgement required by mission policy.

## Durable lifecycle

- `.status/companion-ready.json` records a successful connection to the fixed
  MAVLink TCP endpoint. This transport fact can be published while simulation
  is paused; it does not claim that a heartbeat has already been emitted.
- `.status/mission-ready.json` is written exactly once after both passive facts
  have been latched and is exactly
  `{run_id,ready:true,heartbeat_observed:true,prearm_checks_healthy:true}`.
  Heartbeat-only or healthy-prearm-only telemetry cannot create it, and creating
  it sends no command to the vehicle.
- Mission policy cannot issue `GUIDED` or any other flight command until both
  `RUNNING` and the first public `/clock` sample have been observed.
- `.status/mission-command-delivered.json` is written exactly once after the
  initial `SET_GUIDED` send returns and is exactly
  `{run_id,command:"SET_GUIDED",sim_timestamp_ns:0,delivered:true}`.
- `.status/mission-finished.json` is written only after actual landed/disarmed
  success and is exactly `{run_id,finished:true,sim_timestamp_ns,outcome:"LANDED"}`.
  Failures remain structured failure evidence and can never create or repair
  this completion fact.
- The final structured event precedes
  `.status/quiescence/companion.json`; no output follows that marker.

## Prohibited path

The companion must never directly command or mutate Gazebo.

For `comp2026_auto`, `mission-ready` preserves the established durable schema
and may precede public sensor samples: it means the ROS graph, responsive
executor, DroneKit connection, and blocked original-mission worker gate are
initialized. This is intentionally distinct from permission to start the
mission; the RUNNING-era gate above remains mandatory. Success additionally
requires the original callback's ordered `HOME/DISARMED` then
`HOME/COMPLETE`. The first fatal sensor callback or mission exception stops all
attempt waits and phase emission, prevents terminal success, records the active
phase, and writes current-run runtime failure. Short callback input acceptance
and the first failure claim are serialized with terminal-success publication;
cancellation/status/recovery side effects execute outside that lock. Recovery
is owned once and waits
for the attempt worker to stop before best-effort RTL, land, and disarm.
Callbacks remain responsive until finalization. Quiescence is written only
after the worker has terminated, the ROS executor has stopped and joined, and
the node/DroneKit output producers have closed; a timeout writes failure and no
quiescence marker.

For Phase 2 synthetic finalization, the stub stops publisher/log output before
writing `.status/quiescence/companion.json` with exact current-run quiescence
schema, then remains silent while orchestration aggregates the freeze.

## Deferred decisions

- Live end-to-end competition-attempt tuning remains outside this adapter
  integration; the nested mission remains authoritative for those decisions.
- QGroundControl support and automatic mission retries are deferred. A current
  attempt fails once through the existing recovery and durable failure path.
- The exact nested revision is a local checkout/image-label handoff; no remote
  Git operation is part of companion startup or acceptance.
