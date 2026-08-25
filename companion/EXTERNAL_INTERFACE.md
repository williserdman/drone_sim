# Companion External Interface

## ROS 2 inputs

- Gazebo camera frames on `/camera/onboard/image_raw` through ROS 2 image
  transport at 20 frames per simulated second, using best-effort QoS depth 5
- Authoritative frame-aligned `/clock` using reliable QoS depth 1000

Each frame correlates with `simulation_interfaces/msg/FrameMetadata`, which
carries `run_id`, `frame_id`, stream identity, and a simulation capture
timestamp.

## MAVLink interface

- Output: flight and mission commands to ArduPilot SITL
- Input: vehicle telemetry, modes, command acknowledgements, and `STATUSTEXT`
- Production endpoint: `tcp:ardupilot-sitl:5760` on the Compose network

The `descent_v1` command sequence is `GUIDED`, arm, take off to 1.5 m,
and `LAND`. Every transition requires the ordered positive command ACK and
observed vehicle state; a command send is logged only after PyMAVLink accepts
it. Negative ACKs, unexpected ACKs, mode inconsistency, timestamp regression,
and contact before descent fail the mission without repair.
MAVLink `STATUSTEXT` is retained as structured `mavlink_status_text` evidence
with its severity and the latest authoritative simulation timestamp, including
the exact reason for a normal pre-arm rejection.

Commands derived from imagery retain `source_frame_id` where the adapter permits.

## Timing and ordering

Mission logic is frame- or event-triggered in simulation time. Duplicate frames are idempotently ignored, missing frames are diagnosed, stale `run_id` data is ignored, and loss of `/clock` prevents new simulated decisions. Wall time measures computation and infrastructure health only.

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

## Durable lifecycle

- `.status/companion-ready.json` records a successful connection to the fixed
  MAVLink TCP endpoint. This transport fact can be published while simulation
  is paused; it does not claim that a heartbeat has already been emitted.
- Mission policy still waits for the first ArduPilot heartbeat after `RUNNING`
  before issuing `GUIDED` or any other flight command.
- `.status/mission-finished.json` is written only after actual landed/disarmed
  success and is exactly `{run_id,finished:true,sim_timestamp_ns,outcome:"LANDED"}`.
  Failures remain structured failure evidence and can never create or repair
  this completion fact.
- The final structured event precedes
  `.status/quiescence/companion.json`; no output follows that marker.

## Prohibited path

The companion must never directly command or mutate Gazebo.

For Phase 2 synthetic finalization, the stub stops publisher/log output before
writing `.status/quiescence/companion.json` with exact current-run quiescence
schema, then remains silent while orchestration aggregates the freeze.

## Deferred decisions

- Command-to-frame correlation encoding for future vision-driven missions
