# Companion

[Project overview](../README.md) · [Architecture](../docs/architecture.md) ·
[Runbook](../docs/runbook.md)

The companion owns mission, vision, and autonomy decisions. It turns public
simulation inputs and ArduPilot telemetry into flight commands, payload
requests, mission events, and durable mission lifecycle evidence.

It does **not** own flight stabilization, actuator control, physics, sensor
truth, direct Gazebo mutation, scoring, or aggregate run finalization.

## Configured diagnostic missions

`mission: configured` runs an ordered list of drone operations and stops on the
first failed step. Mode selection and arming are separate calls. An operator
plan can wait for observed armed GUIDED state. Arming records the mission-start
time without imposing a competition countdown.

Use [configured-descent-run.json](../config/configured-descent-run.json) for
automatic takeoff/hold/land, or
[configured-operator-run.json](../config/configured-operator-run.json) to wait for
external arming. Each `mission_plan` has `schema_version: 1` and nonempty `steps`.
A step has `tool`, `args`, and an optional `timeout_sim_s`, default 60. The
normalized plan is frozen in checksum-bound `configuration/run.json`.

[mission_plan.py](src/drone_sim_companion/mission_plan.py) validates tools and
exports `tool_definitions()` for future callers. Invalid tools, arguments, or
numbers fail before ROS or MAVLink resources open.

| Tool | Arguments and observed completion |
| --- | --- |
| `wait_for_state` | Any of `armed`, `mode`, `landed`; waits for fresh matching telemetry. |
| `set_mode` | `mode: GUIDED`; requires acceptance and observed mode. |
| `arm` | Empty arguments; requires fresh GUIDED/prearm readiness and observed arming. |
| `takeoff` | Positive `altitude_m`, optional `tolerance_m`; requires armed GUIDED and reaching the altitude threshold. |
| `goto_waypoint` | `latitude_deg`, `longitude_deg`, positive `altitude_m`, optional `tolerance_m`; waits for position within tolerance. |
| `hold` | Positive `duration_sim_s` shorter than its timeout; maintains the current GUIDED target. |
| `land` | Empty arguments; waits for touchdown and disarm. Already landed/disarmed succeeds. |
| `precision_land` | Integer `marker_id` and positive absolute `settle_by_sim_s` / `acquire_by_sim_s` deadlines; settles, acquires, tracks, and lands from bounded image, range, and attitude evidence. |

Coordinates are WGS84 degrees; altitudes are metres above ArduPilot home.
[operations.py](src/drone_sim_companion/operations.py) exposes `start()`,
`operation_status()`, and `read_vehicle_state()`. One operation owns flight output
at a time, advancing on telemetry and simulation-clock ticks so observation and
abort remain responsive. Deadlines use simulation time; the run wall deadline
bounds stalled infrastructure. An ACK alone does not establish flight completion.

[configured_runtime.py](src/drone_sim_companion/configured_runtime.py) publishes
execution readiness after passive readiness, matching RUNNING, and public clock.
This releases physics during operator waiting without issuing a flight command.
The whole sequence must finish landed/disarmed to publish mission success.
Every mission host consuming accepted calibration verifies all 15 gains and
the profile's effective baseline before its first flight command. Version-2
imports include scenario precision settings; legacy profiles use their preserved
baseline. MAVLink hosts request one complete list; DroneKit hosts inspect the
current cache with `parameters.get(name, wait_ready=False)`. This avoids a
blocking complete-parameter wait inside the runtime loop. Every value must match
before execution readiness or the first flight command; missing values remain
bounded by the run wall deadline,
and a mismatch fails without arming. Calibration values use the artifact's
float32-aware relative/absolute tolerances; the moving precision profile keeps
its stricter absolute tolerance. A single `calibration_parameters_verified`
event records the accepted pre-arm values before mission execution starts.
Roll diagnostics freeze the verified snapshot before their deliberate seed
writes and record those writes as `calibration_parameters_overridden`; their
run-local saved gains never replace the shared calibration.
The configured host publishes a typed `operator-wait-started` status once the
first wait operation is actually running. The production status writer admits
this companion-owned status; the status itself issues no command.
The shared adapter validates the required names, requests one complete MAVLink
parameter list, and lets each host retain only its required replies. This avoids
overflowing ArduPilot's bounded queue for individual parameter-read requests.
Failure or interruption may attempt one local LAND with fresh armed GUIDED/LAND
state; recovery is bounded by the finalization/overall wall deadlines and retains
the failed result. The auxiliary `drone-sim-operator-wait` entry point connects on native SERIAL1
with system ID 253. It waits for same-run RUNNING, execution readiness, an
actually started first wait and fresh heartbeat; GUIDED ACK plus observed mode
precedes ARM ACK plus observed arming. It then stays passive until finalization,
closing `logs/docker/operator.jsonl`. Startup connection retries are bounded by
the frozen startup wall deadline.
The actor continuously drains private SERIAL1 traffic during native warmup so
requested telemetry cannot back up the command-response path. Warmup packets do
not enter the actor state machine and receive no fabricated public timestamp.
Another observed mode prevents that recovery command. Global
finalization cancels a pending recovery and never starts a new one, allowing
teardown to finish when simulation time and the vehicle transport have stopped.

These templates exercise the parent operation runner, not Comp2026's mission
classes. Existing competition missions retain their own execution path.
Payload tools, agent transport, general sensor-read tools, branching, and retries
are deferred. For local commands, see the
[runbook](../docs/runbook.md#local-developer-workflow).

For `moving_pad_v1`, a bounded latest-frame worker starts before the first
flight operation and detects DICT_4X4_250 marker 7 throughout transit. The
flight-owner thread alone forwards fresh BODY_FRD targets and applies LAND or
GUIDED effects. Settlement requires fresh horizontal speed at or below 0.2 m/s
for 0.5 simulated seconds. LAND requires two continuous seconds of unique,
coherent observations no older than 0.25 simulated seconds. Above 0.75 m target
clearance, 0.5 seconds of tracking loss fails and requests GUIDED. Below that
clearance, touchdown has three simulated seconds while valid observations keep
flowing. Each forwarded target retains its accepted camera exposure time;
`LANDING_TARGET.time_usec` uses public simulation microseconds rather than the
later owner-loop send time or host wall time.

The host reads back the full effective precision profile before any flight
command. Marker detection is not an execution-readiness condition, which avoids
a paused-physics startup deadlock. The moving plan holds north yaw during the
eastbound waypoint so the route follows the camera image's long axis. Observed
arm and disarm transitions publish mission events in phase `MOVING_PAD`.
The companion's expected profile is a live command gate, while the two SITL
parameter files remain the launch inputs. A focused test merges those inputs and
requires the gated values to match, preventing silent drift between them.
The current moving-only experiment requires `AHRS_EKF_TYPE=3`, restoring
native precision `PLND_EST_TYPE=1` and the original `PSC_NE_POS_P=1` while
retaining `PLND_OPTIONS=5` and `PLND_LAG=0.04`. Missing or mismatched values
block readiness and flight commands. EKF3 avoids the diagnosed SIM attitude
delta-velocity frame error; native LAND still owns tracking and descent. This
profile completed a 0.5 m/s moving landing with 100/100 and independent artifact
acceptance. The stationary control also landed, but a later contact-stream fault
invalidated that bundle. See the [flight evidence and limits](../docs/handoff.md#moving-pad-verification).

## Entry points and implementation seams

- [pyproject.toml](pyproject.toml) exposes `drone-sim-companion-runtime`.
- [runtime_node.py](src/drone_sim_companion/runtime_node.py) is the live ROS 2,
  MAVLink, mission-selection, lifecycle, and teardown composition root.
- [mission.py](src/drone_sim_companion/mission.py) and
  [controller.py](src/drone_sim_companion/controller.py) contain the pure
  controlled-descent policy and its command side-effect boundary.
- [mavlink_adapter.py](src/drone_sim_companion/mavlink_adapter.py) is the only
  PyMAVLink translation boundary.
- [moving_vision.py](src/drone_sim_companion/moving_vision.py) owns bounded image
  processing; [moving_precision.py](src/drone_sim_companion/moving_precision.py)
  owns the moving-target policy and returns effects to the flight owner.
- [comp2026_host.py](src/drone_sim_companion/comp2026_host.py) adapts simulation
  clock, images, range, payload calls, waypoints, events, and failure recovery
  for the bundled [Comp2026 mission](comp2026/README.md).
- [lifecycle.py](src/drone_sim_companion/lifecycle.py) owns companion readiness,
  terminal mission evidence, and quiescence publication.
- The shared [runtime status contract](../artifacts/src/artifacts/runtime_status.py)
  defines the companion's durable status values. The runtime publishes them
  through the container-side
  [protocol adapter](../artifacts/src/artifacts/runtime_protocol.py).

## Interfaces

### All-axis flight-controller tuning

The `autotune` mission is the calibration path for roll, pitch, and yaw. It
requires normal ArduPilot prearm checks, reads back `AUTOTUNE_AXES=7` and
`ATC_RATE_FF_ENAB=1`, takes off in GUIDED, settles in LOITER, and enters
AUTOTUNE with neutral sticks. After ArduPilot reports success, the companion
returns to LOITER and sends `MAV_CMD_DO_AUX_FUNCTION` function 180 at HIGH.
The command ACK, the complete pilot-testing status, and matching live gain
readback are all required before a two-second stable settle and native LAND.
The runtime refreshes neutral RC overrides twice per wall second until LAND
owns descent; an earlier disarm fails immediately.

Completion requires the all-axis saved-gains status, observed disarm, and a
post-disarm readback matching the tested values. Flight decisions use only the
public clock and MAVLink telemetry. The historical `autotune_roll` and
`hover_roll` diagnostic missions keep their existing behavior. Start the new
mission with [autotune-run.json](../config/autotune-run.json); its 900-second
public window reserves the final 60 seconds for landing or failure recovery.
The reserve begins at public 840 seconds, allowing more time for native tuning.
An airborne failure clears overrides and requests native LAND for at most 45
simulated seconds; landing recovery does not change the failed mission result.
Native rate-D, rate-P, and angle-P gain-determination failures trigger this
recovery immediately, even while the vehicle mode still reports AUTOTUNE.

The runtime consumes the public clock and run state, onboard images, competition
downward range, and ArduPilot MAVLink telemetry. Production MAVLink is fixed to
`tcp:ardupilot-sitl:5760`; flight commands never bypass ArduPilot.

For the exact ROS payloads, read
[RunState.msg](../ros_ws/src/simulation_interfaces/msg/RunState.msg),
[PayloadCommand.srv](../ros_ws/src/simulation_interfaces/srv/PayloadCommand.srv),
and [MissionEvent.msg](../ros_ws/src/simulation_interfaces/msg/MissionEvent.msg).
Camera identity is recorded through
[FrameMetadata.msg](../ros_ws/src/simulation_interfaces/msg/FrameMetadata.msg),
although autonomy consumes the image itself rather than subscribing to the
redundant metadata stream. The ownership, ordering, timing, and durable-status
contracts are centralized in the [architecture guide](../docs/architecture.md).

The moving camera contract is pinned in
[moving_pad_camera_calibration.json](src/drone_sim_companion/moving_pad_camera_calibration.json)
and [moving_pad_camera_mounting.json](src/drone_sim_companion/moving_pad_camera_mounting.json).
It is pinned to the 640x480, 0.6-radian Gazebo camera at body-FLU pose
`0 0 -0.1 0 1.570796327 0`. OpenCV vectors map to body FRD as
`forward=-camera_y`, `right=camera_x`; target down uses the fresh downward range
plus the 0.1 m body offset. The metadata remains fail-closed until a rendered
marker verifies the complete camera path. The moving-pad render fixtures provide
the current verification evidence.

The companion provides MAVLink commands, correlated payload service requests,
ordered mission events, structured diagnostics, and its own readiness,
completion, failure, and quiescence facts. It never publishes physical truth.

## Constraints worth knowing

- Mission decisions use accepted simulation time. Loss of `/clock` prevents new
  simulated decisions; wall time only bounds infrastructure and computation.
- The controlled-descent path requires positive command acknowledgements and
  observed vehicle state. Heartbeat and healthy prearm observations are separate
  passive readiness facts, and commands wait for `RUNNING` plus public clock.
- `companion/comp2026` is tracked in this monorepo and supplies the
  `comp2026_auto` mission. Keep changes there focused and preserve its imported
  history and provenance. The Docker context admits only its explicit import
  closure.
- The current `comp2026_auto` host is incompatible with the imported guarded
  flight API. Both competition suite cases fail before flight. The adapter needs
  explicit identities, supervisor/home setup, and validated release/precision
  policies; those policies are absent from the competition configuration.
  See the dated evidence in [handoff](../docs/handoff.md). Import smoke alone
  does not establish flight compatibility.
- Building the Phase 3 companion image requires
  `SIM_COMP2026_REVISION=$(git rev-parse HEAD)`. Compose
  leaves the build argument empty when it is not supplied so inactive profiles
  and noncompanion configuration still resolve; the companion Dockerfile rejects
  an empty value before package installation or source copies.
- The hosted `drone.auto_attempt` currently imports FM1 and FM2 from `missions/`
  but imports FM3 from `drone/mock_mission.py`; do not assume
  `missions/fm3.py` is the deployed implementation.
- After changing the import closure, run `python3 -m
  drone_sim_companion.comp2026_smoke` in the built companion image. It imports
  the deployed control, camera, LiDAR and original mission functions without
  opening devices or executing a mission.
- That deployed `mock_mission.py` owns payload-marker acquisition and precision
  landing recovery. It establishes a five-frame earth-fixed target anchor,
  rejects stale or inconsistent camera/range observations before MAVLink,
  holds the current position in GUIDED while reacquiring, and permits one
  return to the 4.572 m search hover before failing closed. It validates the
  live flight-controller profile before the first LAND and never disarms or
  requests attachment without confirmed touchdown. Below the configured
  `PLND_ALT_MIN` of 0.75 m, LAND continues without requiring marker visibility;
  this avoids treating the marker's normal near-ground exit from the camera
  view as a recovery event. The exact flight settings are owned by
  [descent.parm](../ardupilot_sitl/params/descent.parm).
- The nested camera API returns the marker vector and source frame timestamp as
  one observation. `comp2026_host.py` supplies bounded, strictly newer frames;
  camera silence therefore becomes an unhealthy observation instead of
  blocking the mission thread.
- Comp2026 startup separates process readiness from permission to enter the
  original mission. Sensor, service, heartbeat, and armability predicates are
  refreshed atomically and fail closed; downward range expires after 0.5
  simulated seconds. The runtime must assign the initial GUIDED mode and write
  its durable delivery fact no later than the inclusive 50 ms public-time
  deadline before the original worker can enter. The executable owners are the
  [delivery window and lifecycle writer](src/drone_sim_companion/lifecycle.py),
  [start gate](src/drone_sim_companion/comp2026_host.py), and
  [runtime composition](src/drone_sim_companion/runtime_node.py); the shared
  status schema owns the exact
  [`MissionCommandDeliveredStatus`](../artifacts/src/artifacts/runtime_status.py)
  fields.
- Before mission code reads competition inputs, startup verifies the resolved
  `course.yaml` and `scenario.yaml` copies against their SHA-256 digests in
  `run.json`.
- Terminal success and the first fatal callback/mission failure are serialized.
  Quiescence follows worker termination, executor shutdown, and closure of all
  output producers; a teardown timeout records failure instead.
- The first runtime failure wins across all modules. A later companion failure
  cannot replace the durable first cause.

## Focused tests

Run from the project root:

```bash
uv run --locked pytest companion/tests -q
```

These host tests cover the pure policies, adapters, mission host, and runtime
composition. They do not prove a live flight; use
the [runbook](../docs/runbook.md) for image and end-to-end procedures.

Run the bundled mission tests separately:

```bash
PYTHONPATH=companion/comp2026/src:companion/comp2026/tests \
  uv run --locked pytest companion/comp2026/tests -q
```
