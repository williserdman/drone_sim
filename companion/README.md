# Companion

[Project overview](../README.md) · [Architecture](../docs/architecture.md) ·
[Runbook](../docs/runbook.md)

The companion owns mission, vision, and autonomy decisions. It turns public
simulation inputs and ArduPilot telemetry into flight commands, payload
requests, mission events, and durable mission lifecycle evidence.

It does **not** own flight stabilization, actuator control, physics, sensor
truth, direct Gazebo mutation, scoring, or aggregate run finalization.

## Where to change Comp 2026 code

`comp2026/` contains the upstream source pinned at
`95dec5105573d1a18b2ad824975d5edf99893006`. The pristine import is on
`integration/comp2026-upstream-95dec510`; `integration/comp2026-testbench-hooks`
adds the optional integration hooks. Both branches retain the testbench base
`0f7d525`. [comp2026-source.json](comp2026-source.json) records the origin,
commits, upstream tree, file hashes, and allowed hook locations.

| Change | Source |
| --- | --- |
| Upstream FM1 and FM2 behavior | [`fm1.py`](comp2026/src/drone/missions/fm1.py), [`fm2.py`](comp2026/src/drone/missions/fm2.py) |
| Upstream FM3 used by the simulator | [`mock_mission.py`](comp2026/src/drone/mock_mission.py) |
| Testbench attempt sequence | [`comp2026_runner.py`](src/drone_sim_companion/comp2026_runner.py) |
| Upstream aircraft commands | [`drone_control.py`](comp2026/src/drone/control/drone_control.py) |
| Testbench admission, output authority, and recovery | [`comp2026_control.py`](src/drone_sim_companion/comp2026_control.py), [`runtime_node.py`](src/drone_sim_companion/runtime_node.py) |
| Frozen simulator inputs and firmware checks | [`comp2026_policy.py`](src/drone_sim_companion/comp2026_policy.py) |
| ROS image, range, payload, and clock adapters | [`comp2026_host.py`](src/drone_sim_companion/comp2026_host.py) |
| Configured diagnostic camera | [`precision_camera/`](src/drone_sim_companion/precision_camera/) |
| Hook tests and simulator regressions | [`test_testbench_hooks.py`](comp2026/tests/test_testbench_hooks.py), [`tests/`](tests/) |
| Course geometry and scenario inputs | [`course.yaml`](../config/course.yaml), [`scenario.yaml`](../config/scenario.yaml) |

`comp2026_auto` calls the actual `drone.missions.fm1.fm1`,
`drone.missions.fm2.fm2`, and `drone.mock_mission.fm3` functions. The testbench
runner sequences FM1, FM2, FM3 for marker IDs 3 and 4, then return Home.
FM2 requests landing and disarm. The runner uses the upstream executable's takeoff setup
before the first FM3 call. A phase `COMPLETE` event means its function returned;
the scorer independently determines physical success. Home completion also
requires fresh landed and disarmed telemetry after the last flight output.

`DroneControl` accepts `vehicle` and `command_listener` keyword arguments.
The testbench supplies its connected `SimulationVehicle` and sets
`command_listener=False` to keep QGC admission with one owner.
`CameraManager` accepts `frame_source` and `calibration_path`; an injected source
avoids opening a physical camera. `Camera` accepts the injected `manager`.
LiDAR and GPIO imports occur only when constructing the corresponding device.
Default hardware constructors remain available. `drone.timebase` uses standard
time by default and public simulation time inside `timebase.configured(clock)`.
Persisted waypoint load times and
expiry retain wall time. The source audit checks unchanged file hashes and the
protected syntax outside the declared hook functions; hook tests cover the
injections themselves.

Upstream FM3 prints `simulated drop!!` and has no payload attachment or release
call. The testbench preserves that behavior and does not infer delivery from a
return value. Upstream ordinary LAND also retains the existing controller logic.
The pinned `PLND_STRICT=2` profile may prevent a landing without a precision
target; the integration does not toggle precision landing around ordinary LAND.
These are flight-acceptance limitations, not completed mission outcomes.

Configured diagnostics use a separate operation runner and the testbench-owned
precision camera. A configured descent does not exercise upstream FM1, FM2, or
FM3. Follow the [QGC development loop](../docs/runbook.md#develop-comp-2026-with-qgc)
for audit, test, build, and run commands. Full-flight acceptance of the upstream
integration remains pending; simulator checks do not approve aircraft deployment.

## Local operator supervision

Resolved runs select `launch_mode: unattended` (the existing default) or
`launch_mode: operator`. In operator mode every companion host connects and
observes the live vehicle during preparation, but its initial mission command
remains blocked until the runtime is RUNNING and the immutable start admission
matches the current run, configuration digest, and start request. Preparation
publishes current heartbeat, prearm, and armed observations through the shared
operator protocol. A start request receives request-bound readiness only from a
fresh (at most two wall seconds old), healthy, disarmed vehicle snapshot.

The admission resets the companion's overall mission wall deadline; time spent
inspecting the prepared simulation does not consume that budget. Public
simulation-time mission deadlines continue to start from the admitted public
epoch. Automatic command ownership stays with the configured, Comp2026,
AutoTune, hover, or controlled-descent host selected by the frozen run.

`Abort Flight` is separate from runtime finalization. A matching durable abort
request cancels further mission intent once, then invokes the selected host's
recovery path. Configured missions use `recover_land`; tuning and hover clear
overrides and select LAND; controlled descent requires fresh armed GUIDED/LAND
state before sending LAND.

For Comp2026, the testbench closes upstream worker output and stops its mission
clock. Upstream exception cleanup cannot regain output authority. After the
worker exits, the testbench can request local LAND using a separate view of the
accepted public clock. Recovery is bounded by 60 simulated seconds and the
finalization wall deadline. External takeover, invalid time, or finalization
interrupts recovery. Recovery success requires fresh landed and disarmed
telemetry newer than its last flight output. It does not change a failed mission
result or establish payload delivery.

The companion keeps observation available and records append-only `accepted`,
`recovering`, `landed_disarmed`, `rejected`, `failed`, or `interrupted` progress.
A terminal recovery result does not finalize the runtime. Stop Simulation remains
the distinct finalization action. A rejected command, missing telemetry, timeout,
QGC authority change, or Stop remains a visible non-success result.

Descriptor-backed runs route project Start 31020 and Abort 31021 through
[`QgcMissionControl`](src/drone_sim_companion/qgc_control.py). The transport
callback only enters a bounded queue. The existing companion owner loop validates
and persists commands, reserves the prepared Start UUID once, and invokes the
existing admission or recovery controller. Start requires MAVLink 2, the frozen
QGC and companion identities, the current integer attempt token, zero parameters
2 through 7, fresh healthy landed/disarmed vehicle facts, and the host's
calibration, native-time, and producer checks. The owner repeats those readiness
checks before it releases a pending admission. A restart after reservation fails
closed and requires a new Prepare. Request-only, admission-only, foreign, and
mismatched durable Start state all fail before another reservation or mission
release.

COMMAND_ACK `ACCEPTED` for Start means that the immutable admission and matching
RUNNING state authorized mission execution. It does not mean the aircraft flew,
landed, scored, or produced an acceptable bundle. Pending Start and recovery use
bounded `IN_PROGRESS` updates. Rejections and terminal recovery results carry a
50-byte readable STATUSTEXT and append command evidence before the ACK is sent.
Abort during pending Start records cancellation without inventing an admission;
Abort after admission delegates to the host's guarded recovery and requires fresh
landed/disarmed observations for success. Finalization blocks new reservations
and recovery output.

Both live MAVLink adapters keep their existing single reader and encoder. Raw
pymavlink dispatches `COMMAND_LONG` before continuing to exact flight-controller
1/1 telemetry; DroneKit dispatches through `SimulationVehicle` before its normal
non-FC filter and queues responses through the existing `MAVWriter`. Each adapter
advertises companion 1/191, accepts prepared QGC 255/190 separately, and
preserves the QGC target on MAVLink 2 ACKs despite DroneKit's ordinary
flight-controller target rewrite. Commands addressed to flight controller 1/1
remain FC-owned and receive no companion ACK.

Descriptor-backed readiness consumes only newly received exact-FC HEARTBEAT,
SYS_STATUS, and EXTENDED_SYS_STATE facts with their wall receipt times. The owner
loop applies those facts, so receive callbacks perform no persistence. Cached
DroneKit attributes and later position packets cannot refresh disarm or ground
evidence. QGC Start also reads fresh Gazebo private physics/camera preflight
before public mission time exists, preserving Gazebo's request-scoped 50 ms
epoch. Unattended and descriptor-absent runs retain their historical behavior.

After Start admission, raw and diagnostic hosts hold an irreversible flight
authority generation. An exact-FC mode or arm change is accepted only when it
matches a transition registered by the operation at the final MAVLink enqueue.
The permission remains with that operation across unrelated waypoint,
landing-target, RC, payload, and parameter writes, then is cleared when the
operation succeeds, fails, or aborts. Consecutive mode and arm commands may
therefore await their separate fresh observations without authorizing a later
transition after the operation ends.
The leading operator wait may instead accept native GUIDED and arm transitions
while that wait is active. Any other fresh mode or arm change cancels onboard
mission intent; a later return to GUIDED cannot restore it. A valid QGC Abort
may transfer authority that the mission still owns to its existing LAND recovery,
but cannot clear an external revocation.

The raw adapter guards each aircraft write, and the diagnostic hosts guard their
shared DroneKit encoder queue. This covers mode, arm, waypoint, landing-target,
RC override, auxiliary command, and parameter writes, including commands
from dynamically constructed adapters. Heartbeats, command replies, and passive
telemetry/parameter requests remain available after revocation. Detection occurs
when the single existing reader processes a new exact-FC heartbeat; the software
contract does not claim instantaneous takeover between samples. Comp2026 uses
the same flight-authority gate at the final DroneKit writer and restricts
mutating output to its mission or recovery owner thread. Its exact-FC authority
observer runs before DroneKit updates cached vehicle attributes. Every flight
output, including recovery, requires a heartbeat received at most two wall
seconds ago. Missing or stale heartbeat evidence revokes authority.
Finalization closes the writer before teardown, including upstream exception
cleanup. Passive telemetry
requests, heartbeats, and command replies remain available. Payload service
discovery and response waiting occur outside the guarded enqueue transaction.

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
| `wait_for_state` | Any of `armed`, `mode`, `landed`; waits for fresh matching telemetry. The leading operator wait grants native authority only for `mode: GUIDED` and `armed: true`. |
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
When `precision_land` successfully enqueues a LAND or GUIDED mode command, the
operation lifecycle emits `precision_mode_requested` with the operation ID and
requested mode. This event proves which active mission requested the command; it
does not claim flight-controller acknowledgement or an observed mode change.

[configured_runtime.py](src/drone_sim_companion/configured_runtime.py) publishes
execution readiness after passive readiness, matching RUNNING, and public clock.
This releases physics during operator waiting without issuing a flight command.
The whole sequence must finish landed/disarmed to publish mission success.
The current simulation contract pins Copter 4.6.3, packed version `0x040603FF`,
and custom revision bytes `92b0cd78`. Parameter readback uses that release's
names and units, including `LAND_SPEED` in cm/s and `ATC_ACCEL_*_MAX` in cdeg/s².
Every mission host consuming accepted calibration verifies all 15 gains and
the profile's effective baseline before its first flight command. Version-2
imports include scenario precision settings; legacy profiles use their preserved
baseline. MAVLink hosts request one complete list; DroneKit hosts inspect the
current cache with `parameters.get(name, wait_ready=False)`. This avoids a
blocking complete-parameter wait inside the runtime loop. The roll host requests
one complete list on its first healthy heartbeat during private warmup, before
the public clock or RUNNING gate. Every value must match
before execution readiness or the first flight command; missing values remain
bounded by the run wall deadline,
and a mismatch fails without arming. Calibration values use the artifact's
float32-aware relative/absolute tolerances; the moving precision profile keeps
its stricter absolute tolerance. A single `calibration_parameters_verified`
event records the accepted pre-arm values before mission execution starts.
Simulation DroneKit connections use `SimulationVehicle` to disable DroneKit's
automatic indexed parameter retries while retaining the real parameter cache,
count handling, loaded transition, explicit full-list requests, and manual reads.
Raw pymavlink and simulation DroneKit receive paths discard every telemetry and
acknowledgement message outside the pinned ArduPilot system ID before it can
change cached vehicle state. Outbound targets remain pinned to that vehicle even
if another peer publishes a heartbeat. Each control channel explicitly requests
10 Hz simulation-time vehicle heartbeats so a slowed SITL run still supplies the
two-wall-second preflight freshness contract without accepting QGC traffic as
vehicle evidence.
This narrow adapter depends on DroneKit 2.9.2 registering its base `PARAM_VALUE`
cache listener before the adapter's callback; the later callback restores the
private `_params_duration` field to infinity after every base update. Hardware
connections omit the optional vehicle class and keep DroneKit's default behavior.
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
native precision `PLND_EST_TYPE=1` and the original `PSC_POSXY_P=1` while
retaining `PLND_OPTIONS=5` and `PLND_LAG=0.04`. Missing or mismatched values
block readiness and flight commands. EKF3 avoids the diagnosed SIM attitude
delta-velocity frame error; native LAND still owns tracking and descent. The
historical Copter 4.7 profile completed a 0.5 m/s moving landing with 100/100
and independent artifact acceptance. Copter 4.6.3 flight validation is pending. The stationary control also landed, but a later contact-stream fault
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
- [precision_camera/](src/drone_sim_companion/precision_camera/) owns configured
  camera observations, calibration, mounting, and recording. Its algorithms and
  regression suites were moved from testbench revision `0f7d525`.
- [moving_vision.py](src/drone_sim_companion/moving_vision.py) owns bounded image
  processing; [moving_precision.py](src/drone_sim_companion/moving_precision.py)
  owns the moving-target policy and returns effects to the flight owner.
- [comp2026_host.py](src/drone_sim_companion/comp2026_host.py) adapts simulation
  clock, images, range, payload calls, waypoints, events, and failure recovery
  for the [upstream mission runner](src/drone_sim_companion/comp2026_runner.py).
- [lifecycle.py](src/drone_sim_companion/lifecycle.py) owns companion readiness,
  terminal mission evidence, and quiescence publication.
- The shared [runtime status contract](../artifacts/src/artifacts/runtime_status.py)
  defines the companion's durable status values. The runtime publishes them
  through the container-side
  [protocol adapter](../artifacts/src/artifacts/runtime_protocol.py).

## Interfaces

### All-axis flight-controller tuning

The `autotune` mission is the calibration path for roll, pitch, and yaw. It
requires normal ArduPilot prearm checks. After the first healthy heartbeat, it
sets `ATC_RAT_RLL_P=0.0675`, `ATC_RAT_RLL_I=0.0675`, and
`AUTOTUNE_AXES=7`, then selects GUIDED and requests the complete parameter
list. Arming requires a fresh, complete readback of all 15 gain inputs and
preserved base parameters. The roll seeds must match within float32 tolerance,
with `AUTOTUNE_AXES=7` and `ATC_RATE_FF_ENAB=1`. Missing values or rejected seed
writes fail within the existing 10-second entry phase. These run-local seed writes
do not change the global base-gain profile. The companion takes off in GUIDED, settles in LOITER for two
seconds, commands and observes ALT_HOLD, then enters AUTOTUNE with neutral
sticks. Entering from ALT_HOLD disables native position hold and its
position-dependent heading updates. After ArduPilot reports success, the
companion returns to LOITER and sends `MAV_CMD_DO_AUX_FUNCTION` function 180 at HIGH.
The command ACK, the complete pilot-testing status, and matching live gain
readback are all required before a two-second stable settle. The mission captures
fresh launch coordinates while disarmed, then returns to them in GUIDED at 5 m
after tuning. LAND requires two continuous seconds within 0.5 m horizontally and
vertically of that waypoint, speed at most 0.2 m/s, and roll/pitch within 5 degrees.
The return is bounded by 60 simulated seconds and the existing landing reserve;
missing/stale position cannot qualify arrival. Return target and arrival events
are recorded. This is a GPS waypoint return, not a marker-guided precision landing.
The runtime refreshes neutral RC overrides twice per wall second through LOITER,
ALT_HOLD entry, and tuning, then clears them before the GUIDED return. An
unexpected mode, earlier disarm, or expired phase deadline fails immediately;
native LAND and gain saving remain under ArduPilot control.

Completion requires the all-axis saved-gains status, observed disarm, and a
post-disarm readback matching the tested values. Flight decisions use only the
public clock and MAVLink telemetry. Start the all-axis mission with
[autotune-run.json](../config/autotune-run.json); its 900-second
public window reserves the final 60 seconds for landing or failure recovery.
The reserve begins at public 840 seconds, allowing more time for native tuning.
An airborne failure clears overrides and requests native LAND for at most 45
simulated seconds; landing recovery does not change the failed mission result.
Native rate-D, rate-P, and angle-P gain-determination failures trigger this
recovery immediately, even while the vehicle mode still reports AUTOTUNE.

The `autotune_roll` diagnostic retains its roll-only seed, `AUTOTUNE_AXES=1`,
aggression, and 120-second public window. After exact roll success, it uses the
same LOITER, gain activation, stable GUIDED return, and native LAND sequence.
Activation requires the accepted auxiliary-command ACK, the exact roll
pilot-testing status, and a fresh five-gain readback. Completion requires the
roll saved-gains status, disarm, touchdown, and one coherent DataFlash save
epoch matching the activated gains. The 120-second deadline remains the outer
bound; this shorter diagnostic does not use the all-axis 60-second landing
reserve. `hover_roll` keeps its existing behavior.

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
- The upstream source and hook scope are recorded in
  [comp2026-source.json](comp2026-source.json). Run
  `uv run --locked python scripts/audit-comp2026-hooks.py` from the repository
  root after changing the imported source. Mission behavior changes require
  separate review from dependency-injection hooks.
- `SimulationCompetitionControl` owns admission, source-filtered native
  observations, one consumed attempt per run, output authority, and local LAND
  recovery. It constructs the actual upstream controller with `SimulationVehicle`
  and `command_listener=False`, leaving command admission to the existing QGC
  owner. It does not substitute another implementation of flight decisions.
- The testbench policy verifies the frozen course, scenario, pinned Copter 4.6.3
  provenance, model, world, and parameter inputs. The image carries these inputs
  at their repository-relative paths under `/opt/drone_sim`. The runtime verifies
  the effective calibration before flight.
- Building the Phase 3 companion image requires
  `SIM_COMP2026_REVISION=$(git rev-parse HEAD)`. The label records the parent
  revision. Source edits require a matching rebuild. The Docker context admits
  only the explicit runtime import closure; run
  `python3 -m drone_sim_companion.comp2026_smoke` in the built image after
  changing that closure.
- The upstream camera receives Gazebo intrinsics and BGR frames through optional
  injection hooks. Its image and coordinate math remain upstream behavior.
  Configured diagnostics use the separate precision camera's verified mounting,
  atomic observation metadata, and 0.25-second exposure-age limit.
- `RosLidar.get_sample()` returns a testbench `LidarSample`; upstream mission
  functions use its scalar `get_distance()` interface. Repeated reads retain
  frame identity. Invalid, regressed, or older-than-0.5-second evidence fails.
- Comp2026 startup requires fresh exact-FC healthy disarmed heartbeat, prearm,
  grounded state, advancing native position time, verified firmware, and the
  complete calibration cache. Missing or unsafe observations keep admission
  closed. After matching RUNNING and accepted public time, the testbench repeats
  ground readiness and enqueues an explicit GUIDED packet by the inclusive
  50 ms public-time deadline. It then waits for observed GUIDED before entering
  upstream FM1. The
  [delivery window](src/drone_sim_companion/lifecycle.py),
  [start gate](src/drone_sim_companion/comp2026_host.py), and
  [runtime composition](src/drone_sim_companion/runtime_node.py) own this ordering.
- Phase completion records a returned upstream function. Home completion also
  waits up to 15 simulated seconds for fresh landed and disarmed telemetry.
  FM3 payload handling and ordinary LAND under `PLND_STRICT=2` remain current
  acceptance gaps described above.
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

Check the upstream hooks, runner, and preserved diagnostic camera regressions:

```bash
uv run --locked python scripts/audit-comp2026-hooks.py
uv run --locked pytest companion/comp2026/tests/test_testbench_hooks.py \
  companion/tests/test_upstream_comp2026_runner.py -q
uv run --locked pytest companion/tests/test_precision_camera_body_mapping.py \
  companion/tests/test_precision_camera_freshness.py \
  companion/tests/test_precision_camera_recording.py \
  companion/tests/test_comp2026_camera.py \
  companion/tests/test_simulator_camera_calibration.py \
  companion/tests/test_configured_live_loop.py -q
```

The imported upstream revision contains no original mission test suite. The hook
tests and testbench regressions do not establish physical flight acceptance.
