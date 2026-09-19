# Companion

[Project overview](../README.md) · [Architecture](../docs/architecture.md) ·
[Runbook](../docs/runbook.md)

The companion owns mission, vision, and autonomy decisions. It turns public
simulation inputs and ArduPilot telemetry into flight commands, payload
requests, mission events, and durable mission lifecycle evidence.

## Configured diagnostic missions

The `configured` runner executes a fixed sequence of shared drone operations.
Each operation has a name, validated arguments, an operation ID, and an observed
terminal result. The runner waits for success before starting the next step;
failure stops the sequence. Mode selection and arming are explicit operations.
An operator-driven plan can instead wait for observed armed GUIDED state.
Arming establishes the mission-start timestamp; no competition countdown applies.

Start from [configured-descent-run.json](../config/configured-descent-run.json)
for automatic flight or
[configured-operator-run.json](../config/configured-operator-run.json) to wait
for an operator. Each run template contains `mission_plan` with `schema_version: 1`
and a nonempty `steps` array. Each step contains `tool`, `args`, and an optional
`timeout_sim_s`, default 60. The complete normalized plan is captured inside the
checksum-bound `configuration/run.json`; the runtime never rereads the template.

[mission_plan.py](src/drone_sim_companion/mission_plan.py) defines and validates
the tool arguments and exports JSON-compatible `tool_definitions()` for future
callers. Unknown tools, unknown arguments, nonfinite numbers, and invalid plans
fail before ROS or MAVLink resources are created.

| Tool | Arguments and completion |
| --- | --- |
| `wait_for_state` | One or more of `armed`, `mode`, `landed`; waits for matching fresh telemetry without issuing commands. |
| `set_mode` | `mode: GUIDED`; requires command acceptance and observed mode. |
| `arm` | Empty arguments; requires fresh GUIDED and healthy prearm state, then command acceptance and observed arming. |
| `takeoff` | Positive `altitude_m`, optional positive `tolerance_m` smaller than the target height; default is the smaller of 0.15 m and 10% of the height. Requires already armed GUIDED state, command acceptance, and reaching the altitude threshold. |
| `goto_waypoint` | `latitude_deg`, `longitude_deg`, positive `altitude_m`, optional positive `tolerance_m`, default 1; waits for fresh position within horizontal and altitude tolerances. |
| `hold` | Positive `duration_sim_s`, shorter than the step timeout; leaves the existing GUIDED target in place while waiting. Requires armed GUIDED state throughout. |
| `land` | Empty arguments; requires command acceptance followed by observed touchdown and disarm. Already landed/disarmed is a successful no-op. |

Waypoints use WGS84 latitude/longitude in degrees and altitude in metres above
ArduPilot home, following
[SET_POSITION_TARGET_GLOBAL_INT](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT).
They are numeric coordinates, not named competition waypoints. Takeoff altitude
also uses metres above home. Arming time is the first observed armed state in
the public simulation epoch; a vehicle already armed when first observed has no
reconstructed earlier timestamp. Step deadlines use simulation time; the run's
wall deadline still bounds stalled infrastructure and operator waiting.

Only one flight operation runs at a time. Operations advance from telemetry and
simulation-clock ticks, leaving the runtime responsive to observation and abort.
The future agent caller can use `DroneOperations.start()`, `operation_status()`,
and `read_vehicle_state()` from
[operations.py](src/drone_sim_companion/operations.py). Calls are serialized on the
runtime owner thread; no agent endpoint is installed. Heartbeats older than three
simulation seconds fail active flight operations. Completed operations retain
their status; an ACK alone never establishes flight completion.

[configured_runtime.py](src/drone_sim_companion/configured_runtime.py) opens a
distinct `mission-execution-ready` gate after passive readiness, matching RUNNING,
and public clock. This lets physics advance during an operator wait without
sending a GUIDED command. Other mission hosts retain their existing start gate.
Only a completed sequence with observed landing and disarm publishes
`mission-finished`. Failure or abort stops the sequence and may attempt one local
LAND when a fresh heartbeat still reports armed GUIDED/LAND. It does not override
another observed mode. Recovery is bounded by the configured finalization wall
budget and the overall wall deadline, and never changes the failed mission result.

Deferred work: precision landing, camera/LiDAR tools, agent transport, named
waypoints, branching, retries, and resource-exclusive actions. Unsupported tools
are rejected before flight. Camera/FM3 remains disabled in the existing QGC
composition; this runner does not change the independent Comp2026 checkout.
Host tests exercise source behavior; current container/flight verification is
recorded separately in [handoff](../docs/handoff.md).

It does **not** own flight stabilization, actuator control, physics, sensor
truth, direct Gazebo mutation, scoring, or aggregate run finalization.

## Guarded `comp2026_auto` host

`comp2026_auto` now composes the nested QGC listener as the sole flight-command
owner. The parent first projects and validates the immutable deployment profile,
listener session, action, runtime policy, course, scenario, and external attempt
state. Only then may it create ROS or DroneKit resources. The listener receives
the projection's sealed artifact snapshot; it does not reopen mutable run paths.

The parent publishes mission-ready and opens QGC command admission only after a
matching `RUNNING` state, accepted public clock, live ROS input executor, actual
connected-vehicle heartbeat within the validated freshness bound, and literal
`is_armable is True`. Its explicit simulator-only telemetry mode installs the
source-filtered collector, proves interval-command ACKs and firmware metadata,
and reaches listener readiness without waiting for cadence from paused physics.
It sends no automatic first command. FM1 uses the nested controller and ROS
range adapter; FM2 uses payload marker ID 2 and reports no attachment support.
FM3/camera construction is disabled in this first binding. The admitted FM1's
first guarded GUIDED transport enqueue is the release signal. The parent
publishes the durable command-delivery fact at the accepted public clock
timestamp, after which the controller requires complete post-gate telemetry on
strictly advancing shared simulation time before ARM or TAKEOFF. Admission
alone never publishes the fact or permits those commands.

The parent publishes `/simulation/mission_events` with reliable,
transient-local depth-100 QoS. This FM1/FM2 composition emits only the ordered
prefix `FM1 STARTED`, `FM1 COMPLETE`, `FM2 STARTED`, `FM2 COMPLETE`, with IDs
0 through 3 and current accepted simulation timestamps. A STARTED event follows
command execution admission and the deadline check. COMPLETE follows a
supervisor-finalized success; final FM2 also follows confirmed `HOME_LANDED`
recovery. Rejection, replay, failure, abort, waypoint updates, and recovery do
not invent completion, HOME, or FM3 events. Publisher failure is terminal for
the host and is never retried.

The host requires both required mission-event consumers before opening
admission: root-namespace `drone_sim_scorekeeper` and the root-namespace
`rosbag2_recorder_...` process. ROS graph endpoint metadata must also report the
exact mission-event type with reliable, transient-local QoS. The host rechecks
both identities for every publish and final flush. It maps simulation
nanoseconds to the ROS `builtin_interfaces/Time` `sim_timestamp` seconds and
nanoseconds fields. Before destroying the publisher, it calls
`wait_for_all_acked` with the validated cleanup timeout while the executor is
still active. A missing subscriber, timeout, or unconfirmed acknowledgement
prevents durable `mission-finished` success.

That four-event prefix can support an 80/150 diagnostic score when the physical
FM1/FM2 evidence also passes. It cannot complete the official
`competition_v1` mission, which still requires FM3 and HOME evidence and reports
`mission_sequence_invalid` for this prefix.
This source composition is not evidence of an image build, integrated SITL run,
scored simulation, hardware acceptance, or flight readiness.

## Simulator QGC runtime policy

[qgc_runtime_policy.py](src/drone_sim_companion/qgc_runtime_policy.py) provides
an offline, inert loader for a caller-supplied FM1/FM2 simulator policy. The
caller supplies the exact lowercase SHA-256 of a regular, non-symlink UTF-8 JSON
file. The loader validates the complete simulator-specific policy and returns
frozen normalized values. It does not open ROS, DroneKit, a transport, or a
device. An evidence-reference hash does not enforce a recovery corridor.

Runtime policy values still come only from the explicitly supplied immutable
QGC input set. Repository default configurations omit that set. Aircraft
deployment, operating-area, independent-safety, and integration gates remain
closed.

## Entry points and implementation seams

- [pyproject.toml](pyproject.toml) exposes `drone-sim-companion-runtime`.
- [runtime_node.py](src/drone_sim_companion/runtime_node.py) is the live ROS 2,
  MAVLink, mission-selection, lifecycle, and teardown composition root.
- [mission.py](src/drone_sim_companion/mission.py) and
  [controller.py](src/drone_sim_companion/controller.py) contain the pure
  controlled-descent policy and its command side-effect boundary.
- [mavlink_adapter.py](src/drone_sim_companion/mavlink_adapter.py) is the only
  PyMAVLink translation boundary.
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

The companion provides MAVLink commands, correlated payload service requests,
ordered mission events, structured diagnostics, and its own readiness,
completion, failure, and quiescence facts. It never publishes physical truth.

## Constraints worth knowing

- Mission decisions use accepted simulation time. Loss of `/clock` prevents new
  simulated decisions; wall time only bounds infrastructure and computation.
- The first QGC host binding disables FM3 and camera construction. Camera
  policies and adapters elsewhere in the source do not make FM3 available.
- The controlled-descent path requires positive command acknowledgements and
  observed vehicle state. Heartbeat and healthy prearm observations are separate
  passive readiness facts, and commands wait for `RUNNING` plus public clock.
- `companion/comp2026` is tracked in this monorepo and supplies the
  `comp2026_auto` mission. Keep changes there focused and preserve its imported
  history and provenance. The Docker context admits only its explicit import
  closure.
- Building the Phase 3 companion image requires
  `SIM_COMP2026_REVISION=$(git rev-parse HEAD)`. Compose
  leaves the build argument empty when it is not supplied so inactive profiles
  and noncompanion configuration still resolve; the companion Dockerfile rejects
  an empty value before package installation or source copies.
- The hosted `drone.auto_attempt` currently imports FM1 and FM2 from `missions/`
  but imports FM3 from `drone/mock_mission.py`; do not assume
  `missions/fm3.py` is the deployed implementation.
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
- Before mission code reads competition inputs, startup verifies the resolved
  `course.yaml` and `scenario.yaml` copies against their SHA-256 digests in
  `run.json`.
- The first runtime failure wins across all modules. A later companion failure
  cannot replace the durable first cause.
- Comp2026 startup separates process readiness from QGC command admission.
  Matching run state, public clock, live input production, connected-vehicle
  heartbeat, and armability checks fail closed. Downward range time comes from
  the ROS source timestamp and is evaluated by the validated nested policy.
- Terminal success and the first fatal callback/mission failure are serialized.
  Quiescence follows worker termination, executor shutdown, and closure of all
  output producers. Unconfirmed nested cleanup or a surviving producer records
  failure and withholds durable quiescence.
- Matching orchestration `FINALIZING`, a durable finalize request, and an
  operator signal latch one nested abort and one monitoring stop without
  stopping the shared simulation clock. The nested runtime keeps that clock
  through its single recovery: an approved original-H transit at cruise
  altitude followed by `LAND`, or its separately approved local-`LAND`
  fallback. Fatal ROS input or executor failures and the absolute wall deadline
  may stop the clock; parent finalization never reacquires flight commands or
  assumes pilot recovery.
- The QGC simulation payload adapter reports no attachment support because its
  ROS service response confirms release rather than persistent attachment. It
  requires a literal-true
  permission predicate with an atomic actuation hook. That hook encloses the
  actual ROS `call_async` dispatch, while the bounded confirmation wait remains
  outside the output transaction. Construction also requires a finite positive
  wall-time budget for a requested simulation-time release delay, so missing or
  frozen simulation time fails without dispatch. Rejected, missing, mismatched,
  stale, or timed out confirmation fails the command. A stale release is not
  retried because an unknown physical completion cannot safely authorize another
  release.

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
