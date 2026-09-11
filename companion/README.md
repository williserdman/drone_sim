# Companion

[Project overview](../README.md) · [Architecture](../docs/architecture.md) ·
[Runbook](../docs/runbook.md)

The companion owns mission, vision, and autonomy decisions. It turns public
simulation inputs and ArduPilot telemetry into flight commands, payload
requests, mission events, and durable mission lifecycle evidence.

It does **not** own flight stabilization, actuator control, physics, sensor
truth, direct Gazebo mutation, scoring, or aggregate run finalization.

## Guarded `comp2026_auto` host

When a resolved run supplies the complete QGC input set, `comp2026_auto`
composes the nested QGC listener as the sole flight-command owner. Without that
set, it retains the tracked automatic mission path. The QGC parent first projects
and validates the immutable deployment profile,
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
range adapter. Limited FM1/FM2 mode keeps the marker-2 release adapter and
reports no attachment support. Full mode uses one confirmed payload adapter for
markers 2, 3, and 4 and reports attachment support; FM3 camera construction
remains disabled. The admitted FM1's
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
an offline, inert loader for caller-supplied simulator policy. It accepts either
the exact `drone-sim-comp2026-fm1-fm2` contract or the exact
`drone-sim-comp2026-full` contract. The full variant requires ordered FM1, FM2,
and FM3 command IDs plus explicit vision and precision controls; its projection
adds exact WA and WM1 pickup waypoints and version-bound packaged calibration
and mounting resources. The caller supplies the exact lowercase SHA-256 of a
regular, non-symlink UTF-8 JSON file. The loader validates the complete
simulator-specific policy and returns frozen normalized values. It does not open
ROS, DroneKit, a transport, or a device. An evidence-reference hash does not
enforce a recovery corridor.

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
- Full QGC mode advertises payload attachment support, but camera construction
  remains disabled. Camera policies and adapters elsewhere in the source do not
  make FM3 runnable yet.
- The controlled-descent path requires positive command acknowledgements and
  observed vehicle state. Heartbeat and healthy prearm observations are separate
  passive readiness facts, and commands wait for `RUNNING` plus public clock.
- `companion/comp2026` is tracked in this monorepo and supplies the
  automatic mission and guarded QGC listener source. Keep changes there focused
  and preserve its imported history and provenance. The Docker context admits
  only its explicit import closure.
- Building the Phase 3 companion image requires
  `SIM_COMP2026_REVISION=$(git rev-parse HEAD)`. Compose
  leaves the build argument empty when it is not supplied so inactive profiles
  and noncompanion configuration still resolve; the companion Dockerfile rejects
  an empty value before package installation or source copies.
- The automatic `drone.auto_attempt` host imports FM1 and FM2 from `missions/`
  but imports FM3 from `drone/mock_mission.py`. The guarded QGC host still
  admits only FM1 and FM2, so `missions/fm3.py` is not an active QGC path.
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
- Matching orchestration `FINALIZING`, a durable finalize request, and an
  operator signal latch one nested QGC abort and one monitoring stop without
  stopping the shared simulation clock. The nested runtime keeps that clock
  through its approved recovery. Fatal ROS input or executor failures and the
  absolute wall deadline may stop the clock.
- The limited QGC simulation payload adapter reports no attachment support and
  releases only marker 2. The full adapter starts with marker 2 attached and
  permits only the confirmed sequence release 2, attach/release 3, then
  attach/release 4. It requires a literal-true permission predicate around the
  exact ROS dispatch and advances only for an accepted, command-ID-matched
  `OK` response with a strictly increasing positive sequence. Any uncertain
  dispatched result latches an indeterminate state and blocks later commands.

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
