# Companion

[Project overview](../README.md) · [Architecture](../docs/architecture.md) ·
[Runbook](../docs/runbook.md)

The companion owns mission, vision, and autonomy decisions. It turns public
simulation inputs and ArduPilot telemetry into flight commands, payload
requests, mission events, and durable mission lifecycle evidence.

It does **not** own flight stabilization, actuator control, physics, sensor
truth, direct Gazebo mutation, scoring, or aggregate run finalization.

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
  for the separately supplied Comp2026 mission checkout.
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
- The controlled-descent path requires positive command acknowledgements and
  observed vehicle state. Heartbeat and healthy prearm observations are separate
  passive readiness facts, and commands wait for `RUNNING` plus public clock.
- `companion/comp2026` is a separate, untracked checkout required for the
  `comp2026_auto` image. Keep changes there minimal and never push it as part of
  this repository. The Docker context admits only its explicit import closure.
- The hosted `drone.auto_attempt` currently imports FM1 and FM2 from `missions/`
  but imports FM3 from `drone/mock_mission.py`; do not assume
  `missions/fm3.py` is the deployed implementation.
- Comp2026 startup separates process readiness from permission to enter the
  original mission. Sensor, service, heartbeat, and armability predicates are
  refreshed atomically and fail closed; downward range expires after 0.5
  simulated seconds.
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
composition. They do not supply the nested checkout or prove a live flight; use
the [runbook](../docs/runbook.md) for image and end-to-end procedures.
