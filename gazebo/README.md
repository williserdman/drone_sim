# Gazebo

[Project overview](../README.md) · [Architecture](../docs/architecture.md) ·
[Runbook](../docs/runbook.md)

Gazebo owns authoritative simulation time, physics, dynamics, collisions,
sensors, cameras, vehicle and payload physical truth, local world/model
resources, and native server evidence. Its runtime validates private Gazebo
outputs before publishing the public ROS 2 view of the world.

It does **not** choose missions, stabilize the aircraft, authorize payload
actions, calculate scores, record the public ROS/video artifacts, or write the
aggregate run manifest. Apart from the explicit electromagnet coordinator seam,
other modules must not consume private Gazebo Transport names or load world
resources as a substitute for public physical truth.

## Entry points and implementation seams

- [pyproject.toml](pyproject.toml) exposes `drone-sim-gazebo-runtime`;
  [runtime_node.py](src/drone_sim_gazebo/runtime/runtime_node.py) is the active
  live composition root.
- [runtime/model.py](src/drone_sim_gazebo/runtime/model.py) defines the pure
  lifecycle/action model, while
  [runtime/entrypoint.py](src/drone_sim_gazebo/runtime/entrypoint.py) applies
  transport control and finalization effects.
- [server/process.py](src/drone_sim_gazebo/server/process.py) owns the headless
  `gz sim` child, native recording, server log, and deadline-bounded shutdown.
- [ros_adapter/node.py](src/drone_sim_gazebo/ros_adapter/node.py) is the sole
  public Gazebo ROS publisher; its pure validation and epoch logic live beside it.
- [competition_config.py](src/drone_sim_gazebo/competition_config.py) validates
  the fixed competition geometry shared with mission/scenario configuration.
- [resources/](resources/) contains the runtime-owned worlds, vehicle models,
  payload models, meshes, and marker textures. Resolution is local and
  fail-closed; remote model fallback is not part of the contract.
- [prepare_competition_assets.py](scripts/prepare_competition_assets.py) generates
  `iris_flight`, `iris_moving_pad`, and `iris_competition` from one physical
  vehicle template. Each unloaded model is 1.66001 kg with ±3.4 motor limits,
  the competition camera/range hardware, and the payload hardpoint. Only
  `iris_competition` receives mission payload attachment plugins.
- [plugin/](plugin/) contains the clock decimator, payload coordinator, and the
  downstream-patched ArduPilot Gazebo integration. Its moving-pad controller
  drives a world-fixed prismatic rail with joint velocity; it never resets the
  deck pose or attaches a landed vehicle.

## Interfaces

The runtime consumes resolved run configuration, run lifecycle/finalization
facts, private bridged Gazebo samples, and the ArduPilot JSON actuator exchange.
Gazebo Transport control and native topic names stay private to this module.

It publishes public `/clock`, onboard and observer images plus metadata,
vehicle ground truth, and—during competition—payload state and downward range.
Read the exact schemas in
[FrameMetadata.msg](../ros_ws/src/simulation_interfaces/msg/FrameMetadata.msg),
[GroundTruth.msg](../ros_ws/src/simulation_interfaces/msg/GroundTruth.msg),
[LandingPadState.msg](../ros_ws/src/simulation_interfaces/msg/LandingPadState.msg), and
[PayloadState.msg](../ros_ws/src/simulation_interfaces/msg/PayloadState.msg).
The full producer/consumer and lifecycle rules are in the
[architecture guide](../docs/architecture.md), not duplicated here.

Gazebo also provides module readiness, source-finished, failure, native state,
server-log, and quiescence evidence. Payload command authorization belongs to
the electromagnet; the Gazebo-side coordinator only applies a correlated
private command and reports confirmed joint state.

The shared [runtime status contract](../artifacts/src/artifacts/runtime_status.py)
defines those durable facts. In particular, typed Gazebo readiness contains a
real bidirectional ArduPilot [`FlightExchange`](../artifacts/src/artifacts/runtime_status.py).
The runtime publishes it through the
[container protocol adapter](../artifacts/src/artifacts/runtime_protocol.py).

## Constraints worth knowing

- Public output is rebased from one configured native epoch. Activation must be
  armed before the exact target; late activation or a skipped target faults
  closed. Public zero precedes buffered output, and the server is not reset.
  Configured missions release the paused epoch on `mission-execution-ready`;
  other missions retain `mission-command-delivered`. A timed-out world-control
  reply is accepted only when world statistics prove the requested effect;
  otherwise the runtime retries once and faults closed.
- Camera and aligned physical-truth samples advance at the configured 20 Hz.
  Invalid, duplicate, regressing, misaligned, or excess samples latch the first
  adapter fault; later input cannot repair it.
- `moving_pad_landing` starts a 3 m deck at ENU `(10, 0)` and releases its
  eastward 0.5 m/s rail motion after the fixed 90 s native warmup.
  `moving_pad_stationary` places the same deck at `(35, 0)` with zero speed for
  the control run. Both use `iris_moving_pad`, a 640x480, 20 Hz, 0.6-radian
  downward camera and range sensor with the shared empty hardpoint but no
  attached payload. ArUco 7 is 0.1 m square at the 0.2 m deck top; the observer
  covers the complete route.
- `/simulation/landing_pad_state` publishes measured deck pose/twist and only
  reports contact when a scoped Iris leg collision touches the scoped deck
  collision. Warmup samples are excluded, and missing native contact evidence
  is a fault rather than an airborne sample.
- Flight readiness requires a paused ArduPilot/Gazebo round trip with servo,
  motor-update, and JSON-send progress and no frame gaps or send errors.
- The passive `phase3_foundation` world has no ArduPilot exchange, so the typed
  readiness contract no longer supports it. The runtime rejects that world
  before starting the Gazebo server. Restoring operator support requires a
  separate contract decision; generic Gazebo endpoint readiness cannot stand in
  for flight exchange evidence.
- Finalization uses one absolute wall deadline. Bridge process groups stop
  before the Gazebo server; quiescence is published only after native artifacts
  are stable and validated.
- `GZ_SIM_RESOURCE_PATH` is replaced with the local models directory. Resource
  and provenance updates must follow the audit/refresh procedure in the
  [runbook](../docs/runbook.md); do not hand-edit generated competition assets.

## Focused tests

Run from the project root:

```bash
uv run --locked pytest gazebo/tests -q
```

The physical carriage gate runs inside the Gazebo build:

```bash
ctest --test-dir /tmp/ardupilot_gazebo/build \
  -R moving_pad_controller_integration_test --output-on-failure
```

ROS-dependent cases may skip on a host without ROS Jazzy. This suite does not
build the image or launch a flight; use the [runbook](../docs/runbook.md) for
container and end-to-end checks.
