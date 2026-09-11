# ArduPilot SITL

[Project overview](../README.md) · [Architecture](../docs/architecture.md) ·
[Runbook](../docs/runbook.md)

This module runs the simulated flight controller: estimation, navigation, and
vehicle-control loops backed by ArduPilot Copter SITL. It connects companion
mission commands to the Gazebo sensor/actuator lockstep exchange.

It does **not** own mission or vision decisions, authoritative physics or
ground truth, scenario behavior, scoring, or aggregate lifecycle state. It is
an ArduPilot SITL wrapper, not a Pixhawk simulator.

## Entry points and owned files

- [pyproject.toml](pyproject.toml) exposes `drone-sim-ardupilot-runtime`.
- [runtime_node.py](src/drone_sim_ardupilot/runtime_node.py) is the production
  process wrapper and durable readiness/failure/quiescence boundary. It publishes
  typed values from the shared
  [runtime status contract](../artifacts/src/artifacts/runtime_status.py) through
  the [container protocol adapter](../artifacts/src/artifacts/runtime_protocol.py).
- [config.py](src/drone_sim_ardupilot/config.py) validates run inputs, resolves
  the Gazebo service once, and constructs the shell-free ArduCopter command.
- [runtime.py](src/drone_sim_ardupilot/runtime.py) supervises SITL, interprets
  readiness output, writes events, and inventories diagnostics.
- [json_peer.py](src/drone_sim_ardupilot/json_peer.py) is a bounded test peer for
  the upstream UDP protocol; production does not use it.
- [descent.parm](params/descent.parm) is the image-baked parameter overlay, and
  [Dockerfile](Dockerfile) pins the upstream build and runtime layout.

## Interfaces

The companion connects over Compose-only MAVLink TCP at
`tcp://ardupilot-sitl:5760`. ArduPilot consumes flight commands and provides
telemetry, modes, state, acknowledgements, and `STATUSTEXT` diagnostics.

The upstream JSON backend exchanges servo outputs and simulated sensor/dynamics
data with `gazebo-runtime:9002` over UDP. Sensor replies retain
`no_time_sync=true` and `no_lockstep=false`; loss of the Gazebo exchange must
prevent free-running simulation progress. Broader ownership and ordering are in the
[architecture guide](../docs/architecture.md).

The module provides its own structured child output, readiness fact, DataFlash
logs, SITL storage, failure evidence, and quiescence marker. A bound MAVLink
listener is not mission readiness: orchestration also waits for the companion
to observe a real heartbeat and healthy prearm status.

The wrapper reads the shared finalize request through `RuntimeProtocol`, which
rejects malformed, unsafe, or changed control files. Shutdown calls the child
stop/reap operation once, then attempts private failure evidence, diagnostic
inventory, terminal logging, and shared failure publication in order even if an
earlier cleanup step fails. The wrapper preserves the first exception and notes
later cleanup errors on it. It publishes quiescence only after stop/reap confirms
that SITL exited, and it always attempts to close the protocol.

## Constraints worth knowing

- The image freezes ArduPilot `Copter-4.7.0` at commit
  `1511f27194f1dcc3728270883047bdf022b3fd53` and builds only `waf copter`.
  Supply-chain details live in
  [ardupilot.json](provenance/ardupilot.json) and
  [LICENSE.ArduPilot.txt](provenance/LICENSE.ArduPilot.txt).
- The pinned JSON backend accepts numeric IPv4 addresses, so startup resolves
  the Compose service name once. Name resolution does not advance simulation.
- Keep `no_time_sync=true`, `no_lockstep=false`, and `--speedup 1`; wall
  deadlines bound unavailable infrastructure, never mission time.
- The parameter overlay deliberately preserves normal prearm checks. Its small
  accelerometer calibration offsets come from upstream SITL defaults; do not
  replace them with force-arm behavior.
- Copter 4.7 uses `LAND_SPD_MS`, not legacy `LAND_SPEED`. Parameter edits require
  rebuilding the image because the overlay is copied at build time.
- The overlay is also the single source for the fast, guarded precision-landing
  profile. `PLND_OPTIONS` retains the normal final descent speed while the
  companion filters measurements and owns hold/reacquire policy. Do not tune a
  missed pickup through the scorer or by silently overriding these values at
  runtime; inspect [descent.parm](params/descent.parm) and its executable
  assertions in [test_config.py](tests/test_config.py).
- ArduPilot's JSON resend message is a recoverable upstream retry diagnostic,
  not by itself peer-loss evidence.
- The private `work/failure.json` file remains a child-process diagnostic. It is
  not a shared runtime status or quiescence record.

## Focused tests

Run from the project root:

```bash
uv run --locked pytest ardupilot_sitl/tests -q
```

The suite exercises configuration, lifecycle, process supervision, output
recognition, and the bounded JSON test peer. It does not build ArduPilot or
prove a live Gazebo flight; follow the [runbook](../docs/runbook.md) for those
checks.
