# ArduPilot SITL External Interface

## Companion MAVLink seam

- Input: flight and mission commands
- Output: telemetry, vehicle mode, state, and command acknowledgements
- Endpoint: `tcp://ardupilot-sitl:5760` on the Compose network only; no host
  port is published.

The SITL process reports its own readiness only after the JSON exchange is
active and its MAVLink TCP listener is bound. Production orchestration also
requires the companion to observe an actual heartbeat before aggregate
readiness; a bound socket alone is not heartbeat evidence.

## Gazebo adapter seam

- Output: actuator commands
- Input: simulated sensors and dynamics

ArduPilot and Gazebo advance in lockstep: the next control update depends on completion of the preceding physics/sensor exchange.

The frozen seam is ArduPilot's `JSON` model targeting
`gazebo-runtime:9002`. Sensor responses carry `no_time_sync=true` and
`no_lockstep=false`. The runtime passes `--speedup 1`; it never enables the
free-running JSON mode. `--sim-port-in 9003` remains explicit for the upstream
interface even though the JSON backend receives replies on the UDP socket that
sent each servo frame.

The pinned upstream socket accepts numeric IPv4 addresses only. At
infrastructure startup the wrapper resolves the Docker service name once and
passes that IPv4 address to ArduPilot; failure to resolve is a startup failure.
This resolution does not schedule or advance simulated time.

## Failure behavior

Loss of either required peer is reported. Loss of the Gazebo exchange prevents continued simulated progress; malformed or unsupported MAVLink commands receive the protocol-defined rejection where available.

After an established exchange, ArduPilot's first JSON resend diagnostic fails
the runtime closed only while the current run is durably `RUNNING`. The same
diagnostic is an expected infrastructure pause before `RUNNING` and after the
current run's `source-finished` or finalization request. Owned evidence is
preserved under `ardupilot_sitl/` in the run directory: DataFlash `logs/*.BIN`,
SITL storage, and `failure.json`. Structured module events wrap third-party
stdout and stderr so the public container stream remains parseable JSON Lines.

For Phase 2 synthetic finalization, the stub stops publisher/log output before
writing `.status/quiescence/ardupilot_sitl.json` with exact current-run
quiescence schema, then remains silent while orchestration aggregates the
freeze.

## Version and supply

- Tag: `Copter-4.7.0`
- Commit: `1511f27194f1dcc3728270883047bdf022b3fd53`
- Build target: `bin/arducopter`
- License and recursive submodule inventory are preserved in the runtime image.
