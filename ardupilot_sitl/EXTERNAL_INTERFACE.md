# ArduPilot SITL External Interface

## Companion MAVLink seam

- Input: flight and mission commands
- Output: telemetry, vehicle mode, state, and command acknowledgements
- Endpoint: `tcp://ardupilot-sitl:5760` on the Compose network only; no host
  port is published.

The SITL process reports its own readiness only after the JSON exchange is
active and its MAVLink TCP listener is bound. Production orchestration also
requires the companion to observe an actual heartbeat and healthy prearm state
before `RUNNING`; a bound socket alone is not flight-readiness evidence. The
parameter overlay passively emits `SYS_STATUS` at 1 Hz during warmup so prearm
readiness can be observed without changing arming checks.

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

The parameter overlay retains normal ArduPilot pre-arm enforcement. Its
accelerometer offsets and scale factors are the calibration markers from the
pinned upstream `Tools/autotest/default_params/copter.parm`; ArduPilot's SITL
defaults require the small non-zero offsets so its two simulated
accelerometers are recognized as calibrated. The mission uses ordinary
GUIDED-mode arming and never sends the force-arm magic value.

The overlay sets the pinned Copter 4.7 parameter `LAND_SPD_MS=0.05`, making
the final LAND-stage vertical target 0.05 m/s. Copter 4.7 renamed the legacy
centimetres-per-second `LAND_SPEED` parameter to the metres-per-second
`LAND_SPD_MS`; the runtime uses the new name directly because every run wipes
SITL storage. This target keeps 50% command-speed headroom below the frozen
0.1 m/s first-contact stability ceiling. It is below the upstream parameter
metadata's recommended 0.3 m/s minimum, but the pinned defaults-file loader
accepts the float without clamping and the LAND controller applies its
absolute value directly as the final descent limit.

## Failure behavior

Loss of either required peer is reported. Loss of the Gazebo exchange prevents continued simulated progress; malformed or unsupported MAVLink commands receive the protocol-defined rejection where available.

ArduPilot's JSON resend diagnostic is preserved as structured child output but
is not classified as peer loss: upstream emits it after a bounded wall-time
receive wait and retransmits the current servo packet so a slow lockstep peer
can recover. Gazebo reports authoritative server, bridge, and adapter failures;
an otherwise silent exchange stall is bounded by the run's host-wall deadline.
Neither mechanism advances simulation time. Owned evidence is preserved under
`ardupilot_sitl/` in the run directory: DataFlash `logs/*.BIN`, SITL storage,
and `failure.json`. Structured module events wrap third-party stdout and stderr
so the public container stream remains parseable JSON Lines.

For Phase 2 synthetic finalization, the stub stops publisher/log output before
writing `.status/quiescence/ardupilot_sitl.json` with exact current-run
quiescence schema, then remains silent while orchestration aggregates the
freeze.

## Version and supply

- Tag: `Copter-4.7.0`
- Commit: `1511f27194f1dcc3728270883047bdf022b3fd53`
- Build target: `bin/arducopter`
- License and recursive submodule inventory are preserved in the runtime image.
