# ArduPilot SITL External Interface

## Companion MAVLink seam

- Input: flight and mission commands
- Output: telemetry, vehicle mode, state, and command acknowledgements

The endpoint is ready only after MAVLink communication is operational.

## Gazebo adapter seam

- Output: actuator commands
- Input: simulated sensors and dynamics

ArduPilot and Gazebo advance in lockstep: the next control update depends on completion of the preceding physics/sensor exchange.

## Failure behavior

Loss of either required peer is reported. Loss of the Gazebo exchange prevents continued simulated progress; malformed or unsupported MAVLink commands receive the protocol-defined rejection where available.

For Phase 2 synthetic finalization, the stub stops publisher/log output before
writing `.status/quiescence/ardupilot_sitl.json` with exact current-run
quiescence schema, then remains silent while orchestration aggregates the
freeze.

## Deferred decisions

- MAVLink ports and routing
- ArduPilot-Gazebo adapter version and transport details
- Exact lockstep readiness signal
