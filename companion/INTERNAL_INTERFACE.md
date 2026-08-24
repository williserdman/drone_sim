# Companion Internal Interface

## Current interface

- `mission.advance(state, telemetry)` is the pure, immutable mission seam. It
  reads no clocks and returns commands plus structured event facts.
- `MissionController` sends returned commands through the `VehicleCommands`
  protocol and commits state only after the send succeeds.
- `MavlinkAdapter` is the only PyMAVLink boundary. It translates the four
  mission commands and stamps received messages with caller-supplied
  simulation time.
- `CompanionLifecycle` owns readiness, mission terminal status, structured
  output, and the final quiescence boundary.
- `runtime_node` alone imports ROS 2 and the real PyMAVLink connection.

## Future seams

Future vision modules may use language-native imports. Their documented
interfaces must preserve `run_id`, source frame identity, and simulation
timestamps, and must not expose direct Gazebo control.
