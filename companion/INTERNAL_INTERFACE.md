# Companion Internal Interface

## Current interface

- `mission.advance(state, telemetry)` is the pure, immutable mission seam. It
  reads no clocks and returns commands plus structured event facts.
- `MissionController` sends returned commands through the `VehicleCommands`
  protocol and commits state only after the send succeeds. Its passive path
  independently latches heartbeat and healthy prearm telemetry without calling
  `mission.advance`. `begin_mission(0)` is the single initial-command seam and
  invokes its delivery callback only after the transport send returns.
- `MavlinkAdapter` is the only PyMAVLink boundary. It translates the four
  mission commands and stamps received messages with caller-supplied
  simulation time.
- `CompanionLifecycle` owns transport readiness, exact one-shot durable
  `mission-ready`, mission terminal status, structured output, and the final
  quiescence boundary.
- `runtime_node` alone imports ROS 2 and the real PyMAVLink connection. It
  persists the controller's passive readiness facts through the generic
  `RuntimeProtocol.write_status` boundary and gates mission processing plus the
  telemetry-stream request on `RUNNING` and receipt of the first public clock.
  It dispatches `begin_mission(0)` before the first post-zero telemetry poll.

## Future seams

Future vision modules may use language-native imports. Their documented
interfaces must preserve `run_id`, source frame identity, and simulation
timestamps, and must not expose direct Gazebo control.
