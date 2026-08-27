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

For resolved `comp2026_auto`, `runtime_node` instead creates one responsive ROS
executor and one blocking original-attempt worker. `Comp2026StartGate`
separates durable process readiness from permission to enter
`run_auto_attempt`; only the latter requires RUNNING-era clock, frame, range,
payload-service, heartbeat, and armable facts. The public-zero GUIDED send is a
lifecycle/rendezvous adaptation and does not enter the original phase machine
or arm the vehicle.

`comp2026_host` owns only focused boundary adapters:

- `SimulationClock` exposes the existing monotonically accepted `/clock` value
  and elapsed-simulation sleeps.
- `RosFrameSource` joins exact current-run image/metadata timestamps, retains a
  bounded pairing window rather than a recording queue, converts RGB bytes to
  BGR NumPy data, and advances `last_timestamp_ns` only on real delivery.
- `RosLidar` retains one genuine timestamped range and rejects age greater than
  500,000,000 simulation nanoseconds.
- `PayloadDropper` maps the original attach/drop shape to one blocking typed
  service call with exact command/response correlation.
- `MissionEventEmitter` numbers nested phase callbacks and stamps them with the
  current accepted simulation clock without rebasing.
- `load_course_waypoints` converts the one resolved H-relative ENU course
  source around DroneKit Home using radius 6,378,137 m, x=east, and y=north.

The nested source is imported from `/opt/drone_sim/comp2026/src`; parent code
does not edit it. Python's startup compatibility hook defines only the removed
`collections.MutableMapping` and `inspect.getargspec` aliases before the pinned
DroneKit import. The package explicitly owns DroneKit's otherwise undeclared
`future` distribution.

On original-attempt failure, recovery requests RTL, land, and disarm
best-effort. The executor continues callbacks until orchestration finalization;
then the clock, frame, gate, and payload waits stop, the worker joins,
quiescence is written, and DroneKit closes.

## Future seams

Further vision changes remain inside the original nested mission or a later
explicit integration task. They must preserve `run_id`, genuine frame identity
and simulation timestamps, and the prohibition on direct Gazebo control.
