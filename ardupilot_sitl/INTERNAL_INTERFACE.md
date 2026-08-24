# ArduPilot SITL Internal Interface

The production process remains independently deployed; siblings use only its
external interfaces. The package exposes a small testable internal boundary:

- `RuntimeConfig` validates run identity/endpoints and produces the shell-free
  ArduCopter command.
- `RuntimeState` / `transition` model readiness, failure, and quiescence without
  consulting wall time.
- `SITLProcess` owns bounded process I/O and infrastructure shutdown.
- `OutputFacts` latches upstream JSON-exchange and MAVLink-listener readiness and
  reports each missing-JSON diagnostic to the lifecycle policy.
- `DurableLifecycle` classifies JSON peer loss from exact current-run
  `runtime-running`, `source-finished`, and finalization facts.
- `JsonPeer` is a bounded test gate for the upstream binary's real 16/32-channel
  UDP servo packet and newline-delimited JSON sensor response. It is not used in
  production and never repairs frame gaps.

Wall timeouts exist only in `SITLProcess` and `JsonPeer` to bound unavailable
infrastructure during startup, testing, and shutdown. No mission or simulated
event decision depends on them.
