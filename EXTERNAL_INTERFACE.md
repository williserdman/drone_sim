# Root External Interface

## Operator surface

The orchestration surface conceptually provides `start`, `reset`, `status`, `collect-results`, and `stop`. Exact CLI or service syntax is deferred.

## Configuration inputs

- Unique `run_id`
- World, vehicle, mission, and scenario configuration
- ROS 2 discovery and network configuration
- Result and log destinations

Secret values must be supplied at runtime and must not be committed.

## Lifecycle operations

- `start`: validate configuration, start modules, and wait for readiness.
- `reset`: issue a new `run_id` and reset authoritative simulation state.
- `status`: report endpoint readiness and infrastructure health.
- `collect-results`: collect scores, logs, and run metadata.
- `stop`: stop the run cleanly without discarding results.

## Observable outputs

Lifecycle status, module diagnostics, score results, logs, recordings, ROS bags, and run metadata are correlated by `run_id`. A terminal run preserves a manifest describing completeness and checksums even when the run failed or was aborted.

## Health semantics

A container is ready only when its required process and communication endpoints are ready. Wall-clock health timeouts may identify a stalled host but never advance simulation state.

## Clock semantics

Gazebo's ROS 2 `/clock` is authoritative. Simulation-aware nodes enable `use_sim_time`.

## Failure behavior

Startup fails closed when required endpoints are unavailable. Loss of clock or lockstep progress pauses simulated decisions. Shutdown preserves available diagnostics and partial results.

## Deferred decisions

- Command-line or orchestration-service syntax
- Reset and readiness wire protocols
- Concrete Docker Compose topology and health-check intervals
