# Orchestration Plan

## Responsibility

Own deterministic run lifecycle, `run_id` creation, configuration validation, Docker Compose startup and shutdown, endpoint readiness, terminal-state selection, and manifest metadata.

## Non-responsibilities

- Scheduling simulated events from wall time
- Physics, flight control, mission policy, scoring, or artifact encoding

## Implementation stages

1. Define and validate immutable run configuration.
2. Implement `CREATED`, `STARTING`, `READY`, `RUNNING`, `FINALIZING`, `COMPLETED`, `FAILED`, and `ABORTED` transitions.
3. Start Compose modules and wait for their real endpoints.
4. Require artifact readiness before simulation time advances.
5. Route every terminal cause through bounded finalization.
6. Write manifest metadata and preserve terminal results.

## Acceptance criteria

- Invalid transitions are rejected deterministically.
- Every run receives a unique identity before container startup.
- Clock or lockstep loss results in a diagnosed failure.
- Completed, failed, and aborted runs all finalize artifacts.

## Phase subplans

- Phase 1: configuration schema, lifecycle contracts, and synthetic readiness
- Phase 2: real artifact readiness and finalization
- Phases 3-6: module-specific readiness and terminal-condition integration

