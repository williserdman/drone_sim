# Orchestration Plan

## Responsibility

Own deterministic run lifecycle, `run_id` creation, configuration validation, Docker Compose startup and shutdown, endpoint readiness, terminal-state selection, and manifest metadata.

## Non-responsibilities

- Scheduling simulated events from wall time
- Physics, flight control, mission policy, scoring, or artifact encoding

## Implementation stages

1. Resolve the immutable operator template to a generated `run_id`, durable
   `configuration/run.json`, and canonical checksum.
2. Implement `CREATED`, `STARTING`, `READY`, `RUNNING`, `FINALIZING`, `COMPLETED`, `FAILED`, and `ABORTED` transitions.
3. Implement the fixed `drone-sim start`, `status`, `abort`, and
   `collect-results` commands and start Compose modules with a unique project.
4. Require artifact readiness before simulation time advances.
5. Route every terminal cause through bounded finalization using the durable
   run-directory control/status protocol and runtime-frozen quiescence barrier.
6. Write manifest metadata and preserve terminal results.

## Acceptance criteria

- Invalid transitions are rejected deterministically.
- Every run receives a unique identity before container startup.
- Clock or lockstep loss results in a diagnosed failure.
- Completed, failed, and aborted runs all finalize artifacts.
- The bag records through `FINALIZING`; the terminal manifest is authoritative.

## Phase subplans

- Phase 1: configuration schema, lifecycle contracts, and synthetic readiness
- Phase 2: real artifact readiness and finalization
- Phases 3-6: module-specific readiness and terminal-condition integration
