# Orchestration External Interface

## Operator operations

`start`, `status`, `abort`, and `collect-results` are the initial conceptual operations. Their CLI names remain stable once Phase 1 implements them.

## Module lifecycle

Modules receive `run_id`, configuration, output paths, and lifecycle state. Required endpoints and artifact recorders report readiness before `RUNNING`. Every terminal cause enters `FINALIZING`, after which the artifacts module reports completeness.

Lifecycle states use the fixed order `CREATED`, `STARTING`, `READY`, `RUNNING`,
`FINALIZING`, `COMPLETED`, `FAILED`, and `ABORTED`. The orchestrator publishes
`simulation_interfaces/msg/RunState` on `/simulation/run_state` with reliable,
transient-local QoS depth 1. Phase 1 proves this publisher with the synthetic
foundation service; production orchestration is implemented in a later phase.

Artifact readiness and completeness use
`simulation_interfaces/msg/ArtifactStatus`; its final topic or service binding
is deferred to Phase 2.

## Timing

Simulation state uses `/clock`. Wall-clock deadlines are restricted to startup, stalled-host detection, finalization, and forced shutdown.

## Failure behavior

Startup fails closed. Partial outputs are preserved. A manifest is produced for completed, failed, and aborted runs.
