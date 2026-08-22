# Orchestration External Interface

## Operator operations

`start`, `status`, `abort`, and `collect-results` are the initial conceptual operations. Their CLI names remain stable once Phase 1 implements them.

## Module lifecycle

Modules receive `run_id`, configuration, output paths, and lifecycle state. Required endpoints and artifact recorders report readiness before `RUNNING`. Every terminal cause enters `FINALIZING`, after which the artifacts module reports completeness.

## Timing

Simulation state uses `/clock`. Wall-clock deadlines are restricted to startup, stalled-host detection, finalization, and forced shutdown.

## Failure behavior

Startup fails closed. Partial outputs are preserved. A manifest is produced for completed, failed, and aborted runs.

