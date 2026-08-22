# Artifacts Internal Interface

## Internal seams

The bundle builder consumes immutable artifact records and produces a manifest. Recorder adapters own ROS bag, image encoding, Docker-log capture, and filesystem validation. Tests replace adapters without changing manifest or lifecycle logic.

## Idempotency

Finalizing the same `run_id` repeatedly produces the same artifact inventory and never overwrites another run.

