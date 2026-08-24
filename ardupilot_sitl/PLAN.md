# ArduPilot SITL Plan

## Responsibility

Act as the simulated flight controller by running estimation, navigation, and vehicle-control loops in lockstep with Gazebo.

## Non-responsibilities

- Mission and vision decisions
- Authoritative physics, sensors, or ground truth
- Scenario logic and scoring

This is an ArduPilot SITL or simulated flight-controller module, not a Pixhawk simulator.

## Implementation constraints

- Prefer a maintained, readily available ArduPilot SITL image when it satisfies the required Gazebo adapter and architecture support; otherwise build a pinned image reproducibly in this directory.
- Support parameter-file injection at startup.
- Keep ArduPilot Dockerfiles, parameter files, entrypoints, and owned configuration under `ardupilot_sitl/`.
- Freeze `Copter-4.7.0` at
  `1511f27194f1dcc3728270883047bdf022b3fd53` and build only `waf copter`.
- Use the upstream JSON backend in lockstep against `gazebo-runtime:9002` and
  bind companion MAVLink on Compose-only TCP 5760.

## Inputs and outputs

It consumes companion MAVLink commands and Gazebo sensor data. It produces MAVLink telemetry and acknowledgements for the companion and actuator outputs for Gazebo.

## Implementation stages

1. Configure the SITL vehicle.
2. Establish the Gazebo adapter.
3. Establish companion MAVLink endpoints.
4. Prove lockstep progress.
5. Expose readiness and diagnostics.
6. Validate command acknowledgement and telemetry flow.

## Acceptance criteria

- Physics steps and flight-control updates remain in lockstep.
- Commands, telemetry, and acknowledgements flow bidirectionally.
- Loss of Gazebo prevents uncontrolled simulated progress.

## Vertical-slice technical debt

- The runtime recognizes the pinned release's startup diagnostics; broader
  cross-version output compatibility is intentionally deferred.
- Exhaustive malformed JSON/servo fault matrices and UDP packet capture are
  deferred. The bounded gate covers valid 16-channel exchange, malformed
  datagrams, and frame gaps.
- Adversarial replacement of owned diagnostic files is deferred to shared
  artifact hardening. Atomic status/failure publication and safe run scoping
  remain required.
- The official Gazebo plugin and flight-capable model are Gazebo-owned Wave 2
  work and are deliberately absent here.
