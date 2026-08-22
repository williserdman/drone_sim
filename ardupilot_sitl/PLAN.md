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

## Deferred decisions

- Vehicle firmware and configuration
- Adapter version and configuration
- MAVLink ports and routing
