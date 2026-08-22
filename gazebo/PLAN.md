# Gazebo Plan

## Responsibility

Own authoritative physics, dynamics, sensors, camera generation, collisions, external forces, vehicle pose, ground truth, and simulation time.

## Non-responsibilities

- Mission decisions
- Flight-control policy
- Scenario activation policy
- Score calculation

## Inputs and outputs

Gazebo consumes ArduPilot actuator outputs and electromagnet physical-effect requests. It produces simulated sensor data, ROS 2 `/clock`, onboard and observer camera frames, ground truth, contacts, native state, server logs, and diagnostics.

## Implementation stages

1. Load the world and vehicle.
2. Establish the ArduPilot adapter.
3. Publish authoritative `/clock`.
4. Prove lockstep physics.
5. Configure onboard and observer cameras at 20 Hz in simulation time.
6. Publish ground truth.
7. Apply validated physical-effect requests.
8. Expose collision and health diagnostics.

## Acceptance criteria

- Physical truth has one owner.
- ArduPilot and Gazebo remain in lockstep at any real-time factor.
- Both camera streams are spaced 0.05 simulated seconds apart.
- Resets isolate runs by `run_id`.

## Deferred decisions

- Gazebo distribution, world, vehicle, and plugins
- ROS 2 topic names, schemas, and QoS
- ArduPilot adapter version
