# Gazebo External Interface

## ArduPilot adapter seam

- Input: actuator outputs from ArduPilot SITL
- Output: simulated sensors and dynamics
- Ordering: lockstep exchange controls physics advancement

## ROS 2 outputs

- Authoritative `/clock`
- Onboard and observer camera frames at 20 frames per simulated second
- Ground truth for the scorekeeper
- Contact, collision, and diagnostic state as required

All run-scoped outputs carry `run_id`; each camera message also carries a stream-specific `frame_id` and its simulation capture timestamp. The onboard stream is identical to the imagery supplied to companion vision.

## ROS 2 inputs

The electromagnet module submits idempotent physical-effect requests containing run identity, event identity, target magnet, desired state, and simulation timestamp. Gazebo validates and realizes them through physics.

## Reset, timing, and failure behavior

Reset clears run-scoped world state before accepting the new `run_id`. Stale-run requests are rejected or ignored with diagnostics. When paused, `/clock` does not advance and simulated events do not occur. Loss of the ArduPilot lockstep peer prevents uncontrolled physics progress.

## Deferred decisions

- Exact topics, schemas, QoS values, and reset endpoint
- World/plugin selection and adapter version
