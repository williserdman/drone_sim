# Gazebo External Interface

## ArduPilot adapter seam

- Input: actuator outputs from ArduPilot SITL
- Output: simulated sensors and dynamics
- Ordering: lockstep exchange controls physics advancement

## ROS 2 outputs

- Authoritative `/clock` using best-effort QoS depth 1
- Onboard `/camera/onboard/image_raw` and observer
  `/camera/observer/image_raw` frames at 20 frames per simulated second, each
  using best-effort QoS depth 5
- `/simulation/ground_truth` using
  `simulation_interfaces/msg/GroundTruth` and best-effort QoS depth 10
- Contact, collision, and diagnostic state as required

All run-scoped outputs carry `run_id`; camera images correlate with
`simulation_interfaces/msg/FrameMetadata`, which carries stream-specific
`frame_id` and simulation capture timestamp. The onboard stream is identical
to the imagery supplied to companion vision.

## ROS 2 inputs

The electromagnet module submits idempotent physical-effect requests containing run identity, event identity, target magnet, desired state, and simulation timestamp. Gazebo validates and realizes them through physics.

## Reset, timing, and failure behavior

Reset clears run-scoped world state before accepting the new `run_id`. Stale-run requests are rejected or ignored with diagnostics. When paused, `/clock` does not advance and simulated events do not occur. Loss of the ArduPilot lockstep peer prevents uncontrolled physics progress.

## Deferred decisions

- Physical-effect request and reset endpoint contracts
- World/plugin selection and adapter version
