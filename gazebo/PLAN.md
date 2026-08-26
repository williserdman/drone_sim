# Gazebo Plan

## Responsibility

Own authoritative physics, dynamics, sensors, camera generation, collisions,
vehicle pose, ground truth, native state, and simulation time.

## Non-responsibilities

- Mission decisions or flight-control policy
- Scenario activation policy or score calculation
- ArduPilot SITL, MAVLink, actuator exchange, motor dynamics, or lockstep
- Companion mission and vision integration
- Electromagnet forces, payload attachment, or competition course policy

## Inputs and outputs

Phase 3 consumes only the resolved run configuration, lifecycle/readiness
facts, and finalization request. It produces ROS 2 `/clock`, onboard and
observer camera frames plus metadata, aligned ground truth/contact state,
native state, server logs, quiescence, and diagnostics. Gazebo Transport is
private to this module.

## Implementation stages

1. Validate immutable world/model resources and provenance.
2. Start one fresh headless Gazebo Harmonic server paused with the resolved seed.
3. Discover private control, clock, camera, pose, contact, and native-recording endpoints.
4. Start private bridges and the public ROS adapter without releasing a sample.
5. At `READY`, unpause private Gazebo-ArduPilot warmup; at `RUNNING`, arm the
   configured fixed native public epoch (default `90.0` s) before its exact
   target, without another unpause or reset. Fail closed on late activation or
   a skipped target; publish zero before draining the bounded pre-zero queue.
6. Publish two 20-sim-Hz camera/metadata streams and timestamp-aligned ground truth.
7. Pause at the configured duration, freeze native evidence, and report source completion.
8. Finalize within the shared bounded deadline and publish only Gazebo's quiescence marker.

## Acceptance criteria

- Physical truth has one owner.
- Both camera streams are spaced 0.05 simulated seconds apart.
- A paused world emits no advancing public simulation samples.
- Resets isolate runs by destroying the server and using a new Compose project and partition.
- The production runtime does not consume `/simulation/camera_pair_ack`.
- No Phase 3 acceptance statement claims ArduPilot lockstep.

## Later phases

Phase 4 owns the ArduPilot adapter and lockstep design. Phase 6 owns the
electromagnet physical-effect request contract and competition behavior.
