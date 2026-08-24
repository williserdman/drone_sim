# Gazebo External Interface

## ArduPilot adapter seam

- Input: actuator outputs from ArduPilot SITL
- Output: simulated sensors and dynamics
- Ordering: lockstep exchange controls physics advancement

## ROS 2 outputs

- Authoritative `/clock` using best-effort QoS depth 1
- Onboard `/camera/onboard/image_raw` and observer
  `/camera/observer/image_raw` frames at 20 frames per simulated second, each
  offered with reliable QoS depth 5. Best-effort mission consumers remain
  compatible, while the archival recorder requests reliable delivery.
- `/simulation/ground_truth` using
  `simulation_interfaces/msg/GroundTruth` and best-effort QoS depth 10
- Contact, collision, and diagnostic state as required

All run-scoped outputs carry `run_id`; camera images correlate with
`simulation_interfaces/msg/FrameMetadata`, which carries stream-specific
`frame_id` and simulation capture timestamp. The onboard stream is identical
to the imagery supplied to companion vision.

## ROS 2 inputs

The electromagnet module submits idempotent physical-effect requests containing run identity, event identity, target magnet, desired state, and simulation timestamp. Gazebo validates and realizes them through physics.

The Phase 2 synthetic source additionally consumes the transport-only
`/simulation/camera_pair_ack` contract documented by artifacts. It ignores
stale run IDs and exact duplicates, rejects wrong streams, timestamps, gaps,
and future acknowledgements, and lets `FINALIZING` preempt an outstanding wait.
This synthetic backpressure does not alter simulation timestamps or join the
fixed rosbag inventory; it is not a production Gazebo physics interface.
Before initial clock/frame output, the Phase 2 source also requires two matched
subscriptions (artifact video and rosbag) on each of its four camera
publishers. Each frame's six fixed publications are drained one per ROS
executor turn in clock, onboard image/metadata, observer image/metadata, and
ground-truth order. The queue is bounded to six and `FINALIZING` clears it
immediately; neither discovery polling nor queue draining supplies simulation
timestamps or modeled latency.

For Phase 2 synthetic finalization, Gazebo stops all publishers and stdout,
writes its fixture files, then atomically writes only
`.status/quiescence/gazebo.json={run_id,module:"gazebo",quiescent:true}`. It
does not write the public aggregate freeze; orchestration publishes
`runtime-frozen.json` only after all six module markers exist.

## Reset, timing, and failure behavior

Reset clears run-scoped world state before accepting the new `run_id`. Stale-run requests are rejected or ignored with diagnostics. When paused, `/clock` does not advance and simulated events do not occur. Loss of the ArduPilot lockstep peer prevents uncontrolled physics progress.

## Deferred decisions

- Physical-effect request and reset endpoint contracts
- World/plugin selection and adapter version
