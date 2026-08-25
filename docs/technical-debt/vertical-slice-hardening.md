# Vertical Slice Deferred Hardening

This ledger records work intentionally excluded from the critical path to a verified `descent_v1` maximum-score run. Promote an item only when concrete runtime evidence shows it blocks launch, correctness, deterministic timing, artifact integrity, or truthful scoring.

- Enumerate and reject unexpected extra native camera sensors beyond the fixed onboard/observer public pair.
- Extend adversarial filesystem substitution coverage beyond the descriptor and identity protections completed in Phase 3 Task 5.
- Investigate unreproduced, extremely narrow process/session identity races after real integration stabilizes.
- Expand malformed lifecycle/status/ROS/MAVLink fault-injection matrices.
- Define rendering-host reproducibility separately from semantic MP4 determinism; byte-identical encodes are not required now.
- Add active electromagnet physics, payload behavior, and the broader competition ruleset.
- Integrate legacy vision/FMs, precision landing, LiDAR, and dropper behavior without modifying the nested repository until explicitly authorized.
- Add multi-vehicle, GUI, long-duration ROS discovery/network partition, and distributed observability stress tests.
- Revisit ground-truth QoS only if a production run reproduces sample loss: the current BEST_EFFORT depth-10 path is low latency, but strict scoring correctly rejects any missing sample. If promoted, change the Gazebo publisher, scorekeeper subscription, and rosbag override together.
- Harden native truth aggregation against contact messages that arrive after a newer odometry sample only if runtime evidence reproduces the delayed cross-topic ordering fault.
- Classify a zero-budget Compose health probe at the startup deadline as the configured startup-deadline cause instead of leaking `subprocess.TimeoutExpired`; this currently obscures diagnostics but does not change the terminal result.
- Retry or otherwise harden transient pre-ready Gazebo flight-status service RPC failures only if another production run reproduces the unretained `Host unreachable` wrapper path seen once during startup.
- Add a truthful cross-container lockstep-stall detector that observes real Gazebo/JSON progress without interpreting ArduPilot's 1.1-second servo-retransmission warning as peer loss. Until then, authoritative Gazebo child/adapter failures remain immediate and a silent live stall is bounded by the run's host-wall deadline.
