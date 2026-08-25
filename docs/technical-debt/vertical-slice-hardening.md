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
- Retain the current reliable ground-truth and frame-clock QoS unless runtime evidence shows it causes a new blocker; strict scoring still rejects any missing aligned sample.
- Harden native truth aggregation against contact messages that arrive after a newer odometry sample only if runtime evidence reproduces the delayed cross-topic ordering fault.
- Classify a zero-budget Compose health probe at the startup deadline as the configured startup-deadline cause instead of leaking `subprocess.TimeoutExpired`; this currently obscures diagnostics but does not change the terminal result.
- Retry or otherwise harden transient pre-ready Gazebo flight-status service RPC failures only if another production run reproduces the unretained `Host unreachable` wrapper path seen once during startup.
- Add a truthful cross-container lockstep-stall detector that observes real Gazebo/JSON progress without interpreting ArduPilot's 1.1-second servo-retransmission warning as peer loss. Until then, authoritative Gazebo child/adapter failures remain immediate and a silent live stall is bounded by the run's host-wall deadline.
- **Promoted from deferred on run `305f7421-6bbc-4d5f-8a83-c5ba50b9338c`:** under severe host contention the upstream Gazebo plugin repeatedly drained multiple sequential actuator packets and retained only the newest, producing real frame gaps and corrupting one-for-one lockstep evidence. Process at most one sequential frame per physics update, preserve exact-duplicate resend recovery, and diagnose a true jump beyond `current + 1` before the next scored run.
- Remove the companion controller's legacy `ready` alias and consolidate its duplicated mission-readiness conjunction after the public interface and winning-run evidence are stable.
- Add a repository-owned Phase 3 slowdown-injection control and normalized
  two-run comparator. The vertical slice may use a reversible, run-scoped
  Docker resource constraint and independently compare the preserved bundles.
- Snapshot source revision/dirty state and active-container image IDs at launch
  rather than resolving mutable worktree/tag state during finalization.
- Externally pin the canonical world, vehicle, mission, scenario, seed, RTF,
  duration, and acceptance-validator image in the semantic inspection command.
  The current winning-run procedure verifies these separately without treating
  the missing abstraction as simulation evidence.
- Add the planned `test-phase3` integration target and Phase 3 verification
  document after the runnable maximum-score and slowdown bundles are preserved.
- Make the acceptance target's nested `make -n` test ignore GNU Make's inherited
  directory-tracing lines. It passes standalone but is non-hermetic when run
  beneath `make test-unit`; this does not affect the operator or acceptance
  command itself.
