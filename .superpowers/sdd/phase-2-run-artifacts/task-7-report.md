# Task 7 report — synthetic artifact runtime stack

## Outcome

Task 7 implements the seven controller-owned Phase 2 services, durable runtime
protocol, ROS orchestration runtime, aggregate rosbag/video runtime, deterministic
synthetic publishers, container images, and the `phase2` Compose profile. This is
synthetic infrastructure evidence only: it does not claim real Gazebo physics,
ArduPilot lockstep, mission behavior, or scoring validity.

## Frozen runtime contracts

The controller-owned services are, in order:

1. `orchestration-runtime` (`orchestration`)
2. `artifacts-runtime` (`artifacts`)
3. `synthetic-companion` (`companion`)
4. `synthetic-ardupilot-sitl` (`ardupilot_sitl`)
5. `synthetic-gazebo` (`gazebo`)
6. `synthetic-electromagnet` (`electromagnet`)
7. `synthetic-scorekeeper` (`scorekeeper`)

Runtime statuses are exact `artifacts-ready`, `runtime-running`,
`source-finished`, `runtime-failure`, `runtime-frozen`, `artifacts-final`, and
`terminal-notified` JSON documents. Host controls are exact `finalize-request`
and `terminal-committed` documents. All are current-run, bounded, descriptor-
relative, no-follow, regular/single-link checked, duplicate/nonfinite rejecting,
first-wins, file-fsynced, atomically replaced, and directory-fsynced.

The fixed bag inventory remains ten topics. `/simulation/camera_pair_ack` is an
internal reliable transient-local depth-1 `FrameMetadata` seam and is not bagged.
The archival camera image/metadata path is reliable depth 5 after repeated live
BEST_EFFORT loss proved incompatible with exact deterministic acceptance. A
bounded six-item publication queue, two-consumer endpoint barrier, 40-pair
recorder buffers, and current-run contiguous ACKs provide backpressure without
feeding wall time into simulation stamps or order.

## TDD and debugging evidence

Persisted REDs covered absent protocol/runtime modules; exact schemas and unsafe
files; partial readiness; callback reordering; stale/duplicate/gapped ACKs;
finalization preemption; publisher discovery; bounded publication order; and
exact archival QoS. Live diagnostics then isolated observer image loss while its
metadata arrived under BEST_EFFORT. The approved reliable archival override made
the exact delivery gate repeatable. A final readiness RED required current-run
STARTING observation before aggregate READY, producing deterministic four-sample
RunState bags. Final self-review also persisted a RED for duplicate ROS node
destruction and removed the redundant explicit video-node destroy call.

Actual image builds exposed and fixed test-stage REDs for missing workspace
metadata, unwanted `uv run` resync, unconditional absent-sibling imports, omitted
config fixtures, and unavailable git provenance. These fixes affect test stages
only except for the shared importlib preloader.

## Verification

- Focused protocol/aggregate/orchestration: `48 passed`.
- Phase 2 static container/runtime contract: `10 passed`.
- Synthetic pure contract: `14 passed`.
- Combined required host gate: `514 passed, 10 skipped`.
- Full repository host gate: `532 passed, 10 skipped`.
- Artifacts test container: `365 passed`.
- Orchestration test container: `143 passed`.
- Synthetic test container: `14 passed`.
- Isolated ROS 2 Jazzy harness: exit `0`.
- Plain and Phase 2 Compose configs: valid.
- Full `docker compose --profile phase2 build`: successful for foundation and
  all seven Phase 2 services.
- Compileall, `git diff --check`, and empty post-test Compose process list: clean.

Three isolated real ROS/Compose aggregate runs used wall delays `0`, `0`, and
`17` ms. All three produced identical normalized evidence: source stamp
`2_000_000_000`, 40 enqueued frames, 40 ACKs in both directions, seven live
services and seven log owners, valid 40-frame H.264/yuv420p/20-FPS videos, valid
MCAP, terminal notification, and exact topic counts/stamps. Video SHA-256 values
were stable across runs:

- onboard: `8f5eab5c28ebb345719c7bf4d31a90f7fb13cb3ae74f3a23135aeb1271967481`
- observer: `3e514d5b4bf655b6e5364af229c84c619af959e5092b4e67b3aea2eefa4b4bbf`

Final source-exact runtime image IDs:

- artifacts: `sha256:ed7a594ad13e05c184498a1f2ede71045a1c694213fa1936a72c4de077c855ea`
- orchestration: `sha256:02adf68812615c1548112574cfb3c3fce93431a57e37bbab66a0e8cd11ed1156`
- synthetic Gazebo: `sha256:7b1cef40bbbcf67b61088dabd32b843099c1e27dbe9e98de4e22c54ffe4d9959`
- synthetic companion: `sha256:363b9aa65ec5b1924fbabc84287bea537f6cc810579870a2b2001a2d6d429bdd`
- synthetic ArduPilot SITL: `sha256:68b45854b7910b352f3352dfbf4a2e196fd0992223872120788b7d0de2e21110`
- synthetic electromagnet: `sha256:6afda3b1d85337d9a320ece9d4bb14f076b7e70fc63eae6d1e61b28752788aa0`
- synthetic scorekeeper: `sha256:5c0038a07f35e9f866d981f75228a7ac289631c80b425a2241d0a4d182b6d0f7`

Final test image IDs are artifacts `sha256:0209e02af1bbd1e65101e79fcaec852a2305e97c6bfcc5571741c991adea39df`,
orchestration `sha256:ff0ada7027a909ea412dcb7e4efaa3579bcfc825a3732bdef2479457e7313ea9`,
and synthetic Phase 2 `sha256:eb3590c38ddfbd0976b3393d3e50e2381e6b08c66b4dab67897dab07b616c27c`.

## Self-review and Task 8 concerns

The runtime protocol remains policy-free below lifecycle modules; artifacts does
not import orchestration; recorder/bag/video implementations from Tasks 3–4 are
reused; the observer fault remains artifacts-owned; simulation timestamps never
derive from wall time; and terminal/quiescence boundaries remain silent.

Task 8 still must exercise the public foreground CLI across COMPLETED, FAILED,
and ABORTED outcomes, capture/partition Docker logs, create and independently
validate the final manifest/bundle, prove fault-path preservation, and keep all
synthetic fixture/non-validity labels. No Task 8 terminal CLI gate or real
Gazebo/ArduPilot/maximum-score claim was made here.
