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
loss under the earlier lossy archival offer proved incompatible with exact
deterministic acceptance. A
bounded six-item publication queue, two-consumer endpoint barrier, 40-pair
recorder buffers, and current-run contiguous ACKs provide backpressure without
feeding wall time into simulation stamps or order.

## TDD and debugging evidence

Persisted REDs covered absent protocol/runtime modules; exact schemas and unsafe
files; partial readiness; callback reordering; stale/duplicate/gapped ACKs;
finalization preemption; publisher discovery; bounded publication order; and
exact archival QoS. Live diagnostics then isolated observer image loss while its
metadata arrived under the earlier lossy offer. The approved reliable archival override made
the exact delivery gate repeatable. A final readiness RED required current-run
STARTING observation before aggregate READY, producing deterministic four-sample
RunState bags. Final self-review also persisted a RED for duplicate ROS node
destruction and removed the redundant explicit video-node destroy call.

Actual image builds exposed and fixed test-stage REDs for missing workspace
metadata, unwanted `uv run` resync, unconditional absent-sibling imports, omitted
config fixtures, and unavailable git provenance. These fixes affect test stages
only except for the shared importlib preloader.

## Review fix round 1/5

Independent review found two critical, three important, and one minor boundary
gap. TDD reproductions covered project-name-derived image lookup, the foundation
profile leak, missing aggregate quiescence ownership, recorder death/stubborn
rosbag behavior, restrictive-umask readability, malformed diagnostic path
types, and stale archival terminology.

The corrected runtime uses explicit stable tags for all seven Phase 2 images
and a separate stable-tagged `foundation` profile. Six non-artifact processes
write exact descriptor-safe `.status/quiescence/<module>.json` markers only
after stopping publishers/stdout; orchestration is the sole aggregate-freeze
writer. Artifacts trusts only that aggregate. Recorder process health is
monitored after readiness, durable first-wins failures precede structured
diagnostics, and an unconfirmed rosbag produces no final report or mutable-bag
hash before bounded teardown. Protocol files/fixtures are explicitly `0644`,
owned directories `0755`, and diagnostic path type errors are typed
`ProtocolError`s.

A three-run live gate exposed one additional startup race: rosbag saw two to
four lifecycle samples because `STARTING` could precede endpoint discovery.
The pre-STARTING infrastructure barrier now requires the exact six runtime
consumers plus rosbag, exact message type, and reliable transient-local QoS.
Focused barrier RED/GREEN ended at `29 passed`.

## Verification

- Focused protocol/aggregate/orchestration: `48 passed`.
- Review-round protocol/runtime/synthetic/controller gate: `202 passed, 4 skipped`.
- Phase 2 static container/runtime contract: `10 passed`.
- Synthetic pure contract: `14 passed`.
- Combined required host gate: `514 passed, 10 skipped`.
- Full repository host gate: `562 passed, 10 skipped`.
- Artifacts test container: `386 passed`.
- Orchestration test container: `149 passed`.
- Synthetic test container: `15 passed`.
- Isolated ROS 2 Jazzy harness: exit `0`.
- Plain and Phase 2 Compose configs: valid.
- Full `docker compose --profile phase2 build`: successful for the exact seven
  stable-tagged Phase 2 services. A separate foundation build and exact
  `docker compose run --rm foundation` integration also passed.
- Two unique project names each started exactly seven services using
  `up --detach --no-build` and resolved the same seven stable image tags.
- Compileall, `git diff --check`, and empty task-scoped post-test Compose process
  list: clean (three unrelated long-running `automation` services were left
  untouched).

Three fresh isolated real ROS/Compose aggregate runs used wall delays `0`, `0`,
and `20` ms. The third also delayed the scorekeeper quiescence marker by `800`
ms; with five markers present, neither aggregate freeze nor artifact final
report existed. All three then produced identical normalized evidence (semantic
SHA-256 `ba61184520279477ec33c879f64cc79fc417efbb0753221c0bcf26c4956a411c`): source stamp
`2_000_000_000`, 40 enqueued frames, 40 ACKs in both directions, seven live
services and seven log owners, valid 40-frame H.264/yuv420p/20-FPS videos, valid
MCAP, terminal notification, and exact topic counts/stamps. Video SHA-256 values
were stable across runs:

- onboard: `8f5eab5c28ebb345719c7bf4d31a90f7fb13cb3ae74f3a23135aeb1271967481`
- observer: `3e514d5b4bf655b6e5364af229c84c619af959e5092b4e67b3aea2eefa4b4bbf`

Final source-exact stable-tag runtime image IDs:

- artifacts: `sha256:5e3db72082fdecf8fded49fcf5edf5f46949abca08909d3e509baa4d566e064f`
- orchestration: `sha256:84a243cf11241e5a63b1d45c032a8d7103940e654a438ce86193672ff3a11f39`
- synthetic Gazebo: `sha256:6c7df7665aecb92fa939a69b66b543c744a3f44108aefd2294ee7f54bad39dfa`
- synthetic companion: `sha256:04e621ef8b54bbafa92f770113ad90898472d3212a97a6f3109610c4ea40d52b`
- synthetic ArduPilot SITL: `sha256:3e9bc89c4d4ab6821592ed47fe70d6999ceee8599380d66d99c1acf955620634`
- synthetic electromagnet: `sha256:a5dbc3c8b7f479825cb7f1e4fae8f17afec9315be129182a4037a30e3dabff2a`
- synthetic scorekeeper: `sha256:c95da4bdb9c333c1ced7b250e60381011beac59e5f6ef3106a94b9826bc0162b`

Final review-round test image IDs are artifacts
`sha256:5d22111881d05fd07bdbbb5a343b463b8d7f4cf0e6ac5eb25a3c98a7ef731844`,
orchestration
`sha256:94b205a2878c5c331ebc936401b068ba0bc2da9d6f9a3140e1aed4667f14c614`,
and synthetic Phase 2
`sha256:1a9449e8d53c6939bf25bc22741e6612f3907601f6e4d0a5c06422feb71fb138`.

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
