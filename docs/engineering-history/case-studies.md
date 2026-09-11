# Engineering case studies

These case studies pull the strongest technical threads out of the chronological ledger. They are meant for design reviews and interviews, where the useful question is usually not "what feature shipped?" but "what constraint forced the design, and how did the evidence change it?"

The links return to the exact ledger entries. Commit hashes and paths are public repository evidence, not transcript references.

## Publishing validated artifacts without race windows

**Constraint.** A run bundle could contain video, logs, native simulator state, and a rosbag. Validation had to describe the same bytes that became public, concurrent finalizers could not overwrite one another, and finalization still had to finish after simulation time stopped. There was also a logical cycle to avoid: a bag could not prove a terminal state that depended on validating that same bag. See [EH-0009](chronology.md#eh-0009-designed-phase-2-around-a-non-cyclic-artifact-finalization-protocol), [EH-0011](chronology.md#eh-0011-made-run-bundle-finalization-deterministic-and-race-resistant), [EH-0013](chronology.md#eh-0013-implemented-dual-simulation-time-video-then-replaced-pathname-publication-with-an-anonymous-inode-design), and [EH-0014](chronology.md#eh-0014-closed-the-video-publication-and-recovery-safety-review).

**Core insight.** A pathname is a weak identity under concurrency. Another process can replace a file or an intermediate directory between validation and publication. The design therefore treated publication as a change in capabilities: write an anonymous inode, close every writable handle, validate through a retained read-only descriptor, then create one no-clobber link. For the bundle as a whole, `manifest.json` became the sole terminal commit after producers reached `FINALIZING` and their evidence had closed.

**Solution.** The artifact code traverses from retained directory descriptors with no-follow opens, rejects hard links and path escapes, hashes canonical trees, and publishes by atomic no-clobber hard link. Losing finalizers compare exact manifest bytes to distinguish an idempotent retry from a conflict. Video follows the same rule with `O_TMPFILE` and descriptor-bound linking. One absolute monotonic wall deadline spans drain, validation, publication, recovery, and teardown.

**Verification.** Commits `37c425e` and `5c2a70c` added race tests for symlink swaps, concurrent writers, and a raw non-UTF-8 filename. Commits `4b397d6`, `c7a1fac`, and `637b18a` covered post-validation mutation, bounded recovery, and sealed FFmpeg logs. The main implementation lives in `artifacts/src/artifacts/manifest.py`, `artifacts/src/artifacts/session.py`, and `artifacts/src/artifacts/_adapters/video.py`.

**Tradeoff.** POSIX cannot publish every file in a multi-file bundle atomically. The terminal manifest supplies bundle-wide authority, so consumers must treat uncommitted files as candidates rather than accepted evidence.

**Interview framing.** This is a filesystem concurrency story about TOCTOU races, capability reduction, idempotency, and commit protocols. The useful design move was replacing "validate, then rename a path" with "retain the object identity until publication."

## Keeping simulation time exact and the public epoch stable

**Constraint.** Physics ran on a 1 ms native clock, evidence used a 50 ms grid, and a slow flight controller sometimes needed many wall minutes of private warmup. Public scoring still had to start at a reproducible zero and end at an exact duration. Binary floating-point configuration and asynchronous callbacks both threatened that contract. See [EH-0030](chronology.md#eh-0030-add-exact-runtime-profiles-and-duration-arithmetic), [EH-0060](chronology.md#eh-0060-rebase-public-evidence-after-private-sitl-warmup), [EH-0068](chronology.md#eh-0068-diagnosed-nondeterministic-flight-timing-and-fixed-the-public-epoch), [EH-0069](chronology.md#eh-0069-required-confirmed-pause-before-exact-run-to-and-defined-command-evidence), and [EH-0102](chronology.md#eh-0102-public-epoch-handshake).

**Core insight.** Simulation time was protocol data, not a display value. The configuration parser reads decimal strings and converts them to integer nanoseconds. Exact grid checks happen on integers. Warmup time stays private, while an epoch mapping rebases native timestamps into the public interval.

**Solution.** `PublicEpoch` and `OutputEpochGate` floor activation to the 50 ms grid, discard pre-epoch samples, and stop mapping at the configured horizon. A post-request barrier waits for camera watermarks and a sufficiently recent clock before taking the epoch snapshot. Later repeatability work chose an exact 90-second native rendezvous, required authoritative `paused=true` before running to it, persisted the initial command handoff, and only then released the world. The startup handshake eventually accepted the first observed public stamp in the closed 0 to 50 ms window because callbacks can advance before a polling loop sees zero.

**Verification.** Commits `dfd7b53` and `59b0aab` covered decimal parsing and integer duration arithmetic. Commits `1ce2c93`, `7feb0c5`, and `070cc20` covered rebasing, callback barriers, exact rendezvous, and confirmed pause order. Commit `fe44a96` added the skipped-zero regression. Relevant code is in `orchestration/src/orchestration/config.py`, `gazebo/src/drone_sim_gazebo/ros_adapter/epoch.py`, and `gazebo/src/drone_sim_gazebo/runtime/runtime_node.py`.

**Tradeoff.** An exact public epoch removes warmup-history variance, but it does not make later wall-scheduled commands land on identical physics iterations. Command scheduling remains a separate source of nondeterminism.

**Interview framing.** This is a representation and distributed-ordering problem. The key points are integer time, explicit clock domains, watermark barriers, and the difference between an acknowledgement that a command was queued and proof that the simulator applied it.

## Joining asynchronous sensor streams with bounded memory

**Constraint.** Cameras, metadata, vehicle truth, payload pose, contact, and joint facts arrived through independent callbacks. They described the same 20 Hz physical ticks but did not arrive in the same order. A latest-value cache fabricated mixed-time state, while an unbounded queue would hide overload and make memory use unpredictable. See [EH-0033](chronology.md#eh-0033-make-camera-and-truth-alignment-bounded-and-fail-closed), [EH-0051](chronology.md#eh-0051-buffer-one-callback-epoch-without-hiding-loss), [EH-0074](chronology.md#eh-0074-payload-state-exact-joins), [EH-0087](chronology.md#eh-0087-exact-recent-state-under-load), [EH-0090](chronology.md#eh-0090-bounded-lookahead-through-bursts), and [EH-0109](chronology.md#eh-0109-payload-timestamp-backlog).

**Core insight.** Freshness is a relation among timestamps, not a property of whichever callback ran last. The join state therefore keeps timestamped, monotonic histories and selects the latest exact common tick. It never interpolates or silently drops a sample to make the data look aligned.

**Solution.** The first camera model kept one unmatched sample per stream and one aligned pair awaiting truth. Measured callback lead then justified a current-plus-one FIFO, 500 ms per-source histories, and finally lookahead matched to subscription depth with a one-second hard cap. Payload state joins pose, contact, and joint truth only at an exact timestamp. Overflow, regression, duplication, missing ticks, and nonfinite values latch a stable failure rather than allowing a later sample to repair the record. A reproduced 2.4-second pause later justified raising the joint-state transport queue from 10 to 1,000 without weakening strict gap detection.

**Verification.** Commits `e3c5ae5`, `4ebc383`, `36efe0c`, and `446b41a` established the fail-closed join and one-epoch lookahead. Commits `bb8b06b`, `1b2d94b`, `4edcdf8`, `a43bae4`, and `fbf4192` covered exact recent-state joins, finite burst bounds, and the reproduced joint backlog. The implementations sit in `gazebo/src/drone_sim_gazebo/ros_adapter/model.py`, `gazebo/src/drone_sim_gazebo/ros_adapter/aggregation.py`, `gazebo/src/drone_sim_gazebo/ros_adapter/payload.py`, and `electromagnet/src/drone_sim_electromagnet/controller.py`.

**Tradeoff.** Larger bounded buffers preserve known bursts but do not reduce callback load. The joint-state fix produced a physically successful 150-point mission, yet a later range timestamp still failed artifact acceptance. Each strict stream needed its own backlog audit.

**Interview framing.** This is the clearest data-structure story in the project. Explain why a bounded FIFO or timestamp-indexed history carries more truth than a latest-value variable, how measurements set the bound, and why overflow should fail visibly in an evidence system.

## Preserving lockstep actuator order under host contention

**Constraint.** Gazebo and the flight controller exchanged sequential UDP actuator frames in lockstep. On a heavily contended host, the simulator's queue-drain optimization kept only the newest datagram. A burst `[F+1, F+2, F+3]` became `F+3`, which skipped control inputs and broke the one-step exchange. Paused startup could also consume the first running frame too early. See [EH-0054](chronology.md#eh-0054-preserve-the-first-running-lockstep-exchange), [EH-0061](chronology.md#eh-0061-preserve-sequential-actuator-frames-under-host-contention), and [EH-0062](chronology.md#eh-0062-bound-duplicate-recovery-feedback).

**Core insight.** "Newest wins" is valid for some telemetry, but not for an ordered control protocol. Queue semantics must match the meaning of the data. Here, every accepted motor update belonged to one controlled physics step.

**Solution.** The paused plugin consumes only bootstrap frame 0 and leaves frame 1 for the first running step. During execution, each receive consumes one datagram instead of draining to the newest. Existing sequence logic processes duplicates and stops on the first sequential or accepted-gap frame. Removing the drain exposed a feedback loop where each duplicate caused a recovery reply and therefore another generated frame, so the patch added one recovery flag per contiguous duplicate burst.

**Verification.** Commit `fba46fc` added the paused-boundary regression and reached 433 passing tests with two skips. Commits `5324c09`, `329b040`, and `cdbdc0d` added real-plugin cases for sequential consumption and bounded duplicate recovery. The public patch and tests are `gazebo/plugin/0001-paused-initial-json.patch` and `gazebo/tests/test_plugin_udp.py`.

**Tradeoff.** Removing queue drain preserves causality but can expose backlog rather than masking it. At the recorded cutoff, the bounded recovery change passed the host suite, while rebuilt-image verification remained pending.

**Interview framing.** This is a compact protocol-design example. Start with the counterexample to "always keep the latest packet," then show how sequence numbers, a one-datagram receive rule, and burst-scoped recovery preserve progress without positive feedback.

## Separating transport, mission, and evidence readiness

**Constraint.** A TCP socket could exist before the vehicle was initialized. A heartbeat proved liveness but not safe arming. Artifact readiness could arrive before Gazebo had produced current-run exchange evidence. Treating all of those facts as one `ready` boolean either deadlocked paused startup or released the mission too early. See [EH-0048](chronology.md#eh-0048-gate-phase-3-readiness-on-current-gazebo-evidence), [EH-0053](chronology.md#eh-0053-separate-transport-readiness-from-mission-readiness), [EH-0058](chronology.md#eh-0058-add-a-private-warmup-and-durable-mission-ready-gate), [EH-0059](chronology.md#eh-0059-publish-passive-prearm-status-during-warmup), and [EH-0110](chronology.md#eh-0110-mission-sensor-lifecycle).

**Core insight.** Readiness was a state machine assembled from facts owned by different services. Transport readiness, vehicle health, current-run simulator evidence, recording readiness, and public output activation needed separate names and deadlines.

**Solution.** Startup declares the companion transport-ready after verified MAVLink TCP discovery. A later gate independently latches heartbeat and healthy prearm state, then durably publishes `mission-ready`. Orchestration publishes Phase 3 `READY` only after both artifact readiness and valid current-run Gazebo exchange. `RUNNING` activates public output and commands. Passive 1 Hz extended status permits the health gate to complete during private warmup without requesting public mission streams. Sensor polling remains active until the complete gate succeeds and stops only after a running attempt has ended.

**Verification.** Commits `1640584` and `c3ded50` checked exact `READY` and `RUNNING` order. Commits `3126a3f`, `c2da292`, `d2ef9cc`, `d76d486`, `92cfc37`, `d3f7c21`, and `3d1a631` covered the transport/mission split and passive prearm path. The state spans `orchestration/src/orchestration/runtime_node.py`, `orchestration/src/orchestration/status_store.py`, `companion/src/drone_sim_companion/lifecycle.py`, and `artifacts/src/artifacts/runtime_protocol.py`.

**Tradeoff.** More explicit states mean more transition cases. Later regressions showed why STARTING, RUNNING, and terminal sensor lifecycles each need tests. A "command delivered" flag cannot stand in for the full readiness predicate.

**Interview framing.** Use this to discuss distributed state machines and ownership. The central lesson is that readiness is not one fact, and each fact needs an authority, persistence rule, deadline, and consumer.

## Serializing payload commands against timestamp-coherent state

**Constraint.** Attach and release commands depended on run identity, mission phase, vehicle zone, grounded state, hardpoint capacity, payload position, and physical confirmation. Those facts arrived concurrently. Two service requests could pass validation against the same state, or callbacks from different ticks could form a state that never existed. See [EH-0074](chronology.md#eh-0074-payload-state-exact-joins), [EH-0075](chronology.md#eh-0075-serialized-payload-authority), and [EH-0101](chronology.md#eh-0101-corrected-atomic-payload-release-scoring).

**Core insight.** Authorization and actuation formed one transaction. The controller had to evaluate a coherent physical snapshot and reserve command authority before another request could do the same. The release tick also had two valid views, attached immediately before the transition and detached at the event timestamp, so scoring needed an explicit side of that boundary.

**Solution.** The adapter publishes recurrent 20 Hz pose, contact, and joint truth joined at exact simulation timestamps. The electromagnet service serializes operations with a dedicated mutex while state and result callbacks use a separate lock. It rejects mixed ticks, stale or regressing facts, multiple simultaneous attachments, malformed confirmations, and mismatched correlations. Scoring evaluates the final attached sample strictly before release, then evaluates post-release behavior from the detached state.

**Verification.** Commits `a3d8a21`, `4bee1ae`, and `99b5253` established recurrent exact joins. Commits `3abd8dd` and `468b7fa` covered concurrent service operations and runtime image behavior. Commit `2610e96` fixed the inclusive-boundary scoring defect; the rerun scored 150/150. See `gazebo/src/drone_sim_gazebo/ros_adapter/payload.py`, `electromagnet/src/drone_sim_electromagnet/runtime_node.py`, and `scorekeeper/src/drone_sim_scorekeeper/competition.py`.

**Tradeoff.** Serialization reduces command concurrency by design. Five-second confirmation initially had no retry protocol, and later work added only one narrow stale-state retry after simulation time advanced.

**Interview framing.** This case connects concurrency control to temporal data modeling. It is a useful answer when asked about mutex scope, coherent snapshots, idempotency, or how to define an atomic event in a sampled physical system.

## Scoring physical truth independently of mission claims

**Constraint.** Mission events could say that a pickup, release, or landing occurred, but those events came from the system being judged. Acceptance also had to distinguish three outcomes that are easy to conflate: the aircraft completed the maneuver, the scoring rules awarded points, and the evidence bundle proved both. See [EH-0076](chronology.md#eh-0076-independent-physical-scoring), [EH-0078](chronology.md#eh-0078-canonical-competition-evidence), [EH-0096](chronology.md#eh-0096-accepted-the-callback-corrected-competition-result), and [EH-0098](chronology.md#eh-0098-made-video-the-canonical-pixel-evidence).

**Core insight.** Events are useful phase markers, not proof of physical success. The scorer and acceptance path therefore derive results from timestamped vehicle and payload truth, then compare those results with the claimed lifecycle. Artifact provenance and source identities are checked separately from scoring.

**Solution.** Competition scoring requires continuous 50 ms traces, exact pickup zones, three-dimensional continuity, strict release freshness, persistent Home validity through disarm, and terminal validity independent of optional points. A second physical oracle in the artifact layer rechecks those rules without accepting an injected scorekeeper result. Validation binds source revisions, image digests, configuration, camera geometry, QoS, and run timing. Raw RGB frames were later removed from MCAP because MP4 already preserved the pixels; MCAP retained frame metadata and physical facts needed to join video frames to the scored timeline.

**Verification.** Commits `7636c4b`, `0253ab7`, and `cae07bd` hardened the score rules. Commits `22760bf`, `208072a`, and `bc1a15b` removed trust and injection gaps from acceptance. Commits `dc6362c` and `198c251` documented an accepted 150/150 mission with a complete bundle. Commit `1fbabaa` established the compact evidence split. Key paths are `scorekeeper/src/drone_sim_scorekeeper/competition.py`, `artifacts/src/artifacts/competition_score_validation.py`, and `artifacts/src/artifacts/acceptance.py`.

**Tradeoff.** Independent logic can drift unless shared contracts and adversarial fixtures constrain it. Removing raw images from MCAP also makes the video-to-metadata join part of evidence validation instead of keeping two copies of every pixel.

**Interview framing.** This is a trust-boundary story. Explain why the evaluator should not trust success events or the primary scorer, then describe how independent derivation, provenance, and canonical representations make a result auditable.

## Moving blocking control work off callback paths

**Constraint.** A synchronous Gazebo `WorldControl` request blocked the ROS executor during public release. Camera and ground-truth callbacks kept arriving, filled bounded lookahead, and failed the run. Large image copies later starved clock, range, control, and metadata work even without an explicit blocking call. See [EH-0085](chronology.md#eh-0085-nonblocking-release-and-roll-control), [EH-0095](chronology.md#eh-0095-separated-companion-callback-lanes-and-latest-state-readiness), and [EH-0110](chronology.md#eh-0110-mission-sensor-lifecycle).

**Core insight.** Buffer growth was a symptom, not the fix. The hot callback path needed to keep draining while slow control work ran elsewhere. Callback classes also had different ordering requirements, so one global reentrant pool was unsafe.

**Solution.** WorldControl moved to a private worker. The companion then used five executor threads with separate ordered lanes for clock/control, range, image, and metadata. Clock and range switched together to latest-value depth 1, with one pending leading range sample, so neither side retained a long stale history. After a running attempt ended, sensor subscriptions and polling shut down in lifecycle order rather than continuing to consume CPU.

**Verification.** Commit `3a022d4` added a regression that proved callbacks progressed during release. The callback-lane sequence in commits `f2099ee`, `aeb24f3`, `b09c511`, `307ed29`, `191bd45`, `8162c94`, `476cc55`, and `502e25c` crossed the former stale-range boundary and completed both payload cycles. Commits `86776dd`, `5d8c593`, `f304ad9`, and `eb6c64c` corrected shutdown ordering; a later run showed companion CPU falling from about 82 percent to 7 percent after mission completion.

**Tradeoff.** Multiple lanes preserve local order but increase the number of concurrency assumptions. Tests must exercise callback interleavings and lifecycle boundaries, not only individual handler results.

**Interview framing.** This is an executor and backpressure story. The diagnostic pivot matters: once a synchronous call was shown to block callback progress, increasing queue capacity would only delay failure.

## Treating QoS and queue depth as correctness

**Constraint.** DDS delivery settings looked like transport tuning, but they changed what the recorded history could prove. Writer acknowledgement did not mean rosbag had persisted a sample. A volatile late subscriber could miss a one-shot event. A reliable publisher paired with a shallow or best-effort reader could still lose exact physical ticks under executor lag. See [EH-0045](chronology.md#eh-0045-preserve-both-artifact-startup-states), [EH-0047](chronology.md#eh-0047-align-physical-readiness-and-event-durability), [EH-0065](chronology.md#eh-0065-fix-physical-video-loss-and-powered-landing-rebound), [EH-0086](chronology.md#eh-0086-reliable-payload-and-loaded-control), and [EH-0109](chronology.md#eh-0109-payload-timestamp-backlog).

**Core insight.** Reliability, durability, and history depth answer different questions. A writer acknowledgement proves delivery to middleware, not application take or persistence. Transient-local durability is right for a one-shot fact that late readers must replay, while volatile delivery is right for events whose history must not be replayed. Queue depth must cover measured callback delay if every sample is part of the evidence contract.

**Solution.** The public artifact-status publisher stayed reliable, transient-local, depth 1, while the private recorder subscription used depth 2 to retain both `[false, true]`. Scenario events became transient-local; score events stayed reliable and volatile. Physical recorder depth increased from 5 to 100 after an A/B test reproduced metadata loss at 5. Payload pose switched from best effort to reliable. The later 2.4-second payload-joint backlog justified depth 1,000.

**Verification.** Commits `8a97a67` and `56422b0` reproduced a delayed reader that acknowledged both states but retained only `[true]` before the depth fix. Commits `3a69dc1` and `dc3a400` tested late one-shot replay. Commit `03a8cf8` paired MCAP evidence with a real DDS depth comparison. Commits `bbaf932` and `fbf4192` covered lost physical samples. Current contracts live in `artifacts/recording-qos.yaml`, `gazebo/config/bridge-competition.yaml`, and their adapter tests; the former root-level recording QoS copy is historical.

**Tradeoff.** Deeper reliable histories consume memory and can increase catch-up work. They should follow measured delay and topic semantics, not become a blanket setting. Strict consumers still reject gaps once the declared bound is exceeded.

**Interview framing.** Use this when discussing messaging systems. The concrete lesson is that QoS is part of the data model when correctness depends on observing a sequence, especially across late subscribers and overloaded readers.

## Profiling past aggregate CPU and reducing scheduler pressure

**Constraint.** The simulation ran below real time even when aggregate host CPU was not exhausted. Merely exposing a GPU did not move rendering off Mesa. After GPU selection worked, low GPU utilization and thousands of context switches per second showed that scheduling, callbacks, and container boundaries still dominated. See [EH-0103](chronology.md#eh-0103-real-time-profile-and-compact-artifacts), [EH-0105](chronology.md#eh-0105-gpu-runtime-and-egl-selection), [EH-0108](chronology.md#eh-0108-clock-video-and-log-load), [EH-0111](chronology.md#eh-0111-native-clock-and-frame-path), [EH-0112](chronology.md#eh-0112-shared-network-namespace), and [EH-0113](chronology.md#eh-0113-bounded-ogre-workers).

**Core insight.** Device availability, renderer selection, utilization, and end-to-end throughput are separate facts. Aggregate CPU can look idle while one lockstep chain suffers callback starvation and scheduler overhead. Performance work therefore needed provenance-backed runtime checks and before/after measurements at each layer.

**Solution.** The GPU overlay mounted an NVIDIA EGL vendor descriptor, forced GLVND selection, and required a surfaceless preflight to identify the renderer. The public 1 kHz clock path moved from Python to a C++ decimator that emits the exact 20 Hz evidence grid. Per-frame JSON logs and a redundant image/metadata join were removed. Simulation services later shared network and IPC namespaces. Finally, the project backported upstream Ogre worker control and chose four workers after a zero-worker trial removed too much useful parallelism.

**Verification.** Commits `a09186f`, `0ee7a2c`, `b7223a1`, `1f14ebd`, and `021fdef` removed llvmpipe and cut Gazebo CPU to about 67 percent in the first clean observation. Commits `c419d95` and `fd1a5d4` cut logs from about 18.8 MB to 25 KB and reduced artifact and scorekeeper CPU, but improved real-time factor only about 1.3 percent. Commits `7340b76`, `5e0c49d`, `f4c86a8`, `4217a94`, and `b45540e` addressed native decimation, namespaces, and bounded render workers. See `gazebo/plugin/ClockDecimator.cc`, `gazebo/config/10_nvidia.json`, `compose.gpu.yaml`, and `gazebo/patches/gz-rendering8-inline-workers.patch`.

**Tradeoff.** Each change removed real work, but no single change solved lockstep throughput. Sharing network and IPC namespaces improved real-time factor by about 3.4 percent, and tail profiling still measured about 6,546 context switches per second. The four-worker choice still needed a clean same-host comparison at the ledger cutoff.

**Interview framing.** This is a performance-method story. Distinguish configuration from proof, use measurements to reject an attractive zero-worker setting, and follow the bottleneck after each optimization instead of declaring victory when one component's CPU falls.
