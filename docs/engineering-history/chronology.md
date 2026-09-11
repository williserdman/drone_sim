# Engineering chronology

This ledger covers engineering work owned by `drone_sim` from 2026-08-22 through 2026-09-05. Work owned by other repositories is excluded unless it directly explains a decision or accepted result here. Current behavior remains defined by [architecture](../architecture.md), the [runbook](../runbook.md), module READMEs, code, configuration, and tests.

Evidence states use four terms. **Verified** means a commit, test, inspection, or recorded run supports the claim. **Inferred** marks a useful conclusion without direct public proof. **Disputed** preserves unresolved conflicting evidence. **Superseded** marks an attempt or interpretation replaced by later evidence. Commit citations are stable public evidence. A path in code text may be historical and is not a claim that the file still exists.

## 2026-08-22

### EH-0001 Defined the simulation ownership and communication model

- Evidence state: Verified. Committed design.
- Problem or constraint: The repository began as a clean slate and needed module boundaries that would keep physics, flight control, mission logic, scenario effects, and scoring from becoming coupled again. Simulation time also had to remain authoritative when the host ran slower than real time.
- Investigation: The design compared direct imports, generic network contracts, and ROS 2. Direct imports did not preserve container boundaries, while a separate custom transport would duplicate clock, image, and QoS facilities already available in ROS 2.
- Decision or result: Gazebo owns physical truth, ArduPilot owns flight control, the companion commands through MAVLink, scenario controllers affect the Gazebo world, and the scorekeeper observes ground truth without commanding the aircraft. ROS 2 Jazzy on Ubuntu 24.04 became the shared transport and clock base. Every module documents local and external interfaces separately.
- Verification: The design and planning documents were reviewed for ownership, communication, timing, and failure-mode consistency before implementation began.
- Later correction or debt: Message schemas and runtime code were intentionally deferred until the module responsibilities stabilized.
- Public evidence: commit `fb90a43865df4858cd3b0674933eb566bf29b909` (root commit), followed by `8709529e4bc98797478eb91602587498d1597dbb` (parent `fb90a43865df4858cd3b0674933eb566bf29b909`). See `docs/superpowers/specs/2026-08-22-module-interfaces-design.md` and `docs/superpowers/plans/2026-08-22-module-documentation-scaffold.md`.

### EH-0002 Created the recursive module-contract scaffold

- Evidence state: Verified. Committed and structurally verified.
- Problem or constraint: Independently developed modules needed a consistent contract shape that worked at both top-level container boundaries and nested library boundaries.
- Investigation: A central contract catalog was considered but rejected because it would drift away from the code it described. Schema-first generation was also deferred because it would force language and build choices too early.
- Decision or result: The root and the initial `companion`, `ardupilot_sitl`, `gazebo`, `electromagnet`, and `scorekeeper` modules each received `PLAN.md`, `INTERNAL_INTERFACE.md`, and `EXTERNAL_INTERFACE.md`. Cross-process behavior belongs in external contracts; sibling imports within one process belong in internal contracts.
- Verification: All 18 expected documents existed, executable placeholders were absent, and a cross-contract audit found no forbidden communication path or conflicting timing rule.
- Later correction or debt: The scaffold described proposed interfaces only. Executable ROS packages, Docker images, and runtime behavior remained later work.
- Public evidence: commits `34de8ca8286ed96719e1102528593c5b7bcebdc1` (parent `8709529e4bc98797478eb91602587498d1597dbb`), `a08eaa681caa5bdb9e93b81e3f8e35c908b36072` (parent `34de8ca8286ed96719e1102528593c5b7bcebdc1`), and `cb383667a628c0a057f7baea3f550c748a9761e4` (parent `a08eaa681caa5bdb9e93b81e3f8e35c908b36072`). See the root and module `PLAN.md`, `INTERNAL_INTERFACE.md`, and `EXTERNAL_INTERFACE.md` files.

### EH-0003 Made a validated run bundle the first vertical slice

- Evidence state: Verified. Committed architecture and roadmap.
- Problem or constraint: A runnable simulation needed a concrete completion boundary before Gazebo, ArduPilot, mission logic, and scoring could be implemented independently. Recording and finalization responsibilities were otherwise likely to leak into shell scripts and unrelated services.
- Investigation: Wholesale migration of the prior orchestration was rejected because it carried coupled timing and artifact assumptions. A clean-room rewrite would discard proven worlds and parameter assets. Selective reuse behind new contracts offered the shortest controlled path.
- Decision or result: Added `orchestration/` to own run IDs, Compose lifecycle, readiness, and state transitions, and `artifacts/` to own ROS bags, onboard and observer MP4s, Gazebo logs, module JSONL logs, checksums, and bundle finalization. The run lifecycle became `CREATED → STARTING → READY → RUNNING → FINALIZING → COMPLETED|FAILED|ABORTED`. Every terminal state preserves recoverable evidence, while only a validated completed bundle counts as success.
- Verification: The checkpoint specified required bundle paths, manifest contents, clock rules, six delivery phases, and integration gates. Later planning tied the final acceptance condition to a verified maximum-score run.
- Later correction or debt: The first implementation phase deliberately used synthetic ROS nodes. Gazebo physics, ArduPilot lockstep, mission behavior, and scoring remained later phases.
- Public evidence: commit `256d7062fec6aa75929c35aa1962fdf41ec2261c` (parent `cb383667a628c0a057f7baea3f550c748a9761e4`) and roadmap commit `5f681058ebcd1fe7f8101c54a767c5134f830b9c` (parent `256d7062fec6aa75929c35aa1962fdf41ec2261c`). See `docs/superpowers/specs/2026-08-22-runnable-simulation-design.md`, `docs/IMPLEMENTATION_ROADMAP.md`, `orchestration/`, and `artifacts/`.

### EH-0004 Implemented and tightened the shared ROS interface package

- Evidence state: Verified. Committed and verified.
- Problem or constraint: All later services needed one typed ROS vocabulary with exact lifecycle values and stable field order.
- Investigation: The first contract test checked lifecycle constants only for membership, allowing the declaration order to drift without failing.
- Decision or result: Added the shared simulation ROS interfaces, then changed the test to compare the complete ordered lifecycle sequence.
- Verification: Seven contract tests passed and the ROS 2 Jazzy workspace built successfully with `colcon`.
- Later correction or debt: This phase proved interface generation and contract shape, not real Gazebo or flight traffic.
- Public evidence: commit `3a1c73e8d859b716eb4cdbe664137f9d1e6e05fb` (parent `068de23732fe375c60fd213f82513a5fa9e02cdc`) and fix `65cf0fade14f628122f5e2f4325915f3343600cb` (parent `3a1c73e8d859b716eb4cdbe664137f9d1e6e05fb`). See `ros_ws/src/simulation_interfaces/` and `tests/contracts/test_ros_interfaces.py`.

### EH-0005 Added strict run configuration and lifecycle types

- Evidence state: Verified. Committed and verified.
- Problem or constraint: Run identity and lifecycle state had to be machine-readable and exact. Loose enums or unenforced UUID annotations would allow configurations that looked valid but failed later.
- Investigation: Review found integer `auto()` enum values where exact uppercase strings were required. It also found that the chosen JSON Schema validator did not enforce UUID `format` by default.
- Decision or result: Lifecycle enums now use the required string values, and validation explicitly rejects malformed run IDs.
- Verification: Nineteen orchestration tests and 26 repository tests passed after the fix.
- Later correction or debt: Allocation-level collision handling remained assigned to the later run controller rather than configuration parsing.
- Public evidence: commit `5cbf16d19ddcea48fe0f53227f96d7021226f9f2` (parent `65cf0fade14f628122f5e2f4325915f3343600cb`) and fix `e74aad20622a275485a897ed216454ef43f16445` (parent `5cbf16d19ddcea48fe0f53227f96d7021226f9f2`). See `orchestration/src/orchestration/lifecycle.py`, `orchestration/src/orchestration/config.py`, `config/run.schema.json`, and `orchestration/tests/test_config.py`.

### EH-0006 Established structured logs and artifact manifests

- Evidence state: Verified. Committed and verified.
- Problem or constraint: Every module needed attributable JSONL logs and every run needed a manifest that could be parsed without accepting non-standard JSON.
- Investigation: The initial serializer accepted non-finite `Decimal` values, emitting `NaN` or `Infinity`, which are not interoperable JSON and could poison later validation.
- Decision or result: Added structured log and manifest models, then required finite numeric values and strict `allow_nan=False` serialization.
- Verification: The artifact suite passed 24 tests and the repository suite passed 50 after the correction.
- Later correction or debt: Phase 1 manifests established the domain shape. Deep file-tree validation, media semantics, and atomic terminal publication were deferred to Phase 2.
- Public evidence: commit `c8b4169696af0e70fb5b05ab4b7ddeb8a7ec466e` (parent `e74aad20622a275485a897ed216454ef43f16445`) and fix `01bc49dfcfcd6b55dd97026ab4b7bd796c9fbb24` (parent `c8b4169696af0e70fb5b05ab4b7ddeb8a7ec466e`). See `artifacts/src/artifacts/structured_log.py`, `artifacts/src/artifacts/manifest.py`, `artifacts/schemas/`, and `artifacts/tests/`.

### EH-0007 Proved the Phase 1 contracts in a synthetic ROS/Compose stack

- Evidence state: Verified. Committed, reviewed, and verified.
- Problem or constraint: The foundation needed a real containerized integration check before Gazebo entered the picture.
- Investigation: The first whole-branch review found that the integration test only checked process success and did not observe DDS delivery. It also found non-finite manifest scores, missing subprocess deadlines, an unasserted stdout sink, and loose ROS field matching. After those fixes, the verification document still cited stale commit, image, and test-count evidence.
- Decision or result: Added a synthetic ROS foundation stack, a real observer for DDS delivery, strict manifest score validation, bounded subprocesses, exact dual-sink assertions, and exact ROS field checks. The stale verification record was corrected in a separate evidence-only commit.
- Verification: The final Phase 1 gate passed 58 unit/contract tests and two Docker integration tests, with Compose and whitespace checks clean.
- Later correction or debt: Benign build-tool fallback warnings and ROS endpoint history/depth introspection limits were recorded. The stack remained synthetic and made no physics claim.
- Public evidence: `9a4fb51b4b649ca05bdc9c35fbfae99a3ef567c5` (parent `01bc49dfcfcd6b55dd97026ab4b7bd796c9fbb24`), `aa24cb1e6baa68cc0523137091c0ffc3c15ed1f3` (parent `9a4fb51b4b649ca05bdc9c35fbfae99a3ef567c5`), `dbd7d639a7369e6a08e0a23d9a7a352b2c5ce035` (parent `aa24cb1e6baa68cc0523137091c0ffc3c15ed1f3`), `7064e860b395a1cb21c3d8c40064b0bbc07e912f` (parent `dbd7d639a7369e6a08e0a23d9a7a352b2c5ce035`), and `2fb3e87360900823f37d1eb8a1c76759f425a19b` (parent `7064e860b395a1cb21c3d8c40064b0bbc07e912f`). See `tests/foundation/`, `tests/integration/`, `compose.yaml`, and `docs/verification/phase-1-foundation.md`.

## 2026-08-23

### EH-0008 Fixed a cold-build stdout assumption before merging Phase 1

- Evidence state: Verified. Committed, merged, and verified.
- Problem or constraint: The feature branch passed while its image was already present, but the clean merge gate rebuilt the image. Docker build progress then appeared on stdout beside application JSON, and the integration test tried to parse every line as an application event.
- Investigation: The failure reproduced only on the cold-build path. The boundary was identified as Compose multiplexing build progress with container stdout, not malformed application output.
- Decision or result: The parser now ignores unrelated non-JSON build progress while still treating malformed JSON-shaped application lines as errors. Phase 1 was then fast-forwarded into the main line.
- Verification: A deliberately cold rebuild passed 58 unit/contract tests and three integration tests. Compose validation and range checks were clean.
- Later correction or debt: Build-output filtering is intentionally narrow. It does not relax validation of lines that claim to be structured application events.
- Public evidence: commit `70d5383a55264d829ab2ed2d1a39602c053e3834` (parent `2fb3e87360900823f37d1eb8a1c76759f425a19b`). See `tests/integration/test_foundation_compose.py` and `docs/verification/phase-1-foundation.md`.

### EH-0009 Designed Phase 2 around a non-cyclic artifact finalization protocol

- Evidence state: Verified. Committed implementation plan.
- Problem or constraint: A completed run could not truthfully publish `COMPLETED` inside a bag whose validity had to be established before completion. Finalization also had to succeed after simulation time stopped.
- Investigation: A containerized orchestrator with Docker-socket access and the prior coupled shell workflow were rejected. The design instead separated host lifecycle authority from ROS runtime recording and introduced a quiescence barrier.
- Decision or result: A host `RunController` owns Compose and terminal selection. `ArtifactSession` owns recorder readiness, drain, validation, and completeness. Publishers stop at `FINALIZING`; the bag closes there; `manifest.json` is the sole authoritative terminal commit; later ROS terminal notifications cannot mutate required evidence. One monotonic wall deadline bounds infrastructure finalization.
- Verification: The eight-task plan specified exact commands, topic/QoS contracts, completed/failed/aborted integration cases, cold-clock readiness, media validation, and no-overwrite behavior.
- Later correction or debt: Gazebo physics and ArduPilot remained later roadmap phases. The plan intentionally reused validation and durability ideas from earlier work, not its archive schema or orchestration scripts.
- Public evidence: commit `938a777348db68019c9cc0c472091e7347529998` (parent `70d5383a55264d829ab2ed2d1a39602c053e3834`). See `docs/superpowers/plans/phase-2-run-artifacts.md`, `orchestration/PLAN.md`, and `artifacts/PLAN.md`.

### EH-0010 Froze Phase 2 configuration, QoS, and deadline contracts

- Evidence state: Verified. Committed and review-tested.
- Problem or constraint: Later recorder and controller tasks needed immutable resolved configuration, exact topic/QoS names, and one shared deadline contract.
- Investigation: Review found that the documentation said only "bounded wall time" without requiring a monotonic source. It also found that a caller-constructed configuration could persist an invalid run ID. A proposed requirement to reject every existing run directory was narrowed because the intended allocator creates the directory before the snapshot writer runs.
- Decision or result: Finalization now uses one absolute monotonic deadline across quiescence, recorder drain, validation, manifest publication, and teardown. Persistence validates the UUID and exclusively creates `configuration/run.json`; run-directory collision ownership stays with the controller.
- Verification: Forty focused and 85 full-suite tests passed on the initial commit; the correction passed 34 configuration and 41 focused contract tests.
- Later correction or debt: Existing-run-directory rejection remained an explicit controller responsibility.
- Public evidence: `befdf06a5df9fa94135f1685fa6f9ec1c8b06b36` (parent `938a777348db68019c9cc0c472091e7347529998`) and `cb34f5998a84a935022f3989c580a1994943ec21` (parent `befdf06a5df9fa94135f1685fa6f9ec1c8b06b36`). See `orchestration/src/orchestration/config.py`, `config/run-template.schema.json`, the root and module interface documents, and `orchestration/tests/test_config.py`.

### EH-0011 Made run-bundle finalization deterministic and race-resistant

- Evidence state: Verified. Committed, corrected, and test-verified.
- Problem or constraint: Presence-only artifact checks could label junk as valid. Concurrent finalizers, symlink swaps, and non-UTF-8 paths also had to fail closed rather than overwrite data or escape the run root.
- Investigation: The first implementation added deep manifests and tree hashing, but review reproduced three defects: two conflicting writers could both succeed through check-then-`replace`; swapping an intermediate directory for a symlink could hash an external file; and a raw non-UTF-8 filename raised `UnicodeEncodeError` instead of producing an invalid result.
- Decision or result: Finalization now records `valid|missing|invalid` with diagnostics and provenance. It traverses from retained directory descriptors with no-follow opens, rejects hard links and path escapes, hashes canonical trees, and publishes manifests with an atomic no-clobber hard link. Losing writers compare the winner's exact bytes for idempotency or conflict.
- Verification: The initial artifact suite passed 73 tests and the full suite 127. The fix passed 61 focused, 77 artifact, and 131 repository tests, including real rename/symlink races and a raw `0xff` filename.
- Later correction or debt: The shallow compatibility `build_manifest` helper remained test-only debt and was scheduled for removal from the controller-facing interface. Media semantics still belonged to the rosbag and video tasks.
- Public evidence: `37c425e27388f7ca150353986fe3d9c77802f90e` (parent `cb34f5998a84a935022f3989c580a1994943ec21`) and `5c2a70c06138bdd79f558e6f8eef4d4340943b52` (parent `37c425e27388f7ca150353986fe3d9c77802f90e`). See `artifacts/src/artifacts/manifest.py`, `artifacts/src/artifacts/session.py`, `artifacts/src/artifacts/validation.py`, `artifacts/schemas/manifest.schema.json`, and `artifacts/tests/`.

### EH-0012 Added an explicit, semantically validated rosbag recorder

- Evidence state: Verified. Committed, corrected, and test-verified.
- Problem or constraint: The bag had to subscribe to a fixed ten-topic contract before clock release, preserve pre-clock readiness evidence, shut down within the shared deadline, and prove that reported hashes described the same bytes that semantic validation read.
- Investigation: Container work first failed because the build environment lacked `wheel`; exact dependency sync then removed the already installed production package until inexact mode was used. A real Jazzy test exposed identity comparison against a pybind QoS enum. Review later found missing topic-type checks, a checksum-to-semantic-read race, unpinned apt additions, and symlink-following log creation.
- Decision or result: The adapter records explicit topics without `--use-sim-time`, checks endpoint types and QoS, stops with bounded SIGINT/TERM/KILL, reads MCAP metadata and messages with ROS type support, validates run and frame correlation, and revalidates the hardened tree after semantic reads. All newly installed packages are version-pinned, and logs use retained no-follow descriptors.
- Verification: Initial commit `4280836c` passed 28 container and 157 host tests. Fix `d527abbf` passed 37 focused and 114 full container tests plus 164 host tests.
- Later correction or debt: Empty publisher sets remain valid for intentionally unpublished topics. Full live-runtime orchestration and clock-release sequencing remained later tasks.
- Public evidence: `4280836c056dba2a94a94d11cc5209bdf1cfb7c6` (parent `5c2a70c06138bdd79f558e6f8eef4d4340943b52`) and `d527abbf60a9bc2b2fae2621b68f8ed67cc9298f` (parent `4280836c056dba2a94a94d11cc5209bdf1cfb7c6`). See `artifacts/src/artifacts/_adapters/rosbag.py`, `artifacts/config/rosbag_qos.yaml`, `artifacts/Dockerfile`, and `artifacts/tests/test_rosbag_adapter.py`.

### EH-0013 Implemented dual simulation-time video, then replaced pathname publication with an anonymous-inode design

- Evidence state: Verified. Three commits verified; final review-driven redesign still in progress at the cutoff.
- Problem or constraint: Onboard and observer recordings had to pair image data with metadata at exact 50 ms simulation intervals, encode H.264/yuv420p, finalize within one deadline, and never publish unvalidated or attacker-swapped bytes.
- Investigation: The first implementation produced valid media but review exposed unbounded pipe close and probes, pathname and symlink output races, configured-count drift, weak decode error handling, exception leaks, and non-transactional startup. Two fix rounds added retained descriptors, descriptor-bound linking, strict decode, inotify guards, full-write loops, bounded hashing, and process cleanup. Further adversarial review still reproduced a final post-guard mutation window, spawn-handoff zombie/log leaks, and check-then-unlink rollback races.
- Decision or result: The simpler chosen model was to encode into an anonymous `O_TMPFILE`, drop every writable handle after encoder exit, validate through a retained read-only descriptor, and create exactly one no-clobber link for either the final MP4 or a recoverable partial. Because `AT_EMPTY_PATH` required an unwanted capability, the design used the kernel-controlled `/proc/self/fd/N` source with `linkat(..., AT_SYMLINK_FOLLOW)` while retaining the descriptor.
- Verification: Commit `4f878771` produced H.264/yuv420p, 320×240, 20/1 FPS output and passed 57 focused, 171 container, and 217 host tests. Fixes `55787c1f` and `e72c3e26` reached 82 and then 107 focused container tests, with 196 and 221 full container tests. The 143-package FFmpeg lock remained unchanged. A real container proved anonymous-inode linking and no-clobber `EEXIST` behavior.
- Later correction or debt: At the end of August 23, the anonymous-inode implementation was green in focused slices but had not yet closed its final safety review. [EH-0014](#eh-0014-closed-the-video-publication-and-recovery-safety-review) records the next day's closure.
- Public evidence: `4f878771f7f68ee6229938edc42afee8215d77e1` (parent `d527abbf60a9bc2b2fae2621b68f8ed67cc9298f`), `55787c1f80d1fc91e3149fb6e93b6d91b04e7e0d` (parent `4f878771f7f68ee6229938edc42afee8215d77e1`), and `e72c3e269f76a38374d8463f61048fa2f3d3f18e` (parent `55787c1f80d1fc91e3149fb6e93b6d91b04e7e0d`). See `artifacts/src/artifacts/_adapters/video.py`, `artifacts/src/artifacts/recorder_node.py`, `artifacts/tests/test_video_adapter.py`, `artifacts/Dockerfile`, and the Phase 2 task report under `.superpowers/sdd/phase-2-run-artifacts/`.

## 2026-08-24

### EH-0014 Closed the video publication and recovery safety review

- Evidence state: Verified.
- Problem or constraint: Dual-video finalization still exposed writable output after validation, unsafe recovery cleanup, and mutable FFmpeg logs.
- Investigation: Adversarial tests reproduced post-validation mutation, spawn-handoff cleanup failures, rollback races, and recovery work that could outlive the shared deadline. Path-based sealing and inotify guards did not close every race.
- Decision or result: Encode into anonymous temporary inodes, drop writable capabilities before validation, publish final or recovery output once without clobbering, keep recovery snapshots within the original deadline, and seal FFmpeg logs with descriptor-safe publication.
- Verification: Tests covered mutation resistance, no-clobber publication, bounded recovery, cleanup, and log sealing.
- Later correction or debt: The terminal manifest later supplies bundle-wide authority because POSIX cannot atomically publish several files as one operation.
- Public evidence: commits `4b397d6`, `c7a1fac`, and `637b18a`; `artifacts/src/artifacts/_adapters/video.py`; `artifacts/tests/test_video_adapter.py`; historical path `.superpowers/sdd/phase-2-run-artifacts/task-4-report.md` in those commits.

### EH-0015 Structured Compose log capture

- Evidence state: Verified. By focused, artifact, and repository tests; committed.
- Problem or constraint: Phase 2 needed exact raw Docker logs and common-schema structured JSONL for seven fixed module owners. The frozen repository spelling `ardupilot_sitl` had to remain unchanged, and publication had to preserve evidence on partial failure.
- Investigation: Parser mutation testing showed that applying `Decimal` to every JSON number broke otherwise valid fractional values nested under `fields`. A durability fault injection also showed that a raw file successfully hard-linked before a later publication error could remain visible while the operation reported the wrong outcome. A final descriptor audit found a retained run-root descriptor leak when `logs/` itself was unsafe.
- Decision or result: Added `DockerLogCapture`, exact service-to-module mapping, raw and structured candidates, strict parsing, no-clobber descriptor-relative publication, and immutable capture results that inventory any already-published files. The fixed output for ArduPilot is `logs/ardupilot_sitl.jsonl`.
- Verification: Commit `6cffe8b0326c89f92c478d22eebf670c152b03e5` passed 67 focused tests, 286 artifact tests with 10 skips, and 340 repository tests with 10 skips.
- Later correction or debt: POSIX cannot atomically publish seven independent files as one transaction. The accepted behavior is fail-closed reporting of the exact published subset.
- Public evidence: `6cffe8b`; `artifacts/src/artifacts/_adapters/docker_logs.py`; `artifacts/tests/test_docker_logs.py`; `.superpowers/sdd/phase-2-run-artifacts/task-5-report.md`.

### EH-0016 Log parser host merge and cleanup hardening

- Evidence state: Verified. By focused reproductions and full regression suites; committed.
- Problem or constraint: Review found that deeply nested JSON and oversized integers escaped the typed parser path and caused all raw diagnostics to be deleted. Timestamp parsing accepted a non-ISO separator, runner failures could lose `stderr`, Task 6 lacked a safe host-event merge seam, and cleanup failures could not amend an already-built result.
- Investigation: Reproductions produced untyped `RecursionError` and `ValueError` with zero raw logs retained. A timezone-profile test briefly failed because the detector classified an attempted offset with seconds as "missing timezone" instead of an invalid profile. The existing `finally` shape could clean resources but could not add cleanup facts to frozen results.
- Decision or result: Parser resource failures became typed diagnostics while every raw stream remains publishable. Timestamp validation now uses an explicit profile. Exception output selects `output` or `stdout` once, then appends `stderr` deterministically. A descriptor-safe `orchestration-host.jsonl.partial` input merges by normalized UTC timestamp, Compose before host on ties, then original line order. Capture now builds a provisional result, performs cleanup, and only then returns or raises the amended immutable result.
- Verification: Commit `758aac1a04b5ad9817dc7deb0107cf4f5113b4ce` passed 89 Docker/structured tests, 308 artifact tests with 10 skips, and 362 repository tests with 10 skips.
- Later correction or debt: Host events remain a separate recovery source and are never appended to raw Docker bytes. A requested host source that exists but cannot be opened stays listed as recovery evidence.
- Public evidence: `758aac1`; `artifacts/src/artifacts/_adapters/docker_logs.py`; `artifacts/tests/test_docker_logs.py`; `.superpowers/sdd/phase-2-run-artifacts/task-5-report.md`.

### EH-0017 Candidate creation cleanup facts

- Evidence state: Verified. By three RED/GREEN regressions and full suites; committed.
- Problem or constraint: Candidate creation could fail before ownership transferred into the tracked candidate list. Unlink errors were suppressed, cleanup directory durability was omitted, and the immutable result could falsely report no leftover. An already-absent post-publication partial was also reported as an inspection failure.
- Investigation: Fault injection left an actual `logs/docker/service-orchestration.log.partial` while reporting `leftover_partials=()` and no cleanup diagnostic.
- Decision or result: Candidate creation now carries ordered cleanup diagnostics and exact leftovers into capture before tracking. Cleanup directory `fsync` failures are retained, and `FileNotFoundError` means the partial is already clean. Primary creation or publication failure stays first in diagnostic order.
- Verification: Commit `3b9d8ab54ccce11294227549f0b6575d0b6e75a8` passed 92 Docker/structured tests, 311 artifact tests with 10 skips, and 365 repository tests with 10 skips.
- Later correction or debt: None recorded in this scope.
- Public evidence: `3b9d8ab`; `artifacts/src/artifacts/_adapters/docker_logs.py`; `artifacts/tests/test_docker_logs.py`.

### EH-0018 Preserve log capture cancellation

- Evidence state: Verified. By control-flow exception identity tests and full suites; committed.
- Problem or constraint: Cleanup had to run for every `BaseException`, but the creation-failure carrier wrapped `KeyboardInterrupt` and `SystemExit` as ordinary capture errors. That could turn operator abort into FAILED.
- Investigation: Injecting `KeyboardInterrupt` during candidate `fsync` returned a typed `DockerLogCaptureError` instead of propagating the original control-flow object. Merely narrowing the handler to `Exception` would have skipped required cleanup.
- Decision or result: Cleanup still covers all `BaseException`. Non-`Exception` control flow is re-raised unchanged after unlink, directory `fsync`, and descriptor close; ordinary failures continue through the typed diagnostic carrier.
- Verification: Commit `065cbad9e72901c86dcefaa6221c905f1395c1d4` passed 94 Docker/structured tests, 313 artifact tests with 10 skips, and 367 repository tests with 10 skips.
- Later correction or debt: None recorded.
- Public evidence: `065cbad`; `artifacts/src/artifacts/_adapters/docker_logs.py`; `artifacts/tests/test_docker_logs.py`.

### EH-0019 Operator lifecycle and CLI

- Evidence state: Verified. By 435 repository tests with 10 expected environment skips; committed.
- Problem or constraint: The project needed a foreground operator lifecycle with four commands, concurrent read-only status/result access, atomic host control files, exact seven-service Compose control, shared deadlines, immutable first-cause semantics, log capture, artifact validation, and manifest-before-notification ordering.
- Investigation: Import-first tests established the missing controller, status-store, and CLI modules. Later focused REDs covered accidental self-competition by the controller's own completion request, nonfinite JSON, incomplete manifest inventory, lost teardown failures, symlink escape, exceptions bypassing finalization, read-only lookup creating directories, deadline resets in provenance calls, and report/hash mismatches.
- Decision or result: Added `StatusStore`, policy-free `ComposeRuntime`, injected-clock `RunController`, and the `drone-sim` commands `start`, `status`, `abort`, and `collect-results`. The lifecycle uses one startup deadline and one finalization deadline, with `min(5 seconds, finalization_wall_seconds / 5)` reserved for `compose down`. First observed terminal cause is durable authority; later failures are diagnostics. The host validates exact video, bag, and report size/hash agreement before manifest commit.
- Verification: Commit `6234cdaad596771c7f94e6601fe95fc5bf408a41` passed 68 focused Task 6 cases, 112 orchestration tests, 432 combined tests with 10 skips, and 435 repository tests with 10 skips. CLI help, lock, compilation, and diff checks also passed.
- Later correction or debt: The implementation still depended on Task 7 for real services and runtime status producers, so it did not yet establish a runnable Phase 2 stack.
- Public evidence: `6234cda`; `orchestration/src/orchestration/status_store.py`; `orchestration/src/orchestration/_adapters/compose.py`; `orchestration/src/orchestration/controller.py`; `orchestration/src/orchestration/cli.py`; `.superpowers/sdd/phase-2-run-artifacts/task-6-report.md`.

### EH-0020 Harden controller authority and deadlines

- Evidence state: Verified. By focused and full suites; committed.
- Problem or constraint: Review showed that synchronous finalization work could overrun its shared deadline, required service death could disappear from `compose ps`, post-manifest failures could contradict immutable terminal state, host log/stdout failures could bypass finalization, and output-root creation could mutate through a symlink before rejecting it. The custom profile variable also did not activate Compose's `phase2` profile.
- Investigation: A fake finalizer advanced the monotonic clock by 1,000 seconds yet the controller returned COMPLETED. A one-service `ps` response passed health checks. Injected terminal-control failure returned FAILED while the committed manifest said COMPLETED. Broken stdout skipped capture and manifest creation. A symlinked output ancestor caused directory creation outside the requested root.
- Decision or result: Propagated cooperative deadline checks through capture, parsing, hashing, validation, and publication. Compose now queries all containers and compares the exact required service set and states. Post-commit control/notification failures become diagnostics. Host observability failure becomes a retained cause while finalization continues. Output roots are descriptor-walked before creation, and `COMPOSE_PROFILES=phase2` activates the runtime profile.
- Verification: Commit `10282af5cc5549963a5d34ef415f0361fd33896d` passed 84 focused tests, 128 orchestration tests, 451 combined tests with 10 skips, and 454 repository tests with 10 skips.
- Later correction or debt: Review found two remaining seams: post-publication validation could still replace committed state, and protocol reads could cross their deadline without before/after checks.
- Public evidence: `10282af`; `orchestration/src/orchestration/controller.py`; `orchestration/src/orchestration/_adapters/compose.py`; `orchestration/src/orchestration/status_store.py`; `artifacts/src/artifacts/{manifest.py,session.py,validation.py}`.

### EH-0021 Typed manifest authority

- Evidence state: Verified and committed.
- Problem or constraint: The controller needed an unambiguous authority boundary at the exact return from manifest publication. Runtime-status waits also had to reject facts observed only after their applicable work or manifest deadline.
- Investigation: A post-publication validation timeout returned FAILED while `manifest.json` remained COMPLETED. A runtime-status read advanced the clock beyond deadline and was still accepted because `_wait_for` neither passed a checker into the read nor checked around it. Self-review found the post-manifest `runtime-failure` observation had the same missing pre-teardown check.
- Decision or result: Added frozen `FinalizationResult` carrying path, run ID, terminal status, and reason. `finalize_with_result()` returns only after atomic publication; the legacy path-only wrapper remains. The controller treats the typed return as terminal authority, and later reread, acknowledgement, notification, or teardown failures add diagnostics only. Status waits now check before and after reads and pass the applicable checker into storage.
- Verification: Commit `d69d76bbb57a4dca2e6f0bb8a6cdeec13674010a` passed 460 repository tests with 10 skips, 131 orchestration tests, and 319 artifact tests with 10 skips.
- Later correction or debt: None remaining in Task 6 scope.
- Public evidence: `d69d76b`; `artifacts/src/artifacts/session.py`; `orchestration/src/orchestration/controller.py`; corresponding unit tests and interface documents.

### EH-0022 Phase 2 container scaffold

- Evidence state: Verified. By static contract tests and Compose rendering; committed.
- Problem or constraint: Task 7 needed seven controller-owned services, runtime and test images, identical run/config mounts, and a `phase2` profile without breaking the existing foundation service.
- Investigation: Five static contract tests failed against the pre-Task 7 tree because the services and image contracts did not exist.
- Decision or result: Added Phase 2 Compose services, orchestration and artifact image changes, the synthetic test image and entrypoint, and `tests/integration/test_phase2_runtime_contract.py`.
- Verification: Commit `ad26520` (`feat: add phase2 runtime containers`) made the five tests pass. Plain and profile-specific `docker compose config --quiet` checks passed.
- Later correction or debt: Runtime ROS behavior and end-to-end CLI acceptance remained for later Task 7 and Task 8 work.
- Public evidence: `ad26520`; `compose.yaml`; `orchestration/Dockerfile`; `artifacts/Dockerfile`; `tests/phase2/Dockerfile`; `tests/integration/test_phase2_runtime_contract.py`.

### EH-0023 Synthetic runtime transport and artifacts

- Evidence state: Verified. Across host tests, three container suites, an isolated Jazzy graph test, and three deterministic seven-service runs; committed.
- Problem or constraint: The synthetic runtime had to produce exact 20 Hz clocks, 40 paired frames per camera, matching metadata and ground truth, two videos, a fixed ten-topic MCAP bag, durable statuses, and silent aggregate finalization without deriving simulated order from wall time.
- Investigation: Container integration exposed ROS setup under `set -u`, pytest module-name collisions, missing root test bootstrap files in images, root-only `0600` statuses, and a project-name mismatch during an early `--no-build` invocation. A fixed scheduler yield did not prevent DDS cross-topic reordering. An ACK-backed launch then deadlocked. Instrumentation ruled out lifecycle QoS and traced the stall to camera publication/drain ordering. BEST_EFFORT archival camera delivery repeatedly lost observer images while metadata arrived. Docker's legacy builder later stalled during image export; BuildKit was unavailable, so the attempt was stopped and retried without machine changes.
- Decision or result: Switched pytest to importlib collection, made runtime status explicitly host-readable, added exact endpoint discovery before publication, bounded image/metadata pairing buffers, a six-item publication queue, current-run contiguous ACKs, and reliable depth-5 archival camera QoS. The internal reliable transient-local camera-pair ACK is deliberately excluded from the ten-topic bag. Wall-delay injection occurs only after event identity and simulated timestamp are fixed.
- Verification: Commit `789686882a7eb5c2819ae10311c635cd19957fb7` passed 532 host tests with 10 skips, 365 artifact-container tests, the other container suites, Compose checks, and a full seven-service build. Three runs with wall delays 0, 0, and 17 ms produced identical 40-frame timestamps, videos, hashes, reports, and terminal facts.
- Later correction or debt: This is synthetic artifact evidence only. It makes no claim about real Gazebo physics, ArduPilot lockstep, mission behavior, or scoring validity.
- Public evidence: `7896868`; `artifacts/src/artifacts/runtime_node.py`; `artifacts/src/artifacts/runtime_protocol.py`; `orchestration/src/orchestration/runtime_node.py`; `config/recording-qos.yaml`; `tests/phase2/`; `.superpowers/sdd/phase-2-run-artifacts/task-7-report.md`.

### EH-0024 Seal synthetic runtime boundaries

- Evidence state: Verified. By two project-independent `--no-build` launches, Phase 1 regression, three deterministic aggregate runs, and full suites; committed.
- Problem or constraint: Review found that per-project image names made fresh CLI runs unable to resolve prebuilt images, unprofiled `foundation` created an eighth service, `runtime-frozen` represented only one producer, recorder failures were not durable, stubborn or prematurely dead rosbag processes could permit invalid publication, status modes were umask-dependent, and malformed path arrays raised raw type errors.
- Investigation: Rendering two unique Compose projects produced different image names. The phase2 service list included foundation. Delayed producer callbacks could occur after the supposed freeze. Under umask 077, requested `0644` files became `0600`.
- Decision or result: Added stable Phase 2 image tags and a separate foundation profile. Each pre-recorder module writes a descriptor-safe quiescence marker; orchestration publishes aggregate freeze only after all required markers exist. Recorder diagnostics now become first-wins durable runtime failure, process health is monitored beyond readiness, and final artifact publication requires confirmed recorder shutdown or safe isolation. Status creation applies explicit modes and validates path element types. Interface text now distinguishes reliable archival transport from later best-effort consumers.
- Verification: Commit `3031516616f885ffb4bced9496dfc040542d4486` passed 562 host tests with 10 skips, container suites of 386, 149, and 15 tests, two uniquely named no-build projects, foundation regression, and three deterministic aggregate runs including delayed quiescence.
- Later correction or debt: Re-review found two remaining races around recorder death during FINALIZING and STARTING publication after barrier failure.
- Public evidence: `3031516`; `compose.yaml`; `artifacts/src/artifacts/{runtime_node.py,runtime_protocol.py}`; `orchestration/src/orchestration/runtime_node.py`; module interface documents.

### EH-0025 Close runtime finalization races

- Evidence state: Verified. By nine new regressions plus focused and repository suites; committed.
- Problem or constraint: Artifact health polling stopped after FINALIZING, allowing rosbag to die while the runtime waited for aggregate freeze and still appear successfully exited. The orchestration discovery loop also called runtime start after timeout, failure, or preemption.
- Investigation: A stubborn-recorder test double initially failed after shutdown-origin checks were added because the new field was consulted before the existing non-exit branch. A final race audit found a process could die between the adapter's first liveness poll and its first signal attempt.
- Decision or result: Recorder health polling continues through FINALIZING. Premature exit is durably reported and blocks validation/publication. `shutdown_requested` changes only when the host actually attempts a signal, distinguishing spontaneous death from expected bounded shutdown. The startup helper calls runtime start exactly once only after exact endpoint readiness; timeout, failure, preemption, or shutdown returns without STARTING.
- Verification: Commit `ba02d5904b081499546454a2fbad64aa6ef7e849` passed 70 focused tests with four skips, 551 combined tests with 10 skips, and 569 repository tests with 10 skips.
- Later correction or debt: None in the reviewed Task 7 scope.
- Public evidence: `ba02d59`; `artifacts/src/artifacts/_adapters/rosbag.py`; `artifacts/src/artifacts/runtime_node.py`; `orchestration/src/orchestration/runtime_node.py`; their focused tests.

### EH-0026 Public CLI Phase 2 acceptance

- Evidence state: Verified. By public CLI runs for COMPLETED, FAILED, and ABORTED plus full repository gates; committed.
- Problem or constraint: Task 8 had to prove real foreground CLI behavior, read-only semantic inspection, complete raw and structured logs, immutable manifests, repeatable result collection, and cleanup while building the stable images only once.
- Investigation: Six unit tests failed because scoring provenance hashed `scoring/result.json` instead of consuming its declared checksum and accepted unsafe evidence. The first full integration run failed all three modes: Compose's NDJSON `ps` output was rejected, failure could not publish all seven structured logs, and abort did not reach durable RUNNING. Cleanup still removed each failed project's resources.
- Decision or result: Accepted actual Compose NDJSON status output, enforced the declared scoring checksum, and had the host pre-create the shared Docker-log directory so container ownership could not block capture. The test gate uses the public CLI and a read-only container inspector for media, MCAP, lifecycle, logs, hashes, and cleanup. Injected observer failure preserves recovery evidence; concurrent abort returns exit 130 and remains ABORTED across repeated commands.
- Verification: Commit `a08f527` added the implementation and acceptance gate; `7a39758` recorded evidence. The gate passed three terminal cases in 127.52 seconds, then `make test` passed 549 unit/contracts with 10 skips, three Phase 1 tests, and three Phase 2 tests after one image build.
- Later correction or debt: The first acceptance review later found five fail-closed/assertion gaps. [EH-0027](#eh-0027-tighten-phase-2-acceptance) records the corrections.
- Public evidence: `a08f527`; `7a39758`; `tests/integration/test_phase2_compose.py`; `tests/phase2/inspect_bundle.py`; `orchestration/src/orchestration/controller.py`; `docs/verification/phase-2-run-artifacts.md`.

### EH-0027 Tighten Phase 2 acceptance

- Evidence state: Verified. By real terminal bundles, full gates, and independent re-review; committed.
- Problem or constraint: Invalid scoring provenance could still yield COMPLETED; failed and aborted tests allowed arbitrary partials; repeated commands compared only terminal state; abort media validation could use decode output as its own predicate; and image-map and timing evidence was incomplete.
- Investigation: The original abort media condition allowed corrupt positive-frame output to produce no hashes and skip every playability assertion. The failed/aborted inventory accepted capture candidates and unknown partial files. Repeated abort and collection could return plausible ABORTED output without proving zero exit, matching run facts, or an unchanged manifest.
- Decision or result: Invalid or missing score provenance now downgrades requested COMPLETED only. Terminal-specific allowlists reject capture, host-source, structured/raw-log, manifest, and unknown partials. Repeated commands require zero exit, exact canonical result equality, and unchanged immutable snapshots. Recorder-local frame counts independently require probe, full decode, valid semantics, and matching hash counts for every positive-frame video.
- Verification: Commit `ec1f068c86b57ffdd92b06f0277c2600555574cf` passed 13 Phase 2 assertions, 553 unit/contracts with 10 skips, and three Phase 1 tests. Real failed and aborted runs had recorder counts 6/5 and 10/10, with matching decoded-hash counts.
- Later correction or debt: Three minor items remained at this point: direct comparison of seven-image maps across four manifests, all three finalization intervals for every representative run, and an unused inspector import.
- Public evidence: `ec1f068`; `orchestration/src/orchestration/controller.py`; `orchestration/tests/test_controller.py`; `tests/integration/test_phase2_compose.py`; `tests/phase2/inspect_bundle.py`; `docs/verification/phase-2-run-artifacts.md`.

### EH-0028 Close the Phase 2 artifact review

- Evidence state: Verified and committed after recovery.
- Problem or constraint: The whole-branch review at commit `eb3dcb7` left one Critical and six Important findings open. An interrupted worktree also contained substantial uncommitted fixes and unsupported completion claims. Phase 2 still needed firm manifest authority, exact media geometry, ordered artifact status, safe scoring provenance, isolated Compose selection, and finalization evidence.
- Investigation: Narrow reproductions showed a post-hard-link deadline split, configuration and media mismatch, incomplete `ArtifactStatus` sequencing, unsafe scoring symlink reads, post-commit operator inconsistency, and ambient Compose selection. The recovered claims were discarded as evidence and every changed path was audited. A stale ledger count said 573 tests passed; the fresh suite found 574.
- Decision or result: Once manifest publication succeeds through its hard-link commit point, a later cooperative deadline check cannot demote it to uncommitted. Typed manifest results became authoritative over mutable operator status. Artifact status became an ordered not-ready, ready, then post-manifest final protocol. Phase 2 geometry was frozen at 320 by 240, and Compose stopped inheriting ambient file, profile, project, or env-file selectors.
- Verification: Build-once and no-rebuild Phase 2 acceptance each passed 14 cases. The repository suite passed 574 tests with 11 expected skips, Phase 1 passed 3 tests, and two source-exact ROS checks passed. Completed, failed, and aborted terminal bundles were inspected.
- Later correction or debt: The verification count was corrected from 573 to 574. This work remained synthetic Phase 2 evidence and made no claim about real Gazebo, ArduPilot, mission execution, or scoring.
- Public evidence: Commit 456b563611db5a6293a0d30b628c56353d41c942. Paths: artifacts/src/artifacts/manifest.py, artifacts/src/artifacts/runtime_node.py, artifacts/src/artifacts/runtime_protocol.py, artifacts/src/artifacts/validation.py, orchestration/src/orchestration/controller.py, orchestration/src/orchestration/status_store.py, orchestration/src/orchestration/_adapters/compose.py, tests/integration/test_phase2_compose.py, docs/verification/phase-2-run-artifacts.md.

### EH-0029 Choose the paused Gazebo ownership boundary

- Evidence state: Verified. Reviewed architecture decision.
- Problem or constraint: Phase 3 needed real Gazebo physics without importing Phase 2 recorder backpressure into the physics loop or splitting lifecycle ownership across containers.
- Investigation: A split Gazebo server and ROS gateway design added discovery and shutdown races. A custom all-in-one C++ plugin would couple Gazebo and ROS ABIs before it was needed.
- Decision or result: Use one run-scoped container with a paused Gazebo Harmonic server and a small lifecycle and ROS adapter. Keep Gazebo Transport private. Gazebo owns simulation time, cameras, ground truth, native state, and server logs. Artifacts owns semantic validation and checksums. Orchestration owns terminal commit. The synthetic camera-pair acknowledgment remains Phase 2-only.
- Verification: The decision was checked against existing lifecycle, recorder, bundle, QoS, and reset contracts. The proposed public cameras remained reliable, 320 by 240 RGB8, and 20 simulated Hz.
- Later correction or debt: ArduPilot lockstep, flight dynamics, mission logic, electromagnet physics, scoring, and a complete production run were deliberately deferred.
- Public evidence: commits `2b65a21` and `a8dd24c`; historical path `docs/superpowers/specs/2026-08-24-phase-3-gazebo-foundation-design.md` in those commits; `gazebo/EXTERNAL_INTERFACE.md`; `gazebo/PLAN.md`; `config/recording-qos.yaml`.

### EH-0030 Add exact runtime profiles and duration arithmetic

- Evidence state: Verified. After review correction.
- Problem or constraint: Configuration was Phase 2-only, Compose ownership was hard-coded, and duration arithmetic could drift through binary floats.
- Investigation: The first RED run reported 54 failures and 76 passes because runtime profiles, simulation configuration, immutable topology, and topology-aware operations did not exist. The first implementation left one old call without an explicit topology argument. Review then found JSON Schema multipleOf 0.05 rejected the valid value 0.15 even though the Decimal parser handled it correctly.
- Decision or result: Add RunConfig.runtime_profile, SimulationConfig, and immutable RuntimeTopology. Keep omission-based Phase 2 compatibility while making the repository default explicit. Convert duration through Decimal string input and integer nanoseconds. Pass topology through every Compose and controller operation. Remove the float-sensitive schema keyword and leave exact 50 ms grid enforcement to the parser. Document world-frame ENU truth.
- Verification: Focused tests reached 130 passes, the affected suite reached 211, and the review fix passed 67 focused and 213 full tests.
- Later correction or debt: Huge finite integral durations and mismatched hand-built topology maps received regression coverage. No open review issue remained.
- Public evidence: Commits dfd7b53 and 59b0aab. Paths: orchestration/src/orchestration/config.py, orchestration/src/orchestration/_adapters/compose.py, config/run.schema.json, config/run-template.schema.json, EXTERNAL_INTERFACE.md, gazebo/EXTERNAL_INTERFACE.md.

### EH-0031 Pin the Harmonic image, package closure, and assets

- Evidence state: Verified and test-verified.
- Problem or constraint: The Gazebo runtime needed immutable supply inputs, exact package versions, local-only model assets, and a reproducible offline image gate.
- Investigation: The approved ROS Gazebo meta-package expanded to a 777-package delta and made cold legacy-builder commits slow. ROS setup failed under nounset. Setuptools mistook resources and provenance for Python packages before source code existed. The legacy Docker builder silently ignored a heredoc body, and the Gazebo version command printed bare 8.11.0 rather than the assumed label.
- Decision or result: Commit the complete sorted apt delta, provenance records, byte-identical imported meshes, a passive local Iris model, and a copied executable image test. Reserve the Python entry point with empty package discovery until runtime code arrives. Parse the observed version format. Preserve upstream license bytes exactly even when local whitespace rules differ.
- Verification: The final offline gate rehashed 11 provenance records, checked all 777 installed versions, confirmed Gazebo 8.11.0 and ros_gz_bridge, rejected an ArduPilot plugin in this stage, and validated the model SDF. The repository suite passed 661 tests with 11 skips.
- Later correction or debt: The 777-package closure and legacy storage driver made builds slow. Runtime package discovery was intentionally deferred to the server/runtime task.
- Public evidence: Commit 2b8cde6. Paths: gazebo/Dockerfile, gazebo/harmonic-packages.lock, gazebo/provenance/, gazebo/resources/models/iris_phase3/, gazebo/pyproject.toml, gazebo/tests/test_image_contract.py, gazebo/tests/test_provenance.py.

### EH-0032 Build the passive world and an identity-stable resource snapshot

- Evidence state: Verified. After a race-condition correction.
- Problem or constraint: Phase 3 needed a fixed local world and a resolver that could not return hashes for resources changed during validation.
- Investigation: Gazebo Harmonic ignored a contact sensor topic override and advertised a deterministic scoped topic. The exact SDF command initially appeared successful but really exited 255 without the local model search path. Review then reproduced two snapshot races: identical-byte inode replacement and temporary content substitution restored between separate hashing passes.
- Decision or result: Add a 1 ms, target 0.1 RTF world with two 20 Hz cameras, pose, odometry, contact, a marker, and passive Iris. Set the image-local SDF model path. Remove the ineffective contact topic declaration. Rework resolution into one retained no-follow descriptor snapshot, hash retained file descriptors, and verify final identities and directory inventories before returning.
- Verification: The corrected exact SDF command printed Valid and exited zero. Focused race and cleanup tests passed; the final task checks passed 18 focused, 20 descriptor-validation, and 32 Gazebo tests.
- Later correction or debt: A minor test gap remained: the camera contract tests selected the two expected cameras but did not explicitly reject a third.
- Public evidence: Commits b25fc7c62196673333d04fbc26d6b8c80f8a5b5e and c3583f22118b40862c52045a7b8f8d709cb85dd4. Paths: gazebo/resources/worlds/phase3_foundation.sdf, gazebo/src/drone_sim_gazebo/worlds/api.py, gazebo/src/drone_sim_gazebo/worlds/INTERNAL_INTERFACE.md, gazebo/tests/test_world_resources.py.

### EH-0033 Make camera and truth alignment bounded and fail-closed

- Evidence state: Verified. After review correction.
- Problem or constraint: Camera streams and truth had to align at exact 50 ms epochs without unbounded queues or recoverable data faults.
- Investigation: The first model allowed a later valid sample to repair a malformed or off-grid sample. A cross-stream mismatch advanced state before raising. Overrun could leave complete true, and converting a huge integer through math.isfinite leaked OverflowError. One UUID test also failed to create a real uppercase variant because it used digits only.
- Decision or result: Keep one sequence per camera, one unmatched slot per camera, one aligned pair awaiting truth, and no independent truth queue. Preflight alignment before mutation. Permanently latch the first processing fault, revoke completion, keep its diagnostic stable, and treat integers as finite without float conversion.
- Verification: The initial Gazebo suite passed 79 tests. The correction reached 55 focused adapter tests and 87 complete Gazebo tests. Independent probes confirmed rejected streams remained unmodified and huge integers passed unchanged.
- Later correction or debt: Installing the package in the runtime image remained a later task.
- Public evidence: commits `e3c5ae5`, `4ebc383`, and `d626cd1`; `gazebo/src/drone_sim_gazebo/ros_adapter/model.py`; `gazebo/tests/test_ros_adapter.py`; `gazebo/src/drone_sim_gazebo/ros_adapter/INTERNAL_INTERFACE.md`.

### EH-0034 Harden paused-server supervision and native evidence

- Evidence state: Verified. After two review rounds.
- Problem or constraint: The server boundary had to replace the child environment safely, supervise the entire process group, honor one deadline, and publish log and native-state evidence without path substitution or false success.
- Investigation: The first two-key environment could not launch the pinned image. Probing established six required keys: PATH, GZ_CONFIG_PATH, LD_LIBRARY_PATH, HOME, GZ_PARTITION, and GZ_SIM_RESOURCE_PATH. Precreating the record directory caused Gazebo to write state(1). Reviews also found signals after deadline expiry, named-path ABA races, premature completion, incomplete child-failure vocabulary, leader-only shutdown, stale deadline checks, post-commit cleanup reversing success, and repairable invalid native summaries. A broad timing test failed once under load and passed alone and in the fresh full rerun.
- Decision or result: Use the exact six-key immutable environment with deterministic HOME. Leave the state directory absent for Gazebo to create, then retain and verify descriptors. Keep the leader unreaped with waitid until group quiescence, resample deadlines after probes, make post-commit cleanup best-effort, and irreversibly latch invalid native evidence. Add typed child-exit and server-stop failure events.
- Verification: The final round passed 104 focused, 191 Gazebo, and 603 broader tests with 11 skips. A live pinned-image stop returned zero and produced a canonical 57,344-byte state.tlog and durable server log.
- Later correction or debt: One non-restarting absolute deadline still had to cross the durable finalization boundary. Transport endpoint readiness remained integration work.
- Public evidence: Commits 83096aa096898087c3dba30433ad61c403d90d1c, 491c38a6487d1f911e38d7931b7508a4357c89f9, and 416a1802cba9c42347be6a78b1f48d94eb6fbe56. Paths: gazebo/src/drone_sim_gazebo/server/process.py, gazebo/src/drone_sim_gazebo/runtime/model.py, gazebo/tests/test_server_process.py, gazebo/tests/test_runtime_model.py.

### EH-0035 Replace isolated hardening with a runnable flight slice

- Evidence state: Verified. Committed implementation plan based on a read-only audit.
- Problem or constraint: The repository had a paused passive Gazebo boundary but still lacked production Compose wiring, a ROS runtime, a flight-capable Iris, pinned SITL, config-driven artifacts, a production mission, and scoring.
- Investigation: The audit separated launch blockers from later hardening and traced the official ArduPilot JSON and UDP interface rather than guessing ports or lockstep behavior.
- Decision or result: Use private Compose networking with Gazebo UDP 9002, ArduPilot JSON targeting gazebo-runtime, MAVLink TCP 5760, lockstep enabled, no SITL wall-time synchronization, and Gazebo as the sole clock publisher. Split work into Gazebo, SITL, artifacts and scoring, and mission and inactive-scenario streams. Reserve Compose, orchestration, root configuration, and acceptance for coordinator integration.
- Verification: The plan tied each stream to component, first-flight, artifact, scoring, and maximum-score gates.
- Later correction or debt: Deferred work included active electromagnet forces, broader missions, hostile filesystem cases beyond known boundaries, rendering-host byte determinism, and distributed discovery stress.
- Public evidence: commit `9a4be27`; historical paths `docs/superpowers/specs/2026-08-24-runnable-vertical-descent-design.md` and `docs/superpowers/plans/runnable-vertical-descent.md` in that commit; `gazebo/PLAN.md`; `ardupilot_sitl/PLAN.md`.

### EH-0036 Run the ROS and Gazebo boundary in a container

- Evidence state: Verified. Component-level live verification.
- Problem or constraint: The passive world existed, but the package, custom messages, bridge, runtime node, endpoint discovery, bounded truth aggregation, and cleanup path were not installed together.
- Investigation: Direct CMake installation did not produce a workspace-root local_setup.bash. Jazzy already shipped another simulation_interfaces package, so the base package shadowed local messages. Fast DDS graph discovery reported queue depth zero as unknown even when local publishers had explicit depths.
- Decision or result: Correct the environment setup location, make the project overlay precede the ROS base, verify graph-visible reliability, and verify exact queue depths on local publisher QoS objects. Add bounded contact aggregation keyed to camera-pair timestamps.
- Verification: Host tests passed 219 cases with one ROS-only skip. The container self-test passed. A live smoke produced four aligned onboard, observer, and truth samples at 50, 100, 150, and 200 ms, then finalized server.log and state.tlog with return code zero.
- Later correction or debt: This was a passive physics smoke. The flight plugin and actuator-controlled motion remained Wave 2 work.
- Public evidence: commits `b026d5f`, `f395635`, and `05aca9b`; `gazebo/src/drone_sim_gazebo/runtime/`; `gazebo/src/drone_sim_gazebo/ros_adapter/`; `gazebo/config/`; `gazebo/Dockerfile`; `gazebo/tests/`; `compose.yaml`.

### EH-0037 Pin and exercise the flight-controller runtime

- Evidence state: Verified. Component-level live verification.
- Problem or constraint: ardupilot_sitl contained documentation but no pinned image, deterministic argv, lifecycle model, JSON peer gate, readiness, or durable diagnostics.
- Investigation: The default Waf job count was effectively serial, so the cached build layer was rerun with bounded parallelism. ArduPilot's JSON socket passed the target through inet_pton and could not send directly to a Docker DNS hostname; it silently used 0.0.0.0.
- Decision or result: Pin Copter 4.7.0 to the source revision recorded in repository provenance, build only the Copter target, resolve the Gazebo service name once to a numeric container-network address, and keep configuration, lifecycle state, and UDP peer behavior independently testable.
- Verification: Waf completed 1,416 tasks. The final image showed 400 Hz JSON exchange, a real MAVLink heartbeat on TCP 5760, clean process quiescence, structured start-to-stop events, SITL storage, a 6.1 MiB DataFlash log, and fail-closed peer loss. The owned suite passed 22 tests.
- Later correction or debt: The broader repository run was intentionally stopped while it was still in long integration tests, so only module-level completion was claimed.
- Public evidence: Commit 31a52f0. Paths: ardupilot_sitl/Dockerfile, ardupilot_sitl/src/drone_sim_ardupilot/, ardupilot_sitl/tests/, ardupilot_sitl/PLAN.md.

### EH-0038 Make physical artifact counts and scoring exact

- Evidence state: Verified. Component-level verification with integration obligations.
- Problem or constraint: Artifacts still assumed 40 frames, production scoring did not exist, and universal native-state validation would break the Phase 2 synthetic format.
- Investigation: Applying state.tlog requirements as a universal default caused real Phase 2 regressions. The scorer also needed a permanent incomplete state after any timestamp gap, duplicate, regression, overrun, or truncation.
- Decision or result: Derive physical frame and truth counts from resolved configuration while keeping Phase 2 fixed at 40. Make physical native validation profile-selected. Implement immutable descent_v1 scoring over exact 50 ms truth with a maximum of 100 points, allocation 20, 40, 20, and 20, and a safe pre-impact downward-speed limit of 1.0 m/s. Persist four rule events plus score.finalized through no-clobber files.
- Verification: The slice passed 510 tests with 11 expected skips. Test traces covered zero, partial, and maximum scores, and the result shape round-tripped through manifest metadata.
- Later correction or debt: Coordinator integration still had to enable physical Gazebo validation, construct the scorer from the resolved expected count, install the scorekeeper runtime, and refuse completion when scoring was incomplete.
- Public evidence: commits `6a80d8e` and `0a0930f`; `artifacts/src/artifacts/`; `artifacts/tests/`; `scorekeeper/src/drone_sim_scorekeeper/descent.py`; `scorekeeper/rules/descent_v1.json`; `scorekeeper/tests/`.

### EH-0039 Add the production mission and seven-service profile

- Evidence state: Verified. Component and static integration verification.
- Problem or constraint: The production profile needed a deterministic mission, a truthful inactive scenario, exact service ownership, and no host-published ports.
- Investigation: The mission boundary was kept independent of the legacy companion tree and tested against a fake MAVLink adapter before live SITL work.
- Decision or result: Implement a pure mission state machine for heartbeat, GUIDED, arm, takeoff, LAND, and landed or disarmed completion. Drive decisions from telemetry and simulation time, using wall time only for unavailable infrastructure. Publish mission-finished only on success. Make the electromagnet runtime publish one deterministic INACTIVE event. Integrate the exact seven services while preserving explicit Phase 2 topology.
- Verification: Mission and scenario work passed 36 owned tests and built both runtime images. Coordinator integration passed 224 relevant tests, and both Compose profiles validated.
- Later correction or debt: No live mission success was claimed at this point.
- Public evidence: commits `146c35a`, `d2373d5`, `75fbce2`, `1d2366a`, `02ea3a1`, and `58b1993`; `companion/src/`; `companion/tests/`; `electromagnet/src/`; `electromagnet/tests/`; `compose.yaml`; `orchestration/src/orchestration/config.py`; `orchestration/src/orchestration/runtime_node.py`; `tests/integration/test_phase3_runtime_contract.py`.

### EH-0040 Add actuator-driven flight and truthful exchange readiness

- Evidence state: Verified as an open state. Mixed live evidence; component fixes verified, final merged lifecycle still open.
- Problem or constraint: The passive model had no IMU, rotors, lift or drag, motor controls, official ArduPilot plugin, or readiness proof tied to actual bidirectional exchange.
- Investigation: Upstream configured unrelated camera targets and required missing GStreamer development metadata, so the build was narrowed to the needed plugin target. A paused pair initially entered lockstep with no prior sensor state and could not recover. An unpaused workaround proved flight but did not prove paused startup. The final image build later wedged in the legacy storage driver.
- Decision or result: Add vertical_descent and iris_flight resources, build the pinned official plugin, and carry a provenance-tracked patch that returns initial sensor JSON after the first servo datagram while simulation stays paused at time zero. Add a plugin-owned status service. Readiness now requires a baseline and then fresh increases in servo receipt, motor updates, and JSON sends, with online true and no gaps or send errors.
- Verification: The patched startup produced JSON and an actual MAVLink heartbeat while repeated Gazebo statistics remained paused at simulation time zero. A separate actuator run raised motor outputs from 1000 to about 1845 and vehicle altitude from about 0.195 m to 2.04 m. Final readiness tests passed 238 cases with one skip.
- Later correction or debt: The paused proof and actuator proof used separate runtime evidence. A normal non-force mission, live flight-camera 50 ms cadence, SITL-loss simulation freeze, and one exact final-image lifecycle remained mandatory integration gates. DART also rejected a decorative mesh collision, while landing-leg primitives remained active.
- Public evidence: commits `d47aad2`, `f093fc6`, `e641dd0`, and `bae979b`; `gazebo/resources/worlds/vertical_descent.sdf`; `gazebo/resources/models/iris_flight/`; `gazebo/plugin/`; `gazebo/src/drone_sim_gazebo/runtime/`; `gazebo/src/drone_sim_gazebo/server/process.py`; `gazebo/tests/test_flight_resources.py`; `gazebo/tests/test_runtime_entrypoint.py`.

### EH-0041 Make Phase 3 completion independently auditable

- Evidence state: Verified. After several corrections; some live integration flakes remained.
- Problem or constraint: Early Phase 3 completion could accept missing scorekeeper runtime, unresolved flight resources, incomplete or stale score claims, weak MCAP lifecycle evidence, placeholder logs, unchecked optional manifest records, and leftover Compose resources.
- Investigation: The first fix still allowed a self-consistent false 100 score because it did not recompute from MCAP truth. Host inspection assumed unavailable ROS and FFmpeg dependencies. A later evaluator chose the first contact in the trace and therefore rejected a valid initially grounded flight. ArtifactStatus delivery also passed graph discovery without proving application persistence, leaving a separate delivery question for the next day.
- Decision or result: Build a read-only bundle inspector that validates manifest invariants and rehashes all records, decodes exact physical MCAP evidence, checks lifecycle and configuration binding, image payloads, finite truth, inactive scenario, seven module logs, and five ordered score events, then independently recomputes descent_v1. Run semantic inspection inside the pinned artifact image and Docker inventory on the host. Match touchdown only after the first airborne sample and add a nonblocking writer-acknowledgment barrier without treating it as persistence proof.
- Verification: Successive commits reached 222 passes with four skips, then 670 with 11 skips, score parity at 100 points and 650,000,000 ns, and 685 passes with 12 skips.
- Later correction or debt: One run saw the Compose network still present at an immediate inventory check, and later attempts lost one best-effort GroundTruth sample and recorded 39 of 40. [EH-0045](#eh-0045-preserve-both-artifact-startup-states) records the delayed-reader reproduction and private depth-two retention fix.
- Public evidence: commits `c133258`, `2dada76`, `63a39ee`, and `66aaac0`; `artifacts/src/artifacts/acceptance.py`; `artifacts/src/artifacts/score_validation.py`; `artifacts/src/artifacts/_adapters/rosbag.py`; `orchestration/src/orchestration/controller.py`; `Makefile`; `artifacts/tests/`.

### EH-0042 Approve the production scoring runtime

- Evidence state: Verified. Independently reviewed and approved for integration.
- Problem or constraint: Score completion had to reflect the exact source-finished boundary, initially grounded flight semantics, ordered durable evidence, and failure-safe finalization.
- Investigation: Review focused on QoS compatibility, the first airborne then later touchdown rule, exact sample continuity, persistence order, and agreement between terminal status and score files.
- Decision or result: The runtime waits for scenario input and truth through the exact source-finished timestamp. It rejects gaps, duplicates, regressions, overruns, count mismatch, and source timestamp mismatch. It fsyncs evidence before publishing five score events, writes score-finished last, and finalizes idempotently.
- Verification: All 27 scorekeeper tests passed and no Critical or Important integration finding remained.
- Later correction or debt: A Docker self-test build stalled during a cached copy layer and was interrupted, so approval relied on focused runtime and contract verification rather than a fresh image build.
- Public evidence: Commit dbc4d2c. Paths: scorekeeper/src/drone_sim_scorekeeper/, scorekeeper/tests/, scorekeeper/rules/descent_v1.json, compose.yaml.

### EH-0043 Distinguish intentional pauses from JSON peer loss

- Evidence state: Verified and test-verified.
- Problem or constraint: SITL permanently treated the first missing-JSON diagnostic after exchange as peer loss, even when Gazebo was intentionally paused before RUNNING, at exact source completion, or during finalization.
- Investigation: The launch audit traced the unconditional latch in OutputFacts and found the runtime evaluated it before lifecycle and finalization state.
- Decision or result: Make missing JSON a current-line observation, then classify it against exact current-run durable facts. It is fatal only during RUNNING when source-finished and finalization intent are absent. Pre-RUNNING, source-complete, finalizing, and signal-driven shutdown pauses are nonfatal. Wrong-run markers cannot hide real active-run loss.
- Verification: The focused suite passed 34 tests. The full repository suite passed 990 tests with 12 skips. Independent review confirmed the production container entry point used the changed runtime.
- Later correction or debt: The narrow fix did not rerun a live Gazebo and SITL flight. That remained part of the final integration gate.
- Public evidence: Commit b47c997. Paths: ardupilot_sitl/src/drone_sim_ardupilot/runtime.py, ardupilot_sitl/src/drone_sim_ardupilot/runtime_node.py, ardupilot_sitl/tests/.

### EH-0044 Component proofs were ahead of end-to-end proof

- Evidence state: Verified as an open state. Open integration status, supported by reviewed component evidence.
- Problem or constraint: Parallel branches had proven passive Gazebo, SITL exchange, mission transitions, scoring, artifact semantics, paused plugin startup, and actuator motion separately. They had not yet proved the full operator path on one exact merged image set.
- Investigation: Several green component reports were narrowed by later launch and boundary reviews. Stale local images, readiness ordering, split flight evidence, GroundTruth loss, and teardown timing all prevented promotion to a final run claim.
- Decision or result: Retain the component commits and require one coordinator-owned acceptance lifecycle rather than combining separate proofs into a success claim.
- Verification: No qualifying seven-service, normal-mission, artifact-complete 100 of 100 bundle was recorded in this slice.
- Later correction or debt: Remaining gates were exact final-image rebuilds, ArduPilot and companion readiness before RUNNING, non-force takeoff and landing, live dual-camera cadence, simulation freeze after SITL loss, complete GroundTruth capture, clean teardown inventory, and preservation of the verified maximum-score bundle.
- Public evidence: commits `9a4be27`, `444e781`, and `b47c997`; `compose.yaml`; `config/default-run.json`; `tests/integration/`; `artifacts/src/artifacts/acceptance.py`; historical plan `docs/superpowers/plans/runnable-vertical-descent.md` in commit `9a4be27`.

## 2026-08-25

### EH-0045 Preserve both artifact startup states

- Evidence state: Verified. Implemented, image-tested, and reviewed.
- Problem or constraint: DDS writer acknowledgement did not prove that rosbag had taken or persisted the initial `ready=false` sample. With depth-one histories, the later `ready=true` sample could replace it before rosbag read either value.
- Investigation: an exact delayed-reader reproduction returned acknowledgement success but retained only `[true]`, disproving the original acknowledgement-as-persistence assumption.
- Decision or result: keep the public `ArtifactStatus` publisher reliable, transient-local, and depth 1, but give the private artifact-owned rosbag subscription depth 2. The delayed reader then retained `[false, true]` without changing the public QoS contract.
- Verification: rebuilt-image delayed-take testing passed, and the installed override, other topic QoS settings, deadline behavior, and diagnostics were reviewed.
- Later correction or debt: writer acknowledgement remains delivery evidence only, not persistence evidence.
- Public evidence: Commits `8a97a67` and `56422b0`; `artifacts/src/artifacts/runtime_node.py`, `artifacts/recording-qos.yaml`, `artifacts/Dockerfile`, `artifacts/tests/test_artifact_status_ros.py`, `artifacts/tests/test_rosbag_adapter.py`.

### EH-0046 Refresh unavailable Gazebo package pins

- Evidence state: Verified. Implemented, clean-built, image-tested, and reviewed.
- Problem or constraint: Ubuntu Noble no longer supplied curl `8.5.0-2ubuntu10.12`, causing an uncached Gazebo image build to fail with exit 123.
- Investigation: a full 777-package audit found exactly four unavailable entries: `curl`, `libcurl3t64-gnutls`, `libcurl4-openssl-dev`, and `libcurl4t64`.
- Decision or result: update only those four exact pins to `8.5.0-2ubuntu10.13`; do not loosen pinning or change installed-delta verification.
- Verification: all 777 pins resolved, 240 Gazebo tests passed with one skip, the uncached image build succeeded, and the image-owned provenance and package checks passed.
- Later correction or debt: standalone test-target building was unavailable because buildx was missing, so the unchanged image test was mounted read-only into the new runtime image.
- Public evidence: Commit `d52dff0`; `gazebo/harmonic-packages.lock`, `gazebo/Dockerfile`, `gazebo/tests/test_gazebo_image`.

### EH-0047 Align physical readiness and event durability

- Evidence state: Verified. Implemented and cross-boundary tested.
- Problem or constraint: Gazebo persisted `flight_exchange` evidence, but orchestration rejected it because its validator allowed only `{run_id, ready}`. Separately, the one-shot scenario event could be lost by late volatile readers.
- Investigation: static comparison of producer, validator, scorekeeper subscriber, recorder QoS, and tests exposed stale two-field fixtures and mismatched durability. The first correction accidentally gave score events a transient-local reader, incompatible with their volatile publisher.
- Decision or result: validate the exact physical exchange document and its progress/error invariants; request transient-local durability for scenario events; keep score events reliable and volatile in a separate profile.
- Verification: the focused suites passed 162 tests with four skips, and a Dockerized late subscriber replayed the pre-subscription scenario event and produced a 100-point score.
- Later correction or debt: best-effort physical streams and delayed contact delivery remained runtime risks at this point.
- Public evidence: Commits `3a69dc1` and `dc3a400`; `orchestration/src/orchestration/controller.py`, `orchestration/tests/test_controller.py`, `scorekeeper/src/drone_sim_scorekeeper/runtime_node.py`, `scorekeeper/src/drone_sim_scorekeeper/self_test.py`, `artifacts/recording-qos.yaml`, `artifacts/tests/test_rosbag_adapter.py`.

### EH-0048 Gate Phase 3 readiness on current Gazebo evidence

- Evidence state: Verified. Diagnosed, implemented, and reviewed.
- Problem or constraint: orchestration looked for the wrong Gazebo lifecycle node, deadline probing obscured that failure, and a later run published `READY` only 58 ms after artifact readiness, before `gazebo-ready` existed.
- Investigation: run reconstruction distinguished wrapper cleanup from a Gazebo crash and showed that a one-second status RPC timeout was a transient availability condition, not necessarily a fatal child failure.
- Decision or result: fix lifecycle discovery, latch artifact readiness, and publish Phase 3 `READY` exactly once only after valid current-run Gazebo exchange evidence. Preserve Phase 2's immediate readiness behavior.
- Verification: 689 tests passed with 12 skips; a focused 160-test boundary review confirmed the exact readiness and `RUNNING` order.
- Later correction or debt: status RPC timeout handling still needed an explicit bounded retry policy, added shortly afterward.
- Public evidence: Commits `1640584` and `c3ded50`; `orchestration/src/orchestration/runtime_node.py`, `orchestration/src/orchestration/controller.py`, `artifacts/src/artifacts/runtime_protocol.py`, and their focused tests.

### EH-0049 Gate Gazebo samples and coalesce contact at 20 Hz

- Evidence state: Verified. Implemented and tested.
- Problem or constraint: physical samples could escape before flight activation, while Gazebo's 1 kHz positive-only contact stream fed an aggregator designed for one 20 Hz truth sample.
- Investigation: production callback ordering showed that treating each contact callback as a complete 50 ms fact was invalid.
- Decision or result: allow readiness clock handling while gating camera, truth, and other physical samples until activation; discard queued 1 ms readiness samples; coalesce contact into bounded 50 ms epochs.
- Verification: the Gazebo adapter tests passed in ROS and on the host; production-shaped burst tests covered contact coalescing.
- Later correction or debt: later work replaced pre-run public clock relay with a private warmup and rebased public epoch.
- Public evidence: Commits `1c2a35b` and `8a413dd`; `gazebo/src/drone_sim_gazebo/ros_adapter/node.py`, `gazebo/src/drone_sim_gazebo/ros_adapter/aggregation.py`, `gazebo/src/drone_sim_gazebo/runtime/entrypoint.py`, and related Gazebo tests.

### EH-0050 Remove RC mode override and fix Jazzy acknowledgement flushing

- Evidence state: Verified. Two independent fixes implemented and tested.
- Problem or constraint: simulated RC channel 5 changed the vehicle from MAVLink-commanded GUIDED back to STABILIZE. Scorekeeper final acknowledgement flushing also used the unsupported Jazzy keyword `timeout_sec`.
- Investigation: the companion correctly detected the real mode regression; a production-shaped test reproduced the ROS `TypeError` for the acknowledgement call.
- Decision or result: set `FLTMODE_CH 0` for the MAVLink-only descent and call acknowledgement flushing with `timeout=Duration(seconds=5.0)`.
- Verification: 56 focused SITL tests and 28 scorekeeper tests passed.
- Later correction or debt: the scorekeeper image had to be rebuilt before the next production run.
- Public evidence: Commits `66d4c8b` and `ca1b292`; `ardupilot_sitl/params/descent.parm`, `ardupilot_sitl/tests/test_config.py`, `scorekeeper/src/drone_sim_scorekeeper/runtime_node.py`, `scorekeeper/tests`.

### EH-0051 Buffer one callback epoch without hiding loss

- Evidence state: Verified. Implemented and full-suite tested.
- Problem or constraint: truth and camera pairing each allowed only one unmatched item, so harmless ROS callback lead looked like dropped or misaligned evidence.
- Investigation: captured ordering showed truth could lead its camera pair by one 50 ms epoch and one camera stream could lead the other by one frame.
- Decision or result: use bounded FIFO state for the current item plus one lookahead. Pair only FIFO heads with exact IDs and timestamps. Reject a third unmatched item, duplicates, and regressing input instead of dropping or synthesizing data.
- Verification: the truth fix passed 248 tests with two skips; the camera fix passed 250 with two skips.
- Later correction or debt: the bound intentionally handles one callback epoch, not arbitrary backlog.
- Public evidence: Commits `36efe0c` and `446b41a`; `gazebo/src/drone_sim_gazebo/ros_adapter/aggregation.py`, `gazebo/src/drone_sim_gazebo/ros_adapter/model.py`, `gazebo/src/drone_sim_gazebo/ros_adapter/live.py`, and their tests.

### EH-0052 Stop treating servo retransmission as peer loss

- Evidence state: Verified. Diagnosed, fixed, and tested.
- Problem or constraint: the ArduPilot wrapper aborted on a servo resend warning caused by one wall-time receive interval, even though lockstep physics was still advancing under heavy slowdown.
- Investigation: post-`RUNNING` evidence showed Gazebo reached 0.315 simulated seconds and enabled both cameras after the wrapper had already classified the warning as fatal.
- Decision or result: preserve the warning in structured logs but stop using it as proof of Gazebo failure. Retain real child failures and the run-level stall deadline as the fail-closed paths.
- Verification: 25 ArduPilot tests, compilation, and diff checks passed.
- Later correction or debt: a separate queue-draining flaw later proved that some slow-host runs could still lose actuator frames.
- Public evidence: Commit `ebfcee6`; `ardupilot_sitl` wrapper code, tests, and `docs/technical-debt/vertical-slice-hardening.md`.

### EH-0053 Separate transport readiness from mission readiness

- Evidence state: Verified. Implemented, corrected, and tested.
- Problem or constraint: waiting for a heartbeat before declaring infrastructure readiness deadlocked paused startup, while treating a heartbeat as arm readiness was also unsafe.
- Investigation: pinned Copter could establish MAVLink TCP and emit a heartbeat before completing vehicle initialization. An initial timeout change also applied `startup_wall_seconds` too broadly to heartbeat waiting.
- Decision or result: define startup readiness as verified MAVLink TCP transport; keep mission commands behind a later real heartbeat and health gate. Restrict `startup_wall_seconds` to TCP discovery and use `max_wall_seconds` for post-connection heartbeat waiting. Retry unavailable Gazebo status probes within the original startup deadline, while malformed replies and child failures remain fatal.
- Verification: 210 scoped tests passed for the readiness split, 255 Gazebo tests passed with two skips for status retry, and 42 companion tests passed after deadline correction.
- Later correction or debt: passive prearm telemetry and a private warmup phase were still needed.
- Public evidence: commits `3126a3f`, `c2da292`, `d2ef9cc`, `5f7a06d`, and `bb3b9f5`; `companion/src/drone_sim_companion/runtime_node.py`; `companion/src/drone_sim_companion/lifecycle.py`; `gazebo/src/drone_sim_gazebo/runtime/entrypoint.py`; `orchestration/src/orchestration/controller.py`; related docs and tests.

### EH-0054 Preserve the first running lockstep exchange

- Evidence state: Verified. Root cause confirmed, implemented, and tested.
- Problem or constraint: while paused, Gazebo consumed servo frames 0 and 1 but replied only to frame 0. SITL waited for frame 1's JSON while the unpaused plugin waited for frame 2.
- Investigation: an earlier bounded-bootstrap change still allowed frame 1 to disappear into paused readiness.
- Decision or result: consume only frame 0 while paused, leave frame 1 for the first running step, and define readiness as one completed servo/JSON exchange.
- Verification: 433 tests passed with two skips, and the downstream plugin patch applied cleanly to the pinned revision.
- Later correction or debt: startup exchange counters prove the boundary handshake, not one-for-one actuator exchange for the full flight.
- Public evidence: commits `945a3d8`, `fba46fc`, and `58f6e03`; `gazebo/plugin/0001-paused-initial-json.patch`; `gazebo/src/drone_sim_gazebo/runtime/entrypoint.py`; `artifacts/src/artifacts/runtime_protocol.py`; `orchestration/src/orchestration/controller.py`; focused tests.

### EH-0055 Repair calibration, finalization, and completion races

- Evidence state: Verified. Three independent fixes implemented and tested.
- Problem or constraint: normal ARM failed because the minimal parameter overlay omitted SITL accelerometer calibration markers; artifact finalization waited only for a ROS callback despite an authoritative durable request; queued DDS callbacks could enter an already frozen Gazebo adapter.
- Investigation: the exact ARM result reproduced `3D Accel calibration needed`. Two failed runs never started recorder finalization. Completion testing reproduced `adapter is frozen` from queued image, contact, and odometry callbacks.
- Decision or result: add upstream calibration values without weakening arming checks, poll and latch the durable finalization request, and deactivate physical input before freezing the adapter.
- Verification: 26 SITL tests passed and the image rebuilt; 465 artifact tests passed with 12 skips; Gazebo ROS and model tests passed for post-completion callbacks.
- Later correction or debt: successful image startup still did not imply the flight controller had finished application initialization.
- Public evidence: Commits `5f3e67a`, `0d805b7`, and `044827c`; `ardupilot_sitl/params/descent.parm`, `artifacts/src/artifacts/runtime_node.py`, `gazebo/src/drone_sim_gazebo/ros_adapter/node.py`, docs, and focused tests.

### EH-0056 Preserve frame-aligned physical evidence for 60 seconds

- Evidence state: Verified. Implemented and verified on an aborted physical run.
- Problem or constraint: a 30-second run ended before cold ArduCopter initialization, and best-effort shallow queues lost 129 of 600 expected frame timestamps and 101 ground-truth samples.
- Investigation: `/clock` remained monotonic but had random 1 to 41 ms gaps. After reliability changes, an aborted run retained every complete pair, with its one-frame asymmetry explained by aborting mid-pair.
- Decision or result: extend the production descent to 60 simulated seconds; use reliable ground truth; use reliable depth-1000 clock transport and recording; emit monotonic frame-correlated public clock samples.
- Verification: the follow-up recorded 470 unique clocks, 468 onboard pairs, 467 observer pairs, and 467 matching ground-truth samples with no missing clock timestamp.
- Later correction or debt: later private-warmup work changed how the public clock begins, but kept the exact 50 ms evidence grid.
- Public evidence: Commits `2b917ff` and `c7d32d6`; `config/default-run.json`, `artifacts/recording-qos.yaml`, `gazebo/config/bridge-flight.yaml`, `gazebo/src/drone_sim_gazebo/ros_adapter/node.py`, and related docs/tests.

### EH-0057 Require healthy prearm state and retain MAVLink diagnostics

- Evidence state: Verified. Diagnosed from pinned source and implemented.
- Problem or constraint: the first heartbeat arrived only 172 ms before ARM, and Copter rejected the command with `Arm: System not initialised`.
- Investigation: packet and source analysis disproved wrong UDP direction, wrong reply port, peer rejection, and packet-loss theories. Sequential frame counters and live traffic showed valid Gazebo-to-SITL exchange; the vehicle was simply initializing at roughly 0.05 real time.
- Decision or result: treat heartbeat as liveness only. Require `MAV_SYS_STATUS_PREARM_CHECK` to be enabled and healthy before ARM, and retain raw status masks and `STATUSTEXT` diagnostics. Keep replies on the actual ephemeral SITL source socket.
- Verification: isolated replay reproduced the rejection, while live counters later showed about 58 bidirectional datagrams per second with zero socket errors and a heartbeat at simulation time 49.
- Later correction or debt: a structured log field named `severity` collided with the logger's common field and was fixed separately.
- Public evidence: Commits `7149efb`, `828e643`, and `edca887`; `companion/src/drone_sim_companion/controller.py`, `companion/src/drone_sim_companion/mavlink_adapter.py`, `companion/src/drone_sim_companion/runtime_node.py`, `companion/src/drone_sim_companion/mission.py`, and tests/docs.

### EH-0058 Add a private warmup and durable mission-ready gate

- Evidence state: Verified. Designed, implemented across modules, and reviewed.
- Problem or constraint: an 18-minute slow-host controller boot could not fit an infrastructure deadline, but exposing its native warmup time would corrupt the fixed public scoring epoch.
- Investigation: redefining `companion-ready` would mix transport and mission semantics; resetting Gazebo would disturb the warmed estimator state. The first companion-only implementation also revealed that the shared protocol did not yet permit `mission-ready`.
- Decision or result: keep `companion-ready` as TCP readiness; latch heartbeat and healthy prearm independently; persist `mission-ready` exactly once; hold mission commands until `RUNNING` and the first public clock; add the exact status to the shared protocol and orchestration gate.
- Verification: 50 companion tests and 66 protocol integration tests passed; cross-task review confirmed the shared status addition resolved the production `ValueError`.
- Later correction or debt: pinned Copter did not emit passive `SYS_STATUS` by default, so the gate could still deadlock.
- Public evidence: Commits `d76d486`, `92cfc37`, and `d3f7c21`; `companion/src/drone_sim_companion/controller.py`, `companion/src/drone_sim_companion/lifecycle.py`, `companion/src/drone_sim_companion/runtime_node.py`, `artifacts/src/artifacts/runtime_protocol.py`, `orchestration/src/orchestration/status_store.py`, `orchestration/src/orchestration/runtime_node.py`, and interface docs/tests.

### EH-0059 Publish passive prearm status during warmup

- Evidence state: Verified. Implemented and tested.
- Problem or constraint: Copter's extended-status stream defaulted to zero. Because active stream requests correctly waited for the public run, passive warmup never received `SYS_STATUS` and could not write `mission-ready`.
- Investigation: tests that injected a healthy telemetry object masked the production dependency.
- Decision or result: set `MAV1_EXT_STAT 1` to emit passive 1 Hz status during warmup without changing arming checks or unrelated stream rates.
- Verification: the new config test first failed because the parameter was absent, then all 27 SITL tests passed.
- Later correction or debt: end-to-end flight still had to prove the passive message arrived on the pinned runtime image.
- Public evidence: Commit `3d1a631`; `ardupilot_sitl/params/descent.parm`, `ardupilot_sitl/tests/test_config.py`, `ardupilot_sitl/EXTERNAL_INTERFACE.md`.

### EH-0060 Rebase public evidence after private SITL warmup

- Evidence state: Verified. Implemented, reviewed, and corrected for callback races.
- Problem or constraint: warmup had to advance Gazebo and SITL without publishing public scoring data, then start an exact zero-based 60-second interval.
- Investigation: the first epoch snapshot could use a stale native clock because queued DDS camera and clock callbacks were independently scheduled. It also allowed clock publication after 60 seconds or completion.
- Decision or result: `READY` unpauses private warmup; `RUNNING` activates output without a second unpause. `PublicEpoch` and `OutputEpochGate` floor the native boundary to 50 ms, publish public clock zero, drop pre-epoch samples, and rebase all physical timestamps. A post-request barrier waits for both camera watermarks and a clock reaching their greatest stamp. Clock mapping stops at the configured horizon and after completion.
- Verification: the initial full Gazebo suite passed 263 tests with two skips; the corrected suite passed 265 with two skips, plus live ROS verification in the runtime image.
- Later correction or debt: the chosen native activation boundary could still differ between runs because it followed readiness timing rather than an absolute native target.
- Public evidence: Commits `1ce2c93` and `7feb0c5`; `gazebo/src/drone_sim_gazebo/ros_adapter/epoch.py`, `gazebo/src/drone_sim_gazebo/ros_adapter/node.py`, `gazebo/src/drone_sim_gazebo/runtime/model.py`, `gazebo/src/drone_sim_gazebo/runtime/entrypoint.py`, `gazebo/PLAN.md`, and focused tests/docs.

### EH-0061 Preserve sequential actuator frames under host contention

- Evidence state: Verified. Diagnosed, implemented, and reviewed against pinned plugin behavior.
- Problem or constraint: a severely contended run advanced only 284 ms of simulation in about 367 wall seconds. The plugin drained queued UDP actuator packets and kept only the newest, so `[F+1,F+2,F+3]` collapsed to `F+3` and skipped control frames.
- Investigation: zero-length UDP queues were initially read as deadlock evidence, but Gazebo was still advancing at about 0.00077 real time. Real-plugin RED tests observed frame 3 instead of frame 1 and hid queued duplicates.
- Decision or result: remove the queue-drain block so each receive consumes one datagram. Let the existing `PreUpdate` loop consume duplicates and stop after the first sequential or accepted gap packet.
- Verification: host Gazebo tests passed, the patch applied to the pinned plugin revision, and review confirmed one motor update per controlled step. Real-image GREEN required a rebuilt image.
- Later correction or debt: removing the drain exposed duplicate-recovery positive feedback.
- Public evidence: Commits `5324c09` and `329b040`; `gazebo/plugin/0001-paused-initial-json.patch`, `gazebo/tests/test_plugin_udp.py`, `gazebo/tests/test_gazebo_image`, `gazebo/tests/test_flight_resources.py`, `gazebo/provenance/ardupilot_gazebo-plugin.json`, and technical-debt/docs files.

### EH-0062 Bound duplicate-recovery feedback

- Evidence state: Verified as an open state. Implemented and reviewed; runtime-image verification remained pending at the time.
- Problem or constraint: after queue draining was removed, every duplicate in a contiguous burst sent recovery JSON. Each response prompted SITL to generate another frame, creating positive feedback.
- Investigation: the discriminating RED case consumed the intended five datagrams but produced six JSON packets where three were expected.
- Decision or result: add one recovery flag per contiguous duplicate burst. Send one recovery JSON, reset after an empty receive or an accepted sequential/gap frame, and retain forward-gap diagnostics.
- Verification: the full host Gazebo suite passed 266 tests with seven skips, the downstream patch applied cleanly, and review confirmed timeout rearming and no loss of the next sequential packet.
- Later correction or debt: the real plugin tests still needed the rebuilt image.
- Public evidence: commits `9c2b8d3` and `cdbdc0d`; `gazebo/plugin/0001-paused-initial-json.patch`; `gazebo/tests/test_plugin_udp.py`; `gazebo/tests/test_flight_resources.py`; `gazebo/provenance/ardupilot_gazebo-plugin.json`; Gazebo interface docs.

### EH-0063 Bind acceptance to exact simulation evidence and trusted provenance

- Evidence state: Verified. Implemented, corrected, and tested.
- Problem or constraint: a one-frame physical bag at timestamp zero passed because validation checked count and cadence but not the required public epoch. Provenance also accepted arbitrary nonempty source/image records and later still accepted seven unique substituted digests.
- Investigation: production-like fixtures demonstrated both false acceptances. Name/count validation alone did not establish provenance authority.
- Decision or result: require `/clock` from zero through configured duration; exact 50 ms camera/truth grids; lifecycle and manifest timing bound to the same epoch; exactly one `drone_sim` source and seven production image names; externally supplied exact revision, dirty state, and digest map through direct, container, CLI, and Make entry points.
- Verification: artifact suites progressed from 476 to 480 passing tests with 12 environment skips; adjacent scorekeeper/orchestration suites passed 259 tests; substitution, omission, and mismatch cases failed as intended.
- Later correction or debt: launch-time source/image capture remained an external operator responsibility, and exact world, vehicle, mission, scenario, seed, and RTF were not yet externally pinned.
- Public evidence: Commits `715f781` and `dc111e8`; `artifacts/src/artifacts/_adapters/rosbag.py`, `artifacts/src/artifacts/acceptance.py`, `artifacts/tests/test_acceptance.py`, `artifacts/tests/test_rosbag_adapter.py`, `artifacts/EXTERNAL_INTERFACE.md`, `Makefile`.

### EH-0064 Request landed telemetry explicitly and tune the first scored landing

- Evidence state: Verified. Implemented and ready-run reviewed.
- Problem or constraint: `MAV_DATA_STREAM_ALL` did not make ArduPilot publish `EXTENDED_SYS_STATE`, so landed state remained unknown and mission completion never triggered. The default 0.5 m/s landing speed also exceeded the 0.1 m/s stable-contact ceiling.
- Investigation: pinned MAVLink behavior confirmed command 511 can request message 245. Flight logs tied the default parameter to roughly 0.49 m/s pre-contact descent.
- Decision or result: request `EXTENDED_SYS_STATE` at 10 Hz with `MAV_CMD_SET_MESSAGE_INTERVAL`; initially set `LAND_SPD_MS 0.05`; raise physical camera publisher and recorder histories to 100 while retaining Phase 2 depth 5.
- Verification: companion tests passed 50; SITL tests passed 28; the exact wire tuple, QoS values, and parameter contract passed review.
- Later correction or debt: `LAND_SPD_MS 0.05` was below the advisory metadata range and needed a physical run. A nested-Make harness issue was recorded as non-production debt.
- Public evidence: Commit `6901542`; `companion/src/drone_sim_companion/mavlink_adapter.py`, `ardupilot_sitl/params/descent.parm`, `artifacts/recording-qos.yaml`, `config/recording-qos.yaml`, `gazebo/src/drone_sim_gazebo/ros_adapter/node.py`, tests, interface docs, and `docs/technical-debt/vertical-slice-hardening.md`.

### EH-0065 Fix physical video loss and powered landing rebound

- Evidence state: Verified. Diagnosed with production evidence, corrected, and integrated.
- Problem or constraint: the first scored run lost onboard metadata because the artifact recorder's reliable depth-5 queue evicted unread samples during executor lag. The same run scored 80 because a gentle first contact was followed by powered rebound.
- Investigation: MCAP proved Gazebo published the missing metadata, while the recorder retained exactly the newest five samples. A real DDS A/B test reproduced loss at depth 5 and no loss at depth 100. For landing, an initial `PSC_D_ACC_IMAX 0.01` proposal was rejected after pinned source showed the controller raises that bound to at least hover throttle.
- Decision or result: use recorder depth 100 only for physical runs, preserve Phase 2 depth 5, and change `LAND_SPD_MS` from 0.05 to 0.10 so the post-contact position target moves beneath the floor faster.
- Verification: the artifact suite passed 482 tests with 12 skips; SITL tests passed 28; review confirmed docs and both QoS profiles.
- Later correction or debt: landing tuning remained empirical and could only be judged by another full run.
- Public evidence: Commit `03a8cf8`; `artifacts/src/artifacts/runtime_node.py`, `artifacts/tests/test_runtime_node.py`, `artifacts/EXTERNAL_INTERFACE.md`, `ardupilot_sitl/params/descent.parm`, `ardupilot_sitl/tests/test_config.py`, `ardupilot_sitl/EXTERNAL_INTERFACE.md`.

### EH-0066 Complete the first 100-point physical descent

- Evidence state: Verified. Completed and independently rescored.
- Problem or constraint: the vertical slice required a complete mission, maximum score, both 1,200-frame videos, exact 20 Hz physical evidence, complete logs, and clean teardown.
- Investigation: earlier runs failed startup, finalization, video retention, landed-state detection, or stable contact. The successful configuration incorporated the accumulated readiness, warmup, QoS, plugin, telemetry, and landing fixes.
- Decision or result: the run completed with `mission_complete` and scored 100/100. First contact occurred at 21.85 s with 0.04358 m/s 3D speed and 0.11185 degrees tilt. All 11 stable-contact samples passed; the window maxima were 0.04358 m/s and 0.63050 degrees.
- Verification: independent repository scoring also returned 100/100; videos, MCAP, native state, structured logs, configuration, checksums, and cleanup were inspected in the following acceptance pass.
- Later correction or debt: a single winning run did not establish deterministic repeatability.
- Public evidence: Commit `03a8cf8`; `runs/` evidence as summarized in `docs/verification`, `scorekeeper/rules/descent_v1.json`, and the acceptance paths under `artifacts/src/artifacts`.

### EH-0067 Align acceptance with the implemented lifecycle

- Evidence state: Verified. Stale contract diagnosed, fixed, and replay-tested.
- Problem or constraint: the complete 100-point bundle failed acceptance because the validator still required `RequestSteps`, although production had replaced the old lifecycle with `READY → SetPaused(false)` and `RUNNING → ActivateOutput`.
- Investigation: no production path constructed `RequestSteps`; it remained a transport/test capability. Adding a fake log event would have made the bundle less truthful. Review then found that reducing actions to an unordered set also allowed reversed lifecycle logs.
- Decision or result: require `ActivateOutput` and enforce the ordered lifecycle subsequence while allowing the later completion pause. Add a regression proving legacy `RequestSteps` cannot substitute for output activation.
- Verification: 25 focused acceptance tests passed, and the frozen successful log replay passed the exact subsequence check.
- Later correction or debt: `ActivateOutput` plus the complete public grid proves public epoch execution, but not full-run one-for-one actuator lockstep by itself.
- Public evidence: Commit `52daec2`; `artifacts/src/artifacts/acceptance.py`, `artifacts/tests/test_acceptance.py`, `artifacts/EXTERNAL_INTERFACE.md`, `docs/technical-debt/vertical-slice-hardening.md`.

### EH-0068 Diagnosed nondeterministic flight timing and fixed the public epoch

- Evidence state: Superseded, then verified.
- Problem or constraint: A valid 100-point descent did not repeat under slowdown. Private warmup ended at different native times, and wall-scheduled command callbacks landed on different physics iterations.
- Investigation: A complete, aligned repeat scored 80/100 after a harder contact. An absolute epoch removed warmup-history variance, but the first constrained fixed-epoch run still bounced. Retagging timestamps was rejected because it would change evidence, not physics.
- Decision or result: Add an exact Decimal-parsed 90-second native epoch on the 50 ms grid. Keep warmup private, stop at that target, publish public zero, and gate the first GUIDED handoff before release.
- Verification: Different warmups converged on the same target in tests. A normal live run independently scored 100/100. The constrained 80-point counterexample drove the pause correction recorded in [EH-0069](#eh-0069-required-confirmed-pause-before-exact-run-to-and-defined-command-evidence).
- Later correction or debt: The rendezvous aligned GUIDED only; later commands could still differ by one physics iteration.
- Public evidence: commit `070cc20`; `orchestration/src/orchestration/config.py`; `gazebo/src/drone_sim_gazebo/ros_adapter/epoch.py`; `companion/src/drone_sim_companion/runtime_node.py`; [runnable MVP verification](../verification/runnable-mvp.md); [timing debt](../technical-debt/vertical-slice-hardening.md).

### EH-0069 Required confirmed pause before exact run-to and defined command evidence

- Evidence state: Superseded, then verified.
- Problem or constraint: WorldControl acknowledged enqueueing, not completed pause application. An absolute run-to issued too early could overshoot. A local MAVLink write also could not prove ArduPilot receipt or application.
- Investigation: The first live attempt crossed the target and reached 91.803 seconds. Source analysis separated local handoff, later `COMMAND_ACK`, and controller application.
- Decision or result: Wait for authoritative `paused=true`, run to the absolute target, publish public zero, persist a transport-queued command fact, then release the world. Keep command acknowledgement separate.
- Verification: Corrected images passed 150 focused and 199 wider tests. Live evidence showed native 90, public zero, durable command handoff, then unpause.
- Later correction or debt: Transport-queued evidence does not claim ArduPilot acceptance. Exact later-command scheduling remained deferred.
- Public evidence: commit `070cc20`; `gazebo/src/drone_sim_gazebo/runtime/runtime_node.py`; `gazebo/tests/test_runtime_node.py`; `companion/src/drone_sim_companion/runtime_node.py`; `artifacts/src/artifacts/runtime_protocol.py`.

### EH-0070 Accepted paired normal and constrained vertical descents

- Evidence state: Verified.
- Problem or constraint: The vertical-descent MVP required repeatability under normal and constrained execution.
- Investigation: The earlier constrained failure led to the fixed epoch, zero-time rendezvous, and confirmed-pause ordering. No scoring tolerance changed.
- Decision or result: Accept the corrected pair as the runnable vertical-descent MVP.
- Verification: Each run produced 1,200 frames per camera over 60 simulated seconds and independently scored 100/100. The regression pass reported 564 passed and 8 environment skips, with no run-scoped resources left.
- Later correction or debt: This evidence covered vertical descent, not the later competition mission.
- Public evidence: commit `070cc20`; [runnable MVP verification](../verification/runnable-mvp.md); `artifacts/tests/test_acceptance.py`; `orchestration/tests/test_controller.py`.

## 2026-08-26

### EH-0071 Retained lifecycle history and approved competition configuration

- Evidence state: Verified. Committed and test-verified, with a documented weakness in the TOCTOU regression.
- Problem or constraint: The competition run needed immutable course and scenario inputs, exact approved settings, a usable default CLI path, and a complete retained lifecycle under callback load. Validation, hashing, and snapshotting could not disagree about which bytes they had read, while depth-one durable run state could collapse READY and RUNNING into one retained sample.
- Investigation: The first configuration implementation passed focused tests but read the source separately for validation and hashing. A review identified the replacement window. The follow-up production code switched to one byte read, but its regression monkeypatched `Path.read_text` after production had moved to `read_bytes`, so the test did not exercise the intended mutation point. Separately, load testing showed the run-state history needed to retain all four lifecycle values rather than only the newest one.
- Decision or result: Validate, hash, and snapshot the same in-memory byte payload. Restrict 640 by 480 recording to `comp2026_auto`, preserve 320 by 240 for earlier missions, and compare typed scalar values so booleans cannot impersonate integers. Raise the reliable transient-local run-state depth to four and make a retained RUNNING fact imply the prior READY transition for the synthetic consumer.
- Verification: The configuration boundary suite reached 119 passing tests, the wider orchestration suite reached 256, and the approved profile hash remained stable. The lifecycle change added a load regression and kept runtime and recorder QoS aligned.
- Later correction or debt: Snapshot files were created exclusively and fsynced, but the multi-file snapshot was not transactional. The TOCTOU production design used one read; the regression itself remained weaker than claimed.
- Public evidence: commits `c06c930`, "Retain complete runtime lifecycle under load", `4e01d63`, "Add competition runtime configuration", and `70b3299`, "Bind competition hashes to validated bytes"; `config/default-run.json`; `config/run.schema.json`; `config/run-template.schema.json`; `orchestration/src/orchestration/config.py`; `orchestration/src/orchestration/runtime_node.py`; `orchestration/tests/test_config.py`; `artifacts/recording-qos.yaml`.

### EH-0072 Payload mission ROS contracts

- Evidence state: Verified. Committed and verified in a ROS Jazzy container build.
- Problem or constraint: The competition runtime had no typed ROS contract for physical payload truth, mission phase evidence, or attach and release commands.
- Investigation: Contract checks first failed because three messages and one service were absent. The existing interface package and generator were reused instead of creating a second transport boundary.
- Decision or result: Add `PayloadState.msg`, `PayloadEvent.msg`, `MissionEvent.msg`, and `PayloadCommand.srv`, with `ATTACH=1` and `RELEASE=2` service constants.
- Verification: The Jazzy build generated all interfaces successfully. Only an existing CMake warning remained.
- Later correction or debt: This entry established wire contracts only. Physical authority, policy, and scoring followed in later entries.
- Public evidence: commit `a3cf00d`, "Add payload mission ROS contracts"; `ros_ws/src/simulation_interfaces/CMakeLists.txt`; `ros_ws/src/simulation_interfaces/msg/PayloadState.msg`; `ros_ws/src/simulation_interfaces/msg/PayloadEvent.msg`; `ros_ws/src/simulation_interfaces/msg/MissionEvent.msg`; `ros_ws/src/simulation_interfaces/srv/PayloadCommand.srv`; `tests/contracts/test_ros_interfaces.py`.

### EH-0073 Physical competition scene

- Evidence state: Verified. Committed, container-built, and exercised against a real Gazebo server.
- Problem or constraint: The scene needed deterministic geometry, exactly one vehicle hardpoint, payload 2 attached at startup, payloads 3 and 4 detached without a transient snap, and physical truth suitable for scoring.
- Investigation: Stock Gazebo `DetachableJoint` always attached on its first update. A startup-detach workaround was rejected because it allowed a transient attachment. The first repository-wide patch also broke unrelated Docker `COPY` inputs with an over-narrow allowlist, allowed three simultaneous attachments, placed detached payloads 10 mm into pads, and omitted PyYAML.
- Decision or result: Vendor the Gazebo 8.11 plugin with a narrow `initially_attached` addition, add `exclusive_parent` arbitration, raise detached payload centers, declare PyYAML, and replace the Docker allowlist with deny-oriented filtering. The coordinator kept blocked attaches pending until the occupied joint disappeared.
- Verification: The real-server test proved startup exclusivity, duplicate suppression, and delayed attachment after removal. The final suite reported 281 passed and 8 skipped; CTest, asset generation, image build, and runtime smoke passed.
- Later correction or debt: Strict integer coercion in Gazebo asset configuration remained deferred. The Docker context stayed near 48 MB without excluding other image inputs.
- Public evidence: commits `f534c78`, "Add physical competition scene", and `b83ec08`, "Fix competition payload physical contracts"; `gazebo/plugin/PayloadCommandCoordinator.cc`; `gazebo/plugin/vendor/gz_sim_8_11_0/DetachableJoint.cc`; `gazebo/resources/worlds/competition_mission.sdf`; `gazebo/resources/models/iris_competition/model.sdf`; `gazebo/scripts/prepare_competition_assets.py`; `gazebo/tests/test_competition_assets.py`.

## 2026-08-27

### EH-0074 Payload state exact joins

- Evidence state: Verified. Committed and verified with Python, CTest, full-suite, real-rclpy, and production-plugin checks.
- Problem or constraint: Pose, contact, and joint facts arrived on separate callbacks. The adapter could not fabricate freshness, depend on callback order, or create invalid ROS names such as `/payload/2`.
- Investigation: Installed-image testing exposed the invalid ROS aliases. Review then found that untimestamped joint strings fabricated freshness, early ticks could leave gaps, and nonfinite derived velocity was unguarded. A later review found that the occupied-hardpoint return path stopped recurrent detached truth.
- Decision or result: Keep Gazebo transport names but publish ROS aliases such as `/payload_2`. Publish joint state as recurrent 20 Hz, simulation-stamped level truth; join all three facts at exact timestamps; reject stale, reordered, missing, and nonfinite inputs; and publish detached truth before returning from blocked attach handling.
- Verification: The initial adapter reached 303 tests. The corrected implementation passed Python tests, CTest, the full suite, a real rclpy smoke test, and the production CTest path.
- Later correction or debt: Live cadence validation was deferred to later end-to-end work.
- Public evidence: commits `a3d8a21`, "Publish competition payload state", `4bee1ae`, "Fix payload fact joining", and `99b5253`, "Keep joint truth recurrent while occupied"; `gazebo/config/bridge-competition.yaml`; `gazebo/src/drone_sim_gazebo/ros_adapter/payload.py`; `gazebo/src/drone_sim_gazebo/ros_adapter/node.py`; `gazebo/plugin/PayloadCommandCoordinator.cc`; `gazebo/tests/test_adapter_node.py`.

### EH-0075 Serialized payload authority

- Evidence state: Verified. Committed and verified with focused and full tests plus a runtime image build.
- Problem or constraint: The prior `descent_v1` path was inert. Competition commands had to validate run identity, phase, zone, grounded state, capacity, position, correlation, and physical confirmation while remaining safe under concurrent requests.
- Investigation: The first implementation handled the physical checks but allowed non-atomic duplicate replay, concurrent distinct commands, old facts overwriting newer state, mixed-tick authorization, recurrent truth freezing, and more than one simultaneous attachment.
- Decision or result: Serialize service operations with a dedicated mutex while keeping state and result callbacks on a separate lock. Require monotonic, tick-coherent physical state and fail closed on mixed ticks, multiple attachments, malformed results, or five-second confirmation timeout.
- Verification: Focused and full test suites passed, and the corrected runtime image was built.
- Later correction or debt: Commands had no retry protocol at this point, and confirmations arriving after five seconds were ignored.
- Public evidence: commits `3abd8dd`, "Add physical payload authority", and `468b7fa`, "Serialize payload authority operations"; `electromagnet/src/drone_sim_electromagnet/controller.py`; `electromagnet/src/drone_sim_electromagnet/payload.py`; `electromagnet/src/drone_sim_electromagnet/runtime_node.py`; `electromagnet/tests/test_runtime_node.py`.

### EH-0076 Independent physical scoring

- Evidence state: Verified. Committed and verified with 70 focused competition tests at the final correction.
- Problem or constraint: Scores had to come from timestamped vehicle and payload truth. Mission and payload events could anchor phases and timestamps but could not prove physical success by themselves.
- Investigation: Early attach continuity used the wrong interval. Review then showed that prestart truth, stale carry, gaps, wrong pickup locations, weak settlement ordering, and invalid Home sequences could score. A later review found that the first qualifying Home landing masked later contradictory truth and that a valid 145-point attempt could not finalize without payload 4.
- Decision or result: Require phase-gated continuous 50 ms traces, exact pickup zones, three-dimensional continuity, strict release freshness, persistent Home validity through disarm and completion, and terminal validity separate from payload-4 points.
- Verification: The first ruleset and lifecycle implementation passed pure scorer tests. The integrity correction passed 67 full and 17 descent tests, and the terminal correction finished at 70 tests.
- Later correction or debt: A self-review narrowed an overbroad phase gate to the exact next checkpoint before the work was accepted.
- Public evidence: commits `7636c4b`, "Add physical competition scoring", `0253ab7`, "Harden competition scoring integrity", and `cae07bd`, "Fix competition terminal scoring"; `scorekeeper/rules/competition_v1.json`; `scorekeeper/src/drone_sim_scorekeeper/competition.py`; `scorekeeper/src/drone_sim_scorekeeper/competition_runtime.py`; `scorekeeper/tests/test_competition_score.py`; `scorekeeper/tests/test_competition_runtime.py`.

### EH-0077 Hosted the external mission behind parent lifecycle gates

- Evidence state: Verified. Parent-repository implementation committed and verified; nested mission internals are outside this slice.
- Problem or constraint: The original nested mission needed to run inside the existing seven-service parent stack without replacing the descent selector. Arming required live RUNNING state, clock, paired frame, range, heartbeat, armability, and payload service readiness.
- Investigation: The first host latched readiness permanently, allowed a callback failure to race with terminal success, admitted non-runtime nested files into Docker, and could report quiescence while worker and executor producers still ran. Re-review found range freshness sampled too early and another success-versus-exception race.
- Decision or result: Re-evaluate live predicates atomically, sample range last under the gate lock, serialize terminal claims and accepted failures, cancel on fatal callbacks, stop producers before declaring quiescence, and narrow the Docker runtime closure.
- Verification: The first lifecycle correction passed 40 focused and 72 full tests. The final ordering fix passed 43 focused and 75 full tests.
- Later correction or debt: The parent runtime installed the nested mission's `future` dependency and Python 3.12 compatibility hook. Broader nested hardware packaging remained separate.
- Public evidence: commits `867061c`, "Document comp2026 runtime integration", `b8c6d3b`, "Plan comp2026 runtime integration", `a89f8bd`, "Host original comp2026 mission", `a270967`, "Fix comp2026 host lifecycle gates", and `7ebb33b`, "Fix comp2026 host race ordering"; `companion/Dockerfile`; `companion/src/drone_sim_companion/comp2026_host.py`; `companion/src/drone_sim_companion/runtime_node.py`; `companion/src/sitecustomize.py`; `companion/tests/test_comp2026_host.py`; `companion/tests/test_runtime_node.py`.

### EH-0078 Canonical competition evidence

- Evidence state: Verified. Committed and repository-suite verified.
- Problem or constraint: Acceptance needed to bind parent and nested source identities, image digests, the downward-range stream, camera geometry, QoS, physical score, and a shared deadline without trusting the scorekeeper alone.
- Investigation: Initial full tests exposed stale doubles and one isolated 9.6 ms timing failure; a cold Gazebo build also hit a transient embedded CTest timeout. Review found the independent oracle could accept invalid pickup and capacity histories and that the writer could certify an injected inspector result. The first correction left a public `inspector=` injection path.
- Decision or result: Add an independent physical oracle, require exact post-transition pickup truth and simultaneous capacity, canonicalize inspector and digest handling, and remove the public inspector injection.
- Verification: Isolated timing retries passed without a production timing change. The initial integration reached 1,263 passed with 20 skipped, and the hardened path reached 1,266 passed with 20 skipped.
- Later correction or debt: A wall-clock artifact test failed once at 146 ms against a 90 ms threshold and passed on isolated and full retry. No unrelated timing threshold was weakened.
- Public evidence: commits `22760bf`, "Wire competition runtime evidence", `208072a`, "Harden competition evidence validation", and `bc1a15b`, "Remove verification inspector bypass"; `artifacts/src/artifacts/acceptance.py`; `artifacts/src/artifacts/competition_score_validation.py`; `artifacts/recording-qos.yaml`; `scripts/write_competition_verification.py`; `artifacts/tests/test_competition_score_validation.py`.

### EH-0079 First live contact fixes

- Evidence state: Verified. Failed live attempt followed by two committed parent fixes.
- Problem or constraint: Compose was healthy, but the mission produced no events and scored zero because readiness never completed.
- Investigation: The first hypothesis blamed a connection or public-epoch race. Inspection instead found that configured private contact topics did not match Gazebo's canonical names. After fixing discovery, the scenario event type also needed exposure at the electromagnet boundary.
- Decision or result: Correct competition payload contact discovery and expose the scenario event type used by the runtime.
- Verification: Later attempts crossed the original readiness blockage, although startup performance and DroneKit readiness still failed independently.
- Later correction or debt: These fixes removed two concrete blockers but did not make the mission complete.
- Public evidence: commits `316f81f`, "Fix competition payload contact discovery", and `fbf220d`, "Expose competition scenario event type"; `gazebo/config/bridge-competition.yaml`; `gazebo/src/drone_sim_gazebo/ros_adapter/topics.py`; `gazebo/tests/test_runtime_node.py`; `electromagnet/src/drone_sim_electromagnet/runtime_node.py`.

### EH-0080 Live readiness and warmup budgets

- Evidence state: Verified. Committed after live failures and targeted measurements.
- Problem or constraint: Attempts 2 and 3 timed out while the simulation advanced only about 0.767 seconds under CPU starvation. DroneKit parameter download blocked startup even though the fields required by the host were current.
- Investigation: Removing the parameter wait cleared that deadlock, but attempt 4 then exceeded the fixed 60-second heartbeat limit. A standalone measurement needed about 28.305 simulated seconds to establish heartbeat. Further attempts showed the private physical warmup was still too short.
- Decision or result: Let the live host gate own readiness, make heartbeat startup budget configurable, and extend the competition startup and physical warmup ceilings without changing simulation-time mission deadlines.
- Verification: Each change allowed the next attempt to pass the specific prior gate and reach a later failure.
- Later correction or debt: Wall-clock allowances addressed slow execution, not physics or mission correctness.
- Public evidence: commits `e8e8b90`, "Defer competition readiness to live gate", `4969841`, "Use resolved heartbeat startup budget", `effab33`, "Allow full competition startup warmup", and `27d6560`, "Extend physical competition warmup"; `companion/src/drone_sim_companion/runtime_node.py`; `config/default-run.json`; `orchestration/tests/test_config.py`.

### EH-0081 Camera delivery and service order

- Evidence state: Verified. Committed and test-verified across repeated live attempts.
- Problem or constraint: Duplicate inherited and competition cameras amplified startup load. At public epoch, the adapter also saw camera timestamp and QoS mismatches, and one attempt hit a Compose DNS and start-order race.
- Investigation: Selecting a unique camera and using reliable depth-5 readers fixed distinct issues, but reliable camera subscriptions during private warmup overloaded attempts 10 and 11.
- Decision or result: Keep one competition camera source, match the reliable transport contract, order Gazebo before ArduPilot with `service_started`, and create private camera readers only when output activates.
- Verification: The camera-source change passed 313 tests with 9 skipped. The QoS correction passed 83 focused and 313 full tests with 9 skipped. Subsequent attempts advanced past the matching startup failures.
- Later correction or debt: Deferring readers reduced warmup load but was later tightened to public-zero activation after flight instability appeared.
- Public evidence: commits `4774ac9`, "Select unique competition camera source", `11f0608`, "Match reliable private camera delivery", `083c82e`, "Order Gazebo before ArduPilot startup", and `c951816`, "Defer private camera readers until activation"; `gazebo/resources/models/iris_competition/model.sdf`; `gazebo/src/drone_sim_gazebo/ros_adapter/node.py`; `gazebo/src/drone_sim_gazebo/runtime/children.py`; `compose.yaml`; `tests/integration/test_phase3_runtime_contract.py`.

### EH-0082 Configured video geometry

- Evidence state: Verified. Committed after an end-to-end FFmpeg failure and covered by adapter and runtime tests.
- Problem or constraint: Attempt 12 reached 51 simulated seconds, but FFmpeg rejected the 640 by 480 competition stream because the recorder still hardcoded the earlier 320 by 240 profile.
- Investigation: The camera stream itself matched the competition contract. The fault was downstream geometry construction in the recorder.
- Decision or result: Resolve width, height, frame rate, and encoding from the run configuration and pass them through the artifact runtime to the video adapter.
- Verification: Focused runtime and video-adapter tests covered the configured geometry. Later runs recorded the 640 by 480 streams.
- Later correction or debt: Full evidence finalization still faced callback backlogs and scan time later in the slice.
- Public evidence: commit `872172b`, "Use resolved competition video geometry"; `artifacts/src/artifacts/_adapters/video.py`; `artifacts/src/artifacts/runtime_configuration.py`; `artifacts/src/artifacts/runtime_node.py`; `artifacts/tests/test_video_adapter.py`.

## 2026-08-28

### EH-0083 Physics owned recurrent contact

- Evidence state: Superseded, then verified. First fix committed and superseded; corrected implementation committed and verified.
- Problem or constraint: Attempt 13 failed on stale payload-4 contact because event-driven BEST_EFFORT contact produced no new fact while an attached payload remained in the same state.
- Investigation: The first recurrent implementation published from the wrong ownership and phase boundary. Disabling recurrence was not viable because readiness depended on current level truth.
- Decision or result: Publish recurrent contact after physics update from the component that owns the physical state, retaining exact simulation timestamps and 20 Hz cadence.
- Verification: Plugin integration tests covered recurrence and coordinator behavior. The parent suite reached 313 passed with 10 skipped.
- Later correction or debt: Recurrent truth increased callback load, which exposed later liveness and queue-capacity problems.
- Public evidence: commits `6b3a208`, "Publish recurrent payload contact truth", and `ea6eec4`, "Read recurrent contact truth after physics"; `gazebo/plugin/vendor/gz_sim_8_11_0/DetachableJoint.cc`; `gazebo/plugin/test/DetachableJointContactIntegrationTest.cc`; `gazebo/config/bridge-competition.yaml`; `gazebo/src/drone_sim_gazebo/ros_adapter/node.py`.

### EH-0084 Slow runtime clock and camera liveness

- Evidence state: Verified. Committed after three separate live failure modes.
- Problem or constraint: Attempt 14 advanced only 11.253 simulated seconds in 480 wall seconds. Attempt 15 then stalled because the runtime assumed the SITL bootstrap clock reset to zero. Attempts 16 and 17 showed intermittent takeoff roll and invalid range while camera consumers loaded startup.
- Investigation: Disabling recurrent truth would have broken readiness, and resetting the clock discarded a valid nonzero SITL state. Activating camera consumers earlier still disturbed vehicle startup; one run reached roll minus 46.31 degrees.
- Decision or result: Increase only wall-clock liveness bounds, preserve the nonzero bootstrap clock, and activate camera consumers at public zero. Simulation deadlines and scoring times remained unchanged.
- Verification: Configuration and plugin tests covered the revised bounds and clock behavior. Later runs progressed beyond startup with lower camera pressure.
- Later correction or debt: These were liveness corrections for a slow host. They did not address control-loop tuning or callback starvation after public release.
- Public evidence: commits `859b1e6`, "Raise competition wall liveness ceilings", `ca94d0a`, "Preserve nonzero SITL bootstrap clock", and `ba1be02`, "Defer private camera readers to public zero"; `config/default-run.json`; `gazebo/plugin/0001-paused-initial-json.patch`; `gazebo/src/drone_sim_gazebo/ros_adapter/node.py`; `gazebo/tests/test_plugin_udp.py`; `gazebo/tests/test_adapter_node.py`.

### EH-0085 Nonblocking release and roll control

- Evidence state: Verified. Committed, regression-tested, and measured in focused live tuning.
- Problem or constraint: Attempt 18 filled the completed-ground-truth lookahead because a synchronous Gazebo WorldControl request blocked the executor. Once callbacks remained live, attempt 19 exposed divergent roll behavior.
- Investigation: Raising buffers would have hidden the executor stall. Default roll settings produced about 6.6 to 12 degrees of half-roll error during focused trials.
- Decision or result: Run WorldControl on a private worker so callbacks continue draining, and apply competition roll-rate gains in the SITL parameter set.
- Verification: Runtime-node tests covered callback progress during release. Focused tuning reduced half-roll error to about 0.071 degrees, and parameter tests pinned the gains.
- Later correction or debt: Pitch instability appeared independently in the next set of runs.
- Public evidence: commits `3a022d4`, "Keep Gazebo callbacks live during public release", and `ca3bb8f`, "Stabilize competition roll-rate control"; `gazebo/src/drone_sim_gazebo/runtime/runtime_node.py`; `gazebo/tests/test_runtime_node.py`; `ardupilot_sitl/params/descent.parm`; `ardupilot_sitl/tests/test_config.py`.

## 2026-08-29

### EH-0086 Reliable payload and loaded control

- Evidence state: Verified. Committed and verified by focused tests and measured live behavior.
- Problem or constraint: Attempt 20 lost the exact payload-2 pose at 139.15 because a BEST_EFFORT reader dropped a sample from a RELIABLE bridge. Attempt 21 then exceeded five and seven-second WorldControl bounds. Attempt 22 developed pitch instability and invalid range. Attempt 23 showed some clean control responses took 36 seconds.
- Investigation: The first control increase to 15 and 17 seconds matched one delayed-server measurement but remained too short. A later 30 and 32-second pair also failed before the observed 36-second response completed.
- Decision or result: Make payload pose delivery reliable, tune pitch P and I to 0.0675 with D at 0.0018, and raise loaded Gazebo control bounds first to 30 and 32 seconds and then to 60 and 62 seconds.
- Verification: The pose regression covered the former 100 ms hole. Pitch tuning produced finite range and stable touchdown. A delayed real server completed under the final control budget.
- Later correction or debt: Longer wall-clock control waits preserved simulation-time semantics but made finalization and operator timeouts more important.
- Public evidence: commits `bbaf932`, "Make payload pose delivery reliable", `df2283e`, "Allow loaded Gazebo control response", `6c5f007`, "Stabilize competition pitch-rate control", `21fb6b3`, "Allow slower Gazebo control responses", and `6b39734`, "Allow minute-long Gazebo control responses"; `gazebo/src/drone_sim_gazebo/ros_adapter/node.py`; `gazebo/src/drone_sim_gazebo/runtime/entrypoint.py`; `gazebo/tests/test_runtime_entrypoint.py`; `ardupilot_sitl/params/descent.parm`.

### EH-0087 Exact recent state under load

- Evidence state: Verified. Committed and verified by regression tests, physical release proof, and a 1,280-test parent run.
- Problem or constraint: Vehicle and payload callbacks for the same tick could arrive in opposite order, producing `STALE_PHYSICAL_STATE`. The companion could also receive a range sample ahead of its visible clock, and two cameras sharing one bridge process starved each other.
- Investigation: A single latest-value cache could not distinguish temporary cross-callback skew from stale data. Rejecting every future range sample discarded the last usable current reading. A shared `ros_gz_image` node accumulated onboard lookahead during camera bursts.
- Decision or result: Retain 500 ms histories per physical source and select the latest exact common timestamp, keep the last valid lidar sample while discarding future arrivals, and split the two image topics into separate bridge children.
- Verification: Regressions covered expiry, conflict, regression, and opposite callback order. A later release succeeded at simulation time 153.65. The bridge split finished with 1,280 passed and 24 skipped.
- Later correction or debt: Callback lead still exceeded the adapter's small fixed buffers in later full runs.
- Public evidence: commits `bb8b06b`, "Join recent exact payload truth", `4fd2dee`, "Retain clock-current lidar truth", and `1e8fd4f`, "Isolate camera transport delivery"; `electromagnet/src/drone_sim_electromagnet/controller.py`; `companion/src/drone_sim_companion/comp2026_host.py`; `gazebo/src/drone_sim_gazebo/runtime/children.py`; `gazebo/tests/test_runtime_children.py`.

## 2026-08-30

### EH-0088 Precision landing physical inputs

- Evidence state: Verified. Committed after failed marker-3 landings and backed by configuration, asset, and calibration tests.
- Problem or constraint: A marker-3 attach was rejected as `NOT_LANDED` even though the payload was grounded. The ground-plane contact sensor missed the raised pad, center error was about 0.314 m, ArduPilot precision landing was disabled, and its measurement path did not match the simulated camera timing.
- Investigation: One run logged precision-landing initialization failure and target loss, drifted 3.68 m, and scored 80. Replay first implicated camera calibration, where the simulated calibration reduced median error from 0.274 m to 0.021 m. Further analysis showed 59 to 84 ms delivery lag, occasionally 100 ms, was more important than the initial 20 ms lag setting.
- Decision or result: Publish recurrent 20 Hz vehicle leg contact, enable MAVLink precision landing, load simulator-specific camera calibration in the parent host, set the measured latency compensation, and use direct target measurements.
- Verification: Gazebo asset and runtime tests pinned leg-contact topics. SITL configuration tests pinned the precision-landing parameters. Calibration tests validated the packaged simulator model.
- Later correction or debt: The lag diagnosis refined the earlier calibration-only hypothesis. Both calibration and measured delivery timing mattered, but lag was the stronger residual cause.
- Public evidence: commits `095d0d5`, "Use vehicle leg contact truth", `95a357b`, "Enable MAVLink precision landing", `544c33a`, "Use simulator camera calibration for Comp2026", `d8c6de6`, "Compensate precision landing camera latency", and `120f39d`, "Use direct precision target measurements"; `gazebo/resources/worlds/competition_mission.sdf`; `gazebo/src/drone_sim_gazebo/ros_adapter/topics.py`; `ardupilot_sitl/params/descent.parm`; `companion/src/drone_sim_companion/gazebo_camera_calibration.json`; `companion/tests/test_simulator_camera_calibration.py`.

### EH-0089 Evidence derived callback bounds

- Evidence state: Verified. Several committed corrections, including one immediate reversal, with regression tests at each bound.
- Problem or constraint: Independent camera and ground-truth callbacks arrived a few 50 ms epochs apart under load. The adapter had to preserve exact timestamps without unbounded queues or silent dropping, while the full competition bag needed more than two minutes to finalize.
- Investigation: The first camera change raised unmatched capacity to three. The next ground-truth change restored camera capacity to two while raising completed truth to three, because evidence then pointed to truth lead instead. A later run proved a three-frame camera lead after all, so capacity three was restored. This sequence matters because the bound followed measured delivery, not speculation.
- Decision or result: Permit the proven three-frame camera lead, keep ground-truth lookahead at three completed samples, and allow 600 wall seconds for finalization without changing the 600-second simulation horizon.
- Verification: Adapter-model and private-aggregation tests reproduced accepted and rejected lead lengths. Configuration tests pinned the new finalization budget.
- Later correction or debt: The measured lead continued to grow on later hosts, requiring subscription-depth-sized bounds the next day.
- Public evidence: commits `862f55c`, "Tolerate observed camera callback skew", `330484a`, "Tolerate observed ground truth callback lead", `709d608`, "Tolerate proven camera callback lead", and `c6e3c20`, "Allow competition bundle finalization"; `gazebo/src/drone_sim_gazebo/ros_adapter/model.py`; `gazebo/src/drone_sim_gazebo/ros_adapter/aggregation.py`; `gazebo/tests/test_adapter_model.py`; `gazebo/tests/test_private_aggregation.py`; `config/default-run.json`.

## 2026-08-31

### EH-0090 Bounded lookahead through bursts

- Evidence state: Verified. Committed and regression-tested through a sequence of evidence-backed capacity changes.
- Problem or constraint: Observer frames and private ground truth continued to lead their matching streams under remote and loaded execution. Exact 20 Hz evidence could not be dropped, but buffers still had to fail closed at a finite limit.
- Investigation: Four-frame observer and truth leads exceeded the previous three-sample limits. Matching only one buffer to transport depth moved the failure to another stage. Camera callbacks then arrived in longer bursts even when timestamps remained valid and ordered.
- Decision or result: Match camera and truth lookahead to their subscription depth, preserve frames through callback bursts, and cap private callback backlog at one second rather than allowing growth without limit.
- Verification: Adapter-model, node, and private-aggregation tests covered accepted burst lengths, exact ordering, and fail-closed overflow.
- Later correction or debt: These changes preserved evidence under known bursts. They did not reduce the underlying callback load.
- Public evidence: commits `1cd87bb`, "Tolerate observed observer callback lead", `90f0d60`, "Tolerate observed ground truth callback lead", `1b2d94b`, "Match camera lookahead to subscription depth", `4edcdf8`, "Match truth lookahead to subscription depth", `6bfdeb4`, "Preserve camera frames through callback bursts", and `a43bae4`, "Bound private callback bursts to one second"; `gazebo/src/drone_sim_gazebo/ros_adapter/model.py`; `gazebo/src/drone_sim_gazebo/ros_adapter/aggregation.py`; `gazebo/src/drone_sim_gazebo/ros_adapter/node.py`; `gazebo/tests/test_adapter_node.py`.

### EH-0091 Artifact tail integrity

- Evidence state: Verified. Committed and covered by artifact and adapter regressions.
- Problem or constraint: A complete competition bundle could contain configured stream geometry different from the old defaults, delayed recorder callbacks, and callbacks that arrived after the 600-second public endpoint.
- Investigation: Fixed assumptions about streams and queue depth could misclassify valid evidence. Allowing late camera or physical callbacks into the public sequence could extend the run beyond its contract, while too-shallow ground-truth QoS lost samples during recorder stalls.
- Decision or result: Validate rosbag and video against resolved recording configuration, retain ground truth through recorder stalls, ignore camera callbacks after the endpoint, bound physical samples to the endpoint, and keep ground-truth publisher depth aligned with the downstream retention need.
- Verification: Rosbag, video, epoch-model, and adapter-node tests covered configured geometry, queue retention, post-end callback rejection, and publisher depth.
- Later correction or debt: Final semantic scanning still needed a larger wall-time budget and handling for optional empty topics.
- Public evidence: commits `e992710`, "Validate competition artifacts by configured streams", `3844050`, "Retain ground truth through recorder stalls", `524c2df`, "Ignore camera callbacks beyond run endpoint", `3f5c262`, "Bound physical samples to run endpoint", and `798d952`, "Retain ground truth at public publisher"; `artifacts/src/artifacts/_adapters/rosbag.py`; `artifacts/src/artifacts/_adapters/video.py`; `artifacts/recording-qos.yaml`; `gazebo/src/drone_sim_gazebo/ros_adapter/epoch.py`; `gazebo/src/drone_sim_gazebo/ros_adapter/node.py`.

### EH-0092 Final release and bundle corrections

- Evidence state: Verified. Committed and regression-tested before the accepted mission was documented.
- Problem or constraint: A release could hit the electromagnet service between exact physical ticks and fail `STALE_PHYSICAL_STATE`. A valid bag could also contain the optional scenario-event topic with zero messages, and full semantic inspection exceeded the prior 600-second finalization budget.
- Investigation: Treating every stale release as terminal discarded a physically safe retry opportunity. The rosbag validator indexed first and last timestamps even when an optional topic had no samples. The earlier 120-second and then 600-second finalization budgets were insufficient under full evidence load.
- Decision or result: Retry exactly one stale release after simulation time advances, report empty optional topics with null endpoints, and set finalization to 900 wall seconds.
- Verification: Companion tests proved a second correlated release request only after the clock advanced. Rosbag tests accepted an empty optional scenario stream, and configuration tests pinned the 900-second budget.
- Later correction or debt: The retry remained deliberately narrow. Other command failures still failed closed.
- Public evidence: commits `0d88434`, "Retry one stale physical release", `864460e`, "Handle empty optional bag topics", and `8082c0d`, "Allow full competition evidence finalization"; `companion/src/drone_sim_companion/comp2026_host.py`; `companion/tests/test_comp2026_host.py`; `artifacts/src/artifacts/_adapters/rosbag.py`; `artifacts/tests/test_rosbag_adapter.py`; `config/default-run.json`.

### EH-0093 Accepted competition MVP

- Evidence state: Verified. Live mission completed, independently inspected, scored, and documented.
- Problem or constraint: Completion required separate proof of physical mission outcome, score, a full 600-second evidence horizon, configured video geometry, safe module logs, and clean runtime teardown.
- Investigation: Earlier runs had reached partial scores of 0, 50, 80, and 145 or failed during finalization. The accepted bundle also contained a legitimate module log larger than the generic 4 MiB document limit, and the production semantic check still constructed its video validator before reading configured geometry.
- Decision or result: Accept a manifest-verified module log up to a bounded 32 MiB, resolve video geometry before semantic inspection, and record the full competition run as the project MVP.
- Verification: The mission scored 150/150, returned Home, disarmed at about 509.10 simulated seconds, completed at 509.15, retained the exact 600-second evidence horizon, and passed independent acceptance.
- Later correction or debt: The recorded parent and nested revisions were dirty, and run artifacts remained local ignored evidence rather than Git content. Later work still needed smaller bundles and faster execution.
- Public evidence: commit `0cd78e0`, "Accept and document competition MVP evidence"; `artifacts/src/artifacts/acceptance.py`; `artifacts/tests/test_acceptance.py`; `docs/verification/comp2026-mvp.md`; `docs/verification/runnable-mvp.md`.

### EH-0094 Remote build package lock refresh

- Evidence state: Verified. Commit recorded; this is a drone_sim build-reproducibility change prompted by cross-repository execution.
- Problem or constraint: The canonical images used exact package locks. A clean ephemeral worker could no longer resolve some pinned security-package revisions, so the unchanged parent stack could not build there.
- Investigation: Repeated builds on one worker encountered stale package indices and unavailable locked revisions. The runner changed hosts and refreshed package metadata, but the parent repository still needed the actual resolved package set committed.
- Decision or result: Update only the affected package-lock records in drone_sim. Remote provisioning and lifecycle logic remained in the separate runner repository and is outside this slice.
- Verification: The commit contains only the two lock files. The same-day record confirms the remote build blocker and the exact parent change; no same-day accepted remote mission was tied to this refresh.
- Later correction or debt: Cold ephemeral builds remained expensive and sensitive to upstream package rotation. Follow-up on September 1 bound operator execution to the invoked checkout instead of the process's ambient installation. Later rotations required narrow Gazebo and RabbitMQ lock refreshes rather than loosening exact pins.
- Public evidence: commits `ddb1381`, "build: refresh remote image package locks", `f7e1bac`, "fix: bind operator to invoked checkout", `fc2c7bc`, "build: lock upgraded Gazebo base packages", `877526f`, "Refresh Gazebo build dependencies", and `d8bb5f5`, "Refresh RabbitMQ runtime dependency"; `artifacts/ffmpeg-packages.lock`; `gazebo/harmonic-packages.lock`; `orchestration/src/orchestration/cli.py`; `orchestration/tests/test_cli.py`.

## 2026-09-01

### EH-0095 Separated companion callback lanes and latest-state readiness

- Evidence state: Superseded, then verified.
- Problem or constraint: A run retained all expected samples but stayed in `WAIT_READY`. Range could lead its clock, and large camera copies starved control, range, and metadata callbacks.
- Investigation: One pending range sample fixed the first loss. Global reentrant callbacks then reordered one clock stream. Depth-100 range history held stale data; changing range alone to depth 1 left it skewed against depth-1,000 clock history.
- Decision or result: Use five executor threads with separate ordered lanes for clock/control, range, image, and metadata. Record mission readiness. Keep one leading range sample, then use latest-value depth 1 for both range and companion clock.
- Verification: The corrected run was current by 0.100 seconds, began its first mission segment at 0.233, crossed the prior stale-range boundary, completed both payload cycles, and returned Home.
- Later correction or debt: This fixed callback order and freshness, not evidence size.
- Public evidence: commits `f2099ee`, `aeb24f3`, `b09c511`, `307ed29`, `191bd45`, `8162c94`, `476cc55`, and `502e25c`; `companion/src/drone_sim_companion/comp2026_host.py`; `companion/src/drone_sim_companion/runtime_node.py`.

### EH-0096 Accepted the callback-corrected competition result

- Evidence state: Verified as a physical mission, a 150/150 score, and an accepted artifact bundle.
- Problem or constraint: Physical completion, scoring, and evidence validity had to be evaluated separately.
- Investigation: The aircraft completed both payload cycles and returned Home. An initial inspection used stale local image digests and misclassified the bundle; the recorded build provenance matched the manifest.
- Decision or result: Validate in the execution environment before teardown and retain its digest map for independent checking.
- Verification: The accepted run returned Home and disarmed at about 315.18 simulated seconds, scored 150/150 with all seven rules passing, preserved the 600-second horizon, and produced a complete 21.72 GiB bundle.
- Later correction or debt: Raw camera frames dominated the bundle and led to the next redesign.
- Public evidence: commits `dc6362c` and `198c251`; `artifacts/src/artifacts/acceptance.py`; [competition MVP verification](../verification/comp2026-mvp.md).

### EH-0097 Landing descent speed

- Evidence state: Verified. Committed and covered by focused configuration tests.
- Problem or constraint: The production descent profile used LAND_SPD_MS 0.10. Landings worked, but the slow final descent extended every payload cycle.
- Investigation: The change first had to be separated from the later roll-control instability. Faster descent was kept narrow so it would not mask the attitude problem.
- Decision or result: Set LAND_SPD_MS to 0.30 in the production ArduPilot parameter file and update the interface documentation and configuration test.
- Verification: The focused test passed, followed by 33 SITL tests. Runtime use required an ArduPilot image rebuild.
- Later correction or debt: Later work found that ArduPilot clamps this parameter to a 0.30 minimum and that precision landing can reduce it near the ground. Those findings did not invalidate this production baseline.
- Public evidence: commit fcbce28, "Use production landing descent speed"; ardupilot_sitl/params/descent.parm; ardupilot_sitl/tests/test_config.py; ardupilot_sitl/EXTERNAL_INTERFACE.md.

### EH-0098 Made video the canonical pixel evidence

- Evidence state: Verified by container inspection and 507 tests; a full size result was still pending on this date.
- Problem or constraint: Two 640 by 480 RGB streams contributed about 22.1 GB to MCAP even though H.264 videos already preserved the pixels.
- Investigation: MCAP was about 95 percent of the bundle. Compression did not remove the duplicate representation.
- Decision or result: Keep pixels in MP4. Keep frame metadata, clock, physical truth, mission facts, and score facts in MCAP. Remove raw images only from canonical bag recording.
- Verification: Container inspection found eight intended MCAP topics and no raw-image recording entries. The suite passed 507 tests with 12 skips.
- Later correction or debt: No full mission had yet confirmed the size estimate.
- Public evidence: commit `1fbabaa`; `artifacts/src/artifacts/_adapters/rosbag.py`; `artifacts/recording-qos.yaml`; `config/recording-qos.yaml`; historical artifact contract paths in that commit.

## 2026-09-02

### EH-0099 Roll autotune workflow

- Evidence state: Verified. Implemented in an isolated branch, verified there, and later committed as part of fe44a96.
- Problem or constraint: The vehicle had a roughly 2.9 Hz roll limit cycle. Its roll inertia and lever arms made the existing shared roll and pitch gains far too aggressive on the roll axis.
- Investigation: Early gains with P and I at 0.02 and D at 0.0005 were stopped because repeated 0 percent messages looked like a stalled tune. DataFlash later showed that the percentage was a success counter and the flight had remained stable. A D value of 0.0018 then produced rate ringing near plus or minus 50 degrees per second and a "Failed to level" result.
- Decision or result: Add explicit roll-only AutoTune and hover missions. The workflow writes a canonical five-parameter ardupilot_sitl/autotune-roll.parm file atomically, discovers it as an artifact, and promotes only roll gains while preserving pitch gains.
- Verification: Focused coverage reached 162 passing tests before flight trials. A repeat tune completed at simulation time 89.05 and produced the artifact. The promoted gains were roll P and I 0.0503722, D 0.000375, angle P 13.1974, and acceleration 2547.76.
- Later correction or debt: DroneKit's full parameter reload timed out after a successful tune, so artifact extraction switched to the coherent DataFlash PARM record. Atomic file permissions were corrected from 0600 to 0644. The fixed 180-second evidence tail also exceeded the one-hour tuning wall limit, so the tuning horizon moved to 120 seconds.
- Public evidence: commit fe44a96, "feat: add persistent roll autotune workflow"; config/autotune-roll-run.json; config/hover-roll-run.json; companion/src/drone_sim_companion/autotune.py; companion/src/drone_sim_companion/hover.py; companion/tests/test_autotune.py; companion/tests/test_hover.py; artifacts/src/artifacts/session.py.

### EH-0100 Roll hover verification

- Evidence state: Verified. Implemented, flown, and included in fe44a96.
- Problem or constraint: A saved AutoTune result needed a short independent stability check before use in the full competition mission.
- Investigation: The first hover attempt stopped at the gyro gate because the mission had no arm retry.
- Decision or result: Add the retry and a 45-second hover mission with a 10-second measured hold.
- Verification: The corrected run completed 100/100. During the hold, maximum roll was 0.056 degrees, roll RMS 0.018 degrees, maximum roll rate 0.91 degrees per second, and rate RMS 0.34 degrees per second. Forty-seven focused tests passed after the retry change.
- Later correction or debt: Repository-wide pytest remained unsuitable because it collected nested hardware-only imports. The maintained parent suite reported 1,330 passed and 24 skipped.
- Public evidence: commit fe44a96; config/hover-roll-run.json; companion/src/drone_sim_companion/hover.py; companion/tests/test_hover.py; companion/tests/test_runtime_node.py.

### EH-0101 Corrected atomic payload-release scoring

- Evidence state: Superseded, then verified.
- Problem or constraint: A lean-bundle mission completed physically but scored 50/150. Payload 2 detached at the release-event timestamp, so an inclusive lookup selected the already-detached sample and blocked later awards.
- Investigation: Payloads 3 and 4 independently met their predicates, locating the defect at the atomic transition boundary.
- Decision or result: Score and independently inspect the final attached sample strictly before release. Set canonical touchdown descent to 0.10 m/s.
- Verification: The rerun scored 150/150, returned Home and disarmed at 513.303 seconds, preserved the 600-second horizon, and passed canonical and independent inspection.
- Later correction or debt: The first run was a successful physical mission with a superseded 50-point interpretation, not an accepted scored bundle.
- Public evidence: commit `2610e96`; `scorekeeper/src/drone_sim_scorekeeper/competition.py`; `artifacts/src/artifacts/competition_score_validation.py`; their focused tests; `ardupilot_sitl/params/descent.parm`.

## 2026-09-03

### EH-0102 Public epoch handshake

- Evidence state: Verified. Implemented and merged through fe44a96 after smoke and suite verification.
- Problem or constraint: A full run could remain paused at public simulation time 0.05 until the wall deadline. The companion sent the initial mission command only when its latest clock value was exactly zero, while an asynchronous callback could advance shared state to 0.002 before the polling loop observed zero.
- Investigation: Existing tests modeled a synchronous zero-to-ack transition and missed the race. The first fix then exposed a schema that still required timestamp zero exactly.
- Decision or result: Latch the first public timestamp in the paused 0 to 0.05-second window. Widen the shared protocol to accept that window and reject values above 50 ms.
- Verification: A skipped-zero regression covered 0, then 0.002, then 0.05. Sixty-seven focused tests passed, smoke run d594 advanced to simulation time 1.0, and the protocol update reached 107 focused tests. Full run 417 completed 150/150.
- Later correction or debt: The standalone acceptance script had drifted from the artifact contract. Strict startup time boundaries also remained sensitive to asynchronous delivery and needed tests that represent callback ordering.
- Public evidence: commit fe44a96; companion/src/drone_sim_companion/runtime_node.py; artifacts/src/artifacts/runtime_node.py; artifacts/src/artifacts/runtime_protocol.py; companion/tests/test_runtime_node.py; artifacts/tests/test_runtime_protocol.py.

### EH-0103 Real-time profile and compact artifacts

- Evidence state: Verified. Committed and covered by simulator, controller, and artifact tests.
- Problem or constraint: The standard competition world targeted a 0.25 real-time factor and produced a 1.83 GB state.tlog. The project needed an explicit 1x profile and smaller canonical evidence without changing world topics or scoring semantics.
- Investigation: Compression testing reduced state.tlog to 254 MB in 21 seconds at zstd level 3 and 217 MB in 95 seconds at level 10. The extra 37 MB saving did not justify the additional 74 seconds.
- Decision or result: Add competition_mission_1x.sdf with 1 ms physics at 1,000 Hz, selected through a configuration allowlist. Store compact state artifacts with zstd level 3 while leaving the rest of the bundle directly usable.
- Verification: The change passed 181 simulator tests, 47 controller tests, compilation, and artifact validation coverage.
- Later correction or debt: The 1x profile changed the target, not the amount of work in the simulation. Later measurements showed CPU scheduling and rendering still limited real-time factor.
- Public evidence: commit `e2b009d`, "Add realtime simulation profile and compact state artifacts", and commit `8b60936`, "Document compact recording contract"; `config/realtime-run.json`; `gazebo/resources/worlds/competition_mission_1x.sdf`; `orchestration/src/orchestration/config.py`; `artifacts/src/artifacts/validation.py`; `artifacts/tests/test_validation.py`.

## 2026-09-04

### EH-0104 Lean recording integration

- Evidence state: Verified. Committed as 35fad59 and merged with the AutoTune branch by 7ab6eb3.
- Problem or constraint: After the AutoTune branch merged, phase-two camera discovery still expected two subscribers for every topic. Lean recording had removed raw images from rosbag, leaving one image consumer and two metadata consumers.
- Investigation: The isolated branch suites passed, but the merged topology exposed the stale synthetic gate. Keeping the old two-subscriber rule would have forced raw image recording back into the canonical bag.
- Decision or result: Make the synthetic discovery gate match the deployed topology. Remove raw-image bag assertions while retaining video evidence checks.
- Verification: The merged tree passed 785 unit and contract tests, 3 foundation tests, and 14 Compose tests.
- Later correction or debt: This failure showed that integration tests had encoded a recording implementation detail instead of the current evidence contract.
- Public evidence: commits 35fad59, "fix: align phase2 camera discovery with lean recording", and 7ab6eb3, "Merge branch 'codex/roll-autotune'"; tests/phase2/synthetic_gazebo.py; tests/phase2/test_synthetic_services.py; tests/integration/test_phase2_compose.py.

### EH-0105 GPU runtime and EGL selection

- Evidence state: Verified. Committed through the GPU overlay and EGL selection series, with runtime preflight evidence.
- Problem or constraint: Exposing NVIDIA devices was insufficient. EGL still selected Mesa, Gazebo spawned 19 llvmpipe threads, and rendering stayed on the CPU.
- Investigation: Initial opt-in runtime, explicit NVIDIA runtime, GPU reservation, and overlay commits established device access but did not select the NVIDIA EGL vendor.
- Decision or result: Add a mounted NVIDIA EGL vendor descriptor and force GLVND selection. Require a surfaceless EGL preflight to report NVIDIA before starting the simulation.
- Verification: The corrected runtime used NVIDIA EGL, allocated about 189 MiB on the GPU, showed no llvmpipe workers, and cut Gazebo CPU from roughly 300 to 430 percent down to about 67 percent in the first clean observation.
- Later correction or debt: A valid EGL path proves which renderer is active. It does not promise high utilization. Later accepted runs measured only about 1 to 7 percent GPU activity because most remaining work was CPU and scheduler bound.
- Public evidence: commits a09186f, 0ee7a2c, b7223a1, 1f14ebd, and 021fdef, "Select NVIDIA EGL for GPU simulation"; compose.gpu.yaml; gazebo/config/10_nvidia.json; tests/integration/test_phase3_runtime_contract.py.

### EH-0106 Startup service order

- Evidence state: Verified. Committed and covered by integration tests.
- Problem or constraint: The companion service could start several seconds before ArduPilot SITL and consume its finite DNS retry budget before SITL became available.
- Investigation: The failure looked like a remote runtime problem until startup timestamps showed that Compose had permitted the dependent service to race ahead.
- Decision or result: Order companion startup after the SITL service starts.
- Verification: Eight focused tests passed, and the dependency worked in the subsequent full runtime sequence.
- Later correction or debt: Service-start ordering does not imply sensor readiness. Later work kept mission readiness polling active until the full sensor gate completed.
- Public evidence: commit c8cefca, "Order companion after SITL startup"; compose.yaml; tests/integration/test_phase3_runtime_contract.py.

### EH-0107 Camera epoch alignment

- Evidence state: Verified. The first committed workaround was disproved. The replacement was committed and passed 205 tests.
- Problem or constraint: The first onboard image at public time 0.1 could pair with truth from 0.05, causing a deterministic evidence-alignment failure.
- Investigation: Commit `925e76a` moved the real-time epoch by configuration. A retry reproduced the fault, which ruled out epoch placement as the cause. Camera subscriptions were being created only at public zero, after truth messages had already begun.
- Decision or result: Pre-arm camera subscriptions during warmup. Discard pre-epoch frames before copying image bytes, and keep the 90-second warmup boundary unchanged.
- Verification: Two hundred five tests passed. Later runs crossed the first-frame boundary with aligned image and truth samples.
- Later correction or debt: Commit `925e76a` records a useful failed attempt, but `7429c2d` contains the durable fix. The explicit public-epoch handshake that exposed the boundary is recorded in [EH-0102](#eh-0102-public-epoch-handshake).
- Public evidence: commits 925e76a, "Align realtime epoch after camera boundary", and 7429c2d, "Arm camera inputs before public epoch"; config/realtime-run.json; gazebo/src/drone_sim_gazebo/ros_adapter/node.py; gazebo/tests/test_adapter_node.py; orchestration/tests/test_config.py.

## 2026-09-05

### EH-0108 Clock video and log load

- Evidence state: Verified. Committed and verified in accepted full runs.
- Problem or constraint: Physics and SITL needed a 1 kHz clock, but forwarding every tick into Python accounted for about 87 percent of observed callbacks. Per-frame JSON logs grew to 18.7 MB, and software overlay encoding consumed CPU.
- Investigation: NVENC first failed with CUDA_ERROR_NOT_PERMITTED. The container had the main GPU devices but lacked the encoder capability node.
- Decision or result: Keep physics and SITL at 1 kHz while publishing the public clock at 20 Hz. Remove per-frame JSON logging. Add optional NVENC overlay encoding and pass the required NVIDIA capability device.
- Verification: An accepted run completed 150/150. Logs fell from about 18.8 MB to 25 KB, artifact CPU from 46.2 to 31.5 percent, and scorekeeper CPU from 31.6 to 9.3 percent. Video remained 640 by 480 at 20 fps.
- Later correction or debt: Camera traffic remained near 35 GB, and real-time factor improved only about 1.3 percent in the first full comparison. The main bottleneck had moved outside these Python and logging paths.
- Public evidence: commits c419d95, "Reduce simulation clock and video CPU load", and fd1a5d4, "Document GPU recording contract"; artifacts/src/artifacts/_adapters/video.py; artifacts/src/artifacts/runtime_node.py; gazebo/src/drone_sim_gazebo/ros_adapter/node.py; compose.gpu.yaml.

### EH-0109 Payload timestamp backlog

- Evidence state: Verified. Committed, reproduced before the fix, and verified by a successful physical mission after the fix.
- Problem or constraint: The adapter required every 50 ms payload joint-state sample, but the ROS bridge and subscriber kept only ten samples. A delayed callback could drop otherwise valid consecutive source timestamps.
- Investigation: A 2.4-second callback pause reproduced the same missing-timestamp fault. Historical state.tlog files omitted the private joint stream, so old evidence could not independently prove whether loss occurred at the source or in transport.
- Decision or result: Raise joint-state queues from 10 to 1,000 while keeping strict rejection of missing samples.
- Verification: Fifty focused tests passed. In the follow-up run, the physical mission lifted the third payload to 10.11 m, released inside the target, and returned Home. The scorekeeper awarded 150/150, and the video was playable.
- Later correction or debt: The artifact and run harness still failed because a later downward-range timestamp was missing. The queue fix solved the reproduced joint-state loss, but other strict timestamp streams needed the same backlog audit.
- Public evidence: commit fbf4192, "Preserve payload joint timestamps through callback backlogs"; gazebo/config/bridge-competition.yaml; gazebo/src/drone_sim_gazebo/ros_adapter/node.py; gazebo/tests/test_adapter_node.py; docs/payload-timestamp-fix.md.

### EH-0110 Mission sensor lifecycle

- Evidence state: Verified. Two initial optimizations were corrected by follow-up commits and regression tests.
- Problem or constraint: Mission-start polling and camera subscriptions consumed CPU after they were needed, but stopping them at the wrong lifecycle point could deadlock startup or discard sensors before RUNNING.
- Investigation: Commit 86776dd stopped mission polling after the launch command. The runtime then froze at 0.05 because the complete sensor gate had not arrived. The first post-mission teardown predicate could also fire before RUNNING.
- Decision or result: Keep readiness polling active until the full mission sensor gate completes. Stop companion sensor traffic only after a running attempt has ended, and preserve sensors throughout startup.
- Verification: The readiness correction passed 108 tests. Post-mission teardown reached 109 tests, then 110 after the pre-RUNNING regression was added. In a later accepted run, companion input flattened at 5.07 GB and CPU fell from about 82 percent to 7 percent after mission completion.
- Later correction or debt: Lifecycle predicates need explicit tests for STARTING, RUNNING, and terminal transitions. A command-delivered flag alone is not readiness.
- Public evidence: commits 86776dd, 5d8c593, f304ad9, and eb6c64c; companion/src/drone_sim_companion/runtime_node.py; companion/tests/test_runtime_node.py.

### EH-0111 Native clock and frame path

- Evidence state: Verified. Committed in two measured reductions, with unit, image-contract, and runtime tests.
- Problem or constraint: Sending the 1 kHz Gazebo clock through Python caused callback overhead. The companion path also joined image and metadata streams, converted every RGB frame, and subscribed to metadata that it did not need.
- Investigation: Aggregate CPU remained idle on some hosts, yet callback starvation and high context-switch counts persisted. This pointed to scheduling pressure and unnecessary per-frame work rather than raw CPU capacity.
- Decision or result: Add a C++ clock decimator on an exact 50 ms grid while keeping physics at 1 ms. Remove the redundant companion image and metadata join, drop the metadata subscriber, and defer RGB conversion until capture.
- Verification: The clock plugin rejects duplicate and off-grid samples. The frame-path change removed 56 lines and passed 110 tests. A corrected full run scored 150, produced a 171 MB artifact bundle, and reached about 0.43 real-time factor.
- Later correction or debt: Tail profiling still showed about 6,546 context switches per second. The changes reduced Python work but did not remove lockstep scheduler overhead.
- Public evidence: commits 7340b76, "Decimate native Gazebo clock before Python", and 14f7fca, "Remove redundant companion frame pairing"; gazebo/plugin/ClockDecimator.cc; gazebo/plugin/test/ClockDecimatorTest.cc; gazebo/config/bridge-competition.yaml; companion/src/drone_sim_companion/comp2026_host.py; companion/tests/test_comp2026_host.py.

### EH-0112 Shared network namespace

- Evidence state: Verified. Initial implementation failed Compose and DDS checks. The compatibility fix was committed and completed an accepted run.
- Problem or constraint: High camera traffic crossed Docker networking between simulation services. Host networking would reduce that path but expose the stack outside the Compose boundary.
- Investigation: The first service-scoped namespace attempt conflicted with ArduPilot's expose setting. After that was removed, DDS selected shared memory because the containers shared networking, but they still had separate IPC namespaces and exchanged no messages.
- Decision or result: Use network_mode service:gazebo for loopback communication and share Gazebo's IPC namespace as well. Record Docker startup failures through orchestration.
- Verification: Ten Compose tests and two failure-path tests passed. A full run completed 150/150. Public real-time factor moved from 0.406 to 0.420, about 3.4 percent.
- Later correction or debt: The small gain showed that container networking was not the dominant limit. Profiling still attributed roughly 45 percent of sampled cycles to kernel scheduler and locking paths.
- Public evidence: commits `5e0c49d` and `f4c86a8`; `compose.gpu.yaml`; `orchestration/src/orchestration/controller.py`; `orchestration/tests/test_controller.py`; `tests/integration/test_phase3_runtime_contract.py`.

### EH-0113 Bounded Ogre workers

- Evidence state: Verified. Upstream behavior was backported, the zero-worker setting was measured and rejected, and a bounded setting was committed.
- Problem or constraint: gz-rendering 8.2.3 predated upstream support for controlling inline Ogre render workers. The default spawned many workers, which increased scheduling activity.
- Investigation: The first source patch used overlapping text substitutions and omitted the provenance directory expected by the image contract. After those were fixed, a zero-worker run reduced Gazebo CPU and context switching but removed useful parallelism. Realtime factor fell to roughly 0.11 to 0.17 under a noisy host.
- Decision or result: Backport the upstream worker control, verify the patched binary and package provenance, then use four bounded workers instead of zero.
- Verification: The image contract checked gz-rendering 8.2.3, the patch revision, binary environment, 795 installed packages, and Gazebo 8.11.0. Twenty-one tests passed for both the backport and bounded-worker change.
- Later correction or debt: The zero-worker run ended before final artifact collection, so it supplied performance evidence but not a complete acceptance bundle. Four workers still needed a clean same-host comparison.
- Public evidence: commits 4217a94, "Backport inline Ogre rendering workers", and b45540e, "Use bounded Ogre rendering workers"; gazebo/patches/gz-rendering8-inline-workers.patch; gazebo/Dockerfile; compose.gpu.yaml; gazebo/tests/test_image_contract.py; tests/integration/test_phase3_runtime_contract.py.

### EH-0114 Documentation consolidation

- Evidence state: Verified. Designed, implemented, committed, and checked for links and focused tests.
- Problem or constraint: The repository lacked a single human entry point. Operational and interface guidance had drifted across plans and internal reports. Examples included stale queue depths and two different recording QoS definitions.
- Investigation: The first design kept only four central documents. Review showed that each of the seven modules still needed a local README for ownership, interfaces, tests, and constraints.
- Decision or result: Make README.md the project entry point. Keep docs/architecture.md, docs/runbook.md, and docs/handoff.md for shared concerns, plus one README per module. Move contribution and documentation-maintenance rules into AGENTS.md. Remove 105 obsolete planning and report files.
- Verification: The implementation passed 81 focused tests with 4 skipped and checked 234 links plus 5 configuration templates.
- Later correction or debt: Exact fields and settings remain authoritative in schemas, configuration, code, and tests. Documentation records behavior and ownership rather than duplicating configuration tables. A fresh checkout still needs an available companion/comp2026 revision.
- Public evidence: commits `212fefa` and `5900d0d`; [repository README](../../README.md); [contribution rules](../../AGENTS.md); [architecture](../architecture.md); [runbook](../runbook.md); [handoff](../handoff.md); module READMEs.

### EH-0115 Started descriptor-safe protocol-file consolidation on an unmerged branch

- Evidence state: Verified as branch-only at the cutoff.
- Problem or constraint: Artifact runtime and host status code independently implemented strict JSON, bounded descriptor-relative reads, identity checks, and atomic writes.
- Investigation: A broad rewrite was rejected. Tests first specified canonical encoding, unsafe-file rejection, read identity and deadlines, exact modes, three write policies, interrupted writes, and concurrency; review strengthened cleanup and variable read chunking.
- Decision or result: Add `artifacts.protocol_files` as the low-level owner while leaving domain schemas and exceptions with each caller.
- Verification: Mainline commits `390bd9f` and `4214163` recorded the design and plan. Branch-only commits `f103007`, `f179f00`, and `179a7e8` built the test contract; `43946d9` implemented it; `d4db4fb` hardened cleanup.
- Later correction or debt: Neither caller had migrated and the broader audit was incomplete at the cutoff. Do not treat the branch code as mainline behavior.
- Public evidence: mainline commits `390bd9f` and `4214163`; local branch commits `f103007`, `f179f00`, `179a7e8`, `43946d9`, and `d4db4fb`, which are not mainline evidence at the cutoff; historical plan paths `docs/superpowers/specs/2026-09-05-code-simplification-hardening-design.md` and `docs/superpowers/plans/2026-09-05-protocol-file-consolidation.md`; branch path `artifacts/src/artifacts/protocol_files.py`.
