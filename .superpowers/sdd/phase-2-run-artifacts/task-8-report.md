# Task 8 implementation, TDD, and verification report

Implementation commit: `a08f52769f0eb3164f7c80f436df977356bfc442`

Base commit: `ba02d5904b081499546454a2fbad64aa6ef7e849`

Final verification date: 2026-08-24 UTC

## Scope delivered

Task 8 adds a real public-CLI acceptance gate for completed, failed, and aborted synthetic runs; a canonical read-only container inspector; one-build `test-phase2` wiring; strict scoring provenance; real Docker Compose NDJSON health parsing; host/container log-directory ownership integration; and removal of the successfully consumed host publication source before manifest inventory.

No companion, physical simulation, mission, or production-scoring implementation was added.

## TDD RED evidence

### Scoring provenance RED

Command:

```text
uv run pytest orchestration/tests/test_controller.py -q
```

Initial output and exit:

```text
FFFFFF................................................ [100%]
6 failed, 48 passed in 6.13s
```

The positive test expected the declared lowercase `dddd...` `scoring_checksum`, but the controller returned SHA-256 `b1b6...` of `scoring/result.json`. Five negative cases showed that uppercase/short checksums, parent traversal, absolute paths, and duplicate evidence were accepted. A later mutation pass added unhashable evidence and invalid/nonfinite scores and correctly produced three more failures, including the prior `TypeError: unhashable type: 'dict'`.

### Full lifecycle/artifact RED required by the brief

Command:

```text
uv run pytest tests/integration/test_phase2_compose.py -v
```

Output and exit:

```text
collected 3 items
test_completed_runs_are_semantically_identical_across_wall_delays FAILED
test_observer_encoder_failure_preserves_readable_recovery_bundle FAILED
test_abort_is_durable_idempotent_and_preserves_readable_bundle FAILED
3 failed in 177.32s
```

The completed public `start` exited 1 with `compose_ps_invalid`; real Compose emitted one JSON object per line while the controller accepted only one object/array. The fault case then lacked all seven published structured logs because container-root had created `logs/docker` mode 0755 and the host could not create archival candidates. Abort never reached durable `RUNNING` because the same `ps` parsing failure forced early `FAILED`.

Incremental completed-case REDs subsequently exposed and drove:

- `docker_log_capture_failed` with `[Errno 13] Permission denied: 'orchestration-runtime.log.partial'`;
- forbidden retained `logs/orchestration-host.jsonl.partial` after successful capture;
- inspector normalization that still included the necessarily different per-run UUID in otherwise identical custom events.

Each failure was observed before its minimal corresponding change.

## Integration changes

- `_ps_cause` accepts both Compose JSON array/object output and the real line-delimited JSON format, while retaining exact seven-service and health checks.
- `_score_metadata` consumes the result document's exact lowercase SHA-256, rejects malformed/nonfinite scores, and rejects duplicate, absolute, non-normalized, parent-traversing, or non-string evidence paths.
- The host creates `logs/docker` before Compose starts, so recorder containers and later host archival share a safe host-owned directory.
- After successful raw/structured archival, the controller descriptor-verifies and durably removes the closed `orchestration-host.jsonl.partial`; capture failure retains it for inventory.
- `inspect_bundle.py` uses FFprobe, strict full decode, per-frame SHA-256, `rosbag2_py` deserialization, and the production `RosbagValidator` inside the stable artifacts image.
- The acceptance suite recomputes all manifest hashes/sizes, reparses seven logs, validates JSON schemas/provenance, tests CLI output shape/idempotence, checks immutable outputs, and verifies per-project container/network cleanup in `finally` blocks.

## GREEN command/output record

| Command | Output | Exit |
| --- | --- | ---: |
| `uv run pytest orchestration/tests/test_controller.py -q` | `55 passed in 9.76s` after the first integration fixes | 0 |
| same focused command after strict score mutation cases | `58 passed in 7.07s` | 0 |
| `DRONE_SIM_PHASE2_IMAGES_BUILT=1 uv run pytest ...::test_observer_encoder_failure... -v` | `1 passed in 31.54s` | 0 |
| `DRONE_SIM_PHASE2_IMAGES_BUILT=1 uv run pytest ...::test_abort_is_durable... -v` | `1 passed in 33.63s` | 0 |
| container inspector comparison over the two retained completed bundles | `diff` emitted no output | 0 |
| `DRONE_SIM_PHASE2_IMAGES_BUILT=1 uv run pytest tests/integration/test_phase2_compose.py -v` | `3 passed in 164.61s` | 0 |
| strengthened immutable/inventory gate, same command | `3 passed in 127.52s` | 0 |
| `make test` | `549 passed, 10 skipped in 25.73s`; Phase 1 `3 passed in 8.63s`; Phase 2 `3 passed in 144.78s` | 0 |
| `docker compose config --quiet` | no output | 0 |
| `docker compose --profile phase2 config --quiet` | no output | 0 |
| `git diff --check` | no output | 0 |
| `uv run python -m py_compile tests/integration/test_phase2_compose.py tests/phase2/inspect_bundle.py orchestration/src/orchestration/controller.py` | no output | 0 |

The `make test` log shows exactly one `docker compose --profile phase2 build` before `DRONE_SIM_PHASE2_IMAGES_BUILT=1 ...test_phase2_compose.py`; no case rebuilt, and production `up` remained `--detach --no-build`.

## Final image digests

```text
drone-sim-orchestration-runtime:phase2  9f2abc95b37c3dd8dff10fbd237fcfdd52f1e6cabad0016cd158a0ad9165acc5
drone-sim-artifacts-runtime:phase2      4bea948cb892487c3a32a6779df93c478be246bcc30461e4a6c2433d8440e901
drone-sim-synthetic-companion:phase2    64d2cec58ad57157c939d27160f88d200ded722eafc00124b4a871f3f73a8efe
drone-sim-synthetic-ardupilot-sitl:phase2 d60b7f2fa0ad65d961ea781b8d9bd6b9062475c78e56463111815a0222135dc3
drone-sim-synthetic-gazebo:phase2       95d7451a1f858f6fa953b244dc4ace5d3eb5314a78c36f3df2c12ed6a7157a8d
drone-sim-synthetic-electromagnet:phase2 287e1e019c387e8dee92adcd63667e6334d2e00ac970fc857e5912a67cfd9ce3
drone-sim-synthetic-scorekeeper:phase2  9fbdccfbba52ad433863c6ef0d63eaadac8bde2a8c09c459e606bef9ca42cef3
```

All four final manifests record these same seven source-exact IDs.

## Real public CLI runs and bundle evidence

Final `make test` output root: `/tmp/pytest-of-willis/pytest-4552/phase2-output0`.

| Mode | Run ID | CLI exit | Manifest SHA-256 | Manifest reason |
| --- | --- | ---: | --- | --- |
| completed, zero delay | `985b5fd9-30f0-469d-bbed-c2f37d6f1180` | 0 | `4890340e9ee4cb32ddc74afe213fb44e84a8b3460fd1ea1916d181756f2fcea3` | `mission_complete` |
| completed, 17 ms delay | `f0dd011e-1380-4bf9-bdc4-e42132f03699` | 0 | `efbbb1a0f5a9613d000e7fd3ccf1e7942ee16c5faab07526a3a45af9baf79219` | `mission_complete` |
| failed, observer after five frames | `d8649952-bba5-4bc5-8287-82eb39848c9f` | 1 | `6ed3db8cc7b578d796361be3d62abc1fc204b9b59641d568b4d9c35d8233cb9a` | `observer encoder fault injected after frame 4` |
| aborted after durable RUNNING | `e8e6c620-ffa5-44a8-aa23-e46c874a5c5a` | 130 | `b7304f85fabaaea788304cf33540ce53864ca6798c8b90b38328df2b4da7799c` | `operator_abort` |

Foreground `start` output was zero or more valid host `StructuredEvent` objects followed by exactly one `run_result`. `status`, both abort calls, and every collection emitted exactly one `run_result`. The second abort and repeated collections preserved `ABORTED` and the exact manifest bytes.

### Completed media, bag, score, and log facts

Both MP4s: one H.264/yuv420p stream, 320×240, `20/1`, 40 frames; FFprobe and strict full decode succeeded; 40 decoded-frame SHA-256 values were captured per stream.

```text
video/onboard.mp4  5036 bytes  8f5eab5c28ebb345719c7bf4d31a90f7fb13cb3ae74f3a23135aeb1271967481
video/observer.mp4 5330 bytes  3e514d5b4bf655b6e5364af229c84c619af959e5092b4e67b3aea2eefa4b4bbf
rosbag tree          18513153 bytes f1685021dc1ba7168867b65e9a0423be914818bf0707c489762f31948989ce54
```

Exact decoded MCAP counts:

```text
/clock 41
/simulation/run_state 4
/simulation/artifact_status 1
/simulation/ground_truth 40
/simulation/scenario_events 1
/simulation/score_events 1
/camera/onboard/image_raw 40
/camera/onboard/frame_metadata 40
/camera/observer/image_raw 40
/camera/observer/frame_metadata 40
```

Lifecycle order was `STARTING, READY, RUNNING, FINALIZING`. Artifact-ready record index 1 preceded first clock index 3. Frame IDs were contiguous 0–39 and image/metadata stamps paired exactly at 50 ms. Normalized timestamps, IDs, image payload SHA-256 values, event payloads, and decoded video-frame SHA-256 values were identical across wall delays.

Manifest scoring was exactly `0.0 / 0.0`. Both result and manifest declared `5b227e82e9217c34c342e37d6ce2872edc28c63cded78e38d3698c673ddefedb`, independently equal to SHA-256 of `tests/phase2/scoring.json`, with safe evidence `scoring/events.jsonl`.

Completed structured-log line counts were orchestration 7, artifacts 282, companion 3, `ardupilot_sitl` 3, gazebo 83, electromagnet 3, and scorekeeper 3. Every line reparsed through `StructuredEvent`. All seven raw Docker logs were present and nonempty. The only completed partials were the exact inventoried recorder diagnostics:

```text
logs/docker/ffmpeg-onboard.log.partial
logs/docker/ffmpeg-observer.log.partial
logs/docker/rosbag2.log.partial
```

No host source partial, manifest candidate, DockerLogCapture candidate, unknown partial, `.control`, or `.status` path appeared in the artifact inventory.

### Failed facts

The bag metadata opened and every serialized message deserialized: 7 clocks; 6 ground truth, image, and metadata messages per stream; 4 lifecycle records; 1 artifact-ready; no scenario/score messages. It was therefore structurally readable and correctly strict-invalid. Onboard was a playable six-frame H.264/yuv420p MP4. The observer was an explicit invalid required record, while its retained five-frame output decoded and its inventoried FFmpeg partial remained. Seven structured and seven raw module logs were preserved.

### Aborted facts

Abort was issued only after exact durable `.status/operator-state.json` state `RUNNING`. The retained bag opened/deserialized: 8 clocks, 7 ground-truth and onboard pairs, 7 observer bag pairs, and no scenario/score events; strict semantics correctly remained invalid. Onboard retained 7 playable frames and observer 6. Repeated public commands did not change any non-control/status file byte, size, or mtime.

### Timing and cleanup

| Run | Wall duration | FINALIZING→capture | FINALIZING→manifest | Capture→manifest |
| --- | ---: | ---: | ---: | ---: |
| completed | 22.119050 s | 2.538457 s | 6.325604 s | 3.787147 s |
| failed | 19.713515 s | 2.134036 s | 7.374521 s | 5.240485 s |
| aborted | 24.487924 s | 1.970398 s | 8.165164 s | 6.194766 s |

Each representative project had 0 Compose-labeled containers and 0 `<project>_default` networks after exit. Cleanup was also repeated in test `finally` blocks.

## Companion status

For the exact assigned-worktree command only, verification temporarily linked the otherwise absent untracked path `companion/comp2026` to `/home/willis/projects/drone_sim/companion/comp2026`. The exact command `git -C companion/comp2026 status --short` exited 0 with no output. The temporary symlink was then removed and is absent from the worktree. No Task 8 change modified, copied, committed, or pushed the companion repository.

## Self-review

- Re-read every Task 8 brief assertion and five binding rulings against the acceptance test.
- Confirmed `start` still uses `--no-build` and the exact seven stable tags.
- Confirmed all simulation comparisons use bag/message stamps, never wall time.
- Mutated score checksum/evidence/value shapes; focused tests reject each malformed case.
- Checked that successful host-source removal occurs only after structured log publication; capture failure leaves recoverable evidence for inventory.
- Independently recomputed manifest file/tree SHA-256 and size for all valid records, including optional raw/recorder diagnostics.
- Verified completed output admits only three ruled recorder partials, while failed/aborted allow only inventoried recovery partials.
- Verified all read-only CLI operations preserve terminal outputs and that manifest publication remains the immutable authority.
- `git diff --check` is clean. The only expected remaining working-tree changes before the evidence commit are this report, the verification document, and controller-owned plan/ledger amendments.

## Exact non-claims

Phase 2 uses synthetic Gazebo and scoring fixtures. Task 8 does not claim Gazebo Harmonic physics, a Gazebo-authoritative clock, native Gazebo state, ArduPilot lockstep, companion behavior, electromagnet physics, mission scoring, or maximum-score acceptance. Fixture `0.0 / 0.0` proves only lifecycle/artifact infrastructure and is explicitly not the eventual maximum-score goal.

## Fix round 1 — five Important findings

### RED evidence

Fail-closed scoring was first expressed through whole-controller cases for absent and malformed provenance plus non-upgrade cases for already requested failure/abort:

```text
$ uv run pytest -q orchestration/tests/test_controller.py -k 'scoring_provenance'
FF..                                                                     [100%]
2 failed, 2 passed, 58 deselected in 1.87s
```

The absent case did reach `FAILED`, but retained reason `mission_complete`; the malformed case incorrectly committed `COMPLETED`. This isolated the missing controller decision after `_score_metadata` returned its invalid sentinel.

The exact partial and positive-frame predicates were then added as negative acceptance tests before changing the helpers:

```text
$ DRONE_SIM_PHASE2_IMAGES_BUILT=1 uv run pytest -q tests/integration/test_phase2_compose.py -k 'terminal_inventory or positive_recorder'
FFFFF.F                                                                  [100%]
6 failed, 1 passed, 3 deselected in 0.54s
```

Failed mode admitted raw-log, structured-log, host-source, unknown-recorder, and unknown partials. The corrupt-positive video case also had no recorder-local semantic assertion helper. The already-existing manifest-candidate rejection was the one passing negative.

### Minimal fixes

- After strict `_score_metadata` parsing, the controller now changes only requested `COMPLETED` to `FAILED/scoring_provenance_invalid` when any required score/checksum fact is absent. Existing `FAILED` and `ABORTED` requests keep their state and original reason. Strict lowercase checksum and safe, unique evidence-path validation are unchanged; valid fixture `0.0 / 0.0` remains valid.
- Terminal inventory now requires the three exact frozen recorder diagnostics in every terminal mode. Only failed/aborted modes may additionally retain `video/onboard.mp4.partial` or `video/observer.mp4.partial`, and those paths must be inventoried. Manifest candidates, DockerLogCapture raw/structured publication candidates, the consumed host source, and every unknown partial fail the gate.
- Read-only CLI checks derive one canonical result from the committed manifest and require exit 0 plus exact equality of all fixed result fields (`result_type`, run ID, state, reason, and manifest path) for every repeated abort/collect/status command. Immutable output snapshots remain exact.
- The container inspector now reads each video count from recorder-owned `.status/artifacts-final.json`. A positive count unconditionally requires FFprobe success, strict full decode, exact H.264/yuv420p/320×240/20-FPS facts, production validator success, and equality with the decoded-hash count. Zero/missing counts require explicit missing/invalid recorder and manifest records. A corrupt-positive negative cannot bypass the assertions.

### GREEN commands and exact output

| Command | Output | Exit |
| --- | --- | ---: |
| `uv run pytest -q orchestration/tests/test_controller.py -k 'scoring_provenance or score_metadata'` | `13 passed, 49 deselected in 3.68s` | 0 |
| `DRONE_SIM_PHASE2_IMAGES_BUILT=1 uv run pytest -q tests/integration/test_phase2_compose.py -k 'terminal_inventory or recorder_frame_count'` | `10 passed, 3 deselected in 0.31s` | 0 |
| `uv run pytest -q orchestration/tests/test_controller.py` | `62 passed in 15.16s` | 0 |
| `make test-phase2` | one seven-image build, then `13 passed in 162.25s` | 0 |
| independent container inspection of all four bundles | four one-line JSON summaries; every positive video count matched probe/decode/validator/hash facts and every bag structurally opened | 0 |
| repeated public abort, collect twice, and status against the committed aborted bundle | four identical canonical `ABORTED/operator_abort/manifest.json` results; manifest hash unchanged | 0 |
| `make test-unit` | `553 passed, 10 skipped in 27.38s` | 0 |
| `make test-foundation` | `3 passed in 5.20s` | 0 |
| `uv run python -m py_compile tests/integration/test_phase2_compose.py tests/phase2/inspect_bundle.py orchestration/src/orchestration/controller.py` | no output | 0 |
| `git -C companion/comp2026 status --short` with the required temporary verification link | no output | 0 |

The temporary companion link was removed immediately after the exact command; `test ! -e companion/comp2026 && test ! -L companion/comp2026` then exited 0.

### Real terminal evidence

| Mode | Run ID | Exit | Bundle | Manifest SHA-256 | Reason |
| --- | --- | ---: | --- | --- | --- |
| completed, 0 ms | `638d540f-5b34-4d4e-8696-933cd10501b6` | 0 | `/tmp/pytest-of-willis/pytest-4558/phase2-output0/638d540f-5b34-4d4e-8696-933cd10501b6` | `c8cd458ccb9a4d4e545f5ba9fe133a0dd6862fda85e611337d1e96bd22deea25` | `mission_complete` |
| completed, 17 ms | `dd0ae4cf-89b1-4c90-82aa-8da8fa0894e2` | 0 | `/tmp/pytest-of-willis/pytest-4558/phase2-output0/dd0ae4cf-89b1-4c90-82aa-8da8fa0894e2` | `d274addacf526af576efdfe140cbfa57ae7026bf09848e2c9d74ad7f0f3d9fa1` | `mission_complete` |
| failed observer | `2d2741fc-5487-4783-8669-ca3f293ccedc` | 1 | `/tmp/pytest-of-willis/pytest-4558/phase2-output0/2d2741fc-5487-4783-8669-ca3f293ccedc` | `82bb40e40122eb3436b7030cb8a8591baaa53d8f87c524e92df4ac179a9dd1d6` | `observer encoder fault injected after frame 4` |
| aborted from durable RUNNING | `c713c2d1-878b-43a4-84b5-2cfa4ee2e453` | 130 | `/tmp/pytest-of-willis/pytest-4558/phase2-output0/c713c2d1-878b-43a4-84b5-2cfa4ee2e453` | `864687131bac9a53dab274242a43f5a77a22d07c563e0598c7191dbcefe8df06` | `operator_abort` |

Every manifest had 25 artifact records, seven exact structured logs, seven nonempty raw logs, the declared `0.0 / 0.0` scoring fixture, and scoring checksum `5b227e82e9217c34c342e37d6ce2872edc28c63cded78e38d3698c673ddefedb`. Completed bags had the exact ten-topic counts: clock 41, run state 4, artifact status 1, ground truth/images/metadata 40, and scenario/score 1. Completed videos were 40/40; failed videos were 6/5; aborted videos were 10/10. Every count was recorder-local and exactly matched decoded hashes. The failed and aborted bags were structurally readable and correctly strict-invalid.

The final manifest image mapping was:

```text
drone-sim-orchestration-runtime:phase2       d42a4503194c1ad4e7dfbb1aa53ea42efbb9ac9b4bc7dae8dad6c626dc2dd2e3
drone-sim-artifacts-runtime:phase2           b2f719a243b1ada11267183038db7d81ead6860a7fe2f2c0cef83599b0c506f4
drone-sim-synthetic-companion:phase2         90dc630531425c43d17a9d821cbfedb62f5e347ae89ad3fececfa78f8cd44ffe
drone-sim-synthetic-ardupilot-sitl:phase2    c3ac9f20eaec2ccf661130ebca9ac9a1fc40b30d5d57417b0154f7043f0f4781
drone-sim-synthetic-gazebo:phase2            8522f693b8b68c176cd26ef053ef5a3869cb26ddab20022630ce7e36473dc4e3
drone-sim-synthetic-electromagnet:phase2     7adbb1f7845dc5166a4628160970e43baabcd6196b92a01cdbabf6d2471f6802
drone-sim-synthetic-scorekeeper:phase2       62e945f5563bbd122efc1c6556e60888fcce71549d1537387e4ea62272bc2dae
```

All four bundles retained only the exact three frozen recorder diagnostics; the allowed recovery-video set was exercised synthetically in both failed and aborted policy tests. All four projects had no labeled containers, and exact network inspection returned exit 1 for every `<project>_default` network. Cleanup remained in `finally` paths.

### Fix-round self-review and exact non-claims

The five Important findings are covered directly: completion-only score downgrade; exact mode-specific partial policy; exit/result/immutability idempotence; recorder-local positive/zero video predicates; and the exact green worktree-relative companion command with no persistent link. The three explicitly deferred minors were not expanded in this round. No companion content or machine configuration changed.

This fix round remains synthetic lifecycle/artifact acceptance only. It does not claim Gazebo Harmonic, real ArduPilot, mission behavior, electromagnet physics, real scoring, maximum-score success, or wall-derived simulation time. The `0.0 / 0.0` result is only the Phase 2 infrastructure fixture and is not the eventual maximum-score goal.

### Final pre-commit verification

After the last formatting and evidence edits, verification was repeated without rebuilding the already stable images:

```text
$ DRONE_SIM_PHASE2_IMAGES_BUILT=1 uv run pytest -q tests/integration/test_phase2_compose.py
.............                                                            [100%]
13 passed in 119.74s (0:01:59)

$ make test-unit
======================= 553 passed, 10 skipped in 19.33s =======================

$ make test-foundation
============================== 3 passed in 4.23s ===============================
```

The combined configuration, compile, and whitespace command (`docker compose config --quiet`; profiled config; `py_compile`; `git diff --check`) exited 0 with no additional output. The exact temporary-link companion command was repeated at exit 0/no output, and link removal plus absence checks again exited 0.
