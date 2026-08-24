# Phase 2 final-review fix report

Base: `eb3dcb77a597c58d59eac5933aa0ab0bd26cd5b2`

Recovery verification window: 2026-08-24 10:50–11:10 UTC

Platform: local Docker Engine, ROS 2 Jazzy runtime images, host `uv` environment

## Recovered worktree audit

The prior worker stopped with 27 changed paths (26 tracked paths plus the new
`artifacts/tests/test_artifact_status_ros.py`), 1,053 inserted lines, 178 removed
lines, no checkpoint commit, and no report. Its edits were preserved and audited
file by file. `git diff --check` was clean before testing. The preliminary
“complete” ledger lines were treated as unverified until the focused and full
gates below passed.

The base-relative diff covers all seven substantive findings and all three
triaged minors:

| Finding | Recovered implementation | Direct regression evidence |
| --- | --- | --- |
| post-link manifest authority | `write_manifest_atomic` records hard-link publication, resolves timeout ambiguity by descriptor-safe identical-byte reread, completes directory `fsync` without a later cooperative rejection, and returns the existing typed `FinalizationResult` seam | timeout raised from inside injected `os.link` still returns the committed typed result; identical existing publication remains authoritative |
| exact geometry | both schemas use `const` 320/240; runtime validation requires exact integer `(320, 240)` before Compose construction | schema mutations at 640/480 fail and the controller factory is never invoked |
| initial, ready, and final `ArtifactStatus` | artifacts waits for the rosbag subscription, publishes exact not-ready then ready samples before clock; rosbag validation requires both; after manifest commit the artifacts runtime descriptor-reads manifest completeness, publishes a sorted live final sample, then writes `terminal-notified` | aggregate unit tests, rosbag ordering tests, real Jazzy MCAP test, and real reliable/transient-local late-joiner test |
| committed result survives status-cache failure | terminal operator-status persistence is diagnostic after commit; `status`, `abort`, and `collect-results` prefer a validated typed manifest result | injected terminal operator-state write failure returns the committed result and all three later commands return the same immutable facts |
| scoring provenance safety | scoring bytes are read through a retained descriptor walk with `O_NOFOLLOW`, regular-file/single-link checks, bounded capture, pre/post identity checks, and safe run-relative paths | parent symlink, file symlink, and external hard-link provenance all fail closed |
| Compose topology hardening | every command pins absolute repository `compose.yaml`; ambient file/env-file/profile/project selectors are removed, implicit `.env` loading is disabled, the Phase 2 profile is set explicitly, and Docker host/TLS/cert/context variables remain | exact argv/environment assertions and the real four-run gate |
| archival camera QoS docs | root and artifact interfaces now document reliable depth 5 for the Phase 2 publisher, video subscriptions, and rosbag overrides; compatible later best-effort mission consumers remain allowed | root/interface diff plus real Jazzy QoS/MCAP coverage |
| equal image mapping minor | the integration gate compares the exact seven-name map across all four manifests | final four-manifest test passed |
| finalization timing minor | the final no-build gate records FINALIZING→capture, capture→manifest, and FINALIZING→manifest per representative run | final timing table below; additive interval assertion passed |
| unused import minor | the unused inspector `ValidationStatus` import is removed | compile and unit gates passed |

No fix outside the synthetic Phase 2 boundary was added. `companion/comp2026`,
real Gazebo, real ArduPilot, mission logic, electromagnet physics, and real
scoring remain outside this wave.

## Recovered RED and GREEN evidence

The regression tests and production edits arrived together in the recovered
uncommitted worktree, so the original worker's interactive RED output was not
available. The base diff itself preserves the RED cause for every test: the base
checked the deadline after `os.link`, accepted arbitrary positive even geometry,
published only the ready status, wrote `terminal-notified` from orchestration,
let a terminal operator-cache write escape, opened scoring by path, inherited
ambient Compose selectors without `--file`, documented best-effort archival
camera QoS, compared image maps only within a manifest, omitted the three timing
intervals, and imported unused `ValidationStatus`.

The recovered regression cases were then run against the complete wave:

```text
$ uv run pytest -q <two post-link manifest publication regressions>
2 passed in 0.21s

$ uv run pytest -q <config, controller authority, scoring, and Compose focus>
56 passed in 1.00s

$ uv run pytest -q <runtime protocol, ArtifactStatus, and rosbag focus>
10 passed, 1 skipped in 0.11s

$ uv run pytest -q <integration helper policy focus>
10 passed in 25.86s

$ docker build --file artifacts/Dockerfile --target test \
    --tag drone-sim-artifacts-test:phase2-final-review .
$ docker run --rm drone-sim-artifacts-test:phase2-final-review \
    uv run --no-sync pytest -q artifacts/tests/test_artifact_status_ros.py \
    artifacts/tests/test_rosbag_adapter.py::test_real_jazzy_mcap_fixture_is_read_via_rosbag2_and_deserialized
2 passed in 1.38s
```

The host skip is intentional because ROS is supplied by the source-exact test
image; the two real Jazzy cases passed there.

## Full verification

All pytest runs used explicit temporary roots. No broad scan of
`/tmp/pytest-of-willis` was used.

| Command | Result | Exit |
| --- | --- | ---: |
| `COMPOSE_PROGRESS=plain PYTEST_ADDOPTS='--basetemp=/tmp/drone-sim-phase2-final-review-build-once' make test-phase2` | built all seven stable images once; 14 passed in 136.55 s | 0 |
| `DRONE_SIM_PHASE2_IMAGES_BUILT=1 PYTEST_ADDOPTS='--basetemp=/tmp/drone-sim-phase2-final-review-no-build' uv run pytest -q tests/integration/test_phase2_compose.py` | final no-rebuild acceptance; 14 passed in 125.58 s | 0 |
| `PYTEST_ADDOPTS='--basetemp=/tmp/drone-sim-phase2-final-review-unit' make test-unit` | 574 passed, 11 expected host skips in 51.58 s | 0 |
| `PYTEST_ADDOPTS='--basetemp=/tmp/drone-sim-phase2-final-review-foundation' make test-foundation` | 3 passed in 11.02 s | 0 |
| `docker compose config --quiet` | no output | 0 |
| `docker compose --profile phase2 config --quiet` | no output | 0 |
| `uv run python -m py_compile tests/integration/test_phase2_compose.py tests/phase2/inspect_bundle.py orchestration/src/orchestration/controller.py artifacts/src/artifacts/runtime_node.py artifacts/src/artifacts/runtime_protocol.py artifacts/src/artifacts/manifest.py artifacts/src/artifacts/validation.py` | no output | 0 |
| `git diff --check` | no output | 0 |
| exact four-project container/network cleanup check | 0 labeled containers and absent `<project>_default` network for every final run | 0 |
| `git -C companion/comp2026 status --short` through the required temporary worktree link | no output; link removed and absence rechecked | 0 |

The final evidence root is
`/tmp/drone-sim-phase2-final-review-no-build/phase2-output0`.

## Final terminal bundles and timing

| Mode | Run ID | Manifest SHA-256 | Run start→manifest | FINALIZING→capture | Capture→manifest | FINALIZING→manifest |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| completed, 0 ms wall delay | `5fe64f68-4b71-4997-8bdd-f485eb5007c7` | `2272607579082f8c129c4d6339b38b4e733468cb09739cf5627a269154fba115` | 22.773276 s | 2.555527 s | 6.171242 s | 8.726769 s |
| completed, 17 ms wall delay | `ad74c6a4-635b-4d8f-9078-ea0761bf8b2e` | `688c8016354af62edb818ea7b9a755fdab9c46dd55f05b9a89d3ff6f27ad71a4` | 21.754857 s | 2.635674 s | 4.683804 s | 7.319478 s |
| failed, observer encoder fault | `3d11667b-3f97-43a6-8ce3-c516099cdcb9` | `a5d87df110e89692286c892124440c3e69f7473f1f8d4896c260b555b8485133` | 17.513297 s | 2.604370 s | 5.184542 s | 7.788912 s |
| aborted from durable RUNNING | `37a82af2-2b83-433c-85a9-960c559e3c9a` | `8e58a3ef4b1c1cf77f430c9d8bf56b2b447c59fe723309adebd8571ddea79cfd` | 19.778649 s | 1.314807 s | 8.638181 s | 9.952988 s |

For every row, FINALIZING→manifest equals FINALIZING→capture plus
capture→manifest within the gate's 1 µs tolerance.

## Digest and provenance facts

All four manifests contain the exact same seven-name mapping:

| Image | Manifest digest |
| --- | --- |
| `drone-sim-orchestration-runtime:phase2` | `200db0cac336e0a28340baf0b7cb1f61b1147a168a158dbd6665a25c262b6c22` |
| `drone-sim-artifacts-runtime:phase2` | `be4900e47022cf76d948fd4334de6df33b240f2abe5a07dc0aabf7bd7bc29b1b` |
| `drone-sim-synthetic-companion:phase2` | `e2f9f5b832cbbf1b3b6c581345ae5a7da23d7f3f4deebd2dd6366a61e44cc7ab` |
| `drone-sim-synthetic-ardupilot-sitl:phase2` | `2dd482fade5fa9a15213549179d44e8561fe8ff32025c590fcdac8c3222e057b` |
| `drone-sim-synthetic-gazebo:phase2` | `62fe43498eedd9ee79d7fb1ee4726dbaaa24aa8918ede3c705afe60c2e71f0d2` |
| `drone-sim-synthetic-electromagnet:phase2` | `79ea3fb7edad678868e161695f4a77f3fe14ae6a22d27c77ade22b180e1ce1b3` |
| `drone-sim-synthetic-scorekeeper:phase2` | `ef17081a3173fbf1bccef9953b2367c8b9f25cdba31cdef111a42e5a0b3f9c4d` |

Every manifest records the base source revision with `dirty=true`, as expected
for a pre-commit acceptance run. Both completed manifests record synthetic
fixture scoring `0.0 / 0.0`; declared checksum
`5b227e82e9217c34c342e37d6ce2872edc28c63cded78e38d3698c673ddefedb`
equals the independent SHA-256 of `tests/phase2/scoring.json`.

## Media, bag, and ArtifactStatus facts

Both completed runs contain identical videos: onboard is 5,036 bytes with
SHA-256 `8f5eab5c28ebb345719c7bf4d31a90f7fb13cb3ae74f3a23135aeb1271967481`;
observer is 5,330 bytes with SHA-256
`3e514d5b4bf655b6e5364af229c84c619af959e5092b4e67b3aea2eefa4b4bbf`.
Each is one playable H.264/yuv420p 320x240 20/1 stream with 40 recorder frames,
40 strict decoded frames, and a valid recorder-local predicate.

The 0 ms completed MCAP is 18,507,904 bytes with SHA-256
`34f8ef6853343ef40b4673d954be85c2087fc203fc6153c7025346911d6d8626`;
the 17 ms MCAP is 18,509,696 bytes with SHA-256
`824880d0841fcc9ba7bbfc030c66946d4de20733e04499fbc9fd52db19a21e1a`.
Both are structurally readable MCAP and semantically valid with exact counts:
clock 41, lifecycle 4, artifact status 2, ground truth 40, scenario 1, score 1,
and 40 image plus metadata messages for both streams. Simulation-normalized
topic stamps, frame IDs, image hashes, video hashes, and custom events matched
across wall delays.

Each completed bag records exactly two simulation-time-zero startup statuses
before its first clock: not-ready/incomplete with missing
`[onboard, observer, rosbag]`, then ready/incomplete with an empty missing list.
The real Jazzy late-joiner regression separately received the reliable,
transient-local final sample carrying `manifest.json`; that sample is
intentionally outside the closed bag. Every final run wrote
`.status/terminal-notified.json` with the matching run ID after manifest mtime
(40.9–288.9 ms later across the four runs).

The failed run retained readable 6-frame onboard and 5-frame observer videos;
its bag was structurally readable and semantically invalid as required for the
early terminal path. The aborted run retained a readable 5-frame observer video,
an explicit missing onboard record with recovery partial, and a structurally
readable early bag. No positive recorder count bypassed probe, full decode,
stream geometry, or validator assertions.

## Log, partial, status, and cleanup facts

Every bundle has nonempty raw Docker logs for all seven services and reparsable
structured logs for all seven modules. Completed structured line counts are
orchestration 7, artifacts 282, companion 3, `ardupilot_sitl` 3, Gazebo 83,
electromagnet 3, and scorekeeper 3. The failed run records 7/41/3/3/14/3/3;
the aborted run records 7/39/3/3/14/3/3 in the same module order.

Completed, failed, and aborted bundles retain the exact three inventoried
recorder diagnostics. The aborted bundle additionally retains only its
inventoried `video/onboard.mp4.partial`; policy regressions reject every raw,
structured, host, manifest-publication, unknown, or uninventoried candidate.

Repeated `abort`, `collect-results`, and `status` calls against the committed
aborted bundle returned exit 0 and identical
`ABORTED/operator_abort/manifest.json` facts without changing immutable bytes.
Fresh cleanup checks found zero project-labeled containers and no exact default
network for each of the four final run IDs.

For the companion check only, the absent untracked path `companion/comp2026`
was temporarily linked to `/home/willis/projects/drone_sim/companion/comp2026`.
The exact command `git -C companion/comp2026 status --short` exited 0 with no
output. The link was removed via the command's exit trap, and both `-e` and `-L`
absence checks then passed. No companion content was modified, copied,
committed, or pushed.

## Remaining concerns and non-claims

No open Phase 2 final-review finding remains. The verified feature is still the
synthetic Phase 2 lifecycle/artifact gate. It does not claim real Gazebo
Harmonic, real ArduPilot, mission completion semantics, electromagnet physics,
real scoring, maximum-score success, hostile same-UID artifact immutability, or
wall-derived simulation time.
