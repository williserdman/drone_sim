# Phase 2 synthetic run-artifact verification

Tested implementation commit: `a08f52769f0eb3164f7c80f436df977356bfc442`

Verification window: 2026-08-24 08:42:53–08:54:20 UTC

Platform: local Docker Engine, ROS 2 Jazzy runtime images, host `uv` test environment

## Gate results

| Command | Exit | Result |
| --- | ---: | --- |
| `make test` | 0 | 549 passed, 10 expected host skips; 3 Phase 1 passed; 3 Phase 2 passed |
| `docker compose config --quiet` | 0 | no output |
| `docker compose --profile phase2 config --quiet` | 0 | no output |
| `git diff --check` | 0 | no output |
| `git -C companion/comp2026 status --short` | 128 | assigned worktree omits the untracked nested repository |
| `git -C /home/willis/projects/drone_sim/companion/comp2026 status --short` | 0 | no output; main-worktree nested repository is clean at `b903eddb56ff1319be219cb1edd84034dba9f4b4` |

`test-phase2` issued one `docker compose --profile phase2 build`, then ran every CLI case with `DRONE_SIM_PHASE2_IMAGES_BUILT=1`; production Compose startup remained `up --detach --no-build`.

## Stable image evidence

| Image | Local immutable image ID |
| --- | --- |
| `drone-sim-orchestration-runtime:phase2` | `9f2abc95b37c3dd8dff10fbd237fcfdd52f1e6cabad0016cd158a0ad9165acc5` |
| `drone-sim-artifacts-runtime:phase2` | `4bea948cb892487c3a32a6779df93c478be246bcc30461e4a6c2433d8440e901` |
| `drone-sim-synthetic-companion:phase2` | `64d2cec58ad57157c939d27160f88d200ded722eafc00124b4a871f3f73a8efe` |
| `drone-sim-synthetic-ardupilot-sitl:phase2` | `d60b7f2fa0ad65d961ea781b8d9bd6b9062475c78e56463111815a0222135dc3` |
| `drone-sim-synthetic-gazebo:phase2` | `95d7451a1f858f6fa953b244dc4ace5d3eb5314a78c36f3df2c12ed6a7157a8d` |
| `drone-sim-synthetic-electromagnet:phase2` | `287e1e019c387e8dee92adcd63667e6334d2e00ac970fc857e5912a67cfd9ce3` |
| `drone-sim-synthetic-scorekeeper:phase2` | `9fbdccfbba52ad433863c6ef0d63eaadac8bde2a8c09c459e606bef9ca42cef3` |

## Terminal bundles

| Terminal | Run ID | Bundle | Manifest SHA-256 | Wall duration | FINALIZING to manifest |
| --- | --- | --- | --- | ---: | ---: |
| completed, zero delay | `985b5fd9-30f0-469d-bbed-c2f37d6f1180` | `/tmp/pytest-of-willis/pytest-4552/phase2-output0/985b5fd9-30f0-469d-bbed-c2f37d6f1180` | `4890340e9ee4cb32ddc74afe213fb44e84a8b3460fd1ea1916d181756f2fcea3` | 22.119050 s | 6.325604 s |
| completed, 17 ms delay | `f0dd011e-1380-4bf9-bdc4-e42132f03699` | `/tmp/pytest-of-willis/pytest-4552/phase2-output0/f0dd011e-1380-4bf9-bdc4-e42132f03699` | `efbbb1a0f5a9613d000e7fd3ccf1e7942ee16c5faab07526a3a45af9baf79219` | 22.620563 s | acceptance-bounded |
| failed, observer fault | `d8649952-bba5-4bc5-8287-82eb39848c9f` | `/tmp/pytest-of-willis/pytest-4552/phase2-output0/d8649952-bba5-4bc5-8287-82eb39848c9f` | `6ed3db8cc7b578d796361be3d62abc1fc204b9b59641d568b4d9c35d8233cb9a` | 19.713515 s | 7.374521 s |
| aborted from durable RUNNING | `e8e6c620-ffa5-44a8-aa23-e46c874a5c5a` | `/tmp/pytest-of-willis/pytest-4552/phase2-output0/e8e6c620-ffa5-44a8-aa23-e46c874a5c5a` | `b7304f85fabaaea788304cf33540ce53864ca6798c8b90b38328df2b4da7799c` | 24.487924 s | 8.165164 s |

The completed manifest records `0.0 / 0.0`; its scoring checksum is `5b227e82e9217c34c342e37d6ce2872edc28c63cded78e38d3698c673ddefedb`, exactly the SHA-256 of `tests/phase2/scoring.json` declared by the synthetic score result.

## Completed semantic facts

The read-only inspector ran inside `drone-sim-artifacts-runtime:phase2`. Both MP4s contain exactly one video stream: H.264, `yuv420p`, 320×240, `20/1`, 40 frames. FFprobe succeeded, full decode succeeded, and 40 decoded-frame SHA-256 values were emitted per stream. The zero-delay files were:

- onboard: 5,036 bytes, SHA-256 `8f5eab5c28ebb345719c7bf4d31a90f7fb13cb3ae74f3a23135aeb1271967481`;
- observer: 5,330 bytes, SHA-256 `3e514d5b4bf655b6e5364af229c84c619af959e5092b4e67b3aea2eefa4b4bbf`;
- MCAP tree: 18,513,153 bytes, SHA-256 `f1685021dc1ba7168867b65e9a0423be914818bf0707c489762f31948989ce54`.

The MCAP inventory was exact: `/clock` 41; `/simulation/run_state` 4; `/simulation/artifact_status` 1; ground truth 40; scenario 1; score 1; and image plus metadata 40 for each onboard/observer stream. Types matched the frozen ten-topic contract. Lifecycle was `STARTING, READY, RUNNING, FINALIZING`; artifact-ready was serialized at record index 1 and the first clock at index 3. Frame IDs were 0–39, image/metadata stamps were paired at 50 ms intervals, and normalized image payload, custom-event, simulation-stamp, ID, and decoded-frame hashes were identical across the two wall delays.

All seven module JSONL logs were nonempty and reparsed. Zero-delay line counts were orchestration 7, artifacts 282, companion 3, `ardupilot_sitl` 3, gazebo 83, electromagnet 3, and scorekeeper 3. Every manifest file/tree size and SHA-256, including raw Docker logs and the three allowed recorder partials, was recomputed independently. No `.control` or `.status` path was inventoried.

## Failed, aborted, immutability, and cleanup facts

The injected observer failure exited 1. Its bag was structurally readable with seven clocks, six ground-truth/image/metadata samples per stream, and no scenario or score sample; strict bag semantics therefore correctly remained invalid. The onboard video fully decoded with six frames. The observer diagnostic record was explicitly invalid while its five-frame retained video decoded, and the inventoried observer FFmpeg partial remained present.

The concurrent abort observed exact durable `RUNNING`, exited 130, and committed `ABORTED`. Its retained bag structurally deserialized all messages; both videos fully decoded (seven onboard, six observer frames). A second abort, one status, and repeated `collect-results` calls returned consistent terminal facts. For all terminal cases, the tests compared every non-control/status output byte, size, and mtime around read-only result commands; no committed output changed.

For each representative project, Docker reported 0 labeled containers and 0 `<project>_default` networks after exit. Cleanup assertions also ran in `finally` paths.

## Exact non-claims

Phase 2 uses synthetic Gazebo and scoring fixtures. This evidence does not claim Gazebo Harmonic physics, a Gazebo-authoritative clock, native Gazebo state, ArduPilot lockstep, companion behavior, electromagnet physics, mission scoring, or maximum-score acceptance. The `0.0 / 0.0` fixture proves infrastructure provenance only; it is not the eventual maximum-score goal.
