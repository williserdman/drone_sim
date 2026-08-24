# Phase 2 synthetic run-artifact verification

Verified tree: Phase 2 final-review fix changes atop `eb3dcb77a597c58d59eac5933aa0ab0bd26cd5b2`

Recovery/final gate window: 2026-08-24 10:50–11:10 UTC

Platform: local Docker Engine, ROS 2 Jazzy runtime images, host `uv` test environment

## Gate results

| Command | Exit | Result |
| --- | ---: | --- |
| `COMPOSE_PROGRESS=plain PYTEST_ADDOPTS='--basetemp=/tmp/drone-sim-phase2-final-review-build-once' make test-phase2` | 0 | one stable seven-image build; 14 passed in 136.55 s, including the four-manifest exact digest/timing check |
| `DRONE_SIM_PHASE2_IMAGES_BUILT=1 PYTEST_ADDOPTS='--basetemp=/tmp/drone-sim-phase2-final-review-no-build' uv run pytest -q tests/integration/test_phase2_compose.py` | 0 | final no-rebuild verification: 14 passed in 125.58 s |
| `PYTEST_ADDOPTS='--basetemp=/tmp/drone-sim-phase2-final-review-unit' make test-unit` | 0 | 574 passed, 11 expected host skips in 51.58 s |
| `PYTEST_ADDOPTS='--basetemp=/tmp/drone-sim-phase2-final-review-foundation' make test-foundation` | 0 | 3 passed in 11.02 s |
| `docker compose config --quiet` | 0 | no output |
| `docker compose --profile phase2 config --quiet` | 0 | no output |
| `git diff --check` | 0 | no output |
| `uv run python -m py_compile tests/integration/test_phase2_compose.py tests/phase2/inspect_bundle.py orchestration/src/orchestration/controller.py artifacts/src/artifacts/runtime_node.py artifacts/src/artifacts/runtime_protocol.py artifacts/src/artifacts/manifest.py artifacts/src/artifacts/validation.py` | 0 | no output |
| source-exact real ROS late-joiner and MCAP focused tests | 0 | 2 passed in 1.38 s in the artifact test image |
| `git -C companion/comp2026 status --short` | 0 | no output |

For the exact companion command only, verification created the temporary untracked symlink `companion/comp2026` to `/home/willis/projects/drone_sim/companion/comp2026`, ran the command above, and removed the symlink. The linked companion repository was not modified, copied, committed, or pushed; the worktree has no remaining link.

`test-phase2` built once before all cases. Every production start continued to use `up --detach --no-build`.

## Stable image evidence

| Image | Manifest digest |
| --- | --- |
| `drone-sim-orchestration-runtime:phase2` | `200db0cac336e0a28340baf0b7cb1f61b1147a168a158dbd6665a25c262b6c22` |
| `drone-sim-artifacts-runtime:phase2` | `be4900e47022cf76d948fd4334de6df33b240f2abe5a07dc0aabf7bd7bc29b1b` |
| `drone-sim-synthetic-companion:phase2` | `e2f9f5b832cbbf1b3b6c581345ae5a7da23d7f3f4deebd2dd6366a61e44cc7ab` |
| `drone-sim-synthetic-ardupilot-sitl:phase2` | `2dd482fade5fa9a15213549179d44e8561fe8ff32025c590fcdac8c3222e057b` |
| `drone-sim-synthetic-gazebo:phase2` | `62fe43498eedd9ee79d7fb1ee4726dbaaa24aa8918ede3c705afe60c2e71f0d2` |
| `drone-sim-synthetic-electromagnet:phase2` | `79ea3fb7edad678868e161695f4a77f3fe14ae6a22d27c77ade22b180e1ce1b3` |
| `drone-sim-synthetic-scorekeeper:phase2` | `ef17081a3173fbf1bccef9953b2367c8b9f25cdba31cdef111a42e5a0b3f9c4d` |

Each representative manifest records the same seven-name digest mapping.

## Terminal bundles

| Terminal | Run ID | Manifest SHA-256 | Run start→manifest | FINALIZING→capture | Capture→manifest | FINALIZING→manifest |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| completed, zero delay | `5fe64f68-4b71-4997-8bdd-f485eb5007c7` | `2272607579082f8c129c4d6339b38b4e733468cb09739cf5627a269154fba115` | 22.773276 s | 2.555527 s | 6.171242 s | 8.726769 s |
| completed, 17 ms delay | `ad74c6a4-635b-4d8f-9078-ea0761bf8b2e` | `688c8016354af62edb818ea7b9a755fdab9c46dd55f05b9a89d3ff6f27ad71a4` | 21.754857 s | 2.635674 s | 4.683804 s | 7.319478 s |
| failed, observer fault | `3d11667b-3f97-43a6-8ce3-c516099cdcb9` | `a5d87df110e89692286c892124440c3e69f7473f1f8d4896c260b555b8485133` | 17.513297 s | 2.604370 s | 5.184542 s | 7.788912 s |
| aborted from durable RUNNING | `37a82af2-2b83-433c-85a9-960c559e3c9a` | `8e58a3ef4b1c1cf77f430c9d8bf56b2b447c59fe723309adebd8571ddea79cfd` | 19.778649 s | 1.314807 s | 8.638181 s | 9.952988 s |

The bundle root is `/tmp/drone-sim-phase2-final-review-no-build/phase2-output0`. Each row
satisfies FINALIZING→manifest = FINALIZING→capture + capture→manifest within the
acceptance tolerance.

All four manifests record synthetic fixture scoring `0.0 / 0.0`. Their strict lowercase declared scoring checksum is `5b227e82e9217c34c342e37d6ce2872edc28c63cded78e38d3698c673ddefedb`, independently equal to the SHA-256 of `tests/phase2/scoring.json`.

## Artifact and terminal facts

The read-only inspector ran inside `drone-sim-artifacts-runtime:phase2`. Each completed video had recorder-local frame count 40, exactly one H.264/yuv420p 320×240 `20/1` stream, a successful probe, a strict successful full decode, and 40 decoded-frame SHA-256 values. The zero-delay onboard and observer files were respectively 5,036 and 5,330 bytes with SHA-256 `8f5eab5c28ebb345719c7bf4d31a90f7fb13cb3ae74f3a23135aeb1271967481` and `3e514d5b4bf655b6e5364af229c84c619af959e5092b4e67b3aea2eefa4b4bbf`. Its MCAP tree was 18,507,904 bytes with SHA-256 `34f8ef6853343ef40b4673d954be85c2087fc203fc6153c7025346911d6d8626`; the 17 ms MCAP was 18,509,696 bytes with SHA-256 `824880d0841fcc9ba7bbfc030c66946d4de20733e04499fbc9fd52db19a21e1a`.

The completed MCAP inventory was exact: `/clock` 41; run state 4; artifact status 2; ground truth 40; scenario 1; score 1; and image plus metadata 40 for both streams. The first artifact status is not ready with exact missing `[onboard, observer, rosbag]`; the second is ready with no missing names; both are stamped zero and precede each bag's first clock. Types, lifecycle, IDs, simulation stamps, image payloads, custom events, and decoded video hashes passed the frozen contract and were normalized-identical across wall delays. Simulation time remained 0–2,000,000,000 ns and did not derive from wall time. A real ROS late-joiner test separately received the reliable/transient-local final status after manifest commit because that notification intentionally occurs after the bag closes.

The failed run's bag structurally deserialized and its strict semantics correctly remained invalid. Recorder-local onboard/observer counts were 6/5; both probes and strict full decodes succeeded and produced exactly 6/5 decoded hashes. The observer required record remained explicitly invalid. The aborted run likewise retained a structurally readable early bag and a positive 5-frame observer video with full validation; onboard was explicitly missing and retained only its inventoried recovery partial. The acceptance predicate comes from `.status/artifacts-final.json`, never from the decoded-hash count itself; zero or absent counts require explicit missing/invalid recorder and manifest records. A corrupt-positive negative regression proves a positive recorder count cannot skip those assertions.

Every terminal bundle contained the exact three inventoried frozen recorder diagnostics: `logs/docker/ffmpeg-onboard.log.partial`, `logs/docker/ffmpeg-observer.log.partial`, and `logs/docker/rosbag2.log.partial`. The aborted bundle additionally retained only its inventoried onboard recovery partial. The gate proves failed/aborted bundles may retain only the two explicit recorder recovery outputs, while rejecting manifest candidates, DockerLogCapture publication candidates, host publication sources after successful capture, structured/raw capture candidates, and unknown partials. All seven raw logs were nonempty. All seven structured logs reparsed; completed line counts were orchestration 7, artifacts 282, companion 3, `ardupilot_sitl` 3, gazebo 83, electromagnet 3, and scorekeeper 3.

Repeated public `abort`, `collect-results`, and `status` commands returned exit 0 and the exact canonical committed facts for `37a82af2-2b83-433c-85a9-960c559e3c9a`: state `ABORTED`, reason `operator_abort`, and `manifest_path` `manifest.json`. The manifest SHA-256 stayed `8e58a3ef4b1c1cf77f430c9d8bf56b2b447c59fe723309adebd8571ddea79cfd`; the automated gate also compared every immutable non-control/status byte, size, and mtime.

After every case, no project-labeled container remained and every exact `<project>_default` network inspection exited 1. A final label query independently confirmed zero containers and networks for all four listed run IDs. Cleanup also remained in `finally` paths.

## Exact non-claims

Phase 2 uses synthetic Gazebo and scoring fixtures. This evidence does not claim Gazebo Harmonic physics, a Gazebo-authoritative clock, native Gazebo state, real ArduPilot or lockstep, companion behavior, electromagnet physics, mission execution/scoring, or maximum-score acceptance. The `0.0 / 0.0` fixture proves infrastructure provenance only; it is not the eventual maximum-score goal.
