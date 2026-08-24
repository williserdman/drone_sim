# Phase 2 synthetic run-artifact verification

Verified tree: Task 8 fix round 1 changes atop `7a39758352b542104bc679af2af1dd0839b1263f`

Verification window: 2026-08-24 09:15–09:33 UTC

Platform: local Docker Engine, ROS 2 Jazzy runtime images, host `uv` test environment

## Gate results

| Command | Exit | Result |
| --- | ---: | --- |
| `make test-phase2` | 0 | one stable seven-image build; 13 passed in 162.25 s, comprising three public-CLI terminal tests and ten negative/allow-policy assertions |
| `DRONE_SIM_PHASE2_IMAGES_BUILT=1 uv run pytest -q tests/integration/test_phase2_compose.py` | 0 | final no-rebuild verification: 13 passed in 119.74 s |
| `make test-unit` | 0 | 553 passed, 10 expected host skips in 27.38 s |
| `make test-foundation` | 0 | 3 passed in 5.20 s |
| `docker compose config --quiet` | 0 | no output |
| `docker compose --profile phase2 config --quiet` | 0 | no output |
| `git diff --check` | 0 | no output |
| `uv run python -m py_compile tests/integration/test_phase2_compose.py tests/phase2/inspect_bundle.py orchestration/src/orchestration/controller.py` | 0 | no output |
| `git -C companion/comp2026 status --short` | 0 | no output |

For the exact companion command only, verification created the temporary untracked symlink `companion/comp2026` to `/home/willis/projects/drone_sim/companion/comp2026`, ran the command above, and removed the symlink. The linked companion repository was not modified, copied, committed, or pushed; the worktree has no remaining link.

`test-phase2` built once before all cases. Every production start continued to use `up --detach --no-build`.

## Stable image evidence

| Image | Local immutable image ID |
| --- | --- |
| `drone-sim-orchestration-runtime:phase2` | `d42a4503194c1ad4e7dfbb1aa53ea42efbb9ac9b4bc7dae8dad6c626dc2dd2e3` |
| `drone-sim-artifacts-runtime:phase2` | `b2f719a243b1ada11267183038db7d81ead6860a7fe2f2c0cef83599b0c506f4` |
| `drone-sim-synthetic-companion:phase2` | `90dc630531425c43d17a9d821cbfedb62f5e347ae89ad3fececfa78f8cd44ffe` |
| `drone-sim-synthetic-ardupilot-sitl:phase2` | `c3ac9f20eaec2ccf661130ebca9ac9a1fc40b30d5d57417b0154f7043f0f4781` |
| `drone-sim-synthetic-gazebo:phase2` | `8522f693b8b68c176cd26ef053ef5a3869cb26ddab20022630ce7e36473dc4e3` |
| `drone-sim-synthetic-electromagnet:phase2` | `7adbb1f7845dc5166a4628160970e43baabcd6196b92a01cdbabf6d2471f6802` |
| `drone-sim-synthetic-scorekeeper:phase2` | `62e945f5563bbd122efc1c6556e60888fcce71549d1537387e4ea62272bc2dae` |

Each representative manifest records the same seven-name digest mapping.

## Terminal bundles

| Terminal | Run ID | Bundle | Manifest SHA-256 | Wall duration |
| --- | --- | --- | --- | ---: |
| completed, zero delay | `638d540f-5b34-4d4e-8696-933cd10501b6` | `/tmp/pytest-of-willis/pytest-4558/phase2-output0/638d540f-5b34-4d4e-8696-933cd10501b6` | `c8cd458ccb9a4d4e545f5ba9fe133a0dd6862fda85e611337d1e96bd22deea25` | 37.339534 s |
| completed, 17 ms delay | `dd0ae4cf-89b1-4c90-82aa-8da8fa0894e2` | `/tmp/pytest-of-willis/pytest-4558/phase2-output0/dd0ae4cf-89b1-4c90-82aa-8da8fa0894e2` | `d274addacf526af576efdfe140cbfa57ae7026bf09848e2c9d74ad7f0f3d9fa1` | 26.285879 s |
| failed, observer fault | `2d2741fc-5487-4783-8669-ca3f293ccedc` | `/tmp/pytest-of-willis/pytest-4558/phase2-output0/2d2741fc-5487-4783-8669-ca3f293ccedc` | `82bb40e40122eb3436b7030cb8a8591baaa53d8f87c524e92df4ac179a9dd1d6` | 17.145711 s |
| aborted from durable RUNNING | `c713c2d1-878b-43a4-84b5-2cfa4ee2e453` | `/tmp/pytest-of-willis/pytest-4558/phase2-output0/c713c2d1-878b-43a4-84b5-2cfa4ee2e453` | `864687131bac9a53dab274242a43f5a77a22d07c563e0598c7191dbcefe8df06` | 20.086726 s |

All four manifests record synthetic fixture scoring `0.0 / 0.0`. Their strict lowercase declared scoring checksum is `5b227e82e9217c34c342e37d6ce2872edc28c63cded78e38d3698c673ddefedb`, independently equal to the SHA-256 of `tests/phase2/scoring.json`.

## Artifact and terminal facts

The read-only inspector ran inside `drone-sim-artifacts-runtime:phase2`. Each completed video had recorder-local frame count 40, exactly one H.264/yuv420p 320×240 `20/1` stream, a successful probe, a strict successful full decode, and 40 decoded-frame SHA-256 values. The zero-delay onboard and observer files were respectively 5,036 and 5,330 bytes with SHA-256 `8f5eab5c28ebb345719c7bf4d31a90f7fb13cb3ae74f3a23135aeb1271967481` and `3e514d5b4bf655b6e5364af229c84c619af959e5092b4e67b3aea2eefa4b4bbf`. The MCAP tree was 18,511,320 bytes with SHA-256 `b3c9b579f188f8e02b15239f772b549648617b03a9b67c7c19a61018321a887b`.

The completed MCAP inventory was exact: `/clock` 41; run state 4; artifact status 1; ground truth 40; scenario 1; score 1; and image plus metadata 40 for both streams. Types, lifecycle, IDs, simulation stamps, image payloads, custom events, and decoded video hashes passed the frozen contract and were normalized-identical across wall delays. Simulation time remained 0–2,000,000,000 ns and did not derive from wall time.

The failed run's bag structurally deserialized and its strict semantics correctly remained invalid. Recorder-local onboard/observer counts were 6/5; both probes and strict full decodes succeeded and produced exactly 6/5 decoded hashes. The observer required record remained explicitly invalid. The aborted run likewise retained a structurally readable bag and two positive 10-frame videos, each fully validated and producing exactly ten hashes. The acceptance predicate comes from `.status/artifacts-final.json`, never from the decoded-hash count itself; zero or absent counts require explicit missing/invalid recorder and manifest records. A corrupt-positive negative regression proves a positive recorder count cannot skip those assertions.

Every terminal bundle contained exactly the three inventoried frozen recorder diagnostics and no other partial: `logs/docker/ffmpeg-onboard.log.partial`, `logs/docker/ffmpeg-observer.log.partial`, and `logs/docker/rosbag2.log.partial`. The gate separately proves failed/aborted bundles may retain only the two explicit recorder recovery outputs, while rejecting manifest candidates, DockerLogCapture publication candidates, host publication sources after successful capture, structured/raw capture candidates, and unknown partials. All seven raw logs were nonempty. All seven structured logs reparsed; completed line counts were orchestration 7, artifacts 282, companion 3, `ardupilot_sitl` 3, gazebo 83, electromagnet 3, and scorekeeper 3.

Repeated public `abort`, `collect-results`, and `status` commands returned exit 0 and the exact canonical committed facts for `c713c2d1-878b-43a4-84b5-2cfa4ee2e453`: state `ABORTED`, reason `operator_abort`, and `manifest_path` `manifest.json`. The manifest SHA-256 stayed `864687131bac9a53dab274242a43f5a77a22d07c563e0598c7191dbcefe8df06`; the automated gate also compared every immutable non-control/status byte, size, and mtime.

After every case, no project-labeled container remained and every exact `<project>_default` network inspection exited 1. Cleanup also remained in `finally` paths.

## Exact non-claims

Phase 2 uses synthetic Gazebo and scoring fixtures. This evidence does not claim Gazebo Harmonic physics, a Gazebo-authoritative clock, native Gazebo state, real ArduPilot or lockstep, companion behavior, electromagnet physics, mission execution/scoring, or maximum-score acceptance. The `0.0 / 0.0` fixture proves infrastructure provenance only; it is not the eventual maximum-score goal.
