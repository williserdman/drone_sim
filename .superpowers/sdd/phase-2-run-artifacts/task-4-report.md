# Task 4 Report: Simulation-Time Camera Videos

## Scope and baseline

- Baseline: `d527abbf60a9bc2b2fae2621b68f8ed67cc9298f`
- Verification audit: `2026-08-23T15:05:36Z`
- Scope: two 20-FPS, 320x240 `rgb8` FFmpeg pipelines, their ROS-facing
  subscriptions/readiness/errors, read-only MP4 validation, and the pinned
  artifact-image FFmpeg extension.
- Out of scope remained untouched: controller/CLI, Compose runtime wiring,
  Docker-log partitioning, Gazebo, ArduPilot, missions, scoring, machine
  configuration, and `companion/comp2026`.

## RED evidence

The import-first test was written before production code and run as:

```text
uv run pytest artifacts/tests/test_video_adapter.py -v
```

It failed during collection with
`ModuleNotFoundError: No module named 'artifacts._adapters.video'` (exit 2,
zero tests collected).

Later test-first hardening cycles produced these expected failures before the
corresponding changes:

- Six focused failures proved FFmpeg stderr was not using a hardened retained
  log and finalization failure paths leaked retained directory descriptors.
- The real Jazzy test first failed because `VideoRecorderNode` was not an
  `rclpy.node.Node`.
- After making it spinable, the same Jazzy test exposed an internal-name
  collision: the implementation had replaced rclpy's `_subscriptions` list,
  breaking node destruction. The owned handles now use a separate public
  read-only tuple.
- A final process-race test reproduced `BrokenPipeError` from closing FFmpeg
  stdin after the encoder exited. Finalization now reports the immutable close
  failure, continues bounded process reaping, retains the partial, and releases
  its descriptors.

## Implementation and files

- `artifacts/src/artifacts/_adapters/video.py`
  - exact shell-free FFmpeg rawvideo command and `ffmpeg`/`ffprobe` preflight;
  - exact timestamp pairing in either arrival order with one pending item per
    side, immutable diagnostics, and fail-closed sequence/format checks;
  - hardened no-follow/single-link output and FFmpeg log paths;
  - caller-owned absolute deadline with bounded wait/TERM/KILL phases;
  - zero-exit, stable semantic validation before Linux `renameat2`
    `RENAME_NOREPLACE` publication;
  - descriptor-backed ffprobe JSON and full FFmpeg decode validation with
    before/after stable checksum enforcement.
- `artifacts/src/artifacts/recorder_node.py`
  - a spinable Jazzy node owning exactly `onboard` and `observer` recorders and
    four best-effort depth-5 subscriptions;
  - discovered subscription counts, aggregate readiness, and structured
    recorder errors without terminal-status selection.
- `artifacts/tests/test_video_adapter.py`
  - 57 host/container behavioral cases covering pairing, every frozen frame
    invariant, bounded pending state, process/finalization races and failures,
    filesystem collisions, semantic validation, a real recorder pipe, and the
    real Jazzy node/QoS shape.
- `artifacts/Dockerfile` and `artifacts/ffmpeg-packages.lock`
  - exact Ubuntu FFmpeg delta installation and build-time verification.
- `artifacts/src/artifacts/__init__.py`
  - exports `VideoStreamRecorder`, `VideoRecorderNode`, and `VideoValidator`
    plus their immutable diagnostic/result types.

## Package-lock evidence

- Base digest remains exactly
  `ros:jazzy-ros-base@sha256:2589a8fba5257307857890173c069852c2abf913a0be7970f172478baecb09e4`.
- The FFmpeg delta contains exactly 143 package/version rows and SHA-256
  `e799e5221753eec70b7ebac97f528c9d80315b504e10aa224f857638668798d7`.
- The clean locked build reported `0 upgraded, 143 newly installed` and its
  before/after `dpkg-query` delta exactly matched the lock before accepting the
  layer.
- The build also failed closed unless `ffmpeg -encoders` contained `libx264`
  and `ffprobe -version` succeeded.
- Final test image:
  `sha256:1fafd88a9cde6eb3cf7b66682dc0298cea5162319c3dcb772dbd4c3912db0e41`.

## Real four-frame evidence

The pinned image encoded four 230400-byte raw RGB frames, machine-probed the
result, and decoded the whole file. The observed fields were:

```json
{
  "codec_name": "h264",
  "codec_type": "video",
  "width": 320,
  "height": 240,
  "pix_fmt": "yuv420p",
  "avg_frame_rate": "20/1",
  "nb_read_frames": "4"
}
```

The independent full decode exited `0`. The container suite separately passed
the real `VideoStreamRecorder` path from four timestamp-paired messages through
`pipe:0`, final validation, collision-safe publication, and a second full
validation.

## GREEN and final verification

```text
uv run pytest artifacts/tests/test_video_adapter.py -v
53 passed, 4 skipped in 0.66s

uv run pytest artifacts/tests -v
163 passed, 8 skipped in 2.00s

docker build -f artifacts/Dockerfile --target test -t drone-sim-artifacts:test .
exit 0; image sha256:1fafd88a9cde6eb3cf7b66682dc0298cea5162319c3dcb772dbd4c3912db0e41

docker run --rm drone-sim-artifacts:test uv run pytest artifacts/tests/test_video_adapter.py -v
57 passed in 6.38s

docker run --rm drone-sim-artifacts:test
171 passed in 7.60s

uv run pytest -v
217 passed, 8 skipped in 7.61s

uv run python -m compileall -q artifacts/src/artifacts
exit 0

git diff --check
exit 0
```

The host skips are only the tests intentionally enforced by
`DRONE_SIM_REQUIRE_ROS_TESTS=1` in the artifact image: real FFmpeg generation,
the real recorder pipe, the full FFmpeg apt lock, and the real Jazzy node.

## Self-review

- Mutation review confirmed tests fail for wrong frame IDs/timestamp deltas,
  altered bytes/shape/encoding/step, ambiguous pending input, command or
  process-policy changes, output/log symlinks and hard links, final-name races,
  wrong probe fields/counts, decode failures, and file mutation during probes.
- FFmpeg stderr uses an owned append-only partial log rather than a pipe, so an
  error-producing encoder cannot deadlock finalization on an unread pipe.
- All finalize exits close parent resources while retaining failed partial
  output and immutable in-memory diagnostics.
- Validation passes the retained no-follow file descriptor to both semantic
  subprocesses and returns no stale checksum or diagnostics if any snapshot
  identity changes.
- The node is a real rclpy node in Jazzy and does not overwrite rclpy internals.
- No unresolved contract or package-lock contradiction remains. The host's
  legacy Docker builder is slow and emits its upstream deprecation warning;
  this does not affect the locked image or test results.
