# Task 4 Report: Simulation-Time Camera Videos

## Scope and baseline

- Baseline: `d527abbf60a9bc2b2fae2621b68f8ed67cc9298f`
- Verification audit: `2026-08-23T16:16:00Z`
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

Review fix round 1 added four deterministic critical RED cases. All four
failed before the fix: the partial pathname was not reserved, a deliberately
slow `stdin.close()` consumed about 101 ms despite a 10 ms deadline, semantic
subprocesses had no timeout input, and a pathname swap after validation
published attacker-controlled replacement bytes. The first production slice
made all four pass.

The next node-focused RED run had three failures: the node did not accept the
configured expected count/startup deadline, and image/metadata callback
exceptions escaped. Those three passed after transactional startup and callback
containment. A later full-suite run exposed an ABA weakness not reliably visible
in an isolated run: moving the same inode away and back need not alter its file
metadata. A retained-inode inotify watch now closes that transient-mutation gap;
the ABA, pathname-swap, and stale-result cases pass together on host and in the
pinned image.

## Implementation and files

- `artifacts/src/artifacts/_adapters/video.py`
  - shell-free FFmpeg rawvideo command and exact-name `libx264`/`ffprobe`
    preflight;
  - exact timestamp pairing in either arrival order with one pending item per
    side, immutable diagnostics, and fail-closed sequence/format checks;
  - hardened no-follow/single-link FFmpeg log paths plus `O_EXCL|O_NOFOLLOW`
    reservation of each output inode before process creation;
  - FFmpeg writes the retained inode via `/proc/self/fd/N` and `pass_fds`;
  - caller-owned absolute deadline bounds stdin close, wait/TERM/KILL,
    ffprobe, and strict full decode;
  - zero-exit, stable semantic validation before descriptor-bound Linux
    `linkat` no-clobber publication of that exact inode;
  - descriptor-backed ffprobe JSON, `-xerror` full decode, same-descriptor
    before/after hashes/fstats, parent-entry checks, and an inode mutation watch;
  - immutable configured expected frame count independent of observations,
    accepting canonical `COMPLETED`, `FAILED`, and `ABORTED` outcomes.
- `artifacts/src/artifacts/recorder_node.py`
  - a spinable Jazzy node owning exactly `onboard` and `observer` recorders and
    four best-effort depth-5 subscriptions;
  - discovered subscription counts, aggregate readiness, and structured
    recorder errors without terminal-status selection;
  - startup rollback under the caller deadline and contained subscription
    callbacks so one failed stream does not terminate the executor.
- `artifacts/tests/test_video_adapter.py`
  - 82 host/container behavioral cases covering pairing, every frozen frame
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
- The build also failed closed unless encoder field 2 equaled exactly
  `libx264` (so `libx264rgb` cannot satisfy the gate) and `ffprobe -version`
  succeeded.
- Final test image:
  `sha256:9e6890cbe198814c6b3723471bee65c6d4aa7c755e15b1e8bfec41c330d9f4a1`.

## Output-command security ruling

The original brief displayed the absolute partial pathname as FFmpeg's final
argument. That pathname cannot bind FFmpeg to the inode validated and later
published: a dangling symlink or pathname replacement can redirect one of
those stages. Security and exact inode identity therefore take precedence, as
the review explicitly directed. The semantic encoding settings remain frozen,
but the implementation reserves the named partial first and passes
`/proc/self/fd/N` plus `-y` so FFmpeg truncates/writes that already retained
regular inode. Publication uses `linkat` through the retained descriptor with
no replacement, verifies the final entry, and removes only the matching
partial link. The final file can never be the pathname replacement validated
or introduced by a race.

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
78 passed, 4 skipped in 1.31s

uv run pytest artifacts/tests -v
188 passed, 8 skipped in 3.65s

docker build -f artifacts/Dockerfile --target test -t drone-sim-artifacts:test .
exit 0; image sha256:9e6890cbe198814c6b3723471bee65c6d4aa7c755e15b1e8bfec41c330d9f4a1

docker run --rm drone-sim-artifacts:test uv run pytest artifacts/tests/test_video_adapter.py -v
82 passed in 10.45s

docker run --rm drone-sim-artifacts:test
196 passed in 8.69s

uv run pytest -v
242 passed, 8 skipped in 9.72s

uv run python -m compileall -q artifacts/src artifacts/tests
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
  subprocesses, hashes that descriptor twice, and returns no stale checksum or
  diagnostics if its inode, content, parent entry, or transient watch changes.
- Slow close, semantic timeouts, unexpected subprocess/validator/signal/wait
  failures, and throwing diagnostic sinks return immutable failure surfaces
  and do not leak retained descriptors.
- The pinned container demonstrated exact `libx264`, a working Linux inotify
  fd, the real four-frame fixture, strict full decode, and descriptor-bound
  publication.
- The node is a real rclpy node in Jazzy and does not overwrite rclpy internals.
- No unresolved contract or package-lock contradiction remains. The host's
  legacy Docker builder is slow and emits its upstream deprecation warning;
  this does not affect the locked image or test results.
