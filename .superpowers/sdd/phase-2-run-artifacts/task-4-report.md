# Task 4 Report: Simulation-Time Camera Videos

## Scope and baseline

- Baseline: `d527abbf60a9bc2b2fae2621b68f8ed67cc9298f`
- Verification audit: `2026-08-23T22:29:42Z`
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

Review fix round 2 began with 18 persisted focused failures covering the
remaining review seams: mutation after semantic validation and after the final
link, run-root/video-directory ABA moves, short/zero/interrupted raw-pipe
writes, post-open descriptor cleanup, missing/empty validation semantics,
per-chunk hash deadlines, real-child reaping after injected process-boundary
failures, bounded preflight/spawn, startup-deadline propagation, and rollback
of the recorder whose own start partially failed. The same selection then
passed `18 passed, 83 deselected`.

The round-2 self-review added six more focused RED cases before their fixes.
They reproduced a child handoff race exactly at the spawn deadline, leaked
resources for a malformed spawned process and for a directory `fstat` failure,
failure to kill a process that returned no stdin, incorrect `MISSING`
classification when the ffprobe executable—not the video—was absent, and an
A→B→A mutation occurring during the final post-link hash. Each now passes.

Review fix round 3 started with 10 deterministic failures and 3 passing
capability/race controls. The failures covered anonymous-output ownership,
read-only validation/publication, recoverable failure links, no-unlink startup
cleanup, and malformed over-reported writes. The same selection then passed
`10 passed, 110 deselected`; the exact and late spawn-handoff selection passed
`3 passed`.

Container verification later exposed two useful RED results. First, Docker's
writable overlay returned `EOPNOTSUPP` for `O_TMPFILE`; the recorder failed
closed as required, while the host bind-mounted run volume supported the
operation. The test target now supplies an anonymous Docker volume for its
pytest run roots so the mandatory plain image commands exercise production-like
volume semantics without a fallback. Second, the default full suite exposed a
thread-order-dependent spawn cleanup cutoff: it was computed after the worker
started. The real-child test failed intermittently, then passed 10 consecutive
runs after the caller computed and owned the cutoff before launching the worker.

## Implementation and files

- `artifacts/src/artifacts/_adapters/video.py`
  - shell-free FFmpeg rawvideo command and exact-name `libx264`/`ffprobe`
    preflight;
  - exact timestamp pairing in either arrival order with one pending item per
    side, immutable diagnostics, and fail-closed sequence/format checks;
  - hardened no-follow/single-link FFmpeg log paths plus an anonymous Linux
    `O_TMPFILE` inode in the retained video directory before process creation;
  - FFmpeg writes a duplicate of that exact anonymous inode through
    `/proc/self/fd/N` and `pass_fds`;
  - caller-owned absolute deadline bounds stdin close, wait/TERM/KILL,
    startup preflight/process creation, descriptor hashing, ffprobe, and strict
    full decode;
  - after confirmed encoder exit: `fsync`, exact-inode read-only reopen,
    permission seal to `0444`, closure of every writable descriptor, stable
    semantic validation, then one descriptor-bound Linux `linkat` no-clobber
    publication of that exact inode;
  - descriptor-backed ffprobe JSON, `-xerror` full decode, same-descriptor
    before/after hashes/fstats, and retained run-root/video/file mutation
    watches;
  - complete raw-pipe writes across short/interrupted writes, fail-closed zero
    progress, exception-safe output preparation, and bounded TERM/KILL fallback
    even when an injected process wrapper throws;
  - synchronized process handoff with a caller-owned cleanup cutoff computed
    before worker launch: exactly one side owns the process, output duplicate,
    and log; late processes are killed/reaped and close their own resources;
  - immutable configured expected frame count independent of observations,
    accepting canonical `COMPLETED`, `FAILED`, and `ABORTED` outcomes.
- `artifacts/src/artifacts/recorder_node.py`
  - a spinable Jazzy node owning exactly `onboard` and `observer` recorders and
    four best-effort depth-5 subscriptions;
  - discovered subscription counts, aggregate readiness, and structured
    recorder errors without terminal-status selection;
  - startup rollback (including the recorder whose own start failed) under the
    caller deadline and contained subscription callbacks so one failed stream
    does not terminate the executor.
- `artifacts/tests/test_video_adapter.py`
  - 119 host/container behavioral cases covering pairing, every frozen frame
    invariant, bounded pending state, process/finalization races and failures,
    filesystem collisions, semantic validation, a real recorder pipe, and the
    real Jazzy node/QoS shape.
- `artifacts/Dockerfile` and `artifacts/ffmpeg-packages.lock`
  - exact Ubuntu FFmpeg delta installation and build-time verification;
  - test-target-only anonymous `/test-run-volume` plus pytest basetemp routing,
    so plain container test commands use an `O_TMPFILE`-capable local volume.
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
  `sha256:3d93ddfe52b03402df8c6c2cd4ac2b4062058de59607dd7a0703ee95c2b4d0e4`.

## Output-command security ruling

The original brief displayed the absolute partial pathname as FFmpeg's final
argument. That pathname cannot bind FFmpeg to the inode validated and later
published: a dangling symlink or pathname replacement can redirect one of
those stages. Security and exact inode identity therefore take precedence, as
the review explicitly directed. The semantic encoding settings remain frozen,
but active recording is anonymous: `O_TMPFILE` creates an unlinked regular
inode in the retained video directory and FFmpeg receives its duplicate through
`/proc/self/fd/N`. After FFmpeg stops, the recorder reopens the exact inode
read-only, verifies identity, seals mode `0444`, and closes all writable fds.
Validation sees only that read-only unlinked descriptor. A single no-clobber
link publishes it directly as final when valid, or as `.partial` when a
nonempty failed/invalid encode is recoverable. There is no active named partial,
check-then-unlink cleanup, rollback unlink, or post-link mutating operation.

The target's default capability set is unchanged (`CapEff
00000000a80425fb`). Direct `linkat(AT_EMPTY_PATH)` returned `ENOENT` without
`CAP_DAC_READ_SEARCH`, so the controller approved `linkat` from the
kernel-controlled `/proc/self/fd/N` source with `AT_SYMLINK_FOLLOW`. Real
container evidence proved source and destination `(st_dev, st_ino)` equality,
mode `0444`, one link, and `EEXIST` no-clobber behavior. No Compose capability
or runtime fallback was added.

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
validation. It also encoded a second real four-frame stream, injected semantic
rejection, preserved the exact anonymous inode as `.partial`, and independently
probed and fully decoded that recovered file as valid.

## GREEN and final verification

```text
uv run pytest artifacts/tests/test_video_adapter.py -v
113 passed, 6 skipped in 2.92s

uv run pytest artifacts/tests -v
223 passed, 10 skipped in 3.99s

docker build -f artifacts/Dockerfile --target test -t drone-sim-artifacts:test .
exit 0; image sha256:3d93ddfe52b03402df8c6c2cd4ac2b4062058de59607dd7a0703ee95c2b4d0e4

docker run --rm drone-sim-artifacts:test uv run pytest artifacts/tests/test_video_adapter.py -v
119 passed in 9.44s

docker run --rm drone-sim-artifacts:test
233 passed in 14.47s

uv run pytest -v
277 passed, 10 skipped in 11.75s

uv run python -m compileall -q artifacts/src artifacts/tests
exit 0

git diff --check
exit 0
```

The host skips are only the tests intentionally enforced by
`DRONE_SIM_REQUIRE_ROS_TESTS=1` in the artifact image: real FFmpeg generation,
the real recorder pipe and recovery path, the full FFmpeg apt lock, and the real
Jazzy node.

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
  subprocesses, hashes that descriptor under the shared deadline, and returns
  no stale checksum or diagnostics if its inode, content, retained parent
  chain, or transient watch changes.
- The recorder-level parent watch spans validation to the single exact-inode
  link. Content cannot change across that boundary because validation owns only
  the sealed read-only anonymous descriptor and every writable duplicate has
  already closed. Publication has no fallible post-link mutation or rollback.
- Slow close, semantic timeouts, unexpected subprocess/validator/signal/wait
  failures, and throwing diagnostic sinks return immutable failure surfaces
  and do not leak retained descriptors.
- Startup preflight and process creation consume only the caller's remaining
  deadline. A synchronized single-owner handoff gives cancellation cleanup a
  pre-reserved budget; timeout races and malformed post-spawn contracts kill
  and reap any child and close the worker-owned output/log descriptors.
- The pinned container demonstrated exact `libx264`, Linux inotify, anonymous
  `O_TMPFILE` creation on both bind and anonymous local volumes, exact read-only
  descriptor linking/no-clobber, real four-frame success and recoverable
  failure fixtures, and strict full decode.
- The node is a real rclpy node in Jazzy and does not overwrite rclpy internals.
- No unresolved contract or package-lock contradiction remains. The host's
  legacy Docker builder is slow and emits its upstream deprecation warning;
  this does not affect the locked image or test results.
