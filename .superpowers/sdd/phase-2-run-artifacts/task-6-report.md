# Task 6 Report: Simulation Operator Lifecycle

## Scope and baseline

- Baseline: `065cbad9e72901c86dcefaa6221c905f1395c1d4`
- Verification audit: `2026-08-24T02:30:31Z`
- Scope: host-owned run allocation and status protocol, policy-free Compose
  subprocess boundary, foreground lifecycle controller, structured host events,
  strict runtime artifact-report validation, manifest finalization, and the four
  installed operator commands.
- Out of scope remained untouched: Task 7 Compose services and runtime nodes,
  Dockerfiles, ROS messages, Gazebo, ArduPilot, mission/scoring behavior,
  machine configuration, and `companion/comp2026`.

## Frozen contract rulings

- `start` streams common-schema `StructuredEvent` JSONL while running and emits
  one typed final result last. The other three commands emit exactly one result
  object and no human stdout prose.
- Task 7 owns `.status/runtime-running.json`; Task 6 validates it before
  durably publishing operator `RUNNING`.
- Compose startup is detached `up --detach --no-build`. Task 5's frozen
  per-service log argv is validated and augmented at the subprocess boundary,
  with remaining time recomputed for all seven calls.
- `.status/artifacts-final.json` has exact top-level
  `{run_id, complete, records}` and three exact-path records with exact keys.
  Host validators independently recompute safe size and SHA-256 and require
  agreement with the runtime semantic report.
- The single finalization budget reserves
  `min(5 seconds, finalization_wall_seconds / 5)` for `compose down`.

The root/module interface documents, Phase 2 plan, and SDD ledger preserve
these rulings for Task 7 consumers.

## RED evidence

The public Task 6 test surface was persisted before production code and run as:

```text
uv run pytest orchestration/tests/test_status_store.py \
  orchestration/tests/test_controller.py orchestration/tests/test_cli.py -v
```

Collection failed with three missing-module errors for `status_store`,
`controller`, and `cli`. Production implementation began only after this real
import-first RED.

Subsequent focused RED-to-GREEN cycles exposed and retained regressions for:

- treating the controller's own COMPLETE finalization request as a competing
  terminal cause, initially leaving eight controller cases failed;
- JSON `NaN`/`Infinity` acceptance and an incomplete manifest inventory;
- teardown failures disappearing instead of becoming durable diagnostics;
- output-root escape through a symlinked ancestor;
- Compose startup and adapter-construction exceptions bypassing finalization;
- read-only lookup creating a missing output root;
- source-provenance Git calls resetting rather than consuming their shared
  deadline; and
- strict report/hash validation and teardown-reserve behavior.

Each correction followed a focused failing test, the smallest production
change, and a focused passing rerun. The final Task 6 surface contains 68 cases.

## Implementation and files

- `orchestration/src/orchestration/status_store.py`
  - exclusive, UUID-contained run allocation and descriptor-safe protocol
    reads;
  - duplicate-key/nonfinite rejecting JSON, regular/single-link/stable snapshot
    checks, a 4 MiB limit, and atomic file plus directory durability;
  - locked first-cause-wins terminal controls, idempotent abort, runtime-owned
    status reads, and read-only validated result collection.
- `orchestration/src/orchestration/_adapters/compose.py`
  - shell-free, explicit-project Compose commands and fixed environment;
  - bounded detached startup, status/stop/down, Task 5 log-command augmentation,
    and digest inspection with decreasing shared timeout.
- `orchestration/src/orchestration/controller.py`
  - injected clocks, factories, UUID generation, and event output;
  - exact lifecycle ordering, cause races, Ctrl-C to ABORTED/130, startup,
    run, finalization, and teardown deadlines;
  - durable host structured events and Task 5 `host_events=True` integration;
  - strict three-record semantic-report parser and specialized safe
    file/tree checksum validators;
  - read-only source/image/configuration provenance and sole production use of
    `ArtifactSession.finalize(FinalizationInput)` before terminal notification.
- `orchestration/src/orchestration/cli.py` and `__main__.py`
  - exact four-command installed interface, deterministic typed JSON results,
    absolute explicit output-root checks, and controlled exit 2 diagnostics.
- `orchestration/tests/test_{status_store,controller,cli}.py`
  - filesystem safety/durability, Compose arrays and budgets, lifecycle success
    and every required failure, artifact report/hash mismatch, host events,
    concurrency, output shape, and exit codes.
- Root/orchestration workspace metadata and `uv.lock`
  - install both workspace packages, make orchestration depend on artifacts,
    and register `drone-sim = "orchestration.cli:main"`.
- `artifacts/src/artifacts/__init__.py`
  - removes compatibility-only `build_manifest` from the public export surface;
    production compatibility remains internal for existing tests.

## Self-review

- Controller policy is isolated from the Compose subprocess adapter; tests use
  injected boundaries rather than ambient Docker or wall-clock state.
- Every command has an explicit project directory, project name, environment,
  argument list, merged output, and bounded timeout. No shell or Docker socket
  is introduced.
- Status reads cannot create a missing output root. Run and protocol paths
  reject symlink ancestors, hard links, non-regular files, corrupt/duplicate
  JSON, unstable snapshots, and pre-existing unsafe targets.
- The primary terminal cause is immutable after first observation; later
  failures are retained as diagnostics. Every path after a Compose start
  attempts finalization and bounded teardown.
- Requested completion cannot become COMPLETED unless log capture, exact report
  parsing, three specialized validators, and report/host size/hash agreement
  all succeed. ABORTED remains ABORTED even with valid artifacts.
- The manifest is committed before terminal notification and no artifact is
  modified afterward. `collect-results` reconstructs and validates the domain
  manifest without creating or rewriting the run.
- A final cleanup pass removed unused imports/test bookkeeping. The focused
  controller suite remained green afterward.

## GREEN and final verification

```text
uv run pytest orchestration/tests/test_status_store.py \
  orchestration/tests/test_controller.py orchestration/tests/test_cli.py -q
68 passed in 3.51s

uv run pytest orchestration/tests -q
112 passed in 7.70s

uv run pytest artifacts/tests orchestration/tests tests/contracts -q
432 passed, 10 skipped in 18.35s

uv run pytest -q
435 passed, 10 skipped in 18.86s

uv run drone-sim --help
usage: drone-sim [-h] {start,status,abort,collect-results} ...

uv run python -m compileall -q orchestration/src orchestration/tests
exit 0

uv lock --check
Resolved 15 packages in 2ms

git diff --check
exit 0
```

The ten skips are pre-existing environment-gated ROS/container/real-FFmpeg
cases; no Task 6 case is skipped.

## Review fix round 1 of 5

Independent review found one critical shared-deadline gap and four important
boundary gaps. The first focused RED run collected 180 affected cases and
reported `14 failed, 166 passed`. Those failures independently reproduced
missing cooperative deadline seams, permissive partial Compose health,
post-manifest status replacement, a second-event `BrokenPipeError` escaping
finalization, output-root creation through a symlink, and missing profile
activation. A later runtime-status callback test produced one additional
focused RED (`TypeError` for the missing keyword seam).

The fix preserves all public call forms and adds:

- optional cooperative deadline checks through Docker-log collection,
  per-line parsing/routing, candidate writes/publication, protocol-file reads,
  regular-file chunks, directory-tree entries, optional discovery, canonical
  manifest encoding, validation, and no-clobber commit;
- separate optional `ArtifactSession` work and commit checks within one total
  deadline. Work exhaustion stops hashing, marks the current and unfinished
  required records timeout-invalid, and attempts a FAILED or ABORTED manifest
  inside the equal manifest reserve; teardown retains its equal reserve;
- exact `ps --all --format json` validation for the frozen seven unique
  running/restarting services and `COMPOSE_PROFILES=phase2` on every command;
- descriptor-relative, no-follow output-root construction that rejects an
  existing symlink ancestor before creating any target-side component;
- immutable manifest authority after validation: terminal-control,
  notification, observability, and teardown failures append diagnostics only;
- fail-once stdout/file host-event handling so append, stream, and close errors
  cannot recursively bypass log capture, manifest finalization, or teardown.

The manifest-reserve ruling supersedes the initial teardown-only reserve. A
fake that advances the monotonic clock beyond the total deadline inside session
finalization returns FAILED, leaves no false manifest claim, and still attempts
down. Ordinary work-deadline exhaustion instead commits an explicit timeout
inventory under the manifest reserve. Additional coverage proves stdout,
partial append, close, terminal-control, notification, COMPLETED, and ABORTED
authority behavior.

Fresh round-1 verification:

```text
uv run pytest orchestration/tests/test_status_store.py \
  orchestration/tests/test_controller.py orchestration/tests/test_cli.py -q
84 passed in 5.33s

uv run pytest orchestration/tests -q
128 passed in 7.19s

uv run pytest artifacts/tests orchestration/tests tests/contracts -q
451 passed, 10 skipped in 24.28s

uv run pytest -q
454 passed, 10 skipped in 21.73s

uv run drone-sim --help
usage: drone-sim [-h] {start,status,abort,collect-results} ...

uv run python -m compileall -q artifacts/src artifacts/tests \
  orchestration/src orchestration/tests
exit 0

uv lock --check
Resolved 15 packages in 70ms

git diff --check
exit 0
```

## Task 7 concerns

- Task 7 must publish the exact `runtime-running`, `artifacts-ready`,
  `source-finished`, `runtime-failure`, `runtime-frozen`, `artifacts-final`,
  `terminal-notified` documents consumed here, with `runtime-running` only
  after the first valid clock and ROS RUNNING publication.
- The artifact runtime must emit the exact three-record report, including a
  nonempty semantic object even for missing/invalid evidence and size/hash
  values describing the stable finalized file/tree snapshot.
- Compose service names and module ownership must match the frozen seven-entry
  log mapping; the `phase2` profile must activate through
  `COMPOSE_PROFILES=phase2` and accept detached `--no-build` startup.
- Task 7 must stop artifact writers before `runtime-frozen`, publish no required
  artifact changes after manifest commit, and acknowledge the terminal control
  only through `terminal-notified`.
- This task uses fakes at the Task 7 boundary and does not claim Phase 2 is
  runnable; that claim remains gated by Tasks 7 and 8 real Compose integration.
