# Task 5 Report: Structured Compose Log Capture

## Scope and baseline

- Baseline: `637b18a649fc9d24cf5fe1ac65ad32d35b03f7a1`
- Verification audit: `2026-08-24T01:35:52Z`
- Scope: host-side, per-service Compose log capture; exact raw byte evidence;
  strict common-event classification; an explicit host-orchestration merge
  seam; seven owned JSONL streams; and durable, no-clobber publication and
  cleanup diagnostics.
- Out of scope remained untouched: Docker/Compose definitions, controller/CLI,
  ROS, video, rosbag, Gazebo, ArduPilot, mission/scoring code, machine
  configuration, and `companion/comp2026`.

## Contract ruling

The initial task brief named `ardupilot` and `logs/ardupilot.jsonl`, while the
frozen repository module, manifest inventory, and Phase 2 design use
`ardupilot_sitl` and `logs/ardupilot_sitl.jsonl`. The controller ruled the
brief spelling a drafting error. Task 5 therefore preserves the frozen
`ardupilot_sitl` interface; the corrected brief and SDD ledger record that
ruling. No manifest or interface-document edit was needed.

## RED evidence

The import-first public-interface test was written before production code and
run as:

```text
uv run pytest artifacts/tests/test_docker_logs.py -v
```

It failed during collection with `ImportError: cannot import name
'DockerLogCapture' from 'artifacts'` (exit 2, zero tests collected). The
complete behavioral surface was then persisted and reproduced the same missing
import before implementation.

Four later test-first self-review cycles exposed real defects before their
fixes:

- A valid event with fractional numbers nested under `fields` failed with
  `Object of type Decimal is not JSON serializable`. JSON fractions now remain
  ordinary finite floats accepted by the existing `StructuredEvent` seam.
- A failure after a structured final name had already been linked omitted that
  visible file from `structured_paths`. Results now explicitly inventory an
  exact linked candidate even when a later publication step fails.
- A failure after a raw final name had already been linked returned false
  success when all seven final names were visible. Raw-publication failure is
  now an independent fail-closed gate while the linked raw path remains
  reported.
- Rejecting a symlinked `logs/` directory leaked the already-opened run-root
  descriptor. Setup now closes that retained descriptor before returning the
  typed filesystem failure.

The corresponding focused RED runs each collected one test and failed for the
expected reason before the minimal production correction. The final surface is
51 Docker-log behavioral cases.

The review-fix regression surface was also persisted before its production
changes. Its first aggregate RED run reported `71 failed, 2 passed`: the
missing keyword-only host-event input stopped the legacy helper calls at the
new interface boundary. After narrowing the helper to pass that keyword only
when the seam was requested, the behavior-specific REDs exposed the parser
resource exceptions, permissive timestamp parsing, discarded runner stderr,
missing host merge, and suppressed cleanup failures. During GREEN, the strict
timestamp diagnostic classifier itself produced one further focused RED
(`72 passed, 1 failed`) for an attempted offset with seconds. The final
focused run reports 73 passing Docker-log cases.

Review fix round 2 persisted three additional cleanup-truth regressions. The
focused RED run selected three tests and reported `3 failed, 73 deselected`:
a raw candidate factory left an unreported `.partial` when cleanup unlink
failed; a structured candidate factory omitted the cleanup directory-`fsync`
failure after unlink; and a publication directory-`fsync` failure after a
successful partial unlink produced a false missing-file ownership diagnostic.
The matching focused GREEN run reported `3 passed, 73 deselected`; the final
Docker-log surface is 76 cases.

Review fix round 3 persisted two cancellation regressions. The focused RED run
reported `2 failed, 76 deselected`: both `KeyboardInterrupt` and
`SystemExit(130)` completed factory cleanup but were then converted into
`DockerLogCaptureError`. The minimal correction retains the `BaseException`
cleanup boundary, then re-raises non-`Exception` control flow unchanged and
wraps only ordinary failures. The focused GREEN run reported
`2 passed, 76 deselected`; the final Docker-log surface is 78 cases.

## Implementation and files

- `artifacts/src/artifacts/_adapters/docker_logs.py`
  - immutable `DockerLogCommandResult`, `DockerLogDiagnostic`, and
    `DockerLogCaptureResult`, plus typed `DockerLogCaptureError`;
  - injected one-argument command runner whose production implementation uses
    a shell-free argument list and merged stdout/stderr bytes;
  - exact one-to-one ordered ownership validation for the seven frozen module
    names and safe Compose project/service identifiers;
  - one exact `docker compose -p ... logs --no-color --no-log-prefix` command
    per service, completing every command before publication decisions;
  - byte-for-byte raw capture, including invalid UTF-8, blank lines, and final
    lines without newline;
  - duplicate-key-preserving JSON classification, including nested duplicate
    rejection only for objects that claim a top-level common field, with
    recursion and oversized-integer parser failures converted to typed
    malformed-attempt diagnostics;
  - strict run/module/ownership, top-level schema, severity/event, finite
    timestamp, the explicit
    `YYYY-MM-DDTHH:MM:SS[.1-6](Z|+/-HH:MM)` wall-time profile, fields-object,
    collision, nonfinite, and invalid-Unicode validation;
  - every accepted event is constructed and serialized through the existing
    `StructuredEvent` domain seam, preserving service and line order and UTC
    `Z` canonicalization;
  - optional, fixed-path `logs/orchestration-host.jsonl.partial` input opened
    relative to the retained `logs/` descriptor with no-follow, single-link,
    regular-file, and before/after snapshot checks; every host line is
    structured-only and passes through the same attempted-event and
    `StructuredEvent` seam with fixed `module=orchestration` and matching
    `run_id`;
  - deterministic orchestration merge ordered by parsed UTC wall timestamp,
    then Compose before host for equal instants, then original source line
    order; the unchanged host source is returned as a recovery path;
  - retained no-follow run/log directory descriptors; exclusive fixed
    `.partial` candidates; complete writes; file `fsync`; mode `0444`; exact
    inode checks; no-clobber hard-link publication; one-link finals; directory
    `fsync`; and owned-candidate-only cleanup;
  - raw publication on command/event/coverage failures, structured
    all-validation-before-publication, and explicit partial-set diagnostics
    where POSIX cannot provide seven-file transactionality;
  - cleanup unlink, cleanup directory-`fsync`, candidate close, and retained
    directory close failures amend the final immutable result, downgrade
    success, and report exact still-named `.partial` paths; candidate factory
    failures carry these facts across the pre-tracking ownership boundary as
    well, while an already-absent owned partial is treated as clean;
  - `KeyboardInterrupt` and `SystemExit` during candidate creation still run
    owned cleanup and durability steps but propagate unchanged for Task 6
    cancellation/exit handling rather than becoming capture failures.
- `artifacts/tests/test_docker_logs.py`
  - 78 cases covering exact commands/order, raw byte preservation, canonical
    routing, all malformed attempted-event classes, explicit ownership and
    seven-module coverage, deep/oversized JSON failures, runner exception byte
    normalization, strict timestamps, descriptor-safe host merge and recovery,
    command failures/exceptions, identifier/path and target safety, durability
    ordering, cleanup failure truth, late publication failures, partial-set
    reporting, and immutable results.
- `artifacts/src/artifacts/__init__.py`
  - exports `DockerLogCapture` and its immutable command/result/diagnostic and
    typed error surfaces.

## Self-review

- Mutation check: command order/arguments, raw bytes, ownership, every schema
  field, module coverage, no-clobber publication, file/directory `fsync`, and
  success/failure state each have a test that fails under the corresponding
  wrong constant, branch, omission, or side effect.
- Raw evidence remains named for every service after command failure or any
  malformed claimed event; no structured file is published in those cases.
- Non-JSON and unclaimed JSON remain raw only. Duplicate-key and nonfinite
  parsing cannot degrade into last-value-wins or silent dropping.
- Preflight rejects unsafe identifiers and every preexisting final/partial
  target before Docker invocation. Descriptor-relative operations do not
  derive a path from untrusted service text after validation.
- Publication errors never report success. If a final name became visible
  before a later failure, the immutable result inventories that partial
  publication so manifest validation can fail closed.
- Requested host input is never listed as Docker raw output. Missing, symlink,
  hard-linked, changed, raw, unclaimed, wrong-run, or wrong-module host input
  becomes a typed failure only after all seven Docker byte streams are
  published. Its source path remains unchanged for recovery.
- Parser recursion and integer-limit failures cannot escape generically or
  delete raw candidates. Cleanup errors cannot be suppressed after a success
  result has been constructed; the result is replaced immutably before the
  typed error is raised.
- The adapter never accesses a Docker socket and adds no second structured-log
  schema.

## Task 6 consumability

Task 6 can request the host seam with `host_events=True` after appending its
structured host lifecycle events to
`logs/orchestration-host.jsonl.partial`. It does not need to modify this
adapter or claim that file as a Compose service stream. The source must remain
a single-link regular file at the fixed contained path until capture finishes;
on success its events are validated and deterministically merged into
`logs/orchestration.jsonl`, while the source remains named as recovery
evidence. Omitting the option preserves Task 5's Compose-only behavior and
does not require the host source to exist.

## Artifact-image decision

No artifact image rebuild was performed. Task 5 runs exclusively on the host,
adds no runtime dependency, and changes no Dockerfile or container behavior.
The existing Dockerfile inventory was inspected: broad `COPY artifacts/src`
and `COPY artifacts/tests` statements automatically include the new module and
tests in any later image build, so there is no enumerated image inventory to
update or image-specific condition to prove in this task. Host package and
repository verification exercise the complete implementation.

## GREEN and final verification

```text
uv run pytest artifacts/tests/test_docker_logs.py artifacts/tests/test_structured_log.py -v
94 passed

uv run pytest artifacts/tests -v
313 passed, 10 skipped

uv run pytest -v
367 passed, 10 skipped

uv run python -m compileall -q artifacts/src artifacts/tests
exit 0

git diff --check
exit 0
```

The ten skips are the pre-existing ROS/container/real-FFmpeg cases whose
environment gates are not available on the host; no Task 5 case is skipped.

## Concerns

- POSIX cannot atomically publish the seven structured files as one set. The
  adapter prepares and validates the complete set first, publishes no-clobber,
  and returns the exact already-linked subset on any late failure; downstream
  manifest validation therefore fails closed rather than claiming a
  transaction.
- The implementation relies on the already-frozen exclusive run-root and
  trusted same-UID/path-stability model in `artifacts/INTERNAL_INTERFACE.md`.
  It does not claim hostile-co-tenant Linux immutability.
