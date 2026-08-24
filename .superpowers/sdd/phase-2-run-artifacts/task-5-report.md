# Task 5 Report: Structured Compose Log Capture

## Scope and baseline

- Baseline: `637b18a649fc9d24cf5fe1ac65ad32d35b03f7a1`
- Verification audit: `2026-08-24T00:33:16Z`
- Scope: host-side, per-service Compose log capture; exact raw byte evidence;
  strict common-event classification; seven owned JSONL streams; and durable,
  no-clobber publication diagnostics.
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
    rejection only for objects that claim a top-level common field;
  - strict run/module/ownership, top-level schema, severity/event, finite
    timestamp, timezone-aware ISO-8601, fields-object, collision, nonfinite,
    and invalid-Unicode validation;
  - every accepted event is constructed and serialized through the existing
    `StructuredEvent` domain seam, preserving service and line order and UTC
    `Z` canonicalization;
  - retained no-follow run/log directory descriptors; exclusive fixed
    `.partial` candidates; complete writes; file `fsync`; mode `0444`; exact
    inode checks; no-clobber hard-link publication; one-link finals; directory
    `fsync`; and owned-candidate-only cleanup;
  - raw publication on command/event/coverage failures, structured
    all-validation-before-publication, and explicit partial-set diagnostics
    where POSIX cannot provide seven-file transactionality.
- `artifacts/tests/test_docker_logs.py`
  - 51 cases covering exact commands/order, raw byte preservation, canonical
    routing, all malformed attempted-event classes, explicit ownership and
    seven-module coverage, command failures/exceptions, identifier/path and
    target safety, durability ordering, cleanup, late publication failures,
    partial-set reporting, and immutable results.
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
- The adapter never accesses a Docker socket and adds no second structured-log
  schema.

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
67 passed

uv run pytest artifacts/tests -v
286 passed, 10 skipped

uv run pytest -v
340 passed, 10 skipped

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
