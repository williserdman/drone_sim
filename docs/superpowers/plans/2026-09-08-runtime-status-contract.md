# Typed runtime-status contract implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace every repository-owned durable runtime-status dictionary and local writer with one typed, fail-closed contract while preserving independent host artifact verification and mission policy.

**Architecture:** Add the pure `artifacts.runtime_status` domain module as the sole owner of the 14 status names, schemas, Python values, and write policies. Keep `RuntimeProtocol` and `StatusStore` as separate runtime and host filesystem adapters over `artifacts.protocol_files`. Migrate every producer, consumer, and test fake to exact registered status types; then delete duplicate validators and bypass writers.

**Tech stack:** Python 3.12, frozen dataclasses, `pytest`, `uv`, Docker Compose, existing descriptor-relative protocol-file primitives.

**Spec:** `docs/superpowers/specs/2026-09-08-runtime-status-contract-design.md`

**Scope:** This is the next independently reviewable slice of the broader codebase-simplification goal. Completing it does not complete the thread goal.

## Global constraints

- Use red-green-refactor for every behavior change: add or convert the focused test, run it and observe the expected failure, implement the minimum production change, and rerun it to green.
- Keep every committed checkpoint green. The RuntimeProtocol signature and all of its string callers move atomically in Task 3; do not add a compatibility overload.
- Preserve simulation behavior, timing, readiness ordering, terminal ordering, score semantics, artifact formats, operator commands, and the companion mission repository.
- Only exact registered status classes may select filenames or write policies. Do not use `issubclass`, `isinstance`, arbitrary status names, or package-root re-exports.
- Keep filesystem safety in `artifacts.protocol_files`; do not duplicate race, link, FIFO, size, duplicate-key, non-finite JSON, locking, mode, fsync, or cleanup logic.
- Keep host artifact reopening, regular-file/tree checks, size checks, checksum checks, media/bag validation, manifest validation, scoring, acceptance, producer state machines, and timestamp-ordering policy independent of the typed status parser.
- Let the exact `TimeoutError` object from a host deadline callback escape unchanged.
- Runtime status files remain mode `0o644`. `RuntimeFailureStatus` alone is `FIRST_WINS`; every other status is `IDENTICAL`.
- Preserve ArduPilot's private `work/failure.json`; only shared readiness, failure, and quiescence publication moves to `RuntimeProtocol`.
- Do not alter historical run evidence, generated or vendored code, licenses/provenance, or unrelated changes. Source edits do not prove an image or flight result.
- Update affected module READMEs, architecture, and handoff in Task 7. Do not edit the runbook because operator procedures do not change.
- Use `apply_patch` for hand edits. Do not merge, push, or copy the absent `companion/comp2026` checkout into this worktree.

---

### Task 1: Add the pure typed status contract

**Files:**
- Create: `artifacts/src/artifacts/runtime_status.py`
- Create: `artifacts/tests/test_runtime_status.py`

**Interfaces:**

```python
class RuntimeStatusError(ValueError): ...

@dataclass(frozen=True)
class RuntimeStatus:
    run_id: str
    name: ClassVar[str]

@dataclass(frozen=True)
class FlightExchange:
    online: bool
    servo_packets_received: int
    motor_updates: int
    duplicate_servo_packets: int
    servo_frame_gaps: int
    json_states_sent: int
    json_send_errors: int
    last_servo_frame: int
    last_json_sim_time_ns: int

@dataclass(frozen=True)
class ArtifactFinalRecord:
    relative_path: str
    status: ValidationStatus
    detail: str
    size_bytes: int | None
    sha256: str | None
    semantic: Mapping[str, Any]

StatusT = TypeVar("StatusT", bound=RuntimeStatus)

def canonical_run_id(value: object) -> str: ...
def status_name(status_type: type[StatusT]) -> str: ...
def parse_status(status_type: type[StatusT], document: object, *, expected_run_id: str) -> StatusT: ...
def status_document(status: RuntimeStatus) -> dict[str, Any]: ...
def status_write_policy(status_type: type[RuntimeStatus]) -> WritePolicy: ...
```

The concrete frozen types and constructor fields are:

- `ArtifactsReadyStatus(run_id)` -> `artifacts-ready`.
- `GazeboReadyStatus(run_id, flight_exchange)` -> `gazebo-ready`.
- `ArduPilotReadyStatus(run_id)` -> `ardupilot-ready`.
- `CompanionReadyStatus(run_id)` -> `companion-ready`.
- `MissionReadyStatus(run_id)` -> `mission-ready`.
- `MissionCommandDeliveredStatus(run_id, sim_timestamp_ns)` -> `mission-command-delivered`.
- `RuntimeRunningStatus(run_id, sim_timestamp_ns)` -> `runtime-running`.
- `SourceFinishedStatus(run_id, sim_timestamp_ns)` -> `source-finished`.
- `MissionFinishedStatus(run_id, sim_timestamp_ns)` -> `mission-finished`.
- `ScoreFinishedStatus(run_id, sim_timestamp_ns)` -> `score-finished`.
- `RuntimeFailureStatus(run_id, module, reason, diagnostic_paths)` -> `runtime-failure` and `WritePolicy.FIRST_WINS`.
- `RuntimeFrozenStatus(run_id)` -> `runtime-frozen`.
- `ArtifactsFinalStatus(run_id, records)` -> `artifacts-final`, with computed `complete`.
- `TerminalNotifiedStatus(run_id)` -> `terminal-notified`.

- [ ] **Step 1: Add the exhaustive registered-status table and observe the missing-module failure**

Use `RUN_ID = "11111111-1111-4111-8111-111111111111"`, a valid `FlightExchange`, and three valid artifact records in fixed order `video/onboard.mp4`, `video/observer.mp4`, `rosbag`. Parameterize all 14 values and their exact documents from the spec. Assert exact name, exact serialized dictionary, exact parsed type/equality, unique names, `IDENTICAL` for 13 types, and `FIRST_WINS` for failure.

Run:

```bash
uv run pytest artifacts/tests/test_runtime_status.py -q
```

Expected RED: collection fails with `ModuleNotFoundError: No module named 'artifacts.runtime_status'`.

- [ ] **Step 2: Implement registered happy paths using one exact-type registry**

Use one private immutable mapping from concrete class to `(name, write_policy)`. Populate it with a decorator, freeze it with `MappingProxyType`, and remove the mutable builder. Every public lookup dispatches with exact membership (`type(status) in registry` or `status_type in registry`). Export the public names through `__all__`.

Serialize the exact wire shapes in the spec, including all constant fields. Parse only exact dictionaries and exact key sets. `canonical_run_id` accepts only a string where `str(UUID(value)) == value`. Constructors and parsing share validation.

Rerun the Task 1 test; expected GREEN for the registered table.

- [ ] **Step 3: Test and implement exact schemas, run identity, constants, and nested values**

Add parameterized failures for non-dictionary input (including a custom `Mapping` that is not an exact `dict`), a missing key, an extra key, invalid/noncanonical/cross-run IDs, every false or wrong constant, booleans and negative integers, and a mission-command timestamp above `50_000_000`.

For `FlightExchange`, require `online is True`; progress counters `servo_packets_received`, `motor_updates`, and `json_states_sent` >= 1; nonnegative exact integer duplicate/frame/time counters; and exact integer zero for `servo_frame_gaps` and `json_send_errors`. Reject missing/extra nested fields.

For failure, require nonempty string module/reason and a tuple of unique relative POSIX diagnostic paths. Parsing accepts a JSON list and converts it to a tuple. Reject empty paths, absolute paths, backslashes, `.`, `..`, empty components, traversal, and non-tuples at construction. An empty tuple is valid.

Run focused schema tests before and after implementation:

```bash
uv run pytest artifacts/tests/test_runtime_status.py -q -k 'schema or run_id or constant or timestamp or flight or failure or diagnostic'
```

- [ ] **Step 4: Test and implement strict artifact records and defensive semantic copies**

Cover all accepted combinations:

```text
valid   + size/hash present
missing + both absent
missing + both present
invalid + both absent
invalid + both present
```

Reject a valid record without both size/hash, all half-present pairs, wrong/raw statuses, empty detail, empty semantic mapping, invalid sizes/hashes, wrong record count/order/path, missing/extra record keys, and a missing/non-boolean/disagreeing aggregate. Require lowercase 64-character SHA-256 values. Require exact `ArtifactFinalRecord` instances in a tuple.

The semantic copier accepts only string-keyed mappings containing `None`, exact booleans/integers, finite floats, strings, mappings, lists, and tuples. It returns fresh dictionaries/lists and rejects cycles, non-finite floats, sets, enums, objects, and non-string keys. Assert both recursive containers and recursion-depth `RecursionError` are translated to `RuntimeStatusError`. Prove that mutating a returned document cannot mutate the status, while mutation of a retained constructor mapping makes the next `status_document` fail closed.

Run:

```bash
uv run pytest artifacts/tests/test_runtime_status.py -q -k 'artifact or semantic or fresh or mutation'
```

- [ ] **Step 5: Reject unregistered subclasses and finish focused verification**

Create an unregistered frozen subclass with `name = "../escape"`. Assert `status_name`, `status_write_policy`, `parse_status`, and `status_document` all raise `RuntimeStatusError`.

Run:

```bash
uv run pytest artifacts/tests/test_runtime_status.py -q
uv run pytest artifacts/tests/test_runtime_status.py artifacts/tests/test_runtime_protocol.py artifacts/tests/test_protocol_files.py artifacts/tests/test_validation.py -q
uv run python -m compileall -q artifacts/src/artifacts/runtime_status.py
uv run python -c 'from artifacts.runtime_status import RuntimeFrozenStatus, status_document; print(status_document(RuntimeFrozenStatus("11111111-1111-4111-8111-111111111111")))'
```

Expected smoke output: `{'run_id': '11111111-1111-4111-8111-111111111111', 'frozen': True}`.

- [ ] **Step 6: Commit the green contract**

```bash
git add artifacts/src/artifacts/runtime_status.py artifacts/tests/test_runtime_status.py
git commit -m "add typed runtime status contract"
```

Documentation impact for this isolated checkpoint: none; Task 7 updates the guides for the complete migration.

---

### Task 2: Make host reads typed and remove duplicate controller parsing

**Files:**
- Modify: `orchestration/src/orchestration/status_store.py`
- Modify: `orchestration/src/orchestration/controller.py`
- Modify: `orchestration/tests/test_status_store.py`
- Modify: `orchestration/tests/test_controller.py`

**Interfaces:**

```python
def StatusStore.read_runtime_status(
    self,
    run_id: str,
    status_type: type[StatusT],
    deadline_check: Callable[[], None] | None = None,
) -> StatusT | None: ...

def RunController._wait_for(
    self,
    run_id: str,
    status_type: type[StatusT],
    ...,
) -> tuple[StatusT | None, TerminalCause | None]: ...
```

- [ ] **Step 1: Add all host-adapter and controller regressions before production edits**

Parameterize all 14 status values: persist `status_document(value)`, read using `type(value)`, and assert exact typed equality. Add malformed documents for `SourceFinishedStatus` without `finished: true`, `RuntimeFrozenStatus` without `frozen: true`, and artifacts-final with empty detail, valid-without-size/hash, half-present size/hash, or reordered records. Preserve a deadline test that asserts the exact raised `TimeoutError` object is unchanged.

In controller tests, add an integration probe where source-finished has a valid timestamp but no true marker. Assert the result and manifest are `FAILED`, the recorded reason identifies a source-finished protocol violation, and the run cannot silently become a successful absent-timing result; do not salvage the untrusted end timestamp from the malformed fact. Add a malformed runtime-frozen probe whose log-capture and artifact-validator factories record entry and assert neither is entered. Express the four artifacts-final disagreement fixtures through shared `status_document` fixtures when valid and explicit malformed wire dictionaries otherwise. Keep independent checksum and size mismatch tests.

Run:

```bash
uv run pytest orchestration/tests/test_status_store.py orchestration/tests/test_controller.py -q
```

Expected RED: the current string inventory cannot accept a class, omits mission-command-delivered, and the old production path accepts same-run malformed source-finished/runtime-frozen documents. Signature failures during this coordinated test conversion are expected until Steps 2–3 are complete.

- [ ] **Step 2: Implement the typed StatusStore adapter**

Delete `_RUNTIME_STATUS_NAMES` and run-ID-only status validation. Derive the filename with `status_name`, read with `read_json_object_at`, and parse with `parse_status(..., expected_run_id=run_id)`. Translate `RuntimeStatusError` and `ProtocolIOError` to `ProtocolFileError`; do not catch `TimeoutError`.

- [ ] **Step 3: Migrate controller waits and delete structural validators**

Use concrete status classes for artifacts/Gazebo/ArduPilot/companion/mission readiness, running, source/mission/score finished, runtime frozen, artifacts final, terminal notified, and runtime failure. Consume `.sim_timestamp_ns`, `.reason`, `.module`, `.flight_exchange`, and `.records` directly. Keep deadline checks immediately before and after reads plus mission-at-most-source and score-equals-source policy.

Delete `_FLIGHT_EXCHANGE_KEYS`, all readiness validators, `_validate_running`, `_source_stamp`, `_mission_stamp`, `_score_stamp`, `_ArtifactReportRecord`, and `ArtifactFinalReport.parse`. Keep `ArtifactFinalReport.first_failure`, `.validators`, and host `_validator`, typed with `ArtifactFinalRecord`, and preserve fixed record order without reparsing or sorting.

Delete the opening controller unit tests that directly call those removed private structural validators after Task 1 provides equivalent schema coverage. Retain controller integration tests for sequencing, terminal outcomes, and independent host artifact checks.

Update controller fakes to accept a status class, compare exact type identity, and use `status_name(status_type)` only for trace strings. Keep external-process `FakeCompose` wire JSON, but serialize contract fixtures with `status_document`.

- [ ] **Step 4: Verify and commit host migration**

```bash
uv run pytest artifacts/tests/test_runtime_status.py orchestration/tests/test_status_store.py orchestration/tests/test_controller.py -q
git add orchestration/src/orchestration/status_store.py orchestration/src/orchestration/controller.py orchestration/tests/test_status_store.py orchestration/tests/test_controller.py
git commit -m "refactor: consume typed runtime statuses on the host"
```

Documentation impact remains deferred to Task 7 so the guides describe the completed cross-process contract once.

---

### Task 3: Atomically cut over RuntimeProtocol and every existing string caller

**Files:**
- Modify: `artifacts/src/artifacts/runtime_protocol.py`
- Modify: `artifacts/src/artifacts/runtime_node.py`
- Modify: `artifacts/tests/test_runtime_protocol.py`
- Modify: `artifacts/tests/test_runtime_node.py`
- Modify: `companion/src/drone_sim_companion/lifecycle.py`
- Modify: `companion/src/drone_sim_companion/runtime_node.py`
- Modify: `companion/tests/test_lifecycle.py`
- Modify: `companion/tests/test_runtime_node.py`
- Modify: `gazebo/src/drone_sim_gazebo/runtime/entrypoint.py`
- Modify: `gazebo/src/drone_sim_gazebo/runtime/runtime_node.py`
- Modify: `gazebo/tests/test_action_executor.py`
- Modify: `gazebo/tests/test_runtime_node.py`
- Modify: `orchestration/src/orchestration/runtime_node.py`
- Modify: `orchestration/tests/test_runtime_node.py`
- Modify: `scorekeeper/src/drone_sim_scorekeeper/runtime.py`
- Modify: `scorekeeper/src/drone_sim_scorekeeper/competition_runtime.py`
- Modify: `scorekeeper/src/drone_sim_scorekeeper/runtime_node.py`
- Modify: `scorekeeper/tests/test_runtime.py`
- Modify: `scorekeeper/tests/test_competition_runtime.py`
- Modify: `scorekeeper/tests/test_runtime_node.py`
- Modify: `tests/phase2/synthetic_gazebo.py`
- Modify: `tests/phase2/test_synthetic_services.py` only where typed fixture assertions require it

**Interfaces:**

```python
def RuntimeProtocol.write_status(self, status: RuntimeStatus) -> Path: ...
def RuntimeProtocol.read_status(self, status_type: type[StatusT]) -> StatusT | None: ...
```

- [ ] **Step 1: Convert adapter tests first and observe signature/policy failures**

Parameterize all 14 values through `write_status(status)` and `read_status(type(status))`. Assert exact typed equality, exact canonical wire bytes, and mode `0o644`. Preserve equal retry and differing retry tests for `IDENTICAL`. Add concurrent distinct `RuntimeFailureStatus` writers: all calls return normally, one valid winner remains, and typed read returns it. Seed malformed runtime-failure bytes and assert a first-wins write raises `ProtocolError` without changing them. Seed malformed status bytes from Task 2 and assert runtime and host adapters translate to their own public errors.

Run the focused adapter tests. Expected RED: old methods require name/document strings and all statuses currently use `IDENTICAL`.

- [ ] **Step 2: Implement only the typed RuntimeProtocol interface**

Derive filenames, documents, and policy through `status_name`, `status_document`, and `status_write_policy`. Parse every present document with `parse_status`. For first-wins writes, validate the persisted winner before returning the path. Translate `RuntimeStatusError` and `ProtocolIOError` to `ProtocolError`.

Move `canonical_run_id` to the contract but re-export it by direct import from `runtime_protocol.py` for existing non-status callers. Delete `_STATUS_NAMES`, `_INITIAL_COMMAND_WINDOW_NS`, `_FLIGHT_EXCHANGE_KEYS`, `_safe_relative_path`, `_nonnegative_integer`, `_valid_flight_exchange`, `_validate_status`, and `_valid_artifacts_final`. Do not add a string compatibility branch.

- [ ] **Step 3: Convert artifacts and orchestration runtime callers and fakes**

First convert artifacts and orchestration runtime tests/fakes to their typed signatures and exact value assertions, then run their focused tests. Expected RED: the production methods still call the removed string signature. Implement the migrations only after observing that failure.

```bash
uv run pytest artifacts/tests/test_runtime_node.py orchestration/tests/test_runtime_node.py -q
```

In artifacts, publish `ArtifactsReadyStatus`, `RuntimeFailureStatus`, `ArtifactsFinalStatus`, and `TerminalNotifiedStatus`; read `RuntimeFrozenStatus`; make `_record` return `ArtifactFinalRecord`. Preserve `finalize()`'s existing outward dictionary contract by returning `status_document(typed_value)`; existing callers index `complete` and record dictionaries.

In orchestration runtime, read `GazeboReadyStatus`; use exact readiness class tuples for ArduPilot/companion/mission; publish `RuntimeRunningStatus`, `RuntimeFrozenStatus`, and `RuntimeFailureStatus`. Preserve state-machine timing and failure diagnostics.

Update all fakes to accept/store typed values and key readable statuses by exact class.

- [ ] **Step 4: Convert companion, Gazebo, scorekeeper, and Phase 2 callers and fakes**

First convert companion, Gazebo, scorekeeper, and Phase 2 fakes/assertions, then run the focused affected test files. Expected RED: production callers still use name/document arguments or return dictionaries. Implement the migrations only after observing those signature/value failures.

```bash
uv run pytest companion/tests/test_lifecycle.py companion/tests/test_runtime_node.py gazebo/tests/test_action_executor.py gazebo/tests/test_runtime_node.py scorekeeper/tests/test_runtime.py scorekeeper/tests/test_competition_runtime.py scorekeeper/tests/test_runtime_node.py tests/phase2/test_synthetic_services.py -q
```

Companion publishes ready, mission-ready, command-delivered, mission-finished, and failure typed values. Its production wrapper accepts a single `RuntimeStatus` and permits only exact companion-owned types.

Gazebo `ActionExecutor` publishes typed source-finished/failure. Rendezvous reads `MissionCommandDeliveredStatus`; main reads `ArtifactsReadyStatus`. Leave the custom Gazebo-ready writer for Task 4.

Scorekeeper failure paths publish `RuntimeFailureStatus` once and delete the fallback dynamic read; the shared adapter now validates and accepts the persisted first-wins value. Driver reads `SourceFinishedStatus` and removes its local structure checks. Leave its custom score-finished writer for Task 5.

The Phase 2 synthetic Gazebo publishes typed source-finished/failure and reads `RuntimeRunningStatus`. Modules using only control, manifest, or quiescence APIs need no status change.

- [ ] **Step 5: Verify no string API call remains and commit the atomic cutover**

```bash
uv run pytest artifacts/tests companion/tests gazebo/tests orchestration/tests scorekeeper/tests tests/phase2/test_synthetic_services.py -q
rg -n "write_status\([[:space:]]*['\"]|read_status\([[:space:]]*['\"]" artifacts/src orchestration/src companion/src gazebo/src electromagnet/src scorekeeper/src ardupilot_sitl/src tests/phase2
```

Expected: tests pass and the search returns no result. Then:

```bash
git add artifacts companion gazebo orchestration scorekeeper tests/phase2
git commit -m "refactor: use typed runtime protocol across processes"
```

---

### Task 4: Replace Gazebo's custom ready writer

**Files:**
- Modify: `gazebo/src/drone_sim_gazebo/runtime/entrypoint.py`
- Modify: `gazebo/src/drone_sim_gazebo/runtime/runtime_node.py`
- Modify: `gazebo/tests/test_runtime_entrypoint.py`
- Modify: `gazebo/tests/test_action_executor.py`

- [ ] **Step 1: Replace writer-specific tests with a shared-adapter publication test**

Feed a ready live transport exchange into the readiness action path and assert its fake protocol receives exactly `GazeboReadyStatus(RUN_ID, FlightExchange(...))`.

Run:

```bash
uv run pytest gazebo/tests/test_runtime_entrypoint.py gazebo/tests/test_action_executor.py -q
```

Expected RED: readiness calls the local `write_gazebo_ready` implementation and never publishes through the protocol fake.

- [ ] **Step 2: Publish readiness through RuntimeProtocol and remove the bypass**

Pass the existing `RuntimeProtocol` into the readiness action path. After the live exchange probe succeeds, freeze its exact nine fields into `FlightExchange` and write `GazeboReadyStatus`. Delete the local `GazeboReadyStatus` writer class plus filesystem imports used only by it. Keep `_validate_flight_status` and `_flight_status_ready`; they validate live transport input, not a durable status document.

- [ ] **Step 3: Verify and commit**

```bash
uv run pytest gazebo/tests -q
rg -n 'write_gazebo_ready|class GazeboReadyStatus' gazebo
git add gazebo/src gazebo/tests
git commit -m "refactor: publish gazebo readiness through runtime protocol"
```

Expected: tests pass and search returns no result.

---

### Task 5: Replace scorekeeper's custom completion writer

**Files:**
- Modify: `scorekeeper/src/drone_sim_scorekeeper/runtime.py`
- Modify: `scorekeeper/src/drone_sim_scorekeeper/competition_runtime.py`
- Modify: `scorekeeper/src/drone_sim_scorekeeper/self_test.py` only if typed reads require it
- Modify: `scorekeeper/tests/test_runtime.py`
- Modify: `scorekeeper/tests/test_competition_runtime.py`
- Modify: `scorekeeper/tests/test_runtime_node.py`
- Delete: `scorekeeper/src/drone_sim_scorekeeper/status.py`
- Delete: `scorekeeper/tests/test_status.py`
- Modify: `scorekeeper/pyproject.toml`
- Modify: `uv.lock`

- [ ] **Step 1: Make both completion tests demand typed protocol publication**

In both runtime suites, remove the injected document-only status writer. Record operation order and assert a successful score persists outputs, publishes and flushes events, then writes `ScoreFinishedStatus(run_id, sim_timestamp_ns)` through the protocol as the last operation. `ScoreResult` has no named timestamp property, so take the already validated integer from `result.finished_status()["sim_timestamp_ns"]` and pass only that integer to the typed value. Failure cases must publish no score-finished value.

Update the two `test_runtime_node.py` constructions that still pass `write_finished=` to rely on and assert against the typed protocol fake instead.

Run:

```bash
uv run pytest scorekeeper/tests/test_runtime.py scorekeeper/tests/test_competition_runtime.py scorekeeper/tests/test_runtime_node.py -q
```

Expected RED: the production default bypasses the protocol fake and calls `write_score_finished` directly.

- [ ] **Step 2: Move score completion to RuntimeProtocol and delete the local writer**

After output persistence, event publication, and flush, call `self.protocol.write_status(ScoreFinishedStatus(...))`. Remove the obsolete callback seam if unused. Delete `status.py` and its implementation-only tests. Add `drone-sim-artifacts==0.1.0` to `scorekeeper/pyproject.toml`; do not add a new local `tool.uv.sources` table because the root workspace already maps it.

Refresh only local workspace metadata:

```bash
uv lock --offline
uv lock --check
git diff -- uv.lock
```

Only the existing Scorekeeper package stanza may change. Stop if the lock refresh changes any external version, URL, hash, marker, or unrelated package.

- [ ] **Step 3: Verify and commit**

```bash
uv run pytest scorekeeper/tests -q
uv run python -m drone_sim_scorekeeper.self_test
rg -n 'write_score_finished|drone_sim_scorekeeper\.status' scorekeeper
git add scorekeeper uv.lock
git commit -m "refactor: publish score completion through runtime protocol"
```

Expected: tests/self-test pass and search returns no result.

---

### Task 6: Move ArduPilot shared facts to RuntimeProtocol

**Files:**
- Modify: `ardupilot_sitl/src/drone_sim_ardupilot/runtime_node.py`
- Modify: `ardupilot_sitl/src/drone_sim_ardupilot/runtime.py` only if shared-write imports become dead
- Modify: `ardupilot_sitl/tests/test_runtime.py`
- Modify: `ardupilot_sitl/tests/test_runtime_node.py`
- Modify: `ardupilot_sitl/Dockerfile`
- Modify: `ardupilot_sitl/pyproject.toml`
- Modify: `uv.lock` only if `uv lock --offline` changes workspace metadata
- Modify: `tests/integration/test_phase3_runtime_contract.py`

- [ ] **Step 1: Add runtime-node publication and image-wiring tests**

Replace the direct readiness writer test with a `main` test that monkeypatches `RuntimeProtocol` and records `ArduPilotReadyStatus` plus `write_quiescence("ardupilot_sitl")`. Add an unexpected-process-exit test asserting `RuntimeFailureStatus` is written through the same adapter while private `work/failure.json` still exists. Keep no-false-failure behavior.

In the Phase 3 static integration test, require the Dockerfile to copy `artifacts/src`, require both source roots in `PYTHONPATH`, and require `drone-sim-artifacts==0.1.0` in both ArduPilot and scorekeeper project metadata.

Run:

```bash
uv run pytest ardupilot_sitl/tests/test_runtime.py ardupilot_sitl/tests/test_runtime_node.py tests/integration/test_phase3_runtime_contract.py -q
```

Expected RED: `main` writes shared JSON directly and the image cannot import the new shared package.

- [ ] **Step 2: Use RuntimeProtocol for the three shared publications**

Construct and close one `RuntimeProtocol` in `main`. Publish `ArduPilotReadyStatus` and `RuntimeFailureStatus` through it and use `write_quiescence("ardupilot_sitl")`. Keep `atomic_document(work / "failure.json", failure)` because that file is private diagnostic evidence.

Add `COPY artifacts/src /opt/drone_sim/artifacts/src` and set `PYTHONPATH=/opt/drone_sim/ardupilot_sitl/src:/opt/drone_sim/artifacts/src` in the Dockerfile. Add `drone-sim-artifacts==0.1.0` to `ardupilot_sitl/pyproject.toml`. Do not expect an ArduPilot stanza in root `uv.lock`; it is not a root workspace member.

- [ ] **Step 3: Verify source behavior and attempt the affected image build**

```bash
uv run pytest ardupilot_sitl/tests tests/integration/test_phase3_runtime_contract.py -q
docker build -f ardupilot_sitl/Dockerfile -t drone-sim-ardupilot-status-contract .
rg -n 'atomic_document\(.*\.status|ardupilot-ready\.json|runtime-failure\.json|quiescence/ardupilot_sitl' ardupilot_sitl/src
```

Expected: tests and build pass; the search returns no shared writer. If the build is blocked only by unavailable environment input, capture the exact failure for Task 7 without weakening the image or test.

- [ ] **Step 4: Commit**

```bash
git add ardupilot_sitl tests/integration/test_phase3_runtime_contract.py uv.lock
git commit -m "refactor: publish ardupilot facts through runtime protocol"
```

---

### Task 7: Update current documentation and run the repository gate

**Files:**
- Modify: `artifacts/README.md`
- Modify: `orchestration/README.md`
- Modify: `companion/README.md`
- Modify: `gazebo/README.md`
- Modify: `electromagnet/README.md`
- Modify: `scorekeeper/README.md`
- Modify: `ardupilot_sitl/README.md`
- Modify: `docs/architecture.md`
- Modify: `docs/handoff.md`

- [ ] **Step 1: Audit the implementation against the approved contract before writing docs**

Confirm all 14 types round-trip through both adapters; the false source-finished regression records a protocol failure instead of silently completing without its otherwise valid start evidence; the malformed runtime-frozen and four artifacts-final regressions exercise production paths; and all required deletions are complete.

Run:

```bash
rg -n '_STATUS_NAMES|_INITIAL_COMMAND_WINDOW_NS|_FLIGHT_EXCHANGE_KEYS|_safe_relative_path|_nonnegative_integer|_valid_flight_exchange|_validate_status|_valid_artifacts_final' artifacts/src/artifacts/runtime_protocol.py
rg -n '_RUNTIME_STATUS_NAMES' orchestration/src/orchestration/status_store.py
rg -n '_FLIGHT_EXCHANGE_KEYS|_validate_ready|_validate_gazebo_ready|_validate_ardupilot_ready|_validate_companion_ready|_validate_mission_ready|_validate_running|_source_stamp|_mission_stamp|_score_stamp|_ArtifactReportRecord|^[[:space:]]+def parse\(' orchestration/src/orchestration/controller.py
rg -n 'write_score_finished|class GazeboReadyStatus' gazebo/src scorekeeper/src
rg -n 'atomic_document' ardupilot_sitl/src/drone_sim_ardupilot/runtime_node.py
rg -n "write_status\([[:space:]]*['\"]|read_status\([[:space:]]*['\"]" artifacts/src orchestration/src companion/src gazebo/src electromagnet/src scorekeeper/src ardupilot_sitl/src tests/phase2
rg -n "read_runtime_status\([[:space:][:alnum:]_,.]*['\"]" orchestration --glob '*.py'
```

The duplicate-validator and legacy-writer searches must be empty. The ArduPilot search may show exactly one private `work/failure.json` diagnostic write and no shared status/quiescence write. The two string-call searches must be empty. Also inspect multiline calls with `rg -n 'write_status|read_status|read_runtime_status'` before declaring the audit clean. Review any same-named helper from an unrelated domain rather than deleting it by name alone.

- [ ] **Step 2: Update the single current documentation path**

Document `runtime_status.py` as the schema/write-policy owner and `runtime_protocol.py`/`StatusStore` as adapters. Explain first-wins failure and exact-type selection in architecture. In module guides, link to the shared owner without copying schema tables and remove local-writer descriptions. Keep independent host artifact proof and acceptance boundaries explicit.

In `docs/handoff.md`, update its date and record exact source-test, Compose, and image-build evidence from this checkout. Distinguish passed, failed, skipped, and unrun checks. State that source verification is not a physical flight result. Do not edit the runbook or historical run artifacts.

- [ ] **Step 3: Run all affected source suites separately**

```bash
uv run --locked pytest artifacts/tests -q
uv run --locked pytest orchestration/tests -q
uv run --locked pytest companion/tests -q
uv run --locked pytest gazebo/tests -q
uv run --locked pytest electromagnet/tests -q
uv run --locked pytest scorekeeper/tests -q
uv run --locked pytest ardupilot_sitl/tests -q
uv run --locked pytest tests/phase2/test_synthetic_services.py tests/contracts tests/integration/test_phase2_runtime_contract.py tests/integration/test_phase3_runtime_contract.py -q
uv run --locked python -m compileall -q artifacts/src orchestration/src companion/src gazebo/src electromagnet/src scorekeeper/src ardupilot_sitl/src
uv lock --check
git diff --check
```

Record results individually; do not collapse an environment-only failure into a source failure or claim skipped work passed.

- [ ] **Step 4: Resolve Compose configurations and rebuild affected images**

Resolve the base, Phase 2, and Phase 3 GPU configurations:

```bash
docker compose config --quiet
docker compose --profile phase2 config --quiet
docker compose -f compose.yaml -f compose.gpu.yaml --profile phase3 config --quiet
```

Then run:

```bash
docker compose --profile phase2 build
SIM_COMP2026_REVISION=$(git -C companion/comp2026 rev-parse HEAD) docker compose --profile phase3 build
```

The isolated worktree is known not to contain `companion/comp2026`. Do not copy it silently or weaken provenance checks. If that blocks the aggregate Phase 3 build, build the available services explicitly:

```bash
docker compose --profile phase3 build orchestration-runtime artifacts-runtime ardupilot-sitl gazebo-runtime electromagnet-runtime scorekeeper-runtime
```

Record pre-build IDs (or `MISSING`) for all 12 affected tags before either build. Afterward, record the same inventory and count a tag as rebuilt only when this task's build output names it and its resulting ID or creation metadata belongs to this build. Do not treat a pre-existing unchanged tag as current evidence:

```bash
docker image inspect --format '{{.RepoTags}} {{.Id}}' \
  drone-sim-orchestration-runtime:phase2 \
  drone-sim-artifacts-runtime:phase2 \
  drone-sim-synthetic-companion:phase2 \
  drone-sim-synthetic-ardupilot-sitl:phase2 \
  drone-sim-synthetic-gazebo:phase2 \
  drone-sim-synthetic-electromagnet:phase2 \
  drone-sim-synthetic-scorekeeper:phase2 \
  drone-sim-companion-runtime:phase3 \
  drone-sim-ardupilot-runtime:phase3 \
  drone-sim-gazebo-runtime:phase3 \
  drone-sim-electromagnet-runtime:phase3 \
  drone-sim-scorekeeper-runtime:phase3
```

Generate the smoke list from tags proven rebuilt in this task, not from the static list below. Smoke-import the contract once from each distinct rebuilt image family; omit an unbuilt or stale tag and record it as unrun:

```bash
# Retain only tags whose rebuild was proven by this task's build output and
# before/after image inventory.
for image in \
  drone-sim-orchestration-runtime:phase2 \
  drone-sim-artifacts-runtime:phase2 \
  drone-sim-synthetic-companion:phase2 \
  drone-sim-companion-runtime:phase3 \
  drone-sim-ardupilot-runtime:phase3 \
  drone-sim-gazebo-runtime:phase3 \
  drone-sim-electromagnet-runtime:phase3 \
  drone-sim-scorekeeper-runtime:phase3; do
  docker run --rm --entrypoint python3 "$image" -c \
    'from artifacts.runtime_status import SourceFinishedStatus; print(SourceFinishedStatus.name)'
done
```

Report the exact environment-only blocker for any image that cannot be rebuilt. The image gate remains incomplete while a required tag is unbuilt. A live mission remains separate and is not part of this source migration.

- [ ] **Step 5: Prove consolidation reduced production code**

Compare production Python line counts against the approved baseline:

```bash
git diff --numstat 9731801a457131273a49683445a9061916887945 -- \
  ':(glob)artifacts/src/**/*.py' ':(glob)orchestration/src/**/*.py' \
  ':(glob)companion/src/**/*.py' ':(glob)ardupilot_sitl/src/**/*.py' \
  ':(glob)gazebo/src/**/*.py' ':(glob)electromagnet/src/**/*.py' \
  ':(glob)scorekeeper/src/**/*.py' \
  | awk '{ added += $1; deleted += $2 } END { printf "production Python: +%d -%d net %+d\n", added, deleted, added-deleted }'
```

The net must be negative; a net increase requires design review before committing Task 7.

- [ ] **Step 6: Commit documentation and verified evidence**

```bash
git add artifacts/README.md orchestration/README.md companion/README.md gazebo/README.md electromagnet/README.md scorekeeper/README.md ardupilot_sitl/README.md docs/architecture.md docs/handoff.md
git commit -m "docs: record typed runtime status ownership and verification"
```

After this task, hand the full branch to a fresh reviewer. The source migration may be code-complete while the verification gate remains explicitly incomplete because of a missing required image; do not state the entire slice passed in that case. Do not merge or push without the user's explicit instruction.
