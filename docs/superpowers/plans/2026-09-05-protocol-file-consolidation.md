# Protocol file consolidation implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace two independent implementations of descriptor-safe protocol JSON I/O with one tested module while preserving every host and runtime behavior.

**Architecture:** Add a low-level `artifacts.protocol_files` module because both artifacts and orchestration already depend on the artifacts package. It owns strict JSON encoding, safe descriptor-relative reads, and atomic writes. `RuntimeProtocol` and `StatusStore` keep their domain schemas, public exceptions, directory ownership, deadlines, permissions, and persistence-policy choices.

**Tech stack:** Python 3.12, Linux descriptor-relative filesystem calls, `fcntl`, `pytest`, `uv`.

**Spec:** `docs/superpowers/specs/2026-09-05-code-simplification-hardening-design.md`

**Scope:** This is the first independently testable part of the broader cleanup.
Completing it does not complete the thread goal. Separate plans will cover the
orchestration audit, artifact recorders, companion/Gazebo runtime paths, policy
modules, ArduPilot, cross-repository waste removal, and the final audit.

## Global constraints

- Preserve flight behavior, simulation timing, QoS, scoring, artifact formats, operator commands, startup ordering, and finalization ordering.
- Keep fail-closed validation.
- Do not change generated code, vendored code, provenance, licenses, run evidence, the nested mission checkout, or unrelated working-tree changes.
- The shared module owns filesystem and JSON mechanics only. Domain schemas remain with their callers.
- `TimeoutError` from an orchestration deadline callback must propagate unchanged.
- Runtime status files remain exactly mode `0o644`; host control and status files remain mode `0o600`.
- Update both affected module READMEs and the shared handoff in the same change.

---

### Task 1: Specify the shared protocol-file interface

**Files:**
- Create: `artifacts/tests/test_protocol_files.py`

**Interfaces:**
- Consumes: Linux file descriptors opened for a trusted protocol directory.
- Produces test requirements for `ProtocolIOError`, `WritePolicy`, `canonical_json`, `read_json_object_at`, and `write_json_object_at`.

- [ ] **Step 1: Add interface tests that fail because the module does not exist**

Create `artifacts/tests/test_protocol_files.py` with fixtures that open a temporary directory using `os.open(tmp_path, os.O_RDONLY | os.O_DIRECTORY)`. Import:

```python
from artifacts.protocol_files import (
    ProtocolIOError,
    WritePolicy,
    canonical_json,
    read_json_object_at,
    write_json_object_at,
)
```

Cover these exact cases:

```python
def test_canonical_json_is_strict_sorted_and_newline_terminated():
    assert canonical_json({"z": 1, "a": "x"}) == b'{"a":"x","z":1}\n'
    with pytest.raises(ProtocolIOError):
        canonical_json({"value": float("nan")})


@pytest.mark.parametrize(
    "payload",
    [b'{"x":1,"x":2}', b'{"x":NaN}', b'[]', b'\xff'],
)
def test_read_rejects_noncanonical_json_objects(directory_fd, tmp_path, payload):
    (tmp_path / "state.json").write_bytes(payload)
    with pytest.raises(ProtocolIOError):
        read_json_object_at(directory_fd, "state.json")


def test_read_calls_deadline_before_during_and_after_io(directory_fd, tmp_path):
    (tmp_path / "state.json").write_text('{"run_id":"x"}')
    calls = 0
    def check():
        nonlocal calls
        calls += 1
    assert read_json_object_at(directory_fd, "state.json", deadline_check=check) == {"run_id": "x"}
    assert calls >= 4


def test_deadline_error_propagates_unchanged(directory_fd, tmp_path):
    (tmp_path / "state.json").write_text('{"run_id":"x"}')
    marker = TimeoutError("budget expired")
    def check():
        raise marker
    with pytest.raises(TimeoutError) as raised:
        read_json_object_at(directory_fd, "state.json", deadline_check=check)
    assert raised.value is marker
```

Add tests for these filesystem cases:

- missing files return `None`;
- files larger than `4 * 1024 * 1024` bytes fail;
- symlinks, hard links, and FIFOs fail without blocking;
- replacing the name between inspection and open fails;
- changing a file during reading fails;
- the read descriptor closes after success and failure.

Add parameterized write-policy tests:

```python
@pytest.mark.parametrize("mode", [0o600, 0o644])
def test_write_sets_exact_mode_and_fsyncs_file_and_directory(...): ...

def test_identical_policy_allows_same_retry_and_rejects_conflict(...): ...
def test_first_wins_policy_returns_original_document(...): ...
def test_replace_policy_atomically_replaces_document(...): ...
def test_interrupted_write_removes_temporary_file(...): ...
def test_concurrent_first_wins_has_one_persisted_winner(...): ...
```

Assert that `write_json_object_at` returns `(persisted_document, created)` and leaves no `.*.tmp` sibling. Monkeypatch `os.write`, `os.fsync`, `os.replace`, or the module's inspection helper at the narrow point needed to force each failure.

- [ ] **Step 2: Run the new test file and verify the expected import failure**

Run:

```bash
.venv/bin/python -m pytest artifacts/tests/test_protocol_files.py -q
```

Expected: collection fails with `ModuleNotFoundError: No module named 'artifacts.protocol_files'`.

- [ ] **Step 3: Commit the failing interface tests**

```bash
git add artifacts/tests/test_protocol_files.py
git commit -m "test protocol file safety interface"
```

---

### Task 2: Implement descriptor-safe JSON persistence once

**Files:**
- Create: `artifacts/src/artifacts/protocol_files.py`
- Modify: `artifacts/tests/test_protocol_files.py`

**Interfaces:**
- Consumes: a trusted directory file descriptor, a single safe filename supplied by domain callers, a JSON mapping, an exact file mode, an optional deadline callback, and a `WritePolicy`.
- Produces:

```python
class ProtocolIOError(RuntimeError): ...

class WritePolicy(Enum):
    IDENTICAL = "identical"
    FIRST_WINS = "first_wins"
    REPLACE = "replace"

def canonical_json(document: Mapping[str, Any]) -> bytes: ...

def read_json_object_at(
    directory_fd: int,
    name: str,
    *,
    deadline_check: Callable[[], None] | None = None,
    max_bytes: int = 4 * 1024 * 1024,
) -> dict[str, Any] | None: ...

def write_json_object_at(
    directory_fd: int,
    name: str,
    document: Mapping[str, Any],
    *,
    mode: int,
    policy: WritePolicy,
) -> tuple[dict[str, Any], bool]: ...
```

- [ ] **Step 1: Add the strict encoder and decoder**

Implement `canonical_json` with `allow_nan=False`, `ensure_ascii=False`, sorted keys, compact separators, and one trailing newline. Decode with an `object_pairs_hook` that rejects duplicate keys and a `parse_constant` callback that rejects `NaN` and infinities. Reject a top-level value that is not a JSON object. Wrap JSON, Unicode, recursion, type, and numeric encoding errors in `ProtocolIOError` with the protocol filename where available.

- [ ] **Step 2: Add one snapshot comparison and safe inspection helper**

Use the fields:

```python
_SNAPSHOT_FIELDS = (
    "st_dev", "st_ino", "st_mode", "st_nlink", "st_size",
    "st_mtime_ns", "st_ctime_ns",
)
```

Inspection must use `follow_symlinks=False`, require a regular file, and require exactly one hard link. Opening must use `O_RDONLY | O_CLOEXEC | O_NOFOLLOW`. Compare the full snapshot before opening, after reading, and against the final named path.

- [ ] **Step 3: Add bounded, deadline-aware reads**

Call `deadline_check` before inspection, before and after each `os.read`, before decoding, and after decoding. Read at most `max_bytes + 1`, then reject excess data. Let `TimeoutError` propagate unchanged. Convert filesystem errors to `ProtocolIOError`. Close the descriptor in `finally`.

- [ ] **Step 4: Add atomic writes with explicit policies**

Lock the directory descriptor with `fcntl.flock(..., LOCK_EX)`. Read any existing document through `read_json_object_at`, then apply:

- `IDENTICAL`: return `(existing, False)` when equal; otherwise raise `ProtocolIOError` containing `conflicts`;
- `FIRST_WINS`: return `(existing, False)` without replacement;
- `REPLACE`: write and atomically replace whether or not a document exists.

Create `.{name}.{uuid4().hex}.tmp` with `O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW`. Call `os.fchmod(descriptor, mode)`, write until complete, `fsync` the file, close it, `os.replace` within the same directory descriptor, and `fsync` the directory. Clean up the descriptor, temporary name, and lock in `finally`.

- [ ] **Step 5: Run the shared tests**

```bash
.venv/bin/python -m pytest artifacts/tests/test_protocol_files.py -q
```

Expected: all tests pass.

- [ ] **Step 6: Run existing protocol tests before adapting callers**

```bash
.venv/bin/python -m pytest artifacts/tests/test_runtime_protocol.py orchestration/tests/test_status_store.py -q
```

Expected baseline: 89 tests pass. Existing callers still use their local implementations.

- [ ] **Step 7: Commit the shared implementation**

```bash
git add artifacts/src/artifacts/protocol_files.py artifacts/tests/test_protocol_files.py
git commit -m "add shared descriptor-safe protocol files"
```

---

### Task 3: Move the artifacts runtime onto the shared module

**Files:**
- Modify: `artifacts/src/artifacts/runtime_protocol.py:1-564`
- Modify: `artifacts/tests/test_runtime_protocol.py`

**Interfaces:**
- Consumes: `canonical_json`, `read_json_object_at`, `write_json_object_at`, `ProtocolIOError`, and `WritePolicy.IDENTICAL` from Task 2.
- Produces: the unchanged public `ProtocolError`, `RuntimeProtocol`, and `canonical_run_id` interface.

- [ ] **Step 1: Add a characterization test for public error translation**

Add a test that writes malformed JSON, calls `RuntimeProtocol.read_finalize_request`, and asserts:

```python
with pytest.raises(ProtocolError, match="contains invalid JSON") as raised:
    protocol.read_finalize_request()
assert type(raised.value) is ProtocolError
assert isinstance(raised.value.__cause__, ProtocolIOError)
```

Import `ProtocolIOError` from `artifacts.protocol_files`. This test must fail before the caller delegates to the shared module because the old implementation raises `ProtocolError` directly.

- [ ] **Step 2: Run the characterization test and verify it fails**

```bash
.venv/bin/python -m pytest artifacts/tests/test_runtime_protocol.py::test_runtime_protocol_translates_shared_io_errors -q
```

Expected: failure because the raised `ProtocolError` has no `ProtocolIOError` cause.

- [ ] **Step 3: Replace local JSON and file helpers**

Delete `_duplicate_pairs`, `_invalid_constant`, `_canonical_json`, `_same_snapshot`, `_inspect`, `_read_at`, `_write_at`, `_FILE_FLAGS`, `_MAX_BYTES`, and their now-unused imports. Add private adapters:

```python
_T = TypeVar("_T")

def _translate_io(call: Callable[[], _T]) -> _T:
    try:
        return call()
    except ProtocolIOError as error:
        raise ProtocolError(str(error)) from error
```

Import `TypeVar` from `typing` for this adapter.

Use `read_json_object_at` for reads. Use `write_json_object_at` with `mode=0o644` and `policy=WritePolicy.IDENTICAL` for runtime status and quiescence markers. Keep `_open_directory_at`, all schema validators, control immutability tracking, and manifest projection in `runtime_protocol.py`.

Use shared `canonical_json` only to compare the first observed control document. Translate its `ProtocolIOError` to the public `ProtocolError` too.

- [ ] **Step 4: Run artifacts protocol tests**

```bash
.venv/bin/python -m pytest artifacts/tests/test_protocol_files.py artifacts/tests/test_runtime_protocol.py -q
```

Expected: all tests pass, including exact `0o644` status permissions, fsync coverage, unsafe-path rejection, immutable controls, and the new exception-cause test.

- [ ] **Step 5: Run the complete artifacts suite**

```bash
.venv/bin/python -m pytest artifacts/tests -q
```

Expected: all runnable tests pass; record any environment skips separately.

- [ ] **Step 6: Commit the artifacts migration**

```bash
git add artifacts/src/artifacts/runtime_protocol.py artifacts/tests/test_runtime_protocol.py
git commit -m "reuse shared protocol files in artifacts runtime"
```

---

### Task 4: Move the orchestration host store onto the shared module

**Files:**
- Modify: `orchestration/src/orchestration/status_store.py:1-463`
- Modify: `orchestration/tests/test_status_store.py`

**Interfaces:**
- Consumes: the Task 2 shared protocol-file interface and all three `WritePolicy` values.
- Produces: the unchanged public `ProtocolFileError`, `TerminalCause`, `OperatorStatus`, and `StatusStore` interface.

- [ ] **Step 1: Add a host error-translation test**

Write malformed JSON to `.status/operator-state.json`, then assert:

```python
with pytest.raises(ProtocolFileError, match="contains invalid JSON") as raised:
    store.read_operator_status(RUN_ID)
assert type(raised.value) is ProtocolFileError
assert isinstance(raised.value.__cause__, ProtocolIOError)
```

This must fail before migration because the old implementation raises `ProtocolFileError` directly.

- [ ] **Step 2: Run the new host test and verify it fails**

```bash
.venv/bin/python -m pytest orchestration/tests/test_status_store.py::test_status_store_translates_shared_io_errors -q
```

Expected: failure because the old error has no `ProtocolIOError` cause.

- [ ] **Step 3: Replace host-local JSON and file helpers**

Delete `_reject_duplicate_pairs`, `_reject_json_constant`, `_canonical_json`, `_inspect_existing`, `_read_document_at`, `_write_document_at`, `_FILE_FLAGS`, `_MAX_PROTOCOL_BYTES`, and unused imports. Add local adapters that translate `ProtocolIOError` to `ProtocolFileError` while allowing `TimeoutError` through unchanged.

Map existing call sites exactly:

- `write_operator_status` uses `mode=0o600`, `WritePolicy.REPLACE`;
- `request_finalization` uses `mode=0o600`, `WritePolicy.FIRST_WINS`;
- `write_terminal_committed` uses `mode=0o600`, `WritePolicy.FIRST_WINS`, then retains its explicit conflicting-document check;
- `read_operator_status`, `read_finalize_request`, `read_runtime_status`, and `validated_manifest_result` use `read_json_object_at`;
- runtime-status and manifest reads pass through their existing `deadline_check`.

Keep output-root allocation, safe directory opening, domain validation, manifest construction, and cleanup in `StatusStore`.

- [ ] **Step 4: Run host protocol tests**

```bash
.venv/bin/python -m pytest artifacts/tests/test_protocol_files.py orchestration/tests/test_status_store.py -q
```

Expected: all tests pass. Confirm the existing cooperative deadline test still proves callback use and `TimeoutError` propagation.

- [ ] **Step 5: Run the complete orchestration suite**

```bash
.venv/bin/python -m pytest orchestration/tests -q
```

Expected: all tests whose declared fixtures are available pass. If nested-revision provenance is unavailable in the isolated worktree, record those environment failures without changing validation.

- [ ] **Step 6: Compare public imports and method names**

```bash
rg -n 'from (artifacts\.runtime_protocol|orchestration\.status_store) import' \
  --glob '*.py' --glob '!companion/comp2026/**'
```

Verify every imported public name still exists. Do not export low-level protocol-file helpers from package `__init__.py`; callers import the explicit module.

- [ ] **Step 7: Commit the orchestration migration**

```bash
git add orchestration/src/orchestration/status_store.py orchestration/tests/test_status_store.py
git commit -m "reuse shared protocol files in host status store"
```

---

### Task 5: Document ownership and verify the slice

**Files:**
- Modify: `artifacts/README.md`
- Modify: `orchestration/README.md`
- Modify: `docs/handoff.md`

**Interfaces:**
- Consumes: the completed shared protocol-file module and unchanged caller interfaces.
- Produces: current ownership notes and verification evidence for this cleanup slice.

- [ ] **Step 1: Update affected module guides**

In `artifacts/README.md`, add `protocol_files.py` to the code map as the shared strict JSON and descriptor-safe persistence implementation. Keep `runtime_protocol.py` described as the runtime schema and lifecycle-facing protocol.

In `orchestration/README.md`, state that `StatusStore` owns host allocation and document policy while delegating low-level safe persistence to `artifacts.protocol_files`. Do not duplicate file flags, limits, or policy tables in prose.

- [ ] **Step 2: Run all focused protocol tests together**

```bash
.venv/bin/python -m pytest \
  artifacts/tests/test_protocol_files.py \
  artifacts/tests/test_runtime_protocol.py \
  orchestration/tests/test_status_store.py -q
```

Expected: all tests pass.

- [ ] **Step 3: Run affected full suites and shared contracts**

```bash
.venv/bin/python -m pytest artifacts/tests orchestration/tests tests/contracts \
  tests/integration/test_phase2_runtime_contract.py -q
```

Expected: all runnable tests pass. Record passed, failed, and skipped counts. Do not weaken source-provenance checks to remove environment failures.

- [ ] **Step 4: Resolve Compose configurations**

```bash
docker compose config --quiet
docker compose --profile phase2 config --quiet
docker compose -f compose.yaml -f compose.gpu.yaml --profile phase3 config --quiet
```

Expected: all three commands exit zero.

- [ ] **Step 5: Measure the simplification**

```bash
git diff --numstat HEAD~4 -- \
  artifacts/src/artifacts/protocol_files.py \
  artifacts/src/artifacts/runtime_protocol.py \
  orchestration/src/orchestration/status_store.py
rg -n '_duplicate_pairs|_reject_duplicate_pairs|_invalid_constant|_reject_json_constant|_read_at|_read_document_at|_write_at|_write_document_at' \
  artifacts/src/artifacts/runtime_protocol.py orchestration/src/orchestration/status_store.py
```

The second command must find no duplicate low-level helpers. Record the net production line change and the invariants now owned by one module. A larger line count is a design failure unless new tests expose a previously unhandled fault.

- [ ] **Step 6: Update the dated handoff**

Add the exact verification results, production-line delta, and any remaining failure to `docs/handoff.md`. State that no runtime images or flight were exercised unless they actually were.

- [ ] **Step 7: Check documentation and whitespace**

```bash
git diff --check
PYTHONPATH=orchestration/src:artifacts/src .venv/bin/python /tmp/drone_sim_check_handoff_docs.py
```

Expected: no whitespace errors, all retained local links resolve, and all five run templates retain their documented values.

- [ ] **Step 8: Commit documentation and verification evidence**

```bash
git add artifacts/README.md orchestration/README.md docs/handoff.md
git commit -m "document shared protocol file ownership"
```

- [ ] **Step 9: Audit the final diff**

```bash
git status --short
git diff --stat HEAD~5..HEAD
git diff HEAD~5..HEAD -- \
  artifacts/src/artifacts/protocol_files.py \
  artifacts/src/artifacts/runtime_protocol.py \
  orchestration/src/orchestration/status_store.py \
  artifacts/tests/test_protocol_files.py \
  artifacts/tests/test_runtime_protocol.py \
  orchestration/tests/test_status_store.py \
  artifacts/README.md orchestration/README.md docs/handoff.md
git -C companion/comp2026 status --short
```

Confirm the slice changed only the listed files plus its design/plan history,
and that unrelated working-tree changes and nested mission status are unchanged.

## Campaign continuation

After this plan passes, inspect current code and write the next independent plan
for orchestration lifecycle and finalization. Continue through the audit order in
the design. Do not mark the thread goal complete when this protocol slice ends.
The final campaign plan must run the design's seven-module completion audit and
remove superseded design/plan files from the active documentation tree.
