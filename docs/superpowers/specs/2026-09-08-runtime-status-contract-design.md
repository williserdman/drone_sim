# Typed runtime-status contract design

## Objective

Give every durable runtime status one schema, one name registry, and one Python
representation. Producers must validate a fact before publication. Host and
runtime consumers must parse the same fact before acting on it.

This change fixes two demonstrated completion bugs:

- The host accepts `source-finished` without `finished: true` and can commit a
  completed manifest from it.
- The host accepts `runtime-frozen` without `frozen: true` and can inspect
  artifacts before publishers are quiescent.

It also removes the contradictory runtime and host definitions of
`artifacts-final`, the incomplete host name registry, and three local status
writers that bypass the shared protocol implementation.

## Chosen approach

Add `artifacts.runtime_status`, a pure domain module containing frozen status
values plus parsing and serialization. `RuntimeProtocol` and `StatusStore` keep
their existing filesystem responsibilities and consume that module.

The public flow becomes:

```text
producer -> RuntimeStatus -> RuntimeProtocol -> JSON file
                                             |
controller <- RuntimeStatus <- StatusStore <-+
```

Callers select a registered status type, not a filename string:

```python
protocol.write_status(RuntimeFrozenStatus(run_id))
finished = protocol.read_status(SourceFinishedStatus)
finished = store.read_runtime_status(
    run_id,
    SourceFinishedStatus,
    deadline_check=check,
)
```

This is an atomic cross-module migration. Do not retain the old
`write_status(name, document)` or `read_status(name)` interface after all
repository callers move.

## Boundaries

`runtime_status.py` owns:

- the 14 durable runtime-status names;
- exact top-level and nested JSON shapes;
- field types and cross-field rules;
- canonical run-ID matching;
- conversion between frozen Python values and JSON objects;
- the persistence policy assigned to each status type.

It does not open files, select directories or modes, hash artifacts, score a
mission, or decide whether a run succeeded.

`protocol_files.py` remains the only owner of canonical JSON bytes,
descriptor-relative I/O, stable reads, advisory locking, atomic replacement,
file modes, fsync, and cleanup.

`RuntimeProtocol` remains the runtime file adapter. `StatusStore` remains the
host run allocator and file adapter. The controller retains independent
artifact validation. The acceptance oracle retains independent physical and
scoring validation.

The following documents stay outside this contract:

- `.status/operator-state.json`;
- `.status/quiescence/<module>.json`;
- `.control/finalize-request.json`;
- `.control/terminal-committed.json`;
- `manifest.json`.

Their semantics belong to later finalization work.

## Public interface

The module exposes these base types and functions:

```python
class RuntimeStatusError(ValueError): ...

@dataclass(frozen=True)
class RuntimeStatus:
    run_id: str
    name: ClassVar[str]

StatusT = TypeVar("StatusT", bound=RuntimeStatus)

def canonical_run_id(value: object) -> str: ...

def status_name(status_type: type[StatusT]) -> str: ...

def parse_status(
    status_type: type[StatusT],
    document: object,
    *,
    expected_run_id: str,
) -> StatusT: ...

def status_document(status: RuntimeStatus) -> dict[str, Any]: ...

def status_write_policy(status_type: type[RuntimeStatus]) -> WritePolicy: ...
```

Only exact registered classes are valid. An arbitrary subclass cannot choose a
filename. Parsers reject non-dictionaries, missing or extra keys, invalid
constants, invalid nested values, noncanonical run IDs, and cross-run facts.

`status_document` returns a fresh JSON-compatible structure. It revalidates
nested mappings before returning, so mutation through a retained mapping cannot
publish an invalid fact.

Move `canonical_run_id` from `runtime_protocol.py` into this module and re-export
it from the old location for existing non-status imports.

## Status types

The registered values are:

```python
ArtifactsReadyStatus
GazeboReadyStatus
ArduPilotReadyStatus
CompanionReadyStatus
MissionReadyStatus
MissionCommandDeliveredStatus
RuntimeRunningStatus
SourceFinishedStatus
MissionFinishedStatus
ScoreFinishedStatus
RuntimeFailureStatus
RuntimeFrozenStatus
ArtifactsFinalStatus
TerminalNotifiedStatus
```

Constant fields such as `ready: true`, `finished: true`, and `frozen: true` are
not constructor arguments. Invalid false marker values cannot be represented by
a status instance.

The constructor fields are fixed:

| Type | Constructor fields after `run_id` |
| --- | --- |
| `ArtifactsReadyStatus` | none |
| `GazeboReadyStatus` | `flight_exchange: FlightExchange` |
| `ArduPilotReadyStatus` | none |
| `CompanionReadyStatus` | none |
| `MissionReadyStatus` | none |
| `MissionCommandDeliveredStatus` | `sim_timestamp_ns: int` |
| `RuntimeRunningStatus` | `sim_timestamp_ns: int` |
| `SourceFinishedStatus` | `sim_timestamp_ns: int` |
| `MissionFinishedStatus` | `sim_timestamp_ns: int` |
| `ScoreFinishedStatus` | `sim_timestamp_ns: int` |
| `RuntimeFailureStatus` | `module: str`, `reason: str`, `diagnostic_paths: tuple[str, ...]` |
| `RuntimeFrozenStatus` | none |
| `ArtifactsFinalStatus` | `records: tuple[ArtifactFinalRecord, ...]`; `complete` is computed |
| `TerminalNotifiedStatus` | none |

Each constructor validates its variable fields. Construction and parsing apply
the same rules.

Variable fields use these supporting values:

```python
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
```

## Exact schemas

Every `run_id` is a canonical lowercase UUID and must equal the selected run.
Every timestamp and counter is an `int`, not a `bool`. Timestamps and ordinary
counters are nonnegative.

| Status | Required JSON fields and values |
| --- | --- |
| `artifacts-ready` | `run_id`, `ready: true` |
| `gazebo-ready` | `run_id`, `ready: true`, `flight_exchange` |
| `ardupilot-ready` | `run_id`, `ready: true`, `json_exchange: true`, `mavlink_endpoint: "tcp://ardupilot-sitl:5760"` |
| `companion-ready` | `run_id`, `ready: true`, `mavlink_endpoint: "tcp://ardupilot-sitl:5760"`, `mavlink_transport_connected: true` |
| `mission-ready` | `run_id`, `ready: true`, `heartbeat_observed: true`, `prearm_checks_healthy: true` |
| `mission-command-delivered` | `run_id`, `command: "SET_GUIDED"`, `sim_timestamp_ns`, `delivered: true`; timestamp at most `50_000_000` |
| `runtime-running` | `run_id`, `state: "RUNNING"`, `sim_timestamp_ns` |
| `source-finished` | `run_id`, `finished: true`, `sim_timestamp_ns` |
| `mission-finished` | `run_id`, `finished: true`, `sim_timestamp_ns`, `outcome: "LANDED"` |
| `score-finished` | `run_id`, `finished: true`, `sim_timestamp_ns` |
| `runtime-failure` | `run_id`, nonempty `module`, nonempty `reason`, `diagnostic_paths` |
| `runtime-frozen` | `run_id`, `frozen: true` |
| `artifacts-final` | `run_id`, computed `complete`, `records` |
| `terminal-notified` | `run_id`, `notified: true` |

`flight_exchange` has exactly these fields:

```text
online: true
servo_packets_received: int >= 1
motor_updates: int >= 1
duplicate_servo_packets: int >= 0
servo_frame_gaps: 0
json_states_sent: int >= 1
json_send_errors: 0
last_servo_frame: int >= 0
last_json_sim_time_ns: int >= 0
```

`runtime-failure.diagnostic_paths` is a list of unique, nonempty relative POSIX
paths. Absolute paths, backslashes, `..`, and empty path components are invalid.
An empty list is valid.

## `artifacts-final` contract

`records` contains exactly these paths in this order:

```text
video/onboard.mp4
video/observer.mp4
rosbag
```

Each record has exactly `relative_path`, `status`, `detail`, `size_bytes`,
`sha256`, and `semantic`.

- `status` is `valid`, `missing`, or `invalid`.
- `detail` is nonempty.
- `semantic` is a nonempty JSON object. This status contract does not interpret
  codecs, frame counts, topic inventories, or physical evidence.
- A valid record has a nonnegative integer size and a lowercase 64-character
  hexadecimal SHA-256 value.
- A missing or invalid record has either both size and hash absent or both
  valid. This preserves facts about a stable file that failed semantic checks.
- `complete` equals whether all three records are valid. Serialization computes
  it rather than trusting a caller-supplied boolean.

These rules combine the runtime's fixed ordering with the host's stricter field
requirements. A syntactically valid report is still only a recorder claim. The
host reopens each named path and verifies its type, size, and checksum before
manifest publication.

## Publication policy

Every runtime status uses mode `0o644`.

Single-owner facts use `WritePolicy.IDENTICAL`. An equal retry succeeds and a
different value conflicts.

`RuntimeFailureStatus` uses `WritePolicy.FIRST_WINS` because it has several
producers. The first valid failure remains the actionable cause. A losing writer
does not overwrite it or fail before quiescence. Module logs retain later
diagnostics. The adapter parses an existing winner before accepting it.

No runtime status uses replace policy.

## Adapter contracts

`RuntimeProtocol` changes to:

```python
def write_status(self, status: RuntimeStatus) -> Path: ...

def read_status(self, status_type: type[StatusT]) -> StatusT | None: ...
```

It converts `RuntimeStatusError` and `ProtocolIOError` to `ProtocolError`.
`write_status` returns the published path. For a first-wins failure this return
does not mean that the caller's value won; consumers read the persisted typed
fact.

`StatusStore` changes to:

```python
def read_runtime_status(
    self,
    run_id: str,
    status_type: type[StatusT],
    deadline_check: Callable[[], None] | None = None,
) -> StatusT | None: ...
```

It converts `RuntimeStatusError` and `ProtocolIOError` to `ProtocolFileError`.
It must let the exact `TimeoutError` from a deadline callback escape unchanged.

The controller's generic wait takes a status type and returns that concrete
type. Readiness and timestamp code consumes fields directly. Delete its second
layer of structural validators.

## Migration

Implement and review the work in this order:

1. Add the pure contract and exhaustive table-driven tests.
2. Move `StatusStore`, the controller, and `ArtifactFinalReport` consumers to
   typed reads. This closes false completion and premature artifact inspection.
3. Move `RuntimeProtocol` to typed parsing and serialization.
4. Convert artifacts, orchestration, companion, Gazebo, electromagnet,
   scorekeeper, Phase 2 synthetic processes, and their test fakes.
5. Replace Gazebo's custom ready writer and scorekeeper's custom finished writer
   with `RuntimeProtocol`.
6. Add the artifacts source to the ArduPilot runtime image. Move its readiness,
   runtime failure, and quiescence files to `RuntimeProtocol`.
7. Delete the old string API, compatibility code, local validators, local status
   writers, and tests of removed implementations.
8. Update current module guides, resolve Compose profiles, run every affected
   suite, and rebuild every affected runtime image before any live claim.

Keep each intermediate commit green. Do not introduce a compatibility overload
that survives the completed slice.

## Required deletions

After migration, remove:

- `_STATUS_NAMES`, `_INITIAL_COMMAND_WINDOW_NS`, `_FLIGHT_EXCHANGE_KEYS`,
  `_safe_relative_path`, `_nonnegative_integer`, `_valid_flight_exchange`,
  `_validate_status`, and `_valid_artifacts_final` from
  `runtime_protocol.py`;
- `_RUNTIME_STATUS_NAMES` and run-ID-only runtime-status validation from
  `status_store.py`;
- `_ArtifactReportRecord`, the parsing half of `ArtifactFinalReport`, readiness
  validators, timestamp validators, and `_FLIGHT_EXCHANGE_KEYS` from
  `controller.py`;
- Gazebo's custom `GazeboReadyStatus` file writer;
- `scorekeeper/status.py` after its runtime publishes the typed fact;
- ArduPilot's `atomic_document` uses for shared readiness, failure, and
  quiescence files;
- all production `write_status(name, document)` and `read_status(name)` calls;
- compatibility overloads and tests tied only to removed implementations.

Keep host checksum and tree validation, media and bag validators, manifest
validation, quiescence validation, control validation, producer state machines,
and the independent acceptance oracle.

## Test requirements

Use red-green-refactor for every behavior change.

Pure contract tests cover:

- a valid round trip for all 14 types and their exact names;
- missing and extra keys, non-dictionary input, invalid and wrong run IDs;
- boolean and negative integer rejection;
- every `flight_exchange` constraint;
- safe and unsafe diagnostic paths;
- exact `source-finished` and `runtime-frozen` markers;
- every artifact-record status, field coupling, order, aggregate, and nested
  mutation;
- rejection of unregistered subclasses.

Adapter and controller tests cover:

- mode `0o644`, equal retry, conflict, and concurrent first-wins failure;
- malformed existing first-wins data;
- identical rejection of malformed status through runtime and host adapters;
- host access to `mission-command-delivered` through the shared inventory;
- exact `TimeoutError` propagation;
- malformed `source-finished` cannot complete a run or erase valid timing;
- malformed `runtime-frozen` cannot start log capture or artifact hashing;
- all previously divergent `artifacts-final` cases now agree;
- host size and checksum mismatches still fail independently;
- existing timestamp ordering and readiness behavior remains unchanged;
- Gazebo, scorekeeper, and ArduPilot publication uses the shared adapter.

Filesystem race, symlink, hard-link, FIFO, size, duplicate-key, and non-finite
JSON tests remain in `test_protocol_files.py`. Do not duplicate them in status
tests.

## Package and image impact

Orchestration, companion, Gazebo, electromagnet, artifacts, and Phase 2 images
already include the artifacts package or source.

Scorekeeper already copies `artifacts/src`; declare its local artifacts package
dependency in `scorekeeper/pyproject.toml`.

ArduPilot must copy `artifacts/src`, add it to `PYTHONPATH`, and declare the
local package dependency in `ardupilot_sitl/pyproject.toml`. The artifacts
contract and file adapter use only the standard library.

Update `uv.lock` only for local workspace metadata. Do not change external
versions.

Rebuild all runtime images that bake the shared protocol or begin using it.
Record new digests through the normal run provenance. Never edit old run
evidence.

This change adds no port, volume, environment variable, ROS interface, system
package, or operator command. It does not require a machine-setup update.

## Documentation

Update the same change:

- `artifacts/README.md` names `runtime_status.py` as the schema owner and
  `runtime_protocol.py` as the runtime file adapter.
- `orchestration/README.md` points validation to the shared contract and retains
  independent host artifact verification.
- `docs/architecture.md` records one schema owner and the first-wins failure
  rule.
- Gazebo, scorekeeper, and ArduPilot module guides remove references to local
  status writers.
- Other affected module guides link the shared contract without copying its
  schema table.
- `docs/handoff.md` records exact source tests, Compose checks, builds, and any
  live evidence. It distinguishes source verification from flight results.

The runbook needs no edit because operator commands and diagnosis procedures do
not change.

## Verification gate

The slice is complete only when:

1. All 14 statuses round-trip through both adapters.
2. The two false-completion probes and four `artifacts-final` disagreements fail
   closed through production call paths.
3. Every repository-owned status producer and consumer uses the typed API or a
   documented non-status interface.
4. Legacy shared-status writers and duplicate validators are absent.
5. All seven module suites and shared contract/integration tests pass, apart
   from explicitly reproduced environment-only failures.
6. Base, Phase 2, and Phase 3 GPU Compose configurations resolve.
7. A line-count audit confirms that consolidation removed more duplicated
   production code than the new contract added. A net increase requires design
   review before merge.
8. Every affected runtime image builds from the changed source. A live mission
   remains a separate physical verification step.

## Rejected approaches

A host-only validation patch would close the immediate false-completion bug but
leave filename strings, dictionaries, local writers, and four competing schema
definitions. It would make the next cleanup harder.

A generic field-rule dictionary would replace explicit code with a private
validation language. Callers would still receive untyped dictionaries, and
cross-field rules would remain hard to read.

Fourteen frozen values are more code than one permissive parser. They delete
more code across the repository and make invalid constant facts impossible to
construct.
