"""Typed runtime-status values and their strict JSON wire contract."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, fields
import math
import re
from types import MappingProxyType
from typing import Any, Callable, ClassVar, TypeVar
from uuid import UUID

from .protocol_files import WritePolicy
from .validation import ValidationStatus


__all__ = [
    "ArduPilotReadyStatus",
    "ArtifactFinalRecord",
    "ArtifactsFinalStatus",
    "ArtifactsReadyStatus",
    "CompanionReadyStatus",
    "FlightExchange",
    "GazeboReadyStatus",
    "MissionCommandDeliveredStatus",
    "MissionFinishedStatus",
    "MissionReadyStatus",
    "RuntimeFailureStatus",
    "RuntimeFrozenStatus",
    "RuntimeRunningStatus",
    "RuntimeStatus",
    "RuntimeStatusError",
    "ScoreFinishedStatus",
    "SourceFinishedStatus",
    "StatusT",
    "TerminalNotifiedStatus",
    "canonical_run_id",
    "parse_status",
    "status_document",
    "status_name",
    "status_write_policy",
]


class RuntimeStatusError(ValueError):
    """A runtime status violates the typed or wire contract."""


def canonical_run_id(value: object) -> str:
    if type(value) is not str:
        raise RuntimeStatusError("run_id must be a canonical UUID")
    try:
        parsed = UUID(value)
    except (ValueError, TypeError, AttributeError) as error:
        raise RuntimeStatusError("run_id must be a canonical UUID") from error
    if str(parsed) != value:
        raise RuntimeStatusError("run_id must be a canonical UUID")
    return value


@dataclass(frozen=True)
class RuntimeStatus:
    run_id: str
    name: ClassVar[str]

    def __post_init__(self) -> None:
        canonical_run_id(self.run_id)


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

    def __post_init__(self) -> None:
        if self.online is not True:
            raise RuntimeStatusError("flight exchange must be online")
        counters = (
            self.servo_packets_received,
            self.motor_updates,
            self.duplicate_servo_packets,
            self.servo_frame_gaps,
            self.json_states_sent,
            self.json_send_errors,
            self.last_servo_frame,
            self.last_json_sim_time_ns,
        )
        if any(type(value) is not int or value < 0 for value in counters):
            raise RuntimeStatusError("flight exchange counters must be nonnegative integers")
        if min(
            self.servo_packets_received,
            self.motor_updates,
            self.json_states_sent,
        ) < 1:
            raise RuntimeStatusError("flight exchange progress counters must be positive")
        if self.servo_frame_gaps != 0 or self.json_send_errors != 0:
            raise RuntimeStatusError("flight exchange error counters must be zero")


def _require_timestamp(value: object) -> None:
    if type(value) is not int or value < 0:
        raise RuntimeStatusError("sim_timestamp_ns must be a nonnegative integer")


def _valid_diagnostic_path(value: object) -> bool:
    if type(value) is not str or not value or "\\" in value or value.startswith("/"):
        return False
    return all(component not in {"", ".", ".."} for component in value.split("/"))


def _copy_semantic_value(value: object, active: set[int]) -> Any:
    if value is None or type(value) in {bool, int, str}:
        return value
    if type(value) is float:
        if not math.isfinite(value):
            raise RuntimeStatusError("semantic floats must be finite")
        return value
    if isinstance(value, Mapping):
        identity = id(value)
        if identity in active:
            raise RuntimeStatusError("semantic containers must not be recursive")
        active.add(identity)
        try:
            copied: dict[str, Any] = {}
            for key, nested in value.items():
                if type(key) is not str:
                    raise RuntimeStatusError("semantic mapping keys must be strings")
                copied[key] = _copy_semantic_value(nested, active)
            return copied
        finally:
            active.remove(identity)
    if type(value) in {list, tuple}:
        identity = id(value)
        if identity in active:
            raise RuntimeStatusError("semantic containers must not be recursive")
        active.add(identity)
        try:
            return [_copy_semantic_value(nested, active) for nested in value]
        finally:
            active.remove(identity)
    raise RuntimeStatusError("semantic value is not JSON-compatible")


def _copy_semantic_mapping(value: object) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise RuntimeStatusError("artifact semantic must be a mapping")
    try:
        copied = _copy_semantic_value(value, set())
    except RecursionError as error:
        raise RuntimeStatusError("artifact semantic exceeds recursion depth") from error
    if not copied:
        raise RuntimeStatusError("artifact semantic must be nonempty")
    return copied


_ARTIFACT_FINAL_PATHS = (
    "video/onboard.mp4",
    "video/observer.mp4",
    "rosbag",
)


_StatusField = tuple[str, Callable[[Any], Any], Callable[[Any], Any]]


@dataclass(frozen=True)
class _StatusDefinition:
    name: str
    write_policy: WritePolicy = WritePolicy.IDENTICAL
    fixed_fields: tuple[tuple[str, Any], ...] = ()
    constructor_fields: tuple[_StatusField, ...] = ()
    derived_fields: tuple[tuple[str, Callable[[RuntimeStatus], Any]], ...] = ()


StatusT = TypeVar("StatusT", bound=RuntimeStatus)


@dataclass(frozen=True)
class ArtifactsReadyStatus(RuntimeStatus):
    pass


@dataclass(frozen=True)
class GazeboReadyStatus(RuntimeStatus):
    flight_exchange: FlightExchange

    def __post_init__(self) -> None:
        super().__post_init__()
        if type(self.flight_exchange) is not FlightExchange:
            raise RuntimeStatusError("flight_exchange must be a FlightExchange")


@dataclass(frozen=True)
class ArduPilotReadyStatus(RuntimeStatus):
    pass


@dataclass(frozen=True)
class CompanionReadyStatus(RuntimeStatus):
    pass


@dataclass(frozen=True)
class MissionReadyStatus(RuntimeStatus):
    pass


@dataclass(frozen=True)
class MissionCommandDeliveredStatus(RuntimeStatus):
    sim_timestamp_ns: int

    def __post_init__(self) -> None:
        super().__post_init__()
        _require_timestamp(self.sim_timestamp_ns)
        if self.sim_timestamp_ns > 50_000_000:
            raise RuntimeStatusError("mission command timestamp exceeds startup window")


@dataclass(frozen=True)
class RuntimeRunningStatus(RuntimeStatus):
    sim_timestamp_ns: int

    def __post_init__(self) -> None:
        super().__post_init__()
        _require_timestamp(self.sim_timestamp_ns)


@dataclass(frozen=True)
class SourceFinishedStatus(RuntimeStatus):
    sim_timestamp_ns: int

    def __post_init__(self) -> None:
        super().__post_init__()
        _require_timestamp(self.sim_timestamp_ns)


@dataclass(frozen=True)
class MissionFinishedStatus(RuntimeStatus):
    sim_timestamp_ns: int

    def __post_init__(self) -> None:
        super().__post_init__()
        _require_timestamp(self.sim_timestamp_ns)


@dataclass(frozen=True)
class ScoreFinishedStatus(RuntimeStatus):
    sim_timestamp_ns: int

    def __post_init__(self) -> None:
        super().__post_init__()
        _require_timestamp(self.sim_timestamp_ns)


@dataclass(frozen=True)
class RuntimeFailureStatus(RuntimeStatus):
    module: str
    reason: str
    diagnostic_paths: tuple[str, ...]

    def __post_init__(self) -> None:
        super().__post_init__()
        if type(self.module) is not str or not self.module:
            raise RuntimeStatusError("failure module must be a nonempty string")
        if type(self.reason) is not str or not self.reason:
            raise RuntimeStatusError("failure reason must be a nonempty string")
        if type(self.diagnostic_paths) is not tuple:
            raise RuntimeStatusError("diagnostic_paths must be a tuple")
        if any(not _valid_diagnostic_path(path) for path in self.diagnostic_paths):
            raise RuntimeStatusError("diagnostic path must be relative POSIX syntax")
        if len(self.diagnostic_paths) != len(set(self.diagnostic_paths)):
            raise RuntimeStatusError("diagnostic paths must be unique")


@dataclass(frozen=True)
class RuntimeFrozenStatus(RuntimeStatus):
    pass


@dataclass(frozen=True)
class ArtifactFinalRecord:
    relative_path: str
    status: ValidationStatus
    detail: str
    size_bytes: int | None
    sha256: str | None
    semantic: Mapping[str, Any]

    def __post_init__(self) -> None:
        if type(self.relative_path) is not str:
            raise RuntimeStatusError("artifact relative_path must be a string")
        if type(self.status) is not ValidationStatus:
            raise RuntimeStatusError("artifact status must be a ValidationStatus")
        if type(self.detail) is not str or not self.detail:
            raise RuntimeStatusError("artifact detail must be a nonempty string")
        if self.size_bytes is not None and (
            type(self.size_bytes) is not int or self.size_bytes < 0
        ):
            raise RuntimeStatusError("artifact size must be a nonnegative integer")
        if self.sha256 is not None and (
            type(self.sha256) is not str
            or re.fullmatch(r"[0-9a-f]{64}", self.sha256) is None
        ):
            raise RuntimeStatusError("artifact sha256 must be a lowercase digest")
        both_present = self.size_bytes is not None and self.sha256 is not None
        both_absent = self.size_bytes is None and self.sha256 is None
        if self.status is ValidationStatus.VALID and not both_present:
            raise RuntimeStatusError("valid artifacts require size and sha256")
        if self.status is not ValidationStatus.VALID and not (
            both_present or both_absent
        ):
            raise RuntimeStatusError("artifact size and sha256 must both be present or absent")
        _copy_semantic_mapping(self.semantic)


@dataclass(frozen=True)
class ArtifactsFinalStatus(RuntimeStatus):
    records: tuple[ArtifactFinalRecord, ...]

    def __post_init__(self) -> None:
        super().__post_init__()
        if type(self.records) is not tuple or len(self.records) != 3:
            raise RuntimeStatusError("artifacts-final requires exactly three records")
        if any(type(record) is not ArtifactFinalRecord for record in self.records):
            raise RuntimeStatusError("artifacts-final records must be ArtifactFinalRecord values")
        if tuple(record.relative_path for record in self.records) != _ARTIFACT_FINAL_PATHS:
            raise RuntimeStatusError("artifacts-final records have the wrong paths or order")

    @property
    def complete(self) -> bool:
        return all(record.status is ValidationStatus.VALID for record in self.records)


@dataclass(frozen=True)
class TerminalNotifiedStatus(RuntimeStatus):
    pass


def _flight_document(value: FlightExchange) -> dict[str, Any]:
    value.__post_init__()
    return {field.name: getattr(value, field.name) for field in fields(FlightExchange)}


_FLIGHT_EXCHANGE_KEYS = frozenset(field.name for field in fields(FlightExchange))


def _parse_flight_exchange(value: object) -> FlightExchange:
    if type(value) is not dict or set(value) != _FLIGHT_EXCHANGE_KEYS:
        raise RuntimeStatusError("runtime status has an invalid schema")
    return FlightExchange(**value)


def _record_document(record: ArtifactFinalRecord) -> dict[str, Any]:
    record.__post_init__()
    document = {
        field.name: getattr(record, field.name)
        for field in fields(ArtifactFinalRecord)
    }
    document["status"] = record.status.value
    document["semantic"] = _copy_semantic_mapping(record.semantic)
    return document


_ARTIFACT_RECORD_KEYS = frozenset(field.name for field in fields(ArtifactFinalRecord))


def _parse_records(value: object) -> tuple[ArtifactFinalRecord, ...]:
    if type(value) is not list or len(value) != 3:
        raise RuntimeStatusError("artifacts-final records must be a three-item list")
    if any(
        type(record) is not dict
        or set(record) != _ARTIFACT_RECORD_KEYS
        or type(record["status"]) is not str
        or type(record["semantic"]) is not dict
        or not record["semantic"]
        for record in value
    ):
        raise RuntimeStatusError("artifact record has an invalid schema")
    try:
        return tuple(
            ArtifactFinalRecord(
                record["relative_path"],
                ValidationStatus(record["status"]),
                record["detail"],
                record["size_bytes"],
                record["sha256"],
                record["semantic"],
            )
            for record in value
        )
    except ValueError as error:
        if isinstance(error, RuntimeStatusError):
            raise
        raise RuntimeStatusError("artifact record status is invalid") from error


def _identity(value: Any) -> Any:
    return value


def _diagnostic_paths_document(value: tuple[str, ...]) -> list[str]:
    return list(value)


def _parse_diagnostic_paths(value: object) -> tuple[str, ...]:
    if type(value) is not list:
        raise RuntimeStatusError("diagnostic_paths must be a JSON list")
    return tuple(value)


def _records_document(value: tuple[ArtifactFinalRecord, ...]) -> list[dict[str, Any]]:
    return [_record_document(record) for record in value]


_VALUE_FIELD = lambda name: (name, _identity, _identity)
_TIMESTAMP_FIELD = _VALUE_FIELD("sim_timestamp_ns")
_READY_FIELDS = (("ready", True),)
_STATUS_REGISTRY: Mapping[type[RuntimeStatus], _StatusDefinition] = MappingProxyType(
    {
        ArtifactsReadyStatus: _StatusDefinition(
            "artifacts-ready", fixed_fields=_READY_FIELDS
        ),
        GazeboReadyStatus: _StatusDefinition(
            "gazebo-ready",
            fixed_fields=_READY_FIELDS,
            constructor_fields=((
                "flight_exchange", _flight_document, _parse_flight_exchange
            ),),
        ),
        ArduPilotReadyStatus: _StatusDefinition(
            "ardupilot-ready",
            fixed_fields=(
                ("ready", True),
                ("json_exchange", True),
                ("mavlink_endpoint", "tcp://ardupilot-sitl:5760"),
            ),
        ),
        CompanionReadyStatus: _StatusDefinition(
            "companion-ready",
            fixed_fields=(
                ("ready", True),
                ("mavlink_endpoint", "tcp://ardupilot-sitl:5760"),
                ("mavlink_transport_connected", True),
            ),
        ),
        MissionReadyStatus: _StatusDefinition(
            "mission-ready",
            fixed_fields=(
                ("ready", True),
                ("heartbeat_observed", True),
                ("prearm_checks_healthy", True),
            ),
        ),
        MissionCommandDeliveredStatus: _StatusDefinition(
            "mission-command-delivered",
            fixed_fields=(("command", "SET_GUIDED"), ("delivered", True)),
            constructor_fields=(_TIMESTAMP_FIELD,),
        ),
        RuntimeRunningStatus: _StatusDefinition(
            "runtime-running", fixed_fields=(("state", "RUNNING"),),
            constructor_fields=(_TIMESTAMP_FIELD,)
        ),
        SourceFinishedStatus: _StatusDefinition(
            "source-finished", fixed_fields=(("finished", True),),
            constructor_fields=(_TIMESTAMP_FIELD,)
        ),
        MissionFinishedStatus: _StatusDefinition(
            "mission-finished",
            fixed_fields=(("finished", True), ("outcome", "LANDED")),
            constructor_fields=(_TIMESTAMP_FIELD,),
        ),
        ScoreFinishedStatus: _StatusDefinition(
            "score-finished", fixed_fields=(("finished", True),),
            constructor_fields=(_TIMESTAMP_FIELD,)
        ),
        RuntimeFailureStatus: _StatusDefinition(
            "runtime-failure",
            WritePolicy.FIRST_WINS,
            constructor_fields=(
                _VALUE_FIELD("module"),
                _VALUE_FIELD("reason"),
                (
                    "diagnostic_paths",
                    _diagnostic_paths_document,
                    _parse_diagnostic_paths,
                ),
            ),
        ),
        RuntimeFrozenStatus: _StatusDefinition(
            "runtime-frozen", fixed_fields=(("frozen", True),)
        ),
        ArtifactsFinalStatus: _StatusDefinition(
            "artifacts-final",
            constructor_fields=(("records", _records_document, _parse_records),),
            derived_fields=(("complete", lambda status: status.complete),),
        ),
        TerminalNotifiedStatus: _StatusDefinition(
            "terminal-notified", fixed_fields=(("notified", True),)
        ),
    }
)
del _READY_FIELDS, _VALUE_FIELD
for _status_type, _status_definition in _STATUS_REGISTRY.items():
    _status_type.name = _status_definition.name
del _status_type, _status_definition


def _definition(status_type: type[RuntimeStatus]) -> _StatusDefinition:
    try:
        return _STATUS_REGISTRY[status_type]
    except (KeyError, TypeError) as error:
        raise RuntimeStatusError("runtime status type is not registered") from error


def status_name(status_type: type[StatusT]) -> str:
    return _definition(status_type).name


def status_write_policy(status_type: type[RuntimeStatus]) -> WritePolicy:
    return _definition(status_type).write_policy


def status_document(status: RuntimeStatus) -> dict[str, Any]:
    definition = _definition(type(status))
    status.__post_init__()
    document = {"run_id": canonical_run_id(status.run_id)}
    document.update(definition.fixed_fields)
    document.update(
        (name, encode(getattr(status, name)))
        for name, encode, _decode in definition.constructor_fields
    )
    document.update((name, derive(status)) for name, derive in definition.derived_fields)
    return document


def _wire_value_matches(value: object, expected: object) -> bool:
    return value is expected if type(expected) is bool else value == expected


def parse_status(
    status_type: type[StatusT],
    document: object,
    *,
    expected_run_id: str,
) -> StatusT:
    definition = _definition(status_type)
    run_id = canonical_run_id(expected_run_id)
    expected_keys = {"run_id"}
    expected_keys.update(name for name, _value in definition.fixed_fields)
    expected_keys.update(name for name, _encode, _decode in definition.constructor_fields)
    expected_keys.update(name for name, _derive in definition.derived_fields)
    if type(document) is not dict or set(document) != expected_keys:
        raise RuntimeStatusError("runtime status has an invalid schema")
    document_run_id = canonical_run_id(document["run_id"])
    if document_run_id != run_id:
        raise RuntimeStatusError("runtime status has the wrong run_id")
    if any(
        not _wire_value_matches(document[name], expected)
        for name, expected in definition.fixed_fields
    ):
        raise RuntimeStatusError("runtime status has an invalid schema")
    try:
        constructor_values = (
            decode(document[name])
            for name, _encode, decode in definition.constructor_fields
        )
        value = status_type(run_id, *constructor_values)
    except RuntimeStatusError:
        raise
    except (TypeError, ValueError) as error:
        raise RuntimeStatusError("runtime status has invalid values") from error
    if any(
        not _wire_value_matches(document[name], derive(value))
        for name, derive in definition.derived_fields
    ):
        raise RuntimeStatusError("runtime status has an invalid derived field")
    return value
