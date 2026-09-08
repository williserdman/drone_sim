"""Typed runtime-status values and their strict JSON wire contract."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math
import re
from types import MappingProxyType
from typing import Any, ClassVar, TypeVar
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


@dataclass(frozen=True)
class _StatusDefinition:
    name: str
    write_policy: WritePolicy


_STATUS_REGISTRY_BUILD: dict[type[RuntimeStatus], _StatusDefinition] = {}
StatusT = TypeVar("StatusT", bound=RuntimeStatus)


def _register_status(
    name: str,
    write_policy: WritePolicy = WritePolicy.IDENTICAL,
):
    def register(status_type: type[StatusT]) -> type[StatusT]:
        status_type.name = name
        _STATUS_REGISTRY_BUILD[status_type] = _StatusDefinition(name, write_policy)
        return status_type

    return register


@_register_status("artifacts-ready")
@dataclass(frozen=True)
class ArtifactsReadyStatus(RuntimeStatus):
    pass


@_register_status("gazebo-ready")
@dataclass(frozen=True)
class GazeboReadyStatus(RuntimeStatus):
    flight_exchange: FlightExchange

    def __post_init__(self) -> None:
        super().__post_init__()
        if type(self.flight_exchange) is not FlightExchange:
            raise RuntimeStatusError("flight_exchange must be a FlightExchange")


@_register_status("ardupilot-ready")
@dataclass(frozen=True)
class ArduPilotReadyStatus(RuntimeStatus):
    pass


@_register_status("companion-ready")
@dataclass(frozen=True)
class CompanionReadyStatus(RuntimeStatus):
    pass


@_register_status("mission-ready")
@dataclass(frozen=True)
class MissionReadyStatus(RuntimeStatus):
    pass


@_register_status("mission-command-delivered")
@dataclass(frozen=True)
class MissionCommandDeliveredStatus(RuntimeStatus):
    sim_timestamp_ns: int

    def __post_init__(self) -> None:
        super().__post_init__()
        _require_timestamp(self.sim_timestamp_ns)
        if self.sim_timestamp_ns > 50_000_000:
            raise RuntimeStatusError("mission command timestamp exceeds startup window")


@_register_status("runtime-running")
@dataclass(frozen=True)
class RuntimeRunningStatus(RuntimeStatus):
    sim_timestamp_ns: int

    def __post_init__(self) -> None:
        super().__post_init__()
        _require_timestamp(self.sim_timestamp_ns)


@_register_status("source-finished")
@dataclass(frozen=True)
class SourceFinishedStatus(RuntimeStatus):
    sim_timestamp_ns: int

    def __post_init__(self) -> None:
        super().__post_init__()
        _require_timestamp(self.sim_timestamp_ns)


@_register_status("mission-finished")
@dataclass(frozen=True)
class MissionFinishedStatus(RuntimeStatus):
    sim_timestamp_ns: int

    def __post_init__(self) -> None:
        super().__post_init__()
        _require_timestamp(self.sim_timestamp_ns)


@_register_status("score-finished")
@dataclass(frozen=True)
class ScoreFinishedStatus(RuntimeStatus):
    sim_timestamp_ns: int

    def __post_init__(self) -> None:
        super().__post_init__()
        _require_timestamp(self.sim_timestamp_ns)


@_register_status("runtime-failure", WritePolicy.FIRST_WINS)
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


@_register_status("runtime-frozen")
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


@_register_status("artifacts-final")
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


@_register_status("terminal-notified")
@dataclass(frozen=True)
class TerminalNotifiedStatus(RuntimeStatus):
    pass


_STATUS_REGISTRY: Mapping[type[RuntimeStatus], _StatusDefinition] = MappingProxyType(
    _STATUS_REGISTRY_BUILD
)
del _STATUS_REGISTRY_BUILD
del _register_status

_FLIGHT_EXCHANGE_KEYS = frozenset(
    (
        "online",
        "servo_packets_received",
        "motor_updates",
        "duplicate_servo_packets",
        "servo_frame_gaps",
        "json_states_sent",
        "json_send_errors",
        "last_servo_frame",
        "last_json_sim_time_ns",
    )
)


def _definition(status_type: type[RuntimeStatus]) -> _StatusDefinition:
    try:
        return _STATUS_REGISTRY[status_type]
    except (KeyError, TypeError) as error:
        raise RuntimeStatusError("runtime status type is not registered") from error


def status_name(status_type: type[StatusT]) -> str:
    return _definition(status_type).name


def status_write_policy(status_type: type[RuntimeStatus]) -> WritePolicy:
    return _definition(status_type).write_policy


def _flight_document(value: FlightExchange) -> dict[str, Any]:
    value.__post_init__()
    return {
        "online": value.online,
        "servo_packets_received": value.servo_packets_received,
        "motor_updates": value.motor_updates,
        "duplicate_servo_packets": value.duplicate_servo_packets,
        "servo_frame_gaps": value.servo_frame_gaps,
        "json_states_sent": value.json_states_sent,
        "json_send_errors": value.json_send_errors,
        "last_servo_frame": value.last_servo_frame,
        "last_json_sim_time_ns": value.last_json_sim_time_ns,
    }


def _record_document(record: ArtifactFinalRecord) -> dict[str, Any]:
    record.__post_init__()
    return {
        "relative_path": record.relative_path,
        "status": record.status.value,
        "detail": record.detail,
        "size_bytes": record.size_bytes,
        "sha256": record.sha256,
        "semantic": _copy_semantic_mapping(record.semantic),
    }


def status_document(status: RuntimeStatus) -> dict[str, Any]:
    _definition(type(status))
    status.__post_init__()
    run_id = canonical_run_id(status.run_id)
    if type(status) is ArtifactsReadyStatus:
        return {"run_id": run_id, "ready": True}
    if type(status) is GazeboReadyStatus:
        return {
            "run_id": run_id,
            "ready": True,
            "flight_exchange": _flight_document(status.flight_exchange),
        }
    if type(status) is ArduPilotReadyStatus:
        return {
            "run_id": run_id,
            "ready": True,
            "json_exchange": True,
            "mavlink_endpoint": "tcp://ardupilot-sitl:5760",
        }
    if type(status) is CompanionReadyStatus:
        return {
            "run_id": run_id,
            "ready": True,
            "mavlink_endpoint": "tcp://ardupilot-sitl:5760",
            "mavlink_transport_connected": True,
        }
    if type(status) is MissionReadyStatus:
        return {
            "run_id": run_id,
            "ready": True,
            "heartbeat_observed": True,
            "prearm_checks_healthy": True,
        }
    if type(status) is MissionCommandDeliveredStatus:
        return {
            "run_id": run_id,
            "command": "SET_GUIDED",
            "sim_timestamp_ns": status.sim_timestamp_ns,
            "delivered": True,
        }
    if type(status) is RuntimeRunningStatus:
        return {"run_id": run_id, "state": "RUNNING", "sim_timestamp_ns": status.sim_timestamp_ns}
    if type(status) is SourceFinishedStatus:
        return {"run_id": run_id, "finished": True, "sim_timestamp_ns": status.sim_timestamp_ns}
    if type(status) is MissionFinishedStatus:
        return {
            "run_id": run_id,
            "finished": True,
            "sim_timestamp_ns": status.sim_timestamp_ns,
            "outcome": "LANDED",
        }
    if type(status) is ScoreFinishedStatus:
        return {"run_id": run_id, "finished": True, "sim_timestamp_ns": status.sim_timestamp_ns}
    if type(status) is RuntimeFailureStatus:
        return {
            "run_id": run_id,
            "module": status.module,
            "reason": status.reason,
            "diagnostic_paths": list(status.diagnostic_paths),
        }
    if type(status) is RuntimeFrozenStatus:
        return {"run_id": run_id, "frozen": True}
    if type(status) is ArtifactsFinalStatus:
        return {
            "run_id": run_id,
            "complete": status.complete,
            "records": [_record_document(record) for record in status.records],
        }
    if type(status) is TerminalNotifiedStatus:
        return {"run_id": run_id, "notified": True}
    raise RuntimeStatusError("runtime status type is not registered")


def parse_status(
    status_type: type[StatusT],
    document: object,
    *,
    expected_run_id: str,
) -> StatusT:
    _definition(status_type)
    run_id = canonical_run_id(expected_run_id)
    if status_type is ArtifactsReadyStatus:
        expected_keys = {"run_id", "ready"}
    elif status_type is GazeboReadyStatus:
        expected_keys = {"run_id", "ready", "flight_exchange"}
    elif status_type is ArduPilotReadyStatus:
        expected_keys = {"run_id", "ready", "json_exchange", "mavlink_endpoint"}
    elif status_type is CompanionReadyStatus:
        expected_keys = {
            "run_id",
            "ready",
            "mavlink_endpoint",
            "mavlink_transport_connected",
        }
    elif status_type is MissionReadyStatus:
        expected_keys = {
            "run_id",
            "ready",
            "heartbeat_observed",
            "prearm_checks_healthy",
        }
    elif status_type is MissionCommandDeliveredStatus:
        expected_keys = {"run_id", "command", "sim_timestamp_ns", "delivered"}
    elif status_type is RuntimeRunningStatus:
        expected_keys = {"run_id", "state", "sim_timestamp_ns"}
    elif status_type in {SourceFinishedStatus, ScoreFinishedStatus}:
        expected_keys = {"run_id", "finished", "sim_timestamp_ns"}
    elif status_type is MissionFinishedStatus:
        expected_keys = {"run_id", "finished", "sim_timestamp_ns", "outcome"}
    elif status_type is RuntimeFailureStatus:
        expected_keys = {"run_id", "module", "reason", "diagnostic_paths"}
    elif status_type is RuntimeFrozenStatus:
        expected_keys = {"run_id", "frozen"}
    elif status_type is ArtifactsFinalStatus:
        expected_keys = {"run_id", "complete", "records"}
    else:
        expected_keys = {"run_id", "notified"}
    if type(document) is not dict or set(document) != expected_keys:
        raise RuntimeStatusError("runtime status has an invalid schema")
    document_run_id = canonical_run_id(document["run_id"])
    if document_run_id != run_id:
        raise RuntimeStatusError("runtime status has the wrong run_id")
    constants: tuple[tuple[str, Any], ...] = ()
    if status_type in {
        ArtifactsReadyStatus,
        GazeboReadyStatus,
        ArduPilotReadyStatus,
        CompanionReadyStatus,
        MissionReadyStatus,
    }:
        constants += (("ready", True),)
    if status_type is ArduPilotReadyStatus:
        constants += (
            ("json_exchange", True),
            ("mavlink_endpoint", "tcp://ardupilot-sitl:5760"),
        )
    elif status_type is CompanionReadyStatus:
        constants += (
            ("mavlink_endpoint", "tcp://ardupilot-sitl:5760"),
            ("mavlink_transport_connected", True),
        )
    elif status_type is MissionReadyStatus:
        constants += (("heartbeat_observed", True), ("prearm_checks_healthy", True))
    elif status_type is MissionCommandDeliveredStatus:
        constants = (("command", "SET_GUIDED"), ("delivered", True))
    elif status_type is RuntimeRunningStatus:
        constants = (("state", "RUNNING"),)
    elif status_type in {SourceFinishedStatus, ScoreFinishedStatus}:
        constants = (("finished", True),)
    elif status_type is MissionFinishedStatus:
        constants = (("finished", True), ("outcome", "LANDED"))
    elif status_type is RuntimeFrozenStatus:
        constants = (("frozen", True),)
    elif status_type is TerminalNotifiedStatus:
        constants = (("notified", True),)
    if any(
        document[key] is not expected
        if type(expected) is bool
        else document[key] != expected
        for key, expected in constants
    ):
        raise RuntimeStatusError("runtime status has an invalid schema")
    if status_type is GazeboReadyStatus:
        exchange = document["flight_exchange"]
        if type(exchange) is not dict or set(exchange) != _FLIGHT_EXCHANGE_KEYS:
            raise RuntimeStatusError("runtime status has an invalid schema")
    if status_type is GazeboReadyStatus:
        value: RuntimeStatus = GazeboReadyStatus(
            run_id, FlightExchange(**exchange)
        )
    elif status_type in {
        MissionCommandDeliveredStatus,
        RuntimeRunningStatus,
        SourceFinishedStatus,
        MissionFinishedStatus,
        ScoreFinishedStatus,
    }:
        value = status_type(run_id, document["sim_timestamp_ns"])
    elif status_type is RuntimeFailureStatus:
        if type(document["diagnostic_paths"]) is not list:
            raise RuntimeStatusError("diagnostic_paths must be a JSON list")
        value = RuntimeFailureStatus(
            run_id,
            document["module"],
            document["reason"],
            tuple(document["diagnostic_paths"]),
        )
    elif status_type is ArtifactsFinalStatus:
        raw_records = document["records"]
        if type(raw_records) is not list or len(raw_records) != 3:
            raise RuntimeStatusError("artifacts-final records must be a three-item list")
        record_keys = {
            "relative_path",
            "status",
            "detail",
            "size_bytes",
            "sha256",
            "semantic",
        }
        if any(
            type(record) is not dict
            or set(record) != record_keys
            or type(record["status"]) is not str
            or type(record["semantic"]) is not dict
            or not record["semantic"]
            for record in raw_records
        ):
            raise RuntimeStatusError("artifact record has an invalid schema")
        try:
            records = tuple(
                ArtifactFinalRecord(
                    record["relative_path"],
                    ValidationStatus(record["status"]),
                    record["detail"],
                    record["size_bytes"],
                    record["sha256"],
                    record["semantic"],
                )
                for record in raw_records
            )
        except ValueError as error:
            if isinstance(error, RuntimeStatusError):
                raise
            raise RuntimeStatusError("artifact record status is invalid") from error
        value = ArtifactsFinalStatus(run_id, records)
        if type(document["complete"]) is not bool or document["complete"] != value.complete:
            raise RuntimeStatusError("artifacts-final complete disagrees with records")
    else:
        value = status_type(run_id)
    return value  # type: ignore[return-value]
