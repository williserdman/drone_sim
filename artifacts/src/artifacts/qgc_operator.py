"""Frozen QGroundControl action identity stored with one resolved run."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import PurePosixPath
import re
from typing import Any, Mapping
from uuid import UUID


QGC_ACTIONS_KIND = "qgc_actions_v1"
QGC_START_COMMAND = 31020
QGC_ABORT_COMMAND = 31021
MAX_ATTEMPT_TOKEN = 16_777_215
QGC_ACTIONS_BUNDLE_PATH = "configuration/qgc-actions.json"
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_TEMPORARILY_REJECTED = 1
MAV_RESULT_DENIED = 2
MAV_RESULT_UNSUPPORTED = 3
MAV_RESULT_FAILED = 4
MAV_RESULT_IN_PROGRESS = 5
MAV_RESULT_CANCELLED = 6
QGC_COMMAND_RESULTS = frozenset(range(7))
QGC_PROGRESS_UNKNOWN = 255
_NONFINITE_PARAMETERS = frozenset({"NaN", "Infinity", "-Infinity"})

_SHA256 = re.compile(r"[0-9a-f]{64}\Z")
_DESCRIPTOR_FIELDS = frozenset(
    {
        "kind",
        "start_command",
        "abort_command",
        "attempt_token",
        "start_request_id",
        "abort_request_id",
        "qgc_source",
        "companion",
        "flight_controller",
        "actions_path",
        "actions_sha256",
    }
)


def _exact_integer(value: object, field: str, minimum: int, maximum: int) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"{field} must be an integer from {minimum} through {maximum}")
    return value


def _canonical_uuid(value: object, field: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{field} must be a canonical UUID")
    try:
        parsed = UUID(value)
    except (ValueError, TypeError, AttributeError) as error:
        raise ValueError(f"{field} must be a canonical UUID") from error
    if str(parsed) != value:
        raise ValueError(f"{field} must be a canonical UUID")
    return value


def _relative_path(value: object) -> str:
    if not isinstance(value, str) or not value or "\\" in value:
        raise ValueError("actions_path must be a safe relative POSIX path")
    path = PurePosixPath(value)
    if path.is_absolute() or value != path.as_posix() or any(
        part in {"", ".", ".."} for part in path.parts
    ):
        raise ValueError("actions_path must be a safe relative POSIX path")
    return value


@dataclass(frozen=True)
class MavlinkIdentity:
    system_id: int
    component_id: int

    def __post_init__(self) -> None:
        _exact_integer(self.system_id, "system_id", 1, 255)
        _exact_integer(self.component_id, "component_id", 1, 255)


QGC_COMPANION_IDENTITY = MavlinkIdentity(system_id=1, component_id=191)
QGC_FLIGHT_CONTROLLER_IDENTITY = MavlinkIdentity(system_id=1, component_id=1)


def _wire_parameter(value: object) -> int | float | str:
    """Keep malformed floating-point input as strict, lossless JSON evidence."""
    if isinstance(value, str) and value in _NONFINITE_PARAMETERS:
        return value
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("command parameters must be numbers")
    if isinstance(value, float):
        if value != value:
            return "NaN"
        if value == float("inf"):
            return "Infinity"
        if value == float("-inf"):
            return "-Infinity"
    return value


@dataclass(frozen=True)
class QgcCommandEnvelope:
    """One COMMAND_LONG delivery after transport metadata has been captured."""

    wire_version: int
    source: MavlinkIdentity
    target: MavlinkIdentity
    command: int
    parameters: tuple[int | float | str, ...]
    received_wall_time_ns: int

    def __post_init__(self) -> None:
        if type(self.wire_version) is not int:
            raise ValueError("wire_version must be an integer")
        if not isinstance(self.source, MavlinkIdentity) or not isinstance(
            self.target, MavlinkIdentity
        ):
            raise ValueError("command identities must use MavlinkIdentity")
        if type(self.command) is not int:
            raise ValueError("command must be an integer")
        if type(self.parameters) is not tuple or len(self.parameters) != 7:
            raise ValueError("parameters must contain exactly seven values")
        object.__setattr__(
            self,
            "parameters",
            tuple(_wire_parameter(value) for value in self.parameters),
        )
        if type(self.received_wall_time_ns) is not int or self.received_wall_time_ns < 0:
            raise ValueError("received_wall_time_ns must be a nonnegative integer")


@dataclass(frozen=True)
class QgcCommandResponse:
    """A targeted COMMAND_ACK and its matching readable STATUSTEXT."""

    command: int
    recipient: MavlinkIdentity
    result: int
    progress: int
    status_text: str

    def __post_init__(self) -> None:
        if type(self.command) is not int:
            raise ValueError("command must be an integer")
        if not isinstance(self.recipient, MavlinkIdentity):
            raise ValueError("recipient must use MavlinkIdentity")
        if type(self.result) is not int or self.result not in QGC_COMMAND_RESULTS:
            raise ValueError("result must be a supported MAV_RESULT")
        if type(self.progress) is not int or not (
            0 <= self.progress <= 100 or self.progress == QGC_PROGRESS_UNKNOWN
        ):
            raise ValueError("progress must be 0 through 100 or 255")
        if (
            not isinstance(self.status_text, str)
            or not self.status_text
            or len(self.status_text.encode("utf-8")) > 50
        ):
            raise ValueError("status_text must be nonempty and at most 50 UTF-8 bytes")


@dataclass(frozen=True)
class QgcCommandStatus:
    """Immutable, ordered evidence for one command response or later outcome."""

    run_id: str
    config_sha256: str
    sequence: int
    envelope: QgcCommandEnvelope
    request_id: str | None
    request_sha256: str | None
    admission_sha256: str | None
    responded_wall_time_ns: int
    response: QgcCommandResponse
    observed_outcome: str | None = None
    schema_version: int = 1

    def __post_init__(self) -> None:
        _canonical_uuid(self.run_id, "run_id")
        if not isinstance(self.config_sha256, str) or _SHA256.fullmatch(
            self.config_sha256
        ) is None:
            raise ValueError("config_sha256 must be a lowercase SHA-256 digest")
        if type(self.sequence) is not int or not 0 <= self.sequence <= 9_999:
            raise ValueError("sequence must be an integer from 0 through 9999")
        if not isinstance(self.envelope, QgcCommandEnvelope):
            raise ValueError("envelope must be a QgcCommandEnvelope")
        if self.request_id is not None:
            _canonical_uuid(self.request_id, "request_id")
        for field, value in (
            ("request_sha256", self.request_sha256),
            ("admission_sha256", self.admission_sha256),
        ):
            if value is not None and (
                not isinstance(value, str) or _SHA256.fullmatch(value) is None
            ):
                raise ValueError(f"{field} must be null or a lowercase SHA-256 digest")
        if type(self.responded_wall_time_ns) is not int or self.responded_wall_time_ns < 0:
            raise ValueError("responded_wall_time_ns must be a nonnegative integer")
        if self.responded_wall_time_ns < self.envelope.received_wall_time_ns:
            raise ValueError("response cannot precede command receipt")
        if not isinstance(self.response, QgcCommandResponse):
            raise ValueError("response must be a QgcCommandResponse")
        if self.response.command != self.envelope.command:
            raise ValueError("response command must match its envelope")
        if self.response.recipient != self.envelope.source:
            raise ValueError("response recipient must match the command source")
        if self.request_id is None and (
            self.request_sha256 is not None or self.admission_sha256 is not None
        ):
            raise ValueError("command digests require a request identity")
        if self.request_id is not None and self.request_sha256 is None:
            raise ValueError("request identity requires its digest")
        if self.observed_outcome is not None and (
            not isinstance(self.observed_outcome, str)
            or not self.observed_outcome
            or len(self.observed_outcome.encode("utf-8")) > 1024
        ):
            raise ValueError("observed_outcome must be null or bounded nonempty text")
        if self.schema_version != 1:
            raise ValueError("schema_version must be 1")


@dataclass(frozen=True)
class QgcOperatorDescriptor:
    kind: str
    start_command: int
    abort_command: int
    attempt_token: int
    start_request_id: str
    abort_request_id: str
    qgc_source: MavlinkIdentity
    companion: MavlinkIdentity
    flight_controller: MavlinkIdentity
    actions_path: str
    actions_sha256: str

    def __post_init__(self) -> None:
        if self.kind != QGC_ACTIONS_KIND:
            raise ValueError(f"kind must be {QGC_ACTIONS_KIND}")
        if type(self.start_command) is not int or self.start_command != QGC_START_COMMAND:
            raise ValueError(f"start_command must be {QGC_START_COMMAND}")
        if type(self.abort_command) is not int or self.abort_command != QGC_ABORT_COMMAND:
            raise ValueError(f"abort_command must be {QGC_ABORT_COMMAND}")
        _exact_integer(self.attempt_token, "attempt_token", 1, MAX_ATTEMPT_TOKEN)
        _canonical_uuid(self.start_request_id, "start_request_id")
        _canonical_uuid(self.abort_request_id, "abort_request_id")
        if self.start_request_id == self.abort_request_id:
            raise ValueError("start and abort request IDs must be distinct")
        identities = (self.qgc_source, self.companion, self.flight_controller)
        if any(not isinstance(identity, MavlinkIdentity) for identity in identities):
            raise ValueError("MAVLink identities must use MavlinkIdentity")
        if len(set(identities)) != len(identities):
            raise ValueError("QGC, companion, and flight-controller identities must be distinct")
        if self.companion != QGC_COMPANION_IDENTITY:
            raise ValueError("companion must use MAVLink identity 1/191")
        if self.flight_controller != QGC_FLIGHT_CONTROLLER_IDENTITY:
            raise ValueError("flight_controller must use MAVLink identity 1/1")
        _relative_path(self.actions_path)
        if not isinstance(self.actions_sha256, str) or _SHA256.fullmatch(
            self.actions_sha256
        ) is None:
            raise ValueError("actions_sha256 must be a lowercase SHA-256 digest")


def _parse_identity(value: object, field: str) -> MavlinkIdentity:
    if not isinstance(value, Mapping) or set(value) != {"system_id", "component_id"}:
        raise ValueError(f"{field} identity has missing or unknown keys")
    return MavlinkIdentity(value["system_id"], value["component_id"])


def parse_qgc_operator_descriptor(document: object) -> QgcOperatorDescriptor:
    """Parse the complete descriptor without accepting protocol extensions."""
    if not isinstance(document, Mapping) or set(document) != _DESCRIPTOR_FIELDS:
        raise ValueError("operator_control descriptor has missing or unknown keys")
    return QgcOperatorDescriptor(
        kind=document["kind"],
        start_command=document["start_command"],
        abort_command=document["abort_command"],
        attempt_token=document["attempt_token"],
        start_request_id=document["start_request_id"],
        abort_request_id=document["abort_request_id"],
        qgc_source=_parse_identity(document["qgc_source"], "qgc_source"),
        companion=_parse_identity(document["companion"], "companion"),
        flight_controller=_parse_identity(
            document["flight_controller"], "flight_controller"
        ),
        actions_path=document["actions_path"],
        actions_sha256=document["actions_sha256"],
    )


def qgc_operator_descriptor_document(
    descriptor: QgcOperatorDescriptor,
) -> dict[str, Any]:
    """Return the JSON form included in the resolved configuration checksum."""
    if not isinstance(descriptor, QgcOperatorDescriptor):
        raise TypeError("descriptor must be a QgcOperatorDescriptor")
    return asdict(descriptor)


__all__ = [
    "MAX_ATTEMPT_TOKEN",
    "MAV_RESULT_ACCEPTED",
    "MAV_RESULT_CANCELLED",
    "MAV_RESULT_DENIED",
    "MAV_RESULT_FAILED",
    "MAV_RESULT_IN_PROGRESS",
    "MAV_RESULT_TEMPORARILY_REJECTED",
    "MAV_RESULT_UNSUPPORTED",
    "MavlinkIdentity",
    "QGC_ABORT_COMMAND",
    "QGC_ACTIONS_BUNDLE_PATH",
    "QGC_ACTIONS_KIND",
    "QGC_COMPANION_IDENTITY",
    "QGC_FLIGHT_CONTROLLER_IDENTITY",
    "QGC_START_COMMAND",
    "QgcCommandEnvelope",
    "QgcCommandResponse",
    "QgcCommandStatus",
    "QgcOperatorDescriptor",
    "parse_qgc_operator_descriptor",
    "qgc_operator_descriptor_document",
]
