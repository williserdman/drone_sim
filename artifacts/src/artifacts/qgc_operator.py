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
    "MavlinkIdentity",
    "QGC_ABORT_COMMAND",
    "QGC_ACTIONS_BUNDLE_PATH",
    "QGC_ACTIONS_KIND",
    "QGC_COMPANION_IDENTITY",
    "QGC_FLIGHT_CONTROLLER_IDENTITY",
    "QGC_START_COMMAND",
    "QgcOperatorDescriptor",
    "parse_qgc_operator_descriptor",
    "qgc_operator_descriptor_document",
]
