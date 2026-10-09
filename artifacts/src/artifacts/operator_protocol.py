"""Durable controls and observations for one operator-owned simulation run."""

from __future__ import annotations

from dataclasses import asdict, dataclass, fields
import hashlib
import os
from pathlib import Path
import re
from typing import Any, Literal, Mapping, TypeVar
from uuid import UUID

from .protocol_files import (
    ProtocolFileReplaced,
    ProtocolIOError,
    WritePolicy,
    canonical_json,
    open_directory,
    read_json_object_at,
    write_json_object_at,
)


_SHA256 = re.compile(r"[0-9a-f]{64}\Z")
_ABORT_STATUS_NAME = re.compile(r"flight-abort-(\d{4})\.json\Z")
_ABORT_STATES = frozenset(
    {"accepted", "recovering", "landed_disarmed", "rejected", "failed", "interrupted"}
)
_COMPONENTS = frozenset({"gazebo", "companion"})
_FRAME_INTERVAL_NS = 50_000_000


class OperatorProtocolError(RuntimeError):
    """An operator protocol value or path violates the run contract."""


class OperatorProtocolConflict(OperatorProtocolError):
    """An immutable decision already exists with different content."""


def _canonical_uuid(value: object, field: str) -> str:
    if not isinstance(value, str):
        raise OperatorProtocolError(f"{field} must be a canonical UUID")
    try:
        parsed = UUID(value)
    except (ValueError, TypeError, AttributeError) as error:
        raise OperatorProtocolError(f"{field} must be a canonical UUID") from error
    if str(parsed) != value:
        raise OperatorProtocolError(f"{field} must be a canonical UUID")
    return value


def _digest(value: Mapping[str, Any]) -> str:
    try:
        return hashlib.sha256(canonical_json(value)).hexdigest()
    except ProtocolIOError as error:
        raise OperatorProtocolError(str(error)) from error


def _sha256(value: object, field: str) -> str:
    if not isinstance(value, str) or _SHA256.fullmatch(value) is None:
        raise OperatorProtocolError(f"{field} must be a lowercase SHA-256")
    return value


def _nonnegative_integer(value: object, field: str) -> int:
    if type(value) is not int or value < 0:
        raise OperatorProtocolError(f"{field} must be a nonnegative integer")
    return value


def _optional_nonnegative_integer(value: object, field: str) -> int | None:
    if value is None:
        return None
    return _nonnegative_integer(value, field)


def _nonempty_text(value: object, field: str) -> str:
    if not isinstance(value, str) or not value or len(value.encode("utf-8")) > 1024:
        raise OperatorProtocolError(f"{field} must be nonempty and at most 1024 bytes")
    return value


class _Document:
    @property
    def sha256(self) -> str:
        return _digest(_document(self))


@dataclass(frozen=True)
class MissionStartRequest(_Document):
    run_id: str
    config_sha256: str
    request_id: str
    schema_version: int = 1

    def __post_init__(self) -> None:
        _identity(self)
        _canonical_uuid(self.request_id, "request_id")
        if self.schema_version != 1:
            raise OperatorProtocolError("schema_version must be 1")


@dataclass(frozen=True)
class MissionStartReadiness(_Document):
    run_id: str
    config_sha256: str
    request_id: str
    request_sha256: str
    observed_wall_time_ns: int
    heartbeat_live: bool
    prearm_checks_healthy: bool
    armed: bool | None
    ready: bool
    detail: str
    schema_version: int = 1

    def __post_init__(self) -> None:
        _identity(self)
        _canonical_uuid(self.request_id, "request_id")
        _sha256(self.request_sha256, "request_sha256")
        _nonnegative_integer(self.observed_wall_time_ns, "observed_wall_time_ns")
        if type(self.heartbeat_live) is not bool or type(self.prearm_checks_healthy) is not bool:
            raise OperatorProtocolError("readiness health facts must be booleans")
        if self.armed is not None and type(self.armed) is not bool:
            raise OperatorProtocolError("armed must be boolean or null")
        if type(self.ready) is not bool:
            raise OperatorProtocolError("ready must be boolean")
        _nonempty_text(self.detail, "detail")
        if self.ready and not (
            self.heartbeat_live
            and self.prearm_checks_healthy
            and self.armed is False
        ):
            raise OperatorProtocolError("ready start decision requires a healthy disarmed vehicle")
        if self.schema_version != 1:
            raise OperatorProtocolError("schema_version must be 1")


@dataclass(frozen=True)
class PreflightStatus(_Document):
    run_id: str
    config_sha256: str
    component: Literal["gazebo", "companion"]
    sequence: int
    observed_wall_time_ns: int
    native_timestamp_ns: int | None = None
    physics_advancing: bool | None = None
    camera_native_timestamp_ns: int | None = None
    heartbeat_live: bool | None = None
    prearm_checks_healthy: bool | None = None
    armed: bool | None = None
    ready: bool = False
    detail: str = "pending"
    schema_version: int = 1

    def __post_init__(self) -> None:
        _identity(self)
        if self.component not in _COMPONENTS:
            raise OperatorProtocolError("preflight component must be gazebo or companion")
        _nonnegative_integer(self.sequence, "sequence")
        _nonnegative_integer(self.observed_wall_time_ns, "observed_wall_time_ns")
        _optional_nonnegative_integer(self.native_timestamp_ns, "native_timestamp_ns")
        _optional_nonnegative_integer(
            self.camera_native_timestamp_ns, "camera_native_timestamp_ns"
        )
        for name in (
            "physics_advancing",
            "heartbeat_live",
            "prearm_checks_healthy",
            "armed",
        ):
            value = getattr(self, name)
            if value is not None and type(value) is not bool:
                raise OperatorProtocolError(f"{name} must be boolean or null")
        if type(self.ready) is not bool:
            raise OperatorProtocolError("ready must be boolean")
        _nonempty_text(self.detail, "detail")
        if self.component == "gazebo" and self.ready and not (
            self.physics_advancing is True
            and self.native_timestamp_ns is not None
            and self.camera_native_timestamp_ns is not None
        ):
            raise OperatorProtocolError("ready Gazebo preflight requires advancing physics and camera")
        if self.component == "companion" and self.ready and not (
            self.heartbeat_live is True
            and self.prearm_checks_healthy is True
            and self.armed is False
        ):
            raise OperatorProtocolError("ready companion preflight requires a healthy disarmed vehicle")
        if self.schema_version != 1:
            raise OperatorProtocolError("schema_version must be 1")


@dataclass(frozen=True)
class StartAdmission(_Document):
    run_id: str
    config_sha256: str
    request_id: str
    request_sha256: str
    native_epoch_ns: int
    public_epoch_ns: int = 0
    schema_version: int = 1

    def __post_init__(self) -> None:
        _identity(self)
        _canonical_uuid(self.request_id, "request_id")
        _sha256(self.request_sha256, "request_sha256")
        native = _nonnegative_integer(self.native_epoch_ns, "native_epoch_ns")
        if native <= 0 or native % _FRAME_INTERVAL_NS != 0:
            raise OperatorProtocolError("native_epoch_ns must be positive and lie on the 50 ms grid")
        if self.public_epoch_ns != 0:
            raise OperatorProtocolError("public_epoch_ns must be zero")
        if self.schema_version != 1:
            raise OperatorProtocolError("schema_version must be 1")


@dataclass(frozen=True)
class FlightAbortRequest(_Document):
    run_id: str
    config_sha256: str
    request_id: str
    admission_sha256: str
    reason: str
    schema_version: int = 1

    def __post_init__(self) -> None:
        _identity(self)
        _canonical_uuid(self.request_id, "request_id")
        _sha256(self.admission_sha256, "admission_sha256")
        _nonempty_text(self.reason, "reason")
        if self.schema_version != 1:
            raise OperatorProtocolError("schema_version must be 1")


@dataclass(frozen=True)
class FlightAbortStatus(_Document):
    run_id: str
    config_sha256: str
    request_id: str
    admission_sha256: str
    sequence: int
    state: str
    observed_wall_time_ns: int
    observed_native_ns: int | None
    detail: str
    schema_version: int = 1

    def __post_init__(self) -> None:
        _identity(self)
        _canonical_uuid(self.request_id, "request_id")
        _sha256(self.admission_sha256, "admission_sha256")
        _nonnegative_integer(self.sequence, "sequence")
        if self.state not in _ABORT_STATES:
            raise OperatorProtocolError("flight abort state is invalid")
        _nonnegative_integer(self.observed_wall_time_ns, "observed_wall_time_ns")
        _optional_nonnegative_integer(self.observed_native_ns, "observed_native_ns")
        _nonempty_text(self.detail, "detail")
        if self.schema_version != 1:
            raise OperatorProtocolError("schema_version must be 1")


DocumentT = TypeVar("DocumentT", bound=_Document)


def _identity(value: _Document) -> None:
    _canonical_uuid(getattr(value, "run_id"), "run_id")
    _sha256(getattr(value, "config_sha256"), "config_sha256")


def _document(value: _Document) -> dict[str, Any]:
    return asdict(value)


def _parse(document_type: type[DocumentT], document: object) -> DocumentT:
    expected = {field.name for field in fields(document_type)}
    if not isinstance(document, dict) or set(document) != expected:
        raise OperatorProtocolError(
            f"{document_type.__name__} has missing or unknown fields"
        )
    try:
        return document_type(**document)
    except TypeError as error:
        raise OperatorProtocolError(f"{document_type.__name__} has invalid fields") from error


class OperatorProtocol:
    """Descriptor-safe access to operator controls and live observations."""

    def __init__(
        self,
        run_directory: Path | str,
        run_id: str,
        config_sha256: str,
    ) -> None:
        self.run_directory = Path(run_directory)
        self.run_id = _canonical_uuid(run_id, "run_id")
        self.config_sha256 = _sha256(config_sha256, "config_sha256")
        try:
            self._run_fd = open_directory(self.run_directory)
        except ProtocolIOError as error:
            raise OperatorProtocolError(f"run directory is unsafe: {error}") from error
        self._observed_immutable: dict[str, bytes] = {}

    def close(self) -> None:
        descriptor = getattr(self, "_run_fd", None)
        if descriptor is not None:
            os.close(descriptor)
            self._run_fd = None

    def __enter__(self) -> "OperatorProtocol":
        return self

    def __exit__(self, *_args: Any) -> None:
        self.close()

    def _open_directory(self, name: str) -> int:
        try:
            return open_directory(name, dir_fd=self._run_fd)
        except ProtocolIOError as error:
            raise OperatorProtocolError(
                f"protocol directory {name!r} is unsafe: {error}"
            ) from error

    def _read(
        self,
        directory: str,
        name: str,
        document_type: type[DocumentT],
        *,
        immutable: bool,
    ) -> DocumentT | None:
        descriptor = self._open_directory(directory)
        try:
            attempts = 1 if immutable else 3
            for attempt in range(attempts):
                try:
                    document = read_json_object_at(descriptor, name)
                    break
                except ProtocolFileReplaced as error:
                    if getattr(error, "__notes__", None) or attempt + 1 == attempts:
                        raise OperatorProtocolError(str(error)) from error
                except ProtocolIOError as error:
                    raise OperatorProtocolError(str(error)) from error
        finally:
            os.close(descriptor)
        if document is None:
            return None
        value = _parse(document_type, document)
        self._assert_bound(value)
        if immutable:
            try:
                encoded = canonical_json(document)
            except ProtocolIOError as error:
                raise OperatorProtocolError(str(error)) from error
            key = f"{directory}/{name}"
            previous = self._observed_immutable.setdefault(key, encoded)
            if previous != encoded:
                raise OperatorProtocolConflict(f"immutable operator document {key!r} changed")
        return value

    def _write(
        self,
        directory: str,
        name: str,
        value: DocumentT,
        *,
        mode: int,
        policy: WritePolicy,
    ) -> tuple[DocumentT, bool]:
        self._assert_bound(value)
        descriptor = self._open_directory(directory)
        try:
            try:
                persisted, created = write_json_object_at(
                    descriptor,
                    name,
                    _document(value),
                    mode=mode,
                    policy=policy,
                )
            except ProtocolIOError as error:
                raise OperatorProtocolError(str(error)) from error
        finally:
            os.close(descriptor)
        parsed = _parse(type(value), persisted)
        self._assert_bound(parsed)
        return parsed, created

    def _assert_bound(self, value: _Document) -> None:
        if value.run_id != self.run_id:
            raise OperatorProtocolError("operator document has the wrong run_id")
        if value.config_sha256 != self.config_sha256:
            raise OperatorProtocolError("operator document has the wrong config_sha256")

    @staticmethod
    def _require_equal(name: str, requested: DocumentT, persisted: DocumentT) -> DocumentT:
        if persisted != requested:
            raise OperatorProtocolConflict(f"{name} conflicts with the persisted value")
        return persisted

    def write_start_request(self, request_id: str) -> MissionStartRequest:
        requested = MissionStartRequest(self.run_id, self.config_sha256, request_id)
        persisted, _created = self._write(
            ".control",
            "mission-start-request.json",
            requested,
            mode=0o600,
            policy=WritePolicy.FIRST_WINS,
        )
        return self._require_equal("mission start request", requested, persisted)

    def read_start_request(self) -> MissionStartRequest | None:
        return self._read(
            ".control",
            "mission-start-request.json",
            MissionStartRequest,
            immutable=True,
        )

    def write_start_readiness(
        self, value: MissionStartReadiness
    ) -> MissionStartReadiness:
        request = self.read_start_request()
        if request is None:
            raise OperatorProtocolError("mission start request is missing")
        if value.request_id != request.request_id or value.request_sha256 != request.sha256:
            raise OperatorProtocolError("mission start readiness does not match its request")
        persisted, _created = self._write(
            ".status",
            "mission-start-readiness.json",
            value,
            mode=0o644,
            policy=WritePolicy.FIRST_WINS,
        )
        return self._require_equal("mission start readiness", value, persisted)

    def read_start_readiness(self) -> MissionStartReadiness | None:
        value = self._read(
            ".status",
            "mission-start-readiness.json",
            MissionStartReadiness,
            immutable=True,
        )
        if value is None:
            return None
        request = self.read_start_request()
        if request is None or value.request_id != request.request_id or value.request_sha256 != request.sha256:
            raise OperatorProtocolError("mission start readiness does not match its request")
        return value

    def write_preflight(self, value: PreflightStatus) -> PreflightStatus:
        self._assert_bound(value)
        previous = self.read_preflight(value.component)
        if previous is not None:
            if value.sequence < previous.sequence:
                raise OperatorProtocolConflict("preflight sequence regressed")
            if value.sequence == previous.sequence:
                return self._require_equal("preflight status", value, previous)
        persisted, _created = self._write(
            ".status",
            f"preflight-{value.component}.json",
            value,
            mode=0o644,
            policy=WritePolicy.REPLACE,
        )
        return self._require_equal("preflight status", value, persisted)

    def read_preflight(
        self, component: Literal["gazebo", "companion"]
    ) -> PreflightStatus | None:
        if component not in _COMPONENTS:
            raise ValueError("preflight component must be gazebo or companion")
        value = self._read(
            ".status",
            f"preflight-{component}.json",
            PreflightStatus,
            immutable=False,
        )
        if value is not None and value.component != component:
            raise OperatorProtocolError("preflight component does not match its path")
        return value

    def write_start_admission(
        self,
        request: MissionStartRequest,
        *,
        native_epoch_ns: int,
        public_epoch_ns: int = 0,
    ) -> StartAdmission:
        self._assert_bound(request)
        persisted_request = self.read_start_request()
        if persisted_request != request:
            raise OperatorProtocolError("start admission does not match the persisted request")
        value = StartAdmission(
            self.run_id,
            self.config_sha256,
            request.request_id,
            request.sha256,
            native_epoch_ns,
            public_epoch_ns,
        )
        persisted, _created = self._write(
            "configuration",
            "start-admission.json",
            value,
            mode=0o644,
            policy=WritePolicy.FIRST_WINS,
        )
        return self._require_equal("start admission", value, persisted)

    def read_start_admission(self) -> StartAdmission | None:
        value = self._read(
            "configuration",
            "start-admission.json",
            StartAdmission,
            immutable=True,
        )
        if value is None:
            return None
        request = self.read_start_request()
        if request is None or value.request_id != request.request_id or value.request_sha256 != request.sha256:
            raise OperatorProtocolError("start admission does not match its request")
        return value

    def start_admission_configuration_record(self) -> tuple[str, str] | None:
        """Return the manifest path and digest for an admitted operator run."""
        admission = self.read_start_admission()
        if admission is None:
            return None
        return ("configuration/start-admission.json", admission.sha256)

    def write_flight_abort_request(
        self,
        request_id: str,
        admission_sha256: str,
        reason: str = "operator_abort",
    ) -> FlightAbortRequest:
        admission = self.read_start_admission()
        if admission is None or admission.sha256 != admission_sha256:
            raise OperatorProtocolError("flight abort request does not match the start admission")
        requested = FlightAbortRequest(
            self.run_id,
            self.config_sha256,
            request_id,
            admission_sha256,
            reason,
        )
        persisted, _created = self._write(
            ".control",
            "flight-abort-request.json",
            requested,
            mode=0o600,
            policy=WritePolicy.FIRST_WINS,
        )
        return self._require_equal("flight abort request", requested, persisted)

    def read_flight_abort_request(self) -> FlightAbortRequest | None:
        value = self._read(
            ".control",
            "flight-abort-request.json",
            FlightAbortRequest,
            immutable=True,
        )
        if value is None:
            return None
        admission = self.read_start_admission()
        if admission is None or value.admission_sha256 != admission.sha256:
            raise OperatorProtocolError("flight abort request does not match the start admission")
        return value

    def read_flight_abort_statuses(self) -> tuple[FlightAbortStatus, ...]:
        descriptor = self._open_directory(".status")
        try:
            try:
                names = sorted(
                    name
                    for name in os.listdir(descriptor)
                    if _ABORT_STATUS_NAME.fullmatch(name) is not None
                )
            except OSError as error:
                raise OperatorProtocolError(f"could not list flight abort statuses: {error}") from error
        finally:
            os.close(descriptor)
        values: list[FlightAbortStatus] = []
        for expected_sequence, name in enumerate(names):
            match = _ABORT_STATUS_NAME.fullmatch(name)
            assert match is not None
            if int(match.group(1)) != expected_sequence:
                raise OperatorProtocolConflict("flight abort status sequence is not contiguous")
            value = self._read(".status", name, FlightAbortStatus, immutable=True)
            assert value is not None
            if value.sequence != expected_sequence:
                raise OperatorProtocolError("flight abort status sequence does not match its path")
            values.append(value)
        request = self.read_flight_abort_request()
        if values and request is None:
            raise OperatorProtocolError("flight abort statuses require a request")
        if request is not None and any(
            value.request_id != request.request_id
            or value.admission_sha256 != request.admission_sha256
            for value in values
        ):
            raise OperatorProtocolError("flight abort status does not match its request")
        return tuple(values)

    def write_flight_abort_status(self, value: FlightAbortStatus) -> FlightAbortStatus:
        self._assert_bound(value)
        request = self.read_flight_abort_request()
        if request is None or (
            value.request_id != request.request_id
            or value.admission_sha256 != request.admission_sha256
        ):
            raise OperatorProtocolError("flight abort status does not match its request")
        existing = self.read_flight_abort_statuses()
        if value.sequence < len(existing):
            return self._require_equal("flight abort status", value, existing[value.sequence])
        if value.sequence != len(existing):
            raise OperatorProtocolConflict("flight abort status sequence must be contiguous")
        persisted, _created = self._write(
            ".status",
            f"flight-abort-{value.sequence:04d}.json",
            value,
            mode=0o644,
            policy=WritePolicy.IDENTICAL,
        )
        return self._require_equal("flight abort status", value, persisted)

    def read_latest_flight_abort_status(self) -> FlightAbortStatus | None:
        values = self.read_flight_abort_statuses()
        return values[-1] if values else None


def validate_admission_binding(
    protocol: OperatorProtocol | None,
    launch_mode: str,
    admission_sha256: str,
) -> StartAdmission | None:
    """Bind a RUNNING producer to the immutable admission advertised in RunState."""
    if launch_mode == "unattended":
        if admission_sha256 != "":
            raise OperatorProtocolError(
                "unattended RunState must not carry an admission digest"
            )
        return None
    if launch_mode != "operator":
        raise OperatorProtocolError("launch_mode must be unattended or operator")
    if protocol is None:
        raise OperatorProtocolError("operator launch requires an operator protocol")
    _sha256(admission_sha256, "admission_sha256")
    admission = protocol.read_start_admission()
    if admission is None:
        raise OperatorProtocolError("operator RunState has no start admission")
    if admission.sha256 != admission_sha256:
        raise OperatorProtocolError("RunState admission digest does not match")
    return admission


__all__ = [
    "FlightAbortRequest",
    "FlightAbortStatus",
    "MissionStartReadiness",
    "MissionStartRequest",
    "OperatorProtocol",
    "OperatorProtocolConflict",
    "OperatorProtocolError",
    "PreflightStatus",
    "StartAdmission",
    "validate_admission_binding",
]
