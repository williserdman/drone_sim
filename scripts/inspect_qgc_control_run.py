#!/usr/bin/env python3
"""Inspect one finalized run for a bounded native-QGC control expectation."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, replace
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import re
from typing import Any, Literal

from pymavlink import mavutil

from artifacts.acceptance import (
    BundleAcceptanceError,
    _validate_manifest_domain,
    _validate_manifest_inventory,
)
from artifacts.manifest import REQUIRED_ARTIFACT_PATHS
from artifacts.operator_protocol import (
    FlightAbortStatus,
    OperatorProtocol,
    OperatorProtocolError,
    StartAdmission,
)
from artifacts.qgc_operator import (
    MAV_RESULT_ACCEPTED,
    MAV_RESULT_IN_PROGRESS,
    MAV_RESULT_UNSUPPORTED,
    QGC_ACTIONS_BUNDLE_PATH,
    QGC_ABORT_COMMAND,
    QGC_START_COMMAND,
    QgcCommandStatus,
    QgcOperatorDescriptor,
)
from artifacts.runtime_protocol import ProtocolError, RuntimeProtocol
from artifacts.runtime_status import RuntimeRunningStatus
from artifacts.validation import (
    ValidationStatus,
    read_regular_file_bytes,
    validate_regular_file,
)
from orchestration.config import load_run_config


Expectation = Literal[
    "routing", "start", "abort", "ordinary-land", "precision-loss"
]
Status = Literal["passed", "failed", "not_required"]
ConnectionFactory = Callable[[str], Any]

EXPECTATIONS: tuple[Expectation, ...] = (
    "routing",
    "start",
    "abort",
    "ordinary-land",
    "precision-loss",
)
NATIVE_TLOG_PATH = "logs/docker/native-qgc.tlog"
INTERVENTION_PATH = "logs/docker/qgc-interventions.jsonl"
COMPANION_LOG_PATH = "logs/companion.jsonl"
STIMULUS_PATH = "logs/docker/precision-loss-stimulus.jsonl"
DATAFLASH_PATTERN = re.compile(r"ardupilot_sitl/logs/[^/]+\.BIN\Z")
QGC_RESPONSE_WINDOW_NS = 5_000_000_000
QGC_WALL_TOLERANCE_NS = 2_000_000
AIRBORNE_CLEARANCE_M = 0.75
TOUCHDOWN_CLEARANCE_M = 0.20
PROFILE_EVENT_NAME = "landing_profile_transition"
PRECISION_FAILURE = "precision target tracking was lost above final clearance"
STIMULUS_REQUEST_TEXT = (
    'name: "moving_pad" position { x: 55 y: 0 z: 0 } '
    "orientation { x: 0 y: 0 z: 0 w: 1 }"
)
STIMULUS_PERSISTENCE_NS = 500_000_000
STIMULUS_PRECISION_MAX_AGE_NS = 500_000_000
STIMULUS_LAND_MAX_AGE_NS = 5_000_000_000
PRECISION_PROFILE = {
    "LAND_SPEED": 50,
    "PLND_ENABLED": 1.0,
    "PLND_TYPE": 1.0,
    "PLND_EST_TYPE": 0.0,
    "PLND_LAG": 0.08,
    "PLND_XY_DIST_MAX": 0.50,
    "PLND_STRICT": 2.0,
    "PLND_RET_MAX": 1.0,
    "PLND_TIMEOUT": 0.50,
    "PLND_ALT_MIN": 0.75,
    "PLND_ALT_MAX": 8.0,
    "PLND_OPTIONS": 4.0,
}
QUIESCENCE_MODULES = (
    "orchestration",
    "companion",
    "ardupilot_sitl",
    "gazebo",
    "electromagnet",
    "scorekeeper",
)
REQUIRED_DIMENSIONS: Mapping[Expectation, frozenset[str]] = {
    "routing": frozenset({"command", "cleanup"}),
    "start": frozenset({"command", "physical", "cleanup"}),
    "abort": frozenset({"command", "physical", "cleanup"}),
    "ordinary-land": frozenset({"command", "physical", "profile", "cleanup"}),
    "precision-loss": frozenset({"command", "physical", "profile", "cleanup"}),
}


class InspectionError(RuntimeError):
    """Evidence is missing, malformed, ambiguous, stale, or mismatched."""


@dataclass(frozen=True)
class Finding:
    status: Status
    detail: str

    def to_dict(self) -> dict[str, str]:
        return {"status": self.status, "detail": self.detail}


@dataclass(frozen=True)
class TlogRow:
    message_type: str
    source_system: int
    source_component: int
    timestamp_ns: int
    mavlink2: bool
    fields: Mapping[str, object]


@dataclass(frozen=True)
class DataFlashRow:
    message_type: str
    time_us: int
    fields: Mapping[str, object]


@dataclass(frozen=True)
class ProfileTransition:
    parameter: str
    action: str
    stage: str
    value: float
    generation: int | None
    sim_timestamp: float
    detail: str | None


@dataclass(frozen=True)
class PrecisionOperation:
    operation_id: str
    start: float
    land: float
    guided: float
    failure: float


@dataclass(frozen=True)
class Evidence:
    directory: Path
    run_id: str
    config_sha256: str
    configuration: Any
    descriptor: QgcOperatorDescriptor
    manifest: Mapping[str, Any]
    manifest_artifacts: Mapping[str, Mapping[str, Any]]
    request: Any | None
    admission: StartAdmission | None
    abort_request: Any | None
    abort_statuses: tuple[FlightAbortStatus, ...]
    command_statuses: tuple[QgcCommandStatus, ...]
    companion_events: tuple[Mapping[str, Any], ...]


def _finite_number(value: object, label: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise InspectionError(f"{label} must be a finite number")
    return float(value)


def _integer(value: object, label: str, *, minimum: int = 0) -> int:
    if type(value) is not int or value < minimum:
        raise InspectionError(f"{label} must be an integer of at least {minimum}")
    return value


def _safe_relative_path(value: object, label: str) -> str:
    if not isinstance(value, str) or not value or "\\" in value:
        raise InspectionError(f"{label} is not a safe relative path")
    path = PurePosixPath(value)
    if path.is_absolute() or path.as_posix() != value or ".." in path.parts:
        raise InspectionError(f"{label} is not a safe relative path")
    return value


def _manifest_artifacts(manifest: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, list):
        raise InspectionError("manifest artifact inventory is invalid")
    result: dict[str, Mapping[str, Any]] = {}
    for row in artifacts:
        if not isinstance(row, Mapping):
            raise InspectionError("manifest artifact inventory is invalid")
        path = _safe_relative_path(row.get("relative_path"), "manifest artifact path")
        if path in result:
            raise InspectionError("manifest artifact inventory contains duplicate paths")
        result[path] = row
    return result


def _require_manifest_artifact(evidence: Evidence, relative_path: str) -> Mapping[str, Any]:
    row = evidence.manifest_artifacts.get(relative_path)
    if row is None:
        raise InspectionError(f"{relative_path} is absent from the manifest inventory")
    if row.get("validation") != ValidationStatus.VALID.value:
        raise InspectionError(f"{relative_path} is not a valid manifest artifact")
    return row


def _read_jsonl(
    evidence: Evidence,
    relative_path: str,
    *,
    max_bytes: int = 32 * 1024 * 1024,
) -> tuple[Mapping[str, Any], ...]:
    _require_manifest_artifact(evidence, relative_path)
    validation, payload = read_regular_file_bytes(
        evidence.directory, relative_path, max_bytes=max_bytes
    )
    if validation.status is not ValidationStatus.VALID or payload is None:
        raise InspectionError(f"{relative_path} is missing, unsafe, or unreadable")
    try:
        text = payload.decode("utf-8")
        lines = text.splitlines()
        if not lines:
            raise InspectionError(f"{relative_path} is empty")
        rows = tuple(json.loads(line) for line in lines)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise InspectionError(f"{relative_path} is not valid UTF-8 JSONL") from error
    if any(not isinstance(row, Mapping) for row in rows):
        raise InspectionError(f"{relative_path} contains a non-object row")
    return rows  # type: ignore[return-value]


def _read_manifest(directory: Path) -> Mapping[str, Any]:
    validation, payload = read_regular_file_bytes(
        directory, "manifest.json", max_bytes=16 * 1024 * 1024
    )
    if validation.status is not ValidationStatus.VALID or payload is None:
        raise InspectionError("manifest.json is missing, unsafe, or unreadable")
    try:
        document = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise InspectionError("manifest.json is invalid JSON") from error
    if not isinstance(document, dict):
        raise InspectionError("manifest.json must contain an object")
    try:
        _validate_manifest_domain(document)
        _validate_manifest_inventory(directory, document)
    except (BundleAcceptanceError, ValueError) as error:
        raise InspectionError(f"manifest validation failed: {error}") from error
    return document


def _validate_configuration_records(
    manifest: Mapping[str, Any],
    descriptor: QgcOperatorDescriptor,
    config_sha256: str,
    admission: StartAdmission | None,
) -> None:
    rows = manifest.get("configurations")
    if not isinstance(rows, list):
        raise InspectionError("manifest configuration inventory is invalid")
    expected = {
        "configuration/run.json": config_sha256,
        QGC_ACTIONS_BUNDLE_PATH: descriptor.actions_sha256,
    }
    if admission is not None:
        expected["configuration/start-admission.json"] = admission.sha256
    actual: dict[str, str] = {}
    for row in rows:
        if not isinstance(row, Mapping):
            raise InspectionError("manifest configuration inventory is invalid")
        path = row.get("relative_path")
        digest = row.get("sha256")
        if isinstance(path, str) and isinstance(digest, str):
            actual[path] = digest
    for path, digest in expected.items():
        if actual.get(path) != digest:
            raise InspectionError(f"manifest configuration binding is wrong for {path}")
    if admission is None and "configuration/start-admission.json" in actual:
        raise InspectionError("manifest inventories a start admission that does not exist")


def _validate_command_status_inventory(
    artifacts: Mapping[str, Mapping[str, Any]],
    statuses: Sequence[QgcCommandStatus],
) -> None:
    expected = {
        f".status/qgc-command-{sequence:04d}.json"
        for sequence in range(len(statuses))
    }
    inventoried = {
        path for path in artifacts if path.startswith(".status/qgc-command-")
    }
    if inventoried != expected:
        raise InspectionError("QGC command evidence inventory does not match status files")


def _load_evidence(run_directory: Path | str) -> Evidence:
    directory = Path(run_directory).resolve()
    try:
        configuration = load_run_config(directory / "configuration/run.json")
    except (OSError, ValueError) as error:
        raise InspectionError(f"immutable run configuration is invalid: {error}") from error
    if directory.name != configuration.run_id or directory.parent != configuration.output_root:
        raise InspectionError("run directory does not match immutable run identity/output binding")
    descriptor = configuration.operator_control
    if configuration.launch_mode != "operator" or descriptor is None:
        raise InspectionError("run has no qgc_actions_v1 operator descriptor")
    manifest = _read_manifest(directory)
    if manifest.get("run_id") != configuration.run_id:
        raise InspectionError("manifest and run configuration identities differ")
    artifacts = _manifest_artifacts(manifest)
    try:
        with OperatorProtocol(
            directory, configuration.run_id, configuration.config_sha256
        ) as protocol:
            request = protocol.read_start_request()
            admission = protocol.read_start_admission()
            abort_request = protocol.read_flight_abort_request()
            abort_statuses = protocol.read_flight_abort_statuses()
            command_statuses = protocol.read_qgc_command_statuses()
    except OperatorProtocolError as error:
        raise InspectionError(f"operator protocol evidence is invalid: {error}") from error
    _validate_configuration_records(
        manifest, descriptor, configuration.config_sha256, admission
    )
    _validate_command_status_inventory(artifacts, command_statuses)
    provisional = Evidence(
        directory,
        configuration.run_id,
        configuration.config_sha256,
        configuration,
        descriptor,
        manifest,
        artifacts,
        request,
        admission,
        abort_request,
        abort_statuses,
        command_statuses,
        (),
    )
    companion_events = _read_jsonl(provisional, COMPANION_LOG_PATH)
    expected_log_keys = {
        "run_id",
        "module",
        "severity",
        "event",
        "sim_timestamp",
        "wall_timestamp",
        "fields",
    }
    if any(
        set(row) != expected_log_keys
        or row.get("run_id") != configuration.run_id
        or row.get("module") != "companion"
        or not isinstance(row.get("event"), str)
        or not isinstance(row.get("fields"), Mapping)
        for row in companion_events
    ):
        raise InspectionError("companion lifecycle evidence is malformed or misbound")
    return replace(provisional, companion_events=companion_events)


def _read_tlog(
    evidence: Evidence,
    connection_factory: ConnectionFactory,
) -> tuple[TlogRow, ...]:
    _require_manifest_artifact(evidence, NATIVE_TLOG_PATH)
    validation = validate_regular_file(evidence.directory, NATIVE_TLOG_PATH)
    if validation.status is not ValidationStatus.VALID or validation.size_bytes == 0:
        raise InspectionError(f"{NATIVE_TLOG_PATH} is missing, unsafe, or empty")
    path = evidence.directory / NATIVE_TLOG_PATH
    try:
        connection = connection_factory(str(path))
    except Exception as error:
        raise InspectionError(f"{NATIVE_TLOG_PATH} could not be opened") from error
    rows: list[TlogRow] = []
    try:
        while len(rows) <= 1_000_000:
            message = connection.recv_match(blocking=False)
            if message is None:
                break
            try:
                fields = message.to_dict()
                timestamp = _finite_number(
                    getattr(message, "_timestamp", None), "native tlog timestamp"
                )
                packet = bytes(message.get_msgbuf())
                rows.append(
                    TlogRow(
                        message.get_type(),
                        int(message.get_srcSystem()),
                        int(message.get_srcComponent()),
                        round(timestamp * 1_000_000_000),
                        bool(packet) and packet[0] == 0xFD,
                        fields,
                    )
                )
            except (AttributeError, TypeError, ValueError, InspectionError) as error:
                raise InspectionError("native tlog contains a malformed message") from error
        else:
            raise InspectionError("native tlog message bound was exceeded")
    finally:
        close = getattr(connection, "close", None)
        if callable(close):
            close()
    if not rows:
        raise InspectionError("native tlog contains no decoded messages")
    return tuple(rows)


def _valid_wire_parameters(status: QgcCommandStatus, descriptor: QgcOperatorDescriptor) -> bool:
    envelope = status.envelope
    parameters = envelope.parameters
    return (
        envelope.wire_version == 2
        and envelope.source == descriptor.qgc_source
        and envelope.target == descriptor.companion
        and envelope.command in {descriptor.start_command, descriptor.abort_command}
        and isinstance(parameters[0], (int, float))
        and not isinstance(parameters[0], bool)
        and math.isfinite(float(parameters[0]))
        and float(parameters[0]) == descriptor.attempt_token
        and all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
            and float(value) == 0.0
            for value in parameters[1:]
        )
    )


def _intervention_matches(
    row: Mapping[str, Any], status: QgcCommandStatus
) -> bool:
    fields = row.get("fields")
    envelope = status.envelope
    if not isinstance(fields, Mapping):
        return False
    return (
        row.get("event") == "qgc_manual_command"
        and row.get("message_type") == "COMMAND_LONG"
        and row.get("command") == envelope.command
        and row.get("source_system") == envelope.source.system_id
        and row.get("source_component") == envelope.source.component_id
        and fields.get("target_system") == envelope.target.system_id
        and fields.get("target_component") == envelope.target.component_id
        and fields.get("command") == envelope.command
        and all(
            isinstance(fields.get(f"param{index}"), (int, float))
            and not isinstance(fields.get(f"param{index}"), bool)
            and math.isclose(
                float(fields[f"param{index}"]),
                float(envelope.parameters[index - 1]),
                rel_tol=0.0,
                abs_tol=1e-6,
            )
            for index in range(1, 8)
        )
    )


def _tlog_command_matches(row: TlogRow, status: QgcCommandStatus) -> bool:
    envelope = status.envelope
    fields = row.fields

    def parameter_matches(index: int) -> bool:
        value = fields.get(f"param{index}")
        return (
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isclose(
                float(value),
                float(envelope.parameters[index - 1]),
                rel_tol=0.0,
                abs_tol=1e-6,
            )
        )

    return (
        row.message_type == "COMMAND_LONG"
        and row.mavlink2
        and (row.source_system, row.source_component)
        == (envelope.source.system_id, envelope.source.component_id)
        and fields.get("target_system") == envelope.target.system_id
        and fields.get("target_component") == envelope.target.component_id
        and fields.get("command") == envelope.command
        and all(parameter_matches(index) for index in range(1, 8))
        and abs(row.timestamp_ns - envelope.received_wall_time_ns)
        <= QGC_WALL_TOLERANCE_NS
    )


def _require_native_exchange(
    status: QgcCommandStatus,
    tlog: Sequence[TlogRow],
    interventions: Sequence[Mapping[str, Any]],
) -> None:
    intervention_matches = [
        row
        for row in interventions
        if _intervention_matches(row, status)
        and type(row.get("wall_time_ns")) is int
        and abs(row["wall_time_ns"] - status.envelope.received_wall_time_ns)
        <= QGC_WALL_TOLERANCE_NS
    ]
    if len(intervention_matches) != 1:
        raise InspectionError(
            f"command {status.sequence} has missing or ambiguous bridge intervention evidence"
        )
    command_matches = [row for row in tlog if _tlog_command_matches(row, status)]
    if len(command_matches) != 1:
        raise InspectionError(
            f"command {status.sequence} has missing or ambiguous native MAVLink 2 bytes"
        )
    command_row = command_matches[0]
    heartbeat = [
        row
        for row in tlog
        if row.message_type == "HEARTBEAT"
        and row.mavlink2
        and (row.source_system, row.source_component)
        == (
            status.envelope.target.system_id,
            status.envelope.target.component_id,
        )
        and 0 <= command_row.timestamp_ns - row.timestamp_ns <= QGC_RESPONSE_WINDOW_NS
    ]
    if not heartbeat:
        raise InspectionError(
            f"command {status.sequence} has no recent companion heartbeat in native tlog"
        )
    response = status.response
    acknowledgements = [
        row
        for row in tlog
        if row.message_type == "COMMAND_ACK"
        and row.mavlink2
        and (row.source_system, row.source_component)
        == (
            status.envelope.target.system_id,
            status.envelope.target.component_id,
        )
        and row.fields.get("command") == response.command
        and row.fields.get("result") == response.result
        and row.fields.get("progress") == response.progress
        and row.fields.get("target_system") == response.recipient.system_id
        and row.fields.get("target_component") == response.recipient.component_id
        and 0 <= row.timestamp_ns - command_row.timestamp_ns <= QGC_RESPONSE_WINDOW_NS
    ]
    if len(acknowledgements) != 1:
        raise InspectionError(
            f"command {status.sequence} has missing or ambiguous targeted native ACK"
        )
    status_texts = [
        row
        for row in tlog
        if row.message_type == "STATUSTEXT"
        and row.mavlink2
        and (row.source_system, row.source_component)
        == (
            status.envelope.target.system_id,
            status.envelope.target.component_id,
        )
        and row.fields.get("text") == response.status_text
        and 0 <= row.timestamp_ns - command_row.timestamp_ns <= QGC_RESPONSE_WINDOW_NS
    ]
    if len(status_texts) != 1:
        raise InspectionError(
            f"command {status.sequence} has missing or ambiguous native status text"
        )


def _start_statuses(evidence: Evidence) -> tuple[QgcCommandStatus, QgcCommandStatus]:
    descriptor = evidence.descriptor
    if evidence.request is None or evidence.admission is None:
        raise InspectionError("QGC Start request/admission evidence is missing")
    starts = [
        status
        for status in evidence.command_statuses
        if status.envelope.command == descriptor.start_command
        and _valid_wire_parameters(status, descriptor)
        and status.request_id == descriptor.start_request_id
        and status.request_id == evidence.request.request_id
        and status.request_sha256 == evidence.request.sha256
    ]
    reserved = [
        status
        for status in starts
        if status.response.result == MAV_RESULT_IN_PROGRESS
        and status.admission_sha256 is None
    ]
    accepted = [
        status
        for status in starts
        if status.response.result == MAV_RESULT_ACCEPTED
        and status.admission_sha256 == evidence.admission.sha256
    ]
    if len(reserved) != 1 or len(accepted) != 1:
        raise InspectionError("QGC Start requires one reserved and one admitted command response")
    if reserved[0].sequence >= accepted[0].sequence:
        raise InspectionError("QGC Start admission response precedes its reservation")
    try:
        with RuntimeProtocol(evidence.directory, evidence.run_id) as runtime:
            running = runtime.read_status(RuntimeRunningStatus)
    except ProtocolError as error:
        raise InspectionError(f"RUNNING evidence is invalid: {error}") from error
    if running is None:
        raise InspectionError("QGC Start has no matching durable RUNNING evidence")
    return reserved[0], accepted[0]


def _check_command(
    evidence: Evidence,
    expectation: Expectation,
    tlog_connection_factory: ConnectionFactory,
) -> Finding:
    if not evidence.command_statuses:
        raise InspectionError("QGC command-status evidence is missing")
    interventions = _read_jsonl(evidence, INTERVENTION_PATH)
    tlog = _read_tlog(evidence, tlog_connection_factory)
    if expectation == "routing":
        if evidence.request is not None or evidence.admission is not None:
            raise InspectionError("routing-only proof unexpectedly contains a Start admission")
        candidates = [
            status
            for status in evidence.command_statuses
            if _valid_wire_parameters(status, evidence.descriptor)
            and status.response.result not in {
                MAV_RESULT_ACCEPTED,
                MAV_RESULT_IN_PROGRESS,
                MAV_RESULT_UNSUPPORTED,
            }
        ]
        if len(candidates) != 1:
            raise InspectionError(
                "routing proof requires one current run-bound rejected command"
            )
        _require_native_exchange(candidates[0], tlog, interventions)
        return Finding(
            "passed",
            "manifest-bound native MAVLink 2 command, bridge record, companion "
            "heartbeat, targeted rejection ACK, and status text agree",
        )

    reserved, accepted = _start_statuses(evidence)
    if evidence.admission is None:
        raise InspectionError("QGC Start admission evidence is missing")
    admission = evidence.admission
    _require_native_exchange(reserved, tlog, interventions)
    _require_native_exchange(accepted, tlog, interventions)
    if expectation == "abort":
        if evidence.abort_request is None:
            raise InspectionError("QGC Abort request is missing")
        candidates = [
            status
            for status in evidence.command_statuses
            if status.envelope.command == QGC_ABORT_COMMAND
            and _valid_wire_parameters(status, evidence.descriptor)
            and status.request_id == evidence.descriptor.abort_request_id
            and status.request_id == evidence.abort_request.request_id
            and status.request_sha256 == evidence.abort_request.sha256
            and status.admission_sha256 == admission.sha256
            and status.observed_outcome == "landed_disarmed"
            and status.response.result == MAV_RESULT_ACCEPTED
        ]
        if len(candidates) != 1:
            raise InspectionError("QGC Abort has no unique bound terminal command response")
        _require_native_exchange(candidates[0], tlog, interventions)
        return Finding(
            "passed",
            "Start admission and terminal Abort response match native tlog and "
            "durable command evidence",
        )
    return Finding(
        "passed",
        "QGC Start reservation, admission, RUNNING status, bridge record, and "
        "native ACK evidence agree",
    )


def _select_dataflash(evidence: Evidence) -> str:
    paths = sorted(
        path
        for path, row in evidence.manifest_artifacts.items()
        if DATAFLASH_PATTERN.fullmatch(path)
        and row.get("validation") == ValidationStatus.VALID.value
    )
    if len(paths) != 1:
        raise InspectionError(
            "manifest must select exactly one valid ArduPilot DataFlash BIN"
        )
    return paths[0]


def _read_dataflash(
    evidence: Evidence,
    connection_factory: ConnectionFactory,
) -> tuple[DataFlashRow, ...]:
    relative_path = _select_dataflash(evidence)
    path = evidence.directory / relative_path
    try:
        connection = connection_factory(str(path))
    except Exception as error:
        raise InspectionError("manifest-selected DataFlash log could not be opened") from error
    rows: list[DataFlashRow] = []
    relevant = {"ARM", "CTUN", "RFND", "MODE", "PL", "PARM"}
    try:
        while len(rows) <= 2_000_000:
            message = connection.recv_match(blocking=False)
            if message is None:
                break
            document = message.to_dict()
            message_type = document.get("mavpackettype")
            if message_type not in relevant:
                continue
            time_us = document.get("TimeUS")
            if type(time_us) is not int or time_us < 0:
                raise InspectionError(
                    f"DataFlash {message_type} row has an invalid TimeUS"
                )
            rows.append(DataFlashRow(str(message_type), time_us, document))
        else:
            raise InspectionError("DataFlash message bound was exceeded")
    finally:
        close = getattr(connection, "close", None)
        if callable(close):
            close()
    for before, after in zip(rows, rows[1:]):
        if after.time_us < before.time_us:
            raise InspectionError("DataFlash relevant timestamps regress")
    return tuple(rows)


def _after_native_epoch(
    evidence: Evidence, rows: Sequence[DataFlashRow]
) -> tuple[DataFlashRow, ...]:
    if evidence.admission is None:
        raise InspectionError("physical proof requires a Start admission")
    epoch_us = evidence.admission.native_epoch_ns // 1_000
    return tuple(row for row in rows if row.time_us >= epoch_us)


def _field_number(row: DataFlashRow, name: str) -> float:
    return _finite_number(row.fields.get(name), f"DataFlash {row.message_type}.{name}")


def _arm_state(row: DataFlashRow) -> bool:
    value = row.fields.get("ArmState")
    if value not in {0, 1, False, True}:
        raise InspectionError("DataFlash ARM.ArmState is invalid")
    return bool(value)


def _airborne_after_admission(
    evidence: Evidence, rows: Sequence[DataFlashRow]
) -> tuple[int, int]:
    admitted = _after_native_epoch(evidence, rows)
    arm_rows = [row for row in admitted if row.message_type == "ARM" and _arm_state(row)]
    if not arm_rows:
        if any(row.message_type in {"ARM", "CTUN", "RFND"} for row in rows):
            raise InspectionError("armed transition is absent after the admitted native epoch")
        raise InspectionError("independent armed transition is missing")
    arm_us = arm_rows[0].time_us
    altitude_rows = [
        row
        for row in admitted
        if row.time_us >= arm_us
        and row.message_type in {"CTUN", "RFND"}
        and _field_number(row, "Alt" if row.message_type == "CTUN" else "Dist")
        > AIRBORNE_CLEARANCE_M
    ]
    if not altitude_rows:
        raise InspectionError("independent airborne transition is missing after arming")
    return arm_us, altitude_rows[0].time_us


def _ground_and_disarm_after(
    rows: Sequence[DataFlashRow],
    boundary_us: int,
) -> tuple[int, int]:
    grounds = [
        row
        for row in rows
        if row.time_us > boundary_us
        and (
            (
                row.message_type == "RFND"
                and _field_number(row, "Dist") <= TOUCHDOWN_CLEARANCE_M
            )
            or (
                row.message_type == "CTUN"
                and _field_number(row, "Alt") <= TOUCHDOWN_CLEARANCE_M
            )
        )
    ]
    if not grounds:
        raise InspectionError("independent touchdown/ground evidence is missing")
    ground_us = grounds[0].time_us
    disarms = [
        row
        for row in rows
        if row.time_us >= ground_us
        and row.message_type == "ARM"
        and not _arm_state(row)
    ]
    if not disarms:
        raise InspectionError("independent disarm after touchdown is missing")
    return ground_us, disarms[0].time_us


def _profile_transitions(evidence: Evidence) -> tuple[ProfileTransition, ...]:
    transitions: list[ProfileTransition] = []
    for row in evidence.companion_events:
        if row.get("event") != PROFILE_EVENT_NAME:
            continue
        fields = row["fields"]
        expected = {"parameter", "action", "stage", "value", "generation"}
        allowed = (
            expected,
            expected | {"detail"},
            (expected - {"generation"}) | {"detail"},
        )
        if set(fields) not in allowed:
            raise InspectionError("landing-profile transition fields are malformed")
        parameter = fields.get("parameter")
        action = fields.get("action")
        stage = fields.get("stage")
        if not isinstance(parameter, str) or action not in {"read", "set"} or stage not in {
            "requested",
            "verified",
            "failed",
        }:
            raise InspectionError("landing-profile transition fields are malformed")
        value = _finite_number(fields.get("value"), "landing-profile value")
        generation_value = fields.get("generation")
        generation = (
            None
            if generation_value is None
            else _integer(generation_value, "landing-profile generation")
        )
        detail = fields.get("detail")
        if stage == "failed":
            if not isinstance(detail, str) or not detail:
                raise InspectionError("failed landing-profile transition lacks detail")
        elif detail is not None:
            raise InspectionError("nonfailed landing-profile transition contains detail")
        if generation is None and stage != "failed":
            raise InspectionError("landing-profile transition generation is missing")
        timestamp = _finite_number(
            row.get("sim_timestamp"), "landing-profile public timestamp"
        )
        if timestamp < 0:
            raise InspectionError("landing-profile public timestamp is negative")
        transitions.append(
            ProfileTransition(
                parameter, action, stage, value, generation, timestamp, detail
            )
        )
    for before, after in zip(transitions, transitions[1:]):
        if after.sim_timestamp < before.sim_timestamp:
            raise InspectionError("landing-profile public timestamps regress")
    return tuple(transitions)


def _single_transition(
    transitions: Sequence[ProfileTransition],
    *,
    parameter: str,
    action: str,
    stage: str,
    value: float,
) -> ProfileTransition:
    matches = [
        item
        for item in transitions
        if item.parameter == parameter
        and item.action == action
        and item.stage == stage
        and math.isclose(item.value, value, rel_tol=1e-6, abs_tol=1e-6)
    ]
    if len(matches) != 1:
        raise InspectionError(
            f"landing profile needs one {parameter} {action} {stage} value {value:g}"
        )
    return matches[0]


def _verified_after(requested: ProfileTransition, verified: ProfileTransition) -> None:
    if (
        requested.generation is None
        or verified.generation is None
        or verified.generation <= requested.generation
        or verified.sim_timestamp < requested.sim_timestamp
    ):
        raise InspectionError("landing-profile verified evidence is not newer than enqueue")


def _native_us(evidence: Evidence, public_seconds: float) -> int:
    if evidence.admission is None:
        raise InspectionError("public/native correlation requires a Start admission")
    return evidence.admission.native_epoch_ns // 1_000 + round(public_seconds * 1_000_000)


def _ordinary_profile(
    evidence: Evidence,
    rows: Sequence[DataFlashRow],
) -> tuple[ProfileTransition, ProfileTransition]:
    transitions = _profile_transitions(evidence)
    if any(item.stage == "failed" for item in transitions):
        raise InspectionError("landing profile contains a failed transition")
    read_requests: list[ProfileTransition] = []
    read_verifications: list[ProfileTransition] = []
    for parameter, expected_value in PRECISION_PROFILE.items():
        requested = _single_transition(
            transitions,
            parameter=parameter,
            action="read",
            stage="requested",
            value=expected_value,
        )
        verified = _single_transition(
            transitions,
            parameter=parameter,
            action="read",
            stage="verified",
            value=expected_value,
        )
        _verified_after(requested, verified)
        read_requests.append(requested)
        read_verifications.append(verified)
    read_boundaries = {item.generation for item in read_requests}
    if len(read_boundaries) != 1 or None in read_boundaries:
        raise InspectionError(
            "full landing-profile read requests do not share one captured boundary"
        )
    illegal_sets = [
        item
        for item in transitions
        if item.action == "set" and item.parameter != "PLND_ENABLED"
    ]
    if illegal_sets:
        raise InspectionError("ordinary landing changed another precision-profile parameter")
    disable_requested = _single_transition(
        transitions,
        parameter="PLND_ENABLED",
        action="set",
        stage="requested",
        value=0.0,
    )
    if any(
        verified.sim_timestamp > disable_requested.sim_timestamp
        for verified in read_verifications
    ):
        raise InspectionError("full landing-profile read was not fresh before disable")
    if (
        disable_requested.generation is None
        or any(
            verified.generation is None
            or disable_requested.generation < verified.generation
            for verified in read_verifications
        )
    ):
        raise InspectionError(
            "PLND_ENABLED disable boundary predates a full-profile observation"
        )
    disable_verified = _single_transition(
        transitions,
        parameter="PLND_ENABLED",
        action="set",
        stage="verified",
        value=0.0,
    )
    _verified_after(disable_requested, disable_verified)
    if not any(
        item.parameter == "PLND_ENABLED"
        and item.action == "set"
        and math.isclose(item.value, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        for item in transitions
    ):
        raise InspectionError("PLND_ENABLED restore evidence is missing")
    restore_requested = _single_transition(
        transitions,
        parameter="PLND_ENABLED",
        action="set",
        stage="requested",
        value=1.0,
    )
    restore_verified = _single_transition(
        transitions,
        parameter="PLND_ENABLED",
        action="set",
        stage="verified",
        value=1.0,
    )
    _verified_after(restore_requested, restore_verified)
    if (
        restore_requested.generation is None
        or disable_verified.generation is None
        or restore_requested.generation < disable_verified.generation
    ):
        raise InspectionError("PLND_ENABLED restore boundary predates verified disable")
    if restore_requested.sim_timestamp <= disable_verified.sim_timestamp:
        raise InspectionError("PLND_ENABLED restore does not follow verified disable")
    parameter_rows = [row for row in rows if row.message_type == "PARM"]
    for row in _after_native_epoch(evidence, parameter_rows):
        name = _parameter_name(row)
        if name not in PRECISION_PROFILE:
            continue
        value = _field_number(row, "Value")
        if name == "PLND_ENABLED":
            if not any(math.isclose(value, allowed, abs_tol=1e-6) for allowed in (0.0, 1.0)):
                raise InspectionError("DataFlash contains an unexpected PLND_ENABLED value")
        elif not math.isclose(value, PRECISION_PROFILE[name], abs_tol=1e-6):
            raise InspectionError(f"DataFlash shows changed precision setting {name}")
    for transition, value, label in (
        (disable_verified, 0.0, "disable"),
        (restore_verified, 1.0, "restore"),
    ):
        transition_us = _native_us(evidence, transition.sim_timestamp)
        matches = [
            row
            for row in parameter_rows
            if _parameter_name(row) == "PLND_ENABLED"
            and math.isclose(_field_number(row, "Value"), value, abs_tol=1e-6)
            and abs(row.time_us - transition_us) <= 1_000_000
        ]
        if not matches:
            raise InspectionError(f"DataFlash does not corroborate PLND_ENABLED {label}")
    return disable_verified, restore_verified


def _parameter_name(row: DataFlashRow) -> str | None:
    value = row.fields.get("Name")
    if isinstance(value, bytes):
        try:
            value = value.decode("ascii")
        except UnicodeDecodeError as error:
            raise InspectionError("DataFlash parameter name is not ASCII") from error
    return value.rstrip("\0 ") if isinstance(value, str) else None


def _check_start_physical(
    evidence: Evidence, rows: Sequence[DataFlashRow]
) -> Finding:
    arm_us, airborne_us = _airborne_after_admission(evidence, rows)
    return Finding(
        "passed",
        f"DataFlash records armed at {arm_us} us and airborne above "
        f"{AIRBORNE_CLEARANCE_M:g} m at {airborne_us} us after admission",
    )


def _check_abort_physical(
    evidence: Evidence, rows: Sequence[DataFlashRow]
) -> Finding:
    expected_states = ("accepted", "recovering", "landed_disarmed")
    states = tuple(status.state for status in evidence.abort_statuses)
    if states != expected_states:
        raise InspectionError(
            "Abort recovery statuses must be ordered accepted, recovering, landed_disarmed"
        )
    for before, after in zip(evidence.abort_statuses, evidence.abort_statuses[1:]):
        if after.observed_wall_time_ns < before.observed_wall_time_ns:
            raise InspectionError("Abort recovery wall timestamps regress")
        if (
            before.observed_native_ns is not None
            and after.observed_native_ns is not None
            and after.observed_native_ns < before.observed_native_ns
        ):
            raise InspectionError("Abort recovery native timestamps regress")
    recovering = evidence.abort_statuses[1]
    if recovering.observed_native_ns is None:
        raise InspectionError("Abort recovery start lacks a native timestamp")
    boundary_us = recovering.observed_native_ns // 1_000
    _airborne_after_admission(evidence, rows)
    ground_us, disarm_us = _ground_and_disarm_after(rows, boundary_us)
    terminal = evidence.abort_statuses[-1]
    if terminal.observed_native_ns is not None and terminal.observed_native_ns // 1_000 < disarm_us:
        raise InspectionError("Abort landed_disarmed status predates independent disarm")
    return Finding(
        "passed",
        f"ordered recovery began at {boundary_us} us; ground at {ground_us} us "
        f"and disarm at {disarm_us} us followed",
    )


def _check_ordinary_physical(
    evidence: Evidence, rows: Sequence[DataFlashRow]
) -> Finding:
    _arm_us, airborne_us = _airborne_after_admission(evidence, rows)
    transitions = _profile_transitions(evidence)
    disable_verified = _single_transition(
        transitions,
        parameter="PLND_ENABLED",
        action="set",
        stage="verified",
        value=0.0,
    )
    disable_us = _native_us(evidence, disable_verified.sim_timestamp)
    lands = [
        row
        for row in rows
        if row.time_us >= disable_us
        and row.message_type == "MODE"
        and _field_number(row, "Mode") == 9.0
        and _field_number(row, "ModeNum") == 9.0
    ]
    if not lands:
        raise InspectionError("DataFlash LAND does not follow verified precision disable")
    land_us = lands[0].time_us
    before_altitudes = [
        _field_number(row, "Alt")
        for row in rows
        if row.message_type == "CTUN" and airborne_us <= row.time_us <= land_us
    ]
    after_altitudes = [
        _field_number(row, "Alt")
        for row in rows
        if row.message_type == "CTUN" and row.time_us > land_us
    ]
    if (
        not before_altitudes
        or not after_altitudes
        or min(after_altitudes) >= max(before_altitudes) - 0.5
    ):
        raise InspectionError("DataFlash does not show descent after LAND")
    ground_us, disarm_us = _ground_and_disarm_after(rows, land_us)
    restore_rows = [
        item
        for item in transitions
        if item.parameter == "PLND_ENABLED"
        and item.action == "set"
        and math.isclose(item.value, 1.0, rel_tol=1e-6, abs_tol=1e-6)
    ]
    if restore_rows:
        restore_requested = _single_transition(
            transitions,
            parameter="PLND_ENABLED",
            action="set",
            stage="requested",
            value=1.0,
        )
        restore_verified = _single_transition(
            transitions,
            parameter="PLND_ENABLED",
            action="set",
            stage="verified",
            value=1.0,
        )
        if (
            _native_us(evidence, restore_requested.sim_timestamp) <= ground_us
            or _native_us(evidence, restore_verified.sim_timestamp) <= ground_us
        ):
            raise InspectionError("PLND_ENABLED restore precedes independent touchdown")
    return Finding(
        "passed",
        f"LAND at {land_us} us is followed by descent, ground at {ground_us} us, "
        f"and disarm at {disarm_us} us",
    )


def _pose(value: object, label: str) -> tuple[int, tuple[float, float, float]]:
    if not isinstance(value, Mapping):
        raise InspectionError(f"stimulus {label} pose is malformed")
    timestamp = _integer(value.get("native_timestamp_ns"), f"stimulus {label} timestamp")
    position = value.get("position")
    orientation = value.get("orientation")
    if (
        not isinstance(position, list)
        or len(position) != 3
        or not isinstance(orientation, list)
        or len(orientation) != 4
    ):
        raise InspectionError(f"stimulus {label} pose is malformed")
    xyz = tuple(_finite_number(item, f"stimulus {label} position") for item in position)
    quaternion = tuple(
        _finite_number(item, f"stimulus {label} orientation") for item in orientation
    )
    if not all(
        math.isclose(actual, expected, abs_tol=1e-3)
        for actual, expected in zip(quaternion, (0.0, 0.0, 0.0, 1.0), strict=True)
    ):
        raise InspectionError(f"stimulus {label} orientation is not identity")
    return timestamp, xyz  # type: ignore[return-value]


def _stimulus(evidence: Evidence) -> Mapping[str, Any]:
    rows = _read_jsonl(evidence, STIMULUS_PATH)
    expected_keys = {
        "schema_version",
        "status",
        "detail",
        "run_id",
        "config_sha256",
        "world",
        "entity",
        "partition",
        "container",
        "pre_command",
        "service",
        "post_command",
    }
    if len(rows) != 3:
        raise InspectionError(
            "successful precision-loss stimulus requires exactly three durable phases"
        )
    if any(set(row) != expected_keys or row.get("schema_version") != 1 for row in rows):
        raise InspectionError("precision-loss stimulus record has an invalid schema")
    attempted, response_received, record = rows
    if [row.get("status") for row in rows] != [
        "attempted",
        "response_received",
        "succeeded",
    ]:
        raise InspectionError("precision-loss stimulus phases are incomplete or out of order")
    if any(not isinstance(row.get("detail"), str) or not row.get("detail") for row in rows):
        raise InspectionError("precision-loss stimulus phase detail is missing")
    stable_keys = {
        "schema_version",
        "run_id",
        "config_sha256",
        "world",
        "entity",
        "partition",
        "container",
        "pre_command",
    }
    if any(
        attempted.get(key) != row.get(key)
        for row in (response_received, record)
        for key in stable_keys
    ):
        raise InspectionError("precision-loss stimulus phase bindings changed")
    if (
        attempted.get("post_command") is not None
        or response_received.get("post_command") is not None
    ):
        raise InspectionError(
            "precision-loss stimulus pre-success phases contain post-command evidence"
        )
    services: list[Mapping[str, Any]] = []
    for row in rows:
        service = row.get("service")
        if not isinstance(service, Mapping):
            raise InspectionError("precision-loss stimulus service evidence is malformed")
        services.append(service)
    attempted_service, response_service, terminal_service = services
    service_keys = {
        "name",
        "request_type",
        "response_type",
        "request_text",
        "request_utf8_hex",
        "request_sha256",
        "requested_wall_time_ns",
        "outcome",
        "returncode",
        "response",
        "stderr",
    }
    if any(set(service) != service_keys for service in services):
        raise InspectionError("precision-loss stimulus service schema is invalid")
    stable_service_keys = service_keys - {"outcome", "returncode", "response", "stderr"}
    if any(
        attempted_service.get(key) != service.get(key)
        for service in (response_service, terminal_service)
        for key in stable_service_keys
    ):
        raise InspectionError("precision-loss stimulus service binding changed between phases")
    if (
        attempted_service.get("outcome") != "pending"
        or any(
            attempted_service.get(key) is not None
            for key in ("returncode", "response", "stderr")
        )
        or response_service.get("outcome") != "returned"
        or terminal_service != response_service
    ):
        raise InspectionError("precision-loss stimulus service phases are inconsistent")
    if (
        record.get("status") != "succeeded"
        or record.get("run_id") != evidence.run_id
        or record.get("config_sha256") != evidence.config_sha256
        or record.get("world") != "moving_pad_stationary"
        or record.get("entity") != "moving_pad"
        or record.get("partition") != "drone_sim_" + evidence.run_id.replace("-", "_")
    ):
        raise InspectionError("precision-loss stimulus result or run binding is invalid")
    container = record.get("container")
    if (
        not isinstance(container, Mapping)
        or set(container) != {"id", "project", "service", "companion_id"}
        or not isinstance(container.get("id"), str)
        or not container.get("id")
        or container.get("project") != "drone-sim-" + evidence.run_id.replace("-", "")
        or container.get("service") != "gazebo-runtime"
        or not isinstance(container.get("companion_id"), str)
        or not container.get("companion_id")
    ):
        raise InspectionError("precision-loss stimulus container binding is invalid")
    service = terminal_service
    request_text = service.get("request_text")
    if (
        service.get("name") != "/world/moving_pad_stationary/set_pose"
        or service.get("request_type") != "gz.msgs.Pose"
        or service.get("response_type") != "gz.msgs.Boolean"
        or request_text != STIMULUS_REQUEST_TEXT
        or service.get("outcome") != "returned"
        or service.get("returncode") != 0
        or not isinstance(service.get("response"), str)
        or re.fullmatch(r"\s*data:\s*true\s*", service["response"]) is None
        or not isinstance(service.get("stderr"), str)
        or not isinstance(request_text, str)
        or service.get("request_utf8_hex") != request_text.encode("utf-8").hex()
        or service.get("request_sha256")
        != hashlib.sha256(request_text.encode("utf-8")).hexdigest()
        or type(service.get("requested_wall_time_ns")) is not int
        or service["requested_wall_time_ns"] < 0
    ):
        raise InspectionError("precision-loss stimulus service did not return verified success")
    pre = record.get("pre_command")
    post = record.get("post_command")
    if (
        not isinstance(pre, Mapping)
        or set(pre)
        != {
            "pad_odometry",
            "range",
            "native_horizon_ns",
            "mission",
            "qgc_interventions",
            "dataflash",
        }
        or not isinstance(post, Mapping)
        or set(post)
        != {
            "first",
            "persistence",
            "displacement_m",
            "minimum_persistence_ns",
            "threshold_comparison",
            "qgc_interventions",
        }
    ):
        raise InspectionError("precision-loss stimulus observations are malformed")
    pre_time, pre_position = _pose(pre.get("pad_odometry"), "pre-command")
    first_time, first_position = _pose(post.get("first"), "first post-command")
    horizon = _integer(pre.get("native_horizon_ns"), "stimulus native horizon")
    if pre_time > horizon or first_time <= horizon:
        raise InspectionError("precision-loss stimulus native chronology is invalid")
    persistence = post.get("persistence")
    if not isinstance(persistence, list) or not persistence or len(persistence) > 16:
        raise InspectionError("precision-loss stimulus persistence observations are missing")
    samples = [_pose(value, "persistent post-command") for value in persistence]
    timestamps = [first_time, *(stamp for stamp, _position in samples)]
    if any(after <= before for before, after in zip(timestamps, timestamps[1:])):
        raise InspectionError("precision-loss stimulus odometry timestamps do not increase")
    if (
        post.get("minimum_persistence_ns") != STIMULUS_PERSISTENCE_NS
        or post.get("threshold_comparison") != "strictly_greater_than"
    ):
        raise InspectionError("precision-loss stimulus persistence contract is invalid")
    if timestamps[-1] - first_time <= STIMULUS_PERSISTENCE_NS:
        raise InspectionError("precision-loss stimulus did not persist over 0.5 seconds")
    if not all(
        math.isclose(actual, expected, abs_tol=0.25)
        for actual, expected in zip(pre_position, (35.0, 0.0, 0.0), strict=True)
    ) or not all(
        all(
            math.isclose(actual, expected, abs_tol=0.25)
            for actual, expected in zip(position, (55.0, 0.0, 0.0), strict=True)
        )
        for _stamp, position in ((first_time, first_position), *samples)
    ):
        raise InspectionError("precision-loss stimulus did not retain the expected pad poses")
    displacement = _finite_number(post.get("displacement_m"), "stimulus displacement")
    if (
        not math.isclose(displacement, 20.0, abs_tol=0.5)
        or not math.isclose(displacement, first_position[0] - pre_position[0], abs_tol=1e-6)
    ):
        raise InspectionError("precision-loss stimulus displacement is not approximately 20 m")
    range_row = pre.get("range")
    if (
        not isinstance(range_row, Mapping)
        or set(range_row) != {"native_timestamp_ns", "distance_m"}
    ):
        raise InspectionError("precision-loss stimulus pre-command range is malformed")
    range_time = _integer(
        range_row.get("native_timestamp_ns"), "stimulus range timestamp"
    )
    if (
        range_time > horizon
        or horizon - range_time > STIMULUS_PRECISION_MAX_AGE_NS
        or horizon - pre_time > STIMULUS_PRECISION_MAX_AGE_NS
    ):
        raise InspectionError("precision-loss stimulus pre-command samples are stale")
    if _finite_number(range_row.get("distance_m"), "stimulus range") <= 0.75:
        raise InspectionError("precision-loss stimulus clearance is not above 0.75 m")
    if pre.get("qgc_interventions") != post.get("qgc_interventions"):
        raise InspectionError("QGC intervention binding changed during physical stimulus")
    return record


def _precision_operation(evidence: Evidence) -> PrecisionOperation:
    starts: list[tuple[str, float]] = []
    modes: list[tuple[str, str, float]] = []
    failures: list[tuple[str, float]] = []
    for row in evidence.companion_events:
        fields = row["fields"]
        event = row["event"]
        timestamp = row.get("sim_timestamp")
        if event == "operation_started" and fields.get("tool") == "precision_land":
            operation_id = fields.get("operation_id")
            if not isinstance(operation_id, str) or not operation_id:
                raise InspectionError("precision operation identifier is malformed")
            starts.append(
                (
                    operation_id,
                    _finite_number(timestamp, "precision operation timestamp"),
                )
            )
        elif event == "precision_mode_requested":
            operation_id = fields.get("operation_id")
            mode = fields.get("mode")
            if not isinstance(operation_id, str) or not isinstance(mode, str):
                raise InspectionError("precision mode request is malformed")
            modes.append(
                (
                    operation_id,
                    mode,
                    _finite_number(timestamp, "precision mode timestamp"),
                )
            )
        elif (
            event == "operation_finished"
            and fields.get("tool") == "precision_land"
            and fields.get("state") == "failed"
            and fields.get("error") == PRECISION_FAILURE
        ):
            operation_id = fields.get("operation_id")
            if not isinstance(operation_id, str) or not operation_id:
                raise InspectionError("precision failure identifier is malformed")
            failures.append(
                (
                    operation_id,
                    _finite_number(timestamp, "precision failure timestamp"),
                )
            )
    if len(starts) != 1 or len(failures) != 1:
        raise InspectionError("precision-loss operation start/failure evidence is incomplete")
    operation_id, start = starts[0]
    if failures[0][0] != operation_id:
        raise InspectionError("precision-loss failure belongs to another operation")
    operation_modes = [(mode, stamp) for owner, mode, stamp in modes if owner == operation_id]
    if [mode for mode, _stamp in operation_modes] != ["LAND", "GUIDED"]:
        raise InspectionError("precision policy must request LAND then GUIDED exactly")
    land = operation_modes[0][1]
    guided = operation_modes[1][1]
    failure = failures[0][1]
    if not start <= land < guided <= failure:
        raise InspectionError("precision operation lifecycle chronology is invalid")
    return PrecisionOperation(operation_id, start, land, guided, failure)


def _validate_stimulus_qgc_binding(
    evidence: Evidence, value: object, accepted: QgcCommandStatus
) -> None:
    if not isinstance(value, Mapping):
        raise InspectionError("stimulus QGC intervention binding is malformed")
    if set(value) != {
        "path",
        "start_wall_time_ns",
        "accepted_start_received_wall_time_ns",
        "rows_checked",
    }:
        raise InspectionError("stimulus QGC intervention binding is malformed")
    start_wall_time_ns = _integer(
        value.get("start_wall_time_ns"), "stimulus Start wall time"
    )
    accepted_wall_time_ns = _integer(
        value.get("accepted_start_received_wall_time_ns"),
        "stimulus accepted Start wall time",
    )
    rows_checked = _integer(
        value.get("rows_checked"), "stimulus QGC rows checked", minimum=1
    )
    interventions = _read_jsonl(evidence, INTERVENTION_PATH)
    intervention_times: list[int] = []
    for row in interventions:
        timestamp = row.get("wall_time_ns")
        if type(timestamp) is not int:
            raise InspectionError("QGC intervention wall time is malformed")
        intervention_times.append(timestamp)
    if any(
        after < before
        for before, after in zip(intervention_times, intervention_times[1:])
    ):
        raise InspectionError("QGC intervention wall timestamps regress")
    if (
        value.get("path") != INTERVENTION_PATH
        or accepted_wall_time_ns != accepted.envelope.received_wall_time_ns
        or rows_checked != len(interventions)
    ):
        raise InspectionError("stimulus QGC intervention binding disagrees with the run")
    starts = [
        row
        for row in interventions
        if _intervention_matches(row, accepted)
        and type(row.get("wall_time_ns")) is int
        and row["wall_time_ns"] <= accepted_wall_time_ns
    ]
    if not starts or starts[-1]["wall_time_ns"] != start_wall_time_ns:
        raise InspectionError("stimulus QGC Start intervention is missing or ambiguous")
    if any(
        type(row.get("wall_time_ns")) is not int
        or (
            row["wall_time_ns"] > accepted_wall_time_ns
            and not _intervention_matches(row, accepted)
        )
        for row in interventions
    ):
        raise InspectionError("QGC takeover evidence follows the accepted mission Start")


def _validate_stimulus_mission_binding(
    evidence: Evidence,
    stimulus: Mapping[str, Any],
    operation: PrecisionOperation,
    accepted: QgcCommandStatus,
) -> None:
    pre = stimulus["pre_command"]
    container = stimulus["container"]
    service = stimulus["service"]
    assert isinstance(pre, Mapping)
    assert isinstance(container, Mapping)
    assert isinstance(service, Mapping)
    mission = pre.get("mission")
    if not isinstance(mission, Mapping) or set(mission) != {
        "operation_id",
        "sim_timestamp",
        "native_start_ns",
        "land_request_sim_timestamp",
        "native_land_request_ns",
        "marker_id",
        "source",
        "source_container_id",
        "source_tail_lines",
    }:
        raise InspectionError("precision-loss stimulus mission binding is malformed")
    if evidence.admission is None:
        raise InspectionError("precision-loss stimulus has no Start admission")
    expected_start_ns = evidence.admission.native_epoch_ns + round(operation.start * 1e9)
    expected_land_ns = evidence.admission.native_epoch_ns + round(operation.land * 1e9)
    if (
        mission.get("operation_id") != operation.operation_id
        or _finite_number(mission.get("sim_timestamp"), "stimulus operation timestamp")
        != operation.start
        or mission.get("native_start_ns") != expected_start_ns
        or _finite_number(
            mission.get("land_request_sim_timestamp"),
            "stimulus LAND request timestamp",
        )
        != operation.land
        or mission.get("native_land_request_ns") != expected_land_ns
        or mission.get("marker_id") != 7
        or mission.get("source") != "live_companion_container_log"
        or mission.get("source_container_id") != container.get("companion_id")
        or mission.get("source_tail_lines") != 4096
    ):
        raise InspectionError(
            "precision-loss stimulus mission binding disagrees with lifecycle evidence"
        )
    horizon_ns = _integer(pre.get("native_horizon_ns"), "stimulus native horizon")
    if horizon_ns < expected_land_ns:
        raise InspectionError("precision-loss stimulus predates the mission LAND request")
    _validate_stimulus_qgc_binding(evidence, pre.get("qgc_interventions"), accepted)
    if _integer(
        service.get("requested_wall_time_ns"), "stimulus request wall time"
    ) <= accepted.envelope.received_wall_time_ns:
        raise InspectionError("precision-loss service request predates accepted mission Start")


def _validate_stimulus_dataflash(
    stimulus: Mapping[str, Any],
    rows: Sequence[DataFlashRow],
) -> None:
    pre = stimulus["pre_command"]
    assert isinstance(pre, Mapping)
    flight = pre.get("dataflash")
    if not isinstance(flight, Mapping) or set(flight) != {
        "precision",
        "mode",
        "precision_max_age_ns",
        "land_max_age_ns",
    }:
        raise InspectionError("precision-loss stimulus DataFlash binding is malformed")
    precision = flight.get("precision")
    mode = flight.get("mode")
    if (
        not isinstance(precision, Mapping)
        or set(precision) != {"time_us", "acquired", "last_measurement_ms"}
        or not isinstance(mode, Mapping)
        or set(mode) != {"time_us", "mode"}
        or flight.get("precision_max_age_ns") != STIMULUS_PRECISION_MAX_AGE_NS
        or flight.get("land_max_age_ns") != STIMULUS_LAND_MAX_AGE_NS
    ):
        raise InspectionError("precision-loss stimulus DataFlash schema is invalid")
    precision_us = _integer(precision.get("time_us"), "stimulus PL time")
    last_measurement_ms = _integer(
        precision.get("last_measurement_ms"), "stimulus PL last measurement"
    )
    mode_us = _integer(mode.get("time_us"), "stimulus MODE time")
    horizon_ns = _integer(pre.get("native_horizon_ns"), "stimulus native horizon")
    mission = pre.get("mission")
    assert isinstance(mission, Mapping)
    operation_start_ns = _integer(
        mission.get("native_start_ns"), "stimulus operation native start"
    )
    land_request_ns = _integer(
        mission.get("native_land_request_ns"), "stimulus LAND request native time"
    )
    if (
        precision.get("acquired") is not True
        or mode.get("mode") != 9
        or precision_us * 1_000 < operation_start_ns
        or mode_us * 1_000 <= land_request_ns
        or precision_us * 1_000 > horizon_ns
        or mode_us * 1_000 > horizon_ns
        or last_measurement_ms * 1_000_000 < operation_start_ns
        or last_measurement_ms * 1_000_000 > horizon_ns
        or horizon_ns - precision_us * 1_000 > STIMULUS_PRECISION_MAX_AGE_NS
        or horizon_ns - last_measurement_ms * 1_000_000
        > STIMULUS_PRECISION_MAX_AGE_NS
        or horizon_ns - mode_us * 1_000 > STIMULUS_LAND_MAX_AGE_NS
    ):
        raise InspectionError("precision-loss stimulus DataFlash snapshot is stale or mismatched")
    precision_matches = [
        row
        for row in rows
        if row.message_type == "PL"
        and row.time_us == precision_us
        and row.fields.get("TAcq") in {1, True}
        and row.fields.get("LastMeasMS") == last_measurement_ms
    ]
    mode_matches = [
        row
        for row in rows
        if row.message_type == "MODE"
        and row.time_us == mode_us
        and row.fields.get("Mode") == 9
        and row.fields.get("ModeNum") == 9
    ]
    if len(precision_matches) != 1 or len(mode_matches) != 1:
        raise InspectionError("precision-loss stimulus DataFlash snapshot is not in the final log")


def _check_precision_physical(
    evidence: Evidence, rows: Sequence[DataFlashRow]
) -> Finding:
    if (
        evidence.configuration.world != "moving_pad_stationary"
        or evidence.configuration.mission != "configured"
        or evidence.configuration.scenario != "moving_pad_v1"
    ):
        raise InspectionError("precision-loss requires the stationary configured precision run")
    stimulus = _stimulus(evidence)
    operation = _precision_operation(evidence)
    _reserved, accepted = _start_statuses(evidence)
    _validate_stimulus_mission_binding(evidence, stimulus, operation, accepted)
    _validate_stimulus_dataflash(stimulus, rows)
    pre = stimulus["pre_command"]
    post = stimulus["post_command"]
    assert isinstance(pre, Mapping) and isinstance(post, Mapping)
    horizon_ns = _integer(pre.get("native_horizon_ns"), "stimulus native horizon")
    first_ns, _first_position = _pose(post.get("first"), "first post-command")
    persistence = post.get("persistence")
    assert isinstance(persistence, list)
    final_ns = _pose(persistence[-1], "persistent post-command")[0]
    acquired = [
        row
        for row in rows
        if row.message_type == "PL"
        and row.fields.get("TAcq") in {1, True}
        and row.time_us * 1_000 <= horizon_ns
    ]
    lost = [
        row
        for row in rows
        if row.message_type == "PL"
        and row.fields.get("TAcq") in {0, False}
        and row.time_us * 1_000 >= first_ns
    ]
    if not acquired:
        raise InspectionError("DataFlash has no target acquisition before physical stimulus")
    if not lost or lost[0].time_us * 1_000 < final_ns:
        raise InspectionError("DataFlash has no persistent target loss after stimulus")
    loss_us = lost[0].time_us
    ranges = [
        row
        for row in rows
        if row.message_type == "RFND"
        and row.time_us <= loss_us
        and loss_us - row.time_us <= 1_000_000
    ]
    if not ranges or _field_number(ranges[-1], "Dist") <= 0.75:
        raise InspectionError("native range does not establish clearance above 0.75 m at loss")
    land_us = _native_us(evidence, operation.land)
    guided_us = _native_us(evidence, operation.guided)
    mode_rows = [row for row in rows if row.message_type == "MODE"]
    lands = [
        row
        for row in mode_rows
        if row.time_us > land_us
        and _field_number(row, "Mode") == 9.0
        and _field_number(row, "ModeNum") == 9.0
    ]
    guides = [
        row
        for row in mode_rows
        if row.time_us > guided_us
        and _field_number(row, "Mode") == 4.0
        and _field_number(row, "ModeNum") == 4.0
    ]
    if not lands or not guides or guides[0].time_us <= lands[0].time_us:
        raise InspectionError("DataFlash does not confirm mission LAND then policy GUIDED")
    return Finding(
        "passed",
        "manifest-bound stimulus, actual acquisition then loss, clearance above "
        "0.75 m, LAND then GUIDED, and exact policy failure agree",
    )


def _check_profile(
    evidence: Evidence,
    expectation: Expectation,
    rows: Sequence[DataFlashRow],
) -> Finding:
    if expectation == "ordinary-land":
        disable, restore = _ordinary_profile(evidence, rows)
        return Finding(
            "passed",
            "fresh full profile read and PLND_ENABLED disable/restore are verified "
            f"at public {disable.sim_timestamp:g}s/{restore.sim_timestamp:g}s; no "
            "other profile setting changed",
        )
    if expectation == "precision-loss":
        parameter_rows = [
            row
            for row in _after_native_epoch(evidence, rows)
            if row.message_type == "PARM"
        ]
        observed: dict[str, list[float]] = {"PLND_ENABLED": [], "PLND_STRICT": []}
        for row in parameter_rows:
            name = _parameter_name(row)
            if name in observed:
                observed[name].append(_field_number(row, "Value"))
        if observed["PLND_ENABLED"] != [1.0] or observed["PLND_STRICT"] != [2.0]:
            raise InspectionError("precision loss did not retain PLND_ENABLED=1 and PLND_STRICT=2")
        if any(
            item.parameter in {"PLND_ENABLED", "PLND_STRICT"} and item.action == "set"
            for item in _profile_transitions(evidence)
        ):
            raise InspectionError("precision-loss lifecycle contains a precision profile write")
        return Finding(
            "passed",
            "DataFlash retains PLND_ENABLED=1 and PLND_STRICT=2 with no lifecycle profile write",
        )
    raise InspectionError("profile inspection was requested for an unsupported expectation")


def _check_cleanup(evidence: Evidence) -> Finding:
    if evidence.manifest.get("incomplete_paths") != []:
        raise InspectionError("final manifest contains incomplete required artifacts")
    if not set(REQUIRED_ARTIFACT_PATHS).issubset(evidence.manifest_artifacts):
        raise InspectionError("final manifest omits required artifact records")
    try:
        with RuntimeProtocol(evidence.directory, evidence.run_id) as runtime:
            finalize = runtime.read_finalize_request()
            terminal = runtime.read_terminal_committed()
            manifest_status = runtime.read_manifest_status()
            quiescence = {
                module: runtime.read_quiescence(module)
                for module in QUIESCENCE_MODULES
            }
    except ProtocolError as error:
        raise InspectionError(f"finalization protocol evidence is invalid: {error}") from error
    if finalize is None:
        raise InspectionError("finalization request is missing")
    if terminal is None:
        raise InspectionError("terminal commit is missing")
    terminal_status = evidence.manifest.get("terminal_status")
    reason = evidence.manifest.get("reason")
    if (
        finalize.get("requested_terminal") != terminal_status
        or finalize.get("reason") != reason
        or terminal.get("terminal_status") != terminal_status
        or terminal.get("reason") != reason
        or terminal.get("manifest_path") != "manifest.json"
    ):
        raise InspectionError("finalize request, terminal commit, and manifest disagree")
    if manifest_status.get("complete") is not True:
        raise InspectionError("manifest status reports incomplete artifacts")
    missing_quiescence = [module for module, row in quiescence.items() if row is None]
    if missing_quiescence:
        raise InspectionError(
            "quiescence evidence is missing for " + ", ".join(missing_quiescence)
        )
    return Finding(
        "passed",
        "finalize request, six quiescence markers, manifest checksums, and terminal "
        "commit agree; host Compose inventory is outside this reader",
    )


def _not_required(expectation: Expectation, dimension: str) -> Finding:
    labels = {
        "physical": "physical flight",
        "profile": "landing-profile",
    }
    return Finding(
        "not_required",
        f"{expectation} does not require {labels.get(dimension, dimension)} proof",
    )


def _failed(detail: str) -> Finding:
    return Finding("failed", detail or "inspection failed without detail")


def inspect_qgc_control_run(
    run_directory: Path | str,
    expectation: str,
    *,
    tlog_connection_factory: ConnectionFactory = mavutil.mavlink_connection,
    dataflash_connection_factory: ConnectionFactory = mavutil.mavlink_connection,
) -> dict[str, Any]:
    """Return separate findings without changing the run directory."""
    if expectation not in EXPECTATIONS:
        raise ValueError(f"expectation must be one of {', '.join(EXPECTATIONS)}")
    selected: Expectation = expectation  # type: ignore[assignment]
    required = REQUIRED_DIMENSIONS[selected]
    findings: dict[str, Finding] = {
        dimension: _not_required(selected, dimension)
        for dimension in ("command", "physical", "profile", "cleanup")
        if dimension not in required
    }
    try:
        evidence = _load_evidence(run_directory)
    except (InspectionError, OSError) as error:
        for dimension in required:
            findings[dimension] = _failed(str(error))
        return _report(selected, None, findings)

    try:
        findings["command"] = _check_command(
            evidence, selected, tlog_connection_factory
        )
    except (InspectionError, OSError, ValueError) as error:
        findings["command"] = _failed(str(error))

    rows: tuple[DataFlashRow, ...] = ()
    if "physical" in required or "profile" in required:
        try:
            rows = _read_dataflash(evidence, dataflash_connection_factory)
        except (InspectionError, OSError, ValueError) as error:
            if "physical" in required:
                findings["physical"] = _failed(str(error))
            if "profile" in required:
                findings["profile"] = _failed(str(error))
    if "physical" in required and "physical" not in findings:
        try:
            if selected == "start":
                findings["physical"] = _check_start_physical(evidence, rows)
            elif selected == "abort":
                findings["physical"] = _check_abort_physical(evidence, rows)
            elif selected == "ordinary-land":
                findings["physical"] = _check_ordinary_physical(evidence, rows)
            elif selected == "precision-loss":
                findings["physical"] = _check_precision_physical(evidence, rows)
            else:
                raise InspectionError("physical inspection is unavailable")
        except (InspectionError, OSError, ValueError) as error:
            findings["physical"] = _failed(str(error))
    if "profile" in required and "profile" not in findings:
        try:
            findings["profile"] = _check_profile(evidence, selected, rows)
        except (InspectionError, OSError, ValueError) as error:
            findings["profile"] = _failed(str(error))
    try:
        findings["cleanup"] = _check_cleanup(evidence)
    except (InspectionError, OSError, ValueError) as error:
        findings["cleanup"] = _failed(str(error))
    return _report(selected, evidence.run_id, findings)


def _report(
    expectation: Expectation,
    run_id: str | None,
    findings: Mapping[str, Finding],
) -> dict[str, Any]:
    ordered = {
        dimension: findings[dimension].to_dict()
        for dimension in ("command", "physical", "profile", "cleanup")
    }
    accepted = all(
        ordered[dimension]["status"] == "passed"
        for dimension in REQUIRED_DIMENSIONS[expectation]
    )
    return {
        "accepted": accepted,
        "expectation": expectation,
        "run_id": run_id,
        "findings": ordered,
        "limitations": [
            "native QGC file selection and clicks require a separate operator record",
            "video, rosbag, score, source/image provenance, and live Compose "
            "resources require the existing full acceptance workflow",
        ],
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--expect", choices=EXPECTATIONS, required=True)
    arguments = parser.parse_args(argv)
    try:
        report = inspect_qgc_control_run(
            arguments.run_directory,
            arguments.expect,
        )
    except Exception as error:
        report = {
            "accepted": False,
            "expectation": arguments.expect,
            "run_id": None,
            "findings": {
                dimension: _failed(str(error)).to_dict()
                for dimension in ("command", "physical", "profile", "cleanup")
            },
            "limitations": [],
        }
    print(json.dumps(report, sort_keys=True, separators=(",", ":")))
    return 0 if report["accepted"] is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
