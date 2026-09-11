"""Prepare one offline QGC action set and its listener session.

This module only reads and writes local files. It has no aircraft transport.
"""

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import uuid

from drone.control.mission_supervisor import (
    AttemptLedger,
    LedgerError,
    MAX_ATTEMPT_ID,
    MIN_ATTEMPT_ID,
    _atomic_write,
    _json_bytes,
    initialize_ledger,
)

SCHEMA_VERSION = 1
SUPPORTED_MAVLINK_DIALECTS = frozenset(("ardupilotmega",))
SUPPORTED_MAVLINK_WIRE_PROTOCOLS = frozenset(("1.0", "2.0"))
FIRMWARE_IDENTIFIER = re.compile(
    r"^ArduCopter (?:\d+\.\d+\.\d+(?:[-+][A-Za-z0-9][A-Za-z0-9.+_-]*)?"
    r"|build (?:[0-9a-fA-F]{7,40}|sha256:[0-9a-fA-F]{64}))$"
)
QGC_IDENTIFIER = re.compile(
    r"^QGroundControl (?:\d+\.\d+\.\d+(?:[-+][A-Za-z0-9][A-Za-z0-9.+_-]*)?"
    r"|build (?:[0-9a-fA-F]{7,40}|sha256:[0-9a-fA-F]{64}))$"
)

COMMANDS = {
    "FM1": 31000,
    "FM2": 31001,
    "FM3": 31002,
    "UPDATE_WA": 31003,
    "UPDATE_WM1": 31004,
    "UPDATE_WM2": 31005,
    "UPDATE_WM3": 31006,
    "UPDATE_WM4": 31007,
    "UPDATE_WM5": 31008,
    "UPDATE_WM6": 31009,
    "UPDATE_L": 31010,
    "UPDATE_TARGET": 31011,
    "CLEAR_PICKUPS": 31012,
    "CLEAR_ALL": 31013,
    "ABORT_AND_RECOVER": 31015,
}

REQUIRED_GATES = frozenset(
    (
        "identity",
        "protocol",
        "firmware",
        "qgc_fixed_params",
        "qgc_retry_behavior",
        "rc_mapping_and_freshness",
        "command_registry",
        "telemetry_rates",
        "sensor_configuration",
        "positive_preflight_evidence",
        "fc_failsafe",
        "rc_precedence",
    )
)


class PreparationError(RuntimeError):
    pass


def _strict_json_bytes(content: bytes, *, label: str) -> object:
    def reject_constant(value: str) -> None:
        raise PreparationError(f"{label} contains non-standard number {value}")

    def unique_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
        result: dict[str, object] = {}
        for key, value in pairs:
            if key in result:
                raise PreparationError(f"{label} contains duplicate key {key!r}")
            result[key] = value
        return result

    try:
        return json.loads(
            content.decode("utf-8"),
            parse_constant=reject_constant,
            object_pairs_hook=unique_object,
        )
    except (UnicodeError, json.JSONDecodeError) as error:
        raise PreparationError(f"{label} is unreadable: {error}") from error


@dataclass(frozen=True)
class DeploymentProfile:
    profile_id: str
    source_system: int
    source_component: int
    target_system: int
    target_component: int
    flight_controller_system: int
    flight_controller_component: int
    mavlink_dialect: str
    mavlink_wire_protocol: str
    firmware: str
    qgc_version: str
    raw_sha256: str
    raw_bytes: bytes


@dataclass(frozen=True)
class PreparedAttempt:
    attempt_id: int
    generation: str
    profile_id: str
    actions_sha256: str


def _required_mapping(data: object, field: str) -> dict:
    if not isinstance(data, dict):
        raise PreparationError(f"deployment profile {field} must be an object")
    return data


def _required_text(data: dict, field: str, context: str) -> str:
    value = data.get(field)
    if not isinstance(value, str) or not value.strip():
        raise PreparationError(f"deployment profile {context}.{field} is required")
    lowered = value.strip().lower()
    if lowered in {"n/a", "na", "none", "pending", "unspecified"} or any(
        marker in lowered for marker in ("unknown", "unverified", "tbd")
    ):
        raise PreparationError(f"deployment profile {context}.{field} is unverified")
    return value.strip()


def _identity_value(identity: dict, field: str) -> int:
    value = identity.get(field)
    if not isinstance(value, int) or isinstance(value, bool) or not 1 <= value <= 255:
        raise PreparationError(f"deployment profile identity.{field} is invalid")
    return value


def _supported_software(software: dict) -> tuple[str, str, str, str]:
    dialect = _required_text(software, "mavlink_dialect", "software")
    if dialect not in SUPPORTED_MAVLINK_DIALECTS:
        raise PreparationError(f"unsupported MAVLink dialect: {dialect}")
    wire_protocol = _required_text(
        software, "mavlink_wire_protocol", "software"
    )
    if wire_protocol not in SUPPORTED_MAVLINK_WIRE_PROTOCOLS:
        raise PreparationError(f"unsupported MAVLink wire protocol: {wire_protocol}")
    if wire_protocol != "2.0":
        raise PreparationError("flight-enabled profiles require MAVLink 2")
    firmware = _required_text(software, "firmware", "software")
    if FIRMWARE_IDENTIFIER.fullmatch(firmware) is None:
        raise PreparationError(
            "deployment profile software.firmware must name a concrete "
            "ArduCopter release or build"
        )
    qgc_version = _required_text(software, "qgc_version", "software")
    if QGC_IDENTIFIER.fullmatch(qgc_version) is None:
        raise PreparationError(
            "deployment profile software.qgc_version must name a concrete "
            "QGroundControl release or build"
        )
    return dialect, wire_protocol, firmware, qgc_version


def load_deployment_profile(path: os.PathLike[str] | str) -> DeploymentProfile:
    profile_path = Path(path)
    try:
        raw = profile_path.read_bytes()
    except OSError as error:
        raise PreparationError(f"deployment profile is unreadable: {error}") from error
    return load_deployment_profile_bytes(raw)


def load_deployment_profile_bytes(raw: bytes) -> DeploymentProfile:
    """Validate one immutable deployment-profile byte snapshot."""
    if not isinstance(raw, bytes):
        raise PreparationError("deployment profile bytes must be immutable")
    data = _strict_json_bytes(raw, label="deployment profile")
    if (
        not isinstance(data, dict)
        or type(data.get("schema_version")) is not int
        or data["schema_version"] != SCHEMA_VERSION
    ):
        raise PreparationError("deployment profile schema version is unsupported")
    if data.get("verified_for_live_use") is not True:
        raise PreparationError("deployment profile is not verified for live use")

    profile_id = _required_text(data, "profile_id", "root")
    identity = _required_mapping(data.get("identity"), "identity")
    software = _required_mapping(data.get("software"), "software")
    rc = _required_mapping(data.get("rc"), "rc")
    registry = _required_mapping(data.get("command_registry"), "command_registry")
    gates = _required_mapping(data.get("safety_gates"), "safety_gates")

    missing_gates = REQUIRED_GATES.difference(gates)
    if missing_gates:
        raise PreparationError(
            f"deployment profile is missing safety gates: {', '.join(sorted(missing_gates))}"
        )
    open_gates = sorted(gate for gate in REQUIRED_GATES if gates.get(gate) is not True)
    if open_gates:
        raise PreparationError(
            f"deployment profile safety gates are not verified: {', '.join(open_gates)}"
        )

    if registry.get("commands") != COMMANDS:
        raise PreparationError("deployment profile command registry does not match")
    if registry.get("abort_31015_available") is not True:
        raise PreparationError("deployment registry has not cleared command 31015")
    evidence = registry.get("collision_check_evidence")
    if not isinstance(evidence, str) or re.fullmatch(r"sha256:[0-9a-f]{64}", evidence) is None:
        raise PreparationError("command 31015 collision-check evidence is missing")

    protocol = _required_text(rc, "protocol", "rc")
    channel = rc.get("mode_channel")
    freshness = rc.get("freshness_seconds")
    mapping = rc.get("mode_mapping")
    if not isinstance(channel, int) or isinstance(channel, bool) or not 1 <= channel <= 18:
        raise PreparationError("deployment profile rc.mode_channel is invalid")
    if (
        not isinstance(freshness, (int, float))
        or isinstance(freshness, bool)
        or not math.isfinite(freshness)
        or freshness <= 0
    ):
        raise PreparationError("deployment profile rc.freshness_seconds is invalid")
    if not isinstance(mapping, dict) or not {"STABILIZE", "GUIDED", "LOITER"}.issubset(mapping):
        raise PreparationError("deployment profile rc.mode_mapping is incomplete")
    for mode, bounds in mapping.items():
        if (
            not isinstance(mode, str)
            or not isinstance(bounds, list)
            or len(bounds) != 2
            or not all(isinstance(value, int) and not isinstance(value, bool) for value in bounds)
            or bounds[0] >= bounds[1]
        ):
            raise PreparationError(f"deployment profile RC mapping for {mode!r} is invalid")

    dialect, wire_protocol, firmware, qgc_version = _supported_software(software)

    profile = DeploymentProfile(
        profile_id=profile_id,
        source_system=_identity_value(identity, "source_system"),
        source_component=_identity_value(identity, "source_component"),
        target_system=_identity_value(identity, "target_system"),
        target_component=_identity_value(identity, "target_component"),
        flight_controller_system=_identity_value(
            identity, "flight_controller_system"
        ),
        flight_controller_component=_identity_value(
            identity, "flight_controller_component"
        ),
        mavlink_dialect=dialect,
        mavlink_wire_protocol=wire_protocol,
        firmware=firmware,
        qgc_version=qgc_version,
        raw_sha256=hashlib.sha256(raw).hexdigest(),
        raw_bytes=raw,
    )
    if profile.target_system != profile.flight_controller_system:
        raise PreparationError(
            "companion target system must match the flight controller system"
        )
    if profile.target_component == profile.flight_controller_component:
        raise PreparationError(
            "companion and flight controller components must be distinct"
        )
    if (profile.source_system, profile.source_component) == (
        profile.flight_controller_system,
        profile.flight_controller_component,
    ):
        raise PreparationError("QGC and flight controller source identities are conflicting")
    if (profile.source_system, profile.source_component) == (
        profile.target_system,
        profile.target_component,
    ):
        raise PreparationError("QGC source and companion target identities are conflicting")

    from .flight_profile import validate_observation_profile

    validate_observation_profile(profile, data)
    return profile


def _action_label(name: str) -> str:
    return name.replace("_", " ").title()


def _build_actions(profile: DeploymentProfile, attempt_id: int, generation: str) -> dict:
    actions = []
    for name, command in COMMANDS.items():
        action = {
            "label": _action_label(name),
            "description": (
                f"Attempt {attempt_id}; generation {generation}; requires active "
                f"target system {profile.target_system}, component "
                f"{profile.target_component}."
            ),
            "mavCmd": command,
            "compId": profile.target_component,
        }
        for index in range(1, 8):
            action[f"param{index}"] = attempt_id if index == 1 else 0
        actions.append(action)
    return {
        "version": 1,
        "fileType": "MavlinkActions",
        "title": f"RPi FM attempt {attempt_id} [{generation}]",
        "actions": actions,
    }


def prepare_attempt(
    profile_path: os.PathLike[str] | str,
    ledger_path: os.PathLike[str] | str,
    session_path: os.PathLike[str] | str,
    actions_path: os.PathLike[str] | str,
    *,
    acknowledge_on_ground: bool,
) -> PreparedAttempt:
    if acknowledge_on_ground is not True:
        raise PreparationError("explicit ground-preparation acknowledgement is required")
    profile_file = Path(profile_path)
    ledger_file = Path(ledger_path).resolve()
    session_file = Path(session_path)
    actions_file = Path(actions_path)
    files = tuple(
        path.resolve() for path in (profile_file, ledger_file, session_file, actions_file)
    )
    lock_file = ledger_file.with_name(f"{ledger_file.name}.lock")
    if lock_file.is_symlink() or len(set(files)) != len(files) or lock_file in files:
        raise PreparationError(
            "profile, ledger, lock, session, and action paths must be distinct"
        )
    profile = load_deployment_profile(profile_file)

    with AttemptLedger(ledger_file).prepare_next() as attempt_id:
        generation = uuid.uuid4().hex
        action_bytes = _json_bytes(_build_actions(profile, attempt_id, generation))
        action_hash = hashlib.sha256(action_bytes).hexdigest()
        session = {
            "schema_version": SCHEMA_VERSION,
            "file_type": "CompanionAttemptSession",
            "generation": generation,
            "attempt_id": attempt_id,
            "profile_id": profile.profile_id,
            "profile_sha256": profile.raw_sha256,
            "actions_sha256": action_hash,
            "required_source": {
                "system": profile.source_system,
                "component": profile.source_component,
            },
            "required_target": {
                "system": profile.target_system,
                "component": profile.target_component,
            },
            "mavlink_dialect": profile.mavlink_dialect,
            "mavlink_wire_protocol": profile.mavlink_wire_protocol,
            "firmware": profile.firmware,
            "qgc_version": profile.qgc_version,
        }
        try:
            _atomic_write(actions_file, action_bytes)
            _atomic_write(session_file, _json_bytes(session))
        except OSError as error:
            raise PreparationError(f"could not persist prepared files: {error}") from error

    return PreparedAttempt(
        attempt_id=attempt_id,
        generation=generation,
        profile_id=profile.profile_id,
        actions_sha256=action_hash,
    )


def load_prepared_session(
    session_path: os.PathLike[str] | str,
    actions_path: os.PathLike[str] | str,
    profile_path: os.PathLike[str] | str,
    *,
    ledger_path: os.PathLike[str] | str | None = None,
) -> PreparedAttempt:
    profile = load_deployment_profile(profile_path)
    try:
        session_bytes = Path(session_path).read_bytes()
        action_bytes = Path(actions_path).read_bytes()
    except OSError as error:
        raise PreparationError(f"prepared files are unreadable: {error}") from error
    return load_prepared_session_bytes(
        session_bytes,
        action_bytes,
        profile,
        ledger_path=ledger_path,
    )


def load_prepared_session_bytes(
    session_bytes: bytes,
    action_bytes: bytes,
    profile: DeploymentProfile,
    *,
    ledger_path: os.PathLike[str] | str | None = None,
) -> PreparedAttempt:
    """Validate one immutable prepared-session and action byte snapshot."""
    if not isinstance(session_bytes, bytes) or not isinstance(action_bytes, bytes):
        raise PreparationError("prepared file bytes must be immutable")
    if not isinstance(profile, DeploymentProfile):
        raise PreparationError("deployment profile must be validated")
    session = _strict_json_bytes(session_bytes, label="prepared session")
    actions = _strict_json_bytes(action_bytes, label="prepared actions")
    if not isinstance(session, dict) or not isinstance(actions, dict):
        raise PreparationError("prepared file schema is invalid")
    action_hash = hashlib.sha256(action_bytes).hexdigest()
    if session.get("actions_sha256") != action_hash:
        raise PreparationError("prepared action hash does not match the session")
    generation = session.get("generation")
    attempt_id = session.get("attempt_id")
    if (
        not isinstance(generation, str)
        or not generation
        or not isinstance(actions.get("title"), str)
        or generation not in actions["title"]
    ):
        raise PreparationError("prepared action generation does not match the session")
    if (
        not isinstance(attempt_id, int)
        or isinstance(attempt_id, bool)
        or not MIN_ATTEMPT_ID <= attempt_id <= MAX_ATTEMPT_ID
    ):
        raise PreparationError("prepared attempt ID is invalid")
    if session.get("profile_id") != profile.profile_id or session.get(
        "profile_sha256"
    ) != profile.raw_sha256:
        raise PreparationError("prepared session does not match the deployment profile")
    if (
        type(session.get("schema_version")) is not int
        or session["schema_version"] != SCHEMA_VERSION
        or session.get("file_type") != "CompanionAttemptSession"
    ):
        raise PreparationError("prepared session schema is invalid")
    expected_session_fields = {
        "required_source": {
            "system": profile.source_system,
            "component": profile.source_component,
        },
        "required_target": {
            "system": profile.target_system,
            "component": profile.target_component,
        },
        "mavlink_dialect": profile.mavlink_dialect,
        "mavlink_wire_protocol": profile.mavlink_wire_protocol,
        "firmware": profile.firmware,
        "qgc_version": profile.qgc_version,
    }
    for identity_field in ("required_source", "required_target"):
        identity = session.get(identity_field)
        if (
            not isinstance(identity, dict)
            or set(identity) != {"system", "component"}
            or type(identity.get("system")) is not int
            or type(identity.get("component")) is not int
        ):
            raise PreparationError("prepared session identity fields are invalid")
    if any(session.get(field) != value for field, value in expected_session_fields.items()):
        raise PreparationError("prepared session deployment facts do not match the profile")
    if type(actions.get("version")) is not int or actions.get(
        "version"
    ) != 1 or actions.get("fileType") != "MavlinkActions":
        raise PreparationError("prepared action file is not QGC MavlinkActions version 1")
    action_list = actions.get("actions")
    if not isinstance(action_list, list) or len(action_list) != len(COMMANDS):
        raise PreparationError("prepared action command set is incomplete")
    for action, command in zip(action_list, COMMANDS.values(), strict=True):
        if (
            not isinstance(action, dict)
            or type(action.get("mavCmd")) is not int
            or action["mavCmd"] != command
            or type(action.get("compId")) is not int
            or action["compId"] != profile.target_component
            or type(action.get("param1")) is not int
            or action["param1"] != attempt_id
            or any(
                type(action.get(f"param{index}")) is not int
                or action[f"param{index}"] != 0
                for index in range(2, 8)
            )
        ):
            raise PreparationError("prepared action command mapping is invalid")

    if ledger_path is not None:
        try:
            AttemptLedger(ledger_path).require_current_unconsumed(attempt_id)
        except LedgerError as error:
            raise PreparationError(str(error)) from error

    return PreparedAttempt(
        attempt_id=attempt_id,
        generation=generation,
        profile_id=profile.profile_id,
        actions_sha256=action_hash,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="operation", required=True)
    initialize = subparsers.add_parser("initialize-ledger")
    initialize.add_argument("ledger")
    prepare = subparsers.add_parser("prepare")
    prepare.add_argument("--profile", required=True)
    prepare.add_argument("--ledger", required=True)
    prepare.add_argument("--session", required=True)
    prepare.add_argument("--actions", required=True)
    prepare.add_argument("--acknowledge-on-ground", action="store_true")
    return parser


def main() -> int:
    arguments = _parser().parse_args()
    if arguments.operation == "initialize-ledger":
        initialize_ledger(arguments.ledger)
        return 0
    prepared = prepare_attempt(
        arguments.profile,
        arguments.ledger,
        arguments.session,
        arguments.actions,
        acknowledge_on_ground=arguments.acknowledge_on_ground,
    )
    print(json.dumps(prepared.__dict__, sort_keys=True))
    return 0
