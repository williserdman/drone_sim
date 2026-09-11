"""Strict offline parser for caller-supplied simulator QGC policy."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat
from typing import Any


_SHA256 = re.compile(r"[0-9a-f]{64}\Z")
_CUSTOM_VERSION = re.compile(r"[0-9a-f]{16}\Z")
_ARDUPILOT_COMMIT = "2a3dc4b7bf2507120f7378a7b2fde73185e0c325"


def _exact_fields(value: object, expected: set[str], name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be an object")
    actual = set(value)
    if actual != expected:
        missing = sorted(expected - actual)
        unknown = sorted(actual - expected)
        raise ValueError(
            f"{name} fields do not match: missing={missing}, unknown={unknown}"
        )
    return value


def _finite_number(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    normalized = float(value)
    if not math.isfinite(normalized):
        raise ValueError(f"{name} must be a finite number")
    return normalized


def _positive_number(name: str, value: object) -> float:
    normalized = _finite_number(name, value)
    if normalized <= 0:
        raise ValueError(f"{name} must be positive")
    return normalized


def _nonnegative_number(name: str, value: object) -> float:
    normalized = _finite_number(name, value)
    if normalized < 0:
        raise ValueError(f"{name} must be non-negative")
    return normalized


def _integer(name: str, value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")
    return value


def _literal(name: str, value: object, expected: object) -> object:
    if value != expected or type(value) is not type(expected):
        raise ValueError(f"{name} must equal {expected!r}")
    return value


def _sha256(name: str, value: object) -> str:
    if not isinstance(value, str) or _SHA256.fullmatch(value) is None:
        raise ValueError(f"{name} must be a lowercase SHA-256")
    return value


def _vector3(name: str, value: object) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{name} must contain exactly three numbers")
    result = tuple(
        _finite_number(f"{name}[{index}]", item) for index, item in enumerate(value)
    )
    return result  # type: ignore[return-value]


@dataclass(frozen=True)
class QGCBindings:
    course_sha256: str
    scenario_sha256: str
    ardupilot_commit: str


@dataclass(frozen=True)
class SimulatorLaunchOrigin:
    latitude_deg: float
    longitude_deg: float
    amsl_m: float
    heading_deg: float


@dataclass(frozen=True)
class QGCConnectionPolicy:
    wait_ready: bool
    heartbeat_timeout_s: float


@dataclass(frozen=True)
class OperatingSitePolicy:
    waypoint_tolerance_m: float
    home_position_tolerance_m: float
    home_altitude_tolerance_m: float
    minimum_recovery_agl_m: float
    maximum_recovery_agl_m: float
    operations: tuple[str, str]
    evidence_reference: str
    recovery_corridor_evidence: str


@dataclass(frozen=True)
class RecoveryPolicy:
    timeout_s: float
    local_land_reserve_s: float


@dataclass(frozen=True)
class TelemetryStartupPolicy:
    command_ack_timeout_s: float
    collection_timeout_s: float
    home_request_timeout_s: float
    poll_interval_s: float
    minimum_distinct_samples: int
    maximum_interval_error_fraction: float


@dataclass(frozen=True)
class AutopilotVersionPolicy:
    firmware_label: str
    flight_sw_version: int
    flight_custom_version: str
    evidence_reference: str

    @property
    def flight_custom_version_bytes(self) -> bytes:
        return bytes.fromhex(self.flight_custom_version)


@dataclass(frozen=True)
class ClearanceCalibrationPolicy:
    beam_direction_body_frd: tuple[float, float, float]
    measured_reference_offset_body_frd_m: tuple[float, float, float]
    lidar_mounting_offset_already_applied: bool
    max_tilt_rad: float
    max_age_seconds: float
    max_skew_seconds: float
    locally_horizontal_planar_surface: bool


@dataclass(frozen=True)
class ReleaseStabilityPolicy:
    hold_seconds: float
    timeout_seconds: float
    max_horizontal_speed_m_s: float
    max_vertical_speed_m_s: float
    max_roll_rad: float
    max_pitch_rad: float
    horizontal_position_tolerance_m: float
    vertical_position_tolerance_m: float
    max_observation_skew_seconds: float
    max_observation_gap_seconds: float
    poll_interval_seconds: float
    waypoint_reissue_interval_seconds: float


@dataclass(frozen=True)
class QGCRuntimePolicy:
    schema_version: int
    purpose: str
    backend: str
    bindings: QGCBindings
    simulator_launch_origin: SimulatorLaunchOrigin
    connection: QGCConnectionPolicy
    operating_site: OperatingSitePolicy
    recovery: RecoveryPolicy
    telemetry_startup: TelemetryStartupPolicy
    autopilot_version: AutopilotVersionPolicy
    clearance_calibration: ClearanceCalibrationPolicy
    release_stability: ReleaseStabilityPolicy
    fc_home_position_tolerance_m: float
    fc_home_altitude_tolerance_m: float
    idle_poll_s: float
    cleanup_timeout_s: float
    payload_delay_wall_timeout_s: float
    enabled_phases: tuple[int, int]


def _parse_bindings(raw: object) -> QGCBindings:
    value = _exact_fields(
        raw, {"course_sha256", "scenario_sha256", "ardupilot_commit"}, "bindings"
    )
    commit = value["ardupilot_commit"]
    _literal("bindings.ardupilot_commit", commit, _ARDUPILOT_COMMIT)
    return QGCBindings(
        course_sha256=_sha256("bindings.course_sha256", value["course_sha256"]),
        scenario_sha256=_sha256("bindings.scenario_sha256", value["scenario_sha256"]),
        ardupilot_commit=commit,
    )


def _parse_origin(raw: object) -> SimulatorLaunchOrigin:
    value = _exact_fields(
        raw,
        {"latitude_deg", "longitude_deg", "amsl_m", "heading_deg"},
        "simulator_launch_origin",
    )
    latitude = _finite_number("simulator_launch_origin.latitude_deg", value["latitude_deg"])
    longitude = _finite_number(
        "simulator_launch_origin.longitude_deg", value["longitude_deg"]
    )
    heading = _finite_number("simulator_launch_origin.heading_deg", value["heading_deg"])
    if not -90 <= latitude <= 90:
        raise ValueError("simulator launch latitude is outside [-90, 90]")
    if not -180 <= longitude <= 180:
        raise ValueError("simulator launch longitude is outside [-180, 180]")
    if not 0 <= heading < 360:
        raise ValueError("simulator launch heading is outside [0, 360)")
    return SimulatorLaunchOrigin(
        latitude, longitude, _finite_number("simulator_launch_origin.amsl_m", value["amsl_m"]), heading
    )


def _parse_connection(raw: object) -> QGCConnectionPolicy:
    value = _exact_fields(raw, {"wait_ready", "heartbeat_timeout_s"}, "connection")
    wait_ready = value["wait_ready"]
    if not isinstance(wait_ready, bool):
        raise ValueError("connection.wait_ready must be a Boolean")
    return QGCConnectionPolicy(
        wait_ready,
        _positive_number("connection.heartbeat_timeout_s", value["heartbeat_timeout_s"]),
    )


def _parse_operating_site(raw: object) -> OperatingSitePolicy:
    value = _exact_fields(
        raw,
        {
            "waypoint_tolerance_m",
            "home_position_tolerance_m",
            "home_altitude_tolerance_m",
            "minimum_recovery_agl_m",
            "maximum_recovery_agl_m",
            "operations",
            "evidence_reference",
            "recovery_corridor_evidence",
        },
        "operating_site",
    )
    minimum = _nonnegative_number(
        "operating_site.minimum_recovery_agl_m", value["minimum_recovery_agl_m"]
    )
    maximum = _positive_number(
        "operating_site.maximum_recovery_agl_m", value["maximum_recovery_agl_m"]
    )
    if maximum <= minimum:
        raise ValueError("maximum recovery AGL must be above minimum recovery AGL")
    operations = value["operations"]
    if operations != ["RETURN", "LOCAL_LAND"]:
        raise ValueError("operating_site.operations must be ['RETURN', 'LOCAL_LAND']")
    return OperatingSitePolicy(
        waypoint_tolerance_m=_positive_number(
            "operating_site.waypoint_tolerance_m", value["waypoint_tolerance_m"]
        ),
        home_position_tolerance_m=_positive_number(
            "operating_site.home_position_tolerance_m", value["home_position_tolerance_m"]
        ),
        home_altitude_tolerance_m=_positive_number(
            "operating_site.home_altitude_tolerance_m", value["home_altitude_tolerance_m"]
        ),
        minimum_recovery_agl_m=minimum,
        maximum_recovery_agl_m=maximum,
        operations=("RETURN", "LOCAL_LAND"),
        evidence_reference=_sha256(
            "operating_site.evidence_reference", value["evidence_reference"]
        ),
        recovery_corridor_evidence=_sha256(
            "operating_site.recovery_corridor_evidence",
            value["recovery_corridor_evidence"],
        ),
    )


def _parse_recovery(raw: object) -> RecoveryPolicy:
    value = _exact_fields(raw, {"timeout_s", "local_land_reserve_s"}, "recovery")
    timeout = _positive_number("recovery.timeout_s", value["timeout_s"])
    reserve = _positive_number(
        "recovery.local_land_reserve_s", value["local_land_reserve_s"]
    )
    if reserve >= timeout:
        raise ValueError("recovery local-LAND reserve must be less than timeout")
    return RecoveryPolicy(timeout, reserve)


def _parse_telemetry(raw: object) -> TelemetryStartupPolicy:
    names = {
        "command_ack_timeout_s",
        "collection_timeout_s",
        "home_request_timeout_s",
        "poll_interval_s",
        "minimum_distinct_samples",
        "maximum_interval_error_fraction",
    }
    value = _exact_fields(raw, names, "telemetry_startup")
    command = _positive_number(
        "telemetry_startup.command_ack_timeout_s", value["command_ack_timeout_s"]
    )
    collection = _positive_number(
        "telemetry_startup.collection_timeout_s", value["collection_timeout_s"]
    )
    home = _positive_number(
        "telemetry_startup.home_request_timeout_s", value["home_request_timeout_s"]
    )
    poll = _positive_number("telemetry_startup.poll_interval_s", value["poll_interval_s"])
    minimum = _integer(
        "telemetry_startup.minimum_distinct_samples", value["minimum_distinct_samples"]
    )
    error_fraction = _positive_number(
        "telemetry_startup.maximum_interval_error_fraction",
        value["maximum_interval_error_fraction"],
    )
    if minimum < 2:
        raise ValueError("minimum_distinct_samples must be at least two")
    if error_fraction > 1:
        raise ValueError("maximum_interval_error_fraction must not exceed one")
    if poll > min(command, collection, home):
        raise ValueError("telemetry poll interval exceeds a timeout")
    return TelemetryStartupPolicy(command, collection, home, poll, minimum, error_fraction)


def _parse_autopilot(raw: object) -> AutopilotVersionPolicy:
    value = _exact_fields(
        raw,
        {
            "firmware_label",
            "flight_sw_version",
            "flight_custom_version",
            "evidence_reference",
        },
        "autopilot_version",
    )
    _literal("autopilot_version.firmware_label", value["firmware_label"], "ArduCopter 4.5.7")
    version = _integer("autopilot_version.flight_sw_version", value["flight_sw_version"])
    if version != 0x040507FF:
        raise ValueError("autopilot version must be packed stable ArduCopter 4.5.7")
    custom = value["flight_custom_version"]
    if not isinstance(custom, str) or _CUSTOM_VERSION.fullmatch(custom) is None:
        raise ValueError("flight_custom_version must be exactly 16 lowercase hex characters")
    return AutopilotVersionPolicy(
        firmware_label="ArduCopter 4.5.7",
        flight_sw_version=version,
        flight_custom_version=custom,
        evidence_reference=_sha256(
            "autopilot_version.evidence_reference", value["evidence_reference"]
        ),
    )


def _parse_clearance(raw: object) -> ClearanceCalibrationPolicy:
    value = _exact_fields(
        raw,
        {
            "beam_direction_body_frd",
            "measured_reference_offset_body_frd_m",
            "lidar_mounting_offset_already_applied",
            "max_tilt_rad",
            "max_age_seconds",
            "max_skew_seconds",
            "locally_horizontal_planar_surface",
        },
        "clearance_calibration",
    )
    beam = _vector3(
        "clearance_calibration.beam_direction_body_frd",
        value["beam_direction_body_frd"],
    )
    if not math.isclose(
        math.sqrt(sum(component * component for component in beam)),
        1.0,
        rel_tol=1e-9,
        abs_tol=1e-9,
    ):
        raise ValueError("clearance beam must be a unit vector")
    if beam[2] <= 0:
        raise ValueError("clearance beam must point into the body-down hemisphere")
    for name in (
        "lidar_mounting_offset_already_applied",
        "locally_horizontal_planar_surface",
    ):
        if value[name] is not True:
            raise ValueError(f"clearance_calibration.{name} must be literal true")
    tilt = _positive_number("clearance_calibration.max_tilt_rad", value["max_tilt_rad"])
    if tilt >= math.pi / 2:
        raise ValueError("clearance max tilt must be less than pi/2")
    return ClearanceCalibrationPolicy(
        beam_direction_body_frd=beam,
        measured_reference_offset_body_frd_m=_vector3(
            "clearance_calibration.measured_reference_offset_body_frd_m",
            value["measured_reference_offset_body_frd_m"],
        ),
        lidar_mounting_offset_already_applied=True,
        max_tilt_rad=tilt,
        max_age_seconds=_positive_number(
            "clearance_calibration.max_age_seconds", value["max_age_seconds"]
        ),
        max_skew_seconds=_positive_number(
            "clearance_calibration.max_skew_seconds", value["max_skew_seconds"]
        ),
        locally_horizontal_planar_surface=True,
    )


def _parse_release_stability(raw: object) -> ReleaseStabilityPolicy:
    names = {
        "hold_seconds",
        "timeout_seconds",
        "max_horizontal_speed_m_s",
        "max_vertical_speed_m_s",
        "max_roll_rad",
        "max_pitch_rad",
        "horizontal_position_tolerance_m",
        "vertical_position_tolerance_m",
        "max_observation_skew_seconds",
        "max_observation_gap_seconds",
        "poll_interval_seconds",
        "waypoint_reissue_interval_seconds",
    }
    value = _exact_fields(raw, names, "release_stability")
    normalized = {
        name: _positive_number(f"release_stability.{name}", value[name])
        for name in names
    }
    if normalized["timeout_seconds"] <= normalized["hold_seconds"]:
        raise ValueError("release timeout must be greater than hold")
    if normalized["poll_interval_seconds"] > normalized["max_observation_gap_seconds"]:
        raise ValueError("release poll interval must not exceed maximum observation gap")
    if normalized["max_roll_rad"] >= math.pi / 2:
        raise ValueError("release max roll must be less than pi/2")
    if normalized["max_pitch_rad"] >= math.pi / 2:
        raise ValueError("release max pitch must be less than pi/2")
    return ReleaseStabilityPolicy(**normalized)


_TOP_LEVEL_FIELDS = {
    "schema_version",
    "purpose",
    "backend",
    "bindings",
    "simulator_launch_origin",
    "connection",
    "operating_site",
    "recovery",
    "telemetry_startup",
    "autopilot_version",
    "clearance_calibration",
    "release_stability",
    "fc_home_position_tolerance_m",
    "fc_home_altitude_tolerance_m",
    "idle_poll_s",
    "cleanup_timeout_s",
    "payload_delay_wall_timeout_s",
    "enabled_phases",
}


def _parse_policy(raw: object) -> QGCRuntimePolicy:
    value = _exact_fields(raw, _TOP_LEVEL_FIELDS, "runtime policy")
    _literal("schema_version", value["schema_version"], 1)
    _literal("purpose", value["purpose"], "drone-sim-comp2026-fm1-fm2")
    _literal("backend", value["backend"], "drone-sim-ros-confirmed-v1")
    enabled_phases = value["enabled_phases"]
    if (
        not isinstance(enabled_phases, list)
        or len(enabled_phases) != 2
        or any(type(phase) is not int for phase in enabled_phases)
        or enabled_phases != [31000, 31001]
    ):
        raise ValueError("enabled_phases must be exactly [31000, 31001]")
    return QGCRuntimePolicy(
        schema_version=1,
        purpose="drone-sim-comp2026-fm1-fm2",
        backend="drone-sim-ros-confirmed-v1",
        bindings=_parse_bindings(value["bindings"]),
        simulator_launch_origin=_parse_origin(value["simulator_launch_origin"]),
        connection=_parse_connection(value["connection"]),
        operating_site=_parse_operating_site(value["operating_site"]),
        recovery=_parse_recovery(value["recovery"]),
        telemetry_startup=_parse_telemetry(value["telemetry_startup"]),
        autopilot_version=_parse_autopilot(value["autopilot_version"]),
        clearance_calibration=_parse_clearance(value["clearance_calibration"]),
        release_stability=_parse_release_stability(value["release_stability"]),
        fc_home_position_tolerance_m=_positive_number(
            "fc_home_position_tolerance_m", value["fc_home_position_tolerance_m"]
        ),
        fc_home_altitude_tolerance_m=_positive_number(
            "fc_home_altitude_tolerance_m", value["fc_home_altitude_tolerance_m"]
        ),
        idle_poll_s=_positive_number("idle_poll_s", value["idle_poll_s"]),
        cleanup_timeout_s=_positive_number(
            "cleanup_timeout_s", value["cleanup_timeout_s"]
        ),
        payload_delay_wall_timeout_s=_positive_number(
            "payload_delay_wall_timeout_s", value["payload_delay_wall_timeout_s"]
        ),
        enabled_phases=(31000, 31001),
    )


def _reject_symlink_components(path: Path) -> Path:
    if ".." in path.parts:
        raise ValueError("runtime policy path must not contain a parent component")
    supplied = path.expanduser()
    absolute = supplied if supplied.is_absolute() else Path.cwd() / supplied
    current = Path(absolute.anchor)
    metadata = None
    try:
        for part in absolute.parts[1:]:
            current /= part
            metadata = os.lstat(current)
            if stat.S_ISLNK(metadata.st_mode):
                raise ValueError(f"runtime policy path contains symlink component: {current}")
    except OSError as error:
        raise ValueError(f"runtime policy file is unavailable: {error}") from error
    if metadata is None or not stat.S_ISREG(metadata.st_mode):
        raise ValueError("runtime policy path must name a regular file")
    return absolute


def load_qgc_runtime_policy(
    path: str | os.PathLike[str], *, expected_sha256: str
) -> QGCRuntimePolicy:
    """Load and validate one inert simulator policy JSON document."""
    _sha256("expected_sha256", expected_sha256)
    if not isinstance(path, (str, os.PathLike)):
        raise ValueError("runtime policy path must be explicit")
    source_path = _reject_symlink_components(Path(path))
    try:
        flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
        descriptor = os.open(source_path, flags)
        if not stat.S_ISREG(os.fstat(descriptor).st_mode):
            os.close(descriptor)
            raise ValueError("runtime policy path must name a regular file")
        with os.fdopen(descriptor, "rb") as source:
            content = source.read()
    except ValueError:
        raise
    except OSError as error:
        raise ValueError(f"runtime policy file is unavailable: {error}") from error
    return load_qgc_runtime_policy_bytes(content, expected_sha256=expected_sha256)


def load_qgc_runtime_policy_bytes(
    content: bytes, *, expected_sha256: str
) -> QGCRuntimePolicy:
    """Load one policy from an immutable, caller-bound byte snapshot."""
    if not isinstance(content, bytes):
        raise ValueError("runtime policy bytes must be immutable")
    _sha256("expected_sha256", expected_sha256)
    if hashlib.sha256(content).hexdigest() != expected_sha256:
        raise ValueError("runtime policy byte SHA-256 does not match expected SHA-256")
    try:
        raw = json.loads(content.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError(f"runtime policy must be valid UTF-8 JSON: {error}") from error
    return _parse_policy(raw)
