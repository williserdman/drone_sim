"""Validated hardware-independent inputs for the live QGC composition.

Constructing these values performs no device or transport access. The live
factory must receive only a configuration that has also been bound to the
already validated deployment and flight profiles.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
from typing import TYPE_CHECKING, Callable

from .. import timebase
from ..common_types import GPSCoord, MissionHome
from ..sensors.lidar.clearance import ClearanceCalibration
from .attempt_setup import DeploymentProfile
from .flight_profile import FlightProfile
from .flight_state import SourceIdentity
from .mission_supervisor import CommandRejected, FM1, FM2, FM3, PHASE_COMMANDS, RecoveryPolicy
from .stability import ReleaseStabilityConfig

if TYPE_CHECKING:
    from ..mock_mission import PrecisionMissionPolicy


def _positive_number(name: str, value: object) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or float(value) <= 0
    ):
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


def _nonnegative_number(name: str, value: object) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or float(value) < 0
    ):
        raise ValueError(f"{name} must be finite and non-negative")
    return float(value)


def _verified_text(name: str, value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be explicit")
    normalized = value.strip()
    lowered = normalized.lower()
    if lowered in {"n/a", "na", "none", "pending", "unspecified"} or any(
        marker in lowered for marker in ("unknown", "unverified", "tbd")
    ):
        raise ValueError(f"{name} is unverified")
    return normalized


def shared_monotonic_ns() -> int:
    """Expose the shared mission clock in integer nanoseconds for camera receipt."""
    value = timebase.monotonic()
    if not math.isfinite(value):
        raise timebase.ClockError("shared timebase returned a nonfinite value")
    return int(value * 1_000_000_000)


@dataclass(frozen=True)
class ConnectionConfig:
    endpoint: str
    source_identity: SourceIdentity
    target_identity: SourceIdentity
    wire_protocol: str
    wait_ready: bool
    heartbeat_timeout_s: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "endpoint", _verified_text("connection endpoint", self.endpoint))
        if not isinstance(self.source_identity, SourceIdentity):
            raise ValueError("connection source identity must be explicit")
        if not isinstance(self.target_identity, SourceIdentity):
            raise ValueError("connection target identity must be explicit")
        if self.source_identity == self.target_identity:
            raise ValueError("connection source and target identities must be distinct")
        if self.wire_protocol not in ("1.0", "2.0"):
            raise ValueError("connection wire protocol is unsupported")
        if not isinstance(self.wait_ready, bool):
            raise ValueError("connection wait_ready must be an explicit Boolean")
        object.__setattr__(
            self,
            "heartbeat_timeout_s",
            _positive_number("heartbeat_timeout_s", self.heartbeat_timeout_s),
        )


@dataclass(frozen=True)
class OperatingSitePolicy:
    waypoint_check: Callable[[str, GPSCoord], bool]
    mission_home_check: Callable[[MissionHome], None]
    recovery_check: Callable[[str, MissionHome, float | None], None]
    evidence_reference: str

    def __post_init__(self) -> None:
        for name in ("waypoint_check", "mission_home_check", "recovery_check"):
            if not callable(getattr(self, name)):
                raise ValueError(f"operating-site {name} must be callable")
        object.__setattr__(
            self,
            "evidence_reference",
            _verified_text("operating-site evidence reference", self.evidence_reference),
        )


@dataclass(frozen=True)
class TelemetryStartupPolicy:
    command_ack_timeout_s: float
    collection_timeout_s: float
    home_request_timeout_s: float
    poll_interval_s: float
    minimum_distinct_samples: int
    maximum_interval_error_fraction: float

    def __post_init__(self) -> None:
        for name in (
            "command_ack_timeout_s",
            "collection_timeout_s",
            "home_request_timeout_s",
            "poll_interval_s",
            "maximum_interval_error_fraction",
        ):
            object.__setattr__(self, name, _positive_number(name, getattr(self, name)))
        if self.poll_interval_s > min(
            self.command_ack_timeout_s,
            self.collection_timeout_s,
            self.home_request_timeout_s,
        ):
            raise ValueError("telemetry poll interval exceeds a bounded operation")
        if (
            not isinstance(self.minimum_distinct_samples, int)
            or isinstance(self.minimum_distinct_samples, bool)
            or self.minimum_distinct_samples < 2
        ):
            raise ValueError("minimum_distinct_samples must be an integer of at least two")
        if self.maximum_interval_error_fraction > 1:
            raise ValueError("maximum_interval_error_fraction must not exceed one")


@dataclass(frozen=True)
class AutopilotVersionContract:
    """Bench-bound raw AUTOPILOT_VERSION values for the selected firmware."""

    firmware_label: str
    flight_sw_version: int
    flight_custom_version: bytes
    evidence_reference: str

    def __post_init__(self) -> None:
        label = _verified_text("firmware label", self.firmware_label)
        if label != "ArduCopter 4.5.7":
            raise ValueError("only the reviewed stable ArduCopter 4.5.7 release is supported")
        if (
            not isinstance(self.flight_sw_version, int)
            or isinstance(self.flight_sw_version, bool)
            or not 0 <= self.flight_sw_version <= 0xFFFFFFFF
        ):
            raise ValueError("flight_sw_version must be an unsigned 32-bit integer")
        if not isinstance(self.flight_custom_version, bytes) or len(
            self.flight_custom_version
        ) != 8:
            raise ValueError("flight_custom_version must contain exactly eight bytes")
        match = re.search(r"\b(\d+)\.(\d+)\.(\d+)\b", label)
        if match is None:
            raise ValueError("firmware label must contain a semantic version")
        packed_version = tuple(
            (self.flight_sw_version >> shift) & 0xFF for shift in (24, 16, 8)
        )
        if packed_version != tuple(int(part) for part in match.groups()):
            raise ValueError("firmware label does not match flight_sw_version")
        if self.flight_sw_version & 0xFF != 0xFF:
            raise ValueError("firmware version must identify an official stable release")
        object.__setattr__(self, "firmware_label", label)
        object.__setattr__(
            self,
            "evidence_reference",
            _verified_text(
                "AUTOPILOT_VERSION evidence reference", self.evidence_reference
            ),
        )


def _load_json_object(path: object, name: str) -> tuple[Path, dict]:
    if not isinstance(path, (str, Path)):
        raise ValueError(f"{name} path must be explicit")
    resolved = Path(path).expanduser().resolve()
    try:
        with resolved.open("r", encoding="utf-8") as source:
            value = json.load(source)
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError(f"{name} file is unavailable or invalid: {error}") from error
    if not isinstance(value, dict):
        raise ValueError(f"{name} file must contain an object")
    return resolved, value


def _finite_matrix(value: object, rows: int, columns: int, name: str) -> tuple[tuple[float, ...], ...]:
    if not isinstance(value, list) or len(value) != rows:
        raise ValueError(f"{name} must be a {rows}x{columns} matrix")
    matrix = []
    for row in value:
        if not isinstance(row, list) or len(row) != columns:
            raise ValueError(f"{name} must be a {rows}x{columns} matrix")
        if any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in row):
            raise ValueError(f"{name} must contain finite numbers")
        normalized = tuple(float(item) for item in row)
        if not all(math.isfinite(item) for item in normalized):
            raise ValueError(f"{name} must contain finite numbers")
        matrix.append(normalized)
    return tuple(matrix)


def _validate_camera_calibration(value: dict) -> None:
    matrix = _finite_matrix(value.get("camera_matrix"), 3, 3, "camera_matrix")
    if matrix[2] != (0.0, 0.0, 1.0) or matrix[0][1] != 0 or matrix[1][0] != 0:
        raise ValueError("camera_matrix uses an unsupported projection form")
    if matrix[0][0] <= 0 or matrix[1][1] <= 0:
        raise ValueError("camera_matrix focal lengths must be positive")
    distortion = value.get("dist_coeff")
    if not isinstance(distortion, list) or not distortion:
        raise ValueError("dist_coeff must be an explicit row or column")
    if len(distortion) == 1 and isinstance(distortion[0], list):
        coefficients = distortion[0]
    elif all(isinstance(row, list) and len(row) == 1 for row in distortion):
        coefficients = [row[0] for row in distortion]
    else:
        raise ValueError("dist_coeff must be an explicit row or column")
    if len(coefficients) not in (4, 5, 8, 12, 14):
        raise ValueError("dist_coeff has an unsupported coefficient count")
    if not all(
        isinstance(item, (int, float))
        and not isinstance(item, bool)
        and math.isfinite(float(item))
        for item in coefficients
    ):
        raise ValueError("dist_coeff must contain finite numbers")
    if value.get("dimensions_verified") is not True:
        raise ValueError("camera calibration dimensions are not verified")
    for name in ("image_width_px", "image_height_px"):
        dimension = value.get(name)
        if not isinstance(dimension, int) or isinstance(dimension, bool) or dimension <= 0:
            raise ValueError(f"verified {name} must be a positive integer")


def _validate_camera_mounting(value: dict) -> None:
    if value.get("verified") is not True:
        raise ValueError("camera mounting is not verified")
    rotation = _finite_matrix(value.get("camera_to_body_frd"), 3, 3, "camera_to_body_frd")
    for row_index in range(3):
        for column_index in range(3):
            dot = sum(rotation[k][row_index] * rotation[k][column_index] for k in range(3))
            expected = 1.0 if row_index == column_index else 0.0
            if not math.isclose(dot, expected, abs_tol=1e-6):
                raise ValueError("camera_to_body_frd must be orthonormal")
    determinant = (
        rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    if not math.isclose(determinant, 1.0, abs_tol=1e-6):
        raise ValueError("camera_to_body_frd must be right-handed")
    offset = value.get("down_offset_m")
    if isinstance(offset, bool) or not isinstance(offset, (int, float)) or not math.isfinite(float(offset)):
        raise ValueError("camera down_offset_m must be finite")


@dataclass(frozen=True)
class VisionConfig:
    marker_size_mm: float
    calibration_path: Path
    mounting_path: Path
    receipt_clock_ns: Callable[[], int]
    max_exposure_age_ns: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "marker_size_mm", _positive_number("marker_size_mm", self.marker_size_mm))
        calibration_path, calibration = _load_json_object(self.calibration_path, "camera calibration")
        mounting_path, mounting = _load_json_object(self.mounting_path, "camera mounting")
        if calibration_path == mounting_path:
            raise ValueError("camera calibration and mounting files must be distinct")
        _validate_camera_calibration(calibration)
        _validate_camera_mounting(mounting)
        object.__setattr__(self, "calibration_path", calibration_path)
        object.__setattr__(self, "mounting_path", mounting_path)
        if self.receipt_clock_ns is not shared_monotonic_ns:
            raise ValueError("vision receipt clock must be the shared monotonic nanosecond clock")
        if (
            not isinstance(self.max_exposure_age_ns, int)
            or isinstance(self.max_exposure_age_ns, bool)
            or self.max_exposure_age_ns <= 0
        ):
            raise ValueError("max_exposure_age_ns must be a positive integer")


@dataclass(frozen=True)
class RangeConfig:
    raw_min_cm: float
    raw_max_cm: float
    mounting_offset_cm: float
    stale_after_seconds: float
    startup_timeout_seconds: float
    poll_interval_seconds: float

    def __post_init__(self) -> None:
        for name in ("raw_min_cm", "mounting_offset_cm"):
            object.__setattr__(self, name, _nonnegative_number(name, getattr(self, name)))
        for name in (
            "raw_max_cm",
            "stale_after_seconds",
            "startup_timeout_seconds",
            "poll_interval_seconds",
        ):
            object.__setattr__(self, name, _positive_number(name, getattr(self, name)))
        if self.raw_max_cm < self.raw_min_cm:
            raise ValueError("raw range must be ordered")
        if self.mounting_offset_cm > self.raw_max_cm:
            raise ValueError("mounting_offset_cm cannot exceed raw_max_cm")
        if self.poll_interval_seconds > self.stale_after_seconds:
            raise ValueError("range poll interval cannot exceed stale interval")


@dataclass(frozen=True)
class PayloadConfig:
    pins: tuple[int, ...]
    release_hold_seconds: float
    permission_check_interval_seconds: float
    min_pulse_width_seconds: float
    max_pulse_width_seconds: float

    def __post_init__(self) -> None:
        if not isinstance(self.pins, tuple) or not self.pins or any(
            not isinstance(pin, int) or isinstance(pin, bool) or pin < 0 for pin in self.pins
        ):
            raise ValueError("payload pins must be an explicit tuple of non-negative integers")
        if len(set(self.pins)) != len(self.pins):
            raise ValueError("payload pins must be unique")
        for name in (
            "release_hold_seconds",
            "permission_check_interval_seconds",
            "min_pulse_width_seconds",
            "max_pulse_width_seconds",
        ):
            object.__setattr__(self, name, _positive_number(name, getattr(self, name)))
        if self.permission_check_interval_seconds > self.release_hold_seconds:
            raise ValueError("payload permission interval cannot exceed release hold")
        if self.max_pulse_width_seconds <= self.min_pulse_width_seconds:
            raise ValueError("payload pulse widths must be ordered")


@dataclass(frozen=True)
class HardwareComponentConfig:
    range_sensor: RangeConfig
    payload: PayloadConfig

    def __post_init__(self) -> None:
        if not isinstance(self.range_sensor, RangeConfig):
            raise ValueError("range_sensor must be explicit validated configuration")
        if not isinstance(self.payload, PayloadConfig):
            raise ValueError("payload must be explicit validated configuration")


@dataclass(frozen=True)
class InjectedComponentConfig:
    backend: str
    evidence_reference: str

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "backend",
            _verified_text("component backend", self.backend),
        )
        object.__setattr__(
            self,
            "evidence_reference",
            _verified_text("component evidence reference", self.evidence_reference),
        )


@dataclass(frozen=True)
class RuntimeConfiguration:
    connection: ConnectionConfig
    waypoint_path: Path
    operating_site: OperatingSitePolicy
    recovery_policy: RecoveryPolicy
    telemetry: TelemetryStartupPolicy
    autopilot_version: AutopilotVersionContract
    vision: VisionConfig | None
    components: HardwareComponentConfig | InjectedComponentConfig
    clearance_calibration: ClearanceCalibration
    release_stability: ReleaseStabilityConfig
    enabled_phases: frozenset[int]
    precision_policy: PrecisionMissionPolicy | None
    cruise_altitude_m: float
    desired_drop_height_m: float
    fc_home_position_tolerance_m: float
    fc_home_altitude_tolerance_m: float
    attempt_timeout_s: float
    idle_poll_s: float
    cleanup_timeout_s: float

    def __post_init__(self) -> None:
        from ..mock_mission import PrecisionMissionPolicy

        expected = (
            ("connection", ConnectionConfig),
            ("operating_site", OperatingSitePolicy),
            ("recovery_policy", RecoveryPolicy),
            ("telemetry", TelemetryStartupPolicy),
            ("autopilot_version", AutopilotVersionContract),
            ("clearance_calibration", ClearanceCalibration),
            ("release_stability", ReleaseStabilityConfig),
        )
        for name, kind in expected:
            if not isinstance(getattr(self, name), kind):
                raise ValueError(f"{name} must be explicit validated configuration")
        if not isinstance(
            self.components, (HardwareComponentConfig, InjectedComponentConfig)
        ):
            raise ValueError("components must be explicit validated configuration")
        if not isinstance(self.waypoint_path, (str, os.PathLike)):
            raise ValueError("waypoint path must be explicit")
        waypoint_path = Path(os.path.abspath(os.fspath(self.waypoint_path)))
        from .mission_info import MissonTracker

        tracker = MissonTracker(
            mission_time_seconds=self.attempt_timeout_s,
            waypoint_path=waypoint_path,
        )
        tracker.snapshot_for_attempt(
            (),
            self.operating_site.waypoint_check,
            optional_names=("L", "TARGET", "WA", "WM1", "WM2", "WM3", "WM4", "WM5", "WM6"),
        )
        object.__setattr__(self, "waypoint_path", waypoint_path)
        if self.recovery_policy.check is not self.operating_site.recovery_check:
            raise ValueError("recovery policy must use the operating-site recovery check")
        if self.recovery_policy.clock is not timebase.monotonic:
            raise ValueError("recovery policy must use the shared timebase")
        if not isinstance(self.enabled_phases, frozenset) or self.enabled_phases not in (
            frozenset((FM1, FM2)),
            frozenset((FM1, FM2, FM3)),
        ):
            raise ValueError("enabled_phases must explicitly enable FM1/FM2, with optional FM3")
        if FM3 in self.enabled_phases:
            if not isinstance(self.vision, VisionConfig):
                raise ValueError("enabled FM3 requires verified vision configuration")
            if not isinstance(self.precision_policy, PrecisionMissionPolicy):
                raise ValueError("enabled FM3 requires explicit validated precision policy")
        if self.vision is not None and not isinstance(self.vision, VisionConfig):
            raise ValueError("vision must be validated configuration or None")
        if self.precision_policy is not None and not isinstance(
            self.precision_policy, PrecisionMissionPolicy
        ):
            raise ValueError("precision_policy must be validated configuration or None")
        if (self.vision is None) != (self.precision_policy is None):
            raise ValueError("vision and precision configuration must be supplied together")
        if self.precision_policy is not None:
            if self.precision_policy.clock is not timebase.monotonic:
                raise ValueError("precision policy must use the shared timebase")
            if self.precision_policy.clearance_calibration is not self.clearance_calibration:
                raise ValueError("precision and release must use the same clearance calibration")
            exposure_age_s = self.vision.max_exposure_age_ns / 1_000_000_000
            if not math.isclose(
                exposure_age_s,
                self.precision_policy.max_exposure_age_s,
                rel_tol=1e-12,
                abs_tol=0.0,
            ):
                raise ValueError("vision and precision exposure age limits must match")
        for name in (
            "cruise_altitude_m",
            "desired_drop_height_m",
            "fc_home_position_tolerance_m",
            "fc_home_altitude_tolerance_m",
            "attempt_timeout_s",
            "idle_poll_s",
            "cleanup_timeout_s",
        ):
            object.__setattr__(self, name, _positive_number(name, getattr(self, name)))
        if self.idle_poll_s > self.attempt_timeout_s:
            raise ValueError("idle_poll_s cannot exceed attempt_timeout_s")
        if self.attempt_timeout_s > 600.0:
            raise ValueError("attempt_timeout_s cannot exceed 600 seconds")
        if self.precision_policy is not None:
            if self.precision_policy.cruise_altitude_m != self.cruise_altitude_m:
                raise ValueError("precision and FM1/FM2 cruise altitude must match")
            if self.precision_policy.desired_drop_height_m != self.desired_drop_height_m:
                raise ValueError("precision and FM2 drop height must match")

    def require_command_enabled(self, command: int) -> None:
        """Admission check for explicitly disabled mission phases."""
        if command in PHASE_COMMANDS and command not in self.enabled_phases:
            raise CommandRejected(f"mission phase {command} is disabled", "DENIED")


def construct_after_runtime_validation(
    config: RuntimeConfiguration,
    *,
    deployment_profile: DeploymentProfile,
    flight_profile: FlightProfile,
    component_factory: Callable[[RuntimeConfiguration], object],
) -> object:
    """Bind explicit runtime inputs to the fixed profile before construction."""
    if not isinstance(config, RuntimeConfiguration):
        raise TypeError("config must be a RuntimeConfiguration")
    if not isinstance(deployment_profile, DeploymentProfile):
        raise TypeError("deployment_profile must be a DeploymentProfile")
    if not isinstance(flight_profile, FlightProfile):
        raise TypeError("flight_profile must be a FlightProfile")
    if not callable(component_factory):
        raise TypeError("component_factory must be callable")
    connection = config.connection
    if connection.source_identity != flight_profile.companion_target:
        raise ValueError("connection source identity does not match the fixed profile")
    if connection.target_identity != flight_profile.flight_controller:
        raise ValueError("connection target identity does not match the fixed profile")
    if connection.wire_protocol != deployment_profile.mavlink_wire_protocol:
        raise ValueError("connection wire protocol does not match the fixed profile")
    if config.autopilot_version.firmware_label != flight_profile.firmware:
        raise ValueError("AUTOPILOT_VERSION firmware label does not match the fixed profile")
    if (
        deployment_profile.profile_id != flight_profile.profile_id
        or deployment_profile.raw_sha256 != flight_profile.raw_sha256
        or deployment_profile.firmware != flight_profile.firmware
    ):
        raise ValueError("deployment and flight profiles are inconsistent")
    expected = (
        deployment_profile.target_system,
        deployment_profile.target_component,
        deployment_profile.flight_controller_system,
        deployment_profile.flight_controller_component,
    )
    actual = (
        connection.source_identity.system_id,
        connection.source_identity.component_id,
        connection.target_identity.system_id,
        connection.target_identity.component_id,
    )
    if actual != expected:
        raise ValueError("connection identities do not match the deployment profile")
    return component_factory(config)
