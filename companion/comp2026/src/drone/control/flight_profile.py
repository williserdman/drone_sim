"""Validated flight-observation profile and inert MAVLink decoders."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from numbers import Real
import re
import threading
from types import MappingProxyType
from typing import Callable, Mapping

from .attempt_setup import (
    DeploymentProfile,
    PreparationError,
    load_deployment_profile,
    load_deployment_profile_bytes,
)
from .flight_state import FailsafeEvidence, FlightState, RCModeBand, SourceIdentity
from .mission_supervisor import AuthorityLost
from .. import timebase


RC_RECEIVER_SENSOR_BIT = 65_536
OBSERVATION_CONTRACT_VERSION = 1
OBSERVATION_CONTRACT_REFERENCE = (
    "ArduPilot/Copter-4.5.7:GCS_Mavlink.cpp:70-86;GCS_Copter.cpp:41-48"
)
REQUIRED_TELEMETRY_MESSAGE_IDS = frozenset((0, 1, 30, 33, 65, 132, 242, 245))
FLIGHT_STATE_FIELDS = frozenset(
    (
        "heartbeat",
        "mode",
        "location",
        "velocity",
        "attitude",
        "landed_state",
        "armed",
        "home",
        "rc_input",
        "range",
        "failsafe",
    )
)


class FlightProfileError(PreparationError):
    """The selected deployment observation contract is unsafe or malformed."""


@dataclass(frozen=True)
class TelemetryRequest:
    message_id: int
    interval_us: int


@dataclass(frozen=True)
class FlightProfile:
    profile_id: str
    raw_sha256: str
    firmware: str
    qgc_source: SourceIdentity
    companion_target: SourceIdentity
    flight_controller: SourceIdentity
    freshness_bounds: Mapping[str, float]
    rc_health_max_age: float
    rc_channel: int
    rc_mode_bands: tuple[RCModeBand, ...]
    heartbeat_type: int
    heartbeat_autopilot: int
    copter_modes: Mapping[int, str]
    failsafe_active_statuses: frozenset[int]
    failsafe_clear_statuses: frozenset[int]
    decoder_contract_version: int
    decoder_contract_evidence: str
    decoder_contract_reference: str
    telemetry_requests: tuple[TelemetryRequest, ...]

    def decoders(
        self,
        clock: Callable[[], float] = timebase.monotonic,
        *,
        flight_state: FlightState | None = None,
    ) -> "ObservationDecoders":
        return ObservationDecoders(self, clock=clock, flight_state=flight_state)


def _mapping(value: object, field: str) -> dict:
    if not isinstance(value, dict):
        raise FlightProfileError(f"deployment profile {field} must be an object")
    return value


def _exact_int(value: object, *, low: int, high: int, field: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or not low <= value <= high:
        raise FlightProfileError(f"deployment profile {field} is invalid")
    return value


def _positive_number(value: object, field: str) -> float:
    if (
        not isinstance(value, Real)
        or isinstance(value, bool)
        or not math.isfinite(float(value))
        or float(value) <= 0
    ):
        raise FlightProfileError(f"deployment profile {field} must be finite and positive")
    return float(value)


def _status_set(value: object, field: str) -> frozenset[int]:
    if not isinstance(value, list) or not value:
        raise FlightProfileError(f"deployment profile {field} is invalid")
    statuses = frozenset(
        _exact_int(item, low=0, high=255, field=field) for item in value
    )
    if len(statuses) != len(value):
        raise FlightProfileError(f"deployment profile {field} contains duplicates")
    return statuses


def validate_observation_profile(
    deployment: DeploymentProfile, data: Mapping[str, object]
) -> FlightProfile:
    """Validate and freeze the observation section already read with the profile."""
    observation = _mapping(data.get("observation"), "observation")
    version = _exact_int(
        observation.get("decoder_contract_version"),
        low=OBSERVATION_CONTRACT_VERSION,
        high=OBSERVATION_CONTRACT_VERSION,
        field="observation decoder contract version",
    )
    evidence = observation.get("decoder_contract_evidence")
    if not isinstance(evidence, str) or re.fullmatch(
        r"sha256:[0-9a-f]{64}", evidence
    ) is None:
        raise FlightProfileError("deployment profile decoder contract evidence is invalid")
    reference = observation.get("decoder_contract_reference")
    if reference != OBSERVATION_CONTRACT_REFERENCE:
        raise FlightProfileError(
            "deployment profile decoder contract reference is unsupported"
        )

    freshness_data = _mapping(
        observation.get("freshness_seconds"), "observation.freshness_seconds"
    )
    if set(freshness_data) != FLIGHT_STATE_FIELDS:
        raise FlightProfileError(
            "deployment profile observation freshness must define every FlightState field"
        )
    freshness = MappingProxyType(
        {
            field: _positive_number(value, f"observation freshness {field}")
            for field, value in freshness_data.items()
        }
    )
    rc_health_age = _positive_number(
        observation.get("sys_status_rc_health_freshness_seconds"),
        "observation SYS_STATUS RC health freshness",
    )

    mode_data = _mapping(observation.get("mode_mapping"), "observation.mode_mapping")
    modes: dict[int, str] = {}
    for raw_mode, name in mode_data.items():
        if not isinstance(raw_mode, str) or not raw_mode.isascii() or not raw_mode.isdigit():
            raise FlightProfileError("deployment profile observation mode number is invalid")
        mode = _exact_int(
            int(raw_mode), low=0, high=0xFFFFFFFF, field="observation mode number"
        )
        if not isinstance(name, str) or not name.strip() or name != name.strip().upper():
            raise FlightProfileError("deployment profile observation mode name is invalid")
        if mode in modes or name in modes.values():
            raise FlightProfileError("deployment profile observation modes are duplicated")
        modes[mode] = name
    if not {"STABILIZE", "GUIDED", "LOITER"}.issubset(modes.values()):
        raise FlightProfileError("deployment profile observation mode mapping is incomplete")

    active = _status_set(
        observation.get("failsafe_active_system_statuses"),
        "observation.failsafe_active_system_statuses",
    )
    clear = _status_set(
        observation.get("failsafe_clear_system_statuses"),
        "observation.failsafe_clear_system_statuses",
    )
    if active.intersection(clear):
        raise FlightProfileError("deployment profile failsafe statuses overlap")
    if active != frozenset((5,)) or clear != frozenset((3, 4)):
        raise FlightProfileError(
            "deployment profile failsafe statuses do not match the selected contract"
        )

    rc_mapping = _mapping(data.get("rc"), "rc")
    band_data = _mapping(rc_mapping.get("mode_mapping"), "rc.mode_mapping")
    bands = []
    occupied: list[tuple[int, int]] = []
    for slot, bounds in band_data.items():
        if (
            not isinstance(slot, str)
            or not slot.strip()
            or not isinstance(bounds, list)
            or len(bounds) != 2
        ):
            raise FlightProfileError("deployment profile RC mode mapping is invalid")
        low = _exact_int(bounds[0], low=0, high=65535, field=f"RC slot {slot}")
        high = _exact_int(bounds[1], low=0, high=65535, field=f"RC slot {slot}")
        if low >= high:
            raise FlightProfileError(f"deployment profile RC slot {slot} is invalid")
        if any(low <= prior_high and prior_low <= high for prior_low, prior_high in occupied):
            raise FlightProfileError("deployment profile RC mode bands overlap")
        occupied.append((low, high))
        bands.append(RCModeBand(slot=slot, minimum_pwm=low, maximum_pwm=high, mode=slot))

    requests_data = observation.get("telemetry_requests")
    if not isinstance(requests_data, list) or not requests_data:
        raise FlightProfileError("deployment profile telemetry requests are required")
    requests = []
    message_ids = set()
    for item in requests_data:
        request = _mapping(item, "observation.telemetry_requests entry")
        if set(request) != {"message_id", "interval_us"}:
            raise FlightProfileError("deployment profile telemetry request schema is invalid")
        message_id = _exact_int(
            request.get("message_id"), low=0, high=0xFFFFFF, field="telemetry message ID"
        )
        interval = _exact_int(
            request.get("interval_us"), low=1, high=0x7FFFFFFF, field="telemetry interval"
        )
        if message_id in message_ids:
            raise FlightProfileError("deployment profile telemetry message ID is duplicate")
        message_ids.add(message_id)
        requests.append(TelemetryRequest(message_id, interval))
    if not REQUIRED_TELEMETRY_MESSAGE_IDS.issubset(message_ids):
        raise FlightProfileError("deployment profile required telemetry message IDs are missing")

    qgc = SourceIdentity(deployment.source_system, deployment.source_component)
    companion = SourceIdentity(deployment.target_system, deployment.target_component)
    controller = SourceIdentity(
        deployment.flight_controller_system, deployment.flight_controller_component
    )
    return FlightProfile(
        profile_id=deployment.profile_id,
        raw_sha256=deployment.raw_sha256,
        firmware=deployment.firmware,
        qgc_source=qgc,
        companion_target=companion,
        flight_controller=controller,
        freshness_bounds=freshness,
        rc_health_max_age=rc_health_age,
        rc_channel=_exact_int(
            rc_mapping.get("mode_channel"), low=1, high=18, field="rc.mode_channel"
        ),
        rc_mode_bands=tuple(bands),
        heartbeat_type=_exact_int(
            observation.get("expected_heartbeat_type"),
            low=0,
            high=255,
            field="expected heartbeat type",
        ),
        heartbeat_autopilot=_exact_int(
            observation.get("expected_heartbeat_autopilot"),
            low=0,
            high=255,
            field="expected heartbeat autopilot",
        ),
        copter_modes=MappingProxyType(modes),
        failsafe_active_statuses=active,
        failsafe_clear_statuses=clear,
        decoder_contract_version=version,
        decoder_contract_evidence=evidence,
        decoder_contract_reference=reference,
        telemetry_requests=tuple(requests),
    )


def load_flight_profile(path: object) -> FlightProfile:
    try:
        deployment = load_deployment_profile(path)  # type: ignore[arg-type]
    except FlightProfileError:
        raise
    except PreparationError as error:
        raise FlightProfileError(str(error)) from error
    return _flight_profile_from_deployment(deployment)


def load_flight_profile_bytes(raw: bytes) -> FlightProfile:
    """Validate the flight profile from one immutable deployment snapshot."""
    try:
        deployment = load_deployment_profile_bytes(raw)
    except FlightProfileError:
        raise
    except PreparationError as error:
        raise FlightProfileError(str(error)) from error
    return _flight_profile_from_deployment(deployment)


def _flight_profile_from_deployment(deployment: DeploymentProfile) -> FlightProfile:
    try:
        data = json.loads(deployment.raw_bytes)
    except (UnicodeError, json.JSONDecodeError) as error:
        raise FlightProfileError(f"deployment profile is unreadable: {error}") from error
    return validate_observation_profile(deployment, data)


def _message_source(message: object) -> SourceIdentity | None:
    try:
        system = message.get_srcSystem()  # type: ignore[attr-defined]
        component = message.get_srcComponent()  # type: ignore[attr-defined]
    except (AttributeError, TypeError, ValueError):
        return None
    if (
        not isinstance(system, int)
        or isinstance(system, bool)
        or not 1 <= system <= 255
        or not isinstance(component, int)
        or isinstance(component, bool)
        or not 1 <= component <= 255
    ):
        return None
    return SourceIdentity(system, component)


def _message_int(message: object, field: str, *, high: int) -> int | None:
    value = getattr(message, field, None)
    if not isinstance(value, int) or isinstance(value, bool) or not 0 <= value <= high:
        return None
    return value


class ObservationDecoders:
    """Source-filtered decoders with independent SYS_STATUS RC evidence."""

    def __init__(
        self,
        profile: FlightProfile,
        *,
        clock: Callable[[], float],
        flight_state: FlightState | None,
    ) -> None:
        if not callable(clock):
            raise TypeError("clock must be callable")
        self.profile = profile
        self._clock = clock
        self._flight_state = flight_state
        self._lock = threading.RLock()
        self._rc_health: bool | None = None
        self._rc_health_received_at: float | None = None
        self._next_rc_health_order = 1
        self._rc_health_accepted_order = 0
        self._rc_health_invalidation_barrier = 0

    def _trusted_heartbeat(self, message: object) -> bool:
        return (
            _message_source(message) == self.profile.flight_controller
            and _message_int(message, "type", high=255) == self.profile.heartbeat_type
            and _message_int(message, "autopilot", high=255)
            == self.profile.heartbeat_autopilot
        )

    def heartbeat_mode_decoder(self, message: object) -> str | None:
        if not self._trusted_heartbeat(message):
            return None
        mode = _message_int(message, "custom_mode", high=0xFFFFFFFF)
        return None if mode is None else self.profile.copter_modes.get(mode)

    def heartbeat_failsafe_decoder(
        self, message: object
    ) -> FailsafeEvidence | None:
        if not self._trusted_heartbeat(message):
            return None
        status = _message_int(message, "system_status", high=255)
        if status in self.profile.failsafe_active_statuses:
            return FailsafeEvidence(True, "FC system status CRITICAL")
        if status in self.profile.failsafe_clear_statuses:
            name = "STANDBY" if status == 3 else "ACTIVE"
            return FailsafeEvidence(False, f"FC system status {name}")
        return None

    def observe_sys_status(self, message: object) -> None:
        if _message_source(message) != self.profile.flight_controller:
            return
        with self._lock:
            observation_order = self._next_rc_health_order
            self._next_rc_health_order += 1
        received_at = self._clock()
        if not isinstance(received_at, Real) or not math.isfinite(float(received_at)):
            with self._lock:
                if (
                    observation_order <= self._rc_health_invalidation_barrier
                    or observation_order <= self._rc_health_accepted_order
                ):
                    return
                self._advance_invalidation_barrier_locked()
                self._rc_health = None
                self._revoke_rc_evidence_locked()
            return
        values = tuple(
            _message_int(message, field, high=0xFFFFFFFF)
            for field in (
                "onboard_control_sensors_present",
                "onboard_control_sensors_enabled",
                "onboard_control_sensors_health",
            )
        )
        health = None
        if all(value is not None for value in values):
            health = all(
                value & RC_RECEIVER_SENSOR_BIT != 0  # type: ignore[operator]
                for value in values
            )
        with self._lock:
            if observation_order <= self._rc_health_invalidation_barrier:
                return
            if (
                self._rc_health_received_at is not None
                and float(received_at) < self._rc_health_received_at
            ):
                return
            self._rc_health = health
            self._rc_health_received_at = float(received_at)
            self._rc_health_accepted_order = max(
                self._rc_health_accepted_order, observation_order
            )
            if health is not True:
                self._advance_invalidation_barrier_locked()
                self._revoke_rc_evidence_locked()

    def _advance_invalidation_barrier_locked(self) -> None:
        """Reject every SYS_STATUS callback reserved before invalidation."""
        self._rc_health_invalidation_barrier = max(
            self._rc_health_invalidation_barrier,
            self._next_rc_health_order - 1,
        )

    def _revoke_rc_evidence_locked(self) -> None:
        """Revoke bound RC state before another health publication can pass."""
        if self._flight_state is not None:
            self._flight_state.invalidate_observation(
                "rc_input",
                source_system=self.profile.flight_controller.system_id,
                source_component=self.profile.flight_controller.component_id,
            )

    def _current_rc_health(self, now: float) -> bool | None:
        with self._lock:
            health = self._rc_health
            received_at = self._rc_health_received_at
            if received_at is None:
                return None
            age = now - received_at
            if not math.isfinite(age) or age < 0 or age > self.profile.rc_health_max_age:
                self._rc_health = None
                self._advance_invalidation_barrier_locked()
                return None
            return health

    def rc_health_decoder(self, message: object, channel: object, pwm: object) -> bool | None:
        if (
            _message_source(message) != self.profile.flight_controller
            or channel != self.profile.rc_channel
            or not isinstance(channel, int)
            or isinstance(channel, bool)
            or not isinstance(pwm, int)
            or isinstance(pwm, bool)
            or not 0 <= pwm <= 65535
        ):
            return None
        packet_pwm = _message_int(message, f"chan{channel}_raw", high=65535)
        if packet_pwm is None or packet_pwm != pwm:
            return None
        with self._lock:
            now = self._clock()
            if not isinstance(now, Real) or not math.isfinite(float(now)):
                return None
            return self._current_rc_health(float(now))

    def _check_rc_health_locked(self, now: object) -> None:
        health = (
            None
            if not isinstance(now, Real) or not math.isfinite(float(now))
            else self._current_rc_health(float(now))
        )
        if health is not True:
            self._advance_invalidation_barrier_locked()
            if health is None:
                self._rc_health = None
            self._revoke_rc_evidence_locked()
            raise AuthorityLost(
                "SYS_STATUS RC health is absent, unhealthy, or expired"
            )

    def check_rc_health(self) -> None:
        """Require independently fresh trusted SYS_STATUS RC health."""
        with self._lock:
            self._check_rc_health_locked(self._clock())

    def check_authority_dependency(self) -> None:
        state = self._flight_state
        if state is None:
            raise AuthorityLost("FlightState dependency is unavailable")
        with self._lock:
            self._check_rc_health_locked(self._clock())
            if state.command_permission() is None:
                raise AuthorityLost("companion authority is unavailable")

    def output_transaction(self, operation: Callable[[], object]) -> object:
        """Hold RC health stable across one downstream output transaction."""
        if not callable(operation):
            raise TypeError("operation must be callable")
        with self._lock:
            self._check_rc_health_locked(self._clock())
            return operation()
