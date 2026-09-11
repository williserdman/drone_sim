"""Source-filtered flight observations and conservative authority classification."""

from __future__ import annotations

import copy
from dataclasses import dataclass, fields, is_dataclass
from enum import Enum
import math
from numbers import Real
import threading
from types import MappingProxyType
from typing import Callable, Mapping, Sequence

from .. import timebase


class Authority(str, Enum):
    COMPANION = "COMPANION"
    PILOT = "PILOT"
    FC_FAILSAFE = "FC_FAILSAFE"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class Observation:
    value: object
    received_at: float
    sequence: int


def is_fresh(observation: Observation, now: float, max_age: float) -> bool:
    """Return whether an observation has a finite, non-future receipt age."""
    if not all(math.isfinite(value) for value in (now, max_age, observation.received_at)):
        return False
    if max_age < 0:
        return False
    age = now - observation.received_at
    return 0 <= age <= max_age


@dataclass(frozen=True)
class SourceIdentity:
    system_id: int
    component_id: int


@dataclass(frozen=True)
class RCModeBand:
    slot: str
    minimum_pwm: int
    maximum_pwm: int
    mode: str


@dataclass(frozen=True)
class RCInput:
    channel: int
    pwm: int
    healthy: bool
    slot: str | None = None


@dataclass(frozen=True)
class FailsafeEvidence:
    active: bool
    reason: str


@dataclass(frozen=True)
class FieldSnapshot:
    observation: Observation
    source: SourceIdentity
    age: float
    fresh: bool
    invalidation_generation: int


@dataclass(frozen=True)
class CommandPermission:
    generation: int


@dataclass(frozen=True)
class FlightSnapshot:
    heartbeat: FieldSnapshot | None
    mode: FieldSnapshot | None
    location: FieldSnapshot | None
    velocity: FieldSnapshot | None
    attitude: FieldSnapshot | None
    landed_state: FieldSnapshot | None
    armed: FieldSnapshot | None
    home: FieldSnapshot | None
    rc_input: FieldSnapshot | None
    range: FieldSnapshot | None
    failsafe: FieldSnapshot | None
    authority: Authority
    commands_suspended: bool
    expected_mode: str | None


_FIELD_NAMES = (
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

MAV_LANDED_STATE_ON_GROUND = 1


def _valid_source_id(value: object, *, component: bool) -> bool:
    lower = 0 if component else 1
    return isinstance(value, int) and not isinstance(value, bool) and lower <= value <= 255


def _value_is_valid(value: object) -> bool:
    if value is None:
        return False
    if isinstance(value, RCInput):
        return (
            isinstance(value.channel, int)
            and isinstance(value.pwm, int)
            and isinstance(value.healthy, bool)
            and (value.slot is None or isinstance(value.slot, str))
        )
    if isinstance(value, Real) and not isinstance(value, bool):
        return math.isfinite(float(value))
    if isinstance(value, Mapping):
        return all(_value_is_valid(item) for item in value.values())
    if isinstance(value, (tuple, list, set, frozenset)):
        return all(_value_is_valid(item) for item in value)
    if is_dataclass(value):
        return all(_value_is_valid(getattr(value, item.name)) for item in fields(value))
    return True


def _freeze_value(value: object) -> object:
    if isinstance(value, Mapping):
        return MappingProxyType({key: _freeze_value(item) for key, item in value.items()})
    if isinstance(value, (tuple, list)):
        return tuple(_freeze_value(item) for item in value)
    if isinstance(value, (set, frozenset)):
        return frozenset(_freeze_value(item) for item in value)
    return copy.deepcopy(value)


class FlightState:
    """Collect observations without issuing aircraft commands."""

    def __init__(
        self,
        *,
        source_system: int,
        source_component: int,
        freshness_bounds: Mapping[str, float],
        rc_channel: int,
        rc_mode_mapping: Sequence[RCModeBand],
        clock: Callable[[], float] = timebase.monotonic,
    ) -> None:
        if not _valid_source_id(source_system, component=False):
            raise ValueError("source_system must be an integer from 1 through 255")
        if not _valid_source_id(source_component, component=True):
            raise ValueError("source_component must be an integer from 0 through 255")
        if not isinstance(rc_channel, int) or isinstance(rc_channel, bool) or rc_channel < 1:
            raise ValueError("rc_channel must be a positive integer")
        if set(freshness_bounds) != set(_FIELD_NAMES):
            raise ValueError("freshness_bounds must define every flight observation field")
        normalized_bounds = {}
        for name, bound in freshness_bounds.items():
            if not isinstance(bound, Real) or not math.isfinite(float(bound)) or bound <= 0:
                raise ValueError(f"freshness bound for {name} must be finite and positive")
            normalized_bounds[name] = float(bound)
        bands = tuple(rc_mode_mapping)
        if not bands:
            raise ValueError("rc_mode_mapping must contain at least one validated band")
        occupied: list[tuple[int, int]] = []
        slots: set[str] = set()
        for band in bands:
            if not isinstance(band, RCModeBand):
                raise ValueError("rc_mode_mapping entries must be RCModeBand values")
            if not band.slot or band.slot in slots or not band.mode.strip():
                raise ValueError("RC bands require unique slots and non-empty modes")
            if band.minimum_pwm < 0 or band.minimum_pwm > band.maximum_pwm:
                raise ValueError("RC band PWM bounds are invalid")
            if any(band.minimum_pwm <= high and low <= band.maximum_pwm for low, high in occupied):
                raise ValueError("RC mode bands must not overlap")
            slots.add(band.slot)
            occupied.append((band.minimum_pwm, band.maximum_pwm))
        modes = {self._normalize_mode(band.mode) for band in bands}
        if "GUIDED" not in modes or not modes.intersection({"LOITER", "STABILIZE"}):
            raise ValueError(
                "rc_mode_mapping requires GUIDED and a LOITER or STABILIZE takeover"
            )

        self.source = SourceIdentity(source_system, source_component)
        self.freshness_bounds = MappingProxyType(normalized_bounds)
        self.rc_channel = rc_channel
        self.rc_mode_mapping = bands
        self._clock = clock
        self._lock = threading.RLock()
        self._observations: dict[str, Observation] = {}
        self._latest_updates: dict[str, tuple[float, int]] = {}
        self._invalidation_generations = {name: 0 for name in _FIELD_NAMES}
        self._authority = Authority.UNKNOWN
        self._commands_suspended = True
        self._attempt_started = False
        self._attempt_revoked = False
        self._expected_mode: tuple[int, str, int] | None = None
        self._next_expected_mode_token = 1
        self._permission_generation = 0
        self._rc_baseline_slot: str | None = None
        self._last_healthy_rc: Observation | None = None
        self._rc_edge: Observation | None = None
        self._output_thread_id: int | None = None

    def update(
        self,
        field: str,
        value: object,
        *,
        received_at: float,
        sequence: int,
        source_system: int,
        source_component: int,
    ) -> bool:
        return self.update_many(
            {field: value},
            received_at=received_at,
            sequence=sequence,
            source_system=source_system,
            source_component=source_component,
        )

    def update_many(
        self,
        values: Mapping[str, object],
        *,
        received_at: float,
        sequence: int,
        source_system: int,
        source_component: int,
    ) -> bool:
        unknown = set(values) - set(_FIELD_NAMES)
        if unknown:
            raise ValueError(f"unknown flight observation field: {sorted(unknown)[0]}")
        if not values:
            raise ValueError("values must contain at least one observation field")
        if SourceIdentity(source_system, source_component) != self.source:
            return False
        if (
            not isinstance(sequence, int)
            or isinstance(sequence, bool)
            or sequence < 0
            or not isinstance(received_at, Real)
            or not math.isfinite(float(received_at))
            or not all(_value_is_valid(value) for value in values.values())
        ):
            return False
        observations = {
            field: Observation(_freeze_value(value), float(received_at), sequence)
            for field, value in values.items()
        }
        with self._lock:
            if not self._is_newer_locked(
                tuple(observations), float(received_at), sequence
            ):
                return False
            self._observations.update(observations)
            for field in observations:
                self._latest_updates[field] = (float(received_at), sequence)
        return True

    def _is_newer_locked(
        self, field_names: Sequence[str], received_at: float, sequence: int
    ) -> bool:
        for field in field_names:
            previous = self._latest_updates.get(field)
            if previous is not None and (
                sequence <= previous[1] or received_at < previous[0]
            ):
                return False
        return True

    def snapshot(self, *, now: float | None = None) -> FlightSnapshot:
        with self._lock:
            sampled_at = self._clock() if now is None else now
            snapshots = {}
            for name in _FIELD_NAMES:
                observation = self._observations.get(name)
                snapshot_observation = (
                    None
                    if observation is None
                    else Observation(
                        value=_freeze_value(observation.value),
                        received_at=observation.received_at,
                        sequence=observation.sequence,
                    )
                )
                snapshots[name] = (
                    None
                    if snapshot_observation is None
                    else FieldSnapshot(
                        observation=snapshot_observation,
                        source=self.source,
                        age=sampled_at - snapshot_observation.received_at,
                        fresh=is_fresh(
                            snapshot_observation,
                            sampled_at,
                            self.freshness_bounds[name],
                        ),
                        invalidation_generation=self._invalidation_generations[name],
                    )
                )
            return FlightSnapshot(
                **snapshots,
                authority=self._authority,
                commands_suspended=self._commands_suspended,
                expected_mode=(
                    None if self._expected_mode is None else self._expected_mode[1]
                ),
            )

    @property
    def authority(self) -> Authority:
        with self._lock:
            return self._authority

    def ordinary_commands_permitted(self) -> bool:
        with self._lock:
            return self._authority is Authority.COMPANION and not self._commands_suspended

    def _slot_for_pwm(self, pwm: int) -> RCModeBand | None:
        for band in self.rc_mode_mapping:
            if band.minimum_pwm <= pwm <= band.maximum_pwm:
                return band
        return None

    @staticmethod
    def _normalize_mode(mode: object) -> str | None:
        candidate = getattr(mode, "name", mode)
        if not isinstance(candidate, str) or not candidate.strip():
            return None
        return candidate.strip().upper()

    def acquire_initial_companion_authority(self, *, now: float | None = None) -> bool:
        """Start one attempt from fresh, explicit ground and RC evidence."""
        with self._lock:
            sampled_at = self._clock() if now is None else now
            if self._attempt_started or self._attempt_revoked:
                return False
            required = {
                name: self._observations.get(name)
                for name in ("heartbeat", "landed_state", "armed", "rc_input")
            }
            if any(observation is None for observation in required.values()):
                return False
            if any(
                not is_fresh(required[name], sampled_at, self.freshness_bounds[name])  # type: ignore[arg-type]
                for name in required
            ):
                return False
            rc_value = required["rc_input"].value  # type: ignore[union-attr]
            landed_value = required["landed_state"].value  # type: ignore[union-attr]
            baseline_band = next(
                (
                    band
                    for band in self.rc_mode_mapping
                    if band.slot == self._rc_baseline_slot
                ),
                None,
            )
            if (
                not isinstance(landed_value, int)
                or isinstance(landed_value, bool)
                or landed_value != MAV_LANDED_STATE_ON_GROUND
                or required["armed"].value is not False  # type: ignore[union-attr]
                or not isinstance(rc_value, RCInput)
                or rc_value.healthy is not True
                or rc_value.slot is None
                or self._rc_baseline_slot != rc_value.slot
                or baseline_band is None
                or self._normalize_mode(baseline_band.mode) != "GUIDED"
            ):
                return False
            self._attempt_started = True
            self._authority = Authority.COMPANION
            self._commands_suspended = False
            self._permission_generation += 1
            return True

    def register_expected_mode(self, mode: object) -> int:
        """Record a command's intended mode without granting command permission."""
        normalized = self._normalize_mode(mode)
        if normalized is None:
            raise ValueError("expected mode must be a non-empty mode name")
        with self._lock:
            token = self._next_expected_mode_token
            self._next_expected_mode_token += 1
            self._expected_mode = (token, normalized, self._permission_generation)
            return token

    def cancel_expected_mode(self, token: int) -> bool:
        with self._lock:
            if self._expected_mode is None or self._expected_mode[0] != token:
                return False
            self._expected_mode = None
            return True

    def observe_mode(
        self,
        mode: object,
        *,
        received_at: float,
        sequence: int,
        source_system: int,
        source_component: int,
    ) -> bool:
        normalized = self._normalize_mode(mode)
        if normalized is None:
            return False
        with self._lock:
            previous = self._observations.get("mode")
            accepted = self.update(
                "mode",
                normalized,
                received_at=received_at,
                sequence=sequence,
                source_system=source_system,
                source_component=source_component,
            )
            if accepted:
                self._process_mode_locked(previous)
            return accepted

    def observe_heartbeat(
        self,
        heartbeat: object,
        *,
        armed: bool,
        mode: object | None,
        received_at: float,
        sequence: int,
        source_system: int,
        source_component: int,
    ) -> bool:
        """Atomically ingest heartbeat, armed state, and decoded source mode."""
        normalized = None if mode is None else self._normalize_mode(mode)
        values = {"heartbeat": heartbeat, "armed": armed}
        if normalized is not None:
            values["mode"] = normalized
        with self._lock:
            affected_fields = ("heartbeat", "armed", "mode")
            if SourceIdentity(source_system, source_component) != self.source:
                return False
            if (
                not isinstance(sequence, int)
                or isinstance(sequence, bool)
                or sequence < 0
                or not isinstance(received_at, Real)
                or not math.isfinite(float(received_at))
                or not self._is_newer_locked(
                    affected_fields, float(received_at), sequence
                )
            ):
                return False
            if (
                not _value_is_valid(heartbeat)
                or not isinstance(armed, bool)
                or normalized is None
            ):
                self._invalidate_locked(
                    affected_fields, float(received_at), sequence
                )
                return True
            previous = self._observations.get("mode")
            accepted = self.update_many(
                values,
                received_at=received_at,
                sequence=sequence,
                source_system=source_system,
                source_component=source_component,
            )
            if accepted:
                self._process_mode_locked(previous)
            return accepted

    def _process_mode_locked(self, previous: Observation | None) -> None:
        observation = self._observations["mode"]
        if not is_fresh(observation, self._clock(), self.freshness_bounds["mode"]):
            return
        self._classify_pilot_if_corroborated_locked()
        if self._authority is Authority.PILOT:
            self._expected_mode = None
            return
        expected = self._expected_mode
        expected_accepted = (
            expected is not None
            and expected[1] == observation.value
            and expected[2] == self._permission_generation
            and self._authority is Authority.COMPANION
            and not self._commands_suspended
        )
        if expected_accepted:
            self._expected_mode = None
            return
        if previous is None or previous.value == observation.value:
            return
        self._revoke_to_unknown_locked()
        self._classify_pilot_if_corroborated_locked()

    def observe_rc_input(
        self,
        *,
        channel: int,
        pwm: int,
        signal_healthy: bool | None,
        received_at: float,
        sequence: int,
        source_system: int,
        source_component: int,
    ) -> bool:
        if channel != self.rc_channel:
            return False
        if not isinstance(pwm, int) or isinstance(pwm, bool):
            return False
        band = self._slot_for_pwm(pwm)
        value = RCInput(
            channel=channel,
            pwm=pwm,
            healthy=signal_healthy is True,
            slot=None if band is None else band.slot,
        )
        with self._lock:
            accepted = self.update(
                "rc_input",
                value,
                received_at=received_at,
                sequence=sequence,
                source_system=source_system,
                source_component=source_component,
            )
            if not accepted:
                return False
            observation = self._observations["rc_input"]
            fresh = is_fresh(
                observation, self._clock(), self.freshness_bounds["rc_input"]
            )
            if not value.healthy or not fresh or value.slot is None:
                self._rc_edge = None
                self._last_healthy_rc = None
                self._rc_baseline_slot = None
                self._revoke_to_unknown_locked()
                return True
            previous_slot = self._rc_baseline_slot
            if previous_slot is None:
                self._rc_baseline_slot = value.slot
            elif value.slot != previous_slot:
                if self._last_healthy_rc is not None and is_fresh(
                    self._last_healthy_rc,
                    observation.received_at,
                    self.freshness_bounds["rc_input"],
                ):
                    self._rc_edge = observation
                else:
                    self._rc_edge = None
                self._rc_baseline_slot = value.slot
                self._revoke_to_unknown_locked()
            self._last_healthy_rc = observation
        return True

    def _revoke_to_unknown_locked(self) -> None:
        self._expected_mode = None
        self._commands_suspended = True
        if self._authority not in {Authority.PILOT, Authority.FC_FAILSAFE}:
            self._authority = Authority.UNKNOWN
        if self._attempt_started:
            self._attempt_revoked = True
        self._permission_generation += 1
        self._raise_if_reentrant_output_revoked_locked()

    def _raise_if_reentrant_output_revoked_locked(self) -> None:
        if self._output_thread_id == threading.get_ident():
            raise PermissionError("authority changed during output preparation")

    def _classify_pilot_if_corroborated_locked(self) -> None:
        if self._authority is Authority.FC_FAILSAFE or self._rc_edge is None:
            return
        mode = self._observations.get("mode")
        if mode is None:
            return
        now = self._clock()
        if not is_fresh(mode, now, self.freshness_bounds["mode"]):
            return
        if not is_fresh(self._rc_edge, now, self.freshness_bounds["rc_input"]):
            return
        if mode.received_at < self._rc_edge.received_at:
            return
        rc_value = self._rc_edge.value
        if not isinstance(rc_value, RCInput) or not rc_value.healthy or rc_value.slot is None:
            return
        band = next(
            (item for item in self.rc_mode_mapping if item.slot == rc_value.slot), None
        )
        if band is None or band.mode.upper() not in {"LOITER", "STABILIZE"}:
            return
        if mode.value != band.mode.upper():
            return
        self._authority = Authority.PILOT
        self._commands_suspended = True
        self._attempt_revoked = True
        self._permission_generation += 1
        self._raise_if_reentrant_output_revoked_locked()

    def observe_failsafe(
        self,
        reason: object,
        *,
        active: bool,
        received_at: float,
        sequence: int,
        source_system: int,
        source_component: int,
    ) -> bool:
        with self._lock:
            accepted = self.update(
                "failsafe",
                (active, reason),
                received_at=received_at,
                sequence=sequence,
                source_system=source_system,
                source_component=source_component,
            )
            observation = self._observations.get("failsafe")
            if (
                accepted
                and active is True
                and observation is not None
                and is_fresh(
                    observation, self._clock(), self.freshness_bounds["failsafe"]
                )
            ):
                self._authority = Authority.FC_FAILSAFE
                self._commands_suspended = True
                self._attempt_revoked = True
                self._permission_generation += 1
                self._raise_if_reentrant_output_revoked_locked()
            return accepted

    def invalidate_observation(
        self,
        field: str,
        *,
        source_system: int,
        source_component: int,
        received_at: float | None = None,
        sequence: int | None = None,
    ) -> bool:
        return self.invalidate_observations(
            (field,),
            source_system=source_system,
            source_component=source_component,
            received_at=received_at,
            sequence=sequence,
        )

    def invalidate_evidence(
        self,
        field_names: Sequence[str],
        *,
        source_system: int,
        source_component: int,
        received_at: float,
        sequence: int,
    ) -> bool:
        """Remove non-authority evidence while retaining an ordering tombstone."""
        names = tuple(field_names)
        unknown = set(names) - set(_FIELD_NAMES)
        if not names:
            raise ValueError("field_names must contain at least one field")
        if unknown:
            raise ValueError(f"unknown flight observation field: {sorted(unknown)[0]}")
        if SourceIdentity(source_system, source_component) != self.source:
            return False
        with self._lock:
            if (
                not isinstance(received_at, Real)
                or not math.isfinite(float(received_at))
                or not isinstance(sequence, int)
                or isinstance(sequence, bool)
                or sequence < 0
                or not self._is_newer_locked(names, float(received_at), sequence)
            ):
                return False
            self._invalidate_locked(
                names, float(received_at), sequence, revoke_authority=False
            )
            return True

    def invalidate_observations(
        self,
        field_names: Sequence[str],
        *,
        source_system: int,
        source_component: int,
        received_at: float | None = None,
        sequence: int | None = None,
    ) -> bool:
        """Atomically remove unusable evidence and revoke stale authorization."""
        names = tuple(field_names)
        unknown = set(names) - set(_FIELD_NAMES)
        if not names:
            raise ValueError("field_names must contain at least one field")
        if unknown:
            raise ValueError(f"unknown flight observation field: {sorted(unknown)[0]}")
        if SourceIdentity(source_system, source_component) != self.source:
            return False
        with self._lock:
            if (received_at is None) != (sequence is None):
                raise ValueError("received_at and sequence must be supplied together")
            if received_at is not None and sequence is not None:
                if (
                    not isinstance(received_at, Real)
                    or not math.isfinite(float(received_at))
                    or not isinstance(sequence, int)
                    or isinstance(sequence, bool)
                    or sequence < 0
                    or not self._is_newer_locked(names, float(received_at), sequence)
                ):
                    return False
                self._invalidate_locked(names, float(received_at), sequence)
            else:
                for field in names:
                    self._observations.pop(field, None)
                if "rc_input" in names:
                    self._rc_edge = None
                    self._last_healthy_rc = None
                    self._rc_baseline_slot = None
                self._revoke_to_unknown_locked()
            return True

    def _invalidate_locked(
        self,
        field_names: Sequence[str],
        received_at: float,
        sequence: int,
        *,
        revoke_authority: bool = True,
    ) -> None:
        for field in field_names:
            self._observations.pop(field, None)
            self._latest_updates[field] = (received_at, sequence)
            self._invalidation_generations[field] += 1
        if "rc_input" in field_names:
            self._rc_edge = None
            self._last_healthy_rc = None
            self._rc_baseline_slot = None
        if revoke_authority:
            self._revoke_to_unknown_locked()

    def command_permission(self) -> CommandPermission | None:
        """Return a short-lived permission for Task 4's final send check."""
        with self._lock:
            if self._authority is not Authority.COMPANION or self._commands_suspended:
                return None
            return CommandPermission(self._permission_generation)

    def permission_is_current(self, permission: CommandPermission) -> bool:
        with self._lock:
            return (
                isinstance(permission, CommandPermission)
                and permission.generation == self._permission_generation
                and self._authority is Authority.COMPANION
                and not self._commands_suspended
            )

    def execute_command_output(
        self,
        permission: CommandPermission,
        *,
        output_gate: Callable[[Callable[[], object]], object],
        validate_snapshot: Callable[[FlightSnapshot], None] | None,
        output: Callable[[], object],
    ) -> tuple[FlightSnapshot, object]:
        """Linearize final flight evidence, abort state, and one real output."""
        if not callable(output_gate) or not callable(output):
            raise TypeError("output gate and output must be callable")
        with self._lock:
            if not self.permission_is_current(permission):
                raise PermissionError("companion flight authority changed before output")
            boundary: list[FlightSnapshot] = []
            result: list[object] = []

            def gated_output() -> object:
                current = self.snapshot()
                if validate_snapshot is not None:
                    validate_snapshot(current)
                if not self.permission_is_current(permission):
                    raise PermissionError(
                        "companion flight authority changed at final output boundary"
                    )
                boundary.append(current)
                self._output_thread_id = threading.get_ident()
                try:
                    value = output()
                finally:
                    self._output_thread_id = None
                result.append(value)
                return value

            output_gate(gated_output)
            if not boundary:
                raise RuntimeError("output gate did not execute its transaction")
            return boundary[0], result[0]
