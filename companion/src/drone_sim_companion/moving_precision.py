"""Pure moving-target precision-landing policy driven by public simulation time."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping


MAX_EVIDENCE_AGE_NS = 250_000_000
SETTLE_DURATION_NS = 500_000_000
TRACKING_REQUIRED_NS = 2_000_000_000
TRACKING_LOSS_NS = 500_000_000
FINAL_HANDOFF_NS = 3_000_000_000
MAX_SETTLED_SPEED_M_S = 0.2
FINAL_CLEARANCE_M = 0.75


@dataclass(frozen=True)
class RangeEvidence:
    distance_m: float
    timestamp_ns: int


@dataclass(frozen=True)
class AttitudeEvidence:
    rpy_rad: tuple[float, float, float]
    timestamp_ns: int


@dataclass(frozen=True)
class MovingObservation:
    camera_timestamp_ns: int
    camera_sequence: int
    marker_id: int
    target_body_frd: tuple[float, float, float]
    range_m: float
    range_timestamp_ns: int
    attitude_rpy_rad: tuple[float, float, float]
    attitude_timestamp_ns: int


@dataclass(frozen=True)
class PrecisionStatus:
    state: str
    error: str = ""
    target_body_frd: tuple[float, float, float] | None = None
    target_timestamp_ns: int | None = None
    requested_mode: str | None = None


def _finite(values: tuple[float, ...]) -> bool:
    return all(type(value) in (int, float) and math.isfinite(value) for value in values)


class MovingPrecisionLanding:
    """Return flight effects for the single owner thread to apply once."""

    def __init__(self) -> None:
        self._started = False
        self._marker_id = -1
        self._settle_by_ns = 0
        self._acquire_by_ns = 0
        self._state = "running"
        self._error = ""
        self._phase = "settling"
        self._settled_since_ns: int | None = None
        self._tracking_since_ns: int | None = None
        self._last_valid_ns: int | None = None
        self._last_sequence = 0
        self._forwarded_sequence = 0
        self._evaluated_sequence = 0
        self._latest: MovingObservation | None = None
        self._mode_effect: str | None = None
        self._land_requested = False
        self._final_handoff_by_ns: int | None = None
        self._last_clearance_m: float | None = None
        self._touchdown_observed = False

    def start(self, marker_id: int, settle_by_ns: int, acquire_by_ns: int) -> None:
        if type(marker_id) is not int or marker_id < 0:
            raise ValueError("marker_id must be a non-negative integer")
        if type(settle_by_ns) is not int or settle_by_ns <= 0:
            raise ValueError("settle_by_ns must be a positive integer")
        if type(acquire_by_ns) is not int or acquire_by_ns <= settle_by_ns:
            raise ValueError("acquire_by_ns must be after settle_by_ns")
        if self._started:
            raise RuntimeError("precision landing is already started")
        self._started = True
        self._marker_id = marker_id
        self._settle_by_ns = settle_by_ns
        self._acquire_by_ns = acquire_by_ns

    def observe(self, observation: MovingObservation) -> None:
        if not isinstance(observation, MovingObservation):
            raise TypeError("observation must be MovingObservation")
        if observation.camera_sequence <= self._last_sequence:
            return
        self._last_sequence = observation.camera_sequence
        self._latest = observation

    def _usable(self, observation: MovingObservation, timestamp_ns: int) -> bool:
        if observation.marker_id != self._marker_id:
            return False
        times = (
            observation.camera_timestamp_ns,
            observation.range_timestamp_ns,
            observation.attitude_timestamp_ns,
        )
        if any(type(value) is not int or value < 0 or value > timestamp_ns for value in times):
            return False
        if timestamp_ns - min(times) > MAX_EVIDENCE_AGE_NS:
            return False
        if max(times) - min(times) > MAX_EVIDENCE_AGE_NS:
            return False
        if (
            not _finite(observation.target_body_frd)
            or not _finite((observation.range_m, *observation.attitude_rpy_rad))
            or observation.range_m <= 0
            or observation.target_body_frd[2] <= 0
        ):
            return False
        return True

    def _fail(self, error: str, *, requested_mode: str | None = None) -> None:
        self._state = "failed"
        self._error = error
        self._mode_effect = requested_mode

    def tick(self, timestamp_ns: int, vehicle_state: Mapping[str, object]) -> PrecisionStatus:
        if not self._started:
            raise RuntimeError("precision landing has not started")
        if type(timestamp_ns) is not int or timestamp_ns < 0:
            raise ValueError("timestamp_ns must be a non-negative integer")
        if self._state != "running":
            mode = self._mode_effect
            self._mode_effect = None
            return PrecisionStatus(self._state, self._error, requested_mode=mode)

        target: tuple[float, float, float] | None = None
        target_timestamp_ns: int | None = None
        observation = self._latest
        usable = observation is not None and self._usable(observation, timestamp_ns)
        new_usable = (
            usable
            and observation is not None
            and observation.camera_sequence > self._evaluated_sequence
        )
        if new_usable:
            assert observation is not None
            if observation.camera_sequence > self._forwarded_sequence:
                target = observation.target_body_frd
                target_timestamp_ns = observation.camera_timestamp_ns
                self._forwarded_sequence = observation.camera_sequence
            if (
                self._tracking_since_ns is None
                or self._last_valid_ns is None
                or observation.camera_timestamp_ns - self._last_valid_ns > MAX_EVIDENCE_AGE_NS
            ):
                self._tracking_since_ns = observation.camera_timestamp_ns
            self._last_valid_ns = observation.camera_timestamp_ns
            self._last_clearance_m = observation.target_body_frd[2]
            self._evaluated_sequence = observation.camera_sequence

        if self._phase == "settling":
            speed = vehicle_state.get("horizontal_speed_m_s")
            speed_stamp = vehicle_state.get("horizontal_speed_timestamp_ns", timestamp_ns)
            speed_fresh = (
                type(speed_stamp) is int
                and 0 <= timestamp_ns - speed_stamp <= MAX_EVIDENCE_AGE_NS
            )
            speed_valid = (
                not isinstance(speed, bool)
                and isinstance(speed, (int, float))
                and math.isfinite(speed)
                and speed <= MAX_SETTLED_SPEED_M_S
            )
            if speed_valid and speed_fresh:
                if self._settled_since_ns is None:
                    self._settled_since_ns = timestamp_ns
                if timestamp_ns - self._settled_since_ns >= SETTLE_DURATION_NS:
                    self._phase = "acquiring"
                    self._tracking_since_ns = (
                        observation.camera_timestamp_ns
                        if new_usable and observation is not None
                        else None
                    )
            else:
                self._settled_since_ns = None
            if self._phase == "settling" and timestamp_ns > self._settle_by_ns:
                self._fail("vehicle did not settle by the precision-landing deadline")

        if self._phase == "acquiring":
            if not usable and self._last_valid_ns is not None and timestamp_ns - self._last_valid_ns > MAX_EVIDENCE_AGE_NS:
                self._tracking_since_ns = None
            if (
                new_usable
                and self._tracking_since_ns is not None
                and observation is not None
                and observation.camera_timestamp_ns - self._tracking_since_ns >= TRACKING_REQUIRED_NS
            ):
                self._phase = "landing"
                self._land_requested = True
                self._mode_effect = "LAND"
                clearance = self._last_clearance_m or math.inf
                if clearance <= FINAL_CLEARANCE_M:
                    self._final_handoff_by_ns = timestamp_ns + FINAL_HANDOFF_NS
            elif timestamp_ns > self._acquire_by_ns:
                self._fail("target was not acquired by the precision-landing deadline")

        if self._phase == "landing":
            if vehicle_state.get("landed") is True:
                self._touchdown_observed = True
            if self._touchdown_observed and vehicle_state.get("armed") is False:
                self._state = "succeeded"
            else:
                clearance = self._last_clearance_m or math.inf
                if clearance <= FINAL_CLEARANCE_M and self._final_handoff_by_ns is None:
                    if self._last_valid_ns is not None and timestamp_ns - self._last_valid_ns <= TRACKING_LOSS_NS:
                        self._final_handoff_by_ns = timestamp_ns + FINAL_HANDOFF_NS
                if self._final_handoff_by_ns is not None:
                    if not self._touchdown_observed and timestamp_ns > self._final_handoff_by_ns:
                        self._fail("touchdown did not follow final precision handoff within three seconds")
                elif self._last_valid_ns is None or timestamp_ns - self._last_valid_ns > TRACKING_LOSS_NS:
                    self._fail("precision target tracking was lost above final clearance", requested_mode="GUIDED")

        mode = self._mode_effect
        self._mode_effect = None
        return PrecisionStatus(
            self._state,
            self._error,
            target,
            target_timestamp_ns,
            mode,
        )

    def abort(self, reason: str) -> PrecisionStatus:
        if self._state == "running":
            self._fail(reason or "precision landing aborted")
        return PrecisionStatus(self._state, self._error)


__all__ = [
    "AttitudeEvidence",
    "MovingObservation",
    "MovingPrecisionLanding",
    "PrecisionStatus",
    "RangeEvidence",
]
