"""Configuration and immutable evidence for payload-release stability."""

from __future__ import annotations

from dataclasses import dataclass
import math


def _positive_number(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be finite and positive")
    normalized = float(value)
    if not math.isfinite(normalized) or normalized <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return normalized


@dataclass(frozen=True)
class ReleaseStabilityConfig:
    """Measured limits and timing policy required to authorize a release."""

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

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            object.__setattr__(self, name, _positive_number(name, getattr(self, name)))
        if self.timeout_seconds <= self.hold_seconds:
            raise ValueError("timeout_seconds must be greater than hold_seconds")
        if self.poll_interval_seconds > self.max_observation_gap_seconds:
            raise ValueError(
                "poll_interval_seconds must not exceed max_observation_gap_seconds"
            )
        if self.max_roll_rad >= math.pi / 2:
            raise ValueError("max_roll_rad must be less than pi/2")
        if self.max_pitch_rad >= math.pi / 2:
            raise ValueError("max_pitch_rad must be less than pi/2")


@dataclass(frozen=True)
class ReleaseEvidence:
    """One coherent observation set used to evaluate the release limits."""

    location_sequence: int
    velocity_sequence: int
    attitude_sequence: int
    location_invalidation_generation: int
    velocity_invalidation_generation: int
    attitude_invalidation_generation: int
    range_sequence: int
    range_invalidation_generation: int
    observed_at: float
    horizontal_error_m: float
    vertical_error_m: float
    projected_clearance_m: float
    horizontal_speed_m_s: float
    vertical_speed_m_s: float
    roll_rad: float
    pitch_rad: float


@dataclass(frozen=True)
class ReleaseHoldConfirmation:
    """One-use proof that a particular release target completed its hold."""

    waypoint: object
    lidar_identity: int
    required_agl_m: float
    evidence: ReleaseEvidence
