"""Calibrated LiDAR projection onto an assumed horizontal local surface."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .lidar import LidarSample


class ClearanceConfigurationError(ValueError):
    """Raised when projection calibration is incomplete or unsupported."""


class ClearanceUnavailableError(RuntimeError):
    """Raised when current observations cannot support a clearance estimate."""


@dataclass(frozen=True)
class AttitudeSample:
    """Body FRD attitude in local NED, with monotonic receipt metadata."""

    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    sampled_at: float
    sequence: int


@dataclass(frozen=True)
class ClearanceCalibration:
    """Explicit geometry and validity bounds for vertical projection.

    ``measured_reference_offset_body_frd_m`` points from the vehicle body
    reference to the origin of ``LidarSample.distance_m``. The LiDAR adapter
    has already subtracted ``mounting_offset_cm`` along the measured range, so
    callers must declare that correction and must not include it again here.
    ``locally_horizontal_planar_surface`` declares the only supported surface
    model. A sloped plane is unsupported.
    """

    beam_direction_body_frd: tuple[float, float, float]
    measured_reference_offset_body_frd_m: tuple[float, float, float]
    lidar_mounting_offset_already_applied: bool
    max_tilt_rad: float
    max_age_seconds: float
    max_skew_seconds: float
    locally_horizontal_planar_surface: bool

    def __post_init__(self) -> None:
        beam = _finite_vector(
            "beam_direction_body_frd", self.beam_direction_body_frd
        )
        offset = _finite_vector(
            "measured_reference_offset_body_frd_m",
            self.measured_reference_offset_body_frd_m,
        )
        norm = math.sqrt(sum(component * component for component in beam))
        if not math.isclose(norm, 1.0, rel_tol=1e-9, abs_tol=1e-9):
            raise ClearanceConfigurationError(
                "beam_direction_body_frd must be a unit vector"
            )
        if beam[2] <= 0:
            raise ClearanceConfigurationError(
                "beam_direction_body_frd must point into the body-down hemisphere"
            )
        if self.lidar_mounting_offset_already_applied is not True:
            raise ClearanceConfigurationError(
                "lidar_mounting_offset_already_applied must be explicitly true"
            )
        if self.locally_horizontal_planar_surface is not True:
            raise ClearanceConfigurationError(
                "locally_horizontal_planar_surface must be explicitly true"
            )
        max_tilt = _finite_scalar("max_tilt_rad", self.max_tilt_rad)
        max_age = _finite_scalar("max_age_seconds", self.max_age_seconds)
        max_skew = _finite_scalar("max_skew_seconds", self.max_skew_seconds)
        if not 0 < max_tilt < math.pi / 2:
            raise ClearanceConfigurationError(
                "max_tilt_rad must be positive and less than pi/2"
            )
        if max_age <= 0:
            raise ClearanceConfigurationError("max_age_seconds must be positive")
        if max_skew <= 0:
            raise ClearanceConfigurationError("max_skew_seconds must be positive")
        object.__setattr__(self, "beam_direction_body_frd", beam)
        object.__setattr__(self, "measured_reference_offset_body_frd_m", offset)
        object.__setattr__(self, "max_tilt_rad", max_tilt)
        object.__setattr__(self, "max_age_seconds", max_age)
        object.__setattr__(self, "max_skew_seconds", max_skew)


@dataclass(frozen=True)
class ProjectedVerticalClearance:
    """A derived NED-down clearance, retaining both source observations."""

    projected_clearance_m: float
    range_sample: LidarSample
    attitude_sample: AttitudeSample


def _finite_scalar(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ClearanceConfigurationError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (OverflowError, ValueError):
        raise ClearanceConfigurationError(f"{name} must be a finite number") from None
    if not math.isfinite(result):
        raise ClearanceConfigurationError(f"{name} must be a finite number")
    return result


def _finite_vector(name: str, value: object) -> tuple[float, float, float]:
    if not isinstance(value, (tuple, list)) or len(value) != 3:
        raise ClearanceConfigurationError(f"{name} must contain three numbers")
    normalized = tuple(
        _finite_scalar(f"{name}[{index}]", item)
        for index, item in enumerate(value)
    )
    return normalized  # type: ignore[return-value]


def _rotate_body_frd_to_local_ned(
    vector: tuple[float, float, float],
    *,
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float]:
    x, y, z = vector
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        cy * cp * x + (cy * sp * sr - sy * cr) * y + (cy * sp * cr + sy * sr) * z,
        sy * cp * x + (sy * sp * sr + cy * cr) * y + (sy * sp * cr - cy * sr) * z,
        -sp * x + cp * sr * y + cp * cr * z,
    )


def project_vertical_clearance(
    range_sample: LidarSample,
    attitude_sample: AttitudeSample,
    calibration: ClearanceCalibration,
    *,
    now: float,
) -> ProjectedVerticalClearance:
    """Project corrected beam range onto NED down over horizontal local ground."""
    if not isinstance(range_sample, LidarSample):
        raise ClearanceUnavailableError("range_sample must be a LidarSample")
    if not isinstance(attitude_sample, AttitudeSample):
        raise ClearanceUnavailableError("attitude_sample must be an AttitudeSample")
    distance_m = _observation_scalar("range distance_m", range_sample.distance_m)
    if distance_m < 0:
        raise ClearanceUnavailableError("range distance_m must be non-negative")
    _positive_sequence("range sequence", range_sample.sequence)
    if (
        not isinstance(range_sample.invalidation_generation, int)
        or isinstance(range_sample.invalidation_generation, bool)
        or range_sample.invalidation_generation < 0
    ):
        raise ClearanceUnavailableError(
            "range invalidation_generation must be a non-negative integer"
        )
    roll = _observation_scalar("attitude roll_rad", attitude_sample.roll_rad)
    pitch = _observation_scalar("attitude pitch_rad", attitude_sample.pitch_rad)
    yaw = _observation_scalar("attitude yaw_rad", attitude_sample.yaw_rad)
    _positive_sequence("attitude sequence", attitude_sample.sequence)
    current_time = _observation_scalar("now", now)
    range_time = _observation_scalar("range sampled_at", range_sample.sampled_at)
    range_age = current_time - range_time
    if range_age < 0:
        raise ClearanceUnavailableError("range sample is future-dated")
    if range_age > calibration.max_age_seconds:
        raise ClearanceUnavailableError("range sample is stale")
    attitude_time = _observation_scalar(
        "attitude sampled_at", attitude_sample.sampled_at
    )
    attitude_age = current_time - attitude_time
    if attitude_age < 0:
        raise ClearanceUnavailableError("attitude sample is future-dated")
    if attitude_age > calibration.max_age_seconds:
        raise ClearanceUnavailableError("attitude sample is stale")
    if abs(range_time - attitude_time) > calibration.max_skew_seconds:
        raise ClearanceUnavailableError("range-attitude sample skew exceeds its bound")
    beam_ned = _rotate_body_frd_to_local_ned(
        calibration.beam_direction_body_frd,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
    )
    beam_down = max(-1.0, min(1.0, beam_ned[2]))
    if beam_down <= 0:
        raise ClearanceUnavailableError("LiDAR beam does not point downward")
    tilt = math.acos(beam_down)
    if tilt > calibration.max_tilt_rad and not math.isclose(
        tilt, calibration.max_tilt_rad, rel_tol=1e-12, abs_tol=1e-12
    ):
        raise ClearanceUnavailableError("LiDAR beam tilt exceeds its bound")
    offset_ned = _rotate_body_frd_to_local_ned(
        calibration.measured_reference_offset_body_frd_m,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
    )
    projected = offset_ned[2] + distance_m * beam_ned[2]
    if not math.isfinite(projected) or projected < 0:
        raise ClearanceUnavailableError("projected clearance is negative or nonfinite")
    return ProjectedVerticalClearance(projected, range_sample, attitude_sample)


def _observation_scalar(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ClearanceUnavailableError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (OverflowError, ValueError):
        raise ClearanceUnavailableError(f"{name} must be a finite number") from None
    if not math.isfinite(result):
        raise ClearanceUnavailableError(f"{name} must be a finite number")
    return result


def _positive_sequence(name: str, value: object) -> None:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ClearanceUnavailableError(f"{name} must be a positive integer")
