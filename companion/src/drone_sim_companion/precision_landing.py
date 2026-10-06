"""Pure geometry and policy values for tick-driven precision landing."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping

from .configured_vision import MarkerObservation


TARGET_HEALTH_NS = 500_000_000
HOLD_TIMEOUT_NS = 5_000_000_000
HOLD_REFRESH_NS = 200_000_000
PRECISION_TIMEOUT_NS = 60_000_000_000
NEAR_GROUND_M = 0.75
ANCHOR_DRIFT_M = 0.20
REACQUIRE_FRAMES = 5

PRECISION_PARAMETERS: Mapping[str, int | float] = {
    "LAND_SPD_MS": 0.50,
    "PLND_ENABLED": 1,
    "PLND_TYPE": 1,
    "PLND_EST_TYPE": 0,
    "PLND_LAG": 0.08,
    "PLND_XY_DIST_MAX": 0.50,
    "PLND_STRICT": 2,
    "PLND_RET_MAX": 1,
    "PLND_TIMEOUT": 0.50,
    "PLND_ALT_MIN": 0.75,
    "PLND_ALT_MAX": 8.0,
    "PLND_OPTIONS": 4,
}


@dataclass(frozen=True)
class MarkerAnchor:
    aruco_id: int
    latitude_deg: float
    longitude_deg: float
    acquired_at_ns: int
    frame_sequence: int


@dataclass(frozen=True)
class BodyFrd:
    forward_m: float
    right_m: float
    down_m: float


def profile_matches(values: Mapping[str, object]) -> bool:
    for name, expected in PRECISION_PARAMETERS.items():
        actual = values.get(name)
        if isinstance(actual, bool) or not isinstance(actual, (int, float)):
            return False
        if not math.isfinite(float(actual)):
            return False
        if isinstance(expected, int):
            if not float(actual).is_integer() or int(actual) != expected:
                return False
        elif not math.isclose(float(actual), expected, rel_tol=0.0, abs_tol=1e-3):
            return False
    return True


def marker_offset_ne(
    observation: MarkerObservation,
    *,
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> tuple[float, float]:
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    forward, right, down = (
        observation.forward_m,
        observation.right_m,
        observation.down_m,
    )
    north = (
        cy * cp * forward
        + (cy * sp * sr - sy * cr) * right
        + (cy * sp * cr + sy * sr) * down
    )
    east = (
        sy * cp * forward
        + (sy * sp * sr + cy * cr) * right
        + (sy * sp * cr - cy * sr) * down
    )
    return north, east


def offset_gps(
    latitude_deg: float,
    longitude_deg: float,
    north_m: float,
    east_m: float,
) -> tuple[float, float]:
    radius = 6_378_137.0
    return (
        latitude_deg + math.degrees(north_m / radius),
        longitude_deg
        + math.degrees(east_m / (radius * math.cos(math.radians(latitude_deg)))),
    )


def horizontal_distance_m(
    first: tuple[float, float], second: tuple[float, float]
) -> float:
    latitude = math.radians((first[0] + second[0]) / 2.0)
    north = math.radians(second[0] - first[0]) * 6_378_137.0
    east = math.radians(second[1] - first[1]) * math.cos(latitude) * 6_378_137.0
    return math.hypot(north, east)


__all__ = [
    "ANCHOR_DRIFT_M",
    "BodyFrd",
    "HOLD_REFRESH_NS",
    "HOLD_TIMEOUT_NS",
    "MarkerAnchor",
    "NEAR_GROUND_M",
    "PRECISION_PARAMETERS",
    "PRECISION_TIMEOUT_NS",
    "REACQUIRE_FRAMES",
    "TARGET_HEALTH_NS",
    "horizontal_distance_m",
    "marker_offset_ne",
    "offset_gps",
    "profile_matches",
]
