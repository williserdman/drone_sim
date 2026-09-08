from dataclasses import dataclass


@dataclass
class RelativePosition:
    x: float
    y: float


@dataclass
class RelPosComplete:
    """Body-frame metres in forward, right, down (FRD) order."""

    x: float
    y: float
    z: float


@dataclass
class NEDMeters:
    north: float
    east: float
    down: float


@dataclass
class GPSCoord:
    lat: float
    long: float
    alt: float


@dataclass(frozen=True)
class MissionHome:
    lat: float
    lon: float
    amsl_m: float
