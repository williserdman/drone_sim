from dataclasses import dataclass


@dataclass
class RelativePosition:
    x: float
    y: float


@dataclass
class RelPosComplete:
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
