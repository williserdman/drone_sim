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
