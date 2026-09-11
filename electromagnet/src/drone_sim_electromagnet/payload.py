"""Pure payload request policy with no ROS or Gazebo side effects."""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
from typing import Mapping


_COMMAND_ID = re.compile(r"[A-Za-z0-9_.:-]{1,128}\Z")


@dataclass(frozen=True)
class PickupZone:
    center_x_m: float
    center_y_m: float
    width_m: float
    height_m: float

    def contains(self, xy: tuple[float, float]) -> bool:
        x, y = xy
        return (
            abs(x - self.center_x_m) <= self.width_m / 2.0
            and abs(y - self.center_y_m) <= self.height_m / 2.0
        )


@dataclass(frozen=True)
class PayloadRequest:
    run_id: str
    aruco_id: int
    action: str
    command_id: str


@dataclass(frozen=True)
class PayloadDecision:
    accepted: bool
    code: str
    wire_command: str | None


@dataclass(frozen=True)
class PayloadWorld:
    vehicle_xy: tuple[float, float]
    vehicle_grounded: bool
    payload_xy: tuple[float, float]
    payload_grounded: bool
    attached_id: int | None


class PayloadAuthority:
    """Validate one-capacity payload actions against current physical facts."""

    def __init__(
        self,
        *,
        run_id: str,
        pickup_zones: Mapping[str, PickupZone],
        payload_zones: Mapping[int, str | None],
        payload_capacity: int,
        max_center_error_m: float,
    ) -> None:
        if payload_capacity != 1:
            raise ValueError("payload capacity must be one")
        if not math.isfinite(max_center_error_m) or max_center_error_m <= 0:
            raise ValueError("max center error must be positive and finite")
        self.run_id = run_id
        self._pickup_zones = dict(pickup_zones)
        self._payload_zones = dict(payload_zones)
        self._max_center_error_m = max_center_error_m

    def decide(self, world: PayloadWorld, request: PayloadRequest) -> PayloadDecision:
        if request.run_id != self.run_id:
            return PayloadDecision(False, "STALE_RUN", None)
        if type(request.command_id) is not str or _COMMAND_ID.fullmatch(
            request.command_id
        ) is None:
            return PayloadDecision(False, "INVALID_COMMAND_ID", None)
        if type(request.aruco_id) is not int or request.aruco_id not in self._payload_zones:
            return PayloadDecision(False, "UNKNOWN_MARKER", None)
        if request.action not in {"attach", "release"}:
            return PayloadDecision(False, "INVALID_ACTION", None)
        if request.action == "release":
            if world.attached_id != request.aruco_id:
                return PayloadDecision(False, "NOT_ATTACHED", None)
            return PayloadDecision(
                True,
                "OK",
                f"payload-command-v1|{request.command_id}|detach",
            )

        zone_name = self._payload_zones[request.aruco_id]
        if zone_name is None or zone_name not in self._pickup_zones:
            return PayloadDecision(False, "WRONG_PICKUP_ZONE", None)
        zone = self._pickup_zones[zone_name]
        if not world.vehicle_grounded:
            return PayloadDecision(False, "NOT_LANDED", None)
        if not zone.contains(world.vehicle_xy):
            return PayloadDecision(False, "OUTSIDE_PICKUP_ZONE", None)
        if not zone.contains(world.payload_xy):
            return PayloadDecision(False, "WRONG_PICKUP_ZONE", None)
        if not world.payload_grounded:
            return PayloadDecision(False, "PAYLOAD_NOT_GROUNDED", None)
        if world.attached_id is not None:
            return PayloadDecision(False, "CAPACITY_OCCUPIED", None)
        center_error = math.hypot(
            world.vehicle_xy[0] - world.payload_xy[0],
            world.vehicle_xy[1] - world.payload_xy[1],
        )
        if center_error > self._max_center_error_m and not math.isclose(
            center_error,
            self._max_center_error_m,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            return PayloadDecision(False, "CENTER_ERROR", None)
        return PayloadDecision(
            True,
            "OK",
            f"payload-command-v1|{request.command_id}|attach",
        )


__all__ = [
    "PayloadAuthority",
    "PayloadDecision",
    "PayloadRequest",
    "PayloadWorld",
    "PickupZone",
]
