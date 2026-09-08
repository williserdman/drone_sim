"""
Begins at waypoint L. Takes off to 30ft AGL, transits to a fire zone, and drops payloads. Drops
must occur at or above 15ft AGL. Dropping below 30ft AGL halves all payload points. Dropping below
15ft loses them entirely. Drops do not need to be successful.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from ..common_types import *
from ..control.drone_control import DroneControl
from ..control.mission_info import MissonTracker
from .utils import log, warn

if TYPE_CHECKING:
    from ..sensors.lidar.lidar import Lidar
    from ..sensors.servo.servo import Dropper


def _require_takeoff_success(result: object) -> None:
    if result is None or (type(result) is int and result == 0):
        return
    raise RuntimeError("FM2 takeoff was not confirmed")


def _require_legacy_success(result: object, operation: str) -> None:
    if type(result) is int and result == 0:
        return
    raise RuntimeError(f"FM2 {operation} was not confirmed")


def fm2(
    mt: MissonTracker,
    controller: DroneControl,
    cruise_alt: int,
    drop_target: GPSCoord,
    dropper: Dropper,
    lidar: Lidar,
    desired_drop_height_m: int = 10,
) -> bool:
    if (
        isinstance(desired_drop_height_m, bool)
        or not isinstance(desired_drop_height_m, (int, float))
        or not math.isfinite(desired_drop_height_m)
        or desired_drop_height_m <= 0
    ):
        raise ValueError("desired_drop_height_m must be finite and positive")

    transit_waypoint = GPSCoord(drop_target.lat, drop_target.long, cruise_alt)
    controller.require_release_configuration()

    ### takeoff from waypoint L
    controller.check_permission()
    log("fm2: taking off from L", controller.vehicle._master)
    _require_takeoff_success(controller.force_arm_takeoff(cruise_alt))

    ### transit from fire zone to drop
    controller.check_permission()
    log("fm2: transiting to drop zone", controller.vehicle._master)
    _require_legacy_success(
        controller.goto_waypoint(transit_waypoint),
        "navigation",
    )

    controller.check_permission()
    stable_waypoint = controller.release_waypoint_for_clearance(
        transit_waypoint,
        lidar,
        desired_agl_m=desired_drop_height_m,
    )
    controller.check_permission()
    _require_legacy_success(
        controller.goto_waypoint(stable_waypoint),
        "release-height correction",
    )
    if controller.hold_waypoint_until_stable(
        stable_waypoint,
        lidar,
        required_agl_m=desired_drop_height_m,
    ) is not True:
        warn("fm2: release stability gate timed out", controller.vehicle._master)
        return False

    controller.release_payload_if_stable(
        dropper,
        stable_waypoint,
        lidar,
        required_agl_m=desired_drop_height_m,
    )
    # TODO: Figure out what to do at the end of fm2. Land and proceed with FM3?

    log(
        "fm2: payload release command attempted; physical release unconfirmed",
        controller.vehicle._master,
    )
    return True
