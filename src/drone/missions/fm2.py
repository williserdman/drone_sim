"""
Begins at waypoint L. Takes off to 30ft AGL, transits to a fire zone, and drops payloads. Drops
must occur at or above 15ft AGL. Dropping below 30ft AGL halves all payload points. Dropping below
15ft loses them entirely. Drops do not need to be successful.
"""

from __future__ import annotations

from ..common_types import *
from ..control.drone_control import DroneControl
from ..control.mission_info import MissonTracker
from .utils import log, warn
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..sensors.lidar.lidar import Lidar
    from ..sensors.servo.servo import Dropper


def fm2(
    mt: MissonTracker,
    controller: DroneControl,
    cruise_alt: int,
    drop_target: GPSCoord,
    dropper: Dropper,
    lidar: Lidar,
    desired_drop_height_m: int = 10,
):
    target_gps = GPSCoord(drop_target.lat, drop_target.long, cruise_alt)

    ### takeoff from waypoint L
    log("fm2: taking off from L", controller.vehicle._master)
    controller.force_arm_takeoff(cruise_alt)

    ### transit from fire zone to drop
    log("fm2: transiting to drop zone", controller.vehicle._master)
    controller.goto_waypoint(target_gps)
    lidar_alt = lidar.get_distance()
    desired_drop_agl = desired_drop_height_m
    if lidar_alt < desired_drop_agl:
        target_gps.alt += desired_drop_agl - lidar_alt
    if not controller.hold_waypoint_until_stable(target_gps):
        warn("fm2: release stability gate timed out", controller.vehicle._master)
        return False
    dropper.drop()
    # TODO: Figure out what to do at the end of fm2. Land and proceed with FM3?

    log("fm2: payload dropped", controller.vehicle._master)
    return True
