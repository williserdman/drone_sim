"""
fm3: Pickup from WA, drop at fire zone, loop with WM payloads, return home.
"""

from ..common_types import *
from ..control.drone_control import DroneControl
from ..control.mission_info import MissonTracker
from .utils import log, warn
import time

# stop looping and return home when this many seconds are left in the mission
HOME_BUFFER = 60


def drop():
    pass


def pickup():
    pass


def fm3(controller: DroneControl, mt: MissonTracker, H: GPSCoord, WA: GPSCoord, WM: GPSCoord, drop_target: GPSCoord, cruise_alt: float):
    ### FIRST PICKUP FROM WA
    log("fm3: transiting to WA for first pickup")
    controller.goto_waypoint(GPSCoord(WA.lat, WA.long, cruise_alt))
    controller.simple_land()
    pickup()
    controller.takeoff(cruise_alt)

    ### FIRST DROP
    log("fm3: transiting to drop zone for first drop")
    controller.goto_waypoint(GPSCoord(drop_target.lat, drop_target.long, cruise_alt))
    drop()
    log("fm3: first drop complete")

    ### LOOP — pickup from WM and drop until time runs low
    while mt.time_left() > HOME_BUFFER:
        log("fm3: transiting to WM for pickup")
        controller.goto_waypoint(GPSCoord(WM.lat, WM.long, cruise_alt))
        controller.simple_land()
        pickup()
        controller.takeoff(cruise_alt)

        log("fm3: transiting to drop zone")
        controller.goto_waypoint(GPSCoord(drop_target.lat, drop_target.long, cruise_alt))
        drop()
        log("fm3: drop complete")

    ### RETURN HOME AND LAND
    warn("fm3: time running low, returning home")
    controller.goto_waypoint(GPSCoord(H.lat, H.long, cruise_alt))
    controller.simple_land()
    log("fm3: landed at home")