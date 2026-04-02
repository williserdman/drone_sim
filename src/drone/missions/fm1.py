"""
Mission:
Begins at Home, takes off to 30ft AGL, transits horizontally to waypoint L, and lands.
Must be carrying at least one payload to proceed to fm2.
Wait for flagger at L to confirm aircraft is inside the box, and judge at H to approve the vertical landing quality, before starting fm2.
"""

from ..common_types import *
from ..control.drone_control import DroneControl
from ..control.mission_info import MissonTracker
from .utils import log, warn


def fm1(controller: DroneControl, mt: MissonTracker, L: GPSCoord, cruise_alt: int):
    log("fm1: taking off from Home")
    controller.takeoff(cruise_alt)

    log("fm1: transiting to waypoint L")
    controller.goto_waypoint(GPSCoord(L.lat, L.long, cruise_alt))

    controller.simple_land()

    log("fm1: landed at L, awaiting flagger and judge approval to start fm2")
