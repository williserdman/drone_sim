"""
Begins at waypoint L. Takes off to 30ft AGL, transits to a fire zone, and drops payloads. Drops 
must occur at or above 15ft AGL. Dropping below 30ft AGL halves all payload points. Dropping below
15ft loses them entirely. Drops do not need to be successful.
"""

from ..common_types import *
from ..control.drone_control import DroneControl
from ..control.mission_info import MissonTracker
from .utils import log, warn
import time


def drop():
    pass


def fm2(controller: DroneControl, mt: MissonTracker, drop_target: GPSCoord, cruise_alt: float):
    ### takeoff from waypoint L
    log("fm2: taking off from L")
    controller.takeoff(cruise_alt)

    ### transit from fire zone to drop
    log("fm2: transiting to drop zone")
    controller.goto_waypoint(GPSCoord(drop_target.lat, drop_target.long, cruise_alt))
    drop()
    log("fm2: payload dropped")
