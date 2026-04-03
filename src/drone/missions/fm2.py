"""
Begins at waypoint L. Takes off to 30ft AGL, transits to a fire zone, and drops payloads. Drops 
must occur at or above 15ft AGL. Dropping below 30ft AGL halves all payload points. Dropping below
15ft loses them entirely. Drops do not need to be successful.
"""

from ..common_types import *
from ..control.drone_control import DroneControl
from ..control.mission_info import MissonTracker
from ..sensors.servo.servo import Dropper
from .utils import log, warn
import time


def fm2(mt: MissonTracker, controller: DroneControl, cruise_alt: int, drop_target: GPSCoord, dropper: Dropper):
    mt.begin_mission()
    mt.begin_aux_timer()

    target_gps = GPSCoord(drop_target.lat, drop_target.long, cruise_alt)

    ### takeoff from waypoint L
    log("fm2: taking off from L")
    controller.force_arm_takeoff(cruise_alt)

    ### transit from fire zone to drop
    log("fm2: transiting to drop zone")
    controller.goto_waypoint(target_gps)
    controller.hold_waypoint_until_stable(target_gps)
    dropper.drop()

    # TODO: Figure out what to do at the end of fm2. Land and proceed with FM3?

    mt.end_mission()

    log("fm2: payload dropped")
