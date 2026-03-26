"""
Mission:
Begins at Home, takes off to 30ft AGL, transits horizontally to waypoint L, and lands.
Must be carrying at least one payload to proceed to fm2.
Wait flagger flag and judge approval at H before starting fm2.
"""

from ..common_types import *
from ..control.drone_control import DroneControl
from ..control.mission_info import MissonTracker
import time
 
 
def fm1(controller: DroneControl, mt: MissonTracker, cruise_alt: float):
    L = mt.getWaypoint("L")
 
    ### Takeoff from home
    controller.takeoff(cruise_alt)
 
    ### Transit to waypoint L and land
    controller.goto_waypoint(GPSCoord(L.lat, L.long, cruise_alt))
    controller.simple_land()
