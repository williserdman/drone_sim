"""
Picks up payloads from WA (yellow) or WM (blue). Drops at a fire zone (F1 or F2). Can then 
loop unlimited times picking up blue payloads from WM and dropping at a fire zone. Must land at 
Home before time runs out.
"""

from ..common_types import *
from ..control.drone_control import DroneControl
from ..control.mission_info import MissonTracker
import time
 
HOME_BUFFER = 60  
 
def drop():
    pass  #trigger dropper/servo here
 
 
def pickup():
    pass  # trigger pickup mechanism here
 
 
def fm3(controller: DroneControl, mt: MissonTracker, cruise_alt: float, drop_target: str = "F2"):
    H      = mt.getWaypoint("H")
    WA     = mt.getWaypoint("WA")
    WM     = mt.getWaypoint("WM")
    target = mt.getWaypoint(drop_target)
 
    ### First pickup point from WA
    controller.goto_waypoint(GPSCoord(WA.lat, WA.long, cruise_alt))
    controller.simple_land()
    pickup()
    controller.takeoff(cruise_alt)
 
    ### First drop
    controller.goto_waypoint(GPSCoord(target.lat, target.long, cruise_alt))
    drop()
 
    ### Pickup from WM and drop until time runs low
    while mt.time_left() > HOME_BUFFER:
        controller.goto_waypoint(GPSCoord(WM.lat, WM.long, cruise_alt))
        controller.simple_land()
        pickup()
        controller.takeoff(cruise_alt)
 
        controller.goto_waypoint(GPSCoord(target.lat, target.long, cruise_alt))
        drop()
 
    ### return home and land
    controller.goto_waypoint(GPSCoord(H.lat, H.long, cruise_alt))
    controller.simple_land()