"""
begins at point H, ascends to 10m and proceeds to waypoint A, proceeds to waypoint B, proceeds to waypoint C: descends
"""

from common_types import *
from control.mission_info import MissonTracker
from control.drone_control import DroneControl
import time
from sensors.camera.camera import Camera
from sensors.lidar.lidar import Lidar
from utils.position_smoother import RelPosSmoother


H = GPSCoord(101010, 101001, 100)
A = GPSCoord(10101, 101010, 100)
B = GPSCoord(101010, 101001, 100)
C = GPSCoord(101010, 101001, 100)
ALT_TOL = 0.4


mt = MissonTracker()
print("mission tracker initialized")
controller = DroneControl(connection_port="/dev/ttyACM1")
print("controller init")
camera = Camera(100)
print("camera init")
lidar = Lidar()
print("lidar init")

mt.begin_mission()

### LAND AT WAYPOINT L
mt.begin_aux_timer()
controller.goto_waypoint(H)
controller.goto_waypoint(A)
controller.goto_waypoint(B)
controller.simple_land()
controller.takeoff(10)
controller.rtl()
