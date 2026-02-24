"""
begins at point H, ascends to 10m and proceeds to waypoint A, proceeds to waypoint B, proceeds to waypoint C: descends
"""

from .common_types import *
from .control.mission_info import MissonTracker
from .control.drone_control import DroneControl
import time
from .sensors.camera.camera import Camera
from .sensors.lidar.lidar import Lidar
from .sensors.servo.servo import Dropper
from .utils.position_smoother import RelPosSmoother


H = GPSCoord(101010, 101001, 100)
A = GPSCoord(10101, 101010, 100)
B = GPSCoord(101010, 101001, 100)
C = GPSCoord(101010, 101001, 100)
ALT_TOL = 0.4


mt = MissonTracker()
print("mission tracker initialized")
controller = DroneControl(connection_port="/dev/ttyACM0")
print("controller init")
camera = Camera(100)
print("camera init")
lidar = Lidar()
print("lidar init")
dropper = Dropper()
print("dropper init")

mt.begin_mission()

### LAND AT WAYPOINT L
mt.begin_aux_timer()

try:
    # controller.force_arm_takeoff(10)
    controller.takeoff(10)
    controller.goto_waypoint(H)
    dropper.drop()
    controller.goto_waypoint(A)
    controller.goto_waypoint(B)
    controller.simple_land()
    controller.takeoff(10)
    controller.rtl()
except Exception as e:
    print(e)
    controller.rtl()
    time.sleep(5)
