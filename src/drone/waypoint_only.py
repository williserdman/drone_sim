"""
begins at point H, ascends to 10m and proceeds to waypoint A, proceeds to waypoint B, proceeds to waypoint C: descends
"""

from .common_types import *
from .control.mission_info import MissonTracker
from .control.drone_control import DroneControl
import time
from .sensors.camera.camera import Camera
from .sensors.servo.servo import Dropper

# from .sensors.lidar.lidar import Lidar
# from .sensors.servo.servo import Dropper
from .utils.position_smoother import RelPosSmoother


H = GPSCoord(41.501318, -81.606382, 10)
A = GPSCoord(41.5013812, -81.606423, 10)
B = GPSCoord(41.501366, -81.606237, 10)
ALT_TOL = 0.4


mt = MissonTracker()
print("mission tracker initialized")
controller = DroneControl(connection_port="/dev/ttyACM0")
print("controller init")
camera = Camera(100)
print("camera init")
# lidar = Lidar()
# print("lidar init")
dropper = Dropper()
print("dropper init")

mt.begin_mission()

### LAND AT WAYPOINT L
mt.begin_aux_timer()

try:
    controller.force_arm_takeoff(10)
    # controller.takeoff(10)
    controller.goto_waypoint(H)
    dropper.drop()
    controller.goto_waypoint(A)
    # controller.simple_land()
    # controller.takeoff(10)
    controller.rtl()
except Exception as e:
    print(e)
    controller.rtl()
    time.sleep(5)
