"""
begins at point H, ascends to 30 ft and proceeds to waypoint A and begins to descend using CV guidance
"""

from .common_types import *
from .control.mission_info import MissonTracker
from .control.drone_control import DroneControl
import time
from .sensors.camera.camera import Camera
from .sensors.lidar.lidar import Lidar
from .sensors.servo.servo import Dropper
from .utils.position_smoother import RelPosSmoother

H = GPSCoord(41.501318, -81.606382, 10)
A = GPSCoord(41.5013812, -81.606423, 10)
ALT_TOL = 0.1
WINDOW = 5


def drop(dropper):
    dropper.drop()


def aruco_land(
    controller: DroneControl,
    camera: Camera,
    lidar: Lidar,
    target_id: int,
    current_pos: GPSCoord,
):

    alt = lidar.get_distance()
    controller.set_guided_mode()
    controller.move_relative_self(RelPosComplete(0, 0, 5))
    print("sent move downward command")
    # time.sleep(4)
    time.sleep(1)
    while alt > ALT_TOL:
        smoother = RelPosSmoother()
        for _ in range(2 * WINDOW):
            update = camera.vec_to_marker(target_id)
            if update:
                print("seen marker")
                smoother.append(update)
        rp = smoother.get_ema()
        if isinstance(rp, RelativePosition):
            controller.move_relative_self(RelPosComplete(rp.x, rp.y, 0.2))
            print("sending move command")
            time.sleep(2)
        else:
            print("no aruco_slow descent")
            controller.move_relative_self(RelPosComplete(0, 0, 0.25))
        alt = lidar.get_distance()

    controller.goto_waypoint(current_pos)
    return


ID = 1

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

try:
    mt.begin_mission()

    ### LAND AT WAYPOINT L
    mt.begin_aux_timer()
    # controller.takeoff(10)
    controller.force_arm_takeoff(10)
    controller.goto_waypoint(H)
    time.sleep(1)
    ### END WAYPOINT L PORTION

    aruco_land(controller, camera, lidar, ID, controller.get_current_gps())
    time.sleep(5)

    # controller.takeoff(10)
    controller.goto_waypoint(A)
    controller.simple_land()
    controller.disarm()
    mt.end_mission()
except Exception as e:
    print("[ERR]", e)
    controller.rtl()
