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

H = GPSCoord(41.5013453, -81.6064230, 10)
A = GPSCoord(41.5013812, -81.606423, 10)
ALT_TOL = 0.01
WINDOW = 5
MULT = 0.3


def aruco_land_precision(
    controller: DroneControl, camera: Camera, lidar: Lidar, target_id: int
):
    controller.set_guided_mode()
    # TODO: ensure we are 3-4 meters using lidar above target before initializing PL sequence
    alt = lidar.get_distance()
    how_much_down = alt - 3
    controller.guide_move_relative_frame(RelPosComplete(0, 0, how_much_down))
    # original_gps = controller.get_current_gps()

    print("[*] Searching for ArUco to initiate Precision Landing...")
    target_found = False

    # 1. Hover and search
    while not target_found:
        update = camera.vec_to_marker_3d(target_id)
        if update:
            print("[*] Target Acquired! Switching to LAND mode.")
            controller.guide_move_relative_frame(RelPosComplete(update.x, update.y, 0))
            target_found = True
        else:
            time.sleep(0.1)

    # controller.set_precision_land_mode()
    controller.set_land_mode()
    alt = lidar.get_distance()
    i = 1

    # Loop until ArduPilot explicitly confirms touchdown
    while not controller.is_on_ground:

        # Get raw 3D update
        raw_update = camera.vec_to_marker_3d(target_id, lidar_alt=alt)

        if raw_update:
            controller.land_send_landing_target(raw_update)

        # Update LiDAR distance periodically
        if i % 5 == 0:
            alt = lidar.get_distance()
            i = 0
        i += 1

        # Add a tiny sleep to prevent maxing out the CPU loop
        time.sleep(0.05)

    print("[*] ArduPilot EKF confirms touchdown!")
    controller.set_guided_mode()
    return


""" 
def aruco_land_guide(
    controller: DroneControl,
    camera: Camera,
    lidar: Lidar,
    target_id: int,
    current_pos: GPSCoord,
):

    alt = lidar.get_distance()
    controller.set_guided_mode()
    controller.guide_move_relative_frame(RelPosComplete(0, 0, 5))
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
            controller.guide_move_relative_frame(RelPosComplete(rp.x, rp.y, 0.2))
            print("sending move command")
            time.sleep(2)
        else:
            print("no aruco_slow descent")
            controller.guide_move_relative_frame(RelPosComplete(0, 0, 0.25))
        alt = lidar.get_distance()

    controller.goto_waypoint(current_pos)
    return
 """

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
    original_gps = controller.get_current_gps()
    controller.force_arm_takeoff(10)
    controller.goto_waypoint(H)
    time.sleep(1)
    ### END WAYPOINT L PORTION

    aruco_land_precision(controller, camera, lidar, ID)
    # aruco_land_guide(controller, camera, lidar, ID, controller.get_current_gps())
    time.sleep(2)

    controller.set_guided_mode()
    controller.force_arm_takeoff(10)
    # controller.climb(10)

    controller.goto_waypoint(A)
    dropper.drop()
    controller.goto_waypoint(original_gps)
    controller.simple_land()
    controller.disarm()
    mt.end_mission()

except Exception as e:
    print("[ERR]", e)
    controller.rtl()
