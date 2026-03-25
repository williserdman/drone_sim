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


def aruco_land_precision(
    controller: DroneControl, camera: Camera, lidar: Lidar, target_id: int
):
    controller.set_guided_mode()
    controller.guide_move_relative_frame(RelPosComplete(0, 0, 6.5))
    original_gps = controller.get_current_gps()

    print("[*] Searching for ArUco to initiate Precision Landing...")
    target_found = False

    # 1. Hover in place and search until we get the first visual hit
    while not target_found:
        # UPDATED: Using 3D Pose Estimation
        update = camera.vec_to_marker_3d(target_id)

        if update:
            print("[*] Target Acquired! Switching to LAND mode.")
            target_found = True
        else:
            # descend??? -> Note: You could add a slow step-down here
            # (e.g., guide_move_relative_frame(0, 0, 0.5)) if you are too high to see it!
            time.sleep(0.1)  # Brief sleep to avoid maxing out CPU while searching

    controller.set_land_mode()
    alt = lidar.get_distance()
    i = 1

    while alt > ALT_TOL:
        # UPDATED: Using 3D Pose Estimation
        update = camera.vec_to_marker_3d(target_id)

        if update:
            # update.x is Forward, update.y is Right.
            # We continue to use LiDAR 'alt' for Z since it is more accurate than camera depth.
            controller.land_send_landing_target(RelPosComplete(update.x, update.y, alt))
            # print(f"Sent precision landing update: Fwd: {update.x:.2f}, Right: {update.y:.2f}")
        else:
            # maybe switch PLND_ settings to pause descent if we lose aruco
            # Tip: Set PLND_STRICT=1 or 2 in ArduPilot to enforce pausing if marker is lost
            pass

        # Update LiDAR distance every 20 loops
        if i % 20 == 0:
            alt = lidar.get_distance()
            i = 0
        i += 1

    print("[*] Touchdown complete.")

    controller.set_guided_mode()
    controller.force_arm_takeoff(original_gps.alt)

    return


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

    aruco_land_precision(controller, camera, lidar, ID)
    time.sleep(1)

    # controller.takeoff(10)
    controller.goto_waypoint(A)
    dropper.drop()
    controller.goto_waypoint(H)
    controller.set_land_mode()
    controller.disarm()
    mt.end_mission()
except Exception as e:
    print("[ERR]", e)
    controller.rtl()
