"""
run mission to pickup and drop arucos at specified waypoints
- mission begins
- for id in range: land and pickup payload
- fly to drop waypoint, wait 2 seconds (to settle), drop
- repeat for all ids in list
"""

from .common_types import *
from .control.mission_info import MissonTracker
from .control.drone_control import DroneControl
import time
from .sensors.camera.camera import Camera
from .sensors.lidar.lidar import Lidar
from .sensors.servo.servo import Dropper
from .utils.position_smoother import RelPosSmoother

ARUCO_PICKUP = GPSCoord(41.5013866, -81.6064099, 10)
DROP_POINT = GPSCoord(41.5016162, -81.6061652, 10)
ALT_TOL = 0.01
WINDOW = 5
MULT = 0.3


def aruco_land_precision(
    controller: DroneControl, camera: Camera, lidar: Lidar, target_id: int
):
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


def pickup_sequence(
    controller: DroneControl, camera: Camera, lidar: Lidar, target_id: int
):
    controller.set_guided_mode()
    # TODO: ensure we are 3-4 meters using lidar above target before initializing PL sequence
    alt = lidar.get_distance()
    how_much_down = alt - 3
    print(f"moving down {how_much_down}m")
    controller.guide_move_relative_frame(RelPosComplete(0, 0, how_much_down))
    # original_gps = controller.get_current_gps()

    time.sleep(4)

    print("[*] Searching for ArUco to initiate Precision Landing...")
    target_found = False

    # 1. Hover and search in 3x3 grid pattern
    grid_size = 1.5  # 1.5m steps for 3m x 3m grid coverage
    grid_positions = [
        (0, 0),  # center
        (grid_size, 0),  # right
        (grid_size, grid_size),  # right-up
        (0, grid_size),  # up
        (-grid_size, grid_size),  # left-up
        (-grid_size, 0),  # left
        (-grid_size, -grid_size),  # left-down
        (0, -grid_size),  # down
        (grid_size, -grid_size),  # right-down
    ]

    for x, y in grid_positions:
        if target_found:
            break

        # Move to grid position
        alt = lidar.get_distance()
        z_adjust = alt - 3
        controller.guide_move_relative_frame(RelPosComplete(x, y, z_adjust))
        time.sleep(3)  # Stabilize at position

        # Search for target at this position
        for _ in range(5):  # Check multiple times at each position
            update = camera.vec_to_marker_3d(target_id)
            if update:
                print("[*] Target Acquired! Switching to LAND mode.")
                controller.vehicle.flush()
                controller.guide_move_relative_frame(
                    RelPosComplete(update.x, update.y, 0)
                )
                target_found = True
                break
            time.sleep(0.1)

    aruco_land_precision(controller, camera, lidar, target_id)


IDs = [4, 5, 6]

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
    mt.begin_aux_timer()
    # controller.takeoff(10)
    original_gps = controller.get_current_gps()
    controller.force_arm_takeoff(10)

    for id in IDs:
        print("going to pickup waypoint")
        controller.goto_waypoint(ARUCO_PICKUP)
        print("init pickup sequence")
        pickup_sequence(controller, camera, lidar, id)
        print("climb")
        # controller.climb(10)
        controller.force_arm_takeoff(10)
        print("going to drop point")
        controller.goto_waypoint(DROP_POINT)
        print("dropping")
        time.sleep(2)
        dropper.drop()

    controller.goto_waypoint(original_gps)
    controller.simple_land()
    controller.disarm()
    mt.end_mission()

except Exception as e:
    print("[ERR]", e)
    controller.rtl()
