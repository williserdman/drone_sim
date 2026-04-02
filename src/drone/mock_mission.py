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

ARUCO_PICKUP = GPSCoord(41.5013920, -81.6064366, 10)
DROP_POINT = GPSCoord(41.5016162, -81.6061652, 10)
ALT_TOL = 0.00
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
    while alt > ALT_TOL:  # not controller.is_landed:
        # while alt > ALT_TOL:

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
    # time.sleep(3)
    # controller.set_guided_mode()
    return


def pickup_sequence(
    controller: DroneControl, camera: Camera, lidar: Lidar, target_id: int
):
    controller.set_guided_mode()

    # 1. Drop down to search altitude
    alt = lidar.get_distance()
    how_much_down = alt - 2
    print(f"moving down {how_much_down}m")

    # You can still use a relative move just for the Z-axis drop,
    # but make sure to wait for it to finish!
    controller.guide_move_relative_frame(RelPosComplete(0, 0, how_much_down))
    time.sleep(5)

    print("[*] Searching for ArUco to initiate Precision Landing...")
    target_found = False

    # 2. CAPTURE THE ANCHOR POINT
    # We grab the absolute GPS location right now. This is the center of our grid.
    center_anchor = controller.get_current_gps()

    # 3. Define the grid as absolute North/East offsets in meters
    grid_size = 1.5
    grid_offsets_ne = [
        (0, 0),  # center
        (0, grid_size),  # right (East)
        (grid_size, grid_size),  # right-up (North-East)
        (grid_size, 0),  # up (North)
        (grid_size, -grid_size),  # left-up (North-West)
        (0, -grid_size),  # left (West)
        (-grid_size, -grid_size),  # left-down (South-West)
        (-grid_size, 0),  # down (South)
        (-grid_size, grid_size),  # right-down (South-East)
    ]

    for dNorth, dEast in grid_offsets_ne:
        if target_found:
            break

        # Calculate the exact GPS coordinate for this grid point
        target_wp = controller.get_location_metres(center_anchor, dNorth, dEast)

        # Use your robust spin-wait goto!
        # The drone will fight the wind until it reaches this exact earth coordinate.
        controller.goto_waypoint(target_wp)

        # Wait a moment for the drone to stabilize its tilt/roll after stopping
        # controller.wait_until_stable()
        # controller.hold_waypoint_until_stable(target_wp)

        # Search for target at this position
        for _ in range(5):
            update = camera.vec_to_marker_3d(target_id)
            if update:
                print("[*] Target Acquired! Switching to LAND mode.")
                controller.vehicle.flush()

                # At this point, the camera has visual, so we can trust the
                # visual relative update to center over the marker.
                controller.guide_move_relative_frame(
                    RelPosComplete(update.x, update.y, 0)
                )
                time.sleep(1)  # Let it center before triggering land
                target_found = True
                break
            time.sleep(0.1)

    # Trigger landing sequence outside the loop
    if target_found:
        aruco_land_precision(controller, camera, lidar, target_id)
    else:
        print("[!] Grid search exhausted, target not found.")


IDs = [5, 7, 8]

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
    original_gps.alt = 10
    controller.force_arm_takeoff(10)

    for id in IDs:
        print("going to pickup waypoint")
        controller.goto_waypoint(ARUCO_PICKUP)
        print("init pickup sequence")
        pickup_sequence(controller, camera, lidar, id)

        time.sleep(3)
        print("climb")
        if controller.vehicle.armed:
            print("vehicle armed")
            controller.set_guided_mode()
            controller.simple_takeoff(10)
        else:
            time.sleep(3)
            controller.force_arm_takeoff(10)

        print("going to drop point")
        controller.goto_waypoint(DROP_POINT)

        camera.save_frame_buffer_async()

        print("dropping")
        controller.hold_waypoint_until_stable(DROP_POINT)
        dropper.drop()

    controller.goto_waypoint(original_gps)
    controller.simple_land()
    controller.disarm()
    mt.end_mission()

except Exception as e:
    print("[ERR]", e)
    controller.rtl()
