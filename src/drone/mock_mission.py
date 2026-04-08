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
import math
from typing import Optional, Tuple

ARUCO_PICKUP = GPSCoord(39.9337075, -75.7802787, 10)
DROP_POINT = GPSCoord(39.9338306, -75.7801814, 10)
ALT_TOL = 0.03
HOVER_ALT_TOL = 1
TARGET_HOVER_HEIGHT = 3
WINDOW = 5
MULT = 0.3


def _read_lidar_or_fallback(
    lidar: Lidar, controller: DroneControl
) -> Tuple[Optional[float], bool]:
    """Return (altitude, has_lidar) with GPS fallback when LiDAR is unavailable."""
    try:
        return lidar.get_distance(), True
    except Exception as lidar_error:
        print(f"[WARN] LiDAR read failed: {lidar_error}. Falling back to GPS altitude.")
        try:
            return controller.get_current_gps().alt, False
        except Exception as gps_error:
            print(f"[ERR] GPS fallback after LiDAR failure also failed: {gps_error}")
            return None, False


def aruco_land_precision(
    controller: DroneControl, camera: Camera, lidar: Lidar, target_id: int
):
    # controller.set_precision_land_mode()
    controller.set_land_mode()

    alt, has_lidar = _read_lidar_or_fallback(lidar, controller)
    if alt is None:
        raise RuntimeError("Unable to determine altitude from LiDAR or GPS.")

    i = 1
    quality = 4
    t0 = time.time()
    timeout = 60.0

    # Loop until ArduPilot explicitly confirms touchdown
    # this for loop will exit after timeout -> 60 seconds
    for i in range(1_000):
        if alt > ALT_TOL and controller.vehicle.armed:
            if time.time() - t0 > timeout:
                raise TimeoutError("Precision-landing timeout waiting for landed state")
            # while alt > ALT_TOL:

            # Get raw 3D update
            if has_lidar:
                raw_update = camera.vec_to_marker_3d(
                    target_id, lidar_alt=alt, quality=quality
                )
            else:
                raw_update = camera.vec_to_marker_3d(target_id, quality=quality)

            if raw_update:
                controller.land_send_landing_target(raw_update)

            # Update LiDAR distance periodically
            if i % 5 == 0:
                if has_lidar:
                    alt, has_lidar = _read_lidar_or_fallback(lidar, controller)
                    if alt is None:
                        raise RuntimeError(
                            "Lost both LiDAR and GPS altitude sources during landing."
                        )

            # Add a tiny sleep to prevent maxing out the CPU loop
            time.sleep(0.05)
        else:
            break

    print("[*] Lidar confirms touchdown!")
    # time.sleep(3)
    # controller.set_guided_mode()
    return


def pickup_sequence(
    controller: DroneControl, camera: Camera, lidar: Lidar, target_id: int
):
    controller.set_guided_mode()
    timeout = 60
    t0 = time.time()

    # 1. Drop down to search altitude
    alt, _ = _read_lidar_or_fallback(lidar, controller)
    if alt is None:
        print("[ERR] Cannot start pickup sequence without altitude data.")
        return False
    how_much_down = alt - TARGET_HOVER_HEIGHT
    print(f"moving down {how_much_down}m")

    # You can still use a relative move just for the Z-axis drop,
    # but make sure to wait for it to finish!
    # controller.guide_move_relative_frame(RelPosComplete(0, 0, how_much_down))
    # time.sleep(5)
    controller.climb(alt - how_much_down)

    # attempt for one minute
    # print("entering height wait")
    # for _ in range (600):
    # if abs(lidar.get_distance() - TARGET_HOVER_HEIGHT) > HOVER_ALT_TOL:
    # break
    # time.sleep(0.1)

    hover_alt, _ = _read_lidar_or_fallback(lidar, controller)
    if hover_alt is not None:
        print(f"[*] Hover alt difference: {abs(hover_alt - TARGET_HOVER_HEIGHT)}")
    else:
        print(
            "[WARN] Hover altitude difference unavailable due to sensor read failures."
        )

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
        val = controller.goto_waypoint(target_wp, position_tol=0.8)
        print(f"return of goto func: {val}")
        # Wait a moment for the drone to stabilize its tilt/roll after stopping
        # controller.wait_until_stable()
        # controller.hold_waypoint_until_stable(target_wp)
        quality = 4

        # Search for target at this position
        for _ in range(5):
            update = camera.vec_to_marker_3d(target_id, quality=quality)
            if update:
                print("[*] Target Acquired! Switching to LAND mode.")
                controller.vehicle.flush()

                # Convert vision-relative correction into an absolute GPS target,
                # similar to the grid-search GPS waypoint approach.
                current_gps = controller.get_current_gps()
                yaw = controller.vehicle.attitude.yaw
                dNorth = update.x * math.cos(yaw) - update.y * math.sin(yaw)  # type: ignore
                dEast = update.x * math.sin(yaw) + update.y * math.cos(yaw)  # type: ignore
                corrected_wp = controller.get_location_metres(
                    current_gps, dNorth, dEast
                )
                val = controller.goto_waypoint(corrected_wp)
                print(f"return of goto func: {val}")

                time.sleep(1)  # Let it center before triggering land
                target_found = True
                break
            time.sleep(0.1)

    # Trigger landing sequence outside the loop
    if target_found:
        aruco_land_precision(controller, camera, lidar, target_id)
        return True
    else:
        print("[!] Grid search exhausted, target not found.")
        return False


def fm3(
    mt: MissonTracker,
    controller: DroneControl,
    camera: Camera,
    lidar: Lidar,
    dropper: Dropper,
    possible_ids: set,
    pickup_point: GPSCoord,
    target_point: GPSCoord,
):
    IDs = list(possible_ids)
    desired_drop_height_m = 10
    try:
        for id in IDs:

            if mt.time_left() < 60:
                return

            print("going to pickup waypoint")
            val = controller.goto_waypoint(pickup_point)
            print(f"return of goto func: {val}")

            print("init pickup sequence")
            success = pickup_sequence(controller, camera, lidar, id)

            controller.set_guided_mode()
            if success:
                print("climb")
                if controller.vehicle.armed and controller.is_landed():
                    print("vehicle armed")
                    controller.simple_takeoff(10)
                elif controller.vehicle.armed:
                    controller.set_guided_mode()
                    controller.climb(10)
                else:
                    time.sleep(8)
                    controller.force_arm_takeoff(10)

                print("going to drop point")
                val = controller.goto_waypoint(target_point)
                print(f"return of goto func: {val}")

                camera.save_frame_buffer_async()

                print("dropping")
                drop_target = GPSCoord(
                    target_point.lat, target_point.long, target_point.alt
                )
                lidar_alt, _ = _read_lidar_or_fallback(lidar, controller)
                if lidar_alt is not None and lidar_alt < desired_drop_height_m:
                    drop_target.alt += desired_drop_height_m - lidar_alt
                    controller.goto_waypoint(drop_target)
                controller.hold_waypoint_until_stable(drop_target)
                dropper.drop()
            else:
                print(f"Skipping drop for ID {id} because pickup failed.")
                controller.climb(10)

    except Exception as e:
        print("[ERR]", e)
        controller.rtl()
        camera.save_frame_buffer_async()
    except KeyboardInterrupt as e:
        print("[ERR]", e)
        controller.rtl()
        camera.save_frame_buffer_async()


if __name__ == "__main__":
    mt = MissonTracker(600)
    mt.begin_mission()
    controller = DroneControl("/dev/ttyACM0")
    camera = Camera(50)
    try:
        lidar = Lidar()
    except Exception as e:
        print(f"[ERR] LiDAR initialization failed: {e}")
        raise SystemExit(1)
    dropper = Dropper()

    # controller.takeoff(10)
    original_gps = controller.get_current_gps()
    original_gps.alt = 10
    controller.force_arm_takeoff(10)

    fm3(mt, controller, camera, lidar, dropper, {6, 7}, ARUCO_PICKUP, DROP_POINT)

    val = controller.goto_waypoint(original_gps)

    print(f"return of goto func: {val}")
    controller.simple_land()
    controller.disarm()
