"""Legacy direct-hardware ArUco landing experiment.

Importing this module is inert. The experiment is not part of the production
QGC command path and its command-line entry point remains disabled.
"""

import time
from typing import Any

from .common_types import GPSCoord, RelPosComplete


H = GPSCoord(41.5013453, -81.6064230, 10)
A = GPSCoord(41.5013812, -81.606423, 10)
ALT_TOL = 0.01
WINDOW = 5
MULT = 0.3
ID = 1


def aruco_land_precision(
    controller: Any, camera: Any, lidar: Any, target_id: int
):
    controller.set_guided_mode()
    alt = lidar.get_distance()
    how_much_down = alt - 3
    controller.guide_move_relative_frame(RelPosComplete(0, 0, how_much_down))

    print("[*] Searching for ArUco to initiate Precision Landing...")
    target_found = False
    while not target_found:
        update = camera.vec_to_marker_3d(target_id)
        if update:
            print("[*] Target Acquired! Switching to LAND mode.")
            controller.guide_move_relative_frame(RelPosComplete(update.x, update.y, 0))
            target_found = True
        else:
            time.sleep(0.1)

    controller.set_land_mode()
    alt = lidar.get_distance()
    iteration = 1

    while not controller.is_on_ground:
        raw_update = camera.vec_to_marker_3d(target_id, lidar_alt=alt)
        if raw_update:
            controller.land_send_landing_target(raw_update)

        if iteration % 5 == 0:
            alt = lidar.get_distance()
            iteration = 0
        iteration += 1
        time.sleep(0.05)

    print("[*] ArduPilot EKF confirms touchdown!")
    controller.set_guided_mode()


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


def run_demo(controller, camera, lidar, dropper, mission_tracker):
    """Run the preserved experiment with dependencies supplied by the caller."""
    try:
        mission_tracker.begin_mission()
        mission_tracker.begin_aux_timer()
        original_gps = controller.get_current_gps()
        controller.force_arm_takeoff(10)
        controller.goto_waypoint(H)
        time.sleep(1)

        aruco_land_precision(controller, camera, lidar, ID)
        time.sleep(2)

        controller.set_guided_mode()
        controller.force_arm_takeoff(10)
        controller.goto_waypoint(A)
        dropper.drop()
        controller.goto_waypoint(original_gps)
        controller.simple_land()
        controller.disarm()
        mission_tracker.end_mission()
    except Exception as exc:
        print("[ERR]", exc)
        controller.rtl()


def main():
    raise RuntimeError(
        "Legacy ArUco landing demo is disabled until it is integrated with "
        "the active mission safety supervisor."
    )


if __name__ == "__main__":
    main()
