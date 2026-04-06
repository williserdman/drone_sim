import time
from pymavlink import mavutil
from ..common_types import *
from .drone_control import DroneControl
from .mission_info import MissonTracker
from ..sensors.camera.camera import Camera
from ..sensors.lidar.lidar import Lidar
from ..sensors.servo.servo import Dropper
from ..missions.fm1 import fm1
from ..missions.fm2 import fm2
from ..mock_mission import fm3
from ..missions.utils import log, warn

CONNECTION_STRING = "/dev/ttyACM0"
BAUD_RATE = 115200
CRUISE_ALT = 10

CMD_FM1 = 31000 # mavutil.mavlink.MAV_CMD_USER_1  # 31000
CMD_FM2 = 31001 # mavutil.mavlink.MAV_CMD_USER_2  # 31001
CMD_FM3 = 31002 # mavutil.mavlink.MAV_CMD_USER_3  # 31002

L = GPSCoord(39.9337075, -75.7802787, CRUISE_ALT)  # Need to change hardcode for comp
# F1 = GPSCoord(41.5015115, -81.6063900, CRUISE_ALT)
# F2 = GPSCoord(41.5016162, -81.6061652, CRUISE_ALT)
# WM = GPSCoord(41.5013420, -81.6063845, CRUISE_ALT)
# WA = GPSCoord(41.5013240, -81.6062797, CRUISE_ALT)
# TARGET = GPSCoord(41.5016162, -81.6061652, CRUISE_ALT)
TARGET = GPSCoord(39.9338306, -75.7801814, CRUISE_ALT)


WA_IDS = {6}
WM_IDS = {7, 8}

ARUCO_SIZE = 75
COMPANION_COMPONENT_ID = 191
GCS_SYSTEM_ID = 200


import queue


def start_repl():
    mt = MissonTracker()
    controller = DroneControl(connection_port=CONNECTION_STRING)
    camera = Camera(ARUCO_SIZE)
    lidar = Lidar()
    dropper = Dropper()

    original_gps = controller.get_current_gps()
    original_gps.alt = CRUISE_ALT

    print("\n[+] System initialized. Waiting for commands from GCS...")

    while True:
        # 1. Send heartbeat for the companion computer
        controller.vehicle._master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )

        try:
            # 2. Check the queue for new commands (blocks for 1 second)
            cmd = controller.command_queue.get(timeout=1.0)

            print(cmd)

            # 3. Route the command
            mt.begin_mission()
            warn("time started: 10:00 minutes", controller.vehicle._master)

            if cmd == CMD_FM1:
                print(">> SUCCESS: Triggering FM1")
                fm1(mt, controller, CRUISE_ALT, L)
                warn("fm1 finished, awaiting command", controller.vehicle._master)

            elif cmd == CMD_FM2:
                print(">> SUCCESS: Triggering FM2")
                fm2(mt, controller, CRUISE_ALT, TARGET, dropper)
                warn("fm2 finished, awaiting command", controller.vehicle._master)

            elif cmd == CMD_FM3:
                print(">> SUCCESS: Triggering FM3")
                fm3(mt, controller, camera, lidar, dropper, WA_IDS, WA, TARGET)
                warn("proceeding to WM targets", controller.vehicle._master)
                fm3(mt, controller, camera, lidar, dropper, WM_IDS, WM, TARGET)
                warn("fm3 finished, returning home", controller.vehicle._master)

                controller.goto_waypoint(original_gps)
                controller.simple_land()
                controller.disarm()

            else:
                print(f">> UNKNOWN: Unmapped COMMAND ID: {cmd}")
                warn("invalid command", controller.vehicle._master)

            # Mark the queue task as done
            controller.command_queue.task_done()

        except queue.Empty:
            # No command received in the last second, just loop and send heartbeat again
            continue


if __name__ == "__main__":
    try:
        start_repl()
    except KeyboardInterrupt:
        print("\n[*] Exiting Listener REPL.")
