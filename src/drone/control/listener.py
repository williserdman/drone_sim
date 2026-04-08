import time
from typing import Optional
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

CMD_FM1 = 31000  # mavutil.mavlink.MAV_CMD_USER_1  # 31000
CMD_FM2 = 31001  # mavutil.mavlink.MAV_CMD_USER_2  # 31001
CMD_FM3 = 31002  # mavutil.mavlink.MAV_CMD_USER_3  # 31002
CMD_DO_NOTHING = 0

CMD_UPDATE_WA = 31003
CMD_UPDATE_WM1 = 31004
CMD_UPDATE_WM2 = 31005
CMD_UPDATE_WM3 = 31006
CMD_UPDATE_WM4 = 31007
CMD_UPDATE_WM5 = 31008
CMD_UPDATE_WM6 = 31009
CMD_UPDATE_L = 31010
CMD_UPDATE_TARGET = 31011

CMD_CLEAR_ALL_PICKUP_WPS = 31013
CMD_CLEAR_ALL_WAYPOINTS = 31014

FOUR_HOURS_SECONDS = 4 * 60 * 60
WAYPOINT_KEYS = {"WA", "WM1", "WM2", "WM3", "WM4", "WM5", "WM6", "L"}
PICKUP_WAYPOINT_KEYS = {"WA", "WM1", "WM2", "WM3", "WM4", "WM5", "WM6"}
ALL_CLEARABLE_WAYPOINT_KEYS = WAYPOINT_KEYS | {"TARGET"}

FM3_WM_ORDER = ["WM1", "WM2", "WM3", "WM4", "WM5", "WM6"]
FM3_MARKER_IDS = {
    "WA": {3},
    "WM1": {4},
    "WM2": {5},
    "WM3": {6},
    "WM4": {7},
    "WM5": {8},
    "WM6": {9},
}

ARUCO_SIZE = 100
COMPANION_COMPONENT_ID = 191
GCS_SYSTEM_ID = 200


import queue


def start_repl():
    mt = MissonTracker()
    controller = DroneControl(connection_port=CONNECTION_STRING)
    camera = Camera(ARUCO_SIZE)
    lidar = Lidar()
    dropper = Dropper()

    waypoint_update_times = {}
    last_waypoint_status_warn = 0.0

    def warn_waypoint_status(force: bool = False):
        nonlocal last_waypoint_status_warn

        now = time.time()
        if not force and (now - last_waypoint_status_warn) < 60:
            return

        recent_waypoints = sorted(
            name
            for name, ts in waypoint_update_times.items()
            if (now - ts) <= FOUR_HOURS_SECONDS
        )
        stale_or_missing_waypoints = sorted(
            name
            for name in WAYPOINT_KEYS
            if name not in waypoint_update_times
            or (now - waypoint_update_times[name]) > FOUR_HOURS_SECONDS
        )

        if recent_waypoints:
            warn(
                f"waypoints updated within 4h: {', '.join(recent_waypoints)}",
                controller.vehicle._master,
            )
        else:
            warn("no waypoints updated within 4h", controller.vehicle._master)

        if stale_or_missing_waypoints:
            warn(
                f"waypoints missing/stale (>4h): {', '.join(stale_or_missing_waypoints)}",
                controller.vehicle._master,
            )

        last_waypoint_status_warn = now

    def update_waypoint(name: str):
        gps = controller.get_current_gps()
        gps.alt = CRUISE_ALT
        mt.set_waypoint(name, gps)
        waypoint_update_times[name] = time.time()
        warn(f"{name} auto updated", controller.vehicle._master)
        warn_waypoint_status(force=True)

    def clear_waypoint(name: str):
        mt.clear_waypoint(name)
        if name in waypoint_update_times:
            del waypoint_update_times[name]
        warn(f"{name} cleared", controller.vehicle._master)
        warn_waypoint_status(force=True)

    def load_all_waypoints():
        loaded = []
        missing = []
        now = time.time()

        for name in sorted(ALL_CLEARABLE_WAYPOINT_KEYS):
            waypoint = mt.get_waypoint(name)
            if waypoint is None:
                missing.append(name)
            else:
                waypoint_update_times[name] = now
                loaded.append(name)

        if loaded:
            warn(
                f"loaded waypoints: {', '.join(loaded)}",
                controller.vehicle._master,
            )

        if missing:
            warn(
                f"unable to load waypoints: {', '.join(missing)}",
                controller.vehicle._master,
            )

        warn_waypoint_status(force=True)

    """ original_gps = controller.get_current_gps()
    original_gps.alt = CRUISE_ALT """

    load_all_waypoints()

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

        original_gps = None

        try:
            # 2. Check the queue for new commands (blocks for 1 second)
            cmd = controller.command_queue.get(timeout=1.0)

            print(cmd)

            if cmd == CMD_FM1:
                mt.begin_mission()
                original_gps = controller.get_current_gps()
                warn("time started: 10:00 minutes", controller.vehicle._master)
                print(">> SUCCESS: Triggering FM1")
                l_waypoint = mt.get_waypoint("L")
                if l_waypoint is None:
                    warn("missing waypoint: L", controller.vehicle._master)
                    controller.command_queue.task_done()
                    continue
                fm1(mt, controller, CRUISE_ALT, l_waypoint)
                warn("fm1 finished, awaiting command", controller.vehicle._master)

            elif cmd == CMD_FM2:
                print(">> SUCCESS: Triggering FM2")
                target_waypoint = mt.get_waypoint("TARGET")
                if target_waypoint is None:
                    warn("missing waypoint: TARGET", controller.vehicle._master)
                    controller.command_queue.task_done()
                    continue
                fm2(mt, controller, CRUISE_ALT, target_waypoint, dropper, lidar)
                warn("fm2 finished, awaiting command", controller.vehicle._master)

            elif cmd == CMD_FM3:
                print(">> SUCCESS: Triggering FM3")
                wa_waypoint = mt.get_waypoint("WA")
                target_waypoint = mt.get_waypoint("TARGET")
                if wa_waypoint is None:
                    warn("missing waypoint: WA", controller.vehicle._master)
                    controller.command_queue.task_done()
                    continue
                if target_waypoint is None:
                    warn("missing waypoint: TARGET", controller.vehicle._master)
                    controller.command_queue.task_done()
                    continue

                fm3(
                    mt,
                    controller,
                    camera,
                    lidar,
                    dropper,
                    FM3_MARKER_IDS["WA"],
                    wa_waypoint,
                    target_waypoint,
                )
                warn("proceeding to WM targets", controller.vehicle._master)

                for wm_name in FM3_WM_ORDER:
                    wm_waypoint = mt.get_waypoint(wm_name)
                    if wm_waypoint is None:
                        warn(f"missing waypoint: {wm_name}", controller.vehicle._master)
                        continue
                    fm3(
                        mt,
                        controller,
                        camera,
                        lidar,
                        dropper,
                        FM3_MARKER_IDS.get(wm_name, set()),
                        wm_waypoint,
                        target_waypoint,
                    )

                warn("fm3 finished, returning home", controller.vehicle._master)

                if original_gps is not None:
                    controller.goto_waypoint(original_gps)
                else:
                    l_waypoint = mt.get_waypoint("L")
                    if l_waypoint is None:
                        warn("missing waypoint: L", controller.vehicle._master)
                    else:
                        controller.goto_waypoint(l_waypoint)
                controller.simple_land()
                controller.disarm()

            elif cmd == CMD_DO_NOTHING:
                print(">> NO-OP: DO NOTHING command received")
                warn("do nothing command received", controller.vehicle._master)

            elif cmd == CMD_UPDATE_WA:
                update_waypoint("WA")

            elif cmd == CMD_UPDATE_WM1:
                update_waypoint("WM1")

            elif cmd == CMD_UPDATE_WM2:
                update_waypoint("WM2")

            elif cmd == CMD_UPDATE_WM3:
                update_waypoint("WM3")

            elif cmd == CMD_UPDATE_WM4:
                update_waypoint("WM4")

            elif cmd == CMD_UPDATE_WM5:
                update_waypoint("WM5")

            elif cmd == CMD_UPDATE_WM6:
                update_waypoint("WM6")

            elif cmd == CMD_UPDATE_L:
                update_waypoint("L")

            elif cmd == CMD_UPDATE_TARGET:
                update_waypoint("TARGET")

            elif cmd == CMD_CLEAR_ALL_PICKUP_WPS:
                for wp_name in sorted(PICKUP_WAYPOINT_KEYS):
                    clear_waypoint(wp_name)

            elif cmd == CMD_CLEAR_ALL_WAYPOINTS:
                for wp_name in sorted(ALL_CLEARABLE_WAYPOINT_KEYS):
                    clear_waypoint(wp_name)

            else:
                print(f">> UNKNOWN: Unmapped COMMAND ID: {cmd}")
                warn("invalid command", controller.vehicle._master)

            # Mark the queue task as done
            controller.command_queue.task_done()

        except queue.Empty:
            # No command received in the last second, just loop and send heartbeat again
            warn_waypoint_status()
            continue


if __name__ == "__main__":
    try:
        start_repl()
    except KeyboardInterrupt:
        print("\n[*] Exiting Listener REPL.")
