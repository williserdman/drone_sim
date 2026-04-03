import time
from pymavlink import mavutil
from ..common_types import *
from drone_control import DroneControl
from mission_info import MissonTracker
from ..sensors.camera.camera import Camera
from ..sensors.lidar.lidar import Lidar
from ..sensors.servo.servo import Dropper
from ..missions.fm1 import fm1
from ..missions.fm2 import fm2

# from ..missions.fm3 import fm3
from ..mock_mission import fm3

CONNECTION_STRING = "tcp:localhost:5763"  # "/dev/ttyAMA0"
BAUD_RATE = 921600
CRUISE_ALT = 10

CMD_FM1 = mavutil.mavlink.MAV_CMD_USER_1  # 31000
CMD_FM2 = mavutil.mavlink.MAV_CMD_USER_2  # 31001
CMD_FM3 = mavutil.mavlink.MAV_CMD_USER_3  # 31002
L = GPSCoord(41.5016162, -81.6061652, CRUISE_ALT)  # Need to change hardcode for comp
F1 = GPSCoord(41.5016162, -81.6061652, CRUISE_ALT)
F2 = GPSCoord(41.5016162, -81.6061652, CRUISE_ALT)


def start_repl():
    print(f"[*] Starting RPi Command Listener on {CONNECTION_STRING}...")

    # connect as companion computer, default source for companion computer is 191
    master = mavutil.mavlink_connection(
        CONNECTION_STRING, baud=BAUD_RATE, source_system=1, source_component=191
    )

    print("[*] Waiting for heartbeat from Pixhawk...")
    master.wait_heartbeat()
    print("[+] Heartbeat received! Ready to receive commands.\n")

    mt = MissonTracker()
    print("mission tracker initialized")
    controller = DroneControl(connection_port="/dev/ttyACM0")
    print("controller init")
    camera = Camera(50)
    print("camera init")
    lidar = Lidar()
    print("lidar init")
    dropper = Dropper()
    print("dropper init")

    while True:
        # i think we have to send at least one heartbeat so px4 knows where the component is
        # should we keep sending it? im not sure if anything beyond the first one is in use
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )

        msg = master.recv_match(
            type=["COMMAND_LONG", "COMMAND_INT"], blocking=True, timeout=1.0
        )

        if not msg:
            continue

        # i think type command_int, not sure which one actually worked
        if msg.target_component == 191:
            if msg.command == 31000:
                print(">> SUCCESS: Received command to trigger FM1")
                fm1(mt, controller, 10, L)
                # fm1(controller, mt, camera, lidar)
            elif msg.command == 31001:
                print(">> SUCCESS: Received command to trigger FM2")
                fm2(mt, controller, 10, F1, dropper)
            elif msg.command == 31002:
                print(">> SUCCESS: Received command to trigger FM3")
                fm3(mt, controller, camera, lidar, dropper)
            else:
                print(f">> UNKNOWN: Received unmapped COMMAND ID: {msg.command}")

            # ack? not sure if this is needed
            master.mav.command_ack_send(
                msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED
            )


if __name__ == "__main__":
    try:
        start_repl()
    except KeyboardInterrupt:
        print("\n[*] Exiting Listener REPL.")
