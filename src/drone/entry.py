from .common_types import *
from .control.drone_control import DroneControl
from .control.mission_info import MissonTracker
from .missions.fm1 import fm1
from .missions.fm2 import fm2
from .missions.fm3 import fm3
import time
import os
import json
 
CRUISE_ALT = 30  # ft AGL
MISSION_DATA_PATH = "mission_data/waypoints.json"
WAYPOINT_IDS = ["H", "L", "F1", "F2", "WA", "WM"]
 
os.makedirs("mission_data", exist_ok=True)
if not os.path.exists(MISSION_DATA_PATH):
    with open(MISSION_DATA_PATH, "w") as f:
        json.dump({}, f)
 
mt = MissonTracker()
print("mission tracker initialized")
controller = DroneControl(connection_port="/dev/ttyACM0")
print("controller init")
 
### REPL LOOP
while True:
    print("\n1. Update GPS waypoints")
    print("2. Update cruise altitude")
    print("3. Run fm1")
    print("4. Run fm2")
    print("5. Run fm3")
    print("6. Exit")
    choice = input("\nSelect: ").strip()
 
    if choice == "1":
        for wp_id in WAYPOINT_IDS:
            ans = input(f"Update {wp_id}? (y/n): ").strip().lower()
            if ans != "y":
                continue
            lat  = float(input("  Latitude:  "))
            lon  = float(input("  Longitude: "))
            alt  = float(input("  Altitude:  "))
            mt.setWaypoint(wp_id, GPSCoord(lat, lon, alt))
            print(f"  {wp_id} updated")
 
    elif choice == "2":
        CRUISE_ALT = float(input("New cruise altitude (ft AGL): "))
        print(f"Cruise altitude set to {CRUISE_ALT}ft")
 
    elif choice == "3":
        mt.begin_mission()
        try:
            fm1(controller, mt, CRUISE_ALT)
        except Exception as e:
            print(e)
            controller.rtl()
        mt.end_mission()
 
    elif choice == "4":
        t = input("Drop target — F1 or F2 [F2]: ").strip().upper()
        drop_target = "F1" if t == "F1" else "F2"
        mt.begin_mission()
        try:
            fm2(controller, mt, CRUISE_ALT, drop_target)
        except Exception as e:
            print(e)
            controller.rtl()
        mt.end_mission()
 
    elif choice == "5":
        t = input("Drop target — F1 or F2 [F2]: ").strip().upper()
        drop_target = "F1" if t == "F1" else "F2"
        mt.begin_mission()
        try:
            fm3(controller, mt, CRUISE_ALT, drop_target)
        except Exception as e:
            print(e)
            controller.rtl()
        mt.end_mission()
 
    elif choice == "6":
        print("Exit")
        break