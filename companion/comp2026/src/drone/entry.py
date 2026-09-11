"""Quarantined legacy mission REPL.

The production QGC entry point is ``drone.control.listener``. This module is
kept only as historical development context because its mission interfaces no
longer match the active flight stack.
"""


CRUISE_ALT = 30  # ft AGL, retained from the legacy tool
WAYPOINT_IDS = ["H", "L", "F1", "F2", "WA", "WM"]


def _legacy_repl():
    """Run the old REPL implementation after an in-process safety review."""
    from .common_types import GPSCoord
    from .control.drone_control import DroneControl
    from .control.mission_info import MissonTracker
    from .missions.fm1 import fm1
    from .missions.fm2 import fm2
    from .missions.fm3 import fm3

    cruise_alt = CRUISE_ALT
    mt = MissonTracker()
    print("mission tracker initialized")
    controller = DroneControl(connection_port="/dev/ttyACM0")
    print("controller init")

    while True:
        print("\n1. Update GPS waypoints")
        print("2. Update cruise altitude")
        print("3. Run fm1")
        print("4. Run fm2")
        print("5. Run fm3")
        print("6. Exit")
        choice = input("\nSelect: ").strip()

        if choice == "1":
            for waypoint_id in WAYPOINT_IDS:
                answer = input(f"Update {waypoint_id}? (y/n): ").strip().lower()
                if answer != "y":
                    continue
                lat = float(input("  Latitude:  "))
                lon = float(input("  Longitude: "))
                alt = float(input("  Altitude:  "))
                mt.setWaypoint(waypoint_id, GPSCoord(lat, lon, alt))
                print(f"  {waypoint_id} updated")

        elif choice == "2":
            cruise_alt = float(input("New cruise altitude (ft AGL): "))
            print(f"Cruise altitude set to {cruise_alt}ft")

        elif choice in {"3", "4", "5"}:
            drop_target = None
            if choice in {"4", "5"}:
                target = input("Drop target - F1 or F2 [F2]: ").strip().upper()
                drop_target = "F1" if target == "F1" else "F2"
            mt.begin_mission()
            try:
                if choice == "3":
                    fm1(controller, mt, cruise_alt)
                elif choice == "4":
                    fm2(controller, mt, cruise_alt, drop_target)
                else:
                    fm3(controller, mt, cruise_alt, drop_target)
            except Exception as exc:
                print(exc)
                controller.rtl()
            mt.end_mission()

        elif choice == "6":
            print("Exit")
            break


def main():
    raise RuntimeError(
        "Legacy mission REPL is disabled: its interfaces do not match the "
        "active safety stack. Use drone.control.listener for the QGC path."
    )


if __name__ == "__main__":
    main()
