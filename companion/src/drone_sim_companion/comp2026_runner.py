"""Testbench sequence around unmodified upstream mission functions.

Phase COMPLETE records that the function returned. The scorer independently
checks physical flight and payload evidence; upstream FM3 has no payload output.
"""

from drone import timebase
from drone.common_types import GPSCoord


def mission_functions():
    from drone.missions.fm1 import fm1
    from drone.missions.fm2 import fm2
    from drone.mock_mission import fm3

    return fm1, fm2, fm3


def run_auto_attempt(*, tracker, controller, camera, lidar, payloads,
                     waypoints, emit, check_active, confirm_landed):
    fm1, fm2, fm3 = mission_functions()
    started = timebase.monotonic()
    tracker.begin_mission()

    def boundary():
        check_active()
        if timebase.monotonic() - started > 600:
            raise TimeoutError("automatic mission exceeded 600 simulated seconds")

    def phase(name, action):
        boundary()
        emit(name, "STARTED")
        action()
        boundary()
        emit(name, "COMPLETE")

    phase("FM1", lambda: fm1(tracker, controller, 10, waypoints["L"]))
    phase("FM2", lambda: fm2(tracker, controller, 10, waypoints["F2"], payloads[2], lidar))
    # Upstream FM2 lands. Follow mock_mission's executable entrypoint, which
    # takes off before calling its first FM3 function.
    boundary()
    controller.force_arm_takeoff(10)
    phase("FM3_3", lambda: fm3(tracker, controller, camera, lidar, {3}, waypoints["WA"], waypoints["F2"]))
    phase("FM3_4", lambda: fm3(tracker, controller, camera, lidar, {4}, waypoints["WM"], waypoints["F2"]))
    boundary()
    emit("HOME", "STARTED")
    home = waypoints["H"]
    controller.goto_waypoint(GPSCoord(home.lat, home.long, 10))
    boundary()
    controller.simple_land()
    boundary()
    controller.disarm()
    boundary()
    if not confirm_landed():
        raise RuntimeError("upstream attempt returned without fresh landed and disarmed telemetry")
    emit("HOME", "DISARMED")
    emit("HOME", "COMPLETE")
