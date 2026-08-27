"""Automatic single-attempt sequencing around the original mission functions."""

from types import SimpleNamespace
from typing import Callable, Mapping

from . import timebase
from .common_types import GPSCoord


MISSION_DEADLINE_SECONDS = 600.0
MISSION_ALTITUDE_METERS = 10.0


def _original_mission_functions():
    from .missions.fm1 import fm1
    from .missions.fm2 import fm2
    from .mock_mission import fm3

    return SimpleNamespace(fm1=fm1, fm2=fm2, fm3=fm3)


def run_auto_attempt(
    *,
    tracker,
    controller,
    camera,
    lidar,
    payloads: Mapping[int, object],
    waypoints: Mapping[str, GPSCoord],
    emit: Callable[[str, str], None],
    mission_functions=None,
) -> None:
    """Run one original FM1/FM2/FM3-3/FM3-4/Home attempt in order."""
    mission_start_sim_time = timebase.time()
    tracker.begin_mission()
    functions = mission_functions or _original_mission_functions()

    def require_deadline() -> None:
        if timebase.time() - mission_start_sim_time > MISSION_DEADLINE_SECONDS:
            raise TimeoutError("automatic mission exceeded 600 simulated seconds")

    def run_phase(name: str, action) -> None:
        require_deadline()
        emit(name, "STARTED")
        with timebase._deadline(
            mission_start_sim_time, MISSION_DEADLINE_SECONDS
        ):
            result = action()
        if result is False:
            raise RuntimeError(f"{name} failed")
        require_deadline()
        emit(name, "COMPLETE")

    run_phase(
        "FM1",
        lambda: functions.fm1(
            tracker, controller, MISSION_ALTITUDE_METERS, waypoints["L"]
        ),
    )
    run_phase(
        "FM2",
        lambda: functions.fm2(
            tracker,
            controller,
            MISSION_ALTITUDE_METERS,
            waypoints["F2"],
            payloads[2],
            lidar,
            desired_drop_height_m=int(MISSION_ALTITUDE_METERS),
        ),
    )
    run_phase(
        "FM3_3",
        lambda: functions.fm3(
            tracker,
            controller,
            camera,
            lidar,
            payloads[3],
            {3},
            waypoints["WA"],
            waypoints["F2"],
        ),
    )
    run_phase(
        "FM3_4",
        lambda: functions.fm3(
            tracker,
            controller,
            camera,
            lidar,
            payloads[4],
            {4},
            waypoints["WM"],
            waypoints["F2"],
        ),
    )

    require_deadline()
    emit("HOME", "STARTED")
    with timebase._deadline(mission_start_sim_time, MISSION_DEADLINE_SECONDS):
        home = waypoints["H"]
        controller.goto_waypoint(
            GPSCoord(home.lat, home.long, MISSION_ALTITUDE_METERS)
        )
        controller.simple_land()
        if controller.disarm() != 0:
            raise RuntimeError("Home disarm was not confirmed")
        require_deadline()
        emit("HOME", "DISARMED")
        timebase.sleep(0.05)
        require_deadline()
        emit("HOME", "COMPLETE")
