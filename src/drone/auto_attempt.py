"""Automatic single-attempt sequencing around the original mission functions."""

import math
from types import SimpleNamespace
from typing import Callable, Mapping

from . import timebase
from .common_types import GPSCoord, MissionHome


MISSION_DEADLINE_SECONDS = 600.0
MISSION_ALTITUDE_METERS = 10.0
PHYSICAL_EVIDENCE_INTERVAL_SECONDS = 0.05


def _finite_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _pinned_mission_home(controller) -> MissionHome:
    home = getattr(controller, "mission_home", None)
    if (
        not isinstance(home, MissionHome)
        or not all(_finite_number(value) for value in (home.lat, home.lon, home.amsl_m))
        or not -90.0 <= home.lat <= 90.0
        or not -180.0 <= home.lon <= 180.0
    ):
        raise RuntimeError("automatic attempt requires a valid pinned mission home")
    return home


def _exact_zero(result: object) -> bool:
    return type(result) is int and result == 0


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
    home = _pinned_mission_home(controller)
    check_permission = getattr(controller, "check_permission", None)
    if not callable(check_permission):
        raise RuntimeError("automatic attempt requires a callable permission check")

    def require_permission() -> None:
        if check_permission() is not None:
            raise RuntimeError("automatic attempt permission check must return None")

    require_permission()
    mission_start_sim_time = timebase.monotonic()
    tracker.begin_mission()
    functions = mission_functions or _original_mission_functions()

    def require_deadline() -> None:
        if timebase.monotonic() - mission_start_sim_time > MISSION_DEADLINE_SECONDS:
            raise TimeoutError("automatic mission exceeded 600 simulated seconds")

    def require_boundary() -> None:
        require_deadline()
        require_permission()
        require_deadline()

    def run_phase(
        name: str,
        action,
        *,
        await_physical_evidence: bool = False,
    ) -> None:
        require_boundary()
        emit(name, "STARTED")
        with timebase._deadline(
            mission_start_sim_time, MISSION_DEADLINE_SECONDS
        ):
            result = action()
        if result is not True:
            raise RuntimeError(f"{name} failed")
        require_boundary()
        if await_physical_evidence:
            timebase.sleep(PHYSICAL_EVIDENCE_INTERVAL_SECONDS)
            require_boundary()
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
        await_physical_evidence=True,
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
        await_physical_evidence=True,
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
        await_physical_evidence=True,
    )

    require_boundary()
    emit("HOME", "STARTED")
    with timebase._deadline(mission_start_sim_time, MISSION_DEADLINE_SECONDS):
        require_boundary()
        if not _exact_zero(
            controller.goto_waypoint(
                GPSCoord(home.lat, home.lon, MISSION_ALTITUDE_METERS)
            )
        ):
            raise RuntimeError("Home navigation failed")
        require_boundary()
        if not _exact_zero(controller.simple_land()):
            raise RuntimeError("Home landing was not confirmed")
        require_boundary()
        if not _exact_zero(controller.disarm()):
            raise RuntimeError("Home disarm was not confirmed")
        require_boundary()
        emit("HOME", "DISARMED")
        require_boundary()
        timebase.sleep(PHYSICAL_EVIDENCE_INTERVAL_SECONDS)
        require_boundary()
        emit("HOME", "COMPLETE")
