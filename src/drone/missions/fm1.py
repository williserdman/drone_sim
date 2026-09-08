"""
Mission:
Begins at Home, takes off to 30ft AGL, transits horizontally to waypoint L, and lands.
Must be carrying at least one payload to proceed to fm2.
Wait for flagger at L to confirm aircraft is inside the box, and judge at H to approve the vertical landing quality, before starting fm2.
"""

from ..common_types import *
from ..control.drone_control import DroneControl
from ..control.mission_info import MissonTracker
from .utils import log, warn


def _require_takeoff_success(result: object) -> None:
    if result is None or (type(result) is int and result == 0):
        return
    raise RuntimeError("FM1 takeoff was not confirmed")


def _require_legacy_success(result: object, operation: str) -> None:
    if type(result) is int and result == 0:
        return
    raise RuntimeError(f"FM1 {operation} was not confirmed")


def fm1(
    mt: MissonTracker, controller: DroneControl, cruise_alt: int, L: GPSCoord
) -> bool:
    controller.check_permission()
    log(
        f"fm1: taking off to configured altitude {cruise_alt} m",
        controller.vehicle._master,
    )
    _require_takeoff_success(controller.force_arm_takeoff(cruise_alt))

    controller.check_permission()
    log("fm1: transiting to waypoint L", controller.vehicle._master)
    _require_legacy_success(
        controller.goto_waypoint(GPSCoord(L.lat, L.long, cruise_alt)),
        "navigation",
    )

    controller.check_permission()
    _require_legacy_success(controller.simple_land(), "landing")

    controller.check_permission()
    _require_legacy_success(controller.disarm(), "disarm")

    log(
        "fm1: landed at L, awaiting flagger and judge approval to start fm2",
        controller.vehicle._master,
    )
    return True
