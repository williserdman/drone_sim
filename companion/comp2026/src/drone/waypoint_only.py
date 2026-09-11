"""Legacy direct-hardware waypoint experiment.

Importing this module is inert. The experiment is not part of the production
QGC command path and its command-line entry point remains disabled.
"""

import time

from .common_types import GPSCoord


H = GPSCoord(41.5013453, -81.6064230, 10)
A = GPSCoord(41.5013812, -81.606423, 10)
ALT_TOL = 0.1


def run_demo(controller, dropper, mission_tracker):
    """Run the preserved experiment with dependencies supplied by the caller."""
    mission_tracker.begin_mission()
    mission_tracker.begin_aux_timer()
    try:
        controller.force_arm_takeoff(10)
        controller.goto_waypoint(H)
        dropper.drop()
        controller.goto_waypoint(A)
        controller.rtl()
    except Exception as exc:
        print(exc)
        controller.rtl()
        time.sleep(5)


def main():
    raise RuntimeError(
        "Legacy waypoint demo is disabled until it is integrated with the "
        "active mission safety supervisor."
    )


if __name__ == "__main__":
    main()
