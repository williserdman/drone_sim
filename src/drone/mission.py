from .common_types import *
from .control.mission_info import MissonTracker
from .control.drone_control import DroneControl
import time
from .sensors.camera.camera import Camera
from .sensors.lidar.lidar import Lidar
from .utils.position_smoother import RelPosSmoother

PICKUP_WA = GPSCoord(101010, 101001, 100)
DROP_POINT = GPSCoord(10101, 101010, 100)
LAND_POINT = GPSCoord(101010, 101001, 100)
HOME_POINT = GPSCoord(101010, 101001, 100)
ALT_TOL = 0.4


def drop():
    pass


def aruco_land(
    controller: DroneControl,
    camera: Camera,
    lidar: Lidar,
    target_id: int,
    current_pos: GPSCoord,
):
    smoother = RelPosSmoother()
    alt = lidar.get_distance()
    while alt > ALT_TOL:
        update = camera.vec_to_marker(target_id)
        if update:
            smoother.append(update)
        rp = smoother.get_ema()
        if isinstance(rp, RelativePosition):
            controller.move_relative_self(RelPosComplete(rp.x, rp.y, alt - 0.5))
            time.sleep(0.1)
        alt = lidar.get_distance()

    controller.goto_waypoint(current_pos)
    return


def do_bomb_run(controller: DroneControl, camera: Camera, lidar: Lidar, target_id: int):
    aruco_land(
        controller, camera, lidar, target_id, controller.get_current_gps()
    )  # and we return to altitude of 30 meters

    controller.goto_waypoint(DROP_POINT)

    drop()

    controller.goto_waypoint(PICKUP_WA)


ids = [1, 2, 3, 4, 5, 6]

mt = MissonTracker()
print("mission tracker initialized")
controller = DroneControl(connection_port="/dev/ttyACM1")
print("controller init")
camera = Camera(100)
print("camera init")
lidar = Lidar()
print("lidar init")

mt.begin_mission()

### LAND AT WAYPOINT L
mt.begin_aux_timer()
controller.goto_waypoint(LAND_POINT)
controller.simple_land()
time.sleep(1)
return_to_home_time = mt.end_aux_timer() + 5  # 5 second buffer
controller.takeoff(30)
### END WAYPOINT L PORTION

### SINGLE BOMB FOR TIMING
controller.goto_waypoint(PICKUP_WA)
mt.begin_aux_timer()
do_bomb_run(controller, camera, lidar, ids.pop(0))
one_pass_time = mt.end_aux_timer()
min_time_for_pass = one_pass_time + return_to_home_time
### END TIMING RUN

### DROP AS MANY AS POSSIBLE
while mt.time_left() > min_time_for_pass and len(ids) > 0:
    do_bomb_run(controller, camera, lidar, ids.pop(0))

### END
controller.goto_waypoint(HOME_POINT)
controller.simple_land()
controller.disarm()
mt.end_mission()
