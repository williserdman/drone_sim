from ..common_types import *

import time
import math
import collections

if not hasattr(collections, "MutableMapping"):
    import collections.abc

    collections.MutableMapping = collections.abc.MutableMapping  # type: ignore

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal  # type: ignore
from pymavlink import mavutil  # type: ignore

# === CONFIG ===
GROUND_SPEED = 3.0
ALT_TOL = 0.8
POS_TOL = 1.0
TIMEOUT_MOVE = 120

FEET_TO_METERS = 0.3048
ALT_30FT = 30 * FEET_TO_METERS  # 9.144 m
ALT_40FT = 40 * FEET_TO_METERS  # 12.192 m


def horiz_distance_m(a: GPSCoord, b: GPSCoord):
    """Approx horizontal distance (meters) between two lat/lon pairs."""
    lat1, lon1 = math.radians(a.lat), math.radians(a.long)
    lat2, lon2 = math.radians(b.lat), math.radians(b.long)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    # Equirectangular approximation (good for small distances)
    x = dlon * math.cos((lat1 + lat2) / 2.0)
    y = dlat
    return 6378137.0 * math.sqrt(x * x + y * y)


def wait_alt(vehicle, target_alt_m, tol=ALT_TOL, timeout=60):
    """Wait until relative altitude is within tol of target_alt_m."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        alt = vehicle.location.global_relative_frame.alt
        if alt is not None and abs(alt - target_alt_m) <= tol:
            return True
        time.sleep(0.2)
    return False


def wait_pos(
    vehicle,
    target_lat,
    target_lon,
    pos_tol=POS_TOL,
    alt_m=None,
    alt_tol=ALT_TOL,
    timeout=TIMEOUT_MOVE,
):
    """Wait until horizontally within pos_tol (and optionally vertically within alt_tol)."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        loc = vehicle.location.global_relative_frame
        if loc is not None:
            current_pos = GPSCoord(loc.lat, loc.lon, loc.alt if loc.alt else 0)
            target_pos = GPSCoord(target_lat, target_lon, 0)
            d = horiz_distance_m(current_pos, target_pos)
            alt_ok = True
            print(
                "Horizontal Distance ",
                d,
                "pos_tol ",
                pos_tol,
                "Altitude ",
                loc.alt,
                "alt_tol ",
                alt_tol,
            )
            if alt_m is not None and loc.alt is not None:
                alt_ok = abs(loc.alt - alt_m) <= alt_tol
            if d <= pos_tol and alt_ok:
                return True
        time.sleep(0.3)
    return False


def arm_and_takeoff(vehicle, target_alt_m):
    # Basic pre-arm wait
    print("[*] Waiting for vehicle to initialize & become armable…")
    while not vehicle.is_armable:
        print(
            "    is_armable:",
            vehicle.is_armable,
            " GPS fix:",
            getattr(vehicle.gps_0, "fix_type", None),
        )
        time.sleep(1)

    print("[*] Setting Guided Mode via Mavlink")
    vehicle._master.mav.set_mode_send(
        vehicle._master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4,
    )
    time.sleep(1)

    print("[*] Arming…")
    vehicle.armed = True
    t0 = time.time()
    while not vehicle.armed and time.time() - t0 < 15:
        print("    waiting for armed…")
        time.sleep(0.5)
    if not vehicle.armed:
        raise RuntimeError(
            "Failed to arm (check prearm checks, safety switch, EKF, GPS)."
        )

    print(f"[*] Taking off to {target_alt_m:.2f} m AGL…")
    vehicle.simple_takeoff(target_alt_m)
    if not wait_alt(vehicle, target_alt_m, tol=max(ALT_TOL, 0.9), timeout=45):
        print("[!] Takeoff altitude tolerance not reached in time; continuing anyway.")


def goto(vehicle, lat, lon, alt_m):
    """Command a GUIDED move to lat/lon/alt (AGL)."""
    target = LocationGlobalRelative(lat, lon, alt_m)
    vehicle.groundspeed = GROUND_SPEED
    vehicle.simple_goto(target)


class DroneControl:
    def __init__(self, connection_port="/dev/cu.usbmodem1103"):
        print(f"Connecting to {connection_port} …")
        vehicle = connect(
            connection_port, wait_ready=True, heartbeat_timeout=60, timeout=120
        )
        # vehicle.wait_ready("gps_0", "mode", "system_status", "attitude", "location")
        self.vehicle = vehicle
        self.cruise_alt = 33  # meters
        pass

    def rtl(self):
        self.vehicle.mode = VehicleMode("RTL")

    def goto_waypoint(self, coord: GPSCoord) -> int:
        goto(self.vehicle, coord.lat, coord.long, self.cruise_alt)
        if not wait_pos(
            self.vehicle,
            coord.lat,
            coord.long,
            POS_TOL,
            self.cruise_alt,
            ALT_TOL,
            TIMEOUT_MOVE,
        ):
            return -1
        return 0

    def move_relative_ned(self, dir: NEDMeters) -> int:
        return 0

    def move_relative_self(self, dir: RelPosComplete) -> int:
        """Send a MAVLink LANDING_TARGET message in body FRD frame using
        the provided relative position (meters, vehicle body frame).
        This sets position_valid=1 and populates x/y/z.
        """
        time_usec = int(time.time() * 1e6)
        msg = self.vehicle.message_factory.landing_target_encode(
            time_usec,
            0,  # target_num
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            0.0,  # angle_x
            0.0,  # angle_y
            0.0,  # distance
            0.0,  # size_x
            0.0,  # size_y
            float(dir.x),
            float(dir.y),
            float(dir.z),
            [0.0, 0.0, 0.0, 0.0],  # q
            0,  # type
            1,  # position_valid
        )
        # send the MAVLink message to the vehicle
        self.vehicle.send_mavlink(msg)

        return 0

    def translate_relative(self, dir: RelativePosition) -> NEDMeters:
        return NEDMeters(0, 0, 0)

    def simple_land(self) -> int:
        """Command the vehicle to land (MODE=LAND) and wait until touchdown.
        This does not attempt to disarm the vehicle; if the autopilot
        disarms automatically upon landing, this will be detected and
        reported as a non-zero return value.
        """
        print("[*] Landing…")
        # Set LAND mode using MAVLink (mode 9 for ArduCopter)
        self.vehicle._master.mav.set_mode_send(
            self.vehicle._master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            9,  # LAND mode
        )
        time.sleep(1)

        t0 = time.time()
        timeout = 180
        while time.time() - t0 < timeout:
            loc = self.vehicle.location.global_relative_frame
            alt = None
            if loc is not None:
                alt = loc.alt
            # If we reached near-ground altitude, consider landed
            if alt is not None and alt <= max(ALT_TOL, 0.3):
                print("[*] Touchdown detected (alt <= tol). Leaving motors armed.")
                return 0
            # If vehicle disarmed during landing, signal failure (we did not disarm)
            if not getattr(self.vehicle, "armed", True):
                print("[!] Vehicle disarmed during landing.")
                return -1
            time.sleep(0.5)
        print("[!] Landing timed out.")
        return -1

    def takeoff(self, alt: int) -> int:
        return 0

    def disarm(self) -> int:
        return 0

    def wait_for_arm(self) -> int:
        return 0

    def get_current_gps(self) -> GPSCoord:
        f = self.vehicle.location.global_relative_frame
        return GPSCoord(f.lat, f.long, f.alt)
