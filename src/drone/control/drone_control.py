from ..common_types import *
import os
from typing import Optional
import queue

# os.environ["MAVLINK20"] = "1"

from .. import timebase as time
import math
import collections

if not hasattr(collections, "MutableMapping"):
    import collections.abc

    collections.MutableMapping = collections.abc.MutableMapping  # type: ignore

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal  # type: ignore
from pymavlink import mavutil  # type: ignore

# === CONFIG ===
GROUND_SPEED = 20.0
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


# good has timeout
def wait_alt(vehicle, target_alt_m, tol=ALT_TOL, timeout=60):
    """Wait until relative altitude is within tol of target_alt_m."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        alt = vehicle.location.global_relative_frame.alt
        if alt is not None and abs(alt - target_alt_m) <= tol:
            return True
        time.sleep(0.2)
    return False


# good has timeout
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


# I hope the drone is on the ground when this is called...
def arm_and_takeoff(vehicle, target_alt_m):
    # Basic pre-arm wait
    print("[*] Waiting for vehicle to initialize & become armable…")
    """ while not vehicle.is_armable:
        print(
            "    is_armable:",
            vehicle.is_armable,
            " GPS fix:",
            getattr(vehicle.gps_0, "fix_type", None),
        )
        time.sleep(1) """

    # print("Disabling pre-arm checks...")
    # vehicle.parameters["ARMING_CHECK"] = 0

    # You cannot arm in GUIDED or AUTO without a GPS fix.
    print("Switching to STABILIZE mode...")
    vehicle.mode = VehicleMode("STABILIZE")

    # Wait for the mode to change
    while not vehicle.mode.name == "STABILIZE":
        print(" Waiting for mode change...")
        time.sleep(1)

    print("Arming motors...")
    vehicle.armed = True

    # Wait until the vehicle is actually armed
    while not vehicle.armed:
        print(" Waiting for arming to complete...")
        time.sleep(1)

    print("Vehicle is ARMED!")

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
            connection_port,
            wait_ready=True,
            heartbeat_timeout=60,
            timeout=120,
            source_system=1,
            source_component=191,
        )
        self.vehicle = vehicle
        self.cruise_alt = 10
        self.boot_time = time.monotonic()
        self.is_on_ground = False

        # Create a thread-safe queue to pass commands to the main thread
        self.command_queue = queue.Queue()

        controller_ref = self

        @self.vehicle.on_message("EXTENDED_SYS_STATE")
        def listener_sys_state(vehicle, name, message):
            controller_ref.is_on_ground = getattr(message, "landed_state", None) == 1

        @self.vehicle.on_message(["COMMAND_LONG", "COMMAND_INT"])
        def listener_commands(vehicle, name, message):
            # 191 is our companion computer component ID
            if message.target_component == 191:
                print(f"[*] Received MAVLink Command: {message.command}")

                # Instantly ACK the command so the GCS knows we got it
                self.vehicle._master.mav.command_ack_send(
                    message.command, mavutil.mavlink.MAV_RESULT_ACCEPTED
                )

                # Pass the command ID to the main thread via the queue
                self.command_queue.put(message.command)

    def is_landed(self) -> bool:
        return self.is_on_ground

    def force_arm_takeoff(self, alt):
        arm_and_takeoff(self.vehicle, alt)

    def rtl(self):
        self.vehicle.mode = VehicleMode("RTL")

    def goto_waypoint(
        self, coord: GPSCoord, position_tol: Optional[float] = None
    ) -> int:
        position_tol = POS_TOL if position_tol is None else position_tol

        target_alt = coord.alt if coord.alt is not None else self.cruise_alt

        # 2. Pass the dynamic target_alt instead of the hardcoded cruise_alt
        goto(self.vehicle, coord.lat, coord.long, target_alt)

        # 3. Ensure wait_pos also checks against the correct altitude
        if not wait_pos(
            self.vehicle,
            coord.lat,
            coord.long,
            position_tol,
            target_alt,
            ALT_TOL,
            TIMEOUT_MOVE,
        ):
            return -1
        return 0

    def move_relative_ned(self, dir: NEDMeters) -> int:
        return 0

    def set_land_mode(self):
        print("Shifting to LAND mode...")
        self.vehicle.mode = VehicleMode("LAND")

    def set_precision_land_mode(self):
        """
        Commands the drone to land at its current location and enforces
        Precision Landing mode.
        """
        print("[*] Encoding MAV_CMD_NAV_LAND with Precision Landing (Mode 2)...")

        msg = self.vehicle.message_factory.command_long_encode(
            0,
            0,  # target_system, target_component (0 routes to the active vehicle)
            mavutil.mavlink.MAV_CMD_NAV_LAND,  # command ID
            0,  # confirmation
            0,  # param 1: Abort Alt (0 = undefined/use default)
            2,  # param 2: Precision Land Mode (2 = Required, 1 = Opportunistic, 0 = Disabled)
            0,  # param 3: Empty
            0,  # param 4: Yaw Angle (0 = current system yaw)
            0,  # param 5: Latitude (0 = current)
            0,  # param 6: Longitude (0 = current)
            0,  # param 7: Altitude (0 = ground level)
        )

        self.vehicle.send_mavlink(msg)

        return

    def set_guided_mode(self):
        print("Shifting to GUIDED mode...")
        for i in range(10):
            # Compare strings to avoid object instance issues
            if self.vehicle.mode != VehicleMode("GUIDED"):
                self.vehicle.mode = VehicleMode("GUIDED")
                time.sleep(1)  # Give it more time to ACK
            else:
                print("[*] Confirmed GUIDED mode.")
                return 0
        print("[!] Failed to set GUIDED mode after 10 attempts.")
        return -1

    def land_send_landing_target(self, dir: RelPosComplete) -> int:
        """
        Sends the MAVLink 1 LANDING_TARGET message to ArduPilot.
        Forces the legacy 8-parameter format (relies on angles + distance).
        Requires ArduPilot to be in LAND mode.
        """
        # 1. Map user coordinates to ArduPilot's BODY_FRD (Forward, Right, Down) frame
        x_forward = float(dir.y)
        y_right = -float(dir.x)
        z_down = float(dir.z)

        # 2. Calculate absolute Euclidean distance
        target_distance = math.sqrt(x_forward**2 + y_right**2 + z_down**2)

        # 3. Calculate angular offsets (radians)
        # CRITICAL: MAVLink 1 does not send X/Y/Z directly. It relies entirely
        # on these angles and the distance to reconstruct the target position.
        angle_x = math.atan2(x_forward, z_down) if z_down > 0 else 0.0
        angle_y = math.atan2(y_right, z_down) if z_down > 0 else 0.0

        # 4. Use 0 so ArduPilot stamps the message with its internal time upon receipt
        current_time_us = time.monotonic() - self.boot_time

        msg = self.vehicle.message_factory.landing_target_encode(
            0,  # current_time_us,  # time_usec (0 = use autopilot system time)
            0,  # target_num (0 = default target)
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # coordinate frame
            angle_x,  # X-axis angular offset (radians)
            angle_y,  # Y-axis angular offset (radians)
            target_distance,  # Scalar distance to target
            0.0,  # size_x (ignored)
            0.0,  # size_y (ignored)
        )

        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()  # Force the buffer to clear immediately

        return 0

    def guide_move_relative_frame(self, dir: RelPosComplete) -> int:
        """Send a MAVLink SET_POSITION_TARGET_LOCAL_NED message in body FRD frame
        using the provided relative position (meters, vehicle body frame).
        """
        # Bitmask: 0b0000111111111000 (0x0DF8)
        # This tells the flight controller to ONLY use the X, Y, Z positions
        # and to ignore velocities, accelerations, and yaw commands.
        type_mask = 0b0000111111111000

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            # int(time.time() * 1e6),  # time_boot_ms in microseconds # maybe this is utc idk
            0,
            0,
            0,  # target_system, target_component (0 routes to the active vehicle)
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # mavutil.mavlink.MAV_FRAME_BODY_FRD,  # coordinate frame (Forward/Right/Down)
            type_mask,  # type_mask
            float(
                dir.y
            ),  # X: Forward (meters) *** in mavlink docs x is forward, however, in my impl , x is right
            -float(dir.x),  # Y: Right (meters)
            float(dir.z),  # Z: Down (meters) - remember, positive is DOWN!
            0,
            0,
            0,  # vx, vy, vz (ignored)
            0,
            0,
            0,  # afx, afy, afz (ignored)
            0,
            0,  # yaw, yaw_rate (ignored)
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
        self.set_land_mode()
        time.sleep(0.1)

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
            time.sleep(0.1)
        print("[!] Landing timed out.")
        return -1

    def takeoff(self, alt: int) -> None:
        # Wait for the mode to change

        # Wait until the vehicle is actually armed
        while not self.vehicle.armed:
            print(" Waiting for arming to complete...")
            time.sleep(0.1)
        print("Vehicle is ARMED!")

        while not self.vehicle.mode.name == "GUIDED":  # type: ignore
            print(" Waiting for mode change...")
            time.sleep(0.1)

        print(f"[*] Taking off to {alt:.2f} m AGL…")
        self.vehicle.simple_takeoff(alt)
        if not wait_alt(self.vehicle, alt, tol=max(ALT_TOL, 0.9), timeout=45):
            print(
                "[!] Takeoff altitude tolerance not reached in time; continuing anyway."
            )
        return

    def disarm(self) -> int:
        self.vehicle.armed = False
        t0 = time.time()
        while self.vehicle.armed:
            if time.time() - t0 >= 15.0:
                print("[!] Disarm confirmation timed out.")
                return -1
            time.sleep(0.1)
        print("[*] Vehicle is DISARMED.")
        return 0

    def wait_for_arm(self) -> int:
        return 0

    def get_current_gps(self) -> GPSCoord:
        f = self.vehicle.location.global_relative_frame
        return GPSCoord(f.lat, f.lon, f.alt)  # type: ignore

    def simple_takeoff(self, alt: int):
        self.vehicle.simple_takeoff(alt)
        if not wait_alt(self.vehicle, alt, tol=max(ALT_TOL, 0.1), timeout=45):
            print(
                "[!] Takeoff altitude tolerance not reached in time; continuing anyway."
            )

    def climb(self, target_alt: float) -> None:
        """Ascend to a specific altitude mid-flight without using takeoff commands."""
        print(f"[*] Climbing to {target_alt:.2f} m AGL...")

        # Get current location
        loc = self.vehicle.location.global_relative_frame
        if loc.lat is None or loc.lon is None:
            print("[!] No GPS fix available for climb.")
            return

        # Command a vertical move to the new altitude
        target_loc = LocationGlobalRelative(loc.lat, loc.lon, target_alt)
        self.vehicle.simple_goto(target_loc)

        # Wait until we reach the target altitude
        if not wait_alt(self.vehicle, target_alt, tol=max(ALT_TOL, 0.9), timeout=45):
            print(
                "[!] Climb altitude tolerance not reached in time; continuing anyway."
            )

    def get_location_metres(
        self, original_location: GPSCoord, dNorth: float, dEast: float
    ):
        """
        Returns a new Location object offset by dNorth and dEast meters
        from the original location.
        """
        earth_radius = 6378137.0  # Radius of "spherical" earth

        # Coordinate offsets in radians
        dLat = dNorth / earth_radius
        dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

        # New position in decimal degrees
        newlat = original_location.lat + (dLat * 180 / math.pi)
        newlon = original_location.long + (dLon * 180 / math.pi)

        # Assuming your GPSCoord or DroneKit Location object structure
        return GPSCoord(newlat, newlon, original_location.alt)

    def wait_until_stable(
        self, vel_threshold=0.3, stable_duration=1.5, timeout=10.0
    ) -> bool:
        """
            Waits until the drone's pitch magnitude drops below a specific threshold
        for a continuous period of time.

            :param vel_threshold: Maximum acceptable absolute pitch in radians.
        :param stable_duration: How many consecutive seconds it must remain below the threshold.
        :param timeout: Maximum time to wait before giving up.
        """
        print(
            f"[*] Waiting for drone to stabilize (|pitch| < {vel_threshold:.3f} rad)..."
        )
        t_start = time.time()
        stable_start_time = None

        while time.time() - t_start < timeout:
            attitude = self.vehicle.attitude

            if attitude is not None and attitude.pitch is not None:
                pitch = abs(attitude.pitch)

                if pitch < vel_threshold:
                    # It's moving slowly enough. Did we just dip below the threshold?
                    if stable_start_time is None:
                        stable_start_time = time.time()
                    # Has it been stable long enough?
                    elif (time.time() - stable_start_time) >= stable_duration:
                        print(
                            f"[*] Drone stabilized. (Current |pitch|: {pitch:.3f} rad)"
                        )
                        return True
            else:
                # It moved too fast, reset the continuous stability timer
                stable_start_time = None

            time.sleep(0.1)

        print("[!] Stabilization timeout reached; moving on anyway.")
        return False

    def hold_waypoint_until_stable(
        self,
        coord: GPSCoord,
        hold_seconds=2.0,
        vel_threshold=0.10,
        pos_tolerance=0.15,
        timeout=30.0,
    ) -> bool:
        """
        Commands and holds a GPS waypoint, then waits until the drone is both
        near the waypoint, at the commanded altitude, and stable (low horizontal
        speed) for hold_seconds.

        :param coord: Target GPS waypoint.
        :param hold_seconds: Continuous stable time required.
        :param vel_threshold: Maximum horizontal speed (m/s) to count as stable.
        :param pos_tolerance: Horizontal distance tolerance to waypoint (m).
        :param timeout: Maximum overall wait time (s).
        """
        target_alt = coord.alt if coord.alt is not None else self.cruise_alt
        target = LocationGlobalRelative(coord.lat, coord.long, target_alt)

        print(
            f"[*] Holding waypoint ({coord.lat:.7f}, {coord.long:.7f}) and waiting {hold_seconds:.1f}s stable..."
        )

        t_start = time.time()
        stable_start_time = None

        while time.time() - t_start < timeout:
            # Re-issue target to emulate a GUIDED loiter-at-waypoint hold
            self.vehicle.simple_goto(target)

            loc = self.vehicle.location.global_relative_frame
            velocity = self.vehicle.velocity

            if (
                loc is not None
                and loc.lat is not None
                and loc.lon is not None
                and velocity is not None
                and len(velocity) >= 2
            ):
                horizontal_speed = math.hypot(velocity[0], velocity[1])

                current_pos = GPSCoord(loc.lat, loc.lon, loc.alt if loc.alt else 0)
                target_pos = GPSCoord(coord.lat, coord.long, 0)
                distance = horiz_distance_m(current_pos, target_pos)
                altitude_ok = loc.alt is not None and loc.alt >= target_alt

                if (
                    distance <= pos_tolerance
                    and horizontal_speed <= vel_threshold
                    and altitude_ok
                ):
                    if stable_start_time is None:
                        stable_start_time = time.time()
                    elif (time.time() - stable_start_time) >= hold_seconds:
                        print(
                            f"[*] Waypoint hold stable. dist={distance:.2f}m speed={horizontal_speed:.3f}m/s"
                        )
                        return True
                else:
                    stable_start_time = None

            time.sleep(0.2)

        print("[!] Waypoint hold stability timeout reached.")
        return False
