"""
run mission to pickup and drop arucos at specified waypoints
- mission begins
- for id in range: land and pickup payload
- fly to drop waypoint, wait 2 seconds (to settle), drop
- repeat for all ids in list
"""

from __future__ import annotations

from .common_types import *
from .control.mission_info import MissonTracker
from .control.drone_control import DroneControl
from . import timebase as time
from .sensors.camera.camera import Camera, MarkerObservation
from .utils.position_smoother import RelPosSmoother
from enum import Enum
import json
import math
from statistics import median
from typing import Optional, Tuple, TYPE_CHECKING

if TYPE_CHECKING:
    from .sensors.lidar.lidar import Lidar
    from .sensors.servo.servo import Dropper

ARUCO_PICKUP = GPSCoord(39.9337075, -75.7802787, 10)
DROP_POINT = GPSCoord(39.9338306, -75.7801814, 10)
ALT_TOL = 0.03
HOVER_ALT_TOL = 1
PRECISION_LANDING_MIN_AGL_METERS = 0.75
TARGET_HOVER_HEIGHT = 4.572
WINDOW = 5
MULT = 0.3
CAMERA_MAX_AGE_SECONDS = 0.25
LIDAR_MAX_AGE_SECONDS = 0.50
TARGET_HEALTH_TIMEOUT_SECONDS = 0.50
HOLD_TIMEOUT_SECONDS = 5.0
HOLD_COMMAND_PERIOD_SECONDS = 0.20
ANCHOR_DRIFT_LIMIT_METERS = 0.20
REACQUIRE_ERROR_LIMIT_METERS = 0.50
REACQUIRE_FRAME_COUNT = 5


class LandingResult(Enum):
    TOUCHDOWN = "touchdown"
    RETRY = "retry"
    FAILED = "failed"


def _json_number(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _precision_log(**fields) -> None:
    clean = {key: _json_number(value) for key, value in fields.items()}
    print(
        "PRECISION_LANDING "
        + json.dumps(clean, sort_keys=True, separators=(",", ":"))
    )


def _horizontal_distance_m(first: GPSCoord, second: GPSCoord) -> float:
    latitude_radians = math.radians((first.lat + second.lat) / 2.0)
    north = math.radians(second.lat - first.lat) * 6378137.0
    east = (
        math.radians(second.long - first.long)
        * math.cos(latitude_radians)
        * 6378137.0
    )
    return math.hypot(north, east)


def _offset_gps(original: GPSCoord, north: float, east: float) -> GPSCoord:
    earth_radius = 6378137.0
    latitude = original.lat + math.degrees(north / earth_radius)
    longitude = original.long + math.degrees(
        east / (earth_radius * math.cos(math.radians(original.lat)))
    )
    return GPSCoord(latitude, longitude, original.alt)


def _observation_reason(
    observation,
    *,
    now_ns: int,
    last_timestamp_ns: int,
    lidar_age_seconds: float,
    horizontal_error_m: float | None,
    anchor_drift_m: float | None,
) -> str | None:
    if observation is None:
        return "no_frame"
    if observation.frame_timestamp_ns <= last_timestamp_ns:
        return "not_new"
    if observation.frame_timestamp_ns > now_ns:
        return "future_frame"
    if (now_ns - observation.frame_timestamp_ns) / 1_000_000_000 > CAMERA_MAX_AGE_SECONDS:
        return "stale_frame"
    if observation.vector is None:
        return "target_absent"
    if not all(
        math.isfinite(value)
        for value in (observation.vector.x, observation.vector.y, observation.vector.z)
    ):
        return "non_finite"
    if observation.vector.z <= 0:
        return "non_positive_down"
    if lidar_age_seconds > LIDAR_MAX_AGE_SECONDS:
        return "stale_lidar"
    if anchor_drift_m is not None and anchor_drift_m > ANCHOR_DRIFT_LIMIT_METERS:
        return "anchor_drift"
    if (
        horizontal_error_m is not None
        and horizontal_error_m > REACQUIRE_ERROR_LIMIT_METERS
    ):
        return "horizontal_error"
    return None


def _marker_offset_ne(update: RelPosComplete, attitude) -> Tuple[float, float]:
    """Project one body-FRD marker vector into earth North/East."""
    roll = attitude.roll
    pitch = attitude.pitch
    yaw = attitude.yaw

    cos_roll, sin_roll = math.cos(roll), math.sin(roll)
    cos_pitch, sin_pitch = math.cos(pitch), math.sin(pitch)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)

    rolled_right = cos_roll * update.y - sin_roll * update.z
    rolled_down = sin_roll * update.y + cos_roll * update.z
    pitched_forward = cos_pitch * update.x + sin_pitch * rolled_down
    pitched_right = rolled_right

    north = cos_yaw * pitched_forward - sin_yaw * pitched_right
    east = sin_yaw * pitched_forward + cos_yaw * pitched_right
    return north, east


def _read_lidar_or_fallback(
    lidar: Lidar, controller: DroneControl
) -> Tuple[Optional[float], bool]:
    """Return (altitude, has_lidar) with GPS fallback when LiDAR is unavailable."""
    try:
        return lidar.get_distance(), True
    except Exception as lidar_error:
        print(f"[WARN] LiDAR read failed: {lidar_error}. Falling back to GPS altitude.")
        try:
            return controller.get_current_gps().alt, False
        except Exception as gps_error:
            print(f"[ERR] GPS fallback after LiDAR failure also failed: {gps_error}")
            return None, False


def aruco_land_precision(
    controller: DroneControl,
    camera: Camera,
    lidar: Lidar,
    target_id: int,
    anchor: GPSCoord | None = None,
    deadline: float | None = None,
    return_result: bool = False,
    retry_number: int = 0,
) -> bool | LandingResult:
    """Land using only healthy target observations, holding to reacquire once."""
    started = time.time()
    deadline = started + 60.0 if deadline is None else deadline
    if controller.set_land_mode() == -1:
        result = LandingResult.FAILED
        return result if return_result else False
    _precision_log(
        state="TRACKING",
        reason="landing_started",
        sim_time=started,
        target_id=target_id,
        retry_number=retry_number,
        requested_mode="LAND",
        observed_mode="LAND",
    )

    alt, has_lidar = _read_lidar_or_fallback(lidar, controller)
    if alt is None:
        result = LandingResult.FAILED
        return result if return_result else False
    lidar_sample_time = started if has_lidar else float("-inf")
    last_healthy_time = started
    last_timestamp_ns = -1
    state = "TRACKING"
    hold_started = None
    hold_waypoint = None
    next_hold_command = None
    reacquired_frames = 0
    below_precision_minimum_logged = False

    while time.time() <= deadline:
        now = time.time()
        if (
            not controller.vehicle.armed
            or alt <= ALT_TOL
            or controller.is_landed()
        ):
            _precision_log(
                state=state,
                reason="touchdown_confirmed",
                sim_time=now,
                target_id=target_id,
                retry_number=retry_number,
            )
            print("[*] Touchdown confirmed!")
            result = LandingResult.TOUCHDOWN
            return result if return_result else True

        try:
            alt = lidar.get_distance()
            has_lidar = True
            lidar_sample_time = now
        except Exception:
            has_lidar = False

        if (
            state == "TRACKING"
            and has_lidar
            and alt <= PRECISION_LANDING_MIN_AGL_METERS
        ):
            if not below_precision_minimum_logged:
                _precision_log(
                    state=state,
                    reason="below_precision_minimum",
                    sim_time=now,
                    target_id=target_id,
                    retry_number=retry_number,
                    agl_m=alt,
                )
                below_precision_minimum_logged = True
            last_healthy_time = now
            time.sleep(0.05)
            continue

        observation = None
        capture_error = None
        try:
            if hasattr(camera, "observe_marker_3d"):
                observation = camera.observe_marker_3d(
                    target_id,
                    lidar_alt=alt if has_lidar else None,
                    quality=4,
                    deadline_sim_ns=int(min(deadline, now + 0.05) * 1_000_000_000),
                )
            else:
                raw_update = camera.vec_to_marker_3d(
                    target_id,
                    lidar_alt=alt if has_lidar else None,
                    quality=4,
                )
                from .sensors.camera.camera import MarkerObservation

                observation = MarkerObservation(
                    vector=raw_update,
                    frame_timestamp_ns=int(now * 1_000_000_000),
                )
        except RuntimeError as error:
            capture_error = str(error)

        if not controller.vehicle.armed or controller.is_landed():
            continue

        vector = observation.vector if observation is not None else None
        horizontal_error = None
        anchor_drift = None
        if vector is not None and all(
            math.isfinite(value) for value in (vector.x, vector.y, vector.z)
        ):
            attitude = getattr(controller.vehicle, "attitude", None)
            if attitude is None:
                north, east = vector.x, vector.y
            else:
                north, east = _marker_offset_ne(vector, attitude)
            horizontal_error = math.hypot(north, east)
            if anchor is not None:
                current = controller.get_current_gps()
                projected = _offset_gps(current, north, east)
                anchor_drift = _horizontal_distance_m(projected, anchor)

        reason = _observation_reason(
            observation,
            now_ns=int(now * 1_000_000_000),
            last_timestamp_ns=last_timestamp_ns,
            lidar_age_seconds=now - lidar_sample_time,
            horizontal_error_m=horizontal_error,
            anchor_drift_m=anchor_drift,
        )
        if observation is not None:
            last_timestamp_ns = max(last_timestamp_ns, observation.frame_timestamp_ns)

        log_fields = {
            "state": state,
            "reason": capture_error or reason or "accepted",
            "sim_time": now,
            "target_id": target_id,
            "frame_timestamp_ns": (
                observation.frame_timestamp_ns if observation is not None else None
            ),
            "frame_age_s": (
                (int(now * 1_000_000_000) - observation.frame_timestamp_ns)
                / 1_000_000_000
                if observation is not None
                else None
            ),
            "agl_m": alt,
            "marker_forward_m": vector.x if vector is not None else None,
            "marker_right_m": vector.y if vector is not None else None,
            "marker_down_m": vector.z if vector is not None else None,
            "horizontal_error_m": horizontal_error,
            "anchor_drift_m": anchor_drift,
            "retry_number": retry_number,
        }

        if state == "TRACKING":
            if reason is None and capture_error is None:
                controller.land_send_landing_target(vector)
                last_healthy_time = now
            else:
                _precision_log(**log_fields)
            if now - last_healthy_time >= TARGET_HEALTH_TIMEOUT_SECONDS:
                if controller.set_guided_mode() != 0:
                    result = LandingResult.FAILED
                    return result if return_result else False
                hold_waypoint = controller.get_current_gps()
                hold_started = now
                next_hold_command = now
                reacquired_frames = 0
                state = "HOLD_REACQUIRE"
                _precision_log(
                    state=state,
                    reason="target_unhealthy",
                    sim_time=now,
                    target_id=target_id,
                    retry_number=retry_number,
                    requested_mode="GUIDED",
                    observed_mode="GUIDED",
                )
        else:
            assert hold_started is not None
            assert hold_waypoint is not None
            assert next_hold_command is not None
            if now >= next_hold_command:
                if controller.send_guided_waypoint(hold_waypoint) != 0:
                    result = LandingResult.FAILED
                    return result if return_result else False
                next_hold_command = now + HOLD_COMMAND_PERIOD_SECONDS
            if reason is None and capture_error is None:
                reacquired_frames += 1
                if reacquired_frames >= REACQUIRE_FRAME_COUNT:
                    if controller.set_land_mode() == -1:
                        result = LandingResult.FAILED
                        return result if return_result else False
                    state = "TRACKING"
                    last_healthy_time = now
                    _precision_log(
                        state=state,
                        reason="target_reacquired",
                        sim_time=now,
                        target_id=target_id,
                        retry_number=retry_number,
                        requested_mode="LAND",
                        observed_mode="LAND",
                    )
            else:
                reacquired_frames = 0
                _precision_log(**log_fields)
            if state == "HOLD_REACQUIRE" and now - hold_started >= HOLD_TIMEOUT_SECONDS:
                _precision_log(
                    state=state,
                    reason="hold_timeout",
                    sim_time=now,
                    target_id=target_id,
                    retry_number=retry_number,
                )
                result = LandingResult.RETRY
                return result if return_result else False

        time.sleep(0.05)

    _precision_log(
        state=state,
        reason="landing_deadline",
        sim_time=time.time(),
        target_id=target_id,
        retry_number=retry_number,
    )
    print("[!] Precision landing ended without touchdown confirmation.")
    result = LandingResult.FAILED
    return result if return_result else False


def _median_anchor(samples: list[GPSCoord]) -> GPSCoord | None:
    if len(samples) != REACQUIRE_FRAME_COUNT:
        return None
    if any(
        _horizontal_distance_m(first, second) > ANCHOR_DRIFT_LIMIT_METERS
        for index, first in enumerate(samples)
        for second in samples[index + 1 :]
    ):
        return None
    return GPSCoord(
        median(sample.lat for sample in samples),
        median(sample.long for sample in samples),
        0.0,
    )


def _acquisition_observation(
    camera: Camera,
    target_id: int,
    deadline: float,
    *,
    lidar_alt: float | None = None,
) -> MarkerObservation | None:
    now = time.time()
    try:
        if hasattr(camera, "observe_marker_3d"):
            observation = camera.observe_marker_3d(
                target_id,
                lidar_alt=lidar_alt,
                quality=4,
                deadline_sim_ns=int(min(deadline, now + 0.1) * 1_000_000_000),
            )
            if observation is None:
                return None
            age = (int(now * 1_000_000_000) - observation.frame_timestamp_ns) / 1e9
            if observation.frame_timestamp_ns > int(now * 1_000_000_000):
                return None
            if age > CAMERA_MAX_AGE_SECONDS:
                return None
            return observation
        vector = camera.vec_to_marker_3d(
            target_id, lidar_alt=lidar_alt, quality=4
        )
        timestamp = camera.last_frame_timestamp
        if timestamp is None:
            return None
        return MarkerObservation(vector=vector, frame_timestamp_ns=int(timestamp))
    except RuntimeError:
        return None


def _acquire_target_anchor(
    controller: DroneControl,
    camera: Camera,
    lidar: Lidar,
    target_id: int,
    deadline: float,
) -> GPSCoord | None:
    print("[*] Searching for ArUco to initiate Precision Landing...")
    center_anchor = controller.get_current_gps()
    grid_size = 1.5
    grid_offsets_ne = [
        (0, 0),
        (0, grid_size),
        (grid_size, grid_size),
        (grid_size, 0),
        (grid_size, -grid_size),
        (0, -grid_size),
        (-grid_size, -grid_size),
        (-grid_size, 0),
        (-grid_size, grid_size),
    ]

    for grid_north, grid_east in grid_offsets_ne:
        if time.time() >= deadline:
            break
        target_wp = controller.get_location_metres(
            center_anchor, grid_north, grid_east
        )
        result = controller.goto_waypoint(target_wp, position_tol=0.15)
        print(f"return of goto func: {result}")
        if result != 0:
            return None

        for _ in range(5):
            correction = _acquisition_observation(camera, target_id, deadline)
            if correction is None or correction.vector is None:
                time.sleep(0.1)
                continue
            update = correction.vector
            if not all(math.isfinite(value) for value in (update.x, update.y, update.z)):
                time.sleep(0.1)
                continue
            if update.z <= 0:
                time.sleep(0.1)
                continue

            print("[*] Target Acquired! Preparing precision LAND.")
            controller.vehicle.flush()
            current_gps = controller.get_current_gps()
            north, east = _marker_offset_ne(update, controller.vehicle.attitude)
            corrected_wp = controller.get_location_metres(current_gps, north, east)
            result = controller.goto_waypoint(corrected_wp, position_tol=0.15)
            print(f"return of goto func: {result}")
            if result != 0:
                return None

            time.sleep(1.0)
            seen_timestamps = {correction.frame_timestamp_ns}
            anchor_samples: list[GPSCoord] = []
            recenter_considered = False
            agl_recenter_considered = False
            while time.time() < deadline:
                centered = _acquisition_observation(camera, target_id, deadline)
                try:
                    acquisition_agl = lidar.get_distance()
                except Exception as error:
                    print(f"[ERR] Cannot verify acquisition AGL: {error}")
                    anchor_samples.clear()
                    time.sleep(0.1)
                    continue
                if (
                    centered is None
                    or centered.vector is None
                    or centered.frame_timestamp_ns in seen_timestamps
                ):
                    time.sleep(0.1)
                    continue
                seen_timestamps.add(centered.frame_timestamp_ns)
                centered_update = centered.vector
                if not all(
                    math.isfinite(value)
                    for value in (
                        centered_update.x,
                        centered_update.y,
                        centered_update.z,
                    )
                ) or centered_update.z <= 0:
                    anchor_samples.clear()
                    time.sleep(0.1)
                    continue
                if abs(acquisition_agl - TARGET_HOVER_HEIGHT) > HOVER_ALT_TOL:
                    anchor_samples.clear()
                    if not agl_recenter_considered:
                        agl_recenter_considered = True
                        if controller.guide_move_relative_frame(
                            RelPosComplete(
                                0,
                                0,
                                acquisition_agl - TARGET_HOVER_HEIGHT,
                            )
                        ) != 0:
                            return None
                    time.sleep(0.1)
                    continue

                north, east = _marker_offset_ne(
                    centered_update, controller.vehicle.attitude
                )
                horizontal_error = math.hypot(north, east)
                first_valid_result = not recenter_considered
                recenter_considered = True
                if first_valid_result and horizontal_error > REACQUIRE_ERROR_LIMIT_METERS:
                    current_gps = controller.get_current_gps()
                    recentered_wp = controller.get_location_metres(
                        current_gps, north * MULT, east * MULT
                    )
                    if controller.goto_waypoint(recentered_wp, position_tol=0.15) != 0:
                        return None
                    anchor_samples.clear()
                elif horizontal_error <= REACQUIRE_ERROR_LIMIT_METERS:
                    current_gps = controller.get_current_gps()
                    projected = _offset_gps(current_gps, north, east)
                    candidate_samples = anchor_samples + [projected]
                    if len(candidate_samples) == REACQUIRE_FRAME_COUNT:
                        anchor = _median_anchor(candidate_samples)
                        if anchor is not None:
                            return anchor
                        anchor_samples = [projected]
                    else:
                        anchor_samples = candidate_samples
                else:
                    anchor_samples.clear()
                time.sleep(0.1)
            return None
    print("[!] Grid search exhausted, target not found.")
    return None


def pickup_sequence(
    controller: DroneControl,
    camera: Camera,
    lidar: Lidar,
    target_id: int,
    dropper: Dropper,
) -> bool:
    if controller.set_guided_mode() != 0:
        return False
    deadline = time.time() + 60.0
    try:
        alt = lidar.get_distance()
    except Exception as error:
        print(f"[ERR] Cannot read acquisition AGL: {error}")
        print("[ERR] Cannot start pickup sequence without altitude data.")
        return False
    how_much_down = alt - TARGET_HOVER_HEIGHT
    print(f"moving down {how_much_down}m")
    if controller.guide_move_relative_frame(RelPosComplete(0, 0, how_much_down)) != 0:
        return False

    while time.time() < deadline:
        try:
            hover_alt = lidar.get_distance()
        except Exception as error:
            print(f"[ERR] Cannot verify acquisition AGL: {error}")
            time.sleep(0.1)
            continue
        if abs(hover_alt - TARGET_HOVER_HEIGHT) <= HOVER_ALT_TOL:
            break
        time.sleep(0.1)
    else:
        print("[!] Acquisition AGL was not reached.")
        return False

    profile_checked = False
    for attempt in range(2):
        anchor = _acquire_target_anchor(
            controller, camera, lidar, target_id, deadline
        )
        if anchor is None:
            return False
        if not profile_checked:
            profile_checked = True
            if not controller.require_precision_landing_profile():
                return False
        result = aruco_land_precision(
            controller,
            camera,
            lidar,
            target_id,
            anchor,
            deadline,
            True,
            attempt,
        )
        if result is True or result is LandingResult.TOUCHDOWN:
            if controller.disarm() != 0:
                return False
            if dropper.attach(target_id) is not True:
                print(f"[!] Attachment rejected for ID {target_id}.")
                return False
            return True
        if result is not LandingResult.RETRY or attempt == 1:
            return False
        if controller.set_guided_mode() != 0:
            return False
        retry_hover = controller.get_current_gps()
        retry_hover.alt = TARGET_HOVER_HEIGHT
        if controller.goto_waypoint(retry_hover, position_tol=0.15) != 0:
            return False
        _precision_log(
            state="SEARCH_HOVER",
            reason="landing_retry",
            sim_time=time.time(),
            target_id=target_id,
            retry_number=1,
            requested_mode="GUIDED",
            observed_mode="GUIDED",
        )
    return False


def fm3(
    mt: MissonTracker,
    controller: DroneControl,
    camera: Camera,
    lidar: Lidar,
    dropper: Dropper,
    possible_ids: set,
    pickup_point: GPSCoord,
    target_point: GPSCoord,
):
    IDs = list(possible_ids)
    desired_drop_height_m = 10
    try:
        for id in IDs:

            if mt.time_left() < 60:
                return False

            print("going to pickup waypoint")
            val = controller.goto_waypoint(pickup_point)
            print(f"return of goto func: {val}")
            if val != 0:
                return False

            print("init pickup sequence")
            success = pickup_sequence(controller, camera, lidar, id, dropper)

            if success:
                if controller.set_guided_mode() != 0:
                    return False
                print("climb")
                if controller.vehicle.armed and controller.is_landed():
                    print("vehicle armed")
                    controller.simple_takeoff(10)
                elif controller.vehicle.armed:
                    controller.set_guided_mode()
                    controller.climb(10)
                else:
                    time.sleep(8)
                    controller.force_arm_takeoff(10)

                print("going to drop point")
                val = controller.goto_waypoint(target_point)
                print(f"return of goto func: {val}")
                if val != 0:
                    return False

                camera.save_frame_buffer_async()

                print("dropping")
                drop_target = GPSCoord(
                    target_point.lat, target_point.long, desired_drop_height_m
                )
                lidar_alt, _ = _read_lidar_or_fallback(lidar, controller)
                if lidar_alt is not None and lidar_alt < desired_drop_height_m:
                    drop_target.alt += desired_drop_height_m - lidar_alt
                    if controller.goto_waypoint(drop_target) != 0:
                        return False
                if not controller.hold_waypoint_until_stable(
                    drop_target, lidar, required_agl_m=10.0
                ):
                    print(f"Skipping drop for ID {id}: stability gate timed out.")
                    return False
                dropper.drop()
            else:
                print(f"Skipping drop for ID {id} because pickup failed.")
                return False

        return True

    except Exception as e:
        print("[ERR]", e)
        controller.rtl()
        camera.save_frame_buffer_async()
        return False
    except KeyboardInterrupt as e:
        print("[ERR]", e)
        controller.rtl()
        camera.save_frame_buffer_async()
        return False


if __name__ == "__main__":
    from .sensors.lidar.lidar import Lidar
    from .sensors.servo.servo import Dropper

    mt = MissonTracker(600)
    mt.begin_mission()
    controller = DroneControl("/dev/ttyACM0")
    camera = Camera(50)
    try:
        lidar = Lidar()
    except Exception as e:
        print(f"[ERR] LiDAR initialization failed: {e}")
        raise SystemExit(1)
    dropper = Dropper()

    # controller.takeoff(10)
    original_gps = controller.get_current_gps()
    original_gps.alt = 10
    controller.force_arm_takeoff(10)

    fm3(mt, controller, camera, lidar, dropper, {6, 7}, ARUCO_PICKUP, DROP_POINT)

    val = controller.goto_waypoint(original_gps)

    print(f"return of goto func: {val}")
    controller.simple_land()
    controller.disarm()
