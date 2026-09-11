from ..common_types import *
import os
import time as wall_time
from typing import Callable, Mapping, Optional
import itertools
import threading
from dataclasses import dataclass

# os.environ["MAVLINK20"] = "1"

from .. import timebase as time
import math
import collections

if not hasattr(collections, "MutableMapping"):
    import collections.abc

    collections.MutableMapping = collections.abc.MutableMapping  # type: ignore

from dronekit import connect, VehicleMode  # type: ignore
from pymavlink import mavutil  # type: ignore

from .flight_state import FailsafeEvidence, FlightState, RCInput, SourceIdentity
from .mission_supervisor import AuthorityLost, FlightOperationError, MissionAbort
from .stability import ReleaseEvidence, ReleaseHoldConfirmation, ReleaseStabilityConfig
from ..sensors.lidar.clearance import (
    AttitudeSample,
    ClearanceCalibration,
    ClearanceUnavailableError,
    project_vertical_clearance,
)
from ..sensors.lidar.lidar import StaleSensorError

# === CONFIG ===
GROUND_SPEED = 10.0
ALT_TOL = 0.8
POS_TOL = 1.0
TIMEOUT_MOVE = 120

PRECISION_LANDING_PARAMETERS = {
    "LAND_SPD_MS": 0.50,
    "PLND_ENABLED": 1,
    "PLND_TYPE": 1,
    "PLND_EST_TYPE": 0,
    "PLND_LAG": 0.08,
    "PLND_XY_DIST_MAX": 0.50,
    "PLND_STRICT": 2,
    "PLND_RET_MAX": 1,
    "PLND_TIMEOUT": 0.50,
    "PLND_ALT_MIN": 0.75,
    "PLND_ALT_MAX": 8.0,
    "PLND_OPTIONS": 4,
}
TIMEOUT_MODE = 10.0
TIMEOUT_ARM = 15.0
TIMEOUT_ASCENT = 45.0
TIMEOUT_LAND = 180.0
TIMEOUT_DISARM = 16.0

FEET_TO_METERS = 0.3048
ALT_30FT = 30 * FEET_TO_METERS  # 9.144 m
ALT_40FT = 40 * FEET_TO_METERS  # 12.192 m


@dataclass(frozen=True)
class CommandAck:
    sequence: int
    command: int
    result: int


@dataclass(frozen=True)
class MissionAck:
    sequence: int
    result: int


class _OutputGuardedWriter:
    """Recheck guarded flight output at DroneKit's actual queue boundary."""

    def __init__(self, delegate):
        self.delegate = delegate
        self.queue = delegate.queue
        self._local = threading.local()

    def transaction(self, operation, enqueue_check):
        if not callable(operation) or not callable(enqueue_check):
            raise TypeError("transport operation and enqueue check must be callable")
        if getattr(self._local, "enqueue_check", None) is not None:
            raise RuntimeError("nested guarded transport output is unsupported")
        self._local.enqueue_check = enqueue_check
        try:
            return operation()
        finally:
            self._local.enqueue_check = None

    def write(self, packet):
        enqueue_check = getattr(self._local, "enqueue_check", None)
        if enqueue_check is not None:
            return enqueue_check(lambda: self.delegate.write(packet))
        return self.delegate.write(packet)

    def read(self):
        return self.delegate.read()


class CommandAckTracker:
    """Correlate targeted MAVLink 2 COMMAND_ACK messages from the pinned FC."""

    def __init__(
        self,
        *,
        wire_protocol: str,
        source_system: int,
        source_component: int,
        target_system: int,
        target_component: int,
    ):
        if wire_protocol != "2.0":
            raise ValueError("flight command ACK correlation requires MAVLink 2")
        self._source_system = source_system
        self._source_component = source_component
        self._target_system = target_system
        self._target_component = target_component
        self._sequence = 0
        self._acks = collections.deque(maxlen=256)
        self._lock = threading.Lock()

    def boundary(self) -> int:
        with self._lock:
            return self._sequence

    def observe(
        self,
        *,
        command,
        result,
        source_system,
        source_component,
        target_system,
        target_component,
    ) -> bool:
        if (
            isinstance(command, bool)
            or not isinstance(command, int)
            or isinstance(result, bool)
            or not isinstance(result, int)
            or source_system != self._source_system
            or source_component != self._source_component
            or target_system != self._target_system
            or target_component != self._target_component
        ):
            return False
        with self._lock:
            self._sequence += 1
            self._acks.append(CommandAck(self._sequence, command, result))
        return True

    def result_after(self, *, command: int, boundary: int) -> int | None:
        with self._lock:
            progress_seen = False
            for ack in self._acks:
                if ack.sequence <= boundary or ack.command != command:
                    continue
                if ack.result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                    progress_seen = True
                    continue
                return ack.result
            if progress_seen:
                return mavutil.mavlink.MAV_RESULT_IN_PROGRESS
        return None

    def enqueue(self, output) -> tuple[int, object]:
        """Capture a boundary and enqueue while observations are excluded."""
        with self._lock:
            boundary = self._sequence
            result = output()
            return boundary, result


class MissionAckTracker:
    """Correlate MISSION_ACK by pinned FC, companion recipient, and mission type."""

    def __init__(
        self,
        *,
        source_system: int,
        source_component: int,
        target_system: int,
        target_component: int,
        mission_type: int,
    ):
        self._source_system = source_system
        self._source_component = source_component
        self._target_system = target_system
        self._target_component = target_component
        self._mission_type = mission_type
        self._sequence = 0
        self._acks = collections.deque(maxlen=256)
        self._lock = threading.Lock()

    def boundary(self) -> int:
        with self._lock:
            return self._sequence

    def observe_message(self, message) -> bool:
        result = getattr(message, "type", None)
        mission_type = getattr(
            message, "mission_type", mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )
        try:
            source_system = message.get_srcSystem()
            source_component = message.get_srcComponent()
        except Exception:
            return False
        if (
            isinstance(result, bool)
            or not isinstance(result, int)
            or source_system != self._source_system
            or source_component != self._source_component
            or getattr(message, "target_system", None) != self._target_system
            or getattr(message, "target_component", None) != self._target_component
            or mission_type != self._mission_type
        ):
            return False
        with self._lock:
            self._sequence += 1
            self._acks.append(MissionAck(self._sequence, result))
        return True

    def result_after(self, *, boundary: int) -> int | None:
        with self._lock:
            for ack in self._acks:
                if ack.sequence > boundary:
                    return ack.result
        return None

    def enqueue(self, output, *, boundary_callback=None) -> tuple[int, object]:
        with self._lock:
            boundary = self._sequence
            if boundary_callback is not None:
                boundary_callback(boundary)
            result = output()
            return boundary, result


class MissionAckError(FlightOperationError):
    pass


class MissionAckTimeout(MissionAckError):
    pass


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
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        alt = vehicle.location.global_relative_frame.alt
        if _finite_number(alt) and abs(float(alt) - target_alt_m) <= tol:
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
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        loc = vehicle.location.global_relative_frame
        if loc is not None and _finite_number(loc.lat) and _finite_number(loc.lon):
            current_pos = GPSCoord(float(loc.lat), float(loc.lon), 0)
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
            if alt_m is not None:
                alt_ok = _finite_number(loc.alt) and abs(float(loc.alt) - alt_m) <= alt_tol
            if d <= pos_tol and alt_ok:
                return True
        time.sleep(0.3)
    return False


# I hope the drone is on the ground when this is called...
def _finite_number(value) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def arm_and_takeoff(vehicle, target_alt_m):
    raise NotImplementedError(
        "use DroneControl.force_arm_takeoff with an installed authority guard"
    )


def _send_guided_waypoint(
    vehicle,
    lat,
    lon,
    amsl_m,
    target_system,
    target_component,
    permission_check=None,
):
    """Send an AMSL GUIDED waypoint without losing lat/lon precision."""
    if permission_check is None:
        raise AuthorityLost("guided waypoint output has no installed guard")
    permission_check()
    vehicle._master.mav.mission_item_int_send(
        target_system,
        target_component,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2,
        0,
        0,
        0,
        0,
        0,
        int(round(lat * 1e7)),
        int(round(lon * 1e7)),
        amsl_m,
    )


def goto(vehicle, lat, lon, alt_m, permission_check=None):
    raise NotImplementedError(
        "use DroneControl.goto_waypoint with a pinned mission home"
    )


class DroneControl:
    def __init__(
        self,
        connection_port="/dev/cu.usbmodem1103",
        *,
        source_identity: SourceIdentity | None = None,
        flight_controller_target: SourceIdentity | None = None,
        wire_protocol: str | None = None,
        wait_ready: bool = True,
        heartbeat_timeout: float = 60,
        flight_state: FlightState | None = None,
        permission_guard: Callable[[], None] | None = None,
        heartbeat_mode_decoder: Callable[[object], str | None] | None = None,
        rc_health_decoder: Callable[[object, int, int], bool | None] | None = None,
        sys_status_observer: Callable[[object], None] | None = None,
        failsafe_decoders: Mapping[
            str, Callable[[object], FailsafeEvidence | None]
        ]
        | None = None,
        mission_home_check: Callable[[MissionHome], None] | None = None,
        fc_home_position_tolerance_m: float | None = None,
        fc_home_altitude_tolerance_m: float | None = None,
        clearance_calibration: ClearanceCalibration | None = None,
        release_stability_config: ReleaseStabilityConfig | None = None,
        home_request_timeout_s: float | None = None,
        telemetry_poll_interval_s: float | None = None,
        guided_output_delivery_callback: Callable[[], None] | None = None,
    ):
        if not isinstance(source_identity, SourceIdentity):
            raise ValueError("source_identity must be explicit")
        if not isinstance(flight_controller_target, SourceIdentity):
            raise ValueError("flight_controller_target must be explicit")
        if source_identity == flight_controller_target:
            raise ValueError("connection source and FC target must be distinct")
        if source_identity.system_id == 0 or source_identity.component_id == 0:
            raise ValueError("companion source identity must be nonzero")
        if flight_state is not None and flight_state.source != flight_controller_target:
            raise ValueError("FlightState source must match flight_controller_target")
        if flight_state is not None and wire_protocol != "2.0":
            raise ValueError("flight-enabled control requires configured MAVLink 2")
        if wire_protocol not in ("1.0", "2.0", None):
            raise ValueError("unsupported MAVLink wire protocol")
        if flight_state is None and any(
            value is not None
            for value in (
                heartbeat_mode_decoder,
                rc_health_decoder,
                sys_status_observer,
                failsafe_decoders,
            )
        ):
            raise ValueError("observation decoders require a FlightState")
        if heartbeat_mode_decoder is not None and not callable(heartbeat_mode_decoder):
            raise ValueError("heartbeat_mode_decoder must be callable")
        if rc_health_decoder is not None and not callable(rc_health_decoder):
            raise ValueError("rc_health_decoder must be callable")
        if sys_status_observer is not None and not callable(sys_status_observer):
            raise ValueError("sys_status_observer must be callable")
        configured_failsafe_decoders = dict(failsafe_decoders or {})
        if any(
            not isinstance(name, str) or not name or not callable(decoder)
            for name, decoder in configured_failsafe_decoders.items()
        ):
            raise ValueError("failsafe decoders require message names and callables")
        for name, tolerance in (
            ("fc_home_position_tolerance_m", fc_home_position_tolerance_m),
            ("fc_home_altitude_tolerance_m", fc_home_altitude_tolerance_m),
        ):
            if tolerance is not None and (
                not _finite_number(tolerance) or tolerance <= 0
            ):
                raise ValueError(f"{name} must be finite and positive")
        if clearance_calibration is not None and not isinstance(
            clearance_calibration, ClearanceCalibration
        ):
            raise ValueError("clearance_calibration must be a ClearanceCalibration")
        if release_stability_config is not None and not isinstance(
            release_stability_config, ReleaseStabilityConfig
        ):
            raise ValueError(
                "release_stability_config must be a ReleaseStabilityConfig"
            )
        if (home_request_timeout_s is None) != (telemetry_poll_interval_s is None):
            raise ValueError("post-arm HOME request timing must be configured together")
        if home_request_timeout_s is not None:
            if not _finite_number(home_request_timeout_s) or home_request_timeout_s <= 0:
                raise ValueError("home_request_timeout_s must be finite and positive")
            if (
                not _finite_number(telemetry_poll_interval_s)
                or telemetry_poll_interval_s <= 0
                or telemetry_poll_interval_s > home_request_timeout_s
            ):
                raise ValueError("telemetry_poll_interval_s is invalid")
        if (
            guided_output_delivery_callback is not None
            and not callable(guided_output_delivery_callback)
        ):
            raise ValueError("guided_output_delivery_callback must be callable or None")
        print(f"Connecting to {connection_port} …")
        vehicle = connect(
            connection_port,
            wait_ready=wait_ready,
            heartbeat_timeout=heartbeat_timeout,
            timeout=120,
            source_system=source_identity.system_id,
            source_component=source_identity.component_id,
        )
        actual_wire_protocol = getattr(
            getattr(vehicle, "_master", None), "WIRE_PROTOCOL_VERSION", None
        )
        if flight_state is not None and actual_wire_protocol != "2.0":
            try:
                close = getattr(vehicle, "close", None)
                if callable(close):
                    close()
            finally:
                raise ValueError(
                    "flight-enabled control requires actual MAVLink 2"
                )
        handler = getattr(vehicle, "_handler", None)
        discovered_system = getattr(handler, "target_system", None)
        if (
            not isinstance(discovered_system, int)
            or isinstance(discovered_system, bool)
            or discovered_system != flight_controller_target.system_id
        ):
            try:
                close = getattr(vehicle, "close", None)
                if callable(close):
                    close()
            finally:
                raise ValueError(
                    "discovered flight controller system does not match the pinned target"
                )
        self.vehicle = vehicle
        self.source_identity = source_identity
        self.flight_controller_target = flight_controller_target
        self.wire_protocol = wire_protocol
        self.cruise_alt = 10
        self.boot_time = time.monotonic()
        self.is_on_ground = False
        self.flight_state = flight_state
        self.permission_guard = permission_guard
        self.mission_home_check = mission_home_check
        self.fc_home_position_tolerance_m = fc_home_position_tolerance_m
        self.fc_home_altitude_tolerance_m = fc_home_altitude_tolerance_m
        self.clearance_calibration = clearance_calibration
        self.release_stability_config = release_stability_config
        self.home_request_timeout_s = home_request_timeout_s
        self.telemetry_poll_interval_s = telemetry_poll_interval_s
        self.guided_output_delivery_callback = guided_output_delivery_callback
        self._guided_output_delivery_reported = False
        self._release_hold_confirmation: ReleaseHoldConfirmation | None = None
        self._mission_home: MissionHome | None = None
        self._last_arm_boundary_sequence: int | None = None
        self._command_ack_tracker = (
            CommandAckTracker(
                wire_protocol=wire_protocol,
                source_system=flight_state.source.system_id,
                source_component=flight_state.source.component_id,
                target_system=source_identity.system_id,
                target_component=source_identity.component_id,
            )
            if flight_state is not None
            else None
        )
        self._mission_ack_tracker = (
            MissionAckTracker(
                source_system=flight_state.source.system_id,
                source_component=flight_state.source.component_id,
                target_system=source_identity.system_id,
                target_component=source_identity.component_id,
                mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
            if flight_state is not None
            else None
        )
        self._mission_ack_transaction_lock = threading.Lock()
        self._mission_ack_failed = False
        self._command_ack_transaction_lock = threading.Lock()
        self._telemetry_transaction_lock = threading.Lock()
        self._dependency_output_transaction = None
        self._supervisor_output_transaction = None
        writer = getattr(
            getattr(getattr(vehicle, "_master", None), "mav", None), "file", None
        )
        if (
            writer is not None
            and type(writer).__module__ == "dronekit.mavlink"
            and type(writer).__name__ == "MAVWriter"
        ):
            guarded_writer = _OutputGuardedWriter(writer)
            vehicle._master.mav.file = guarded_writer
            self._transport_output_transaction = guarded_writer.transaction
        else:
            self._transport_output_transaction = None
        self._startup_telemetry_open = True
        self._startup_telemetry_verifier = None
        self._startup_telemetry_verification_state = "missing"
        self._startup_telemetry_verification_lock = threading.Lock()
        observation_sequences = itertools.count(1)

        def observation_metadata(message):
            return {
                "received_at": time.monotonic(),
                "sequence": next(observation_sequences),
                "source_system": message.get_srcSystem(),
                "source_component": message.get_srcComponent(),
            }

        def message_matches_source(message):
            return (
                self.flight_state is not None
                and message.get_srcSystem() == self.flight_state.source.system_id
                and message.get_srcComponent() == self.flight_state.source.component_id
            )

        controller_ref = self

        @self.vehicle.on_message("EXTENDED_SYS_STATE")
        def listener_sys_state(vehicle, name, message):
            landed_state = getattr(message, "landed_state", None)
            metadata = observation_metadata(message)
            valid_landed_states = {
                mavutil.mavlink.MAV_LANDED_STATE_UNDEFINED,
                mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
                mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF,
                mavutil.mavlink.MAV_LANDED_STATE_LANDING,
            }
            if (
                not isinstance(landed_state, int)
                or isinstance(landed_state, bool)
                or landed_state not in valid_landed_states
            ):
                if message_matches_source(message):
                    self.flight_state.invalidate_observation(
                        "landed_state", **metadata
                    )
                    controller_ref.is_on_ground = False
                return
            on_ground = (
                landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
            )
            if self.flight_state is not None:
                accepted = self.flight_state.update(
                    "landed_state",
                    landed_state,
                    **metadata,
                )
                if accepted:
                    controller_ref.is_on_ground = on_ground
            else:
                controller_ref.is_on_ground = on_ground

        @self.vehicle.on_message("COMMAND_ACK")
        def listener_command_ack(vehicle, name, message):
            tracker = self._command_ack_tracker
            if tracker is not None:
                tracker.observe(
                    command=getattr(message, "command", None),
                    result=getattr(message, "result", None),
                    source_system=message.get_srcSystem(),
                    source_component=message.get_srcComponent(),
                    target_system=getattr(message, "target_system", None),
                    target_component=getattr(message, "target_component", None),
                )

        @self.vehicle.on_message("MISSION_ACK")
        def listener_mission_ack(vehicle, name, message):
            tracker = self._mission_ack_tracker
            if tracker is not None:
                tracker.observe_message(message)

        if self.flight_state is not None:

            @self.vehicle.on_message("HEARTBEAT")
            def listener_heartbeat(vehicle, name, message):
                metadata = observation_metadata(message)
                heartbeat_failsafe_decoder = configured_failsafe_decoders.get(
                    "HEARTBEAT"
                )
                if heartbeat_failsafe_decoder is not None and message_matches_source(
                    message
                ):
                    try:
                        heartbeat_failsafe = heartbeat_failsafe_decoder(message)
                    except Exception:
                        heartbeat_failsafe = None
                    if (
                        not isinstance(heartbeat_failsafe, FailsafeEvidence)
                        or not isinstance(heartbeat_failsafe.active, bool)
                        or not isinstance(heartbeat_failsafe.reason, str)
                        or not heartbeat_failsafe.reason
                    ):
                        self.flight_state.invalidate_observation(
                            "failsafe",
                            **metadata,
                        )
                    else:
                        self.flight_state.observe_failsafe(
                            heartbeat_failsafe.reason,
                            active=heartbeat_failsafe.active,
                            **metadata,
                        )
                base_mode = getattr(message, "base_mode", None)
                if isinstance(base_mode, bool) or not isinstance(base_mode, int):
                    if message_matches_source(message):
                        self.flight_state.invalidate_observations(
                            ("heartbeat", "armed", "mode"),
                            **metadata,
                        )
                    return
                decoded_mode = None
                if heartbeat_mode_decoder is not None and message_matches_source(message):
                    try:
                        decoded_mode = heartbeat_mode_decoder(message)
                    except Exception:
                        decoded_mode = None
                self.flight_state.observe_heartbeat(
                    (
                        getattr(message, "type", None),
                        getattr(message, "autopilot", None),
                        base_mode,
                        getattr(message, "custom_mode", None),
                        getattr(message, "system_status", None),
                    ),
                    armed=bool(
                        base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    ),
                    mode=decoded_mode,
                    **metadata,
                )

            @self.vehicle.on_message("GLOBAL_POSITION_INT")
            def listener_global_position(vehicle, name, message):
                metadata = observation_metadata(message)
                accepted = self.flight_state.update_many(
                    {
                        "location": (
                            getattr(message, "lat", None),
                            getattr(message, "lon", None),
                            getattr(message, "alt", None),
                            getattr(message, "relative_alt", None),
                        ),
                        "velocity": (
                            getattr(message, "vx", None),
                            getattr(message, "vy", None),
                            getattr(message, "vz", None),
                        ),
                    },
                    **metadata,
                )
                if not accepted and message_matches_source(message):
                    self.flight_state.invalidate_evidence(
                        ("location", "velocity"), **metadata
                    )

            @self.vehicle.on_message("ATTITUDE")
            def listener_attitude(vehicle, name, message):
                metadata = observation_metadata(message)
                accepted = self.flight_state.update(
                    "attitude",
                    (
                        getattr(message, "roll", None),
                        getattr(message, "pitch", None),
                        getattr(message, "yaw", None),
                        getattr(message, "rollspeed", None),
                        getattr(message, "pitchspeed", None),
                        getattr(message, "yawspeed", None),
                    ),
                    **metadata,
                )
                if not accepted and message_matches_source(message):
                    self.flight_state.invalidate_evidence(("attitude",), **metadata)

            @self.vehicle.on_message("HOME_POSITION")
            def listener_home(vehicle, name, message):
                self.flight_state.update(
                    "home",
                    (
                        getattr(message, "latitude", None),
                        getattr(message, "longitude", None),
                        getattr(message, "altitude", None),
                    ),
                    **observation_metadata(message),
                )

            @self.vehicle.on_message("DISTANCE_SENSOR")
            def listener_range(vehicle, name, message):
                self.flight_state.update(
                    "range",
                    (
                        getattr(message, "current_distance", None),
                        getattr(message, "orientation", None),
                        getattr(message, "covariance", None),
                    ),
                    **observation_metadata(message),
                )

            @self.vehicle.on_message("RC_CHANNELS")
            def listener_rc_channels(vehicle, name, message):
                pwm = getattr(message, f"chan{self.flight_state.rc_channel}_raw", None)
                metadata = observation_metadata(message)
                if isinstance(pwm, bool) or not isinstance(pwm, int):
                    if message_matches_source(message):
                        self.flight_state.invalidate_observation(
                            "rc_input",
                            **metadata,
                        )
                    return
                decoded_health = None
                if rc_health_decoder is not None and message_matches_source(message):
                    try:
                        candidate = rc_health_decoder(
                            message, self.flight_state.rc_channel, pwm
                        )
                    except Exception:
                        candidate = None
                    if isinstance(candidate, bool):
                        decoded_health = candidate
                self.flight_state.observe_rc_input(
                    channel=self.flight_state.rc_channel,
                    pwm=pwm,
                    signal_healthy=decoded_health,
                    **metadata,
                )

            if sys_status_observer is not None:

                @self.vehicle.on_message("SYS_STATUS")
                def listener_sys_status(vehicle, name, message):
                    sys_status_observer(message)

            for failsafe_message_name, failsafe_decoder in (
                configured_failsafe_decoders.items()
            ):
                if failsafe_message_name == "HEARTBEAT":
                    continue

                @self.vehicle.on_message(failsafe_message_name)
                def listener_failsafe(
                    vehicle,
                    name,
                    message,
                    decoder=failsafe_decoder,
                ):
                    if not message_matches_source(message):
                        return
                    metadata = observation_metadata(message)
                    try:
                        evidence = decoder(message)
                    except Exception:
                        evidence = None
                    if (
                        not isinstance(evidence, FailsafeEvidence)
                        or not isinstance(evidence.active, bool)
                        or not isinstance(evidence.reason, str)
                        or not evidence.reason
                    ):
                        self.flight_state.invalidate_observation(
                            "failsafe",
                            **metadata,
                        )
                        return
                    self.flight_state.observe_failsafe(
                        evidence.reason,
                        active=evidence.active,
                        **metadata,
                    )

    def close_startup_telemetry(self) -> None:
        command_transaction_lock = getattr(
            self, "_command_ack_transaction_lock", None
        )
        if command_transaction_lock is None:
            command_transaction_lock = threading.Lock()
            self._command_ack_transaction_lock = command_transaction_lock
        with self._telemetry_transaction_lock, command_transaction_lock:
            self._startup_telemetry_open = False

    def set_telemetry_interval(
        self,
        message_id: int,
        interval_us: int,
        *,
        timeout_s: float,
        poll_interval_s: float,
    ) -> None:
        """Issue only MAV_CMD_SET_MESSAGE_INTERVAL during bounded startup."""
        if self._startup_telemetry_open is not True:
            raise AuthorityLost("startup telemetry configuration is closed")
        self._run_telemetry_command(
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            message_id,
            interval_us,
            timeout_s=timeout_s,
            poll_interval_s=poll_interval_s,
        )

    def request_autopilot_version(
        self,
        *,
        timeout_s: float,
        poll_interval_s: float,
    ) -> None:
        """Request actual FC firmware metadata during bounded startup only."""
        if self._startup_telemetry_open is not True:
            raise AuthorityLost("startup telemetry configuration is closed")
        self._run_telemetry_command(
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
            0,
            timeout_s=timeout_s,
            poll_interval_s=poll_interval_s,
        )

    def request_home_position(
        self,
        *,
        timeout_s: float,
        poll_interval_s: float,
    ) -> None:
        """Request post-arm HOME_POSITION through the guarded narrow seam."""
        self.check_permission()
        boundary = self.flight_state.snapshot().home
        baseline = self._field_sequence(boundary)
        deadline = time.monotonic() + float(timeout_s)
        self._run_telemetry_command(
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION,
            0,
            timeout_s=timeout_s,
            poll_interval_s=poll_interval_s,
        )
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("HOME_POSITION request timed out")
        self._wait_until(
            deadline,
            lambda current: self._postcommand(current.home, baseline),
            "requested HOME_POSITION was not observed",
        )

    def _run_telemetry_command(
        self,
        command: int,
        message_id: int,
        interval_us: int,
        *,
        timeout_s: float,
        poll_interval_s: float,
    ) -> None:
        if (
            not isinstance(message_id, int)
            or isinstance(message_id, bool)
            or not 0 <= message_id <= 0xFFFFFF
        ):
            raise ValueError("telemetry message_id must be a 24-bit integer")
        if (
            not isinstance(interval_us, int)
            or isinstance(interval_us, bool)
            or interval_us < 0
            or (command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and interval_us == 0)
        ):
            raise ValueError("telemetry interval_us is invalid")
        if not _finite_number(timeout_s) or timeout_s <= 0:
            raise ValueError("telemetry ACK timeout must be finite and positive")
        if (
            not _finite_number(poll_interval_s)
            or poll_interval_s <= 0
            or poll_interval_s > timeout_s
        ):
            raise ValueError("telemetry ACK poll interval is invalid")
        tracker = self._command_ack_tracker
        if tracker is None:
            raise FlightOperationError("telemetry ACK tracking is unavailable")
        target = self.flight_controller_target
        encode = getattr(self.vehicle.message_factory, "command_long_encode", None)
        if not callable(encode):
            raise FlightOperationError("telemetry command encoder is unavailable")
        startup_privileged = (
            command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
            and interval_us > 0
        ) or (
            command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE
            and message_id == mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION
            and interval_us == 0
        )
        home_request = (
            command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE
            and message_id == mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION
            and interval_us == 0
        )
        if not startup_privileged and not home_request:
            raise ValueError("telemetry command/message combination is not allowlisted")
        command_transaction_lock = getattr(
            self, "_command_ack_transaction_lock", None
        )
        if command_transaction_lock is None:
            command_transaction_lock = threading.Lock()
            self._command_ack_transaction_lock = command_transaction_lock
        with self._telemetry_transaction_lock, command_transaction_lock:
            if startup_privileged and self._startup_telemetry_open is not True:
                raise AuthorityLost("startup telemetry configuration is closed")
            message = encode(
                target.system_id,
                target.component_id,
                command,
                0,
                float(message_id),
                float(interval_us),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            boundary = None

            def capture_ack_boundary(enqueue=None):
                nonlocal boundary
                if enqueue is None:
                    boundary = tracker.boundary()
                    return None
                boundary, result = tracker.enqueue(enqueue)
                return result

            if home_request:
                self._send_guarded(
                    lambda: self.vehicle.send_mavlink(message),
                    before_final=capture_ack_boundary,
                )
            else:
                transport_transaction = getattr(
                    self, "_transport_output_transaction", None
                )
                if not callable(transport_transaction):
                    raise FlightOperationError(
                        "guarded MAVLink transport is not installed"
                    )
                transport_transaction(
                    lambda: self.vehicle.send_mavlink(message),
                    capture_ack_boundary,
                )
            if boundary is None:
                raise FlightOperationError(
                    "telemetry command ACK boundary was not captured"
                )
            clock = time if home_request else wall_time
            deadline = clock.monotonic() + float(timeout_s)
            while True:
                if home_request:
                    self.check_permission()
                result = tracker.result_after(command=command, boundary=boundary)
                if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    return
                if result is not None and result != mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                    raise FlightOperationError(
                        f"telemetry command {command} was rejected with result {result}"
                    )
                remaining = deadline - clock.monotonic()
                if remaining <= 0:
                    raise TimeoutError("telemetry command ACK timed out")
                clock.sleep(min(float(poll_interval_s), remaining))

    def is_landed(self) -> bool:
        if self.flight_state is None:
            return False
        landed = self.flight_state.snapshot().landed_state
        return bool(
            landed is not None
            and landed.fresh
            and self._is_on_ground_value(landed.observation.value)
        )

    def check_permission(self) -> None:
        guard = getattr(self, "permission_guard", None)
        if guard is None or not callable(guard):
            raise AuthorityLost("flight output has no installed supervisor guard")
        guard_result = guard()
        if guard_result is not None:
            raise AuthorityLost("supervisor guard violated its success contract")
        state = getattr(self, "flight_state", None)
        if state is None or not state.ordinary_commands_permitted():
            raise AuthorityLost("companion flight authority is not current")

    def _command_permission(self):
        self.check_permission()
        permission = self.flight_state.command_permission()
        if permission is None:
            raise AuthorityLost("companion flight authority is not current")
        return permission

    def _check_command_boundary(self, permission) -> None:
        self.check_permission()
        if not self.flight_state.permission_is_current(permission):
            raise AuthorityLost("companion flight authority changed before output")

    def _send_guarded(
        self,
        output,
        *,
        validate_snapshot=None,
        before_final=None,
    ):
        return self._execute_guarded_output(
            output,
            validate_snapshot=validate_snapshot,
            before_final=before_final,
            require_transport_enqueue=True,
        )

    def _validate_guarded(self, validate_snapshot):
        return self._execute_guarded_output(
            lambda: None,
            validate_snapshot=validate_snapshot,
            require_transport_enqueue=False,
        )

    def _actuate_guarded(self, output, *, validate_snapshot=None):
        return self._execute_guarded_output(
            output,
            validate_snapshot=validate_snapshot,
            require_transport_enqueue=False,
        )

    def _require_vital_output_snapshot(self, snapshot) -> None:
        missing = [
            name
            for name in ("heartbeat", "mode", "rc_input", "failsafe")
            if (field := getattr(snapshot, name, None)) is None or not field.fresh
        ]
        rc_input = (
            None if snapshot.rc_input is None else snapshot.rc_input.observation.value
        )
        if not isinstance(rc_input, RCInput) or rc_input.healthy is not True:
            missing.append("rc_input")
        failsafe = (
            None if snapshot.failsafe is None else snapshot.failsafe.observation.value
        )
        failsafe_clear = (
            isinstance(failsafe, FailsafeEvidence) and failsafe.active is False
        ) or (
            isinstance(failsafe, tuple)
            and len(failsafe) == 2
            and failsafe[0] is False
        )
        if not failsafe_clear:
            missing.append("failsafe")
        if missing:
            source = self.flight_state.source
            try:
                self.flight_state.invalidate_observations(
                    tuple(dict.fromkeys(missing)),
                    source_system=source.system_id,
                    source_component=source.component_id,
                )
            except PermissionError as error:
                raise AuthorityLost(
                    "vital flight evidence is absent, stale, or unsafe"
                ) from error
            raise AuthorityLost("vital flight evidence is absent, stale, or unsafe")

    def _execute_guarded_output(
        self,
        output,
        *,
        validate_snapshot=None,
        before_final=None,
        require_transport_enqueue: bool,
    ):
        if before_final is not None:
            self.check_permission()
        permission = self._command_permission()
        dependency_transaction = getattr(self, "_dependency_output_transaction", None)
        supervisor_transaction = getattr(self, "_supervisor_output_transaction", None)
        transport_transaction = getattr(self, "_transport_output_transaction", None)
        if not callable(dependency_transaction) or not callable(supervisor_transaction):
            raise AuthorityLost("flight output transactions are not installed")
        if require_transport_enqueue and not callable(transport_transaction):
            raise AuthorityLost("guarded MAVLink transport is not installed")
        enqueue_boundary = []

        def validate_boundary(current):
            self._require_vital_output_snapshot(current)
            if validate_snapshot is not None:
                validate_snapshot(current)
            if not self.flight_state.permission_is_current(permission):
                raise PermissionError(
                    "companion flight authority changed at final output boundary"
                )

        def state_transaction():
            def enqueue_check(enqueue=None):
                def final_supervisor_check():
                    current = self.flight_state.snapshot()
                    validate_boundary(current)
                    enqueue_boundary.append(current)
                    if enqueue is None:
                        if before_final is not None:
                            before_final()
                        return None
                    if before_final is not None:
                        return before_final(enqueue)
                    return enqueue()

                return dependency_transaction(
                    lambda: supervisor_transaction(final_supervisor_check)
                )

            def transport_output():
                if require_transport_enqueue:
                    return transport_transaction(output, enqueue_check)
                return output()

            try:
                return self.flight_state.execute_command_output(
                    permission,
                    output_gate=self._supervisor_output_transaction,
                    validate_snapshot=validate_boundary,
                    output=transport_output,
                )
            except PermissionError as error:
                raise AuthorityLost(str(error)) from error

        state_boundary, _result = dependency_transaction(state_transaction)
        if require_transport_enqueue and not enqueue_boundary:
            raise RuntimeError("flight output did not reach its guarded enqueue boundary")
        return enqueue_boundary[-1] if enqueue_boundary else state_boundary

    def install_output_transactions(
        self,
        *,
        dependency_transaction: Callable[[Callable[[], object]], object],
        supervisor_transaction: Callable[[Callable[[], object]], object],
        transport_transaction: Callable[
            [Callable[[], object], Callable[[], None]], object
        ]
        | None = None,
    ) -> None:
        if not callable(dependency_transaction) or not callable(supervisor_transaction):
            raise TypeError("output transactions must be callable")
        selected_transport = (
            getattr(self, "_transport_output_transaction", None)
            if transport_transaction is None
            else transport_transaction
        )
        if not callable(selected_transport):
            raise TypeError("a guarded transport enqueue transaction is required")
        self._dependency_output_transaction = dependency_transaction
        self._supervisor_output_transaction = supervisor_transaction
        self._transport_output_transaction = selected_transport

    def install_startup_telemetry_verifier(
        self,
        verifier: Callable[[], None] | None,
        *,
        verified: bool = False,
    ) -> None:
        """Require one staged simulator telemetry proof before flight output."""
        if not isinstance(verified, bool):
            raise TypeError("startup telemetry verified state must be a Boolean")
        if verified:
            if verifier is not None:
                raise ValueError("verified startup telemetry cannot retain a verifier")
        elif not callable(verifier):
            raise TypeError("pending startup telemetry verifier must be callable")
        lock = getattr(self, "_startup_telemetry_verification_lock", None)
        if lock is None:
            lock = threading.Lock()
            self._startup_telemetry_verification_lock = lock
        with lock:
            if getattr(self, "_startup_telemetry_verification_state", "missing") != "missing":
                raise RuntimeError("startup telemetry verifier is already installed")
            self._startup_telemetry_verifier = verifier
            self._startup_telemetry_verification_state = (
                "verified" if verified else "pending"
            )

    def _verify_startup_telemetry_before_arm(self) -> None:
        lock = getattr(self, "_startup_telemetry_verification_lock", None)
        if lock is None:
            raise FlightOperationError("startup telemetry verification is not installed")
        with lock:
            state = self._startup_telemetry_verification_state
            if state == "verified":
                return
            if state == "missing":
                raise FlightOperationError(
                    "startup telemetry verification is not installed"
                )
            if state == "failed":
                raise FlightOperationError("startup telemetry verification failed")
            if getattr(self, "_guided_output_delivery_reported", False) is not True:
                raise FlightOperationError(
                    "startup telemetry verification requires guarded GUIDED delivery"
                )
            verifier = self._startup_telemetry_verifier
            if not callable(verifier):
                self._startup_telemetry_verification_state = "failed"
                raise FlightOperationError("startup telemetry verification failed")
            try:
                result = verifier()
                if result is not None:
                    raise TypeError("startup telemetry verifier must return None")
            except BaseException as error:
                self._startup_telemetry_verification_state = "failed"
                if isinstance(
                    error,
                    (TimeoutError, time.ClockError, MissionAbort, AuthorityLost),
                ):
                    raise
                raise FlightOperationError(
                    "startup telemetry verification failed"
                ) from error
            self._startup_telemetry_verification_state = "verified"

    def _require_startup_telemetry_verified(self) -> None:
        lock = getattr(self, "_startup_telemetry_verification_lock", None)
        if lock is None:
            raise FlightOperationError("startup telemetry is not verified")
        with lock:
            if self._startup_telemetry_verification_state != "verified":
                raise FlightOperationError("startup telemetry is not verified")

    @staticmethod
    def _field_sequence(field) -> int:
        return -1 if field is None else field.observation.sequence

    @staticmethod
    def _is_on_ground_value(value) -> bool:
        return (
            isinstance(value, int)
            and not isinstance(value, bool)
            and value == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
        )

    @staticmethod
    def _is_in_air_value(value) -> bool:
        return (
            isinstance(value, int)
            and not isinstance(value, bool)
            and value == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR
        )

    @staticmethod
    def _is_landing_value(value) -> bool:
        return (
            isinstance(value, int)
            and not isinstance(value, bool)
            and value == mavutil.mavlink.MAV_LANDED_STATE_LANDING
        )

    @staticmethod
    def _is_active_flight_value(value) -> bool:
        return (
            isinstance(value, int)
            and not isinstance(value, bool)
            and value
            in {
                mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
                mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF,
                mavutil.mavlink.MAV_LANDED_STATE_LANDING,
            }
        )

    @staticmethod
    def _postcommand(field, baseline_sequence: int) -> bool:
        return bool(
            field is not None
            and field.fresh
            and field.observation.sequence > baseline_sequence
        )

    def _deadline(self, timeout: float) -> float:
        self._validate_timeout(timeout)
        return time.monotonic() + timeout

    def _wait_until(self, deadline: float, predicate, failure: str) -> None:
        while time.monotonic() < deadline:
            self.check_permission()
            if predicate(self.flight_state.snapshot()):
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(0.1, remaining))
        raise FlightOperationError(failure)

    def _wait(self, timeout: float, predicate, failure: str) -> None:
        self._wait_until(self._deadline(timeout), predicate, failure)

    def _send_acknowledged(
        self,
        output,
        *,
        command: int,
        deadline: float,
        validate_snapshot=None,
        validate_ack_snapshot=None,
    ):
        tracker = getattr(self, "_command_ack_tracker", None)
        if tracker is None:
            raise AuthorityLost("command acknowledgement tracking is unavailable")
        transaction_lock = getattr(self, "_command_ack_transaction_lock", None)
        if transaction_lock is None:
            transaction_lock = threading.Lock()
            self._command_ack_transaction_lock = transaction_lock
        with transaction_lock:
            ack_boundary = None

            def capture_ack_boundary(enqueue=None):
                nonlocal ack_boundary
                if enqueue is None:
                    ack_boundary = tracker.boundary()
                    return None
                ack_boundary, result = tracker.enqueue(enqueue)
                return result

            state_boundary = self._send_guarded(
                output,
                validate_snapshot=validate_snapshot,
                before_final=capture_ack_boundary,
            )
            if ack_boundary is None:
                raise FlightOperationError(
                    "command transaction boundary was not captured"
                )

            def acknowledged(current):
                if validate_ack_snapshot is not None:
                    validate_ack_snapshot(current)
                result = tracker.result_after(command=command, boundary=ack_boundary)
                if result is None or result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                    return False
                if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    raise FlightOperationError(
                        f"command {command} acknowledgement rejected with result {result}"
                    )
                return True

            self._wait_until(
                deadline,
                acknowledged,
                f"command {command} acknowledgement was not received",
            )
            return state_boundary

    def _send_mission_acknowledged(
        self,
        output,
        *,
        deadline: float,
        validate_snapshot=None,
    ):
        tracker = getattr(self, "_mission_ack_tracker", None)
        transaction_lock = getattr(self, "_mission_ack_transaction_lock", None)
        if tracker is None or transaction_lock is None:
            raise AuthorityLost("mission acknowledgement tracking is unavailable")
        with transaction_lock:
            if getattr(self, "_mission_ack_failed", False):
                raise MissionAckError(
                    "a prior MISSION_ACK transaction failed; further GUIDED "
                    "mission output is forbidden"
                )
            ack_boundary = None

            def capture_ack_boundary(enqueue=None):
                nonlocal ack_boundary
                if enqueue is None:
                    ack_boundary = tracker.boundary()
                    return None

                def record_boundary(boundary):
                    nonlocal ack_boundary
                    ack_boundary = boundary

                ack_boundary, result = tracker.enqueue(
                    enqueue, boundary_callback=record_boundary
                )
                return result

            try:
                state_boundary = self._send_guarded(
                    output,
                    validate_snapshot=validate_snapshot,
                    before_final=capture_ack_boundary,
                )
                if ack_boundary is None:
                    raise FlightOperationError(
                        "mission acknowledgement boundary was not captured"
                    )
                while True:
                    self.check_permission()
                    result = tracker.result_after(boundary=ack_boundary)
                    if result == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                        return state_boundary
                    if result is not None:
                        raise MissionAckError(
                            f"MISSION_ACK rejected with result {result}"
                        )
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise MissionAckTimeout("MISSION_ACK was not received")
                    time.sleep(min(0.1, remaining))
            except BaseException:
                if ack_boundary is not None:
                    self._mission_ack_failed = True
                raise

    def _prepare_command_long(self, command: int, *parameters: float):
        if len(parameters) != 7 or not all(_finite_number(value) for value in parameters):
            raise FlightOperationError("command-long parameters must be seven finite values")
        target = self.flight_controller_target
        return self.vehicle.message_factory.command_long_encode(
            target.system_id,
            target.component_id,
            command,
            0,
            *(float(value) for value in parameters),
        )

    def _prepared_mode_command(self, mode: str):
        mapping = getattr(self.vehicle, "_mode_mapping", None)
        if not isinstance(mapping, Mapping) or mode not in mapping:
            raise FlightOperationError(f"flight mode {mode!r} is unavailable")
        return self._prepare_command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mapping[mode],
            0,
            0,
            0,
            0,
            0,
        )

    @staticmethod
    def _validate_timeout(timeout: float) -> None:
        if not _finite_number(timeout) or timeout <= 0:
            raise ValueError("timeout must be finite and positive")

    def _require_fresh(self, snapshot, *names):
        missing = [
            name
            for name in names
            if (field := getattr(snapshot, name)) is None or not field.fresh
        ]
        if missing:
            raise FlightOperationError(
                "missing or stale flight evidence: " + ", ".join(missing)
            )

    @staticmethod
    def _mode_name(snapshot):
        if snapshot.mode is None or not snapshot.mode.fresh:
            return None
        return str(snapshot.mode.observation.value).upper()

    @staticmethod
    def _location_value(field):
        if field is None:
            return None
        value = field.observation.value
        if not isinstance(value, tuple) or len(value) != 4:
            return None
        if not all(_finite_number(item) for item in value):
            return None
        return tuple(float(item) for item in value)

    def _set_mode(self, mode: str, timeout: float, *, validate_snapshot=None) -> int:
        deadline = self._deadline(timeout)
        self.check_permission()
        snapshot = self.flight_state.snapshot()
        self._require_fresh(snapshot, "heartbeat", "mode")
        message = self._prepared_mode_command(mode)
        expected_token = self.flight_state.register_expected_mode(mode)

        def validate_mode_boundary(candidate):
            if validate_snapshot is not None:
                validate_snapshot(candidate)
            if mode != "GUIDED" or getattr(
                self, "_guided_output_delivery_reported", False
            ):
                return
            self._require_fresh(candidate, "armed", "landed_state")
            armed = candidate.armed.observation.value
            landed = candidate.landed_state.observation.value
            if (
                armed is not False
                or not isinstance(landed, int)
                or isinstance(landed, bool)
                or landed != mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
            ):
                raise FlightOperationError(
                    "first GUIDED delivery requires grounded and disarmed state"
                )

        def enqueue() -> None:
            self.vehicle.send_mavlink(message)
            callback = getattr(self, "guided_output_delivery_callback", None)
            if (
                mode == "GUIDED"
                and not getattr(self, "_guided_output_delivery_reported", False)
            ):
                self._guided_output_delivery_reported = True
                if callback is not None:
                    callback()

        try:
            boundary = self._send_acknowledged(
                enqueue,
                command=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                deadline=deadline,
                validate_snapshot=validate_mode_boundary,
            )
            baseline = self._field_sequence(boundary.mode)
            self._wait_until(
                deadline,
                lambda current: self._postcommand(current.mode, baseline)
                and self._mode_name(current) == mode,
                f"{mode} mode was not confirmed",
            )
        except Exception:
            self.flight_state.cancel_expected_mode(expected_token)
            raise
        return 0

    def force_arm_takeoff(self, alt):
        if not _finite_number(alt) or alt <= 0:
            raise FlightOperationError("takeoff altitude must be finite and positive")
        snapshot = self.flight_state.snapshot() if self.flight_state is not None else None
        self.check_permission()
        if snapshot is None:
            raise FlightOperationError("preflight observations are unavailable")
        self._require_fresh(snapshot, "heartbeat", "location", "landed_state", "armed")
        landed = snapshot.landed_state.observation.value
        if not self._is_on_ground_value(landed):
            raise FlightOperationError("takeoff requested while not confirmed on ground")
        if snapshot.armed.observation.value is not False:
            raise FlightOperationError("takeoff requested while already armed")
        if self.mission_home is None:
            raise FlightOperationError("takeoff requires a pinned mission home")
        if not all(
            _finite_number(value) and value > 0
            for value in (
                getattr(self, "fc_home_position_tolerance_m", None),
                getattr(self, "fc_home_altitude_tolerance_m", None),
            )
        ):
            raise FlightOperationError("FC home consistency tolerances are not configured")
        self.set_guided_mode()
        self.arm()
        self.takeoff(alt)

    def rtl(self):
        return self._set_mode("RTL", TIMEOUT_MODE)

    @property
    def mission_home(self) -> MissionHome | None:
        return getattr(self, "_mission_home", None)

    def set_mission_home(self, home: MissionHome) -> None:
        """Pin the attempt datum from one approved, fresh launch observation."""
        if not isinstance(home, MissionHome) or not all(
            _finite_number(value) for value in (home.lat, home.lon, home.amsl_m)
        ):
            raise FlightOperationError("mission home must contain finite values")
        if not -90.0 <= home.lat <= 90.0 or not -180.0 <= home.lon <= 180.0:
            raise FlightOperationError("mission home latitude or longitude is invalid")
        existing = self.mission_home
        if existing is not None:
            if existing == home:
                return
            raise FlightOperationError("mission home cannot change during an attempt")
        checker = getattr(self, "mission_home_check", None)
        if checker is None or not callable(checker):
            raise FlightOperationError("mission home has no operating-area approval")
        self.check_permission()
        snapshot = self.flight_state.snapshot()
        self._require_fresh(snapshot, "location", "landed_state", "armed")
        if not self._is_on_ground_value(snapshot.landed_state.observation.value):
            raise FlightOperationError("mission home requires exact ON_GROUND evidence")
        if snapshot.armed.observation.value is not False:
            raise FlightOperationError("mission home requires disarmed evidence")
        location = self._location_value(snapshot.location)
        if location is None:
            raise FlightOperationError("mission home launch location is invalid")
        lat_raw, lon_raw, amsl_mm, _relative_mm = location
        observed = MissionHome(lat_raw / 1e7, lon_raw / 1e7, amsl_mm / 1000.0)
        if observed != home:
            raise FlightOperationError("mission home does not match the launch observation")
        if checker(home) is not None:
            raise FlightOperationError("operating-area check must return None")
        self.check_permission()
        confirmed = self.flight_state.snapshot()
        self._require_fresh(confirmed, "location", "landed_state", "armed")
        if not self._is_on_ground_value(confirmed.landed_state.observation.value):
            raise FlightOperationError("mission home requires exact ON_GROUND evidence")
        if confirmed.armed.observation.value is not False:
            raise FlightOperationError("mission home requires disarmed evidence")
        confirmed_location = self._location_value(confirmed.location)
        if confirmed_location is None or MissionHome(
            confirmed_location[0] / 1e7,
            confirmed_location[1] / 1e7,
            confirmed_location[2] / 1000.0,
        ) != home:
            raise FlightOperationError("mission home launch observation changed")
        self._mission_home = home
        print(
            f"[*] Mission home pinned: {home.lat:.7f}, {home.lon:.7f}, "
            f"{home.amsl_m:.3f} m AMSL"
        )

    def goto_waypoint(
        self,
        coord: GPSCoord,
        position_tol: Optional[float] = None,
        timeout: float = TIMEOUT_MOVE,
    ) -> int:
        return self._goto_waypoint(coord, position_tol, timeout)

    def goto_recovery_waypoint(
        self,
        coord: GPSCoord,
        *,
        approve_target_amsl: Callable[[float], None],
        position_tol: Optional[float] = None,
        timeout: float = TIMEOUT_MOVE,
    ) -> int:
        if not callable(approve_target_amsl):
            raise TypeError("approve_target_amsl must be callable")
        return self._goto_waypoint(
            coord,
            position_tol,
            timeout,
            recovery_target_approval=approve_target_amsl,
        )

    def send_guided_waypoint(self, coord: GPSCoord) -> int:
        """Send one guarded, nonblocking earth-fixed GUIDED hold waypoint."""
        home = self.mission_home
        if home is None:
            raise FlightOperationError("GUIDED hold requires a pinned mission home")
        if not isinstance(coord, GPSCoord) or not all(
            _finite_number(value) for value in (coord.lat, coord.long, coord.alt)
        ):
            raise FlightOperationError("GUIDED hold waypoint is malformed")
        if not -90.0 <= coord.lat <= 90.0 or not -180.0 <= coord.long <= 180.0:
            raise FlightOperationError("GUIDED hold waypoint is outside coordinate bounds")
        target_amsl = home.amsl_m + float(coord.alt)

        def validate(current):
            self._require_fresh(current, "mode", "armed", "landed_state")
            if (
                self._mode_name(current) != "GUIDED"
                or current.armed.observation.value is not True
                or not self._is_in_air_value(current.landed_state.observation.value)
            ):
                raise FlightOperationError(
                    "GUIDED hold requires confirmed armed GUIDED flight"
                )

        self._send_guarded(
            lambda: _send_guided_waypoint(
                self.vehicle,
                coord.lat,
                coord.long,
                target_amsl,
                self.flight_controller_target.system_id,
                self.flight_controller_target.component_id,
                lambda: None,
            ),
            validate_snapshot=validate,
        )
        return 0

    def require_precision_landing_profile(self) -> bool:
        """Return true only when ArduPilot exposes the approved landing profile."""
        parameters = getattr(self.vehicle, "parameters", None)
        if parameters is None:
            return False
        for name, expected in PRECISION_LANDING_PARAMETERS.items():
            try:
                actual = parameters.get(name)
            except (AttributeError, KeyError, TypeError, ValueError):
                return False
            if not _finite_number(actual):
                return False
            numeric = float(actual)
            if isinstance(expected, int):
                matches = numeric.is_integer() and int(numeric) == expected
            else:
                matches = math.isclose(
                    numeric,
                    expected,
                    rel_tol=0.0,
                    abs_tol=1e-3,
                )
            if not matches:
                return False
        return True

    def _goto_waypoint(
        self,
        coord: GPSCoord,
        position_tol: Optional[float],
        timeout: float,
        *,
        recovery_target_approval: Callable[[float], None] | None = None,
    ) -> int:
        deadline = self._deadline(timeout)
        position_tol = POS_TOL if position_tol is None else position_tol
        target_alt = coord.alt if coord.alt is not None else self.cruise_alt
        self.check_permission()
        home = self.mission_home
        if home is None:
            raise FlightOperationError("waypoint requires a pinned mission home")
        target_amsl = home.amsl_m + float(target_alt)
        if not all(
            _finite_number(value)
            for value in (coord.lat, coord.long, target_alt, position_tol)
        ):
            raise FlightOperationError("waypoint contains non-finite coordinates")
        snapshot = self.flight_state.snapshot()
        self._require_fresh(snapshot, "mode", "armed", "location", "landed_state")
        if self._mode_name(snapshot) != "GUIDED" or snapshot.armed.observation.value is not True:
            raise FlightOperationError("waypoint requires confirmed armed GUIDED flight")
        if not self._is_in_air_value(snapshot.landed_state.observation.value):
            raise FlightOperationError("waypoint requires confirmed airborne state")
        speed_message = self._prepare_command_long(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            1,
            GROUND_SPEED,
            -1,
            0,
            0,
            0,
            0,
        )
        self._send_acknowledged(
            lambda: self.vehicle.send_mavlink(speed_message),
            command=mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            deadline=deadline,
        )
        packed_target_amsl: float | None = None

        def send_waypoint():
            nonlocal packed_target_amsl
            packed_target_amsl = target_amsl
            _send_guided_waypoint(
                self.vehicle,
                coord.lat,
                coord.long,
                packed_target_amsl,
                self.flight_controller_target.system_id,
                self.flight_controller_target.component_id,
                lambda: None,
            )

        def validate_waypoint_boundary(candidate):
            nonlocal target_amsl
            self._require_fresh(
                candidate, "mode", "armed", "location", "landed_state"
            )
            if (
                self._mode_name(candidate) != "GUIDED"
                or candidate.armed.observation.value is not True
                or not self._is_in_air_value(candidate.landed_state.observation.value)
            ):
                raise FlightOperationError(
                    "waypoint requires confirmed airborne GUIDED state"
                )
            if recovery_target_approval is not None:
                current = self._location_value(candidate.location)
                if current is None:
                    raise FlightOperationError(
                        "recovery waypoint location is invalid"
                    )
                current_amsl = current[2] / 1000.0
                if current_amsl > target_amsl:
                    if packed_target_amsl is not None:
                        raise FlightOperationError(
                            "recovery altitude changed after packet preparation"
                        )
                    if recovery_target_approval(current_amsl) is not None:
                        raise FlightOperationError(
                            "recovery target approval must return None"
                        )
                    target_amsl = current_amsl

        transport_boundary = self._send_mission_acknowledged(
            send_waypoint,
            deadline=deadline,
            validate_snapshot=validate_waypoint_boundary,
        )
        baseline = self._field_sequence(transport_boundary.location)

        def arrived(current):
            field = current.location
            if not self._postcommand(field, baseline):
                return False
            location = self._location_value(field)
            if location is None:
                return False
            lat_raw, lon_raw, amsl_mm, _relative_mm = location
            position = GPSCoord(float(lat_raw) / 1e7, float(lon_raw) / 1e7, 0)
            target = GPSCoord(float(coord.lat), float(coord.long), 0)
            return (
                horiz_distance_m(position, target) <= position_tol
                and abs(float(amsl_mm) / 1000.0 - target_amsl) <= ALT_TOL
            )

        self._wait_until(deadline, arrived, "waypoint arrival was not confirmed")
        return 0

    def move_relative_ned(self, dir: NEDMeters) -> int:
        raise NotImplementedError("NED movement has no verified operation contract")

    def set_land_mode(self, timeout: float = TIMEOUT_MODE):
        return self._set_mode("LAND", timeout)

    def set_precision_land_mode(self):
        """
        Commands the drone to land at its current location and enforces
        Precision Landing mode.
        """
        raise NotImplementedError(
            "precision NAV_LAND operation contract is not implemented"
        )

    def set_guided_mode(self, timeout: float = TIMEOUT_MODE):
        return self._set_mode("GUIDED", timeout)

    def land_send_landing_target(self, dir: RelPosComplete) -> int:
        """
        Sends the MAVLink 1 LANDING_TARGET message to ArduPilot.
        Forces the legacy 8-parameter format (relies on angles + distance).
        Requires ArduPilot to be in LAND mode.
        """
        if not all(_finite_number(value) for value in (dir.x, dir.y, dir.z)):
            raise FlightOperationError("landing target must be finite")
        self.check_permission()
        snapshot = self.flight_state.snapshot()
        self._require_fresh(snapshot, "mode")
        if self._mode_name(snapshot) != "LAND":
            raise FlightOperationError("landing target requires confirmed LAND mode")
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

        msg = self.vehicle.message_factory.landing_target_encode(
            0,
            0,
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            angle_x,
            angle_y,
            target_distance,
            0.0,
            0.0,
        )

        def send():
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()

        self._send_guarded(send)

        return 0

    def guide_move_relative_frame(
        self, dir: RelPosComplete, timeout: float = TIMEOUT_MOVE
    ) -> int:
        """Send a MAVLink SET_POSITION_TARGET_LOCAL_NED message in body FRD frame
        using the provided relative position (meters, vehicle body frame).
        """
        self._validate_timeout(timeout)
        deadline = time.monotonic() + float(timeout)
        if not all(_finite_number(value) for value in (dir.x, dir.y, dir.z)):
            raise FlightOperationError("relative offset must be finite")
        self.check_permission()
        snapshot = self.flight_state.snapshot()
        self._require_fresh(
            snapshot, "mode", "armed", "location", "attitude", "landed_state"
        )
        if self._mode_name(snapshot) != "GUIDED" or snapshot.armed.observation.value is not True:
            raise FlightOperationError("relative movement requires confirmed armed GUIDED flight")
        if not self._is_in_air_value(snapshot.landed_state.observation.value):
            raise FlightOperationError("relative movement requires confirmed airborne flight")
        location = self._location_value(snapshot.location)
        if location is None:
            raise FlightOperationError("relative movement location is invalid")
        home = self.mission_home
        if home is None:
            raise FlightOperationError("relative movement requires a pinned mission home")
        lat_raw, lon_raw, amsl_mm, _relative_mm = location
        _roll, _pitch, yaw, *_rates = snapshot.attitude.observation.value
        if not all(_finite_number(value) for value in (lat_raw, lon_raw, amsl_mm, yaw)):
            raise FlightOperationError("relative movement observations are invalid")
        forward, right, down = float(dir.x), float(dir.y), float(dir.z)
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            self.flight_controller_target.system_id,
            self.flight_controller_target.component_id,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0x0DF8,
            forward,
            right,
            down,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

        def send():
            self.vehicle.send_mavlink(msg)

        target = None

        def validate_boundary(boundary):
            nonlocal target
            self._require_fresh(
                boundary, "mode", "armed", "location", "attitude", "landed_state"
            )
            if (
                self._mode_name(boundary) != "GUIDED"
                or boundary.armed.observation.value is not True
                or not self._is_in_air_value(boundary.landed_state.observation.value)
            ):
                raise FlightOperationError(
                    "relative movement requires confirmed armed GUIDED airborne flight"
                )
            current = self._location_value(boundary.location)
            attitude = boundary.attitude.observation.value
            if current is None or not isinstance(attitude, tuple) or len(attitude) < 3:
                raise FlightOperationError("relative movement observations are invalid")
            current_lat, current_lon, current_amsl, _current_relative = current
            current_yaw = attitude[2]
            if not all(
                _finite_number(value)
                for value in (current_lat, current_lon, current_amsl, current_yaw)
            ):
                raise FlightOperationError("relative movement observations are invalid")
            start = GPSCoord(
                float(current_lat) / 1e7,
                float(current_lon) / 1e7,
                float(current_amsl) / 1000.0 - home.amsl_m,
            )
            north = forward * math.cos(float(current_yaw)) - right * math.sin(float(current_yaw))
            east = forward * math.sin(float(current_yaw)) + right * math.cos(float(current_yaw))
            target = self.get_location_metres(start, north, east)
            target.alt = start.alt - down

        boundary = self._send_guarded(send, validate_snapshot=validate_boundary)
        if target is None:
            raise FlightOperationError("relative movement target was not captured")
        baseline = self._field_sequence(boundary.location)

        def arrived(current):
            field = current.location
            if not self._postcommand(field, baseline):
                return False
            location = self._location_value(field)
            if location is None:
                return False
            current_lat, current_lon, current_amsl, _current_relative = location
            position = GPSCoord(
                float(current_lat) / 1e7,
                float(current_lon) / 1e7,
                float(current_amsl) / 1000.0 - home.amsl_m,
            )
            return (
                horiz_distance_m(position, target) <= POS_TOL
                and abs(position.alt - target.alt) <= ALT_TOL
            )

        self._wait_until(deadline, arrived, "relative movement was not confirmed")

        return 0

    def translate_relative(self, dir: RelativePosition) -> NEDMeters:
        raise NotImplementedError("relative translation is not implemented")

    def simple_land(
        self, timeout: float = TIMEOUT_LAND, mode_timeout: float = TIMEOUT_MODE
    ) -> int:
        """Select LAND and require new touchdown or safe auto-disarm evidence."""
        self._validate_timeout(timeout)
        self._validate_timeout(mode_timeout)
        self.check_permission()
        before = self.flight_state.snapshot()
        self._require_fresh(before, "mode", "armed", "landed_state")
        landed_before = before.landed_state.observation.value
        if self._is_on_ground_value(landed_before):
            if before.armed.observation.value is False:
                return 0
            raise FlightOperationError(
                "touchdown was not attributable to this landing operation"
            )
        if self._is_landing_value(landed_before) or self._mode_name(before) == "LAND":
            return self.confirm_landing(timeout=timeout)

        interrupted = [False]

        def require_no_landing_transition(boundary):
            self._require_fresh(boundary, "mode", "armed", "landed_state")
            landed_now = boundary.landed_state.observation.value
            if (
                self._mode_name(boundary) == "LAND"
                or self._is_on_ground_value(landed_now)
                or self._is_landing_value(landed_now)
            ):
                interrupted[0] = True
                raise FlightOperationError(
                    "landing state changed before LAND mode output"
                )

        try:
            self._set_mode(
                "LAND",
                mode_timeout,
                validate_snapshot=require_no_landing_transition,
            )
        except FlightOperationError:
            if not interrupted[0]:
                raise
            return self.confirm_landing(timeout=timeout)
        landing_transition = self.flight_state.snapshot()
        landing_start_sequence = max(
            self._field_sequence(landing_transition.landed_state),
            self._field_sequence(landing_transition.armed),
        )
        armed_at_transition = (
            landing_transition.armed is not None
            and landing_transition.armed.fresh
            and landing_transition.armed.observation.value is True
        )

        def touchdown(current):
            landed = current.landed_state
            armed = current.armed
            explicit_ground = (
                self._postcommand(landed, landing_start_sequence)
                and self._is_on_ground_value(landed.observation.value)
            )
            if explicit_ground:
                return True
            contradictory_flight = (
                landed is not None
                and landed.fresh
                and self._is_active_flight_value(landed.observation.value)
            )
            auto_disarmed = (
                armed_at_transition
                and self._postcommand(armed, landing_start_sequence)
                and armed.observation.value is False
            )
            return auto_disarmed and not contradictory_flight

        self._wait(timeout, touchdown, "touchdown was not confirmed")
        return 0

    def confirm_landing(self, timeout: float = TIMEOUT_LAND) -> int:
        """Confirm an accepted LAND descent without transmitting another mode."""
        self._validate_timeout(timeout)
        self.check_permission()
        before = self.flight_state.snapshot()
        self._require_fresh(before, "mode", "armed", "landed_state")
        landed_before = before.landed_state.observation.value
        if self._is_on_ground_value(landed_before):
            return 0
        if (
            self._mode_name(before) != "LAND"
            and not self._is_landing_value(landed_before)
        ):
            raise FlightOperationError("landing confirmation requires accepted LAND")
        baseline = max(
            self._field_sequence(before.landed_state),
            self._field_sequence(before.armed),
        )
        armed_before = before.armed.observation.value is True

        def touchdown(current):
            landed = current.landed_state
            armed = current.armed
            explicit_ground = (
                self._postcommand(landed, baseline)
                and self._is_on_ground_value(landed.observation.value)
            )
            if explicit_ground:
                return True
            contradictory = (
                landed is not None
                and landed.fresh
                and self._is_active_flight_value(landed.observation.value)
            )
            return bool(
                armed_before
                and self._postcommand(armed, baseline)
                and armed.observation.value is False
                and not contradictory
            )

        self._wait(timeout, touchdown, "touchdown was not confirmed")
        return 0

    def takeoff(self, alt: float, timeout: float = TIMEOUT_ASCENT) -> None:
        deadline = self._deadline(timeout)
        if not _finite_number(alt) or alt <= 0:
            raise FlightOperationError("takeoff altitude must be finite and positive")
        self.check_permission()
        self._require_startup_telemetry_verified()
        home = self.mission_home
        if home is None:
            raise FlightOperationError("takeoff requires a pinned mission home")
        arm_boundary = getattr(self, "_last_arm_boundary_sequence", None)
        if arm_boundary is None:
            raise FlightOperationError("takeoff requires a current post-arm FC home")
        self._wait_until(
            deadline,
            lambda current: current.home is not None
            and current.home.fresh
            and current.home.observation.sequence > arm_boundary,
            "fresh post-arm FC home was not observed",
        )
        snapshot = self.flight_state.snapshot()
        self._require_fresh(
            snapshot, "mode", "armed", "landed_state", "location", "home"
        )
        if self._mode_name(snapshot) != "GUIDED" or snapshot.armed.observation.value is not True:
            raise FlightOperationError("takeoff requires confirmed armed GUIDED state")
        landed = snapshot.landed_state.observation.value
        if not self._is_on_ground_value(landed):
            raise FlightOperationError("takeoff requested while already airborne")
        start_location = self._location_value(snapshot.location)
        if start_location is None:
            raise FlightOperationError("takeoff location is invalid")
        start_alt = start_location[2] / 1000.0
        if float(alt) <= start_alt - home.amsl_m:
            raise FlightOperationError(
                "takeoff altitude must be above current relative altitude"
            )

        fc_target_alt = None
        bound_fc_home = None

        def validate_takeoff_boundary(boundary):
            self._require_startup_telemetry_verified()
            nonlocal start_alt, fc_target_alt, bound_fc_home
            self._require_fresh(
                boundary, "mode", "armed", "landed_state", "location", "home"
            )
            if (
                self._mode_name(boundary) != "GUIDED"
                or boundary.armed.observation.value is not True
            ):
                raise FlightOperationError(
                    "takeoff requires confirmed armed GUIDED state"
                )
            if not self._is_on_ground_value(
                boundary.landed_state.observation.value
            ):
                raise FlightOperationError(
                    "takeoff requested while already airborne"
                )
            boundary_location = self._location_value(boundary.location)
            if boundary_location is None:
                raise FlightOperationError("takeoff location is invalid")
            fc_home = boundary.home.observation.value
            if (
                boundary.home.observation.sequence <= arm_boundary
                or not isinstance(fc_home, tuple)
                or len(fc_home) != 3
                or not all(_finite_number(value) for value in fc_home)
            ):
                raise FlightOperationError("fresh post-arm FC home is invalid")
            horizontal_tolerance = getattr(
                self, "fc_home_position_tolerance_m", None
            )
            altitude_tolerance = getattr(
                self, "fc_home_altitude_tolerance_m", None
            )
            if not all(
                _finite_number(value) and value > 0
                for value in (horizontal_tolerance, altitude_tolerance)
            ):
                raise FlightOperationError("FC home consistency tolerances are not configured")
            fc_home_coord = GPSCoord(fc_home[0] / 1e7, fc_home[1] / 1e7, 0.0)
            current_coord = GPSCoord(
                boundary_location[0] / 1e7, boundary_location[1] / 1e7, 0.0
            )
            if horiz_distance_m(fc_home_coord, current_coord) > horizontal_tolerance:
                raise FlightOperationError("FC home position is inconsistent")
            fc_home_amsl = fc_home[2] / 1000.0
            current_amsl = boundary_location[2] / 1000.0
            if abs(current_amsl - fc_home_amsl) > altitude_tolerance:
                raise FlightOperationError("FC home altitude is inconsistent")
            candidate_target_alt = home.amsl_m + float(alt) - fc_home_amsl
            candidate_binding = (
                boundary.home.observation.sequence,
                fc_home,
                candidate_target_alt,
            )
            if bound_fc_home is not None and candidate_binding != bound_fc_home:
                raise FlightOperationError(
                    "FC home changed after the takeoff packet was prepared"
                )
            if float(alt) <= current_amsl - home.amsl_m:
                raise FlightOperationError(
                    "takeoff altitude must be above current original-home altitude"
                )
            if candidate_target_alt <= 0:
                raise FlightOperationError("translated FC takeoff altitude is invalid")
            if bound_fc_home is None:
                start_alt = current_amsl
                fc_target_alt = candidate_target_alt
                bound_fc_home = candidate_binding

        def send_takeoff():
            if fc_target_alt is None:
                raise FlightOperationError("takeoff datum was not bound")
            takeoff_message = self._prepare_command_long(
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0,
                0,
                0,
                0,
                0,
                fc_target_alt,
            )
            return self.vehicle.send_mavlink(takeoff_message)

        boundary = self._send_acknowledged(
            send_takeoff,
            command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            deadline=deadline,
            validate_snapshot=validate_takeoff_boundary,
        )
        self._wait_until(
            deadline,
            lambda current: self._ascent_reached(
                current,
                boundary,
                home.amsl_m + float(alt),
                start_alt,
                require_liftoff=True,
            ),
            "ascent was not confirmed",
        )

    def _ascent_reached(
        self,
        snapshot,
        boundary,
        target_alt: float,
        start_alt: float,
        *,
        require_liftoff: bool,
    ) -> bool:
        field = snapshot.location
        if not self._postcommand(field, self._field_sequence(boundary.location)):
            return False
        location = self._location_value(field)
        if location is None:
            return False
        home = self.mission_home
        if home is None:
            return False
        current_alt = location[2] / 1000.0
        landed = snapshot.landed_state
        armed = snapshot.armed
        if (
            landed is None
            or not landed.fresh
            or armed is None
            or not armed.fresh
            or armed.observation.value is not True
            or not self._is_in_air_value(landed.observation.value)
        ):
            return False
        if require_liftoff and not self._postcommand(
            landed, self._field_sequence(boundary.landed_state)
        ):
            return False
        return (
            current_alt > start_alt
            and abs(current_alt - target_alt) <= max(ALT_TOL, 0.9)
        )

    def arm(self, timeout: float = TIMEOUT_ARM) -> int:
        deadline = self._deadline(timeout)
        self.check_permission()
        self._verify_startup_telemetry_before_arm()
        self._require_startup_telemetry_verified()
        snapshot = self.flight_state.snapshot()
        self._require_fresh(snapshot, "heartbeat", "mode", "armed", "landed_state", "location")
        if self._mode_name(snapshot) != "GUIDED":
            raise FlightOperationError("arming requires confirmed GUIDED mode")
        landed = snapshot.landed_state.observation.value
        if not self._is_on_ground_value(landed):
            raise FlightOperationError("arming requires confirmed on-ground state")
        if snapshot.armed.observation.value is not False:
            raise FlightOperationError("vehicle is already armed")
        if getattr(self.vehicle, "is_armable", False) is not True:
            raise FlightOperationError("flight controller pre-arm checks are not satisfied")
        self._last_arm_boundary_sequence = None

        def validate_arm_boundary(boundary):
            self._require_startup_telemetry_verified()
            self._require_fresh(
                boundary,
                "heartbeat",
                "mode",
                "armed",
                "landed_state",
                "location",
            )
            if self._mode_name(boundary) != "GUIDED":
                raise FlightOperationError("arming requires confirmed GUIDED mode")
            if not self._is_on_ground_value(boundary.landed_state.observation.value):
                raise FlightOperationError("arming requires confirmed on-ground state")
            if boundary.armed.observation.value is not False:
                raise FlightOperationError("vehicle is already armed")
            if getattr(self.vehicle, "is_armable", False) is not True:
                raise FlightOperationError(
                    "flight controller pre-arm checks are not satisfied"
                )

        arm_message = self._prepare_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        boundary = self._send_acknowledged(
            lambda: self.vehicle.send_mavlink(arm_message),
            command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            deadline=deadline,
            validate_snapshot=validate_arm_boundary,
        )
        baseline = self._field_sequence(boundary.armed)
        self._wait_until(
            deadline,
            lambda current: self._postcommand(current.armed, baseline)
            and current.armed.observation.value is True,
            "arm was not confirmed",
        )
        confirmed = self.flight_state.snapshot()
        if not (
            confirmed.armed is not None
            and confirmed.armed.fresh
            and confirmed.armed.observation.value is True
            and confirmed.armed.observation.sequence > baseline
        ):
            raise FlightOperationError("arm confirmation became invalid")
        self._last_arm_boundary_sequence = confirmed.armed.observation.sequence
        home_request_timeout_s = getattr(self, "home_request_timeout_s", None)
        if home_request_timeout_s is not None:
            self.request_home_position(
                timeout_s=home_request_timeout_s,
                poll_interval_s=self.telemetry_poll_interval_s,
            )
        return 0

    def disarm(self, timeout: float = TIMEOUT_DISARM) -> int:
        deadline = self._deadline(timeout)
        self.check_permission()
        snapshot = self.flight_state.snapshot()
        self._require_fresh(snapshot, "landed_state", "armed")
        landed = snapshot.landed_state.observation.value
        if not self._is_on_ground_value(landed):
            raise FlightOperationError("disarm requires current touchdown evidence")
        if snapshot.armed.observation.value is False:
            self._last_arm_boundary_sequence = None
            return 0
        def require_ground(boundary):
            landed_now = boundary.landed_state
            if (
                landed_now is None
                or not landed_now.fresh
                or not isinstance(landed_now.observation.value, int)
                or isinstance(landed_now.observation.value, bool)
                or landed_now.observation.value
                != mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
            ):
                raise FlightOperationError(
                    "disarm requires current touchdown evidence"
                )

        disarm_message = self._prepare_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        boundary = self._send_acknowledged(
            lambda: self.vehicle.send_mavlink(disarm_message),
            command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            deadline=deadline,
            validate_snapshot=require_ground,
            validate_ack_snapshot=require_ground,
        )
        baseline = self._field_sequence(boundary.armed)

        def disarmed_on_ground(current):
            landed_now = current.landed_state
            if (
                landed_now is None
                or not landed_now.fresh
                or not self._is_on_ground_value(landed_now.observation.value)
            ):
                raise FlightOperationError(
                    "disarm requires current touchdown evidence"
                )
            return (
                self._postcommand(current.armed, baseline)
                and current.armed.observation.value is False
            )

        self._wait_until(
            deadline,
            disarmed_on_ground,
            "disarm was not confirmed",
        )
        self._last_arm_boundary_sequence = None
        return 0

    def wait_for_arm(self) -> int:
        raise NotImplementedError("use the bounded arm operation")

    def get_current_gps(self) -> GPSCoord:
        home = self.mission_home
        if home is None:
            raise FlightOperationError("current GPS requires a pinned mission home")
        self.check_permission()
        snapshot = self.flight_state.snapshot()
        self._require_fresh(snapshot, "location")
        location = self._location_value(snapshot.location)
        if location is None:
            raise FlightOperationError("current GPS location is invalid")
        return GPSCoord(
            location[0] / 1e7,
            location[1] / 1e7,
            location[2] / 1000.0 - home.amsl_m,
        )

    def flight_snapshot(self):
        if self.flight_state is None:
            raise FlightOperationError("flight observations are unavailable")
        return self.flight_state.snapshot()

    def simple_takeoff(self, alt: int):
        return self.takeoff(alt)

    def climb(
        self, target_alt: float, timeout: float = TIMEOUT_ASCENT
    ) -> None:
        """Ascend to an original-home-relative altitude using an AMSL setpoint."""
        deadline = self._deadline(timeout)
        if not _finite_number(target_alt):
            raise FlightOperationError("climb altitude must be finite")
        self.check_permission()
        home = self.mission_home
        if home is None:
            raise FlightOperationError("climb requires a pinned mission home")
        target_amsl = home.amsl_m + float(target_alt)
        print(f"[*] Climbing to {target_alt:.2f} m above original H...")

        snapshot = self.flight_state.snapshot()
        self._require_fresh(snapshot, "mode", "armed", "location", "landed_state")
        if self._mode_name(snapshot) != "GUIDED" or snapshot.armed.observation.value is not True:
            raise FlightOperationError("climb requires confirmed armed GUIDED flight")
        landed = snapshot.landed_state.observation.value
        if not self._is_in_air_value(landed):
            raise FlightOperationError("climb requires positive airborne state")

        def validate_climb_boundary(boundary):
            self._require_fresh(
                boundary, "mode", "armed", "location", "landed_state"
            )
            if (
                self._mode_name(boundary) != "GUIDED"
                or boundary.armed.observation.value is not True
            ):
                raise FlightOperationError(
                    "climb requires confirmed armed GUIDED flight"
                )
            if (
                not self._is_in_air_value(
                    boundary.landed_state.observation.value
                )
            ):
                raise FlightOperationError("climb requires positive airborne state")
            location = self._location_value(boundary.location)
            if location is None:
                raise FlightOperationError("climb location is invalid")
            lat_raw, lon_raw, amsl_mm, _relative_mm = location
            current_amsl = amsl_mm / 1000.0
            if target_amsl <= current_amsl:
                raise FlightOperationError(
                    "climb target must be above current altitude"
                )
            return float(lat_raw) / 1e7, float(lon_raw) / 1e7, current_amsl

        transport_boundary = None

        def send_climb():
            nonlocal transport_boundary, current_amsl
            transport_boundary = self.flight_state.snapshot()
            target_lat, target_lon, current_amsl = validate_climb_boundary(
                transport_boundary
            )
            self.vehicle._master.mav.mission_item_int_send(
                self.flight_controller_target.system_id,
                self.flight_controller_target.component_id,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                2,
                0,
                0,
                0,
                0,
                0,
                int(round(target_lat * 1e7)),
                int(round(target_lon * 1e7)),
                target_amsl,
            )

        current_amsl = None
        transport_boundary = self._send_mission_acknowledged(
            send_climb,
            deadline=deadline,
            validate_snapshot=validate_climb_boundary,
        )
        if transport_boundary is None or current_amsl is None:
            raise FlightOperationError("climb transport boundary was not captured")
        self._wait_until(
            deadline,
            lambda current: self._ascent_reached(
                current,
                transport_boundary,
                target_amsl,
                current_amsl,
                require_liftoff=False,
            ),
            "climb ascent was not confirmed",
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
        t_start = time.monotonic()
        stable_start_time = None

        while time.monotonic() - t_start < timeout:
            self.check_permission()
            snapshot = self.flight_state.snapshot()
            attitude = snapshot.attitude

            if (
                attitude is not None
                and attitude.fresh
                and isinstance(attitude.observation.value, tuple)
                and len(attitude.observation.value) >= 2
                and _finite_number(attitude.observation.value[1])
            ):
                pitch = abs(float(attitude.observation.value[1]))

                if pitch < vel_threshold:
                    # It's moving slowly enough. Did we just dip below the threshold?
                    if stable_start_time is None:
                        stable_start_time = time.monotonic()
                    # Has it been stable long enough?
                    elif (time.monotonic() - stable_start_time) >= stable_duration:
                        print(
                            f"[*] Drone stabilized. (Current |pitch|: {pitch:.3f} rad)"
                        )
                        return True
                else:
                    stable_start_time = None
            else:
                # It moved too fast, reset the continuous stability timer
                stable_start_time = None

            time.sleep(0.1)

        print("[!] Stabilization timeout reached; moving on anyway.")
        return False

    def _release_configuration(
        self,
    ) -> tuple[ReleaseStabilityConfig, ClearanceCalibration]:
        config = getattr(self, "release_stability_config", None)
        calibration = getattr(self, "clearance_calibration", None)
        if not isinstance(config, ReleaseStabilityConfig) or not isinstance(
            calibration, ClearanceCalibration
        ):
            raise FlightOperationError(
                "release stability requires explicit policy and clearance calibration"
            )
        return config, calibration

    def require_release_configuration(self) -> None:
        """Fail before release-dependent flight output if policy is incomplete."""
        self._release_configuration()

    @staticmethod
    def _release_target(coord: GPSCoord, home: MissionHome) -> tuple[float, float, float]:
        if not all(_finite_number(value) for value in (coord.lat, coord.long, coord.alt)):
            raise FlightOperationError("release waypoint must contain finite coordinates")
        latitude = float(coord.lat)
        longitude = float(coord.long)
        if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
            raise FlightOperationError("release waypoint coordinates are out of bounds")
        return latitude, longitude, home.amsl_m + float(coord.alt)

    @staticmethod
    def _attitude_sample(field) -> AttitudeSample:
        if field is None or not field.fresh:
            raise FlightOperationError("release attitude is missing or stale")
        value = field.observation.value
        if (
            not isinstance(value, tuple)
            or len(value) < 3
            or not all(_finite_number(item) for item in value[:3])
        ):
            raise FlightOperationError("release attitude is invalid")
        return AttitudeSample(
            roll_rad=float(value[0]),
            pitch_rad=float(value[1]),
            yaw_rad=float(value[2]),
            sampled_at=field.observation.received_at,
            sequence=field.observation.sequence,
        )

    def _release_evidence(
        self,
        snapshot,
        coord: GPSCoord,
        lidar,
        required_agl_m: float,
        *,
        now: float,
    ) -> ReleaseEvidence:
        config, calibration = self._release_configuration()
        if not _finite_number(required_agl_m) or required_agl_m <= 0:
            raise FlightOperationError("required release AGL must be finite and positive")
        home = self.mission_home
        if home is None:
            raise FlightOperationError("release stability requires a pinned mission home")
        _, _, target_amsl = self._release_target(coord, home)
        self._require_fresh(snapshot, "mode", "armed", "location", "velocity", "attitude")
        if (
            self._mode_name(snapshot) != "GUIDED"
            or snapshot.armed.observation.value is not True
        ):
            raise FlightOperationError("release stability requires armed GUIDED flight")
        location = self._location_value(snapshot.location)
        velocity = snapshot.velocity.observation.value
        if location is None:
            raise FlightOperationError("release location is invalid")
        if (
            not isinstance(velocity, tuple)
            or len(velocity) < 3
            or not all(_finite_number(item) for item in velocity[:3])
        ):
            raise FlightOperationError("release velocity is invalid")
        attitude = self._attitude_sample(snapshot.attitude)
        try:
            range_sample = lidar.get_sample()
            projected = project_vertical_clearance(
                range_sample, attitude, calibration, now=now
            )
        except (StaleSensorError, ClearanceUnavailableError) as error:
            raise FlightOperationError("release clearance is unavailable") from error
        observed_times = (
            snapshot.location.observation.received_at,
            snapshot.velocity.observation.received_at,
            snapshot.attitude.observation.received_at,
            range_sample.sampled_at,
        )
        if max(observed_times) - min(observed_times) > config.max_observation_skew_seconds:
            raise FlightOperationError("release observations are not coherent")
        horizontal_speed = math.hypot(
            float(velocity[0]) / 100.0, float(velocity[1]) / 100.0
        )
        vertical_speed = abs(float(velocity[2]) / 100.0)
        horizontal_error = horiz_distance_m(
            GPSCoord(location[0] / 1e7, location[1] / 1e7, 0.0),
            GPSCoord(float(coord.lat), float(coord.long), 0.0),
        )
        vertical_error = abs(location[2] / 1000.0 - target_amsl)
        return ReleaseEvidence(
            location_sequence=snapshot.location.observation.sequence,
            velocity_sequence=snapshot.velocity.observation.sequence,
            attitude_sequence=snapshot.attitude.observation.sequence,
            location_invalidation_generation=snapshot.location.invalidation_generation,
            velocity_invalidation_generation=snapshot.velocity.invalidation_generation,
            attitude_invalidation_generation=snapshot.attitude.invalidation_generation,
            range_sequence=range_sample.sequence,
            range_invalidation_generation=range_sample.invalidation_generation,
            observed_at=now,
            horizontal_error_m=horizontal_error,
            vertical_error_m=vertical_error,
            projected_clearance_m=projected.projected_clearance_m,
            horizontal_speed_m_s=horizontal_speed,
            vertical_speed_m_s=vertical_speed,
            roll_rad=attitude.roll_rad,
            pitch_rad=attitude.pitch_rad,
        )

    @staticmethod
    def _release_evidence_is_stable(
        evidence: ReleaseEvidence,
        config: ReleaseStabilityConfig,
        required_agl_m: float,
    ) -> bool:
        return (
            evidence.horizontal_speed_m_s <= config.max_horizontal_speed_m_s
            and evidence.vertical_speed_m_s <= config.max_vertical_speed_m_s
            and abs(evidence.roll_rad) <= config.max_roll_rad
            and abs(evidence.pitch_rad) <= config.max_pitch_rad
            and evidence.horizontal_error_m
            <= config.horizontal_position_tolerance_m
            and evidence.vertical_error_m <= config.vertical_position_tolerance_m
            and evidence.projected_clearance_m >= float(required_agl_m)
        )

    def release_waypoint_for_clearance(
        self, coord: GPSCoord, lidar, *, desired_agl_m: float
    ) -> GPSCoord:
        """Return a target whose altitude corrects current calibrated clearance."""
        self.check_permission()
        config, calibration = self._release_configuration()
        if not _finite_number(desired_agl_m) or desired_agl_m <= 0:
            raise FlightOperationError("desired release AGL must be finite and positive")
        home = self.mission_home
        if home is None:
            raise FlightOperationError("release correction requires a pinned mission home")
        self._release_target(coord, home)
        snapshot = self.flight_state.snapshot()
        self._require_fresh(snapshot, "location", "attitude")
        location = self._location_value(snapshot.location)
        if location is None:
            raise FlightOperationError("release correction location is invalid")
        attitude = self._attitude_sample(snapshot.attitude)
        try:
            range_sample = lidar.get_sample()
            projected = project_vertical_clearance(
                range_sample,
                attitude,
                calibration,
                now=time.monotonic(),
            )
        except (StaleSensorError, ClearanceUnavailableError) as error:
            raise FlightOperationError("release clearance is unavailable") from error
        if (
            max(
                snapshot.location.observation.received_at,
                attitude.sampled_at,
                range_sample.sampled_at,
            )
            - min(
                snapshot.location.observation.received_at,
                attitude.sampled_at,
                range_sample.sampled_at,
            )
            > config.max_observation_skew_seconds
        ):
            raise FlightOperationError("release correction observations are not coherent")
        current_original_home_alt = location[2] / 1000.0 - home.amsl_m
        corrected_alt = (
            current_original_home_alt
            + float(desired_agl_m)
            - projected.projected_clearance_m
        )
        if not math.isfinite(corrected_alt):
            raise FlightOperationError("release correction altitude is invalid")
        return GPSCoord(float(coord.lat), float(coord.long), corrected_alt)

    @staticmethod
    def _evidence_sequences(evidence: ReleaseEvidence) -> tuple[int, int, int, int]:
        return (
            evidence.location_sequence,
            evidence.velocity_sequence,
            evidence.attitude_sequence,
            evidence.range_sequence,
        )

    @staticmethod
    def _evidence_generations(evidence: ReleaseEvidence) -> tuple[int, int, int, int]:
        return (
            evidence.location_invalidation_generation,
            evidence.velocity_invalidation_generation,
            evidence.attitude_invalidation_generation,
            evidence.range_invalidation_generation,
        )

    def hold_waypoint_until_stable(
        self,
        coord: GPSCoord,
        lidar,
        required_agl_m: float,
    ) -> bool:
        """Hold a pinned AMSL waypoint until distinct coherent evidence is stable."""
        config, _ = self._release_configuration()
        self._release_hold_confirmation = None
        if not _finite_number(required_agl_m) or required_agl_m <= 0:
            raise FlightOperationError("required release AGL must be finite and positive")
        self.check_permission()
        home = self.mission_home
        if home is None:
            raise FlightOperationError("waypoint hold requires a pinned mission home")
        latitude, longitude, navigation_amsl = self._release_target(coord, home)
        started_at = time.monotonic()
        stable_started_at = None
        last_accepted_at = None
        last_evidence = None
        next_reissue_at = started_at

        def reset_window() -> None:
            nonlocal stable_started_at, last_accepted_at
            stable_started_at = None
            last_accepted_at = None

        def consume_evidence(
            evidence: ReleaseEvidence, observed_at: float
        ) -> tuple[bool, bool]:
            """Fold one complete read into continuity state as (reset, ready)."""
            nonlocal stable_started_at, last_accepted_at, last_evidence
            if not self._release_evidence_is_stable(
                evidence, config, required_agl_m
            ):
                reset_window()
                last_evidence = evidence
                return True, False
            if last_evidence is None:
                stable_started_at = observed_at
                last_accepted_at = observed_at
                last_evidence = evidence
                return False, False

            sequences = self._evidence_sequences(evidence)
            previous_sequences = self._evidence_sequences(last_evidence)
            generation_changed = self._evidence_generations(
                evidence
            ) != self._evidence_generations(last_evidence)
            regressed = any(
                current < previous
                for current, previous in zip(sequences, previous_sequences)
            )
            all_distinct = all(
                current > previous
                for current, previous in zip(sequences, previous_sequences)
            )
            gap_exceeded = (
                last_accepted_at is not None
                and observed_at - last_accepted_at
                > config.max_observation_gap_seconds
            )
            if regressed:
                reset_window()
                last_evidence = evidence
                return True, False
            if generation_changed:
                stable_started_at = observed_at
                last_accepted_at = observed_at
                last_evidence = evidence
                return True, False
            if gap_exceeded:
                reset_window()
                if all_distinct:
                    stable_started_at = observed_at
                    last_accepted_at = observed_at
                    last_evidence = evidence
                return True, False
            if not all_distinct:
                return False, False
            if stable_started_at is None:
                stable_started_at = observed_at
            last_accepted_at = observed_at
            last_evidence = evidence
            return (
                False,
                observed_at - stable_started_at >= config.hold_seconds,
            )

        while time.monotonic() - started_at < config.timeout_seconds:
            self.check_permission()
            now = time.monotonic()
            try:
                evidence = self._release_evidence(
                    self.flight_state.snapshot(now=now),
                    coord,
                    lidar,
                    required_agl_m,
                    now=now,
                )
            except FlightOperationError:
                reset_window()
                time.sleep(config.poll_interval_seconds)
                continue

            _reset, ready = consume_evidence(evidence, now)

            if now >= next_reissue_at:
                reissue_evidence = []

                def validate_boundary(boundary):
                    reissue_evidence.append(
                        self._release_evidence(
                            boundary,
                            coord,
                            lidar,
                            required_agl_m,
                            now=time.monotonic(),
                        )
                    )

                try:
                    self._send_mission_acknowledged(
                        lambda: _send_guided_waypoint(
                            self.vehicle,
                            latitude,
                            longitude,
                            navigation_amsl,
                            self.flight_controller_target.system_id,
                            self.flight_controller_target.component_id,
                            lambda: None,
                        ),
                        deadline=started_at + config.timeout_seconds,
                        validate_snapshot=validate_boundary,
                    )
                except FlightOperationError as error:
                    if isinstance(error, MissionAckError):
                        raise
                    reset_window()
                    time.sleep(config.poll_interval_seconds)
                    continue
                if not reissue_evidence:
                    raise FlightOperationError(
                        "release reissue evidence was not captured"
                    )
                reissue_reset = False
                reissue_ready = False
                for boundary_evidence in reissue_evidence:
                    reset, current_ready = consume_evidence(
                        boundary_evidence, time.monotonic()
                    )
                    reissue_reset = reissue_reset or reset
                    if not reissue_reset:
                        reissue_ready = reissue_ready or current_ready
                ready = False if reissue_reset else ready or reissue_ready
                next_reissue_at = now + config.waypoint_reissue_interval_seconds

            if ready:
                handover_evidence = None
                handover_ready = True

                def validate_handover(boundary):
                    nonlocal handover_evidence, handover_ready
                    observed_at = time.monotonic()
                    handover_evidence = self._release_evidence(
                        boundary,
                        coord,
                        lidar,
                        required_agl_m,
                        now=observed_at,
                    )
                    reset, current_ready = consume_evidence(
                        handover_evidence, observed_at
                    )
                    handover_ready = (
                        False if reset else handover_ready or current_ready
                    )

                try:
                    self._validate_guarded(validate_handover)
                except FlightOperationError:
                    reset_window()
                    time.sleep(config.poll_interval_seconds)
                    continue
                if handover_evidence is None:
                    raise FlightOperationError(
                        "release handover evidence was not captured"
                    )
                if not handover_ready:
                    time.sleep(config.poll_interval_seconds)
                    continue
                self._release_hold_confirmation = ReleaseHoldConfirmation(
                    waypoint=coord,
                    lidar_identity=id(lidar),
                    required_agl_m=float(required_agl_m),
                    evidence=handover_evidence,
                )
                return True

            time.sleep(config.poll_interval_seconds)

        return False

    def release_payload_if_stable(
        self,
        dropper,
        coord: GPSCoord,
        lidar,
        *,
        required_agl_m: float,
    ) -> None:
        """Consume one completed hold and guard the final payload command."""
        if not _finite_number(required_agl_m) or required_agl_m <= 0:
            raise FlightOperationError("required release AGL must be finite and positive")
        confirmation = getattr(self, "_release_hold_confirmation", None)
        self._release_hold_confirmation = None
        if (
            not isinstance(confirmation, ReleaseHoldConfirmation)
            or confirmation.waypoint != coord
            or confirmation.lidar_identity != id(lidar)
            or confirmation.required_agl_m != float(required_agl_m)
        ):
            raise FlightOperationError("release confirmation is missing or mismatched")
        config, _ = self._release_configuration()
        confirmation_age = time.monotonic() - confirmation.evidence.observed_at
        if (
            not math.isfinite(confirmation_age)
            or confirmation_age < 0
            or confirmation_age > config.max_observation_gap_seconds
        ):
            raise FlightOperationError("release confirmation expired")
        last_boundary_evidence = confirmation.evidence

        def validate_boundary(snapshot):
            nonlocal last_boundary_evidence
            confirmation_age = time.monotonic() - confirmation.evidence.observed_at
            if (
                not math.isfinite(confirmation_age)
                or confirmation_age < 0
                or confirmation_age > config.max_observation_gap_seconds
            ):
                raise FlightOperationError("release confirmation expired")
            evidence = self._release_evidence(
                snapshot,
                coord,
                lidar,
                required_agl_m,
                now=time.monotonic(),
            )
            if not self._release_evidence_is_stable(
                evidence, config, required_agl_m
            ):
                raise FlightOperationError(
                    "release observations are outside stability limits"
                )
            if (
                self._evidence_generations(evidence)
                != self._evidence_generations(confirmation.evidence)
                or any(
                    current < previous
                    for current, previous in zip(
                        self._evidence_sequences(evidence),
                        self._evidence_sequences(last_boundary_evidence),
                    )
                )
            ):
                raise FlightOperationError("release confirmation is no longer current")
            last_boundary_evidence = evidence

        def release():
            if dropper.drop() is not None:
                raise RuntimeError("payload release command returned an invalid result")
        guarded_drop = getattr(dropper, "drop_with_guard", None)
        if callable(guarded_drop):
            guarded_drop(
                lambda output: self._actuate_guarded(
                    output, validate_snapshot=validate_boundary
                ),
                lambda output: self._actuate_guarded(output),
            )
        else:
            self._actuate_guarded(release, validate_snapshot=validate_boundary)
