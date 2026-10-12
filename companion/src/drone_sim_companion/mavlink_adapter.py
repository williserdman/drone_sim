"""Thin PyMAVLink translation boundary."""

from __future__ import annotations

import math
from typing import Any

from .mission import Ack, CommandKind, Telemetry
from .precision_landing import BodyFrd, PRECISION_PARAMETERS


FLIGHT_PARAMETERS = frozenset({
    "WP_SPD",
    "WP_ACC",
    "WP_JERK",
    "LAND_SPD_HIGH_MS",
    "LAND_ALT_LOW_M",
})
OBSERVED_PARAMETERS = frozenset(PRECISION_PARAMETERS) | FLIGHT_PARAMETERS


class MavlinkAdapter:
    def __init__(self, connection: Any, mavutil_module: Any) -> None:
        self._connection = connection
        self._mavutil = mavutil_module
        self._mode: str | None = None
        self._armed: bool | None = None
        self._landed: bool | None = None
        self._parameters: dict[str, float] = {}

    def send(self, command: CommandKind, altitude_m: float | None) -> None:
        mavlink = self._mavutil.mavlink
        parameters = [0.0] * 7
        if command is CommandKind.SET_GUIDED:
            command_id = mavlink.MAV_CMD_DO_SET_MODE
            parameters[0] = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            parameters[1] = 4  # Copter GUIDED custom mode, frozen for Copter-4.7.0.
        elif command is CommandKind.ARM:
            command_id = mavlink.MAV_CMD_COMPONENT_ARM_DISARM
            parameters[0] = 1
        elif command is CommandKind.TAKEOFF:
            command_id = mavlink.MAV_CMD_NAV_TAKEOFF
            if altitude_m is None:
                raise ValueError("takeoff command requires altitude")
            parameters[6] = altitude_m
        elif command is CommandKind.LAND:
            command_id = mavlink.MAV_CMD_NAV_LAND
        else:  # pragma: no cover - Enum exhaustiveness guard.
            raise ValueError(f"unsupported mission command {command!r}")
        self._connection.mav.command_long_send(
            self._connection.target_system,
            self._connection.target_component,
            command_id,
            0,
            *parameters,
        )

    def send_waypoint(
        self, latitude_deg: float, longitude_deg: float, altitude_m: float
    ) -> None:
        values = (latitude_deg, longitude_deg, altitude_m)
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            for value in values
        ):
            raise ValueError("waypoint coordinates and altitude must be finite numbers")
        if not -90.0 <= latitude_deg <= 90.0:
            raise ValueError("waypoint latitude must be between -90 and 90 degrees")
        if not -180.0 <= longitude_deg <= 180.0:
            raise ValueError("waypoint longitude must be between -180 and 180 degrees")
        mavlink = self._mavutil.mavlink
        type_mask = (
            mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        self._connection.mav.set_position_target_global_int_send(
            0,
            self._connection.target_system,
            self._connection.target_component,
            mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            int(round(latitude_deg * 1e7)),
            int(round(longitude_deg * 1e7)),
            altitude_m,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def send_mode(self, mode: str) -> None:
        custom_mode = {"GUIDED": 4, "LAND": 9}.get(mode)
        if custom_mode is None:
            raise ValueError(f"unsupported mode {mode!r}")
        mavlink = self._mavutil.mavlink
        self._connection.mav.command_long_send(
            self._connection.target_system,
            self._connection.target_component,
            mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode,
            0,
            0,
            0,
            0,
            0,
        )

    def send_landing_target(self, vector: BodyFrd) -> None:
        values = (vector.forward_m, vector.right_m, vector.down_m)
        if any(isinstance(value, bool) or not isinstance(value, (int, float))
               or not math.isfinite(value) for value in values):
            raise ValueError("landing target must contain finite body-FRD values")
        if vector.down_m <= 0:
            raise ValueError("landing target must be below the vehicle")
        # ArduPilot reconstructs body forward from -angle_y and body right
        # from angle_x for the legacy eight-field LANDING_TARGET message.
        angle_x = math.atan2(vector.right_m, vector.down_m)
        angle_y = math.atan2(-vector.forward_m, vector.down_m)
        distance = math.sqrt(sum(float(value) ** 2 for value in values))
        self._connection.mav.landing_target_send(
            0,
            0,
            self._mavutil.mavlink.MAV_FRAME_BODY_FRD,
            angle_x,
            angle_y,
            distance,
            0.0,
            0.0,
        )

    def request_precision_parameters(self) -> None:
        request = getattr(self._connection.mav, "param_request_read_send", None)
        if request is None:
            raise RuntimeError("MAVLink parameter requests are unavailable")
        for name in PRECISION_PARAMETERS:
            request(
                self._connection.target_system,
                self._connection.target_component,
                name.encode("ascii"),
                -1,
            )

    def configure_flight(self, parameters: dict[str, float]) -> None:
        for name, value in parameters.items():
            if name not in FLIGHT_PARAMETERS:
                raise ValueError(f"unknown flight parameter: {name!r}")
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value)):
                raise ValueError(f"flight parameter {name} must be a finite number")

        set_parameter = getattr(self._connection.mav, "param_set_send", None)
        request_parameter = getattr(
            self._connection.mav, "param_request_read_send", None
        )
        if set_parameter is None or request_parameter is None:
            raise RuntimeError("MAVLink parameter configuration is unavailable")

        for name in parameters:
            self._parameters.pop(name, None)
        for name, value in parameters.items():
            set_parameter(
                self._connection.target_system,
                self._connection.target_component,
                name.encode("ascii"),
                float(value),
                self._mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
        for name in parameters:
            request_parameter(
                self._connection.target_system,
                self._connection.target_component,
                name.encode("ascii"),
                -1,
            )

    def request_vfs_baseline(self) -> None:
        mavlink = self._mavutil.mavlink
        self._connection.mav.command_long_send(
            self._connection.target_system,
            self._connection.target_component,
            mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            getattr(mavlink, "MAVLINK_MSG_ID_HOME_POSITION", 242),
            0,
            0,
            0,
            0,
            0,
            0,
        )
        self.request_precision_parameters()

    def request_telemetry(self, *, rate_hz: int = 10) -> None:
        if not isinstance(rate_hz, int) or isinstance(rate_hz, bool) or rate_hz <= 0:
            raise ValueError("telemetry rate must be a positive integer")
        self._connection.mav.request_data_stream_send(
            self._connection.target_system,
            self._connection.target_component,
            self._mavutil.mavlink.MAV_DATA_STREAM_ALL,
            rate_hz,
            1,
        )
        for message_id in (
            245,  # EXTENDED_SYS_STATE
            getattr(self._mavutil.mavlink, "MAVLINK_MSG_ID_HEARTBEAT", 0),
        ):
            self._connection.mav.command_long_send(
                self._connection.target_system,
                self._connection.target_component,
                self._mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                1_000_000 // rate_hz,
                0,
                0,
                0,
                0,
                0,
            )

    def poll(self, timestamp_ns: int) -> Telemetry | None:
        message = self._connection.recv_match(blocking=False)
        if message is None:
            return None
        kind = message.get_type()
        mavlink = self._mavutil.mavlink
        if kind == "HEARTBEAT":
            self._mode = str(self._mavutil.mode_string_v10(message)).upper()
            self._armed = bool(message.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            bad_states = {
                getattr(mavlink, name)
                for name in ("MAV_STATE_CRITICAL", "MAV_STATE_EMERGENCY", "MAV_STATE_FLIGHT_TERMINATION")
                if hasattr(mavlink, name)
            }
            system_status = getattr(message, "system_status", None)
            return Telemetry(
                timestamp_ns,
                heartbeat=True,
                mode=self._mode,
                armed=self._armed,
                landed=self._landed,
                failsafe=system_status in bad_states if system_status is not None else None,
                failsafe_reason=("autopilot state is critical" if system_status in bad_states else None),
            )
        if kind == "COMMAND_ACK":
            commands = {
                mavlink.MAV_CMD_DO_SET_MODE: CommandKind.SET_GUIDED,
                mavlink.MAV_CMD_COMPONENT_ARM_DISARM: CommandKind.ARM,
                mavlink.MAV_CMD_NAV_TAKEOFF: CommandKind.TAKEOFF,
                mavlink.MAV_CMD_NAV_LAND: CommandKind.LAND,
            }
            command = commands.get(int(message.command))
            result = int(message.result)
            if command is None or result == mavlink.MAV_RESULT_IN_PROGRESS:
                return None
            return Telemetry(
                timestamp_ns,
                ack=Ack(command, result == mavlink.MAV_RESULT_ACCEPTED, result),
            )
        if kind == "SYS_STATUS":
            prearm_bit = int(mavlink.MAV_SYS_STATUS_PREARM_CHECK)
            enabled = bool(int(message.onboard_control_sensors_enabled) & prearm_bit)
            healthy = bool(int(message.onboard_control_sensors_health) & prearm_bit)
            return Telemetry(
                timestamp_ns,
                prearm_checks_healthy=enabled and healthy,
            )
        if kind == "GLOBAL_POSITION_INT":
            latitude = getattr(message, "lat", None)
            longitude = getattr(message, "lon", None)
            return Telemetry(
                timestamp_ns,
                mode=self._mode,
                armed=self._armed,
                landed=self._landed,
                relative_altitude_m=float(message.relative_alt) / 1000.0,
                vertical_speed_m_s=-float(message.vz) / 100.0,
                horizontal_velocity_north_m_s=(
                    float(message.vx) / 100.0 if getattr(message, "vx", None) is not None else None
                ),
                horizontal_velocity_east_m_s=(
                    float(message.vy) / 100.0 if getattr(message, "vy", None) is not None else None
                ),
                latitude_deg=float(latitude) / 1e7 if latitude is not None else None,
                longitude_deg=float(longitude) / 1e7 if longitude is not None else None,
            )
        if kind == "ATTITUDE":
            return Telemetry(
                timestamp_ns,
                roll_rad=float(message.roll),
                pitch_rad=float(message.pitch),
                yaw_rad=float(message.yaw),
            )
        if kind == "HOME_POSITION":
            return Telemetry(
                timestamp_ns,
                home_latitude_deg=float(message.latitude) / 1e7,
                home_longitude_deg=float(message.longitude) / 1e7,
                home_altitude_m=float(message.altitude) / 1000.0,
            )
        if kind == "PARAM_VALUE":
            raw_name = message.param_id
            if isinstance(raw_name, bytes):
                name = raw_name.decode("ascii", errors="strict").rstrip("\x00")
            else:
                name = str(raw_name).rstrip("\x00")
            if name in OBSERVED_PARAMETERS:
                self._parameters[name] = float(message.param_value)
            return Telemetry(timestamp_ns, precision_parameters=dict(self._parameters))
        if kind == "EXTENDED_SYS_STATE":
            self._landed = int(message.landed_state) == mavlink.MAV_LANDED_STATE_ON_GROUND
            return Telemetry(
                timestamp_ns,
                mode=self._mode,
                armed=self._armed,
                landed=self._landed,
            )
        if kind == "STATUSTEXT":
            return Telemetry(
                timestamp_ns,
                status_text=str(message.text),
                status_severity=int(message.severity),
            )
        return None

__all__ = ["MavlinkAdapter"]
