"""Thin PyMAVLink translation boundary."""

from __future__ import annotations

from typing import Any

from .mission import Ack, CommandKind, Telemetry


class MavlinkAdapter:
    def __init__(self, connection: Any, mavutil_module: Any) -> None:
        self._connection = connection
        self._mavutil = mavutil_module
        self._mode: str | None = None
        self._armed: bool | None = None
        self._landed: bool | None = None

    def send(self, command: CommandKind, altitude_m: float | None) -> None:
        mavlink = self._mavutil.mavlink
        parameters = [0.0] * 7
        if command is CommandKind.SET_GUIDED:
            command_id = mavlink.MAV_CMD_DO_SET_MODE
            parameters[0] = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            parameters[1] = 4  # Copter GUIDED custom mode for the pinned 4.5.7 target.
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
        self._connection.mav.command_long_send(
            self._connection.target_system,
            self._connection.target_component,
            self._mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            245,  # EXTENDED_SYS_STATE
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
            return Telemetry(
                timestamp_ns,
                heartbeat=True,
                mode=self._mode,
                armed=self._armed,
                landed=self._landed,
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
            return Telemetry(
                timestamp_ns,
                mode=self._mode,
                armed=self._armed,
                landed=self._landed,
                relative_altitude_m=float(message.relative_alt) / 1000.0,
                vertical_speed_m_s=-float(message.vz) / 100.0,
            )
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
