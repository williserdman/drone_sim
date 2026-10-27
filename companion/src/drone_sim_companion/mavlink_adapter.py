"""Thin PyMAVLink translation boundary."""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from contextlib import ExitStack
from typing import Any

from .flight_authority import ExpectedTransition, FlightAuthorityGate
from .mission import Ack, CommandKind, Telemetry
from .qgc_transport import VehicleFactObservation


FLIGHT_CONTROLLER_SYSTEM_ID = 1
FLIGHT_CONTROLLER_COMPONENT_ID = 1


def is_flight_controller_message(message: Any) -> bool:
    """Accept only messages from the pinned flight-controller component."""
    system_source = getattr(message, "get_srcSystem", None)
    component_source = getattr(message, "get_srcComponent", None)
    if not callable(system_source) or not callable(component_source):
        return False
    try:
        system_id = system_source()
        component_id = component_source()
    except Exception:
        return False
    return (
        isinstance(system_id, int)
        and not isinstance(system_id, bool)
        and system_id == FLIGHT_CONTROLLER_SYSTEM_ID
        and isinstance(component_id, int)
        and not isinstance(component_id, bool)
        and component_id == FLIGHT_CONTROLLER_COMPONENT_ID
    )


def request_flight_controller_heartbeat(
    mav: Any, mavlink: Any, *, rate_hz: int = 10
) -> None:
    """Request a wall-fresh heartbeat despite a slowed SITL simulation clock."""
    if not isinstance(rate_hz, int) or isinstance(rate_hz, bool) or rate_hz <= 0:
        raise ValueError("heartbeat rate must be a positive integer")
    mav.command_long_send(
        FLIGHT_CONTROLLER_SYSTEM_ID,
        FLIGHT_CONTROLLER_COMPONENT_ID,
        mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavlink.MAVLINK_MSG_ID_HEARTBEAT,
        1_000_000 // rate_hz,
        0,
        0,
        0,
        0,
        0,
    )


class MavlinkAdapter:
    def __init__(
        self,
        connection: Any,
        mavutil_module: Any,
        *,
        qgc_io: Any | None = None,
        authority_gate: FlightAuthorityGate | None = None,
        wall_time_ns: Any = time.time_ns,
    ) -> None:
        self._connection = connection
        self._mavutil = mavutil_module
        self._mode: str | None = None
        self._armed: bool | None = None
        self._landed: bool | None = None
        self._qgc_io = qgc_io
        self._authority_gate = authority_gate
        self._wall_time_ns = wall_time_ns
        self._operator_observations: deque[VehicleFactObservation] = deque(maxlen=64)
        self._operator_observations_lock = threading.Lock()

    def drain_operator_observations(self) -> tuple[VehicleFactObservation, ...]:
        with self._operator_observations_lock:
            observations = tuple(self._operator_observations)
            self._operator_observations.clear()
            return observations

    def _observe_operator_fact(self, message: Any, timestamp_ns: int) -> None:
        kind = message.get_type()
        mavlink = self._mavutil.mavlink
        values: dict[str, Any] = {}
        if kind == "HEARTBEAT":
            values = {
                "heartbeat_live": True,
                "armed": bool(message.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                "mode": str(self._mavutil.mode_string_v10(message)).upper(),
            }
        elif kind == "SYS_STATUS":
            prearm = int(mavlink.MAV_SYS_STATUS_PREARM_CHECK)
            values = {
                "prearm_checks_healthy": bool(
                    int(message.onboard_control_sensors_enabled) & prearm
                )
                and bool(int(message.onboard_control_sensors_health) & prearm)
            }
        elif kind == "EXTENDED_SYS_STATE":
            values = {
                "landed": int(message.landed_state)
                == mavlink.MAV_LANDED_STATE_ON_GROUND
            }
        if values:
            received_wall_time_ns = self._wall_time_ns()
            with self._operator_observations_lock:
                self._operator_observations.append(
                    VehicleFactObservation(
                        received_wall_time_ns,
                        native_timestamp_ns=timestamp_ns,
                        **values,
                    )
                )
            if kind == "HEARTBEAT" and self._authority_gate is not None:
                self._authority_gate.observe(
                    Telemetry(
                        received_wall_time_ns,
                        heartbeat=True,
                        mode=values["mode"],
                        armed=values["armed"],
                    )
                )

    def _send_guarded(
        self,
        output: Any,
        *expected: ExpectedTransition,
    ) -> None:
        gate = self._authority_gate
        if gate is None:
            output()
            return
        with ExitStack() as transactions:
            for transition in expected:
                transactions.enter_context(gate.output_transaction(transition))
            if not expected:
                transactions.enter_context(gate.output_transaction())
            output()

    def send(self, command: CommandKind, altitude_m: float | None) -> None:
        mavlink = self._mavutil.mavlink
        parameters = [0.0] * 7
        if command is CommandKind.SET_GUIDED:
            command_id = mavlink.MAV_CMD_DO_SET_MODE
            parameters[0] = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            parameters[1] = 4  # Copter GUIDED custom mode, frozen for Copter-4.6.3.
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
        expected: tuple[ExpectedTransition, ...] = ()
        if command is CommandKind.SET_GUIDED:
            expected = (ExpectedTransition(mode="GUIDED"),)
        elif command is CommandKind.ARM:
            expected = (ExpectedTransition(armed=True),)
        elif command is CommandKind.LAND:
            expected = (
                ExpectedTransition(mode="LAND"),
                ExpectedTransition(armed=False),
            )
        self._send_guarded(
            lambda: self._connection.mav.command_long_send(
                FLIGHT_CONTROLLER_SYSTEM_ID,
                FLIGHT_CONTROLLER_COMPONENT_ID,
                command_id,
                0,
                *parameters,
            ),
            *expected,
        )

    def send_waypoint(
        self,
        latitude_deg: float,
        longitude_deg: float,
        altitude_m: float,
        *,
        yaw_rad: float | None = None,
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
        if yaw_rad is not None and (
            type(yaw_rad) not in (int, float) or not math.isfinite(yaw_rad)
        ):
            raise ValueError("waypoint yaw must be finite")
        mavlink = self._mavutil.mavlink
        type_mask = (
            mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        if yaw_rad is None:
            type_mask |= mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        self._send_guarded(
            lambda: self._connection.mav.set_position_target_global_int_send(
                0,
                FLIGHT_CONTROLLER_SYSTEM_ID,
                FLIGHT_CONTROLLER_COMPONENT_ID,
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
                0.0 if yaw_rad is None else float(yaw_rad),
                0.0,
            )
        )

    def send_landing_target(
        self,
        forward_m: float,
        right_m: float,
        down_m: float,
        *,
        exposure_timestamp_ns: int,
    ) -> None:
        values = (forward_m, right_m, down_m)
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in values):
            raise ValueError("landing target vector must contain finite numbers")
        if down_m <= 0:
            raise ValueError("landing target down distance must be positive")
        distance = math.sqrt(forward_m**2 + right_m**2 + down_m**2)
        self._send_guarded(
            lambda: self._connection.mav.landing_target_send(
                exposure_timestamp_ns // 1_000,
                0,
                self._mavutil.mavlink.MAV_FRAME_BODY_FRD,
                math.atan2(forward_m, down_m),
                math.atan2(right_m, down_m),
                distance,
                0.0,
                0.0,
                float(forward_m),
                float(right_m),
                float(down_m),
                (1.0, 0.0, 0.0, 0.0),
                0,
                1,
            )
        )

    def request_parameters(self, names: tuple[str, ...]) -> None:
        if not isinstance(names, tuple) or not names:
            raise ValueError("parameter names must be a nonempty tuple")
        for name in names:
            if not isinstance(name, str) or not name or len(name.encode("ascii")) > 16:
                raise ValueError("parameter names must be nonempty MAVLink ASCII names")
        self._connection.mav.param_request_list_send(
            FLIGHT_CONTROLLER_SYSTEM_ID,
            FLIGHT_CONTROLLER_COMPONENT_ID,
        )

    def request_telemetry(self, *, rate_hz: int = 10) -> None:
        if not isinstance(rate_hz, int) or isinstance(rate_hz, bool) or rate_hz <= 0:
            raise ValueError("telemetry rate must be a positive integer")
        self._connection.mav.request_data_stream_send(
            FLIGHT_CONTROLLER_SYSTEM_ID,
            FLIGHT_CONTROLLER_COMPONENT_ID,
            self._mavutil.mavlink.MAV_DATA_STREAM_ALL,
            rate_hz,
            1,
        )
        request_flight_controller_heartbeat(
            self._connection.mav, self._mavutil.mavlink, rate_hz=rate_hz
        )
        self._connection.mav.command_long_send(
            FLIGHT_CONTROLLER_SYSTEM_ID,
            FLIGHT_CONTROLLER_COMPONENT_ID,
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
        message = None
        for _ in range(100):
            candidate = self._connection.recv_match(blocking=False)
            if candidate is None:
                return None
            if self._qgc_io is not None:
                self._qgc_io.observe_message(candidate)
            if is_flight_controller_message(candidate):
                message = candidate
                break
        if message is None:
            return None
        self._observe_operator_fact(message, timestamp_ns)
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
            result = int(message.result)
            if int(message.command) == mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
                if result not in {
                    mavlink.MAV_RESULT_ACCEPTED,
                    mavlink.MAV_RESULT_IN_PROGRESS,
                }:
                    raise RuntimeError(
                        "flight controller rejected telemetry interval request "
                        f"with MAVLink result {result}"
                    )
                return None
            commands = {
                mavlink.MAV_CMD_DO_SET_MODE: CommandKind.SET_GUIDED,
                mavlink.MAV_CMD_COMPONENT_ARM_DISARM: CommandKind.ARM,
                mavlink.MAV_CMD_NAV_TAKEOFF: CommandKind.TAKEOFF,
                mavlink.MAV_CMD_NAV_LAND: CommandKind.LAND,
            }
            command = commands.get(int(message.command))
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
            vx = getattr(message, "vx", None)
            vy = getattr(message, "vy", None)
            horizontal_speed = (
                math.hypot(float(vx), float(vy)) / 100.0
                if vx is not None and vy is not None
                else None
            )
            return Telemetry(
                timestamp_ns,
                mode=self._mode,
                armed=self._armed,
                landed=self._landed,
                relative_altitude_m=float(message.relative_alt) / 1000.0,
                vertical_speed_m_s=-float(message.vz) / 100.0,
                latitude_deg=float(latitude) / 1e7 if latitude is not None else None,
                longitude_deg=float(longitude) / 1e7 if longitude is not None else None,
                horizontal_speed_m_s=horizontal_speed,
            )
        if kind == "ATTITUDE":
            attitude = (float(message.roll), float(message.pitch), float(message.yaw))
            if not all(math.isfinite(value) for value in attitude):
                raise ValueError("ATTITUDE contains a non-finite angle")
            return Telemetry(
                timestamp_ns,
                attitude_rpy_rad=attitude,
                attitude_timestamp_ns=timestamp_ns,
            )
        if kind == "PARAM_VALUE":
            raw_name = message.param_id
            if isinstance(raw_name, bytes):
                name = raw_name.split(b"\0", 1)[0].decode("ascii")
            else:
                name = str(raw_name).split("\0", 1)[0]
            value = float(message.param_value)
            if not name or not math.isfinite(value):
                raise ValueError("PARAM_VALUE is malformed")
            return Telemetry(
                timestamp_ns,
                parameter_name=name,
                parameter_value=value,
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

__all__ = [
    "FLIGHT_CONTROLLER_COMPONENT_ID",
    "FLIGHT_CONTROLLER_SYSTEM_ID",
    "MavlinkAdapter",
    "is_flight_controller_message",
    "request_flight_controller_heartbeat",
]
