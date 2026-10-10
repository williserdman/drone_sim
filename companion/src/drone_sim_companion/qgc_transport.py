"""QGroundControl command adapters for the existing MAVLink connections."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
import queue
import time
from typing import Any, Callable

from artifacts.qgc_operator import (
    MavlinkIdentity,
    QgcCommandEnvelope,
    QgcCommandResponse,
)
from .qgc_control import QgcMissionControl


COMPANION_IDENTITY = MavlinkIdentity(1, 191)
FLIGHT_CONTROLLER_IDENTITY = MavlinkIdentity(1, 1)


@dataclass(frozen=True)
class VehicleFactObservation:
    received_wall_time_ns: int
    native_timestamp_ns: int | None = None
    heartbeat_live: bool | None = None
    prearm_checks_healthy: bool | None = None
    armed: bool | None = None
    landed: bool | None = None
    mode: str | None = None


class QgcHostControl:
    """Expose the existing host-control surface through QGC admission."""

    def __init__(
        self,
        *,
        protocol: Any,
        operator: Any,
        descriptor: Any,
        io: Any,
        readiness_reasons: Callable[[], tuple[str, ...]],
        is_finalizing: Callable[[], bool],
    ) -> None:
        self._operator = operator
        self._qgc = QgcMissionControl(
            protocol,
            operator,
            descriptor,
            io,
            readiness_reasons,
            is_finalizing,
        )

    def observe_vehicle(self, **facts: Any) -> None:
        self._operator.observe_vehicle(**facts)

    def poll_start(self, *, run_state: Any) -> Any:
        return self._qgc.poll_start(run_state=run_state)

    def poll_abort(self, **arguments: Any) -> None:
        self._qgc.poll_abort(**arguments)

    def fail_recovery(self, detail: str) -> None:
        self._operator.fail_recovery(detail)

    def observe_mission_terminal(self, outcome: str) -> None:
        self._qgc.observe_mission_terminal(outcome)

    def __getattr__(self, name: str) -> Any:
        return getattr(self._operator, name)


def configure_dronekit_mavlink2() -> None:
    """Select MAVLink 2 before DroneKit constructs its encoder and queue."""
    os.environ["MAVLINK20"] = "1"
    from pymavlink import mavutil

    mavutil.set_dialect("ardupilotmega")
    if mavutil.mavlink.WIRE_PROTOCOL_VERSION != "2.0":
        raise RuntimeError("pymavlink did not select MAVLink 2")


def _identity(message: object, prefix: str = "src") -> MavlinkIdentity | None:
    system_getter = getattr(message, f"get_{prefix}System", None)
    component_getter = getattr(message, f"get_{prefix}Component", None)
    if not callable(system_getter) or not callable(component_getter):
        return None
    try:
        system = system_getter()
        component = component_getter()
    except Exception:
        return None
    if (
        type(system) is not int
        or type(component) is not int
        or not 1 <= system <= 255
        or not 1 <= component <= 255
    ):
        return None
    return MavlinkIdentity(system, component)


def _wire_version(message: object) -> int:
    get_buffer = getattr(message, "get_msgbuf", None)
    if callable(get_buffer):
        try:
            frame = bytes(get_buffer())
        except Exception:
            frame = b""
        if frame[:1] == b"\xfd":
            return 2
        if frame[:1] == b"\xfe":
            return 1
    return 0


def _command_envelope(
    message: object, *, wall_time_ns: Callable[[], int]
) -> QgcCommandEnvelope | None:
    get_type = getattr(message, "get_type", None)
    if not callable(get_type) or get_type() != "COMMAND_LONG":
        return None
    source = _identity(message)
    target_system = getattr(message, "target_system", None)
    target_component = getattr(message, "target_component", None)
    if (
        source is None
        or type(target_system) is not int
        or type(target_component) is not int
        or not 1 <= target_system <= 255
        or not 1 <= target_component <= 255
    ):
        return None
    target = MavlinkIdentity(target_system, target_component)
    if target == FLIGHT_CONTROLLER_IDENTITY:
        return None
    command = getattr(message, "command", None)
    if type(command) is not int:
        return None
    parameters = tuple(getattr(message, f"param{index}", math.nan) for index in range(1, 8))
    return QgcCommandEnvelope(
        _wire_version(message),
        source,
        target,
        command,
        parameters,
        wall_time_ns(),
    )


def _status_bytes(text: str) -> bytes:
    return text.encode("utf-8")


class PymavlinkQgcCommandIO:
    """Observe and answer QGC commands on one raw pymavlink connection."""

    def __init__(
        self,
        connection: Any,
        mavlink: Any,
        *,
        wall_time_ns: Callable[[], int] = time.time_ns,
    ) -> None:
        encoder = getattr(connection, "mav", None)
        if encoder is None:
            raise TypeError("connection must expose one MAVLink encoder")
        if (
            getattr(encoder, "srcSystem", None),
            getattr(encoder, "srcComponent", None),
        ) != (COMPANION_IDENTITY.system_id, COMPANION_IDENTITY.component_id):
            raise ValueError("MAVLink encoder must use companion identity 1/191")
        self._connection = connection
        self._mavlink = mavlink
        self._wall_time_ns = wall_time_ns
        self._receive: Callable[[QgcCommandEnvelope], None] | None = None
        encoder.heartbeat_send(
            mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavlink.MAV_STATE_ACTIVE,
        )

    def install(self, receive: Callable[[QgcCommandEnvelope], None]) -> None:
        if not callable(receive):
            raise TypeError("receive must be callable")
        if self._receive is not None:
            raise RuntimeError("QGC command callback is already installed")
        self._receive = receive

    def observe_message(self, message: object) -> bool:
        envelope = _command_envelope(message, wall_time_ns=self._wall_time_ns)
        if envelope is None:
            return False
        if self._receive is None:
            raise RuntimeError("QGC command callback is not installed")
        self._receive(envelope)
        return True

    def send_ack(self, response: QgcCommandResponse) -> None:
        if not isinstance(response, QgcCommandResponse):
            raise TypeError("response must be a QgcCommandResponse")
        mav = self._connection.mav
        mav.command_ack_send(
            response.command,
            response.result,
            response.progress,
            0,
            response.recipient.system_id,
            response.recipient.component_id,
        )
        mav.statustext_send(
            self._mavlink.MAV_SEVERITY_INFO,
            _status_bytes(response.status_text),
        )


class _TargetPreservingMessage:
    """Hide target fields from DroneKit's FC-targeting send wrapper."""

    __slots__ = ("_message",)

    def __init__(self, message: Any) -> None:
        self._message = message

    def pack(self, mav: object, *, force_mavlink1: bool = False) -> bytes:
        return self._message.pack(mav, force_mavlink1=False)


class DroneKitQgcCommandIO:
    """Observe and answer QGC commands through DroneKit's existing queue."""

    def __init__(
        self,
        vehicle: Any,
        mavlink: Any,
        *,
        wall_time_ns: Callable[[], int] = time.time_ns,
    ) -> None:
        master = getattr(vehicle, "_master", None)
        encoder = getattr(master, "mav", None)
        if encoder is None or not callable(getattr(vehicle, "send_mavlink", None)):
            raise TypeError("vehicle must expose the DroneKit MAVLink interface")
        if getattr(master, "WIRE_PROTOCOL_VERSION", None) != "2.0":
            raise ValueError("DroneKit connection must use MAVLink 2")
        if (getattr(encoder, "srcSystem", None), getattr(encoder, "srcComponent", None)) != (
            COMPANION_IDENTITY.system_id,
            COMPANION_IDENTITY.component_id,
        ):
            raise ValueError("DroneKit encoder must use companion identity 1/191")
        writer = getattr(getattr(encoder, "file", None), "delegate", getattr(encoder, "file", None))
        if (
            writer is None
            or type(writer).__module__ != "dronekit.mavlink"
            or type(writer).__name__ != "MAVWriter"
            or not isinstance(getattr(writer, "queue", None), queue.Queue)
        ):
            raise ValueError("DroneKit encoder must use its existing MAVWriter queue")
        self._vehicle = vehicle
        self._mavlink = mavlink
        self._encoder = encoder
        self._wall_time_ns = wall_time_ns
        self._receive: Callable[[QgcCommandEnvelope], None] | None = None
        heartbeat = encoder.heartbeat_encode(
            mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavlink.MAV_STATE_ACTIVE,
        )
        vehicle.send_mavlink(heartbeat)

    def install(self, receive: Callable[[QgcCommandEnvelope], None]) -> None:
        if not callable(receive):
            raise TypeError("receive must be callable")
        if self._receive is not None:
            raise RuntimeError("QGC command callback is already installed")
        install = getattr(self._vehicle, "install_qgc_message_callback", None)
        if not callable(install):
            raise TypeError("SimulationVehicle QGC dispatch hook is unavailable")
        self._receive = receive
        install(self.observe_message)

    def observe_message(self, message: object) -> bool:
        envelope = _command_envelope(message, wall_time_ns=self._wall_time_ns)
        if envelope is None:
            return False
        if self._receive is None:
            raise RuntimeError("QGC command callback is not installed")
        self._receive(envelope)
        return True

    def send_ack(self, response: QgcCommandResponse) -> None:
        if not isinstance(response, QgcCommandResponse):
            raise TypeError("response must be a QgcCommandResponse")
        ack = self._encoder.command_ack_encode(
            response.command,
            response.result,
            response.progress,
            0,
            response.recipient.system_id,
            response.recipient.component_id,
        )
        self._vehicle.send_mavlink(_TargetPreservingMessage(ack))
        status = self._encoder.statustext_encode(
            self._mavlink.MAV_SEVERITY_INFO,
            _status_bytes(response.status_text),
        )
        self._vehicle.send_mavlink(status)


__all__ = [
    "COMPANION_IDENTITY",
    "DroneKitQgcCommandIO",
    "FLIGHT_CONTROLLER_IDENTITY",
    "PymavlinkQgcCommandIO",
    "QgcHostControl",
    "VehicleFactObservation",
    "configure_dronekit_mavlink2",
]
