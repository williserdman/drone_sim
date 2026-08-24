"""Side-effect boundary for the pure mission policy."""

from __future__ import annotations

from collections.abc import Callable
from typing import Protocol

from .mission import CommandKind, MissionPhase, MissionState, Telemetry, advance


class VehicleCommands(Protocol):
    def send(self, command: CommandKind, altitude_m: float | None) -> None: ...


EventSink = Callable[[str, int, dict[str, object]], None]


class MissionController:
    def __init__(self, vehicle: VehicleCommands, emit: EventSink) -> None:
        self._vehicle = vehicle
        self._emit = emit
        self.state = MissionState.initial()
        self.ready = False

    def observe_readiness(self, telemetry: Telemetry) -> None:
        """Latch the infrastructure heartbeat without advancing mission policy."""
        if telemetry.heartbeat and not self.ready:
            self.ready = True
            self._emit("heartbeat_observed", telemetry.timestamp_ns, {})

    def consume(self, telemetry: Telemetry) -> None:
        previous = self.state
        if previous.phase in {MissionPhase.FAILED, MissionPhase.LANDED}:
            return
        self.observe_readiness(telemetry)
        transition = advance(previous, telemetry)
        for command in transition.commands:
            self._vehicle.send(command.kind, command.altitude_m)
        self.state = transition.state
        if (
            previous.phase is MissionPhase.WAIT_GUIDED_MODE
            and transition.state.phase is MissionPhase.WAIT_ARM_ACK
        ):
            self._emit("mode_confirmed", telemetry.timestamp_ns, {"mode": "GUIDED"})
        if (
            previous.phase is MissionPhase.WAIT_ARMED
            and transition.state.phase is MissionPhase.WAIT_TAKEOFF_ACK
        ):
            self._emit("armed_observed", telemetry.timestamp_ns, {})
        if telemetry.relative_altitude_m is not None:
            fields: dict[str, object] = {"relative_altitude_m": telemetry.relative_altitude_m}
            if telemetry.vertical_speed_m_s is not None:
                fields["vertical_speed_m_s"] = telemetry.vertical_speed_m_s
            self._emit("altitude_observed", telemetry.timestamp_ns, fields)
        for event in transition.events:
            self._emit(event.name, event.timestamp_ns, dict(event.fields))


def process_telemetry(
    controller: MissionController,
    telemetry: Telemetry,
    *,
    mission_running: bool,
) -> None:
    """Separate pre-clock readiness from RUNNING mission transitions."""
    if mission_running:
        controller.consume(telemetry)
    else:
        controller.observe_readiness(telemetry)


__all__ = ["MissionController", "VehicleCommands", "process_telemetry"]
