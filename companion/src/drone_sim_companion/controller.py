"""Side-effect boundary for the pure mission policy."""

from __future__ import annotations

from collections.abc import Callable
from typing import Protocol

from .mission import CommandKind, MissionPhase, MissionState, Telemetry, advance


class VehicleCommands(Protocol):
    def send(self, command: CommandKind, altitude_m: float | None) -> None: ...


EventSink = Callable[[str, int, dict[str, object]], None]
CommandDeliveredSink = Callable[[CommandKind, int], None]


def mission_policy_active(*, mission_running: bool, public_clock_observed: bool) -> bool:
    return mission_running and public_clock_observed


class MissionController:
    def __init__(
        self,
        vehicle: VehicleCommands,
        emit: EventSink,
        command_delivered: CommandDeliveredSink | None = None,
    ) -> None:
        self._vehicle = vehicle
        self._emit = emit
        self._command_delivered = command_delivered or (lambda _command, _stamp: None)
        self.state = MissionState.initial()
        self._heartbeat_observed = False
        self._prearm_checks_healthy = False

    @property
    def heartbeat_observed(self) -> bool:
        return self._heartbeat_observed

    @property
    def prearm_checks_healthy(self) -> bool:
        return self._prearm_checks_healthy

    @property
    def mission_ready(self) -> bool:
        return self._heartbeat_observed and self._prearm_checks_healthy

    @property
    def ready(self) -> bool:
        """Backward-compatible heartbeat-liveness fact."""
        return self._heartbeat_observed

    def observe_readiness(self, telemetry: Telemetry) -> None:
        """Latch passive telemetry facts without advancing mission policy."""
        if telemetry.heartbeat and not self._heartbeat_observed:
            self._heartbeat_observed = True
            self._emit("heartbeat_observed", telemetry.timestamp_ns, {})
        if telemetry.prearm_checks_healthy is True and not self._prearm_checks_healthy:
            self._prearm_checks_healthy = True
            self._emit("prearm_checks_healthy", telemetry.timestamp_ns, {})

    def begin_mission(self, timestamp_ns: int) -> None:
        """Start from latched passive readiness at the exact public epoch."""
        if timestamp_ns != 0:
            raise ValueError("mission must begin at public simulation time zero")
        if not self.mission_ready:
            raise RuntimeError("mission cannot begin before passive readiness")
        if self.state.phase is not MissionPhase.WAIT_HEARTBEAT:
            return
        self.consume(
            Telemetry(
                timestamp_ns=timestamp_ns,
                heartbeat=True,
                prearm_checks_healthy=True,
            )
        )

    def consume(self, telemetry: Telemetry) -> None:
        previous = self.state
        if previous.phase in {MissionPhase.FAILED, MissionPhase.LANDED}:
            return
        if telemetry.status_text is not None:
            self._emit(
                "mavlink_status_text",
                telemetry.timestamp_ns,
                {
                    "status_severity": telemetry.status_severity,
                    "text": telemetry.status_text,
                },
            )
        self.observe_readiness(telemetry)
        transition = advance(previous, telemetry)
        for command in transition.commands:
            self._vehicle.send(command.kind, command.altitude_m)
            self._command_delivered(command.kind, command.timestamp_ns)
        self.state = transition.state
        if (
            previous.phase is MissionPhase.WAIT_GUIDED_MODE
            and transition.state.phase is MissionPhase.WAIT_PREARM_READY
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
    public_clock_observed: bool,
) -> None:
    """Separate pre-clock readiness from RUNNING mission transitions."""
    if mission_policy_active(
        mission_running=mission_running,
        public_clock_observed=public_clock_observed,
    ):
        controller.consume(telemetry)
    else:
        controller.observe_readiness(telemetry)


__all__ = [
    "MissionController",
    "VehicleCommands",
    "mission_policy_active",
    "process_telemetry",
]
