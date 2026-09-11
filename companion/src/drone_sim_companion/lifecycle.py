"""Durable companion readiness, terminal status, and quiescence boundary."""

from __future__ import annotations

from datetime import datetime, timezone
from decimal import Decimal
from typing import Any, Protocol, TextIO

from artifacts.structured_log import StructuredEvent, write_event
from artifacts.runtime_status import (
    CompanionReadyStatus,
    MissionCommandDeliveredStatus,
    MissionFinishedStatus,
    MissionReadyStatus,
    RuntimeFailureStatus,
    RuntimeStatus,
)

from .mission import CommandKind, MissionPhase, MissionState


INITIAL_COMMAND_WINDOW_NS = 50_000_000


class LifecycleProtocol(Protocol):
    def write_status(self, status: RuntimeStatus) -> Any: ...

    def write_quiescence(self, module: str) -> Any: ...


class CompanionLifecycle:
    def __init__(self, *, run_id: str, protocol: LifecycleProtocol, stream: TextIO) -> None:
        self._run_id = run_id
        self._protocol = protocol
        self._stream = stream
        self._ready = False
        self._mission_ready = False
        self._terminal = False
        self._quiescent = False

    def emit(self, event: str, timestamp_ns: int | None, fields: dict[str, object]) -> None:
        if self._quiescent:
            return
        write_event(
            self._stream,
            StructuredEvent(
                run_id=self._run_id,
                module="companion",
                severity="ERROR" if event == "mission_failed" else "INFO",
                event=event,
                sim_timestamp=(Decimal(timestamp_ns) / Decimal(1_000_000_000))
                if timestamp_ns is not None
                else None,
                wall_timestamp=datetime.now(timezone.utc),
                fields=fields,
            ),
        )

    def mark_transport_ready(self) -> None:
        if self._ready:
            return
        self._protocol.write_status(CompanionReadyStatus(self._run_id))
        self.emit(
            "ready",
            None,
            {
                "mavlink_endpoint": "tcp://ardupilot-sitl:5760",
                "mavlink_transport_connected": True,
            },
        )
        self._ready = True

    def observe_mission_readiness(
        self,
        *,
        heartbeat_observed: bool,
        prearm_checks_healthy: bool,
    ) -> None:
        if self._mission_ready or not (heartbeat_observed and prearm_checks_healthy):
            return
        self._protocol.write_status(MissionReadyStatus(self._run_id))
        self.emit("mission_ready", None, {})
        self._mission_ready = True

    def observe_command_delivery(self, command: CommandKind, timestamp_ns: int) -> None:
        if (
            command is not CommandKind.SET_GUIDED
            or not 0 <= timestamp_ns <= INITIAL_COMMAND_WINDOW_NS
        ):
            return
        self._protocol.write_status(
            MissionCommandDeliveredStatus(self._run_id, timestamp_ns)
        )

    def observe_terminal(self, state: MissionState) -> None:
        if self._terminal or state.phase not in {MissionPhase.LANDED, MissionPhase.FAILED}:
            return
        timestamp_ns = state.last_timestamp_ns or 0
        if state.phase is MissionPhase.LANDED:
            self._protocol.write_status(MissionFinishedStatus(self._run_id, timestamp_ns))
            self.emit("mission_finished", timestamp_ns, {"outcome": "LANDED"})
        else:
            self._protocol.write_status(
                RuntimeFailureStatus(
                    self._run_id,
                    "companion",
                    state.failure_reason,
                    ("logs/docker/companion.log.partial",),
                )
            )
            self.emit(
                "mission_failed",
                timestamp_ns,
                {"reason": state.failure_reason},
            )
        self._terminal = True

    def finalize(self, timestamp_ns: int | None) -> None:
        if self._quiescent:
            return
        self.emit("finalizing", timestamp_ns, {})
        self._quiescent = True
        self._protocol.write_quiescence("companion")


__all__ = ["CompanionLifecycle", "LifecycleProtocol"]
