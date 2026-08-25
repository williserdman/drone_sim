"""Durable companion readiness, terminal status, and quiescence boundary."""

from __future__ import annotations

from datetime import datetime, timezone
from decimal import Decimal
from typing import Any, Protocol, TextIO

from artifacts.structured_log import StructuredEvent, write_event

from .mission import MissionPhase, MissionState


class LifecycleProtocol(Protocol):
    def write_status(self, name: str, document: dict[str, object]) -> Any: ...

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
        document: dict[str, object] = {
            "run_id": self._run_id,
            "ready": True,
            "mavlink_endpoint": "tcp://ardupilot-sitl:5760",
            "mavlink_transport_connected": True,
        }
        self._protocol.write_status("companion-ready", document)
        self.emit(
            "ready",
            None,
            {
                "mavlink_endpoint": document["mavlink_endpoint"],
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
        self._protocol.write_status(
            "mission-ready",
            {
                "run_id": self._run_id,
                "ready": True,
                "heartbeat_observed": True,
                "prearm_checks_healthy": True,
            },
        )
        self.emit("mission_ready", None, {})
        self._mission_ready = True

    def observe_terminal(self, state: MissionState) -> None:
        if self._terminal or state.phase not in {MissionPhase.LANDED, MissionPhase.FAILED}:
            return
        timestamp_ns = state.last_timestamp_ns or 0
        if state.phase is MissionPhase.LANDED:
            self._protocol.write_status(
                "mission-finished",
                {
                    "run_id": self._run_id,
                    "finished": True,
                    "sim_timestamp_ns": timestamp_ns,
                    "outcome": "LANDED",
                },
            )
            self.emit("mission_finished", timestamp_ns, {"outcome": "LANDED"})
        else:
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
