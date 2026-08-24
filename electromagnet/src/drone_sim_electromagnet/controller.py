"""Scenario publication, observability, and quiescence boundary."""

from __future__ import annotations

from collections.abc import Callable
from datetime import datetime, timezone
from decimal import Decimal
from typing import Any, Protocol, TextIO

from artifacts.structured_log import StructuredEvent, write_event

from .scenario import InactiveScenarioEvent, ScenarioPolicy


class QuiescenceProtocol(Protocol):
    def write_quiescence(self, module: str) -> Any: ...


class ScenarioController:
    def __init__(
        self,
        *,
        run_id: str,
        policy: ScenarioPolicy,
        publish: Callable[[InactiveScenarioEvent], None],
        protocol: QuiescenceProtocol,
        stream: TextIO,
    ) -> None:
        self._run_id = run_id
        self._policy = policy
        self._publish = publish
        self._protocol = protocol
        self._stream = stream
        self._ready = False
        self._quiescent = False

    def _emit(
        self,
        event: str,
        timestamp_ns: int | None,
        fields: dict[str, object],
        *,
        severity: str = "INFO",
    ) -> None:
        if self._quiescent:
            return
        write_event(
            self._stream,
            StructuredEvent(
                run_id=self._run_id,
                module="electromagnet",
                severity=severity,
                event=event,
                sim_timestamp=(Decimal(timestamp_ns) / Decimal(1_000_000_000))
                if timestamp_ns is not None
                else None,
                wall_timestamp=datetime.now(timezone.utc),
                fields=fields,
            ),
        )

    def mark_ready(self) -> None:
        if not self._ready:
            self._emit("ready", None, {"scenario": "descent_v1", "physical_force": False})
            self._ready = True

    def observe_clock(self, timestamp_ns: int) -> None:
        if self._quiescent:
            return
        event = self._policy.observe_clock(timestamp_ns)
        if event is None:
            return
        self._publish(event)
        self._emit(
            "scenario_event_published",
            event.timestamp_ns,
            {
                "event_id": event.event_id,
                "magnet_id": event.magnet_id,
                "state": event.state,
                "physical_force": False,
            },
        )

    def finalize(self, timestamp_ns: int | None) -> None:
        if self._quiescent:
            return
        self._emit("finalizing", timestamp_ns, {})
        self._quiescent = True
        self._protocol.write_quiescence("electromagnet")

    def fail(self, timestamp_ns: int | None, reason: str) -> None:
        self._emit("scenario_failed", timestamp_ns, {"reason": reason}, severity="ERROR")


__all__ = ["ScenarioController"]
