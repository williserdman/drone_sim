"""Scenario publication, observability, and quiescence boundary."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime, timezone
from decimal import Decimal
from threading import Event, Lock, RLock
from typing import Any, Protocol, TextIO

from artifacts.structured_log import StructuredEvent, write_event

from .scenario import InactiveScenarioEvent, ScenarioPolicy
from .payload import PayloadAuthority, PayloadRequest, PayloadWorld


_PHYSICAL_HISTORY_HORIZON_NS = 500_000_000


@dataclass(frozen=True)
class PhysicalResult:
    command_id: str
    status: str
    state: str
    code: str


def parse_physical_result(value: object) -> PhysicalResult:
    if not isinstance(value, str):
        raise ValueError("physical result must use payload-result-v1")
    parts = value.split("|")
    if len(parts) != 5 or parts[0] != "payload-result-v1":
        raise ValueError("physical result must use payload-result-v1")
    _, command_id, status, state, code = parts
    if (
        not command_id
        or status not in {"confirmed", "error"}
        or state not in {"attached", "detached", "unknown"}
        or not code
    ):
        raise ValueError("physical result must use payload-result-v1")
    if status == "confirmed" and (state == "unknown" or code != "OK"):
        raise ValueError("physical result must use payload-result-v1")
    if status == "error" and state != "unknown":
        raise ValueError("physical result must use payload-result-v1")
    return PhysicalResult(command_id, status, state, code)


@dataclass(frozen=True)
class PayloadResponse:
    accepted: bool
    code: str
    detail: str
    command_id: str
    response_sequence: int


@dataclass(frozen=True)
class PayloadEventRecord:
    run_id: str
    timestamp_ns: int
    event_id: int
    aruco_id: int
    command_id: str
    action: str
    state: str
    code: str


@dataclass(frozen=True)
class _VehicleFact:
    timestamp_ns: int
    xy: tuple[float, float]
    grounded: bool


@dataclass(frozen=True)
class _PayloadFact:
    timestamp_ns: int
    xy: tuple[float, float]
    grounded: bool
    attached: bool


@dataclass
class _PendingResult:
    marker_id: int
    event: Event
    value: PhysicalResult | None = None


class PayloadGateway:
    """Join current Gazebo facts to one physically confirmed command response."""

    def __init__(
        self,
        authority: PayloadAuthority,
        *,
        publish_command: Callable[[int, str], None],
        publish_event: Callable[[PayloadEventRecord], None],
        confirmation_timeout_seconds: float = 5.0,
    ) -> None:
        if confirmation_timeout_seconds <= 0:
            raise ValueError("confirmation timeout must be positive")
        self._authority = authority
        self._payload_ids = authority.payload_ids
        self._publish_command = publish_command
        self._publish_event = publish_event
        self._confirmation_timeout_seconds = confirmation_timeout_seconds
        self._vehicle: _VehicleFact | None = None
        self._payloads: dict[int, _PayloadFact] = {}
        self._vehicle_history: dict[int, _VehicleFact] = {}
        self._payload_histories: dict[int, dict[int, _PayloadFact]] = {
            marker: {} for marker in self._payload_ids
        }
        self._pending: dict[str, _PendingResult] = {}
        self._responses: dict[str, tuple[PayloadRequest, PayloadResponse]] = {}
        self._response_sequence = 0
        self._event_id = 0
        self._lock = RLock()
        self._operation_lock = Lock()

    @property
    def attached_id(self) -> int | None:
        with self._lock:
            attached_id, valid = self._attachment_state_locked()
            return attached_id if valid else None

    def _attachment_state_locked(self) -> tuple[int | None, bool]:
        attached_ids = [
            marker for marker, fact in self._payloads.items() if fact.attached
        ]
        if len(attached_ids) > 1:
            return None, False
        return (attached_ids[0] if attached_ids else None), True

    def accept_vehicle(
        self,
        run_id: str,
        timestamp_ns: int,
        xy: tuple[float, float],
        grounded: bool,
    ) -> None:
        if run_id != self._authority.run_id:
            return
        with self._lock:
            if self._vehicle is not None and timestamp_ns <= self._vehicle.timestamp_ns:
                return
            self._vehicle = _VehicleFact(timestamp_ns, xy, grounded)
            self._vehicle_history[timestamp_ns] = self._vehicle
            self._prune_histories_locked()

    def accept_payload(
        self,
        run_id: str,
        timestamp_ns: int,
        aruco_id: int,
        xy: tuple[float, float],
        grounded: bool,
        attached: bool,
    ) -> None:
        if run_id != self._authority.run_id or aruco_id not in self._payload_ids:
            return
        with self._lock:
            previous = self._payloads.get(aruco_id)
            if previous is not None and timestamp_ns <= previous.timestamp_ns:
                return
            self._payloads[aruco_id] = _PayloadFact(
                timestamp_ns, xy, grounded, attached
            )
            self._payload_histories[aruco_id][timestamp_ns] = self._payloads[aruco_id]
            self._prune_histories_locked()

    def _prune_histories_locked(self) -> None:
        latest_timestamps = [fact.timestamp_ns for fact in self._payloads.values()]
        if self._vehicle is not None:
            latest_timestamps.append(self._vehicle.timestamp_ns)
        if not latest_timestamps:
            return
        cutoff = max(latest_timestamps) - _PHYSICAL_HISTORY_HORIZON_NS
        histories: tuple[dict[int, object], ...] = (
            self._vehicle_history,
            *self._payload_histories.values(),
        )
        for history in histories:
            for timestamp_ns in tuple(history):
                if timestamp_ns < cutoff:
                    del history[timestamp_ns]

    def accept_result(self, aruco_id: int, wire: object) -> None:
        result = parse_physical_result(wire)
        with self._lock:
            pending = self._pending.get(result.command_id)
            if pending is None or pending.marker_id != aruco_id or pending.value is not None:
                return
            pending.value = result
            pending.event.set()

    def ready(
        self,
        *,
        result_publishers: frozenset[int],
        service_ready: bool,
    ) -> bool:
        with self._lock:
            timestamps = {
                fact.timestamp_ns for fact in self._payloads.values()
            }
            if self._vehicle is not None:
                timestamps.add(self._vehicle.timestamp_ns)
            _, attachment_valid = self._attachment_state_locked()
            return (
                self._vehicle is not None
                and set(self._payloads) == self._payload_ids
                and len(timestamps) == 1
                and attachment_valid
                and result_publishers == self._payload_ids
                and service_ready
            )

    def _response(
        self,
        request: PayloadRequest,
        *,
        accepted: bool,
        code: str,
        detail: str,
        cache: bool = True,
    ) -> PayloadResponse:
        with self._lock:
            self._response_sequence += 1
            response = PayloadResponse(
                accepted,
                code,
                detail,
                request.command_id,
                self._response_sequence,
            )
            if cache:
                self._responses[request.command_id] = (request, response)
            return response

    def _snapshot(
        self, request: PayloadRequest
    ) -> tuple[PayloadWorld | None, str | None]:
        with self._lock:
            if self._vehicle is None or set(self._payloads) != self._payload_ids:
                return None, "NOT_READY"
            common_timestamps = set(self._vehicle_history)
            for history in self._payload_histories.values():
                common_timestamps.intersection_update(history)
            if not common_timestamps:
                return None, "STALE_PHYSICAL_STATE"
            timestamp_ns = max(common_timestamps)
            vehicle = self._vehicle_history[timestamp_ns]
            payloads = {
                marker: history[timestamp_ns]
                for marker, history in self._payload_histories.items()
            }
            attached_ids = [
                marker for marker, fact in payloads.items() if fact.attached
            ]
            attached_id = attached_ids[0] if len(attached_ids) == 1 else None
            attachment_valid = len(attached_ids) <= 1
            if not attachment_valid:
                return None, "INVALID_PHYSICAL_STATE"
            current_attached_id, current_attachment_valid = (
                self._attachment_state_locked()
            )
            if not current_attachment_valid:
                return None, "INVALID_PHYSICAL_STATE"
            if (
                current_attached_id != attached_id
                or self._vehicle.grounded != vehicle.grounded
                or any(
                    self._payloads[marker].grounded != fact.grounded
                    or self._payloads[marker].attached != fact.attached
                    for marker, fact in payloads.items()
                )
            ):
                return None, "STALE_PHYSICAL_STATE"
            payload = payloads.get(request.aruco_id)
            if payload is None:
                payload = _PayloadFact(0, (0.0, 0.0), False, False)
            return (
                PayloadWorld(
                    vehicle_xy=vehicle.xy,
                    vehicle_grounded=vehicle.grounded,
                    payload_xy=payload.xy,
                    payload_grounded=payload.grounded,
                    attached_id=attached_id,
                ),
                None,
            )

    def execute(self, request: PayloadRequest) -> PayloadResponse:
        with self._operation_lock:
            return self._execute_serialized(request)

    def _execute_serialized(self, request: PayloadRequest) -> PayloadResponse:
        with self._lock:
            previous = self._responses.get(request.command_id)
            if previous is not None:
                if previous[0] == request:
                    return previous[1]
                return self._response(
                    request,
                    accepted=False,
                    code="COMMAND_ID_CONFLICT",
                    detail="command_id was already used by a different request",
                    cache=False,
                )

        world, snapshot_error = self._snapshot(request)
        if world is None:
            assert snapshot_error is not None
            return self._response(
                request,
                accepted=False,
                code=snapshot_error,
                detail="current vehicle and payload truth is incomplete or inconsistent",
            )
        decision = self._authority.decide(world, request)
        if decision.wire_command is None:
            return self._response(
                request,
                accepted=decision.accepted,
                code=decision.code,
                detail=decision.code,
            )

        pending = _PendingResult(request.aruco_id, Event())
        with self._lock:
            self._pending[request.command_id] = pending
        try:
            self._publish_command(request.aruco_id, decision.wire_command)
        except Exception:
            with self._lock:
                self._pending.pop(request.command_id, None)
            return self._response(
                request,
                accepted=False,
                code="COORDINATOR_PUBLISH_FAILED",
                detail="coordinator command could not be published",
            )

        confirmed = pending.event.wait(self._confirmation_timeout_seconds)
        with self._lock:
            self._pending.pop(request.command_id, None)
            physical = pending.value
        if not confirmed or physical is None:
            return self._response(
                request,
                accepted=False,
                code="PHYSICAL_CONFIRMATION_TIMEOUT",
                detail="no matching physical confirmation within five wall seconds",
            )

        expected_state = "attached" if request.action == "attach" else "detached"
        accepted = (
            physical.status == "confirmed"
            and physical.state == expected_state
            and physical.code == "OK"
        )
        code = (
            "OK"
            if accepted
            else physical.code
            if physical.status == "error"
            else "PHYSICAL_CONFIRMATION_MISMATCH"
        )
        if not accepted:
            return self._response(
                request,
                accepted=False,
                code=code,
                detail="Gazebo rejected the physical payload command",
            )

        with self._lock:
            fact = self._payloads[request.aruco_id]
            self._payloads[request.aruco_id] = _PayloadFact(
                timestamp_ns=fact.timestamp_ns,
                xy=fact.xy,
                grounded=fact.grounded,
                attached=request.action == "attach",
            )
            self._payload_histories[request.aruco_id][fact.timestamp_ns] = (
                self._payloads[request.aruco_id]
            )
            timestamp_ns = max(
                self._vehicle.timestamp_ns if self._vehicle is not None else 0,
                self._payloads[request.aruco_id].timestamp_ns,
            )
            event = PayloadEventRecord(
                run_id=self._authority.run_id,
                timestamp_ns=timestamp_ns,
                event_id=self._event_id,
                aruco_id=request.aruco_id,
                command_id=request.command_id,
                action=request.action,
                state=physical.state,
                code=physical.code,
            )
            self._event_id += 1
        response = self._response(
            request,
            accepted=True,
            code="OK",
            detail=f"physical payload {physical.state}",
        )
        self._publish_event(event)
        return response


class QuiescenceProtocol(Protocol):
    def write_quiescence(self, module: str) -> Any: ...


class ScenarioController:
    def __init__(
        self,
        *,
        run_id: str,
        policy: ScenarioPolicy | None,
        publish: Callable[[InactiveScenarioEvent], None],
        protocol: QuiescenceProtocol,
        stream: TextIO,
        scenario: str = "descent_v1",
    ) -> None:
        self._run_id = run_id
        self._policy = policy
        self._publish = publish
        self._protocol = protocol
        self._stream = stream
        self._scenario = scenario
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
            self._emit(
                "ready",
                None,
                {
                    "scenario": self._scenario,
                    "physical_force": self._scenario
                    in {"competition_v1", "search_delivery_v1"},
                },
            )
            self._ready = True

    def observe_clock(self, timestamp_ns: int) -> None:
        if self._quiescent:
            return
        if self._policy is None:
            raise RuntimeError("competition payload authority has no inactive clock policy")
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


__all__ = [
    "PayloadEventRecord",
    "PayloadGateway",
    "PayloadResponse",
    "PhysicalResult",
    "ScenarioController",
    "parse_physical_result",
]
