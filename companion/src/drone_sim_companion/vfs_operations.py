"""Non-flight adapters used by configured VFS operations."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import threading
from typing import Any, Callable

from .configured_vision import RangeObservation
from .mission import Telemetry


@dataclass(frozen=True)
class PayloadConfirmation:
    request_id: str
    confirmed: bool
    reason: str = ""


@dataclass(frozen=True)
class PhaseDecision:
    request_id: str
    phase: str
    checkpoint: str
    approved: bool
    eligible_aruco_ids: tuple[int, ...] = ()
    reason: str = ""
    request_event_id: int = 0


@dataclass(frozen=True)
class VehicleStatusSnapshot:
    timestamp_ns: int
    sequence_id: int
    armed: bool
    mode: str
    failsafe_active: bool
    failsafe_reason: str


class VehicleStatusStream:
    """Sample last observed FC status on the public simulation-time grid."""

    def __init__(self, *, interval_ns: int = 100_000_000) -> None:
        self._interval_ns = interval_ns
        self._mode: str | None = None
        self._armed: bool | None = None
        self._failsafe: bool | None = None
        self._failsafe_reason: str | None = None
        self._last_grid_ns = 0
        self._sequence_id = 0
        self._latest_snapshot: VehicleStatusSnapshot | None = None
        self._first_disarmed_published_ns: int | None = None

    def observe(self, telemetry: Telemetry) -> None:
        if telemetry.mode is not None:
            mode = telemetry.mode.strip().upper()
            if mode:
                self._mode = mode
        if telemetry.armed is not None:
            self._armed = telemetry.armed
            if telemetry.armed:
                self._first_disarmed_published_ns = None
        if telemetry.failsafe is not None:
            reason = (telemetry.failsafe_reason or "").strip()
            if not telemetry.failsafe or reason:
                self._failsafe = telemetry.failsafe
                self._failsafe_reason = reason

    def sample(self, public_clock_ns: int) -> VehicleStatusSnapshot | None:
        grid_ns = (public_clock_ns // self._interval_ns) * self._interval_ns
        if grid_ns <= self._last_grid_ns:
            return None
        if self._mode is None or self._armed is None or self._failsafe is None:
            return None
        self._last_grid_ns = grid_ns
        snapshot = VehicleStatusSnapshot(
            timestamp_ns=grid_ns,
            sequence_id=self._sequence_id,
            armed=self._armed,
            mode=self._mode,
            failsafe_active=self._failsafe,
            failsafe_reason=self._failsafe_reason or "",
        )
        self._sequence_id += 1
        self._latest_snapshot = snapshot
        if snapshot.armed:
            self._first_disarmed_published_ns = None
        elif self._first_disarmed_published_ns is None:
            self._first_disarmed_published_ns = snapshot.timestamp_ns
        return snapshot

    def disarmed_ready(self, public_clock_ns: int) -> bool:
        snapshot = self._latest_snapshot
        first = self._first_disarmed_published_ns
        return (
            self._armed is False
            and snapshot is not None
            and snapshot.armed is False
            and first is not None
            and public_clock_ns - first >= self._interval_ns
            and 0 <= public_clock_ns - snapshot.timestamp_ns <= 500_000_000
        )


class ConfiguredRangefinder:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: RangeObservation | None = None
        self._sequence = 0

    def accept(self, message, timestamp_ns: int) -> None:
        ranges = tuple(message.ranges)
        if len(ranges) != 1:
            raise ValueError("downward range must contain exactly one beam")
        distance = float(ranges[0])
        minimum, maximum = float(message.range_min), float(message.range_max)
        if not math.isfinite(distance) or not minimum <= distance <= maximum:
            raise ValueError("downward range is invalid")
        with self._lock:
            if self._latest is not None and timestamp_ns < self._latest.timestamp_ns:
                raise ValueError("downward range timestamp regressed")
            self._sequence += 1
            self._latest = RangeObservation(self._sequence, timestamp_ns, distance)

    def latest(self, timestamp_ns: int) -> RangeObservation:
        with self._lock:
            observation = self._latest
        if observation is None or not 0 <= timestamp_ns - observation.timestamp_ns <= 500_000_000:
            raise RuntimeError("fresh downward range is unavailable")
        return observation


class PayloadActions:
    def __init__(self, run_id: str, client, service_type) -> None:
        self._run_id = run_id
        self._client = client
        self._service_type = service_type
        self._futures: dict[str, Any] = {}

    def request(self, action: str, aruco_id: int, request_id: str, delivery_mode) -> None:
        del delivery_mode  # The physical authority validates the requested state.
        request = self._service_type.Request()
        request.run_id = self._run_id
        request.aruco_id = aruco_id
        request.action = (
            request.ATTACH if action == "attach" else request.RELEASE
        )
        request.command_id = request_id
        self._futures[request_id] = self._client.call_async(request)

    def confirmation(self, request_id: str) -> PayloadConfirmation | None:
        future = self._futures.get(request_id)
        if future is None or not future.done():
            return None
        response = future.result()
        if response is None or response.command_id != request_id:
            return PayloadConfirmation(request_id, False, "uncorrelated payload response")
        sequence = getattr(response, "response_sequence", None)
        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence <= 0:
            return PayloadConfirmation(request_id, False, "invalid payload response sequence")
        return PayloadConfirmation(
            request_id,
            response.accepted is True,
            "" if response.accepted is True else f"{response.code}: {response.detail}",
        )

    def cancel(self) -> None:
        for future in tuple(self._futures.values()):
            if not future.done():
                future.cancel()


class AttemptMonitor:
    def __init__(self, run_id: str) -> None:
        self.run_id = run_id
        self.active_elapsed_ns = 0
        self.approval_timestamp_ns = 0
        self.state = 0
        self.sequence_id = -1

    def accept(self, message) -> None:
        if message.run_id != self.run_id or message.sequence_id <= self.sequence_id:
            return
        self.sequence_id = message.sequence_id
        self.active_elapsed_ns = message.active_elapsed_ns
        self.approval_timestamp_ns = message.approval_timestamp_ns
        self.state = message.state


class PhaseExchange:
    def __init__(
        self,
        run_id: str,
        publish_event,
        *,
        check_ready: Callable[[int], bool] | None = None,
    ) -> None:
        self.run_id = run_id
        self._publish_event = publish_event
        self._check_ready = check_ready
        self._decisions: dict[str, PhaseDecision] = {}
        self._latest_fm2: PhaseDecision | None = None
        self._requests: dict[str, tuple[str, str, int]] = {}
        self._pending_checks: dict[str, tuple[str, str]] = {}

    def begin(self, phase: str, request_id: str, stamp_ns: int) -> None:
        self._publish_event(phase, "STARTED", request_id, stamp_ns)

    def check(self, phase: str, checkpoint: str, request_id: str, stamp_ns: int) -> None:
        self._pending_checks[request_id] = (phase, checkpoint)
        if self._check_ready is None:
            self._emit_check(request_id, stamp_ns)
        else:
            self.advance(stamp_ns)

    def advance(self, stamp_ns: int) -> None:
        if self._check_ready is not None and not self._check_ready(stamp_ns):
            return
        for request_id in tuple(self._pending_checks):
            self._emit_check(request_id, stamp_ns)

    def _emit_check(self, request_id: str, stamp_ns: int) -> None:
        phase, checkpoint = self._pending_checks[request_id]
        detail = json.dumps(
            {"checkpoint": checkpoint, "request_id": request_id},
            sort_keys=True,
            separators=(",", ":"),
        )
        event_id = self._publish_event(phase, "CHECK_REQUESTED", detail, stamp_ns)
        if isinstance(event_id, bool) or not isinstance(event_id, int):
            raise RuntimeError("phase publisher did not return an event ID")
        self._requests[request_id] = (phase, checkpoint, event_id)
        del self._pending_checks[request_id]

    def accept(self, message) -> None:
        if message.run_id != self.run_id or message.outcome not in (
            message.APPROVED, message.REJECTED, message.ERROR
        ):
            return
        expected = self._requests.get(message.request_id)
        if expected != (message.phase, message.checkpoint, message.request_event_id):
            return
        decision = PhaseDecision(
            request_id=message.request_id,
            phase=message.phase,
            checkpoint=message.checkpoint,
            approved=message.outcome == message.APPROVED,
            eligible_aruco_ids=tuple(int(value) for value in message.eligible_aruco_ids),
            reason=message.reason_code,
            request_event_id=message.request_event_id,
        )
        self._decisions.setdefault(decision.request_id, decision)
        if decision.phase == "FM2" and decision.approved:
            self._latest_fm2 = decision

    def decision(self, request_id: str) -> PhaseDecision | None:
        return self._decisions.get(request_id)

    def evaluate(self, condition):
        from .mission_plan import ConditionResult

        decision = self._latest_fm2
        if decision is None:
            raise RuntimeError("FM2 eligibility decision is unavailable")
        eligible = set(decision.eligible_aruco_ids)
        if condition.kind == "any_payload_eligible":
            matched = bool(eligible)
        else:
            matched = condition.payload_id in eligible
        return ConditionResult(
            matched,
            "eligible payload selected" if matched else "payload was not eligible",
        )


__all__ = [
    "AttemptMonitor",
    "ConfiguredRangefinder",
    "PayloadActions",
    "PayloadConfirmation",
    "PhaseDecision",
    "PhaseExchange",
    "VehicleStatusSnapshot",
    "VehicleStatusStream",
]
