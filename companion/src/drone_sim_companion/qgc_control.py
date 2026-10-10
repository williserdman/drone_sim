"""Single-owner QGroundControl command admission and recovery evidence."""

from __future__ import annotations

from collections import deque
import math
from threading import Lock
import time
from typing import Callable, Protocol

from artifacts.operator_protocol import (
    FlightAbortRequest,
    MissionStartRequest,
    OperatorProtocol,
    OperatorProtocolError,
)
from artifacts.qgc_operator import (
    MAV_RESULT_ACCEPTED,
    MAV_RESULT_CANCELLED,
    MAV_RESULT_DENIED,
    MAV_RESULT_FAILED,
    MAV_RESULT_IN_PROGRESS,
    MAV_RESULT_TEMPORARILY_REJECTED,
    MAV_RESULT_UNSUPPORTED,
    QgcCommandEnvelope,
    QgcCommandResponse,
    QgcCommandStatus,
    QgcOperatorDescriptor,
)

from .operator_control import CompanionOperatorControl, RunningStateBinding, StartDecision


_INGRESS_LIMIT = 64
_PROGRESS_INTERVAL_NS = 1_000_000_000


class QgcCommandIO(Protocol):
    """Transport output paired with the transport's single receive callback."""

    def install(self, receive: Callable[[QgcCommandEnvelope], None]) -> None: ...
    def send_ack(self, response: QgcCommandResponse) -> None: ...


def _status_text(value: str) -> str:
    encoded = value.encode("utf-8")
    if len(encoded) <= 50:
        return value
    return encoded[:50].decode("utf-8", errors="ignore") or "QGC command rejected"


class QgcMissionControl:
    """Serialize QGC command callbacks onto one companion runtime owner."""

    def __init__(
        self,
        protocol: OperatorProtocol,
        operator: CompanionOperatorControl,
        descriptor: QgcOperatorDescriptor,
        io: QgcCommandIO,
        readiness_reasons: Callable[[], tuple[str, ...]],
        is_finalizing: Callable[[], bool],
        *,
        wall_time_ns: Callable[[], int] = time.time_ns,
    ) -> None:
        self._protocol = protocol
        self._operator = operator
        self._descriptor = descriptor
        self._io = io
        self._host_readiness_reasons = readiness_reasons
        self._is_finalizing = is_finalizing
        self._finalizing = False
        self._wall_time_ns = wall_time_ns
        self._lock = Lock()
        self._ingress: deque[QgcCommandEnvelope] = deque()
        self._overflow_command: QgcCommandEnvelope | None = None
        existing = protocol.read_qgc_command_statuses()
        self._sequence = len(existing)
        self._restart_blocked = protocol.read_start_request() is not None
        self._start_envelope: QgcCommandEnvelope | None = None
        self._start_request: MissionStartRequest | None = None
        self._start_state = "restart_blocked" if self._restart_blocked else "idle"
        self._last_start_response: QgcCommandResponse | None = None
        self._last_progress_wall_ns: int | None = None
        self._abort_envelope: QgcCommandEnvelope | None = None
        self._abort_request: FlightAbortRequest | None = None
        self._abort_state = "idle"
        self._last_abort_response: QgcCommandResponse | None = None
        self._last_abort_protocol_state: str | None = None
        self._mission_outcome: str | None = None
        io.install(self.accept)

    def accept(self, command: QgcCommandEnvelope) -> None:
        """Bound callback work to an in-memory enqueue operation."""
        if not isinstance(command, QgcCommandEnvelope):
            raise TypeError("command must be a QgcCommandEnvelope")
        with self._lock:
            if len(self._ingress) >= _INGRESS_LIMIT:
                if self._overflow_command is None:
                    self._overflow_command = command
                return
            self._ingress.append(command)

    def poll_start(self, *, run_state: RunningStateBinding | None) -> StartDecision:
        self._finalization_active()
        self._drain()
        if self._finalization_active():
            if self._start_state == "pending":
                self._cancel_pending_start("simulation finalization cancelled Start")
            return StartDecision(False)
        if self._start_state == "admitted":
            decision = self._operator.poll_start(run_state=run_state)
            return StartDecision(
                decision.command_allowed,
                admitted_now=False,
                admission_sha256=decision.admission_sha256,
            )
        if self._start_state != "pending":
            return StartDecision(False)
        reasons = self._readiness_reasons()
        if reasons:
            self._pending_progress("Start pending: " + reasons[0])
            return StartDecision(False)
        try:
            decision = self._operator.poll_start(run_state=run_state)
        except OperatorProtocolError as error:
            self._start_state = "failed"
            self._respond_start(MAV_RESULT_FAILED, 100, f"Start failed: {error}")
            return StartDecision(False)
        if not decision.command_allowed:
            self._pending_progress("Start is waiting for admission")
            return decision
        self._start_state = "admitted"
        self._respond_start(
            MAV_RESULT_ACCEPTED,
            100,
            "Start admitted; mission outcome remains pending",
            admission_sha256=decision.admission_sha256,
        )
        return decision

    def poll_abort(
        self,
        *,
        finalizing: bool,
        begin_recovery: Callable[[str], bool],
    ) -> None:
        effective_finalizing = self._finalization_active(finalizing)
        self._drain()
        if self._abort_state != "pending":
            return
        self._operator.poll_abort(
            finalizing=effective_finalizing,
            begin_recovery=begin_recovery,
        )
        latest = self._protocol.read_latest_flight_abort_status()
        if latest is None or latest.state == self._last_abort_protocol_state:
            return
        self._last_abort_protocol_state = latest.state
        if latest.state in {"accepted", "recovering"}:
            self._respond_abort(MAV_RESULT_IN_PROGRESS, 50, latest.detail)
        elif latest.state == "landed_disarmed":
            self._abort_state = "complete"
            self._respond_abort(
                MAV_RESULT_ACCEPTED,
                100,
                "Abort recovery confirmed landed and disarmed",
                observed_outcome="landed_disarmed",
            )
        elif latest.state == "interrupted":
            self._abort_state = "complete"
            self._respond_abort(
                MAV_RESULT_CANCELLED,
                100,
                latest.detail,
                observed_outcome="interrupted",
            )
        elif latest.state == "rejected":
            self._abort_state = "complete"
            self._respond_abort(
                MAV_RESULT_DENIED,
                100,
                latest.detail,
                observed_outcome="rejected",
            )
        else:
            self._abort_state = "complete"
            self._respond_abort(
                MAV_RESULT_FAILED,
                100,
                latest.detail,
                observed_outcome="failed",
            )

    def observe_mission_terminal(self, outcome: str) -> None:
        """Attach an observed terminal mission result without changing Start ACK."""
        if not isinstance(outcome, str) or not outcome:
            raise ValueError("outcome must be nonempty")
        previous = self._mission_outcome
        self._mission_outcome = outcome
        if self._start_state != "admitted" or outcome == previous:
            return
        assert self._start_envelope is not None
        assert self._last_start_response is not None
        self._record(
            self._start_envelope,
            self._last_start_response,
            request=self._start_request,
            admission_sha256=self._operator.admission_sha256,
            observed_outcome=outcome,
            send=False,
        )

    def _drain(self) -> None:
        with self._lock:
            commands = tuple(self._ingress)
            self._ingress.clear()
            overflow = self._overflow_command
            self._overflow_command = None
        for command in commands:
            rejection = self._validate(command)
            if rejection is not None:
                result, detail = rejection
                self._record(
                    command,
                    QgcCommandResponse(
                        command.command,
                        command.source,
                        result,
                        100,
                        _status_text(detail),
                    ),
                )
            elif command.command == self._descriptor.start_command:
                self._handle_start(command)
            else:
                self._handle_abort(command)
        if overflow is not None:
            rejection = self._validate(overflow)
            result, progress, detail = (
                (rejection[0], 100, rejection[1])
                if rejection is not None
                else (
                    MAV_RESULT_TEMPORARILY_REJECTED,
                    0,
                    "QGC command ingress is busy; retry",
                )
            )
            self._record(
                overflow,
                QgcCommandResponse(
                    overflow.command,
                    overflow.source,
                    result,
                    progress,
                    _status_text(detail),
                ),
            )

    def _validate(self, command: QgcCommandEnvelope) -> tuple[int, str] | None:
        if command.wire_version != 2:
            return MAV_RESULT_DENIED, "QGC command requires MAVLink 2"
        if command.source != self._descriptor.qgc_source:
            return MAV_RESULT_DENIED, "QGC command source identity is not authorized"
        if command.target != self._descriptor.companion:
            return MAV_RESULT_DENIED, "QGC command target must be companion 1/191"
        if command.command not in {
            self._descriptor.start_command,
            self._descriptor.abort_command,
        }:
            return MAV_RESULT_UNSUPPORTED, "QGC command is unsupported"
        token = command.parameters[0]
        if (
            isinstance(token, bool)
            or not isinstance(token, (int, float))
            or not math.isfinite(token)
            or token != int(token)
            or int(token) != self._descriptor.attempt_token
        ):
            return MAV_RESULT_DENIED, "QGC command attempt token is invalid or stale"
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value != 0
            for value in command.parameters[1:]
        ):
            return MAV_RESULT_DENIED, "QGC command parameters 2 through 7 must be zero"
        return None

    def _handle_start(self, command: QgcCommandEnvelope) -> None:
        if self._start_state == "admitted":
            assert self._last_start_response is not None
            self._record(
                command,
                self._last_start_response,
                request=self._start_request,
                admission_sha256=self._operator.admission_sha256,
            )
            return
        if self._start_state == "pending":
            self._respond_start(
                MAV_RESULT_IN_PROGRESS,
                0,
                "Start is pending durable admission",
                envelope=command,
            )
            return
        if self._start_state == "cancelled":
            self._record(
                command,
                QgcCommandResponse(
                    command.command,
                    command.source,
                    MAV_RESULT_CANCELLED,
                    100,
                    "Start was cancelled; use a new Prepare",
                ),
                request=self._start_request,
                observed_outcome="pending_start_cancelled",
            )
            return
        if self._restart_blocked or self._start_state == "failed":
            self._record(
                command,
                QgcCommandResponse(
                    command.command,
                    command.source,
                    MAV_RESULT_DENIED,
                    100,
                    "Reserved Start cannot resume; use a new Prepare",
                ),
            )
            return
        if self._mission_outcome is not None:
            self._record(
                command,
                QgcCommandResponse(
                    command.command,
                    command.source,
                    MAV_RESULT_DENIED,
                    100,
                    "Terminal run cannot accept Start",
                ),
                observed_outcome=self._mission_outcome,
            )
            return
        if self._finalization_active():
            self._record(
                command,
                QgcCommandResponse(
                    command.command,
                    command.source,
                    MAV_RESULT_DENIED,
                    100,
                    "Simulation finalization blocks Start",
                ),
            )
            return
        reasons = self._readiness_reasons()
        if reasons:
            self._record(
                command,
                QgcCommandResponse(
                    command.command,
                    command.source,
                    MAV_RESULT_TEMPORARILY_REJECTED,
                    0,
                    _status_text("Start unavailable: " + reasons[0]),
                ),
            )
            return
        self._start_envelope = command
        try:
            self._start_request = self._protocol.write_start_request(
                self._descriptor.start_request_id
            )
        except Exception as error:
            self._start_state = "failed"
            self._record(
                command,
                QgcCommandResponse(
                    command.command,
                    command.source,
                    MAV_RESULT_FAILED,
                    100,
                    _status_text(f"Start reservation failed: {error}"),
                ),
            )
            return
        self._start_state = "pending"
        self._respond_start(
            MAV_RESULT_IN_PROGRESS,
            0,
            "Start reserved; waiting for durable admission",
        )

    def _handle_abort(self, command: QgcCommandEnvelope) -> None:
        if self._abort_state != "idle":
            assert self._last_abort_response is not None
            self._record(
                command,
                self._last_abort_response,
                request=self._abort_request,
                admission_sha256=self._operator.admission_sha256,
            )
            return
        if self._finalization_active():
            self._record(
                command,
                QgcCommandResponse(
                    command.command,
                    command.source,
                    MAV_RESULT_CANCELLED,
                    100,
                    "Simulation finalization blocks Abort recovery",
                ),
                observed_outcome="finalizing",
            )
            return
        if self._start_state == "pending":
            self._abort_envelope = command
            self._cancel_pending_start("QGC Abort cancelled pending Start")
            self._abort_state = "complete"
            self._respond_abort(
                MAV_RESULT_ACCEPTED,
                100,
                "Abort cancelled pending Start before admission",
                observed_outcome="pending_start_cancelled",
            )
            return
        if self._start_state != "admitted" or self._operator.admission_sha256 is None:
            self._record(
                command,
                QgcCommandResponse(
                    command.command,
                    command.source,
                    MAV_RESULT_DENIED,
                    100,
                    "Abort requires an admitted active Start",
                ),
            )
            return
        self._abort_envelope = command
        try:
            self._abort_request = self._protocol.write_flight_abort_request(
                self._descriptor.abort_request_id,
                self._operator.admission_sha256,
                "QGC requested guarded recovery",
            )
        except Exception as error:
            self._abort_state = "complete"
            self._respond_abort(
                MAV_RESULT_FAILED,
                100,
                f"Abort reservation failed: {error}",
                observed_outcome="failed",
            )
            return
        self._abort_state = "pending"
        self._respond_abort(
            MAV_RESULT_IN_PROGRESS,
            0,
            "Abort reserved; guarded recovery is pending",
        )

    def _cancel_pending_start(self, detail: str) -> None:
        if self._start_state != "pending":
            return
        self._start_state = "cancelled"
        self._respond_start(
            MAV_RESULT_CANCELLED,
            100,
            detail,
            observed_outcome="pending_start_cancelled",
        )

    def _pending_progress(self, detail: str) -> None:
        if self._operator.wall_deadline_expired:
            return
        now = self._wall_time_ns()
        if (
            self._last_progress_wall_ns is not None
            and now - self._last_progress_wall_ns < _PROGRESS_INTERVAL_NS
        ):
            return
        self._last_progress_wall_ns = now
        self._respond_start(MAV_RESULT_IN_PROGRESS, 0, detail)

    def _readiness_reasons(self) -> tuple[str, ...]:
        return (*self._operator.qgc_readiness_reasons(), *self._host_readiness_reasons())

    def _finalization_active(self, explicit: bool = False) -> bool:
        if not self._finalizing and (explicit or self._is_finalizing()):
            self._finalizing = True
        return self._finalizing

    def _respond_start(
        self,
        result: int,
        progress: int,
        detail: str,
        *,
        envelope: QgcCommandEnvelope | None = None,
        admission_sha256: str | None = None,
        observed_outcome: str | None = None,
    ) -> None:
        command = envelope or self._start_envelope
        assert command is not None
        response = QgcCommandResponse(
            command.command,
            command.source,
            result,
            progress,
            _status_text(detail),
        )
        self._last_start_response = response
        if result == MAV_RESULT_IN_PROGRESS:
            self._last_progress_wall_ns = self._wall_time_ns()
        self._record(
            command,
            response,
            request=self._start_request,
            admission_sha256=admission_sha256,
            observed_outcome=observed_outcome,
        )

    def _respond_abort(
        self,
        result: int,
        progress: int,
        detail: str,
        *,
        observed_outcome: str | None = None,
    ) -> None:
        assert self._abort_envelope is not None
        response = QgcCommandResponse(
            self._abort_envelope.command,
            self._abort_envelope.source,
            result,
            progress,
            _status_text(detail),
        )
        self._last_abort_response = response
        self._record(
            self._abort_envelope,
            response,
            request=self._abort_request,
            admission_sha256=self._operator.admission_sha256,
            observed_outcome=observed_outcome,
        )

    def _record(
        self,
        envelope: QgcCommandEnvelope,
        response: QgcCommandResponse,
        *,
        request: object | None = None,
        admission_sha256: str | None = None,
        observed_outcome: str | None = None,
        send: bool = True,
    ) -> None:
        request_id = getattr(request, "request_id", None)
        request_sha256 = getattr(request, "sha256", None)
        status = QgcCommandStatus(
            run_id=self._protocol.run_id,
            config_sha256=self._protocol.config_sha256,
            sequence=self._sequence,
            envelope=envelope,
            request_id=request_id,
            request_sha256=request_sha256,
            admission_sha256=admission_sha256,
            responded_wall_time_ns=max(
                self._wall_time_ns(), envelope.received_wall_time_ns
            ),
            response=response,
            observed_outcome=observed_outcome,
        )
        self._protocol.write_qgc_command_status(status)
        self._sequence += 1
        if send:
            self._io.send_ack(response)


__all__ = ["QgcCommandIO", "QgcMissionControl"]
