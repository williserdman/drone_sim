"""Nonblocking payload command adapter for configured missions."""

from __future__ import annotations

from dataclasses import dataclass
import re
from typing import Any


_COMMAND_ID = re.compile(r"[A-Za-z0-9_.:-]{1,128}\Z")
_PAYLOAD_IDS = frozenset((2, 3, 4))
_MAX_STATE_AGE_NS = 500_000_000


@dataclass(frozen=True)
class _PayloadState:
    timestamp_ns: int
    attached: bool
    grounded: bool


@dataclass
class _PendingCommand:
    aruco_id: int
    action: str
    command_id: str
    dispatched_at_ns: int
    future: Any
    response_confirmed: bool = False


class ConfiguredPayloadClient:
    """Send one payload request and confirm it from recurrent public truth."""

    def __init__(self, run_id: str, client: object, request_type: type) -> None:
        if not isinstance(run_id, str) or not run_id:
            raise ValueError("payload run_id must be a nonempty string")
        if not callable(getattr(client, "service_is_ready", None)) or not callable(
            getattr(client, "call_async", None)
        ):
            raise TypeError("payload client must provide service_is_ready and call_async")
        if not callable(request_type):
            raise TypeError("payload request type must be callable")
        self._run_id = run_id
        self._client = client
        self._request_type = request_type
        self._states: dict[int, _PayloadState] = {}
        self._pending: _PendingCommand | None = None
        self._attached_id: int | None = None
        self._last_tick_ns = 0

    @property
    def ready(self) -> bool:
        return self._client.service_is_ready() is True

    @property
    def attached_id(self) -> int | None:
        return self._attached_id

    def observe(self, message: object) -> None:
        if getattr(message, "run_id", None) != self._run_id:
            raise RuntimeError("payload state run_id does not match this run")
        aruco_id = getattr(message, "aruco_id", None)
        if type(aruco_id) is not int or aruco_id not in _PAYLOAD_IDS:
            raise RuntimeError("payload state marker must be integer 2, 3, or 4")
        attached = getattr(message, "attached", None)
        grounded = getattr(message, "grounded", None)
        if type(attached) is not bool or type(grounded) is not bool:
            raise RuntimeError("payload attached and grounded state must be boolean")
        stamp = getattr(message, "sim_timestamp", None)
        sec = getattr(stamp, "sec", None)
        nanosec = getattr(stamp, "nanosec", None)
        if (
            type(sec) is not int
            or sec < 0
            or type(nanosec) is not int
            or not 0 <= nanosec < 1_000_000_000
        ):
            raise RuntimeError("payload state source timestamp is malformed")
        timestamp_ns = sec * 1_000_000_000 + nanosec
        previous = self._states.get(aruco_id)
        if previous is not None and timestamp_ns <= previous.timestamp_ns:
            raise RuntimeError("payload state source timestamps must increase")
        self._states[aruco_id] = _PayloadState(
            timestamp_ns=timestamp_ns,
            attached=attached,
            grounded=grounded,
        )

        pending = self._pending
        if pending is not None and pending.aruco_id == aruco_id:
            return
        if attached:
            self._attached_id = aruco_id
        elif self._attached_id == aruco_id:
            self._attached_id = None

    def start(
        self,
        aruco_id: int,
        action: str,
        operation_id: str,
        timestamp_ns: int,
    ) -> None:
        if self._pending is not None:
            raise RuntimeError("another payload action is already pending")
        if type(aruco_id) is not int or aruco_id not in _PAYLOAD_IDS:
            raise ValueError("payload marker must be integer 2, 3, or 4")
        if action not in {"ATTACH", "RELEASE"}:
            raise ValueError("payload action must be ATTACH or RELEASE")
        if action == "ATTACH" and aruco_id == 2:
            raise ValueError("payload 2 starts attached and cannot be attached")
        if type(timestamp_ns) is not int or timestamp_ns < 0:
            raise ValueError("payload dispatch timestamp must be a nonnegative integer")
        if not isinstance(operation_id, str) or not operation_id:
            raise ValueError("payload operation_id must be a nonempty string")
        command_id = f"{self._run_id}:configured:{operation_id}"
        if _COMMAND_ID.fullmatch(command_id) is None:
            raise ValueError("payload operation_id cannot form a valid command_id")
        if not self.ready:
            raise RuntimeError("payload service is not ready")

        request = self._request_type()
        request.run_id = self._run_id
        request.aruco_id = aruco_id
        request.action = getattr(self._request_type, action)
        request.command_id = command_id
        future = self._client.call_async(request)
        if not callable(getattr(future, "done", None)) or not callable(
            getattr(future, "result", None)
        ):
            cancel = getattr(future, "cancel", None)
            if callable(cancel):
                cancel()
            raise RuntimeError("payload client returned an invalid future")
        self._pending = _PendingCommand(
            aruco_id=aruco_id,
            action=action,
            command_id=command_id,
            dispatched_at_ns=timestamp_ns,
            future=future,
        )

    def tick(self, timestamp_ns: int) -> bool:
        if type(timestamp_ns) is not int or timestamp_ns < self._last_tick_ns:
            raise RuntimeError("payload tick timestamp regressed or is malformed")
        self._last_tick_ns = timestamp_ns
        pending = self._pending
        if pending is None:
            return False

        if not pending.response_confirmed and pending.future.done():
            try:
                response = pending.future.result()
            except BaseException as error:
                self._clear_pending()
                raise RuntimeError("payload response failed") from error
            self._validate_response(pending, response)
            pending.response_confirmed = True

        state = self._states.get(pending.aruco_id)
        if state is not None and timestamp_ns - state.timestamp_ns > _MAX_STATE_AGE_NS:
            self._clear_pending()
            raise RuntimeError("payload public state became stale")
        expected_attached = pending.action == "ATTACH"
        if (
            pending.response_confirmed
            and state is not None
            and pending.dispatched_at_ns < state.timestamp_ns <= timestamp_ns
            and state.attached is expected_attached
        ):
            completed = self._clear_pending()
            if expected_attached:
                self._attached_id = completed.aruco_id
            elif self._attached_id == completed.aruco_id:
                self._attached_id = None
            return True
        return False

    def cancel(self) -> None:
        pending = self._pending
        if pending is None:
            return
        self._pending = None
        cancel = getattr(pending.future, "cancel", None)
        if callable(cancel):
            cancel()

    def _validate_response(
        self, pending: _PendingCommand, response: object
    ) -> None:
        if response is None:
            self._clear_pending()
            raise RuntimeError("payload response was missing")
        if getattr(response, "command_id", None) != pending.command_id:
            self._clear_pending()
            raise RuntimeError("payload response command_id did not match")
        sequence = getattr(response, "response_sequence", None)
        if type(sequence) is not int or sequence <= 0:
            self._clear_pending()
            raise RuntimeError("payload response sequence was invalid")
        accepted = getattr(response, "accepted", None)
        code = getattr(response, "code", None)
        if accepted is False:
            self._clear_pending()
            detail = getattr(response, "detail", "")
            raise RuntimeError(f"payload response rejected: {code}: {detail}")
        if accepted is not True:
            self._clear_pending()
            raise RuntimeError("payload response accepted field was invalid")
        if code != "OK":
            self._clear_pending()
            raise RuntimeError("payload response code was not OK")

    def _clear_pending(self) -> _PendingCommand:
        pending = self._pending
        if pending is None:
            raise RuntimeError("payload action is not pending")
        self._pending = None
        return pending


__all__ = ["ConfiguredPayloadClient"]
