"""Irreversible flight-output authority for QGC-supervised mission hosts."""

from __future__ import annotations

from contextlib import contextmanager
from contextlib import ExitStack
from dataclasses import dataclass
import re
import threading
from typing import Any, Iterator

from .mission import Telemetry


_SHA256 = re.compile(r"[0-9a-f]{64}")


class FlightAuthorityLost(RuntimeError):
    """Raised when onboard flight output no longer owns the aircraft."""


@dataclass(frozen=True)
class ExpectedTransition:
    mode: str | None = None
    armed: bool | None = None

    def __post_init__(self) -> None:
        if self.mode is None and self.armed is None:
            raise ValueError("expected transition requires mode or armed")
        if self.mode is not None:
            if not isinstance(self.mode, str) or not self.mode.strip():
                raise ValueError("expected mode must be a nonempty string")
            object.__setattr__(self, "mode", self.mode.strip().upper())
        if self.armed is not None and not isinstance(self.armed, bool):
            raise TypeError("expected armed state must be Boolean")


@dataclass
class _Permission:
    mode: str | None
    armed: bool | None

    @classmethod
    def from_transition(
        cls,
        transition: ExpectedTransition,
        *,
        mode: str | None,
        armed: bool | None,
    ) -> "_Permission | None":
        expected_mode = transition.mode if transition.mode != mode else None
        expected_armed = transition.armed if transition.armed != armed else None
        if expected_mode is None and expected_armed is None:
            return None
        return cls(expected_mode, expected_armed)

    @property
    def empty(self) -> bool:
        return self.mode is None and self.armed is None


class FlightAuthorityGate:
    """Serialize FC transition evidence with the final aircraft-output enqueue."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._admission_sha256: str | None = None
        self._mode: str | None = None
        self._armed: bool | None = None
        self._last_observation_ns: int | None = None
        self._pending: list[_Permission] = []
        self._output_depth = 0
        self._staged: list[_Permission] = []
        self._operator_wait: _Permission | None = None
        self._reason: str | None = None

    @property
    def revoked(self) -> bool:
        with self._lock:
            return self._reason is not None

    @property
    def reason(self) -> str | None:
        with self._lock:
            return self._reason

    @property
    def admitted(self) -> bool:
        with self._lock:
            return self._admission_sha256 is not None

    def admit(self, admission_sha256: str) -> None:
        if not isinstance(admission_sha256, str) or _SHA256.fullmatch(
            admission_sha256
        ) is None:
            raise ValueError("authority admission must be a lowercase SHA-256 digest")
        with self._lock:
            if self._reason is not None:
                raise FlightAuthorityLost(self._reason)
            if self._mode is None or self._armed is None:
                raise FlightAuthorityLost(
                    "fresh flight-controller mode and arm baseline is unavailable"
                )
            if self._admission_sha256 is None:
                self._admission_sha256 = admission_sha256
            elif self._admission_sha256 != admission_sha256:
                raise FlightAuthorityLost("flight admission identity changed")

    def revoke(self, reason: str) -> None:
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("authority revocation requires a reason")
        with self._lock:
            if self._reason is None:
                self._reason = reason.strip()
                self._pending.clear()
                self._operator_wait = None

    def _require_output(self) -> None:
        if self._reason is not None:
            raise FlightAuthorityLost(self._reason)
        if self._admission_sha256 is None:
            raise FlightAuthorityLost("flight output has no admitted mission owner")

    @contextmanager
    def output_transaction(
        self, expected: ExpectedTransition | None = None
    ) -> Iterator[None]:
        """Hold authority through one actual output call and then own its transition."""
        if expected is not None and not isinstance(expected, ExpectedTransition):
            raise TypeError("expected must be an ExpectedTransition")
        with self._lock:
            self._require_output()
            outermost = self._output_depth == 0
            previous = list(self._pending)
            if outermost:
                self._staged = []
            self._output_depth += 1
            try:
                yield
                if expected is not None:
                    permission = _Permission.from_transition(
                        expected, mode=self._mode, armed=self._armed
                    )
                    if permission is not None:
                        self._staged.append(permission)
            except BaseException:
                if outermost:
                    self._staged = []
                    self._pending = previous
                raise
            finally:
                self._output_depth -= 1
            if outermost:
                self._pending = self._staged
                self._staged = []

    @contextmanager
    def operator_wait(self, expected: ExpectedTransition) -> Iterator[None]:
        """Permit native GUIDED/arm changes only while one wait step is active."""
        if not isinstance(expected, ExpectedTransition):
            raise TypeError("operator wait requires an ExpectedTransition")
        if expected.mode not in (None, "GUIDED") or expected.armed not in (
            None,
            True,
        ):
            raise ValueError("operator wait may permit only GUIDED and arm transitions")
        with self._lock:
            self._require_output()
            if self._operator_wait is not None:
                raise RuntimeError("another operator wait transition is active")
            permission = _Permission.from_transition(
                expected, mode=self._mode, armed=self._armed
            )
            self._operator_wait = permission
        try:
            yield
        finally:
            with self._lock:
                self._operator_wait = None

    def _accept(self, field: str, value: object) -> bool:
        for permission in tuple(self._pending):
            if getattr(permission, field) == value:
                setattr(permission, field, None)
                if permission.empty:
                    self._pending.remove(permission)
                return True
        wait = self._operator_wait
        if wait is not None and getattr(wait, field) == value:
            setattr(wait, field, None)
            return True
        return False

    def observe(self, telemetry: Telemetry) -> None:
        """Apply one newly received exact-FC heartbeat transition observation."""
        if not isinstance(telemetry, Telemetry):
            raise TypeError("authority observation must be Telemetry")
        if not telemetry.heartbeat:
            return
        if (
            not isinstance(telemetry.timestamp_ns, int)
            or isinstance(telemetry.timestamp_ns, bool)
            or telemetry.timestamp_ns < 0
        ):
            return
        mode = telemetry.mode.strip().upper() if isinstance(telemetry.mode, str) else None
        armed = telemetry.armed if isinstance(telemetry.armed, bool) else None
        with self._lock:
            if (
                self._last_observation_ns is not None
                and telemetry.timestamp_ns < self._last_observation_ns
            ):
                return
            self._last_observation_ns = telemetry.timestamp_ns
            previous_mode, previous_armed = self._mode, self._armed
            if mode is not None:
                self._mode = mode
            if armed is not None:
                self._armed = armed
            if self._admission_sha256 is None or self._reason is not None:
                return
            changes = (
                ("mode", previous_mode, mode),
                ("armed", previous_armed, armed),
            )
            for field, previous, current in changes:
                if current is None or previous is None or current == previous:
                    continue
                if not self._accept(field, current):
                    display = current if field == "mode" else str(current).lower()
                    self.revoke(
                        f"unexpected flight-controller {field} transition to {display}"
                    )
                    return


_COPTER_MODES = {
    0: "STABILIZE",
    2: "ALT_HOLD",
    4: "GUIDED",
    5: "LOITER",
    9: "LAND",
    15: "AUTOTUNE",
}


class DroneKitAuthorityWriter:
    """Guard DroneKit aircraft packets at its existing MAVWriter queue."""

    _OBSERVATION_MESSAGES = {
        "COMMAND_ACK",
        "HEARTBEAT",
        "PARAM_REQUEST_LIST",
        "PARAM_REQUEST_READ",
        "PING",
        "REQUEST_DATA_STREAM",
        "STATUSTEXT",
        "TIMESYNC",
    }
    _OBSERVATION_COMMANDS = {511, 512, 520}

    def __init__(self, delegate: Any, gate: FlightAuthorityGate, mavlink: Any):
        if not callable(getattr(delegate, "write", None)):
            raise TypeError("DroneKit authority writer requires a writable delegate")
        self.delegate = delegate
        self.queue = getattr(delegate, "queue", None)
        self._gate = gate
        self._mavlink = mavlink

    def _decode(self, packet: object) -> Any:
        if not isinstance(packet, (bytes, bytearray)):
            raise FlightAuthorityLost("MAVLink output packet is not encoded bytes")
        decoder = self._mavlink.MAVLink(None)
        try:
            return decoder.decode(bytearray(packet))
        except Exception as error:
            raise FlightAuthorityLost("MAVLink output packet could not be classified") from error

    def _expected(self, message: Any) -> tuple[ExpectedTransition, ...]:
        kind = str(message.get_type())
        if kind == "SET_MODE":
            mode = _COPTER_MODES.get(int(getattr(message, "custom_mode", -1)))
            if mode is None:
                return ()
            if mode == "LAND":
                return (
                    ExpectedTransition(mode="LAND"),
                    ExpectedTransition(armed=False),
                )
            return (ExpectedTransition(mode=mode),)
        if kind != "COMMAND_LONG":
            return ()
        command = int(getattr(message, "command", -1))
        if command == 176:
            mode = _COPTER_MODES.get(int(float(getattr(message, "param2", -1))))
            if mode is None:
                return ()
            if mode == "LAND":
                return (
                    ExpectedTransition(mode="LAND"),
                    ExpectedTransition(armed=False),
                )
            return (ExpectedTransition(mode=mode),)
        if command == 400:
            return (ExpectedTransition(armed=float(message.param1) >= 0.5),)
        if command == 21:
            return (
                ExpectedTransition(mode="LAND"),
                ExpectedTransition(armed=False),
            )
        return ()

    def write(self, packet: object) -> object:
        message = self._decode(packet)
        kind = str(message.get_type())
        if kind in self._OBSERVATION_MESSAGES:
            return self.delegate.write(packet)
        if kind == "COMMAND_LONG" and int(getattr(message, "command", -1)) in (
            self._OBSERVATION_COMMANDS
        ):
            return self.delegate.write(packet)
        expected = self._expected(message)
        with ExitStack() as transactions:
            for transition in expected:
                transactions.enter_context(self._gate.output_transaction(transition))
            if not expected:
                transactions.enter_context(self._gate.output_transaction())
            return self.delegate.write(packet)

    def read(self) -> object:
        return self.delegate.read()


def install_dronekit_authority(
    vehicle: object,
    gate: FlightAuthorityGate,
    mavlink: Any,
) -> DroneKitAuthorityWriter:
    """Replace one DroneKit encoder writer without opening another transport."""
    master = getattr(vehicle, "_master", None)
    encoder = getattr(master, "mav", None)
    writer = getattr(encoder, "file", None)
    if encoder is None or writer is None:
        raise TypeError("vehicle does not expose a DroneKit MAVLink writer")
    if isinstance(writer, DroneKitAuthorityWriter):
        if writer._gate is not gate:
            raise RuntimeError("DroneKit writer already has another authority gate")
        return writer
    guarded = DroneKitAuthorityWriter(writer, gate, mavlink)
    encoder.file = guarded
    return guarded


__all__ = [
    "DroneKitAuthorityWriter",
    "ExpectedTransition",
    "FlightAuthorityGate",
    "FlightAuthorityLost",
    "install_dronekit_authority",
]
