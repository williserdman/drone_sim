from __future__ import annotations

import math
from typing import Callable, Protocol, Sequence

from ... import timebase
from ...control.mission_supervisor import AuthorityLost


class ServoDevice(Protocol):
    def max(self) -> None: ...

    def mid(self) -> None: ...


class InertServoFactory(Protocol):
    """Construct a servo without selecting or driving an output position."""

    def __call__(self, pin: int, *, initial_value: None) -> ServoDevice: ...


class PayloadPermissionError(PermissionError):
    """Raised when payload actuation lacks current permission."""


def _gpiozero_factory(
    min_pulse_width: float | None, max_pulse_width: float | None
) -> InertServoFactory:
    if (
        isinstance(min_pulse_width, bool)
        or not isinstance(min_pulse_width, (int, float))
        or isinstance(max_pulse_width, bool)
        or not isinstance(max_pulse_width, (int, float))
    ):
        raise ValueError("GPIO servo pulse widths require explicit configuration")
    if (
        not math.isfinite(min_pulse_width)
        or not math.isfinite(max_pulse_width)
        or min_pulse_width <= 0
        or max_pulse_width <= min_pulse_width
    ):
        raise ValueError("GPIO servo pulse widths must be finite, positive, and ordered")

    from gpiozero import Servo  # type: ignore

    def create(pin: int, *, initial_value: None) -> ServoDevice:
        return Servo(
            pin,
            initial_value=initial_value,
            min_pulse_width=min_pulse_width,
            max_pulse_width=max_pulse_width,
        )

    return create


class Dropper:
    supports_attachment = False

    def __init__(
        self,
        *,
        pins: Sequence[int],
        permission: Callable[[], bool],
        release_hold_seconds: float,
        permission_check_interval_seconds: float,
        servo_factory: InertServoFactory | None = None,
        sleeper: Callable[[float], None] = timebase.sleep,
        min_pulse_width: float | None = None,
        max_pulse_width: float | None = None,
    ):
        pins = tuple(pins)
        if not pins or any(
            isinstance(pin, bool) or not isinstance(pin, int) or pin < 0
            for pin in pins
        ):
            raise ValueError(
                "pins must contain one or more non-negative integer GPIO identifiers"
            )
        if len(set(pins)) != len(pins):
            raise ValueError("pins must be unique")
        if not callable(permission):
            raise ValueError("permission must be callable")
        if (
            isinstance(release_hold_seconds, bool)
            or not isinstance(release_hold_seconds, (int, float))
            or not math.isfinite(release_hold_seconds)
            or release_hold_seconds <= 0
        ):
            raise ValueError("release_hold_seconds must be finite and positive")
        if (
            isinstance(permission_check_interval_seconds, bool)
            or not isinstance(permission_check_interval_seconds, (int, float))
            or not math.isfinite(permission_check_interval_seconds)
            or permission_check_interval_seconds <= 0
            or permission_check_interval_seconds > release_hold_seconds
        ):
            raise ValueError(
                "permission_check_interval_seconds must be positive and no longer than the hold"
            )

        create_servo = servo_factory or _gpiozero_factory(
            min_pulse_width, max_pulse_width
        )
        self._permission = permission
        self._release_hold_seconds = float(release_hold_seconds)
        self._permission_check_interval_seconds = float(
            permission_check_interval_seconds
        )
        self._sleep = sleeper
        self.servos = []
        for pin in pins:
            self._require_permission()
            self.servos.append(create_servo(pin, initial_value=None))

    def _require_permission(self) -> None:
        try:
            permitted = self._permission() is True
        except Exception as error:
            raise PayloadPermissionError("payload permission check failed") from error
        if not permitted:
            raise PayloadPermissionError("payload actuation permission is not current")

    def _actuate(self, output: Callable[[], None]) -> None:
        transaction = getattr(self._permission, "actuate", None)
        if not callable(transaction):
            raise PayloadPermissionError(
                "payload atomic actuation transaction is not installed"
            )
        transaction(output)

    def drop(self) -> None:
        self.drop_with_guard(self._actuate, self._actuate)

    def drop_with_guard(
        self,
        release_actuate: Callable[[Callable[[], None]], None],
        continuation_actuate: Callable[[Callable[[], None]], None],
    ) -> None:
        if not callable(release_actuate) or not callable(continuation_actuate):
            raise TypeError("release and continuation actuators must be callable")

        primary_error: BaseException | None = None
        primary_traceback = None
        try:
            for servo in self.servos:
                release_actuate(servo.max)

            remaining = self._release_hold_seconds
            while remaining > 0:
                self._require_permission()
                duration = min(self._permission_check_interval_seconds, remaining)
                self._sleep(duration)
                remaining -= duration
        except BaseException as error:
            primary_error = error
            primary_traceback = error.__traceback__

        first_neutralization_error: BaseException | None = None
        for servo in self.servos:
            try:
                continuation_actuate(servo.mid)
            except BaseException as error:
                if first_neutralization_error is None:
                    first_neutralization_error = error
                if isinstance(error, AuthorityLost):
                    break

        if primary_error is not None:
            raise primary_error.with_traceback(primary_traceback)
        if first_neutralization_error is not None:
            raise first_neutralization_error

    def attach(self, target_id: int) -> bool:
        raise NotImplementedError(
            "this payload hardware has no attachment or confirmation mechanism"
        )
